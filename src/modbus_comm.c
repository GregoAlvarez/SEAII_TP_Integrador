/*
modbus_comm.c
Implementación del módulo de comunicación Modbus RTU
 */

#include "modbus_comm.h"
#include "config/pin_config.h"
#include "config/modbus_map.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "mbcontroller.h"

static const char *TAG = "MODBUS_COMM";     // Tag de debug

// ============================================
// Variables privadas
// ============================================
static uint16_t holding_regs[HR_TOTAL_COUNT] = {0}; //Se inicinalizan en 0, congruente con sus finciones
static void *modbus_ctx = NULL;
static QueueHandle_t cmd_queue = NULL;
static QueueHandle_t status_queue = NULL;
static TaskHandle_t modbus_task_events_handle = NULL;
static TaskHandle_t modbus_task_status_handle = NULL;

// ============================================
// Prototipos privados
// ============================================
static void modbus_comm_events_task(void *param);
static void modbus_comm_status_task(void *param);
static void process_modbus_events(void);
static void process_status_updates(void);
static void update_config_valid(void);

// ============================================
// Implementación pública
// ============================================


//Se incializa la comunicación modbus y recursos asociados
esp_err_t modbus_comm_init(void)
{
    // Cola de comandos (Modbus_Events produce -> System Manager consume)
    cmd_queue = xQueueCreate(5, sizeof(uint16_t));
    if (cmd_queue == NULL) {
        ESP_LOGE(TAG, "Error creando cmd_queue");
        return ESP_FAIL;
    }

    // Cola de actualizaciones de estado (tareas producen -> Modbus_status consume)
    status_queue = xQueueCreate(20, sizeof(status_update_t));
    if (status_queue == NULL) {
        ESP_LOGE(TAG, "Error creando status_queue");
        return ESP_FAIL;
    }

    // Configuración comunicación Modbus
    mb_communication_info_t config = {
        .mode = MB_RTU,
        .ser_opts.port = MODBUS_UART_PORT,
        .ser_opts.baudrate = MODBUS_BAUDRATE,
        .ser_opts.parity = MB_PARITY_NONE,
        .ser_opts.uid = MODBUS_SLAVE_ADDR,
        .ser_opts.data_bits = UART_DATA_8_BITS,
        .ser_opts.stop_bits = UART_STOP_BITS_1
    };

    // Se genera el contexto del esclavo modbus, según la config.
    esp_err_t err = mbc_slave_create_serial(&config, &modbus_ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error creando slave Modbus: %d", err);
        return err;
    }

    // Configuración pines UART
    uart_set_pin(MODBUS_UART_PORT, MODBUS_TX_PIN, MODBUS_RX_PIN, MODBUS_DE_RE_PIN, UART_PIN_NO_CHANGE);

    // se define área de holding registers
    mb_register_area_descriptor_t area = {
        .start_offset = 0,
        .type = MB_PARAM_HOLDING,   //HR
        .access = MB_ACCESS_RW,     //Todos serán en teoría de lectura/escritura
        .address = (void*)holding_regs, //Dirección del arreglo de HR en memoria
        .size = sizeof(holding_regs)
    };
    
    // Se asocian los HR al contexto modbus
    err = mbc_slave_set_descriptor(modbus_ctx, area);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error registrando área Modbus: %d", err);
        return err;
    }

    // Inicia comunicación Modbus
    err = mbc_slave_start(modbus_ctx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando Modbus: %d", err);
        return err;
    }

    ESP_LOGI(TAG, "Modbus iniciado - Slave:%d Baud:%d", MODBUS_SLAVE_ADDR, MODBUS_BAUDRATE);

    return ESP_OK;
}


/*
Se crean dos tareas: una para procesar eventos del maestro (lecturas/escrituras) y 
otra para actualizar los registros de estado basándose en las actualizaciones recibidas de otras tareas.
No se usa una sola ya que el comportamiento de ambas es independiente de la otra:
- Se debe poder actualizar con la frecuencia deseada los registros de estado, independientemente de si hay operaciones del maestro.
- Se debe poder procesar eventos del maestro en tiempo real, sin esperar a que se actualicen los registros de estado.
Se asigna la misma prioridad a ambas, ya que no hay una que sea más critica, se ejecutarán según el time slicing del scheduler. 
Si se asignara mayor prioridad a una, podría causar starvation de la otra tarea.
*/
esp_err_t modbus_comm_start_tasks(void)
{
    //Tarea para procesar comandos del maestro modbus
    BaseType_t ret = xTaskCreatePinnedToCore(
        modbus_comm_events_task,
        "modbus_comm_events",
        4096,
        NULL,
        10,             // Prioridad
        &modbus_task_events_handle,
        1               // Core 
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Error creando tarea modbus_comm_events");
        return ESP_FAIL;
    }
    // Tarea para actualizar registros de estado
    ret = xTaskCreatePinnedToCore(
        modbus_comm_status_task,
        "modbus_comm_status",
        4096,
        NULL,
        10,             // Prioridad
        &modbus_task_status_handle,
        1               // Core 1
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Error creando tarea modbus_comm_status");
        return ESP_FAIL;
    }

    return ESP_OK;
}

//Obtener la cola de comandos desde otras tareas
QueueHandle_t modbus_comm_get_cmd_queue(void)
{
    return cmd_queue;
}

//Obtener la cola de status desde otras tareas
QueueHandle_t modbus_comm_get_status_queue(void)
{
    return status_queue;
}

//Acceso a registro de config. valida
uint8_t modbus_comm_is_config_valid(void)
{
    uint8_t valid = 0;
    mbc_slave_lock(modbus_ctx);
    valid = (uint8_t)holding_regs[HR_CONFIG_VALID];
    mbc_slave_unlock(modbus_ctx);

    return valid;
}

//Acceso a registro de período de muestreo
uint16_t modbus_comm_get_sample_period(void)
{
    uint16_t period = 0;
    mbc_slave_lock(modbus_ctx); // Tomo mutex de la librería para acceder a los HR
    period = holding_regs[HR_IMU_SAMPLE_PERIOD];
    mbc_slave_unlock(modbus_ctx);  //Librero mutex

    return period;
}

// ============================================
// Implementación privada
// ============================================

// Tarea de procesamiento de eventos de comunicación modbus
static void modbus_comm_events_task(void *param)
{
    ESP_LOGI(TAG, "Tarea modbus_comm_events iniciada");

    while (1) {
        // Procesa eventos Modbus (lecturas/escrituras del maestro)
        process_modbus_events();

    }
}


//Tarea para procesar actualizaciones de estado desde otras tareas. 
static void modbus_comm_status_task(void *param)
{
    ESP_LOGI(TAG, "Tarea modbus_comm_status iniciada");

    while (1) {
        // Procesar actualizaciones de estado desde otras tareas hacia los HR solo si hay datos nuevos.
        process_status_updates();

        //vTaskDelay(pdMS_TO_TICKS(50)); //Restrinjo el ciclo a mínimo 50ms para no saturar CPU.
    }
}

/*
Procesa eventos de lectura/escritura del maestro Modbus.
Detecta escrituras en registros de comando (una vez fueron hechas) y las envía a System Manager.
 */
static void process_modbus_events(void)
{
    mb_param_info_t info; //Tipo de dato de librería para recibir info de eventos.
    
    // Espera a evento de la librería modbus. Bloquea la tarea mientras tanto.
    esp_err_t err = mbc_slave_get_param_info(modbus_ctx, &info, portMAX_DELAY);
    if (err != ESP_OK) {
        return;
    }

    // Hubo escritura o lectura en HR (podría haber otro tipo de eventos, nunca va a ser el caso acá).
    if (info.type == MB_EVENT_HOLDING_REG_WR) { //Solo interesan las escrituras.
        
        // Hubo escritura en registro de comando
        if (info.mb_offset == HR_SYSTEM_COMMAND) {
            uint16_t cmd;

            mbc_slave_lock(modbus_ctx); // Tomo mutex de la librería para acceder a los HR
            cmd = (uint16_t)holding_regs[HR_SYSTEM_COMMAND]; // Buffereo el comando para procesarlo fuera de mutex
            holding_regs[HR_SYSTEM_COMMAND] = CMD_IDLE;  // Reset comando, ya lo leí
            mbc_slave_unlock(modbus_ctx);   //libero mutex
            
            // Envío comando a System Manager
            if (xQueueSend(cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) { //Se intenta enviar el comando a la cola con timeout
                ESP_LOGW(TAG, "cmd_queue llena, comando perdido");
            } else {
                ESP_LOGI(TAG, "Comando recibido: %d", cmd);
            }
        }
        
        // Hubo escritura en registro de período de muestreo
        if (info.mb_offset == HR_IMU_SAMPLE_PERIOD) {
            update_config_valid();
        }
    }
}

/*
 Procesa actualizaciones de estado recibidas de otras tareas.
 Actualiza los holding registers correspondientes.
 */
static void process_status_updates(void)
{
    status_update_t update;
    // Pispea la cola para ver si hay elementos. Si no hay, se bloquea la tarea hasta que haya. Si hay, entra para procesar de a uno.
    // Esto no extrae un elemento de la cola, solo lo lee. Evita que se extraiga un elemento sin saber si se podrá procesar.
    while (xQueuePeek(status_queue, &update, portMAX_DELAY) == pdTRUE){
        
        mbc_slave_lock(modbus_ctx); // Tomo mutex de la librería para acceder a los HR

        //Si puedo tomar el mutex, extraigo el elemento de la cola para procesarlo.
        if(xQueueReceive(status_queue,&update,0)!= pdTRUE){
            //xSemaphoreGive(hr_mutex);   //Si no pude extraerlo, libero el mutex y paso a la siguiente iteracion. 
            mbc_slave_unlock(modbus_ctx); //Si no pude extraerlo, libero el mutex y paso a la siguiente iteracion. 
            ESP_LOGE(TAG, "No se pudo extraer el elemento de la cola");  //Esto no debería suceder nunca ya que solo esta tarea consume de la cola, entonces no habría forma de que otra "robe" elementos en el interín.
            continue; //Si no puedo recibir el elemento, libero el mutex y paso a la siguiente iteración (no debería pasar porque peek me aseguró que había un elemento, pero por las dudas lo manejo)
        }

        //En función del tipo de actualización, se actualizan los HR correspondientes
        switch (update.type) {
            case STATUS_UPDATE_STATE:
                holding_regs[HR_SYSTEM_STATE] = update.data.state;
                break;

            case STATUS_UPDATE_CMD_STATUS:
                holding_regs[HR_COMMAND_STATUS] = update.data.cmd_status;
                break;

            case STATUS_UPDATE_ERROR:
                holding_regs[HR_ERROR_CODE] = update.data.error_code;
                break;

            case STATUS_UPDATE_IMU_DATA:
                holding_regs[HR_IMU_ACCEL_X] = (uint16_t)update.data.imu_data.accel_x; //se castea por el tipo de dato de los HR.  
                holding_regs[HR_IMU_ACCEL_Y] = (uint16_t)update.data.imu_data.accel_y;  
                holding_regs[HR_IMU_ACCEL_Z] = (uint16_t)update.data.imu_data.accel_z;
                holding_regs[HR_IMU_SAMPLE_COUNT] = update.data.imu_data.sample_count;
                break;
        }

        mbc_slave_unlock(modbus_ctx); //libero mutex
    }
    
}

/*
Actualiza el flag CONFIG_VALID basándose en el período de muestreo. CONFIG_VALID = 1 si el período está dentro de los límites válidos.
 */
static void update_config_valid(void)
{
    mbc_slave_lock(modbus_ctx); // Tomo mutex de la librería para acceder a los HR
    uint16_t period = holding_regs[HR_IMU_SAMPLE_PERIOD];
    if (period >= IMU_SAMPLE_PERIOD_MIN && period <= IMU_SAMPLE_PERIOD_MAX) {
        holding_regs[HR_CONFIG_VALID] = 1;
        ESP_LOGI(TAG, "Config válida - período: %d ms", period);
    } else {
        ESP_LOGI(TAG, "Config inválida - período: %d ms. Debe estar entre %d y %d ms", period, IMU_SAMPLE_PERIOD_MIN, IMU_SAMPLE_PERIOD_MAX);
        holding_regs[HR_CONFIG_VALID] = 0;
        holding_regs[HR_IMU_SAMPLE_PERIOD] = 0;
    }
    mbc_slave_unlock(modbus_ctx); //libero mutex
}
