#include "system_manager.h"
#include "modbus_comm.h"
#include "imu_task.h"
#include "config/modbus_map.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "SYS_MGR"; // Tag de debug

// ============================================
// Variables privadas
// ============================================
static EventGroupHandle_t system_events = NULL;
static QueueSetHandle_t queue_set = NULL;
static TaskHandle_t sys_mgr_task_handle = NULL;
static uint8_t current_state = STATE_IDLE;

// ============================================
// Prototipos privados
// ============================================
static void system_manager_task(void *param);
static void handle_command(uint16_t command);
static void handle_error(void);
static void transition_to_state(uint8_t new_state);
static void publish_state(uint8_t state);
static void publish_cmd_status(uint8_t status);


// ============================================
// Implementación pública
// ============================================

//Crea event group general del sistema, será devuelto por system_manager_get_event_group() hacia otros archivos
esp_err_t system_manager_init(void)
{
    //Event group para comandar otras tareas
    system_events = xEventGroupCreate();
    if (system_events == NULL) {
        ESP_LOGE(TAG, "Error creando event group");
        return ESP_FAIL;
    }

    // Traigo recursos de otros módulos
    QueueHandle_t cmd_queue = modbus_comm_get_cmd_queue();
    SemaphoreHandle_t error_sem = imu_task_get_error_semaphore();

    if (cmd_queue == NULL || error_sem == NULL) {
        ESP_LOGE(TAG, "Recursos no disponibles para queue set");
        return ESP_FAIL;
    }

    // Creo queue set (tamaño es cant. de "notificaciones" = longitud de cmd_queue + 1 para el semáforo)
    queue_set = xQueueCreateSet(5 + 1); 
    if (queue_set == NULL) {
        ESP_LOGE(TAG, "Error creando queue set");
        return ESP_FAIL;
    }
    // Agrega cmd_queue al set
    if (xQueueAddToSet(cmd_queue, queue_set) != pdPASS) {
        ESP_LOGE(TAG, "Error agregando cmd_queue al set");
        return ESP_FAIL;
    }
    // Agrega semáforo de error al set
    if (xQueueAddToSet(error_sem, queue_set) != pdPASS) {
        ESP_LOGE(TAG, "Error agregando error_sem al set");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "System Manager inicializado");
    return ESP_OK;
}

//lanzamiento de tarea de system manager
esp_err_t system_manager_start_task(void)
{
    BaseType_t ret = xTaskCreatePinnedToCore(
        system_manager_task,
        "sys_manager",
        4096,
        NULL,
        8,              // Prioridad
        &sys_mgr_task_handle,
        1               // Core 1
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Error creando tarea System Manager");
        return ESP_FAIL;
    }

    return ESP_OK;
}

//Devuelve event group del sistema
EventGroupHandle_t system_manager_get_event_group(void)
{
    return system_events;
}

// ============================================
// Implementación privada
// ============================================

/*
Tarea principal: System Manager.
Procesa comandos del maestro Modbus. 
 */
static void system_manager_task(void *param)
{
    QueueHandle_t cmd_queue = modbus_comm_get_cmd_queue();
    SemaphoreHandle_t error_sem = imu_task_get_error_semaphore();
    QueueSetMemberHandle_t activated;
    uint16_t cmd;

    ESP_LOGI(TAG, "Tarea System Manager iniciada");
   
    publish_state(STATE_IDLE);
    publish_cmd_status(STATUS_OK);

    while (1) {
        // Se bloquea la tarea hasta quealgún elemento del set tenga datos.
        //Este mecanismo permite bloquear la tarea ante dos tipos de eventos, sino no podría supervisar ambos
        activated = xQueueSelectFromSet(queue_set, portMAX_DELAY);
        //TODO: Leer nota 3 de xQueueSelectFromSet. Se cumple porque esta es la unica tarea consumidora de ambos
        if (activated == (QueueSetMemberHandle_t)cmd_queue) {
            // Llegó un comando del maestro Modbus
            if (xQueueReceive(cmd_queue, &cmd, 0) == pdTRUE) {
                handle_command(cmd);
            }
        } 
        else if (activated == (QueueSetMemberHandle_t)error_sem) {
            // Llegó notificación de error desde IMU Task
            // Tomar el semáforo para "consumir" la notificación
            xSemaphoreTake(error_sem, 0);
            handle_error();
        }
    }
}

/*
Procesa un comando recibido del maestro Modbus.
 */
static void handle_command(uint16_t command)
{
    ESP_LOGI(TAG, "Comando recibido: %d (estado actual: %d)", command, current_state);

    // CMD_RESTART se acepta desde cualquier estado y tiene prioridad de evaluación
    if (command == CMD_RESTART) {
        ESP_LOGW(TAG, "Reinicio solicitado por maestro");
        esp_restart();
    }

    
    switch (current_state) {
        case STATE_IDLE:
            if (command == CMD_START_ACQ) {
                // Verifica configuración válida, solo al iniciar adquisición.
                if (!modbus_comm_is_config_valid()) {
                    ESP_LOGW(TAG, "Configuración inválida");
                    publish_cmd_status(STATUS_ERROR);   
                    publish_error(ERR_CONFIG_INVALID); 
                    return;
                }else {
                    publish_cmd_status(STATUS_OK);
                    publish_error(ERR_NONE); 
                }

                // Obtiene el periodo de muestreo config. por master y lo setea en la tarea IMU
                uint16_t period = modbus_comm_get_sample_period();
                imu_task_set_sample_period(period);

                // Transición a estado adquiriendo
                transition_to_state(STATE_ACQUIRING);

                // Señaliza inicio a IMU Task
                xEventGroupSetBits(system_events, EVENT_IMU_START);

                //publish_cmd_status(STATUS_EXECUTING);
                ESP_LOGI(TAG, "Adquisición iniciada - período: %d ms", period);
            } else { //IDLE o STOP, si era restart no llega a acá
                ESP_LOGW(TAG, "Comando ignorado - estado: %d", current_state);
                publish_cmd_status(STATUS_ERROR);   //Comando rechazado
            }
            break;

        case STATE_ACQUIRING:
            if (command == CMD_STOP_ACQ) {
                // Señaliza parada a IMU Task
                xEventGroupSetBits(system_events, EVENT_IMU_STOP);

                // Transición a estado idle
                transition_to_state(STATE_IDLE);

                publish_cmd_status(STATUS_OK);
                ESP_LOGI(TAG, "Adquisición detenida");
            } else {    //IDLE o START, si era restart no llega a acá
                ESP_LOGW(TAG, "Comando ignorado - estado: %d", current_state);
                publish_cmd_status(STATUS_ERROR);   //Comando rechazado
            }
            break;

        case STATE_ERROR:
            // En estado ERROR solo se acepta CMD_RESTART (ya manejado arriba)
            // Solo se puede llegar a través de la llamada a handle error
            ESP_LOGW(TAG, "En estado ERROR, solo se acepta CMD_RESTART");
            publish_cmd_status(STATUS_ERROR);   //Comando rechazado
            break;

        default:
            ESP_LOGW(TAG, "Estado desconocido: %d", current_state);
    }
}

/*
 Realiza la transición a un nuevo estado.
 */
static void transition_to_state(uint8_t new_state)
{
    if (current_state != new_state) {
        ESP_LOGI(TAG, "Transición: %d -> %d", current_state, new_state);
        current_state = new_state;
        publish_state(new_state);
    }
}

/*
Publica el estado actual en la cola de status.
 */
static void publish_state(uint8_t state)
{
    QueueHandle_t status_queue = modbus_comm_get_status_queue();
    status_update_t update = {
        .type = STATUS_UPDATE_STATE,
        .data.state = state
    };
    xQueueSend(status_queue, &update, pdMS_TO_TICKS(100));
}

/*
 Publica el estado del comando en la cola de status.
 */
static void publish_cmd_status(uint8_t status)
{
    QueueHandle_t status_queue = modbus_comm_get_status_queue();
    status_update_t update = {
        .type = STATUS_UPDATE_CMD_STATUS,
        .data.cmd_status = status
    };
    xQueueSend(status_queue, &update, pdMS_TO_TICKS(100));
}

/*
 Publica un código de error en la cola de status.
 */
void publish_error(uint8_t error_code)
{
    QueueHandle_t status_queue = modbus_comm_get_status_queue();
    status_update_t update = {
        .type = STATUS_UPDATE_ERROR,
        .data.error_code = error_code
    };
    xQueueSend(status_queue, &update, pdMS_TO_TICKS(100));
}

// Maneja notificación de error desde IMU Task
static void handle_error(void)
{
    ESP_LOGE(TAG, "Error de comunicación IMU recibido");

    // Detener adquisición si estaba activa (si no estuviera, al estar la tarea bloqueada nunca saltaria el error)
    if (current_state == STATE_ACQUIRING) {
        xEventGroupSetBits(system_events, EVENT_IMU_STOP); // se va a despertar la tarea IMU y va a parar la adq.
    }

    // Transiciona a estado ERROR, es una formalidad, porque no va a entrar hasta no recinbir un comando, 
    //pero solo puede recibir el de restart. Si recibe otro solo va a mostrar un cartel
    transition_to_state(STATE_ERROR);
    
    // Publica código de error y status
    publish_error(ERR_IMU_COMM);
    

    ESP_LOGE(TAG, "Sistema en estado ERROR - requiere CMD_RESTART");
}