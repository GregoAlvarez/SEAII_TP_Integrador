/*
imu_task.c
Manejo de adquisición de datos de la IMU MPU6050
 */

#include "imu_task.h"
#include "config/pin_config.h"
#include "config/modbus_map.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include <string.h>

// Librería MPU6050 de jrowberg
#include "I2Cdev.h"
#include "MPU6050.h"

#define MAX_CONSECUTIVE_FAILURES 10  //Cantidad de reintentos antes de anunciar error de sensor

static const char *TAG = "IMU_TASK"; // Tag de debug

// ============================================
// Variables privadas
// ============================================
static MPU6050 mpu;
static EventGroupHandle_t system_events = NULL;
static QueueHandle_t status_q = NULL;
static TaskHandle_t imu_task_handle = NULL;
static uint16_t sample_period_ms = 0;     // Valor por defecto
static SemaphoreHandle_t error_semaphore = NULL;

// ============================================
// Prototipos privados
// ============================================
static void imu_task(void *param);
static esp_err_t i2c_master_init(void);
static void publish_imu_data(int16_t ax, int16_t ay, int16_t az, int16_t sample_counter);
static void notify_error(void);


// ============================================
// Implementación pública
// ============================================


// Inicialización de hardware I2C y sensor MPU6050
esp_err_t imu_task_init(void)
{
    // Semáforo de error (binario)
    error_semaphore = xSemaphoreCreateBinary();
    if (error_semaphore == NULL) {
        ESP_LOGE(TAG, "Error creando error_semaphore");
        return ESP_FAIL;
    }

    // Inicializa bus I2C
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando I2C: %d", err);
        return err;
    }

    // Inicializa MPU6050
    mpu.initialize();
    
    // Verifica conexión
    if (!mpu.testConnection()) {
        ESP_LOGE(TAG, "MPU6050 no detectado");
        return ESP_FAIL;
    }

    // Configura rango de acelerómetro a ±2g 
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

    ESP_LOGI(TAG, "MPU6050 inicializado correctamente");
    return ESP_OK;
}

//Se lanza tarea de adquisición de datos, recibe punteros de event group para control y cola donde publicar datos.
//Se guardan en variables globales "privadas" del archivo
esp_err_t imu_task_start(EventGroupHandle_t event_group, QueueHandle_t status_queue)
{
    if (event_group == NULL || status_queue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    //Sincronizo los event group con el del resto del proyeto
    system_events = event_group;
    status_q = status_queue;

    BaseType_t ret = xTaskCreatePinnedToCore(
        imu_task,
        "imu_task",
        4096,
        NULL,
        12,             // Prioridad
        &imu_task_handle,
        1               // Core 1
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Error creando tarea IMU");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void imu_task_set_sample_period(uint16_t period_ms)
{
    sample_period_ms = period_ms;
    ESP_LOGI(TAG, "Período de muestreo: %d ms", period_ms);
}

// Devuelve semáforo de error
SemaphoreHandle_t imu_task_get_error_semaphore(void)
{
    return error_semaphore;
}


// ============================================
// Implementación privada
// ============================================

/*
 Tarea de adquisición IMU.
 Espera señal de inicio, luego muestrea y publica en la cola periódicamente hasta recibir stop.
 */
static void imu_task(void *param)
{
    int16_t ax = 0, ay = 0, az = 0, sample_counter = 0;
    EventBits_t bits;
    bool acquiring = false;
    uint8_t consecutive_failures = 0;


    ESP_LOGI(TAG, "Tarea IMU iniciada");

    while (1) {
        // Espera eventos de control. Se bloquea la tarea esperando que se activen los bits de inicio o stop.
        bits = xEventGroupWaitBits(
            system_events,  // Event group recibido en el init
            EVENT_IMU_START | EVENT_IMU_STOP,   // Bits a esperar
            pdTRUE,         // Limpiar el/los bits que despertaron la tarea al salir
            pdFALSE,        // OR entre bits, despierta si alguno se activa
            acquiring ? pdMS_TO_TICKS(sample_period_ms) : portMAX_DELAY //Si se está midiendo, esperar t de muestreo, sino  se bloquea hasta recibir inicio
        );

        // Si se dio el evento de inicio
        if (bits & EVENT_IMU_START) {
            acquiring = true;
            sample_counter = 0;
            ESP_LOGI(TAG, "Adquisición iniciada");
        }

        // Sis e dio el evento de stop
        if (bits & EVENT_IMU_STOP) {
            ESP_LOGI(TAG, "Adquisición detenida - muestras: %d", sample_counter);
            acquiring = false;
            sample_counter = 0;
            ax = ay = az = 0; //Resetear datos
            publish_imu_data(ax, ay, az, sample_counter); //Vacío los HR
        }

        // Si está adquiriendo: leer y publicar datos
        if (acquiring) {
            // Verifica comunicación antes de leer 
            if (!mpu.testConnection()) {//TODO: Se podría hacer que pruebe cada cierto tiempo
                consecutive_failures++;
                ESP_LOGW(TAG, "Fallo comunicación IMU (%d/%d)", consecutive_failures, MAX_CONSECUTIVE_FAILURES);
                
                if (consecutive_failures >= MAX_CONSECUTIVE_FAILURES) {
                    ESP_LOGE(TAG, "Falla I2C confirmada");
                    acquiring = false;
                    notify_error();
                }
            } else {
                consecutive_failures = 0;
                mpu.getAcceleration(&ax, &ay, &az);
                sample_counter++;
                publish_imu_data(ax, ay, az, sample_counter);
            }
        }

    }
}

/*
Inicializa el bus I2C como master.
 */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf;
    
    // Inicializa la estructura a 0 para evitar basura en campos opcionales
    // (como clk_flags que a veces causa problemas si no se limpia)
    memset(&conf, 0, sizeof(i2c_config_t));

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)IMU_SDA_PIN; 
    conf.scl_io_num = (gpio_num_t)IMU_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    
    conf.master.clk_speed = I2C_FREQ_HZ;
    conf.clk_flags = 0; 

    esp_err_t err = i2c_param_config(I2C_NUM_0, &conf); 
    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

/*
 Publica datos de aceleración en la cola de status.
 */
static void publish_imu_data(int16_t ax, int16_t ay, int16_t az, int16_t sample_counter)
{
    status_update_t update;
    
    update.type = STATUS_UPDATE_IMU_DATA;
    update.data.imu_data.accel_x = ax;
    update.data.imu_data.accel_y = ay;
    update.data.imu_data.accel_z = az;
    update.data.imu_data.sample_count = sample_counter;

    // Envío no bloqueante, no importa si se pierde algún dato
    if (xQueueSend(status_q, &update, 0) != pdTRUE) {
        // Cola llena, descartar dato (no es crítico)
        ESP_LOGI(TAG, "DATO IMU DESCARTADO - COLA LLENA");
    }
}

// Notifica error a otras tareas mediante semáforo
static void notify_error(void)
{
    xSemaphoreGive(error_semaphore);
}