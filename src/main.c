/**
 * TP SEA II - Alvarez Gregorio
 * 
 * Sistema que integra:
 * - Comunicación Modbus RTU como esclavo
 * - Adquisición periódica de datos IMU (MPU6050)
 */


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "modbus_comm.h"
#include "system_manager.h"
#include "imu_task.h"

static const char *TAG = "MAIN"; // Tag de debug

void app_main(void)
{
    ESP_LOGI(TAG, "=== TP SEA II ===");

    //============Creación de recursos=============

    // Inicializa comunicación Modbus
    if (modbus_comm_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando Modbus");
        //TODO: implementar mecanismo de aviso de error de inicializacion (por fuera de las tareas)
        return;
    }

    // Inicializa IMU
    if (imu_task_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando IMU");
        return;
    }

    // Crea event group general y 
    if (system_manager_init() != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando System Manager");
        return;
    }

    //============Lanzamiento de tareas=============

    // Inicializa tarea de manejo de comm Modbus 
    if (modbus_comm_start_tasks() != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando tarea Modbus");
        return;
    }

    // Inicializa tarea de adquisición IMU (necesita event group y status queue)
    EventGroupHandle_t events = system_manager_get_event_group();
    QueueHandle_t status_queue = modbus_comm_get_status_queue();
    
    if (imu_task_start(events, status_queue) != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando tarea IMU");
        return;
    }

    // Inicializa tarea System Manager, que ejecuta la maquina de estados
    if (system_manager_start_task() != ESP_OK) {
        ESP_LOGE(TAG, "Error iniciando tarea System Manager");
        return;
    }

    ESP_LOGI(TAG, "Sistema inicializado correctamente");
    ESP_LOGI(TAG, "Esperando comandos Modbus...");

    // scheduler toma el control
}
