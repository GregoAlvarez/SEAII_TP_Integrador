/*
imu_task.h
Tarea de adquisición de datos de la IMU MPU6050
*/

#ifdef __cplusplus
extern "C" {
#endif
 
#ifndef IMU_TASK_H
#define IMU_TASK_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
//#include "freertos/semphr.h"


// ============================================
// Bits del Event Group para control de IMU
// ============================================
#define EVENT_IMU_START     (1 << 0)
#define EVENT_IMU_STOP      (1 << 1)

// ============================================
// Funciones públicas
// ============================================


esp_err_t imu_task_init(void);

esp_err_t imu_task_start(EventGroupHandle_t event_group, QueueHandle_t status_queue);

void imu_task_set_sample_period(uint16_t period_ms);

SemaphoreHandle_t imu_task_get_error_semaphore(void);


#endif // IMU_TASK_H

#ifdef __cplusplus
}
#endif