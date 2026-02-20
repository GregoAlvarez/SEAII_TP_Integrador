/*
Tarea coordinadora del sistema (máquina de estados)
 */

#ifndef SYSTEM_MANAGER_H
#define SYSTEM_MANAGER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// ============================================
// Funciones públicas
// ============================================

esp_err_t system_manager_init(void);

esp_err_t system_manager_start_task(void);

EventGroupHandle_t system_manager_get_event_group(void);

void publish_error(uint8_t error_code);


#endif // SYSTEM_MANAGER_H
