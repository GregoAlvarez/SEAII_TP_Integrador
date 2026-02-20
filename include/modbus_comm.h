/*
Wrapper sobre esp-modbus que encapsula la inicialización,
gestión de registros y actualización de estado.
 */

#ifndef MODBUS_COMM_H
#define MODBUS_COMM_H

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "config/modbus_map.h"

// ============================================
// Configuración Modbus
// ============================================
#define MODBUS_SLAVE_ADDR       1
#define MODBUS_BAUDRATE         9600

// ============================================
// Funciones públicas
// ============================================

esp_err_t modbus_comm_init(void);

esp_err_t modbus_comm_start_tasks(void);

QueueHandle_t modbus_comm_get_cmd_queue(void);

QueueHandle_t modbus_comm_get_status_queue(void);

uint8_t modbus_comm_is_config_valid(void);

uint16_t modbus_comm_get_sample_period(void);

#endif // MODBUS_COMM_H
