/*
 * Estructura de registros:
 * - HR 0-9: Comandos
 * - HR 10-19: Configuración  
 * - HR 20-29: Estado
 * - HR 30-39: Datos IMU
 */

#ifndef MODBUS_MAP_H
#define MODBUS_MAP_H

#include <stdint.h>

// ============================================
// Direcciones de Holding Registers
// ============================================
//Se usan rangos y se deja espacio para facilitar lectura y dejar la posibilidad de añadir nuevos

//Comandos
#define HR_SYSTEM_COMMAND       0       // Comando principal (escritura por maestro)

// Configuración (escritura por maestro)
#define HR_IMU_SAMPLE_PERIOD    10      // Período de muestreo IMU en ms

// Estado (solo lectura por maestro)
#define HR_SYSTEM_STATE         20      // Estado actual del sistema
#define HR_COMMAND_STATUS       21      // Estado del último comando
#define HR_ERROR_CODE           22      // Código de error

// Datos IMU (solo lectura por maestro)
#define HR_IMU_ACCEL_X          30      // Aceleración X
#define HR_IMU_ACCEL_Y          31      // Aceleración Y
#define HR_IMU_ACCEL_Z          32      // Aceleración Z
#define HR_IMU_SAMPLE_COUNT     33      // Contador de muestras
#define HR_CONFIG_VALID         34       // Flag de configuración válida (solo lectura)


// ============================================
// Tamaño total del área de registros
// ============================================
#define HR_TOTAL_COUNT          40      // Registros 0-39

// ============================================
// Valores de SYSTEM_COMMAND
// ============================================
#define CMD_IDLE                0
#define CMD_START_ACQ           1
#define CMD_STOP_ACQ            2
#define CMD_RESTART             3

// ============================================
// Valores de SYSTEM_STATE
// ============================================
#define STATE_IDLE              0
#define STATE_ACQUIRING         1
#define STATE_ERROR             2   //Error de sistema, revisar el codigo de error

// ============================================
// Valores de COMMAND_STATUS
// ============================================
#define STATUS_ERROR               0    //Comando rechazado
#define STATUS_OK                  1    //Comando aceptado

// ============================================
// Códigos de error
// ============================================
#define ERR_NONE                0
#define ERR_CONFIG_INVALID      1
#define ERR_IMU_COMM            2   //Requiere reestart del sistema. Se acepta por comando.

// ============================================
// Límites de configuración
// ============================================
#define IMU_SAMPLE_PERIOD_MIN   10      // Mínimo 10 ms
#define IMU_SAMPLE_PERIOD_MAX   10000   // Máximo 10 segundos

// ============================================
// Estructura para actualizaciones de estado
// ============================================
typedef enum {
    STATUS_UPDATE_STATE,
    STATUS_UPDATE_CMD_STATUS,
    STATUS_UPDATE_ERROR,
    STATUS_UPDATE_IMU_DATA
} status_update_type_t;

//Tagged union, para eficientizar el uso de memoria, utilizando el campo indicado por "type"
typedef struct {
    status_update_type_t type;
    union {
        uint8_t state;
        uint8_t cmd_status;
        uint8_t error_code;
        struct {
            int16_t accel_x;
            int16_t accel_y;
            int16_t accel_z;
            uint16_t sample_count;
        } imu_data;
    } data;
} status_update_t;



#endif // MODBUS_MAP_H
