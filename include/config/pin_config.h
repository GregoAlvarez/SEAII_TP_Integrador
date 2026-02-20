//Definiciones de pines

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// ============================================
// UART1 - Modbus RTU (RS485)
// ============================================
#define MODBUS_UART_PORT    UART_NUM_1
#define MODBUS_TX_PIN       11
#define MODBUS_RX_PIN       13
#define MODBUS_DE_RE_PIN    UART_PIN_NO_CHANGE      // Control DE/RE del transceiver RS485

// ============================================
// I2C - IMU MPU6050
// ============================================
#define I2C_PORT            I2C_NUM_0
#define IMU_SDA_PIN         18
#define IMU_SCL_PIN         8
#define I2C_FREQ_HZ         400000  // 400 kHz

// ============================================
// Dirección I2C del MPU6050
// ============================================
#define MPU6050_ADDR        0x68

#endif // PIN_CONFIG_H
