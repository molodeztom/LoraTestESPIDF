/* Definitions for Lora Library for ESP-IDF*/
#ifndef Lora_E32_h
#define Lora_E32_h
#endif

// Pin configuration
#define E32_M0_GPIO 10
#define E32_M1_GPIO 11
#define E32_AUX_GPIO 14
#define E32_TXD_GPIO 12 // TXD Pin on ESP32
#define E32_RXD_GPIO 13 // RXD Pin on ESP32

// UART configuration
#define E32_UART_PORT UART_NUM_1
#define BUF_SIZE 1024

#define CONFIG_CMD_LEN 6
#define RESPONSE_LEN 6