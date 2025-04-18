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

enum E32_UART_PARITY
{
    E32_UART_PARITY_8N1 = 0b00,
    E32_UART_PARITY_8O1 = 0b01,
    E32_UART_PARITY_8E1 = 0b10
};

enum E32_UART_BAUD_RATE
{
    E32_UART_BAUD_RATE_1200 = 0b000,
    E32_UART_BAUD_RATE_2400 = 0b001,
    E32_UART_BAUD_RATE_4800 = 0b010,
    E32_UART_BAUD_RATE_9600 = 0b011,
    E32_UART_BAUD_RATE_19200 = 0b100,
    E32_UART_BAUD_RATE_38400 = 0b101,
    E32_UART_BAUD_RATE_57600 = 0b110,
    E32_UART_BAUD_RATE_115200 = 0b111
};

enum AIR_DATA_RATE
{
    AIR_DATA_RATE_300 = 0b000,
    AIR_DATA_RATE_1200 = 0b001,
    AIR_DATA_RATE_2400 = 0b010,
    AIR_DATA_RATE_4800 = 0b011,
    AIR_DATA_RATE_9600 = 0b100,
    AIR_DATA_RATE_19200 = 0b101
};

enum TRANSMISSION
{
    TRANSMISSION_TRANSPARENT = 0b0,
    TRANSMISSION_FIXED = 0b1
};

enum IO_DRIVE_MODE
{
    IO_DRIVE_MODE_PUSH_PULL = 0b0,
    IO_DRIVE_MODE_OPEN_DRAIN = 0b1
};

enum WIRELESS_WAKEUP_TIME
{
    WIRELESS_WAKEUP_TIME_250MS = 0b000,
    WIRELESS_WAKEUP_TIME_500MS = 0b001,
    WIRELESS_WAKEUP_TIME_750MS = 0b010,
    WIRELESS_WAKEUP_TIME_1000MS = 0b011,
    WIRELESS_WAKEUP_TIME_1250MS = 0b100,
    WIRELESS_WAKEUP_TIME_1500MS = 0b101,
    WIRELESS_WAKEUP_TIME_1750MS = 0b110,
    WIRELESS_WAKEUP_TIME_2000MS = 0b111
};

enum FEC
{
    FEC_DISABLE = 0b0,
    FEC_ENABLE = 0b1
};

enum TRANSMISSION_POWER
{
    TRANSMISSION_POWER_30dBm = 0b00,
    TRANSMISSION_POWER_27dBm = 0b01,
    TRANSMISSION_POWER_24dBm = 0b10,
    TRANSMISSION_POWER_21dBm = 0b11
};

enum MODE
{
    MODE_TRANSPARENT = 0b00,
    MODE_FIXED = 0b01,
    MODE_PROGRAMMING = 0b10,
    MODE_SLEEP = 0b11
};

#pragma pack(push, 1) // no padding between struct members
typedef struct
{
    uint8_t airDataRate : 3;  // bit 0-2
    uint8_t uartBaudRate : 3; // bit 3-5
    uint8_t uartParity : 2;   // bit 6-7
} e32_speed_t;

typedef struct
{
    uint8_t transmissionPower : 2;  // bit 0-1
    uint8_t fec : 1;                // bit 2
    uint8_t wirelessWakeupTime : 3; // bit 3-5
    uint8_t ioDriveMode : 1;        // bit 6
    uint8_t fixedTransmission : 1;  // bit 7
} e32_option_t;

typedef struct 
{
    uint8_t HEAD;
    uint8_t ADDH;
    uint8_t ADDL;
    e32_speed_t SPED;
    uint8_t CHAN;
    e32_option_t OPTION;
} e32_config_t;
#pragma pack(pop)

void e32_init_config(e32_config_t *config)
{
    config->HEAD = 0xC0; // save parameter when power-down
    config->ADDH = 0x00;
    config->ADDL = 0x00;
    config->SPED.uartParity = E32_UART_PARITY_8N1;
    config->SPED.uartBaudRate = E32_UART_BAUD_RATE_9600;
    config->SPED.airDataRate = AIR_DATA_RATE_2400;
    config->CHAN = 0x06;                                         // Kanal 7 (902.875MHz)
    config->OPTION.fixedTransmission = TRANSMISSION_TRANSPARENT; // Transparent mode
    config->OPTION.ioDriveMode = IO_DRIVE_MODE_PUSH_PULL;
    config->OPTION.wirelessWakeupTime = WIRELESS_WAKEUP_TIME_250MS;
    config->OPTION.fec = FEC_ENABLE;
    config->OPTION.transmissionPower = TRANSMISSION_POWER_30dBm; // 30dBm
}
