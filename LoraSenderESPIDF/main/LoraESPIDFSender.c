/**************************************************************************
 E32-900T30D LoRa Sender


  Hardware:
  ESP32-S3-DevKitC-1 mit Wroom N16R8
  LoRa E32-900T30D connected M0 M1 and Rx Tx

  Try sending a message to remote LoRa

  Project settings:
  ESP-IDF config editor:
  -> LORA debug settings: y for extended output

  History: master if not shown otherwise
  20250406:V0.1: initial version
  20250407:V0.2: added M0 and M1 control
  20250407:V0.3: added UART control
  20250407:V0.4: send configuration command to E32-900T30D
  20250407:V0.5: use function to send data to E32 module
  20250408:V0.6: added function to initialize IO
  20250409:V0.7: added function to read configuration from E32 module
  20250414:V0.8: added function to decode configuration data

  */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "led_strip.h"
#include <string.h>

static const char *TAG = "LORA_Sender";

// Konfiguration der Pins
#define E32_M0_GPIO 10
#define E32_M1_GPIO 11
#define E32_AUX_GPIO 14
#define E32_TXD_GPIO 12 // TXD Pin on ESP32
#define E32_RXD_GPIO 13 // RXD Pin on ESP32

// UART Konfiguration
#define E32_UART_PORT UART_NUM_1
#define BUF_SIZE 1024

#define CONFIG_CMD_LEN 6
#define RESPONSE_LEN 6

// forward declaration
void wait_for_aux();
void set_mode(uint8_t m0, uint8_t m1);
void init_io(void);
void get_config(void);
esp_err_t e32_send_data(const uint8_t *data, size_t len);
void decode_config(uint8_t *data, int len);

void app_main(void)
{
    // esp_log_level_set("*", ESP_LOG_WARN);  // Nur INFO und höher (WARN, ERROR)
    esp_log_level_set("LORA_Sender", ESP_LOG_INFO); // Nur INFO und höher (WARN, ERROR)
    ESP_LOGI(TAG, "LoRAESPIDFSender V0.7");
#if CONFIG_DEBUG_LORA
    ESP_LOGI(TAG, "Debug Lora enabled");
#endif
    init_io();                     // initialize IO pins
    get_config();                  // read configuration from E32 module
    vTaskDelay(pdMS_TO_TICKS(50)); // wait for command to be processed
                                   // continously send a sample message to E32 module
                                   /*     while (1)
                                       {
                                           ESP_LOGI(TAG, "send sample message");
                                           char *test_msg = "Hello LoRa!\n";
                                           ESP_ERROR_CHECK(e32_send_data((uint8_t *)test_msg, strlen(test_msg)));
                                           vTaskDelay(pdMS_TO_TICKS(1000)); // delay for 1 second
                                       } */

    ESP_LOGI(TAG, "ready.");
}

void wait_for_aux()
{
    // AUX is HIGH, when module is ready
    while (!gpio_get_level(E32_AUX_GPIO))
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void set_mode(uint8_t m0, uint8_t m1)
{
    gpio_set_level(E32_M0_GPIO, m0);
    gpio_set_level(E32_M1_GPIO, m1);
    vTaskDelay(pdMS_TO_TICKS(50));
    wait_for_aux();
}

// send data to E32 module
esp_err_t e32_send_data(const uint8_t *data, size_t len)
{
    int bytes_written = uart_write_bytes(E32_UART_PORT, (const char *)data, len);
    ESP_LOGI(TAG, "%d Bytes send", len);
    return (bytes_written == len) ? ESP_OK : ESP_FAIL;
}

void init_io()
{
    ESP_LOGI(TAG, "Initialisiere IO-Pins");
    // configure command pins M0 and M1
    gpio_config_t mode_conf = {
        .intr_type = GPIO_INTR_DISABLE,                                // no interrupt
        .mode = GPIO_MODE_OUTPUT,                                      // set as output mode
        .pin_bit_mask = (1ULL << E32_M0_GPIO) | (1ULL << E32_M1_GPIO), // bit mask of the pins, use a bit for each pin
        .pull_down_en = GPIO_PULLDOWN_DISABLE,                         // disable pull-down mode
        .pull_up_en = GPIO_PULLUP_DISABLE                              // disable pull-up mode
    };
    gpio_config(&mode_conf);
    // configure AUX pin
    gpio_config_t aux_conf = {
        .intr_type = GPIO_INTR_DISABLE,         // no interrupt
        .mode = GPIO_MODE_INPUT,                // set as input mode
        .pin_bit_mask = (1ULL << E32_AUX_GPIO), // bit mask of the pins, use a bit for each pin
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // disable pull-down mode
        .pull_up_en = GPIO_PULLUP_DISABLE       // disable pull-up mode
    };
    gpio_config(&aux_conf);

    // configure UART with the given settings
    uart_config_t uart_config = {
        .baud_rate = 9600,                     // baud rate
        .data_bits = UART_DATA_8_BITS,         // data bits
        .parity = UART_PARITY_DISABLE,         // no parity
        .stop_bits = UART_STOP_BITS_1,         // stop bits
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // no flow control
    };
    uart_driver_install(E32_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);                                 // install UART driver
    uart_param_config(E32_UART_PORT, &uart_config);                                                  // configure UART parameters
    uart_set_pin(E32_UART_PORT, E32_TXD_GPIO, E32_RXD_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // set UART pins
#if CONFIG_DEBUG_LORA
    gpio_dump_io_configuration(stdout, (1ULL << 10) | (1ULL << 11) | (1ULL << 12) | (1ULL << 13) | (1ULL << 14));
    ESP_LOGI(TAG, "GPIO M0: %d, M1: %d, TXD: %d, RXD: %d, AUX: %d", E32_M0_GPIO, E32_M1_GPIO, E32_TXD_GPIO, E32_RXD_GPIO, E32_AUX_GPIO);
#endif
}

void get_config()
{
    // read configuration from E32 module and print it in hex format
    uint8_t read_cmd[] = {0xC1, 0xC1, 0xC1}; // command to read configuration
    uint8_t response[RESPONSE_LEN] = {0};

    ESP_LOGI(TAG, "set programming mode");
    set_mode(1, 1);

    ESP_LOGI(TAG, "Send configuration read command");
    ESP_ERROR_CHECK(e32_send_data(read_cmd, sizeof(read_cmd)));
    vTaskDelay(pdMS_TO_TICKS(50)); // wait for command to be processed

    wait_for_aux();
    uint8_t rx_buffer[BUF_SIZE];
    int len = uart_read_bytes(E32_UART_PORT, rx_buffer, BUF_SIZE, pdMS_TO_TICKS(200));
    if (len > 0)
    {
        ESP_LOGI(TAG, "Configuration received (%d bytes):", len);
        for (int i = 0; i < len; i++)
            printf("%02X ", rx_buffer[i]);
        printf("\n");

        decode_config(rx_buffer, len); // Decode and print config in plain text
    }
    else
    {
        ESP_LOGW(TAG, "No response from E32 module");
    }

    set_mode(0, 0); // Set back to normal mode (M0=0, M1=0)
    printf("configurationTest 1 Default \n");
    uint8_t rx_buffer1[] = {0xC0, 0x00, 0x00, 0x1A, 0x17, 0x44};
    decode_config(rx_buffer1, 6); // Decode and print config in plain text

    printf("configurationTest 2 \n");
    uint8_t rx_buffer2[] = {0xC0, 0x1A, 0x07, 0x78, 0x03, 0xAA};
    decode_config(rx_buffer2, 6); // Decode and print config in plain text

    printf("configurationTest 3 \n");
    uint8_t rx_buffer3[] = {0xC0, 0x0A, 0x35, 0xA9, 0x07, 0x4D};
    decode_config(rx_buffer3, 6); // Decode and print config in plain text

    printf("configurationTest 4 \n");
    uint8_t rx_buffer4[] = {0xC0, 0x1F, 0xFE, 0x33, 0x01, 0x93};
    decode_config(rx_buffer4, 6); // Decode and print config in plain text

    printf("configurationTest 5 \n");
    uint8_t rx_buffer5[] = {0xC0, 0x01, 0x00, 0xE2, 0x05, 0x5C};
    decode_config(rx_buffer5, 6); // Decode and print config in plain text

    printf("configurationTest 6 \n");
    uint8_t rx_buffer6[] = {0xC0, 0x02, 0x05, 0x5C, 0x06, 0xA2};
    decode_config(rx_buffer6, 6); // Decode and print config in plain text
}

void decode_config(uint8_t *data, int len)
{
    if (len < 6)
    {
        printf("Invalid configuration data length: %d bytes\n", len);
        return;
    }

    for (int i = 0; i < len; i++)
        printf("%02X ", data[i]);
    printf("\n");

    // Extract fields based on E32 response format
    uint8_t header = data[0];
    uint16_t address = (data[1] << 8) | data[2];
    uint8_t sped = data[3];
    uint8_t channel = data[4];
    uint8_t option = data[5];

    channel = channel & 0x1F; // Bit 0-4 are channel number, bit 5-7 is reserved

    // Parse SPED byte
    const char *uart_parity_bit[] = {
        "8N1", "8O1", "8E1", "8N1(11)"};
    const char *uart_baudrates[] = {
        "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"};
    const char *air_rates[] = {
        "0.3 kbps", "1.2 kbps", "2.4 kbps", "4.8 kbps", "9.6 kbps", "19.2 kbps", "Invalid", "Invalid"};

    uint8_t uart_baud = (sped & 0x38) >> 3; // Mask to get the UART baud rate bits
    uint8_t air_rate = (sped & 0x7);
    uint8_t uart_parity = (sped & 0xC0) >> 6; // Mask to get the UART parity bits

    // Parse OPTION byte
    const char *transmission_mode_str[] = {
        "Transparent", "Fixed", "Reserved", "Reserved"};
    const char *io_mode_str[] = {
        "TXD, AUX OpenColOut,  RXD OpenColIn", "TXD, AUX PushPullOut, RXD PullUpIn" };
    const char *tx_power_str[] = {
        "30 dBm", "27 dBm", "24 dBm", "21 dBm"};

    uint8_t transmission_mode = (option & 0x80) >> 7; // Mask to get the I/O mode bits
    uint8_t io_mode = (option & 0x40) >> 6;           // Mask to get the I/O mode bits
    uint8_t wakeup_time = (option & 0x38) >> 3;       // Mask to get the wakeup time bits
    uint8_t fec_enabled = (option & 0x4) >> 2;        // Mask to get the FEC enabled bits
    uint8_t tx_power = (option & 0x3) ;           // Mask to get the TX power bits

  

    printf("E32 Module Configuration:\n");
    printf("Header: 0x%02X\n", header);
    printf("Address: 0x%04X\n", address);
    printf("UART Parity: %s \n", uart_parity < 4 ? uart_parity_bit[uart_parity] : "Unknown");
    printf("UART Baud Rate: %s bps\n", uart_baud < 8 ? uart_baudrates[uart_baud] : "Unknown");
    printf("Air Data Rate: %s\n", air_rate < 6 ? air_rates[air_rate] : "Unknown");
    printf("Channel: %d (%.1f MHz)\n", channel, 862.0 + channel);
    printf("Transmission Mode: %s\n", transmission_mode_str[transmission_mode]);
    printf("I/O Mode: %s\n", io_mode_str[io_mode]);
    printf("Wakeup Time: %d ms\n", (wakeup_time + 1) * 250); // Wakeup time in ms
    printf("FEC Enabled: %s\n", fec_enabled ? "Yes" : "No");
    printf("TX Power: %s\n", tx_power_str[tx_power]);
}
