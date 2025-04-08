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
  20240409:V0.7: added function to read configuration from E32 module

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
#define E32_TXD_GPIO 12
#define E32_RXD_GPIO 13

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
        ESP_LOGI(TAG, "configuration is (%d Byte):", len);
        for (int i = 0; i < len; i++)
        {
            printf("%02X ", rx_buffer[i]);
        }
        printf("\n");
    }
    else
    {
        ESP_LOGW(TAG, "No response from E32 module");
    }
    set_mode(0, 0); // set normal mode
}

void app_main(void)
{
    // esp_log_level_set("*", ESP_LOG_WARN);  // Nur INFO und höher (WARN, ERROR)
    esp_log_level_set("LORA_Sender", ESP_LOG_INFO); // Nur INFO und höher (WARN, ERROR)
    ESP_LOGI(TAG, "LoRAESPIDFSender V0.6");
#if CONFIG_DEBUG_LORA
    ESP_LOGI(TAG, "Debug Lora enabled");
#endif
    init_io();                     // initialize IO pins
    get_config();                  // read configuration from E32 module
    vTaskDelay(pdMS_TO_TICKS(50)); // wait for command to be processed
                                   // continously send a sample message to E32 module
    while (1)
    {
        ESP_LOGI(TAG, "send sample message");
        char *test_msg = "Hello LoRa!\n";
        ESP_ERROR_CHECK(e32_send_data((uint8_t *)test_msg, strlen(test_msg)));
        vTaskDelay(pdMS_TO_TICKS(1000)); // delay for 1 second
    }

    ESP_LOGI(TAG, "ready.");
}
