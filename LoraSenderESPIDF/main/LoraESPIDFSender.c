/**************************************************************************
 E32-900T30D LoRa Sender


  Hardware:
  ESP32-S3-DevKitC-1 mit Wroom N16R8
  LoRa E32-900T30D connected M0 M1 and Rx Tx

  Try sending a message to remote LoRa


  History: master if not shown otherwise
  20250406:V0.1: initial version
  20250407:V0.2: added M0 and M1 control
  20250407:V0.3: added UART control
  20250407:V0.4:
  */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "led_strip.h"

static const char *TAG = "LORA_Sender";

// Konfiguration der Pins
#define E32_M0_GPIO 10
#define E32_M1_GPIO 11
#define E32_AUX_GPIO 14
#define E32_TXD_GPIO 13
#define E32_RXD_GPIO 12

// UART Konfiguration
#define E32_UART_PORT UART_NUM_1
#define BUF_SIZE 128

#define CONFIG_CMD_LEN 6
#define RESPONSE_LEN 6

/* Set the GPIO level according to the state (LOW or HIGH)*/
// gpio_set_level(BLINK_GPIO, s_led_state);

void app_main(void)
{
    ESP_LOGI(TAG, "Starte E32 Sender");
    // now set direction with gpio_config
    gpio_config_t mode_conf = {
        .intr_type = GPIO_INTR_DISABLE,                                // no interrupt
        .mode = GPIO_MODE_INPUT_OUTPUT,                                // set as output mode
        .pin_bit_mask = (1ULL << E32_M0_GPIO) | (1ULL << E32_M1_GPIO), // bit mask of the pins, use a bit for each pin
        .pull_down_en = GPIO_PULLDOWN_DISABLE,                         // disable pull-down mode
        .pull_up_en = GPIO_PULLUP_DISABLE                              // disable pull-up mode
    };
    // configure GPIO with the given settings
    gpio_config(&mode_conf);
    gpio_config_t aux_conf = {
        .intr_type = GPIO_INTR_DISABLE,         // no interrupt
        .mode = GPIO_MODE_INPUT,                // set as input mode
        .pin_bit_mask = (1ULL << E32_AUX_GPIO), // bit mask of the pins, use a bit for each pin
        .pull_down_en = GPIO_PULLDOWN_DISABLE,  // disable pull-down mode
        .pull_up_en = GPIO_PULLUP_DISABLE       // disable pull-up mode
    };
    // configure GPIO with the given settings
    gpio_config(&aux_conf);

    // configure UART with the given settings
    uart_config_t uart_config = {
        .baud_rate = 9600,                     // baud rate
        .data_bits = UART_DATA_8_BITS,         // data bits
        .parity = UART_PARITY_DISABLE,         // no parity
        .stop_bits = UART_STOP_BITS_1,         // stop bits
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // no flow control
    };
    // configure UART with the given settings
    uart_param_config(E32_UART_PORT, &uart_config);                                                  // configure UART parameters
    uart_set_pin(E32_UART_PORT, E32_TXD_GPIO, E32_RXD_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // set UART pins
    uart_driver_install(E32_UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0);                              // install UART driver

    /*
    gpio_set_direction(E32_M0_GPIO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(E32_M1_GPIO, GPIO_MODE_INPUT_OUTPUT);
    */
    gpio_dump_io_configuration(stdout, (1ULL << 10) | (1ULL << 11) | (1ULL << 12) | (1ULL << 13) | (1ULL << 14));

    while (1)
    {
        // set M0 manually to HIGH
        gpio_set_level(E32_M0_GPIO, 1);
        gpio_set_level(E32_M1_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500)); // kurz warten
        ESP_LOGI(TAG, "M0 Level nach setzen: %d", gpio_get_level(E32_M0_GPIO));
        ESP_LOGI(TAG, "M1 Level nach löschen: %d", gpio_get_level(E32_M1_GPIO));
        // set M0 manually to LOW
        gpio_set_level(E32_M0_GPIO, 0);
        gpio_set_level(E32_M1_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500)); // kurz warten
        ESP_LOGI(TAG, "M0 Level nach löschen: %d", gpio_get_level(E32_M0_GPIO));
        ESP_LOGI(TAG, "M1 Level nach setzen: %d", gpio_get_level(E32_M1_GPIO));

        /*

         ESP_LOGI(TAG, "M0 Level: %d", gpio_get_level(E32_M0_GPIO));
         ESP_LOGI(TAG, "M1 Level: %d", gpio_get_level(E32_M1_GPIO));
         ESP_LOGI(TAG, "AUX Level: %d", gpio_get_level(E32_AUX_GPIO));

         */

        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
