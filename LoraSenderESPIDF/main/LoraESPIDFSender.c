/**************************************************************************
 E32-900T30D LoRa Sender


  Hardware:
  ESP32-S3-DevKitC-1 mit Wroom N16R8
  LoRa E32-900T30D connected M0 M1 and Rx Tx

  Try sending a message to remote LoRa


  History: master if not shown otherwise
  20250406:V0.1 a64le: initial version
  20250407:V0.2 a64le: added M0 and M1 control
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
#define E32_UART_PORT UART_NUM_2
#define BUF_SIZE 128

#define CONFIG_CMD_LEN 6
#define RESPONSE_LEN 6


    /* Set the GPIO level according to the state (LOW or HIGH)*/
    //gpio_set_level(BLINK_GPIO, s_led_state);

    void app_main(void)
{
    ESP_LOGI(TAG, "Starte E32 Sender");
    gpio_set_direction(E32_M0_GPIO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(E32_M1_GPIO, GPIO_MODE_INPUT_OUTPUT);
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
