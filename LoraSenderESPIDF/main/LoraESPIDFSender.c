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
  20250407:V0.4: send configuration command to E32-900T30D
  20250407:V0.5: use function to send data to E32 module

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

/* Set the GPIO level according to the state (LOW or HIGH)*/
// gpio_set_level(BLINK_GPIO, s_led_state);

void wait_for_aux()
{
    // AUX ist HIGH, wenn das Modul bereit ist
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
    //  wait_for_aux();
}

// send data to E32 module
esp_err_t e32_send_data(const uint8_t *data, size_t len)
{
    int bytes_written = uart_write_bytes(E32_UART_PORT, (const char *)data, len);
    ESP_LOGI(TAG, "%d Bytes gesendet", len);
    return (bytes_written == len) ? ESP_OK : ESP_FAIL;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starte E32 Sender");
    // now set direction with gpio_config
    gpio_config_t mode_conf = {
        .intr_type = GPIO_INTR_DISABLE,                                // no interrupt
        .mode = GPIO_MODE_OUTPUT,                                      // set as output mode
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
    uart_driver_install(E32_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);                                 // install UART driver
    uart_param_config(E32_UART_PORT, &uart_config);                                                  // configure UART parameters
    uart_set_pin(E32_UART_PORT, E32_TXD_GPIO, E32_RXD_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // set UART pins

    /*
    gpio_set_direction(E32_M0_GPIO, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(E32_M1_GPIO, GPIO_MODE_INPUT_OUTPUT);
    */
    gpio_dump_io_configuration(stdout, (1ULL << 10) | (1ULL << 11) | (1ULL << 12) | (1ULL << 13) | (1ULL << 14));
    /*
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
        */

    ESP_LOGI(TAG, "Setze in Programmiermodus");
    set_mode(1, 1);

    uint8_t read_cmd[] = {0xC1, 0xC1, 0xC1};
    uint8_t response[RESPONSE_LEN] = {0};

    ESP_LOGI(TAG, "Sende Konfigurationsabfrage");
    ESP_ERROR_CHECK(e32_send_data(read_cmd, sizeof(read_cmd)));
    vTaskDelay(pdMS_TO_TICKS(50)); // Warten bis Befehl verarbeitet

    wait_for_aux();

    uint8_t rx_buffer[BUF_SIZE];
    int len = uart_read_bytes(E32_UART_PORT, rx_buffer, BUF_SIZE, pdMS_TO_TICKS(200));
    if (len > 0)
    {
        ESP_LOGI(TAG, "Antwort (%d Byte):", len);
        for (int i = 0; i < len; i++)
        {
            printf("%02X ", rx_buffer[i]);
        }
        printf("\n");
    }
    else
    {
        ESP_LOGW(TAG, "Keine Antwort erhalten");
    }
    set_mode(0, 0); // Setze in normalen Modus
    vTaskDelay(pdMS_TO_TICKS(50)); // Warten bis Befehl verarbeitet
    while (1)
    {
        /* code */

        ESP_LOGI(TAG, "Beispielnachricht senden");
        char *test_msg = "Hello LoRa!\n";
        ESP_ERROR_CHECK(e32_send_data((uint8_t *)test_msg, strlen(test_msg)));
        vTaskDelay(pdMS_TO_TICKS(1000)); // Warten bis Befehl verarbeitet
    }

    ESP_LOGI(TAG, "Fertig.");
}
