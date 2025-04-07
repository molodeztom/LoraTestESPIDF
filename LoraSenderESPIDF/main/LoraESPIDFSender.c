/**************************************************************************
 E32-900T30D LoRa Sender


  Hardware:
  ESP32-S3-DevKitC-1 mit Wroom N16R8
  LoRa E32-900T30D connected M0 M1 and Rx Tx

  Try sending a message to remote LoRa


  History: master if not shown otherwise
  20250406:a64le: initial version
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

#define BLINK_GPIO CONFIG_BLINK_GPIO
// forward declaration
void handle_e32_error(esp_err_t err, const char *operation);


static led_strip_handle_t led_strip;

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");

    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };

    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
}

    /* Set the GPIO level according to the state (LOW or HIGH)*/
    //gpio_set_level(BLINK_GPIO, s_led_state);


/*
void wait_for_aux()
{
    // AUX ist HIGH, wenn das Modul bereit ist
    while (!gpio_get_level(E32_AUX_GPIO))
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// helper to set the mode of the E32 module
//  0 = normal mode, 1 = sleep mode, 2 = wake up mode
void e32_set_mode(uint8_t m0, uint8_t m1)
{
    gpio_set_level(E32_M0_GPIO, m0);
    gpio_set_level(E32_M1_GPIO, m1);
    vTaskDelay(pdMS_TO_TICKS(200)); // Warten bis Moduswechsel abgeschlossen
                                    // wait_for_aux();
}

void initUART()
{
    esp_err_t ret;
    // GPIOs initialisieren
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << E32_M0_GPIO) | (1ULL << E32_M1_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // <-- Wichtig!
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "GPIO Konfiguration fehlgeschlagen: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "GPIOs konfiguriert");

    ESP_LOGI(TAG, "Setze M0 und M1 jetzt manuell auf HIGH");
    gpio_set_level(E32_M0_GPIO, 1);
    gpio_set_level(E32_M1_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(200)); // kurz warten
    ESP_LOGI(TAG, "M0 Level nach setzen: %d", gpio_get_level(E32_M0_GPIO));
    ESP_LOGI(TAG, "M1 Level nach setzen: %d", gpio_get_level(E32_M1_GPIO));

    // AUX Pin als Input konfigurieren (optional)
    gpio_config_t aux_conf = {
        .pin_bit_mask = (1ULL << E32_AUX_GPIO),
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&aux_conf);

    // UART initialisieren
    uart_config_t uart_config = {
        .baud_rate = 9600, // Standard-Baudrate des Moduls
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_driver_install(E32_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(E32_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(E32_UART_PORT, E32_TXD_GPIO, E32_RXD_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "GPIO and UART initialisiert");
}

// Daten senden
esp_err_t e32_send_data(const uint8_t *data, size_t len)
{
    int bytes_written = uart_write_bytes(E32_UART_PORT, (const char *)data, len);
    return (bytes_written == len) ? ESP_OK : ESP_FAIL;
}

// Daten empfangen mit Timeout
esp_err_t e32_receive_data(uint8_t *buffer, size_t len, uint32_t timeout_ms)
{
    int bytes_read = uart_read_bytes(E32_UART_PORT, buffer, len, pdMS_TO_TICKS(timeout_ms));
    return (bytes_read == len) ? ESP_OK : ESP_FAIL;
}
    */

void handle_e32_error(esp_err_t err, const char *operation)
{
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "%s erfolgreich", operation);
        return;
    }

    // Speziell für UART-Fehler
    if (err == ESP_FAIL)
    {
        ESP_LOGE(TAG, "%s fehlgeschlagen (allgemeiner Fehler)", operation);
        return;
    }

    // Dekodierung des Fehlercodes
    const char *err_name = esp_err_to_name(err);
    const char *err_desc = "";

    // Benutzerdefinierte Fehlerbeschreibungen
    switch (err)
    {
    case ESP_ERR_INVALID_ARG:
        err_desc = "Ungültiges Argument";
        break;
    case ESP_ERR_TIMEOUT:
        err_desc = "Timeout aufgetreten";
        break;
    case ESP_ERR_NOT_SUPPORTED:
        err_desc = "Funktion nicht unterstützt";
        break;
    case ESP_ERR_NO_MEM:
        err_desc = "Kein Speicher verfügbar";
        break;
    default:
        err_desc = "Unbekannter Fehler";
    }

    ESP_LOGE(TAG, "Fehler bei %s: %s (0x%x) - %s",
             operation, err_name, err, err_desc);

    // Bei kritischen Fehlern evtl. Neustart
    if (err == ESP_ERR_NO_MEM || err == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Kritischer Fehler, Neustart...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
}
    void app_main(void)
{
    ESP_LOGI(TAG, "Starte E32 Sender");
    //configure_led();
    //led_strip_set_pixel(led_strip, 0, 0, 0, 0);
    /* Refresh the strip to send data */
    //led_strip_refresh(led_strip);
    

    while (1)
    {
        // set M0 manually to HIGH
        gpio_set_level(E32_M0_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(200)); // kurz warten
        ESP_LOGI(TAG, "M0 Level nach setzen: %d", gpio_get_level(E32_M0_GPIO));
        // set M0 manually to LOW
        gpio_set_level(E32_M0_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(200)); // kurz warten
        ESP_LOGI(TAG, "M0 Level nach setzen: %d", gpio_get_level(E32_M0_GPIO));

        /* Configure the peripheral according to the LED type */
        // configure_led();
        /*ESP_LOGI(TAG, "InitUART");
        initUART();

        ESP_LOGI(TAG, "Setze in Programmiermodus");
        e32_set_mode(1, 1);

        ESP_LOGI(TAG, "M0 Level: %d", gpio_get_level(E32_M0_GPIO));
        ESP_LOGI(TAG, "M1 Level: %d", gpio_get_level(E32_M1_GPIO));
        ESP_LOGI(TAG, "AUX Level: %d", gpio_get_level(E32_AUX_GPIO));

        uint8_t cmd[] = {0xC1, 0x00, 0x09};
        uart_write_bytes(E32_UART_PORT, (const char *)cmd, sizeof(cmd));
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
        }*/

        ESP_LOGI(TAG, "Fertig.");

        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
