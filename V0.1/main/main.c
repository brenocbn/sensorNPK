/**
 * @file main.c
 * @brief ESP32-C3 SuperMini Soil Sensor Modbus-RTU reader over RS485.
 * 
 * Hardware:
 *  - ESP32-C3 SuperMini
 *  - MAX485 Transceiver (DI -> GPIO4, RO -> GPIO5, DE/RE -> GPIO6)
 *  - Baud Rate: 9600, 8-N-1, Half-Duplex via Hardware RTS.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define TAG "SOIL_SENSOR"

/* =========================================================================
 * Hardware & UART Definitions
 * ========================================================================= */
#define RS485_UART_NUM          (UART_NUM_1)
#define RS485_TXD_PIN           (GPIO_NUM_4)
#define RS485_RXD_PIN           (GPIO_NUM_5)
#define RS485_RTS_PIN           (GPIO_NUM_6)
#define RS485_CTS_PIN           (UART_PIN_NO_CHANGE)

#define RS485_BAUD_RATE         (9600)
#define RS485_BUF_SIZE          (256)
#define RS485_READ_TIMEOUT_MS   (500)
#define POLLING_INTERVAL_MS     (4000)

/* =========================================================================
 * Modbus Frame Definitions
 * ========================================================================= */
#define MODBUS_EXPECTED_RESP_LEN (21)

// Query: Addr 0x00, Func 0x03, StartReg 0x0000, RegCount 0x0008, CRC: 0xDD45
static const uint8_t s_query_frame[] = {
    0x00, 0x03, 0x00, 0x00, 0x00, 0x08, 0x45, 0xDD
};

typedef struct {
    float    temperature;   /**< Unit: °C */
    float    moisture;      /**< Unit: % RH */
    uint16_t ec;            /**< Unit: µS/cm */
    uint16_t salinity;      /**< Unit: mg/L or raw index */
    uint16_t nitrogen;      /**< Unit: mg/kg */
    uint16_t phosphorus;    /**< Unit: mg/kg */
    uint16_t potassium;     /**< Unit: mg/kg */
    float    ph;            /**< Unit: pH */
} soil_metrics_t;

/* =========================================================================
 * CRC16 Calculation (Modbus standard: Poly 0xA001, Init 0xFFFF)
 * ========================================================================= */
static uint16_t modbus_crc16(const uint8_t *buffer, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)buffer[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* =========================================================================
 * RS-485 UART Initialization
 * ========================================================================= */
static esp_err_t rs485_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate  = RS485_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(RS485_UART_NUM, RS485_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART_NUM, RS485_TXD_PIN, RS485_RXD_PIN, RS485_RTS_PIN, RS485_CTS_PIN));
    
    // Enable Hardware RS485 Half-Duplex direction control on the RTS pin
    ESP_ERROR_CHECK(uart_set_mode(RS485_UART_NUM, UART_MODE_RS485_HALF_DUPLEX));

    ESP_LOGI(TAG, "RS485 initialized: TX=GPIO%d, RX=GPIO%d, RTS=GPIO%d, Baud=%d", 
             RS485_TXD_PIN, RS485_RXD_PIN, RS485_RTS_PIN, RS485_BAUD_RATE);
    return ESP_OK;
}

/* =========================================================================
 * Sensor Query & Parse Routine
 * ========================================================================= */
static esp_err_t soil_sensor_read(soil_metrics_t *metrics)
{
    if (metrics == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Flush any stale bytes inside RX FIFO before transmission
    ESP_ERROR_CHECK(uart_flush_input(RS485_UART_NUM));

    // Send the Modbus request packet
    int tx_bytes = uart_write_bytes(RS485_UART_NUM, (const char *)s_query_frame, sizeof(s_query_frame));
    if (tx_bytes != sizeof(s_query_frame)) {
        ESP_LOGE(TAG, "Failed to write complete request frame");
        return ESP_FAIL;
    }

    // Allocate static buffer on the task stack
    uint8_t rx_buf[RS485_BUF_SIZE];
    int rx_len = uart_read_bytes(RS485_UART_NUM, rx_buf, MODBUS_EXPECTED_RESP_LEN, pdMS_TO_TICKS(RS485_READ_TIMEOUT_MS));

    if (rx_len <= 0) {
        ESP_LOGW(TAG, "Sensor read timeout (no response)");
        return ESP_ERR_TIMEOUT;
    }

    if (rx_len < MODBUS_EXPECTED_RESP_LEN) {
        ESP_LOGW(TAG, "Incomplete packet: expected %d bytes, got %d bytes", MODBUS_EXPECTED_RESP_LEN, rx_len);
        return ESP_ERR_INVALID_SIZE;
    }

    // Check if device reported a Modbus exception (Bit 7 set on function code)
    if (rx_buf[1] & 0x80) {
        ESP_LOGE(TAG, "Modbus exception response: code 0x%02X", rx_buf[2]);
        return ESP_FAIL;
    }

    // Validate Modbus CRC16
    uint16_t computed_crc = modbus_crc16(rx_buf, MODBUS_EXPECTED_RESP_LEN - 2);
    uint16_t received_crc = (uint16_t)rx_buf[MODBUS_EXPECTED_RESP_LEN - 2] | 
                           ((uint16_t)rx_buf[MODBUS_EXPECTED_RESP_LEN - 1] << 8);

    if (computed_crc != received_crc) {
        ESP_LOGE(TAG, "CRC validation failure: computed 0x%04X != received 0x%04X", computed_crc, received_crc);
        return ESP_ERR_INVALID_CRC;
    }

    // Confirm function code (0x03) and payload byte count (16 bytes = 0x10)
    if (rx_buf[1] != 0x03 || rx_buf[2] != 0x10) {
        ESP_LOGE(TAG, "Unexpected function or byte count: 0x%02X, 0x%02X", rx_buf[1], rx_buf[2]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* =====================================================================
     * Register Parsing (Big-Endian network order)
     * ===================================================================== */
    // Register 0: Soil Temperature (signed 16-bit, resolution 0.1°C)
    int16_t raw_temp = (int16_t)((rx_buf[3] << 8) | rx_buf[4]);
    metrics->temperature = (float)raw_temp / 10.0f;

    // Register 1: Soil Moisture (resolution 0.1% RH)
    uint16_t raw_moist = (uint16_t)((rx_buf[5] << 8) | rx_buf[6]);
    metrics->moisture = (float)raw_moist / 10.0f;

    // Register 2: Electrical Conductivity (EC, 1 µS/cm)
    metrics->ec = (uint16_t)((rx_buf[7] << 8) | rx_buf[8]);

    // Register 3: Salinity (Salt index)
    metrics->salinity = (uint16_t)((rx_buf[9] << 8) | rx_buf[10]);

    // Register 4: Nitrogen (mg/kg)
    metrics->nitrogen = (uint16_t)((rx_buf[11] << 8) | rx_buf[12]);

    // Register 5: Phosphorus (mg/kg)
    metrics->phosphorus = (uint16_t)((rx_buf[13] << 8) | rx_buf[14]);

    // Register 6: Potassium (mg/kg)
    metrics->potassium = (uint16_t)((rx_buf[15] << 8) | rx_buf[16]);

    // Register 7: Soil pH (resolution 0.1)
    uint16_t raw_ph = (uint16_t)((rx_buf[17] << 8) | rx_buf[18]);
    metrics->ph = (float)raw_ph / 10.0f;

    return ESP_OK;
}

/* =========================================================================
 * Background Polling Task
 * ========================================================================= */
static void soil_sensor_task(void *pvParameters)
{
    soil_metrics_t metrics;

    while (1) {
        esp_err_t err = soil_sensor_read(&metrics);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "------------------ Soil Sensor Readings ------------------");
            ESP_LOGI(TAG, " Temperature : %.1f °C", metrics.temperature);
            ESP_LOGI(TAG, " Moisture    : %.1f %%", metrics.moisture);
            ESP_LOGI(TAG, " EC          : %u µS/cm", metrics.ec);
            ESP_LOGI(TAG, " Salinity    : %u mg/L", metrics.salinity);
            ESP_LOGI(TAG, " Nitrogen (N): %u mg/kg", metrics.nitrogen);
            ESP_LOGI(TAG, " Phosph. (P) : %u mg/kg", metrics.phosphorus);
            ESP_LOGI(TAG, " Potass. (K) : %u mg/kg", metrics.potassium);
            ESP_LOGI(TAG, " pH Level    : %.1f", metrics.ph);
            ESP_LOGI(TAG, "----------------------------------------------------------");
        } else {
            ESP_LOGE(TAG, "Sensor acquisition failed (%s)", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(POLLING_INTERVAL_MS));
    }
}

/* =========================================================================
 * Application Entrypoint
 * ========================================================================= */
void app_main(void)
{
    ESP_LOGI(TAG, "Starting Soil Multiparameter RS485 Station on ESP32-C3 SuperMini...");

    // Initialize UART in Half-Duplex RS485 Mode
    ESP_ERROR_CHECK(rs485_init());

    // Create polling task with a 4KB stack size
    BaseType_t ret = xTaskCreate(
        soil_sensor_task,
        "soil_sensor_task",
        4096,
        NULL,
        tskIDLE_PRIORITY + 2,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create soil_sensor_task!");
    }
}