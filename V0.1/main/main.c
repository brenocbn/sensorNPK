/**
 * @file main.c
 * @brief ESP32-C3 SuperMini Soil Sensor Station (Modbus-RTU + SoftAP + Web Dashboard)
 * Target Framework: Native ESP-IDF v6.0.2
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define TAG "SOIL_STATION"

/* =========================================================================
 * Hardware & Configuration
 * ========================================================================= */
#define RS485_UART_PORT         (UART_NUM_1)
#define RS485_TXD_PIN           (GPIO_NUM_4) // Connected to DI/TXD of HW-519
#define RS485_RXD_PIN           (GPIO_NUM_5) // Connected to RO/RXD of HW-519
#define RS485_BAUD_RATE         (9600)
#define UART_RX_BUF_SIZE        (256)

#define WIFI_AP_SSID            "Sensor_NPK_Pro"
#define WIFI_AP_PASSWORD        "password123"
#define WIFI_AP_MAX_CONN        (4)
#define WIFI_AP_CHANNEL         (1)

#define MODBUS_READ_TIMEOUT_MS  (500)
#define POLLING_INTERVAL_MS     (3500)

/* =========================================================================
 * Data Structures & Global State
 * ========================================================================= */
typedef struct {
    float    temperature;   /**< Unit: °C */
    float    moisture;      /**< Unit: % RH */
    uint16_t ec;            /**< Unit: µS/cm */
    uint16_t salinity;      /**< Unit: mg/L / index */
    uint16_t nitrogen;      /**< Unit: mg/kg */
    uint16_t phosphorus;    /**< Unit: mg/kg */
    uint16_t potassium;     /**< Unit: mg/kg */
    float    ph;            /**< Unit: pH */
    bool     valid;         /**< Frame validation flag */
} soil_data_t;

static soil_data_t      s_sensor_data = {0};
static SemaphoreHandle_t s_data_mutex  = NULL;

// Modbus Query: Addr 0x00, Func 0x03, StartReg 0x0000, RegCount 0x0008, CRC: 0xDD45
static const uint8_t s_modbus_query_frame[] = {
    0x00, 0x03, 0x00, 0x00, 0x00, 0x08, 0x45, 0xDD
};

/* =========================================================================
 * Embedded Web Page (HTML/CSS/JS Responsive Dashboard)
 * ========================================================================= */
static const char s_index_html[] = 
"<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1.0\">"
"<title>Soil NPK Pro Dashboard</title>"
"<style>"
":root{--bg:#090d16;--card:#131c2e;--card-border:#1e293b;--text:#f8fafc;--sub:#94a3b8;--accent:#10b981;--warn:#f59e0b;}"
"*{margin:0;padding:0;box-sizing:border-box;font-family:-apple-system,BlinkMacSystemFont,\"Segoe UI\",Roboto,sans-serif;}"
"body{background:var(--bg);color:var(--text);padding:1.5rem;max-width:1100px;margin:0 auto;}"
"header{display:flex;justify-content:space-between;align-items:center;margin-bottom:2rem;border-bottom:1px solid var(--card-border);padding-bottom:1rem;}"
".title{font-size:1.5rem;font-weight:700;color:var(--text);display:flex;align-items:center;gap:0.5rem;}"
".title span{color:var(--accent);}"
".badge{padding:0.35rem 0.75rem;border-radius:9999px;font-size:0.8rem;font-weight:600;text-transform:uppercase;}"
".badge.live{background:rgba(16,185,129,0.15);color:var(--accent);border:1px solid var(--accent);}"
".badge.err{background:rgba(239,68,68,0.15);color:#ef4444;border:1px solid #ef4444;}"
".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:1.25rem;}"
".card{background:var(--card);border:1px solid var(--card-border);border-radius:1rem;padding:1.25rem;position:relative;overflow:hidden;box-shadow:0 4px 6px -1px rgba(0,0,0,0.3);}"
".card::before{content:'';position:absolute;top:0;left:0;right:0;height:3px;background:var(--card-accent,var(--accent));}"
".card-header{font-size:0.85rem;color:var(--sub);margin-bottom:0.5rem;text-transform:uppercase;letter-spacing:0.05em;font-weight:600;}"
".card-value{font-size:2.2rem;font-weight:700;color:#fff;display:flex;align-items:baseline;gap:0.25rem;}"
".card-unit{font-size:0.9rem;font-weight:400;color:var(--sub);}"
"footer{margin-top:2rem;text-align:center;font-size:0.8rem;color:var(--sub);}"
"</style></head><body>"
"<header>"
"  <div class=\"title\">🌱 Soil<span>NPK</span> Pro</div>"
"  <div id=\"status\" class=\"badge live\">CONNECTING...</div>"
"</header>"
"<main class=\"grid\">"
"  <div class=\"card\" style=\"--card-accent:#38bdf8;\"><div class=\"card-header\">Temperature</div><div class=\"card-value\"><span id=\"temp\">--</span><span class=\"card-unit\">°C</span></div></div>"
"  <div class=\"card\" style=\"--card-accent:#0284c7;\"><div class=\"card-header\">Soil Moisture</div><div class=\"card-value\"><span id=\"moist\">--</span><span class=\"card-unit\">% RH</span></div></div>"
"  <div class=\"card\" style=\"--card-accent:#eab308;\"><div class=\"card-header\">Conductivity (EC)</div><div class=\"card-value\"><span id=\"ec\">--</span><span class=\"card-unit\">µS/cm</span></div></div>"
"  <div class=\"card\" style=\"--card-accent:#8b5cf6;\"><div class=\"card-header\">Salinity</div><div class=\"card-value\"><span id=\"sal\">--</span><span class=\"card-unit\">mg/L</span></div></div>"
"  <div class=\"card\" style=\"--card-accent:#10b981;\"><div class=\"card-header\">Nitrogen (N)</div><div class=\"card-value\"><span id=\"n\">--</span><span class=\"card-unit\">mg/kg</span></div></div>"
"  <div class=\"card\" style=\"--card-accent:#f97316;\"><div class=\"card-header\">Phosphorus (P)</div><div class=\"card-value\"><span id=\"p\">--</span><span class=\"card-unit\">mg/kg</span></div></div>"
"  <div class=\"card\" style=\"--card-accent:#ec4899;\"><div class=\"card-header\">Potassium (K)</div><div class=\"card-value\"><span id=\"k\">--</span><span class=\"card-unit\">mg/kg</span></div></div>"
"  <div class=\"card\" style=\"--card-accent:#14b8a6;\"><div class=\"card-header\">Soil pH</div><div class=\"card-value\"><span id=\"ph\">--</span><span class=\"card-unit\">pH</span></div></div>"
"</main>"
"<footer>ESP32-C3 SuperMini • HW-519 Auto-Direction RS485 • ESP-IDF v6.0.2</footer>"
"<script>"
"async function fetchData(){"
"  try{"
"    const res = await fetch('/api/data');"
"    if(!res.ok) throw new Error();"
"    const d = await res.json();"
"    document.getElementById('temp').innerText = d.temperature.toFixed(1);"
"    document.getElementById('moist').innerText = d.moisture.toFixed(1);"
"    document.getElementById('ec').innerText = d.ec;"
"    document.getElementById('sal').innerText = d.salinity;"
"    document.getElementById('n').innerText = d.n;"
"    document.getElementById('p').innerText = d.p;"
"    document.getElementById('k').innerText = d.k;"
"    document.getElementById('ph').innerText = d.ph.toFixed(1);"
"    const badge = document.getElementById('status');"
"    badge.innerText = 'ONLINE';"
"    badge.className = 'badge live';"
"  }catch(err){"
"    const badge = document.getElementById('status');"
"    badge.innerText = 'SENSOR OFFLINE';"
"    badge.className = 'badge err';"
"  }"
"}"
"setInterval(fetchData, 3000);"
"fetchData();"
"</script></body></html>";

/* =========================================================================
 * Modbus CRC16 Standard Implementation
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
 * Standard UART Initialization (HW-519: Automatic Flow Control)
 * ========================================================================= */
static esp_err_t rs485_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate  = RS485_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(RS485_UART_PORT, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART_PORT, &uart_config));
    
    // Set standard TX/RX pins without RTS/CTS since HW-519 handles direction
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART_PORT, RS485_TXD_PIN, RS485_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(RS485_UART_PORT, UART_MODE_UART));

    ESP_LOGI(TAG, "UART1 initialized for HW-519 on TX=GPIO%d, RX=GPIO%d (9600 8-N-1)", RS485_TXD_PIN, RS485_RXD_PIN);
    return ESP_OK;
}

/* =========================================================================
 * Sensor Polling Task
 * ========================================================================= */
static void sensor_polling_task(void *pvParameters)
{
    uint8_t rx_buf[UART_RX_BUF_SIZE];

    while (1) {
        // Discard stale RX FIFO bytes
        ESP_ERROR_CHECK(uart_flush_input(RS485_UART_PORT));

        // Write Modbus query frame
        uart_write_bytes(RS485_UART_PORT, (const char *)s_modbus_query_frame, sizeof(s_modbus_query_frame));

        // Wait for incoming response
        int rx_len = uart_read_bytes(RS485_UART_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(MODBUS_READ_TIMEOUT_MS));

        if (rx_len >= 19) {
            uint16_t computed_crc = modbus_crc16(rx_buf, rx_len - 2);
            uint16_t received_crc = (uint16_t)rx_buf[rx_len - 2] | ((uint16_t)rx_buf[rx_len - 1] << 8);

            if (computed_crc == received_crc) {
                // Determine register offset: standard Modbus FC 0x03 frame starts data at byte index 3
                size_t offset = (rx_buf[1] == 0x03) ? 3 : 0;

                int16_t raw_temp = (int16_t)((rx_buf[offset + 0] << 8) | rx_buf[offset + 1]);
                float temp       = (float)raw_temp / 10.0f;
                float moist      = (float)((rx_buf[offset + 2] << 8) | rx_buf[offset + 3]) / 10.0f;
                uint16_t ec      = (uint16_t)((rx_buf[offset + 4] << 8) | rx_buf[offset + 5]);
                uint16_t sal     = (uint16_t)((rx_buf[offset + 6] << 8) | rx_buf[offset + 7]);
                uint16_t n       = (uint16_t)((rx_buf[offset + 8] << 8) | rx_buf[offset + 9]);
                uint16_t p       = (uint16_t)((rx_buf[offset + 10] << 8) | rx_buf[offset + 11]);
                uint16_t k       = (uint16_t)((rx_buf[offset + 12] << 8) | rx_buf[offset + 13]);
                float ph         = (float)((rx_buf[offset + 14] << 8) | rx_buf[offset + 15]) / 10.0f;

                if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    s_sensor_data.temperature = temp;
                    s_sensor_data.moisture    = moist;
                    s_sensor_data.ec          = ec;
                    s_sensor_data.salinity    = sal;
                    s_sensor_data.nitrogen    = n;
                    s_sensor_data.phosphorus  = p;
                    s_sensor_data.potassium   = k;
                    s_sensor_data.ph          = ph;
                    s_sensor_data.valid       = true;
                    xSemaphoreGive(s_data_mutex);
                }

                ESP_LOGI(TAG, "Sensor parsed -> T:%.1f°C, M:%.1f%%, EC:%u, Sal:%u, N:%u, P:%u, K:%u, pH:%.1f",
                         temp, moist, ec, sal, n, p, k, ph);
            } else {
                ESP_LOGW(TAG, "CRC Check Failed: Calc=0x%04X, Recv=0x%04X", computed_crc, received_crc);
            }
        } else {
            ESP_LOGW(TAG, "Timeout or incomplete frame received (%d bytes)", rx_len);
        }

        vTaskDelay(pdMS_TO_TICKS(POLLING_INTERVAL_MS));
    }
}

/* =========================================================================
 * HTTP Handlers
 * ========================================================================= */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, s_index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_data_get_handler(httpd_req_t *req)
{
    char response[256];
    soil_data_t local_copy;

    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        local_copy = s_sensor_data;
        xSemaphoreGive(s_data_mutex);
    } else {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    snprintf(response, sizeof(response),
             "{\"temperature\":%.1f,\"moisture\":%.1f,\"ec\":%u,\"salinity\":%u,\"n\":%u,\"p\":%u,\"k\":%u,\"ph\":%.1f}",
             local_copy.temperature,
             local_copy.moisture,
             local_copy.ec,
             local_copy.salinity,
             local_copy.nitrogen,
             local_copy.phosphorus,
             local_copy.potassium,
             local_copy.ph);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;

    ESP_LOGI(TAG, "Starting HTTP Server on port %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        const httpd_uri_t root_uri = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = root_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root_uri);

        const httpd_uri_t api_uri = {
            .uri      = "/api/data",
            .method   = HTTP_GET,
            .handler  = api_data_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &api_uri);
        return server;
    }

    ESP_LOGE(TAG, "Failed to start HTTP server!");
    return NULL;
}

/* =========================================================================
 * Wi-Fi SoftAP Initialization (ESP-IDF v6.0.2 Native API)
 * ========================================================================= */
static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif != NULL);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid           = WIFI_AP_SSID,
            .ssid_len       = strlen(WIFI_AP_SSID),
            .channel        = WIFI_AP_CHANNEL,
            .password       = WIFI_AP_PASSWORD,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode       = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required   = false,
            },
        },
    };

    if (strlen(WIFI_AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "SoftAP Started. SSID: %s | Pass: %s | Gateway: 192.168.4.1", 
             WIFI_AP_SSID, WIFI_AP_PASSWORD);
}

/* =========================================================================
 * Main Application Entry Point
 * ========================================================================= */
void app_main(void)
{
    // 1. Initialize NVS (Required for Wi-Fi storage)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Create FreeRTOS Mutex for safe shared data access
    s_data_mutex = xSemaphoreCreateMutex();
    assert(s_data_mutex != NULL);

    // 3. Initialize RS485 UART Driver
    ESP_ERROR_CHECK(rs485_uart_init());

    // 4. Initialize SoftAP Network Stack
    wifi_init_softap();

    // 5. Start Web Server
    start_webserver();

    // 6. Launch Modbus Polling Task on ESP32-C3
    BaseType_t task_created = xTaskCreate(
        sensor_polling_task,
        "modbus_task",
        4096,
        NULL,
        tskIDLE_PRIORITY + 3,
        NULL
    );
    assert(task_created == pdPASS);

    ESP_LOGI(TAG, "ESP32-C3 SuperMini Agronomic Station Initialized successfully.");
}