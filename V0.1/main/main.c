/**
 * @file main.c
 * @brief ESP32-C3 SuperMini Soil Sensor Station with Captive Portal Dashboard.
 * Framework: Strictly Native ESP-IDF v6.0.2 (C11)
 *
 * Hardware:
 *  - ESP32-C3 SuperMini (RISC-V single-core)
 *  - HW-519 RS485 Transceiver (Auto TX/RX switching, standard UART mode)
 *  - UART1: TX on GPIO 4, RX on GPIO 5 (9600 Baud, 8-N-1)
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

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#define TAG "AGRI_PORTAL"

/* =========================================================================
 * Hardware & Protocol Configuration
 * ========================================================================= */
#define RS485_UART_PORT         (UART_NUM_1)
#define RS485_TXD_PIN           (GPIO_NUM_4)
#define RS485_RXD_PIN           (GPIO_NUM_5)
#define RS485_BAUD_RATE         (9600)
#define UART_RX_BUF_SIZE        (256)

#define WIFI_AP_SSID            "Sensor_NPK_Pro"
#define WIFI_AP_PASSWORD        "password123"
#define WIFI_AP_CHANNEL         (1)
#define WIFI_AP_MAX_CONN        (4)
#define PORTAL_IP_ADDR          "192.168.4.1"

#define MODBUS_READ_TIMEOUT_MS  (500)
#define POLLING_INTERVAL_MS     (3500)

/* =========================================================================
 * Soil Sensor Data Structures
 * ========================================================================= */
typedef struct {
    float    temperature;   /**< Unit: °C */
    float    moisture;      /**< Unit: % RH */
    uint16_t ec;            /**< Unit: µS/cm */
    uint16_t salinity;      /**< Unit: mg/L */
    uint16_t nitrogen;      /**< Unit: mg/kg */
    uint16_t phosphorus;    /**< Unit: mg/kg */
    uint16_t potassium;     /**< Unit: mg/kg */
    float    ph;            /**< Unit: pH */
    bool     valid;         /**< Validation flag */
} soil_metrics_t;

static soil_metrics_t   s_sensor_data = {0};
static SemaphoreHandle_t s_data_mutex  = NULL;

// Modbus Query: Addr 0x00, Func 0x03, StartReg 0x0000, RegCount 0x0008, CRC: 0xDD45
static const uint8_t s_modbus_query_frame[] = {
    0x00, 0x03, 0x00, 0x00, 0x00, 0x08, 0x45, 0xDD
};

/* =========================================================================
 * Embedded Web Page (Responsive Mobile Dashboard)
 * ========================================================================= */
static const char s_index_html[] =
"<!DOCTYPE html><html lang=\"en\"><head><meta charset=\"UTF-8\">"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1.0,maximum-scale=1.0,user-scalable=no\">"
"<title>Soil NPK Pro</title>"
"<style>"
":root{--bg:#090d16;--card:#131c2e;--card-border:#1e293b;--text:#f8fafc;--sub:#94a3b8;--accent:#10b981;}"
"*{margin:0;padding:0;box-sizing:border-box;font-family:-apple-system,BlinkMacSystemFont,\"Segoe UI\",Roboto,sans-serif;}"
"body{background:var(--bg);color:var(--text);padding:1rem;max-width:900px;margin:0 auto;}"
"header{display:flex;justify-content:space-between;align-items:center;margin-bottom:1.5rem;border-bottom:1px solid var(--card-border);padding-bottom:0.75rem;}"
".title{font-size:1.3rem;font-weight:700;display:flex;align-items:center;gap:0.4rem;}"
".title span{color:var(--accent);}"
".badge{padding:0.25rem 0.65rem;border-radius:9999px;font-size:0.75rem;font-weight:700;letter-spacing:0.05em;}"
".badge.live{background:rgba(16,185,129,0.15);color:var(--accent);border:1px solid var(--accent);}"
".badge.err{background:rgba(239,68,68,0.15);color:#ef4444;border:1px solid #ef4444;}"
".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:1rem;}"
".card{background:var(--card);border:1px solid var(--card-border);border-radius:0.75rem;padding:1rem;position:relative;overflow:hidden;}"
".card::before{content:'';position:absolute;top:0;left:0;right:0;height:3px;background:var(--card-accent,var(--accent));}"
".card-header{font-size:0.75rem;color:var(--sub);margin-bottom:0.35rem;text-transform:uppercase;font-weight:600;}"
".card-value{font-size:1.8rem;font-weight:700;display:flex;align-items:baseline;gap:0.2rem;}"
".card-unit{font-size:0.8rem;font-weight:400;color:var(--sub);}"
"footer{margin-top:1.5rem;text-align:center;font-size:0.75rem;color:var(--sub);}"
"</style></head><body>"
"<header>"
"  <div class=\"title\">🌱 Soil<span>NPK</span> Pro</div>"
"  <div id=\"status\" class=\"badge live\">CONNECTING</div>"
"</header>"
"<main class=\"grid\">"
"  <div class=\"card\" style=\"--card-accent:#38bdf8;\"><div class=\"card-header\">Temperature</div><div class=\"card-value\"><span id=\"temp\">--</span><span class=\"card-unit\">°C</span></div></div>"
"  <div class=\"card\" style=\"--card-accent:#0284c7;\"><div class=\"card-header\">Moisture</div><div class=\"card-value\"><span id=\"moist\">--</span><span class=\"card-unit\">% RH</span></div></div>"
"  <div class=\"card\" style=\"--card-accent:#eab308;\"><div class=\"card-header\">Conductivity (EC)</div><div class=\"card-value\"><span id=\"ec\">--</span><span class=\"card-unit\">µS/cm</span></div></div>"
"  <div class=\"card\" style=\"--card-accent:#8b5cf6;\"><div class=\"card-header\">Salinity</div><div class=\"card-value\"><span id=\"sal\">--</span><span class=\"card-unit\">mg/L</span></div></div>"
"  <div class=\"card\" style=\"--card-accent:#10b981;\"><div class=\"card-header\">Nitrogen (N)</div><div class=\"card-value\"><span id=\"n\">--</span><span class=\"card-unit\">mg/kg</span></div></div>"
"  <div class=\"card\" style=\"--card-accent:#f97316;\"><div class=\"card-header\">Phosphorus (P)</div><div class=\"card-value\"><span id=\"p\">--</span><span class=\"card-unit\">mg/kg</span></div></div>"
"  <div class=\"card\" style=\"--card-accent:#ec4899;\"><div class=\"card-header\">Potassium (K)</div><div class=\"card-value\"><span id=\"k\">--</span><span class=\"card-unit\">mg/kg</span></div></div>"
"  <div class=\"card\" style=\"--card-accent:#14b8a6;\"><div class=\"card-header\">Soil pH</div><div class=\"card-value\"><span id=\"ph\">--</span><span class=\"card-unit\">pH</span></div></div>"
"</main>"
"<footer>ESP32-C3 SuperMini • Captive Portal • RS485 Modbus</footer>"
"<script>"
"async function poll(){"
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
"    const s = document.getElementById('status');"
"    s.innerText = 'ONLINE'; s.className = 'badge live';"
"  }catch(e){"
"    const s = document.getElementById('status');"
"    s.innerText = 'OFFLINE'; s.className = 'badge err';"
"  }"
"}"
"setInterval(poll, 3000);"
"poll();"
"</script></body></html>";

/* =========================================================================
 * Modbus CRC16 Calculation
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
 * RS-485 UART Driver (Standard Mode for HW-519 Auto-Direction)
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
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART_PORT, RS485_TXD_PIN, RS485_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(RS485_UART_PORT, UART_MODE_UART));

    ESP_LOGI(TAG, "RS485 standard UART setup: TX=GPIO%d, RX=GPIO%d", RS485_TXD_PIN, RS485_RXD_PIN);
    return ESP_OK;
}

/* =========================================================================
 * Sensor Polling Task
 * ========================================================================= */
static void sensor_polling_task(void *pvParameters)
{
    uint8_t rx_buf[UART_RX_BUF_SIZE];

    while (1) {
        ESP_ERROR_CHECK(uart_flush_input(RS485_UART_PORT));
        uart_write_bytes(RS485_UART_PORT, (const char *)s_modbus_query_frame, sizeof(s_modbus_query_frame));

        int rx_len = uart_read_bytes(RS485_UART_PORT, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(MODBUS_READ_TIMEOUT_MS));

        if (rx_len >= 19) {
            uint16_t computed_crc = modbus_crc16(rx_buf, rx_len - 2);
            uint16_t received_crc = (uint16_t)rx_buf[rx_len - 2] | ((uint16_t)rx_buf[rx_len - 1] << 8);

            if (computed_crc == received_crc) {
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

                ESP_LOGI(TAG, "Readings: Temp=%.1f°C, Moist=%.1f%%, EC=%u, Sal=%u, N=%u, P=%u, K=%u, pH=%.1f",
                         temp, moist, ec, sal, n, p, k, ph);
            } else {
                ESP_LOGW(TAG, "CRC Error: Calc=0x%04X, Recv=0x%04X", computed_crc, received_crc);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(POLLING_INTERVAL_MS));
    }
}

/* =========================================================================
 * DNS Server (Captive Portal UDP Port 53)
 * ========================================================================= */
#define DNS_PORT        (53)
#define DNS_MAX_LEN     (512)

typedef struct __attribute__((packed)) {
    uint16_t id;
    uint16_t flags;
    uint16_t qdcount;
    uint16_t ancount;
    uint16_t nscount;
    uint16_t arcount;
} dns_header_t;

typedef struct __attribute__((packed)) {
    uint16_t ptr_name;
    uint16_t type;
    uint16_t class_type;
    uint32_t ttl;
    uint16_t data_len;
    uint32_t ip_addr;
} dns_answer_t;

static void dns_server_task(void *pvParameters)
{
    uint8_t rx_buffer[DNS_MAX_LEN];
    uint8_t tx_buffer[DNS_MAX_LEN];
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    socklen_t client_addr_len = sizeof(client_addr);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create DNS socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    server_addr.sin_family      = AF_INET;
    server_addr.sin_port        = htons(DNS_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "DNS socket bind failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Captive DNS responder active on UDP port 53");

    while (1) {
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0,
                           (struct sockaddr *)&client_addr, &client_addr_len);
        if (len < (int)sizeof(dns_header_t)) {
            continue;
        }

        dns_header_t *req_hdr = (dns_header_t *)rx_buffer;
        
        // Filter out non-queries
        if ((ntohs(req_hdr->flags) & 0x8000) != 0) {
            continue;
        }

        // Copy entire question block
        memcpy(tx_buffer, rx_buffer, len);
        dns_header_t *resp_hdr = (dns_header_t *)tx_buffer;

        // Construct standard DNS authoritative answer
        resp_hdr->flags   = htons(0x8180); // Response + Recursion desired + No error
        resp_hdr->qdcount = req_hdr->qdcount;
        resp_hdr->ancount = htons(1);
        resp_hdr->nscount = 0;
        resp_hdr->arcount = 0;

        // Append A record answer pointing to 192.168.4.1
        dns_answer_t answer;
        answer.ptr_name   = htons(0xC00C); // Compression pointer to question name at offset 12
        answer.type       = htons(1);      // Type A
        answer.class_type = htons(1);      // Class IN
        answer.ttl        = htonl(60);     // 60 seconds
        answer.data_len   = htons(4);
        answer.ip_addr    = inet_addr(PORTAL_IP_ADDR);

        if (len + sizeof(dns_answer_t) <= DNS_MAX_LEN) {
            memcpy(tx_buffer + len, &answer, sizeof(dns_answer_t));
            sendto(sock, tx_buffer, len + sizeof(dns_answer_t), 0,
                   (struct sockaddr *)&client_addr, client_addr_len);
        }
    }

    close(sock);
    vTaskDelete(NULL);
}

/* =========================================================================
 * HTTP Handlers & Captive Redirection
 * ========================================================================= */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, s_index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_data_get_handler(httpd_req_t *req)
{
    char response[256];
    soil_metrics_t local_copy;

    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        local_copy = s_sensor_data;
        xSemaphoreGive(s_data_mutex);
    } else {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    snprintf(response, sizeof(response),
             "{\"temperature\":%.1f,\"moisture\":%.1f,\"ec\":%u,\"salinity\":%u,\"n\":%u,\"p\":%u,\"k\":%u,\"ph\":%.1f}",
             local_copy.temperature, local_copy.moisture, local_copy.ec,
             local_copy.salinity, local_copy.nitrogen, local_copy.phosphorus,
             local_copy.potassium, local_copy.ph);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
}

// Redirect captive probe requests (Android, iOS, Windows) directly to root
static esp_err_t captive_redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://" PORTAL_IP_ADDR "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

// Catch-all 404 handler to enforce redirection when clients query arbitrary URLs
static esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://" PORTAL_IP_ADDR "/");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;

    if (httpd_start(&server, &config) == ESP_OK) {
        // Core Web Endpoints
        const httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_register_uri_handler(server, &root_uri);

        const httpd_uri_t api_uri = { .uri = "/api/data", .method = HTTP_GET, .handler = api_data_get_handler };
        httpd_register_uri_handler(server, &api_uri);

        // Android Captive Probes
        const httpd_uri_t g204 = { .uri = "/generate_204", .method = HTTP_GET, .handler = captive_redirect_handler };
        httpd_register_uri_handler(server, &g204);
        const httpd_uri_t gen204 = { .uri = "/gen_204", .method = HTTP_GET, .handler = captive_redirect_handler };
        httpd_register_uri_handler(server, &gen204);

        // Apple / iOS / macOS Captive Probe
        const httpd_uri_t apple = { .uri = "/hotspot-detect.html", .method = HTTP_GET, .handler = captive_redirect_handler };
        httpd_register_uri_handler(server, &apple);

        // Windows Captive Probes
        const httpd_uri_t ncsi = { .uri = "/ncsi.txt", .method = HTTP_GET, .handler = captive_redirect_handler };
        httpd_register_uri_handler(server, &ncsi);
        const httpd_uri_t conn = { .uri = "/connecttest.txt", .method = HTTP_GET, .handler = captive_redirect_handler };
        httpd_register_uri_handler(server, &conn);

        // Catch-all 404 redirection for any other domain
        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);

        ESP_LOGI(TAG, "HTTP Server active with captive redirection endpoints");
        return server;
    }

    ESP_LOGE(TAG, "Failed to launch HTTP Server!");
    return NULL;
}

/* =========================================================================
 * Wi-Fi SoftAP Configuration
 * ========================================================================= */
static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    assert(ap_netif != NULL);

    // Explicitly lock down Gateway and Subnet
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));
    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

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
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP configured: SSID='%s', IP=%s", WIFI_AP_SSID, PORTAL_IP_ADDR);
}

/* =========================================================================
 * Application Entrypoint
 * ========================================================================= */
void app_main(void)
{
    // 1. Initialize NVS Flash
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Synchronization Mutex
    s_data_mutex = xSemaphoreCreateMutex();
    assert(s_data_mutex != NULL);

    // 3. Initialize Standard UART for HW-519
    ESP_ERROR_CHECK(rs485_uart_init());

    // 4. Start SoftAP
    wifi_init_softap();

    // 5. Start Web Server with Captive Redirects
    start_webserver();

    // 6. Start DNS Server Task for Captive Portal Lookups
    BaseType_t dns_task = xTaskCreate(
        dns_server_task,
        "dns_server_task",
        3072,
        NULL,
        tskIDLE_PRIORITY + 4,
        NULL
    );
    assert(dns_task == pdPASS);

    // 7. Start Modbus Polling Task
    BaseType_t sensor_task = xTaskCreate(
        sensor_polling_task,
        "sensor_polling_task",
        4096,
        NULL,
        tskIDLE_PRIORITY + 3,
        NULL
    );
    assert(sensor_task == pdPASS);

    ESP_LOGI(TAG, "Station running. Connect to '%s' to launch Captive Portal.", WIFI_AP_SSID);
}