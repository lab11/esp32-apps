/* Simple HTTP Server accessible on the Wi-Fi network

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/api.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mdns.h"

#define TAG "WIFI SERVER"

/* Example Web Page */
#define HTML "HTTP/1.1 200 OK\r\n Content-Type:text/html\r\n\r\n" \
    "<html>" \
        "<head>" \
            "<title>ESP32 Wi-Fi Server Example</title>" \
            "<meta name='viewport' content='width=device-width,initial-scale=1.0'>" \
        "</head>" \
        "<body style='background:#e43; color:#fff; font-family:monospace; text-align:center'>" \
            "<h2>ESP32 HTTP Server</h2>" \
            "<p>NETWORK<br><b>%s</b></p>" \
            "<p>UPTIME<br><b>%ds</b></p>" \
        "</body>" \
    "</html>"

/* Leave as is to set Wi-Fi configuration using 'make menuconfig'
   Or set it below - ie #define WIFI_SSID "mywifissid" */
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD

/* Event group & bit to signal when Wi-Fi connected */
static EventGroupHandle_t wifi_group;
const int CONNECTED_BIT = BIT0;

void http_serve_task(void *pvParameters) {
    struct netconn *client, *nc = netconn_new(NETCONN_TCP);
    if (!nc) {
        ESP_LOGE(TAG,"Failed to allocate socket");
        vTaskDelete(NULL);
    }
    netconn_bind(nc, IP_ADDR_ANY, 80);
    netconn_listen(nc);
    char buf[1024];
    while (1) {
        if ( netconn_accept(nc, &client) == ERR_OK ) {
            struct netbuf *nb;
            if ( netconn_recv(client, &nb) == ERR_OK ) {
                void *data;
                u16_t len;
                netbuf_data(nb, &data, &len);
                /* If request is GET, send HTML response */
                if (!strncmp(data, "GET ", 4)) {
                    ESP_LOGI(TAG,"Received data:\n%.*s\n", len, (char*)data);
                    snprintf(buf, sizeof(buf), HTML, 
                        WIFI_SSID, xTaskGetTickCount()*portTICK_PERIOD_MS/1000);
                    netconn_write(client, buf, strlen(buf), NETCONN_COPY);
                }
            }
            netbuf_delete(nb);
        }
        ESP_LOGI(TAG,"Closing connection");
        netconn_close(client);
        netconn_delete(client);
    }
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_group, CONNECTED_BIT);
            ESP_LOGI(TAG, "Connected to Wi-Fi network");
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

void app_main() {
    /* Initialize NVS */
    ESP_ERROR_CHECK( nvs_flash_init() );

    /* Initialize Wi-Fi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = { .sta={.ssid=WIFI_SSID,.password=WIFI_PASS} };
    wifi_group = xEventGroupCreate();
    tcpip_adapter_init();
    ESP_LOGI(TAG, "Connecting to Wi-Fi network: %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    xEventGroupWaitBits(wifi_group, CONNECTED_BIT, false, true, portMAX_DELAY);

    /* Initialize mDNS */
    ESP_ERROR_CHECK( mdns_init() );
    mdns_hostname_set("esp32");
    mdns_instance_name_set("ESP32");
    mdns_service_add("ESP32-Server", "_http", "_tcp", 80, NULL, 0);

    /* Start server */
    xTaskCreate(&http_serve_task, "http_serve_task", 1024*8, NULL, 8, NULL);
}
