/* UDP Send example 

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip/udp.h"

/* The examples use simple WiFi and target UDP configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/

#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

#define WEB_SERVER CONFIG_UDP_ADDR
#define WEB_PORT CONFIG_UDP_PORT


/* FreeRTOS event group to signal when we are connected & ready to send a packet */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

static const char *TAG = "example";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void udp_send_task(void *pvParameters)
{
        struct udp_pcb *pcb;
        struct pbuf *p;
        ip_addr_t addr;
        err_t err;
        char msg[] = "testing\n";
        
        pcb = udp_new();

        if (ipaddr_aton(WEB_SERVER, &addr)) {
            while (1){
                /* Wait for connection */
                xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

                /* Allocate packet and copy message */
                p = pbuf_alloc(PBUF_TRANSPORT,sizeof(msg),PBUF_RAM);
                memcpy (p->payload, msg, sizeof(msg));

                /* Send UDP packet */
                err = udp_sendto(pcb, p, &addr, WEB_PORT);
                if (err == ERR_OK) {
                    ESP_LOGI(TAG, "Sent UDP packet to %s:%d : %s",WEB_SERVER,WEB_PORT,msg);
                } else {
                    ESP_LOGE(TAG, "lwIP UDP Send Error (%d)", err);
                }

                /* De-allocate, delay, and repeat */
                pbuf_free(p);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            } 
        } else {
            /* Error if IP address is invalid */
            ESP_LOGE(TAG,"Bad Target Address: %s",WEB_SERVER);
            vTaskDelay(10000 / portTICK_PERIOD_MS); 
        }
}

void app_main()
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_wifi();
    xTaskCreate(&udp_send_task, "udp_send_task", 4096, NULL, 5, NULL);
}
