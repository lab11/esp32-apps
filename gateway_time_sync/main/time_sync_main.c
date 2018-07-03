/* Time Sync - Central (Gateway) Role

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
*
* Gets accurate time via SNTP & performs time sync for nearby BLE peripherals
*
****************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "apps/sntp/sntp.h"

#define TAG "TIME SYNC"

/* Leave as is to set Wi-Fi configuration using 'make menuconfig'.
   Or set it below - ie #define WIFI_SSID "mywifissid" */
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD

/* Event group & bit to signal when connected & ready to make a request */
static EventGroupHandle_t wifi_group;
const int CONNECTED_BIT = BIT0;

/* # of restarts since 1st boot - saved in RTC memory to keep value in sleep */
RTC_DATA_ATTR static int boot_count = 0;


static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
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

static void get_local_time(void)
{
    /* Wait for time to update, set timezone, & log time */
    time_t now = 0;
    struct tm timeinfo = { 0 };
    char strftime_buf[64];
    for (int retry = 10; now < 1500000000 /* ~Jul 2017 */ && retry; --retry) {
        ESP_LOGI(TAG, "Waiting for system time to set... (%d/10)", 11-retry);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
    }
    setenv("TZ", "PST8PDT,M3.2.0/2,M11.1.0", 1); // Pacific time
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time in Berkeley is: %s", strftime_buf);
}

void app_main()
{
    ESP_LOGI(TAG, "Boot count: %d", ++boot_count);
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

    /* Initialize SNTP & get current time */
    ESP_LOGI(TAG, "Obtaining time from pool.ntp.org...");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
    get_local_time();
}
