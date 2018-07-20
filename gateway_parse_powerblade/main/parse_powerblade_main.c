/* Parse PowerBlade

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/netdb.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_log.h"

#define TAG "PARSE POWERBLADE"

/* Endpoint for data */
#define WEB_SERVER "requestbin.fullcontact.com"
#define WEB_PORT "80"
#define WEB_URL "http://" WEB_SERVER "/s7qxyqs7"

/* Leave as is to set Wi-Fi configuration using 'make menuconfig'
   Or set it below - ie #define WIFI_SSID "mywifissid" */
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD

/* Event group & bit to signal when Wi-Fi connected */
static EventGroupHandle_t wifi_group;
const int CONNECTED_BIT = BIT0;

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_group, CONNECTED_BIT);
            ESP_LOGI(TAG, "Connected to network: %s", WIFI_SSID);
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

static void http_post_data(uint8_t * addr, uint8_t * data) {
    /* Prepare request */
    char REQUEST[300];
    sprintf(REQUEST,"POST %s HTTP/1.0\r\nContent-Type:application/json\r\n\r\n{\"device\":\"PowerBlade\",\"address\": \"c098e57001a1\",\"sequence_number\": \"5546567\",\"rms_voltage\": \"120.39\",\"power\": \"1309.17\",\"apparent_power\": \"1339.42\",\"energy\": \"120603.00\",\"power_factor\": \"0.98\" }",WEB_URL);
    ESP_LOGI(TAG, "%s", REQUEST);

    /* Send HTTP request */
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM }, *res;
    struct timeval receiving_timeout = { .tv_sec = 5, .tv_usec = 0 };
    char recv_buf[64];
    int s, r;
    ESP_LOGI(TAG, "Sending data"); 
    if ( !getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res) && res ) {
        if ( (s = socket(res->ai_family,res->ai_socktype,0)) >= 0 ) {
            if (!connect(s,res->ai_addr,res->ai_addrlen) && write(s, REQUEST, strlen(REQUEST)) >= 0 &&
                 setsockopt(s,SOL_SOCKET,SO_RCVTIMEO,&receiving_timeout,sizeof(receiving_timeout)) >= 0 ) {
                do {
                    bzero(recv_buf, sizeof(recv_buf));
                    r = read(s, recv_buf, sizeof(recv_buf)-1);
                    for(int i = 0; i < r; i++) {
                        putchar(recv_buf[i]);
                    }
                } while(r > 0);
                putchar('\n');
            } else {
                ESP_LOGE(TAG, "Send failed");
            }
            close(s);
        } else {
            ESP_LOGE(TAG, "Failed to allocate socket");
        }
        freeaddrinfo(res);
    } else {
        ESP_LOGE(TAG, "DNS lookup failed");
    }
    vTaskDelay(50);
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    switch(event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            esp_ble_gap_start_scanning(0);
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            } else {
                ESP_LOGI(TAG,"Scanning...");
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            struct ble_scan_result_evt_param sr = param->scan_rst;
            switch(sr.search_evt) {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                    if (sr.bda[0]==0xC0 && sr.bda[1]==0x98 && sr.bda[2]==0xE5 && sr.bda[3]==0x70 && sr.adv_data_len>18) {
                        ESP_LOGI(TAG, "--------Device Found----------");
                        esp_log_buffer_hex("PARSE POWERBLADE: ADDR", sr.bda, ESP_BD_ADDR_LEN);
                        esp_log_buffer_hex("PARSE POWERBLADE: DATA", sr.ble_adv, sr.adv_data_len);
                          http_post_data(sr.bda, sr.ble_adv);
                        break;
                    }
                }
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:{
            if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
                ESP_LOGE(TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            } else {
                ESP_LOGI(TAG,"Halted scanning");
            }
            break;
        }
        default:
            break;
    }
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

     /* Initialize BLE */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT) );
    ESP_ERROR_CHECK( esp_bt_controller_init(&bt_cfg) );
    ESP_ERROR_CHECK( esp_bt_controller_enable(ESP_BT_MODE_BLE) );
    ESP_ERROR_CHECK( esp_bluedroid_init() );
    ESP_ERROR_CHECK( esp_bluedroid_enable() );
    ESP_ERROR_CHECK( esp_ble_gap_register_callback(esp_gap_cb) );
    ESP_ERROR_CHECK( esp_ble_gap_set_scan_params(&ble_scan_params) );
}
