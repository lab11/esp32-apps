/* Test Forwarding (BLE Scan + HTTP POST using plain POSIX sockets)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "apps/sntp/sntp.h"
#include "lwip/netdb.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_log.h"

#define TAG "TEST_FORWARD"

/* Endpoint for data */
#define WEB_SERVER "webhookinbox.com"
#define WEB_PORT "80"
#define WEB_URL "http://api.webhookinbox.com/i/SYISElS7/in/"

/* Leave as is to set Wi-Fi configuration using 'make menuconfig'
   Or set it below - ie #define WIFI_SSID "mywifissid" */
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD

/* Event group & bit to signal when Wi-Fi connected */
static EventGroupHandle_t wifi_group;
const int CONNECTED_BIT = BIT0;

/* Hex string helper function */
static void get_hex_string(uint8_t* data, char* string, size_t len) {
    for (int i = 0; i < len; i++) {
        sprintf(string+2*i, "%02x", data[i]);
    }
}

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

static void http_post(uint8_t * addr, uint8_t * data, size_t len) {
    int s, r;
    char recv_buf[64], request[200], data_str[len*2], addr_str[ESP_BD_ADDR_LEN*2];
    struct timeval receiving_timeout = { .tv_sec = 5, .tv_usec = 0 };
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM }, *res;

//                 /* If device is ours (by address), print useful portions of advertisement */
//                     if (bda[0]==0xC0 && bda[1]==0x98 && bda[2]==0xE5 && bda[3]==0x00 && bda[4]==0x77) {
//                         uint8_t *adv = scan_result->scan_rst.ble_adv;
//                         uint16_t interval = ((adv[10] << 8) | adv[9]) * 5 / 8;
//                         uint32_t counter =  ((adv[18] << 24) | (adv[17] << 16) | (adv[16] << 8) | (adv[15]));
//                         printf("%02x%02x%02x%02x%02x%02x,%d,%d\n",
//                             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], // address
//                             counter, // sequence number
//                             interval); // advertisement interval
//                     }
//                 }

    get_hex_string(data, data_str, len);
    get_hex_string(addr, addr_str, ESP_BD_ADDR_LEN);
    sprintf(request, "POST %s HTTP/1.1\r\nContent-Type: application/json\r\nContent-Length: %d \r\n\r\n{\"address\":\"%s\",\"data\":\"%s\"}", WEB_URL, 24 + len*2 + ESP_BD_ADDR_LEN*2, addr_str, data_str);    
    ESP_LOGI(TAG, "%s", request);
    if ( !getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res) && res ) {
        if ( (s = socket(res->ai_family,res->ai_socktype,0)) >= 0 ) {
            if (!connect(s,res->ai_addr,res->ai_addrlen) && write(s, request, strlen(request)) >= 0 &&
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
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    switch(event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            esp_ble_gap_start_scanning(0);
            break;
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            } else {
                ESP_LOGI(TAG,"Scanning...");
            }
            break;
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                http_post(param->scan_rst.bda, param->scan_rst.ble_adv, param->scan_rst.adv_data_len);
            }
            break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
                ESP_LOGE(TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            } else {
                ESP_LOGI(TAG,"Halted scanning");
            }
            break;
        default:
            break;
    }
}

void app_main() {
    /* Initialize NVS */
    ESP_ERROR_CHECK( nvs_flash_init() );

    /* Initialize Wi-Fi */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t wifi_config = { .sta={ .ssid=WIFI_SSID, .password=WIFI_PASS } };
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
    esp_ble_scan_params_t scan_params = { BLE_SCAN_TYPE_ACTIVE, BLE_ADDR_TYPE_PUBLIC, BLE_SCAN_FILTER_ALLOW_ALL, 0x100, 0x100, 1 };
    ESP_ERROR_CHECK( esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT) );
    ESP_ERROR_CHECK( esp_bt_controller_init(&bt_cfg) );
    ESP_ERROR_CHECK( esp_bt_controller_enable(ESP_BT_MODE_BLE) );
    ESP_ERROR_CHECK( esp_bluedroid_init() );
    ESP_ERROR_CHECK( esp_bluedroid_enable() );
    ESP_ERROR_CHECK( esp_ble_gap_register_callback(esp_gap_cb) );
    ESP_ERROR_CHECK( esp_ble_gap_set_scan_params(&scan_params) );
}






// #include <stdio.h>
// #include <stdint.h>
// #include <string.h>

// #include "esp_bt.h"
// #include "nvs_flash.h"
// #include "esp_log.h"
// #include "esp_bt_defs.h"
// #include "esp_bt_main.h"
// #include "esp_gatt_defs.h"
// #include "esp_gattc_api.h"
// #include "esp_gap_ble_api.h"
// #include "freertos/FreeRTOS.h"

// static const char* TAG = "SCAN";

// static esp_ble_scan_params_t ble_scan_params = {
//     .scan_type              = BLE_SCAN_TYPE_ACTIVE,
//     .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
//     .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
//     .scan_interval          = 0x100,
//     .scan_window            = 0x100
// };

// static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
//     esp_err_t err;

//     switch(event) {
//         case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
//             uint32_t duration = 0;
//             esp_ble_gap_start_scanning(duration);
//             break;
//         }
//         case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
//             if((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
//                 ESP_LOGE(TAG,"Scan start failed: %s", esp_err_to_name(err));
//             }
//             else {
//                 ESP_LOGI(TAG,"Start scanning...");
//             }
//             break;
//         }
//         case ESP_GAP_BLE_SCAN_RESULT_EVT: {
//             esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;
//             switch(scan_result->scan_rst.search_evt)
//             {
//                 case ESP_GAP_SEARCH_INQ_RES_EVT: {
//                 /* If device is ours (by address), print useful portions of advertisement */
//                     uint8_t *bda = scan_result->scan_rst.bda;
//                     if (bda[0]==0xC0 && bda[1]==0x98 && bda[2]==0xE5 && bda[3]==0x00 && bda[4]==0x77) {
//                         uint8_t *adv = scan_result->scan_rst.ble_adv;
//                         uint16_t interval = ((adv[10] << 8) | adv[9]) * 5 / 8;
//                         uint32_t counter =  ((adv[18] << 24) | (adv[17] << 16) | (adv[16] << 8) | (adv[15]));
//                         printf("%02x%02x%02x%02x%02x%02x,%d,%d\n",
//                             bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], // address
//                             counter, // sequence number
//                             interval); // advertisement interval
//                     }
//                 }
//                 default:
//                     break;
//             }
//             break;
//         }
//         case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: {
//             if((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
//                 ESP_LOGE(TAG,"Scan stop failed: %s", esp_err_to_name(err));
//             }
//             else {
//                 ESP_LOGI(TAG,"Stop scan successfully");
//             }
//             break;
//         }
//         default:
//             break;
//     }
// }

// void esp_appRegister(void) {
//     esp_err_t status;

//     ESP_LOGI(TAG,"Register callback");

//     /* register the scan callback function to the gap module */
//     if((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
//         ESP_LOGE(TAG,"gap register error: %s", esp_err_to_name(status));
//         return;
//     }
// }

// void esp_init(void) {
//     esp_bluedroid_init();
//     esp_bluedroid_enable();
//     esp_appRegister();
// }

// void app_main() {
//     ESP_ERROR_CHECK(nvs_flash_init());
//     ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
//     esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//     esp_bt_controller_init(&bt_cfg);
//     esp_bt_controller_enable(ESP_BT_MODE_BLE);

//     esp_init();

//     /* set scan parameters */
//     esp_ble_gap_set_scan_params(&ble_scan_params);
//     printf("Starting!\n");
// }
