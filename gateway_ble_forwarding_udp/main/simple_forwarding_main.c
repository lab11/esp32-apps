/* Simple Forwarding (BLE Scan + HTTP POST using plain POSIX sockets)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
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

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = BIT0;

struct udp_pcb *pcb;
struct pbuf *p;
ip_addr_t ip;
err_t udp_err;

static const char* TAG = "SIMPLE_FORWARD";

/* declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};

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
    
    /* Initialize UDP */
    pcb = udp_new();
    if (ipaddr_aton(WEB_SERVER, &ip)) {
        /* Connect to endpoint IP */
        udp_err = udp_connect(pcb,&ip,WEB_PORT);
        if (udp_err == ERR_OK) {
            ESP_LOGI(TAG, "Connect UDP to %s:%d",WEB_SERVER,WEB_PORT);
        } else {
            ESP_LOGE(TAG, "lwIP UDP Connect Error (%d)", udp_err);
        }
    } else {
        /* Error if IP address is invalid */
        ESP_LOGE(TAG,"Bad Target Address: %s",WEB_SERVER);
    }
}

static void udp_send_task(uint8_t * addr, uint8_t * data, size_t len)
{
    char PACKET[ESP_BD_ADDR_LEN*2+len*2+18];
    char DATA[len*2];
    char ADDR[ESP_BD_ADDR_LEN*2];

    /* Wait for connection */
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

    /* Convert address & data to hex strings */
    for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
        sprintf(ADDR+2*i, "%02x", addr[i]);
    }
    for (int i = 0; i < len; i++) {
        sprintf(DATA+2*i, "%02x", data[i]);
    }
    
    /* Generate packet */
    sprintf(PACKET,"\nADDRESS: %s\nDATA: %s\n", ADDR, DATA);

    /* Allocate packet and copy message */
    p = pbuf_alloc(PBUF_TRANSPORT,sizeof(PACKET),PBUF_RAM);
    memcpy (p->payload, PACKET, sizeof(PACKET));

    /* Send UDP packet */
    udp_err = udp_send(pcb, p);
    if (udp_err == ERR_OK) {
        ESP_LOGI(TAG, "Sent UDP packet to %s:%d : %s",WEB_SERVER,WEB_PORT,PACKET);
    } else {
        ESP_LOGE(TAG, "lwIP UDP Send Error (%d)", udp_err);
    }

    /* De-allocate */
    pbuf_free(p);
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    esp_err_t err;

    switch(event)
    {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            uint32_t duration = 0;
            esp_ble_gap_start_scanning(duration);
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
            if((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG,"Scan start failed: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(TAG,"Start scanning...");
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t* scan_result = (esp_ble_gap_cb_param_t*)param;
            switch(scan_result->scan_rst.search_evt)
            {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                    ESP_LOGI(TAG, "--------Device Found----------");
                    esp_log_buffer_hex("SIMPLE_FORWARD: ADDR", scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                    ESP_LOGI(TAG, "RSSI:%d dbm", scan_result->scan_rst.rssi);
                    esp_log_buffer_hex("SIMPLE_FORWARD: DATA", scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len);
                    udp_send_task(scan_result->scan_rst.bda, scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len);
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:{
            if((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG,"Scan stop failed: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(TAG,"Stop scan successfully");
            }
            break;
        }
        default:
            break;
    }
}

void esp_appRegister(void)
{
    esp_err_t status;

    ESP_LOGI(TAG,"Register callback");

    /*<! register the scan callback function to the gap module */
    if((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG,"gap register error: %s", esp_err_to_name(status));
        return;
    }
}

void esp_init(void)
{
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_appRegister();
}

void app_main()
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_init();
    initialise_wifi();
    esp_ble_gap_set_scan_params(&ble_scan_params);
}
