/* Parse PowerBlade

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "apps/sntp/sntp.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_http_client.h"
#include "esp_event_loop.h"
#include "esp_wifi.h"
#include "esp_log.h"

#define TAG "PARSE POWERBLADE"

/* Endpoint for data */
#define WEB_URL "http://api.webhookinbox.com/i/CjHOjT40/in/"

/* Structure of the POST request body to send parsed data*/
#define BODY "%s{" \
        "\"device\":\"PowerBlade\"," \
        "\"id\":\"%s\"," \
        "\"sequence_number\":\"%u\"," \
        "\"rms_voltage\":\"%.2f\"," \
        "\"power\":\"%.2f\"," \
        "\"apparent_power\":\"%.2f\"," \
        "\"energy\":\"%.2f\"," \
        "\"power_factor\":\"%.2f\"," \
        "\"_meta\":{" \
            "\"received_time\":\"%s\"," \
            "\"device_id\":\"%s\"," \
            "\"receiver\":\"esp32-gateway\"," \
            "\"gateway_id\":\"%s\"" \
        "}" \
    "}%s"

/* Leave as is to set Wi-Fi configuration using 'make menuconfig'
   Or set it below - ie #define WIFI_SSID "mywifissid" */
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD

/* Event group & bit to signal when Wi-Fi connected */
static EventGroupHandle_t wifi_group;
const int CONNECTED_BIT = BIT0;

/* Strings */
static char body[32768], recv_buf[64], iso[20], gateway[17], id[17];

/* Struct for parsed data */
typedef struct {
    uint8_t device[ESP_BD_ADDR_LEN];
    uint32_t seq_num;
    time_t time_received;
    double v_rms;
    double real_power;
    double app_power;
    double watt_hours;
    double pf;
} parsed_item;
static parsed_item items[100];
static size_t items_length = 0;

/* ID String helper function */
static void get_id_string(uint8_t* id, char* id_string) {
    for (int i = 0; i < 6; i++) {
        sprintf(id_string+3*i, "%02x%s", id[i], i<5 ? ":" : "");
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

static void parse_data(esp_bd_addr_t device, time_t now, uint8_t *data) {
    /* Example AD: 02 01 06 17 ff e0 02 11 02 00 54 8a 1d 4f ff 79 09 c8 0c 83 0c b7 01 11 98 ba 42 */
    uint32_t sequence_num   = data[9]<<24 | data[10]<<16 | data[11]<<8 | data[12];
    uint32_t pscale         = data[13]<<8 | data[14];
    uint32_t vscale         = data[15];
    uint32_t whscale        = data[16];
    uint32_t v_rms          = data[17];
    uint32_t real_power     = data[18]<<8 | data[19];
    uint32_t apparent_power = data[20]<<8 | data[21];
    uint32_t watt_hours     = data[22]<<24 | data[23]<<16 | data[24]<<8 | data[25];
    double volt_scale       = vscale / 200.0;
    double power_scale      = (pscale & 0x0FFF) / pow(10.0, (pscale & 0xF000)>>12);
    size_t n                = items_length++;
    items[n].seq_num        = sequence_num;
    items[n].time_received  = now;
    items[n].v_rms          = v_rms * volt_scale;
    items[n].real_power     = real_power * power_scale;
    items[n].app_power      = apparent_power * power_scale;
    items[n].watt_hours     = volt_scale > 0 ? (watt_hours << whscale) * (power_scale / 3600.0) : watt_hours;
    items[n].pf             = items[n].real_power / items[n].app_power;
    memcpy(items[n].device, device, ESP_BD_ADDR_LEN);
}

static void http_post_task() {
    while (1) {
        vTaskDelay( 1000 / portTICK_PERIOD_MS);
        if (!items_length) { continue; };
        esp_http_client_config_t config = { .url = WEB_URL };
        esp_http_client_handle_t client = esp_http_client_init(&config);

        sprintf(body, "{\"items\":[");
        for (int n=0; n<items_length; n++) {
            get_id_string(items[n].device, id);
            strftime(iso, sizeof iso, "%FT%TZ", gmtime(&(items[n].time_received)));
            sprintf(body, BODY, body, id, items[n].seq_num, items[n].v_rms, items[n].real_power, items[n].app_power, items[n].watt_hours, items[n].pf, iso, id, gateway, n==items_length-1?"":",");
        }
        sprintf(body,"%s]}",body);
        ESP_LOGI(TAG, "HTTP POST to %s: \n\n%s\n", WEB_URL, body);


        esp_http_client_set_url(client, WEB_URL);
        esp_http_client_set_method(client, HTTP_METHOD_POST);
        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_post_field(client, body, strlen(body));
        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            int r;
            items_length = 0;
            ESP_LOGI(TAG, "HTTP POST Status = %d, Response:", esp_http_client_get_status_code(client));
            do {
                r = esp_http_client_read(client, recv_buf, sizeof(recv_buf)-1);
                for (int i = 0; i < r; i++) {
                    putchar(recv_buf[i]);
                }
            } while(r > 0);
            putchar('\n');
        } else {
            ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(err));
        }
        esp_http_client_cleanup(client);
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
                struct ble_scan_result_evt_param sr = param->scan_rst;
                if (sr.bda[0]==0xC0 && sr.bda[1]==0x98 && sr.bda[2]==0xE5 && sr.bda[3]==0x70 && sr.adv_data_len>18) {
                    time_t now;
                    time(&now);
                    esp_log_buffer_hex("\nPARSE POWERBLADE: >>>>>>> FOUND DEVICE", sr.bda, 6);
                    esp_log_buffer_hex("PARSE POWERBLADE: DATA", sr.ble_adv, sr.adv_data_len);
                    if (sr.ble_adv[3]<0x17 || sr.ble_adv[7]!=0x11 || sr.ble_adv[6] != 0x02 || sr.ble_adv[5] != 0xE0 || sr.ble_adv[8]!=0x02) {
                        ESP_LOGE(TAG,"No parseable data in this packet");
                    } else if (now < 1500000000) {
                        ESP_LOGE(TAG,"Still setting gateway time. Ignoring packet...");
                    } else {
                        parse_data(sr.bda, now, sr.ble_adv);
                    }
                }
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
    esp_ble_scan_params_t scan_params = { BLE_SCAN_TYPE_ACTIVE, BLE_ADDR_TYPE_PUBLIC, BLE_SCAN_FILTER_ALLOW_ALL, 0x100, 0x100, BLE_SCAN_DUPLICATE_ENABLE };
    ESP_ERROR_CHECK( esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT) );
    ESP_ERROR_CHECK( esp_bt_controller_init(&bt_cfg) );
    ESP_ERROR_CHECK( esp_bt_controller_enable(ESP_BT_MODE_BLE) );
    ESP_ERROR_CHECK( esp_bluedroid_init() );
    ESP_ERROR_CHECK( esp_bluedroid_enable() );
    ESP_ERROR_CHECK( esp_ble_gap_register_callback(esp_gap_cb) );
    ESP_ERROR_CHECK( esp_ble_gap_set_scan_params(&scan_params) );

    /* Initialize meta (Gateway ID & Time) */
    uint8_t mac[6];
    ESP_ERROR_CHECK( esp_efuse_mac_get_default(mac) );
    get_id_string(mac, gateway);
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    /* Initiate looped HTTP POST task */
    xTaskCreate(&http_post_task, "http_post_task", 32*configMINIMAL_STACK_SIZE, NULL, 10, NULL);
}
