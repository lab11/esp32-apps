/* Influx Forward Test

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/apps/sntp.h"
#include "lwip/api.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_event_loop.h"
#include "esp_gap_ble_api.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#define TAG "INFUXTEST"
#define BATCH_SIZE 100     // max amount of ad data to store
#define POST_INTERVAL 5000 // how often to attempt sending to influx (ms)

/* Struct for parsed TestDevice data (using PowerBlade template) */
typedef struct {
    uint8_t id[6];  // device id
    uint32_t sn;    // sequence number
    double vr;      // rms voltage
    double rp;      // real power in watts
    double ap;      // apparent power in volt-amperes (~watts)
    double wh;      // energy in watt-hours
    double pf;      // power factor
    time_t ts;      // time in seconds
    long tn;    // remainder in nanoseconds
} testdevice_item;

/* Template Influx string for TestDevice data (using PowerBlade template) */
#define TD_INFLUX "test," \
                  "device_class=\"TestDevice\"," \
                  "device_id=\"%s\"," \
                  "gateway_id=\"%s\"," \
                  "receiver=\"esp32-gateway\" " \
                  "sequence_number=%u," \
                  "rms_voltage=%.2f," \
                  "power=%.2f," \
                  "apparent_power=%.2f," \
                  "energy=%.2f," \
                  "power_factor=%.2f " \
                  "%ld%09ld\n" // timestamp

/* Log template :      time | id |  seq # |  rms voltage |    power |    apparent power |    energy | p factor */
#define TD_LOG "%ld%09ld ns | %s | #%010u | %10.2f V_rms | %10.2f W | %10.2f W_apparent | %10.2f Wh | %1.2f pf\n"

#define is_testdevice(id,ad_len) (id[0]==0xC0 && id[1]==0x98 && id[2]==0xE5 && id[3]==0x00 && id[4]==0x77 && ad_len>18)

#define get_id_string(id, id_string) for(int i=0; i<6; i++) sprintf(id_string+3*i, "%02x%s", id[i], i<5 ? ":" : "")

static char gateway[17], id[17];
static uint32_t sequence_nums[0x1000];
struct timespec tv = {0, 0};

static testdevice_item items[BATCH_SIZE];
static int items_start = 0, items_end = 0;

/* Event group to signal when Wi-Fi is connected */
static EventGroupHandle_t wifi_group;

/* Values for Influx Endpoint */
static char host[128] = CONFIG_HOST, datb[32] = CONFIG_DATB, user[32] = CONFIG_USER, pswd[32] = CONFIG_PSWD;

static char recv_buf[64], url[256];
static char ssid[32] = "";
static wifi_config_t sta_config = { .sta={.ssid=CONFIG_WIFI_SSID,.password=CONFIG_WIFI_PASSWORD} };

nvs_handle storage;

/* Strings */
static char body[BATCH_SIZE*0x100], entry[0x200];

int8_t parse_testdevice_data(uint8_t *tdid, uint8_t *data, uint8_t length, testdevice_item *item) {
    /* TestDevice Device Check */
    if (!is_testdevice(tdid,length)) {
        return 0;
    }

    /* Retrieve system time */
    clock_gettime(CLOCK_REALTIME, &tv);

    /* Check for new sequence number */
    uint16_t idx = 0xFFF & ((tdid[4]<<8) + tdid[5]);
    uint32_t sequence_num = ((data[18] << 24) | (data[17] << 16) | (data[16] << 8) | (data[15]));
    if (sequence_num == sequence_nums[idx]) {
        return 0;
    }
    sequence_nums[idx] = sequence_num;
    
    item->sn                 = sequence_num;
    item->ts                 = tv.tv_sec;
    item->tn                 = tv.tv_nsec;
    item->vr                 = 0;
    item->rp                 = 0;
    item->ap                 = 0;
    item->wh                 = 0;
    item->pf                 = 0;
    memcpy(item->id, tdid, 6);

    /* Output to serial */
    get_id_string(item->id, id);
    printf(TD_LOG, item->ts, item->tn, id, item->sn, item->vr, item->rp, item->ap, item->wh, item->pf);

    /* Time check - if not set, do not store/send data */
    if (tv.tv_sec < 1500000000) {
        ESP_LOGW(TAG,"Time not set. Check Internet connection...");
        return 0;
    }
    
    return 1;
}

void testdevice_item_to_influx_string(testdevice_item *item, char *out) {
    if (!gateway[0]) {
        uint8_t mac[6];
        ESP_ERROR_CHECK( esp_efuse_mac_get_default(mac) );
        get_id_string(mac, gateway);
    }
    get_id_string(item->id, id);
    sprintf(out, TD_INFLUX, id, gateway, item->sn, item->vr, item->rp, item->ap, item->wh, item->pf, item->ts, item->tn);
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            nvs_open("storage", NVS_READWRITE, &storage);
            size_t ssid_len = 32, pswd_len = 64;
            nvs_get_str(storage, "ssid", (char*)sta_config.sta.ssid, &ssid_len);
            nvs_get_str(storage, "pswd", (char*)sta_config.sta.password, &pswd_len);
            nvs_close(storage);
            esp_wifi_set_config(WIFI_IF_STA, &sta_config);
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_group, BIT0);
            nvs_open("storage", NVS_READWRITE, &storage);
            nvs_set_str(storage, "ssid", (char*)sta_config.sta.ssid);
            nvs_set_str(storage, "pswd", (char*)sta_config.sta.password);
            nvs_close(storage);
            strcpy(ssid, (char*)sta_config.sta.ssid);
            ESP_LOGI(TAG, "Connected to network: %s", ssid);
            sntp_setoperatingmode(SNTP_OPMODE_POLL);
            sntp_setservername(0, "pool.ntp.org");
            sntp_init(); // Set time
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "Disconnected from network");
            esp_wifi_connect();
            xEventGroupClearBits(wifi_group, BIT0);
            break;
        default:
            break;
    }
    return ESP_OK;
}

void initialize_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    nvs_open("storage", NVS_READWRITE, &storage);
    size_t host_len = 128, cred_len = 32;
    nvs_get_str(storage, "iflx_host", host, &host_len);
    nvs_get_str(storage, "iflx_datb", datb, &cred_len);
    nvs_get_str(storage, "iflx_user", user, &cred_len);
    nvs_get_str(storage, "iflx_pswd", pswd, &cred_len);
    nvs_close(storage);
    ESP_ERROR_CHECK(ret);
}

void initialize_wifi() {
    esp_log_level_set("wifi", ESP_LOG_NONE);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    wifi_group = xEventGroupCreate();
    tcpip_adapter_init();
    ESP_LOGI(TAG, "Connecting to Wi-Fi network: %s...", sta_config.sta.ssid);
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    xEventGroupWaitBits(wifi_group, BIT0, false, true, portMAX_DELAY);
}

int http_post(char *url, char *body) {
    uint8_t status = 0;
    esp_http_client_config_t config = { .url = url };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_post_field(client, body, strlen(body));
    esp_err_t error = esp_http_client_perform(client);
    if (error) {
        ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(error));
    } else {
        status = esp_http_client_get_status_code(client);
        if (status != 204) {
            ESP_LOGW(TAG, "HTTP POST Status = %d, Response:", esp_http_client_get_status_code(client));
            while (esp_http_client_read(client, recv_buf, sizeof(recv_buf)-1) > 0 || (putchar('\n') && 0) ) {
                printf(recv_buf);
            }
        }
    }
    esp_http_client_cleanup(client);
    return status;
}

int http_post_to_influx(char *body) {
    if (user[0] && pswd[0]) {
        sprintf(url, "%s/write?db=%s&u=%s&p=%s", host, datb, user, pswd);
    } else {
        sprintf(url, "%s/write?db=%s", host, datb);
    }
    return http_post(url, body);
}

static void http_post_task() {
    while (1) {
        vTaskDelay( POST_INTERVAL / portTICK_PERIOD_MS);
        if (items_end - items_start) {
            /* If new data is available, set up a POST request */
            int n = items_start;
            for (body[0] = 0; n != items_end; n = (n + 1) % (sizeof(items) / sizeof(testdevice_item))) {
                testdevice_item_to_influx_string(&items[n], entry);
                sprintf(body, "%s %s", body, entry);
            }
            ESP_LOGI(TAG, "HTTP POST %d Items to Influx", (n-items_start+BATCH_SIZE)%BATCH_SIZE);
            if (http_post_to_influx(body) == 204) {
                items_start = n; // update cursor on influx success (status == 204)
            }
        }
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    switch(event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            esp_ble_gap_start_scanning(0);
            break;
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Scan start failed, error status = %x", param->scan_start_cmpl.status);
            } else {
                ESP_LOGI(TAG,"Scanning...");
            }
            break;
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                /* If ad is TestDevice data, populate current item with parsed values & point to next item */
                if (parse_testdevice_data((param->scan_rst).bda, (param->scan_rst).ble_adv, (param->scan_rst).adv_data_len, &items[items_end])) {
                    items_end = (items_end + 1) % (sizeof(items) / sizeof(testdevice_item));
                    if ((items_end-items_start+BATCH_SIZE)%BATCH_SIZE==BATCH_SIZE-1) {
                        esp_restart();
                    }
                }
            }
            break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
                ESP_LOGE(TAG, "Scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            } else {
                ESP_LOGI(TAG,"Halted scanning");
            }
            break;
        default:
            break;
    }
}

void initialize_ble() {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_ble_scan_params_t scan_params = { BLE_SCAN_TYPE_PASSIVE, BLE_ADDR_TYPE_PUBLIC, BLE_SCAN_FILTER_ALLOW_ALL, 0xA0, 0xA0, 0 };
    ESP_ERROR_CHECK( esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT) );
    ESP_ERROR_CHECK( esp_bt_controller_init(&bt_cfg) );
    ESP_ERROR_CHECK( esp_bt_controller_enable(ESP_BT_MODE_BLE) );
    ESP_ERROR_CHECK( esp_bluedroid_init() );
    ESP_ERROR_CHECK( esp_bluedroid_enable() );
    ESP_ERROR_CHECK( esp_ble_gap_register_callback(esp_gap_cb) );
    ESP_ERROR_CHECK( esp_ble_gap_set_scan_params(&scan_params) );
}

void app_main() {
    initialize_nvs();  // Initialize non-volatile storage
    initialize_ble();  // Set up BLE & register callbacks
    initialize_wifi(); // Set up Wi-Fi & connect to network, if specified    
    xTaskCreate(&http_post_task, "http_post_task", 32*configMINIMAL_STACK_SIZE, NULL, 10, NULL); // Initiate looped HTTP POST task
}
