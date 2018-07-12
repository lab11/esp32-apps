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

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "apps/sntp/sntp.h"

#define TAG "TIME SYNC"
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_CHAR_UUID           0xFF01

/* Leave as is to set Wi-Fi configuration using 'make menuconfig'.
   Or set it below - ie #define WIFI_SSID "mywifissid" */
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD

#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

/* Event group & bit to signal when Wi-Fi connected */
static EventGroupHandle_t wifi_group;
const int CONNECTED_BIT = BIT0;

/* # of restarts since 1st boot - saved in RTC memory to keep value in sleep */
RTC_DATA_ATTR static int boot_count = 0;


static bool connect    = false;
static bool get_service   = false;
static const char remote_device_name[] = "ESP_TIME_SYNC";
static esp_gattc_char_elem_t  *char_elem_result   = NULL;

static esp_bt_uuid_t remote_time_service_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
};

static esp_bt_uuid_t remote_time_char_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = REMOTE_CHAR_UUID,},
};

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE,
    .scan_interval          = 0x50,
    .scan_window            = 0x30
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,
    },
};

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

static void get_local_time(void) {
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

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "scan start success");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
            case ESP_GAP_SEARCH_INQ_RES_EVT:
                adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
                // esp_log_buffer_char(TAG, adv_name, adv_name_len);
                if (adv_name != NULL) {
                    if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                        ESP_LOGI(TAG, "Found device %s\n", remote_device_name);
                        esp_log_buffer_hex(TAG, scan_result->scan_rst.bda, 6);
                        if (connect == false) {
                            connect = true;
                            ESP_LOGI(TAG, "connect to the remote device.");
                            esp_ble_gap_stop_scanning();
                            esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        }
                    }
                }
                break;
            default:
                break;
            }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "stop scan successfully");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(TAG, "stop adv successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    // ESP_LOGI(TAG, "EVT %d, gattc if %d", event, gattc_if);

    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGE(TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }
    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    switch (event) {
        case ESP_GATTC_REG_EVT: {
            // ESP_LOGI(TAG, "REG_EVT");
            esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
            if (scan_ret){
                ESP_LOGE(TAG, "set scan params error, error code = %x", scan_ret);
            }
            break;
        }
        case ESP_GATTC_CONNECT_EVT:{
            ESP_LOGI(TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", param->connect.conn_id, gattc_if);
            gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
            memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            // esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, param->connect.conn_id);
            // if (mtu_ret){
            //     ESP_LOGE(TAG, "config MTU error, error code = %x", mtu_ret);
            // }
            esp_ble_gattc_search_service(gattc_if, param->connect.conn_id, &remote_time_service_uuid);
            break;
        }
        case ESP_GATTC_OPEN_EVT:
            if (param->open.status != ESP_GATT_OK){
                ESP_LOGE(TAG, "open failed, status %d", param->open.status);
                break;
            }
            ESP_LOGI(TAG, "open success");
            break;
        case ESP_GATTC_CFG_MTU_EVT:
            if (param->cfg_mtu.status != ESP_GATT_OK){
                ESP_LOGE(TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
            }
            ESP_LOGI(TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
            break;
        case ESP_GATTC_SEARCH_RES_EVT: {
            char UUID[param->search_res.srvc_id.uuid.len];
            for (int i = 0; i < param->search_res.srvc_id.uuid.len; i++) {
                sprintf(UUID+2*i, "%02X ", param->search_res.srvc_id.uuid.uuid.uuid128[i]);
            }
            ESP_LOGI(TAG, "SERVICE FOUND: %s", UUID); // Print discovered service
            if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && param->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
                get_service = true;
                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = param->search_res.start_handle;
                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = param->search_res.end_handle;
            }
            break;
        }
        case ESP_GATTC_SEARCH_CMPL_EVT:
            ESP_LOGI(TAG,"Search complete");
            if (param->search_cmpl.status != ESP_GATT_OK){
                ESP_LOGE(TAG, "search service failed, error status = %x", param->search_cmpl.status);
                break;
            }
            if (get_service){
                uint16_t count = 0;
                esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                        param->search_cmpl.conn_id,
                                                                        ESP_GATT_DB_CHARACTERISTIC,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                        INVALID_HANDLE,
                                                                        &count);
                if (status != ESP_GATT_OK){
                    ESP_LOGE(TAG, "esp_ble_gattc_get_attr_count error");
                }
                if (count > 0) {
                    char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                    if (!char_elem_result){
                        ESP_LOGE(TAG, "gattc no mem");
                    }else {
                        status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                                param->search_cmpl.conn_id,
                                                                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                remote_time_char_uuid,
                                                                char_elem_result,
                                                                &count);
                        if (status != ESP_GATT_OK){
                            ESP_LOGE(TAG, "esp_ble_gattc_get_char_by_uuid error");
                        }
                        if (count > 0 && char_elem_result[0].properties){
                            gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                            time_t now = 0;
                            time(&now);
                            ESP_LOGI(TAG,"Writing %lx",now);
                            esp_ble_gattc_write_char( gattc_if,
                                                    gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                    gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                    sizeof(now),
                                                    &now,
                                                    ESP_GATT_WRITE_TYPE_RSP,
                                                    ESP_GATT_AUTH_REQ_NONE);
                        }
                    }
                    /* free char_elem_result */
                    free(char_elem_result);
                }else {
                    ESP_LOGE(TAG, "no char found");
                }
            }
            break;
        case ESP_GATTC_WRITE_CHAR_EVT:
            if (param->write.status != ESP_GATT_OK){
                ESP_LOGE(TAG, "write char failed, error status = %x", param->write.status);
            }else{
                ESP_LOGI(TAG, "write char success");
            }
            esp_ble_gattc_close(gattc_if,param->connect.conn_id);
            break;
        case ESP_GATTC_DISCONNECT_EVT:
            connect = false;
            get_service = false;
            // ESP_LOGI(TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", param->disconnect.reason);
            ESP_LOGI(TAG, "Disconnected\n");
            esp_ble_gap_start_scanning(0);
            break;
        default:
            break;
    }
}

void app_main() {
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

    /* Initialize BLE & initiate scan */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT) );
    ESP_ERROR_CHECK( esp_bt_controller_init(&bt_cfg) );
    ESP_ERROR_CHECK( esp_bt_controller_enable(ESP_BT_MODE_BLE) );
    ESP_ERROR_CHECK( esp_bluedroid_init() );
    ESP_ERROR_CHECK( esp_bluedroid_enable() );
    ESP_ERROR_CHECK( esp_ble_gap_register_callback(esp_gap_cb) );
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_gattc_cb) );
    ESP_ERROR_CHECK( esp_ble_gattc_app_register(PROFILE_A_APP_ID) );
    ESP_ERROR_CHECK( esp_ble_gatt_set_local_mtu(500) );
}

