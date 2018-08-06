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
#include "driver/gpio.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gattc_api.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "lwip/netdb.h"
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
#define BLINK_GPIO       5

/* # of restarts since 1st boot - saved in RTC memory to keep value in sleep */
RTC_DATA_ATTR static int boot_count = 0;

/* Event group & bit to signal when Wi-Fi connected */
static EventGroupHandle_t wifi_group;
const int CONNECTED_BIT = BIT0;

static bool connected                           = false;
static bool get_service                         = false;
static const char remote_device_name[]          = "ESP_TIME_SYNC";
static char *city                               = "Berkeley";                 // default city
static char *tz                                 = "PST8PDT,M3.2.0/2,M11.1.0"; // default timezone

static esp_gattc_char_elem_t *char_elem_result  = NULL;
static esp_bt_uuid_t remote_time_service_uuid   = { .len = ESP_UUID_LEN_16, .uuid = {.uuid16 = REMOTE_SERVICE_UUID} };
static esp_bt_uuid_t remote_time_char_uuid      = { .len = ESP_UUID_LEN_16, .uuid = {.uuid16 = REMOTE_CHAR_UUID} };

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

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

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
            ESP_LOGI(TAG, "Disconnected from Wi-Fi network");
            xEventGroupClearBits(wifi_group, CONNECTED_BIT);
            esp_wifi_connect();
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void http_get_tz(void) {
    /* Send HTTP request to get location/timezone (defaults to Berkeley on fail) */
    struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM }, *res;
    struct timeval receiving_timeout = { .tv_sec = 5, .tv_usec = 0 };
    char recv_buf[1024], *loc;
    int s, r;
    ESP_LOGI(TAG, "Setting timezone..."); 
    if ( !getaddrinfo("timezoneapi.io", "80", &hints, &res) && res ) {
        if ( (s = socket(res->ai_family,res->ai_socktype,0)) >= 0 ) {
            if (!connect(s,res->ai_addr,res->ai_addrlen) &&
                 write(s, "GET /api/ip HTTP/1.0\r\nHost: timezoneapi.io\r\n\r\n", 52) >= 0 &&
                 setsockopt(s,SOL_SOCKET,SO_RCVTIMEO,&receiving_timeout,sizeof(receiving_timeout)) >= 0 ) {
                do {
                    bzero(recv_buf, sizeof(recv_buf));
                    r = read(s, recv_buf, sizeof(recv_buf)-1);
                    if ((loc = strstr(recv_buf, "\"city\":"))) {
                        size_t len = strchr((loc+8),'\"') - loc - 8;
                        memcpy(city = malloc(len+1), loc+8, len);
                        city[len] = 0;
                    } 
                    if ((loc = strstr(recv_buf, "\"tz_string\":"))) {
                        size_t len = strchr((loc+13),'\"') - loc - 13;
                        memcpy(tz=malloc(len+1), loc+13, len);
                        tz[len] = 0;
                    }
                } while(r > 0);
            }
            close(s);
        }
        freeaddrinfo(res);
    } 
}

static void get_local_time(void) {
    time_t now = 0;
    struct tm timeinfo = { 0 };
    char strftime_buf[64];
    /* Wait for time to update */
    for (int retry = 10; now < 1500000000 /* ~Jul 2017 */; --retry) {
        ESP_LOGI(TAG, "Waiting for system time to set... (%d/10)", 11-retry);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        if (!retry) esp_restart();
    }
    /* Update timzone */
    http_get_tz();
    setenv("TZ", tz, 1);
    tzset();
    /* Log local time */
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    printf("\n >>>>>  Current time in %s: %s  <<<<< \n\n", city, strftime_buf);
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            esp_ble_gap_start_scanning(0);
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
                break;
            }
            ESP_LOGI(TAG, "Scanning...");
            break;
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            switch (param->scan_rst.search_evt) {
                case ESP_GAP_SEARCH_INQ_RES_EVT:
                    adv_name = esp_ble_resolve_adv_data(param->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
                    if (adv_name != NULL) {
                        if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                            ESP_LOGI(TAG, "Found device %s", remote_device_name);
                            esp_log_buffer_hex(TAG, param->scan_rst.bda, 6);
                            if (connected == false) {
                                connected = true;
                                esp_ble_gap_stop_scanning();
                                ESP_LOGI(TAG, "Connecting...");
                                esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, param->scan_rst.bda, param->scan_rst.ble_addr_type, true);
                            }
                        }
                    }
                    break;
                default:
                    break;
            }
            break;
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
                ESP_LOGE(TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
                break;
            }
            ESP_LOGI(TAG, "Halted scanning!");
            break;
        default:
            break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGE(TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }
    /* If the gatts_if equal to a defineded profile, call the appropriate callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || gattc_if == gl_profile_tab[idx].gattc_if) {
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
            esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
            if (scan_ret){
                ESP_LOGE(TAG, "set scan params error, error code = %x", scan_ret);
            }
            break;
        }
        case ESP_GATTC_CONNECT_EVT:{
            gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
            memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            esp_ble_gattc_search_service(gattc_if, param->connect.conn_id, &remote_time_service_uuid);
            break;
        }
        case ESP_GATTC_OPEN_EVT:
            if (param->open.status != ESP_GATT_OK){
                ESP_LOGE(TAG, "open failed, status %d", param->open.status);
                break;
            }
            ESP_LOGI(TAG, "Connected! Searching services...");
            break;
        case ESP_GATTC_SEARCH_RES_EVT: {
            char UUID[param->search_res.srvc_id.uuid.len];
            for (int i = 0; i < param->search_res.srvc_id.uuid.len; i++) {
                sprintf(UUID+2*i, "%02X ", param->search_res.srvc_id.uuid.uuid.uuid128[i]);
            }
            ESP_LOGI(TAG, "SERVICE FOUND: %s", UUID); // Print discovered service
            if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && param->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = param->search_res.start_handle;
                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = param->search_res.end_handle;
                get_service = true;
            }
            break;
        }
        case ESP_GATTC_SEARCH_CMPL_EVT:
            if (param->search_cmpl.status != ESP_GATT_OK){
                ESP_LOGE(TAG, "search service failed, error status = %x", param->search_cmpl.status);
                break;
            }
            if (get_service){
                uint16_t count = 0;
                esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if, param->search_cmpl.conn_id, ESP_GATT_DB_CHARACTERISTIC,
                        gl_profile_tab[PROFILE_A_APP_ID].service_start_handle, gl_profile_tab[PROFILE_A_APP_ID].service_end_handle, INVALID_HANDLE, &count);
                if (status != ESP_GATT_OK){
                    ESP_LOGE(TAG, "esp_ble_gattc_get_attr_count error");
                }
                if (count > 0) {
                    char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                    if (!char_elem_result){
                        ESP_LOGE(TAG, "gattc no mem");
                    } else {
                        status = esp_ble_gattc_get_char_by_uuid( gattc_if, param->search_cmpl.conn_id, gl_profile_tab[PROFILE_A_APP_ID].service_start_handle, 
                            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle, remote_time_char_uuid, char_elem_result, &count);
                        if (status != ESP_GATT_OK){
                            ESP_LOGE(TAG, "esp_ble_gattc_get_char_by_uuid error");
                        }
                        if (count > 0 && char_elem_result[0].properties){
                            gl_profile_tab[PROFILE_A_APP_ID].char_handle = char_elem_result[0].char_handle;
                            esp_ble_gattc_read_char( gattc_if, gl_profile_tab[PROFILE_A_APP_ID].conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle, ESP_GATT_AUTH_REQ_NONE);
                        }
                    }
                    free(char_elem_result);
                } else {
                    ESP_LOGE(TAG, "no char found");
                }
            }
            break;
        case ESP_GATTC_READ_CHAR_EVT:
                if (param->read.status == ESP_GATT_OK) {
                    time_t *now = (time_t *) param->read.value;
                    ESP_LOGI(TAG,"Peripheral Time: %lds %ldus",(time_t)now[0],(suseconds_t)now[1]);
                }
                struct timeval tv = { 0, 0 };
                gettimeofday(&tv,NULL);
                char *tz = getenv("TZ");
                uint32_t now[2] = {tv.tv_sec, tv.tv_usec};
                ESP_LOGI(TAG,"Writing %us %uus",now[0],now[1]);
                esp_ble_gattc_write_char( gattc_if, gl_profile_tab[PROFILE_A_APP_ID].conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                    sizeof(now), (uint8_t *) &now, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
            break;
        case ESP_GATTC_WRITE_CHAR_EVT:
            if (param->write.status != ESP_GATT_OK){
                ESP_LOGE(TAG, "write char failed, error status = %x", param->write.status);
            }
            esp_ble_gattc_close(gattc_if,param->connect.conn_id);
            break;
        case ESP_GATTC_DISCONNECT_EVT:
            connected = false;
            get_service = false;
            ESP_LOGI(TAG, "Disconnected!\n");
            esp_ble_gap_start_scanning(0);
            break;
        default:
            break;
    }
}

void blink_task(void *pvParameter) {
    struct timeval tv = { 0, 0 };
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) {
        gettimeofday(&tv,NULL);
        gpio_set_level(BLINK_GPIO, tv.tv_sec%2);
        vTaskDelay( (1000 - (tv.tv_usec/1000)) / portTICK_PERIOD_MS);
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

    /* Initiate blink task */
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
}
