/* Simple HTTP Server accessible on the Wi-Fi network

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/api.h"
#include "lwip/apps/sntp.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_http_client.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mdns.h"

#define TAG "WIFI SERVER"

/* Config Web Page */
#define HTML "HTTP/1.1 200 OK\r\n Content-Type:text/html\r\n\r\n" \
    "<html>" \
        "<head>" \
            "<title>ESP32 Config</title>" \
            "<meta name='viewport' content='width=device-width,initial-scale=1.0'>" \
            "<style>" \
                "* {color:#fff; font-family:monospace; text-align:center; padding:0; border:none;}" \
                "input {background:#0003; width:90vw; height:8vh; font-size:3.75vh; margin:1vh 0; outline:none; -webkit-appearance: none;}" \
                "input:read-only {background:#0000}" \
            "</style>" \
        "</head>" \
        "<body style='background:#e43; color:#fff; font-family:monospace; text-align:center'>" \
            "<h2>ESP32 Config</h2><br/>" \
            "<p>NETWORK<br/><input id='ssid' type='text' value='%s' readonly /></p>" \
            "<p>UPTIME<br/><input id='time' type='text' value='%ds' readonly /></p>" \
            "<p>INFLUX HOST<br/><input id='host' type='text' value='%s' /></p>" \
            "<p>INFLUX DATABASE<br/><input id='datb' type='text' value='%s' /></p>" \
            "<iframe style='display:none'></iframe>" \
            "<script>" \
                "document.querySelectorAll('input').forEach( i => {" \
                    "i.onblur = _ => document.querySelector('iframe').contentWindow.location.href = '/' + i.id + '=' + i.value;" \
                    "i.addEventListener('keyup', e => e.keyCode == 13 && i.blur() );" \
                "});" \
            "</script>" \
        "</body>" \
    "</html>"

/* Leave as is to set Wi-Fi configuration using 'make menuconfig'
   Or set it below - ie #define WIFI_SSID "mywifissid" */
#define WIFI_SSID CONFIG_WIFI_SSID
#define WIFI_PASS CONFIG_WIFI_PASSWORD

/* Structure of the data to be sent */
#define DAT "%s data," \
            "device_class=\"PowerBlade\"," \
            "device_id=\"%s\"," \
            "gateway_id=\"%s\"," \
            "receiver=\"esp32-gateway\" " \
            "sequence_number=%u," \
            "rms_voltage=%.2f," \
            "power=%.2f," \
            "apparent_power=%.2f," \
            "energy=%.2f," \
            "power_factor=%.2f " \
            "%ld%06ld000\n" // timestamp

#define URL "%s/write?db=%s"

/* Struct for parsed data */
typedef struct {
    uint8_t id[ESP_BD_ADDR_LEN]; // device id
    uint32_t sn;                 // sequence number
    double vr;                   // rms voltage
    double rp;                   // real power in watts
    double ap;                   // apparent power in volt-amperes (~watts)
    double wh;                   // energy in watt-hours
    double pf;                   // power factor
    time_t ts;                   // time in seconds
    suseconds_t tu;              // remainder in µseconds
} item;
static item items[100];
static int items_start = 0, items_end = 0;

/* Event group to signal when Wi-Fi is connected */
static EventGroupHandle_t wifi_group;

/* Strings */
static char body[32768], recv_buf[64], gateway[17], id[17], url[256];

static char host[128] = CONFIG_HOST;
static char db[64] = CONFIG_DB;


void http_serve_task(void *pvParameters) {
    struct netconn *client, *nc = netconn_new(NETCONN_TCP);
    if (!nc) {
        ESP_LOGE(TAG,"Failed to allocate socket");
        vTaskDelete(NULL);
    }
    netconn_bind(nc, IP_ADDR_ANY, 80);
    netconn_listen(nc);
    char buf[2048];
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

                    u16_t vlen = (void*) strstr(data," HTTP/") - data - 10;
                    if (!strncmp(data+4, "/host=", 6)) {
                        strncpy(host,data+10,vlen);
                        host[vlen] = 0;
                        ESP_LOGI(TAG,"host: %s, len: %d",host,vlen);
                    } else if (!strncmp(data+4, "/datb=", 6)) {
                        strncpy(db,data+10,vlen);
                        db[vlen] = 0;
                        ESP_LOGI(TAG,"db: %s, len: %d",db,vlen);
                    } 
                    snprintf(buf, sizeof(buf), HTML, WIFI_SSID, xTaskGetTickCount()*portTICK_PERIOD_MS/1000, host, db);
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
            xEventGroupSetBits(wifi_group, BIT0);
            ESP_LOGI(TAG, "Connected to network: %s", WIFI_SSID);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_group, BIT0);
            break;
        default:
            break;
    }
    return ESP_OK;
}

/* ID String helper function */
static void get_id_string(uint8_t* id, char* id_string) {
    for (int i = 0; i < 6; i++) {
        sprintf(id_string+3*i, "%02x%s", id[i], i<5 ? ":" : "");
    }
}

static void parse_data(esp_bd_addr_t device, struct timeval tv, uint8_t *data) {
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
    items[items_end].sn     = sequence_num;
    items[items_end].ts     = tv.tv_sec;
    items[items_end].tu     = tv.tv_usec;
    items[items_end].vr     = v_rms * volt_scale;
    items[items_end].rp     = real_power * power_scale;
    items[items_end].ap     = apparent_power * power_scale;
    items[items_end].wh     = volt_scale > 0 ? (watt_hours << whscale) * (power_scale / 3600.0) : watt_hours;
    items[items_end].pf     = items[items_end].rp / items[items_end].ap;
    memcpy(items[items_end].id, device, ESP_BD_ADDR_LEN);
    items_end = (items_end + 1) % (sizeof(items) / sizeof(item));
}

static void http_post_task() {
    while (1) {
        vTaskDelay( 5000 / portTICK_PERIOD_MS);
        if (items_end - items_start) {
            /* If new data is available, set up a POST request */
            int n = items_start;
            for (body[0] = 0; n != items_end; n = (n + 1) % (sizeof(items) / sizeof(item))) {
                get_id_string(items[n].id, id);
                sprintf(body, DAT, body, id, gateway, items[n].sn, items[n].vr, items[n].rp, items[n].ap, items[n].wh, items[n].pf, items[n].ts, items[n].tu);
            }

            /* Send request */
            sprintf(url, URL, host, db);
            ESP_LOGI(TAG, "HTTP POST %d Items to %s: \n\n%s\n", (n-items_start+100)%100, url, body);
            esp_http_client_config_t config = { .url = url };
            esp_http_client_handle_t client = esp_http_client_init(&config);
            esp_http_client_set_url(client, url);
            esp_http_client_set_method(client, HTTP_METHOD_POST);
            esp_http_client_set_post_field(client, body, strlen(body));
            esp_err_t error = esp_http_client_perform(client);
            if (error) {
                ESP_LOGE(TAG, "HTTP POST request failed: %s", esp_err_to_name(error));
            } else {
                ESP_LOGI(TAG, "HTTP POST Status = %d, Response:", esp_http_client_get_status_code(client));
                items_start = esp_http_client_get_status_code(client) == 204 ? n : items_start; // update cursor on influx success (status == 204)
                while (esp_http_client_read(client, recv_buf, sizeof(recv_buf)-1) > 0 || (putchar('\n') && 0) ) {
                    printf(recv_buf);
                }
            }
            esp_http_client_cleanup(client);
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
                ESP_LOGE(TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            } else {
                ESP_LOGI(TAG,"Scanning...");
            }
            break;
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                struct ble_scan_result_evt_param sr = param->scan_rst;
                if (sr.bda[0]==0xC0 && sr.bda[1]==0x98 && sr.bda[2]==0xE5 && sr.bda[3]==0x70 && sr.adv_data_len>18) {
                    struct timeval tv = {0, 0};
                    gettimeofday(&tv, NULL);
                    esp_log_buffer_hex("\nFOUND DEVICE", sr.bda, 6);
                    esp_log_buffer_hex("DATA", sr.ble_adv, sr.adv_data_len);
                    if (sr.ble_adv[3]<0x17 || sr.ble_adv[7]!=0x11 || sr.ble_adv[6] != 0x02 || sr.ble_adv[5] != 0xE0 || sr.ble_adv[8]!=0x02) {
                        ESP_LOGE(TAG,"No parseable data in this packet");
                    } else if (tv.tv_sec < 1500000000) {
                        ESP_LOGE(TAG,"Still setting gateway time. Ignoring packet...");
                    } else {
                        parse_data(sr.bda, tv, sr.ble_adv);
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
    xEventGroupWaitBits(wifi_group, BIT0, false, true, portMAX_DELAY);

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

    /* Initialize mDNS */
    ESP_ERROR_CHECK( mdns_init() );
    mdns_hostname_set("esp32");
    mdns_instance_name_set("ESP32");
    mdns_service_add("ESP32-Server", "_http", "_tcp", 80, NULL, 0);

    /* Initialize meta (Gateway ID & Time) */
    uint8_t mac[6];
    ESP_ERROR_CHECK( esp_efuse_mac_get_default(mac) );
    get_id_string(mac, gateway);
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    /* Start server */
    xTaskCreate(&http_serve_task, "http_serve_task", 1024*8, NULL, 8, NULL);

    /* Initiate looped HTTP POST task */
    xTaskCreate(&http_post_task, "http_post_task", 32*configMINIMAL_STACK_SIZE, NULL, 10, NULL);
}
