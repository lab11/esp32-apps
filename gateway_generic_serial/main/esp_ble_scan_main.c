/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


/****************************************************************************
*
* This file is used for BLE scan.
*
****************************************************************************/

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_defs.h"
#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"

static const char* TAG = "SCAN";

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_PASSIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x0A0,
    .scan_window            = 0x0A0
};

static uint8_t print_buf[6+4+1+1+62];

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param) {
    esp_err_t err;

    switch(event) {
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
            switch(scan_result->scan_rst.search_evt) {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                    // parse data
                    // Struct documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_gap_ble.html#_CPPv4N22esp_ble_gap_cb_param_t25ble_scan_result_evt_paramE
                    memset(print_buf, 0, 6+4+1+1+62);
                    memcpy(&(print_buf[0]), scan_result->scan_rst.bda, 6);
                    memcpy(&(print_buf[6]), &(scan_result->scan_rst.rssi), 4);
                    memcpy(&(print_buf[10]), &(scan_result->scan_rst.ble_evt_type), 1);
                    uint8_t adv_len = scan_result->scan_rst.adv_data_len + scan_result->scan_rst.scan_rsp_len;
                    print_buf[11] = adv_len;
                    memcpy(&(print_buf[12]), scan_result->scan_rst.ble_adv, adv_len);

                    // print data
                    for (int i=0; i<12+adv_len; i++) {
                        printf("%02x", print_buf[i]);
                    }
                    printf("\n");
                }
                default:
                    break;
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: {
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

void esp_appRegister(void) {
    esp_err_t status;

    ESP_LOGI(TAG,"Register callback");

    /* register the scan callback function to the gap module */
    if((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG,"gap register error: %s", esp_err_to_name(status));
        return;
    }
}

void esp_init(void) {
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_appRegister();
}

void app_main() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);

    esp_init();

    /* set scan parameters */
    esp_ble_gap_set_scan_params(&ble_scan_params);
    printf("Starting BLE scans\n");
}
