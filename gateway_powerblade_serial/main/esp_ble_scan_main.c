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
                    // first check BLE address to see if this is a powerblade
                    uint8_t *bda = scan_result->scan_rst.bda;
                    if (bda[0]==0xC0 && bda[1]==0x98 && bda[2]==0xE5 && bda[3]==0x70) {
                        uint8_t adv_len = scan_result->scan_rst.adv_data_len;
                        uint8_t *adv = scan_result->scan_rst.ble_adv;

                        // confirm that this has a powerblade payload
                        //  * manufacturer-specific data section
                        //  * company identifier = 0x02E0
                        //  * Lab11 service ID = 0x11
                        //  * powerblade version = 2
                        if (adv_len >= 9 && adv[4] == 0xFF && adv[5] == 0xE0 && adv[6] == 0x02 && adv[7] == 0x11 && adv[8] == 2) {
                            uint32_t sequence_num   = adv[9]<<24 | adv[10]<<16 | adv[11]<<8 | adv[12];
                            uint32_t pscale         = adv[13]<<8 | adv[14];
                            uint32_t vscale         = adv[15];
                            uint32_t whscale        = adv[16];
                            uint32_t v_rms          = adv[17];
                            uint32_t real_power     = adv[18]<<8 | adv[19];
                            uint32_t apparent_power = adv[20]<<8 | adv[21];
                            uint32_t watt_hours     = adv[22]<<24 | adv[23]<<16 | adv[24]<<8 | adv[25];
                            uint32_t flags          = adv[26];
                            double volt_scale       = vscale / 200.0;
                            double power_scale      = (pscale & 0x0FFF) / pow(10.0, (pscale & 0xF000)>>12);

                            double v_rms_disp = v_rms*volt_scale;
                            double real_power_disp = real_power*power_scale;
                            double app_power_disp = apparent_power*power_scale;
                            double watt_hours_disp = 0.0;
                            if (volt_scale > 0) {
                                watt_hours_disp = watt_hours*pow(2.0, whscale)*(power_scale/3600.0);
                            } else {
                                watt_hours_disp = watt_hours;
                            }
                            double pf_disp = real_power_disp / app_power_disp;

                            printf("PowerBlade %X:%X:%X:%X:%X:%X\n", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
                            printf("      Sequence Number: %d\n", sequence_num);
                            printf("          RMS Voltage: %.2f V\n", v_rms_disp);
                            printf("           Real Power: %.2f W\n", real_power_disp);
                            printf("       Apparent Power: %.2f VA\n", app_power_disp);
                            printf("Cumulative Energy Use: %.2f Wh\n", watt_hours_disp);
                            printf("         Power Factor: %.2f\n", pf_disp);
                            printf("                Flags: 0x%02X\n", flags);
                            printf("\n");
                        }
                    }
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
    printf("Starting!\n");
}
