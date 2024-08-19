/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee Sleepy end device Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "esp_check.h"
#include "hal/gpio_types.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zb_sleepy_end_device.h"
#include "esp_task_wdt.h"
#ifdef CONFIG_PM_ENABLE
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "esp_sleep.h"
#endif
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "string.h"
#include "iot_button.h"

/**
 * @note Make sure set idf.py menuconfig in zigbee component as zigbee end device!
*/
#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

static const char *TAG = "ESP_ZB_SLEEP";

uint8_t batPercentage = 55;
int16_t temperature = 125;

button_config_t gpio_btn_cfg = {
    .type = BUTTON_TYPE_GPIO,
    .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
    .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
    .gpio_button_config = {
        .gpio_num = GPIO_INPUT_IO_TOGGLE_SWITCH,
        .active_level = 0,
    },
};

static void zdevice_leave_cb() {
    ESP_LOGI(TAG, "Device left network");
}


static void ieee_cb(esp_zb_zdp_status_t zdo_status, esp_zb_ieee_addr_t ieee_addr, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Response IEEE address: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x", ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4],
                 ieee_addr[3], ieee_addr[2], ieee_addr[1], ieee_addr[0]);
    }
}

static void button_event_cb(void *arg, void *data)
{
    ESP_LOGI(TAG, "Button event %d", (button_event_t)data);

    if (BUTTON_SINGLE_CLICK == iot_button_get_event(arg)) {
        ESP_LOGI(TAG, "Connectivity state: %s", state_name[connectionState]);
        if (connectionState == connected) {
            ESP_EARLY_LOGI(TAG, "Update battery voltage");
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_set_attribute_val(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID, 
                &batPercentage, false);
            esp_zb_lock_release();
            ESP_EARLY_LOGI(TAG, "Report battery voltage");
            esp_zb_zcl_report_attr_cmd_t cmd_req;
            cmd_req.zcl_basic_cmd.dst_endpoint = HA_ESP_LIGHT_ENDPOINT;
            cmd_req.zcl_basic_cmd.src_endpoint = HA_ESP_LIGHT_ENDPOINT;
            cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
            cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            cmd_req.clusterID = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
            cmd_req.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
            cmd_req.attributeID = ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID;
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_report_attr_cmd_req(&cmd_req);
            esp_zb_lock_release();

            ESP_EARLY_LOGI(TAG, "Update temperature reading");
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_set_attribute_val(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, 
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &temperature, false);
            esp_zb_lock_release(); 

            ESP_EARLY_LOGI(TAG, "Report temp sensor reading");
            esp_zb_zcl_report_attr_cmd_t cmd_req2;
            cmd_req2.zcl_basic_cmd.dst_endpoint = HA_ESP_LIGHT_ENDPOINT;
            cmd_req2.zcl_basic_cmd.src_endpoint = HA_ESP_LIGHT_ENDPOINT;
            cmd_req2.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
            cmd_req2.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            cmd_req2.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
            cmd_req2.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
            cmd_req2.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_report_attr_cmd_req(&cmd_req2);
            esp_zb_lock_release();
            ESP_LOGI(TAG, "Reports sent");
       }
    }
    if (BUTTON_LONG_PRESS_UP == iot_button_get_event(arg)) {
        if (connectionState == disconnected) {
            ESP_LOGI(TAG, "Starting network steering");
            connectionState = connecting;
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            esp_zb_lock_release();
        } else {
            ESP_LOGI(TAG, "Request leaving network");
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zdo_mgmt_leave_req_param_t leave_req;
            esp_zb_ieee_addr_t ieee_addr;
            uint16_t short_addr = esp_zb_get_short_address();
            esp_zb_ieee_address_by_short(short_addr, ieee_addr);
            memcpy(leave_req.device_address, ieee_addr, sizeof(esp_zb_ieee_addr_t));
            leave_req.dst_nwk_addr = short_addr;
            esp_zb_zdo_device_leave_req(&leave_req, zdevice_leave_cb, NULL);
            esp_zb_lock_release();
        }
    }
}

void button_init(uint32_t button_num)
{
    button_config_t btn_cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
        .gpio_num = button_num,
        .active_level = BUTTON_ACTIVE_LEVEL,
#if GPIO_INPUT_IO_TOGGLE_SWITCH
            .enable_power_save = true,
#endif
        },
    };
    button_handle_t btn = iot_button_create(&btn_cfg);
    assert(btn);
    esp_err_t err = iot_button_register_cb(btn, BUTTON_PRESS_DOWN, button_event_cb, (void *)BUTTON_PRESS_DOWN);
    err |= iot_button_register_cb(btn, BUTTON_SINGLE_CLICK, button_event_cb, (void *)BUTTON_SINGLE_CLICK);
    err |= iot_button_register_cb(btn, BUTTON_DOUBLE_CLICK, button_event_cb, (void *)BUTTON_DOUBLE_CLICK);
    err |= iot_button_register_cb(btn, BUTTON_LONG_PRESS_UP, button_event_cb, (void *)BUTTON_LONG_PRESS_UP);
    ESP_ERROR_CHECK(err);
}

static esp_err_t deferred_driver_init(void)
{
    button_init(GPIO_INPUT_IO_TOGGLE_SWITCH);
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(
        1ULL << CONFIG_GPIO_INPUT_IO_WAKEUP, ESP_EXT1_WAKEUP_ANY_LOW));

#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
    rtc_gpio_pulldown_dis(CONFIG_GPIO_INPUT_IO_WAKEUP);
    rtc_gpio_pullup_en(CONFIG_GPIO_INPUT_IO_WAKEUP);
#else
    gpio_pulldown_dis(CONFIG_GPIO_INPUT_IO_WAKEUP);
    gpio_pullup_en(CONFIG_GPIO_INPUT_IO_WAKEUP);
#endif
    return ESP_OK;
}

/********************* Define functions **************************/

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee bdb commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        connectionState = connecting;
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Not joined any Zigbee network");
                //esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                connectionState = disconnected;
            } else {
                ESP_LOGI(TAG, "Device rebooted");
                connectionState = connected;
            }
        } else {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %d)", err_status);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            connectionState = connected;
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %d)", err_status);
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
      case ESP_ZB_ZDO_SIGNAL_LEAVE:
        if (connectionState == connected) {
            ESP_LOGI(TAG, "Left network");
            esp_zb_bdb_reset_via_local_action();
            connectionState = disconnected;
        }
        break;
      case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        if (connectionState == connected) {
            ESP_LOGI(TAG, "Zigbee can sleep");
            esp_zb_sleep_now();
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_ESP_LIGHT_ENDPOINT) {
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
            }
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack with Zigbee end-device config */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    
    /* Enable zigbee light sleep */
    esp_zb_sleep_enable(true);

    esp_zb_init(&zb_nwk_cfg);

    esp_zb_sleep_set_threshold(500);

    /* set the on-off light device config */
    esp_zb_on_off_light_cfg_t light_cfg = ESP_ZB_DEFAULT_ON_OFF_LIGHT_CONFIG();
    esp_zb_ep_list_t *esp_zb_on_off_light_ep = esp_zb_on_off_light_ep_create(HA_ESP_LIGHT_ENDPOINT, &light_cfg);
    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = ESP_MANUFACTURER_NAME,
        .model_identifier = ESP_MODEL_IDENTIFIER,
    };

    esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_on_off_light_ep, HA_ESP_LIGHT_ENDPOINT, &info);
    esp_zcl_utility_add_ep_power_config(esp_zb_on_off_light_ep, HA_ESP_LIGHT_ENDPOINT, &batPercentage);
    esp_zcl_utility_add_ep_temp_config(esp_zb_on_off_light_ep, HA_ESP_LIGHT_ENDPOINT, &temperature);
    esp_zb_device_register(esp_zb_on_off_light_ep);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    esp_zb_zdo_bind_req_param_t bind_req_temp;
    bind_req_temp.req_dst_addr = esp_zb_get_short_address();
    esp_zb_ieee_address_by_short(bind_req_temp.req_dst_addr, bind_req_temp.src_address);
    bind_req_temp.src_endp = HA_ESP_LIGHT_ENDPOINT;
    bind_req_temp.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
    bind_req_temp.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    esp_zb_ieee_address_by_short(0x0000, bind_req_temp.dst_address_u.addr_long);
    bind_req_temp.dst_endp = HA_ESP_LIGHT_ENDPOINT;
    esp_zb_zdo_device_bind_req(&bind_req_temp, NULL, NULL);

    esp_zb_zdo_bind_req_param_t bind_req_batv;
    bind_req_batv.req_dst_addr = esp_zb_get_short_address();
    esp_zb_ieee_address_by_short(bind_req_batv.req_dst_addr, bind_req_batv.src_address);
    bind_req_batv.src_endp = HA_ESP_LIGHT_ENDPOINT;
    bind_req_batv.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG;
    bind_req_batv.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    esp_zb_ieee_address_by_short(0x0000, bind_req_batv.dst_address_u.addr_long);
    bind_req_batv.dst_endp = HA_ESP_LIGHT_ENDPOINT;
    esp_zb_zdo_device_bind_req(&bind_req_batv, NULL, NULL);

    esp_zb_main_loop_iteration();
}

void app_main(void)
{
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Device starting up");
    connectionState = disconnected;

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    /* esp zigbee light sleep initialization*/
    ESP_ERROR_CHECK(esp_zb_power_save_init());
    /* load Zigbee platform config to initialization */
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
