/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee light driver example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_check.h"
#include "esp_zigbee_core.h"
#include "zcl/esp_zigbee_zcl_power_config.h"

/*! Maximum length of ManufacturerName string field */
#define ESP_ZB_ZCL_CLUSTER_ID_BASIC_MANUFACTURER_NAME_MAX_LEN 32

/*! Maximum length of ModelIdentifier string field */
#define ESP_ZB_ZCL_CLUSTER_ID_BASIC_MODEL_IDENTIFIER_MAX_LEN 32

#define ESP_TEMP_SENSOR_MIN_VALUE       (-10)   /* Local sensor min measured value (degree Celsius) */
#define ESP_TEMP_SENSOR_MAX_VALUE       (+90)   /* Local sensor max measured value (degree Celsius) */

/** optional basic manufacturer information */
typedef struct zcl_basic_manufacturer_info_s {
    char *manufacturer_name;
    char *model_identifier;
} zcl_basic_manufacturer_info_t;

/**
 * @brief Adds manufacturer information to the ZCL basic cluster of endpoint
 * 
 * @param[in] ep_list The pointer to the endpoint list with @p endpoint_id
 * @param[in] endpoint_id The endpoint identifier indicating where the ZCL basic cluster resides
 * @param[in] info The pointer to the basic manufacturer information
 * @return
 *      - ESP_OK: On success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t esp_zcl_utility_add_ep_basic_manufacturer_info(esp_zb_ep_list_t *ep_list, uint8_t endpoint_id, zcl_basic_manufacturer_info_t *info);

esp_err_t esp_zcl_utility_add_ep_power_config(esp_zb_ep_list_t *ep_list, uint8_t endpoint_id, void *value_p);

esp_err_t esp_zcl_utility_add_ep_temp_config(esp_zb_ep_list_t *ep_list, uint8_t endpoint_id, void *value_p);

#ifdef __cplusplus
} // extern "C"
#endif