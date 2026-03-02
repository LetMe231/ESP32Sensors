/* main.c - BLE Mesh Vendor Server with Sensor Integration */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file main.c
 * @brief BLE Mesh vendor server with multi-sensor integration and periodic publishing.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"
#include "config.h"
#include "sensors/aht20.h"
#include "sensors/max30101.h"
#include "sensors/SGP30.h"

// ============================================================================
// Logging and Constants
// ============================================================================

/** @brief Log tag for this module. */
#define TAG "MESH_SERVER"

// BLE Mesh Vendor Model Configuration
#define CID_ESP                                 0x02E5
#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT       0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER       0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SEND         ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS       ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

/** @brief Sensor status opcode for publication. */
#define OP_SENSOR_STATUS                       ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

// Sensor Data Payload Structure (little-endian)
#define SENSOR_PAYLOAD_SIZE                    18
#define SENSOR_TEMP_OFFSET                     0   // 2 bytes, int16
#define SENSOR_HUMIDITY_OFFSET                 2   // 2 bytes, uint16
#define SENSOR_RED_OFFSET                      4   // 4 bytes, uint32
#define SENSOR_IR_OFFSET                       8   // 4 bytes, uint32
#define SENSOR_HR_OFFSET                       12  // 1 byte, uint8
#define SENSOR_SPO2_OFFSET                     13  // 1 byte, uint8
#define SENSOR_ECO2_OFFSET                     14  // 2 bytes, uint16
#define SENSOR_TVOC_OFFSET                     16  // 2 bytes, uint16

// Timer Intervals (microseconds)
#define SENSOR_PUBLISH_INTERVAL_US             (2 * 1000 * 1000)  // 2 seconds
#define SGP30_MEASUREMENT_INTERVAL_US          (1 * 1000 * 1000)  // 1 second

// Sentinel values for invalid sensor readings
#define SENSOR_INVALID_U8                      0xFF
#define SENSOR_INVALID_U16                     0xFFFF
#define SENSOR_INVALID_U32                     0xFFFFFFFF
#define SENSOR_INVALID_S16                     ((int16_t)0xFFFF)

/** @brief Model publication definition for sensor data. */
ESP_BLE_MESH_MODEL_PUB_DEFINE(vnd_pub, 3 + SENSOR_PAYLOAD_SIZE, ROLE_NODE);

// ============================================================================
// Global Variables
// ============================================================================

/** @brief BLE Mesh publication timer. */
static esp_timer_handle_t pub_timer;

/** @brief Vendor model instance for publishing sensor data. */
static esp_ble_mesh_model_t *s_vnd_model;

/** @brief Device UUID for BLE Mesh provisioning. */
static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10 };

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * @brief Pack sensor values into a fixed 18-byte payload.
 * 
 * Payload layout (little-endian):
 *   Offset  Size  Field
 *   ------  ----  ---------------------
 *   0       2     Temperature (°C × 100)
 *   2       2     Humidity (% × 100)
 *   4       4     RED raw value
 *   8       4     IR raw value
 *   12      1     Heart rate (bpm)
 *   13      1     SpO2 (%)
 *   14      2     eCO2 (ppm)
 *   16      2     TVOC (ppb)
 * 
 * @param[out] payload    Output buffer (must be SENSOR_PAYLOAD_SIZE bytes)
 * @param[in]  temp_x100  Temperature in 0.01°C units (int16)
 * @param[in]  humidity_x100 Humidity in 0.01% units (uint16)
 * @param[in]  raw_red    RED channel raw value
 * @param[in]  raw_ir     IR channel raw value
 * @param[in]  heart_rate Heart rate in bpm
 * @param[in]  spo2       SpO2 percentage
 * @param[in]  eco2       eCO2 in ppm
 * @param[in]  tvoc       TVOC in ppb
 */
static void pack_sensor_payload(uint8_t *payload,
                                 int16_t temp_x100,
                                 uint16_t humidity_x100,
                                 uint32_t raw_red,
                                 uint32_t raw_ir,
                                 uint8_t heart_rate,
                                 uint8_t spo2,
                                 uint16_t eco2,
                                 uint16_t tvoc)
{
    if (!payload) {
        return;
    }

    // Temperature (2 bytes, little-endian)
    payload[SENSOR_TEMP_OFFSET]     = (uint8_t)(temp_x100 & 0xFF);
    payload[SENSOR_TEMP_OFFSET + 1] = (uint8_t)((temp_x100 >> 8) & 0xFF);

    // Humidity (2 bytes, little-endian)
    payload[SENSOR_HUMIDITY_OFFSET]     = (uint8_t)(humidity_x100 & 0xFF);
    payload[SENSOR_HUMIDITY_OFFSET + 1] = (uint8_t)((humidity_x100 >> 8) & 0xFF);

    // RED (4 bytes, little-endian)
    payload[SENSOR_RED_OFFSET]     = (uint8_t)(raw_red & 0xFF);
    payload[SENSOR_RED_OFFSET + 1] = (uint8_t)((raw_red >> 8) & 0xFF);
    payload[SENSOR_RED_OFFSET + 2] = (uint8_t)((raw_red >> 16) & 0xFF);
    payload[SENSOR_RED_OFFSET + 3] = (uint8_t)((raw_red >> 24) & 0xFF);

    // IR (4 bytes, little-endian)
    payload[SENSOR_IR_OFFSET]     = (uint8_t)(raw_ir & 0xFF);
    payload[SENSOR_IR_OFFSET + 1] = (uint8_t)((raw_ir >> 8) & 0xFF);
    payload[SENSOR_IR_OFFSET + 2] = (uint8_t)((raw_ir >> 16) & 0xFF);
    payload[SENSOR_IR_OFFSET + 3] = (uint8_t)((raw_ir >> 24) & 0xFF);

    // Heart Rate (1 byte)
    payload[SENSOR_HR_OFFSET] = heart_rate;

    // SpO2 (1 byte)
    payload[SENSOR_SPO2_OFFSET] = spo2;

    // eCO2 (2 bytes, little-endian)
    payload[SENSOR_ECO2_OFFSET]     = (uint8_t)(eco2 & 0xFF);
    payload[SENSOR_ECO2_OFFSET + 1] = (uint8_t)((eco2 >> 8) & 0xFF);

    // TVOC (2 bytes, little-endian)
    payload[SENSOR_TVOC_OFFSET]     = (uint8_t)(tvoc & 0xFF);
    payload[SENSOR_TVOC_OFFSET + 1] = (uint8_t)((tvoc >> 8) & 0xFF);
}

#if ENABLE_SGP30
/** @brief Cached SGP30 values updated at 1 Hz. */
static esp_timer_handle_t sgp30_timer;
static uint16_t cached_eco2 = SENSOR_INVALID_U16;
static uint16_t cached_tvoc = SENSOR_INVALID_U16;
static uint32_t sgp30_read_count = 0;
static uint32_t sgp30_error_count = 0;

/**
 * @brief Timer callback to keep SGP30 measurements at 1 Hz.
 * 
 * SGP30 air quality sensor requires continuous 1Hz measurements to maintain
 * a valid air quality baseline. This timer ensures the baseline doesn't reset.
 */
static void sgp30_timer_cb(void *arg)
{
    (void)arg;  // Unused parameter
    
    esp_err_t err = sgp30_read_values(&cached_eco2, &cached_tvoc);
    if (err != ESP_OK) {
        cached_eco2 = SENSOR_INVALID_U16;
        cached_tvoc = SENSOR_INVALID_U16;
        sgp30_error_count++;
        if (sgp30_error_count % 10 == 0) {
            ESP_LOGW(TAG, "SGP30: %lu consecutive errors, %lu successful reads",
                     sgp30_error_count, sgp30_read_count);
        }
        return;
    }
    sgp30_read_count++;
}
#endif

// ============================================================================
// BLE Mesh Configuration
// ============================================================================

/** @brief Configuration Server model state. */
static esp_ble_mesh_cfg_srv_t config_server = {
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
};

/** @brief Root model list for the primary element. */
static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

/** @brief Vendor model operation table. */
static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SEND, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

/** @brief Vendor model instances for this node. */
static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
    vnd_op,  &vnd_pub, NULL),
};

/** @brief Element list for this composition. */
static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

/** @brief Node composition data. */
static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

/** @brief Provisioning capabilities and identifiers. */
static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
    /* PB-ADV and PB-GATT bearers are both enabled */
    /* Device will enter unprovisioned mode and wait for provisioning */
};

/**
 * @brief Sensor data publication callback.
 * 
 * Called periodically (every 2 seconds) to read sensor data and publish
 * to BLE Mesh network. Collects data from AHT20, MAX30101, and SGP30 sensors
 * and publishes as a single 18-byte payload.
 */
static void pub_timer_cb(void *arg)
{
    (void)arg;  // Unused parameter
    
    if (!s_vnd_model || !s_vnd_model->pub) {
        ESP_LOGW(TAG, "Vendor model or publication not ready");
        return;
    }

    esp_err_t err;
    int16_t temp_x100 = SENSOR_INVALID_S16;
    uint16_t humidity_x100 = SENSOR_INVALID_U16;
    uint32_t raw_red = SENSOR_INVALID_U32;
    uint32_t raw_ir = SENSOR_INVALID_U32;
    uint8_t heart_rate = SENSOR_INVALID_U8;
    uint8_t spo2 = SENSOR_INVALID_U8;
    uint16_t eco2 = SENSOR_INVALID_U16;
    uint16_t tvoc = SENSOR_INVALID_U16;

#if ENABLE_AHT20
    // Read temperature and humidity
    err = aht20_read_x100(&temp_x100, &humidity_x100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AHT20 read failed: %s", esp_err_to_name(err));
        temp_x100 = SENSOR_INVALID_S16;
        humidity_x100 = SENSOR_INVALID_U16;
    }
#endif

#if ENABLE_MAX30101
    // Get latest raw PPG values and computed vital signs
    max30101_get_raw_values(&raw_red, &raw_ir);
    heart_rate = max30101_get_heart_rate();
    spo2 = max30101_get_spo2();
#endif

#if ENABLE_SGP30
    // Use cached values from 1Hz timer
    // SGP30 requires continuous 1Hz measurements to maintain baseline
    eco2 = cached_eco2;
    tvoc = cached_tvoc;
    ESP_LOGD(TAG, "SGP30: eCO2=%d ppm, TVOC=%d ppb", eco2, tvoc);
#endif

    // Pack sensor data into payload
    uint8_t payload[SENSOR_PAYLOAD_SIZE] = {0};
    pack_sensor_payload(payload, temp_x100, humidity_x100, raw_red, raw_ir,
                       heart_rate, spo2, eco2, tvoc);

    // Publish sensor data to BLE Mesh network
    err = esp_ble_mesh_model_publish(
        s_vnd_model,
        OP_SENSOR_STATUS,
        SENSOR_PAYLOAD_SIZE,
        payload,
        ROLE_NODE
    );

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Publish failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGD(TAG, "Published: T=%d.%02d°C RH=%d.%02d%% HR=%ubpm SpO2=%d%% "
                      "eCO2=%d ppm TVOC=%d ppb",
                 temp_x100 / 100, (temp_x100 < 0) ? -(temp_x100 % 100) : (temp_x100 % 100),
                 humidity_x100 / 100, humidity_x100 % 100,
                 heart_rate, spo2, eco2, tvoc);
    }
}

/**
 * @brief Handle provisioning completion event.
 * 
 * Called when the device has been successfully provisioned to the BLE Mesh network.
 * Updates LED status to indicate successful provisioning.
 * 
 * @param[in] net_idx   Network index assigned to this node
 * @param[in] addr      Unicast address assigned to this node
 * @param[in] flags     Provisioning flags
 * @param[in] iv_index  Initialization Vector index for the network
 */
static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "Provisioning complete - net_idx=0x%04x, addr=0x%04x", net_idx, addr);
    ESP_LOGD(TAG, "Flags=0x%02x, IV index=0x%08" PRIx32, flags, iv_index);
    
    // Turn off green LED to indicate provisioning is complete
    board_led_operation(LED_GREEN, LED_OFF);
}

/**
 * @brief BLE Mesh provisioning callback for node events.
 */
static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        
        // After provisioning, device should now be advertising as GATT proxy
        // If using PB-ADV, you need a GATT proxy relay to reach this device
        // If using PB-GATT, the connection should persist for configuration
        if (param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV) {
            ESP_LOGI(TAG, "Provisioned via PB-ADV - device will advertise as GATT proxy");
        } else {
            ESP_LOGI(TAG, "Provisioned via PB-GATT - persistent GATT connection may be available");
        }
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "BLE Mesh provisioning complete");
        ESP_LOGI(TAG, "Device provisioned - net_idx=0x%04x, addr=0x%04x",
                 param->node_prov_complete.net_idx, param->node_prov_complete.addr);
        
        prov_complete(param->node_prov_complete.net_idx,
                     param->node_prov_complete.addr,
                     param->node_prov_complete.flags,
                     param->node_prov_complete.iv_index);
        
        // Start sensor data publication timer
        s_vnd_model = &vnd_models[0];

        const esp_timer_create_args_t pub_timer_args = {
            .callback = &pub_timer_cb,
            .name = "sensor_pub_timer"
        };
        ESP_ERROR_CHECK(esp_timer_create(&pub_timer_args, &pub_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(pub_timer, SENSOR_PUBLISH_INTERVAL_US));
        ESP_LOGI(TAG, "Sensor publication timer started (2 second interval)");
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}



/**
 * @brief Configuration server callback for model state changes.
 */
static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        default:
            break;
        }
    }
}

/**
 * @brief Vendor model callback for send/receive events.
 * 
 * Handles incoming vendor model messages and responds appropriately.
 */
static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SEND) {
            uint16_t tid = *(uint16_t *)param->model_operation.msg;
            ESP_LOGD(TAG, "Received message - opcode=0x%06" PRIx32 ", tid=0x%04x",
                    param->model_operation.opcode, tid);
            
            // Send response
            esp_err_t err = esp_ble_mesh_server_model_send_msg(
                &vnd_models[0],
                param->model_operation.ctx,
                ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                sizeof(tid),
                (uint8_t *)&tid);
            
            if (err) {
                ESP_LOGW(TAG, "Failed to send response: %s", esp_err_to_name(err));
            }
        }
        break;

    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code) {
            ESP_LOGW(TAG, "Message send failed - opcode=0x%06" PRIx32 ": %s",
                    param->model_send_comp.opcode,
                    esp_err_to_name(param->model_send_comp.err_code));
        } else {
            ESP_LOGD(TAG, "Message sent successfully - opcode=0x%06" PRIx32,
                    param->model_send_comp.opcode);
        }
        break;
    default:
        break;
    }
}

/**
 * @brief Initialize BLE Mesh stack and provisioning bearers.
 * 
 * Configures the BLE Mesh node with composition, provisioning capability,
 * and registers event callbacks.
 * 
 * @return ESP_OK on success, or error code on failure
 */
static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    // Register event callbacks
    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);

    // Initialize BLE Mesh stack
    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BLE Mesh stack: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "BLE Mesh stack initialized");

    // Set unprovisioned device name for discovery
    err = esp_ble_mesh_set_unprovisioned_device_name(BLE_MESH_DEVICE_NAME);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set device name: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Device name: %s", BLE_MESH_DEVICE_NAME);
    }

    // Enable provisioning on both ADV and GATT bearers
    err = esp_ble_mesh_node_prov_enable(
        (esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable provisioning: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "BLE Mesh provisioning enabled (PB-ADV + PB-GATT)");

    // Turn on green LED to indicate waiting for provisioning
    board_led_operation(LED_GREEN, LED_ON);

    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    return ESP_OK;
}

/**
 * @brief Application entry point.
 * 
 * Initializes all subsystems: NVS, board, sensors, Bluetooth, and BLE Mesh.
 */
void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "=== BLE Mesh Multi-Sensor Server Starting ===");

    // ========================================
    // Initialize Non-Volatile Storage (NVS)
    // ========================================
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_LOGI(TAG, "Erasing NVS due to no free pages");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS initialized");

    // ========================================
    // Initialize Board Peripherals
    // ========================================
    board_init();
    ESP_LOGI(TAG, "Board initialized (RGB LED ready)");

    // ========================================
    // Initialize Sensors
    // ========================================
#if ENABLE_AHT20
    ESP_LOGI(TAG, "Initializing AHT20 (temperature/humidity sensor)");
    ESP_ERROR_CHECK(aht20_init());
#endif

#if ENABLE_MAX30101
    ESP_LOGI(TAG, "Initializing MAX30101 (heart rate/SpO2 sensor)");
    ESP_ERROR_CHECK(max30101_init());
    ESP_ERROR_CHECK(max30101_start_sampling());
#endif

#if ENABLE_SGP30
    ESP_LOGI(TAG, "Initializing SGP30 (air quality sensor)");
    ESP_ERROR_CHECK(sgp30_init());
    
    // Start 1Hz timer immediately
    // SGP30 requires continuous 1Hz measurements to maintain air quality baseline
    const esp_timer_create_args_t sgp30_timer_args = {
        .callback = &sgp30_timer_cb,
        .name = "sgp30_baseline_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&sgp30_timer_args, &sgp30_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(sgp30_timer, SGP30_MEASUREMENT_INTERVAL_US));
    ESP_LOGI(TAG, "SGP30 baseline timer started (1 second interval)");
#endif

    // ========================================
    // Initialize Bluetooth Stack
    // ========================================
    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth initialization failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Bluetooth stack initialized");

    // Set the GAP device name
    err = esp_ble_gap_set_device_name(BLE_MESH_DEVICE_NAME);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set BLE GAP device name: %s", esp_err_to_name(err));
    }

    // ========================================
    // Initialize BLE Mesh
    // ========================================
    ble_mesh_get_dev_uuid(dev_uuid);
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "BLE Mesh initialization failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "=== All subsystems initialized successfully ===");
    ESP_LOGI(TAG, "Waiting for provisioning (green LED on)...");
}
