/* main.c - BLE Mesh SIG Sensor Server with Multi-Sensor Integration */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file main.c
 * @brief BLE Mesh SIG Sensor Server (Model ID 0x1100) with multi-sensor
 *        integration and periodic publishing.
 *
 * Migrated from vendor model (CID 0x02E5) to standard SIG Sensor Server
 * so that any compliant Sensor Client (e.g. nRF Mesh Sensor CLI) can
 * receive the data without vendor-specific knowledge.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"
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
#define TAG "MESH_SENSOR"

/** @brief Espressif company ID (used in composition data, NOT in model). */
#define CID_ESP 0x02E5
static uint32_t ble_seq = 0;
// ============================================================================
// SIG Sensor Property IDs
// Must match the Sensor Client on the nRF gateway!
// 0x004F and 0x0076 are official Bluetooth SIG properties.
// 0x0008, 0x0100-0x0104 are within the SIG property range but used here
// as application-specific properties for custom sensor values.
// ============================================================================

#define PROP_TEMPERATURE  0x004F   /**< Present Indoor Ambient Temperature (SIG) */
#define PROP_HUMIDITY     0x0076   /**< Present Indoor Relative Humidity (SIG) */
#define PROP_ECO2         0x0008   /**< eCO2 concentration (ppm) */
#define PROP_HEART_RATE   0x0100   /**< Heart rate (bpm) */
#define PROP_SPO2         0x0101   /**< Blood oxygen saturation (%) */
#define PROP_TVOC         0x0102   /**< Total VOC (ppb) */
#define PROP_RAW_RED      0x0103   /**< Raw RED PPG channel */
#define PROP_RAW_IR       0x0104   /**< Raw IR PPG channel */
#define PROP_SEQ          0x07FF   /**< Custom property for sequence number (for testing) */

// Timer Intervals (microseconds)
#define SENSOR_PUBLISH_INTERVAL_US  (2 * 1000 * 1000)  /**< 2 seconds */
#define SGP30_MEASUREMENT_INTERVAL_US (1 * 1000 * 1000) /**< 1 second */

// Sentinel values for invalid sensor readings
#define SENSOR_INVALID_U8   0xFF
#define SENSOR_INVALID_U16  0xFFFF
#define SENSOR_INVALID_U32  0xFFFFFFFF
#define SENSOR_INVALID_S16  ((int16_t)0x8000)

// ============================================================================
// Sensor Status Marshalling Helpers
// ============================================================================

/**
 * @brief Maximum marshalled Sensor Status payload size.
 *
 * Each property uses Format A (2-byte header) + up to 4 bytes data.
 * 9 properties x 6 bytes = 54 bytes max.
 */
#define MAX_SENSOR_PAYLOAD 54

/**
 * @brief Append one sensor property in Sensor Data Format A.
 *
 * BLE Mesh Sensor Data Format A (property_id <= 0x07FF):
 *   Bit  0     : Format = 0
 *   Bits 1-4   : Length (zero-based: 0 = 1 byte, 1 = 2 bytes, ...)
 *   Bits 5-15  : Property ID (11 bits)
 *   Followed by raw value bytes (little-endian)
 *
 * @param[out] buf         Output buffer
 * @param[in]  property_id Sensor Property ID (must be <= 0x07FF)
 * @param[in]  data        Raw value bytes (little-endian)
 * @param[in]  data_len    Length of raw value in bytes (1-16)
 * @return Number of bytes written (2 + data_len)
 */
static uint16_t marshal_format_a(uint8_t *buf, uint16_t property_id,
                                  const uint8_t *data, uint8_t data_len)
{
    /*
     * Format A MPID (16 bits, little-endian):
     *   bit  0      = format (0 for Format A)
     *   bits 1-4    = zero-based length  (actual_length - 1)
     *   bits 5-15   = property ID (11 bits)
     */
    uint16_t mpid = ((uint16_t)(property_id & 0x7FF) << 5)
                  | ((uint16_t)((data_len - 1) & 0xF) << 1)
                  | 0; /* format bit = 0 */

    buf[0] = (uint8_t)(mpid & 0xFF);
    buf[1] = (uint8_t)((mpid >> 8) & 0xFF);
    memcpy(&buf[2], data, data_len);
    return 2 + data_len;
}

/** @brief Helper: marshal a uint8 property. */
__attribute__((unused))
static uint16_t marshal_u8(uint8_t *buf, uint16_t prop, uint8_t val)
{
    return marshal_format_a(buf, prop, &val, 1);
}

/** @brief Helper: marshal a uint16 (little-endian) property. */
__attribute__((unused))
static uint16_t marshal_u16(uint8_t *buf, uint16_t prop, uint16_t val)
{
    uint8_t raw[2] = { (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    return marshal_format_a(buf, prop, raw, 2);
}

/** @brief Helper: marshal an int16 (little-endian) property. */
__attribute__((unused))
static uint16_t marshal_s16(uint8_t *buf, uint16_t prop, int16_t val)
{
    uint8_t raw[2] = { (uint8_t)(val & 0xFF), (uint8_t)((val >> 8) & 0xFF) };
    return marshal_format_a(buf, prop, raw, 2);
}

/** @brief Helper: marshal a uint32 (little-endian) property. */
__attribute__((unused))
static uint16_t marshal_u32(uint8_t *buf, uint16_t prop, uint32_t val)
{
    uint8_t raw[4] = {
        (uint8_t)(val & 0xFF),
        (uint8_t)((val >> 8) & 0xFF),
        (uint8_t)((val >> 16) & 0xFF),
        (uint8_t)((val >> 24) & 0xFF),
    };
    return marshal_format_a(buf, prop, raw, 4);
}

// ============================================================================
// Sensor State Definitions for SIG Sensor Server
// ============================================================================

/*
 * We count how many sensor properties are active at compile time so the
 * sensor_states[] array is exactly the right size.
 */
#define TEMP_SENSORS  ((ENABLE_AHT20) ? 2 : 0)      /* temp + humidity */
#define MAX_SENSORS   ((ENABLE_MAX30101) ? 4 : 0)    /* red, ir, hr, spo2 */
#define SGP_SENSORS   ((ENABLE_SGP30) ? 2 : 0)       /* eco2, tvoc */
#define SEQ_SENSORS  1     
#define NUM_SENSORS   (TEMP_SENSORS + MAX_SENSORS + SGP_SENSORS + SEQ_SENSORS)

/* Guard against zero-length array (at least 1 sensor must be enabled) */
#if NUM_SENSORS == 0
#error "At least one sensor must be enabled (ENABLE_AHT20, ENABLE_MAX30101, or ENABLE_SGP30)"
#endif


NET_BUF_SIMPLE_DEFINE_STATIC(raw_seq_buf, 4); /* Reusable buffer for marshaling sequence numbers (if needed) */
/*
 * Static net_buf_simple buffers for each sensor's raw data.
 * The Sensor Server state references these so it can answer GET requests.
 */
#if ENABLE_AHT20
NET_BUF_SIMPLE_DEFINE_STATIC(raw_temp, 2);
NET_BUF_SIMPLE_DEFINE_STATIC(raw_hum, 2);
#endif

#if ENABLE_MAX30101
NET_BUF_SIMPLE_DEFINE_STATIC(raw_red_buf, 4);
NET_BUF_SIMPLE_DEFINE_STATIC(raw_ir_buf, 4);
NET_BUF_SIMPLE_DEFINE_STATIC(raw_hr_buf, 1);
NET_BUF_SIMPLE_DEFINE_STATIC(raw_spo2_buf, 1);
#endif

#if ENABLE_SGP30
NET_BUF_SIMPLE_DEFINE_STATIC(raw_eco2, 2);
NET_BUF_SIMPLE_DEFINE_STATIC(raw_tvoc, 2);
#endif

/**
 * @brief Sensor states array.
 *
 * Each entry maps a Property ID to its descriptor and raw-data buffer.
 * The Sensor Server model uses these to answer Sensor Get /
 * Sensor Descriptor Get requests automatically.
 */
static esp_ble_mesh_sensor_state_t sensor_states[NUM_SENSORS] = {
    /* ---------- AHT20 ---------- */
#if ENABLE_AHT20
    {
        .sensor_property_id = PROP_TEMPERATURE,
        .descriptor = {
            .positive_tolerance = 0,
            .negative_tolerance = 0,
            .sampling_function  = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
            .measure_period     = 0x14, /* ~2 s  (raw value exponent) */
            .update_interval    = 0x14,
        },
        .sensor_data = {
            .format    = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length    = 2 - 1, /* field stores (length - 1) */
            .raw_value = &raw_temp,
        },
    },
    {
        .sensor_property_id = PROP_HUMIDITY,
        .descriptor = {
            .positive_tolerance = 0,
            .negative_tolerance = 0,
            .sampling_function  = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
            .measure_period     = 0x14,
            .update_interval    = 0x14,
        },
        .sensor_data = {
            .format    = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length    = 2 - 1,
            .raw_value = &raw_hum,
        },
    },
#endif /* ENABLE_AHT20 */

    /* ---------- MAX30101 ---------- */
#if ENABLE_MAX30101
    {
        .sensor_property_id = PROP_RAW_RED,
        .descriptor = {
            .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
            .measure_period    = 0x14,
            .update_interval   = 0x14,
        },
        .sensor_data = {
            .format    = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length    = 4 - 1,
            .raw_value = &raw_red_buf,
        },
    },
    {
        .sensor_property_id = PROP_RAW_IR,
        .descriptor = {
            .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
            .measure_period    = 0x14,
            .update_interval   = 0x14,
        },
        .sensor_data = {
            .format    = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length    = 4 - 1,
            .raw_value = &raw_ir_buf,
        },
    },
    {
        .sensor_property_id = PROP_HEART_RATE,
        .descriptor = {
            .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
            .measure_period    = 0x14,
            .update_interval   = 0x14,
        },
        .sensor_data = {
            .format    = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length    = 1 - 1,
            .raw_value = &raw_hr_buf,
        },
    },
    {
        .sensor_property_id = PROP_SPO2,
        .descriptor = {
            .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
            .measure_period    = 0x14,
            .update_interval   = 0x14,
        },
        .sensor_data = {
            .format    = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length    = 1 - 1,
            .raw_value = &raw_spo2_buf,
        },
    },
#endif /* ENABLE_MAX30101 */

    /* ---------- SGP30 ---------- */
#if ENABLE_SGP30
    {
        .sensor_property_id = PROP_ECO2,
        .descriptor = {
            .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
            .measure_period    = 0x12, /* ~1 s */
            .update_interval   = 0x14,
        },
        .sensor_data = {
            .format    = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length    = 2 - 1,
            .raw_value = &raw_eco2,
        },
    },
    {
        .sensor_property_id = PROP_TVOC,
        .descriptor = {
            .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
            .measure_period    = 0x12,
            .update_interval   = 0x14,
        },
        .sensor_data = {
            .format    = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length    = 2 - 1,
            .raw_value = &raw_tvoc,
        },
    },
#endif /* ENABLE_SGP30 */
    {
        .sensor_property_id = PROP_SEQ,
        .descriptor = {
            .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
            .measure_period    = 0x14,
            .update_interval   = 0x14,
        },
        .sensor_data = {
            .format    = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length    = 4 - 1,
            .raw_value = &raw_seq_buf,
        },
    },
};


// ============================================================================
// BLE Mesh Model Definitions
// ============================================================================

/** @brief Device UUID for BLE Mesh provisioning. */
static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10 };

/** @brief Configuration Server model state. */
static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit       = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay              = ESP_BLE_MESH_RELAY_DISABLED,
    .relay_retransmit   = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon             = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy         = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy         = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state       = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state       = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl        = 7,
};

/** @brief Sensor Server instance. */
static esp_ble_mesh_sensor_srv_t sensor_server = {
    .rsp_ctrl = {
        .status_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
    },
    .state_count = NUM_SENSORS,
    .states      = sensor_states,
};

/**
 * @brief Sensor Server publication context.
 *
 * Buffer must hold the full marshalled Sensor Status payload.
 * 2 (access header margin) + MAX_SENSOR_PAYLOAD.
 */
ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_pub, 2 + MAX_SENSOR_PAYLOAD, ROLE_NODE);

/** @brief SIG models on the primary element. */
static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_SENSOR_SRV(&sensor_pub, &sensor_server),
};

/** @brief Element list (single element, SIG models only, no vendor models). */
static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

/** @brief Node composition data. */
static esp_ble_mesh_comp_t composition = {
    .cid           = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements      = elements,
};

/** @brief Provisioning capabilities. */
static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

// ============================================================================
// Global Variables
// ============================================================================

/** @brief Publication timer handle. */
static esp_timer_handle_t pub_timer;

/** @brief Pointer to the Sensor Server model (set after provisioning). */
static esp_ble_mesh_model_t *s_sensor_model;

#if ENABLE_SGP30
/** @brief SGP30 1 Hz baseline timer. */
static esp_timer_handle_t sgp30_timer;
static uint16_t cached_eco2 = SENSOR_INVALID_U16;
static uint16_t cached_tvoc = SENSOR_INVALID_U16;
static uint32_t sgp30_read_count  = 0;
static uint32_t sgp30_error_count = 0;

/**
 * @brief 1 Hz timer to keep the SGP30 baseline algorithm running.
 */
static void sgp30_timer_cb(void *arg)
{
    (void)arg;
    esp_err_t err = sgp30_read_values(&cached_eco2, &cached_tvoc);
    if (err != ESP_OK) {
        cached_eco2 = SENSOR_INVALID_U16;
        cached_tvoc = SENSOR_INVALID_U16;
        sgp30_error_count++;
        if (sgp30_error_count % 10 == 0) {
            ESP_LOGW(TAG, "SGP30: %lu consecutive errors, %lu good reads",
                     sgp30_error_count, sgp30_read_count);
        }
        return;
    }
    sgp30_read_count++;
}
#endif /* ENABLE_SGP30 */

// ============================================================================
// Sensor Data Publication (Timer Callback)
// ============================================================================

/**
 * @brief Update the NET_BUF_SIMPLE data buffers used by the sensor states.
 *
 * This keeps the Sensor Server state in sync so that any incoming
 * Sensor Get requests also return fresh data.
 */
static void update_sensor_state_buffers(int16_t temp_x100, uint16_t hum_x100,
                                         uint32_t raw_red, uint32_t raw_ir,
                                         uint8_t hr, uint8_t spo2,
                                         uint16_t eco2, uint16_t tvoc, uint32_t seq)
{

net_buf_simple_reset(&raw_seq_buf);
net_buf_simple_add_le32(&raw_seq_buf, seq);
#if ENABLE_AHT20
    net_buf_simple_reset(&raw_temp);
    net_buf_simple_add_le16(&raw_temp, (uint16_t)temp_x100);

    net_buf_simple_reset(&raw_hum);
    net_buf_simple_add_le16(&raw_hum, hum_x100);
#endif

#if ENABLE_MAX30101
    net_buf_simple_reset(&raw_red_buf);
    net_buf_simple_add_le32(&raw_red_buf, raw_red);

    net_buf_simple_reset(&raw_ir_buf);
    net_buf_simple_add_le32(&raw_ir_buf, raw_ir);

    net_buf_simple_reset(&raw_hr_buf);
    net_buf_simple_add_u8(&raw_hr_buf, hr);

    net_buf_simple_reset(&raw_spo2_buf);
    net_buf_simple_add_u8(&raw_spo2_buf, spo2);
#endif

#if ENABLE_SGP30
    net_buf_simple_reset(&raw_eco2);
    net_buf_simple_add_le16(&raw_eco2, eco2);

    net_buf_simple_reset(&raw_tvoc);
    net_buf_simple_add_le16(&raw_tvoc, tvoc);
#endif

    /* Suppress unused-parameter warnings for disabled sensors */
    (void)temp_x100; (void)hum_x100;
    (void)raw_red; (void)raw_ir;
    (void)hr; (void)spo2;
    (void)eco2; (void)tvoc; (void)seq;
}

/**
 * @brief Periodic publication callback.
 *
 * Reads all enabled sensors, marshals the data in BLE Mesh Sensor Status
 * format (Format A), and publishes via the SIG Sensor Server model using
 * opcode 0x0052 (Sensor Status).
 */
static void pub_timer_cb(void *arg)
{
    uint32_t seq = ble_seq++;
    (void)arg;

    if (!s_sensor_model || !s_sensor_model->pub) {
        ESP_LOGW(TAG, "Sensor model or publication not ready");
        return;
    }

    esp_err_t err;
    int16_t  temp_x100     = SENSOR_INVALID_S16;
    uint16_t humidity_x100 = SENSOR_INVALID_U16;
    uint32_t raw_red       = SENSOR_INVALID_U32;
    uint32_t raw_ir        = SENSOR_INVALID_U32;
    uint8_t  heart_rate    = SENSOR_INVALID_U8;
    uint8_t  spo2_val      = SENSOR_INVALID_U8;
    uint16_t eco2          = SENSOR_INVALID_U16;
    uint16_t tvoc          = SENSOR_INVALID_U16;

    /* ---- Read sensors ---- */

#if ENABLE_AHT20
    err = aht20_read_x100(&temp_x100, &humidity_x100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AHT20 read failed: %s", esp_err_to_name(err));
        temp_x100      = SENSOR_INVALID_S16;
        humidity_x100  = SENSOR_INVALID_U16;
    }
#endif

#if ENABLE_MAX30101
    max30101_get_raw_values(&raw_red, &raw_ir);
    heart_rate = max30101_get_heart_rate();
    spo2_val   = max30101_get_spo2();
#endif

#if ENABLE_SGP30
    eco2 = cached_eco2;
    tvoc = cached_tvoc;
    ESP_LOGD(TAG, "SGP30: eCO2=%u ppm, TVOC=%u ppb", eco2, tvoc);
#endif

    /* ---- Update sensor state buffers (for GET requests) ---- */
    update_sensor_state_buffers(temp_x100, humidity_x100,
                                 raw_red, raw_ir,
                                 heart_rate, spo2_val,
                                 eco2, tvoc, seq);

    /* ---- Marshal Sensor Status payload ---- */
    uint8_t payload[MAX_SENSOR_PAYLOAD];
    uint16_t offset = 0;
offset += marshal_u32(&payload[offset], PROP_SEQ, seq);
#if ENABLE_AHT20
    offset += marshal_s16(&payload[offset], PROP_TEMPERATURE, temp_x100);
    offset += marshal_u16(&payload[offset], PROP_HUMIDITY, humidity_x100);
#endif

#if ENABLE_MAX30101
    offset += marshal_u32(&payload[offset], PROP_RAW_RED, raw_red);
    offset += marshal_u32(&payload[offset], PROP_RAW_IR, raw_ir);
    offset += marshal_u8(&payload[offset],  PROP_HEART_RATE, heart_rate);
    offset += marshal_u8(&payload[offset],  PROP_SPO2, spo2_val);
#endif

#if ENABLE_SGP30
    offset += marshal_u16(&payload[offset], PROP_ECO2, eco2);
    offset += marshal_u16(&payload[offset], PROP_TVOC, tvoc);
#endif


    /* ---- Publish Sensor Status (opcode 0x0052) ---- */
    err = esp_ble_mesh_model_publish(
        s_sensor_model,
        ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS,
        offset,
        payload,
        ROLE_NODE);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Sensor Status publish failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGD(TAG, "Published Sensor Status (%u bytes)", offset);
#if ENABLE_AHT20
        ESP_LOGI(TAG, "  Seq=%lu  T=%d.%02d°C  RH=%u.%02u%%",
                 (unsigned long)seq,
                 temp_x100 / 100,
                 (temp_x100 < 0) ? -(temp_x100 % 100) : (temp_x100 % 100),
                 humidity_x100 / 100, humidity_x100 % 100);
#endif
#if ENABLE_MAX30101
        ESP_LOGI(TAG, "  Seq=%lu  HR=%u bpm  SpO2=%u%%  RED=%lu  IR=%lu",
                 (unsigned long)seq,
                 heart_rate, spo2_val,
                 (unsigned long)raw_red, (unsigned long)raw_ir);
#endif
#if ENABLE_SGP30
        ESP_LOGI(TAG, "  Seq=%lu  eCO2=%u ppm  TVOC=%u ppb", 
                 (unsigned long)seq, eco2, tvoc);
#endif
    }
}

// ============================================================================
// BLE Mesh Callbacks
// ============================================================================

/**
 * @brief Handle provisioning completion.
 */
static void prov_complete(uint16_t net_idx, uint16_t addr,
                          uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "Provisioning complete - net_idx=0x%04x, addr=0x%04x",
             net_idx, addr);
    ESP_LOGD(TAG, "Flags=0x%02x, IV index=0x%08" PRIx32, flags, iv_index);
    board_led_operation(LED_GREEN, LED_OFF);
}

/**
 * @brief Provisioning event callback.
 */
static void ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                     esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "PROV_REGISTER_COMP, err=%d",
                 param->prov_register_comp.err_code);
        break;

    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "PROV_ENABLE_COMP, err=%d",
                 param->node_prov_enable_comp.err_code);
        break;

    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "PROV_LINK_OPEN, bearer %s",
                 param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV
                 ? "PB-ADV" : "PB-GATT");
        break;

    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "PROV_LINK_CLOSE, bearer %s",
                 param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV
                 ? "PB-ADV" : "PB-GATT");
        break;

    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "Device provisioned: net_idx=0x%04x, addr=0x%04x",
                 param->node_prov_complete.net_idx,
                 param->node_prov_complete.addr);

        prov_complete(param->node_prov_complete.net_idx,
                      param->node_prov_complete.addr,
                      param->node_prov_complete.flags,
                      param->node_prov_complete.iv_index);

        /* Store pointer to the Sensor Server model (second in root_models[]) */
        s_sensor_model = &root_models[1];

        /* Start periodic sensor publication */
        const esp_timer_create_args_t timer_args = {
            .callback = &pub_timer_cb,
            .name     = "sensor_pub_timer",
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &pub_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(pub_timer,
                                                  SENSOR_PUBLISH_INTERVAL_US));
        ESP_LOGI(TAG, "Sensor publication timer started (2 s interval)");
        break;

    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "PROV_RESET");
        break;

    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "SET_UNPROV_DEV_NAME_COMP, err=%d",
                 param->node_set_unprov_dev_name_comp.err_code);
        break;

    default:
        break;
    }
}

/**
 * @brief Configuration Server callback.
 */
static void ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                      esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event != ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        return;
    }

    switch (param->ctx.recv_op) {
    case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        ESP_LOGI(TAG, "AppKey Add: net_idx=0x%04x, app_idx=0x%04x",
                 param->value.state_change.appkey_add.net_idx,
                 param->value.state_change.appkey_add.app_idx);
        ESP_LOG_BUFFER_HEX("AppKey",
                           param->value.state_change.appkey_add.app_key, 16);
        break;

    case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
        ESP_LOGI(TAG, "Model App Bind: elem=0x%04x, app_idx=0x%04x, "
                 "cid=0x%04x, mod=0x%04x",
                 param->value.state_change.mod_app_bind.element_addr,
                 param->value.state_change.mod_app_bind.app_idx,
                 param->value.state_change.mod_app_bind.company_id,
                 param->value.state_change.mod_app_bind.model_id);
        break;

    case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
        ESP_LOGI(TAG, "Model Sub Add: elem=0x%04x, sub_addr=0x%04x, "
                 "cid=0x%04x, mod=0x%04x",
                 param->value.state_change.mod_sub_add.element_addr,
                 param->value.state_change.mod_sub_add.sub_addr,
                 param->value.state_change.mod_sub_add.company_id,
                 param->value.state_change.mod_sub_add.model_id);
        break;

    default:
        break;
    }
}

/**
 * @brief Sensor Server callback.
 *
 * With AUTO_RSP enabled, the mesh stack answers Sensor Get /
 * Sensor Descriptor Get requests automatically using sensor_states[].
 * This callback is for logging only.
 *
 * ESP-IDF 5.5 event names:
 *   ESP_BLE_MESH_SENSOR_SERVER_STATE_CHANGE_EVT
 *   ESP_BLE_MESH_SENSOR_SERVER_RECV_GET_MSG_EVT
 *   ESP_BLE_MESH_SENSOR_SERVER_RECV_SET_MSG_EVT
 */
static void ble_mesh_sensor_server_cb(esp_ble_mesh_sensor_server_cb_event_t event,
                                      esp_ble_mesh_sensor_server_cb_param_t *param)
{
    ESP_LOGI(TAG, "Sensor server event: %d, opcode: 0x%04" PRIx32,
             event, param->ctx.recv_op);

    switch (event) {
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_GET_MSG_EVT:
        ESP_LOGI(TAG, "Sensor Get from 0x%04x, opcode 0x%04" PRIx32,
                 param->ctx.addr, param->ctx.recv_op);
        break;

    case ESP_BLE_MESH_SENSOR_SERVER_RECV_SET_MSG_EVT:
        ESP_LOGI(TAG, "Sensor Set from 0x%04x, opcode 0x%04" PRIx32,
                 param->ctx.addr, param->ctx.recv_op);
        break;

    case ESP_BLE_MESH_SENSOR_SERVER_STATE_CHANGE_EVT:
        ESP_LOGI(TAG, "Sensor state change, opcode 0x%04" PRIx32,
                 param->ctx.recv_op);
        break;

    default:
        ESP_LOGD(TAG, "Unhandled sensor server event: %d", event);
        break;
    }
}

// ============================================================================
// BLE Mesh Initialization
// ============================================================================

/**
 * @brief Initialize BLE Mesh stack with SIG Sensor Server model.
 */
static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    /* Register callbacks */
    esp_ble_mesh_register_prov_callback(ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(ble_mesh_config_server_cb);
    esp_ble_mesh_register_sensor_server_callback(ble_mesh_sensor_server_cb);

    /* Initialize mesh stack */
    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mesh stack init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "BLE Mesh stack initialized");

    /* Set unprovisioned device name */
    err = esp_ble_mesh_set_unprovisioned_device_name(BLE_MESH_DEVICE_NAME);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Set device name failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Device name: %s", BLE_MESH_DEVICE_NAME);
    }

    /* Enable provisioning on ADV + GATT bearers */
    err = esp_ble_mesh_node_prov_enable(
        (esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Enable provisioning failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Green LED = waiting for provisioning */
    board_led_operation(LED_GREEN, LED_ON);
    ESP_LOGI(TAG, "BLE Mesh Sensor Server node ready (PB-ADV + PB-GATT)");

    return ESP_OK;
}

// ============================================================================
// Application Entry Point
// ============================================================================

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "=== BLE Mesh SIG Sensor Server Starting ===");

    /* ---- NVS ---- */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_LOGI(TAG, "Erasing NVS (no free pages)");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* ---- Board ---- */
    board_init();
    ESP_LOGI(TAG, "Board initialized");

    /* ---- Sensors ---- */
#if ENABLE_AHT20
    ESP_LOGI(TAG, "Initializing AHT20 (temperature / humidity)");
    ESP_ERROR_CHECK(aht20_init());
#endif

#if ENABLE_MAX30101
    ESP_LOGI(TAG, "Initializing MAX30101 (heart rate / SpO2)");
    ESP_ERROR_CHECK(max30101_init());
    ESP_ERROR_CHECK(max30101_start_sampling());
#endif

#if ENABLE_SGP30
    ESP_LOGI(TAG, "Initializing SGP30 (air quality)");
    ESP_ERROR_CHECK(sgp30_init());

    const esp_timer_create_args_t sgp30_args = {
        .callback = &sgp30_timer_cb,
        .name     = "sgp30_baseline",
    };
    ESP_ERROR_CHECK(esp_timer_create(&sgp30_args, &sgp30_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(sgp30_timer,
                                              SGP30_MEASUREMENT_INTERVAL_US));
    ESP_LOGI(TAG, "SGP30 baseline timer started (1 s interval)");
#endif

    /* ---- Bluetooth ---- */
    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth init failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Bluetooth stack initialized");

    err = esp_ble_gap_set_device_name(BLE_MESH_DEVICE_NAME);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Set GAP name failed: %s", esp_err_to_name(err));
    }

    /* ---- BLE Mesh ---- */
    ble_mesh_get_dev_uuid(dev_uuid);
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "BLE Mesh init failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "=== All subsystems initialized ===");
    ESP_LOGI(TAG, "Waiting for provisioning (green LED on)...");
}