/* main.c - BLE Mesh Sensor Node (baseline-test stable)
 *
 * Ziele dieser Version:
 *  - BLE-Test-Control via Vendor Model: ARM / START / STOP
 *  - START darf mehrfach empfangen werden, ohne neu zu schedulen
 *  - periodische Test-Publishes bleiben unsegmentiert: 9 bis 10 Byte
 *  - kein Boot-Marker-Publish im Testlauf, damit keine 20+ Byte Pakete entstehen
 *  - Sensor Status enthält immer: SEQ + genau 1 Messwert
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_pm.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_health_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"
#include "config.h"
#include "sensors/aht20.h"
#include "sensors/max30101.h"
#include "sensors/SGP30.h"

#define TAG "MESH_SENSOR"
#define CID_ESP 0x02E5

static atomic_uint ble_seq = 0;
static uint32_t    g_boot_id = 0;

/* ================================================================
 * NVS persistence: boot_id + sequence high-water mark
 * ================================================================ */
#define NVS_NS          "ble_node"
#define NVS_KEY_BOOT_ID "boot_id"
#define NVS_KEY_SEQ_HI  "seq_hi"

static void boot_state_load(void)
{
    nvs_handle_t h;

    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed — boot_id=1, seq=0");
        g_boot_id = 1;
        return;
    }

    uint32_t prev = 0;
    nvs_get_u32(h, NVS_KEY_BOOT_ID, &prev);
    g_boot_id = prev + 1;
    nvs_set_u32(h, NVS_KEY_BOOT_ID, g_boot_id);

#if BASELINE_TEST
    uint32_t seq_hi = 0;
    nvs_get_u32(h, NVS_KEY_SEQ_HI, &seq_hi);
    atomic_store(&ble_seq, seq_hi + 16);
#endif

    nvs_commit(h);
    nvs_close(h);

    ESP_LOGI(TAG, "Boot state: boot_id=%lu seq_start=%u",
             (unsigned long)g_boot_id,
             (unsigned)atomic_load(&ble_seq));
}

#if BASELINE_TEST
static void boot_state_save_seq(uint32_t seq)
{
    if ((seq & 0x1F) != 0) {
        return;
    }

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }

    nvs_set_u32(h, NVS_KEY_SEQ_HI, seq);
    nvs_commit(h);
    nvs_close(h);
}
#endif

/* ================================================================
 * Property IDs
 * ================================================================ */
#define PROP_TEMPERATURE  0x004F
#define PROP_HUMIDITY     0x0076
#define PROP_ECO2         0x0008
#define PROP_HEART_RATE   0x0100
#define PROP_SPO2         0x0101
#define PROP_TVOC         0x0102
#define PROP_RAW_RED      0x0103
#define PROP_RAW_IR       0x0104
#define PROP_SEQ          0x07FF
#define PROP_SWITCH       0x0105
#define PROP_LIGHT_STATE  0x0106

/* Diese bleiben definiert, werden aber in dieser Version NICHT periodisch gesendet,
 * weil sie die Payload wieder auf 20+ Byte aufblasen würden. */
#define PROP_TX_MS        0x0110
#define PROP_BOOT_ID      0x0111

/* ================================================================
 * Publish / control tuning
 * ================================================================ */
/* SENSOR_PUBLISH_INTERVAL_MS is the canonical setting.
 * Override SENSOR_PUBLISH_INTERVAL_MS to match for legacy log messages. */
#ifndef SENSOR_PUBLISH_INTERVAL_MS
#  if BASELINE_TEST
#    define SENSOR_PUBLISH_INTERVAL_MS 200
#  else
#    define SENSOR_PUBLISH_INTERVAL_MS 5000
#  endif
#endif

#define SENSOR_PUBLISH_INTERVAL_US  ((uint64_t)SENSOR_PUBLISH_INTERVAL_MS * 1000ULL)
#define SGP30_MEASUREMENT_INTERVAL_US    (1ULL * 1000ULL * 1000ULL)

#define SENSOR_INVALID_U8    0xFF
#define SENSOR_INVALID_U16   0xFFFF
#define SENSOR_INVALID_U32   0xFFFFFFFF
#define SENSOR_INVALID_S16   ((int16_t)0x8000)

/* Muss klein bleiben. Wir brechen zusätzlich bei >11 Byte ab. */
#define MAX_SENSOR_PAYLOAD      24
#define CONTROL_QUIET_TIME_MS   300

#ifndef BUTTON_EVENT_RETX_COUNT
#  if BASELINE_TEST
#    define BUTTON_EVENT_RETX_COUNT 7
#  else
#    define BUTTON_EVENT_RETX_COUNT 7
#  endif
#endif

#ifndef BUTTON_EVENT_RETX_INTERVAL_MS
#  if BASELINE_TEST
#    define BUTTON_EVENT_RETX_INTERVAL_MS 40
#  else
#    define BUTTON_EVENT_RETX_INTERVAL_MS 120
#  endif
#endif

/* ================================================================
 * Time helpers / quiet time
 * ================================================================ */
static volatile uint32_t s_mesh_quiet_until_ms = 0;
static uint8_t s_pub_phase = 0;

static inline uint32_t now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static inline void mesh_quiet_for(uint32_t ms)
{
    s_mesh_quiet_until_ms = now_ms() + ms;
}

static inline bool mesh_quiet_active(void)
{
    return ((int32_t)(s_mesh_quiet_until_ms - now_ms()) > 0);
}

/* ================================================================
 * Deterministic thesis test control
 * ================================================================ */
typedef struct {
    bool running;
    bool armed;
    bool scheduled;
    uint32_t run_id;
    uint32_t scenario;
    uint32_t start_ms;
} test_run_state_t;

static test_run_state_t s_test_run = {
    .running   = false,
    .armed     = false,
    .scheduled = false,
    .run_id    = 0,
    .scenario  = TEST_SCENARIO_NONE,
    .start_ms  = 0,
};

static inline uint32_t test_elapsed_ms(void)
{
    if (!s_test_run.running) {
        return 0;
    }

    uint32_t now = now_ms();

    if ((int32_t)(now - s_test_run.start_ms) < 0) {
        return 0;
    }

    return now - s_test_run.start_ms;
}

#if TEST_PROFILE
static bool test_alert_window_active(void)
{
    uint32_t t = test_elapsed_ms();
    return t >= TEST_ALERT_START_MS && t < TEST_ALERT_END_MS;
}

static bool test_this_ble_node_alerts(void)
{
    if (!test_alert_window_active()) {
        return false;
    }

    if (s_test_run.scenario == TEST_SCENARIO_BLE_ALERT ||
        s_test_run.scenario == TEST_SCENARIO_BOTH_ALERT) {
        return TEST_NODE_ID == TEST_ALERT_BLE_NODE_ID;
    }

    return false;
}

static int16_t test_ble_temp_x100(void)
{
    return test_this_ble_node_alerts() ? 3600 : 2500;
}

static uint16_t test_ble_hum_x100(void)
{
    return 4200;
}

static uint16_t test_ble_eco2(void)
{
    return test_this_ble_node_alerts() ? 1500 : 450;
}

static uint16_t test_ble_tvoc(void)
{
    return test_this_ble_node_alerts() ? 700 : 400;
}
#endif

/* ================================================================
 * Marshalling helpers
 * ================================================================ */
static uint16_t marshal_format_a(uint8_t *buf,
                                 uint16_t property_id,
                                 const uint8_t *data,
                                 uint8_t data_len)
{
    uint16_t mpid = ((uint16_t)(property_id & 0x7FF) << 5)
                  | ((uint16_t)((data_len - 1) & 0xF) << 1)
                  | 0;

    buf[0] = (uint8_t)(mpid & 0xFF);
    buf[1] = (uint8_t)((mpid >> 8) & 0xFF);
    memcpy(&buf[2], data, data_len);

    return 2 + data_len;
}

__attribute__((unused))
static uint16_t marshal_u8(uint8_t *buf, uint16_t prop, uint8_t val)
{
    return marshal_format_a(buf, prop, &val, 1);
}

__attribute__((unused))
static uint16_t marshal_u16(uint8_t *buf, uint16_t prop, uint16_t val)
{
    uint8_t raw[2] = {
        (uint8_t)(val),
        (uint8_t)(val >> 8),
    };

    return marshal_format_a(buf, prop, raw, 2);
}

__attribute__((unused))
static uint16_t marshal_s16(uint8_t *buf, uint16_t prop, int16_t val)
{
    uint8_t raw[2] = {
        (uint8_t)(val),
        (uint8_t)((val >> 8) & 0xFF),
    };

    return marshal_format_a(buf, prop, raw, 2);
}

__attribute__((unused))
static uint16_t marshal_u32(uint8_t *buf, uint16_t prop, uint32_t val)
{
    uint8_t raw[4] = {
        (uint8_t)(val),
        (uint8_t)(val >> 8),
        (uint8_t)(val >> 16),
        (uint8_t)(val >> 24),
    };

    return marshal_format_a(buf, prop, raw, 4);
}

static uint32_t get_next_seq(void)
{
    uint32_t s = atomic_fetch_add(&ble_seq, 1);

#if BASELINE_TEST
    boot_state_save_seq(s);
#endif

    return s;
}

/* ================================================================
 * Sensor model state
 * ================================================================ */
#define TEMP_SENSORS  ((ENABLE_AHT20)    ? 2 : 0)
#define MAX_SENSORS   ((ENABLE_MAX30101) ? 4 : 0)
#define SGP_SENSORS   ((ENABLE_SGP30)    ? 2 : 0)
#define BTN_SENSORS   ((ENABLE_BUTTON)   ? 1 : 0)
#define LGT_SENSORS   ((ENABLE_LIGHT)    ? 1 : 0)
#define SEQ_SENSORS   1
#define NUM_SENSORS   (TEMP_SENSORS + MAX_SENSORS + SGP_SENSORS + BTN_SENSORS + LGT_SENSORS + SEQ_SENSORS)

#if NUM_SENSORS == 0
#error "At least one sensor must be enabled"
#endif

NET_BUF_SIMPLE_DEFINE_STATIC(raw_seq_buf, 4);

#if ENABLE_AHT20
NET_BUF_SIMPLE_DEFINE_STATIC(raw_temp, 2);
NET_BUF_SIMPLE_DEFINE_STATIC(raw_hum,  2);
#endif

#if ENABLE_MAX30101
NET_BUF_SIMPLE_DEFINE_STATIC(raw_red_buf,  4);
NET_BUF_SIMPLE_DEFINE_STATIC(raw_ir_buf,   4);
NET_BUF_SIMPLE_DEFINE_STATIC(raw_hr_buf,   1);
NET_BUF_SIMPLE_DEFINE_STATIC(raw_spo2_buf, 1);
#endif

#if ENABLE_SGP30
NET_BUF_SIMPLE_DEFINE_STATIC(raw_eco2, 2);
NET_BUF_SIMPLE_DEFINE_STATIC(raw_tvoc, 2);
#endif

#if ENABLE_BUTTON
NET_BUF_SIMPLE_DEFINE_STATIC(raw_switch, 1);
#endif

#if ENABLE_LIGHT
NET_BUF_SIMPLE_DEFINE_STATIC(raw_light_state, 1);
#endif

static esp_ble_mesh_sensor_state_t sensor_states[NUM_SENSORS] = {
#if ENABLE_AHT20
    {
        .sensor_property_id = PROP_TEMPERATURE,
        .descriptor = {
            .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
            .measure_period = 0x14,
            .update_interval = 0x14,
        },
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 2 - 1,
            .raw_value = &raw_temp,
        },
    },
    {
        .sensor_property_id = PROP_HUMIDITY,
        .descriptor = {
            .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
            .measure_period = 0x14,
            .update_interval = 0x14,
        },
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 2 - 1,
            .raw_value = &raw_hum,
        },
    },
#endif

#if ENABLE_MAX30101
    {
        .sensor_property_id = PROP_RAW_RED,
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 4 - 1,
            .raw_value = &raw_red_buf,
        },
    },
    {
        .sensor_property_id = PROP_RAW_IR,
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 4 - 1,
            .raw_value = &raw_ir_buf,
        },
    },
    {
        .sensor_property_id = PROP_HEART_RATE,
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 1 - 1,
            .raw_value = &raw_hr_buf,
        },
    },
    {
        .sensor_property_id = PROP_SPO2,
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 1 - 1,
            .raw_value = &raw_spo2_buf,
        },
    },
#endif

#if ENABLE_SGP30
    {
        .sensor_property_id = PROP_ECO2,
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 2 - 1,
            .raw_value = &raw_eco2,
        },
    },
    {
        .sensor_property_id = PROP_TVOC,
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 2 - 1,
            .raw_value = &raw_tvoc,
        },
    },
#endif

#if ENABLE_BUTTON
    {
        .sensor_property_id = PROP_SWITCH,
        .descriptor = {
            .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
            .measure_period = 0,
            .update_interval = 0,
        },
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 1 - 1,
            .raw_value = &raw_switch,
        },
    },
#endif

#if ENABLE_LIGHT
    {
        .sensor_property_id = PROP_LIGHT_STATE,
        .descriptor = {
            .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
            .measure_period = 0,
            .update_interval = 0,
        },
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 1 - 1,
            .raw_value = &raw_light_state,
        },
    },
#endif

    {
        .sensor_property_id = PROP_SEQ,
        .descriptor = {
            .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
            .measure_period = 0x14,
            .update_interval = 0x14,
        },
        .sensor_data = {
            .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
            .length = 4 - 1,
            .raw_value = &raw_seq_buf,
        },
    },
};

/* ================================================================
 * Sensor cache for BASELINE_TEST
 * ================================================================ */
#if BASELINE_TEST

#if ENABLE_AHT20
static int16_t  s_cached_temp_x100 = 0;
static uint16_t s_cached_hum_x100  = 0;
#endif

#if ENABLE_MAX30101
static uint32_t s_cached_raw_red = 0;
static uint32_t s_cached_raw_ir  = 0;
static uint8_t  s_cached_hr      = 0;
static uint8_t  s_cached_spo2    = 0;
#endif

static esp_timer_handle_t s_sensor_read_timer = NULL;

static void sensor_read_cb(void *arg)
{
    (void)arg;

#if ENABLE_AHT20
    int16_t t;
    uint16_t h;
    aht20_read_x100(&t, &h);
    s_cached_temp_x100 = t;
    s_cached_hum_x100  = h;
#endif

#if ENABLE_MAX30101
    max30101_get_raw_values(&s_cached_raw_red, &s_cached_raw_ir);
    s_cached_hr   = max30101_get_heart_rate();
    s_cached_spo2 = max30101_get_spo2();
#endif
}

#endif

/* ================================================================
 * Global mesh model pointers
 * ================================================================ */
static esp_ble_mesh_model_t *s_sensor_model = NULL;

/* ================================================================
 * WS2812 RGB LED / Light
 * ================================================================ */
#if ENABLE_LIGHT

static rmt_channel_handle_t s_rmt_chan = NULL;
static rmt_encoder_handle_t s_rmt_enc  = NULL;
static bool s_light_on = false;

static void led_hw_init(void)
{
    rmt_tx_channel_config_t cfg = {
        .gpio_num          = LIGHT_GPIO_PIN,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = 10000000,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&cfg, &s_rmt_chan));

    rmt_bytes_encoder_config_t enc = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 4,
            .level1 = 0,
            .duration1 = 8,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 8,
            .level1 = 0,
            .duration1 = 4,
        },
        .flags.msb_first = 1,
    };

    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&enc, &s_rmt_enc));
    ESP_ERROR_CHECK(rmt_enable(s_rmt_chan));

    ESP_LOGI(TAG, "WS2812 init GPIO%d", LIGHT_GPIO_PIN);
}

static void led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t grb[3] = { g, r, b };
    rmt_transmit_config_t tx = { .loop_count = 0 };

    rmt_transmit(s_rmt_chan, s_rmt_enc, grb, sizeof(grb), &tx);
    rmt_tx_wait_all_done(s_rmt_chan, 100);
}

static void light_feedback_publish(void)
{
    if (!s_sensor_model || !s_sensor_model->pub) {
        ESP_LOGW(TAG, "Light feedback: sensor model not ready");
        return;
    }

    uint8_t payload[16];
    uint16_t off = 0;
    uint32_t seq = get_next_seq();

    off += marshal_u32(payload + off, PROP_SEQ, seq);
    off += marshal_u8(payload + off, PROP_LIGHT_STATE, s_light_on ? 1 : 0);

    if (off > 11) {
        ESP_LOGE(TAG, "BUG: light feedback would be segmented: %u bytes", off);
        return;
    }

    esp_err_t err = esp_ble_mesh_model_publish(
        s_sensor_model,
        ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS,
        off,
        payload,
        ROLE_NODE);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Light feedback publish failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Light feedback published: state=%s seq=%lu len=%u",
                 s_light_on ? "ON" : "OFF",
                 (unsigned long)seq,
                 off);
    }
}

static void light_set(bool on)
{
    s_light_on = on;

    if (on) {
        led_set_rgb(10, 10, 10);
    } else {
        led_set_rgb(0, 0, 0);
    }

    ESP_LOGI(TAG, "Light %s", on ? "ON" : "OFF");

    light_feedback_publish();
    mesh_quiet_for(CONTROL_QUIET_TIME_MS);
}

#endif

/* ================================================================
 * Button
 * ================================================================ */
#if ENABLE_BUTTON

static TaskHandle_t s_btn_task_handle = NULL;
static volatile bool s_btn_irq_blocked = false;
static esp_ble_mesh_model_t *s_sensor_model_btn = NULL;

static void IRAM_ATTR btn_isr_handler(void *arg)
{
    (void)arg;

    if (s_btn_irq_blocked) {
        return;
    }

    if (s_btn_task_handle == NULL) {
        return;
    }

    s_btn_irq_blocked = true;
    gpio_intr_disable(BUTTON_GPIO_PIN);

    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_btn_task_handle, &woken);
    portYIELD_FROM_ISR(woken);
}

static void button_hw_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };

    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO_PIN, btn_isr_handler, NULL));

    ESP_LOGI(TAG, "Button init GPIO%d", BUTTON_GPIO_PIN);
}

#define GATEWAY_MESH_ADDR  0x0001
#define BUTTON_SEND_TTL    2
#define BUTTON_NET_IDX     0x0000
#define BUTTON_APP_IDX     0x0000

static void button_send_once(uint32_t event_seq)
{
    if (!s_sensor_model_btn) {
        ESP_LOGW(TAG, "Button: model not ready");
        return;
    }

    uint8_t payload[16];
    uint16_t len = 0;

    len += marshal_u32(payload + len, PROP_SEQ, event_seq);
    len += marshal_u8(payload + len, PROP_SWITCH, 1);

    if (len > 11) {
        ESP_LOGE(TAG, "BUG: button payload would be segmented: %u bytes", len);
        return;
    }

    esp_ble_mesh_msg_ctx_t ctx = {
        .net_idx  = BUTTON_NET_IDX,
        .app_idx  = BUTTON_APP_IDX,
        .addr     = GATEWAY_MESH_ADDR,
        .send_ttl = BUTTON_SEND_TTL,
        .send_rel = false,
    };

    esp_err_t err = esp_ble_mesh_server_model_send_msg(
        s_sensor_model_btn,
        &ctx,
        ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS,
        len,
        payload);

    ESP_LOGI(TAG, "Button TX seq=%lu -> 0x%04X %s",
             (unsigned long)event_seq,
             GATEWAY_MESH_ADDR,
             err == ESP_OK ? "OK" : esp_err_to_name(err));
}

static void button_task(void *arg)
{
    (void)arg;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(20));
        if (gpio_get_level(BUTTON_GPIO_PIN) != 0) {
            s_btn_irq_blocked = false;
            gpio_intr_enable(BUTTON_GPIO_PIN);
            continue;
        }

        s_btn_irq_blocked = false;
        gpio_intr_enable(BUTTON_GPIO_PIN);

        if (!s_sensor_model_btn || !s_sensor_model_btn->pub) {
            ESP_LOGW(TAG, "Button: model not ready");
            continue;
        }

        uint32_t event_seq = get_next_seq();

        ESP_LOGI(TAG, "Button ACCEPTED -> event seq=%lu",
                 (unsigned long)event_seq);

        mesh_quiet_for(CONTROL_QUIET_TIME_MS);

        for (int i = 0; i < BUTTON_EVENT_RETX_COUNT; i++) {
            button_send_once(event_seq);

            if (i + 1 < BUTTON_EVENT_RETX_COUNT) {
                vTaskDelay(pdMS_TO_TICKS(BUTTON_EVENT_RETX_INTERVAL_MS));
            }
        }
    }
}

#endif

/* ================================================================
 * Generic OnOff Server
 * ================================================================ */
#if ENABLE_LIGHT

static esp_ble_mesh_gen_onoff_srv_t onoff_server = {
    .rsp_ctrl = {
        .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
        .set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    },
};

static void ble_mesh_generic_server_cb(
    esp_ble_mesh_generic_server_cb_event_t event,
    esp_ble_mesh_generic_server_cb_param_t *param)
{
    if (event != ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT) {
        return;
    }

    if (param->ctx.recv_op != ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET &&
        param->ctx.recv_op != ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) {
        return;
    }

    bool on = param->value.set.onoff.onoff;

    light_set(on);
    onoff_server.state.onoff = on ? 1 : 0;

    if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
        esp_ble_mesh_server_model_send_msg(
            param->model,
            &param->ctx,
            ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,
            sizeof(onoff_server.state.onoff),
            &onoff_server.state.onoff);
    }
}

#endif
#ifndef ENABLE_BLE_RELAY
#define ENABLE_BLE_RELAY 0
#endif
/* ================================================================
 * BLE Mesh model definitions
 * ================================================================ */
static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10 };

static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit     = ESP_BLE_MESH_TRANSMIT(2, 20),
#if ENABLE_BLE_RELAY
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
#else
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
#endif
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon           = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy       = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy       = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state     = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state     = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl      = 7,
};

static esp_ble_mesh_sensor_srv_t sensor_server = {
    .rsp_ctrl    = { .status_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP },
    .state_count = NUM_SENSORS,
    .states      = sensor_states,
};

static esp_ble_mesh_sensor_setup_srv_t sensor_setup_server = {
    .rsp_ctrl    = { .status_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP },
    .state_count = NUM_SENSORS,
    .states      = sensor_states,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_pub,       2 + MAX_SENSOR_PAYLOAD, ROLE_NODE);
ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_setup_pub, 2 + MAX_SENSOR_PAYLOAD, ROLE_NODE);

#if ENABLE_LIGHT
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub, 2 + 3, ROLE_NODE);
#endif

#if BASELINE_TEST
static void test_schedule_start(uint32_t start_in_ms);
static void test_stop(void);

#define TEST_VENDOR_MODEL_ID 0x1234
#define TEST_OP_CTRL        ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)
#define TEST_OP_ACK         ESP_BLE_MESH_MODEL_OP_3(0x02, CID_ESP)


#define TEST_CMD_ARM    1
#define TEST_CMD_START  2
#define TEST_CMD_STOP   3

static uint32_t get_le32(const uint8_t *p)
{
    return ((uint32_t)p[0])
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static uint16_t get_le16(const uint8_t *p)
{
    return ((uint16_t)p[0])
         | ((uint16_t)p[1] << 8);
}

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void send_test_ack(esp_ble_mesh_model_t *model,
                          esp_ble_mesh_msg_ctx_t *rx_ctx,
                          uint8_t cmd,
                          uint32_t run_id,
                          uint32_t scenario)
{
    if (!model || !rx_ctx) {
        ESP_LOGW(TAG, "TEST ACK skipped: model/ctx missing");
        return;
    }

    uint8_t buf[8];

    buf[0] = 0xA5;                       // magic
    buf[1] = cmd;                        // ARM/START/STOP
    put_le32(&buf[2], run_id);           // run_id
    buf[6] = (uint8_t)scenario;          // scenario
    buf[7] = (uint8_t)TEST_NODE_ID;      // sender node id

    esp_ble_mesh_msg_ctx_t ctx = {
        .net_idx  = rx_ctx->net_idx,
        .app_idx  = rx_ctx->app_idx,
        .addr     = rx_ctx->addr,        // reply to sender, usually gateway 0x0001
        .send_ttl = 3,
        .send_rel = false,
    };

    esp_err_t err = esp_ble_mesh_server_model_send_msg(
        model,
        &ctx,
        TEST_OP_ACK,
        sizeof(buf),
        buf);

    ESP_LOGI(TAG,
             "TEST ACK cmd=%u run=%lu scenario=%lu node=%d -> 0x%04x %s",
             cmd,
             (unsigned long)run_id,
             (unsigned long)scenario,
             TEST_NODE_ID,
             ctx.addr,
             err == ESP_OK ? "OK" : esp_err_to_name(err));
}

static uint32_t s_last_ack_run_id;
static uint8_t  s_last_ack_cmd;
static uint32_t s_last_ack_time_ms;

static bool should_send_test_ack(uint8_t cmd, uint32_t run_id)
{
    uint32_t now = now_ms();

    if (s_last_ack_run_id == run_id &&
        s_last_ack_cmd == cmd &&
        (now - s_last_ack_time_ms) < 3000) {
        return false;
    }

    s_last_ack_run_id = run_id;
    s_last_ack_cmd = cmd;
    s_last_ack_time_ms = now;
    return true;
}

static void test_handle_ctrl_msg(esp_ble_mesh_model_t *model,
                                 esp_ble_mesh_msg_ctx_t *rx_ctx,
                                 const uint8_t *data,
                                 uint16_t len)
{
    if (!data || len < 1) {
        return;
    }

    uint8_t cmd = data[0];

    uint32_t run_id      = 0;
    uint32_t scenario    = TEST_SCENARIO_NONE;
    uint32_t start_in_ms = 0;

    if (len >= 5) {
        run_id = get_le32(&data[1]);
    }

    if (len >= 6) {
        scenario = data[5];
    }

    if (len >= 8) {
        start_in_ms = get_le16(&data[6]);
    }

    ESP_LOGI(TAG,
             "TEST ctrl rx: cmd=%u run=%lu scenario=%lu start_in_ms=%lu len=%u",
             cmd,
             (unsigned long)run_id,
             (unsigned long)scenario,
             (unsigned long)start_in_ms,
             len);
    
    if (should_send_test_ack(cmd, run_id)) {
        send_test_ack(model, rx_ctx, cmd, run_id, scenario);
    }

    switch (cmd) {
    case TEST_CMD_ARM:
        s_test_run.armed     = true;
        s_test_run.running   = false;
        s_test_run.scheduled = false;
        s_test_run.run_id    = run_id;
        s_test_run.scenario  = scenario;
        s_test_run.start_ms  = 0;

        ESP_LOGI(TAG,
                 "TEST armed: node=%d run=%lu scenario=%lu boot_id=%lu",
                 TEST_NODE_ID,
                 (unsigned long)s_test_run.run_id,
                 (unsigned long)s_test_run.scenario,
                 (unsigned long)g_boot_id);
        break;

    case TEST_CMD_START:
        if (!s_test_run.armed || s_test_run.run_id != run_id) {
            ESP_LOGW(TAG,
                     "TEST start ignored: armed=%d stored_run=%lu requested_run=%lu",
                     s_test_run.armed,
                     (unsigned long)s_test_run.run_id,
                     (unsigned long)run_id);
            return;
        }

        test_schedule_start(start_in_ms);
        break;

    case TEST_CMD_STOP:
        test_stop();
        break;

    default:
        ESP_LOGW(TAG, "Unknown TEST cmd: %u", cmd);
        break;
    }
}

static esp_ble_mesh_model_op_t test_model_ops[] = {
    /* Keep min_len permissive.
     * The handler validates/parses the actual payload length itself.
     * A too-strict min_len can make ESP-IDF drop the vendor message before
     * ble_mesh_custom_model_cb() is called.
     */
    ESP_BLE_MESH_MODEL_OP(TEST_OP_CTRL, 1),
    ESP_BLE_MESH_MODEL_OP_END,
};

static void ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                     esp_ble_mesh_model_cb_param_t *param)
{
    if (event != ESP_BLE_MESH_MODEL_OPERATION_EVT) {
        return;
    }

    if (param->model_operation.opcode != TEST_OP_CTRL) {
        return;
    }

    test_handle_ctrl_msg(param->model_operation.model,
                         param->model_operation.ctx,
                         param->model_operation.msg,
                         param->model_operation.length);
}
#endif

static uint8_t health_test_ids[] = { 0x00 };

ESP_BLE_MESH_HEALTH_PUB_DEFINE(health_pub, 0, ROLE_NODE);

static esp_ble_mesh_health_srv_t health_server = {
    .health_test = {
        .id_count   = ARRAY_SIZE(health_test_ids),
        .test_ids   = health_test_ids,
        .company_id = CID_ESP,
    },
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_HEALTH_SRV(&health_server, &health_pub),

    ESP_BLE_MESH_MODEL_SENSOR_SRV(&sensor_pub, &sensor_server),
    ESP_BLE_MESH_MODEL_SENSOR_SETUP_SRV(&sensor_setup_pub, &sensor_setup_server),

#if ENABLE_LIGHT
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub, &onoff_server),
#endif
};

#if BASELINE_TEST
static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP,
                              TEST_VENDOR_MODEL_ID,
                              test_model_ops,
                              NULL,
                              NULL),
};
#endif

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(
        0,
        root_models,
#if BASELINE_TEST
        vnd_models
#else
        ESP_BLE_MESH_MODEL_NONE
#endif
    ),
};

static esp_ble_mesh_comp_t composition = {
    .cid           = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements      = elements,
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

/* ================================================================
 * Publication timer
 * ================================================================ */
static esp_timer_handle_t pub_timer = NULL;

#if ENABLE_SGP30 && !TEST_PROFILE
static uint16_t cached_eco2 = SENSOR_INVALID_U16;
static uint16_t cached_tvoc = SENSOR_INVALID_U16;

static void sgp30_task(void *arg)
{
    (void)arg;

    TickType_t next_wake = xTaskGetTickCount();

    while (1) {
        esp_err_t err = sgp30_read_values(&cached_eco2, &cached_tvoc);
        if (err != ESP_OK) {
            cached_eco2 = SENSOR_INVALID_U16;
            cached_tvoc = SENSOR_INVALID_U16;
        }

        vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(1000));
    }
}
#endif

static uint8_t periodic_phase_count(void)
{
    uint8_t n = 0;

#if ENABLE_AHT20
    n += 2;
#endif
#if ENABLE_MAX30101
    n += 4;
#endif
#if ENABLE_SGP30
    n += 2;
#endif
#if ENABLE_LIGHT
    n += 1;
#endif

    return (n > 0) ? n : 1;
}

static void append_periodic_phase_payload(uint8_t phase,
                                          uint8_t *payload,
                                          uint16_t *off,
                                          int16_t temp_x100,
                                          uint16_t humidity_x100,
                                          uint32_t raw_red,
                                          uint32_t raw_ir,
                                          uint8_t heart_rate,
                                          uint8_t spo2_val,
                                          uint16_t eco2,
                                          uint16_t tvoc)
{
    uint8_t idx = 0;

#if ENABLE_AHT20
    if (phase == idx++) {
        *off += marshal_s16(payload + *off, PROP_TEMPERATURE, temp_x100);
        return;
    }

    if (phase == idx++) {
        *off += marshal_u16(payload + *off, PROP_HUMIDITY, humidity_x100);
        return;
    }
#endif

#if ENABLE_MAX30101
    if (phase == idx++) {
        *off += marshal_u32(payload + *off, PROP_RAW_RED, raw_red);
        return;
    }

    if (phase == idx++) {
        *off += marshal_u32(payload + *off, PROP_RAW_IR, raw_ir);
        return;
    }

    if (phase == idx++) {
        *off += marshal_u8(payload + *off, PROP_HEART_RATE, heart_rate);
        return;
    }

    if (phase == idx++) {
        *off += marshal_u8(payload + *off, PROP_SPO2, spo2_val);
        return;
    }
#endif

#if ENABLE_SGP30
    if (phase == idx++) {
        *off += marshal_u16(payload + *off, PROP_ECO2, eco2);
        return;
    }

    if (phase == idx++) {
        *off += marshal_u16(payload + *off, PROP_TVOC, tvoc);
        return;
    }
#endif

#if ENABLE_LIGHT
    if (phase == idx++) {
        *off += marshal_u8(payload + *off, PROP_LIGHT_STATE, s_light_on ? 1 : 0);
        return;
    }
#endif
}

static void pub_timer_cb(void *arg)
{
    (void)arg;

    if (!s_sensor_model || !s_sensor_model->pub) {
        return;
    }

    if (mesh_quiet_active()) {
        return;
    }

    uint32_t seq = get_next_seq();

    int16_t  temp_x100     = SENSOR_INVALID_S16;
    uint16_t humidity_x100 = SENSOR_INVALID_U16;
    uint32_t raw_red       = SENSOR_INVALID_U32;
    uint32_t raw_ir        = SENSOR_INVALID_U32;
    uint8_t  heart_rate    = SENSOR_INVALID_U8;
    uint8_t  spo2_val      = SENSOR_INVALID_U8;
    uint16_t eco2          = SENSOR_INVALID_U16;
    uint16_t tvoc          = SENSOR_INVALID_U16;

#if ENABLE_AHT20
#  if TEST_PROFILE
    temp_x100     = test_ble_temp_x100();
    humidity_x100 = test_ble_hum_x100();
#  elif BASELINE_TEST
    temp_x100     = s_cached_temp_x100;
    humidity_x100 = s_cached_hum_x100;
#  else
    aht20_read_x100(&temp_x100, &humidity_x100);
#  endif

    net_buf_simple_reset(&raw_temp);
    net_buf_simple_add_le16(&raw_temp, (uint16_t)temp_x100);

    net_buf_simple_reset(&raw_hum);
    net_buf_simple_add_le16(&raw_hum, humidity_x100);
#endif

#if ENABLE_MAX30101
#  if BASELINE_TEST
    raw_red    = s_cached_raw_red;
    raw_ir     = s_cached_raw_ir;
    heart_rate = s_cached_hr;
    spo2_val   = s_cached_spo2;
#  else
    max30101_get_raw_values(&raw_red, &raw_ir);
    heart_rate = max30101_get_heart_rate();
    spo2_val   = max30101_get_spo2();
#  endif

    net_buf_simple_reset(&raw_red_buf);
    net_buf_simple_add_le32(&raw_red_buf, raw_red);

    net_buf_simple_reset(&raw_ir_buf);
    net_buf_simple_add_le32(&raw_ir_buf, raw_ir);

    net_buf_simple_reset(&raw_hr_buf);
    net_buf_simple_add_u8(&raw_hr_buf, heart_rate);

    net_buf_simple_reset(&raw_spo2_buf);
    net_buf_simple_add_u8(&raw_spo2_buf, spo2_val);
#endif

#if ENABLE_SGP30
#  if TEST_PROFILE
    eco2 = test_ble_eco2();
    tvoc = test_ble_tvoc();
#  else
    eco2 = cached_eco2;
    tvoc = cached_tvoc;
#  endif

    net_buf_simple_reset(&raw_eco2);
    net_buf_simple_add_le16(&raw_eco2, eco2);

    net_buf_simple_reset(&raw_tvoc);
    net_buf_simple_add_le16(&raw_tvoc, tvoc);
#endif

#if ENABLE_BUTTON
    net_buf_simple_reset(&raw_switch);
    net_buf_simple_add_u8(&raw_switch, 0);
#endif

#if ENABLE_LIGHT
    net_buf_simple_reset(&raw_light_state);
    net_buf_simple_add_u8(&raw_light_state, s_light_on ? 1 : 0);
#endif

    net_buf_simple_reset(&raw_seq_buf);
    net_buf_simple_add_le32(&raw_seq_buf, seq);

    uint8_t payload[MAX_SENSOR_PAYLOAD];
    uint16_t off = 0;

    /* Immer SEQ + genau ein Messwert. Keine TX_MS, kein BOOT_ID, kein Test-Node. */
    off += marshal_u32(payload + off, PROP_SEQ, seq);

    uint16_t base_off = off;
    uint8_t phase = s_pub_phase++ % periodic_phase_count();

    append_periodic_phase_payload(phase,
                                  payload,
                                  &off,
                                  temp_x100,
                                  humidity_x100,
                                  raw_red,
                                  raw_ir,
                                  heart_rate,
                                  spo2_val,
                                  eco2,
                                  tvoc);

    if (off == base_off) {
        return;
    }

    if (off > 11) {
        ESP_LOGE(TAG, "BUG: BLE Mesh publish would be segmented: %u bytes", off);
        return;
    }

    esp_err_t err = esp_ble_mesh_model_publish(
        s_sensor_model,
        ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS,
        off,
        payload,
        ROLE_NODE);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Publish failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Published seq=%lu phase=%u (%u bytes)",
                 (unsigned long)seq,
                 phase,
                 off);
    }
}

#if BASELINE_TEST
static esp_timer_handle_t pub_start_timer = NULL;

static void pub_start_cb(void *arg)
{
    (void)arg;

    s_test_run.running   = true;
    s_test_run.scheduled = false;
    s_pub_phase = TEST_INITIAL_PUB_PHASE;

    pub_timer_cb(NULL);

    esp_err_t timer_err = esp_timer_start_periodic(pub_timer,
                                                   SENSOR_PUBLISH_INTERVAL_US);
    if (timer_err != ESP_OK) {
        ESP_LOGW(TAG, "TEST PUB periodic start failed: %s",
                 esp_err_to_name(timer_err));
        return;
    }

    ESP_LOGI(TAG,
             "TEST PUB started: node=%d run=%lu scenario=%lu offset=%dms interval=%dms initial_phase=%d",
             TEST_NODE_ID,
             (unsigned long)s_test_run.run_id,
             (unsigned long)s_test_run.scenario,
             TEST_TX_OFFSET_MS,
             SENSOR_PUBLISH_INTERVAL_MS,
             TEST_INITIAL_PUB_PHASE);
}

static void test_schedule_start(uint32_t start_in_ms)
{
    if (!s_test_run.armed) {
        ESP_LOGW(TAG, "TEST start ignored: not armed");
        return;
    }

    if (s_test_run.scenario == TEST_SCENARIO_NONE) {
        ESP_LOGW(TAG, "TEST start ignored: scenario=NONE");
        return;
    }

    if (start_in_ms < 1000) {
        ESP_LOGW(TAG, "TEST start ignored: start_in_ms too small");
        return;
    }

    if (!pub_timer) {
        ESP_LOGW(TAG, "TEST start ignored: pub_timer not ready");
        return;
    }

    if (s_test_run.running) {
        ESP_LOGI(TAG,
                 "TEST start duplicate ignored: already running node=%d run=%lu",
                 TEST_NODE_ID,
                 (unsigned long)s_test_run.run_id);
        return;
    }

    if (s_test_run.scheduled) {
        ESP_LOGI(TAG,
                 "TEST start duplicate ignored: already scheduled node=%d run=%lu",
                 TEST_NODE_ID,
                 (unsigned long)s_test_run.run_id);
        return;
    }

    if (pub_start_timer == NULL) {
        const esp_timer_create_args_t start_args = {
            .callback = &pub_start_cb,
            .name = "pub_start",
        };

        ESP_ERROR_CHECK(esp_timer_create(&start_args, &pub_start_timer));
    }

    esp_timer_stop(pub_timer);
    esp_timer_stop(pub_start_timer);

    s_test_run.running   = false;
    s_test_run.scheduled = true;
    s_test_run.start_ms  = now_ms() + start_in_ms;

    uint32_t delay_ms = start_in_ms + TEST_TX_OFFSET_MS;

    ESP_ERROR_CHECK(esp_timer_start_once(pub_start_timer,
                                         (uint64_t)delay_ms * 1000ULL));

    ESP_LOGI(TAG,
             "TEST scheduled: node=%d run=%lu scenario=%lu starts_in=%lums offset=%dms",
             TEST_NODE_ID,
             (unsigned long)s_test_run.run_id,
             (unsigned long)s_test_run.scenario,
             (unsigned long)delay_ms,
             TEST_TX_OFFSET_MS);
}

static void test_stop(void)
{
    if (pub_timer) {
        esp_timer_stop(pub_timer);
    }

    if (pub_start_timer) {
        esp_timer_stop(pub_start_timer);
    }

    s_test_run.running   = false;
    s_test_run.armed     = false;
    s_test_run.scheduled = false;

    ESP_LOGI(TAG,
             "TEST stopped: node=%d run=%lu",
             TEST_NODE_ID,
             (unsigned long)s_test_run.run_id);
}
#endif

/* ================================================================
 * BLE Mesh callbacks
 * ================================================================ */
static void prov_complete(uint16_t net_idx,
                          uint16_t addr,
                          uint8_t flags,
                          uint32_t iv_index)
{
    (void)flags;
    (void)iv_index;

    ESP_LOGI(TAG, "Provisioned: net_idx=0x%04x addr=0x%04x", net_idx, addr);
    board_led_operation(LED_GREEN, LED_OFF);
}

static void prov_led_timeout_cb(void *arg)
{
    (void)arg;
    board_led_operation(LED_GREEN, LED_OFF);
}

static esp_timer_handle_t prov_led_timer = NULL;

static void ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                     esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "PROV_REGISTER_COMP err=%d",
                 param->prov_register_comp.err_code);
        break;

    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "PROV_ENABLE_COMP err=%d",
                 param->node_prov_enable_comp.err_code);
        break;

    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        prov_complete(param->node_prov_complete.net_idx,
                      param->node_prov_complete.addr,
                      param->node_prov_complete.flags,
                      param->node_prov_complete.iv_index);

        s_sensor_model = sensor_server.model;

#if ENABLE_BUTTON
        s_sensor_model_btn = sensor_server.model;
#endif

#if BASELINE_TEST
        if (s_sensor_read_timer == NULL) {
            const esp_timer_create_args_t sr_args = {
                .callback = sensor_read_cb,
                .name = "sens_read",
            };

            ESP_ERROR_CHECK(esp_timer_create(&sr_args, &s_sensor_read_timer));
            ESP_ERROR_CHECK(esp_timer_start_periodic(s_sensor_read_timer,
                                                     SENSOR_PUBLISH_INTERVAL_US));
        }
#endif

        if (pub_timer == NULL) {
            const esp_timer_create_args_t timer_args = {
                .callback = &pub_timer_cb,
                .name = "sensor_pub",
            };
            ESP_ERROR_CHECK(esp_timer_create(&timer_args, &pub_timer));
        }

#if BASELINE_TEST
        s_pub_phase = TEST_INITIAL_PUB_PHASE;

        ESP_LOGI(TAG,
                 "BASELINE_TEST ready: node=%d offset=%dms interval=%ds boot_id=%lu; waiting for BLE test arm/start",
                 TEST_NODE_ID,
                 TEST_TX_OFFSET_MS,
                 SENSOR_PUBLISH_INTERVAL_MS,
                 (unsigned long)g_boot_id);
#else
        ESP_ERROR_CHECK(esp_timer_start_periodic(pub_timer,
                                                 SENSOR_PUBLISH_INTERVAL_US));

        ESP_LOGI(TAG, "Publication timer started (%d ms)",
                 SENSOR_PUBLISH_INTERVAL_MS);
#endif
        break;

    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "PROV_RESET");
        break;

    default:
        break;
    }
}

static void ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                      esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event != ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        return;
    }

    switch (param->ctx.recv_op) {
    case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
        ESP_LOGI(TAG, "AppKey added");
        break;

    case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
        ESP_LOGI(TAG, "Model AppKey bound");
        break;

    case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
        ESP_LOGI(TAG, "Model subscription added");
        break;

    default:
        break;
    }
}

static void ble_mesh_sensor_server_cb(esp_ble_mesh_sensor_server_cb_event_t event,
                                      esp_ble_mesh_sensor_server_cb_param_t *param)
{
    ESP_LOGD(TAG,
             "Sensor event %d opcode 0x%04" PRIx32,
             event,
             param->ctx.recv_op);
}

/* ================================================================
 * BLE Mesh init
 * ================================================================ */
static esp_err_t ble_mesh_init(void)
{
    esp_ble_mesh_register_prov_callback(ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(ble_mesh_config_server_cb);
    esp_ble_mesh_register_sensor_server_callback(ble_mesh_sensor_server_cb);

#if BASELINE_TEST
    esp_ble_mesh_register_custom_model_callback(ble_mesh_custom_model_cb);
#endif

#if ENABLE_LIGHT
    esp_ble_mesh_register_generic_server_callback(ble_mesh_generic_server_cb);
#endif

    esp_err_t err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mesh init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_ble_mesh_set_unprovisioned_device_name(BLE_MESH_DEVICE_NAME);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Set name failed: %s", esp_err_to_name(err));
    }

    err = esp_ble_mesh_node_prov_enable(
        (esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV |
                                     ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Enable prov failed: %s", esp_err_to_name(err));
        return err;
    }

    board_led_operation(LED_GREEN, LED_ON);

    const esp_timer_create_args_t led_args = {
        .callback = prov_led_timeout_cb,
        .name = "prov_led",
    };

    if (esp_timer_create(&led_args, &prov_led_timer) == ESP_OK) {
        esp_timer_start_once(prov_led_timer,
                             (uint64_t)PROV_LED_TIMEOUT_S * 1000ULL * 1000ULL);
    }

    ESP_LOGI(TAG, "BLE Mesh ready — waiting for provisioning");
    return ESP_OK;
}

/* ================================================================
 * app_main
 * ================================================================ */
void app_main(void)
{
#if BASELINE_TEST
    ESP_LOGI(TAG, "=== BLE Mesh Sensor Server [BASELINE_TEST STABLE] ===");
#else
    ESP_LOGI(TAG, "=== BLE Mesh Sensor Server [STABLE BLE MODE] ===");
#endif

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    boot_state_load();

    esp_pm_config_t pm_config = {
        .max_freq_mhz       = PM_MAX_CPU_FREQ_MHZ,
        .min_freq_mhz       = PM_MIN_CPU_FREQ_MHZ,
        .light_sleep_enable = false,
    };

    if (esp_pm_configure(&pm_config) != ESP_OK) {
        ESP_LOGW(TAG, "PM configure failed — no light sleep");
    }

    board_init();

#if ENABLE_AHT20
    ESP_LOGI(TAG, "Init AHT20");
    ESP_ERROR_CHECK(aht20_init());
#endif

#if ENABLE_MAX30101
    ESP_LOGI(TAG, "Init MAX30101");
    ESP_ERROR_CHECK(max30101_init());
    ESP_ERROR_CHECK(max30101_start_sampling());
#endif

#if ENABLE_SGP30 && !TEST_PROFILE
    ESP_LOGI(TAG, "Init SGP30");
    ESP_ERROR_CHECK(sgp30_init());

    BaseType_t sgp_ok = xTaskCreate(sgp30_task,
                                    "sgp30",
                                    3072,
                                    NULL,
                                    3,
                                    NULL);
    configASSERT(sgp_ok == pdPASS);
#endif

#if ENABLE_BUTTON
    button_hw_init();

    BaseType_t ok = xTaskCreate(button_task,
                                "button",
                                3072,
                                NULL,
                                8,
                                &s_btn_task_handle);
    configASSERT(ok == pdPASS);
    configASSERT(s_btn_task_handle != NULL);

    s_btn_irq_blocked = false;
    gpio_intr_enable(BUTTON_GPIO_PIN);

    ESP_LOGI(TAG, "Button ready GPIO%d", BUTTON_GPIO_PIN);
#endif

#if ENABLE_LIGHT
    led_hw_init();
    led_set_rgb(0, 0, 0);
    ESP_LOGI(TAG, "Light ready GPIO%d", LIGHT_GPIO_PIN);
#endif

    ESP_ERROR_CHECK(bluetooth_init());
    esp_ble_gap_set_device_name(BLE_MESH_DEVICE_NAME);

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Gateway-Erkennungspräfix erzwingen. */
    dev_uuid[0] = 0x32;
    dev_uuid[1] = 0x10;

    ESP_LOGI(TAG,
             "FINAL BLE Mesh UUID: %02X %02X %02X %02X ...",
             dev_uuid[0],
             dev_uuid[1],
             dev_uuid[2],
             dev_uuid[3]);

    ESP_ERROR_CHECK(ble_mesh_init());

    ESP_LOGI(TAG, "=== All subsystems initialized ===");
}
