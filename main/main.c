/* main.c - BLE Mesh SIG Sensor Server + Button + Light
 *
 * Aktivierbare Features via build_flags:
 *
 *   -DENABLE_AHT20=1        Temperatur + Luftfeuchtigkeit
 *   -DENABLE_MAX30101=1     Herzrate + SpO2
 *   -DENABLE_SGP30=1        eCO2 + TVOC
 *   -DENABLE_BUTTON=1       Taster (GPIO21 default, -DBUTTON_GPIO_PIN=X)
 *   -DENABLE_LIGHT=1        WS2812 LED + Generic OnOff Server Model
 *   -DLIGHT_FEEDBACK=1      Publish nach jedem ON/OFF mit PROP_LIGHT_STATE
 *
 * ───────────────────────────────────────────────────────────────
 *  BASELINE_TEST mode (-DBASELINE_TEST=1)
 * ───────────────────────────────────────────────────────────────
 *  Deterministischer Messmodus für Thesis-Reliability-Tests:
 *
 *   • Strikt konstantes Publish-Intervall (esp_timer, phase-locked)
 *   • Seq-Counter NVS-persistiert → Reboots bleiben nachvollziehbar
 *   • BOOT-Marker beim Provisioning-Abschluss
 *   • BUTTON_EVENT_RETX_COUNT=1 (kein Retry-Smoothing, misst echten
 *     Layer-2-Verlust statt Retry-Kompensation)
 *   • PROP_TX_MS und PROP_BOOT_ID im Payload → Gateway kann
 *     Latenz-Deltas und Reboot-Grenzen in der Messung erkennen
 *   • Command-ACK via LIGHT_FEEDBACK mit RTT-Info
 *
 * Thesis-Hinweis: Der Button-Retry ist ein echter Feature-Teil des
 * Normalbetriebs (BLE Mesh hat kein transportlevel ACK → Retry ist
 * der Standardweg). Für die Messung wird er bewusst deaktiviert um
 * die darunter liegende Layer-2-Reliability isoliert zu zeigen. Der
 * Unterschied zwischen beiden Modi ist selbst ein guter Datenpunkt.
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <stdatomic.h>

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
 * NVS-Persistenz für boot_id + seq high-water mark
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
    /* +16 Safety-Abstand für ungespeicherte Pakete vor Reboot */
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
    /* Alle 32 Pakete in NVS sichern */
    if ((seq & 0x1F) != 0) return;
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
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

/* BASELINE_TEST zusätzliche Props */
#define PROP_TX_MS        0x0110  /**< Node-Uptime beim Senden (u32 ms) */
#define PROP_BOOT_ID      0x0111  /**< Boot-Counter (u32) */
#define PROP_CMD_RTT      0x0112  /**< Last cmd RTT in ms (u16) */

/* ── Publish-Intervall ──────────────────────────────────────────
 * Override via -DSENSOR_PUBLISH_INTERVAL_S=<s>. BASELINE_TEST
 * setzt auf 1s für 1Hz-Grid. */
#if BASELINE_TEST
#  ifdef SENSOR_PUBLISH_INTERVAL_S
#    undef SENSOR_PUBLISH_INTERVAL_S
#  endif
#  define SENSOR_PUBLISH_INTERVAL_S 1
#endif

#define SENSOR_PUBLISH_INTERVAL_US  ((uint64_t)SENSOR_PUBLISH_INTERVAL_S * 1000 * 1000)
#define SGP30_MEASUREMENT_INTERVAL_US (1 * 1000 * 1000)

#define SENSOR_INVALID_U8   0xFF
#define SENSOR_INVALID_U16  0xFFFF
#define SENSOR_INVALID_U32  0xFFFFFFFF
#define SENSOR_INVALID_S16  ((int16_t)0x8000)
#define MAX_SENSOR_PAYLOAD  80  /* erhöht für BASELINE-Props */

/* ── Button-Retry ──────────────────────────────────────────────
 * Normal: 4× mit 100ms Abstand (Retry-Smoothing).
 * BASELINE_TEST: 1× um echte Layer-2-Reliability zu messen. */
#if BASELINE_TEST
#  define BUTTON_EVENT_RETX_COUNT        1
#  define BUTTON_EVENT_RETX_INTERVAL_MS  0
#else
#  define BUTTON_EVENT_RETX_COUNT        4
#  define BUTTON_EVENT_RETX_INTERVAL_MS  100
#endif

/* ================================================================
 * Marshalling Helpers
 * ================================================================ */

static uint16_t marshal_format_a(uint8_t *buf, uint16_t property_id,
                                  const uint8_t *data, uint8_t data_len)
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
{ return marshal_format_a(buf, prop, &val, 1); }

__attribute__((unused))
static uint16_t marshal_u16(uint8_t *buf, uint16_t prop, uint16_t val)
{
    uint8_t raw[2] = { (uint8_t)(val), (uint8_t)(val >> 8) };
    return marshal_format_a(buf, prop, raw, 2);
}

__attribute__((unused))
static uint16_t marshal_s16(uint8_t *buf, uint16_t prop, int16_t val)
{
    uint8_t raw[2] = { (uint8_t)(val), (uint8_t)((val >> 8) & 0xFF) };
    return marshal_format_a(buf, prop, raw, 2);
}

__attribute__((unused))
static uint16_t marshal_u32(uint8_t *buf, uint16_t prop, uint32_t val)
{
    uint8_t raw[4] = {
        (uint8_t)(val), (uint8_t)(val>>8),
        (uint8_t)(val>>16), (uint8_t)(val>>24),
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
 * Sensor States
 * ================================================================ */

#define TEMP_SENSORS  ((ENABLE_AHT20)    ? 2 : 0)
#define MAX_SENSORS   ((ENABLE_MAX30101) ? 4 : 0)
#define SGP_SENSORS   ((ENABLE_SGP30)    ? 2 : 0)
#define BTN_SENSORS   ((ENABLE_BUTTON)   ? 1 : 0)
#define LGT_SENSORS   ((ENABLE_LIGHT)    ? 1 : 0)
#define SEQ_SENSORS   1
#define NUM_SENSORS   (TEMP_SENSORS + MAX_SENSORS + SGP_SENSORS + \
                       BTN_SENSORS + LGT_SENSORS + SEQ_SENSORS)

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
    { .sensor_property_id = PROP_TEMPERATURE,
      .descriptor = { .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
                      .measure_period = 0x14, .update_interval = 0x14 },
      .sensor_data = { .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
                       .length = 2-1, .raw_value = &raw_temp } },
    { .sensor_property_id = PROP_HUMIDITY,
      .descriptor = { .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
                      .measure_period = 0x14, .update_interval = 0x14 },
      .sensor_data = { .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
                       .length = 2-1, .raw_value = &raw_hum } },
#endif
#if ENABLE_MAX30101
    { .sensor_property_id = PROP_RAW_RED,
      .sensor_data = { .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
                       .length = 4-1, .raw_value = &raw_red_buf } },
    { .sensor_property_id = PROP_RAW_IR,
      .sensor_data = { .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
                       .length = 4-1, .raw_value = &raw_ir_buf } },
    { .sensor_property_id = PROP_HEART_RATE,
      .sensor_data = { .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
                       .length = 1-1, .raw_value = &raw_hr_buf } },
    { .sensor_property_id = PROP_SPO2,
      .sensor_data = { .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
                       .length = 1-1, .raw_value = &raw_spo2_buf } },
#endif
#if ENABLE_SGP30
    { .sensor_property_id = PROP_ECO2,
      .sensor_data = { .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
                       .length = 2-1, .raw_value = &raw_eco2 } },
    { .sensor_property_id = PROP_TVOC,
      .sensor_data = { .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
                       .length = 2-1, .raw_value = &raw_tvoc } },
#endif
#if ENABLE_BUTTON
    { .sensor_property_id = PROP_SWITCH,
      .descriptor = { .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
                      .measure_period = 0, .update_interval = 0 },
      .sensor_data = { .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
                       .length = 1-1, .raw_value = &raw_switch } },
#endif
#if ENABLE_LIGHT
    { .sensor_property_id = PROP_LIGHT_STATE,
      .descriptor = { .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
                      .measure_period = 0, .update_interval = 0 },
      .sensor_data = { .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
                       .length = 1-1, .raw_value = &raw_light_state } },
#endif
    { .sensor_property_id = PROP_SEQ,
      .descriptor = { .sampling_function = ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED,
                      .measure_period = 0x14, .update_interval = 0x14 },
      .sensor_data = { .format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
                       .length = 4-1, .raw_value = &raw_seq_buf } },
};

/* ================================================================
 * Sensor-Cache  (BASELINE_TEST: entkoppelt von Sendepfad)
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
#if ENABLE_AHT20
    int16_t t; uint16_t h;
    aht20_read_x100(&t, &h);
    s_cached_temp_x100 = t;
    s_cached_hum_x100  = h;
#endif
#if ENABLE_MAX30101
    max30101_get_raw_values(&s_cached_raw_red, &s_cached_raw_ir);
    s_cached_hr   = max30101_get_heart_rate();
    s_cached_spo2 = max30101_get_spo2();
#endif
    /* SGP30 hat bereits seinen eigenen 1s-Timer (sgp30_timer_cb),
     * nicht doppelt. */
}

#endif /* BASELINE_TEST */

/* ================================================================
 * WS2812 RGB LED  (ENABLE_LIGHT)
 * ================================================================ */
#if ENABLE_LIGHT

static rmt_channel_handle_t s_rmt_chan = NULL;
static rmt_encoder_handle_t s_rmt_enc  = NULL;
static bool s_light_on = false;
static esp_ble_mesh_model_t *s_sensor_model_ptr = NULL;

/* Command-Tracking (BASELINE_TEST) */
#if BASELINE_TEST
static uint32_t s_last_cmd_rx_ms   = 0;
static uint32_t s_last_cmd_done_ms = 0;
#endif

static void led_hw_init(void)
{
    rmt_tx_channel_config_t cfg = {
        .gpio_num          = LIGHT_GPIO_PIN,
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = 10000000,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    rmt_new_tx_channel(&cfg, &s_rmt_chan);
    rmt_bytes_encoder_config_t enc = {
        .bit0 = { .level0=1,.duration0=4,.level1=0,.duration1=8 },
        .bit1 = { .level0=1,.duration0=8,.level1=0,.duration1=4 },
        .flags.msb_first = 1,
    };
    rmt_new_bytes_encoder(&enc, &s_rmt_enc);
    rmt_enable(s_rmt_chan);
    ESP_LOGI(TAG, "WS2812 init GPIO%d", LIGHT_GPIO_PIN);
}

static void led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t grb[3] = { g, r, b };
    rmt_transmit_config_t tx = { .loop_count = 0 };
    rmt_transmit(s_rmt_chan, s_rmt_enc, grb, 3, &tx);
    rmt_tx_wait_all_done(s_rmt_chan, 100);
}

static void light_publish_state(void)
{
#if LIGHT_FEEDBACK
    if (!s_sensor_model_ptr) return;

    net_buf_simple_reset(&raw_light_state);
    net_buf_simple_add_u8(&raw_light_state, s_light_on ? 1 : 0);

    uint8_t payload[32];
    uint16_t len = 0;
    len += marshal_u8(payload + len, PROP_LIGHT_STATE, s_light_on ? 1 : 0);

#if BASELINE_TEST
    /* Command-Timing anhängen: Gateway kann RTT messen */
    uint16_t rtt_ms = (s_last_cmd_done_ms > s_last_cmd_rx_ms)
                    ? (uint16_t)(s_last_cmd_done_ms - s_last_cmd_rx_ms)
                    : 0;
    len += marshal_u16(payload + len, PROP_CMD_RTT, rtt_ms);
    len += marshal_u32(payload + len, PROP_BOOT_ID, g_boot_id);
    len += marshal_u32(payload + len, PROP_TX_MS,
                       (uint32_t)(esp_timer_get_time() / 1000ULL));
#endif

    esp_err_t err = esp_ble_mesh_model_publish(
        s_sensor_model_ptr,
        ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS,
        len, payload, ROLE_NODE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Light state publish failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Light state published: %s", s_light_on ? "ON" : "OFF");
    }
#endif
}

static void light_set(bool on)
{
#if BASELINE_TEST
    s_last_cmd_rx_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
#endif
    s_light_on = on;
    if (on) led_set_rgb(10, 10, 10);
    else    led_set_rgb(0,  0,  0);
#if BASELINE_TEST
    s_last_cmd_done_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
#endif
    ESP_LOGI(TAG, "Light %s", on ? "ON" : "OFF");
    light_publish_state();
}

#endif /* ENABLE_LIGHT */

/* ================================================================
 * Button  (ENABLE_BUTTON)
 * ================================================================ */
#if ENABLE_BUTTON

static TaskHandle_t s_btn_task_handle = NULL;
static volatile bool s_btn_irq_blocked = false;
static esp_ble_mesh_model_t *s_sensor_model_btn = NULL;

static void IRAM_ATTR btn_isr_handler(void *arg)
{
    if (s_btn_irq_blocked) return;
    if (s_btn_task_handle == NULL) return;
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

    uint8_t payload[32];
    uint16_t len = 0;
    len += marshal_u32(payload + len, PROP_SEQ, event_seq);
    len += marshal_u8(payload + len, PROP_SWITCH, 1);
#if BASELINE_TEST
    len += marshal_u32(payload + len, PROP_TX_MS,
                       (uint32_t)(esp_timer_get_time() / 1000ULL));
    len += marshal_u32(payload + len, PROP_BOOT_ID, g_boot_id);
#endif

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
        payload
    );

    ESP_LOGI(TAG, "Button TX seq=%lu -> 0x%04X %s",
             (unsigned long)event_seq,
             GATEWAY_MESH_ADDR,
             err == ESP_OK ? "OK" : esp_err_to_name(err));
}
static void button_task(void *arg)
{
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

        for (int i = 0; i < BUTTON_EVENT_RETX_COUNT; i++) {
            button_send_once(event_seq);

            if (i + 1 < BUTTON_EVENT_RETX_COUNT) {
                vTaskDelay(pdMS_TO_TICKS(BUTTON_EVENT_RETX_INTERVAL_MS));
            }
        }
    }
}

#endif /* ENABLE_BUTTON */

/* ================================================================
 * Generic OnOff Server  (ENABLE_LIGHT)
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
    if (event != ESP_BLE_MESH_GENERIC_SERVER_RECV_SET_MSG_EVT) return;
    if (param->ctx.recv_op != ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET &&
        param->ctx.recv_op != ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK) return;

    bool on = param->value.set.onoff.onoff;
    light_set(on);

    if (param->ctx.recv_op == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
        onoff_server.state.onoff = on ? 1 : 0;
        esp_ble_mesh_server_model_send_msg(
            param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_STATUS,
            sizeof(onoff_server.state.onoff),
            &onoff_server.state.onoff);
    }
}

#endif /* ENABLE_LIGHT */

/* ================================================================
 * BLE Mesh Model Definitions
 * ================================================================ */

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10 };

static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit     = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay            = ESP_BLE_MESH_RELAY_ENABLED,
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
    .default_ttl = 7,
};

static esp_ble_mesh_sensor_srv_t sensor_server = {
    .rsp_ctrl    = { .status_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP },
    .state_count = NUM_SENSORS,
    .states      = sensor_states,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_pub, 2 + MAX_SENSOR_PAYLOAD, ROLE_NODE);

#if ENABLE_LIGHT
ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub, 2 + 3, ROLE_NODE);
#endif

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_SENSOR_SRV(&sensor_pub, &sensor_server),
#if ENABLE_LIGHT
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub, &onoff_server),
#endif
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid           = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements      = elements,
};

static esp_ble_mesh_prov_t provision = { .uuid = dev_uuid };

/* ================================================================
 * Publication Timer
 * ================================================================ */

static esp_timer_handle_t pub_timer;
static esp_ble_mesh_model_t *s_sensor_model;

#if ENABLE_SGP30
static esp_timer_handle_t sgp30_timer;
static uint16_t cached_eco2 = SENSOR_INVALID_U16;
static uint16_t cached_tvoc = SENSOR_INVALID_U16;

static void sgp30_timer_cb(void *arg)
{
    esp_err_t err = sgp30_read_values(&cached_eco2, &cached_tvoc);
    if (err != ESP_OK) {
        cached_eco2 = SENSOR_INVALID_U16;
        cached_tvoc = SENSOR_INVALID_U16;
    }
}
#endif

#if BASELINE_TEST
/* Boot-Marker als spezielles Paket mit seq=0 und boot=1 flag */
static bool s_boot_marker_sent = false;

static void send_boot_marker(void)
{
    if (!s_sensor_model || !s_sensor_model->pub) return;

    uint8_t payload[64];
    uint16_t off = 0;

    /* Marker: boot=1 via PROP_SWITCH=0xFF (sentinel).
     * Gateway erkennt Kombination (boot_id + switch=0xFF) als Boot-Event. */
    off += marshal_u32(payload + off, PROP_SEQ, 0);
    off += marshal_u32(payload + off, PROP_BOOT_ID, g_boot_id);
    off += marshal_u32(payload + off, PROP_TX_MS,
                       (uint32_t)(esp_timer_get_time() / 1000ULL));

    esp_reset_reason_t reason = esp_reset_reason();
    off += marshal_u8(payload + off, PROP_SWITCH, 0xFF);  /* sentinel */
    off += marshal_u8(payload + off, 0x0113 /* PROP_RESET_REASON */,
                      (uint8_t)reason);

    esp_err_t err = esp_ble_mesh_model_publish(
        s_sensor_model,
        ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS,
        off, payload, ROLE_NODE);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "BOOT marker sent: boot_id=%lu reason=%d",
                 (unsigned long)g_boot_id, (int)reason);
        s_boot_marker_sent = true;
    } else {
        ESP_LOGW(TAG, "BOOT marker send failed: %s", esp_err_to_name(err));
    }
}
#endif /* BASELINE_TEST */

static void pub_timer_cb(void *arg)
{
    uint32_t seq = get_next_seq();
    if (!s_sensor_model || !s_sensor_model->pub) return;

#if BASELINE_TEST
    /* Beim ersten Tick: BOOT-Marker einschieben */
    if (!s_boot_marker_sent) {
        send_boot_marker();
    }
#endif

    int16_t  temp_x100     = SENSOR_INVALID_S16;
    uint16_t humidity_x100 = SENSOR_INVALID_U16;
    uint32_t raw_red       = SENSOR_INVALID_U32;
    uint32_t raw_ir        = SENSOR_INVALID_U32;
    uint8_t  heart_rate    = SENSOR_INVALID_U8;
    uint8_t  spo2_val      = SENSOR_INVALID_U8;
    uint16_t eco2          = SENSOR_INVALID_U16;
    uint16_t tvoc          = SENSOR_INVALID_U16;

#if ENABLE_AHT20
#  if BASELINE_TEST
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
    net_buf_simple_reset(&raw_red_buf); net_buf_simple_add_le32(&raw_red_buf, raw_red);
    net_buf_simple_reset(&raw_ir_buf);  net_buf_simple_add_le32(&raw_ir_buf, raw_ir);
    net_buf_simple_reset(&raw_hr_buf);  net_buf_simple_add_u8(&raw_hr_buf, heart_rate);
    net_buf_simple_reset(&raw_spo2_buf);net_buf_simple_add_u8(&raw_spo2_buf, spo2_val);
#endif

#if ENABLE_SGP30
    eco2 = cached_eco2; tvoc = cached_tvoc;
    net_buf_simple_reset(&raw_eco2); net_buf_simple_add_le16(&raw_eco2, eco2);
    net_buf_simple_reset(&raw_tvoc); net_buf_simple_add_le16(&raw_tvoc, tvoc);
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

    /* Payload zusammenbauen */
    uint8_t payload[MAX_SENSOR_PAYLOAD];
    uint16_t off = 0;
    off += marshal_u32(payload + off, PROP_SEQ, seq);
#if BASELINE_TEST
    off += marshal_u32(payload + off, PROP_TX_MS,
                       (uint32_t)(esp_timer_get_time() / 1000ULL));
    off += marshal_u32(payload + off, PROP_BOOT_ID, g_boot_id);
#endif
#if ENABLE_AHT20
    off += marshal_s16(payload + off, PROP_TEMPERATURE, temp_x100);
    off += marshal_u16(payload + off, PROP_HUMIDITY,    humidity_x100);
#endif
#if ENABLE_MAX30101
    off += marshal_u32(payload + off, PROP_RAW_RED,    raw_red);
    off += marshal_u32(payload + off, PROP_RAW_IR,     raw_ir);
    off += marshal_u8(payload  + off, PROP_HEART_RATE, heart_rate);
    off += marshal_u8(payload  + off, PROP_SPO2,       spo2_val);
#endif
#if ENABLE_SGP30
    off += marshal_u16(payload + off, PROP_ECO2, eco2);
    off += marshal_u16(payload + off, PROP_TVOC, tvoc);
#endif
#if ENABLE_LIGHT
    off += marshal_u8(payload + off, PROP_LIGHT_STATE, s_light_on ? 1 : 0);
#endif

    esp_err_t err = esp_ble_mesh_model_publish(
        s_sensor_model,
        ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS,
        off, payload, ROLE_NODE);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Publish failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Published seq=%lu (%u bytes)", (unsigned long)seq, off);
    }

    (void)temp_x100; (void)humidity_x100;
    (void)raw_red; (void)raw_ir; (void)heart_rate; (void)spo2_val;
    (void)eco2; (void)tvoc;
}

/* ================================================================
 * BLE Mesh Callbacks
 * ================================================================ */

static void prov_complete(uint16_t net_idx, uint16_t addr,
                          uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "Provisioned: net_idx=0x%04x addr=0x%04x", net_idx, addr);
    board_led_operation(LED_GREEN, LED_OFF);
}

static void prov_led_timeout_cb(void *arg)
{
    board_led_operation(LED_GREEN, LED_OFF);
}

static esp_timer_handle_t prov_led_timer;

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

        s_sensor_model = &root_models[1];

#if ENABLE_BUTTON
        s_sensor_model_btn = &root_models[1];
#endif
#if ENABLE_LIGHT
        s_sensor_model_ptr = &root_models[1];
#endif

#if BASELINE_TEST
        /* Sensor-Read-Timer (entkoppelt vom Publish) */
        {
            const esp_timer_create_args_t sr_args = {
                .callback = sensor_read_cb, .name = "sens_read",
            };
            esp_timer_create(&sr_args, &s_sensor_read_timer);
            esp_timer_start_periodic(s_sensor_read_timer,
                                      SENSOR_PUBLISH_INTERVAL_US);
        }
#endif

        /* Publikations-Timer: phase-locked via esp_timer */
        {
            const esp_timer_create_args_t timer_args = {
                .callback = &pub_timer_cb, .name = "sensor_pub"
            };
            ESP_ERROR_CHECK(esp_timer_create(&timer_args, &pub_timer));
            ESP_ERROR_CHECK(esp_timer_start_periodic(pub_timer,
                                                      SENSOR_PUBLISH_INTERVAL_US));
        }
        ESP_LOGI(TAG, "Publication timer started (%d ms)",
                 SENSOR_PUBLISH_INTERVAL_S * 1000);
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
    if (event != ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) return;
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
    default: break;
    }
}

static void ble_mesh_sensor_server_cb(esp_ble_mesh_sensor_server_cb_event_t event,
                                      esp_ble_mesh_sensor_server_cb_param_t *param)
{
    ESP_LOGD(TAG, "Sensor event %d opcode 0x%04" PRIx32, event, param->ctx.recv_op);
}

/* ================================================================
 * BLE Mesh Init
 * ================================================================ */

static esp_err_t ble_mesh_init(void)
{
    esp_ble_mesh_register_prov_callback(ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(ble_mesh_config_server_cb);
    esp_ble_mesh_register_sensor_server_callback(ble_mesh_sensor_server_cb);
#if ENABLE_LIGHT
    esp_ble_mesh_register_generic_server_callback(ble_mesh_generic_server_cb);
#endif

    esp_err_t err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) { ESP_LOGE(TAG, "Mesh init failed"); return err; }

    err = esp_ble_mesh_set_unprovisioned_device_name(BLE_MESH_DEVICE_NAME);
    if (err != ESP_OK) ESP_LOGW(TAG, "Set name failed");

    err = esp_ble_mesh_node_prov_enable(
        (esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) { ESP_LOGE(TAG, "Enable prov failed"); return err; }

    board_led_operation(LED_GREEN, LED_ON);
    const esp_timer_create_args_t led_args = {
        .callback = prov_led_timeout_cb, .name = "prov_led"
    };
    if (esp_timer_create(&led_args, &prov_led_timer) == ESP_OK) {
        esp_timer_start_once(prov_led_timer,
                             (uint64_t)PROV_LED_TIMEOUT_S * 1000 * 1000);
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
    ESP_LOGI(TAG, "=== BLE Mesh Sensor Server [BASELINE_TEST] ===");
#else
    ESP_LOGI(TAG, "=== BLE Mesh SIG Sensor Server Starting ===");
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

    /* ── Sensoren ──────────────────────────────────────────────── */
#if ENABLE_AHT20
    ESP_LOGI(TAG, "Init AHT20");
    ESP_ERROR_CHECK(aht20_init());
#endif
#if ENABLE_MAX30101
    ESP_LOGI(TAG, "Init MAX30101");
    ESP_ERROR_CHECK(max30101_init());
    ESP_ERROR_CHECK(max30101_start_sampling());
#endif
#if ENABLE_SGP30
    ESP_LOGI(TAG, "Init SGP30");
    ESP_ERROR_CHECK(sgp30_init());
    const esp_timer_create_args_t sgp30_args = {
        .callback = sgp30_timer_cb, .name = "sgp30"
    };
    ESP_ERROR_CHECK(esp_timer_create(&sgp30_args, &sgp30_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(sgp30_timer,
                                              SGP30_MEASUREMENT_INTERVAL_US));
#endif

    /* ── Button ───────────────────────────────────────────────── */
#if ENABLE_BUTTON
    button_hw_init();

    BaseType_t ok = xTaskCreate(
        button_task,
        "button",
        3072,
        NULL,
        8,
        &s_btn_task_handle
    );
    configASSERT(ok == pdPASS);
    configASSERT(s_btn_task_handle != NULL);

    s_btn_irq_blocked = false;
    gpio_intr_enable(BUTTON_GPIO_PIN);

    ESP_LOGI(TAG, "Button ready GPIO%d", BUTTON_GPIO_PIN);
#endif

    /* ── Light ────────────────────────────────────────────────── */
#if ENABLE_LIGHT
    led_hw_init();
    led_set_rgb(0, 0, 0);
    ESP_LOGI(TAG, "Light ready GPIO%d", LIGHT_GPIO_PIN);
#endif

    /* ── Bluetooth + Mesh ─────────────────────────────────────── */
    ESP_ERROR_CHECK(bluetooth_init());
    esp_ble_gap_set_device_name(BLE_MESH_DEVICE_NAME);
    ble_mesh_get_dev_uuid(dev_uuid);
    ESP_ERROR_CHECK(ble_mesh_init());

    ESP_LOGI(TAG, "=== All subsystems initialized ===");
}