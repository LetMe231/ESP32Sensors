#include "config.h"
#include "SGP30.h"

#if ENABLE_SGP30

#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "SGP30"

#define SGP30_ADDR                     0x58
#define MAX_CONSECUTIVE_ERRORS         5

#define SGP30_CMD_IAQ_INIT_MSB         0x20
#define SGP30_CMD_IAQ_INIT_LSB         0x03

#define SGP30_CMD_MEASURE_IAQ_MSB      0x20
#define SGP30_CMD_MEASURE_IAQ_LSB      0x08

#define SGP30_CMD_GET_BASELINE_MSB     0x20
#define SGP30_CMD_GET_BASELINE_LSB     0x15

#define SGP30_CMD_SET_BASELINE_MSB     0x20
#define SGP30_CMD_SET_BASELINE_LSB     0x1E

#define SGP30_CMD_SET_HUMIDITY_MSB     0x20
#define SGP30_CMD_SET_HUMIDITY_LSB     0x61

#define SGP30_INIT_DELAY_MS            20
#define SGP30_MEASUREMENT_DELAY_MS     25
#define SGP30_MIN_MEASUREMENT_PERIOD_MS 1000
#define SGP30_RETRY_DELAY_MS           5
#define SGP30_READ_RETRIES             3
#define SGP30_BASELINE_REFRESH_MS      (60 * 60 * 1000) /* 1 h */

static i2c_master_bus_handle_t s_sgp30_bus = NULL;
static i2c_master_dev_handle_t s_sgp30 = NULL;

static uint8_t s_consecutive_errors = 0;
static bool s_initialized = false;

/* cached latest valid measurement */
static bool s_has_valid_sample = false;
static uint16_t s_last_eco2 = 400;
static uint16_t s_last_tvoc = 0;

/* track timing so we do not measure faster than 1 Hz */
static TickType_t s_last_measure_tick = 0;
static bool s_last_measure_tick_valid = false;

/* cached baseline for restore after reinit */
static bool s_has_cached_baseline = false;
static uint16_t s_cached_baseline_eco2 = 0;
static uint16_t s_cached_baseline_tvoc = 0;
static TickType_t s_last_baseline_refresh_tick = 0;
static bool s_last_baseline_refresh_tick_valid = false;

/* cached humidity compensation word for restore after reinit */
static bool s_has_humidity_comp = false;
static uint16_t s_abs_humidity_word = 0;

/* ────────────────────────────────────────────────────────── */
/* Helpers                                                   */
/* ────────────────────────────────────────────────────────── */

static uint8_t sgp30_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static esp_err_t sgp30_write_raw(const uint8_t *buf, size_t len, int timeout_ms)
{
    if (s_sgp30 == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit(s_sgp30, buf, len, timeout_ms);
}

static esp_err_t sgp30_send_cmd2(uint8_t msb, uint8_t lsb, int timeout_ms)
{
    uint8_t cmd[2] = { msb, lsb };
    return sgp30_write_raw(cmd, sizeof(cmd), timeout_ms);
}

static esp_err_t sgp30_read_words_checked(uint8_t *data, size_t len, int timeout_ms)
{
    if (s_sgp30 == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = i2c_master_receive(s_sgp30, data, len, timeout_ms);
    if (err != ESP_OK) {
        return err;
    }

    /* each word is 2 data bytes + 1 CRC byte */
    if ((len % 3) != 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    for (size_t i = 0; i < len; i += 3) {
        uint8_t crc = sgp30_crc8(&data[i], 2);
        if (crc != data[i + 2]) {
            ESP_LOGW(TAG, "CRC mismatch at word %u: got 0x%02X expected 0x%02X",
                     (unsigned)(i / 3), data[i + 2], crc);
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    return ESP_OK;
}

static esp_err_t sgp30_set_abs_humidity_word_internal(uint16_t abs_humidity_word)
{
    uint8_t buf[5];
    uint8_t word[2] = {
        (uint8_t)(abs_humidity_word >> 8),
        (uint8_t)(abs_humidity_word & 0xFF)
    };

    buf[0] = SGP30_CMD_SET_HUMIDITY_MSB;
    buf[1] = SGP30_CMD_SET_HUMIDITY_LSB;
    buf[2] = word[0];
    buf[3] = word[1];
    buf[4] = sgp30_crc8(word, 2);

    esp_err_t err = sgp30_write_raw(buf, sizeof(buf), 200);
    if (err == ESP_OK) {
        s_abs_humidity_word = abs_humidity_word;
        s_has_humidity_comp = true;
    }

    return err;
}

static esp_err_t sgp30_get_iaq_baseline_internal(uint16_t *eco2_baseline, uint16_t *tvoc_baseline)
{
    uint8_t data[6];

    esp_err_t err = sgp30_send_cmd2(SGP30_CMD_GET_BASELINE_MSB, SGP30_CMD_GET_BASELINE_LSB, 200);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    err = sgp30_read_words_checked(data, sizeof(data), 200);
    if (err != ESP_OK) {
        return err;
    }

    if (eco2_baseline) {
        *eco2_baseline = ((uint16_t)data[0] << 8) | data[1];
    }
    if (tvoc_baseline) {
        *tvoc_baseline = ((uint16_t)data[3] << 8) | data[4];
    }

    return ESP_OK;
}

static esp_err_t sgp30_set_iaq_baseline_internal(uint16_t eco2_baseline, uint16_t tvoc_baseline)
{
    uint8_t buf[8];
    uint8_t eco2_word[2] = {
        (uint8_t)(eco2_baseline >> 8),
        (uint8_t)(eco2_baseline & 0xFF)
    };
    uint8_t tvoc_word[2] = {
        (uint8_t)(tvoc_baseline >> 8),
        (uint8_t)(tvoc_baseline & 0xFF)
    };

    buf[0] = SGP30_CMD_SET_BASELINE_MSB;
    buf[1] = SGP30_CMD_SET_BASELINE_LSB;
    buf[2] = eco2_word[0];
    buf[3] = eco2_word[1];
    buf[4] = sgp30_crc8(eco2_word, 2);
    buf[5] = tvoc_word[0];
    buf[6] = tvoc_word[1];
    buf[7] = sgp30_crc8(tvoc_word, 2);

    esp_err_t err = sgp30_write_raw(buf, sizeof(buf), 200);
    if (err == ESP_OK) {
        s_cached_baseline_eco2 = eco2_baseline;
        s_cached_baseline_tvoc = tvoc_baseline;
        s_has_cached_baseline = true;
    }

    return err;
}

static esp_err_t sgp30_reinit_sensor(void)
{
    ESP_LOGW(TAG, "Reinitializing SGP30 after %u consecutive errors", s_consecutive_errors);

    esp_err_t err = sgp30_send_cmd2(SGP30_CMD_IAQ_INIT_MSB, SGP30_CMD_IAQ_INIT_LSB, 500);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IAQ init during reinit failed: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(SGP30_INIT_DELAY_MS));

    if (s_has_humidity_comp) {
        err = sgp30_set_abs_humidity_word_internal(s_abs_humidity_word);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to restore humidity compensation: %s", esp_err_to_name(err));
        }
    }

    if (s_has_cached_baseline) {
        err = sgp30_set_iaq_baseline_internal(s_cached_baseline_eco2, s_cached_baseline_tvoc);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to restore IAQ baseline: %s", esp_err_to_name(err));
        }
    }

    s_consecutive_errors = 0;
    s_last_measure_tick_valid = false;

    ESP_LOGI(TAG, "SGP30 reinitialized");
    return ESP_OK;
}

static void sgp30_wait_for_next_measure_slot(void)
{
    if (!s_last_measure_tick_valid) {
        return;
    }

    TickType_t now = xTaskGetTickCount();
    TickType_t min_period = pdMS_TO_TICKS(SGP30_MIN_MEASUREMENT_PERIOD_MS);
    TickType_t elapsed = now - s_last_measure_tick;

    if (elapsed < min_period) {
        vTaskDelay(min_period - elapsed);
    } else if (elapsed > pdMS_TO_TICKS(1500)) {
        ESP_LOGW(TAG, "SGP30 read interval too slow (%lu ms). Best stability is 1 Hz.",
                 (unsigned long)((elapsed * 1000UL) / configTICK_RATE_HZ));
    }
}

static void sgp30_refresh_cached_baseline_if_due(void)
{
    TickType_t now = xTaskGetTickCount();

    if (s_last_baseline_refresh_tick_valid) {
        TickType_t elapsed = now - s_last_baseline_refresh_tick;
        if (elapsed < pdMS_TO_TICKS(SGP30_BASELINE_REFRESH_MS)) {
            return;
        }
    }

    uint16_t eco2_baseline = 0;
    uint16_t tvoc_baseline = 0;
    esp_err_t err = sgp30_get_iaq_baseline_internal(&eco2_baseline, &tvoc_baseline);
    if (err == ESP_OK) {
        s_cached_baseline_eco2 = eco2_baseline;
        s_cached_baseline_tvoc = tvoc_baseline;
        s_has_cached_baseline = true;
        s_last_baseline_refresh_tick = now;
        s_last_baseline_refresh_tick_valid = true;

        ESP_LOGI(TAG, "Baseline refreshed: eCO2=0x%04X TVOC=0x%04X",
                 eco2_baseline, tvoc_baseline);
    } else {
        ESP_LOGW(TAG, "Baseline refresh failed: %s", esp_err_to_name(err));
    }
}

/* ────────────────────────────────────────────────────────── */
/* Public API                                                */
/* ────────────────────────────────────────────────────────── */

esp_err_t sgp30_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = SGP30_I2C_PORT,
        .scl_io_num = SGP30_SCL_GPIO,
        .sda_io_num = SGP30_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_sgp30_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SGP30_ADDR,
        .scl_speed_hz = SGP30_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(s_sgp30_bus, &dev_cfg, &s_sgp30);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(err));
        return err;
    }

    err = sgp30_send_cmd2(SGP30_CMD_IAQ_INIT_MSB, SGP30_CMD_IAQ_INIT_LSB, 500);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SGP30 IAQ init failed: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(SGP30_INIT_DELAY_MS));

    s_consecutive_errors = 0;
    s_initialized = true;
    s_last_measure_tick_valid = false;
    s_has_valid_sample = false;

    ESP_LOGI(TAG, "SGP30 initialized");
    return ESP_OK;
}

esp_err_t sgp30_read_values(uint16_t *eco2, uint16_t *tvoc)
{
    if (!s_initialized || s_sgp30 == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (eco2 == NULL && tvoc == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    sgp30_wait_for_next_measure_slot();

    uint8_t data[6];
    esp_err_t err = sgp30_send_cmd2(SGP30_CMD_MEASURE_IAQ_MSB, SGP30_CMD_MEASURE_IAQ_LSB, 500);
    if (err != ESP_OK) {
        s_consecutive_errors++;
        ESP_LOGW(TAG, "Measure command failed: %s (count=%u)",
                 esp_err_to_name(err), s_consecutive_errors);

        if (s_consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
            (void)sgp30_reinit_sensor();
        }

        if (s_has_valid_sample) {
            if (eco2) *eco2 = s_last_eco2;
            if (tvoc) *tvoc = s_last_tvoc;
        }

        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(SGP30_MEASUREMENT_DELAY_MS));

    int retries = SGP30_READ_RETRIES;
    while (retries-- > 0) {
        err = sgp30_read_words_checked(data, sizeof(data), 500);
        if (err == ESP_OK) {
            break;
        }

        if (retries > 0) {
            vTaskDelay(pdMS_TO_TICKS(SGP30_RETRY_DELAY_MS));
        }
    }

    if (err != ESP_OK) {
        s_consecutive_errors++;
        ESP_LOGW(TAG, "Read failed: %s (count=%u)",
                 esp_err_to_name(err), s_consecutive_errors);

        if (s_consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
            (void)sgp30_reinit_sensor();
        }

        if (s_has_valid_sample) {
            if (eco2) *eco2 = s_last_eco2;
            if (tvoc) *tvoc = s_last_tvoc;
        }

        return err;
    }

    s_consecutive_errors = 0;
    s_last_measure_tick = xTaskGetTickCount();
    s_last_measure_tick_valid = true;

    s_last_eco2 = ((uint16_t)data[0] << 8) | data[1];
    s_last_tvoc = ((uint16_t)data[3] << 8) | data[4];
    s_has_valid_sample = true;

    if (eco2) {
        *eco2 = s_last_eco2;
    }
    if (tvoc) {
        *tvoc = s_last_tvoc;
    }

    sgp30_refresh_cached_baseline_if_due();

    return ESP_OK;
}

/*
 * temperature_c_centi: 2534 = 25.34 °C
 * relative_humidity_centi: 4567 = 45.67 %RH
 */
esp_err_t sgp30_set_absolute_humidity(uint16_t temperature_c_centi, uint16_t relative_humidity_centi)
{
    if (!s_initialized || s_sgp30 == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    double temp_c = (double)temperature_c_centi / 100.0;
    double rh = (double)relative_humidity_centi / 100.0;

    if (rh < 0.0) rh = 0.0;
    if (rh > 100.0) rh = 100.0;

    /* absolute humidity in g/m^3 */
    double sat_vapor_pressure = 6.112 * exp((17.62 * temp_c) / (243.12 + temp_c));
    double vapor_pressure = (rh / 100.0) * sat_vapor_pressure;
    double abs_humidity_g_m3 = 216.7 * (vapor_pressure / (273.15 + temp_c));

    /* Sensirion uses 8.8 fixed-point g/m^3 */
    uint32_t abs_humidity_word = (uint32_t)((abs_humidity_g_m3 * 256.0) + 0.5);

    if (abs_humidity_word > 0xFFFF) {
        abs_humidity_word = 0xFFFF;
    }

    esp_err_t err = sgp30_set_abs_humidity_word_internal((uint16_t)abs_humidity_word);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Setting humidity compensation failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Humidity compensation set: T=%.2f°C RH=%.2f%% word=0x%04X",
             temp_c, rh, (unsigned)abs_humidity_word);

    return ESP_OK;
}

esp_err_t sgp30_get_iaq_baseline(uint16_t *eco2_baseline, uint16_t *tvoc_baseline)
{
    if (!s_initialized || s_sgp30 == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (eco2_baseline == NULL || tvoc_baseline == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return sgp30_get_iaq_baseline_internal(eco2_baseline, tvoc_baseline);
}

esp_err_t sgp30_set_iaq_baseline(uint16_t eco2_baseline, uint16_t tvoc_baseline)
{
    if (!s_initialized || s_sgp30 == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return sgp30_set_iaq_baseline_internal(eco2_baseline, tvoc_baseline);
}

#endif /* ENABLE_SGP30 */