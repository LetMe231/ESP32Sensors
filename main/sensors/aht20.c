/*
 * AHT20 Temperature and Humidity Sensor Driver
 * 
 * I2C Address: 0x38
 * Measurement Range: -40°C to +85°C, 0% to 100% RH
 */

#include "config.h"
#include "aht20.h"

#if ENABLE_AHT20

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include <math.h>

#define TAG "AHT20"

// AHT20 Configuration from config.h
#define AHT20_ADDR 0x38

static i2c_master_bus_handle_t s_aht20_bus = NULL;
static i2c_master_dev_handle_t s_aht20 = NULL;

/**
 * @brief Initialize the AHT20 device and I2C bus.
 */
esp_err_t aht20_init(void)
{
    // Initialize I2C bus
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = AHT20_I2C_PORT,
        .scl_io_num = AHT20_SCL_GPIO,
        .sda_io_num = AHT20_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_aht20_bus), TAG, "I2C bus init failed");

    // Add AHT20 device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AHT20_ADDR,
        .scl_speed_hz = AHT20_FREQ_HZ,
    };

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_aht20_bus, &dev_cfg, &s_aht20), TAG, "Device add failed");

    // Give sensor time after power-up
    vTaskDelay(pdMS_TO_TICKS(120));

    // Check calibration status
    uint8_t cmd = 0x71;
    uint8_t status = 0;
    esp_err_t err = i2c_master_transmit_receive(s_aht20, &cmd, 1, &status, 1, 100);
    if (err != ESP_OK) return err;

    // If not calibrated, send initialization command
    if ((status & 0x18) != 0x18) {
        uint8_t init_cmd[3] = { 0xBE, 0x08, 0x00 };
        err = i2c_master_transmit(s_aht20, init_cmd, sizeof(init_cmd), 100);
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "AHT20 initialized successfully");
    return ESP_OK;
}

/**
 * @brief Read temperature and humidity in fixed-point format.
 */
esp_err_t aht20_read_x100(int16_t *temp_x100, uint16_t *rh_x100)
{
    if (!s_aht20) {
        return ESP_ERR_INVALID_STATE;
    }

    // Trigger measurement: 0xAC 0x33 0x00
    uint8_t trig[3] = { 0xAC, 0x33, 0x00 };
    ESP_RETURN_ON_ERROR(i2c_master_transmit(s_aht20, trig, sizeof(trig), 100), TAG, "Trigger failed");

    // Wait for conversion (~80ms)
    vTaskDelay(pdMS_TO_TICKS(85));

    // Read 6 bytes (status + 5 data bytes)
    uint8_t d[6] = {0};
    ESP_RETURN_ON_ERROR(i2c_master_receive(s_aht20, d, sizeof(d), 100), TAG, "Read failed");

    // If busy bit still set, retry
    for (int i = 0; (d[0] & 0x80) && i < 3; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        ESP_RETURN_ON_ERROR(i2c_master_receive(s_aht20, d, sizeof(d), 100), TAG, "Read retry failed");
    }
    if (d[0] & 0x80) return ESP_ERR_TIMEOUT;

    // Parse 20-bit humidity and temperature values
    uint32_t raw_rh = ((uint32_t)d[1] << 12) | ((uint32_t)d[2] << 4) | ((uint32_t)d[3] >> 4);
    uint32_t raw_t  = (((uint32_t)d[3] & 0x0F) << 16) | ((uint32_t)d[4] << 8) | (uint32_t)d[5];

    // Convert to physical values using datasheet formulas
    // RH = (SRH / 2^20) * 100%
    // T = (ST / 2^20) * 200 - 50°C
    float rh = (raw_rh * 100.0f) / 1048576.0f;
    float t  = (raw_t  * 200.0f) / 1048576.0f - 50.0f;

    // Convert to fixed-point (x100)
    int32_t tx = (int32_t)lroundf(t * 100.0f);
    int32_t hx = (int32_t)lroundf(rh * 100.0f);

    // Clamp values
    if (tx < -4000) tx = -4000;
    if (tx >  8500) tx =  8500;
    if (hx <     0) hx =     0;
    if (hx > 10000) hx = 10000;

    *temp_x100 = (int16_t)tx;
    *rh_x100   = (uint16_t)hx;

    return ESP_OK;
}

#endif // ENABLE_AHT20
