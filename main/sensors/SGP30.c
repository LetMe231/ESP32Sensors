#include "config.h"
#include "SGP30.h"

#if ENABLE_SGP30

#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"

#define TAG "SGP30"
#define SGP30_ADDR 0x58
#define MAX_CONSECUTIVE_ERRORS 5

static i2c_master_bus_handle_t s_sgp30_bus = NULL;
static i2c_master_dev_handle_t s_sgp30 = NULL;
static uint8_t s_consecutive_errors = 0;

// Internal function to reinitialize the sensor
/**
 * @brief Reinitialize the SGP30 after repeated I2C failures.
 */
static esp_err_t sgp30_reinit_sensor(void)
{
    ESP_LOGW(TAG, "Reinitializing sensor after %d consecutive errors", s_consecutive_errors);
    
    // Send init command
    uint8_t init_cmd[2] = { 0x20, 0x03 };
    esp_err_t err = i2c_master_transmit(s_sgp30, init_cmd, sizeof(init_cmd), 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor reinit failed: %d", err);
        return err;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    s_consecutive_errors = 0;
    ESP_LOGI(TAG, "Sensor reinitialized successfully");
    return ESP_OK;
}

/**
 * @brief Initialize the SGP30 device and I2C bus.
 */
esp_err_t sgp30_init(void)
{
    // Initialize I2C bus
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = SGP30_I2C_PORT,
        .scl_io_num = SGP30_SCL_GPIO,
        .sda_io_num = SGP30_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_sgp30_bus), TAG, "I2C bus init failed");

    // Add SGP30 device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SGP30_ADDR,
        .scl_speed_hz    = SGP30_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_sgp30_bus, &dev_cfg, &s_sgp30), TAG, "Device add failed");

    // Initialize sensor (send initialization command)
    uint8_t init_cmd[2] = { 0x20, 0x03 }; // Init Air Quality algorithm
    ESP_RETURN_ON_ERROR(i2c_master_transmit(s_sgp30, init_cmd, sizeof(init_cmd), 100), TAG, "Sensor init failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;
}

/**
 * @brief Read eCO2 and TVOC measurements from the SGP30.
 */
esp_err_t sgp30_read_values(uint16_t *eco2, uint16_t *tvoc)
{
    uint8_t cmd[2] = { 0x20, 0x08 }; // Measure Air Quality command
    uint8_t data[6];
    
    // SGP30 needs 12ms to complete measurement
    // Use longer timeout when BLE is active (up to 2 seconds)
    esp_err_t err = i2c_master_transmit(s_sgp30, cmd, sizeof(cmd), 2000);
    if (err != ESP_OK) {
        s_consecutive_errors++;
        ESP_LOGW(TAG, "Failed to send measure command: %d (error count: %d)", err, s_consecutive_errors);
        
        // Attempt recovery after multiple failures
        if (s_consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
            sgp30_reinit_sensor();
        }
        return err;
    }
    
    // Wait for measurement to complete
    // Use longer delay when system is busy (BLE active)
    vTaskDelay(pdMS_TO_TICKS(25));  // Increased from 15ms to 25ms
    
    // Read the result with retries
    int retries = 3;
    while (retries > 0) {
        err = i2c_master_receive(s_sgp30, data, sizeof(data), 2000);
        if (err == ESP_OK) {
            break;
        }
        retries--;
        if (retries > 0) {
            vTaskDelay(pdMS_TO_TICKS(5));  // Short delay between retries
        }
    }
    
    if (err != ESP_OK) {
        s_consecutive_errors++;
        ESP_LOGW(TAG, "Failed to read values after retries: %d (error count: %d)", err, s_consecutive_errors);
        
        // Attempt recovery after multiple failures
        if (s_consecutive_errors >= MAX_CONSECUTIVE_ERRORS) {
            sgp30_reinit_sensor();
        }
        return err;
    }
    
    // Success - reset error counter
    s_consecutive_errors = 0;
    
    // Parse eCO2 and TVOC (each 2 bytes + CRC)
    if (eco2) {
        *eco2 = ((uint16_t)data[0] << 8) | data[1];
    }
    if (tvoc) {
        *tvoc = ((uint16_t)data[3] << 8) | data[4];
    }
    return ESP_OK;
}

#endif // ENABLE_SGP30
