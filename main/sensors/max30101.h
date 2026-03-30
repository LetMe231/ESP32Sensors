#pragma once

/**
 * @file max30101.h
 * @brief MAX30101 heart rate and SpO2 sensor interface.
 */

#include "config.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if ENABLE_MAX30101

/**
 * @brief Initialize the MAX30101 sensor
 * 
 * Configures I2C bus, enables sensor via INIT pin, and sets up
 * SpO2 mode with RED and IR LEDs at 100Hz sampling rate.
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t max30101_init(void);

/**
 * @brief Start the background task for continuous PPG sampling
 * 
 * This task samples the MAX30101 at 50 Hz and continuously computes
 * heart rate and SpO2 values in the background.
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t max30101_start_sampling(void);

/**
 * @brief Get the latest computed heart rate
 * 
 * @return uint8_t Heart rate in BPM (0 if invalid/not ready)
 */
uint8_t max30101_get_heart_rate(void);

/**
 * @brief Get the latest computed SpO2 value
 * 
 * @return uint8_t SpO2 percentage (0 if invalid/not ready)
 */
uint8_t max30101_get_spo2(void);

/**
 * @brief Get the latest raw RED and IR sensor values
 * 
 * @param red Pointer to store raw RED LED value
 * @param ir Pointer to store raw IR LED value
 */
void max30101_get_raw_values(uint32_t *red, uint32_t *ir);

#else

// Stub functions when MAX30101 is disabled
/**
 * @brief Stub initialization when MAX30101 is disabled.
 */
static inline esp_err_t max30101_init(void) {
    return ESP_OK;
}

/**
 * @brief Stub sampler start when MAX30101 is disabled.
 */
static inline esp_err_t max30101_start_sampling(void) {
    return ESP_OK;
}

/**
 * @brief Stub heart rate getter when MAX30101 is disabled.
 */
static inline uint8_t max30101_get_heart_rate(void) {
    return 0;
}

/**
 * @brief Stub SpO2 getter when MAX30101 is disabled.
 */
static inline uint8_t max30101_get_spo2(void) {
    return 0;
}

/**
 * @brief Stub raw value getter when MAX30101 is disabled.
 */
static inline void max30101_get_raw_values(uint32_t *red, uint32_t *ir) {
    if (red) *red = 0;
    if (ir) *ir = 0;
}

#endif // ENABLE_MAX30101

#ifdef __cplusplus
}
#endif
