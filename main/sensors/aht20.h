#pragma once

/**
 * @file aht20.h
 * @brief AHT20 temperature and humidity sensor interface.
 */

#include "config.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if ENABLE_AHT20

/**
 * @brief Initialize the AHT20 sensor I2C bus and device
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t aht20_init(void);

/**
 * @brief Read temperature and humidity from AHT20
 * 
 * @param temp_x100 Pointer to store temperature in 0.01°C units (e.g., 2530 = 25.30°C)
 * @param rh_x100 Pointer to store relative humidity in 0.01% units (e.g., 6523 = 65.23%)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t aht20_read_x100(int16_t *temp_x100, uint16_t *rh_x100);

#else

// Stub functions when AHT20 is disabled
/**
 * @brief Stub initialization when AHT20 is disabled.
 */
static inline esp_err_t aht20_init(void) {
    return ESP_OK;
}

/**
 * @brief Stub read when AHT20 is disabled.
 */
static inline esp_err_t aht20_read_x100(int16_t *temp_x100, uint16_t *rh_x100) {
    if (temp_x100) *temp_x100 = 0;
    if (rh_x100) *rh_x100 = 0;
    return ESP_OK;
}

#endif // ENABLE_AHT20

#ifdef __cplusplus
}
#endif
