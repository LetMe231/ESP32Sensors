#ifndef __SGP30_H__
#define __SGP30_H__

/**
 * @file SGP30.h
 * @brief SGP30 air quality sensor interface.
 */

#include "esp_err.h"

#if ENABLE_SGP30
/**
 * @brief Initialize the SGP30 sensor
 * 
 * Configures I2C bus and initializes the sensor for air quality measurements.
 * 
 * @return esp_err_t ESP_OK on success
 */

esp_err_t sgp30_init(void);

/**
 * @brief Read eCO2 and TVOC values from SGP30
 * 
 * Reads the latest eCO2 and TVOC values from the SGP30 sensor.
 * 
 * @param eco2 Pointer to store the eCO2 value (in ppm)
 * @param tvoc Pointer to store the TVOC value (in ppb)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sgp30_read_values(uint16_t *eco2, uint16_t *tvoc);

#else

// Stub functions when SGP30 is disabled
/**
 * @brief Stub initialization when SGP30 is disabled.
 */
static inline esp_err_t sgp30_init(void) {
    return ESP_OK;
}

/**
 * @brief Stub read when SGP30 is disabled.
 */
static inline esp_err_t sgp30_read_values(uint16_t *eco2, uint16_t *tvoc) {
    if (eco2) *eco2 = 0;
    if (tvoc) *tvoc = 0;
    return ESP_OK;
}

#endif // ENABLE_SGP30

#endif // __SGP30_H__
