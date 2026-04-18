#ifndef __SGP30_H__
#define __SGP30_H__

/**
 * @file SGP30.h
 * @brief SGP30 air quality sensor interface.
 */

#include <stdint.h>
#include "config.h"
#include "esp_err.h"

#if ENABLE_SGP30

/**
 * @brief Initialize the SGP30 sensor.
 *
 * Configures I2C bus and initializes the sensor for air quality measurements.
 *
 * @return ESP_OK on success.
 */
esp_err_t sgp30_init(void);

/**
 * @brief Read eCO2 and TVOC values from SGP30.
 *
 * Reads the latest eCO2 and TVOC values from the SGP30 sensor.
 *
 * @param eco2 Pointer to store the eCO2 value (ppm).
 * @param tvoc Pointer to store the TVOC value (ppb).
 * @return ESP_OK on success.
 */
esp_err_t sgp30_read_values(uint16_t *eco2, uint16_t *tvoc);

/**
 * @brief Set humidity compensation for the SGP30 IAQ algorithm.
 *
 * @param temperature_c_centi Temperature in centi-degrees Celsius.
 *        Example: 2534 = 25.34 °C
 * @param relative_humidity_centi Relative humidity in centi-percent.
 *        Example: 4567 = 45.67 %RH
 * @return ESP_OK on success.
 */
esp_err_t sgp30_set_absolute_humidity(uint16_t temperature_c_centi,
                                      uint16_t relative_humidity_centi);

/**
 * @brief Read the current IAQ baseline from the sensor.
 *
 * @param eco2_baseline Pointer to store eCO2 baseline.
 * @param tvoc_baseline Pointer to store TVOC baseline.
 * @return ESP_OK on success.
 */
esp_err_t sgp30_get_iaq_baseline(uint16_t *eco2_baseline,
                                 uint16_t *tvoc_baseline);

/**
 * @brief Restore a previously saved IAQ baseline.
 *
 * @param eco2_baseline Saved eCO2 baseline.
 * @param tvoc_baseline Saved TVOC baseline.
 * @return ESP_OK on success.
 */
esp_err_t sgp30_set_iaq_baseline(uint16_t eco2_baseline,
                                 uint16_t tvoc_baseline);

#else

static inline esp_err_t sgp30_init(void)
{
    return ESP_OK;
}

static inline esp_err_t sgp30_read_values(uint16_t *eco2, uint16_t *tvoc)
{
    if (eco2) *eco2 = 0;
    if (tvoc) *tvoc = 0;
    return ESP_OK;
}

static inline esp_err_t sgp30_set_absolute_humidity(uint16_t temperature_c_centi,
                                                    uint16_t relative_humidity_centi)
{
    (void)temperature_c_centi;
    (void)relative_humidity_centi;
    return ESP_OK;
}

static inline esp_err_t sgp30_get_iaq_baseline(uint16_t *eco2_baseline,
                                               uint16_t *tvoc_baseline)
{
    if (eco2_baseline) *eco2_baseline = 0;
    if (tvoc_baseline) *tvoc_baseline = 0;
    return ESP_OK;
}

static inline esp_err_t sgp30_set_iaq_baseline(uint16_t eco2_baseline,
                                               uint16_t tvoc_baseline)
{
    (void)eco2_baseline;
    (void)tvoc_baseline;
    return ESP_OK;
}

#endif /* ENABLE_SGP30 */

#endif /* __SGP30_H__ */