#pragma once

/**
 * @file config.h
 * @brief Compile-time configuration for different ESP32-S3 sensor configurations
 * 
 * This file defines which sensors and I2C configurations are enabled for the build.
 * Configuration is controlled via build flags in platformio.ini
 */

// ============================================================================
// SENSOR ENABLE/DISABLE SWITCHES (set via -D flag in build_flags)
// ============================================================================

// Default configuration: Both sensors enabled
/** @brief Enable AHT20 sensor driver (1=enabled, 0=disabled). */
#ifndef ENABLE_AHT20
#define ENABLE_AHT20 0
#endif

/** @brief Enable MAX30101 sensor driver (1=enabled, 0=disabled). */
#ifndef ENABLE_MAX30101
#define ENABLE_MAX30101 0
#endif

/** @brief Enable SGP30 sensor driver (1=enabled, 0=disabled). */
#ifndef ENABLE_SGP30
#define ENABLE_SGP30 0
#endif

/** @brief Unprovisioned BLE Mesh device name. */
#ifndef BLE_MESH_DEVICE_NAME
#define BLE_MESH_DEVICE_NAME "ESP32-S3-Sensor"
#endif
// ============================================================================
// I2C CONFIGURATION - AHT20 (Temperature/Humidity)
// ============================================================================

#if ENABLE_AHT20
#ifndef AHT20_I2C_PORT
#define AHT20_I2C_PORT I2C_NUM_0
#endif

#ifndef AHT20_SDA_GPIO
#define AHT20_SDA_GPIO 4
#endif

#ifndef AHT20_SCL_GPIO
#define AHT20_SCL_GPIO 5
#endif

#ifndef AHT20_FREQ_HZ
#define AHT20_FREQ_HZ 100000
#endif
#endif // ENABLE_AHT20

// ============================================================================
// I2C CONFIGURATION - MAX30101 (Heart Rate/SpO2)
// ============================================================================

#if ENABLE_MAX30101
#ifndef MAX30101_I2C_PORT
#define MAX30101_I2C_PORT I2C_NUM_1
#endif

#ifndef MAX30101_SDA_GPIO
#define MAX30101_SDA_GPIO 6
#endif

#ifndef MAX30101_SCL_GPIO
#define MAX30101_SCL_GPIO 7
#endif

#ifndef MAX30101_FREQ_HZ
#define MAX30101_FREQ_HZ 100000
#endif

#ifndef MAX30101_INIT_GPIO
#define MAX30101_INIT_GPIO 16
#endif
#endif // ENABLE_MAX30101

// ============================================================================
// SGP30 CONFIGURATION - (Air Quality Sensor)
// ============================================================================

#if ENABLE_SGP30
#ifndef SGP30_I2C_PORT
#define SGP30_I2C_PORT I2C_NUM_0
#endif

#ifndef SGP30_SDA_GPIO
#define SGP30_SDA_GPIO 1
#endif
#ifndef SGP30_SCL_GPIO
#define SGP30_SCL_GPIO 2
#endif
#ifndef SGP30_FREQ_HZ
#define SGP30_FREQ_HZ 100000
#endif
#endif // ENABLE_SGP30

// ============================================================================
// EXAMPLE CONFIGURATIONS
// ============================================================================

/*
 * Build command examples:
 * 
 * Both sensors (default):
 *   platformio run -e esp32s3
 * 
 * Only AHT20:
 *   platformio run -e esp32s3-aht20-only
 * 
 * Only MAX30101:
 *   platformio run -e esp32s3-max30101-only
 * 
 * Custom pins (AHT20 on I2C1):
 *   platformio run -e esp32s3-custom
 */
