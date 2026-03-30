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
// BLE MESH SENSOR PROPERTY IDs
// ============================================================================

// Standard Bluetooth SIG Property IDs
#define PROP_TEMPERATURE     0x004F
#define PROP_HUMIDITY        0x0076
#define PROP_ECO2            0x0008

// Custom Property IDs (0x0100+ für nicht-standardisierte Sensoren)
#define PROP_HEART_RATE      0x0100
#define PROP_SPO2            0x0101
#define PROP_TVOC            0x0102
#define PROP_RAW_RED         0x0103
#define PROP_RAW_IR          0x0104


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
// POWER MANAGEMENT CONFIGURATION
// ============================================================================

/**
 * @brief Sensor publish interval in seconds.
 *
 * Increase this to reduce BLE TX duty cycle and extend battery life.
 * 30 s gives a good balance between latency and power for battery nodes.
 * Override with -DSENSOR_PUBLISH_INTERVAL_S=<n> in build flags.
 */
#ifndef SENSOR_PUBLISH_INTERVAL_S
#define SENSOR_PUBLISH_INTERVAL_S 30
#endif

/**
 * @brief Maximum CPU frequency (MHz) used by esp_pm_configure.
 *
 * 80 MHz is sufficient for BLE Mesh + I2C and saves ~35 % active current
 * vs the default 240 MHz. Set to 160 or 240 if more throughput is needed.
 */
#ifndef PM_MAX_CPU_FREQ_MHZ
#define PM_MAX_CPU_FREQ_MHZ 80
#endif

/**
 * @brief Minimum CPU frequency (MHz) / RTC clock used during light sleep.
 *
 * The FreeRTOS tickless idle will drop to this frequency when no task is
 * runnable. 10 MHz keeps RTC and BLE wakeup logic alive.
 */
#ifndef PM_MIN_CPU_FREQ_MHZ
#define PM_MIN_CPU_FREQ_MHZ 10
#endif

/**
 * @brief Seconds to keep the provisioning LED on while unprovisioned.
 *
 * After this timeout the green LED is turned off to stop wasting ~15 mA.
 * The node continues advertising; the LED simply turns off.
 */
#ifndef PROV_LED_TIMEOUT_S
#define PROV_LED_TIMEOUT_S 60
#endif

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
