# Sensors Directory

This directory contains all sensor driver modules for the BLE Mesh server.

## Structure

```
sensors/
├── aht20.h/c         - Temperature & Humidity sensor (I2C 0x38)
├── max30101.h/c      - Heart Rate & SpO2 sensor (I2C 0x57)
├── SGP30.h/c         - Air Quality sensor (I2C 0x58)
└── README.md         - This file
```

## Adding a New Sensor

To add a new sensor, follow this pattern:

1. Create `sensors/your_sensor.h` with API and stub functions
2. Create `sensors/your_sensor.c` with implementation wrapped in `#ifdef ENABLE_YOUR_SENSOR`
3. Add `#define ENABLE_YOUR_SENSOR 1` and GPIO pins to `config.h`
4. Add `enable_your_sensor.c` to `CMakeLists.txt` SRCS
5. Add `#include "sensors/your_sensor.h"` to `main.c`
6. Initialize in `app_main()` with `#ifdef ENABLE_YOUR_SENSOR` guards

See [main/../BUILD_VARIANTS.md](../BUILD_VARIANTS.md) for configuration details.

## Sensor Features

### AHT20 (Temperature & Humidity)
- **Bus:** I2C Bus 0 (GPIO 4 SDA, 5 SCL)
- **Address:** 0x38
- **Output:** Temperature (°C), Humidity (%)
- **API:** `aht20_init()`, `aht20_read_x100()`

### MAX30101 (Heart Rate & SpO2)
- **Bus:** I2C Bus 1 (GPIO 6 SDA, 7 SCL, GPIO 16 INIT)
- **Address:** 0x57
- **Output:** Heart Rate (BPM), SpO2 (%), Raw RED/IR values
- **API:** `max30101_init()`, `max30101_start_sampling()`, `max30101_get_heart_rate()`, `max30101_get_spo2()`, `max30101_get_raw_values()`
- **Background Task:** Continuous 100Hz sampling with algorithms

### SGP30 (Air Quality)
- **Bus:** I2C Bus 0 (GPIO 4 SDA, 5 SCL)
- **Address:** 0x58
- **Output:** eCO2 (ppm), TVOC (ppb)
- **API:** `sgp30_init()`, `sgp30_read_values()`

## Configuration

Each sensor can be independently enabled/disabled via `config.h`:

```c
#define ENABLE_AHT20 1      // Enable AHT20
#define ENABLE_MAX30101 1   // Enable MAX30101
#define ENABLE_SGP30 1      // Enable SGP30
```

When disabled, stub functions return safe defaults (ESP_OK, 0 values) with zero runtime overhead.
