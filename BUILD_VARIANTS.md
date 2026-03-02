# Multi-Variant ESP32-S3 BLE Mesh Firmware

This firmware supports compiling multiple variants for different sensor configurations, all from a single codebase. Use conditional compilation to enable/disable sensors and customize GPIO pins.

## Quick Start

### Build for Standard Configuration (Both Sensors)
```bash
platformio run -e esp32s3           # Build
platformio run -e esp32s3 -t upload # Upload to device
```

### Build for AHT20 Only (Temp/Humidity)
```bash
platformio run -e esp32s3-aht20-only -t upload
```

### Build for MAX30101 Only (Heart Rate/SpO2)
```bash
platformio run -e esp32s3-max30101-only -t upload
```

### Build with Custom Pins
```bash
platformio run -e esp32s3-custom -t upload
# Edit platformio.ini esp32s3-custom section to customize GPIO pins
```

## Configuration System

### How It Works

1. **config.h** - Central configuration file with sensor enable/disable flags and GPIO pin definitions
2. **platformio.ini** - Build environments with `build_flags` to override config.h defaults
3. **aht20.c/h, max30101.c/h** - Wrapped with `#ifdef` guards; stub functions when disabled
4. **main.c** - Conditional sensor initialization based on config.h

### Configuration Hierarchy

```
platformio.ini build_flags (-D defines)
    ↓ (overrides)
config.h #define values
    ↓ (used by)
aht20.c/h, max30101.c/h
    ↓ (controlled by)
main.c #ifdef blocks
```

## Available Build Environments

### 1. **esp32s3** (DEFAULT - Both Sensors)
**Sensors:** AHT20 + MAX30101  
**Use when:** You have both sensors connected

**Hardware Setup:**
```
AHT20 (I2C Bus 0):
  - SDA: GPIO 4
  - SCL: GPIO 5
  - Power: 3.3V
  - GND: GND

MAX30101 (I2C Bus 1):
  - SDA: GPIO 6
  - SCL: GPIO 7
  - INIT: GPIO 16
  - Power: 3.3V
  - GND: GND
```

**Build & Upload:**
```bash
platformio run -e esp32s3 -t upload
```

### 2. **esp32s3-aht20-only** (AHT20 ONLY)
**Sensors:** AHT20 only  
**Use when:** You only have the temperature/humidity sensor

**Hardware Setup:**
```
AHT20 (I2C Bus 0):
  - SDA: GPIO 4
  - SCL: GPIO 5
  - Power: 3.3V
  - GND: GND

(No MAX30101 connected)
```

**Build & Upload:**
```bash
platformio run -e esp32s3-aht20-only -t upload
```

**Data Published:**
```
Payload (14 bytes):
  [0-1]:   temp_x100 (actual temperature)
  [2-3]:   rh_x100 (actual humidity)
  [4-11]:  0x00 (no MAX30101 data)
  [12-13]: 0x00 (no HR/SpO2)
```

### 3. **esp32s3-max30101-only** (MAX30101 ONLY)
**Sensors:** MAX30101 only  
**Use when:** You only have the heart rate/SpO2 sensor

**Hardware Setup:**
```
(No AHT20 connected)

MAX30101 (I2C Bus 1):
  - SDA: GPIO 6
  - SCL: GPIO 7
  - INIT: GPIO 16
  - Power: 3.3V
  - GND: GND
```

**Build & Upload:**
```bash
platformio run -e esp32s3-max30101-only -t upload
```

**Data Published:**
```
Payload (14 bytes):
  [0-3]:   0x00 (no AHT20 data)
  [4-7]:   raw RED (actual sensor data)
  [8-11]:  raw IR (actual sensor data)
  [12]:    heart_rate (actual HR)
  [13]:    spo2 (actual SpO2)
```

### 4. **esp32s3-custom** (TEMPLATE - Edit for Your Pins)
**Use when:** Your sensors are on different GPIO pins

**Steps to Customize:**

1. Edit `platformio.ini` and find the `[env:esp32s3-custom]` section:

```ini
[env:esp32s3-custom]
platform = espressif32
board = esp32-s3-devkitc-1
framework = espidf
upload_port = COM8
monitor_port = COM8
monitor_speed = 115200
build_flags = 
    -DENABLE_AHT20=1              # Set to 0 to disable AHT20
    -DENABLE_MAX30101=1            # Set to 0 to disable MAX30101
    ;-------- CUSTOMIZE THESE GPIO PINS --------
    -DAHT20_I2C_PORT=0             # I2C bus number
    -DAHT20_SDA_GPIO=4             # ← Change these
    -DAHT20_SCL_GPIO=5             # ← Change these
    -DMAX30101_I2C_PORT=1          # I2C bus number
    -DMAX30101_SDA_GPIO=6          # ← Change these
    -DMAX30101_SCL_GPIO=7          # ← Change these
    -DMAX30101_INIT_GPIO=16        # ← Change this
```

2. Replace the GPIO numbers with your wiring:

```bash
# Example: AHT20 on GPIO 8,9 and MAX30101 on GPIO 10,11,12
build_flags = 
    -DENABLE_AHT20=1
    -DENABLE_MAX30101=1
    -DAHT20_I2C_PORT=0
    -DAHT20_SDA_GPIO=8
    -DAHT20_SCL_GPIO=9
    -DMAX30101_I2C_PORT=1
    -DMAX30101_SDA_GPIO=10
    -DMAX30101_SCL_GPIO=11
    -DMAX30101_INIT_GPIO=12
```

3. Build and upload:

```bash
platformio run -e esp32s3-custom -t upload
```

## Configuration Reference

### config.h Flags

| Define | Default | Purpose |
|--------|---------|---------|
| `ENABLE_AHT20` | 1 | Enable AHT20 sensor (0=disabled) |
| `ENABLE_MAX30101` | 1 | Enable MAX30101 sensor (0=disabled) |
| `AHT20_I2C_PORT` | 0 | I2C bus for AHT20 (0 or 1) |
| `AHT20_SDA_GPIO` | 4 | SDA pin for AHT20 |
| `AHT20_SCL_GPIO` | 5 | SCL pin for AHT20 |
| `MAX30101_I2C_PORT` | 1 | I2C bus for MAX30101 (0 or 1) |
| `MAX30101_SDA_GPIO` | 6 | SDA pin for MAX30101 |
| `MAX30101_SCL_GPIO` | 7 | SCL pin for MAX30101 |
| `MAX30101_INIT_GPIO` | 16 | INIT pin for MAX30101 |

### How to Override Defaults

**Option 1: Edit config.h (Permanent)**
```c
// main/config.h
#define ENABLE_AHT20 0        // Disable AHT20
#define ENABLE_MAX30101 1      // Enable MAX30101
#define MAX30101_SDA_GPIO 10   // Use GPIO 10 instead of 6
```

**Option 2: Use platformio.ini build_flags (Temporary, per environment)**
```ini
[env:my-variant]
build_flags = -DENABLE_AHT20=0 -DENABLE_MAX30101=1 -DMAX30101_SDA_GPIO=10
```

**Option 3: Command-line build flags**
```bash
platformio run -e esp32s3 --build-flag="-DENABLE_AHT20=0"
```

## Payload Format

All variants publish the same 14-byte payload structure:

```
Byte  Field         Type      Size  Notes
─────────────────────────────────────────────────────────
0-1   temp_x100     int16_t   2     Temperature × 100 (°C)
2-3   rh_x100       uint16_t  2     Humidity × 100 (%)
4-7   raw_red       uint32_t  4     MAX30101 RED channel
8-11  raw_ir        uint32_t  4     MAX30101 IR channel
12    heart_rate    uint8_t   1     Heart rate (bpm)
13    spo2          uint8_t   1     SpO2 (%)
─────────────────────────────────────────────────────────
      TOTAL                   14    All little-endian
```

**When sensor is disabled:**
- Fields are set to 0x00
- No I2C initialization attempts
- No RX sampling task spawned
- Full code is eliminated by compiler

## Compile-Time vs Runtime Configuration

This system uses **compile-time configuration** (preprocessor #ifdefs), NOT runtime configuration. Benefits:

✅ **No runtime overhead** - Disabled code completely removed  
✅ **Smaller binary size** - Only compiled code is linked  
✅ **Zero branches** - No runtime if-statements for disabled features  
✅ **Simple to understand** - Clear config.h and platformio.ini  

**Trade-off:** Must recompile to change sensor configuration (no over-the-air reconfiguration)

## Troubleshooting

### Build fails with "undefined reference to axt20_init"
- **Cause:** Trying to call sensor function without it being compiled
- **Solution:** Check that sensor is enabled in config.h or build_flags
- **Check:** Run `platformio run -v` to see actual build flags

### I2C address conflicts (both sensors on same bus)
- **Solution:** By default, sensors use different I2C buses
- **If needed:** Edit `AHT20_I2C_PORT` and `MAX30101_I2C_PORT` in config.h or platformio.ini
- **Note:** Ensure I2C buses are initialized in the I2C driver code

### GPIO pin conflicts
- **Cause:** Two sensors trying to use same GPIO
- **Solution:** Assign different pins in config.h or build_flags
- **Check:** Verify your wiring diagram matches the configuration

### Sensor stuck during initialization
- **Check:** Are GPIO pins correct?
- **Check:** Is power supply adequate? (AHT20 needs ~5mA, MAX30101 ~20mA)
- **Check:** Is I2C bus initialized for both sensors?
- **Solution:** Check build logs with `platformio run -e env-name -v`

## Advanced Usage

### Adding a Third Sensor

1. Create `main/third_sensor.c` and `main/third_sensor.h`
2. Wrap with `#ifdef ENABLE_THIRD_SENSOR`
3. Add to `config.h`:
   ```c
   #define ENABLE_THIRD_SENSOR 1
   #define THIRD_SENSOR_I2C_PORT 0
   #define THIRD_SENSOR_SDA_GPIO 8
   ```
4. Add stub functions in header for when disabled
5. Add to `main/CMakeLists.txt`:
   ```cmake
   idf_component_register(SRCS aht20.c max30101.c third_sensor.c main.c ...)
   ```
6. Update `pub_timer_cb()` in main.c with `#ifdef ENABLE_THIRD_SENSOR` blocks
7. Create new environment in `platformio.ini` with appropriate build_flags

### Creating an "All Disabled" Build

For testing/debugging without sensors:
```ini
[env:esp32s3-no-sensors]
build_flags = 
    -DENABLE_AHT20=0
    -DENABLE_MAX30101=0
```
This compiles BLE Mesh and publishing code only; helpful for gateway testing.

## Building for Multiple Devices

### Workflow
```bash
# Board A: Both sensors
platformio run -e esp32s3 -t upload --upload-port COM3

# Board B: AHT20 only
platformio run -e esp32s3-aht20-only -t upload --upload-port COM4

# Board C: Custom pins (edit config first)
platformio run -e esp32s3-custom -t upload --upload-port COM5
```

### Using pio project config
Create separate PlatformIO projects if you need different persistent settings per board.

## Summary

| Environment | Sensors | Use Case |
|------------|---------|----------|
| `esp32s3` | AHT20 + MAX30101 | All-in-one sensor board |
| `esp32s3-aht20-only` | AHT20 | Temperature/humidity only |
| `esp32s3-max30101-only` | MAX30101 | Heart rate/SpO2 only |
| `esp32s3-custom` | Flexible | Custom GPIO pins |

Choose the environment that matches your hardware, build, and upload!
