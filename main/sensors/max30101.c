/*
 * MAX30101 Heart Rate and SpO2 Sensor Driver
 * 
 * I2C Address: 0x57
 * Features:
 * - Photoplethysmography (PPG) with RED and IR LEDs
 * - Local heart rate computation (30-220 BPM)
 * - Local SpO2 computation (70-100%)
 * - Continuous background sampling at 100Hz
 */

#include "config.h"
#include "max30101.h"

#if ENABLE_MAX30101

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include <string.h>

#define TAG "MAX30101"

// MAX30101 I2C Address
#define MAX30101_ADDR 0x57

// MAX30101 Registers
#define REG_INTR_STATUS_1   0x00
#define REG_INTR_STATUS_2   0x01
#define REG_INTR_ENABLE_1   0x02
#define REG_INTR_ENABLE_2   0x03
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C   // Red LED
#define REG_LED2_PA         0x0D   // IR LED
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12

// HR/SpO2 Algorithm Parameters
#define SAMPLE_RATE_HZ       25   // Reduced from 100 Hz — saves ~75 % sensor power
#define SAMPLE_PERIOD_MS     (1000 / SAMPLE_RATE_HZ)  // 40 ms per sample
#define SAMPLE_BUFFER_SIZE   100  // 100 samples @ 25 Hz = 4 seconds of PPG data
#define MIN_PEAK_DISTANCE    10   // At 25 Hz: 10 samples = 0.4 s → max ~150 BPM
#define PEAK_THRESHOLD_RATIO 0.6f // Peak threshold: 60% above minimum

// PPG Data Buffer
typedef struct {
    uint32_t red_samples[SAMPLE_BUFFER_SIZE];
    uint32_t ir_samples[SAMPLE_BUFFER_SIZE];
    uint16_t index;
    uint16_t count;
    uint32_t last_peak_index;
    uint32_t peak_intervals[5];
    uint8_t interval_index;
    uint8_t interval_count;
} ppg_buffer_t;

static i2c_master_bus_handle_t s_max30101_bus = NULL;
static i2c_master_dev_handle_t s_max30101 = NULL;
static ppg_buffer_t s_ppg_buffer = {0};
static uint8_t s_latest_hr = 0;
static uint8_t s_latest_spo2 = 0;
static uint32_t s_latest_raw_red = 0;
static uint32_t s_latest_raw_ir = 0;

// Low-level I2C functions
/**
 * @brief Write a single MAX30101 register.
 */
static esp_err_t max30101_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_max30101, buf, sizeof(buf), 100);
}

/**
 * @brief Read bytes from a MAX30101 register.
 */
static esp_err_t max30101_read(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(s_max30101, &reg, 1, out, len, 100);
}

/**
 * @brief Read a single RED/IR sample from FIFO.
 */
static esp_err_t max30101_read_sample(uint32_t *red, uint32_t *ir)
{
    uint8_t d[6];
    esp_err_t err = max30101_read(REG_FIFO_DATA, d, sizeof(d));
    if (err) return err;

    // Parse 18-bit values (3 bytes each for RED and IR)
    uint32_t raw_red = ((uint32_t)d[0] << 16) | ((uint32_t)d[1] << 8) | d[2];
    uint32_t raw_ir  = ((uint32_t)d[3] << 16) | ((uint32_t)d[4] << 8) | d[5];

    raw_red &= 0x3FFFF; // 18-bit mask
    raw_ir  &= 0x3FFFF;

    *red = raw_red;
    *ir  = raw_ir;
    return ESP_OK;
}

// PPG Buffer Management
/**
 * @brief Add a sample to the rolling PPG buffer.
 */
static void ppg_add_sample(uint32_t red, uint32_t ir)
{
    s_ppg_buffer.red_samples[s_ppg_buffer.index] = red;
    s_ppg_buffer.ir_samples[s_ppg_buffer.index] = ir;
    s_ppg_buffer.index = (s_ppg_buffer.index + 1) % SAMPLE_BUFFER_SIZE;
    if (s_ppg_buffer.count < SAMPLE_BUFFER_SIZE) {
        s_ppg_buffer.count++;
    }
}

// Heart Rate Calculation (Peak Detection Algorithm)
/**
 * @brief Compute heart rate from the IR signal buffer.
 */
static uint8_t calculate_heart_rate(void)
{
    if (s_ppg_buffer.count < SAMPLE_BUFFER_SIZE) {
        return 0; // Not enough samples
    }

    // Find min/max in IR signal
    uint32_t min_val = UINT32_MAX, max_val = 0;
    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
        if (s_ppg_buffer.ir_samples[i] < min_val) min_val = s_ppg_buffer.ir_samples[i];
        if (s_ppg_buffer.ir_samples[i] > max_val) max_val = s_ppg_buffer.ir_samples[i];
    }

    if (max_val <= min_val || max_val < 10000) {
        return 0; // No valid signal
    }

    uint32_t threshold = min_val + (uint32_t)((max_val - min_val) * PEAK_THRESHOLD_RATIO);

    // Peak detection
    uint32_t last_peak = 0;
    bool above_threshold = false;

    for (int i = 1; i < SAMPLE_BUFFER_SIZE - 1; i++) {
        uint32_t curr = s_ppg_buffer.ir_samples[i];
        uint32_t prev = s_ppg_buffer.ir_samples[i - 1];
        uint32_t next = s_ppg_buffer.ir_samples[i + 1];

        if (curr > threshold && curr >= prev && curr >= next) {
            if (!above_threshold && (last_peak == 0 || (i - last_peak) >= MIN_PEAK_DISTANCE)) {
                if (last_peak > 0) {
                    uint32_t interval = i - last_peak;
                    s_ppg_buffer.peak_intervals[s_ppg_buffer.interval_index] = interval;
                    s_ppg_buffer.interval_index = (s_ppg_buffer.interval_index + 1) % 5;
                    if (s_ppg_buffer.interval_count < 5) s_ppg_buffer.interval_count++;
                }
                last_peak = i;
                above_threshold = true;
            }
        } else if (curr < threshold) {
            above_threshold = false;
        }
    }

    // Calculate average interval
    if (s_ppg_buffer.interval_count < 2) {
        return 0;
    }

    uint32_t avg_interval = 0;
    for (int i = 0; i < s_ppg_buffer.interval_count; i++) {
        avg_interval += s_ppg_buffer.peak_intervals[i];
    }
    avg_interval /= s_ppg_buffer.interval_count;

    if (avg_interval == 0) return 0;

    // Convert to BPM using actual sample rate
    uint32_t bpm = (SAMPLE_RATE_HZ * 60) / avg_interval;
    
    // Sanity check (30-220 BPM)
    if (bpm < 30 || bpm > 220) return 0;

    return (uint8_t)bpm;
}

// SpO2 Calculation (AC/DC Ratio Method)
/**
 * @brief Compute SpO2 using AC/DC ratio of RED and IR.
 */
static uint8_t calculate_spo2(void)
{
    if (s_ppg_buffer.count < SAMPLE_BUFFER_SIZE) {
        return 0;
    }

    // Calculate AC and DC components for RED and IR
    uint32_t red_min = UINT32_MAX, red_max = 0;
    uint32_t ir_min = UINT32_MAX, ir_max = 0;
    uint64_t red_sum = 0, ir_sum = 0;

    for (int i = 0; i < SAMPLE_BUFFER_SIZE; i++) {
        uint32_t red = s_ppg_buffer.red_samples[i];
        uint32_t ir = s_ppg_buffer.ir_samples[i];

        if (red < red_min) red_min = red;
        if (red > red_max) red_max = red;
        if (ir < ir_min) ir_min = ir;
        if (ir > ir_max) ir_max = ir;

        red_sum += red;
        ir_sum += ir;
    }

    uint32_t red_dc = (uint32_t)(red_sum / SAMPLE_BUFFER_SIZE);
    uint32_t ir_dc = (uint32_t)(ir_sum / SAMPLE_BUFFER_SIZE);
    uint32_t red_ac = red_max - red_min;
    uint32_t ir_ac = ir_max - ir_min;

    // Check for valid signal
    if (red_dc < 10000 || ir_dc < 10000 || red_ac == 0 || ir_ac == 0) {
        return 0;
    }

    // Calculate R value: (AC_red/DC_red) / (AC_ir/DC_ir)
    float r = ((float)red_ac / (float)red_dc) / ((float)ir_ac / (float)ir_dc);

    // Empirical formula (adjust based on calibration)
    // SpO2 = 110 - 25*R
    float spo2 = 110.0f - 25.0f * r;

    // Sanity check (70-100%)
    if (spo2 < 70.0f || spo2 > 100.0f) {
        return 0;
    }

    return (uint8_t)spo2;
}

// Background Sampling Task
/**
 * @brief FreeRTOS task that samples PPG data at 100 Hz.
 */
static void ppg_sampling_task(void *arg)
{
    ESP_LOGI(TAG, "PPG sampling task started");
    
    while (1) {
        uint32_t red = 0, ir = 0;
        esp_err_t err = max30101_read_sample(&red, &ir);
        
        if (err == ESP_OK && ir > 1000) { // Valid signal check
            ppg_add_sample(red, ir);
            
            // Store latest raw values
            s_latest_raw_red = red;
            s_latest_raw_ir = ir;
            
            // Update HR and SpO2 every 10 samples (~400 ms at 25 Hz)
            static uint8_t sample_count = 0;
            if (++sample_count >= 10) {
                sample_count = 0;
                s_latest_hr = calculate_heart_rate();
                s_latest_spo2 = calculate_spo2();
                
                if (s_latest_hr > 0 || s_latest_spo2 > 0) {
                    ESP_LOGD(TAG, "HR=%u SpO2=%u%%", s_latest_hr, s_latest_spo2);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS)); // 25 Hz sampling
    }
}

// Public API Implementation
/**
 * @brief Initialize MAX30101 I2C bus and configuration.
 */
esp_err_t max30101_init(void)
{
    // Initialize I2C bus
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = MAX30101_I2C_PORT,
        .scl_io_num = MAX30101_SCL_GPIO,
        .sda_io_num = MAX30101_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_max30101_bus), TAG, "I2C bus init failed");

    // Add MAX30101 device
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MAX30101_ADDR,
        .scl_speed_hz    = MAX30101_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_max30101_bus, &dev_cfg, &s_max30101), TAG, "Device add failed");

    // Configure INIT pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MAX30101_INIT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "GPIO config failed");
    gpio_set_level(MAX30101_INIT_GPIO, 1); // Enable sensor
    vTaskDelay(pdMS_TO_TICKS(10));

    // Soft reset
    ESP_RETURN_ON_ERROR(max30101_write(REG_MODE_CONFIG, 0x40), TAG, "Reset failed");
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure interrupts
    max30101_write(REG_INTR_ENABLE_1, 0x40); // PPG_RDY
    max30101_write(REG_INTR_ENABLE_2, 0x00);

    // FIFO configuration
    max30101_write(REG_FIFO_CONFIG, 0x00);
    max30101_write(REG_FIFO_WR_PTR, 0x00);
    max30101_write(REG_OVF_COUNTER, 0x00);
    max30101_write(REG_FIFO_RD_PTR, 0x00);

    // SpO2 mode (RED + IR)
    max30101_write(REG_MODE_CONFIG, 0x03);

    // SpO2 config: ADC=4096nA, 25 sps (bits[4:2]=0b001), 411 us pulse width
    // REG_SPO2_CONFIG: [6:5]=ADC range 4096nA, [4:2]=sample rate, [1:0]=pulse width 411us
    // 0x27 = 0b00_100_11 → ADC=4096nA, 25sps, 411us  (was 0x3B = 100sps)
    max30101_write(REG_SPO2_CONFIG, 0x27);

    // LED currents — reduced from 0x24 (~7.2 mA) to 0x0F (~4.7 mA) to save power.
    // Adequate for finger/wrist contact; increase to 0x1F if signal is too weak.
    max30101_write(REG_LED1_PA, 0x0F); // RED
    max30101_write(REG_LED2_PA, 0x0F); // IR

    ESP_LOGI(TAG, "MAX30101 initialized successfully");
    return ESP_OK;
}

/**
 * @brief Start background PPG sampling task.
 */
esp_err_t max30101_start_sampling(void)
{
    BaseType_t ret = xTaskCreate(ppg_sampling_task, "ppg_sample", 4096, NULL, 5, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sampling task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Return the latest computed heart rate.
 */
uint8_t max30101_get_heart_rate(void)
{
    return s_latest_hr;
}

/**
 * @brief Return the latest computed SpO2 value.
 */
uint8_t max30101_get_spo2(void)
{
    return s_latest_spo2;
}

/**
 * @brief Return the latest raw RED/IR values.
 */
void max30101_get_raw_values(uint32_t *red, uint32_t *ir)
{
    if (red) *red = s_latest_raw_red;
    if (ir) *ir = s_latest_raw_ir;
}

#endif // ENABLE_MAX30101
