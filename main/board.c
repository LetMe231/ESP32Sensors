/* board.c - Board-specific hardware initialization and control */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"

/** @brief Log tag for board module. */
#define TAG "BOARD"

/** @brief Number of RGB LEDs. */
#define LED_COUNT 3

/** @brief LED state tracking structure with color information. */
typedef struct {
    uint32_t pin;        /**< GPIO pin number */
    const char *color;   /**< Color name */
    uint8_t state;       /**< Current LED state (on/off) */
} led_state_t;

/** @brief RGB LED array. */
static led_state_t rgb_leds[LED_COUNT] = {
    { LED_RED,   "red",   LED_OFF },
    { LED_GREEN, "green", LED_OFF },
    { LED_BLUE,  "blue",  LED_OFF },
};

/**
 * @brief Set LED GPIO level and track state change.
 * 
 * @param[in] pin   GPIO pin number
 * @param[in] onoff LED state (LED_ON = 1 or LED_OFF = 0)
 */
void board_led_operation(uint8_t pin, uint8_t onoff)
{
    for (int i = 0; i < LED_COUNT; i++) {
        if (rgb_leds[i].pin != pin) {
            continue;
        }

        // Skip redundant state changes
        if (onoff == rgb_leds[i].state) {
            ESP_LOGD(TAG, "LED %s already %s",
                     rgb_leds[i].color, (onoff ? "on" : "off"));
            return;
        }

        gpio_set_level((gpio_num_t)pin, onoff);
        rgb_leds[i].state = onoff;
        ESP_LOGD(TAG, "LED %s turned %s",
                 rgb_leds[i].color, (onoff ? "on" : "off"));
        return;
    }

    ESP_LOGW(TAG, "LED pin 0x%02x not found in LED table", pin);
}

/**
 * @brief Initialize RGB LED GPIO pins as outputs.
 */
static void board_led_init(void)
{
    for (int i = 0; i < LED_COUNT; i++) {
        gpio_reset_pin((gpio_num_t)rgb_leds[i].pin);
        gpio_set_direction((gpio_num_t)rgb_leds[i].pin, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)rgb_leds[i].pin, LED_OFF);
        rgb_leds[i].state = LED_OFF;
        ESP_LOGD(TAG, "Initialized %s LED on pin %lu", rgb_leds[i].color, rgb_leds[i].pin);
    }
}

/**
 * @brief Initialize board-level hardware (LEDs, etc.).
 */
void board_init(void)
{
    board_led_init();
    ESP_LOGI(TAG, "Board initialized successfully");
}
