/* board.h - Board-specific hooks */

/**
 * @file board.h
 * @brief Board-specific LED and GPIO helpers.
 */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif /**< __cplusplus */

#include "driver/gpio.h"

// RGB LED pin definitions (ESP32-S3-DevKitC-1)
#define LED_RED   GPIO_NUM_18
#define LED_GREEN GPIO_NUM_8
#define LED_BLUE  GPIO_NUM_3

// Legacy aliases for compatibility
#define LED_R     LED_RED
#define LED_G     LED_GREEN
#define LED_B     LED_BLUE

#define LED_ON    1
#define LED_OFF   0

/**
 * @brief Turn RGB LED on or off.
 * 
 * @param[in] pin   GPIO pin number (use LED_RED, LED_GREEN, or LED_BLUE)
 * @param[in] onoff LED state (LED_ON or LED_OFF)
 */
void board_led_operation(uint8_t pin, uint8_t onoff);

/**
 * @brief Initialize board-level hardware.
 */
void board_init(void);

#ifdef __cplusplus
}
#endif /**< __cplusplus */

#endif /* _BOARD_H_ */
