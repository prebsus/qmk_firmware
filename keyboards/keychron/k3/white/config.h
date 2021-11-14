/* Copyright 2020 Adam Honse <calcprogrammer1@gmail.com>
 * Copyright 2020 Dimitris Mantzouranis <d3xter93@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "config_common.h"

/* USB Device descriptor parameter */
#define VENDOR_ID                   0x05AC
#define PRODUCT_ID                  0x024F
#define DEVICE_VER                  0x0001

#define MANUFACTURER    Keychron
#define PRODUCT         K3                        // Updated based on K3 RBG
#define DESCRIPTION     K3 Optical White Hotswap  // Updated based on K3 RBG

#define WAIT_FOR_USB                              // Not in K3 RBG - include?
#define USB_MAX_POWER_CONSUMPTION   100           // Not in K3 RBG - include?

/* key matrix size */
#define MATRIX_ROWS                 6             // Updated based on K3 RBG
#define MATRIX_COLS                 16            // Updated based on K3 RBG

#define DIODE_DIRECTION             COL2ROW

#define MATRIX_COL_PINS             { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15 }  // Updated based on my mapping/tracing
#define MATRIX_ROW_PINS             { C4, C5, C6, C7, C8 }                                                    // Updated based on my mapping/tracing

/* LED matrix */                                  // NEED TO TRACE OUT AND UPDATE
#define LED_MATRIX_ROWS             MATRIX_ROWS
#define LED_MATRIX_ROW_CHANNELS     1
#define LED_MATRIX_ROWS_HW          (LED_MATRIX_ROWS * LED_MATRIX_ROW_CHANNELS)
#define LED_MATRIX_ROW_PINS         { C0, C1, C2, D4, C9, C10 }

#define LED_MATRIX_COLS             MATRIX_COLS
#define LED_MATRIX_COL_PINS         { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, D0 }

#define DRIVER_LED_TOTAL            87

/* Backlight configuration */
#define RGB_MATRIX_VAL_STEP             32      // Not in K3 RBG - include?
#define RGB_DISABLE_WHEN_USB_SUSPENDED  true    // Not in K3 RBG - include?

/* Connects each switch in the dip switch to the GPIO pin of the MCU */
#define DIP_SWITCH_PINS             { D7 }      // NEED TO FIGURE OUT WHICH THIS IS FOR MY MCU

/* Debounce reduces chatter (unintended double-presses) - set 0 if debouncing is not needed */
#define DEBOUNCE                    5           // 0 in K3 RGB, can always change later....

/* LED Status indicators */
#define LED_CAPS_LOCK_PIN           B3        // NEED TO FIGURE OUT WHICH THIS IS FOR MY MCU
#define LED_PIN_ON_STATE            1         // NEED TO FIGURE OUT WHICH THIS IS FOR MY MCU

#define LED_MAC_PIN                 B4        // NEED TO FIGURE OUT WHICH THIS IS FOR MY MCU
#define LED_WIN_PIN                 B5        // NEED TO FIGURE OUT WHICH THIS IS FOR MY MCU

/* Enable NKRO by default */
#define FORCE_NKRO

/* Disable the following animation because they are not interesting in monochrome */
#define DISABLE_RGB_MATRIX_ALPHAS_MODS        // THIS SECTION I CAN PROBABLY KEEP AS SAME
#define DISABLE_RGB_MATRIX_GRADIENT_UP_DOWN
#define DISABLE_RGB_MATRIX_GRADIENT_LEFT_RIGHT
#define DISABLE_RGB_MATRIX_BAND_SAT
#define DISABLE_RGB_MATRIX_BAND_PINWHEEL_SAT
#define DISABLE_RGB_MATRIX_BAND_SPIRAL_SAT
#define DISABLE_RGB_MATRIX_CYCLE_ALL
#define DISABLE_RGB_MATRIX_RAINBOW_MOVING_CHEVRON
#define DISABLE_RGB_MATRIX_DUAL_BEACON

/* TODO: Disabled the following animation because they crash the keyboard. Probably due to a too small stack */
#define DISABLE_RGB_MATRIX_RAINDROPS
#define DISABLE_RGB_MATRIX_JELLYBEAN_RAINDROPS
