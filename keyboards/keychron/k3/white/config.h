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
/* NB! This matches what I see in Sonix flasher, so even though it's the same as C1 it should be ok, right? */
#define VENDOR_ID                   0x05AC  // OK
#define PRODUCT_ID                  0x024F
#define DEVICE_VER                  0x0001  // OK

/* NB! not sure what DESCRIPTION does here, but this should be ok... */
#define MANUFACTURER    Keychron
#define PRODUCT         K3
#define DESCRIPTION     K3 Optical White

/* NB! not in K3 rgb or K7 optical, but seems reasonable to add... */
#define WAIT_FOR_USB  // Forces the keyboard to wait for a USB connection to be established before it starts up
#define USB_MAX_POWER_CONSUMPTION   100  // sets the maximum power (in mA) over USB for the device (default: 500)

/* key matrix size */
#define MATRIX_ROWS                 6  // OK
#define MATRIX_COLS                 16  // OK

#define DIODE_DIRECTION             COL2ROW  // OK

#define MATRIX_COL_PINS             { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15 }  // OK
#define MATRIX_ROW_PINS             { C4, C5, C6, C7, C8 }  // OK

/* LED matrix */
/*  NB! NEED TO TRACE OUT AND UPDATE, this is in config_led.h for K7 Optical bringup */
#define LED_MATRIX_ROWS             MATRIX_ROWS  // OK
#define LED_MATRIX_ROW_CHANNELS     1  // OK, for white (presumably - have q for IsaacDynamo)
#define LED_MATRIX_ROWS_HW          (LED_MATRIX_ROWS * LED_MATRIX_ROW_CHANNELS)  // OK
#define LED_MATRIX_ROW_PINS         { C0, C1, C2, D4, C9, C10 }  // NEED TO FIGURE OUT WHICH THIS IS FOR MY MCU

#define LED_MATRIX_COLS             MATRIX_COLS  // OK
#define LED_MATRIX_COL_PINS         { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, D0 }  // NEED TO FIGURE OUT WHICH THIS IS FOR MY MCU

#define DRIVER_LED_TOTAL            87  // OK

/* Backlight configuration */
#define RGB_MATRIX_VAL_STEP             32  // See question for IsaacDynamo
#define RGB_DISABLE_WHEN_USB_SUSPENDED  true  // OK This just turn LEDs off when computer goes to sleep
/* NB! Check everything on LED above this */

/* Connects each switch in the dip switch to the GPIO pin of the MCU */
/* NB! NEED TO FIGURE OUT WHICH THIS IS FOR MY MCU (should be same, not included for K3 rgb or K7 rgb) */
#define DIP_SWITCH_PINS             { D7 }  // NEED TO FIGURE OUT WHICH THIS IS FOR MY MCU

/* Debounce reduces chatter (unintended double-presses) - set 0 if debouncing is not needed (was 5 for C1)*/
#define DEBOUNCE                    0  // OK

/* LED Status indicators */
#define LED_CAPS_LOCK_PIN           B3        // NEED TO FIGURE OUT WHICH THIS IS FOR MY MCU
#define LED_PIN_ON_STATE            1         // NEED TO FIGURE OUT WHICH THIS IS FOR MY MCU

#define LED_MAC_PIN                 B4        // NEED TO FIGURE OUT WHICH THIS IS FOR MY MCU
#define LED_WIN_PIN                 B5        // NEED TO FIGURE OUT WHICH THIS IS FOR MY MCU

/* Enable NKRO by default */
#define FORCE_NKRO  // OK

/* Disable the following animation because they are not interesting in monochrome */
/* NB! THIS SECTION I CAN PROBABLY KEEP AS SAME */
#define DISABLE_RGB_MATRIX_ALPHAS_MODS
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


/* Optical Matrix to swap pin status (taken from K7 optical bringup) */
#if defined(OPTICAL_MATRIX)
#define PRESSED_KEY_PIN_STATE 1
#define SKIP_THIS_NOF_MATRIX_SCANS 1
#define DELAY_ENABLE 1
#define SCAN_ON_EXTRA_ROW 1
#endif