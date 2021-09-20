/*
Copyright 2021 Adam Honse <calcprogrammer1@gmail.com>
Copyright 2011 Jun Wako <wakojun@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Ported to QMK by Stephen Peery <https://github.com/smp4488/>
*/

// Key and LED matrix driver for SN32F260.
// This driver will take full control of CT16B1 and GPIO in MATRIX_ROW_PINS, MATRIX_COL_PINS, LED_MATRIX_ROW_PINS and LED_MATRIX_COL_PINS.

#include <stdint.h>
#include <stdbool.h>
#include <string.h> // memset()
#include <SN32F260.h>
#include "ch.h"
#include "hal.h"
#include "quantum.h"
#include "matrix.h"
#include "debounce.h"

#if LED_MATRIX_ENABLE
#include "led_matrix.h"
#endif

#if RGB_MATRIX_ENABLE
#include "rgb_matrix.h"
#endif


// TODO fix CT16.h include for SN32F260, The one found in SN32F260_Startkit_Package_V1.6R is not complete
//#include "CT16.h"
#define mskCT16_PWM0EN_EN    (1<<0)
#define mskCT16_PWM1EN_EN    (1<<1)
#define mskCT16_PWM2EN_EN    (1<<2)
#define mskCT16_PWM3EN_EN    (1<<3)
#define mskCT16_PWM4EN_EN    (1<<4)
#define mskCT16_PWM5EN_EN    (1<<5)
#define mskCT16_PWM6EN_EN    (1<<6)
#define mskCT16_PWM7EN_EN    (1<<7)
#define mskCT16_PWM8EN_EN    (1<<8)
#define mskCT16_PWM9EN_EN    (1<<9)
#define mskCT16_PWM10EN_EN   (1<<10)
#define mskCT16_PWM11EN_EN   (1<<11)
#define mskCT16_PWM12EN_EN   (1<<12)
#define mskCT16_PWM13EN_EN   (1<<13)
#define mskCT16_PWM14EN_EN   (1<<14)
#define mskCT16_PWM15EN_EN   (1<<15)
#define mskCT16_PWM16EN_EN   (1<<16)
#define mskCT16_PWM17EN_EN   (1<<17)
#define mskCT16_PWM18EN_EN   (1<<18)
#define mskCT16_PWM19EN_EN   (1<<19)
#define mskCT16_PWM20EN_EN   (1<<20)
#define mskCT16_PWM21EN_EN   (1<<21)
#define mskCT16_PWM22EN_EN   (1<<22)
#define mskCT16_PWM23EN_EN   (1<<23)

#define SN_CT16B1_MCTRL3_MR23IE_Pos       (9UL)                     /*!< MR23IE (Bit 9)                                        */
#define SN_CT16B1_MCTRL3_MR23IE_Msk       (0x200UL)                 /*!< MR23IE (Bitfield-Mask: 0x01)                          */
#define SN_CT16B1_MCTRL3_MR23RST_Pos      (10UL)                    /*!< MR23RST (Bit 10)                                      */
#define SN_CT16B1_MCTRL3_MR23RST_Msk      (0x400UL)                 /*!< MR23RST (Bitfield-Mask: 0x01)                         */
#define SN_CT16B1_MCTRL3_MR23STOP_Pos     (11UL)                    /*!< MR23STOP (Bit 11)                                     */
#define SN_CT16B1_MCTRL3_MR23STOP_Msk     (0x800UL)                 /*!< MR23STOP (Bitfield-Mask: 0x01)                        */

#define SN_CT16B1_IC_MR23IC_Pos           (23UL)                    /*!< MR23IC (Bit 23)                                       */
#define SN_CT16B1_IC_MR23IC_Msk           (0x800000UL)              /*!< MR23IC (Bitfield-Mask: 0x01)                          */

#define mskCT16_MR23IE_EN       SN_CT16B1_MCTRL3_MR23IE_Msk
#define mskCT16_MR23STOP_EN     SN_CT16B1_MCTRL3_MR23STOP_Msk
#define mskCT16_MR23IC          SN_CT16B1_IC_MR23IC_Msk

#define mskCT16_CRST					(1<<1)	
#define	mskCT16_CEN_EN  				(1<<0)



static const pin_t row_pins[MATRIX_ROWS] = MATRIX_ROW_PINS;
static const pin_t col_pins[MATRIX_COLS] = MATRIX_COL_PINS;
static const pin_t led_row_pins[LED_MATRIX_ROWS_HW] = LED_MATRIX_ROW_PINS;
static const pin_t led_col_pins[LED_MATRIX_COLS] = LED_MATRIX_COL_PINS;
static uint8_t mr_offset[22] = {0};
static uint32_t pwm_en_msk = 0;

matrix_row_t raw_matrix[MATRIX_ROWS]; //raw values
matrix_row_t last_matrix[MATRIX_ROWS] = {0};  // raw values
matrix_row_t matrix[MATRIX_ROWS]; //debounced values

static bool matrix_changed = false;
static uint8_t current_row = 0;

/* Fake RGB Led matrix driver - Work in progress */
/* Only the red channel is used. RGB matrix is currently better supported then LED matrix */
#if RGB_MATRIX_ENABLE

uint8_t led_state[DRIVER_LED_TOTAL];

#define LED_STATE(row, mr) get_led_state(row, mr)

static inline int get_led_state(int row, int mr){
    int led_id = g_led_config.matrix_co[row][mr_offset[mr]];
    if(led_id == NO_LED) return 0;

    return led_state[led_id];
}

void init(void) {
    // NOP
}

static void flush(void){
    // NOP
}

void set_color(int index, uint8_t r, uint8_t g, uint8_t b) {
    led_state[index] = r;
}

static void set_color_all(uint8_t r, uint8_t g, uint8_t b) {
    memset(led_state, r, sizeof(led_state));
}

const rgb_matrix_driver_t rgb_matrix_driver = {
    .init          = init,
    .flush         = flush,
    .set_color     = set_color,
    .set_color_all = set_color_all,
};

// Small rand() implementation
int rand(void)
{
   // static unsigned int z4;
   unsigned int z1, z2, z3, z4;
   int r;
   unsigned int b;

   z1 = timer_read32();
   z2 = 12345;
   z3 = 12345;
   z4 = z1 << 8;

   b  = ((z1 << 6) ^ z1) >> 13;
   z1 = ((z1 & 4294967294U) << 18) ^ b;
   b  = ((z2 << 2) ^ z2) >> 27;
   z2 = ((z2 & 4294967288U) << 2) ^ b;
   b  = ((z3 << 13) ^ z3) >> 21;
   z3 = ((z3 & 4294967280U) << 7) ^ b;
   b  = ((z4 << 3) ^ z4) >> 12;
   z4 = ((z4 & 4294967168U) << 13) ^ b;

   r = z1 ^ z2 ^ z3 ^ z4;
   // z4 = r;

   return r;
}


#endif

/* Led matrix driver - Work in progress */
#if LED_MATRIX_ENABLE

#define LED_STATE(row, mr) (led_state[LED_MATRIX_COLS * row + mr_offset[mr]])

uint8_t led_state[LED_MATRIX_ROWS * LED_MATRIX_COLS];

static void init(void){
    // NOP
}

static void flush(void){
    // NOP
}

static void set_value(int index, uint8_t value){
    // Not implemented
    // TODO index to row,col position
    led_state[0] = value;
}

static void set_value_all(uint8_t value){
    memset(led_state, value, sizeof(led_state));
}

const led_matrix_driver_t led_matrix_driver = {
    .init           = init,
    .flush          = flush,
    .set_value      = set_value,
    .set_value_all  = set_value_all,
};

#endif

/* Uniform backlight driver */
#if BACKLIGHT_ENABLE && !LED_MATRIX_ENABLE

uint8_t led_all_value;

#define LED_STATE(row, mr) (led_all_value)

void backlight_set(uint8_t level) {
    led_all_value = 255 * level / BACKLIGHT_LEVELS;
}

#endif

__attribute__((weak)) void matrix_init_kb(void) { matrix_init_user(); }

__attribute__((weak)) void matrix_scan_kb(void) { matrix_scan_user(); }

__attribute__((weak)) void matrix_init_user(void) {}

__attribute__((weak)) void matrix_scan_user(void) {}

inline matrix_row_t matrix_get_row(uint8_t row) { return matrix[row]; }

void matrix_print(void) {}

static void init_pins(void) {

#if(DIODE_DIRECTION == ROW2COL)
    //  Unselect ROWs
    for (uint8_t x = 0; x < MATRIX_ROWS; x++) {
        setPinInputHigh(row_pins[x]);
    }

    // Unselect COLs
    for (uint8_t x = 0; x < MATRIX_COLS; x++) {
        setPinOutput(col_pins[x]);
        writePinHigh(col_pins[x]);
    }

#elif(DIODE_DIRECTION == COL2ROW)
    //  Unselect ROWs
    for (uint8_t x = 0; x < MATRIX_ROWS; x++) {
        setPinOutput(row_pins[x]);
        writePinHigh(row_pins[x]);
    }

    // Unselect COLs
    for (uint8_t x = 0; x < MATRIX_COLS; x++) {
        setPinInputHigh(col_pins[x]);
    }
#else
#error DIODE_DIRECTION must be one of COL2ROW or ROW2COL!
#endif

    // MATRIX_COL_PINS and  LED_MATRIX_COL_PINS can be the same, in that case the earlier configuration is overrule by this one.
    for (uint8_t x = 0; x < LED_MATRIX_COLS; x++) {
        setPinOutput(led_col_pins[x]);
        writePinHigh(led_col_pins[x]);
    }

    for (uint8_t x = 0; x < LED_MATRIX_ROWS_HW; x++) {
        setPinOutput(led_row_pins[x]);
        writePinLow(led_row_pins[x]);
    }
}

void matrix_init(void) {
    // initialize key pins
    init_pins();

    // initialize matrix state: all keys off
    for (uint8_t i = 0; i < MATRIX_ROWS; i++) {
        raw_matrix[i] = 0;
        matrix[i]     = 0;
    }

    debounce_init(MATRIX_ROWS);

    matrix_init_quantum();

    // Enable Timer Clock
    SN_SYS1->AHBCLKEN_b.CT16B1CLKEN = 1;

    pwm_en_msk = 0;
    
    // Enable PWM function, IOs and select the PWM modes for the LED column pins
    for(uint8_t i = 0; i < LED_MATRIX_COLS; i++) {
        switch(led_col_pins[i]) {
            case A0:
                pwm_en_msk |= mskCT16_PWM0EN_EN;
                mr_offset[0] = i;
                break;
            case A1:
                pwm_en_msk |= mskCT16_PWM1EN_EN;
                mr_offset[1] = i;
                break;
            case A2:
                pwm_en_msk |= mskCT16_PWM2EN_EN;
                mr_offset[2] = i;
                break;
            case A3:
                pwm_en_msk |= mskCT16_PWM3EN_EN;
                mr_offset[3] = i;
                break;
            case A4:
                pwm_en_msk |= mskCT16_PWM4EN_EN;
                mr_offset[4] = i;
                break;
            case A5:
                pwm_en_msk |= mskCT16_PWM5EN_EN;
                mr_offset[5] = i;
                break;
            case A6:
                pwm_en_msk |= mskCT16_PWM6EN_EN;
                mr_offset[6] = i;
                break;
            case A7:
                pwm_en_msk |= mskCT16_PWM7EN_EN;
                mr_offset[7] = i;
                break;
            case A8:
                pwm_en_msk |= mskCT16_PWM8EN_EN;
                mr_offset[8] = i;
                break;
            case A9:
                pwm_en_msk |= mskCT16_PWM9EN_EN;
                mr_offset[9] = i;
                break;
            case A10:
                pwm_en_msk |= mskCT16_PWM10EN_EN;
                mr_offset[10] = i;
                break;
            case A11:
                pwm_en_msk |= mskCT16_PWM11EN_EN;
                mr_offset[11] = i;
                break;
            case A12:
                pwm_en_msk |= mskCT16_PWM12EN_EN;
                mr_offset[12] = i;
                break;
            case A13:
                pwm_en_msk |= mskCT16_PWM13EN_EN;
                mr_offset[13] = i;
                break;
            case A14:
                pwm_en_msk |= mskCT16_PWM14EN_EN;
                mr_offset[14] = i;
                break;
            case A15:
                pwm_en_msk |= mskCT16_PWM15EN_EN;
                mr_offset[15] = i;
                break;
            case D0:
                pwm_en_msk |= mskCT16_PWM16EN_EN;
                mr_offset[16] = i;
                break;
            case D1:
                pwm_en_msk |= mskCT16_PWM17EN_EN;
                mr_offset[17] = i;
                break;
            case D2:
                pwm_en_msk |= mskCT16_PWM18EN_EN;
                mr_offset[18] = i;
                break;
            case D3:
                pwm_en_msk |= mskCT16_PWM19EN_EN;
                mr_offset[19] = i;
                break;
            case D4:
                pwm_en_msk |= mskCT16_PWM20EN_EN;
                mr_offset[20] = i;
                break;
            case D5:
                pwm_en_msk |= mskCT16_PWM21EN_EN;
                mr_offset[21] = i;
                break;
        }
    }

    // Set PWM mode 1 for LED colom pins. PWMCTRL, PWMCTRL1 and PWMCTRL2 don't need to be set, there reset values are zero.
    SN_CT16B1->PWMENB = pwm_en_msk;

    // Set match interrupts and TC rest
    SN_CT16B1->MCTRL3 = (mskCT16_MR23IE_EN | mskCT16_MR23STOP_EN);

    // COL match register
    SN_CT16B1->MR23 = 0xFF;

    // Set prescale value
    SN_CT16B1->PRE = 0x02;

    //Set CT16B1 as the up-counting mode.
	SN_CT16B1->TMRCTRL = (mskCT16_CRST);

    // Wait until timer reset done.
    while (SN_CT16B1->TMRCTRL & mskCT16_CRST);

    // Let TC start counting.
    SN_CT16B1->TMRCTRL |= mskCT16_CEN_EN;

    NVIC_ClearPendingIRQ(CT16B1_IRQn);
    nvicEnableVector(CT16B1_IRQn, 0);
}

uint8_t matrix_scan(void) {
    matrix_changed = false;
    for (uint8_t current_col = 0; current_col < MATRIX_COLS; current_col++) {
        for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++) {
            // Determine if the matrix changed state
            if ((last_matrix[row_index] != raw_matrix[row_index])) {
                matrix_changed         = true;
                last_matrix[row_index] = raw_matrix[row_index];
            }
        }
    }

    debounce(raw_matrix, matrix, MATRIX_ROWS, matrix_changed);

    matrix_scan_quantum();

    return matrix_changed;
}

/**
 * @brief   CT16B1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_CT16B1_HANDLER) {

    OSAL_IRQ_PROLOGUE();

    chSysDisable();

    // Clear match interrupt status
    SN_CT16B1->IC = mskCT16_MR23IC; 

    // Turn the selected row off
    writePinLow(led_row_pins[current_row]);

    // Disable PWM outputs on column pins
    SN_CT16B1->PWMIOENB = 0;

    // Move to the next row
    current_row = (current_row + 1) % LED_MATRIX_ROWS_HW;

    if(current_row == 0) {

#if(DIODE_DIRECTION == ROW2COL)
        // Read the key matrix
        for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
            // Enable the column
            writePinLow(col_pins[col_index]);

            for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++) {
                // Check row pin state
                if (readPin(row_pins[row_index]) == 0) {
                    // Pin LO, set col bit
                    raw_matrix[row_index] |= (MATRIX_ROW_SHIFTER << col_index);
                } else {
                    // Pin HI, clear col bit
                    raw_matrix[row_index] &= ~(MATRIX_ROW_SHIFTER << col_index);
                }
            }

            // Disable the column
            writePinHigh(col_pins[col_index]);
        }

#elif(DIODE_DIRECTION == COL2ROW)
        // MATRIX_COL_PINS and LED_MATRIX_COL_PINS can be the same. So configure MATRIX_COL_PINS as input, but this might mess-up the LED_MATRIX_COL_PINS.
        for (uint8_t x = 0; x < MATRIX_COLS; x++) {
            setPinInputHigh(col_pins[x]);
        }

        // Read the key matrix
        for (uint8_t row_index = 0; row_index < MATRIX_ROWS; row_index++) {
            // Enable the row
            writePinLow(row_pins[row_index]);

            for (uint8_t col_index = 0; col_index < MATRIX_COLS; col_index++) {
                // Check row pin state
                if (readPin(col_pins[col_index]) == 0) {
                    // Pin LO, set col bit
                    raw_matrix[row_index] |= (MATRIX_ROW_SHIFTER << col_index);
                } else {
                    // Pin HI, clear col bit
                    raw_matrix[row_index] &= ~(MATRIX_ROW_SHIFTER << col_index);
                }
            }

            // Disable the row
            writePinHigh(row_pins[row_index]);
        }

        // Done with MATRIX_COL_PINS, re-init LED_MATRIX_COL_PINS that might have been messed-up.
        for (uint8_t x = 0; x < LED_MATRIX_COLS; x++) {
            setPinOutput(led_col_pins[x]);
            writePinHigh(led_col_pins[x]);
        }

#endif

    }

    //Set CT16B1 as the up-counting mode.
    SN_CT16B1->TMRCTRL = (mskCT16_CRST);

    // Wait until timer reset done.
    while (SN_CT16B1->TMRCTRL & mskCT16_CRST);

    SN_CT16B1->MR0  = LED_STATE(current_row, 0);
    SN_CT16B1->MR1  = LED_STATE(current_row, 1);
    SN_CT16B1->MR2  = LED_STATE(current_row, 2);
    SN_CT16B1->MR3  = LED_STATE(current_row, 3);
    SN_CT16B1->MR4  = LED_STATE(current_row, 4);
    SN_CT16B1->MR5  = LED_STATE(current_row, 5);
    SN_CT16B1->MR6  = LED_STATE(current_row, 6);
    SN_CT16B1->MR7  = LED_STATE(current_row, 7);
    SN_CT16B1->MR8  = LED_STATE(current_row, 8);
    SN_CT16B1->MR9  = LED_STATE(current_row, 9);
    SN_CT16B1->MR10 = LED_STATE(current_row, 10);
    SN_CT16B1->MR11 = LED_STATE(current_row, 11);
    SN_CT16B1->MR12 = LED_STATE(current_row, 12);
    SN_CT16B1->MR13 = LED_STATE(current_row, 13);
    SN_CT16B1->MR14 = LED_STATE(current_row, 14);
    SN_CT16B1->MR15 = LED_STATE(current_row, 15);
    SN_CT16B1->MR16 = LED_STATE(current_row, 16);
    SN_CT16B1->MR17 = LED_STATE(current_row, 17);
    SN_CT16B1->MR18 = LED_STATE(current_row, 18);
    SN_CT16B1->MR19 = LED_STATE(current_row, 19);
    SN_CT16B1->MR20 = LED_STATE(current_row, 20);
    SN_CT16B1->MR21 = LED_STATE(current_row, 21);

    uint32_t new_pwm_en = 0;
    if(SN_CT16B1->MR0 > 0)  new_pwm_en |= mskCT16_PWM0EN_EN;
    if(SN_CT16B1->MR1 > 0)  new_pwm_en |= mskCT16_PWM1EN_EN;
    if(SN_CT16B1->MR2 > 0)  new_pwm_en |= mskCT16_PWM2EN_EN;
    if(SN_CT16B1->MR3 > 0)  new_pwm_en |= mskCT16_PWM3EN_EN;
    if(SN_CT16B1->MR4 > 0)  new_pwm_en |= mskCT16_PWM4EN_EN;
    if(SN_CT16B1->MR5 > 0)  new_pwm_en |= mskCT16_PWM5EN_EN;
    if(SN_CT16B1->MR6 > 0)  new_pwm_en |= mskCT16_PWM6EN_EN;
    if(SN_CT16B1->MR7 > 0)  new_pwm_en |= mskCT16_PWM7EN_EN;
    if(SN_CT16B1->MR8 > 0)  new_pwm_en |= mskCT16_PWM8EN_EN;
    if(SN_CT16B1->MR9 > 0)  new_pwm_en |= mskCT16_PWM9EN_EN;
    if(SN_CT16B1->MR10 > 0) new_pwm_en |= mskCT16_PWM10EN_EN;
    if(SN_CT16B1->MR11 > 0) new_pwm_en |= mskCT16_PWM11EN_EN;
    if(SN_CT16B1->MR12 > 0) new_pwm_en |= mskCT16_PWM12EN_EN;
    if(SN_CT16B1->MR13 > 0) new_pwm_en |= mskCT16_PWM13EN_EN;
    if(SN_CT16B1->MR14 > 0) new_pwm_en |= mskCT16_PWM14EN_EN;
    if(SN_CT16B1->MR15 > 0) new_pwm_en |= mskCT16_PWM15EN_EN;
    if(SN_CT16B1->MR16 > 0) new_pwm_en |= mskCT16_PWM16EN_EN;
    if(SN_CT16B1->MR17 > 0) new_pwm_en |= mskCT16_PWM17EN_EN;
    if(SN_CT16B1->MR18 > 0) new_pwm_en |= mskCT16_PWM18EN_EN;
    if(SN_CT16B1->MR19 > 0) new_pwm_en |= mskCT16_PWM19EN_EN;
    if(SN_CT16B1->MR20 > 0) new_pwm_en |= mskCT16_PWM20EN_EN;
    if(SN_CT16B1->MR21 > 0) new_pwm_en |= mskCT16_PWM21EN_EN;
 
    // Only enable PWM IO's that have a duty cycle > 0%
    SN_CT16B1->PWMIOENB = pwm_en_msk & new_pwm_en;

    // Set match interrupts and TC rest
    SN_CT16B1->MCTRL3 = (mskCT16_MR23IE_EN | mskCT16_MR23STOP_EN);

    // Turn the current row on
    writePinHigh(led_row_pins[current_row]);

    // Let TC start counting.
    SN_CT16B1->TMRCTRL |= mskCT16_CEN_EN;

    chSysEnable();

    OSAL_IRQ_EPILOGUE();
}
