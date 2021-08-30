/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef MCUCONF_H
#define MCUCONF_H

/*
 * SN32F26x drivers configuration.
 * The following settings override the default settings present in
 * the various device driver implementation headers.
 * Note that the settings for each driver only have effect if the whole
 * driver is enabled in halconf.h.
 *
 * IRQ priorities:
 * 3...0       Lowest...Highest.
 *
 * DMA priorities:
 * 0...3        Lowest...Highest.
 */

#define SN32F26x_MCUCONF
#define PLATFORM_MCUCONF

/*
 * HAL driver system settings.
 */

/*
 * SN driver system settings.
 */
#define SN32_HAS_GPIOA TRUE
#define SN32_HAS_GPIOB TRUE
#define SN32_HAS_GPIOC TRUE
#define SN32_HAS_GPIOD TRUE

// Disable reset and SWD, so they can be used as GPIO
#define SN32_PAL_DISABLE_RESET  TRUE
#define SN32_PAL_DISABLE_SWD    TRUE

/*
 * USB driver system settings.
 */
#define CRT1_AREAS_NUMBER 1
#define PLATFORM_USB_USE_USB1 TRUE

/*
 * Timer driver system settings.
 */
// Defaults are correct


#endif /* MCUCONF_H */
