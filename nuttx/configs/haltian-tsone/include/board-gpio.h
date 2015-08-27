/************************************************************************************
 * configs/haltian-tsone/include/board-gpio.h
 * include/arch/board/board-gpio.h
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *
 * Authors:
 *   Sami Pelkonen <sami.pelkonen@haltian.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_GPIO_H
#define __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/chip/chip.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

#define BOARD_NGPIOS             STM32_NGPIO
#define BOARD_GPIO_PORTS         ((STM32_NGPIO + 15) >> 4)

/* Each port bit of the general-purpose I/O (GPIO) ports can be individually configured
 * by software in several modes:
 *
 *  - Input floating
 *  - Input pull-up
 *  - Input-pull-down
 *  - Output open-drain with pull-up or pull-down capability
 *  - Output push-pull with pull-up or pull-down capability
 *  - Alternate function push-pull with pull-up or pull-down capability
 *  - Alternate function open-drain with pull-up or pull-down capability
 *  - Analog
 */

/* Input set */
#define BOARD_GPIOINPUT_HIGH     (1 << 20)                  /* Bit 20: If input high */
#define BOARD_GPIOINPUT_LOW      (0)

/* Mode */
#define BOARD_GPIOMODE_SHIFT     (18)                       /* Bits 18-19: GPIO port mode */
#define BOARD_GPIOMODE_MASK      (3 << BOARD_GPIOMODE_SHIFT)
#  define BOARD_GPIOINPUT        (0 << BOARD_GPIOMODE_SHIFT)     /* Input mode */
#  define BOARD_GPIOOUTPUT       (1 << BOARD_GPIOMODE_SHIFT)     /* General purpose output mode */
#  define BOARD_GPIOALT          (2 << BOARD_GPIOMODE_SHIFT)     /* Alternate function mode */
#  define BOARD_GPIOANALOG       (3 << BOARD_GPIOMODE_SHIFT)     /* Analog mode */

/* Input/output pull-ups/downs (not used with analog) */
#define BOARD_GPIOPUPD_SHIFT     (16)                       /* Bits 16-17: Pull-up/pull down */
#define BOARD_GPIOPUPD_MASK      (3 << BOARD_GPIOPUPD_SHIFT)
#  define BOARD_GPIOFLOAT        (0 << BOARD_GPIOPUPD_SHIFT)     /* No pull-up, pull-down */
#  define BOARD_GPIOPULLUP       (1 << BOARD_GPIOPUPD_SHIFT)     /* Pull-up */
#  define BOARD_GPIOPULLDOWN     (2 << BOARD_GPIOPUPD_SHIFT)     /* Pull-down */

/* Alternate Functions */
#define BOARD_GPIOAF_SHIFT       (12)                       /* Bits 12-15: Alternate function */
#define BOARD_GPIOAF_MASK        (15 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF(n)        ((n) << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF0          (0 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF1          (1 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF2          (2 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF3          (3 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF4          (4 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF5          (5 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF6          (6 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF7          (7 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF8          (8 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF9          (9 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF10         (10 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF11         (11 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF12         (12 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF13         (13 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF14         (14 << BOARD_GPIOAF_SHIFT)
#  define BOARD_GPIOAF15         (15 << BOARD_GPIOAF_SHIFT)

/* Output/Alt function frequency selection */
#define BOARD_GPIOSPEED_SHIFT    (10)                       /* Bits 10-11: GPIO frequency selection */
#define BOARD_GPIOSPEED_MASK     (3 << BOARD_GPIOSPEED_SHIFT)
#  define BOARD_GPIOSPEED_400KHz (0 << BOARD_GPIOSPEED_SHIFT)     /* 400 kHz Very low speed output */
#  define BOARD_GPIOSPEED_2MHz   (1 << BOARD_GPIOSPEED_SHIFT)     /* 2 MHz Low speed output */
#  define BOARD_GPIOSPEED_10MHz  (2 << BOARD_GPIOSPEED_SHIFT)     /* 10 MHz Medium speed output */
#  define BOARD_GPIOSPEED_40MHz  (3 << BOARD_GPIOSPEED_SHIFT)     /* 40 MHz High speed output */

/* Output/Alt function type selection */
#define BOARD_GPIOOPENDRAIN      (1 << 9)                   /* Bit9: 1=Open-drain output */
#define BOARD_GPIOPUSHPULL       (0)                        /* Bit9: 0=Push-pull output */

#define BOARD_GPIOOUTPUT_SET     (1 << 8)                   /* Bit 8: If output, inital value of output */
#define BOARD_GPIOOUTPUT_CLEAR   (0)

#define BOARD_GPIOEXTI           (1 << 8)                    /* Bit 8: Configure as EXTI interrupt */

/* GPIO port */
#define BOARD_GPIOPORT_SHIFT     (4)                        /* Bit 4-7:  Port number */
#define BOARD_GPIOPORT_MASK      (15 << BOARD_GPIOPORT_SHIFT)
#  define BOARD_GPIOPORT(n)      (n << BOARD_GPIOPORT_SHIFT)     /*   GPIOn */
#  define BOARD_GPIOPORTA        (0 << BOARD_GPIOPORT_SHIFT)     /*   GPIOA */
#  define BOARD_GPIOPORTB        (1 << BOARD_GPIOPORT_SHIFT)     /*   GPIOB */
#  define BOARD_GPIOPORTC        (2 << BOARD_GPIOPORT_SHIFT)     /*   GPIOC */
#  define BOARD_GPIOPORTD        (3 << BOARD_GPIOPORT_SHIFT)     /*   GPIOD */
#  define BOARD_GPIOPORTE        (4 << BOARD_GPIOPORT_SHIFT)     /*   GPIOE */
#if defined (CONFIG_STM32_STM32L15XX)
#  define BOARD_GPIOPORTH        (5 << BOARD_GPIOPORT_SHIFT)     /*   GPIOH */
#  define BOARD_GPIOPORTF        (6 << BOARD_GPIOPORT_SHIFT)     /*   GPIOF */
#  define BOARD_GPIOPORTG        (7 << BOARD_GPIOPORT_SHIFT)     /*   GPIOG */
#else
#  define BOARD_GPIOPORTF        (5 << BOARD_GPIOPORT_SHIFT)     /*   GPIOF */
#  define BOARD_GPIOPORTG        (6 << BOARD_GPIOPORT_SHIFT)     /*   GPIOG */
#  define BOARD_GPIOPORTH        (7 << BOARD_GPIOPORT_SHIFT)     /*   GPIOH */
#  define BOARD_GPIOPORTI        (8 << BOARD_GPIOPORT_SHIFT)     /*   GPIOI */
#  define BOARD_GPIOPORTJ        (9 << BOARD_GPIOPORT_SHIFT)     /*   GPIOJ */
#  define BOARD_GPIOPORTK        (10 << BOARD_GPIOPORT_SHIFT)    /*   GPIOK */
#endif

/* GPIO pin */
#define BOARD_GPIOPIN_SHIFT      (0)                        /* Bits 0-3: GPIO number: 0-15 */
#define BOARD_GPIOPIN_MASK       (15 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN(n)       (n << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN0         (0 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN1         (1 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN2         (2 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN3         (3 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN4         (4 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN5         (5 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN6         (6 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN7         (7 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN8         (8 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN9         (9 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN10        (10 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN11        (11 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN12        (12 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN13        (13 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN14        (14 << BOARD_GPIOPIN_SHIFT)
#  define BOARD_GPIOPIN15        (15 << BOARD_GPIOPIN_SHIFT)

/* Number of pins on GPIO port */
#if defined (CONFIG_STM32_STM32L15XX)
#  define BOARD_GPIOPORT_PINS(n) ((n == 5) ? 3 : 16)        /* GPIO port H only has 3 pins. */
#else
#  define BOARD_GPIOPORT_PINS(n) (16)
#endif

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
* Name: board_gpio_map
*
* Description:
*   Get GPIO port character from port index and pin
*
* Returns:
*   Character 'A' .. on success
*   'x' in onvalid port
*
************************************************************************************/
EXTERN char board_gpioport_char(uint32_t const pinset);

/************************************************************************************
* Name: board_char_gpioport
*
* Description:
*   Get GPIO port index from port character e.g. A
*
* Returns:
*   Port index on success
*   -1 in invalid port
*
************************************************************************************/
int board_char_gpioport(char portchar);

/************************************************************************************
* Name: board_configgpio
*
* Description:
*   Configure a GPIO pin based on bit-encoded description of the pin.
*   Once it is configured as Alternative (BOARD_GPIOALT|BOARD_GPIOCNF_AFPP|...)
*   function, it must be unconfigured with stm32_unconfiggpio() with
*   the same cfgset first before it can be set to non-alternative function.
*
* Returns:
*   OK on success
*   ERROR on invalid port, or when pin is locked as ALT function.
*
************************************************************************************/
EXTERN int board_configgpio(uint32_t cfgset);

/************************************************************************************
* Name: board_gpioconfig
*
* Description:
*   Get GPIO configuration
*
* Returns:
*   OK on success
*   ERROR on invalid port
*
************************************************************************************/
EXTERN int board_gpioconfig(uint32_t pinset, uint32_t * const _cfgset);

/************************************************************************************
 * Name: board_unconfiggpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set it
 *   into default HiZ state (and possibly mark it's unused) and unlock it whether
 *   it was previsouly selected as alternative function (BOARD_GPIOALT|BOARD_GPIOCNF_AFPP|...).
 *
 *   This is a safety function and prevents hardware from schocks, as unexpected
 *   write to the Timer Channel Output GPIO to fixed '1' or '0' while it should
 *   operate in PWM mode could produce excessive on-board currents and trigger
 *   over-current/alarm function.
 *
 * Returns:
 *  OK on success
 *  ERROR on invalid port
 *
 ************************************************************************************/
EXTERN int board_unconfiggpio(uint32_t cfgset);

/************************************************************************************
 * Name: board_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/
EXTERN void board_gpiowrite(uint32_t pinset, bool value);

/************************************************************************************
 * Name: board_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/
EXTERN bool board_gpioread(uint32_t pinset);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_HALTIAN_TSONE_B1_INCLUDE_BOARD_GPIO_H */
