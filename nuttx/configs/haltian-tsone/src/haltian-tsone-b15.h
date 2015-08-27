/****************************************************************************************************
 * configs/haltian-tsone/src/haltian-tsone-b15.h
 * arch/arm/src/board/haltian-tsone-b15.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************************************/

#ifndef __CONFIGS_HALTIAN_TSONE_SRC_HALTIAN_TSONE_B15_INTERNAL_H
#define __CONFIGS_HALTIAN_TSONE_SRC_HALTIAN_TSONE_B15_INTERNAL_H

#ifndef __CONFIGS_HALTIAN_TSONE_SRC_HALTIAN_TSONE_INTERNAL_H
#error "Use <haltian-tsone.h> instead of <haltian-tsone-b15.h>"
#endif

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* Configuration ************************************************************************************/
/* How many SPI modules does this chip support? */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 3
#  undef CONFIG_STM32_SPI3
#endif

/* Thingsee One B1.5-B2.0 configuration *************************************************************/

/* Board features */
#define BOARD_HAS_MODEM_POWER_SWITCH 1

/* Chip-select pin configuration */
enum e_board_chip_select_pins {
  CHIP_SELECT_SPI1_WLAN = 0,
  CHIP_SELECT_SPI2_DISPLAY,
  CHIP_SELECT_SPI3_SDCARD,

  __CHIP_SELECT_PIN_MAX,
  CONFIG_NUM_CHIP_SELECT_PINS = __CHIP_SELECT_PIN_MAX,
};

/* Chip-select GPIO configs, with output set to deassert state */
#define GPIO_CHIP_SELECT_WLAN     (GPIO_PORTE | GPIO_PIN5 | GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_SPEED_40MHz)
#define GPIO_CHIP_SELECT_DISPLAY  (GPIO_PORTD | GPIO_PIN14 | GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_SPEED_40MHz)
#define GPIO_CHIP_SELECT_SDCARD   (GPIO_PORTD | GPIO_PIN12 | GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_SPEED_40MHz)

/* Regulator control GPIOs */
#define GPIO_REGULATOR_GPS        (GPIO_PORTB | GPIO_PIN9 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | GPIO_SPEED_400KHz)
#define GPIO_REGULATOR_WLAN       (GPIO_PORTD | GPIO_PIN6 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | GPIO_SPEED_400KHz)
#define GPIO_REGULATOR_SDCARD     (GPIO_PORTD | GPIO_PIN10 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | GPIO_SPEED_400KHz)
#define GPIO_REGULATOR_BLUETOOTH  (GPIO_PORTD | GPIO_PIN5 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | GPIO_SPEED_400KHz)
#define GPIO_REGULATOR_DISPLAY    (GPIO_PORTB | GPIO_PIN8 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | GPIO_SPEED_400KHz)

/* Power control GPIOs */
#define GPIO_PWR_SWITCH_CAPSENSE_9AXIS (GPIO_PORTB | GPIO_PIN12 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | GPIO_SPEED_400KHz)
#define GPIO_PWR_SWITCH_MODEM          (GPIO_PORTD | GPIO_PIN11 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | GPIO_SPEED_400KHz)
#define GPIO_PWR_SWITCH_VBAT_ADC       (up_dyngpio(DYN_GPIO_CTRL_VBAT))

/* VBAT measurement with ADC1 */
#define ADC1_MEASURE_VBAT_CHANNEL 9
#define GPIO_VBAT_MEASURE_ADC     (GPIO_ADC1_IN9)

/* POWER_ON, RESET_N GPIOs for u-blox modem */
#define GPIO_MODEM_POWER_ON      (GPIO_PORTC | GPIO_PIN8 | GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_OPENDRAIN | GPIO_FLOAT | GPIO_SPEED_400KHz)
#define GPIO_MODEM_RESET_N       (GPIO_PORTC | GPIO_PIN9 | GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_OPENDRAIN | GPIO_FLOAT | GPIO_SPEED_400KHz)
#define GPIO_MODEM_TX_BURST      (up_dyngpio(DYN_GPIO_MODEM_TX_BURST))

/* EN and IRQ GPIOs for CC3000 */
#define GPIO_WIFI_EN             (GPIO_PORTE | GPIO_PIN3 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | GPIO_SPEED_40MHz)
#define GPIO_WIFI_INT            (up_dyngpio(DYN_GPIO_WIFI_INT))

/* IRQ GPIO for LPS25H Barometer */
#define GPIO_LPS25H_PRES_INT     (GPIO_PORTE | GPIO_PIN9 | GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT)

/* IRQ GPIOs for LIS2DH Accelerometer */
#define GPIO_LIS2DH_INT1         (GPIO_PORTE | GPIO_PIN6 | GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT)
#define GPIO_LIS2DH_INT2         (GPIO_PORTE | GPIO_PIN8 | GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT)

#ifdef CONFIG_BQ24251_CHARGER
#  define GPIO_BQ24251_STAT      (GPIO_PORTC | GPIO_PIN5 | GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT)
/* Charge Enable is active-low type. */
#  define GPIO_BQ24251_CE        (GPIO_PORTC | GPIO_PIN4 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | GPIO_SPEED_400KHz)
#endif

#ifdef CONFIG_HTS221_HUMIDITY
#  define GPIO_HTS221_INT        (GPIO_PORTE | GPIO_PIN10 | GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT)
#endif

#ifdef CONFIG_INPUT_CYPRESS_MBR3108
#  define GPIO_MBR3108_INT       (GPIO_PORTD | GPIO_PIN4 | GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT)
#endif

#if CONFIG_MAX44009_SENSOR
#  define GPIO_MAX44009_INT      (GPIO_PORTB | GPIO_PIN0 | GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT)
#endif


/* Bluetooth pins */
#define GPIO_BT_RESET_N          (GPIO_PORTC | GPIO_PIN0 | GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_PULLUP)
#define GPIO_BT_RESERVED1        (up_dyngpio(DYN_GPIO_BT_RESERVED1))
#define GPIO_BT_RESERVED2        (up_dyngpio(DYN_GPIO_BT_RESERVED2))
#define GPIO_BT_RESERVED3        (GPIO_PORTC | GPIO_PIN3 | GPIO_OUTPUT | GPIO_PUSHPULL)
#define GPIO_BT_CTS              (GPIO_PORTA | GPIO_PIN0 | GPIO_OUTPUT | GPIO_PUSHPULL)
#define GPIO_BT_RTS              (GPIO_PORTA | GPIO_PIN1 | GPIO_OUTPUT | GPIO_PUSHPULL)

#ifdef CONFIG_LSM9DS1_SENS /* LSM9DS1 */
#  define GPIO_LSM9DS1_INT1_AG   (GPIO_PORTE | GPIO_PIN11 | GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT)
#  define GPIO_LSM9DS1_INT2_AG   (GPIO_PORTE | GPIO_PIN12 | GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT)
#  define GPIO_LSM9DS1_DEN_AG    (GPIO_PORTE | GPIO_PIN13 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | GPIO_SPEED_400KHz)
#  define GPIO_LSM9DS1_INT_M     (GPIO_PORTE | GPIO_PIN14 | GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT)
#  define GPIO_LSM9DS1_DRDY_M    (GPIO_PORTE | GPIO_PIN15 | GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT)
#endif

#define GPIO_LCD_SSD1309_RESET   (GPIO_PORTD | GPIO_PIN13 | GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_SPEED_400KHz)
#define GPIO_LCD_SSD1309_CMDDATA (GPIO_PORTD | GPIO_PIN15 | GPIO_OUTPUT | GPIO_OUTPUT_SET | GPIO_SPEED_400KHz)

/* External GPIO pads */
#define GPIO_PAD_J9005           (GPIO_PORTA | GPIO_PIN4 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | GPIO_SPEED_400KHz)
#define GPIO_PAD_J9006           (GPIO_PORTA | GPIO_PIN8 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | GPIO_SPEED_400KHz)
#define GPIO_PAD_J9007           (up_dyngpio(DYN_GPIO_PAD_J9007))
#define GPIO_PAD_J9008           (GPIO_PORTC | GPIO_PIN7 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | GPIO_SPEED_400KHz)
#define GPIO_PAD_J9010           (GPIO_PORTD | GPIO_PIN0 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR | GPIO_SPEED_400KHz)
#define GPIO_PAD_J9016           (up_dyngpio(DYN_GPIO_PAD_J9016))

/* LED definitions *********************************************************************************/

/* The Thingsee One B1.5 board has no LEDs. */

/* The Thingsee One B1.7 board has one LED.
 *
 * LED1:      Green LED is a user LED connected wrong way to the I/O PC7
 */

/* The Thingsee One B2.0 board has one LED.
 *
 * LED1:      Green LED is a user LED connected to the I/O PD7
 */

#define GPIO_LED1               (up_dyngpio(DYN_GPIO_LED1))

/* HW Watchdog definitions *************************************************************************/

/* The Thingsee One B2.0 board uses a HW watchdog instead of IWDG. */

#define GPIO_HWWDG_WAKE         (up_dyngpio(DYN_GPIO_HWWDG_WAKE))
#define GPIO_HWWDG_DONE         (up_dyngpio(DYN_GPIO_HWWDG_DONE))

/* Button definitions ******************************************************************************/

/* The Thingsee One board has one button.
 *
 * BUTTON_POWERKEY: Power-key/Multi-purpose button connected to the I/O PC13
 */

#define GPIO_BTN_POWERKEY       (GPIO_PORTC | GPIO_PIN13 | GPIO_INPUT | GPIO_EXTI | GPIO_FLOAT)
#define GPIO_BTN_POWERKEY_LOW_WHEN_PRESSED false

#define MIN_IRQBUTTON           BOARD_BUTTON_POWERKEY
#define MAX_IRQBUTTON           BOARD_BUTTON_POWERKEY

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32ldiscovery board.
 *
 ****************************************************************************************************/

void weak_function stm32_spiinitialize(void);

/****************************************************************************************************
 * Name: nsh_cc3000_start
 *
 * Description:
 *   Initialize CC3000 module and attempt connection to predefined SSID.
 *
 ****************************************************************************************************/

void nsh_cc3000_start(void);

/****************************************************************************************************
 * Name: up_memlcd_initialize
 *
 * Description:
 *   Initialize the Sharp Memory LCD device.
 *
 ****************************************************************************************************/

int up_memlcd_initialize(void);

/****************************************************************************************************
 * Name: up_boot_standby_mode
 *
 * Description:
 *   Handling for low-power standby mode.
 *
 ****************************************************************************************************/

void up_boot_standby_mode(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_HALTIAN_TSONE_B1_SRC_HALTIAN_TSONE_B1_INTERNAL_H */

