/************************************************************************************
 * configs/haltian-tsone/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *
 * Authors:
 *   Sami Pelkonen <sami.pelkonen@haltian.com>
 *   Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#ifndef __CONFIG_STM32L1_TSONE_B15_INCLUDE_BOARD_H
#define __CONFIG_STM32L1_TSONE_B15_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* Four different clock sources can be used to drive the system clock (SYSCLK):
 *
 * - HSI high-speed internal oscillator clock
 *   Generated from an internal 16 MHz RC oscillator
 * - HSE high-speed external oscillator clock
 *   Normally driven by an external crystal (B4801A).
 * - PLL clock
 * - MSI multispeed internal oscillator clock
 *   The MSI clock signal is generated from an internal RC oscillator. Seven frequency
 *   ranges are available: 65.536 kHz, 131.072 kHz, 262.144 kHz, 524.288 kHz, 1.048 MHz,
 *   2.097 MHz (default value) and 4.194 MHz.
 *
 * The devices have the following two secondary clock sources
 * - LSI low-speed internal RC clock
 *   Drives the watchdog and RTC.  Approximately 37KHz
 * - LSE low-speed external oscillator clock
 *   Driven by 32.768KHz crystal (B4800) on the OSC32_IN and OSC32_OUT pins.
 */

#ifndef CONFIG_BOARD_HSE_XTAL_FREQUENCY
#  define CONFIG_BOARD_HSE_XTAL_FREQUENCY 8
#endif

#if CONFIG_BOARD_HSE_XTAL_FREQUENCY == 8

/* PLL Configuration
 *
 *   - PLL source is HSE      -> 8MHz input
 *   - PLL multiplier is 12   -> 96MHz PLL VCO clock output (for USB)
 *   - PLL output divider 3   -> 32MHz divided down PLL VCO clock output
 *
 * Resulting SYSCLK frequency is 8MHz x 12 / 3 = 32MHz
 */
#  define STM32_BOARD_XTAL       8000000ul        /* replacement crystal (8Mhz) */
#  define BOARD_PLLMUL           12
#  define BOARD_CFGR_PLLMUL      RCC_CFGR_PLLMUL_CLKx12
#  define BOARD_PLLDIV           3
#  define BOARD_CFGR_PLLDIV      RCC_CFGR_PLLDIV_3

#elif CONFIG_BOARD_HSE_XTAL_FREQUENCY == 12
/* PLL Configuration
 *
 *   - PLL source is HSE      -> 12MHz input
 *   - PLL multiplier is 8    -> 96MHz PLL VCO clock output (for USB)
 *   - PLL output divider 3   -> 32MHz divided down PLL VCO clock output
 *
 * Resulting SYSCLK frequency is 12MHz x 8 / 3 = 32MHz
 */
#  define STM32_BOARD_XTAL       12000000ul       /* replacement crystal (12Mhz) */
#  define BOARD_PLLMUL           8
#  define BOARD_CFGR_PLLMUL      RCC_CFGR_PLLMUL_CLKx8
#  define BOARD_PLLDIV           3
#  define BOARD_CFGR_PLLDIV      RCC_CFGR_PLLDIV_3

#else
#  error "Unknown HSE frequency"
#endif

#define STM32_HSI_FREQUENCY      16000000ul       /* Approximately 16MHz */
#define STM32_HSE_FREQUENCY      STM32_BOARD_XTAL
#define STM32_MSI_FREQUENCY      2097000          /* Default is approximately 2.097Mhz */
#define STM32_LSI_FREQUENCY      37000            /* Approximately 37KHz */
#define STM32_LSE_FREQUENCY      32768            /* B4800 on board */

/*
 * USB/SDIO:
 *   If the USB or SDIO interface is used in the application, the PLL VCO
 *   clock (defined by STM32_CFGR_PLLMUL) must be programmed to output a 96
 *   MHz frequency. This is required to provide a 48 MHz clock to the USB or
 *   SDIO (SDIOCLK or USBCLK = PLLVCO/2).
 * SYSCLK
 *   The system clock is derived from the PLL VCO divided by the output division factor.
 * Limitations:
 *   96 MHz as PLLVCO when the product is in range 1 (1.8V),
 *   48 MHz as PLLVCO when the product is in range 2 (1.5V),
 *   24 MHz when the product is in range 3 (1.2V).
 *   Output division to avoid exceeding 32 MHz as SYSCLK.
 *   The minimum input clock frequency for PLL is 2 MHz (when using HSE as PLL source).
 */

#ifdef CONFIG_STM32_USB
#  define STM32_CFGR_PLLSRC      RCC_CFGR_PLLSRC
#  define STM32_CFGR_PLLMUL      BOARD_CFGR_PLLMUL
#  define STM32_CFGR_PLLDIV      BOARD_CFGR_PLLDIV
/* PLL VCO Frequency is 96MHz */
#  define STM32_PLL_FREQUENCY    (BOARD_PLLMUL * STM32_HSE_FREQUENCY)
#else
#  undef STM32_CFGR_PLLSRC                               /* Source is 0MHz nothing */
#  undef STM32_CFGR_PLLMUL                               /* PLLMUL = 0 */
#  undef STM32_CFGR_PLLDIV                               /* PLLDIV = 0 */
#  undef STM32_PLL_FREQUENCY                             /* PLL VCO Frequency is 0MHz */
#endif

/* Use the PLL and set the SYSCLK source to be the divided down PLL VCO output
 * frequency (STM32_PLL_FREQUENCY divided by the PLLDIV value).
 */

#ifdef CONFIG_STM32_USB
#  define STM32_SYSCLK_SW        RCC_CFGR_SW_PLL         /* Use the PLL as the SYSCLK */
#  define STM32_SYSCLK_SWS       RCC_CFGR_SWS_PLL
/* SYSCLK frequency is 96MHz / PLLDIV = 32MHz */
#  define STM32_SYSCLK_FREQUENCY (STM32_PLL_FREQUENCY / BOARD_PLLDIV)
#else
#  define STM32_SYSCLK_SW        RCC_CFGR_SW_HSE         /* Use the HSE as the SYSCLK */
#  define STM32_SYSCLK_SWS       RCC_CFGR_SWS_HSE
#  define STM32_SYSCLK_FREQUENCY STM32_HSE_FREQUENCY     /* SYSCLK frequency is same as HSE */
#endif

/* AHB clock (HCLK) is SYSCLK (32MHz with USB, HSE Mhz otherwise) */

#define STM32_RCC_CFGR_HPRE      RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY     STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK         STM32_HCLK_FREQUENCY    /* Same as above, to satisfy compiler */

/* APB2 clock (PCLK2) is HCLK (32MHz with USB, HSE Mhz otherwise) */

#define STM32_RCC_CFGR_PPRE2     RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY    STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN         (STM32_PCLK2_FREQUENCY)

/* APB2 timers 9, 10, and 11 will receive PCLK2. */

#define STM32_APB2_TIM9_CLKIN    (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN   (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK (32MHz with USB, HSE Mhz otherwise) */

#define STM32_RCC_CFGR_PPRE1     RCC_CFGR_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY    (STM32_HCLK_FREQUENCY)

/* APB1 timers 2-7 will receive PCLK1 */

#define STM32_APB1_TIM2_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN    (STM32_PCLK1_FREQUENCY)

/* Alternate Pin Functions **********************************************************/

/* USART1 connected to GPS (no HW flow control, baud: 9600) */
#  define GPS_USART_NUM          1
#  define GPS_SERIAL_DEVNAME     "/dev/ttyS0"
#  define GPIO_USART1_RX         GPIO_USART1_RX_1 /* PA10 */
#  define GPIO_USART1_TX         GPIO_USART1_TX_1 /* PA9 */

/* USART2 connected to BT (HW flow control, baud: 115200) */

#  define BLUETOOTH_SERIAL_DEVNAME "/dev/ttyS1"
#  define GPIO_USART2_RX         GPIO_USART2_RX_1  /* PA3 */
#  define GPIO_USART2_TX         GPIO_USART2_TX_1  /* PA2 */
#  define GPIO_USART2_CTS        GPIO_USART2_CTS_1 /* PA0 */
#  define GPIO_USART2_RTS        GPIO_USART2_RTS_1 /* PA1 */

/* USART3 connected to modem (HW flow control, baud: 115200) */

#  define MODEM_USART_NUM        3
#  define MODEM_SERIAL_DEVNAME   "/dev/ttyS2"
#  define GPIO_USART3_RX         GPIO_USART3_RX_3  /* PD9 */
#  define GPIO_USART3_TX         GPIO_USART3_TX_3  /* PD8 */
#  define GPIO_USART3_CTS        GPIO_USART3_CTS_1 /* PB13 */
#  define GPIO_USART3_RTS        GPIO_USART3_RTS_1 /* PB14 */

/* USART5 is for output (no HW flow control, baud 115200) */

#define DEBUG_SERIAL_DEVNAME     "/dev/ttyS3"

/* I2C1: Accelerometer 1 (LIS2DH), Humidity/Temperature (HTS221),
 *       Ambient light (MAX44009EDT+), Pressure (LPS25H) and
 *       Charger (BQ24251YFF). */
#  define GPIO_I2C1_SCL          GPIO_I2C1_SCL_1 /* PB6 */
#  define GPIO_I2C1_SDA          GPIO_I2C1_SDA_1 /* PB7 */

/* I2C2: Capsense (CY8CMR3108LQXI), 9-asix inertial module (LSM9DS1) and
 *       pads available for prototyping. */
/*#define GPIO_I2C2_SCL          GPIO_I2C2_SCL // PB10 */
/*#define GPIO_I2C2_SDA          GPIO_I2C2_SDA // PB11 */

/* SPI1 CC3000 WIFI */
#  define GPIO_SPI1_MISO         (GPIO_SPI1_MISO_2 | GPIO_SPEED_40MHz | GPIO_PUSHPULL) /* PA6 */
#  define GPIO_SPI1_MOSI         (GPIO_SPI1_MOSI_2 | GPIO_SPEED_40MHz | GPIO_PUSHPULL) /* PA7 */
#  define GPIO_SPI1_SCK          (GPIO_SPI1_SCK_1 | GPIO_SPEED_40MHz | GPIO_PUSHPULL)  /* PA5 */

/* SPI2: Display SSD1309 and pads available for prototyping. */
#  define GPIO_SPI2_MISO         (GPIO_SPI2_MISO_2 | GPIO_SPEED_40MHz | GPIO_PUSHPULL) /* PD1 */
#  define GPIO_SPI2_MOSI         (GPIO_SPI2_MOSI_1 | GPIO_SPEED_40MHz | GPIO_PUSHPULL) /* PD3 */
#  define GPIO_SPI2_SCK          (GPIO_SPI2_SCK_2 | GPIO_SPEED_40MHz | GPIO_PUSHPULL)  /* PB15 */

/* SPI3 SDCARD */
#  define GPIO_SPI3_MISO         (GPIO_SPI3_MISO_2 | GPIO_SPEED_40MHz | GPIO_PUSHPULL) /* PC11 */
#  define GPIO_SPI3_MOSI         (GPIO_SPI3_MOSI_1 | GPIO_SPEED_40MHz | GPIO_PUSHPULL) /* PB5 */
#  define GPIO_SPI3_SCK          (GPIO_SPI3_SCK_2 | GPIO_SPEED_40MHz | GPIO_PUSHPULL)  /* PC10 */

/* Misc *****************************************************************************/

/* LED definitions ******************************************************************/
/*
 * The Thingsee One B1.7 & B2.0 boards has one LED.
 *
 * LED1:      Green LED is a user LED connected to the I/O <DYN_GPIO_LED1>
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with stm32_setled() */

#define BOARD_LED1               0 /* User LED1 */
#define BOARD_NLEDS              1

/* LED bits for use with board_led_on/off() */

#define BOARD_LED1_BIT           (1 << BOARD_LED1)

/* Additional control bits bits */
#define BOARD_LED_ALL_OFF        (1 << (BOARD_NLEDS + 0))

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board.
 * The following definitions describe how NuttX controls the LEDs:
 *
 *   SYMBOL                Meaning                 LED state
 *                                                   LED1     LED2     LED3     LED4
 *   -------------------  -----------------------  -------- -------- -------- --------
 *   LED_STARTED          NuttX has been started     OFF      OFF      OFF      OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF      OFF      OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF      OFF      OFF      OFF
 *   LED_STACKCREATED     Idle stack created         OFF      OFF      OFF      OFF
 *   LED_INIRQ            In an interrupt            <NC>     <NC>     <NC>     <NC>
 *   LED_SIGNAL           In a signal handler        <NC>     <NC>     <NC>     <NC>
 *   LED_ASSERTION        An assertion failed        <NC>     <NC>     <NC>     ON
 *   LED_PANIC            The system has crashed     Blinking <NC>     <NC>     <NC>
 *   LED_IDLE             STM32 is is sleep mode     <NC>     <NC>     <NC>     <NC>
 *
 *   <NC> = No change
 */

#define LED_STARTED             (BOARD_LED1_BIT)
#define LED_HEAPALLOCATE        BOARD_LED_ALL_OFF
#define LED_IRQSENABLED         BOARD_LED_ALL_OFF
#define LED_STACKCREATED        BOARD_LED_ALL_OFF
#define LED_INIRQ               (0)
#define LED_SIGNAL              (0)
#define LED_ASSERTION           (BOARD_LED1_BIT)
#define LED_PANIC               (BOARD_LED1_BIT)
#define LED_IDLE                (0)

#endif  /* __CONFIG_STM32L1_TSONE_B15_INCLUDE_BOARD_H */
