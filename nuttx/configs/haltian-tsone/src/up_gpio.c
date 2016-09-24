/************************************************************************************
 * configs/haltian-tsone/src/up_gpio.c
 * arch/arm/src/board/up_gpio.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *
 * Authors:
 *   Sami Pelkonen <sami.pelkonen@haltian.com>
 *   Juha Niskanen <juha.niskanen@haltian.com>
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <arch/board/board-gpio.h>
#include <arch/board/board-device.h>
#include "stm32.h"
#include "stm32_syscfg.h"
#include "haltian-tsone.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/* GPIO_NON_EXISTING encodes non-existing GPIO PP15. */

#define GPIO_NON_EXISTING 0xffffffffU

/************************************************************************************
 * Private Data
 ************************************************************************************/

static const uint32_t g_dyn_gpios_b1_7[CONFIG_NUM_DYN_GPIO_MAX] =
{
  [DYN_GPIO_WIFI_INT] =
      (GPIO_PORTE | GPIO_PIN4 | GPIO_INPUT | GPIO_EXTI | GPIO_SPEED_40MHz),

  [DYN_GPIO_CTRL_VBAT] =
      (GPIO_PORTE | GPIO_PIN7 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |
          GPIO_SPEED_400KHz),

  [DYN_GPIO_LED1] =
      (GPIO_PORTC | GPIO_PIN7 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |
          GPIO_SPEED_10MHz),

  [DYN_GPIO_PAD_J9007] =
      (GPIO_PORTC | GPIO_PIN6 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |
          GPIO_SPEED_400KHz),

  [DYN_GPIO_PAD_J9016] =
      (GPIO_PORTD | GPIO_PIN7 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |
          GPIO_SPEED_400KHz),

  [DYN_GPIO_BT_RESERVED1] =
      (GPIO_PORTC | GPIO_PIN1 | GPIO_INPUT | GPIO_FLOAT),

  [DYN_GPIO_BT_RESERVED2] =
      (GPIO_PORTC | GPIO_PIN2 | GPIO_INPUT | GPIO_FLOAT),

  [DYN_GPIO_MODEM_TX_BURST] =
      (GPIO_PORTH | GPIO_PIN2 | GPIO_ANALOG | GPIO_SPEED_400KHz),

  [DYN_GPIO_HWWDG_WAKE] = GPIO_NON_EXISTING,
  [DYN_GPIO_HWWDG_DONE] = GPIO_NON_EXISTING,
};

static const uint32_t g_dyn_gpios_b2_0[CONFIG_NUM_DYN_GPIO_MAX] =
{
  [DYN_GPIO_WIFI_INT] =
      (GPIO_PORTE | GPIO_PIN7 | GPIO_INPUT | GPIO_EXTI | GPIO_SPEED_40MHz),

  [DYN_GPIO_CTRL_VBAT] =
      (GPIO_PORTE | GPIO_PIN4 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |
          GPIO_SPEED_400KHz),

  [DYN_GPIO_LED1] =
      (GPIO_PORTD | GPIO_PIN7 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |
          GPIO_SPEED_10MHz),

  [DYN_GPIO_PAD_J9007] =
      (GPIO_PORTC | GPIO_PIN2 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |
          GPIO_SPEED_400KHz),

  [DYN_GPIO_MODEM_TX_BURST] =
      (GPIO_PORTH | GPIO_PIN2 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |
          GPIO_SPEED_400KHz),

  [DYN_GPIO_HWWDG_WAKE] =
      (GPIO_PORTC | GPIO_PIN1 | GPIO_INPUT | GPIO_EXTI | GPIO_SPEED_400KHz),

  [DYN_GPIO_HWWDG_DONE] =
      (GPIO_PORTC | GPIO_PIN6 | GPIO_OUTPUT | GPIO_OUTPUT_CLEAR |
          GPIO_SPEED_400KHz),

  [DYN_GPIO_PAD_J9016]    = GPIO_NON_EXISTING,
  [DYN_GPIO_BT_RESERVED1] = GPIO_NON_EXISTING,
  [DYN_GPIO_BT_RESERVED2] = GPIO_NON_EXISTING,
};

static const uint32_t *g_dyn_gpios = NULL;

static uint32_t g_port_config[BOARD_NGPIOS];

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static inline void gpio_add_excluded(uint16_t *excluded_bitmap, uint32_t pinset)
{
  unsigned int port = (pinset & BOARD_GPIOPORT_MASK) >> BOARD_GPIOPORT_SHIFT;
  unsigned int pin = (pinset & BOARD_GPIOPIN_MASK) >> BOARD_GPIOPIN_SHIFT;

  if (pinset == GPIO_NON_EXISTING)
    return;

  excluded_bitmap[port] |= 1 << pin;
}

static inline void gpio_exclude_unused_external_pads(uint16_t *excluded_bitmap)
{
  const uint32_t ppmask = GPIO_PIN_MASK | GPIO_PORT_MASK;

#ifdef GPIO_PAD_J9005
  gpio_add_excluded(excluded_bitmap, GPIO_PAD_J9005 & ppmask);
#endif

#ifdef GPIO_PAD_J9006
  gpio_add_excluded(excluded_bitmap, GPIO_PAD_J9006 & ppmask);
#endif

#ifdef GPIO_PAD_J9007
  gpio_add_excluded(excluded_bitmap, GPIO_PAD_J9007 & ppmask);
#endif

#ifdef GPIO_PAD_J9008
  gpio_add_excluded(excluded_bitmap, GPIO_PAD_J9008 & ppmask);
#endif

#ifdef GPIO_PAD_J9010
  gpio_add_excluded(excluded_bitmap, GPIO_PAD_J9010 & ppmask);
#endif

#ifdef GPIO_PAD_J9016
  /* On B2.0, J9016 is VDD_EXT. */
  if (board_get_hw_ver() < BOARD_HWVER_B2_0)
    {
      gpio_add_excluded(excluded_bitmap, GPIO_PAD_J9016 & ppmask);
    }
#endif
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: gpio_initialize_unused_pads
 ************************************************************************************/

void gpio_initialize_unused_pads(void)
{
#ifdef GPIO_PAD_J9005
  stm32_configgpio(GPIO_PAD_J9005);
#endif
#ifdef GPIO_PAD_J9006
  stm32_configgpio(GPIO_PAD_J9006);
#endif
#ifdef GPIO_PAD_J9007
  stm32_configgpio(GPIO_PAD_J9007);
#endif
#ifdef GPIO_PAD_J9008
  stm32_configgpio(GPIO_PAD_J9008);
#endif
#ifdef GPIO_PAD_J9010
  stm32_configgpio(GPIO_PAD_J9010);
#endif
#ifdef GPIO_PAD_J9016
  /* On B2.0, J9016 is VDD_EXT. */
  if (board_get_hw_ver() < BOARD_HWVER_B2_0)
    stm32_configgpio(GPIO_PAD_J9016);
#endif
}

/************************************************************************************
 * Name: up_configure_dynamic_gpios
 ************************************************************************************/

void up_configure_dynamic_gpios(void)
{
  int32_t hwver;

  hwver = board_detect_board_hwver();

  switch (hwver)
    {
      case BOARD_HWVER_B1_5:
      case BOARD_HWVER_B1_7:
        g_dyn_gpios = g_dyn_gpios_b1_7;
        break;
      case BOARD_HWVER_B2_0:
        g_dyn_gpios = g_dyn_gpios_b2_0;
        break;
    }
}

/************************************************************************************
 * Name: up_dyngpio
 ************************************************************************************/

uint32_t up_dyngpio(enum e_board_dyn_gpios id)
{
  uint32_t pincfg;

  DEBUGASSERT(g_dyn_gpios);
  DEBUGASSERT((int)id >= 0 && (int)id < CONFIG_NUM_DYN_GPIO_MAX);

  pincfg = g_dyn_gpios[id];

  DEBUGASSERT(pincfg != GPIO_NON_EXISTING);

  return pincfg;
}

/************************************************************************************
 * Name: gpio_initialize_sdcard_pins
 ************************************************************************************/

void gpio_initialize_sdcard_pins(void)
{
  uint32_t gpio_off_mask;

  /* At initialization SDcard is powered off, and does not react on GPIOs.
   * Force GPIOs low to avoid leak current on ESD chip.
   */

  gpio_off_mask = ~(GPIO_PUPD_MASK | GPIO_MODE_MASK | GPIO_OUTPUT_SET);
  stm32_configgpio((GPIO_SPI3_MOSI & gpio_off_mask) | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT);
  stm32_configgpio((GPIO_SPI3_MISO & gpio_off_mask) | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT);
  stm32_configgpio((GPIO_SPI3_SCK & gpio_off_mask) | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT);
  stm32_configgpio((GPIO_CHIP_SELECT_SDCARD & gpio_off_mask) | GPIO_OUTPUT_CLEAR | GPIO_OUTPUT);

  up_mdelay(10);
}

/************************************************************************************
 * Name: up_reconfigure_gpios_for_pmstop
 *
 * Description:
 *   Reconfigure GPIOs to proper state for STOP mode.
 *
 ************************************************************************************/

void up_reconfigure_gpios_for_pmstop(void)
{
  const uint32_t ppmask = GPIO_PIN_MASK | GPIO_PORT_MASK;
  uint16_t excluded_gpios[BOARD_GPIO_PORTS] = {};
  uint32_t pins_left;
  uint32_t pin;
  uint8_t port;
  int pinpos = 0;

  /*
   * Board hangs when restoring the GPIOs PC12, PC14 or PC15 after the
   * deep-sleep, no matter if we configure them as BOARD_GPIOANALOG here or not.
   * PC14 and PC15 are connected to OSC32_IN and OSC32_OUT, respectively and PC12
   * is UART5 TX used for debugging. Therefore, leave them as they were.
   *
   * Modem power control switch control line on pin D11 is missing pull-down resistor
   * and must not be left floating.
   *
   * Pins from D13 to D15 (DISP_RES#, DISP_CS#, and DISP D/C) for LCD must be left out.
   * So do two pins for SPI2. (TODO: Why?)
   *
   * All pins to SDCard must be handled specially (forced to low) and skipped here
   * to avoid leak current. (TODO: Why? Is this issue with ESD protection chip on
   * SDCard bus?)
   */

  /* OSC32_IN/OUT */

  gpio_add_excluded(excluded_gpios, GPIO_PORTC | GPIO_PIN14);
  gpio_add_excluded(excluded_gpios, GPIO_PORTC | GPIO_PIN15);

  /* GPIO_HWWDG_DONE */

  gpio_add_excluded(excluded_gpios, GPIO_HWWDG_DONE & ppmask);

#ifdef CONFIG_BOARD_RECONFIGURE_GPIOS_SKIP_JTAG
  /* Skip JTAG */

  gpio_add_excluded(excluded_gpios, GPIO_PORTB | GPIO_PIN3);
  gpio_add_excluded(excluded_gpios, GPIO_PORTB | GPIO_PIN4);
  gpio_add_excluded(excluded_gpios, GPIO_PORTA | GPIO_PIN13);
  gpio_add_excluded(excluded_gpios, GPIO_PORTA | GPIO_PIN14);
  gpio_add_excluded(excluded_gpios, GPIO_PORTA | GPIO_PIN15);
#endif

  /* Unused external pads should be pulled down to prevent
   * ESD chip leak current. */

  gpio_exclude_unused_external_pads(excluded_gpios);

#ifdef CONFIG_STM32_UART5
  /* Skip debug trace uart TX */

  gpio_add_excluded(excluded_gpios, GPIO_UART5_TX & ppmask);
#endif

  /* GPS GPIOs need to be left as is. */

  gpio_add_excluded(excluded_gpios, GPIO_USART1_RX & ppmask);
  gpio_add_excluded(excluded_gpios, GPIO_USART1_TX & ppmask);

  /* These modem GPIOs need to be left as is, as otherwise
   * modem level-shifters might start oscillate.
   */

  gpio_add_excluded(excluded_gpios, GPIO_USART3_TX & ppmask);
  gpio_add_excluded(excluded_gpios, GPIO_USART3_RTS & ppmask);
  gpio_add_excluded(excluded_gpios, GPIO_USART3_RX & ppmask);
  gpio_add_excluded(excluded_gpios, GPIO_USART3_CTS & ppmask);
  gpio_add_excluded(excluded_gpios, GPIO_MODEM_TX_BURST & ppmask);

  /* These modem GPIOs need to be left as is, as otherwise
   * modem will reset/power-off unexpectedly.
   */

  gpio_add_excluded(excluded_gpios, GPIO_MODEM_POWER_ON & ppmask);
  gpio_add_excluded(excluded_gpios, GPIO_MODEM_RESET_N & ppmask);
  gpio_add_excluded(excluded_gpios, GPIO_PWR_SWITCH_MODEM & ppmask);

  /* Sensor middleware wants to keep this powered on around deepsleep. */

  gpio_add_excluded(excluded_gpios, GPIO_PWR_SWITCH_CAPSENSE_9AXIS & ppmask);

  /* Skip display pins. */

#ifdef CONFIG_LCD_SSD1306
  gpio_add_excluded(excluded_gpios, GPIO_SPI2_MOSI & ppmask);
  gpio_add_excluded(excluded_gpios, GPIO_SPI2_SCK & ppmask);
  gpio_add_excluded(excluded_gpios, GPIO_CHIP_SELECT_DISPLAY & ppmask);
  gpio_add_excluded(excluded_gpios, GPIO_LCD_SSD1309_RESET & ppmask);
  gpio_add_excluded(excluded_gpios, GPIO_LCD_SSD1309_CMDDATA & ppmask);
#else
  /* Display regulator used for <custom HW modification external
   * power-supply purposes>, needs to stay on over deepsleep. */

  gpio_add_excluded(excluded_gpios, GPIO_REGULATOR_DISPLAY);
#endif

  /* Skip SDcard pins. */

  gpio_add_excluded(excluded_gpios, GPIO_SPI3_MOSI & ppmask);
  gpio_add_excluded(excluded_gpios, GPIO_SPI3_MISO & ppmask);
  gpio_add_excluded(excluded_gpios, GPIO_SPI3_SCK & ppmask);
  gpio_add_excluded(excluded_gpios, GPIO_CHIP_SELECT_SDCARD & ppmask);

  pinpos = 0;
  pins_left = BOARD_NGPIOS;
  for (port = 0; port < BOARD_GPIO_PORTS; port++)
    {
      uint8_t pins = (pins_left >= BOARD_GPIOPORT_PINS(port) ?
                      BOARD_GPIOPORT_PINS(port) : pins_left);

      for (pin = 0; pin < pins; pin++)
        {
          uint32_t pinset = BOARD_GPIOPORT(port) | BOARD_GPIOPIN(pin);
          uint32_t cfgset;
          int pincurr = pinpos++;

          DEBUGASSERT(pincurr < BOARD_NGPIOS);

          /* Read and save pin configuration */

          board_gpioconfig(pinset, &cfgset);
          //lldbg("reading %c %d 0x%08x\n", board_gpioport_char(pinset), pin, cfgset);

          g_port_config[pincurr] = 0xFFFFFFFFU; /* Encodes impossible GPIO PP15. */

          /* Leave all EXTI input lines configured. */

          if (((cfgset & BOARD_GPIOMODE_MASK) == BOARD_GPIOINPUT &&
              (cfgset & BOARD_GPIOEXTI)))
            {
              /* EXTI inputs need to be left as is. */

              continue;
            }

          /* Check if this GPIO is in the exclusion list. */

          if (excluded_gpios[port] & (1 << pin))
            {
              /* Excluded ports are left as is. */

              continue;
            }

          g_port_config[pincurr] = cfgset;

          /* Configure pin as analog input */

          //lldbg("setting %c %d 0x%08x\n", board_gpioport_char(pinset), pin, pinset | BOARD_GPIOANALOG);
          stm32_configgpio(pinset | BOARD_GPIOANALOG);
        }

      pins_left -= pins;
    }
}

/************************************************************************************
 * Name: up_restore_gpios_after_pmstop
 *
 * Description:
 *   Restore GPIOs to state prior to entering STOP mode.
 *
 ************************************************************************************/

void up_restore_gpios_after_pmstop(void)
{
  const uint32_t ppmask = GPIO_PIN_MASK | GPIO_PORT_MASK;
  uint32_t pins_left;
  uint32_t pin;
  uint8_t port;
  int pinpos;

  pinpos = 0;
  pins_left = BOARD_NGPIOS;
  for (port = 0; port < BOARD_GPIO_PORTS; port++)
    {
      uint8_t pins = (pins_left >= BOARD_GPIOPORT_PINS(port) ?
                      BOARD_GPIOPORT_PINS(port) : pins_left);

      for (pin = 0; pin < pins; pin++)
        {
          uint32_t pinset = BOARD_GPIOPORT(port) | BOARD_GPIOPIN(pin);
          int pincurr = pinpos++;

          DEBUGASSERT(pincurr < BOARD_NGPIOS);

          /* Restore pin configuration */

          if (g_port_config[pincurr] != 0xFFFFFFFFU)
            {
              DEBUGASSERT((g_port_config[pincurr] & ppmask) == pinset);

              stm32_configgpio(g_port_config[pincurr]);

              //lldbg("writing %d 0x%08x\n", pincurr, g_port_config[pincurr]);
            }
        }
    }
}

/************************************************************************************
* Name: board_gpio_map
*
* Description:
*   Get GPIO port character from port index and pin
*
* Returns:
*   Character 'A' .. on success
*   'x' in invalid port
*
************************************************************************************/
char board_gpioport_char(uint32_t const pinset)
{
  unsigned int port;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;
  if (port >= STM32_NGPIO_PORTS)
    return 'x';

#if CONFIG_STM32_STM32L15XX

  if (port == 5)
    return 'H';
#if STM32_NGPIO_PORTS > 6
  else if (port >= 6)
    return 'F' + port - 6;
#endif
  else
    return 'A' + port;

#else
  return 'A' + port;
#endif
}

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
int board_char_gpioport(char portchar)
{
  if (!isalpha(portchar))
    return -1;

  portchar = toupper(portchar);

#if CONFIG_STM32_STM32L15XX

  if (portchar == 'H')
    return 5; /* GPIO port H has index 5 in STM32L1 family */
  else if (portchar >= 'A' && portchar <= 'E')
    return (portchar - 'A');
#if STM32_NGPIO_PORTS > 6
  else if (portchar >= 'F' && portchar <= 'G')
    return (portchar - 'F' + 6);
#endif
  else
    return -1;

#else

  if (portchar >= 'A' && portchar <= ('A' + STM32_NGPIO_PORTS))
    return (portchar - 'A');
  else
    return -1;

#endif
}

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
int board_configgpio(uint32_t cfgset)
{
  return stm32_configgpio(cfgset);
}

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
int board_gpioconfig(uint32_t pinset, uint32_t * const _cfgset)
{
  uintptr_t base;
  unsigned int port;
  unsigned int pin;
  unsigned int pos;
  uint32_t regval;
  unsigned int regoffset;
  int shift;
  irqstate_t flags;
  uint32_t cfgset = 0;

  port = (pinset & GPIO_PORT_MASK) >> GPIO_PORT_SHIFT;

  if (port >= STM32_NGPIO_PORTS)
    return -1;

  /* Get the port base address */

  base = g_gpiobase[port];

  /* Get the pin number  */

  pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  /* Set initial configuration */

  cfgset = BOARD_GPIOPORT(port) | BOARD_GPIOPIN(pin);

  /* Interrupts must be disabled from here on out so that we have mutually
   * exclusive access to all of the GPIO configuration registers.
   */

  flags = irqsave();

  /* MODER */

  regval  = getreg32(base + STM32_GPIO_MODER_OFFSET);
  regval &= GPIO_MODER_MASK(pin);
  regval >>= GPIO_MODER_SHIFT(pin);
  cfgset |= regval << GPIO_MODE_SHIFT;

  /* PUPDR */

  regval  = getreg32(base + STM32_GPIO_PUPDR_OFFSET);
  regval &= GPIO_PUPDR_MASK(pin);
  regval >>= GPIO_PUPDR_SHIFT(pin);
  cfgset |= regval << GPIO_PUPD_SHIFT;

  /* AF */

  if (pin < 8)
    {
      regoffset = STM32_GPIO_AFRL_OFFSET;
      pos       = pin;
    }
  else
    {
      regoffset = STM32_GPIO_AFRH_OFFSET;
      pos       = pin - 8;
    }

  regval  = getreg32(base + regoffset);
  regval &= GPIO_AFR_MASK(pos);
  regval >>= GPIO_AFR_SHIFT(pos);
  cfgset |= regval << GPIO_AF_SHIFT;

  /* OSPEED */

  regval  = getreg32(base + STM32_GPIO_OSPEED_OFFSET);
  regval &= GPIO_OSPEED_MASK(pin);
  regval >>= GPIO_OSPEED_SHIFT(pin);
  cfgset |= regval << GPIO_SPEED_SHIFT;

  /* OTYPER */

  regval  = getreg32(base + STM32_GPIO_OTYPER_OFFSET);
  regval &= GPIO_OTYPER_OD(pin);
  regval >>= pin;
  cfgset |= (regval ? GPIO_OPENDRAIN : GPIO_PUSHPULL);

  /* Output set */

  if ((cfgset & GPIO_MODE_MASK) == GPIO_OUTPUT)
    {
      regval = getreg32(base + STM32_GPIO_ODR_OFFSET);

      if (regval & GPIO_ODR(pin))
        cfgset |=  GPIO_OUTPUT_SET;
    }
  else if ((cfgset & GPIO_MODE_MASK) == GPIO_INPUT)
    {
      uint32_t exti_imr;

      /* Input set */

      regval = getreg32(base + STM32_GPIO_IDR_OFFSET);
      if (regval & GPIO_IDR(pin))
        cfgset |= BOARD_GPIOINPUT_HIGH;

      /* EXTI */

      regval  = getreg32(STM32_SYSCFG_EXTICR(pin));
      shift   = SYSCFG_EXTICR_EXTI_SHIFT(pin);
      regval &= (SYSCFG_EXTICR_PORT_MASK << shift);
      regval >>= shift;

      exti_imr = getreg32(STM32_EXTI_IMR);
      if ((regval == port) && (exti_imr & STM32_EXTI_BIT(pin)))
        cfgset |= BOARD_GPIOEXTI;
    }

  irqrestore(flags);

  *_cfgset = cfgset;

  return OK;
}

/************************************************************************************
 * Name: board_unconfiggpio
 *
 * Description:
 *   Unconfigure a GPIO pin based on bit-encoded description of the pin, set it
 *   into default HiZ state (and possibly mark it's unused) and unlock it whether
 *   it was previously selected as alternative function (BOARD_GPIOALT|BOARD_GPIOCNF_AFPP|...).
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
int board_unconfiggpio(uint32_t cfgset)
{
  return stm32_unconfiggpio(cfgset);
}

/************************************************************************************
 * Name: board_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 ************************************************************************************/
void board_gpiowrite(uint32_t pinset, bool value)
{
  return stm32_gpiowrite(pinset, value);
}

/************************************************************************************
 * Name: board_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 ************************************************************************************/
bool board_gpioread(uint32_t pinset)
{
  return stm32_gpioread(pinset);
}

