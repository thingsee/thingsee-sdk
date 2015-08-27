/****************************************************************************
 * configs/haltian-tsone/src/up_modem.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *   Authors: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <unistd.h>
#include <fcntl.h>

#include "chip.h"
#include "stm32_gpio.h"

#include <arch/board/board.h>
#include <arch/board/board-modem.h>
#include <arch/board/board-pwrctl.h>
#include "haltian-tsone.h"
#include "up_modem.h"
#include "up_serialrxdma_poll.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#  else
#    define message printf
#  endif
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef BOARD_HAS_MODEM_POWER_SWITCH

/****************************************************************************
 * Name: board_modem_reset
 *
 * Description:
 *   Reset modem with RESET_N gpio.
 *
 ****************************************************************************/

static void board_modem_reset(void)
{
  uint32_t msec;

  /* Reset modem by toggling RESET_N pin. */

  msec = board_modem_reset_pin_set(false);
  usleep(msec * 1000);

  msec = board_modem_reset_pin_set(true);
  usleep(msec * 1000);
}

/****************************************************************************
 * Name: board_modem_toogle_poweron
 *
 * Description:
 *   Start modem with POWER_ON gpio.
 *
 ****************************************************************************/

static void board_modem_toogle_poweron(void)
{
  uint32_t msec;

  /* Power-on modem by toggling POWER_ON pin. */

  msec = board_modem_poweron_pin_set(false);
  usleep(msec * 1000);

  msec = board_modem_poweron_pin_set(true);
  usleep(msec * 1000);
}

#endif /*!BOARD_HAS_MODEM_POWER_SWITCH*/

/****************************************************************************
 * Name: board_modem_setup_gpios
 ****************************************************************************/

static void board_modem_setup_gpios(bool poweredoff)
{
#ifdef BOARD_HAS_MODEM_POWER_SWITCH
  if (poweredoff)
    {
      uint32_t gpio_off_mask = ~(GPIO_PUPD_MASK | GPIO_MODE_MASK | GPIO_OUTPUT_SET);

      /* We need to pull modem GPIOs down, as otherwise level-shifters between
       * MCU and modem can start oscillate and cause power-usage to jump up
       * to 0.5 mA.
       */

      stm32_configgpio((GPIO_USART3_TX & gpio_off_mask) |
                       (GPIO_OUTPUT_CLEAR | GPIO_OUTPUT));
      stm32_configgpio((GPIO_USART3_RTS & gpio_off_mask) |
                       (GPIO_OUTPUT_CLEAR | GPIO_OUTPUT));

      stm32_configgpio((GPIO_USART3_RX & gpio_off_mask) |
                       (GPIO_OUTPUT_CLEAR | GPIO_OUTPUT));
      stm32_configgpio((GPIO_USART3_CTS & gpio_off_mask) |
                       (GPIO_OUTPUT_CLEAR | GPIO_OUTPUT));
    }
  else
#endif /* BOARD_HAS_MODEM_POWER_SWITCH */
    {
      /* Configure modem GPIOs for serial-port AF. */

      stm32_configgpio(GPIO_USART3_TX);
      stm32_configgpio(GPIO_USART3_RTS);
      stm32_configgpio(GPIO_USART3_RX);
      stm32_configgpio(GPIO_USART3_CTS);
    }

  stm32_configgpio(GPIO_MODEM_TX_BURST);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_modem_reset_pin_set
 *
 * Description:
 *   Control RESET_N gpio.
 *
 * Input Parameters:
 *   set:  enable or disable pin.
 *
 * Return value:
 *   Recommended time in milliseconds to sleep after setting pin for proper
 *   operation of modem.
 *
 ****************************************************************************/

uint32_t board_modem_reset_pin_set(bool set)
{
  if (!set)
    {
      /* RESET_N must be down for at least 50 msec. */

      stm32_gpiowrite(GPIO_MODEM_RESET_N, false);

      return 50 + 10;
    }
  else
    {
      stm32_gpiowrite(GPIO_MODEM_RESET_N, true);

      /* Small wait after reset. */

      return 50 + 10;
    }
}

/****************************************************************************
 * Name: board_modem_poweron_pin_set
 *
 * Description:
 *   Control POWER_ON gpio.
 *
 * Input Parameters:
 *   set:  enable or disable pin.
 *
 * Return value:
 *   Recommended time in milliseconds to sleep after setting pin for proper
 *   operation of modem.
 *
 ****************************************************************************/

uint32_t board_modem_poweron_pin_set(bool set)
{
  if (!set)
    {
      /* POWER_ON must be down for at least 5 msec. */

      stm32_gpiowrite(GPIO_MODEM_POWER_ON, false);

      return 5 + 5;
    }
  else
    {
      /*
       * Approx. 30 msec after POWER_ON up, serial interface should be operational.
       * Internal reset will still take up to 3 seconds.
       */

      stm32_gpiowrite(GPIO_MODEM_POWER_ON, true);

      return 30 + 10;
    }
}

/****************************************************************************
 * Name: board_modem_initialize
 *
 * Description:
 *   Initialize the u-blox modem.
 *
 * Input Parameters:
*    is_vcc_off:   Value is set 'true' if modem is fully powered off at
*                  start-up.
*
 * Return:
 *   Opened modem serial port file descriptor.
 *
 ****************************************************************************/

int board_modem_initialize(bool *is_vcc_off)
{
  DEBUGASSERT(is_vcc_off);

  /* Setup control GPIOs for modem. */

  stm32_configgpio(GPIO_MODEM_POWER_ON);
  stm32_configgpio(GPIO_MODEM_RESET_N);

#ifdef BOARD_HAS_MODEM_POWER_SWITCH
  /* Board has power-switch for controlling VCC for modem module and is powered
   * off at board power-on. */

  *is_vcc_off = true;
#else
  /* Modem might be already on. Do reset with RESET_N pin. */

  board_modem_reset();

  /* Do initial power-up. */

  board_modem_toogle_poweron();

  /* No power-switch, modem starts up at board power-on. */

  *is_vcc_off = false;
#endif

  /* Open serial. */

  return open(MODEM_SERIAL_DEVNAME, O_RDWR, 0666);
}

/****************************************************************************
 * Name: board_modem_deinitialize
 *
 * Description:
 *   Deinitialize the u-blox modem.
 *
 ****************************************************************************/

int board_modem_deinitialize(int fd)
{
  /* Close file-descriptor. */

  return close(fd);
}

/****************************************************************************
 * Name: board_modem_vcc_set
 *
 * Description:
 *   Set modem VCC.
 *
 * Input Parameters:
 *   on:   New requested VCC on/off state.
 *
 * Return:
 *   Return new VCC state.
 *
 ****************************************************************************/

bool board_modem_vcc_set(bool on)
{
#ifdef BOARD_HAS_MODEM_POWER_SWITCH
  static bool vcc_state = false;

  if (vcc_state == on)
    return on;

  if (on)
    {
      board_modem_setup_gpios(false);

      board_pwrctl_get(PWRCTL_SWITCH_MODEM);
      up_launch_serialrxdma_poll();
    }
  else
    {
      board_modem_setup_gpios(true);

      board_pwrctl_put(PWRCTL_SWITCH_MODEM);
    }

  vcc_state = on;

  return on;
#else
  /* No switch, VCC always on. */

  return true;
#endif
}

/****************************************************************************
 * Name: up_modem_initialize_gpios
 *
 * Description:
 *   Set modem GPIOs to initial state.
 *
 ****************************************************************************/

void up_modem_initialize_gpios(void)
{
  /* Setup serial GPIOs for modem (modem is powered off). */

  board_modem_setup_gpios(true);
}
