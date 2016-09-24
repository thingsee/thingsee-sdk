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

/* Maximum deep-sleep time when modem in low-activity mode (note: high activity
 * level prevents deep-sleep altogether). */

#define MODEM_LOW_ACTIVITY_DEEPSLEEP_SECS 3

/* How recent activity level change to zero prevent deep-sleep completely
 * (allow faster state changes for modem library). */

#define MODEM_VERY_RECENT_ACTIVITY_OFF_MSECS    100

/* How recent activity level change to zero are translated to low-activity
 * level. */

#define MODEM_QUITE_RECENT_ACTIVITY_OFF_MSECS   500

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

#ifndef MODEM_PM_DEBUG
#  define modem_pm_lldbg(...) ((void)0)
#else
#  define modem_pm_lldbg(...) lldbg(__VA_ARGS__)
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

static struct
{
  unsigned int    count;
  struct timespec last_mod_time;
} g_modem_activity[__BOARD_MODEM_ACTIVITY_MAX];

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
  uint32_t gpio_off_mask = ~(GPIO_PUPD_MASK | GPIO_MODE_MASK | GPIO_OUTPUT_SET);

  if (poweredoff)
    {
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

      stm32_configgpio((GPIO_MODEM_TX_BURST & gpio_off_mask) |
                       (GPIO_OUTPUT_CLEAR | GPIO_OUTPUT));

      /* We need to configure POWER_ON and RESET_N as analog input for lower
       * power consumption. */

      stm32_configgpio((GPIO_MODEM_RESET_N & gpio_off_mask) | GPIO_ANALOG);
      stm32_configgpio((GPIO_MODEM_POWER_ON & gpio_off_mask) | GPIO_ANALOG);
    }
  else
    {
      /* Configure modem GPIOs for serial-port AltFunc. */

      stm32_configgpio(GPIO_USART3_TX);
      stm32_configgpio(GPIO_USART3_RTS);
      stm32_configgpio(GPIO_USART3_RX);
      stm32_configgpio(GPIO_USART3_CTS);

      /* TX_BURST currently unused, configure as analog input for low-power. */

      stm32_configgpio((GPIO_MODEM_TX_BURST & gpio_off_mask) | GPIO_ANALOG);

      /* Setup control GPIOs for modem. */

      stm32_configgpio(GPIO_MODEM_POWER_ON);
      stm32_configgpio(GPIO_MODEM_RESET_N);
    }
}

/****************************************************************************
 * Name: board_modem_setup_suspend_gpios
 ****************************************************************************/

static void board_modem_setup_suspend_gpios(bool suspend)
{
  uint32_t gpio_off_mask = ~(GPIO_PUPD_MASK | GPIO_MODE_MASK | GPIO_OUTPUT_SET);
  bool poweredoff = stm32_gpioread(GPIO_PWR_SWITCH_MODEM);

  if (poweredoff)
    {
      /* No need to change configuration for suspend when modem has been
       * powered off. */

      return;
    }

  if (suspend)
    {
      /* Set inputs as floating analog input. Ring-int line is configured for
       * EXTI interrupt and is left as is. */

      stm32_configgpio((GPIO_USART3_RX & gpio_off_mask) | GPIO_ANALOG);
      stm32_configgpio((GPIO_USART3_CTS & gpio_off_mask) | GPIO_ANALOG);
      stm32_configgpio((GPIO_MODEM_TX_BURST & gpio_off_mask) | GPIO_ANALOG);

      /* These lines have external pull-up and can be switched to analog input
       * mode. */

      stm32_configgpio((GPIO_MODEM_POWER_ON & gpio_off_mask) | GPIO_ANALOG);
      stm32_configgpio((GPIO_MODEM_RESET_N & gpio_off_mask) | GPIO_ANALOG);
    }
  else
    {
      /* Restore inputs. */

      stm32_configgpio(GPIO_USART3_RX);
      stm32_configgpio(GPIO_USART3_CTS);
      stm32_configgpio((GPIO_MODEM_TX_BURST & gpio_off_mask) | GPIO_ANALOG);

      /* Restore outputs. */

      stm32_configgpio(GPIO_MODEM_POWER_ON);
      stm32_configgpio(GPIO_MODEM_RESET_N);
    }
}

/****************************************************************************
 * Name: modem_msecs_from_recent_activity
 ****************************************************************************/

static int64_t modem_msecs_from_recent_activity(void)
{
  struct timespec currts;
  const struct timespec *lastoff;
  int64_t diff_msecs;
  int i;
  int64_t min_msecs = INT64_MAX;

  clock_gettime(CLOCK_MONOTONIC, &currts);

  /* Check if activity reached zero recently. */

  for (i = 0; i < __BOARD_MODEM_ACTIVITY_MAX; i++)
    {
      lastoff = &g_modem_activity[i].last_mod_time;

      if (g_modem_activity[i].count > 0)
        {
          continue;
        }

      diff_msecs = currts.tv_sec - lastoff->tv_sec;
      diff_msecs *= MSEC_PER_SEC;
      diff_msecs += (currts.tv_nsec - lastoff->tv_nsec) / NSEC_PER_MSEC;

      if (diff_msecs < min_msecs)
        {
          min_msecs = diff_msecs;
          if (min_msecs <= 0)
            {
              break;
            }
        }
    }

  return min_msecs;
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
 * Name: board_modem_rts_pin_ctrl
 *
 * Description:
 *   Control MODEM RTS/CTS gpio.
 *
 * Input Parameters:
 *   ctrl: set pin LOW/HIGH or restore RTS mode or read RTS pin.
 *
 ****************************************************************************/

bool board_modem_pin_ctrl(enum e_board_modem_pin pin,
                          enum e_board_modem_pin_ctrl ctrl)
{
  uint32_t pinset;

  switch (pin)
    {
      case BOARD_MODEM_PIN_RTS:
        pinset = GPIO_USART3_RTS;
        break;
      case BOARD_MODEM_PIN_CTS:
        pinset = GPIO_USART3_CTS;
        break;
      case BOARD_MODEM_PIN_TX_BURST:
        pinset = GPIO_MODEM_TX_BURST;
        break;
      default:
        DEBUGASSERT(false);
        return false;
    }

  switch (ctrl)
    {
      case BOARD_MODEM_PIN_CTRL_ALT_MODE:
        stm32_configgpio(pinset);
        return false;

      case BOARD_MODEM_PIN_CTRL_LOW:
        stm32_configgpio((pinset & ~GPIO_MODE_MASK) |
                         (GPIO_OUTPUT | GPIO_OUTPUT_CLEAR));
        return false;

      case BOARD_MODEM_PIN_CTRL_HIGH:
        stm32_configgpio((pinset & ~GPIO_MODE_MASK) |
                         (GPIO_OUTPUT | GPIO_OUTPUT_SET));
        return false;

      case BOARD_MODEM_PIN_CTRL_READ:
        return stm32_gpioread(pinset & ~GPIO_MODE_MASK);

      default:
        DEBUGASSERT(false);
        return false;
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

  memset(&g_modem_activity, 0, sizeof(g_modem_activity));

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
 * Name: board_set_modem_activity
 *
 * Description:
 *   Change activity state, for controlling power-saving/deep-sleep activity
 *
 ****************************************************************************/

void board_set_modem_activity(enum e_board_modem_activity type, bool active)
{
  irqstate_t flags;
  unsigned int new_count;

  flags = irqsave();

  if (active)
    {
      new_count = g_modem_activity[type].count + 1;

      modem_pm_lldbg("%s activity count increase: %d => %d\n",
                     type == BOARD_MODEM_ACTIVITY_HIGH ? "high" : "low",
                     g_modem_activity[type].count, new_count);
      DEBUGASSERT(new_count > g_modem_activity[type].count);

      g_modem_activity[type].count = new_count;
    }
  else
    {
      new_count = g_modem_activity[type].count - 1;

      modem_pm_lldbg("%s activity count decrease: %d => %d\n",
                     type == BOARD_MODEM_ACTIVITY_HIGH ? "high" : "low",
                     g_modem_activity[type].count, new_count);
      DEBUGASSERT(new_count < g_modem_activity[type].count);

      g_modem_activity[type].count = new_count;
    }

  clock_gettime(CLOCK_MONOTONIC, &g_modem_activity[type].last_mod_time);

  irqrestore(flags);
}

/****************************************************************************
 * Name: up_modem_gpio_suspend
 *
 * Description:
 *   Set modem GPIOs to suspended state
 *
 ****************************************************************************/

void up_modem_gpio_suspend(bool suspend)
{
  board_modem_setup_suspend_gpios(suspend);
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

/****************************************************************************
 * Name: up_modem_deep_sleep_readiness
 *
 * Description:
 *   Check current modem activity and adjust deep-sleep accordingly.
 *
 * Input Parameters:
 *   deepsleep_msecs:   Pointer to deep-sleep wake-up interval
 *
 * Return:
 *   Return true if deep-sleep allowed.
 *
 ****************************************************************************/

bool up_modem_deep_sleep_readiness(uint32_t *deepsleep_msecs)
{
  uart_dev_t *serdev;
  irqstate_t flags;
  bool ret;
  int64_t recent_activity_msecs = -1;

  flags = irqsave();

  if (!stm32_gpioread(GPIO_PWR_SWITCH_MODEM))
    {
      /* Modem powered off, allow deep-sleep. */

      ret = true;
      goto check_low_activity;
    }

#ifdef SERIAL_HAVE_DMA
  /* Trigger serial Rx DMA poll to fetch data from DMA buffer. */

  stm32_serial_dma_poll();
#endif

  /* Get modem serial port (USART3) instance. */

  serdev = stm32_serial_get_uart(MODEM_USART_NUM);
  if (!serdev)
    {
      /* Port uninitialized? */

      ret = true;
      goto check_low_activity;
    }

  /* Check if modem serial (USART3) has data pending (Rx & Tx). */

  if (serdev->xmit.head != serdev->xmit.tail)
    {
      modem_pm_lldbg("modem USART Tx busy\n");

      ret = false;
      goto out;
    }
  if (serdev->recv.head != serdev->recv.tail)
    {
      modem_pm_lldbg("modem USART Rx busy\n");

      ret = false;
      goto out;
    }

  /* Check modem activity levels (reported from modem library to board
   * level). */

  if (g_modem_activity[BOARD_MODEM_ACTIVITY_HIGH].count > 0)
    {
      /* High activity level, prevents deep-sleep. */

      modem_pm_lldbg("modem high activity, busy\n");

      ret = false;
      goto out;
    }

  /* Let very recent activity change to zero to prevent deep-sleep. */

  recent_activity_msecs = modem_msecs_from_recent_activity();
  if (recent_activity_msecs <= MODEM_VERY_RECENT_ACTIVITY_OFF_MSECS)
    {
      modem_pm_lldbg("modem very recent activity, busy\n");

      ret = false;
      goto out;
    }

check_low_activity:

  ret = true;

  /* Check if deep-sleep seconds should be reduced based on low activity. */

  if (*deepsleep_msecs < MODEM_LOW_ACTIVITY_DEEPSLEEP_SECS * 1000 &&
      *deepsleep_msecs != 0)
    {
      /* Deep-sleep seconds already below threshold. */

      goto out;
    }

  if (g_modem_activity[BOARD_MODEM_ACTIVITY_LOW].count > 0)
    {
      /* Low activity level, reduce deep-sleep interval. */

      modem_pm_lldbg("modem low activity, deep-sleep secs %d=>%d\n",
                     *deepsleep_secs, MODEM_LOW_ACTIVITY_DEEPSLEEP_SECS);

      *deepsleep_msecs = MODEM_LOW_ACTIVITY_DEEPSLEEP_SECS * 1000;
    }
  else
    {
      if (recent_activity_msecs == -1)
        {
          recent_activity_msecs = modem_msecs_from_recent_activity();
        }

      if (recent_activity_msecs <= MODEM_QUITE_RECENT_ACTIVITY_OFF_MSECS)
        {
          /* Recent activity, reduce deep-sleep interval. */

          modem_pm_lldbg("modem recent activity (%lld), "
                         "deep-sleep secs %d=>%d\n",
                         recent_activity_msecs, *deepsleep_secs,
                         MODEM_LOW_ACTIVITY_DEEPSLEEP_SECS);

          *deepsleep_msecs = MODEM_LOW_ACTIVITY_DEEPSLEEP_SECS * 1000;
        }
    }

out:
  irqrestore(flags);
  return ret;
}

