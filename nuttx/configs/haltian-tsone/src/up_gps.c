/****************************************************************************
 * configs/haltian-tsone/src/up_gps.c
 *
 *   Copyright (C) 2014-2017 Haltian Ltd. All rights reserved.
 *   Authors: Sami Pelkonen <sami.pelkonen@haltian.com>
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
#include <sys/ioctl.h>

#include "chip.h"
#include "stm32_gpio.h"

#include <arch/board/board.h>
#include <arch/board/board-gps.h>
#include <arch/board/board-pwrctl.h>
#include "haltian-tsone.h"
#include "up_serialrxdma_poll.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How many milliseconds before given 'message expected' time MCU needs to
 * wake-up to reliably receive message. */

#define PM_WAKE_TIME_OFFSET_MSEC 500

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct {
  struct timespec pm_wake_abstime;
  bool powered;
} g_board_gps;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void board_gps_setup_gpios(bool on)
{
  const uint32_t ppmask = GPIO_PIN_MASK | GPIO_PORT_MASK;

  if (!on)
    {
      /* Make GPIOs analog input. */

      stm32_configgpio((GPIO_USART1_RX & ppmask) | GPIO_ANALOG);
      stm32_configgpio((GPIO_USART1_TX & ppmask) | GPIO_ANALOG);
    }
  else
    {
      /* Enable USART GPIOs */

      stm32_configgpio(GPIO_USART1_RX);
      stm32_configgpio(GPIO_USART1_TX);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gps_power
 *
 * Description:
 *   Power GPS chip on / off
 *
 ****************************************************************************/

void board_gps_power(bool on)
{
  if (!g_board_gps.powered && on)
    {
      /* Power on GPS chip */

      board_pwrctl_get(PWRCTL_REGULATOR_GPS);
      board_gps_setup_gpios(true);

      g_board_gps.powered = true;
      up_launch_serialrxdma_poll();
    }
  else if (g_board_gps.powered && !on)
    {
      /* Power off GPS chip */

      board_gps_setup_gpios(false);
      board_pwrctl_put(PWRCTL_REGULATOR_GPS);

      g_board_gps.powered = false;
      g_board_gps.pm_wake_abstime.tv_sec = 0;
      g_board_gps.pm_wake_abstime.tv_nsec = 0;
    }
}

/****************************************************************************
 * Name: board_gps_initialize
 *
 * Description:
 *   Initialize the u-blox GPS.
 *
 * Return:
 *   Opened GPS serial port file descriptor.
 *
 ****************************************************************************/

int board_gps_initialize(void)
{
  g_board_gps.pm_wake_abstime.tv_sec = 0;
  g_board_gps.pm_wake_abstime.tv_nsec = 0;

  /* Open serial. */

  return open(GPS_SERIAL_DEVNAME, O_RDWR, 0666);
}

/****************************************************************************
 * Name: board_gps_pm_set_next_message_time
 *
 * Description:
 *   Power-management hint for board level. GPS library can inform board level
 *   when next (navigation) message is expected to be received over UART.
 *   Expected time is given as absolute time in CLOCK_MONOTONIC domain.
 *
 ****************************************************************************/

void board_gps_pm_set_next_message_time(const struct timespec *msg_abstime)
{
  irqstate_t flags;

  flags = irqsave();

  g_board_gps.pm_wake_abstime = *msg_abstime;

  /* Prepare to wake-up slightly early, to allow MCU UART peripheral to
   * activate. */

  g_board_gps.pm_wake_abstime.tv_nsec -=
      PM_WAKE_TIME_OFFSET_MSEC * NSEC_PER_MSEC;
  while (g_board_gps.pm_wake_abstime.tv_nsec < 0)
    {
      g_board_gps.pm_wake_abstime.tv_nsec += NSEC_PER_SEC;
      g_board_gps.pm_wake_abstime.tv_sec--;
    }

  irqrestore(flags);
}

/****************************************************************************
 * Name: up_gps_deep_sleep_readiness
 *
 * Description:
 *   Check when next GPS message is expected and adjust deep-sleep accordingly.
 *
 * Input Parameters:
 *   deepsleep_msecs:   Pointer to deep-sleep wake-up interval
 *
 * Return:
 *   Return true if deep-sleep allowed.
 *
 ****************************************************************************/

bool up_gps_deep_sleep_readiness(uint32_t *deepsleep_msecs)
{
  uart_dev_t *serdev;
  struct timespec ts;
  int32_t wake_msecs;
  irqstate_t flags;
  int ret;

  if (!stm32_gpioread(GPIO_REGULATOR_GPS))
    {
      /* GPS powered-off, allow deep-sleep. */

      return true;
    }

#ifdef SERIAL_HAVE_DMA
  /* Trigger serial Rx DMA poll to fetch data from DMA buffer. */

  stm32_serial_dma_poll();
#endif

  /* Get GPS serial port (USART1) instance. */

  serdev = stm32_serial_get_uart(GPS_USART_NUM);
  if (!serdev)
    {
      /* Port uninitialized? GPS powered-off? */

      return true;
    }

  /* Check if GPS serial (USART1) has data pending (Rx & Tx). */

  if (serdev->xmit.head != serdev->xmit.tail)
    {
      return false;
    }
  if (serdev->recv.head != serdev->recv.tail)
    {
      return false;
    }

  /* Check if reported next message time is in future. */

  ret = clock_gettime(CLOCK_MONOTONIC, &ts);
  DEBUGASSERT(ret != ERROR);

  flags = irqsave();

  if (g_board_gps.pm_wake_abstime.tv_sec < ts.tv_sec ||
      (g_board_gps.pm_wake_abstime.tv_sec == ts.tv_sec &&
       g_board_gps.pm_wake_abstime.tv_nsec <= ts.tv_nsec))
    {
      /* We are past wake time, do not allow deep-sleep. */

      ret = false;
      goto out;
    }

  wake_msecs = g_board_gps.pm_wake_abstime.tv_sec - ts.tv_sec;
  if (wake_msecs > (INT32_MAX / 1000))
    {
      /* Wake-time far in future, allow deep-sleep. */

      ret = true;
      goto out;
    }

  wake_msecs *= 1000;
  wake_msecs += (g_board_gps.pm_wake_abstime.tv_nsec - ts.tv_nsec) /
                NSEC_PER_MSEC;

  if (wake_msecs < *deepsleep_msecs)
    {
      /* Set new deep-sleep wake-up timer. */

      *deepsleep_msecs = wake_msecs;
    }

  /* Allow deep-sleep if there is time left to wake-up. */

  ret = (wake_msecs > 0);

out:
  irqrestore(flags);
  return ret;
}

/****************************************************************************
 * Name: board_gps_deinitialize
 *
 * Description:
 *   Deinitialize the u-blox GPS.
 *
 ****************************************************************************/

int board_gps_deinitialize(int fd)
{
  /* Close file-descriptor. */

  return close(fd);
}

/****************************************************************************
 * Name: board_gps_tx_buffer_empty
 *
 * Description:
 *   Check whether GPS UART TX buffer is empty
 *
 ****************************************************************************/

bool board_gps_tx_buffer_empty(int fd)
{
  int bytes_free;
  int ret;

#ifdef SERIAL_HAVE_DMA
  /* Trigger serial Rx DMA poll to fetch data from DMA buffer. */

  stm32_serial_dma_poll();
#endif

  ret = ioctl(fd, FIONWRITE, (unsigned long)&bytes_free);
  if (ret < 0)
    return true;

  /* Serial driver buffer holds N bytes, but has max capacity N-1 */

  return bytes_free == (CONFIG_USART1_TXBUFSIZE - 1);
}

/****************************************************************************
 * Name: up_gps_initialize_gpios
 *
 * Description:
 *   Set GPS GPIOs to initial state.
 *
 ****************************************************************************/

void up_gps_initialize_gpios(void)
{
  /* Setup serial GPIOs for modem (GPS is powered off). */

  board_gps_setup_gpios(false);
}
