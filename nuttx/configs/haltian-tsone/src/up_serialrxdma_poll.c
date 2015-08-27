/****************************************************************************
 * config/haltian-tsone/src/up_serialrxdma_poll.c
 * arch/arm/src/board/up_serialrxdma_poll.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
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

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>

#include <arch/board/board.h>

#include "haltian-tsone.h"

#include "stm32.h"
#include "stm32_uart.h"
#include "up_serialrxdma_poll.h"

#ifdef SERIAL_HAVE_DMA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SERIAL_RXDMA_POLL_MSEC (1000 / 25)

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct
{
  WDOG_ID wid;
  bool has_been_launched:1;
} g_serialrxdma_poll;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void up_serialrxdma_poller(int argc, uint32_t arg1, ...)
{
  irqstate_t flags;

  flags = irqsave();
  g_serialrxdma_poll.has_been_launched = false;
  irqrestore(flags);

  /* Poll RX DMA on serial ports. */

  stm32_serial_dma_poll();

  /* Relaunch timer. */

  up_launch_serialrxdma_poll();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_launch_serialrxdma_poll(void)
{
  irqstate_t flags;
  bool has_port_enabled = false;
  int ret;

  flags = irqsave();

  /*
   * Check if any of serial ports are active (chip at other end is powered
   * on).
   */

#ifdef CONFIG_USART1_RXDMA
  has_port_enabled = has_port_enabled || stm32_gpioread(GPIO_REGULATOR_GPS);
#endif
#ifdef CONFIG_USART2_RXDMA
  has_port_enabled = has_port_enabled || stm32_gpioread(GPIO_REGULATOR_BLUETOOTH);
#endif
#ifdef CONFIG_USART3_RXDMA
  has_port_enabled = has_port_enabled || stm32_gpioread(GPIO_PWR_SWITCH_MODEM);
#endif

  /*
   * Start timer only if there is some USART device enabled and work has
   * not started yet.
   */

  if (has_port_enabled && !g_serialrxdma_poll.has_been_launched)
    {
      if (!g_serialrxdma_poll.wid)
        {
          g_serialrxdma_poll.wid = wd_create();
          if (!g_serialrxdma_poll.wid)
            {
              dbg("Could not create watchdog timer!\n");
            }
        }

      if (g_serialrxdma_poll.wid)
        {
          ret = wd_start(g_serialrxdma_poll.wid,
                         MSEC2TICK(SERIAL_RXDMA_POLL_MSEC),
                         up_serialrxdma_poller, 0);
          DEBUGASSERT(ret == OK); /* Returns error only on invalid input. */

          g_serialrxdma_poll.has_been_launched = true;
        }
    }

  irqrestore(flags);
}

#endif /* SERIAL_HAVE_DMA */

