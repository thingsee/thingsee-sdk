/****************************************************************************
 * configs/stm32vldiscovery/src/stm32_leds.c
 *
 *   Copyright (C) 2012-2013, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Freddie Chopin <freddie_chopin@op.pl>
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/power/pm.h>
#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "stm32.h"
#include "stm32vldiscovery.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG_LEDS enables debug output from this file (needs CONFIG_DEBUG
 * with CONFIG_DEBUG_VERBOSE too)
 */

#ifdef CONFIG_DEBUG_LEDS
#  define leddbg  lldbg
#  define ledvdbg llvdbg
#else
#  define leddbg(x...)
#  define ledvdbg(x...)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ledinit
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void stm32_ledinit(void)
{
  stm32_configgpio(GPIO_LED1);    /* Configure LED1 GPIO for output */
}

/****************************************************************************
 * Name: board_led_on
 ****************************************************************************/

void board_led_on(int led)
{
  if (led == 1)
   {
     stm32_gpiowrite(GPIO_LED1, true);
   }
}

/****************************************************************************
 * Name: board_led_off
 ****************************************************************************/

void board_led_off(int led)
{
  if (led == 0)
    {
      stm32_gpiowrite(GPIO_LED1, false);
    }
}

#endif /* CONFIG_ARCH_LEDS */
