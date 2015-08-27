/****************************************************************************
 * configs/haltian-tsone/src/up_autoleds.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <arch/board/board-device.h>

#include "chip.h"
#include "stm32.h"
#include "haltian-tsone.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 2 LEDs on
 * board the STM32L-Discovery.  The following definitions describe how NuttX
 * controls the LEDs:
 *
 *   SYMBOL                Meaning                 LED state
 *                                                   LED1     LED2     LED3     LED4
 *   -------------------  -----------------------  -------- -------- -------- --------
 *   LED_STARTED          NuttX has been started     OFF      OFF      OFF      OFF
 *   LED_HEAPALLOCATE     Heap has been allocated    OFF      OFF      OFF      OFF
 *   LED_IRQSENABLED      Interrupts enabled         OFF      OFF      OFF      OFF
 *   LED_STACKCREATED     Idle stack created         ON       OFF      OFF      OFF
 *   LED_INIRQ            In an interrupt            <NC>     ON       <NC>     <NC>
 *   LED_SIGNAL           In a signal handler        <NC>     <NC>     ON       <NC>
 *   LED_ASSERTION        An assertion failed        <NC>     <NC>     <NC>     ON
 *   LED_PANIC            The system has crashed     Blinking <NC>     <NC>     <NC>
 *   LED_IDLE             STM32 is is sleep mode     <NC>     <NC>     <NC>     <NC>
 */

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
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_led_initialize
 ****************************************************************************/

void board_led_initialize(void)
{
  /* Configure LED1 GPIO for output */

  if (board_get_hw_ver() >= BOARD_HWVER_B2_0)
    stm32_configgpio(GPIO_LED1);
}

/****************************************************************************
 * Name: board_led_on
 ****************************************************************************/

void board_led_on(int led_bits)
{
  if (board_get_hw_ver() >= BOARD_HWVER_B2_0)
    {
      if (led_bits & (BOARD_LED1_BIT | BOARD_LED_ALL_OFF))
        {
          stm32_gpiowrite(GPIO_LED1, !!(led_bits & BOARD_LED1_BIT));
        }
    }
}

/****************************************************************************
 * Name: board_led_off
 ****************************************************************************/

void board_led_off(int led_bits)
{
  if (board_get_hw_ver() >= BOARD_HWVER_B2_0)
    {
      if (led_bits & (BOARD_LED1_BIT | BOARD_LED_ALL_OFF))
        {
          stm32_gpiowrite(GPIO_LED1, false);
        }
    }
}

#endif /* CONFIG_ARCH_LEDS */
