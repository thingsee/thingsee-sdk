/************************************************************************************
 * configs/haltian-tsone/src/up_pwr_exti.c
 * arch/arm/src/board/up_pwr_exti.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *
 * Authors:
 *   Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
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
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <debug.h>

#include <arch/board/board.h>
#include <arch/board/board-reset.h>
#include <arch/board/board-battery.h>

#include "stm32.h"
#include "stm32_exti_pwr.h"

/* Exported global variable */

bool g_charger_connected = false;

/****************************************************************************
 * Name: board_pwr_charger_connected
 *
 * Description:
 *  This is called by charger module to inform board-level SW about charging
 *  status.
 *
 ****************************************************************************/

void board_pwr_charger_connected(const char *porttype)
{
  g_charger_connected = (porttype != NULL);
}

/****************************************************************************
 * Name: board_pwr_checkvbat
 *
 * Description:
 *  Check if operating voltage is too low, and go to standby mode if it is.
 *
 *  This is called from deepsleep code and mainloop as an alternative to
 *  PVD detection.
 *
 * Input Parameters:
 *  checktwice  - Do the check twice, useful when calling this from
 *                contexts where the 2G modem might be on.
 *
 ****************************************************************************/

void board_pwr_checkvbat(bool checktwice)
{
  int failures = 0;
  int checks = checktwice ? 2 : 1;

  if (!g_charger_connected)
    {
      int i, ret;
      uint32_t val = 0;

      for (i = 0; i < checks; i++)
        {
          ret = board_get_battery_voltage(&val);
          if (ret < 0)
            {
              /* We go to standby also when ADC fails. */

              lldbg("BUG BUG: VBAT Reset ADC failure! ret = %d\n", ret);
              failures = checks;
              break;
            }
          if (val < 3050)
            {
              lldbg("Maybe do VBAT Reset soon (voltage=%u mV)!\n", val);
              failures++;
            }

          /* Sleep 2.3ms. The value is chosen so that no two 2G modem
             TX bursts, of length 577us and period 4.615ms, can occur during
             both measurements. TX burst causes worst case 170mV voltage drop,
             which is normal behavior. */
          if (i < checks - 1)
            {
              up_udelay(2300);
            }
        }
      if (failures == checks)
        {
          lldbg("VBAT Reset! Going to standby mode...\n");
          board_go_to_standby();
        }
    }
}

/****************************************************************************
 * Name: board_pwr_checkpvd
 *
 * Description:
 *  Check if operating voltage is too low, and go to standby mode if it is.
 *
 *  This is called both from PVD interrupt handler and deepsleep code.
 *
 ****************************************************************************/

void board_pwr_checkpvd(void)
{
  uint32_t regval;
  irqstate_t flags;

  flags = irqsave();

  regval = getreg32(STM32_PWR_CSR);
  if ((regval & PWR_CSR_PVDO) && !g_charger_connected)
    {
      lldbg("PVD Reset (PWR_CSR=0x%02x)! Going to standby mode...\n", regval);
      board_go_to_standby();
    }

  irqrestore(flags);
}

/****************************************************************************
 * Name: stm32_pvd_interrupt
 *
 * Description:
 *  PVD interrupt handle
 *
 ****************************************************************************/

static int stm32_pvd_interrupt(int irq, void *context)
{
  if (irq == STM32_IRQ_PVD)
    {
      board_pwr_checkpvd();
    }

  return OK;
}

/****************************************************************************
 * Name: board_pwr_enablepvd
 *
 * Description:
 *  Triggers interrupt if operating voltage is too low
 *
 ****************************************************************************/

void board_pwr_enablepvd(void)
{
  /* Register interrupt handler. */

  stm32_exti_pvd(true, false, false, stm32_pvd_interrupt);

  /* Set Power Voltage Detector to 2.5V */

  stm32_pwr_setpvd(PWR_CR_2p5V);

  stm32_pwr_enablepvd();
}

