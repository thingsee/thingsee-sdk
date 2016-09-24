/****************************************************************************
 * apps/system/ubmodem/ubmodem_substate_poweron.c
 *
 *   Copyright (C) 2014-2016 Haltian Ltd. All rights reserved.
 *   Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <apps/system/ubmodem.h>

#include "ubmodem_internal.h"
#include "ubmodem_hw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int modem_do_power_on_timer_handler(struct ubmodem_s *modem,
                                           const int timer_id, void * const arg)
{
  struct modem_sub_power_on_s *sub = &modem->sub.power_on;
  uint32_t timer_msecs = 0;
  int err;

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);

  switch (sub->step++)
    {
    case 0:
      if (modem->is_vcc_off)
        {
          /* Modem module is fully powered-off. Turn on power. */

          modem->is_vcc_off = !ubmodem_hw_vcc_set(modem, true);

          /* Modem is now powering on and will reach UBMODEM_LEVEL_POWERED_ON
           * automatically. Give modem short time to power on. */

          timer_msecs = 1000;
          break;
        }

      sub->step++;

      /* No break */

    case 1:

      /* Pull reset pin down. */

      timer_msecs = ubmodem_hw_reset_pin_set(modem, false);
      break;

    case 2:
      /* Pull reset pin up. */

      timer_msecs = ubmodem_hw_reset_pin_set(modem, true);
      break;

    case 3:
      if (sub->is_reset && !modem->is_powered_off)
        {
          /* This is reset sequence and modem is already powered on, done. */

          __ubmodem_reached_level(modem, UBMODEM_LEVEL_POWERED_ON);

          return 0;
        }

      /* Pull power_on pin down. */

      timer_msecs = ubmodem_hw_poweron_pin_set(modem, false);
      break;

    case 4:

      /* Pull power_on pin up. */

      timer_msecs = ubmodem_hw_poweron_pin_set(modem, true);
      break;

    case 5:
      /*
       * Power-on completed.
       */

      modem->is_powered_off = false;

      __ubmodem_reached_level(modem, UBMODEM_LEVEL_POWERED_ON);

      return OK;
    }

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);
  err = __ubmodem_set_timer(modem, timer_msecs, &modem_do_power_on_timer_handler,
                          modem);
  if (err == ERROR)
    {
      /* Error here? Add assert? Or just try bailout? */

      MODEM_DEBUGASSERT(modem, false);

      (void)modem_do_power_on_timer_handler(modem, -1, modem);
    }

  return OK;
}

static int recover_modem_task(struct ubmodem_s *modem, void *privptr)
{
  /* Go to stuck hardware level from current and then retry current. */

  __ubmodem_network_cleanup(modem);
  __ubmodem_retry_current_level(modem, UBMODEM_LEVEL_POWERED_OFF);

  /* Return error to tell task starter that task did not start new work on
   * state machine. */

  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_substate_start_do_power_on
 *
 * Description:
 *   Start modem power-on sequence/sub-state machine.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_do_power_on(struct ubmodem_s *modem)
{
  struct modem_sub_power_on_s *sub = &modem->sub.power_on;

  MODEM_DEBUGASSERT(modem, modem->level == UBMODEM_LEVEL_POWERED_OFF);

  if (!modem->is_powered_off)
    {
      __ubmodem_reached_level(modem, UBMODEM_LEVEL_POWERED_ON);
      return;
    }

  /*
   * Power-on sequence is:
   *
   * 1. If VCC off: Turn VCC on.
   * 2. Pull RESET_N pin down, wait.
   * 3. Pull RESET_N pin up, wait.
   * 4. Pull POWER_ON pin down, wait.
   * 5. Pull POWER_ON pin up, wait.
   *
   */

  /* Reset sub-state data and initiate sub-state machine work. */

  memset(sub, 0, sizeof(*sub));
  sub->is_reset = false;
  sub->step = 0;

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);
  (void)modem_do_power_on_timer_handler(modem, -1, modem);
}

/****************************************************************************
 * Name: __ubmodem_substate_start_do_reset
 *
 * Description:
 *   Start modem reset sequence/sub-state machine.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_do_reset(struct ubmodem_s *modem)
{
  struct modem_sub_power_on_s *sub = &modem->sub.power_on;

  /*
   * Reset sequence is:
   *
   * 1. If VCC off: Turn VCC on.
   * 2. Pull RESET_N pin down, wait.
   * 3. Pull RESET_N pin up, wait.
   * 4. Done if modem was powered on.
   * 5. Pull POWER_ON pin down, wait.
   * 6. Pull POWER_ON pin up, wait.
   *
   */

  /* Reset sub-state data and initiate sub-state machine work. */

  memset(sub, 0, sizeof(*sub));
  sub->is_reset = true;
  sub->step = 0;

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);
  (void)modem_do_power_on_timer_handler(modem, -1, modem);
}

/****************************************************************************
 * Name: __ubmodem_recover_stuck_hardware
 *
 * Description:
 *   Recover/reset stuck modem hardware
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

int __ubmodem_recover_stuck_hardware(struct ubmodem_s *modem)
{
  /* Add modem task. */

  return __ubmodem_add_task(modem, recover_modem_task, NULL);
}
