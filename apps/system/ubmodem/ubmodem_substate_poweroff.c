/****************************************************************************
 * apps/system/ubmodem/ubmodem_substate_poweroff.c
 *
 *   Copyright (C) 2014-2017 Haltian Ltd. All rights reserved.
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
#include <stdbool.h>
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

static const struct at_cmd_def_s cmd_ATpCPWROFF =
{
  .name         = "+CPWROFF",
  .resp_format  = NULL,
  .resp_num     = 0,
  .timeout_dsec = MODEM_CMD_POWEROFF_TIMEOUT,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int modem_do_power_off_timer_handler(struct ubmodem_s *modem,
                                            const int timer_id,
                                            void * const arg)
{
  modem->poweroff_timer = -1;

  /* Completed sleeping, modem should be powered off now. We can turn off
   * modem power-switch. */

  modem->is_vcc_off = !ubmodem_hw_vcc_set(modem, false);

  modem->is_powered_off = true;
  __ubmodem_reached_level(modem, UBMODEM_LEVEL_POWERED_OFF);

  return OK;
}

static void ATpCPWROFF_handler(struct ubmodem_s *modem,
                               const struct at_cmd_def_s *cmd,
                               const struct at_resp_info_s *info,
                               const uint8_t *resp_stream,
                               size_t stream_len, void *priv)
{
  int err;

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCPWROFF);

  if (resp_status_is_error_or_timeout(info->status))
    {
      /*
       * Failed power-off? Inform about stuck hardware and proceed with
       * hardware reset.
       */

      __ubmodem_reached_level(modem, UBMODEM_LEVEL_STUCK_HARDWARE);

      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  if (modem->target_level > UBMODEM_LEVEL_POWERED_OFF)
    {
      /* High target level has been requested, we can skip full power-off. */

      modem->is_powered_off = true;
      __ubmodem_reached_level(modem, UBMODEM_LEVEL_POWERED_OFF);
    }
  else
    {
      /* After "OK" response to AT+CPWROFF, we need to wait several seconds.
       *
       * "Figure 14: SARA-G3 series power-off sequence description" in "SARA-G3
       * series - System Integration Manual" shows that time to get OK from modem
       * is ~2.5 seconds and module becomes powered-off after another ~2.5 seconds.
       * However, documentations states "The duration of the switch-off routine
       * phases can largely differ from the values reported in Figure 14,
       * depending on the network settings and the concurrent activities of the
       * module performing a network detach". Wait for 5 seconds before fully
       * powering off modem.
       */

      err = __ubmodem_set_timer(modem, 5000, &modem_do_power_off_timer_handler,
                                modem);
      if (err == ERROR)
        {
          /* Error here? Add assert? Or just try bailout? */

          MODEM_DEBUGASSERT(modem, false);

          (void)modem_do_power_off_timer_handler(modem, -1, modem);
        }
      else
        {
          modem->poweroff_timer = err;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_cancel_full_poweroff
 *
 * Description:
 *   Cancel timer for full modem power-off.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_cancel_full_poweroff(struct ubmodem_s *modem)
{
  if (modem->target_level > UBMODEM_LEVEL_POWERED_OFF &&
      modem->poweroff_timer > -1)
    {
      __ubmodem_remove_timer(modem, modem->poweroff_timer);
      modem->poweroff_timer = -1;

      modem->is_powered_off = true;
      __ubmodem_reached_level(modem, UBMODEM_LEVEL_POWERED_OFF);
    }
}

/****************************************************************************
 * Name: __ubmodem_substate_start_do_power_off
 *
 * Description:
 *   Start modem power-off sequence/sub-state machine.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_do_power_off(struct ubmodem_s *modem)
{
  int err;

  /*
   * Power-off sequence is:
   *
   * 1. Send "AT+CPWROFF".
   * 2. Response "OK".
   *    ==> modem_do_power_off_handler()
   * 3. One-shot timer for ~2.5 seconds.
   *    ==> modem_do_power_off_timer_handler()
   * 4. Done.
   */

  __ubmodem_network_cleanup(modem);

#ifdef CONFIG_UBMODEM_VOICE
  __ubmodem_voice_control_cleanup(modem);
#endif

  /* Send power off command. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpCPWROFF, ATpCPWROFF_handler, modem, "");
  MODEM_DEBUGASSERT(modem, err == OK);
}
