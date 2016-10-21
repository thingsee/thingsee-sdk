/****************************************************************************
 * apps/system/ubmodem/ubmodem_substate_sim.c
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

#define SIM_TRY_DELAY_MSEC 2000
#define SIM_MAX_RETRIES 5

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at_cmd_def_s cmd_ATpCPIN =
{
  .name         = "+CPIN",
  .resp_format  = (const uint8_t[]){ RESP_FMT_STRING_TO_EOL },
  .resp_num     = 1,
  .timeout_dsec = MODEM_CMD_SIM_MGMT_TIMEOUT,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void check_ATpCPIN_handler(struct ubmodem_s *modem,
                                  const struct at_cmd_def_s *cmd,
                                  const struct at_resp_info_s *info,
                                  const uint8_t *resp_stream,
                                  size_t stream_len, void *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void set_ATpCPIN_handler(struct ubmodem_s *modem,
                                const struct at_cmd_def_s *cmd,
                                const struct at_resp_info_s *info,
                                const uint8_t *resp_stream,
                                size_t stream_len, void *priv)
{
  struct modem_sub_setup_sim_s *sub = priv;
  int err;

  /*
   * Response handler for AT+CPIN="<pin>"
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCPIN);

  if (resp_status_is_error_or_timeout(info->status))
    {
      /* Wrong PIN? */

      __ubmodem_common_failed_command(modem, cmd, info, "=\"<pin>\"");
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Check PIN/SIM state. */

  memset(sub, 0, sizeof(*sub));
  sub->try_count = 0;
  sub->first_check = false;

  err = __ubmodem_send_cmd(modem, &cmd_ATpCPIN, check_ATpCPIN_handler,
                       sub, "%s", "?");
  MODEM_DEBUGASSERT(modem, err == OK);
}

static int modem_setup_sim_retry_timer_handler(struct ubmodem_s *modem,
                                               const int timer_id,
                                               void * const arg)
{
  int err;
  struct modem_sub_setup_sim_s *sub = arg;

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);
  err = __ubmodem_send_cmd(modem, &cmd_ATpCPIN, check_ATpCPIN_handler,
                       sub, "%s", "?");
  MODEM_DEBUGASSERT(modem, err == OK);
  return OK;
}

static void check_ATpCPIN_handler(struct ubmodem_s *modem,
                                  const struct at_cmd_def_s *cmd,
                                  const struct at_resp_info_s *info,
                                  const uint8_t *resp_stream,
                                  size_t stream_len, void *priv)
{
  char buf[16];
  const char *str = NULL;
  struct modem_sub_setup_sim_s *sub = priv;
  int err;
  bool ok;

  /*
   * Response handler for AT+CPIN?
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCPIN);

  if (resp_status_is_error(info->status) &&
      info->errorcode == 3 && ++sub->try_count < SIM_MAX_RETRIES)
  {
    /*
     * After power-on, SIM is not accessible for short period.
     * Sleep for sometime and retry.
     */

    ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);
    err = __ubmodem_set_timer(modem, SIM_TRY_DELAY_MSEC,
                            &modem_setup_sim_retry_timer_handler,
                            sub);
    if (err == ERROR)
      {
        /* Error here? Add assert? Or just try bailout? */

        MODEM_DEBUGASSERT(modem, false);

        (void)modem_setup_sim_retry_timer_handler(modem, -1, sub);
      }

    return;
  }

  if (resp_status_is_error_or_timeout(info->status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "?");
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Check response */

  ok = __ubmodem_stream_get_string(&resp_stream, &stream_len, &str, NULL);
  MODEM_DEBUGASSERT(modem, ok);

  if (strcmp(str, "READY") == 0)
    {
      /* PIN not needed. */

      __ubmodem_reached_level(modem, UBMODEM_LEVEL_SIM_ENABLED);

      return;
    }

  if (sub->first_check && strcmp(str, "SIM PIN") == 0)
    {
      /* PIN is needed */

      ok = __ubmodem_config_get_value(modem, "modem.pin", buf, sizeof(buf));
      if (!ok)
        {
          /* Cannot setup PIN */

          __ubmodem_level_transition_failed(modem, "PIN required.");

          return;
        }

      err = __ubmodem_send_cmd(modem, &cmd_ATpCPIN, set_ATpCPIN_handler,
                           sub, "=\"%s\"", buf);
      MODEM_DEBUGASSERT(modem, err == OK);
      return;
    }

  /* PUK or something else needed? */

  __ubmodem_level_transition_failed(modem, "AT%s: %s.", cmd->name, str);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_substate_start_setup_sim
 *
 * Description:
 *   Start modem SIM setup sequence.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_setup_sim(struct ubmodem_s *modem)
{
  struct modem_sub_setup_sim_s *sub = &modem->sub.setup_sim;
  int err;

  MODEM_DEBUGASSERT(modem, modem->level == UBMODEM_LEVEL_CMD_PROMPT);

  /*
   * Setup SIM sequence is:
   *
   * 1. Send "AT+CPIN?".
   * 2. Check if PIN is already set (or not required).
   * 3. Send 'AT+CPIN="<pin>"' if not set.
   * 4. Check if PIN was accepted.
   */

  /* Reset sub-state data and initiate sub-state machine work. */

  memset(sub, 0, sizeof(*sub));
  sub->try_count = 0;
  sub->first_check = true;

  /* Setup SIM after short delay, to give HW time to initialize. */

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);
  err = __ubmodem_set_timer(modem, SIM_TRY_DELAY_MSEC,
                          &modem_setup_sim_retry_timer_handler,
                          sub);
  if (err == ERROR)
    {
      /* Error here? Add assert? Or just try bailout? */

      MODEM_DEBUGASSERT(modem, false);

      (void)modem_setup_sim_retry_timer_handler(modem, -1, sub);
    }
}

