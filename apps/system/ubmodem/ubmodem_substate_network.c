/****************************************************************************
 * apps/system/ubmodem/ubmodem_substate_network.c
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

#define NETWORK_REGISTRATION_CHECK_SECS 4

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at_cmd_def_s cmd_ATpCOPS =
{
  .name         = "+COPS",
  .resp_format  =
    (const uint8_t[]){
      RESP_FMT_INT8,
      RESP_FMT_INT8,
      RESP_FMT_QUOTED_STRING,
    },
  .resp_num     = 3,
  .timeout_dsec = MODEM_CMD_NETWORK_TIMEOUT,
};

static const struct at_cmd_def_s cmd_ATpCFUN =
{
  .name         = "+CFUN",
  .resp_format  = (const uint8_t[]){ RESP_FMT_INT8, RESP_FMT_INT8 },
  .resp_num     = 2,
  .timeout_dsec = MODEM_CMD_NETWORK_TIMEOUT,
};

static const struct at_cmd_def_s cmd_ATpCREG =
{
  .name         = "+CREG",
  .resp_format  = NULL,
  .resp_num     = 0,
};

static const struct at_cmd_def_s urc_ATpCREG =
{
  .name         = "+CREG",
  .resp_format  = (const uint8_t[]){ RESP_FMT_INT8 },
  .resp_num     = 1,
};

static const char *creg_strings[] =
{
  "MT not searching connection.",
  "Registered to home network.",
  "MT searching connection.",
  "Network registration denied.",
  "Unknown registration error.",
  "Registered to roaming network."
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*** Network +CREG URC monitor **********************************************/

#ifdef CONFIG_UBMODEM_CREG_WAIT_NETWORK_TO_RESTORE
static int reregister_network_failed(struct ubmodem_s *modem, void *priv)
{
  ubdbg("\n");

  /* Mark for retry, 'level transition failed' can modify the actual target
   * level. */

  __ubmodem_network_cleanup(modem);
  __ubmodem_retry_current_level(modem, UBMODEM_LEVEL_SIM_ENABLED);

  /* Could not reregister, shutdown modem. */

  __ubmodem_level_transition_failed(modem, "%s", "NETWORK: Lost connection.");

  /* Return error to tell task starter that task did not start new work on
   * state machine. */

  return ERROR;
}

static int network_reregister_timer_handler(struct ubmodem_s *modem,
                                            const int timer_id,
                                            void * const arg)
{
  MODEM_DEBUGASSERT(modem, modem->creg_timer_id == timer_id);

  /* One-shot timer, not registered anymore. */

  if (timer_id != -1)
    {
      /* One-shot timer, not registered anymore. */

      modem->creg_timer_id = -1;

      ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);
    }

  /* Issue new task. We cannot issue new state machine work from
   * URC 'context' as other work might be active in main state
   * machine. Task will be run with main state machine in proper
   * state. */

  __ubmodem_add_task(modem, reregister_network_failed, NULL);

  return OK;
}
#endif

static int retry_network_through_sim(struct ubmodem_s *modem, void *priv)
{
  ubdbg("\n");

  /* Go to SIM enabled level from current and then retry current. */

  __ubmodem_network_cleanup(modem);
  __ubmodem_retry_current_level(modem, UBMODEM_LEVEL_SIM_ENABLED);

  /* Return error to tell task starter that task did not start new work on
   * state machine. */

  return ERROR;
}

static void monitor_urc_pCREG_handler(struct ubmodem_s *modem,
                                      const struct at_cmd_def_s *cmd,
                                      const struct at_resp_info_s *info,
                                      const uint8_t *resp_stream,
                                      size_t stream_len, void *priv)
{
  int8_t val;
  int ret;

  (void)ret;

  /*
   * Response handler for +CREG URC.
   */

  MODEM_DEBUGASSERT(modem, cmd == &urc_ATpCREG);

  if (info->status != RESP_STATUS_URC)
    {
      /* Should not happen. */

      MODEM_DEBUGASSERT(modem, false);
      return;
    }

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &val))
    {
      /* Should not happen. */

      MODEM_DEBUGASSERT(modem, false);
      return;
    }

  dbg("Network registration URC, +CREG=%d.\n", val);

  if (val >= 0 && val < ARRAY_SIZE(creg_strings))
    {
      dbg("%s\n", creg_strings[val]);
    }
  else
    {
      dbg("Unknown +CREG status: %d\n", val);
    }

  if (modem->creg_timer_id >= 0)
    {
      __ubmodem_remove_timer(modem, modem->creg_timer_id);

      modem->creg_timer_id = -1;

      ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);
    }

  switch (val)
    {
      default: /* Unknown? */
      case 1: /* Switched to home network? */
      case 5: /* Switched to roaming? */
        return;

      case 2: /* Looking for connection. */
#ifdef CONFIG_UBMODEM_CREG_WAIT_NETWORK_TO_RESTORE
        /* Start connection search timeout timer. */

        ret = __ubmodem_set_timer(modem, MODEM_CMD_NETWORK_TIMEOUT * 100,
                                  &network_reregister_timer_handler, modem);
        MODEM_DEBUGASSERT(modem, ret != ERROR);

        modem->creg_timer_id = ret;

        ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);

        return;
#else
        /* Reset network through SIM reinitialization. Appears to work
         * better in roaming conditions. TODO: Study why, do we understand
         * function of modem in low-signal environments properly? */
#endif

      case 3: /* Registration failed? */
      case 4: /* Unknown registration error? */
      case 0: /* MT not searching for connection. */

        /* Unregister +CREG URC */

        __ubparser_unregister_response_handler(&modem->parser,
                                               urc_ATpCREG.name);
        modem->creg_urc_registered = false;

        /* Issue new task. We cannot issue new state machine work from
         * URC 'context' as other work might be active in main state
         * machine. Task will be run with main state machine in proper
         * state. */

        __ubmodem_add_task(modem, retry_network_through_sim, NULL);

        return;
    }
}

/*** Network registration ****************************************************/

static bool handle_cops_cme_error(struct ubmodem_s *modem,
                                  const struct at_resp_info_s *info)
{
  /* Ignore CME error 15 and 3, as without network COPS=0 will eventually
   * report error and also give +CREG URC messages with the correct indication
   * of the issue.
   */

  if (info->errorcode == 15 || info->errorcode == 3)
    return false;

  if (info->errorcode == 30)
    {
      /* No network service. */

      __ubmodem_level_transition_failed(modem, "NETWORK: No network service.");
      return true;
    }
  else
    {
      __ubmodem_level_transition_failed(modem, "NETWORK: Could not connect, error %d",
                                        info->errorcode);
      return true;
    }
}

static int network_register_timeout_handler(struct ubmodem_s *modem,
                                            const int timer_id,
                                            void * const arg)
{
  struct modem_sub_setup_network_s *sub = &modem->sub.setup_network;
  struct timespec curr_ts;
  const char *reason;
  int ret;

  MODEM_DEBUGASSERT(modem, modem->creg_timer_id == timer_id);

  if (timer_id != -1)
    {
      /* One-shot timer, not registered anymore. */

      modem->creg_timer_id = -1;

      ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);
    }

  (void)clock_gettime(CLOCK_MONOTONIC, &curr_ts);

  if (curr_ts.tv_sec - sub->net_reg_start_ts.tv_sec <=
      MODEM_CMD_NETWORK_REGISTRATION_TIMEOUT / 10)
    {
      /* Too soon, reregister timer. */

      ret = __ubmodem_set_timer(modem, NETWORK_REGISTRATION_CHECK_SECS * 1000,
                                &network_register_timeout_handler, sub);
      MODEM_DEBUGASSERT(modem, ret != ERROR);

      modem->creg_timer_id = ret;

      ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);

      return OK;
    }

  /* Could not register in time. Disable RF. */

  if (timer_id == -1)
    reason = "NETWORK: Denied.";
  else
    reason = "NETWORK: Timeout.";

  /* Mark for retry, 'level transition failed' can modify the actual target
   * level. */

  __ubmodem_network_cleanup(modem);
  __ubmodem_retry_current_level(modem, UBMODEM_LEVEL_SIM_ENABLED);

  /* Failed to connect network. */

  __ubmodem_level_transition_failed(modem, "%s", reason);

  return OK;
}

static void network_registration_successed(struct ubmodem_s *modem,
                                           struct modem_sub_setup_network_s *sub)
{
  /* Switch +CREG URC handler to monitor variant. */

  __ubparser_unregister_response_handler(&modem->parser, urc_ATpCREG.name);
  __ubparser_register_response_handler(&modem->parser, &urc_ATpCREG,
                                       monitor_urc_pCREG_handler, modem, true);
  DEBUGASSERT(modem->creg_urc_registered);

  /* Report successful cellular network connection. */

  sub->keep_creg_urc = true;
  __ubmodem_reached_level(modem, UBMODEM_LEVEL_NETWORK);
}

static void retry_ATpCOPS_handler(struct ubmodem_s *modem,
                                  const struct at_cmd_def_s *cmd,
                                  const struct at_resp_info_s *info,
                                  const uint8_t *resp_stream,
                                  size_t stream_len, void *priv)
{
  struct modem_sub_setup_network_s *sub = &modem->sub.setup_network;
  int status = info->status;
  int ret;

  /*
   * Response handler for retried AT+COPS=0
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCOPS);

  if (status == RESP_STATUS_CME_ERROR)
    {
      if (handle_cops_cme_error(modem, info))
        return;
    }
  else if (resp_status_is_error_or_timeout(status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "=0");
      return;
    }
  else if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  DEBUGASSERT(
      sub->network_state == NETWORK_SETUP_RETRYING_NETWORK_REGISTRATION);
  sub->network_state = NETWORK_SETUP_WAITING_NETWORK_REGISTRATION;

  /*
   * Now waiting for +CREG URC.
   *
   * Register timer for timeout.
   */

  ret = __ubmodem_set_timer(modem, NETWORK_REGISTRATION_CHECK_SECS * 1000,
                            &network_register_timeout_handler, sub);
  MODEM_DEBUGASSERT(modem, ret != ERROR);

  modem->creg_timer_id = ret;

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);
}

static int network_retry_registration_timer_handler(struct ubmodem_s *modem,
                                                    const int timer_id,
                                                    void * const arg)
{
  struct modem_sub_setup_network_s *sub = &modem->sub.setup_network;
  int err;

  MODEM_DEBUGASSERT(modem, modem->creg_timer_id == timer_id);

  if (timer_id != -1)
    {
      /* One-shot timer, not registered anymore. */

      modem->creg_timer_id = -1;

      ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);
    }

  DEBUGASSERT(
      sub->network_state == NETWORK_SETUP_RETRYING_NETWORK_REGISTRATION);

  if (sub->received_creg_while_retrying > 0)
    {
      ubdbg("Handling deferred +CREG: %d\n", sub->received_creg_while_retrying);

      network_registration_successed(modem, sub);
    }
  else
    {
      err = __ubmodem_send_cmd(modem, &cmd_ATpCOPS,
                               retry_ATpCOPS_handler, sub, "%s", "=0");
      MODEM_DEBUGASSERT(modem, err == OK);
    }

  return OK;
}

static void registration_urc_pCREG_handler(struct ubmodem_s *modem,
                                           const struct at_cmd_def_s *cmd,
                                           const struct at_resp_info_s *info,
                                           const uint8_t *resp_stream,
                                           size_t stream_len, void *priv)
{
  struct modem_sub_setup_network_s *sub = &modem->sub.setup_network;
  int8_t val;
  int ret;

  DEBUGASSERT(modem->state == MODEM_STATE_IN_SUBSTATE);

  /*
   * Response handler for +CREG URC.
   */

  MODEM_DEBUGASSERT(modem, cmd == &urc_ATpCREG);

  if (info->status != RESP_STATUS_URC)
    {
      /* Should not happen. */

      MODEM_DEBUGASSERT(modem, false);
      return;
    }

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &val))
    {
      /* Should not happen. */

      MODEM_DEBUGASSERT(modem, false);
      return;
    }

  dbg("Network registration URC, +CREG=%d.\n", val);

  if (val >= 0 && val < ARRAY_SIZE(creg_strings))
    {
      dbg("%s\n", creg_strings[val]);
    }
  else
    {
      dbg("Unknown +CREG status: %d\n", val);
    }

  if (sub->network_state < NETWORK_SETUP_WAITING_NETWORK_REGISTRATION)
    {
      /* Not ready yet, spurious +CREG URC? */

      ubdbg("Ignored spurious +CREG URC.\n");

      return;
    }

  switch (val)
  {
    case 1:
    case 5:
      /*
       * If 'val' is 1, registered: home network.
       * If 'val' is 5, registered: roamed.
       */

      if (sub->network_state == NETWORK_SETUP_RETRYING_NETWORK_REGISTRATION)
        {
          /* Cannot do state transition to UBMODEM_LEVEL_NETWORK right now
           * as still retrying AT+COPS=0. Defer handling to
           * retry_ATpCOPS_handler. */

          sub->received_creg_while_retrying = val;
        }
      else
        {
          network_registration_successed(modem, sub);
        }

      return;

    case 0:
      /*
       * MT not searching for connection.
       */

      if (sub->network_state == NETWORK_SETUP_WAITING_NETWORK_REGISTRATION)
        {
          /* Stop timer before issuing command. */

          if (modem->creg_timer_id >= 0)
            {
              __ubmodem_remove_timer(modem, modem->creg_timer_id);

              modem->creg_timer_id = -1;

              ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);
            }

          /* Keep retrying automatic network registration until timeout. */

          sub->network_state = NETWORK_SETUP_RETRYING_NETWORK_REGISTRATION;
          sub->received_creg_while_retrying = -1;

          ret = __ubmodem_set_timer(modem, 1000,
                                    &network_retry_registration_timer_handler,
                                    sub);
          MODEM_DEBUGASSERT(modem, ret != ERROR);

          modem->creg_timer_id = ret;

          ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);
        }

      return;

    case 4:
      /*
       * If 'val' is 4, unknown registration error.
       *
       * Network registration flow-chart in "AT Commands Examples"
       * (UBX-13001820, R09, p.24) tells that proper action for this
       * event is to 'Wait'.
       */

      return;

    case 3:
      /*
       * If 'val' is 3, registration was denied.
       *
       * Network registration flow-chart in "AT Commands Examples"
       * (UBX-13001820, R09, p.24) tells that proper action for this
       * event is 'DTE intervention may be required'. Further information
       * on event type '3':
       *
       * "3: the registration fails after a Location Update Reject;
       *     possible causes are:
       *     - Illegal MS
       *     - Illegal ME
       *     - IMSI unknown at HLR
       *     - PLMN not allowed
       *     - Location area not allowed
       *     - Roaming not allowed in this location area
       *     - Network failure
       *     - Network congestion
       *
       *   If the registration type is manual, then no further attempt is
       *   made to search for a new PLMN or register with it. If the
       *   registration type is automatic, the MS may look for an allowed
       *   PLMN if the rejection cause was roaming restriction. In case of
       *   illegal MS / ME, there could be possible problems with either
       *   the SIM card or with the MT's identity (IMEI): user intervention
       *   may be required."
       *
       * Modem can still be continuing connection effort after this event
       * especially in roaming conditions. Thus, we do not react on this
       * message but keep waiting for timeout or other indication of
       * network error.
       */

      return;

    case 2:
      /*
       * If 'val' is 2, modem is trying to register.
       */

      return;

    default:
      /*
       * Unknown.
       */

      return;
  }
}

static int network_start_register_handler(struct ubmodem_s *modem,
                                          const int timer_id,
                                          void * const arg)
{
  struct modem_sub_setup_network_s *sub = &modem->sub.setup_network;
  int err;

  MODEM_DEBUGASSERT(modem, modem->creg_timer_id == timer_id);

  if (timer_id != -1)
    {
      /* One-shot timer, not registered anymore. */

      modem->creg_timer_id = -1;

      ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);
    }

  /* Keep retrying automatic network registration until timeout. */

  sub->network_state = NETWORK_SETUP_RETRYING_NETWORK_REGISTRATION;
  sub->received_creg_while_retrying = -1;

  err = __ubmodem_send_cmd(modem, &cmd_ATpCOPS,
                           retry_ATpCOPS_handler, sub, "%s", "=0");
  MODEM_DEBUGASSERT(modem, err == OK);
  return OK;
}

static void disable_ATpCOPS_handler(struct ubmodem_s *modem,
                                    const struct at_cmd_def_s *cmd,
                                    const struct at_resp_info_s *info,
                                    const uint8_t *resp_stream,
                                    size_t stream_len, void *priv)
{
  struct modem_sub_setup_network_s *sub = &modem->sub.setup_network;
  int ret;
  int status = info->status;

  /*
   * Response handler for AT+COPS=0
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCOPS);

  if (status == RESP_STATUS_CME_ERROR)
    {
      if (handle_cops_cme_error(modem, info))
        return;
    }
  else if (resp_status_is_error_or_timeout(status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "=2");
      return;
    }
  else if (status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /*
   * Now waiting for +CREG URC.
   *
   * Register timer for +COPS=0.
   */

  sub->network_state = NETWORK_SETUP_WAITING_NETWORK_REGISTRATION;

  ret = __ubmodem_set_timer(modem, 1000, &network_start_register_handler, sub);
  MODEM_DEBUGASSERT(modem, ret != ERROR);

  modem->creg_timer_id = ret;

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);
}

static void ATpCFUN_handler(struct ubmodem_s *modem,
                            const struct at_cmd_def_s *cmd,
                            const struct at_resp_info_s *info,
                            const uint8_t *resp_stream,
                            size_t stream_len, void *priv)
{
  struct modem_sub_setup_network_s *sub = &modem->sub.setup_network;
  int err;

  /*
   * Response handler for AT+CFUN=1
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCFUN);

  if (resp_status_is_error_or_timeout(info->status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "=1");
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /*
   * Modem RF enabled, now disable previous automatic network registration.
   * +CREG:0 will be received and +COPS=0 will be issued to start new automatic
   * network registration.
   */

  sub->network_state = NETWORK_SETUP_STARTING_NETWORK_REGISTRATION;
  (void)clock_gettime(CLOCK_MONOTONIC, &sub->net_reg_start_ts);

  err = __ubmodem_send_cmd(modem, &cmd_ATpCOPS, disable_ATpCOPS_handler, sub,
                           "%s", "=2");
  MODEM_DEBUGASSERT(modem, err == OK);
}

static void ATpCREG_handler(struct ubmodem_s *modem,
                            const struct at_cmd_def_s *cmd,
                            const struct at_resp_info_s *info,
                            const uint8_t *resp_stream,
                            size_t stream_len, void *priv)
{
  struct modem_sub_setup_network_s *sub = &modem->sub.setup_network;
  int err;

  /*
   * Response handler for AT+CREG=1
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCREG);

  if (resp_status_is_error_or_timeout(info->status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "=1");
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Register "+CREG" URC handler. */

  __ubparser_register_response_handler(&modem->parser, &urc_ATpCREG,
                                       registration_urc_pCREG_handler,
                                       sub, true);
  modem->creg_urc_registered = true;
  sub->network_state = NETWORK_SETUP_ENABLING_RF;

  /* Enable modem RF. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpCFUN, ATpCFUN_handler, sub, "%s", "=1");
  MODEM_DEBUGASSERT(modem, err == OK);
}

static void setup_network_cleanup(struct ubmodem_s *modem)
{
  struct modem_sub_setup_network_s *sub = &modem->sub.setup_network;

  if (modem->creg_timer_id != -1)
    {
      /* Unregister timeout. */

      __ubmodem_remove_timer(modem, modem->creg_timer_id);

      modem->creg_timer_id = -1;

      ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);
    }

  if (modem->creg_urc_registered && !sub->keep_creg_urc)
    {
      /* Unregister +CREG URC */

      __ubparser_unregister_response_handler(&modem->parser,
                                             urc_ATpCREG.name);
      modem->creg_urc_registered = false;
    }
}

/*** Network disconnection ***************************************************/

static void set_ATpCFUN_off_handler(struct ubmodem_s *modem,
                                    const struct at_cmd_def_s *cmd,
                                    const struct at_resp_info_s *info,
                                    const uint8_t *resp_stream,
                                    size_t stream_len, void *priv)
{
  /*
   * Response handler for AT+CFUN=0
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCFUN);

  if (resp_status_is_error_or_timeout(info->status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "=0");
      return;
    }
  else if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Done unregistering from network. */

  __ubmodem_reached_level(modem, UBMODEM_LEVEL_SIM_ENABLED);
}

static int network_unregister_timer_handler(struct ubmodem_s *modem,
                                            const int timer_id,
                                            void * const arg)
{
  struct modem_sub_setup_network_s *sub = &modem->sub.setup_network;
  int err;

  /* Send +CFUN=0 */

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);
  err = __ubmodem_send_cmd(modem, &cmd_ATpCFUN, set_ATpCFUN_off_handler, sub,
                           "%s", "=0");
  MODEM_DEBUGASSERT(modem, err == OK);

  return OK;
}

static void set_ATpCOPS_off_handler(struct ubmodem_s *modem,
                                    const struct at_cmd_def_s *cmd,
                                    const struct at_resp_info_s *info,
                                    const uint8_t *resp_stream,
                                    size_t stream_len, void *priv)
{
  struct modem_sub_setup_network_s *sub = &modem->sub.setup_network;
  int ret;

  /*
   * Response handler for AT+COPS=2
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCOPS);

  if (resp_status_is_error_or_timeout(info->status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "=2");
      return;
    }
  else if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Wait for sometime so modem can complete unregistering from network. */

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);
  ret = __ubmodem_set_timer(modem, 1500, &network_unregister_timer_handler,
                            sub);
  if (ret == ERROR)
    {
      /* Error here? Add assert? Or just try bailout? */

      MODEM_DEBUGASSERT(modem, false);

      (void)network_unregister_timer_handler(modem, -1, sub);
      return;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_substate_start_setup_network
 *
 * Description:
 *   Start modem network registration sequence.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_setup_network(struct ubmodem_s *modem)
{
  struct modem_sub_setup_network_s *sub = &modem->sub.setup_network;
  int err;

  MODEM_DEBUGASSERT(modem, modem->level == UBMODEM_LEVEL_SIM_ENABLED);

  /*
   * Setup network sequence is:
   *
   * 1. Enable "+CREG" URC.
   * 2. Send "AT+CFUN=1".
   * 3. Send "AT+COPS=0".
   * 4. Wait until connected (Wait information from +CREG)
   *
   */

  MODEM_DEBUGASSERT(modem, modem->creg_urc_registered == false);
  MODEM_DEBUGASSERT(modem, modem->creg_timer_id == -1);

  /* Reset sub-state data and initiate sub-state machine work. */

  memset(sub, 0, sizeof(*sub));
  sub->network_state = NETWORK_SETUP_ENABLING_CREG;

  /* Setup sub-state clean-up function. */

  modem->substate_cleanup_fn = setup_network_cleanup;

  /* Enable +CREG. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpCREG, ATpCREG_handler, sub,
                           "%s", "=1");
  MODEM_DEBUGASSERT(modem, err == OK);
}

/****************************************************************************
 * Name: __ubmodem_network_cleanup
 *
 * Description:
 *   Clean-up network connection state.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_network_cleanup(struct ubmodem_s *modem)
{
  if (modem->level == UBMODEM_LEVEL_GPRS)
    {
      /* Uninitialize GRPS */

      __ubmodem_gprs_cleanup(modem);
    }

  if (modem->level >= UBMODEM_LEVEL_NETWORK)
    {
      if (modem->creg_timer_id != -1)
        {
          /* Unregister timeout. */

          __ubmodem_remove_timer(modem, modem->creg_timer_id);

          modem->creg_timer_id = -1;

          ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);
        }

      if (modem->creg_urc_registered)
        {
          /* Unregister +CREG URC */

          __ubparser_unregister_response_handler(&modem->parser,
                                                 urc_ATpCREG.name);
          modem->creg_urc_registered = false;
        }
    }
}

/****************************************************************************
 * Name: __ubmodem_substate_start_disconnect_network
 *
 * Description:
 *   Start modem network unregistration sequence.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_disconnect_network(struct ubmodem_s *modem)
{
  struct modem_sub_setup_network_s *sub = &modem->sub.setup_network;
  int err;

  MODEM_DEBUGASSERT(modem, modem->level == UBMODEM_LEVEL_NETWORK ||
                           modem->level == UBMODEM_LEVEL_GPRS);

  /*
   * Disconnect network sequence is:
   *
   * 1. Send "AT+COPS=2".
   * 2. Send "AT+CFUN=0".
   *
   */

  __ubmodem_network_cleanup(modem);

  /* Reset sub-state data and initiate sub-state machine work. */

  memset(sub, 0, sizeof(*sub));
  sub->network_state = NETWORK_SETUP_DISCONNECTING;

  /* Disable network. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpCOPS, set_ATpCOPS_off_handler, sub,
                           "%s", "=2");
  MODEM_DEBUGASSERT(modem, err == OK);
}
