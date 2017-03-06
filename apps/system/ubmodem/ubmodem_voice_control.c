/****************************************************************************
 * apps/system/ubmodem/ubmodem_voice_control.c
 *
 *   Copyright (C) 2015-2017 Haltian Ltd. All rights reserved.
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
#include <errno.h>

#include <apps/system/ubmodem.h>

#ifdef CONFIG_NETUTILS_DNSCLIENT
#include <apps/netutils/dnsclient.h>
#endif

#include "ubmodem_internal.h"
#include "ubmodem_voice.h"
#include "ubmodem_hw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define VOICE_MODEM_PROBE_SECS             7
#define VOICE_MODEM_PROBE_FAILS            3

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

enum modem_ucallstat_e
{
  MODEM_UCALLSTAT_ACTIVE = 0,
  MODEM_UCALLSTAT_HOLD,
  MODEM_UCALLSTAT_DIALING,
  MODEM_UCALLSTAT_ALERTING,
  MODEM_UCALLSTAT_RINGING,
  MODEM_UCALLSTAT_WAITING,
  MODEM_UCALLSTAT_DISCONNECTED,
  MODEM_UCALLSTAT_CONNECTED,
};

enum modem_cli_validity_e
{
  MODEM_CLI_VALID = 0,
  MODEM_CLI_WITHHELD,
  MODEM_CLI_NOT_AVAILABLE,
};

/****************************************************************************
 * Private Function Prototypes.
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at_cmd_def_s urc_ATpCRING =
{
  .name         = "+CRING",
  .resp_format  =
    (const uint8_t[]){
      RESP_FMT_STRING_TO_EOL /* RING type*/
    },
  .resp_num     = 1,
};

static const struct at_cmd_def_s urc_ATpUCALLSTAT =
{
  .name         = "+UCALLSTAT",
  .resp_format  =
    (const uint8_t[]){
      RESP_FMT_INT32, /* call_id */
      RESP_FMT_INT8,  /* stat */
    },
  .resp_num     = 2,
};

static const struct at_cmd_def_s urc_ATpCLIP =
{
  .name         = "+CLIP",
  .resp_format  =
    (const uint8_t[]){
      RESP_FMT_QUOTED_STRING, /* number */
      RESP_FMT_INT16,         /* type */
      RESP_FMT_QUOTED_STRING, /* sub-address */
      RESP_FMT_INT16,         /* sa_type */
      RESP_FMT_QUOTED_STRING, /* phone-book name for number */
      RESP_FMT_INT8,          /* CLI validity */
    },
  .resp_num     = 6,
};

static const struct at_cmd_def_s cmd_ATA =
{
  .name         = "A",
  .timeout_dsec = 20 * 10,
};

static const struct at_cmd_def_s cmd_ATpCHUP =
{
  .name         = "+CHUP",
  .timeout_dsec = 20 * 10,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void voice_probe_check_cmee_result(struct ubmodem_s *modem,
                                          bool cmee_ok, int cmee_setting,
                                          void *priv)
{
  int err;

  /* Release main state machine for next task. */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);

  if (!cmee_ok && modem->voice.probe_fails++ >= VOICE_MODEM_PROBE_FAILS)
    {
      /* Modem HW is in unknown state, attempt to recover. */

      err = __ubmodem_recover_stuck_hardware(modem);
      MODEM_DEBUGASSERT(modem, err != ERROR);
    }
}

static int voice_probe_check_cmee(struct ubmodem_s *modem, void *priv)
{
  int err;

  modem->voice.probe_task_queued = false;

  if (!modem->voice.pm_activity_enabled)
    {
      return ERROR;
    }

  if (modem->level < UBMODEM_LEVEL_CMD_PROMPT)
    {
      return ERROR;
    }

  /* Check CMEE setting to see if modem has reseted or not. */

  err = __ubmodem_check_cmee_status(modem, voice_probe_check_cmee_result, modem);
  MODEM_DEBUGASSERT(modem, err == OK);

  return OK;
}

static int voice_probe_fn(struct ubmodem_s *modem, const int timer_id,
                          void * const arg)
{
  int ret;

  modem->voice.probe_timerid = -1;

  if (!modem->voice.pm_activity_enabled)
    {
      return OK;
    }

  /* Start CMEE probe task. */

  if (!modem->voice.probe_task_queued)
    {
      ret = __ubmodem_add_task(modem, voice_probe_check_cmee, modem);
      MODEM_DEBUGASSERT(modem, ret != ERROR);
      modem->voice.probe_task_queued = true;
    }

  /* Reregister timer. */

  modem->voice.probe_timerid = __ubmodem_set_timer(modem,
                                 VOICE_MODEM_PROBE_SECS * 1000,
                                 voice_probe_fn, modem);
  MODEM_DEBUGASSERT(modem, modem->voice.probe_timerid >= 0);

  return OK;
}

static void modem_voice_pm_activity(struct ubmodem_s *modem,
                                    bool ringing_or_active)
{
  if (ringing_or_active == modem->voice.pm_activity_enabled)
    {
      /* Already in this state. */

      return;
    }

  modem->voice.pm_activity_enabled = ringing_or_active;

  modem->voice.probe_fails = 0;
  if (modem->voice.probe_timerid >= 0)
    {
      __ubmodem_remove_timer(modem, modem->voice.probe_timerid);
      modem->voice.probe_timerid = -1;
    }

  if (ringing_or_active)
    {
      /* Voice call incoming, ringing. After ringing call is eventually
       * disconnected. Report low activity so that we get disconnect URC from
       * modem after hangup (no ring-indication anymore to wake-up MCU). */

      ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_LOW, true);

      /* Start probing to detect modem hang-ups (low voltage situation etc). */

      modem->voice.probe_timerid = __ubmodem_set_timer(modem,
                                     VOICE_MODEM_PROBE_SECS * 1000,
                                     voice_probe_fn, modem);
      MODEM_DEBUGASSERT(modem, modem->voice.probe_timerid >= 0);
    }
  else
    {
      /* Voice call disconnected, report change of activity to
       * power-management. */

      ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_LOW, false);
    }
}

static void modem_voice_report_ringing(struct ubmodem_s *modem)
{
  if (modem->voice.ring_reported)
    {
      /* Already reported. */

      return;
    }

  if (!modem->voice.got_clip)
    {
      /* No CLIP status yet. */

      return;
    }

  if (!modem->voice.ringing)
    {
      /* Not ringing. */

      return;
    }

  /* Report new ringing event. */

  __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_CALL_RINGING,
                          &modem->voice.clip, sizeof(modem->voice.clip));

  modem->voice.ring_reported = true;
}

static void modem_voice_report_disconnected(struct ubmodem_s *modem)
{
  if (modem->voice.disconnect_reported)
    {
      /* Already reported. */

      return;
    }

  /* Report disconnect event. */

  __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_CALL_DISCONNECTED, NULL, 0);

  modem->voice.disconnect_reported = true;
}

static void modem_voice_new_clip(struct ubmodem_s *modem, bool valid,
                                 const char *number, uint16_t number_type,
                                 const char *subaddr, uint16_t sa_type)
{
  /* Store CLIP. */

  if (valid)
    {
      snprintf(modem->voice.clip.number,
               sizeof(modem->voice.clip.number),
               "%s", number);
      snprintf(modem->voice.clip.subaddress,
               sizeof(modem->voice.clip.subaddress),
               "%s", subaddr);
      modem->voice.clip.number_type = number_type;
      modem->voice.clip.subaddress_type = sa_type;
    }
  else
    {
      memset(&modem->voice.clip, 0, sizeof(modem->voice.clip));
    }

  modem->voice.got_clip = true;

  /* Report state, generate event if needed. */

  modem_voice_report_ringing(modem);
}

static void modem_voice_clear_clip(struct ubmodem_s *modem)
{
  modem->voice.got_clip = false;
  memset(&modem->voice.clip, 0, sizeof(modem->voice.clip));
}

static void modem_voice_active(struct ubmodem_s *modem)
{
  modem->voice.active = true;
  modem->voice.ring_reported = false;
  modem->voice.disconnect_reported = false;

  modem_voice_pm_activity(modem, true);

  /* Report that voice-call is active. */

  __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_CALL_ACTIVE, NULL, 0);

#ifdef CONFIG_NETUTILS_DNSCLIENT
  /* Voice-call can block GRPS communication and cause DNS queries
   * to fail. Clear accumulated fail count in this case. */

  dns_clear_lookup_failed_count();

  /* TODO: We should instead control socket availability, and prevent opening
   *       new sockets when voice.ringing/active. There is however one catch:
   *       on 3G modem, voice-call and GRPS can transmit simultaneously. */
#endif
}

static void modem_voice_ringing(struct ubmodem_s *modem)
{
  modem->voice.active = false;
  modem->voice.ringing = true;
  modem->voice.disconnect_reported = false;

  modem_voice_pm_activity(modem, true);

  /* Report state, generate event if needed. */

  modem_voice_report_ringing(modem);

#ifdef CONFIG_NETUTILS_DNSCLIENT
  /* Voice-call can block GRPS communication and cause DNS queries
   * to fail. Clear accumulated fail count in this case. */

  dns_clear_lookup_failed_count();

  /* TODO: We should instead control socket availability, and prevent opening
   *       new sockets when voice.ringing/active. There is however one catch:
   *       on 3G modem, voice-call and GRPS can transmit simultaneously. */
#endif
}

static void modem_voice_disconnected(struct ubmodem_s *modem, bool ctrl_audio)
{
  modem->voice.ring_reported = false;
  modem->voice.active = false;
  modem->voice.ringing = false;
  modem_voice_clear_clip(modem);

  if (ctrl_audio)
    {
      /* No need to keep audio on if not ringing or active. */

      ubmodem_audio_setup(modem, false, false);
    }

  /* Report state, generate event if needed. */

  modem_voice_report_disconnected(modem);

  modem_voice_pm_activity(modem, false);
}

static void modem_voice_update_ucallstat(struct ubmodem_s *modem,
                                         enum modem_ucallstat_e stat)
{
  switch (stat)
    {
      case MODEM_UCALLSTAT_RINGING:
        /* Incoming, ringing. */

        modem_voice_ringing(modem);
        break;

      case MODEM_UCALLSTAT_ACTIVE:
        /* Voice-call active. */

        modem_voice_active(modem);
        break;

      case MODEM_UCALLSTAT_CONNECTED:
        /* Ignored. */

        break;

      case MODEM_UCALLSTAT_DISCONNECTED:
      case MODEM_UCALLSTAT_HOLD:
      case MODEM_UCALLSTAT_DIALING:
      case MODEM_UCALLSTAT_ALERTING:
      case MODEM_UCALLSTAT_WAITING:
      default:
        /* Currently, handle all other as disconnected state. */

        modem_voice_disconnected(modem, true);
        break;
    }
}

/****************************************************************************
 * Name: urc_voice_cring_handler
 *
 * Desciption:
 *   Handler for +CRING URC
 ****************************************************************************/

static void urc_voice_cring_handler(struct ubmodem_s *modem,
                                    const struct at_cmd_def_s *cmd,
                                    const struct at_resp_info_s *info,
                                    const uint8_t *resp_stream,
                                    size_t stream_len, void *priv)
{
  const char *type;
  uint16_t typelen;

  /*
   * URC handler for 'data available for sockets on modem'
   */

  MODEM_DEBUGASSERT(modem, cmd == &urc_ATpCRING);
  MODEM_DEBUGASSERT(modem, info->status == RESP_STATUS_URC);

  /* Get type. */

  if (!__ubmodem_stream_get_string(&resp_stream, &stream_len, &type, &typelen))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Just report for now. */

  dbg("+CRING: type='%s'\n", type);
}

/****************************************************************************
 * Name: urc_voice_ucallstat_handler
 *
 * Desciption:
 *   Handler for +UCALLSTAT URC
 ****************************************************************************/

static void urc_voice_ucallstat_handler(struct ubmodem_s *modem,
                                        const struct at_cmd_def_s *cmd,
                                        const struct at_resp_info_s *info,
                                        const uint8_t *resp_stream,
                                        size_t stream_len, void *priv)
{
  int8_t stat;
  int32_t call_id;
  const char *stat_str;

  /*
   * URC handler for 'data available for sockets on modem'
   */

  MODEM_DEBUGASSERT(modem, cmd == &urc_ATpUCALLSTAT);
  MODEM_DEBUGASSERT(modem, info->status == RESP_STATUS_URC);

  /* Get call_id. */

  if (!__ubmodem_stream_get_int32(&resp_stream, &stream_len, &call_id))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Get status. */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &stat))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Report. */

  switch (stat)
    {
      case MODEM_UCALLSTAT_ACTIVE:
        stat_str = "active";
        break;
      case MODEM_UCALLSTAT_HOLD:
        stat_str = "hold";
        break;
      case MODEM_UCALLSTAT_DIALING:
        stat_str = "dialing (MO)";
        break;
      case MODEM_UCALLSTAT_ALERTING:
        stat_str = "alerting (MO)";
        break;
      case MODEM_UCALLSTAT_RINGING:
        stat_str = "ringing (MT)";
        break;
      case MODEM_UCALLSTAT_WAITING:
        stat_str = "waiting (MT)";
        break;
      case MODEM_UCALLSTAT_DISCONNECTED:
        stat_str = "disconnected";
        break;
      case MODEM_UCALLSTAT_CONNECTED:
        stat_str = "connected";
        break;
      default:
        stat_str = NULL;
        break;
    }

  dbg("+UCALLSTAT: call_id=%d, status: %s (%d).\n", call_id, stat_str, stat);

  /* Update CLIP state. */

  if (stat_str == NULL)
    stat = MODEM_UCALLSTAT_DISCONNECTED;

  modem_voice_update_ucallstat(modem, stat);
}

/****************************************************************************
 * Name: urc_voice_clip_handler
 *
 * Desciption:
 *   Handler for +CLIP URC
 ****************************************************************************/

static void urc_voice_clip_handler(struct ubmodem_s *modem,
                                   const struct at_cmd_def_s *cmd,
                                   const struct at_resp_info_s *info,
                                   const uint8_t *resp_stream,
                                   size_t stream_len, void *priv)
{
  const char *number;
  uint16_t number_len;
  int16_t number_type;
  const char *subaddr;
  uint16_t sa_len;
  int16_t sa_type;
  const char *alpha;
  uint16_t alpha_len;
  int8_t cli_valid;
  const char *cli_valid_str;

  /*
   * URC handler for 'data available for sockets on modem'
   */

  MODEM_DEBUGASSERT(modem, cmd == &urc_ATpCLIP);
  MODEM_DEBUGASSERT(modem, info->status == RESP_STATUS_URC);

  /* Get number. */

  if (!__ubmodem_stream_get_string(&resp_stream, &stream_len, &number,
                                   &number_len))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Get number type. */

  if (!__ubmodem_stream_get_int16(&resp_stream, &stream_len, &number_type))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Get sub-address. */

  if (!__ubmodem_stream_get_string(&resp_stream, &stream_len, &subaddr,
                                   &sa_len))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Get sub-address type. */

  if (!__ubmodem_stream_get_int16(&resp_stream, &stream_len, &sa_type))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Get alpha. */

  if (!__ubmodem_stream_get_string(&resp_stream, &stream_len, &alpha,
                                   &alpha_len))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Get CLI validity. */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &cli_valid))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Report. */

  switch (cli_valid)
    {
      case MODEM_CLI_VALID:
        cli_valid_str = "valid";
        break;
      case MODEM_CLI_WITHHELD:
        cli_valid_str = "withheld";
        break;
      case MODEM_CLI_NOT_AVAILABLE:
        cli_valid_str = "not available";
        break;
      default:
        cli_valid_str = NULL;
        break;
    }

  dbg("+CLIP: CLI %s: number='%s', type=%d, sa='%s', satype=%d, alpha='%s'.\n",
      cli_valid_str, number, number_type, subaddr, sa_type, alpha);

  /* Update CLIP state. */

  if (cli_valid_str == NULL)
    cli_valid = MODEM_CLI_NOT_AVAILABLE;

  modem_voice_new_clip(modem, cli_valid == MODEM_CLI_VALID, number, number_type,
                       subaddr, sa_type);
}

static void modem_audio_check_cmee(struct ubmodem_s *modem, bool cmee_ok,
                                    int cmee_setting, void *priv)
{
  if (!cmee_ok)
    {
      int err;

      /* Modem hardware stuck or reseted, launch task to restore function. */

      err = __ubmodem_recover_stuck_hardware(modem);
      MODEM_DEBUGASSERT(modem, err != ERROR);
    }

  /* Reactions to call state changes are based on +UCALLSTAT. Just complete
   * task work. */

  /* Update main state machine. */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);
}

static void ATA_handler(struct ubmodem_s *modem, const struct at_cmd_def_s *cmd,
                        const struct at_resp_info_s *info,
                        const uint8_t *resp_stream, size_t stream_len,
                        void *priv)
{
  uintptr_t mutepriv = (uintptr_t)priv;
  bool mute_mic = mutepriv >> 1;
  bool mute_speaker = mutepriv & 1;

  if (resp_status_is_error_or_timeout(info->status))
    {
      int err;

      /* In case of error, check CMEE setting to detect stuck hardware. */

      err =  __ubmodem_check_cmee_status(modem, modem_audio_check_cmee, modem);
      MODEM_DEBUGASSERT(modem, err == OK);
      return;
    }
  else if (info->status == RESP_STATUS_OK)
    {
      /* Configure audio. */

      ubmodem_audio_setup(modem, !mute_speaker, !mute_mic);
    }

  /* Reactions to call state changes are based on +UCALLSTAT. Just complete
   * task work. */

  /* Update main state machine. */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);
}

static int start_task_voice_answer(struct ubmodem_s *modem, void *priv)
{
  int err;

  if (modem->level < UBMODEM_LEVEL_NETWORK)
    {
      return ERROR;
    }

  /* Send answer call command 'ATA' */

  err = __ubmodem_send_cmd(modem, &cmd_ATA, ATA_handler, priv, "");
  MODEM_DEBUGASSERT(modem, err == OK);

  return OK;
}

static void ATpCHUP_handler(struct ubmodem_s *modem,
                            const struct at_cmd_def_s *cmd,
                            const struct at_resp_info_s *info,
                            const uint8_t *resp_stream, size_t stream_len,
                            void *priv)
{
  if (resp_status_is_error_or_timeout(info->status))
    {
      int err;

      /* In case of error, check CMEE setting to detect stuck hardware. */

      err =  __ubmodem_check_cmee_status(modem, modem_audio_check_cmee, modem);
      MODEM_DEBUGASSERT(modem, err == OK);
      return;
    }

  /* Reactions to call state changes are based on +UCALLSTAT. Just complete
   * task work. */

  /* Update main state machine. */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);
}

static int start_task_voice_hangup(struct ubmodem_s *modem, void *priv)
{
  int err;

  if (modem->level < UBMODEM_LEVEL_NETWORK)
    {
      return ERROR;
    }

  /* Send hangup call command 'AT+CHUP' */

  err = __ubmodem_send_cmd(modem, &cmd_ATpCHUP, ATpCHUP_handler, modem, "");
  MODEM_DEBUGASSERT(modem, err == OK);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubmodem_voice_answer
 *
 * Description:
 *   Answer pending voice-call
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void ubmodem_voice_answer(struct ubmodem_s *modem, bool mute_mic,
                          bool mute_speaker)
{
  uintptr_t mutepriv;

  DEBUGASSERT(modem);

  if (modem->level < UBMODEM_LEVEL_NETWORK)
    {
      return;
    }

  mutepriv = (mute_mic << 1) | mute_speaker;

  (void)__ubmodem_add_task(modem, start_task_voice_answer, (void *)mutepriv);
}

/****************************************************************************
 * Name: ubmodem_voice_hangup
 *
 * Description:
 *   Hang-up pending or active voice-call
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void ubmodem_voice_hangup(struct ubmodem_s *modem)
{
  DEBUGASSERT(modem);

  if (modem->level < UBMODEM_LEVEL_NETWORK)
    {
      return;
    }

  (void)__ubmodem_add_task(modem, start_task_voice_hangup, NULL);
}

/****************************************************************************
 * Name: __ubmodem_voice_control_setup
 *
 * Description:
 *   Setup voice-call control
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_voice_control_setup(struct ubmodem_s *modem)
{
  if (modem->voice_urcs_registered)
    return;

  memset(&modem->voice, 0, sizeof(modem->voice));

  modem->voice.probe_timerid = -1;

  __ubparser_register_response_handler(&modem->parser, &urc_ATpCRING,
                                       urc_voice_cring_handler,
                                       modem, true);
  __ubparser_register_response_handler(&modem->parser, &urc_ATpUCALLSTAT,
                                       urc_voice_ucallstat_handler,
                                       modem, true);
  __ubparser_register_response_handler(&modem->parser, &urc_ATpCLIP,
                                       urc_voice_clip_handler,
                                       modem, true);
  modem->voice_urcs_registered = true;
}

/****************************************************************************
 * Name: __ubmodem_voice_control_cleanup
 *
 * Description:
 *   Cleanup and disable voice-call control
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_voice_control_cleanup(struct ubmodem_s *modem)
{
  if (!modem->voice_urcs_registered)
    return;

  modem_voice_disconnected(modem, false);
  __ubmodem_audio_cleanup(modem);

  __ubparser_unregister_response_handler(&modem->parser, urc_ATpCRING.name);
  __ubparser_unregister_response_handler(&modem->parser, urc_ATpUCALLSTAT.name);
  __ubparser_unregister_response_handler(&modem->parser, urc_ATpCLIP.name);
  modem->voice_urcs_registered = false;

  memset(&modem->voice, 0, sizeof(modem->voice));
}
