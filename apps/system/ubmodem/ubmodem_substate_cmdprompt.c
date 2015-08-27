/****************************************************************************
 * apps/system/ubmodem/ubmodem_substate_cmdprompt.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_UBMODEM_POWERSAVE_GUARD_PERIOD
#  define CONFIG_UBMODEM_POWERSAVE_GUARD_PERIOD 0
#endif

#if CONFIG_UBMODEM_POWERSAVE_GUARD_PERIOD > 0
#  if CONFIG_UBMODEM_POWERSAVE_GUARD_PERIOD < 40 || \
      CONFIG_UBMODEM_POWERSAVE_GUARD_PERIOD > 65000
#    error "CONFIG_UBMODEM_POWERSAVE_GUARD_PERIOD not in valid range"
#  endif
#endif

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct modem_read_cb_s
{
  struct ubmodem_s *modem;
  void (*string_fn)(const char *modem_read_buf,
                    size_t modem_read_len,
                    void *modem_read_arg);
  void *arg;
};

/****************************************************************************
 * Private Function Prototypes.
 ****************************************************************************/

#ifdef CONFIG_UBMODEM_POWERSAVE
static const ubmodem_send_config_func_t *
get_powersave_config_cmds(struct ubmodem_s *modem, size_t *ncmds);
#endif

static int delay_after_initial_handler(struct ubmodem_s *modem,
                                       const int timer_id, void * const arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at_cmd_def_s cmd_initial =
{
  .name         = "",
  .resp_format  = NULL,
  .resp_num     = 0,
  .timeout_dsec = 5, /* 500 msec */
};

static const struct at_cmd_def_s cmd_ATpCMEE =
{
  .name         = "+CMEE",
  .resp_format  = (const uint8_t[]){ RESP_FMT_INT8 },
  .resp_num     = 1,
};

static const struct at_cmd_def_s cmd_ATpINVALIDCOMMANDTEST =
{
  .name         = "+INVALIDCOMMANDTEST",
  .resp_format  = NULL,
  .resp_num     = 0,
};

static const struct at_cmd_def_s cmd_ATpCFUN =
{
  .name         = "+CFUN",
  .resp_format  = (const uint8_t[]){ RESP_FMT_INT8, RESP_FMT_INT8 },
  .resp_num     = 2,
  .timeout_dsec = MODEM_CMD_NETWORK_TIMEOUT,
};

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

#ifdef CONFIG_UBMODEM_POWERSAVE
static const struct at_cmd_def_s cmd_ATpUPSV =
{
  .name         = "+UPSV",
  .resp_format  = NULL,
  .resp_num     = 0,
};
#endif

static const ubmodem_configs_func_t ubmodem_config_groups[] =
{
#ifdef CONFIG_UBMODEM_POWERSAVE
  get_powersave_config_cmds,
#endif
#ifdef CONFIG_UBMODEM_VOICE
  __ubmodem_voice_get_config_commands,
  __ubmodem_voice_get_audiopath_config_commands,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_UBMODEM_POWERSAVE

static void config_enable_powersave(struct ubmodem_s *modem)
{
  const int guard = CONFIG_UBMODEM_POWERSAVE_GUARD_PERIOD;
  int err;

  err = __ubmodem_send_cmd(modem, &cmd_ATpUPSV,
                           __ubmodem_cmdprompt_generic_config_handler,
                           (void *)&cmd_ATpUPSV,
                           !guard ? "=1" : "=1,%d", guard);
  MODEM_DEBUGASSERT(modem, err == OK);
}

static const ubmodem_send_config_func_t *
get_powersave_config_cmds(struct ubmodem_s *modem, size_t *ncmds)
{
  static const ubmodem_send_config_func_t commands[] =
  {
    config_enable_powersave,
  };

  *ncmds = ARRAY_SIZE(commands);

  return commands;
}

#endif /* CONFIG_UBMODEM_POWERSAVE */

static void ATpCOPS_handler(struct ubmodem_s *modem,
                            const struct at_cmd_def_s *cmd,
                            const struct at_resp_info_s *info,
                            const uint8_t *resp_stream,
                            size_t stream_len, void *priv)
{
  struct modem_sub_setup_cmd_prompt_s *sub = priv;

  /*
   * Response handler for disable operation selection, 'AT+COPS=2'
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCOPS);

  if (resp_status_is_error_or_timeout(info->status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "=2");
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* RF disabled, configure modem. */

  sub->configure_group_pos = 0;
  sub->configure_cmd_pos = 0;

  __ubmodem_cmdprompt_configure_next(modem);
}

static void ATpCFUN_handler(struct ubmodem_s *modem,
                            const struct at_cmd_def_s *cmd,
                            const struct at_resp_info_s *info,
                            const uint8_t *resp_stream,
                            size_t stream_len, void *priv)
{
  struct modem_sub_setup_cmd_prompt_s *sub = priv;
  int err;

  /*
   * Response handler for disabling modem RF, 'AT+CFUN=0'
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCFUN);

  if (resp_status_is_error_or_timeout(info->status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "=0");
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /*
   * Make sure that modem does not do network registration too soon after
   * SIM setup.
   */

  memset(sub, 0, sizeof(*sub));
  sub->try_count = 0;

  err = __ubmodem_send_cmd(modem, &cmd_ATpCOPS, ATpCOPS_handler, sub, "%s", "=2");
  MODEM_DEBUGASSERT(modem, err == OK);
}

static void ATpINVALIDCOMMANDTEST_handler(struct ubmodem_s *modem,
                                          const struct at_cmd_def_s *cmd,
                                          const struct at_resp_info_s *info,
                                          const uint8_t *resp_stream,
                                          size_t stream_len, void *priv)
{
  struct modem_sub_setup_cmd_prompt_s *sub = priv;
  int err;

  /*
   * Response handler for invalid command test.
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpINVALIDCOMMANDTEST);

  if (info->status != RESP_STATUS_CME_ERROR || info->errorcode != 100)
    {
      /*
       * Expected "+CMEE: 100" for invalid command. Command-line is not
       * setup correctly.
       */

      /* Retry enabling enabling CMEE error codes. */

      err = __ubmodem_set_timer(modem, 100, &delay_after_initial_handler, sub);
      if (err == ERROR)
        {
          /* Error here? Add assert? Or just try bailout? */

          MODEM_DEBUGASSERT(modem, false);

          (void)delay_after_initial_handler(modem, -1, sub);
        }

      return;
    }

  /*
   * Make sure that modem does not attempt RF before we want it to.
   * Set RF off with "AT+CFUN=0"
   */

  memset(sub, 0, sizeof(*sub));
  sub->try_count = 0;

  err = __ubmodem_send_cmd(modem, &cmd_ATpCFUN, ATpCFUN_handler, sub, "%s", "=0");
  MODEM_DEBUGASSERT(modem, err == OK);
}

static void ATpCMEE_handler(struct ubmodem_s *modem,
                            const struct at_cmd_def_s *cmd,
                            const struct at_resp_info_s *info,
                            const uint8_t *resp_stream,
                            size_t stream_len, void *priv)
{
  struct modem_sub_setup_cmd_prompt_s *sub = priv;
  int err;

  /*
   * Response handler for enable extended errors 'AT+CMEE'
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCMEE);

  if ((info->status == RESP_STATUS_TIMEOUT ||
       info->status == RESP_STATUS_ERROR) && ++sub->try_count < 10)
    {
      /*
       * Initial command might have been issued multiple times, with left over
       * responses in buffer. So lets retry few times.
       */

      err = __ubmodem_send_cmd(modem, &cmd_ATpCMEE, ATpCMEE_handler, sub, "%s",
                           "=1");
      MODEM_DEBUGASSERT(modem, err == OK);
      return;
    }

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
   * Test that CMEE is enabled with invalid/unknown command.
   */

  err = __ubmodem_send_cmd(modem, &cmd_ATpINVALIDCOMMANDTEST,
                       ATpINVALIDCOMMANDTEST_handler, sub,
                       "%s", "?");
  MODEM_DEBUGASSERT(modem, err == OK);
}

static int delay_after_initial_handler(struct ubmodem_s *modem,
                                       const int timer_id, void * const arg)
{
  struct modem_sub_setup_cmd_prompt_s *sub = arg;
  int err;

  /* Echo disabled, do next command. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpCMEE, ATpCMEE_handler, sub, "%s", "=1");
  MODEM_DEBUGASSERT(modem, err == OK);
  return OK;
}

static void initial_handler(struct ubmodem_s *modem,
                            const struct at_cmd_def_s *cmd,
                            const struct at_resp_info_s *info,
                            const uint8_t *resp_stream,
                            size_t stream_len, void *priv)
{
  struct modem_sub_setup_cmd_prompt_s *sub = priv;
  int err;

  /*
   * Response handler for disable echo command 'ATE0'
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_initial);

  if ((info->status == RESP_STATUS_TIMEOUT ||
       info->status == RESP_STATUS_ERROR) && ++sub->try_count < 10)
    {
      /* As this is first command, retry few times to wake-up modem. */

      err = __ubmodem_send_cmd(modem, &cmd_initial, initial_handler, sub, "%s",
                           "E0");
      MODEM_DEBUGASSERT(modem, err == OK);
      return;
    }

  if (resp_status_is_error_or_timeout(info->status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "E0");
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /*
   * Delay for 100 msec to flush input buffer from modem (we might have issued
   * multiple retries and got multiple OK responses, of which this is the
   * first).
   */

  memset(sub, 0, sizeof(*sub));
  sub->try_count = 0;

  err = __ubmodem_set_timer(modem, 100, &delay_after_initial_handler, sub);
  if (err == ERROR)
    {
      /* Error here? Add assert? Or just try bailout? */

      MODEM_DEBUGASSERT(modem, false);

      (void)delay_after_initial_handler(modem, -1, sub);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_cmdprompt_configure_next
 *
 * Description:
 *   Continue modem configuration.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_cmdprompt_configure_next(struct ubmodem_s *modem)
{
  struct modem_sub_setup_cmd_prompt_s *sub = &modem->sub.setup_cmd_prompt;
  ubmodem_configs_func_t get_commands_fn;
  const ubmodem_send_config_func_t *cmds;
  size_t ncmds;

  do
    {
      ncmds = 0;
      cmds = NULL;

      if (sub->configure_group_pos >= ARRAY_SIZE(ubmodem_config_groups))
        {
          /* All configuration completed. update modem main state. */

          __ubmodem_reached_level(modem, UBMODEM_LEVEL_CMD_PROMPT);
          return;
        }

      /* Get config commands array for current group. */

      get_commands_fn = ubmodem_config_groups[sub->configure_group_pos];
      if (get_commands_fn)
        {
          cmds = get_commands_fn(modem, &ncmds);
        }

      /* Have we completed this group? */

      if (sub->configure_cmd_pos >= ncmds || cmds == NULL)
        {
          /* Completed all commands for this group, or empty group*/

          sub->configure_group_pos++;
          sub->configure_cmd_pos = 0;
          continue;
        }
      else if (cmds[sub->configure_cmd_pos] == NULL)
        {
          /* Null command, skip. */

          sub->configure_cmd_pos++;
          continue;
        }
      else
        {
          /* Got config command function. */

          break;
        }
    }
  while (true);

  /* Proceed with next command. */

  (*cmds[sub->configure_cmd_pos++])(modem);
}

/****************************************************************************
 * Name: __ubmodem_cmdprompt_generic_config_handler
 *
 * Description:
 *   Generic AT response handler for simple configuration commands
 *
 ****************************************************************************/

void
__ubmodem_cmdprompt_generic_config_handler(struct ubmodem_s *modem,
                                           const struct at_cmd_def_s *cmd,
                                           const struct at_resp_info_s *info,
                                           const uint8_t *resp_stream,
                                           size_t stream_len, void *priv)
{
  MODEM_DEBUGASSERT(modem, cmd == priv);

  if (info->status != RESP_STATUS_OK)
    {
      __ubmodem_common_failed_command(modem, cmd, info, "");
      return;
    }

  /* Proceed with next configuration commands */

  __ubmodem_cmdprompt_configure_next(modem);
}

/****************************************************************************
 * Name: __ubmodem_substate_start_setup_cmd_prompt
 *
 * Description:
 *   Start modem AT command prompt setup sequence.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_setup_cmd_prompt(struct ubmodem_s *modem)
{
  struct modem_sub_setup_cmd_prompt_s *sub = &modem->sub.setup_cmd_prompt;
  int err;

  MODEM_DEBUGASSERT(modem, modem->level == UBMODEM_LEVEL_POWERED_ON);

  /* Setup command prompt sequence is:
   *
   * 1. Send 'ATE0' with 0.5 seconds timeout.
   * 2a. Response "OK"  ==> Send "AT+CMEE=1"
   * 2b. Timeout "ATE0" ==> Repeat 1.
   * 3. Get response "OK" to "AT+CMEE=1". ==> Send "AT+CFUN=0".
   * 4. Get response "OK" to "AT+CFUN=0". ==> Send "AT+COPS=2".
   * 5. Done.
   */

  /* Reset sub-state data and initiate sub-state machine work. */

  memset(sub, 0, sizeof(*sub));
  sub->try_count = 0;

  /* Select baudrate 115200, select frame type "8N1", disable echo. */

  err = __ubmodem_send_cmd(modem, &cmd_initial, initial_handler, sub, "%s", "E0");
  MODEM_DEBUGASSERT(modem, err == OK);
}
