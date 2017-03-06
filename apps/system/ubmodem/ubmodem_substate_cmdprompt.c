/****************************************************************************
 * apps/system/ubmodem/ubmodem_substate_cmdprompt.c
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

#ifndef CONFIG_UBMODEM_POWERSAVE_GUARD_PERIOD
#  define CONFIG_UBMODEM_POWERSAVE_GUARD_PERIOD 0
#endif

#if CONFIG_UBMODEM_POWERSAVE_GUARD_PERIOD > 0
#  if CONFIG_UBMODEM_POWERSAVE_GUARD_PERIOD < 40 || \
      CONFIG_UBMODEM_POWERSAVE_GUARD_PERIOD > 65000
#    error "CONFIG_UBMODEM_POWERSAVE_GUARD_PERIOD not in valid range"
#  endif
#endif

#ifndef CONFIG_UBMODEM_FAST_DORMANCY_DELAY_TIMER
#  define CONFIG_UBMODEM_FAST_DORMANCY_DELAY_TIMER 3
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

static const ubmodem_send_config_func_t *
get_common_config_cmds(struct ubmodem_s *modem, size_t *ncmds);

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

static const struct at_cmd_def_s cmd_enable_hw_flowcontrol =
{
  .name         = "&K3\\Q3",
  .resp_num     = 0,
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

static const struct at_cmd_def_s cmd_ATI0 =
{
  .name         = "I0",
  .resp_format  = (const uint8_t[]){ RESP_FMT_STRING_TO_EOL },
  .resp_num     = 1,
  .flag_plain   = true,
};

static const struct at_cmd_def_s cmd_ATpCGMR =
{
  .name         = "+CGMR",
  .resp_format  = (const uint8_t[]){ RESP_FMT_STRING_TO_EOL },
  .resp_num     = 1,
  .flag_plain   = true,
};

static const struct at_cmd_def_s cmd_ATpUMWI =
{
  .name         = "+UMWI",
  .resp_format  = NULL,
  .resp_num     = 0
};

static const struct at_cmd_def_s cmd_ATpUHSDUPA =
{
  .name         = "+UHSDUPA",
  .resp_format  = NULL,
  .resp_num     = 0
};

static const struct at_cmd_def_s cmd_ATpUFDAC =
{
  .name         = "+UFDAC",
  .resp_format  = NULL,
  .resp_num     = 0
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
  get_common_config_cmds,
#ifdef CONFIG_UBMODEM_POWERSAVE
  get_powersave_config_cmds,
#endif
#ifdef CONFIG_UBMODEM_VOICE
  __ubmodem_voice_get_config_commands,
  __ubmodem_voice_get_audiopath_config_commands,
#endif
  __ubmodem_celllocate_get_config_cmds,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ATI0_handler(struct ubmodem_s *modem,
                            const struct at_cmd_def_s *cmd,
                            const struct at_resp_info_s *info,
                            const uint8_t *resp_stream,
                            size_t stream_len, void *priv)
{
  bool ok;
  const char *str;

  modem->model = UBMODEM_MODEL_UNKNOWN;
  modem->order_code[0] = '\0';

  if (info->status == RESP_STATUS_OK)
    {
      /* Parse order code. */

      ok = __ubmodem_stream_get_string(&resp_stream, &stream_len, &str, NULL);
      if (!ok)
        {
          str = "UNKNOWN";
        }
      else
        {
          snprintf(modem->order_code, sizeof(modem->order_code), "%s", str);
        }

      /* Get model from order code. */

      if (strncmp(str, "SARA-G", 6) == 0)
        {
          int sara_g_number = atoi(str + 6);

          /* This is SARA-G series modem. */

          switch (sara_g_number)
            {
              case 300:
                modem->model = UBMODEM_MODEL_SARA_G300;
                break;
              case 310:
                modem->model = UBMODEM_MODEL_SARA_G310;
                break;
              case 340:
                modem->model = UBMODEM_MODEL_SARA_G340;
                break;
              case 350:
                modem->model = UBMODEM_MODEL_SARA_G350;
                break;
              default:
                modem->model = UBMODEM_MODEL_SARA_G_UNKNOWN;
                break;
            }
        }
      else if (strncmp(str, "SARA-U", 6) == 0)
        {
          int sara_u_number = atoi(str + 6);

          /* This is SARA-U series modem. */

          switch (sara_u_number)
            {
              case 260:
                modem->model = UBMODEM_MODEL_SARA_U260;
                break;
              case 270:
                modem->model = UBMODEM_MODEL_SARA_U270;
                break;
              case 280:
                modem->model = UBMODEM_MODEL_SARA_U280;
                break;
              default:
                modem->model = UBMODEM_MODEL_SARA_U_UNKNOWN;
                break;
            }
        }

      dbg("Modem order code: %s\n", str);
    }

  /* Continue with next configuration. */

  __ubmodem_cmdprompt_generic_config_handler(modem, cmd, info, NULL, 0,
                                             (void *)cmd);
}

static void config_get_modem_model(struct ubmodem_s *modem)
{
  int err;

  /* ATI0 gives 'order code', has modem model at head. */

  err = __ubmodem_send_cmd(modem, &cmd_ATI0, ATI0_handler, NULL, "");
  MODEM_DEBUGASSERT(modem, err == OK);
}

static void ATpCGMR_handler(struct ubmodem_s *modem,
                            const struct at_cmd_def_s *cmd,
                            const struct at_resp_info_s *info,
                            const uint8_t *resp_stream,
                            size_t stream_len, void *priv)
{
  const char *str = "UNKNOWN";

  if (info->status == RESP_STATUS_OK)
    {
      (void)__ubmodem_stream_get_string(&resp_stream, &stream_len, &str, NULL);
    }

  dbg("Modem FW version: %s\n", str);

  /* Continue with next configuration. */

  __ubmodem_cmdprompt_generic_config_handler(modem, cmd, info, NULL, 0,
                                             (void *)cmd);
}

static void config_get_modem_fw_version(struct ubmodem_s *modem)
{
  int err;

  err = __ubmodem_send_cmd(modem, &cmd_ATpCGMR, ATpCGMR_handler, NULL, "");
  MODEM_DEBUGASSERT(modem, err == OK);
}

static void config_disable_umwi_urc(struct ubmodem_s *modem)
{
  int err;

  if (__ubmodem_is_model_sara_g(modem))
    {
      /* +UMWI not available on Sara-G */

      /* Continue with next configuration. */

      struct at_resp_info_s info_ok = {};

      info_ok.status = RESP_STATUS_OK;

      __ubmodem_cmdprompt_generic_config_handler(modem, &cmd_ATpUMWI,
                                                 &info_ok, NULL, 0,
                                                 (void *)&cmd_ATpUMWI);
      return;
    }

  /* Disable +UMWI URC. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpUMWI,
                           __ubmodem_cmdprompt_generic_config_handler,
                           (void *)&cmd_ATpUMWI, "=0");
  MODEM_DEBUGASSERT(modem, err == OK);
}

static void config_disable_hsdupa(struct ubmodem_s *modem)
{
  int err;

  if (__ubmodem_is_model_sara_g(modem))
    {
      /* HSDPA/HSUPA not available on Sara-G */

      /* Continue with next configuration. */

      struct at_resp_info_s info_ok = {};

      info_ok.status = RESP_STATUS_OK;

      __ubmodem_cmdprompt_generic_config_handler(modem, &cmd_ATpUHSDUPA,
                                                 &info_ok, NULL, 0,
                                                 (void *)&cmd_ATpUHSDUPA);
      return;
    }

  /* Disable HSDPA/HSUPA for 3G. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpUHSDUPA,
                           __ubmodem_cmdprompt_generic_config_handler,
                           (void *)&cmd_ATpUHSDUPA, "=0,8,0,6");
  MODEM_DEBUGASSERT(modem, err == OK);
}

static void config_setup_fast_dormancy(struct ubmodem_s *modem)
{
  int err;

  if (__ubmodem_is_model_sara_g(modem))
    {
      /* Fast Dormancy not available on Sara-G */

      /* Continue with next configuration. */

      struct at_resp_info_s info_ok = {};

      info_ok.status = RESP_STATUS_OK;

      __ubmodem_cmdprompt_generic_config_handler(modem, &cmd_ATpUFDAC,
                                                 &info_ok, NULL, 0,
                                                 (void *)&cmd_ATpUFDAC);
      return;
    }

#if defined(CONFIG_UBMODEM_FAST_DORMANCY_DELAY_TIMER) && \
    CONFIG_UBMODEM_FAST_DORMANCY_DELAY_TIMER > 0
  char setup[16];

  snprintf(setup, sizeof(setup), "2,%d,%d",
           CONFIG_UBMODEM_FAST_DORMANCY_DELAY_TIMER,
           CONFIG_UBMODEM_FAST_DORMANCY_DELAY_TIMER);
#else
  const char *setup = "3"; /* 3 => disable auto FD. */
#endif

  /* Setup autonomous Fast Dormancy for 3G. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpUFDAC,
                           __ubmodem_cmdprompt_generic_config_handler,
                           (void *)&cmd_ATpUFDAC, "=%s", setup);
  MODEM_DEBUGASSERT(modem, err == OK);
}

static const ubmodem_send_config_func_t *
get_common_config_cmds(struct ubmodem_s *modem, size_t *ncmds)
{
  static const ubmodem_send_config_func_t commands[] =
  {
    config_get_modem_model,
    config_get_modem_fw_version,
    config_disable_umwi_urc,
    config_disable_hsdupa,
    config_setup_fast_dormancy,
  };

  *ncmds = ARRAY_SIZE(commands);

  return commands;
}

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

      ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);
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

static void enable_hw_flowcontrol_handler(struct ubmodem_s *modem,
                                          const struct at_cmd_def_s *cmd,
                                          const struct at_resp_info_s *info,
                                          const uint8_t *resp_stream,
                                          size_t stream_len, void *priv)
{
  struct modem_sub_setup_cmd_prompt_s *sub = priv;
  int err;

  /*
   * Response handler for enable HW flowcontrol (AT&K3\Q3)
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_enable_hw_flowcontrol);

  if ((info->status == RESP_STATUS_TIMEOUT ||
       info->status == RESP_STATUS_ERROR) && ++sub->try_count < 10)
    {
      /*
       * Initial command might have been issued multiple times, with left over
       * responses in buffer. So lets retry few times.
       */

      err = __ubmodem_send_cmd(modem, &cmd_enable_hw_flowcontrol,
                               enable_hw_flowcontrol_handler, sub, "");
      MODEM_DEBUGASSERT(modem, err == OK);
      return;
    }

  if (resp_status_is_error_or_timeout(info->status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "");
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /*
   * Enable CMEE error codes.
   */

  err = __ubmodem_send_cmd(modem, &cmd_ATpCMEE, ATpCMEE_handler, sub,
                           "%s", "=1");
  MODEM_DEBUGASSERT(modem, err == OK);
}

static int delay_after_initial_handler(struct ubmodem_s *modem,
                                       const int timer_id, void * const arg)
{
  struct modem_sub_setup_cmd_prompt_s *sub = arg;
  int err;

  /* Echo disabled, do next command. */

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);
  err = __ubmodem_send_cmd(modem, &cmd_enable_hw_flowcontrol,
                           enable_hw_flowcontrol_handler, sub, "");
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
      if (++modem->cmd_prompt_ate0_attempts >= 4)
        {
          /* ATE0 has failed repeatedly. Either voltage level is too low
           * or modem not connected. Report to upper layer about the issue.
           *
           * For Sara-G to start-up, battery voltage must be >= 3.30V
           * For Sara-U to start-up, battery voltage must be >= 3.35V
           */

          enum ubmodem_error_event_e error =
              UBMODEM_ERROR_STARTUP_VOLTAGE_TOO_LOW;

          __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_ERROR, &error,
                                  sizeof(error));
        }

      __ubmodem_common_failed_command(modem, cmd, info, "E0");
      return;
    }

  modem->cmd_prompt_ate0_attempts = 0;

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

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);
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
  struct modem_sub_setup_cmd_prompt_s *sub = &modem->sub.setup_cmd_prompt;

  MODEM_DEBUGASSERT(modem, cmd == priv);

  if (info->status != RESP_STATUS_OK)
    {
      if (sub->configure_retries++ >= UBMODEM_CONFIGURE_RETRIES)
        {
          __ubmodem_common_failed_command(modem, cmd, info, "");
        }
      else
        {
          /* Proceed with retry current configuration command */

          sub->configure_cmd_pos--;
          __ubmodem_cmdprompt_configure_next(modem);
        }

      return;
    }

  sub->configure_retries = 0;

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
