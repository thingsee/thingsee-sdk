/****************************************************************************
 * apps/system/ubmodem/ubmodem_info.c
 *
 *   Copyright (C) 2015-2017 Haltian Ltd. All rights reserved.
 *   Author: Timo Voutilainen <timo.voutilainen@haltian.com>
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

#include "ubmodem_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SIM_TRY_DELAY_MSEC 1000
#define SIM_RETRIES        5

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct info_priv_s
{
  ubmodem_info_cb_t result_cb;
  void *caller_data;
  ubmodem_info_type_t type;
  const struct at_cmd_def_s *cmd;
  modem_response_callback_t at_handler;
  const char *args;
  enum ubmodem_func_level_e min_level;
  int retries;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Request IMEI */
static const struct at_cmd_def_s cmd_ATpCGSN_query_string =
{
  .name         = "+CGSN",
  .resp_format  = (const uint8_t[]){ RESP_FMT_STRING_TO_EOL },
  .resp_num     = 1,
  .flag_plain   = true,
};

/* Request IMSI */
static const struct at_cmd_def_s cmd_ATpCIMI_query_string =
{
  .name         = "+CIMI",
  .resp_format  = (const uint8_t[]){ RESP_FMT_STRING_TO_EOL },
  .resp_num     = 1,
  .flag_plain   = true,
  .timeout_dsec = MODEM_CMD_SIM_MGMT_TIMEOUT,
};

/* Request ICCID */
static const struct at_cmd_def_s cmd_ATpCCID_query_string =
{
  .name         = "+CCID",
  .resp_format  = (const uint8_t[]){ RESP_FMT_STRING_TO_EOL },
  .resp_num     = 1,
  .timeout_dsec = MODEM_CMD_SIM_MGMT_TIMEOUT,
};

/* Request MSISDN */
static const struct at_cmd_def_s cmd_ATpCNUM_query_string =
{
  .name               = "+CNUM",
  .resp_format  =
    (const uint8_t[]){
      RESP_FMT_QUOTED_STRING,
      RESP_FMT_STRING,
      RESP_FMT_INT16,
    },
  .resp_num           = 3,
  .flag_multiple_resp = true, /* Multiple MSISDN possible, we pick first. */
  .timeout_dsec       = MODEM_CMD_SIM_MGMT_TIMEOUT,
};

/* Request Operator Name (Short ROM) */
static const struct at_cmd_def_s cmd_ATpUDOPN_query_string =
{
  /* response_type = 0 */
  .name         = "+UDOPN",
  .resp_format  =
    (const uint8_t[]){
      RESP_FMT_INT8,
      RESP_FMT_QUOTED_STRING,
      RESP_FMT_QUOTED_STRING,
    },
  .resp_num     = 3,
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int ubmodem_start_read_info(struct ubmodem_s *modem, void *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int modem_sim_retry_timer_handler(struct ubmodem_s *modem,
                                         const int timer_id, void * const priv)
{
  struct info_priv_s *info_priv = (struct info_priv_s *)priv;
  int err;

  info_priv->retries++;

  err = ubmodem_start_read_info(modem, priv);
  if (err != 0)
    {
      /* Go to waiting state */

      __ubmodem_change_state(modem, MODEM_STATE_WAITING);
    }

  return OK;
}

static bool sim_not_ready(struct ubmodem_s *modem,
                          const struct at_resp_info_s *info,
                          struct info_priv_s *info_priv)
{
  int err;

  if (info_priv->min_level == UBMODEM_LEVEL_SIM_ENABLED)
    {
      if (info->status == RESP_STATUS_CME_ERROR &&
          (info->errorcode >= 13 && info->errorcode <= 15) &&
          info_priv->retries < SIM_RETRIES)
        {
          /* SIM reading command, retry for sometime if SIM CME error 13-15 is
           * returned. */

          err = __ubmodem_set_timer(modem, SIM_TRY_DELAY_MSEC,
                                    &modem_sim_retry_timer_handler,
                                    info_priv);
          if (err == ERROR)
            {
              /* Error here? Add assert? Or just try bailout? */

              MODEM_DEBUGASSERT(modem, false);

              (void)modem_sim_retry_timer_handler(modem, -1, info_priv);
            }

          return false;
        }
    }

  return true;
}

static void AT_string_resp_handler(struct ubmodem_s *modem,
                                   const struct at_cmd_def_s *cmd,
                                   const struct at_resp_info_s *info,
                                   const uint8_t *resp_stream,
                                   size_t stream_len, void *priv)
{
  struct info_priv_s info_priv = *(struct info_priv_s *)priv;
  uint16_t str_len = 0;
  const char *str = NULL;

  MODEM_DEBUGASSERT(modem, cmd == info_priv.cmd);

  if (!sim_not_ready(modem, info, priv))
    {
      return;
    }

  ubdbg("Stream length: %d\n", stream_len);

  free(priv);

  if (resp_status_is_error_or_timeout(info->status))
    {
      info_priv.result_cb(info_priv.caller_data, NULL, 0, false,
                          info_priv.type);
      __ubmodem_common_failed_command(modem, cmd, info, "");
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }


  if (!__ubmodem_stream_get_string(&resp_stream, &stream_len, &str, &str_len))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Finally call user's callback */

  info_priv.result_cb(info_priv.caller_data, str, str_len, true, info_priv.type);

  /* Go to waiting state */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);
}

static void CCID_resp_handler(struct ubmodem_s *modem,
                              const struct at_cmd_def_s *cmd,
                              const struct at_resp_info_s *info,
                              const uint8_t *stream,
                              size_t stream_len, void *priv)
{
  const char *iccid;
  struct info_priv_s info_priv = *(struct info_priv_s *)priv;

  if (!sim_not_ready(modem, info, priv))
    {
      return;
    }

  free(priv);

  if (resp_status_is_error_or_timeout(info->status))
    {
      info_priv.result_cb(info_priv.caller_data, NULL, 0, false,
                          info_priv.type);
      __ubmodem_common_failed_command(modem, cmd, info, "");
      return;
    }

  if (!__ubmodem_stream_get_string(&stream, &stream_len, &iccid, NULL))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  info_priv.result_cb(info_priv.caller_data, iccid, strlen(iccid),
                      true, info_priv.type);

  /* Go to waiting state */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);
}

static void CNUM_resp_handler(struct ubmodem_s *modem,
                              const struct at_cmd_def_s *cmd,
                              const struct at_resp_info_s *info,
                              const uint8_t *stream,
                              size_t stream_len, void *priv)
{
  struct info_priv_s *info_priv = (struct info_priv_s *)priv;

  if (!sim_not_ready(modem, info, priv))
    {
      return;
    }

  if (resp_status_is_error_or_timeout(info->status))
    {
      info_priv->result_cb(info_priv->caller_data, NULL, 0, false,
                           info_priv->type);
      __ubmodem_common_failed_command(modem, cmd, info, "");
      free(priv);
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      if (info_priv->result_cb)
        {
          const char *name;
          const char *msisdn;
          int16_t num_type;

          if (!__ubmodem_stream_get_string(&stream, &stream_len, &name, NULL))
            {
              MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
              return;
            }

          if (!__ubmodem_stream_get_string(&stream, &stream_len, &msisdn, NULL))
            {
              MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
              return;
            }

          if (!__ubmodem_stream_get_int16(&stream, &stream_len, &num_type))
            {
              MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
              return;
            }

          info_priv->result_cb(info_priv->caller_data, msisdn, strlen(msisdn),
                               true, info_priv->type);
          info_priv->result_cb = NULL;
        }
    }
  else
    {
      if (info_priv->result_cb)
        {
          info_priv->result_cb(info_priv->caller_data, NULL, 0, false,
                               info_priv->type);
        }

      /* Go to waiting state */

      __ubmodem_change_state(modem, MODEM_STATE_WAITING);
      free(priv);
    }
}

static void UDOPN_resp_handler(struct ubmodem_s *modem,
                               const struct at_cmd_def_s *cmd,
                               const struct at_resp_info_s *info,
                               const uint8_t *stream,
                               size_t stream_len, void *priv)
{
  int8_t type;
  const char *name;
  const char *display_condition;
  struct info_priv_s info_priv = *(struct info_priv_s *)priv;

  if (!sim_not_ready(modem, info, priv))
    {
      return;
    }

  free(priv);

  if (resp_status_is_error_or_timeout(info->status))
    {
      info_priv.result_cb(info_priv.caller_data, NULL, 0, false,
                          info_priv.type);
      __ubmodem_common_failed_command(modem, cmd, info, "");
      return;
    }

  if (!__ubmodem_stream_get_int8(&stream, &stream_len, &type))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  if (!__ubmodem_stream_get_string(&stream, &stream_len, &name, NULL))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  if (!__ubmodem_stream_get_string(&stream, &stream_len, &display_condition, NULL))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  info_priv.result_cb(info_priv.caller_data, name, strlen(name), true, info_priv.type);

  /* Go to waiting state */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);
}

static int ubmodem_start_read_info(struct ubmodem_s *modem, void *priv)
{
  struct info_priv_s *info_priv = priv;
  int err;

  if (modem->level < info_priv->min_level)
    {
      goto err_out;
    }

  err = __ubmodem_send_cmd(modem, info_priv->cmd, info_priv->at_handler,
                           info_priv, "%s", info_priv->args);
  MODEM_DEBUGASSERT(modem, err == OK);
  return OK;

err_out:
  info_priv->result_cb(info_priv->caller_data, NULL, 0, false, info_priv->type);
  free(info_priv);
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubmodem_get_info
 *
 * Description:
 *   Attempt to start reading info from modem
 *
 * Input Parameters:
 *   modem      - pointer to modem
 *   result_cb  - pointer to caller's callback function
 *   priv       - pointer to caller's private data
 *   type       - type of info requested
 *
 * Returned valued:
 *   ERROR if failed, OK on success.
 *
 ****************************************************************************/

int ubmodem_get_info(struct ubmodem_s *modem, ubmodem_info_cb_t result_cb,
                     void *priv, ubmodem_info_type_t type)
{
  struct info_priv_s *info_priv;
  int err;

  DEBUGASSERT(modem);

  if (modem->level < UBMODEM_LEVEL_POWERED_ON)
    {
      err = ENODEV;
      goto err_out;
    }

  info_priv = calloc(1, sizeof(*info_priv));
  if (info_priv == NULL)
    {
      err = ENOMEM;
      goto err_out;
    }

  info_priv->result_cb = result_cb;
  info_priv->caller_data = priv;
  info_priv->type = type;
  info_priv->args = "";
  info_priv->min_level = UBMODEM_LEVEL_POWERED_ON;

  switch (info_priv->type)
  {
  case UB_INFO_IMEI:
    info_priv->cmd = &cmd_ATpCGSN_query_string;
    info_priv->at_handler = AT_string_resp_handler;
    break;
  case UB_INFO_IMSI:
    info_priv->cmd = &cmd_ATpCIMI_query_string;
    info_priv->at_handler = AT_string_resp_handler;
    info_priv->min_level = UBMODEM_LEVEL_SIM_ENABLED;
    break;
  case UB_INFO_ICCID:
    info_priv->cmd = &cmd_ATpCCID_query_string;
    info_priv->at_handler = CCID_resp_handler;
    info_priv->min_level = UBMODEM_LEVEL_SIM_ENABLED;
    break;
  case UB_INFO_MSISDN_1:
    info_priv->cmd = &cmd_ATpCNUM_query_string;
    info_priv->at_handler = CNUM_resp_handler;
    info_priv->min_level = UBMODEM_LEVEL_SIM_ENABLED;
    break;
  case UB_INFO_UDOPN:
    info_priv->cmd = &cmd_ATpUDOPN_query_string;
    info_priv->at_handler = UDOPN_resp_handler;
    info_priv->args = "=1";
    info_priv->min_level = UBMODEM_LEVEL_NETWORK;
    break;
  case UB_INFO_MCC_MNC:
    info_priv->cmd = &cmd_ATpUDOPN_query_string;
    info_priv->at_handler = UDOPN_resp_handler;
    info_priv->args = "=0";
    info_priv->min_level = UBMODEM_LEVEL_NETWORK;
    break;
  default:
    free(info_priv);
    err = EINVAL;
    goto err_out;
    break;
  }

  return __ubmodem_add_task(modem, ubmodem_start_read_info, info_priv);

err_out:

  errno = err;
  return ERROR;
}
