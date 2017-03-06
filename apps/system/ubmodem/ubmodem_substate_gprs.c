/****************************************************************************
 * apps/system/ubmodem/ubmodem_substate_gprs.c
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
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>

#include <apps/system/ubmodem.h>

#include "ubmodem_internal.h"
#include "ubmodem_usrsock.h"
#include "ubmodem_hw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CGATT_TRY_DELAY_MSEC (5 * 1000)
#define CGATT_TRY_COUNT 6

#define GPRS_DISCONNECT_CGATT_ABORT_RETRIES 20

#define UPSND_CHECK_RETRY_DELAY_SECS 10
#define UPSND_CHECK_RETRIES 6

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at_cmd_def_s cmd_ATpCGATT =
{
  .name         = "+CGATT",
  .resp_format  = (const uint8_t[]){ RESP_FMT_INT8, },
  .resp_num     = 1,
  .timeout_dsec = MODEM_CMD_NETWORK_TIMEOUT,
};

static const struct at_cmd_def_s cmd_ATpCGATT_disable =
{
  .name         = "+CGATT",
  .resp_format  = NULL,
  .resp_num     = 0,
  .timeout_dsec = 10 * 10, /* 10 seconds timeout */
};

static const struct at_cmd_def_s cmd_ATpUPSND_query_int8 =
{
  .name         = "+UPSND",
  .resp_format  =
    (const uint8_t[]){
      RESP_FMT_INT8,
      RESP_FMT_INT8,
      RESP_FMT_INT8,
    },
  .resp_num     = 3,
  .timeout_dsec = 10 * 10, /* 10 seconds timeout */
};

static const struct at_cmd_def_s cmd_ATpUPSND_query_string =
{
  .name         = "+UPSND",
  .resp_format  =
    (const uint8_t[]){
      RESP_FMT_INT8,
      RESP_FMT_INT8,
      RESP_FMT_QUOTED_STRING,
    },
  .resp_num     = 3,
  .timeout_dsec = 10 * 10, /* 10 seconds timeout */
};

static const struct at_cmd_def_s cmd_ATpUPSD =
{
 .name         = "+UPSD",
 .resp_format  = NULL,
 .resp_num     = 0,
};

static const struct at_cmd_def_s cmd_ATpUPSDA =
{
  .name         = "+UPSDA",
  .resp_format  = NULL,
  .resp_num     = 0,
};

static const struct at_cmd_def_s cmd_ATpUPSDA_activate =
{
  .name         = "+UPSDA",
  .resp_format  = NULL,
  .resp_num     = 0,
  .timeout_dsec = 150 * 10,
};

static const struct at_cmd_def_s cmd_ATpUPSDA_deactivate =
{
  .name         = "+UPSDA",
  .resp_format  = NULL,
  .resp_num     = 0,
  .timeout_dsec = MODEM_CMD_NETWORK_TIMEOUT,
};

static const struct at_cmd_def_s cmd_ATpUUPSDD_urc =
{
  .name         = "+UUPSDD",
  .resp_format  = (const uint8_t[]){ RESP_FMT_INT8 },
  .resp_num     = 1,
};

static const struct at_cmd_def_s cmd_abort =
{
  .name         = "ABORTED",
  .resp_format  = NULL,
  .resp_num     = 0,
  .timeout_dsec = 10, /* 1 second */
};

static int8_t apn_conf_type[5] = { 1, 6, 2, 3, 7 };

static int8_t apn_conf_format[5] = { 's', 'i', 's', 's', 's' };

static const char *apn_conf_type_name[5] =
{
  "modem.apn_name",
  "modem.apn_authentication",
  "modem.apn_user",
  "modem.apn_password",
  "modem.apn_ipaddr"
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ATpUPSD_handler(struct ubmodem_s *modem,
                            const struct at_cmd_def_s *cmd,
                            const struct at_resp_info_s *info,
                            const uint8_t *resp_stream,
                            size_t stream_len, void *priv);

static void set_ATpCGATT_handler(struct ubmodem_s *modem,
                                 const struct at_cmd_def_s *cmd,
                                 const struct at_resp_info_s *info,
                                 const uint8_t *resp_stream,
                                 size_t stream_len, void *priv);

static int retry_check_cgatt(struct ubmodem_s *modem, const int timer_id,
                             void * const arg);

static int retry_upsnd_check(struct ubmodem_s *modem, const int timer_id,
                             void * const arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void check_ipconfig_ATpUPSDA_handler(struct ubmodem_s *modem,
                                            const struct at_cmd_def_s *cmd,
                                            const struct at_resp_info_s *info,
                                            const uint8_t *resp_stream,
                                            size_t stream_len, void *priv)
{
  struct modem_sub_gprs_s *sub = priv;
  int8_t val = 0;
  uint16_t strlen = 0;
  const char *str = 0;
  struct in_addr ipaddr = {};
  int err;

  /*
   * Response handler for 'AT+UPSND=0,<pos>'
   */

  if (resp_status_is_error_or_timeout(info->status))
    {
      char buf[8];

      snprintf(buf, sizeof(buf), "=0,%d", sub->ipconfig.pos);

      __ubmodem_common_failed_command(modem, cmd, info, buf);
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Check stream for IP address. */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &val) || val != 0)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &val) ||
      val != sub->ipconfig.pos)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  if (!__ubmodem_stream_get_string(&resp_stream, &stream_len, &str, &strlen))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  switch (sub->ipconfig.pos)
    {
    case 0:
      /* Parse current IP-address. */

      if (inet_pton(AF_INET, str, &ipaddr.s_addr) < 0)
        {
          dbg("invalid IP4 address: '%s'\n", str);
          ipaddr.s_addr = 0;
        }

      memset(&sub->ipconfig.ipcfg, 0, sizeof(sub->ipconfig.ipcfg));
      sub->ipconfig.ipcfg.ipaddr = ipaddr;

      /* Next get primary DNS server address. */

      sub->ipconfig.pos = 1;

      break;

    case 1:
      /* Parse primary DNS server address. */

      if (inet_pton(AF_INET, str, &ipaddr.s_addr) < 0)
        {
          dbg("invalid primary DNS server address: '%s'\n", str);
          ipaddr.s_addr = 0;
        }

      sub->ipconfig.ipcfg.dns1 = ipaddr;

      /* Next get secondary DNS server address. */

      sub->ipconfig.pos = 2;

      break;

    case 2:
      /* Parse secondary DNS server address. */

      if (inet_pton(AF_INET, str, &ipaddr.s_addr) < 0)
        {
          dbg("invalid secondary DNS server address: '%s'\n", str);
          ipaddr.s_addr = 0;
        }

      sub->ipconfig.ipcfg.dns2 = ipaddr;

      /* Done reading IP configuration. */

      sub->ipconfig.pos = -1;

      break;
    }

  if (sub->ipconfig.pos < 0)
    {
      /* Done reading IP configuration. */

#ifdef CONFIG_UBMODEM_USRSOCK
      /* Initialize user sockets. */

      __ubmodem_usrsock_initialize(modem, &sub->ipconfig.ipcfg);
#endif

      /* Report that we have active GPRS connection. */

      __ubmodem_reached_level(modem, UBMODEM_LEVEL_GPRS);

      /* Report IP configuration. */

      __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_IP_ADDRESS,
                              &sub->ipconfig.ipcfg,
                              sizeof(sub->ipconfig.ipcfg));

      return;
    }

  /* Read next value. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpUPSND_query_string,
                           check_ipconfig_ATpUPSDA_handler, sub, "=0,%d",
                           sub->ipconfig.pos);
  MODEM_DEBUGASSERT(modem, err == OK);
}

static int reattempt_gprs_connection(struct ubmodem_s *modem, void *priv)
{
  ubdbg("\n");

  /* Go to network enabled level from current and then retry current. */

  __ubmodem_retry_current_level(modem, UBMODEM_LEVEL_NETWORK);

  /* Return error to tell task starter that task did not start new work on
   * state machine. */

  return ERROR;
}

static void urc_pUUPSDD_handler(struct ubmodem_s *modem,
                                const struct at_cmd_def_s *cmd,
                                const struct at_resp_info_s *info,
                                const uint8_t *resp_stream,
                                size_t stream_len, void *priv)
{
  int8_t val;

  /*
   * Response handler for +UUPSDD URC.
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpUUPSDD_urc);

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

  ubdbg("val: %d\n", val);

  /*
   * PSD connection lost.
   */

  /* Unregister +UUPSDD URC */

  __ubparser_unregister_response_handler(&modem->parser,
                                         cmd_ATpUUPSDD_urc.name);
  modem->uupsdd_urc_registered = false;

  /* Add task to start reconnection. */

  __ubmodem_add_task(modem, reattempt_gprs_connection, NULL);
}

static void activate_ATpUPSDA_handler(struct ubmodem_s *modem,
                                      const struct at_cmd_def_s *cmd,
                                      const struct at_resp_info_s *info,
                                      const uint8_t *resp_stream,
                                      size_t stream_len, void *priv)
{
  struct modem_sub_gprs_s *sub = priv;
  int err;

  /*
   * Response handler for 'AT+UPSDA=0,3'
   */

  if (resp_status_is_error_or_timeout(info->status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "=0,3");
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /*
   * Internal context activated.
   */

  /* Register URC handler */

  __ubparser_register_response_handler(&modem->parser, &cmd_ATpUUPSDD_urc,
                                       urc_pUUPSDD_handler, modem, true);
  modem->uupsdd_urc_registered = true;

  /* Get IP address and IP addresses of DNS servers. */

  sub->ipconfig.pos = 0;

  err = __ubmodem_send_cmd(modem, &cmd_ATpUPSND_query_string,
                           check_ipconfig_ATpUPSDA_handler, sub, "=0,%d",
                           sub->ipconfig.pos);
  MODEM_DEBUGASSERT(modem, err == OK);
}

static void send_next_apn_configuration(struct ubmodem_s *modem)
{
  char buf[64];
  struct modem_sub_gprs_s *sub = &modem->sub.gprs;
  int err;

  /* Get next non-empty APN configuration setting. */

  while (++sub->apn_conf.pos < (int)ARRAY_SIZE(apn_conf_type))
    {
      if (!__ubmodem_config_get_value(modem, apn_conf_type_name[sub->apn_conf.pos],
                                  buf, sizeof(buf)))
        continue;

      if (buf[0] == '\0')
        continue;

      if (apn_conf_format[sub->apn_conf.pos] == 's')
        {
          /* Text string in buffer. */

          snprintf(sub->apn_conf.value, sizeof(sub->apn_conf.value), "=0,%d,\"%s\"",
                   apn_conf_type[sub->apn_conf.pos], buf);
        }
      else if (apn_conf_format[sub->apn_conf.pos] == 'i')
        {
          /* Integer string in buffer. */

          snprintf(sub->apn_conf.value, sizeof(sub->apn_conf.value), "=0,%d,%s",
                   apn_conf_type[sub->apn_conf.pos], buf);
        }
      else
        {
          MODEM_DEBUGASSERT(modem, false);
        }

      err = __ubmodem_send_cmd(modem, &cmd_ATpUPSD, ATpUPSD_handler, sub, "%s",
                           sub->apn_conf.value);
      MODEM_DEBUGASSERT(modem, err == OK);
      return;
    }

  /* Configuration complete, activate internal PDP context. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpUPSDA_activate,
                       activate_ATpUPSDA_handler, sub, "%s", "=0,3");
  MODEM_DEBUGASSERT(modem, err == OK);
}

static void ATpUPSD_handler(struct ubmodem_s *modem,
                            const struct at_cmd_def_s *cmd,
                            const struct at_resp_info_s *info,
                            const uint8_t *resp_stream,
                            size_t stream_len, void *priv)
{
  struct modem_sub_gprs_s *sub = priv;

  /*
   * Response handler for 'AT+UPSD=0,<id>,"<value>"'
   */

  if (resp_status_is_error_or_timeout(info->status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, sub->apn_conf.value);
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Set next APN configuration value */

  send_next_apn_configuration(modem);
}

static void start_configuring_pdp_context(struct ubmodem_s *modem)
{
  struct modem_sub_gprs_s *sub = &modem->sub.gprs;
  int err;

  /* Reset sub-state data. */

  sub->apn_conf.pos = -1;
  strcpy(sub->apn_conf.value, "=0,0");

  /* Reset previous configuration. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpUPSDA, ATpUPSD_handler, sub, "%s",
                       sub->apn_conf.value);
  MODEM_DEBUGASSERT(modem, err == OK);
  return;
}

static void deactivate_ATpUPSDA_handler(struct ubmodem_s *modem,
                                        const struct at_cmd_def_s *cmd,
                                        const struct at_resp_info_s *info,
                                        const uint8_t *resp_stream,
                                        size_t stream_len, void *priv)
{
  /*
   * Response handler for 'AT+UPSDA=0,4'
   */

  if (resp_status_is_error_or_timeout(info->status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "=0,4");
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /*
   * Internal context deactivated. Continue with context configuration.
   */

  start_configuring_pdp_context(modem);
}

static void check_ATpUPSND_handler(struct ubmodem_s *modem,
                                   const struct at_cmd_def_s *cmd,
                                   const struct at_resp_info_s *info,
                                   const uint8_t *resp_stream,
                                   size_t stream_len, void *priv)
{
  struct modem_sub_gprs_s *sub = &modem->sub.gprs;
  int8_t val;
  int err;
  int ret;

  /*
   * Response handler for 'AT+UPSND=0,8'
   */

  if (resp_status_is_error_or_timeout(info->status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "=0,8");
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Read profile ID. */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &val) ||
      val != 0)
    {
      /* Should not happen. Retry. */

      goto retry;
    }

  /* Read parameter tag. */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &val) ||
      val != 8)
    {
      /* Should not happen. Retry. */

      goto retry;
    }

  /* Read parameter value. */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &val))
    {
      /* Should not happen. Retry. */

      goto retry;
    }

  if (val == 0)
    {
      /*
       * Internal context not connected. Continue with context
       * configuration.
       */

      start_configuring_pdp_context(modem);
      return;
    }

  /* Deactivate internal context. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpUPSDA_deactivate,
                       deactivate_ATpUPSDA_handler, sub, "%s", "=0,4");
  MODEM_DEBUGASSERT(modem, err == OK);
  return;

retry:
  if (sub->retry++ >= UPSND_CHECK_RETRIES)
    {
      __ubmodem_level_transition_failed(modem,
          "GPRS: Could not read state of internal PDP context.");
      return;
    }

  /* Modem not ready to proceed. */

  ret = __ubmodem_set_timer(modem, UPSND_CHECK_RETRY_DELAY_SECS * 1000,
                            &retry_upsnd_check, sub);
  MODEM_DEBUGASSERT(modem, ret != ERROR);
}

static int retry_upsnd_check(struct ubmodem_s *modem, const int timer_id,
                             void * const arg)
{
  struct modem_sub_gprs_s *sub = &modem->sub.gprs;
  int err;

  /* Check state of PDP context. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpUPSND_query_int8,
                           check_ATpUPSND_handler, sub, "%s", "=0,8");
  MODEM_DEBUGASSERT(modem, err == OK);

  return OK;
}

static void start_setup_internal_pdp_context(struct ubmodem_s *modem)
{
  struct modem_sub_gprs_s *sub = &modem->sub.gprs;

  sub->retry = 0;
  retry_upsnd_check(modem, -1, sub);
}

static int modem_set_cgatt_retry_timer_handler(struct ubmodem_s *modem,
                                               const int timer_id,
                                               void * const arg)
{
  int err;
  struct modem_sub_gprs_s *sub = &modem->sub.gprs;

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);
  err = __ubmodem_send_cmd(modem, &cmd_ATpCGATT, set_ATpCGATT_handler,
                       sub, "%s", "=1");
  MODEM_DEBUGASSERT(modem, err == OK);
  return OK;
}

static void set_ATpCGATT_handler(struct ubmodem_s *modem,
                                 const struct at_cmd_def_s *cmd,
                                 const struct at_resp_info_s *info,
                                 const uint8_t *resp_stream,
                                 size_t stream_len, void *priv)
{
  struct modem_sub_gprs_s *sub = &modem->sub.gprs;

  /*
   * Response handler for 'AT+CGATT=1'
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCGATT);

  if (resp_status_is_error_or_timeout(info->status))
    {
      if (--sub->retry > 0)
        {
          int err;

          /* Retry few times before giving up. */

          ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);
          err = __ubmodem_set_timer(modem, CGATT_TRY_DELAY_MSEC,
                                  &modem_set_cgatt_retry_timer_handler,
                                  sub);
          if (err == ERROR)
            {
              /* Error here? Add assert? Or just try bailout? */

              MODEM_DEBUGASSERT(modem, false);

              (void)modem_set_cgatt_retry_timer_handler(modem, -1, sub);
            }
        }
      else
        {
          __ubmodem_common_failed_command(modem, cmd, info, "=1");
        }
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  start_setup_internal_pdp_context(modem);
}

static void check_ATpCGATT_handler(struct ubmodem_s *modem,
                                   const struct at_cmd_def_s *cmd,
                                   const struct at_resp_info_s *info,
                                   const uint8_t *resp_stream,
                                   size_t stream_len, void *priv)
{
  struct modem_sub_gprs_s *sub = &modem->sub.gprs;
  int8_t val;
  int err;

  /*
   * Response handler for 'AT+CGATT?'
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCGATT);

  if (resp_status_is_error_or_timeout(info->status))
    {
      __ubmodem_common_failed_command(modem, cmd, info, "?");
      return;
    }

  if (info->status != RESP_STATUS_OK)
    {
      /* Should not get here. */

      MODEM_DEBUGASSERT(modem, false);
      return;
    }

  /* Check GPRS state */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &val))
    {
      /* Happens when received 'OK' for previous command. Retry after short
       * wait. */

      ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);
      err = __ubmodem_set_timer(modem, 500, retry_check_cgatt, sub);
      MODEM_DEBUGASSERT(modem, err != ERROR);
      return;
    }

  if (val == 1)
    {
      /* GPRS enabled, then enable internal pdp context. */

      start_setup_internal_pdp_context(modem);
      return;
    }

  /* Enable GPRS. */

  sub->retry = CGATT_TRY_COUNT;

  err = __ubmodem_send_cmd(modem, &cmd_ATpCGATT, set_ATpCGATT_handler, sub,
                           "%s", "=1");
  MODEM_DEBUGASSERT(modem, err == OK);
}

static int retry_check_cgatt(struct ubmodem_s *modem, const int timer_id,
                             void * const arg)
{
  int err;

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);

  /* Check +CGATT. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpCGATT, check_ATpCGATT_handler, arg,
                           "%s", "?");
  MODEM_DEBUGASSERT(modem, err == OK);
  return OK;
}

static void disable_ATpCGATT_handler(struct ubmodem_s *modem,
                                     const struct at_cmd_def_s *cmd,
                                     const struct at_resp_info_s *info,
                                     const uint8_t *resp_stream,
                                     size_t stream_len, void *priv)
{
  struct modem_sub_gprs_s *sub = &modem->sub.gprs;
  int err;

  /*
   * Response handler for 'AT+CGATT=0'
   */

  if (info->status == RESP_STATUS_TIMEOUT)
    {
      if (cmd != &cmd_abort ||
          sub->retry++ < GPRS_DISCONNECT_CGATT_ABORT_RETRIES)
        {
          /* Attempt to abort command. Modem responds with "OK" (Sara-G) or
           * "ABORTED" (Sara-U). */

          err = __ubmodem_send_raw(modem, &cmd_abort, disable_ATpCGATT_handler,
                                   priv, "\r\n", 2);
          MODEM_DEBUGASSERT(modem, err == OK);

          return;
        }
      else
        {
          /* Drop from network level to stuck hardware, need clean-up. */

          __ubmodem_network_cleanup(modem);

          /* Could not abort, assume stuck HW. */

          __ubmodem_reached_level(modem, UBMODEM_LEVEL_STUCK_HARDWARE);
          return;
        }
    }

  if (resp_status_is_error_or_timeout(info->status))
    {
      /* Internal context might have been deactived by modem already, thus
       * making command to return error. Just continue. */

      __ubmodem_reached_level(modem, UBMODEM_LEVEL_NETWORK);

      return;
    }

  /*
   * Internal context deactivated.
   */

  __ubmodem_reached_level(modem, UBMODEM_LEVEL_NETWORK);
}

static int reinitialize_gprs_task(struct ubmodem_s *modem, void *privptr)
{
  if (modem->level < UBMODEM_LEVEL_GPRS)
    {
      /* Need to be in GRPS state to be able to reinitialize. */

      return ERROR;
    }

  /* Go to network level from GRPS level and then retry GPRS. */

  __ubmodem_retry_current_level(modem, UBMODEM_LEVEL_NETWORK);

  /* Return error to tell task starter that task did not start new work on
   * state machine. */

  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_substate_start_open_gprs
 *
 * Description:
 *   Open GPRS connection sequence.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_open_gprs(struct ubmodem_s *modem)
{
  struct modem_sub_gprs_s *sub = &modem->sub.gprs;
  int err;

  MODEM_DEBUGASSERT(modem, modem->level == UBMODEM_LEVEL_NETWORK);

  /*
   * Open sequence is:
   *
   * 1. Send "AT+CGATT?".
   * 2. If (response[0] == 0) ==> Send "AT+CGATT=1".
   *    else                  ==> 3.
   * 3. Send "AT+UPSND=0,8".
   * 4. If (response[2] == 1) ==> Send "AT+UPSDA=0,4".
   *    If (response[2] != 1) ==> 5.
   * 5. Send 'AT+UPSD=0,1,"<APN_NAME>"'.
   * 6. Send 'AT+UPSD=0,2,"<APN_USERNAME>"'.
   * 7. Send 'AT+UPSD=0,3,"<APN_PASSWD"'.
   * 8. Send 'AT+UPSD=0,7,"0.0.0.0"'.
   * 9. Send 'AT+UPSDA=0,3'.
   * 10. Done.
   */

  /* Reset sub-state data and initiate sub-state machine work. */

  memset(sub, 0, sizeof(*sub));

  /* Check +CGATT. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpCGATT, check_ATpCGATT_handler, sub,
                       "%s", "?");
  MODEM_DEBUGASSERT(modem, err == OK);
}

/****************************************************************************
 * Name: __ubmodem_grps_cleanup
 *
 * Description:
 *   Clean-up GPRS connection state.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_gprs_cleanup(struct ubmodem_s *modem)
{
  if (modem->uupsdd_urc_registered)
    {
      __ubparser_unregister_response_handler(&modem->parser,
                                             cmd_ATpUUPSDD_urc.name);
      modem->uupsdd_urc_registered = false;
    }

#ifdef CONFIG_UBMODEM_USRSOCK
  /* Clean-up any active sockets first. */

  __ubmodem_usrsock_uninitialize(modem);
#endif

  /* Disable cell location callback and generate CellLocate-failed event. */

  __ubmodem_cell_locate_cleanup(modem, false);

  /* Cancel FTP download and generate FTP failed event. */

  __ubmodem_ftp_download_cleanup(modem);
}

/****************************************************************************
 * Name: __ubmodem_substate_start_close_gprs
 *
 * Description:
 *   Close GPRS connection sequence.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_substate_start_close_gprs(struct ubmodem_s *modem)
{
  struct modem_sub_gprs_s *sub = &modem->sub.gprs;
  int err;

  MODEM_DEBUGASSERT(modem, modem->level == UBMODEM_LEVEL_GPRS);

  __ubmodem_gprs_cleanup(modem);

  /*
   * Close sequence is:
   *
   * 1. Send "AT+CGATT=0".
   * 10. Done.
   */

  /* Reset sub-state data and initiate sub-state machine work. */

  memset(sub, 0, sizeof(*sub));

  /* Deactivate internal context. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpCGATT_disable,
                           disable_ATpCGATT_handler, sub, "%s", "=0");
  MODEM_DEBUGASSERT(modem, err == OK);
}

/****************************************************************************
 * Name: __ubmodem_reinitialize_gprs
 *
 * Description:
 *   Reinitialize GPRS connection
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

int __ubmodem_reinitialize_gprs(struct ubmodem_s *modem)
{
  /* Add modem task. */

  return __ubmodem_add_task(modem, reinitialize_gprs_task, NULL);
}
