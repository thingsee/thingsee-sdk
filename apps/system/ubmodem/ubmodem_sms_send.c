/****************************************************************************
 * apps/system/ubmodem/ubmodem_sms_send.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
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

#include "ubmodem_internal.h"
#include "ubmodem_pdu_util.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MODEM_SEND_SMS_CONFIG_RETRIES    60
#define MODEM_SEND_SMS_CONFIG_RETRY_SECS 1

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct send_sms_priv_s
{
  ubmodem_send_sms_cb_t result_cb;
  void *result_priv;
  uint8_t *recv_pdu;
  size_t recv_pdulen;
  uint8_t *msg_pdu;
  size_t msg_pdulen;
  char *pdu;
  size_t pdulen;
  struct multipdu_iter_s multi_iter;
  int8_t config_pos;
  int8_t config_retries;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at_cmd_def_s cmd_ATpCMGS_write_pdu =
{
  .name             = "+CMGS",
  .resp_format      =
      (const uint8_t[]){
        RESP_FMT_INT32,
      },
  .resp_num         = 1,
  .timeout_dsec     = MODEM_CMD_NETWORK_TIMEOUT,
};

static const struct at_cmd_def_s cmd_ATpCMGS_open_prompt =
{
  .name             = "+CMGS",
  .resp_format      = NULL,
  .resp_num         = 0,
  .flag_pdu_prompt  = true,
};

static const struct at_cmd_def_s cmd_ATpCGSMS =
{
  .name             = "+CGSMS",
  .resp_format      = NULL,
  .resp_num         = 0,
};

static const struct at_cmd_def_s cmd_ATpCMGF =
{
  .name             = "+CMGF",
  .resp_format      = NULL,
  .resp_num         = 0,
};

static const struct at_cmd_def_s cmd_ATpCSCA =
{
  .name             = "+CSCA",
  .resp_format      =
      (const uint8_t[]){
        RESP_FMT_QUOTED_STRING,
        RESP_FMT_INT16,
      },
  .resp_num         = 2,
  .timeout_dsec     = MODEM_CMD_SIM_MGMT_TIMEOUT,
};

static const struct
{
  const struct at_cmd_def_s *cmd;
  const char *args;
} sms_config_cmds[] =
  {
    /* Select service for MO SMS messages, PSD preferred (CSD used if PSD not
     * avail). This allows sending SMS when GRPS is activated. */

    { &cmd_ATpCGSMS, "=2" },

    /* PDU mode format for sending messages. */

    { &cmd_ATpCMGF, "=0" },

    /* Print SMSC. */

    { &cmd_ATpCSCA, "?" },
  };

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int send_multipart_sms_pdu(struct ubmodem_s *modem, void *priv);
static int configure_sms(struct ubmodem_s *modem, void *priv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void pdu_write_handler(struct ubmodem_s *modem,
                              const struct at_cmd_def_s *cmd,
                              const struct at_resp_info_s *info,
                              const uint8_t *resp_stream,
                              size_t stream_len, void *priv)
{
  struct send_sms_priv_s *sms_priv = priv;

  free(sms_priv->pdu);
  sms_priv->pdu = NULL;

  /*
   * Response handler for PDU write.
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCMGS_write_pdu);

  if (resp_status_is_error_or_timeout(info->status))
    {
      /* Failed to send SMS? */

      free(sms_priv->msg_pdu);
      free(sms_priv->recv_pdu);

      if (sms_priv->result_cb)
        {
          sms_priv->result_cb(modem, false, sms_priv->result_priv);
        }

      free(sms_priv);

      goto done;
    }

  /* More parts to send? */

  if (sms_priv->multi_iter.part_pos < sms_priv->multi_iter.part_count)
    {
      /* Prepare to send next part. */

      if (send_multipart_sms_pdu(modem, sms_priv) == ERROR)
        {
          goto done;
        }

      return;
    }

  if (sms_priv->result_cb)
    {
      sms_priv->result_cb(modem, true, sms_priv->result_priv);
    }

done:
  __ubmodem_change_state(modem, MODEM_STATE_WAITING);
}

static void pdu_prompt_handler(struct ubmodem_s *modem,
                               const struct at_cmd_def_s *cmd,
                               const struct at_resp_info_s *info,
                               const uint8_t *resp_stream,
                               size_t stream_len, void *priv)
{
  struct send_sms_priv_s *sms_priv = priv;
  int err;

  /*
   * Response handler for send SMS, 'AT+CMGS=<pdulen>'
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCMGS_open_prompt);

  if (resp_status_is_error_or_timeout(info->status) ||
      info->status != RESP_STATUS_DATAPROMPT)
    {
      /*
       * Failed to open pdu prompt? This should not happen, modem should
       * always return '>' and only fail after writing of specified data.
       */

      free(sms_priv->pdu);
      free(sms_priv->msg_pdu);
      free(sms_priv->recv_pdu);

      if (sms_priv->result_cb)
        {
          sms_priv->result_cb(modem, false, sms_priv->result_priv);
        }

      free(sms_priv);

      __ubmodem_change_state(modem, MODEM_STATE_WAITING);
      return;
    }

  /*
   * PDU prompt is now open.
   */

  /* Replace null-term with 'CTRL-Z'. */

  sms_priv->pdu[sms_priv->pdulen] = '\x1A';

  err = __ubmodem_send_raw(modem, &cmd_ATpCMGS_write_pdu, pdu_write_handler,
                           sms_priv, sms_priv->pdu, sms_priv->pdulen + 1);
  MODEM_DEBUGASSERT(modem, err == OK);
}

static int send_multipart_sms_pdu(struct ubmodem_s *modem, void *priv)
{
  struct send_sms_priv_s *sms_priv = priv;
  int len;
  int err;

  if (modem->level < UBMODEM_LEVEL_NETWORK)
    {
      free(sms_priv->pdu);
      free(sms_priv->msg_pdu);
      free(sms_priv->recv_pdu);

      if (sms_priv->result_cb)
        {
          sms_priv->result_cb(modem, false, sms_priv->result_priv);
        }

      free(sms_priv);

      return ERROR;
    }

  /* Get next PDU for message. */

  sms_priv->pdulen = __ubmodem_prepare_pdu_string(sms_priv->recv_pdu,
                                                  sms_priv->recv_pdulen,
                                                  sms_priv->msg_pdu,
                                                  sms_priv->msg_pdulen,
                                                  &sms_priv->multi_iter,
                                                  &sms_priv->pdu);
  if (sms_priv->pdulen <= 0)
    {
      free(sms_priv->pdu);
      free(sms_priv->msg_pdu);
      free(sms_priv->recv_pdu);

      if (sms_priv->result_cb)
        {
          sms_priv->result_cb(modem, false, sms_priv->result_priv);
        }

      free(sms_priv);

      return ERROR;
    }

  /* PDU octet length without SMSC part. */

  len = sms_priv->pdulen / 2 - __ubmodem_get_pdu_smsc_len(sms_priv->pdu);

  /* Open text/PDU prompt. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpCMGS_open_prompt,
                           pdu_prompt_handler, sms_priv,
                           "=%d", len);
  MODEM_DEBUGASSERT(modem, err == OK);
  return OK;
}

static int retry_configure_timer_handler(struct ubmodem_s *modem,
                                         const int timer_id,
                                         void * const arg)
{
  struct send_sms_priv_s *sms_priv = arg;

  if (configure_sms(modem, sms_priv) == ERROR)
    {
      __ubmodem_change_state(modem, MODEM_STATE_WAITING);
    }
  return OK;
}

static void configure_sms_handler(struct ubmodem_s *modem,
                                  const struct at_cmd_def_s *cmd,
                                  const struct at_resp_info_s *info,
                                  const uint8_t *resp_stream,
                                  size_t stream_len, void *priv)
{
  struct send_sms_priv_s *sms_priv = priv;
  int err;

  if (resp_status_is_error_or_timeout(info->status))
    {
      if (sms_priv->config_pos == ARRAY_SIZE(sms_config_cmds) &&
          sms_priv->config_retries++ < MODEM_SEND_SMS_CONFIG_RETRIES &&
          modem->level >= UBMODEM_LEVEL_NETWORK)
        {
          /* Reading message service address failed. This happens when
           * network connection has just been created and modem has not
           * fully initialized network connection. Wait for short period and
           * retry. */

          /*  Retry configuration. */

          sms_priv->config_pos = 0;

          err = __ubmodem_set_timer(modem,
                                    MODEM_SEND_SMS_CONFIG_RETRY_SECS * 1000,
                                    &retry_configure_timer_handler,
                                    sms_priv);
          if (err == ERROR)
            {
              /* Error here? Add assert? Or just try bailout? */

              MODEM_DEBUGASSERT(modem, false);

              (void)retry_configure_timer_handler(modem, -1, sms_priv);
            }

          return;
        }

      /* Failed to setup SMS sending? */

      free(sms_priv->pdu);
      free(sms_priv->msg_pdu);
      free(sms_priv->recv_pdu);

      if (sms_priv->result_cb)
        {
          sms_priv->result_cb(modem, false, sms_priv->result_priv);
        }

      free(sms_priv);

      __ubmodem_change_state(modem, MODEM_STATE_WAITING);
      return;
    }

  if (sms_priv->config_pos < ARRAY_SIZE(sms_config_cmds))
    {
      /* Perform next configuration. */

      if (configure_sms(modem, sms_priv) == ERROR)
        {
          __ubmodem_change_state(modem, MODEM_STATE_WAITING);
          return;
        }
    }
  else
    {
      /* Prepare to send first part. */

      if (send_multipart_sms_pdu(modem, sms_priv) == ERROR)
        {
          __ubmodem_change_state(modem, MODEM_STATE_WAITING);
          return;
        }
    }
}

static int configure_sms(struct ubmodem_s *modem, void *priv)
{
  struct send_sms_priv_s *sms_priv = priv;
  int err;

  if (modem->level < UBMODEM_LEVEL_NETWORK)
    {
      goto err_out;
    }

  if (sms_priv->config_pos < ARRAY_SIZE(sms_config_cmds))
    {
      const struct at_cmd_def_s *cmd =
          sms_config_cmds[sms_priv->config_pos].cmd;
      const char *args = sms_config_cmds[sms_priv->config_pos].args;

      sms_priv->config_pos++;

      err = __ubmodem_send_cmd(modem, cmd, configure_sms_handler, sms_priv,
                               args);
      MODEM_DEBUGASSERT(modem, err == OK);
      return OK;
    }

err_out:
  free(sms_priv->pdu);
  free(sms_priv->msg_pdu);
  free(sms_priv->recv_pdu);

  if (sms_priv->result_cb)
    {
      sms_priv->result_cb(modem, false, sms_priv->result_priv);
    }

  free(sms_priv);

  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubmodem_send_sms
 *
 * Description:
 *   Attempt to send SMS.
 *
 * Input Parameters:
 *   modem     : Pointer for modem library object from ubmodem_initialize
 *   receiver  : Phone number to send SMS
 *   message   : Text message to send, UTF-8 encoded
 *   result_cb : SMS sent/failed result callback
 *   result_priv: private data for callback
 *
 * Returned valued:
 *   ERROR if failed, OK on success.
 *
 ****************************************************************************/

int ubmodem_send_sms(struct ubmodem_s *modem, const char *receiver,
                     const char *message, ubmodem_send_sms_cb_t result_cb,
                     void *result_priv)
{
  static uint8_t g_multipart_ref = 0x80;
  struct send_sms_priv_s *sms_priv;
  uint8_t *msg_pdu = NULL;
  uint8_t *recv_pdu = NULL;
  int msg_pdulen;
  int recv_pdulen;
  int err;

  DEBUGASSERT(modem);

  /* Network connectivity required for SMS sending. */

  if (modem->level < UBMODEM_LEVEL_NETWORK)
    {
      err = ENONET;
      goto err_out;
    }

  /* Convert receiver phone number to PDU block. */

  recv_pdulen = __ubmodem_address_to_pdu(receiver, &recv_pdu);
  if (recv_pdulen <= 0)
    {
      err = ENOMEM;
      goto err_out;
    }

  /* Convert message to PDU block. */

  msg_pdulen = __ubmodem_utf8_to_pdu(message, &msg_pdu);
  if (msg_pdulen < 0)
    {
      err = ENOMEM;
      goto err_out;
    }

  /* Prepare SMS sending task. */

  sms_priv = calloc(1, sizeof(*sms_priv));
  if (sms_priv == NULL)
    {
      err = ENOMEM;
      goto err_out;
    }

  sms_priv->msg_pdulen = msg_pdulen;
  sms_priv->msg_pdu = msg_pdu;
  sms_priv->recv_pdulen = recv_pdulen;
  sms_priv->recv_pdu = recv_pdu;
  sms_priv->result_cb = result_cb;
  sms_priv->result_priv = result_priv;
  sms_priv->pdu = NULL;
  sms_priv->config_pos = 0;
  sms_priv->config_retries = 0;
  __ubmodem_pdu_multipart_iterator_init(&sms_priv->multi_iter,
                                        msg_pdulen,
                                        g_multipart_ref++);

  /* Add modem task. */

  return __ubmodem_add_task(modem, configure_sms, sms_priv);

err_out:
  free(msg_pdu);
  free(recv_pdu);
  errno = err;
  return ERROR;
}
