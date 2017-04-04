/****************************************************************************
 * apps/system/ubmodem/ubmodem_util.c
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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <apps/system/ubmodem.h>

#include "ubmodem_command.h"
#include "ubmodem_parser.h"
#include "ubmodem_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct modem_check_cmee_priv_s
{
  modem_check_cmee_func_t callback_fn;
  void *callback_priv;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at_cmd_def_s cmd_ATpCMEE_read =
{
  .name         = "+CMEE",
  .resp_format  =
  (const uint8_t[]){
    RESP_FMT_INT8,
  },
  .resp_num     = 1,
};

/****************************************************************************
 * Private Function
 ****************************************************************************/

static bool check_apn_authentication(struct ubmodem_s *modem,
                                     char *buf, size_t buflen)
{
  bool use_auth = false;
  char tmp[16];

  /* APN authentication is needed if APN username/password are given. */

  tmp[0] = '\0';
  if (__ubmodem_config_get_value(modem, "modem.apn_user", tmp, sizeof(tmp)))
    {
      if (tmp[0] != '\0')
        {
          /* APN username configured. */

          use_auth = true;
        }
    }

  tmp[0] = '\0';
  if (__ubmodem_config_get_value(modem, "modem.apn_password", tmp, sizeof(tmp)))
    {
      if (tmp[0] != '\0')
        {
          /* APN password configured. */

          use_auth = true;
        }
    }

  if (buf && buflen > 0)
    {
      /* Give authentication setting. Use 'PAP' if username/password have been
       * set. Otherwise disable APN authentication.*/

      snprintf(buf, buflen, "%o", use_auth);
    }

  return true;
}

static void read_CMEE_handler(struct ubmodem_s *modem,
                              const struct at_cmd_def_s *cmd,
                              const struct at_resp_info_s *info,
                              const uint8_t *resp_stream,
                              size_t stream_len, void *priv)
{
  struct modem_check_cmee_priv_s cmee_cb =
      *(struct modem_check_cmee_priv_s *)priv;
  int8_t cmee_setting = -1;
  bool cmee_ok = false;

  free(priv);

  /*
   * Response handler for 'AT+CMEE?'
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpCMEE_read);

  if (resp_status_is_error_or_timeout(info->status) ||
      info->status != RESP_STATUS_OK)
    {
      goto out;
    }

  /* Read CMEE setting. */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &cmee_setting))
    {
      cmee_setting = -1;
      goto out;
    }

  /* CMEE is configured to '1' at initialization. Other setting means that
   * modem is in unknown state (possibly has reseted itself). */

  cmee_ok = (cmee_setting == 1);

out:

  cmee_cb.callback_fn(modem, cmee_ok, cmee_setting, cmee_cb.callback_priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void __ubmodem_common_failed_command(struct ubmodem_s *modem,
                                 const struct at_cmd_def_s *cmd,
                                 const struct at_resp_info_s *info,
                                 const char *reason)
{
  if (info->status == RESP_STATUS_TIMEOUT)
    {
      /* Modem did not reply? */

      __ubmodem_level_transition_failed(modem, "AT%s%s timed-out.", cmd->name,
                                    reason);
      return;
    }

  if (resp_status_is_error(info->status))
    {
      /* Unexpected error? */

      __ubmodem_level_transition_failed(modem, "AT%s%s error: %d.", cmd->name,
                                    reason, info->errorcode);
      return;
    }
}

/****************************************************************************
 * Name: __ubmodem_config_get_value
 *
 * Description:
 *   Get configuration values for modem.
 *
 * Input Parameters:
 *   variable : Name of configuration item to read
 *   buf      : Buffer for output value
 *   buflen   : Size of buffer
 *
 * Return value:
 *   true: Found variable, value stored to 'buf'.
 *   false: Did not find variable by this name.
 *
 ****************************************************************************/

bool __ubmodem_config_get_value(struct ubmodem_s *modem,
                                const char *variable, char *buf,
                                size_t buflen)
{
  bool ret = false;

  DEBUGASSERT(modem);

  if (modem->config.func)
    {
      /* Get configuration outside module. */

      ret = modem->config.func(modem, variable, buf, buflen,
                               modem->config.priv);
    }

  if (!(ret && buflen > 0 && buf[0] != '\0') &&
      strcmp(variable, "modem.apn_authentication") == 0)
    {
      /* Manually check need for APN authentication by checking
       * APN username/password. */

      return check_apn_authentication(modem, buf, buflen);
    }

  return ret;
}

/****************************************************************************
 * Name: ubmodem_set_config_callback
 *
 * Description:
 *   Setup configuration callback for modem.
 *
 * Input Parameters:
 *   config_cb : Configuration callback
 *   priv      : Private data pointer for callback
 *
 ****************************************************************************/

void ubmodem_set_config_callback(struct ubmodem_s *modem,
                                  ubmodem_config_fn_t config_cb, void *priv)
{
  DEBUGASSERT(modem);

  modem->config.func = config_cb;
  modem->config.priv = priv;
}

/****************************************************************************
 * Name: __ubmodem_set_nonblocking
 *
 * Description:
 *   Setup modem file descriptor non-blocking or blocking
 *
 * Input Parameters:
 *   set    : Set non-blocking if true; set blocking if false
 *
 * Returned Values:
 *   OK: If success
 *   ERROR: If failed
 *
 ****************************************************************************/

int __ubmodem_set_nonblocking(struct ubmodem_s *modem, bool set)
{
  int flags, ret;

  DEBUGASSERT(modem);

  if (set == modem->is_nonblocking)
    return OK;

  flags = fcntl(modem->serial_fd, F_GETFL, 0);
  if (flags == ERROR)
    return ERROR;

  if (!set)
    flags &= ~O_NONBLOCK;
  else
    flags |= O_NONBLOCK;

  ret = fcntl(modem->serial_fd, F_SETFL, flags);
  if (ret == ERROR)
    return ERROR;

  modem->is_nonblocking = set;

  return OK;
}

/****************************************************************************
 * Name: __ubmodem_check_cmee_status
 *
 * Description:
 *   Check current CMEE setting
 *
 * Input Parameters:
 *   callback_fn    : callback result function
 *   callback_priv  : callback private data
 *
 * Returned Values:
 *   OK: If success
 *   ERROR: If failed
 *
 ****************************************************************************/

int __ubmodem_check_cmee_status(struct ubmodem_s *modem,
                                modem_check_cmee_func_t callback_fn,
                                void *callback_priv)
{
  struct modem_check_cmee_priv_s *cmee_cb;
  int err;

  cmee_cb = calloc(1, sizeof(*cmee_cb));
  if (!cmee_cb)
    {
      return ERROR;
    }

  cmee_cb->callback_fn = callback_fn;
  cmee_cb->callback_priv = callback_priv;

  err = __ubmodem_send_cmd(modem, &cmd_ATpCMEE_read, read_CMEE_handler,
                           cmee_cb, "?");
  MODEM_DEBUGASSERT(modem, err == OK);

  return OK;
}

/****************************************************************************
 * Name: __ubmodem_seed_urandom
 ****************************************************************************/

void __ubmodem_seed_urandom(void *buf, size_t buflen)
{
  int fd;

  fd = open("/dev/urandom", O_WRONLY);
  if (fd < 0)
    fd = open("/dev/random", O_WRONLY);

  if (fd >= 0)
    {
      (void)write(fd, buf, buflen);
      close(fd);
    }
}
