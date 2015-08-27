/****************************************************************************
 * apps/system/ubmodem/ubmodem_util.c
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
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

#include <apps/system/ubmodem.h>

#include "ubmodem_command.h"
#include "ubmodem_parser.h"
#include "ubmodem_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
                            const char *variable, char *buf, size_t buflen)
{
  DEBUGASSERT(modem);

  if (modem->config.func)
    {
      /* Get configuration outside module. */

      return modem->config.func(modem, variable, buf, buflen,
                                modem->config.priv);
    }

  return false;
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
