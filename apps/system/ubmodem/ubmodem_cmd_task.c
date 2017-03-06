/****************************************************************************
 * apps/system/ubmodem/ubmodem_cmd_task.c
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

#include "ubmodem_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct command_task_s
{
  const struct at_cmd_def_s *cmd;
  modem_command_task_callback_t callback;
  void *callback_priv;
  void *buf;
  size_t buflen;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void command_callback(struct ubmodem_s *modem,
                             const struct at_cmd_def_s *cmd,
                             const struct at_resp_info_s *info,
                             const uint8_t *resp_stream,
                             size_t stream_len,
                             void *priv)
{
  struct command_task_s *cmdtask = priv;

  cmdtask->callback(cmd, info, resp_stream, stream_len, cmdtask->callback_priv);

  free(cmdtask);

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);
}

static int start_raw_command(struct ubmodem_s *modem, void *priv)
{
  struct command_task_s *cmdtask = priv;
  int err;

  if (modem->level < UBMODEM_LEVEL_CMD_PROMPT)
    {
      free(cmdtask);
      return ERROR;
    }

  err = __ubmodem_send_raw(modem, cmdtask->cmd, command_callback, cmdtask,
                           cmdtask->buf, cmdtask->buflen);
  MODEM_DEBUGASSERT(modem, err == OK);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubmodem_start_raw_command
 ****************************************************************************/

int ubmodem_send_raw_command(struct ubmodem_s *modem,
                             const struct at_cmd_def_s *cmd,
                             const modem_command_task_callback_t callback,
                             void *callback_priv, const void *buf,
                             size_t buflen)
{
  struct command_task_s *cmdtask;

  DEBUGASSERT(modem);

  cmdtask = calloc(1, sizeof(*cmdtask) + buflen);
  if (!cmdtask)
    {
      errno = ENOMEM;
      return ERROR;
    }

  cmdtask->cmd = cmd;
  cmdtask->callback = callback;
  cmdtask->callback_priv = callback_priv;
  cmdtask->buf = (char *)cmdtask + sizeof(*cmdtask);
  cmdtask->buflen = buflen;

  memmove(cmdtask->buf, buf, buflen);

  /* Add modem task. */

  return __ubmodem_add_task(modem, start_raw_command, cmdtask);
}
