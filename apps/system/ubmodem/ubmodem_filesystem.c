/****************************************************************************
 * apps/system/ubmodem/ubmodem_filesystem.c
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
#include "ubmodem_hw.h"

#ifndef CONFIG_UBMODEM_DISABLE_FILESYSTEM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at_cmd_def_s cmd_ATpUDELFILE =
{
  .name             = "+UDELFILE",
  .resp_format      = NULL,
  .resp_num         = 0,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void cmd_ATpUDELFILE_handler(struct ubmodem_s *modem,
                                    const struct at_cmd_def_s *cmd,
                                    const struct at_resp_info_s *info,
                                    const uint8_t *resp_stream,
                                    size_t stream_len, void *privptr)
{
  struct ubmodem_filesystem_oper_s *priv = privptr;

  /*
   * Response handler for +UDELFILE command.
   */

  if (info->status == RESP_STATUS_OK)
    {
      priv->result = OK;
    }
  else
    {
      priv->result = ERROR;
    }

  /* Update main state machine. */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);

  /* Report success/failure. */

  __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_FILESYSTEM_OPER_COMPLETE,
                          priv, sizeof(*priv));
  free(priv);

  if (info->status == RESP_STATUS_TIMEOUT)
    {
      int ret;

      /* Timeout for UDELFILE command can mean only one thing: Modem is stuck
       * in bad state. */

      /* Prepare task for bringing modem alive from stuck state. */

      ret = __ubmodem_recover_stuck_hardware(modem);
      MODEM_DEBUGASSERT(modem, ret != ERROR);
      return;
    }
}

static int start_filesystem_operation(struct ubmodem_s *modem, void *privptr)
{
  struct ubmodem_filesystem_oper_s *priv = privptr;
  int err;

  if (modem->level < UBMODEM_LEVEL_CMD_PROMPT)
    {
      priv->result = ERROR;
      __ubmodem_publish_event(modem,
                              UBMODEM_EVENT_FLAG_FILESYSTEM_OPER_COMPLETE,
                              priv, sizeof(*priv));
      free(priv);
      return ERROR;
    }

  /* Launch filesystem operation. */

  switch (priv->type)
    {
    case MODEM_FILESYSTEM_DELETE:
      {
        err = __ubmodem_send_cmd(modem, &cmd_ATpUDELFILE,
                                 cmd_ATpUDELFILE_handler,
                                 priv, "=\"%s\"", priv->oper.delete.name);
        MODEM_DEBUGASSERT(modem, err == OK);
      }

      break;

    default:
      DEBUGASSERT(false);
      break;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubmodem_filesystem_delete
 *
 * Description:
 *   Delete file in modem filesystem
 *
 * Input Parameters:
 *   modem           : Modem structure
 *   filename        : Name of file to delete
 *
 * Returned valued:
 *   ERROR if failed, OK on successfully start of task.
 *
 ****************************************************************************/

int ubmodem_filesystem_delete(struct ubmodem_s *modem, const char *filename)
{
  struct ubmodem_filesystem_oper_s *priv;
  ssize_t len;

  DEBUGASSERT(modem);
  DEBUGASSERT(filename);

  if (modem->level < UBMODEM_LEVEL_CMD_PROMPT)
    {
      errno = ENODEV;
      return ERROR;
    }

  priv = calloc(1, sizeof(*priv));
  if (!priv)
    {
      return ERROR;
    }

  priv->type = MODEM_FILESYSTEM_DELETE;

  len = snprintf(priv->oper.delete.name, sizeof(priv->oper.delete.name), "%s",
                 filename);
  if (len >= sizeof(priv->oper.delete.name))
    {
      /* Too long filename. */

      free(priv);
      errno = EINVAL;
      return ERROR;
    }

  /* Add modem task. */

  return __ubmodem_add_task(modem, start_filesystem_operation, priv);
}

#endif /* !CONFIG_UBMODEM_DISABLE_FILESYSTEM */
