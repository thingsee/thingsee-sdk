/****************************************************************************
 * apps/system/ubmodem/ubmodem_usrsock_close.c
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
#include <arpa/inet.h>
#include <sys/socket.h>
#include <errno.h>

#include <apps/system/ubmodem.h>

#include <nuttx/net/usrsock.h>

#include "ubmodem_internal.h"
#include "ubmodem_usrsock.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at_cmd_def_s cmd_ATpUSOCL =
{
  .name         = "+USOCL",
  .resp_format  = NULL,
  .resp_num     = 0,
  .timeout_dsec = MODEM_CMD_SOCKET_TIMEOUT,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: close_socket_handler
 *
 * Description:
 *   Handler for socket close command result.
 ****************************************************************************/

static void close_socket_handler(struct ubmodem_s *modem,
                                 const struct at_cmd_def_s *cmd,
                                 const struct at_resp_info_s *info,
                                 const uint8_t *resp_stream,
                                 size_t stream_len, void *priv)
{
  struct modem_socket_s *sock = priv;

  /*
   * Response handler for +USOCL socket close.
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpUSOCL);

  if (resp_status_is_error_or_timeout(info->status) ||
      info->status != RESP_STATUS_OK)
    {
      /* Treat failed socket close as OK, as +USOCL does not give socket
       * errors. */
    }

  /* Socket is closed now. */

  /* Report that connection was closed. */

  sock->is_closed = true;
  sock->modem_sd = -1;

  (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, 0);

  /* Mark socket for removal. */

  sock->is_freeing_pending = true;
  __ubsocket_work_done(sock);
}

/****************************************************************************
 * Public Internal Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_close_socket
 *
 * Description:
 *   Send close socket AT command.
 ****************************************************************************/

void __ubmodem_close_socket(struct modem_socket_s *sock)
{
  struct ubmodem_s *modem = sock->modem;
  int err;

  /* Attempt to close socket. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpUSOCL, close_socket_handler, sock,
                           "=%d", sock->modem_sd);
  MODEM_DEBUGASSERT(modem, err == OK);
}

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_close_request
 *
 * Description:
 *   Handle close request for socket
 ****************************************************************************/

int __ubmodem_usrsock_handle_close_request(struct ubmodem_s *modem,
                                           void *reqbuf)
{
  struct usrsock_request_close_s *req = reqbuf;
  struct modem_socket_s *sock;
  int err;

  /* Get socket for request. */

  sock = __ubmodem_socket_get_by_usockid(modem, req->usockid);
  if (!sock)
    {
      /* Give error result. */

      err = -EBADF;
      goto err_out;
    }

  /* Check if socket is already closed. */

  if (sock->is_closed || sock->modem_sd < 0)
    {
      /* Already closed, but usrsock does not know that yet, return success. */

      err = 0;

      /* Mark socket for removal. */

      sock->is_freeing_pending = true;

      /* Wake-up main state machine. */

      __ubsocket_update_state(sock);
      __ubmodem_wake_waiting_state(modem, false);

      goto err_out;
    }

  /* Mark socket for closing. */

  sock->is_closing = true;
  sock->req = req->head;

  /* Wake-up main state machine. */

  __ubsocket_update_state(sock);
  __ubmodem_wake_waiting_state(modem, false);

  /* Report that request has been read and is being processed. */

  return __ubmodem_usrsock_send_response(modem, reqbuf, true, -EINPROGRESS);

err_out:
  return __ubmodem_usrsock_send_response(modem, reqbuf, false, err);
}
