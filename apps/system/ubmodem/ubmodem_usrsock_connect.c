/****************************************************************************
 * apps/system/ubmodem/ubmodem_usrsock_connect.c
 *
 *   Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
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

static const struct at_cmd_def_s cmd_ATpUSOCO =
{
  .name         = "+USOCO",
  .resp_format  = NULL,
  .resp_num     = 0,
  .timeout_dsec = MODEM_CMD_SOCKET_CONNECT_TIMEOUT,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: connect_socket_handler
 ****************************************************************************/

static void connect_socket_handler(struct ubmodem_s *modem,
                                   const struct at_cmd_def_s *cmd,
                                   const struct at_resp_info_s *info,
                                   const uint8_t *resp_stream,
                                   size_t stream_len, void *priv)
{
  struct modem_socket_s *sock = priv;

  /*
   * Response handler for connect TCP sockets,
   * 'AT+USOCO=<id>,"<hostname>",<hostport>'
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpUSOCO);

  if (sock->is_closed)
    {
      /* Socket has been closed, report error. */

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);
      __ubsocket_work_done(sock);

      return;
    }

  if (info->status == RESP_STATUS_ERROR)
    {
      /* Not CME error, fetch sockets error. */

      __ubmodem_get_socket_error(sock);
      return;
    }

  if (resp_status_is_error_or_timeout(info->status) ||
      info->status != RESP_STATUS_OK)
    {
      /* Some other error, report generic error. */

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);
      __ubsocket_work_done(sock);

      return;
    }

  /* Report success and inform that Tx is ready. */

  (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, 0);
  (void)__ubmodem_usrsock_send_event(sock, USRSOCK_EVENT_SENDTO_READY);

  /* Done connecting. */

  __ubsocket_work_done(sock);
}

/****************************************************************************
 * Public Internal Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_connect_socket
 *
 * Description:
 *   Send connect sockets AT command.
 ****************************************************************************/

void __ubmodem_connect_socket(struct modem_socket_s *sock)
{
  struct ubmodem_s *modem = sock->modem;
  const uint8_t *ip = (const uint8_t *)&sock->connect.peeraddr.sin_addr;
  int err;

  /* Attempt to connect sockets. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpUSOCO, connect_socket_handler, sock,
                           "=%d,\"%d.%d.%d.%d\",%d",
                           sock->modem_sd, ip[0], ip[1], ip[2], ip[3],
                           ntohs(sock->connect.peeraddr.sin_port));
  MODEM_DEBUGASSERT(modem, err == OK);
}

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_connect_request
 *
 * Description:
 *   Handle connect request for sockets
 ****************************************************************************/

int __ubmodem_usrsock_handle_connect_request(struct ubmodem_s *modem,
                                            void *reqbuf)
{
  struct usrsock_request_connect_s *req = reqbuf;
  struct modem_socket_s *sock;
  struct sockaddr_in connaddr = {};
  ssize_t rlen;
  int err;

  /* Get sockets for request. */

  sock = __ubmodem_socket_get_by_usockid(modem, req->usockid);
  if (!sock)
    {
      err = -EBADF;
      goto err_out;
    }

  /* Check if sockets is closed. */

  if (sock->is_closed || sock->modem_sd < 0)
    {
      err = -EPIPE;
      goto err_out;
    }

  /* Is given address of correct size? */

  if (req->addrlen < sizeof(struct sockaddr_in))
    {
      err = -EINVAL;
      goto err_out;
    }

  /* Read address. */

  rlen = read(modem->sockets.usrsockfd, &connaddr, sizeof(connaddr));
  if (rlen < 0)
    {
      err = -errno;
      dbg("Error reading %d bytes of request: ret=%d, errno=%d\n",
          sizeof(connaddr), (int)rlen, errno);
      goto err_out;
    }
  if (rlen != sizeof(connaddr))
    {
      err = -EMSGSIZE;
      goto err_out;
    }

  /* Check if address is IPv4 type. */

  if (connaddr.sin_family != AF_INET)
    {
      /* Invalid address family. */

      err = -EINVAL;
      goto err_out;
    }

  if (sock->type == SOCK_STREAM)
    {
      /* Mark sockets for TCP connect. */

      sock->connect.peeraddr = connaddr;
      sock->is_waiting_conn = true;
      sock->req = req->head;

      /* Wake-up main state machine. */

      __ubsocket_update_state(sock);
      __ubmodem_wake_waiting_state(modem, false);

      /* Report that request has been read and is being processed. */

      return __ubmodem_usrsock_send_response(modem, reqbuf, true, -EINPROGRESS);
    }
  else
    {
      /* Store new peer address for UDP */

      sock->connect.peeraddr = connaddr;
      sock->is_waiting_conn = false;

      /* Report that request has been read and is being processed. */

      return __ubmodem_usrsock_send_response(modem, reqbuf, false, 0);
    }

err_out:
  return __ubmodem_usrsock_send_response(modem, reqbuf, false, err);
}
