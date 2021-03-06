/****************************************************************************
 * apps/system/ubmodem/ubmodem_usrsock_bind.c
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

static const struct at_cmd_def_s cmd_ATpUSOLI =
{
  .name         = "+USOLI",
  .resp_format  = NULL,
  .resp_num     = 0,
  .timeout_dsec = MODEM_CMD_SOCKET_TIMEOUT,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setsockopt_handler
 ****************************************************************************/

static void bind_udp_handler(struct ubmodem_s *modem,
                             const struct at_cmd_def_s *cmd,
                             const struct at_resp_info_s *info,
                             const uint8_t *resp_stream,
                             size_t stream_len, void *priv)
{
  struct modem_socket_s *sock = priv;

  /*
   * Response handler for +USOLI.
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpUSOLI);

  if (sock->is_closed || sock->modem_sd < 0)
    {
      /* Socket has been closed, report error. */

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EBADF);

      __ubsocket_work_done(sock);

      return;
    }

  if (resp_status_is_error_or_timeout(info->status) ||
      info->status != RESP_STATUS_OK)
    {
      /* Invalid socket value, report generic error. */

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EINVAL);

      __ubsocket_work_done(sock);

      return;
    }

  /* Response to request with success. */

  (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, 0);

  /* Mark work as done. */

  __ubsocket_work_done(sock);
}

/****************************************************************************
 * Public Internal Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_bind_socket
 *
 * Description:
 *   Prepare binding socket.
 ****************************************************************************/

void __ubmodem_bind_socket(struct modem_socket_s *sock)
{
  struct ubmodem_s *modem = sock->modem;
  int err;

  err = __ubmodem_send_cmd(modem, &cmd_ATpUSOLI, bind_udp_handler,
                           sock, "=%d,%d", sock->modem_sd,
                           ntohs(sock->bind.addr.sin_port));
  MODEM_DEBUGASSERT(modem, err == OK);
}

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_bind_request
 *
 * Description:
 *   Handle bind request for sockets
 *
 ****************************************************************************/

int __ubmodem_usrsock_handle_bind_request(struct ubmodem_s *modem,
                                          void *reqbuf)
{
  struct usrsock_request_bind_s *req = reqbuf;
  struct modem_socket_s *sock;
  size_t rlen;
  int err;
  struct sockaddr_in addr = {};
  bool ignore = false;

  /* Get sockets for request. */

  sock = __ubmodem_socket_get_by_usockid(modem, req->usockid);
  if (!sock)
    {
      /* Give error result. */

      err = -EBADF;
      goto err_out;
    }

  /* Check if sockets is closed. */

  if (sock->is_closed || sock->modem_sd < 0)
    {
      err = -EBADF;
      goto err_out;
    }

  /* Input value large enough? */

  if (req->addrlen < sizeof(addr))
    {
      err = -EINVAL;
      goto err_out;
    }

  /* Read address. */

  rlen = read(modem->sockets.usrsockfd, &addr, sizeof(addr));
  if (rlen < 0)
    {
      err = -errno;
      dbg("Error reading %d bytes of request: ret=%d, errno=%d\n",
          sizeof(addr), (int)rlen, errno);
      goto err_out;
    }
  if (rlen != sizeof(addr))
    {
      err = -EMSGSIZE;
      goto err_out;
    }

  if (req->addrlen > sizeof(addr))
    {
      /* Skip extra bytes of address buffer */

      rlen = lseek(modem->sockets.usrsockfd,
                   req->addrlen - sizeof(addr), SEEK_CUR);
      if (rlen < 0)
        {
          err = -errno;
          goto err_out;
        }
    }

  if (addr.sin_family != AF_INET)
    {
      err = -EINVAL;
      goto err_out;
    }

  if (sock->type == SOCK_STREAM)
    {
      /* For now, silently ignore bind on TCP sockets. */

      ignore = true;
    }
  else if (sock->type == SOCK_DGRAM && ntohs(addr.sin_port) == 0)
    {
      /* Do nothing if UDP port is 'any'. */

      ignore = true;
    }

  if (ignore)
    {
      sock->is_socket_config_pending = false;

      /* Report that request succeeded. */

      (void)__ubmodem_usrsock_send_response(modem, reqbuf, false, 0);
    }
  else
    {
      /* Mark pending getsock/setsock operation. */

      sock->req = req->head;
      sock->is_socket_config_pending = true;
      sock->bind.addr = addr;

      /* Update socket state. */

      __ubsocket_update_state(sock);
      __ubmodem_wake_waiting_state(modem, false);

      /* Inform usrsock link that request was fully read. */

      (void)__ubmodem_usrsock_send_response(modem, reqbuf, true, -EINPROGRESS);
    }

  return 0;

err_out:
  return __ubmodem_usrsock_send_response(modem, reqbuf, false, err);
}
