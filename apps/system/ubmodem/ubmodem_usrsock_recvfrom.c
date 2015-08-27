/****************************************************************************
 * apps/system/ubmodem/ubmodem_usrsock_recvfrom.c
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

static const struct at_cmd_def_s cmd_ATpUSORD =
{
  .name         = "+USORD",
  .resp_format  =
  (const uint8_t[]){
    RESP_FMT_INT8,
    RESP_FMT_DATALEN,
    RESP_FMT_QUOTED_DATABUF,
  },
  .resp_num     = 3,
  .timeout_dsec = MODEM_CMD_SOCKET_TIMEOUT,
};

static const struct at_cmd_def_s cmd_ATpUSORF =
{
  .name         = "+USORF",
  .resp_format  =
  (const uint8_t[]){
    RESP_FMT_INT8,
    RESP_FMT_QUOTED_STRING,
    RESP_FMT_INT32,
    RESP_FMT_DATALEN,
    RESP_FMT_QUOTED_DATABUF,
  },
  .resp_num     = 5,
  .timeout_dsec = MODEM_CMD_SOCKET_TIMEOUT,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: socket_recvfrom_handler
 ****************************************************************************/

static void socket_recvfrom_handler(struct ubmodem_s *modem,
                                    const struct at_cmd_def_s *cmd,
                                    const struct at_resp_info_s *info,
                                    const uint8_t *resp_stream,
                                    size_t stream_len, void *priv)
{
  struct usrsock_message_datareq_ack_s resp = {};
  struct modem_socket_s *sock = priv;
  uint16_t buflen;
  int8_t sockid;
  const char *buf;
  size_t wlen;
  struct sockaddr_in addr = {};

  /*
   * Response handler for +USORF sockets data read.
   */

  MODEM_DEBUGASSERT(modem, cmd == ((sock->type == SOCK_DGRAM) ?
                                   &cmd_ATpUSORF : &cmd_ATpUSORD));

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
      dbg("%s error, status: %d\n", cmd->name, info->status);

      /* Reading failed? This should not happen as we read only if modem
       * reported that there is data to be read. */

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);
      __ubsocket_work_done(sock);

      return;
    }

  /* Read the sockets number. */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &sockid))
    {
      /* Invalid data from modem? Might be missing character from "+USORF:"
       * header. */

      dbg("Invalid %s response, %s, stream_len: %d\n", cmd->name, "no sockid",
          stream_len);

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);
      __ubsocket_work_done(sock);

      return;
    }

  if (sockid != sock->modem_sd)
    {
      /* Some other error, report generic error. */

      dbg("Invalid %s response, sockid mismatch, got: %d, expect: %d\n",
          cmd->name, sockid, sock->modem_sd);

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);
      __ubsocket_work_done(sock);
      return;
    }

  if (sock->type == SOCK_DGRAM)
    {
      int32_t port;
      const char *addrstr;
      uint16_t addrstrlen;

      /* Read address */

      if (!__ubmodem_stream_get_string(&resp_stream, &stream_len, &addrstr,
                                       &addrstrlen))
        {
          dbg("Invalid %s response, %s, stream_len: %d\n", cmd->name, "no address",
              stream_len);

          (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);
          __ubsocket_work_done(sock);

          return;
        }

      /* Read port */

      if (!__ubmodem_stream_get_int32(&resp_stream, &stream_len, &port))
        {
          dbg("Invalid %s response, %s, stream_len: %d\n", cmd->name, "no port",
              stream_len);

          (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);
          __ubsocket_work_done(sock);

          return;
        }

      /* Prepare peer address for response */

      addr.sin_addr.s_addr = inet_addr(addrstr);
      addr.sin_port = htons(port);
      addr.sin_family = AF_INET;
    }
  else
    {
      /* Prepare peer address for response */

      addr = sock->connect.peeraddr;
    }

  /* Read data buffer */

  if (!__ubmodem_stream_get_string(&resp_stream, &stream_len, &buf, &buflen))
    {
      dbg("Invalid %s response, %s, stream_len: %d\n", cmd->name, "no buflen",
          stream_len);

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);
      __ubsocket_work_done(sock);

      return;
    }

  /* Stream now has data, feed data into usrsock link. */

  resp.reqack.xid = sock->req.xid;
  resp.reqack.head.msgid = USRSOCK_MESSAGE_RESPONSE_DATA_ACK;
  resp.reqack.head.flags = 0;
  resp.reqack.result = buflen;
  resp.valuelen_nontrunc = sizeof(struct sockaddr_in);
  resp.valuelen = resp.valuelen_nontrunc;
  if (resp.valuelen > sock->recv.max_addrlen)
    {
      resp.valuelen = sock->recv.max_addrlen;
    }

  /* Give debug trace if active. */

  if (modem->active_events & UBMODEM_EVENT_FLAG_TRACE_USRSOCK)
    {
      int tmp[4] = {
        UBMODEM_TRACE_USRSOCK_DATARESP,
        resp.reqack.xid,
        resp.reqack.result,
        resp.valuelen
      };

      __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_TRACE_USRSOCK,
                              tmp, sizeof(tmp));
    }

  /* Send response header. */

  wlen = write(modem->sockets.usrsockfd, &resp, sizeof(resp));
  MODEM_DEBUGASSERT(modem, wlen == sizeof(resp));

  if (resp.valuelen > 0)
    {
      /* Send address. */

      wlen = write(modem->sockets.usrsockfd, &addr, resp.valuelen);
      MODEM_DEBUGASSERT(modem, wlen == resp.valuelen);
    }

  /* Send data. */

  wlen = write(modem->sockets.usrsockfd, buf, resp.reqack.result);
  MODEM_DEBUGASSERT(modem, wlen == resp.reqack.result);

  /* Done reading. */

  __ubsocket_work_done(sock);
}

/****************************************************************************
 * Public Internal Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_recvfrom_socket
 *
 * Description:
 *   Receive data from sockets
 ****************************************************************************/

void __ubmodem_recvfrom_socket(struct modem_socket_s *sock)
{
  struct ubmodem_s *modem = sock->modem;
  int err;
  size_t inlen;
  size_t max_stream_len;
  size_t recvlen;

  if (sock->is_closed)
    {
      /* Socket closed! */

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);
      __ubsocket_work_done(sock);
      return;
    }

  inlen = sock->recv.avail;

  if (inlen > sock->recv.max_buflen)
    inlen = sock->recv.max_buflen;

  if (sock->type == SOCK_DGRAM)
    {
      max_stream_len = __ubmodem_stream_max_readbuf_len(&cmd_ATpUSORF);
      max_stream_len -= INET_ADDRSTRLEN;
    }
  else
    {
      max_stream_len = __ubmodem_stream_max_readbuf_len(&cmd_ATpUSORD);
    }

  MODEM_DEBUGASSERT(modem, inlen > 0); /* sockets state machine should not allow
                                        * recv if input buffer has no space
                                        * left. */

  recvlen = max_stream_len;
  if (recvlen > MODEM_MAX_BINARY_SOCKET_READ_BYTES)
    recvlen = MODEM_MAX_BINARY_SOCKET_READ_BYTES;
  if (recvlen > inlen)
    recvlen = inlen;

  if (sock->type == SOCK_DGRAM)
    {
      /* Read data and peer address from modem. */

      err = __ubmodem_send_cmd(modem, &cmd_ATpUSORF, socket_recvfrom_handler,
                               sock, "=%d,%d", sock->modem_sd, recvlen);
      MODEM_DEBUGASSERT(modem, err == OK);
    }
  else
    {
      /* Read data from modem. */

      err = __ubmodem_send_cmd(modem, &cmd_ATpUSORD, socket_recvfrom_handler,
                               sock, "=%d,%d", sock->modem_sd, recvlen);
      MODEM_DEBUGASSERT(modem, err == OK);
    }
}

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_recvfrom_request
 *
 * Description:
 *   Handle recvfrom request for sockets
 ****************************************************************************/

int __ubmodem_usrsock_handle_recvfrom_request(struct ubmodem_s *modem,
                                              void *reqbuf)
{
  struct usrsock_request_recvfrom_s *req = reqbuf;
  struct modem_socket_s *sock;
  int err;

  /* Get sockets for request. */

  sock = __ubmodem_socket_get_by_usockid(modem, req->usockid);
  if (!sock)
    {
      /* Give error result. */

      err = -EBADF;
      goto err_out;
    }

  MODEM_DEBUGASSERT(modem, sock->type == SOCK_DGRAM || sock->type == SOCK_STREAM);

  /* Check if sockets is closed. */

  if (sock->is_closed || sock->modem_sd < 0)
    {
      err = -EPIPE;
      goto err_out;
    }

  if (sock->recv.avail == 0)
    {
      /* No data available at modem. */

      err = -EAGAIN;
      goto err_out;
    }

  /* TODO: Check for pending blocking operation (TCP connect) */

  /* Prepare to receive data from modem. */

  sock->req = req->head;
  sock->is_waiting_recv = true;
  sock->recv.max_addrlen = req->max_addrlen;
  sock->recv.max_buflen = req->max_buflen;

  __ubsocket_update_state(sock);
  __ubmodem_wake_waiting_state(modem, false);

  /* Report that request has been read and processing has started. */

  return __ubmodem_usrsock_send_response(modem, reqbuf, true, -EINPROGRESS);

err_out:
  return __ubmodem_usrsock_send_response(modem, reqbuf, false, err);
}
