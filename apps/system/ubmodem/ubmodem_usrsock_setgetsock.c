/****************************************************************************
 * apps/system/ubmodem/ubmodem_usrsock_setgetsock.c
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

static const struct at_cmd_def_s cmd_ATpUSOSO =
{
  .name         = "+USOSO",
  .resp_format  = NULL,
  .resp_num     = 0,
  .timeout_dsec = MODEM_CMD_SOCKET_TIMEOUT,
};

static const struct at_cmd_def_s cmd_ATpUSOGO =
{
  .name         = "+USOGO",
  .resp_format  =
  (const uint8_t[]){
    RESP_FMT_INT32,
    RESP_FMT_INT32,
  },
  .resp_num     = 2,
  .timeout_dsec = MODEM_CMD_SOCKET_TIMEOUT,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: setsockopt_handler
 ****************************************************************************/

static void setsockopt_handler(struct ubmodem_s *modem,
                               const struct at_cmd_def_s *cmd,
                               const struct at_resp_info_s *info,
                               const uint8_t *resp_stream,
                               size_t stream_len, void *priv)
{
  struct modem_socket_s *sock = priv;

  /*
   * Response handler for +USOSO.
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpUSOSO);

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
 * Name: getsockopt_handler
 ****************************************************************************/

static void getsockopt_handler(struct ubmodem_s *modem,
                               const struct at_cmd_def_s *cmd,
                               const struct at_resp_info_s *info,
                               const uint8_t *resp_stream,
                               size_t stream_len, void *priv)
{
  struct usrsock_message_datareq_ack_s resp = {};
  struct modem_socket_s *sock = priv;
  int32_t optval[2];
  struct linger lingerval = {};
  int intboolval = 0;
  void *valbuf;
  size_t valsize;
  ssize_t wlen;

  /*
   * Response handler for +USOGO.
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpUSOGO);

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

  /* Read first optval. */

  if (!__ubmodem_stream_get_int32(&resp_stream, &stream_len, &optval[0]))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  if (sock->setgetsock.option == MODEM_SO_LINGER)
    {
      /* Read second optval. */

      if (!__ubmodem_stream_get_int32(&resp_stream, &stream_len, &optval[1]))
        {
          MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
          return;
        }

      lingerval.l_onoff = !!optval[0];
      lingerval.l_linger = optval[1] / 1000;
      valbuf = &lingerval;
      valsize = sizeof(lingerval);
    }
  else
    {
      intboolval = !!optval[0];
      valbuf = &intboolval;
      valsize = sizeof(intboolval);
    }

  DEBUGASSERT(valsize == sock->setgetsock.valuesize);

  /* Feed result to usrsock link. */

  resp.reqack.xid = sock->req.xid;
  resp.reqack.head.msgid = USRSOCK_MESSAGE_RESPONSE_DATA_ACK;
  resp.reqack.head.flags = 0;
  resp.reqack.result = OK;
  resp.valuelen_nontrunc = valsize;
  resp.valuelen = valsize;

  /* Give debug trace if active. */

  if (modem->active_events & UBMODEM_EVENT_FLAG_TRACE_USRSOCK)
    {
      int tmp[4] = {
        UBMODEM_TRACE_USRSOCK_DATARESP,
        resp.reqack.xid,
        resp.reqack.result,
        valsize
      };

      __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_TRACE_USRSOCK,
                              tmp, sizeof(tmp));
    }

  /* Send response header. */

  wlen = write(modem->sockets.usrsockfd, &resp, sizeof(resp));
  MODEM_DEBUGASSERT(modem, wlen == sizeof(resp));

  /* Send value. */

  wlen = write(modem->sockets.usrsockfd, valbuf, resp.valuelen);
  MODEM_DEBUGASSERT(modem, wlen == resp.valuelen);

  /* Mark work as done. */

  __ubsocket_work_done(sock);
}

/****************************************************************************
 * Name: modem_usrsock_handle_setgetsockopt_request
 *
 * Description:
 *   Handle setsockopt/getsockopt request for sockets
 *
 ****************************************************************************/

static int
modem_usrsock_handle_setgetsockopt_request(struct ubmodem_s *modem,
                                           struct usrsock_request_common_s *head,
                                           int req_usockid, int req_level,
                                           int req_option, int req_valuelen,
                                           bool set)
{
  struct modem_socket_s *sock;
  int err;
  size_t valuesize;
  int modem_option;
  int modem_level;

  /* Get sockets for request. */

  sock = __ubmodem_socket_get_by_usockid(modem, req_usockid);
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

  /* NuttX is only supporting socket level options. */

  if (req_level != SOL_SOCKET)
    {
      err = -ENOPROTOOPT;
      goto err_out;
    }

  modem_level = MODEM_SOL_SOCKET;

  /* Map NuttX options to modem. */

  switch (req_option)
    {
      case SO_BROADCAST:
        valuesize = sizeof(int);
        modem_option = MODEM_SO_BOARDCAST;
        break;

      case SO_REUSEADDR:
        valuesize = sizeof(int);
        modem_option = MODEM_SO_REUSEADDR;
        break;

      case SO_KEEPALIVE:
        valuesize = sizeof(int);
        modem_option = MODEM_SO_KEEPALIVE;
        break;

      case SO_LINGER:
        valuesize = sizeof(struct linger);
        modem_option = MODEM_SO_LINGER;
        break;

      default:
        err = -ENOPROTOOPT;
        goto err_out;
    }

  /* Input value large enough? */

  if (req_valuelen < valuesize)
    {
      err = -EINVAL;
      goto err_out;
    }

  /* Mark pending getsock/setsock operation. */

  sock->req = *head;
  sock->is_socket_config_pending = true;
  sock->setgetsock.option = modem_option;
  sock->setgetsock.level = modem_level;
  sock->setgetsock.valuesize = valuesize;

  /* Update socket state. */

  __ubsocket_update_state(sock);
  __ubmodem_wake_waiting_state(modem, false);

  if (!set)
    {
      /* Inform usrsock link that request was fully read. */

      (void)__ubmodem_usrsock_send_response(modem, head, true,
                                            -EINPROGRESS);
    }

  return 0;

err_out:
  return __ubmodem_usrsock_send_response(modem, head, false, err);
}

/****************************************************************************
 * Public Internal Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_setsockopt_socket
 *
 * Description:
 *   Prepare setting socket option.
 ****************************************************************************/

void __ubmodem_setsockopt_socket(struct modem_socket_s *sock)
{
  struct ubmodem_s *modem = sock->modem;
  int err;
  char buf[sizeof(struct linger)];
  size_t rlen;

  /* Read value. */

  rlen = read(modem->sockets.usrsockfd, &buf, sock->setgetsock.valuesize);
  if (rlen < 0)
    {
      err = -errno;
      dbg("Error reading %d bytes of request: ret=%d, errno=%d\n",
          sock->setgetsock.valuesize, (int)rlen, errno);
      goto err_out;
    }
  if (rlen != sock->setgetsock.valuesize)
    {
      err = -EMSGSIZE;
      goto err_out;
    }

  /* Inform usrsock link that request was fully read. */

  (void)__ubmodem_usrsock_send_response(modem, &sock->req, true, -EINPROGRESS);

  /* SO_LINGER has different value type than the rest. */

  if (sock->setgetsock.option == MODEM_SO_LINGER)
    {
      struct linger *li = (void *)buf;
      int64_t linger_msec;

      /* Input is integer boolean flag and delay value. */

      linger_msec = li->l_linger * 1000;

      if (linger_msec >= INT16_MAX)
        linger_msec = INT16_MAX;
      if (linger_msec < 0)
        linger_msec = 0;

      if (!li->l_onoff)
        linger_msec = 0;

      err = __ubmodem_send_cmd(modem, &cmd_ATpUSOSO, setsockopt_handler,
                               sock, "=%d,%d,%d,%d,%d", sock->modem_sd,
                               sock->setgetsock.level,
                               sock->setgetsock.option,
                               !!li->l_onoff, (int)linger_msec);
      MODEM_DEBUGASSERT(modem, err == OK);
      return;
    }
  else
    {
      int *onoff = (void *)buf;
      int setting;

      /* Input is integer boolean flag. */

      setting = !!*onoff;

      err = __ubmodem_send_cmd(modem, &cmd_ATpUSOSO, setsockopt_handler,
                               sock, "=%d,%d,%d,%d", sock->modem_sd,
                               sock->setgetsock.level,
                               sock->setgetsock.option,
                               setting);
      MODEM_DEBUGASSERT(modem, err == OK);
      return;
    }

err_out:
  (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, err);
}

/****************************************************************************
 * Name: __ubmodem_getsockopt_socket
 *
 * Description:
 *   Prepare getting socket option.
 ****************************************************************************/

void __ubmodem_getsockopt_socket(struct modem_socket_s *sock)
{
  struct ubmodem_s *modem = sock->modem;
  int err;

  err = __ubmodem_send_cmd(modem, &cmd_ATpUSOGO, getsockopt_handler,
                           sock, "=%d,%d,%d", sock->modem_sd,
                           sock->setgetsock.level,
                           sock->setgetsock.option);
  MODEM_DEBUGASSERT(modem, err == OK);
}

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_setsockopt_request
 *
 * Description:
 *   Handle setsockopt request for sockets
 *
 ****************************************************************************/

int __ubmodem_usrsock_handle_setsockopt_request(struct ubmodem_s *modem,
                                                void *reqbuf)
{
  struct usrsock_request_setsockopt_s *req = reqbuf;

  return modem_usrsock_handle_setgetsockopt_request(modem, &req->head,
                                                    req->usockid, req->level,
                                                    req->option, req->valuelen,
                                                    true);
}

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_getsockopt_request
 *
 * Description:
 *   Handle getsockopt request for sockets
 *
 ****************************************************************************/

int __ubmodem_usrsock_handle_getsockopt_request(struct ubmodem_s *modem,
                                                void *reqbuf)
{
  struct usrsock_request_getsockopt_s *req = reqbuf;

  return modem_usrsock_handle_setgetsockopt_request(modem, &req->head,
                                                    req->usockid, req->level,
                                                    req->option,
                                                    req->max_valuelen, false);
}
