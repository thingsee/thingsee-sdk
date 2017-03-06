/****************************************************************************
 * apps/system/ubmodem/ubmodem_usrsock_socket.c
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
#include "ubmodem_hw.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at_cmd_def_s cmd_ATpUSOCR =
{
  .name         = "+USOCR",
  .resp_format  = (const uint8_t[]){ RESP_FMT_INT8 },
  .resp_num     = 1,
  .timeout_dsec = MODEM_CMD_SOCKET_TIMEOUT,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: open_socket_handler
 *
 * Description:
 *   Handler for sockets open command result.
 ****************************************************************************/

static void open_socket_handler(struct ubmodem_s *modem,
                                const struct at_cmd_def_s *cmd,
                                const struct at_resp_info_s *info,
                                const uint8_t *resp_stream,
                                size_t stream_len, void *priv)
{
  struct modem_socket_s *sock = priv;
  int8_t sockid;

  /*
   * Response handler for open TCP/UDP sockets, 'AT+USOCR=<socket_type>'
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpUSOCR);

  if (info->status == RESP_STATUS_ERROR)
    {
      /* Not CME error, fetch sockets error. */

      __ubmodem_get_socket_error(sock);
      return;
    }

  if (info->status == RESP_STATUS_TIMEOUT)
    {
      int ret;

      /* Timeout for create socket command can mean only one thing: Modem
       * is stuck in bad state. */

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -ENETDOWN);

      /* Mark socket for removal. */

      sock->is_freeing_pending = true;
      __ubsocket_work_done(sock);

      /* Prepare task for bringing modem alive from stuck state. */

      ret = __ubmodem_recover_stuck_hardware(modem);
      MODEM_DEBUGASSERT(modem, ret != ERROR);
      return;
    }

  if (resp_status_is_error_or_timeout(info->status) ||
      info->status != RESP_STATUS_OK)
    {
      /* CME error or timeout when opening sockets. */

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -ENOBUFS);

      /* Mark sockets for removal. */

      sock->is_freeing_pending = true;
      __ubsocket_work_done(sock);
      return;
    }

  /* Read the sockets number. */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &sockid))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* TCP/UDP sockets opened. */

  sock->modem_sd = sockid;

  /* Send result for usrsock request, give the allocated usockid as ID for
   * this sockets link. */

  (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, sock->usockid);

  /* If this is UDP sockets, send sendto-ready event. */

  if (sock->type == SOCK_DGRAM)
    {
      (void)__ubmodem_usrsock_send_event(sock, USRSOCK_EVENT_SENDTO_READY);
    }

  /* Mark work completed. */

  __ubsocket_work_done(sock);
}

/****************************************************************************
 * Name: modem_socket_get_num_active
 *
 * Description:
 *   Get number of active sockets (sockets that open at modem side).
 *
 ****************************************************************************/

static int modem_socket_get_num_active(struct ubmodem_s *modem)
{
  struct modem_socket_s *item;
  int num = 0;

  item = (void *)sq_peek(&modem->sockets.list);
  while (item)
    {
      num += !(item->state == MODEM_SOCKET_STATE_FREE);
      item = (void *)sq_next(&item->node);
    }

  return num;
}

/****************************************************************************
 * Public Internal Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubmodem_open_socket
 *
 * Description:
 *   Send open sockets AT command.
 ****************************************************************************/

void __ubmodem_open_socket(struct modem_socket_s *sock)
{
  enum modem_socket_type_e type = sock->type == SOCK_STREAM ?
      MODEM_SOCKET_TYPE_TCP : MODEM_SOCKET_TYPE_UDP;
  struct ubmodem_s *modem = sock->modem;
  int err;

  /* Attempt to open internal TCP/IP stack sockets. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpUSOCR, open_socket_handler, sock,
                           "=%d", type);
  MODEM_DEBUGASSERT(modem, err == OK);
}

/****************************************************************************
 * Name: ubmodem_usrsock_handle_socket_request
 *
 * Description:
 *   Handle request for new sockets
 *
 ****************************************************************************/

int __ubmodem_usrsock_handle_socket_request(struct ubmodem_s *modem,
                                            void *reqbuf)
{
  struct usrsock_request_socket_s *req = reqbuf;
  struct modem_socket_s *sock;
  struct modem_socket_s *item;
  int16_t usockid;
  int err;

  /* Check if sockets is of supported type (IPv4). */

  if (req->domain != AF_INET)
    {
      /* Give error result. */

      err = -EAFNOSUPPORT;
      goto err_out;
    }

  /* Check if sockets is of supported type (TCP or UDP). */

  if (req->type != SOCK_STREAM && req->type != SOCK_DGRAM)
    {
      /* Give error result. */

      err = -EPROTONOSUPPORT;
      goto err_out;
    }

  /* Check number open sockets. */

  if (modem_socket_get_num_active(modem) >= MODEM_MAX_SOCKETS_OPEN)
    {
      /* Give error result. */

      err = -ENOBUFS;
      goto err_out;
    }

  /* Allocate new sockets structure and add to sockets buffer. */

  sock = calloc(1, sizeof(*sock));
  if (!sock)
    {
      /* Give error result. */

      err = -ENOMEM;
      goto err_out;
    }

  /* Pick unused sockets id. */

  do
    {
      usockid = (modem->sockets.usockid_counter++ & 0x7ff) + 1;
      if (usockid <= 0 || usockid > INT16_MAX)
        {
          continue;
        }

      /* Walk queue and find sockets by ID. */

      item = (void *)sq_peek(&modem->sockets.list);
      while (item)
        {
          if (item->usockid == usockid)
            break;

          item = (void *)sq_next(&item->node);
        }

      /* If no socket using this ID, pick it for new socket. */

      if (item == NULL)
        {
          break;
        }
    }
  while (true);

  /* Setup structure. */

  sock->state = MODEM_SOCKET_STATE_OPENING;
  sock->modem = modem;
  sock->modem_sd = -1;
  sock->type = req->type;
  sock->usockid = usockid;
  sock->timerid = -1;
  sock->req = req->head;

  /* Add structure to list. */

  sq_addfirst(&sock->node, &modem->sockets.list);

  /* New socket, inform power-management of low activity increase. */

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_LOW, true);

  /* Wake-up main state machine. */

  __ubmodem_wake_waiting_state(modem, false);

  /* Report that request has been read and is being processed. */

  return __ubmodem_usrsock_send_response(modem, reqbuf, true, -EINPROGRESS);

err_out:
  return __ubmodem_usrsock_send_response(modem, reqbuf, false, err);
}
