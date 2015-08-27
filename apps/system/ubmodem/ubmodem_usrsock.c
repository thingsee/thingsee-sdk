/****************************************************************************
 * apps/system/ubmodem/ubmodem_usrsock.c
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
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct ubmodem_usrsock_handler_s
{
  unsigned int hdrlen;
  int (*handler_fn)(struct ubmodem_s *modem, void *req);
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at_cmd_def_s urc_ATpUUSORD =
{
  .name         = "+UUSORD",
  .resp_format  =
  (const uint8_t[]){
    RESP_FMT_INT8,
    RESP_FMT_INT32
  },
  .resp_num     = 2,
};

static const struct at_cmd_def_s urc_ATpUUSOCL =
{
  .name         = "+UUSOCL",
  .resp_format  = (const uint8_t[]){ RESP_FMT_INT8 },
  .resp_num     = 1,
};

static const struct at_cmd_def_s cmd_ATpUSOCTL_get_lasterror =
{
  .name         = "+USOCTL",
  .resp_format  =
  (const uint8_t[]){
    RESP_FMT_INT8,
    RESP_FMT_INT8,
    RESP_FMT_INT32
  },
  .resp_num     = 3,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modem_socket_error_to_system_errno
 *
 * Description:
 *   Map modem sockets error code to system errno.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 ****************************************************************************/

static int modem_socket_error_to_system_errno(int modem_socket_error)
{
  switch (modem_socket_error)
    {
    case MODEM_SOCKET_NOERR:
      return 0;
    case MODEM_SOCKET_ENOENT:
      return ENOENT;
    case MODEM_SOCKET_EINTR:
      return EINTR;
    case MODEM_SOCKET_EIO:
      return EIO;
    case MODEM_SOCKET_EBADF:
      return EBADF;
    case MODEM_SOCKET_ECHILD:
      return ECHILD;
    case MODEM_SOCKET_EWOULDBLOCK:
      return EAGAIN;
    case MODEM_SOCKET_ENOMEM:
      return ENOMEM;
    case MODEM_SOCKET_EFAULT:
      return EFAULT;
    case MODEM_SOCKET_EINVAL:
      return EINVAL;
    case MODEM_SOCKET_EPIPE:
      return EPIPE;
    case MODEM_SOCKET_ENOSYS:
      return ENOSYS;
    case MODEM_SOCKET_ENOPROTOOPT:
      return ENOPROTOOPT;
    case MODEM_SOCKET_EADDRINUSE:
      return EADDRINUSE;
    case MODEM_SOCKET_ECONNABORTED:
      return ECONNABORTED;
    case MODEM_SOCKET_ECONNRESET:
      return ECONNRESET;
    case MODEM_SOCKET_ENOBUFS:
      return ENOBUFS;
    case MODEM_SOCKET_ENOTCONN:
      return ENOTCONN;
    case MODEM_SOCKET_ESHUTDOWN:
      return /*ESHUTDOWN*/ EPIPE;
    case MODEM_SOCKET_ETIMEDOUT:
      return ETIMEDOUT;
    case MODEM_SOCKET_EHOSTUNREACH:
      return EHOSTUNREACH;
    case MODEM_SOCKET_EINPROGRESS:
      return EINPROGRESS;
    default:
      return EPIPE;
    }
}

/****************************************************************************
 * Name: get_socket_error_handler
 *
 * Description:
 *   Handler for get sockets error command result.
 ****************************************************************************/

static void get_socket_error_handler(struct ubmodem_s *modem,
                                     const struct at_cmd_def_s *cmd,
                                     const struct at_resp_info_s *info,
                                     const uint8_t *resp_stream,
                                     size_t stream_len, void *priv)
{
  struct modem_socket_s *sock = priv;
  int32_t error_code;
  int8_t sockid;
  int8_t ctlreqid;
  int err;

  /*
   * Response handler for get sockets error, 'AT+USOCTL=<sock>,1'
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpUSOCTL_get_lasterror);

  if (sock->is_closed || sock->modem_sd < 0)
    {
      /* Socket closed, report EPIPE. */

      err = -EPIPE;
      goto out;
    }

  if (resp_status_is_error_or_timeout(info->status) ||
      info->status != RESP_STATUS_OK)
    {
      /* Error when reading error? */

      err = -EPIPE;
      goto out;
    }

  /* Read the sockets number. */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &sockid) ||
      sockid != sock->modem_sd)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Read the sockets control request identifier (should be 1). */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &ctlreqid) ||
      ctlreqid != 1)
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Read the error code. */

  if (!__ubmodem_stream_get_int32(&resp_stream, &stream_len, &error_code))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Map modem sockets error to system error code. */

  err = -modem_socket_error_to_system_errno(error_code);

out:
  /* Some sockets states require special handling after fetching error code. */

  switch (sock->state)
    {
      case MODEM_SOCKET_STATE_OPENING:
        /* Opening sockets failed, mark for freeing. */

        sock->is_freeing_pending = true;
        break;

      case MODEM_SOCKET_STATE_SENDING:
        if (err == -EAGAIN)
          {
            if (sock->type == SOCK_STREAM)
              {
                /* We need to start polling modem to figure out if modem has
                 * enough space in tx-buffer. */

                __ubmodem_handle_sendto_buffer_full(sock);
              }
            else
              {
                /* For UDP, consider packet lost. */

                err = sock->send.buflen;
              }
          }
        break;

      default:
        break;
    }

  /* Send error result. */

  (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, err);

  /* Mark work completed. */

  __ubsocket_work_done(sock);
}

/****************************************************************************
 * Name: urc_socket_read_handler
 *
 * Desciption:
 *   Handler for +UUSORD URC (data received by modem on sockets).
 ****************************************************************************/

static void urc_socket_read_handler(struct ubmodem_s *modem,
                                    const struct at_cmd_def_s *cmd,
                                    const struct at_resp_info_s *info,
                                    const uint8_t *resp_stream,
                                    size_t stream_len, void *priv)
{
  struct modem_socket_s *sock;
  int8_t sockid;
  int32_t datalen;

  /*
   * URC handler for 'data available for sockets on modem'
   */

  MODEM_DEBUGASSERT(modem, cmd == &urc_ATpUUSORD);
  MODEM_DEBUGASSERT(modem, info->status == RESP_STATUS_URC);

  /* Get sockid. */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &sockid))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Get datalen. */

  if (!__ubmodem_stream_get_int32(&resp_stream, &stream_len, &datalen))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Get sockets structure for this sockets. */

  sock = (void *)sq_peek(&modem->sockets.list);
  while (sock)
    {
      if (sock->modem_sd == sockid && !sock->is_closed)
        break;

      sock = (void *)sq_next(&sock->node);
    }

  if (!sock)
    {
      /* Can happen if we just issued +USOCL and sockets was marked closed. */

      return;
    }

  if (sock->is_closed)
    {
      /* Already closed! */

      return;
    }

  /* Update sockets with new available data length. */

  sock->recv.avail = datalen;

  /* Inform usrsock link about available data. */

  (void)__ubmodem_usrsock_send_event(sock, USRSOCK_EVENT_RECVFROM_AVAIL);

  /* Wake-up main state machine. */

  __ubmodem_wake_waiting_state(modem, false);
}

/****************************************************************************
 * Name: urc_socket_close_handler
 *
 * Desciption:
 *   Handler for +UUSOCL URC (sockets closed by modem).
 ****************************************************************************/

static void urc_socket_close_handler(struct ubmodem_s *modem,
                                     const struct at_cmd_def_s *cmd,
                                     const struct at_resp_info_s *info,
                                     const uint8_t *resp_stream,
                                     size_t stream_len, void *priv)
{
  struct modem_socket_s *sock;
  int8_t sockid;

  /*
   * URC handler for 'data available for sockets on modem'
   */

  MODEM_DEBUGASSERT(modem, cmd == &urc_ATpUUSOCL);
  MODEM_DEBUGASSERT(modem, info->status == RESP_STATUS_URC);

  /* Get sockid. */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &sockid))
    {
      MODEM_DEBUGASSERT(modem, false); /* Should not get here. */
      return;
    }

  /* Get sockets structure for this sockets. */

  sock = (void *)sq_peek(&modem->sockets.list);
  while (sock)
    {
      if (sock->modem_sd == sockid)
        break;

      sock = (void *)sq_next(&sock->node);
    }

  if (!sock)
    {
      return;
    }

  /* Cannot be already closed as when setting 'is_closed = true',
   * sock->modem_sd is set to -1.
   */

  MODEM_DEBUGASSERT(modem, !sock->is_closed);
  MODEM_DEBUGASSERT(modem, sock->modem_sd >= 0);

  /* Mark as closed. */

  sock->is_closed = true;
  sock->modem_sd = -1;

  /* Inform usrsock link about closure. */

  (void)__ubmodem_usrsock_send_event(sock, USRSOCK_EVENT_REMOTE_CLOSED);

  /* Continue processing. */

  __ubsocket_update_state(sock);

  /* Wake-up main state machine. */

  __ubmodem_wake_waiting_state(modem, false);
}

/****************************************************************************
 * Name: modem_read_rest_of_request
 *
 * Description:
 *   Read the additional part of request header trailing reading common header.
 *
 ****************************************************************************/

static int
modem_read_rest_of_request(struct ubmodem_s *modem,
                           const struct usrsock_request_common_s *common_hdr,
                           void *req, size_t reqsize)
{
  ssize_t rlen;
  int err;

  if (reqsize == sizeof(*common_hdr))
    return 0;

  rlen = read(modem->sockets.usrsockfd, (uint8_t *)req + sizeof(*common_hdr),
              reqsize - sizeof(*common_hdr));
  if (rlen < 0)
    {
      err = errno;
      dbg("Error reading %d bytes of request: ret=%d, errno=%d\n",
          reqsize - sizeof(*common_hdr), (int)rlen, errno);
      return -err;
    }
  if (rlen + sizeof(*common_hdr) != reqsize)
    {
      return -EMSGSIZE;
    }

  return 0;
}

/****************************************************************************
 * Name: modem_remove_xid_from_poll_off_list
 ****************************************************************************/

static void modem_remove_xid_from_poll_off_list(struct ubmodem_s *modem,
                                                uint8_t xid)
{
  int i;

  if (modem->sockets.poll_off_count == 0)
    return;

  for (i = 0; i < ARRAY_SIZE(modem->sockets.poll_off_list); i++)
    {
      /* Find right slot */

      if (modem->sockets.poll_off_list[i] != xid)
        continue;

      /* Clear */

      modem->sockets.poll_off_list[i] = 0;
      modem->sockets.poll_off_count--;

      return;
    }

  /* Already removed. */
}

/****************************************************************************
 * Name: modem_add_xid_to_poll_off_list
 ****************************************************************************/

static void modem_add_xid_to_poll_off_list(struct ubmodem_s *modem,
                                           uint8_t xid)
{
  int i;

  for (i = 0; i < ARRAY_SIZE(modem->sockets.poll_off_list); i++)
    {
      /* Find free slot */

      if (modem->sockets.poll_off_list[i] != 0)
        continue;

      /* Fill in xid */

      modem->sockets.poll_off_list[i] = xid;
      modem->sockets.poll_off_count++;

      return;
    }

  /* No free, this should not happen (max xid in flight is max sockets + 1). */

  DEBUGASSERT(false);
  return;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_get_socket_error
 *
 * Description:
 *   Send get sockets error AT command.
 ****************************************************************************/

void __ubmodem_get_socket_error(struct modem_socket_s *sock)
{
  struct ubmodem_s *modem = sock->modem;
  int err;

  err = __ubmodem_send_cmd(modem, &cmd_ATpUSOCTL_get_lasterror,
                           get_socket_error_handler, sock,
                           "=%d,1", sock->modem_sd);
  MODEM_DEBUGASSERT(modem, err == OK);
}

/****************************************************************************
 * Name: __ubmodem_usrsock_send_response
 *
 * Description:
 *   Send response for request
 *
 ****************************************************************************/

int __ubmodem_usrsock_send_response(struct ubmodem_s *modem,
                                    struct usrsock_request_common_s *req,
                                    bool inprogress,
                                    int result)
{
  struct usrsock_message_req_ack_s resp = {};
  ssize_t wlen;

  resp.xid = req->xid;
  resp.head.msgid = USRSOCK_MESSAGE_RESPONSE_ACK;
  resp.head.flags = 0;
  resp.head.flags = inprogress ? USRSOCK_MESSAGE_FLAG_REQ_IN_PROGRESS : 0;
  resp.result = result;

  wlen = write(modem->sockets.usrsockfd, &resp, sizeof(resp));
  if (wlen < 0)
    return -errno;
  if (wlen != sizeof(resp))
    return -ENOSPC;

  /* Request response sent, re-enable POLLIN. */

  modem_remove_xid_from_poll_off_list(modem, req->xid);

  /* Give trace output if active. */

  if (modem->active_events & UBMODEM_EVENT_FLAG_TRACE_USRSOCK)
    {
      int tmp[4] = {
        UBMODEM_TRACE_USRSOCK_RESP,
        req->xid,
        result,
        inprogress
      };

      __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_TRACE_USRSOCK,
                              tmp, sizeof(tmp));
    }

  return 0;
}

/****************************************************************************
 * Name: __ubmodem_usrsock_send_event
 *
 * Description:
 *   Send event to usrsock link
 *
 ****************************************************************************/

int __ubmodem_usrsock_send_event(struct modem_socket_s *sock, uint16_t events)
{
  struct ubmodem_s *modem = sock->modem;
  struct usrsock_message_socket_event_s event = {};
  ssize_t wlen;

  event.head.msgid = USRSOCK_MESSAGE_SOCKET_EVENT;
  event.head.flags = USRSOCK_MESSAGE_FLAG_EVENT;
  event.usockid = sock->usockid;
  event.events = events;

  wlen = write(sock->modem->sockets.usrsockfd, &event, sizeof(event));
  if (wlen < 0)
    return -errno;
  if (wlen != sizeof(event))
    return -ENOSPC;

  if (modem->active_events & UBMODEM_EVENT_FLAG_TRACE_USRSOCK)
    {
      int tmp[4] = {
        UBMODEM_TRACE_USRSOCK_EVENT,
        event.usockid,
        event.events,
        0
      };

      __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_TRACE_USRSOCK,
                              tmp, sizeof(tmp));
    }

  return 0;
}

/****************************************************************************
 * Name: __ubmodem_socket_get_by_usockid
 *
 * Description:
 *   Get sockets by usrsock link identifier 'usockid'.
 *
 ****************************************************************************/

struct modem_socket_s *
__ubmodem_socket_get_by_usockid(struct ubmodem_s *modem, int16_t usockid)
{
  struct modem_socket_s *item;

  item = (void *)sq_peek(&modem->sockets.list);
  while (item)
    {
      if (item->usockid == usockid)
        return item;

      item = (void *)sq_next(&item->node);
    }

  return NULL;
}

/****************************************************************************
 * Name: __ubsocket_update_state
 *
 * Description:
 *   Update sockets state based on pending operations.
 ****************************************************************************/

void __ubsocket_update_state(struct modem_socket_s *sock)
{
  bool make_first = false;

  /* If state is waiting, check if there new work to do. */

  if (sock->state != MODEM_SOCKET_STATE_INACTIVE)
    {
      return; /* Busy at other work. */
    }

  if (sock->is_freeing_pending)
    {
      /* Free inactive closed socket. */

      sock->state = MODEM_SOCKET_STATE_FREE;
      make_first = true;

      /* Free pending timer */

      if (sock->timerid >= 0)
        {
          __ubmodem_remove_timer(sock->modem, sock->timerid);
          sock->timerid = -1;
        }

      goto new_state;
    }

  if (sock->is_closing)
    {
      /* Closing sockets has been requested. */

      sock->state = MODEM_SOCKET_STATE_CLOSING;
      make_first = true;
      goto new_state;
    }

  if (sock->is_setgetsock_pending)
    {
      /* getsockopt/setsockopt has been requested. */

      sock->state = MODEM_SOCKET_STATE_SETGETSOCK;
      make_first = true;
      goto new_state;
    }

  if (sock->is_waiting_conn)
    {
      /* TCP connect for sockets has been requested. */

      sock->state = MODEM_SOCKET_STATE_CONNECTING;
      make_first = true;
      goto new_state;
    }

  if (sock->tx_buf_full_recheck)
    {
      /* TCP connect for sockets has been requested. */

      sock->state = MODEM_SOCKET_STATE_CHECKING_TXFULL;
      goto new_state;
    }

  if (sock->is_waiting_recv)
    {
      /* Data to be received. */

      sock->state = MODEM_SOCKET_STATE_RECEIVING;
      goto new_state;
    }

  if (sock->send.buflen > 0)
    {
      /* Data to be send. */

      sock->state = MODEM_SOCKET_STATE_SENDING;
      goto new_state;
    }

  return;

new_state:
  if (make_first)
    {
      /* Make sockets first of the list. */

      sq_rem(&sock->node, &sock->modem->sockets.list);
      sq_addfirst(&sock->node, &sock->modem->sockets.list);
    }
  else
    {
      /* Make sockets last of the list. */

      sq_rem(&sock->node, &sock->modem->sockets.list);
      sq_addlast(&sock->node, &sock->modem->sockets.list);
    }
}

/****************************************************************************
 * Name: __ubsocket_work_done
 *
 * Description:
 *   Mark current work item as complete and mark sockets inactive.
 ****************************************************************************/

void __ubsocket_work_done(struct modem_socket_s *sock)
{
  /* Clear current work state. */

  switch (sock->state)
    {
      case MODEM_SOCKET_STATE_SENDING:
        sock->send.buflen = 0;
        break;

      case MODEM_SOCKET_STATE_RECEIVING:
        sock->is_waiting_recv = false;
        break;

      case MODEM_SOCKET_STATE_CONNECTING:
        sock->is_waiting_conn = false;
        break;

      case MODEM_SOCKET_STATE_CHECKING_TXFULL:
        sock->tx_buf_full_recheck = false;
        break;

      case MODEM_SOCKET_STATE_SETGETSOCK:
        sock->is_setgetsock_pending = false;
        break;

      default:
        break;
    }

  /* Mark sockets as inactive. */

  sock->state = MODEM_SOCKET_STATE_INACTIVE;

  /* Check if there is more work. */

  __ubsocket_update_state(sock);

  /* Update main state machine. */

  __ubmodem_change_state(sock->modem, MODEM_STATE_WAITING);
}

/****************************************************************************
 * Name: __ubmodem_has_usrsock_work
 *
 * Description:
 *   Check if there is sockets with active work waiting.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 * Return value:
 *   true if there is sockets waiting to do work.
 *
 ****************************************************************************/

bool __ubmodem_has_usrsock_work(struct ubmodem_s *modem)
{
  struct modem_socket_s *sock;

  /* Walk queue and pick first item with work. */

  sock = (void *)sq_peek(&modem->sockets.list);
  while (sock)
    {
      /* If there is item with work, return true. */

      if (sock->state != MODEM_SOCKET_STATE_INACTIVE)
        return true;

      sock = (void *)sq_next(&sock->node);
    }

  return false;
}

/****************************************************************************
 * Name: __ubmodem_do_usrsock_work
 *
 * Description:
 *   Perform sockets work.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 ****************************************************************************/

void __ubmodem_do_usrsock_work(struct ubmodem_s *modem)
{
  struct modem_socket_s *sock;

  /* Walk queue and pick first item with work. */

  sock = (void *)sq_peek(&modem->sockets.list);
  while (sock)
    {
      if (sock->state != MODEM_SOCKET_STATE_INACTIVE)
        break;

      sock = (void *)sq_next(&sock->node);
    }

  /* No active items. */

  if (!sock)
    {
      __ubmodem_change_state(modem, MODEM_STATE_WAITING);
      return;
    }

  switch (sock->state)
    {
      case MODEM_SOCKET_STATE_OPENING:
        __ubmodem_open_socket(sock);
        break;

      case MODEM_SOCKET_STATE_CLOSING:
        __ubmodem_close_socket(sock);
        break;

      case MODEM_SOCKET_STATE_CONNECTING:
        __ubmodem_connect_socket(sock);
        break;

      case MODEM_SOCKET_STATE_SENDING:
        __ubmodem_sendto_socket(sock);
        break;

      case MODEM_SOCKET_STATE_RECEIVING:
        __ubmodem_recvfrom_socket(sock);
        break;

      case MODEM_SOCKET_STATE_CHECKING_TXFULL:
        __ubmodem_check_txfull_socket(sock);
        break;

      case MODEM_SOCKET_STATE_SETGETSOCK:
        __ubmodem_do_setgetsock_socket(sock);
        break;

      case MODEM_SOCKET_STATE_FREE:

        /* Remove item from list and free. */

        sq_rem(&sock->node, &modem->sockets.list);
        free(sock);

        /* Update main state machine. */

        __ubmodem_change_state(modem, MODEM_STATE_WAITING);

        break;

      case MODEM_SOCKET_STATE_INACTIVE:
        /*
         * In these states, sub-state machine is waiting result for AT command
         * or timeout event.
         */

        break;
      default:
        MODEM_DEBUGASSERT(modem, false);
        break;
    }
}

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_request
 *
 * Description:
 *   Handle pending request from /dev/usrsock
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 ****************************************************************************/

int __ubmodem_usrsock_handle_request(struct ubmodem_s *modem)
{
  static const struct ubmodem_usrsock_handler_s modem_usrsock_request_handlers[USRSOCK_REQUEST__MAX] =
    {
      [USRSOCK_REQUEST_SOCKET] =
        {
          sizeof(struct usrsock_request_socket_s),
          __ubmodem_usrsock_handle_socket_request,
        },
      [USRSOCK_REQUEST_CLOSE] =
        {
          sizeof(struct usrsock_request_close_s),
          __ubmodem_usrsock_handle_close_request,
        },
      [USRSOCK_REQUEST_CONNECT] =
        {
          sizeof(struct usrsock_request_connect_s),
          __ubmodem_usrsock_handle_connect_request,
        },
      [USRSOCK_REQUEST_SENDTO] =
        {
          sizeof(struct usrsock_request_sendto_s),
          __ubmodem_usrsock_handle_sendto_request,
        },
      [USRSOCK_REQUEST_RECVFROM] =
        {
          sizeof(struct usrsock_request_recvfrom_s),
          __ubmodem_usrsock_handle_recvfrom_request,
        },
      [USRSOCK_REQUEST_SETSOCKOPT] =
        {
          sizeof(struct usrsock_request_setsockopt_s),
          __ubmodem_usrsock_handle_setsockopt_request,
        },
      [USRSOCK_REQUEST_GETSOCKOPT] =
        {
          sizeof(struct usrsock_request_getsockopt_s),
          __ubmodem_usrsock_handle_getsockopt_request,
        },
    };
  const struct ubmodem_usrsock_handler_s *handlers =
      modem_usrsock_request_handlers;
  struct usrsock_request_common_s *common_hdr;
  uint8_t hdrbuf[16];
  int8_t reqid;
  ssize_t rlen;
  int err;

  common_hdr = (void *)hdrbuf;

  /* Read common header for request. */

  rlen = read(modem->sockets.usrsockfd, common_hdr, sizeof(*common_hdr));
  if (rlen < 0)
    {
      err = -errno;
      goto err_out;
    }
  if (rlen != sizeof(*common_hdr))
    {
      err = -EMSGSIZE;
      goto err_out;
    }

  /* Check if we support this request type. */

  reqid = common_hdr->reqid;
  if (reqid >= USRSOCK_REQUEST__MAX || !handlers[reqid].handler_fn)
    {
      dbg("Unknown request type: %d\n", reqid);

      /* Give error result. */

      err = -EOPNOTSUPP;
      goto err_out;
    }

  MODEM_DEBUGASSERT(modem, handlers[reqid].hdrlen < sizeof(hdrbuf));

  /* Read rest of the request. */

  rlen = modem_read_rest_of_request(modem, common_hdr, hdrbuf,
                                    handlers[reqid].hdrlen);
  if (rlen < 0)
    {
      err = -rlen;
      goto err_out;
    }

  /* Avoid waking on POLLIN from usrsock until current request has been
   * processed fully. */

  modem_add_xid_to_poll_off_list(modem, common_hdr->xid);

  /* Give trace output if active. */

  if (modem->active_events & UBMODEM_EVENT_FLAG_TRACE_USRSOCK)
    {
      int tmp[4] = {
        UBMODEM_TRACE_USRSOCK_REQ,
        common_hdr->reqid,
        common_hdr->xid,
        0,
      };

      __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_TRACE_USRSOCK,
                              tmp, sizeof(tmp));
    }

  /* Handle request. */

  return handlers[reqid].handler_fn(modem, hdrbuf);

err_out:
  return __ubmodem_usrsock_send_response(modem, common_hdr, false, err);
}

/****************************************************************************
 * Name: __ubmodem_usrsock_initialize
 *
 * Description:
 *   Initialize /dev/usrsock interface.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *   addr  : Current IP4-address for GPRS connection.
 *
 ****************************************************************************/

void __ubmodem_usrsock_initialize(struct ubmodem_s *modem,
                                  const struct ubmodem_event_ip_address_s *ipcfg)
{
  modem->sockets.ipcfg = *ipcfg;
  modem->sockets.poll_off_count = 0;

  /* Register sockets URC handlers if not already. */

  if (!modem->socket_urcs_registered)
    {
      __ubparser_register_response_handler(&modem->parser, &urc_ATpUUSORD,
                                           urc_socket_read_handler,
                                           modem, true);
      __ubparser_register_response_handler(&modem->parser, &urc_ATpUUSOCL,
                                           urc_socket_close_handler,
                                           modem, true);
      modem->socket_urcs_registered = true;
    }

  /* Open /dev/usrsock interface */

  MODEM_DEBUGASSERT(modem, modem->sockets.usrsockfd == -1);

  modem->sockets.usrsockfd = open("/dev/usrsock", O_RDWR);
  if (modem->sockets.usrsockfd < 0)
    {
      dbg("failed to open /dev/usrsock\n");
      return;
    }
}

/****************************************************************************
 * Name: __ubmodem_usrsock_uninitialize
 *
 * Description:
 *   Close /dev/usrsock interface; Remove and free all sockets.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 ****************************************************************************/

void __ubmodem_usrsock_uninitialize(struct ubmodem_s *modem)
{
  struct modem_socket_s *sock, *next;

  /*
   * Called when closing GPRS connection. Kill and remove all connections.
   */

  /* Close usrsock interface. */

  if (modem->sockets.usrsockfd >= 0)
    {
      close(modem->sockets.usrsockfd);
      modem->sockets.usrsockfd = -1;
    }

  /* Unregister URCs. */

  if (modem->socket_urcs_registered)
    {
      __ubparser_unregister_response_handler(&modem->parser, urc_ATpUUSOCL.name);
      __ubparser_unregister_response_handler(&modem->parser, urc_ATpUUSORD.name);
      modem->socket_urcs_registered = false;
    }

  /* Walk-through all open sockets and kill-em. */

  for (sock = (void *)sq_peek(&modem->sockets.list); sock; sock = next)
    {
      next = (void *)sq_next(&sock->node);

      /* Remove item from list. */

      sq_rem(&sock->node, &modem->sockets.list);

      if (sock->timerid != -1)
        {
          __ubmodem_remove_timer(modem, sock->timerid);
        }

      free(sock);
    }

  /* Clean-up state. */

  modem->sockets.poll_off_count = 0;
}

