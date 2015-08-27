/****************************************************************************
 * apps/system/ubmodem/ubmodem_usrsock_sendto.c
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

static const struct at_cmd_def_s cmd_ATpUSOST_binary =
{
  .name         = "+USOST",
  .resp_format  = NULL,
  .resp_num     = 0,
  .timeout_dsec = MODEM_CMD_SOCKET_TIMEOUT,
  .flag_data_prompt
                = true,
};

static const struct at_cmd_def_s cmd_ATpUSOWR_binary =
{
  .name         = "+USOWR",
  .resp_format  = NULL,
  .resp_num     = 0,
  .timeout_dsec = MODEM_CMD_SOCKET_TIMEOUT,
  .flag_data_prompt
                = true,
};

static const struct at_cmd_def_s cmd_ATpUSOWR_data =
{
  .name         = "+USOWR",
  .resp_format  =
  (const uint8_t[]){
    RESP_FMT_INT8,
    RESP_FMT_INT16,
  },
  .resp_num     = 2,
  .timeout_dsec = MODEM_CMD_SOCKET_TIMEOUT,
};

static const struct at_cmd_def_s cmd_ATpUSOST_data =
{
  .name         = "+USOST",
  .resp_format  =
  (const uint8_t[]){
    RESP_FMT_INT8,
    RESP_FMT_INT16,
  },
  .resp_num     = 2,
  .timeout_dsec = MODEM_CMD_SOCKET_TIMEOUT,
};

static const struct at_cmd_def_s cmd_ATpUSOCTL_get_tcp_unack_bytes =
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
 * Private Function Prototypes
 ****************************************************************************/

static void modem_setup_sendto_buffer_full_timer(struct modem_socket_s *sock);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: socket_write_handler
 ****************************************************************************/

static void socket_write_handler(struct ubmodem_s *modem,
                                 const struct at_cmd_def_s *cmd,
                                 const struct at_resp_info_s *info,
                                 const uint8_t *resp_stream,
                                 size_t stream_len, void *priv)
{
  struct modem_socket_s *sock = priv;
  int8_t sockid;
  int16_t buflen;

  /*
   * Response handler for +USOWR data prompt raw buffer write.
   */

  MODEM_DEBUGASSERT(modem, cmd == ((sock->type == SOCK_DGRAM) ?
                                   &cmd_ATpUSOST_data : &cmd_ATpUSOWR_data));

  if (sock->is_closed || sock->modem_sd < 0)
    {
      /* Socket has been closed, report error. */

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);
      __ubsocket_work_done(sock);

      return;
    }

  if (info->status == RESP_STATUS_ERROR)
    {
      /* Not CME error, fetch sockets error.
       *
       * For sendto, error might be EAGAIN. This means that buffer at modem is
       * full. Problematically, there is no event from modem telling us when
       * buffer is not full anymore. Logic for handling this case starts is in
       * __ubmodem_handle_sendto_buffer_full().
       */

      __ubmodem_get_socket_error(sock);
      return;
    }

  if (resp_status_is_error_or_timeout(info->status) ||
      info->status != RESP_STATUS_OK)
    {
      dbg("%s error, status: %d\n", cmd->name, info->status);

      /* Some other error, report generic error. */

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);
      __ubsocket_work_done(sock);

      return;
    }

  /* Read the sockets number. */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &sockid))
    {
      /* Invalid data from modem? Might be missing character from "+USOWR:"
       * header. */

      dbg("Invalid %s response, %s, stream_len: %d\n", cmd->name, "no sockid",
          stream_len);

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);
      __ubsocket_work_done(sock);

      return;
    }

  /* Read the written data length. */

  if (!__ubmodem_stream_get_int16(&resp_stream, &stream_len, &buflen))
    {
      /* Invalid data from modem? */

      dbg("Invalid %s response, %s, stream_len: %d\n", cmd->name, "no buflen",
          stream_len);

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);
      __ubsocket_work_done(sock);

      return;
    }

  if (sockid != sock->modem_sd || buflen != sock->send.buflen)
    {
      dbg("Invalid %s response, sockid: %d (expected: %d), "
          "buflen: %d (expected: %d)\n", cmd->name, sockid, sock->modem_sd,
          buflen, sock->send.buflen);

      /* Some other error, report generic error. */

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);
      __ubsocket_work_done(sock);

      return;
    }

  /* Report work as done. */

  (void)__ubmodem_usrsock_send_response(modem, &sock->req, false,
                                        sock->send.buflen);

  /* Mark send work as done. */

  __ubsocket_work_done(sock);

  /* Give sendto ready event for usrsock link. */

  (void)__ubmodem_usrsock_send_event(sock, USRSOCK_EVENT_SENDTO_READY);
}

/****************************************************************************
 * Name: get_sendto_tmpbuf
 ****************************************************************************/

static void *get_sendto_tmpbuf(struct modem_socket_s *sock, size_t *length)
{
  struct ubmodem_s *modem = sock->modem;

  /* Reuse response stream buffer for reading data buffer from usrsock link.
   *
   * This is safe because stream buffer cannot be in use at this stage; Last
   * character received from modem was '@' and serial line is now in
   * data prompt mode. Modem will not send any URC events in data prompt mode.
   */

  *length = sizeof(modem->parser.stream.buf);
  return modem->parser.stream.buf;
}

/****************************************************************************
 * Name: get_sendto_tmpbuf_length
 ****************************************************************************/

static size_t get_sendto_tmpbuf_length(struct modem_socket_s *sock)
{
  size_t length;

  (void)get_sendto_tmpbuf(sock, &length);

  return length;
}

/****************************************************************************
 * Name: data_prompt_delay_handler
 ****************************************************************************/

static int data_prompt_delay_handler(struct ubmodem_s *modem,
                                     int timer_id, void * const arg)
{
  struct modem_socket_s *sock = arg;
  int err;
  ssize_t rlen;
  size_t tmpbuflen;
  uint8_t *tmpbuf;

  sock->timerid = -1;

  /*
   * Data prompt is active and we have waited for 50 msec.
   */

  tmpbuf = get_sendto_tmpbuf(sock, &tmpbuflen);
  MODEM_DEBUGASSERT(modem, tmpbuf != NULL);
  MODEM_DEBUGASSERT(modem, modem->parser.stream.pos == 0);
  MODEM_DEBUGASSERT(modem, tmpbuflen >= sock->send.buflen);

  /* Read data buffer from usrsock link. */

  rlen = read(modem->sockets.usrsockfd, tmpbuf, sock->send.buflen);
  if (rlen < 0)
    {
      dbg("Error reading %d bytes of request: ret=%d, errno=%d\n",
          sock->send.buflen, (int)rlen, errno);
      MODEM_DEBUGASSERT(modem, false);
    }
  else
    {
      MODEM_DEBUGASSERT(modem, rlen == sock->send.buflen);
    }

  /* Data has been read, inform usrsock link that request is being processed.
   */

  (void)__ubmodem_usrsock_send_response(modem, &sock->req, true, -EINPROGRESS);

  if (sock->type == SOCK_DGRAM)
    {
      err = __ubmodem_send_raw(modem, &cmd_ATpUSOST_data, socket_write_handler,
                               sock, tmpbuf, sock->send.buflen);
      MODEM_DEBUGASSERT(modem, err == OK);
      return OK;
    }
  else
    {
      err = __ubmodem_send_raw(modem, &cmd_ATpUSOWR_data, socket_write_handler,
                               sock, tmpbuf, sock->send.buflen);
      MODEM_DEBUGASSERT(modem, err == OK);
      return OK;
    }
}

/****************************************************************************
 * Name: data_prompt_handler
 ****************************************************************************/

static void data_prompt_handler(struct ubmodem_s *modem,
                                const struct at_cmd_def_s *cmd,
                                const struct at_resp_info_s *info,
                                const uint8_t *resp_stream,
                                size_t stream_len, void *priv)
{
  struct modem_socket_s *sock = priv;

  /*
   * Response handler for open binary write, 'AT+USOWR=<sockid>,<datalen>'
   */

  MODEM_DEBUGASSERT(modem, (sock->type == SOCK_DGRAM) ?
      (cmd == &cmd_ATpUSOST_binary) : (cmd == &cmd_ATpUSOWR_binary));

  if (resp_status_is_error_or_timeout(info->status) ||
      info->status != RESP_STATUS_DATAPROMPT)
    {
      /*
       * Failed to open data prompt? This should not happen, modem should
       * always return '@' and only fail after writing of specified.
       */

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);

      __ubsocket_work_done(sock);
      return;
    }

  /*
   * Data prompt is now open. We are now required to wait minimum of 50 msec
   * before sending any data.
   */

  sock->timerid = __ubmodem_set_timer(modem, MODEM_DATA_PROMPT_DELAY_MSEC,
                                      &data_prompt_delay_handler, sock);
  if (sock->timerid == -1)
    {
      /* Error here? Add assert? Or just try bailout? */

      MODEM_DEBUGASSERT(modem, false);

      (void)data_prompt_delay_handler(modem, -1, sock);
    }
}

/****************************************************************************
 * Name: txbuf_full_recheck_delay_handler
 ****************************************************************************/

static int txbuf_full_recheck_delay_handler(struct ubmodem_s *modem,
                                            int timer_id, void * const arg)
{
  struct modem_socket_s *sock = arg;

  sock->timerid = -1;

  /* Restart sockets state machine with new work */

  sock->tx_buf_full_recheck = true;
  __ubsocket_update_state(sock);

  /* Wake-up main state machine. */

  __ubmodem_wake_waiting_state(modem, false);

  return OK;
}

/****************************************************************************
 * Name: modem_setup_sendto_buffer_full_timer
 ****************************************************************************/

static void modem_setup_sendto_buffer_full_timer(struct modem_socket_s *sock)
{
  struct ubmodem_s *modem = sock->modem;

  /* Buffer still full. Refetch state after timeout. */

  sock->timerid = __ubmodem_set_timer(modem, MODEM_TX_BUF_FULL_RECHECK_MSEC,
                                      &txbuf_full_recheck_delay_handler, sock);
  if (sock->timerid == -1)
    {
      /* Error here? Add assert? Or just try bailout? */

      (void)txbuf_full_recheck_delay_handler(modem, -1, sock);
    }
}

/****************************************************************************
 * Name: socket_write_handler
 ****************************************************************************/

static void get_tcp_unack_bytes_handler(struct ubmodem_s *modem,
                                        const struct at_cmd_def_s *cmd,
                                        const struct at_resp_info_s *info,
                                        const uint8_t *resp_stream,
                                        size_t stream_len, void *priv)
{
  struct modem_socket_s *sock = priv;
  int8_t sockid;
  int8_t ctlreqid;
  int32_t nbytes_unacked;

  /*
   * Response handler for +USOCTL=<sock>,11
   */

  MODEM_DEBUGASSERT(modem, cmd == &cmd_ATpUSOCTL_get_tcp_unack_bytes);

  if (sock->is_closed || sock->modem_sd < 0)
    {
      /* Socket has been closed, do not continue polling send buffer. */

      __ubsocket_work_done(sock);
      return;
    }

  if (resp_status_is_error_or_timeout(info->status) ||
      info->status != RESP_STATUS_OK)
    {
      dbg("%s error, status: %d\n", cmd->name, info->status);

      goto err_sendto_ready;
    }

  /* Read the sockets number. */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &sockid))
    {
      dbg("Invalid %s response, %s, stream_len: %d\n", cmd->name, "no sockid",
          stream_len);

      goto err_sendto_ready;
    }

  if (sockid != sock->modem_sd)
    {
      dbg("Invalid %s response, sockid mismatch, got: %d, expect: %d\n",
          cmd->name, sockid, sock->modem_sd);

      goto err_sendto_ready;
    }

  /* Read the sockets control request identifier (should be 11). */

  if (!__ubmodem_stream_get_int8(&resp_stream, &stream_len, &ctlreqid))
    {
      dbg("Invalid %s response, %s, stream_len: %d\n", cmd->name, "no ctlreqid",
          stream_len);

      goto err_sendto_ready;
    }

  if (ctlreqid != 11)
    {
      dbg("Invalid %s response, ctlreqid mismatch, got: %d, expect: %d\n",
          cmd->name, ctlreqid, 11);

      goto err_sendto_ready;
    }

  /* Read the number of unacknowledged bytes buffered. */

  if (!__ubmodem_stream_get_int32(&resp_stream, &stream_len, &nbytes_unacked))
    {
      dbg("Invalid %s response, %s, stream_len: %d\n", cmd->name, "no nbytes_unacked",
          stream_len);

      goto err_sendto_ready;
    }

  ndbg("number of unacknowledged bytes: %d (prev: %d)\n",
       nbytes_unacked, sock->send.num_unack_start);

  if (sock->send.num_unack_start < 0)
    {
      sock->send.num_unack_start = nbytes_unacked;
    }

  if (nbytes_unacked == 0 || sock->send.num_unack_start - nbytes_unacked >=
      MODEM_MAX_BINARY_SOCKET_WRITE_BYTES)
    {
      /* We now have enough space for maximum sized write buffer at the modem
       * side.
       */

      sock->send.num_unack_start = -1;
      sock->tx_buf_full = false;
      if (sock->timerid >= 0)
        {
          __ubmodem_remove_timer(modem, sock->timerid);
          sock->timerid = -1;
        }

      (void)__ubmodem_usrsock_send_event(sock, USRSOCK_EVENT_SENDTO_READY);

      __ubsocket_work_done(sock);
      return;
    }

  /* TX buffer still full, reregister timer for recheck. */

  modem_setup_sendto_buffer_full_timer(sock);

  __ubsocket_work_done(sock);
  return;

err_sendto_ready:
  /* Error reading socket info. To recover, lets inform usrsock link that
   * sendto is ready and let actual sendto request to handle error.
   */

  /* Give sendto ready event for usrsock link. */

  (void)__ubmodem_usrsock_send_event(sock, USRSOCK_EVENT_SENDTO_READY);
  __ubsocket_work_done(sock);
}

/****************************************************************************
 * Public Internal Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_check_txfull_socket
 *
 * Description:
 *   Prepare to recheck if tx-buffer is still full
 ****************************************************************************/

void __ubmodem_check_txfull_socket(struct modem_socket_s *sock)
{
  struct ubmodem_s *modem = sock->modem;
  int err;

  /* Fetch state of sockets send buffer. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpUSOCTL_get_tcp_unack_bytes,
                           get_tcp_unack_bytes_handler,
                           sock, "=%d,11", sock->modem_sd);
  MODEM_DEBUGASSERT(modem, err == OK);
}

/****************************************************************************
 * Name: __ubmodem_handle_sendto_buffer_full
 *
 * Description:
 *   Prepare to check modem send buffer state.
 ****************************************************************************/

void __ubmodem_handle_sendto_buffer_full(struct modem_socket_s *sock)
{
  DEBUGASSERT(sock->state == MODEM_SOCKET_STATE_SENDING);

  if (sock->is_closed || sock->modem_sd < 0)
    {
      return;
    }

  /* Mark sockets outgoing buffer as full. */

  sock->tx_buf_full = true;
  sock->tx_buf_full_recheck = true;
  sock->send.num_unack_start = -1;
}

/****************************************************************************
 * Name: __ubmodem_sendto_socket
 *
 * Description:
 *   Prepare send data to sockets.
 ****************************************************************************/

void __ubmodem_sendto_socket(struct modem_socket_s *sock)
{
  struct ubmodem_s *modem = sock->modem;
  size_t tmpbuflen;
  int err;

  if (sock->is_closed)
    {
      /* Socket has been closed, report error. */

      (void)__ubmodem_usrsock_send_response(modem, &sock->req, false, -EPIPE);

      __ubsocket_work_done(sock);
      return;
    }

  /* Adjust buffer size to temporary buffer max-length. */

  tmpbuflen = get_sendto_tmpbuf_length(sock);
  if (tmpbuflen < sock->send.buflen)
    sock->send.buflen = tmpbuflen;

  if (sock->type == SOCK_DGRAM)
    {
      const uint8_t *ip = (const uint8_t *)&sock->send.toaddr.sin_addr;

      /* Open data prompt for outputting UDP binary data. */

      err = __ubmodem_send_cmd(modem, &cmd_ATpUSOST_binary, data_prompt_handler,
                               sock, "=%d,\"%d.%d.%d.%d\",%d,%d",
                               sock->modem_sd, ip[0], ip[1], ip[2], ip[3],
                               ntohs(sock->send.toaddr.sin_port),
                               sock->send.buflen);
      MODEM_DEBUGASSERT(modem, err == OK);
    }
  else
    {
      /* Open data prompt for outputting TCP binary data. */

      err = __ubmodem_send_cmd(modem, &cmd_ATpUSOWR_binary, data_prompt_handler,
                               sock, "=%d,%d", sock->modem_sd,
                               sock->send.buflen);
      MODEM_DEBUGASSERT(modem, err == OK);
    }
}

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_sendto_request
 *
 * Description:
 *   Handle sendto request for sockets
 *
 ****************************************************************************/

int __ubmodem_usrsock_handle_sendto_request(struct ubmodem_s *modem,
                                            void *reqbuf)
{
  struct usrsock_request_sendto_s *req = reqbuf;
  struct modem_socket_s *sock;
  size_t rlen;
  int err;
  struct sockaddr_in toaddr = {};

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
      err = -EPIPE;
      goto err_out;
    }

  MODEM_DEBUGASSERT(modem, sock->type == SOCK_DGRAM || sock->type == SOCK_STREAM);

  if (sock->type == SOCK_DGRAM)
    {
      /* Check if sendto address buffer has correct size. */

      if (req->addrlen < sizeof(struct sockaddr_in))
        {
          if (req->addrlen == 0 &&
              sock->connect.peeraddr.sin_addr.s_addr != htonl(0))
            {
              /* No address for sendto request, use previously provided
               * connect peer address instead.
               */

              toaddr = sock->connect.peeraddr;
            }
          else if (req->addrlen > 0)
            {
              /* Invalid address length. */

              err = -EINVAL;
              goto err_out;
            }
          else
            {
              /* Address required. */

              err = -EDESTADDRREQ;
              goto err_out;
            }
        }
      else
        {
          /* Read address. */

          rlen = read(modem->sockets.usrsockfd, &toaddr, sizeof(toaddr));
          if (rlen < 0)
            {
              err = -errno;
              dbg("Error reading %d bytes of request: ret=%d, errno=%d\n",
                  sizeof(toaddr), (int)rlen, errno);
              goto err_out;
            }
          if (rlen != sizeof(toaddr))
            {
              err = -EMSGSIZE;
              goto err_out;
            }

          if (req->addrlen > sizeof(toaddr))
            {
              /* Skip extra bytes of address buffer */

              rlen = lseek(modem->sockets.usrsockfd,
                           req->addrlen - sizeof(toaddr), SEEK_CUR);
              if (rlen < 0)
                {
                  err = -errno;
                  goto err_out;
                }
            }
        }

      /* Check if address is IPv4 type. */

      if (toaddr.sin_family != AF_INET)
        {
          /* Invalid address family. */

          err = -EINVAL;
          goto err_out;
        }

      /* Check if packet is too large to handle */

      if (req->buflen > UBMODEM_USRSOCK_UDP_MAX_PACKET_PAYLOAD)
        {
          err = -EMSGSIZE;
          goto err_out;
        }
    }
  else
    {
      if (req->addrlen > sizeof(toaddr))
        {
          /* For TCP sockets, skip extra bytes of address buffer */

          rlen = lseek(modem->sockets.usrsockfd, req->addrlen, SEEK_CUR);
          if (rlen < 0)
            {
              err = -errno;
              goto err_out;
            }
        }
    }

  if (sock->tx_buf_full)
    {
      /* Buffer full. */

      err = -EAGAIN;
      goto err_out;
    }

  /* TODO: Check if modem is currently processing blocking
   * operation (TCP connect). */

  /* We cannot be having send already in progress, check for leaking
   * state. */

  MODEM_DEBUGASSERT(modem, sock->send.buflen == 0);

  /* Setup sendto. */

  sock->send.toaddr = toaddr;
  sock->send.buflen = req->buflen;
  sock->req = req->head;

  __ubsocket_update_state(sock);
  __ubmodem_wake_waiting_state(modem, false);

  return 0;

err_out:
  return __ubmodem_usrsock_send_response(modem, reqbuf, false, err);
}
