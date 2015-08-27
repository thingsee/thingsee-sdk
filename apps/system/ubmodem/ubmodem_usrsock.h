/****************************************************************************
 * apps/system/ubmodem/ubmodem_usrsock.h
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

#ifndef __SYSTEM_UBMODEM_UBMODEM_USRSOCK_H_
#define __SYSTEM_UBMODEM_UBMODEM_USRSOCK_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_UBMODEM_USRSOCK

#include <stdint.h>
#include <stdbool.h>
#include <arpa/inet.h>
#include <nuttx/net/usrsock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/* Socket type values for modem. */

enum modem_socket_type_e
{
  MODEM_SOCKET_TYPE_TCP = 6,
  MODEM_SOCKET_TYPE_UDP = 17,
};

/* Socket option types for modem. */

enum modem_socket_options_e
{
  MODEM_SOL_SOCKET            = 65535,
  MODEM_SO_REUSEADDR          = 4,
  MODEM_SO_KEEPALIVE          = 8,
  MODEM_SO_BOARDCAST          = 32,
  MODEM_SO_LINGER             = 128,
  MODEM_SO_REUSEADDR_AND_PORT = 512,
};

/* Socket status values from modem. */

enum modem_socket_status_e
{
  MODEM_SOCKET_STATUS_INACTIVE = 0,
  MODEM_SOCKET_STATUS_LISTEN = 1,
  MODEM_SOCKET_STATUS_SYN_SENT = 2,
  MODEM_SOCKET_STATUS_SYN_RCVD = 3,
  MODEM_SOCKET_STATUS_ESTABILISHED = 4,
  MODEM_SOCKET_STATUS_FIN_WAIT_1 = 5,
  MODEM_SOCKET_STATUS_FIN_WAIT_2 = 6,
  MODEM_SOCKET_STATUS_CLOSE_WAIT = 7,
  MODEM_SOCKET_STATUS_CLOSING = 8,
  MODEM_SOCKET_STATUS_LAST_ACK = 9,
  MODEM_SOCKET_STATUS_TIME_WAIT = 10,

  __MODEM_SOCKET_STATUS_FIRST = MODEM_SOCKET_STATUS_INACTIVE,
  __MODEM_SOCKET_STATUS_LAST = MODEM_SOCKET_STATUS_TIME_WAIT,
};

/* Socket error codes from modem. */

enum modem_socket_errors_e
{
  MODEM_SOCKET_NOERR = 0,
  MODEM_SOCKET_ENOENT = 2,
  MODEM_SOCKET_EINTR = 4,
  MODEM_SOCKET_EIO = 5,
  MODEM_SOCKET_EBADF = 9,
  MODEM_SOCKET_ECHILD = 10,
  MODEM_SOCKET_EWOULDBLOCK = 11,
  MODEM_SOCKET_ENOMEM = 12,
  MODEM_SOCKET_EFAULT = 14,
  MODEM_SOCKET_EINVAL = 22,
  MODEM_SOCKET_EPIPE = 32,
  MODEM_SOCKET_ENOSYS = 38,
  MODEM_SOCKET_ENOPROTOOPT = 92,
  MODEM_SOCKET_EADDRINUSE = 98,
  MODEM_SOCKET_ECONNABORTED = 103,
  MODEM_SOCKET_ECONNRESET = 104,
  MODEM_SOCKET_ENOBUFS = 105,
  MODEM_SOCKET_ENOTCONN = 107,
  MODEM_SOCKET_ESHUTDOWN = 108,
  MODEM_SOCKET_ETIMEDOUT = 110,
  MODEM_SOCKET_EHOSTUNREACH = 113,
  MODEM_SOCKET_EINPROGRESS = 115,
};

/* Socket state machine states. */

enum modem_socket_state_e
{
  MODEM_SOCKET_STATE_INACTIVE = 0,
  MODEM_SOCKET_STATE_OPENING,
  MODEM_SOCKET_STATE_CLOSING,
  MODEM_SOCKET_STATE_CONNECTING,
  MODEM_SOCKET_STATE_SENDING,
  MODEM_SOCKET_STATE_RECEIVING,
  MODEM_SOCKET_STATE_CHECKING_TXFULL,
  MODEM_SOCKET_STATE_SETGETSOCK,
  MODEM_SOCKET_STATE_FREE,
};

struct modem_socket_s
{
  sq_entry_t node;
  struct ubmodem_s *modem;

  int16_t usockid;                  /* Socket identifier used over usrsock link. */
  int8_t modem_sd;                  /* Socket identifier given by modem. */
  int8_t type;                      /* Socket type (SOCK_STREAM, SOCK_DGRAM) */
  enum modem_socket_state_e state;  /* State for socket state machine */
  int32_t timerid;                  /* Id for active timer */

  bool is_closed:1;                 /* Socket closed by remote/modem/usrsock. */
  bool is_waiting_recv:1;           /* Usrsock link is waiting to receive data from socket. */
  bool is_waiting_conn:1;           /* Usrsock link is waiting for TCP connect completion. */
  bool is_closing:1;                /* Socket waiting to be closed. */
  bool tx_buf_full:1;               /* Outgoing buffer is full. */
  bool tx_buf_full_recheck:1;       /* Rechecking if tx-buffer is full.*/
  bool is_setgetsock_pending:1;     /* Pending get/setsock operation. */
  bool is_freeing_pending:1;        /* Pending socket freeing operation. */

  struct usrsock_request_common_s req;

  /* Connect state for socket */

  struct
  {
    struct sockaddr_in peeraddr;
  } connect;

  /* Tx state for socket */

  struct
  {
    struct sockaddr_in toaddr;
    uint16_t buflen;
    int32_t num_unack_start;
  } send;

  /* Rx state for socket */

  struct
  {
    uint16_t avail;
    uint16_t max_addrlen;
    uint16_t max_buflen;
  } recv;

  /* getsockopt/setsockopt/getsockname state */

  struct
  {
    uint16_t level;
    uint16_t option;
    uint16_t valuesize;
  } setgetsock;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
                                  const struct ubmodem_event_ip_address_s *ipcfg);

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

void __ubmodem_usrsock_uninitialize(struct ubmodem_s *modem);

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

int __ubmodem_usrsock_handle_request(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_has_usrsock_work
 *
 * Description:
 *   Check if there is socket with active work waiting.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 * Return value:
 *   true if there is socket waiting to do work.
 *
 ****************************************************************************/

bool __ubmodem_has_usrsock_work(struct ubmodem_s *modem);

/****************************************************************************
 * Name: __ubmodem_do_usrsock_work
 *
 * Description:
 *   Perform socket work.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 ****************************************************************************/

void __ubmodem_do_usrsock_work(struct ubmodem_s *modem);

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
                                    int result);

/****************************************************************************
 * Name: __ubmodem_usrsock_send_event
 *
 * Description:
 *   Send event to usrsock link
 *
 ****************************************************************************/

int __ubmodem_usrsock_send_event(struct modem_socket_s *sock, uint16_t events);

/****************************************************************************
 * Name: __ubmodem_socket_get_by_usockid
 *
 * Description:
 *   Get socket by usrsock link identifier 'usockid'.
 *
 ****************************************************************************/

struct modem_socket_s *
__ubmodem_socket_get_by_usockid(struct ubmodem_s *modem, int16_t usockid);

/****************************************************************************
 * Name: __ubsocket_update_state
 *
 * Description:
 *   Update socket state based on pending operations.
 ****************************************************************************/

void __ubsocket_update_state(struct modem_socket_s *sock);

/****************************************************************************
 * Name: __ubsocket_work_done
 *
 * Description:
 *   Mark current work item as complete and mark socket inactive.
 ****************************************************************************/

void __ubsocket_work_done(struct modem_socket_s *sock);

/****************************************************************************
 * Name: __ubmodem_get_socket_error
 *
 * Description:
 *   Send get socket error AT command.
 ****************************************************************************/

void __ubmodem_get_socket_error(struct modem_socket_s *sock);

/****************************************************************************
 * Name: __ubmodem_open_socket
 *
 * Description:
 *   Send open socket AT command.
 ****************************************************************************/

void __ubmodem_open_socket(struct modem_socket_s *sock);

/****************************************************************************
 * Name: __ubmodem_close_socket
 *
 * Description:
 *   Send close socket AT command.
 ****************************************************************************/

void __ubmodem_close_socket(struct modem_socket_s *sock);

/****************************************************************************
 * Name: __ubmodem_connect_socket
 *
 * Description:
 *   Send connect socket AT command.
 ****************************************************************************/

void __ubmodem_connect_socket(struct modem_socket_s *sock);

/****************************************************************************
 * Name: __ubmodem_recvfrom_socket
 *
 * Description:
 *   Receive data from socket
 ****************************************************************************/

void __ubmodem_recvfrom_socket(struct modem_socket_s *sock);

/****************************************************************************
 * Name: __ubmodem_sendto_socket
 *
 * Description:
 *   Prepare send data to socket.
 ****************************************************************************/

void __ubmodem_sendto_socket(struct modem_socket_s *sock);

/****************************************************************************
 * Name: __ubmodem_check_txfull_socket
 *
 * Description:
 *   Prepare to recheck if tx-buffer is still full
 ****************************************************************************/

void __ubmodem_check_txfull_socket(struct modem_socket_s *sock);

/****************************************************************************
 * Name: __ubmodem_handle_sendto_buffer_full
 *
 * Description:
 *   Prepare to check modem send buffer state.
 ****************************************************************************/

void __ubmodem_handle_sendto_buffer_full(struct modem_socket_s *sock);

/****************************************************************************
 * Name: __ubmodem_do_setgetsock_socket
 *
 * Description:
 *   Prepare set or get socket option.
 ****************************************************************************/

void __ubmodem_do_setgetsock_socket(struct modem_socket_s *sock);

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_socket_request
 *
 * Description:
 *   Handle request for new socket
 *
 ****************************************************************************/

int __ubmodem_usrsock_handle_socket_request(struct ubmodem_s *modem,
                                            void *reqbuf);

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_close_request
 *
 * Description:
 *   Handle close request for socket
 ****************************************************************************/

int __ubmodem_usrsock_handle_close_request(struct ubmodem_s *modem,
                                           void *reqbuf);

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_connect_request
 *
 * Description:
 *   Handle connect request for socket
 ****************************************************************************/

int __ubmodem_usrsock_handle_connect_request(struct ubmodem_s *modem,
                                             void *reqbuf);

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_sendto_request
 *
 * Description:
 *   Handle sendto request for socket
 *
 ****************************************************************************/

int __ubmodem_usrsock_handle_sendto_request(struct ubmodem_s *modem,
                                            void *reqbuf);

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_recvfrom_request
 *
 * Description:
 *   Handle recvfrom request for socket
 ****************************************************************************/

int __ubmodem_usrsock_handle_recvfrom_request(struct ubmodem_s *modem,
                                              void *reqbuf);

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_setsockopt_request
 *
 * Description:
 *   Handle setsockopt request for sockets
 *
 ****************************************************************************/

int __ubmodem_usrsock_handle_setsockopt_request(struct ubmodem_s *modem,
                                                void *reqbuf);

/****************************************************************************
 * Name: __ubmodem_usrsock_handle_getsockopt_request
 *
 * Description:
 *   Handle getsockopt request for sockets
 *
 ****************************************************************************/

int __ubmodem_usrsock_handle_getsockopt_request(struct ubmodem_s *modem,
                                                void *reqbuf);

#endif /* CONFIG_UBMODEM_USRSOCK */

#endif /* __SYSTEM_UBMODEM_UBMODEM_USRSOCK_H_ */
