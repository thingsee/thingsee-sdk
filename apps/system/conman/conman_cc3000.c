/****************************************************************************
 * apps/system/conman/conman_cc3000.c
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

#include <pthread.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <poll.h>
#include <arpa/inet.h>

#include <nuttx/net/usrsock.h>
#include <arch/board/board-wireless.h>
#include <apps/netutils/dnsclient.h>

#include <nuttx/wireless/wireless.h>
#include <nuttx/wireless/cc3000.h>
#include <nuttx/wireless/cc3000/include/cc3000_upif.h>
#include <nuttx/wireless/cc3000/nvmem.h>
#include <nuttx/wireless/cc3000/include/sys/types.h>
#include <nuttx/wireless/cc3000/include/sys/socket.h>
#include <nuttx/wireless/cc3000/wlan.h>
#include <nuttx/wireless/cc3000/hci.h>
#include <nuttx/wireless/cc3000/security.h>
#include <nuttx/wireless/cc3000/netapp.h>

#include "conman_dbg.h"
#include "conman_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PTH_VERIFY(x) DEBUGVERIFY(-(x))

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define NETAPP_IPCONFIG_IP_OFFSET                  (0)
#define NETAPP_IPCONFIG_SUBNET_OFFSET              (4)
#define NETAPP_IPCONFIG_GW_OFFSET                  (8)
#define NETAPP_IPCONFIG_DHCP_OFFSET                (12)
#define NETAPP_IPCONFIG_DNS_OFFSET                 (16)
#define NETAPP_IPCONFIG_MAC_OFFSET                 (20)
#define NETAPP_IPCONFIG_SSID_OFFSET                (26)

/* Undo CC3000 BSD socket API hack. */

#undef socket
#undef closesocket
#undef bind
#undef connect
#undef listen
#undef accept
#undef send
#undef sendto
#undef recv
#undef recvfrom
#undef setsockopt
#undef getsockopt
#undef gethostbyname
#undef mdnsadvertiser

/* Error codes from CC3000 SDK v1.14 */

#define ERROR_WIFI_ALREADY_DISCONNECTED -129
#define NOT_ENOUGH_SOCKETS       -128
#define SOCKET_ALREADY_EXISTS    -127
#define NOT_SUPPORTED            -126
#define TCP_OPEN_FAILED          -124
#define BAD_SOCKET_DATA          -123
#define SOCKET_NOT_FOUND         -122
#define SOCKET_TIMED_OUT         -121
#define BAD_IP_HEADER            -120
#define NEED_TO_LISTEN           -119
#define RECV_TIMED_OUT           -118
#define NEED_TO_SEND             -114
#define UNABLE_TO_SEND           -113
#define DHCP_ERROR               -100
#define DHCP_LEASE_EXPIRED       -99
#define ARP_REQUEST_FAILED       -95
#define DHCP_LEASE_RENEWING      -92
#define IGMP_ERROR               -91
#define INVALID_VALUE            -85
#define DNS_ID_ERROR             -75
#define DNS_OPCODE_ERROR         -74
#define DNS_RCODE_ERROR          -73
#define DNS_COUNT_ERROR          -72
#define DNS_TYPE_ERROR           -71
#define DNS_CLASS_ERROR          -70
#define DNS_NOT_FOUND            -69
#define SOCKET_BUFFER_TOO_SMALL  -68
#define REASSEMBLY_ERROR         -64
#define REASSEMBLY_TIMED_OUT     -63
#define BAD_REASSEMBLY_DATA      -62
#define UNABLE_TO_TCP_SEND       -60
#define ERROR_WIFI_NOT_CONNECTED -59
#define SEND_FAILED_ARP_IN_PROCESS -58
#define RECV_FAILED_SOCKET_INACTIVE -57

/* Limit for UDP packet payload size. */

#define CC3000_USRSOCK_UDP_MAX_PACKET_PAYLOAD 548

#define CC3000_USRSOCK_IOBUF_LEN CC3000_USRSOCK_UDP_MAX_PACKET_PAYLOAD

/* Maximum number for open sockets. */

#define CC3000_MAX_SOCKETS 5

/* Default timeout values for CC3000 */

#define CC3000_DEFAULT_AUC_DHCP 14400
#define CC3000_DEFAULT_AUC_ARP 3600
#define CC3000_DEFAULT_AUC_KEEPALIVE 10
#define CC3000_DEFAULT_AUC_INACTIVITY 60

/* Scan result bitfield parsing */

#define CC3000_SCAN_RESULTS_IS_VALID(res) (((res)->bitfield >> (0 + 0)) & 0x1)
#define CC3000_SCAN_RESULTS_RSSI(res)     (((res)->bitfield >> (0 + 1)) & 0x7f)
#define CC3000_SCAN_RESULTS_SECURITY(res) (((res)->bitfield >> (8 + 0)) & 0x3)
#define CC3000_SCAN_RESULTS_SSID_LEN(res) (((res)->bitfield >> (8 + 2)) & 0x3f)

/* How long to wait before reading scan results? */

#define SCAN_READ_INTERVAL 10

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct cc3000_scan_results_s
{
  uint32_t num_networks_found;
  uint32_t results;
  uint16_t bitfield;
  uint16_t frame_time;
  char ssid[32];
  uint8_t bssid[6];
};

struct wifi_thread_params_s
{
  int security;
  const char *ssid;
  const char *password;
};

struct wifi_status_s
{
  bool connected;
  struct in_addr ipaddr;
  struct in_addr dnsaddr;
};

struct wifi_socket_stat_s
{
  bool opened:1;
  bool remote_closed:1;
};

struct wifi_priv_s
{
  pthread_mutex_t mutex;
  pthread_t tid;
  bool stop:1;
  bool stopped:1;
  bool connected:1;
  bool established:1;
  bool scanning_only:1;
  bool do_scan:1;
  bool scan_started:1;
  int outfd;
  int infd;
  int main_outfd;
  int main_infd;
  struct wifi_socket_stat_s sockets[CC3000_MAX_SOCKETS];
  struct in_addr ipaddr;
  struct in_addr dnsaddr;
  struct wifi_thread_params_s cfg;
};

struct wifi_socket_s
{
  bool active:1;
  bool connected:1;
  bool send_ready:1;
  bool recv_avail:1;
  bool remote_closed:1;
  bool recv_zero_read:1;

  int16_t wifisd;
  int16_t usockid;
  uint16_t type;

  struct sockaddr_in remoteaddr;
};

struct wifi_usrsock_s
{
  int linkfd;

  struct wifi_socket_s sockets[CC3000_MAX_SOCKETS];

  uint8_t iobuf[CC3000_USRSOCK_IOBUF_LEN];
};

struct cc3000_usrsock_handler_s
{
  unsigned int hdrlen;
  int (*handler_fn)(struct wifi_usrsock_s *usrsock, void *req);
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct wifi_priv_s *wifi;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stop_wifi_thread(struct conman_s *conman);

static int cc3000_error_to_errno(int err)
{
  switch (err)
    {
      default:
      case ERROR_WIFI_ALREADY_DISCONNECTED:
      case ERROR_WIFI_NOT_CONNECTED:
      case TCP_OPEN_FAILED:
      case BAD_SOCKET_DATA:
      case BAD_IP_HEADER:
      case NEED_TO_SEND:
      case UNABLE_TO_SEND:
      case ARP_REQUEST_FAILED:
      case UNABLE_TO_TCP_SEND:
      case RECV_FAILED_SOCKET_INACTIVE:
        return -EPIPE;

      case INVALID_VALUE:
      case NEED_TO_LISTEN:
        return -EINVAL;

      case SOCKET_TIMED_OUT:
        return -ETIMEDOUT;

      case SOCKET_ALREADY_EXISTS:
        return -EALREADY;

      case NOT_SUPPORTED:
        return -EOPNOTSUPP;

      case SOCKET_NOT_FOUND:
        return -EBADF;

      case SEND_FAILED_ARP_IN_PROCESS:
        return -EAGAIN;

      case SOCKET_BUFFER_TOO_SMALL:
      case NOT_ENOUGH_SOCKETS:
        return -ENOBUFS;
    }
}

static int cc3000_usrsock_send_response(struct wifi_usrsock_s *usrsock,
                                        struct usrsock_request_common_s *req,
                                        bool inprogress, int result)
{
  struct usrsock_message_req_ack_s resp = {};
  ssize_t wlen;

  resp.xid = req->xid;
  resp.head.msgid = USRSOCK_MESSAGE_RESPONSE_ACK;
  resp.head.flags = 0;
  resp.head.flags = inprogress ? USRSOCK_MESSAGE_FLAG_REQ_IN_PROGRESS : 0;
  resp.result = result;

  wlen = write(usrsock->linkfd, &resp, sizeof(resp));
  if (wlen < 0)
    return -errno;
  if (wlen != sizeof(resp))
    return -ENOSPC;

  return 0;
}

static int cc3000_usrsock_send_event(struct wifi_usrsock_s *usrsock,
                                     struct wifi_socket_s *sock,
                                     uint16_t events)
{
  struct usrsock_message_socket_event_s event = {};
  ssize_t wlen;

  conman_dbg("events: %04x\n", events);

  event.head.msgid = USRSOCK_MESSAGE_SOCKET_EVENT;
  event.head.flags = USRSOCK_MESSAGE_FLAG_EVENT;
  event.usockid = sock->usockid;
  event.events = events;

  wlen = write(usrsock->linkfd, &event, sizeof(event));
  if (wlen < 0)
    return -errno;
  if (wlen != sizeof(event))
    return -ENOSPC;

  return 0;
}

static int
cc3000_usrsock_read_rest_of_request(struct wifi_usrsock_s *usrsock,
                           const struct usrsock_request_common_s *common_hdr,
                           void *req, size_t reqsize)
{
  ssize_t rlen;
  int err;

  if (reqsize == sizeof(*common_hdr))
    return 0;

  rlen = read(usrsock->linkfd, (uint8_t *)req + sizeof(*common_hdr),
              reqsize - sizeof(*common_hdr));
  if (rlen < 0)
    {
      err = errno;
      conman_dbg("Error reading %d bytes of request: ret=%d, errno=%d\n",
                 reqsize - sizeof(*common_hdr), (int)rlen, errno);
      return -err;
    }
  if (rlen + sizeof(*common_hdr) != reqsize)
    {
      return -EMSGSIZE;
    }

  return 0;
}

static struct wifi_socket_s *
cc3000_usrsock_get_free_sock(struct wifi_usrsock_s *usrsock)
{
  struct wifi_socket_s *sock;
  int i;

  for (i = 0; i < ARRAY_SIZE(usrsock->sockets); i++)
    {
      sock = &usrsock->sockets[i];

      if (!sock->active)
        {
          memset(sock, 0, sizeof(*sock));
          return sock;
        }
    }

  return NULL;
}

static struct wifi_socket_s *
cc3000_usrsock_get_sock(struct wifi_usrsock_s *usrsock, int16_t usockid)
{
  struct wifi_socket_s *sock;
  int i;

  for (i = 0; i < ARRAY_SIZE(usrsock->sockets); i++)
    {
      sock = &usrsock->sockets[i];

      if (sock->active && sock->usockid == usockid)
        {
          return sock;
        }
    }

  return NULL;
}

static int cc3000_usrsock_handle_socket_request(struct wifi_usrsock_s *usrsock,
                                                void *reqbuf)
{
  struct usrsock_request_socket_s *req = reqbuf;
  struct wifi_socket_s *sock;
  uint32_t optval;
  int err;
  int ret;
  int i;

  conman_dbg("domain: %d, type: %d, protocol: %d.\n",
             req->domain, req->type, req->protocol);

  /* Check if sockets is of supported type (IPv4). */

  if (req->domain != AF_INET)
    {
      /* Give error result. */

      err = -EAFNOSUPPORT;
      conman_dbg("error: %s.\n", "unsupported domain");
      goto err_out;
    }

  /* Check if sockets is of supported type (TCP or UDP). */

  if (req->type != SOCK_STREAM && req->type != SOCK_DGRAM)
    {
      /* Give error result. */

      err = -EPROTONOSUPPORT;
      conman_dbg("error: %s.\n", "unsupported type");
      goto err_out;
    }

  /* Get free socket slot. */

  sock = cc3000_usrsock_get_free_sock(usrsock);
  if (!sock)
    {
      /* Give error result. */

      err = -ENOBUFS;
      goto err_out;
    }

  /* Open WiFi socket */

  conman_dbg("cc3000_socket...\n");
  ret = cc3000_socket(req->domain, req->type, 0);
  if (ret < 0)
    {
      err = cc3000_error_to_errno(ret);
      conman_dbg("cc3000_socket failed: %d:%d\n", ret, err);
      goto err_out;
    }

  PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
  wifi->sockets[ret].opened = true;
  wifi->sockets[ret].remote_closed = false;
  PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));

  sock->active = true;
  sock->wifisd = ret;
  sock->type = req->type;
  conman_dbg("opened socket %d\n", ret);

  optval = CC3000_SOCK_ON;
  conman_dbg("cc3000_setsockopt(CC3000_SOCKOPT_RECV_NONBLOCK = ON)...\n");
  (void)cc3000_setsockopt(sock->wifisd, CC3000_SOL_SOCKET,
                          CC3000_SOCKOPT_RECV_NONBLOCK, &optval,
                          sizeof(optval));

  /* Pick unique usockid. */

  sock->usockid = (sock - usrsock->sockets) + 0xCC3;

  conman_dbg("cc3000_socket opened, usockid: %d.\n", sock->usockid);

  /* Send result for usrsock request, give the allocated usockid as ID for
   * this sockets link.
   */

  ret = cc3000_usrsock_send_response(usrsock, reqbuf, false, sock->usockid);
  if (ret < 0)
    {
      return ERROR;
    }

  if (sock->type == SOCK_DGRAM)
    {
      /* Output ready for UDP socket, inform usrsock link. */

      (void)cc3000_usrsock_send_event(usrsock, sock,
                                      USRSOCK_EVENT_SENDTO_READY);
      sock->send_ready = true;
    }

  /* Now go through open sockets and mark any socket with same wifisd as
   * remote closed. */

  for (i = 0; i < ARRAY_SIZE(usrsock->sockets); i++)
    {
      struct wifi_socket_s *osock = &usrsock->sockets[i];

      if (osock != sock && osock->wifisd == sock->wifisd)
        {
          osock->wifisd = -1;
        }
    }

  return OK;

err_out:
  return cc3000_usrsock_send_response(usrsock, reqbuf, false, err);
}

static int cc3000_usrsock_handle_close_request(struct wifi_usrsock_s *usrsock,
                                               void *reqbuf)
{
  struct usrsock_request_close_s *req = reqbuf;
  struct wifi_socket_s *sock;
  int ret;

  sock = cc3000_usrsock_get_sock(usrsock, req->usockid);
  if (!sock)
    {
      /* Give error result. */

      ret = -EBADF;
      conman_dbg("error: %s.\n", "unknown usockid");
      goto out;
    }

  if (sock->wifisd < 0)
    {
      /* Already remote closed? */
    }
  else
    {
      /* Close socket. */

      ret = cc3000_closesocket(sock->wifisd);
      if (ret < 0)
        {
          conman_dbg("closesocket failed: %d:%d\n", ret,
                     cc3000_error_to_errno(ret));

          /* Ignore error and continue. */
        }
    }
  conman_dbg("closed socket %d\n", sock->wifisd);

  PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
  wifi->sockets[sock->wifisd].opened = false;
  wifi->sockets[sock->wifisd].remote_closed = false;
  PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));

  /* Report that connection was closed. */

  sock->active = false;
  sock->wifisd = -1;
  sock->usockid = -1;
  ret = 0;

out:
  return cc3000_usrsock_send_response(usrsock, reqbuf, false, ret);
}

static int cc3000_usrsock_handle_sendto_request(struct wifi_usrsock_s *usrsock,
                                                void *reqbuf)
{
  struct usrsock_request_sendto_s *req = reqbuf;
  struct sockaddr_in toaddr = {};
  struct wifi_socket_s *sock;
  size_t iobuflen;
  ssize_t rlen;
  int wlen;
  int err;
  int ret;

  sock = cc3000_usrsock_get_sock(usrsock, req->usockid);
  if (!sock)
    {
      /* Give error result. */

      err = -EBADF;
      conman_dbg("error: %s.\n", "unknown usockid");
      goto err_out;
    }

  if (sock->wifisd < 0)
    {
      /* Closed? */

      err = -EPIPE;
      conman_dbg("error: %s.\n", "socket closed");
      goto err_out;
    }

  DEBUGASSERT(sock->type == SOCK_DGRAM || sock->type == SOCK_STREAM);

  if (sock->type == SOCK_DGRAM)
    {
      /* Check if sendto address buffer has correct size. */

      if (req->addrlen < sizeof(struct sockaddr_in))
        {
          if (req->addrlen == 0 &&
              sock->remoteaddr.sin_addr.s_addr != htonl(0))
            {
              /* No address for sendto request, use previously provided
               * connect peer address instead.
               */

              toaddr = sock->remoteaddr;
            }
          else if (req->addrlen > 0)
            {
              /* Invalid address length. */

              err = -EINVAL;
              conman_dbg("error: %s.\n", "invalid addrlen");
              goto err_out;
            }
          else
            {
              /* Address required. */

              err = -EDESTADDRREQ;
              conman_dbg("error: %s.\n", "destination address required");
              goto err_out;
            }
        }
      else
        {
          /* Read address. */

          rlen = read(usrsock->linkfd, &toaddr, sizeof(toaddr));
          if (rlen < 0)
            {
              err = -errno;
              conman_dbg("Error reading %d bytes of request: ret=%d, errno=%d\n",
                  sizeof(toaddr), (int)rlen, errno);
              goto err_out;
            }
          else if (rlen != sizeof(toaddr))
            {
              err = -EMSGSIZE;
              goto err_out;
            }

          if (req->addrlen > sizeof(toaddr))
            {
              /* Skip extra bytes of address buffer */

              rlen = lseek(usrsock->linkfd, req->addrlen - sizeof(toaddr),
                           SEEK_CUR);
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
          conman_dbg("error: %s.\n", "invalid address family");
          goto err_out;
        }

      /* Check if packet is too large to handle */

      if (req->buflen > CC3000_USRSOCK_UDP_MAX_PACKET_PAYLOAD)
        {
          err = -EMSGSIZE;
          conman_dbg("error: %s.\n", "too large UDP payload");
          goto err_out;
        }
    }
  else
    {
      if (req->addrlen > 0)
        {
          /* For TCP sockets, skip extra bytes of address buffer */

          rlen = lseek(usrsock->linkfd, req->addrlen, SEEK_CUR);
          if (rlen < 0)
            {
              err = -errno;
              goto err_out;
            }
        }
    }

  if (sock->remote_closed)
    {
      err = -EPIPE;
      conman_dbg("error: %s.\n", "remote closed");
      goto err_out;
    }

  /* Read buffer to send. */

  iobuflen = req->buflen;
  if (iobuflen > sizeof(usrsock->iobuf))
    iobuflen = sizeof(usrsock->iobuf);

  rlen = read(usrsock->linkfd, usrsock->iobuf, iobuflen);
  if (rlen < 0)
    {
      err = -errno;
      conman_dbg("Error reading %d bytes of request: ret=%d, errno=%d\n",
          iobuflen, (int)rlen, errno);
      goto err_out;
    }
  else if (rlen != iobuflen)
    {
      err = -EMSGSIZE;
      goto err_out;
    }

  /* Mark socket not ready for send. */

  sock->send_ready = false;

  /* Attempt to send. */

  if (sock->type == SOCK_DGRAM)
    {
      conman_dbg("%s: inbuflen: %d\n", "cc3000_sendto", iobuflen);

      toaddr.sin_family = CC3000_AF_INET;
      wlen = cc3000_sendto(sock->wifisd, usrsock->iobuf, iobuflen, 0,
                           (void *)&toaddr, sizeof(toaddr));
    }
  else
    {
      conman_dbg("%s: inbuflen: %d\n", "cc3000_send", iobuflen);

      wlen = cc3000_send(sock->wifisd, usrsock->iobuf, iobuflen, 0);
    }

  if (wlen > 0)
    {
      conman_dbg("send: %d\n", wlen);

      /* Mark socket ready for send. */

      sock->send_ready = true;
    }
  else if (wlen < 0)
    {
      if (wlen == -2)
        {
          err = -EAGAIN; /* No buffers available. */
        }
      else
        {
          err = cc3000_error_to_errno(wlen);
        }

      conman_dbg("send: %d (%d)\n", wlen, err);

      goto err_out;
    }

  ret = cc3000_usrsock_send_response(usrsock, reqbuf, false, wlen);

  if (sock->send_ready)
    {
      /* Output ready for socket, inform usrsock link. */

      (void)cc3000_usrsock_send_event(usrsock, sock,
                                      USRSOCK_EVENT_SENDTO_READY);
    }

  return ret;

err_out:
  return cc3000_usrsock_send_response(usrsock, reqbuf, false, err);
}

static int
cc3000_usrsock_handle_recvfrom_request(struct wifi_usrsock_s *usrsock,
                                       void *reqbuf)
{
  struct usrsock_request_recvfrom_s *req = reqbuf;
  struct usrsock_message_datareq_ack_s resp = {};
  struct sockaddr_in fromaddr = {};
  struct wifi_socket_s *sock;
  socklen_t fromlen = 0;
  size_t iobuflen;
  ssize_t wlen;
  int rlen;
  int err;

  sock = cc3000_usrsock_get_sock(usrsock, req->usockid);
  if (!sock)
    {
      /* Give error result. */

      err = -EBADF;
      conman_dbg("error: %s.\n", "unknown usockid");
      goto err_out;
    }

  if (sock->wifisd < 0)
    {
      /* Closed? */

      err = -EPIPE;
      conman_dbg("error: %s.\n", "socket closed");
      goto err_out;
    }

  if (!sock->recv_avail)
    {
      /* No data available at wifi. */

      err = -EAGAIN;
      conman_dbg("error: %s.\n", "recv not available");
      goto err_out;
    }

  if (sock->remote_closed)
    {
      /* Connection closed. */

      if (sock->recv_zero_read)
        {
          err = -EPIPE;
          conman_dbg("done: %s.\n", "connection closed, recv empty");
          goto err_out;
        }
    }

  /* Attempt to recv. */

  iobuflen = sizeof(usrsock->iobuf);
  if (sizeof(usrsock->iobuf) > req->max_buflen)
    {
      iobuflen = req->max_buflen;
    }

  /* Mark not recv avail */

  sock->recv_avail = false;

  if (sock->type == SOCK_DGRAM)
    {
      conman_dbg("%s, iobuflen: %d\n", "cc3000_recvfrom", iobuflen);

      fromlen = sizeof(fromaddr);
      rlen = cc3000_recvfrom(sock->wifisd, usrsock->iobuf, iobuflen, 0,
                             (void *)&fromaddr, &fromlen);
    }
  else
    {
      conman_dbg("%s, iobuflen: %d\n", "cc3000_recv", iobuflen);

      rlen = cc3000_recv(sock->wifisd, usrsock->iobuf, iobuflen, 0);
    }

  if (rlen < 0)
    {
      err = cc3000_error_to_errno(rlen);
      conman_dbg("recv failed: %d:%d\n", rlen, err);
      goto err_out;
    }
  else if (rlen == 0)
    {
      sock->recv_zero_read = true;
    }

  conman_dbg("recv %d bytes\n", rlen);

  /* Feed received data into usrsock link. */

  resp.reqack.xid = req->head.xid;
  resp.reqack.head.msgid = USRSOCK_MESSAGE_RESPONSE_DATA_ACK;
  resp.reqack.head.flags = 0;
  resp.reqack.result = rlen;
  resp.valuelen_nontrunc = fromlen;
  resp.valuelen = resp.valuelen_nontrunc;
  if (resp.valuelen > req->max_addrlen)
    {
      resp.valuelen = req->max_addrlen;
    }

  /* Send response header. */

  wlen = write(usrsock->linkfd, &resp, sizeof(resp));
  DEBUGASSERT(wlen == sizeof(resp));

  if (resp.valuelen > 0)
    {
      /* Send address. */

      wlen = write(usrsock->linkfd, &fromaddr, resp.valuelen);
      DEBUGASSERT(wlen == resp.valuelen);
    }

  if (resp.reqack.result)
    {
      /* Send data. */

      wlen = write(usrsock->linkfd, usrsock->iobuf, resp.reqack.result);
      DEBUGASSERT(wlen == resp.reqack.result);
    }

  if (sock->recv_avail)
    {
      /* Input ready for socket, inform usrsock link. */

      (void)cc3000_usrsock_send_event(usrsock, sock,
                                      USRSOCK_EVENT_RECVFROM_AVAIL);
    }

  return OK;

err_out:
  return cc3000_usrsock_send_response(usrsock, reqbuf, false, err);
}

static int cc3000_usrsock_handle_connect_request(struct wifi_usrsock_s *usrsock,
                                                 void *reqbuf)
{
  struct usrsock_request_connect_s *req = reqbuf;
  struct sockaddr_in connaddr = {};
  struct wifi_socket_s *sock;
  ssize_t rlen;
  int err;
  int ret;

  conman_dbg("usockid: %d, addrlen: %d\n", req->usockid, req->addrlen);

  sock = cc3000_usrsock_get_sock(usrsock, req->usockid);
  if (!sock)
    {
      err = -EBADF;
      conman_dbg("error: %s.\n", "unknown usockid");
      goto err_out;
    }

  /* Check if sockets is closed. */

  if (sock->wifisd < 0)
    {
      err = -EPIPE;
      conman_dbg("error: %s.\n", "socket closed");
      goto err_out;
    }

  /* Is given address of correct size? */

  if (req->addrlen < sizeof(struct sockaddr_in))
    {
      err = -EINVAL;
      conman_dbg("error: %s.\n", "invalid addrlen");
      goto err_out;
    }

  /* Read address. */

  rlen = read(usrsock->linkfd, &connaddr, sizeof(connaddr));
  if (rlen < 0)
    {
      err = -errno;
      conman_dbg("Error reading %d bytes of request: ret=%d, errno=%d\n",
          sizeof(connaddr), (int)rlen, errno);
      goto err_out;
    }
  else if (rlen != sizeof(connaddr))
    {
      err = -EMSGSIZE;
      goto err_out;
    }

  conman_dbg("addr.sin_family: %d\n", connaddr.sin_family);

  /* Check if address is IPv4 type. */

  if (connaddr.sin_family != AF_INET)
    {
      /* Invalid address family. */

      err = -EINVAL;
      conman_dbg("error: %s.\n", "invalid address family");
      goto err_out;
    }

  /* Store peer address. */

  sock->remoteaddr = connaddr;

  if (sock->type == SOCK_DGRAM)
    {
      conman_dbg("UDP connect done.\n");

      /* Report that request has been read and is being processed. */

      return cc3000_usrsock_send_response(usrsock, reqbuf, false, 0);
    }
  else
    {
      conman_dbg("TCP connect in progress.\n");

      /* Report that request has been read and is being processed. */

      err = cc3000_usrsock_send_response(usrsock, reqbuf, true, -EINPROGRESS);
      if (err < 0)
        {
          goto err_out;
        }

      /* Launch connect. Note: this is blocking operation on CC3000. */

      ret = cc3000_connect(sock->wifisd, (void *)&sock->remoteaddr,
                           sizeof(sock->remoteaddr));
      if (ret < 0)
        {
          err = cc3000_error_to_errno(ret);
          conman_dbg("cc3000_connect failed: %d:%d\n", ret, err);
          goto err_out;
        }

      conman_dbg("TCP connect succeeded.\n");

      /* Success! Socket has been connected to remote peer. */

      sock->connected = true;

      return cc3000_usrsock_send_response(usrsock, reqbuf, false, 0);
    }

err_out:
  return cc3000_usrsock_send_response(usrsock, reqbuf, false, err);
}

static int cc3000_usrsock_handle_request(struct wifi_usrsock_s *usrsock)
{
  static const struct cc3000_usrsock_handler_s
                      cc3000_usrsock_request_handlers[USRSOCK_REQUEST__MAX] =
    {
      [USRSOCK_REQUEST_SOCKET] =
        {
          sizeof(struct usrsock_request_socket_s),
          cc3000_usrsock_handle_socket_request,
        },
      [USRSOCK_REQUEST_CLOSE] =
        {
          sizeof(struct usrsock_request_close_s),
          cc3000_usrsock_handle_close_request,
        },
      [USRSOCK_REQUEST_CONNECT] =
        {
          sizeof(struct usrsock_request_connect_s),
          cc3000_usrsock_handle_connect_request,
        },
      [USRSOCK_REQUEST_SENDTO] =
        {
          sizeof(struct usrsock_request_sendto_s),
          cc3000_usrsock_handle_sendto_request,
        },
      [USRSOCK_REQUEST_RECVFROM] =
        {
          sizeof(struct usrsock_request_recvfrom_s),
          cc3000_usrsock_handle_recvfrom_request,
        },
#if 0
      /* CC3000 does not expose any useful options to use at NuttX
       * set/getsockopt, thus not implemented.
       */
      [USRSOCK_REQUEST_SETSOCKOPT] =
        {
          sizeof(struct usrsock_request_setsockopt_s),
          cc3000_usrsock_handle_setsockopt_request,
        },
      [USRSOCK_REQUEST_GETSOCKOPT] =
        {
          sizeof(struct usrsock_request_getsockopt_s),
          cc3000_usrsock_handle_getsockopt_request,
        },
#endif
    };
  const struct cc3000_usrsock_handler_s *handlers =
      cc3000_usrsock_request_handlers;
  struct usrsock_request_common_s *common_hdr;
  uint8_t hdrbuf[16];
  int8_t reqid;
  ssize_t rlen;
  int err;

  common_hdr = (void *)hdrbuf;

  /* Read common header for request. */

  rlen = read(usrsock->linkfd, common_hdr, sizeof(*common_hdr));
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
      conman_dbg("Unknown request type: %d\n", reqid);

      /* Give error result. */

      err = -EOPNOTSUPP;
      goto err_out;
    }

  DEBUGASSERT(handlers[reqid].hdrlen < sizeof(hdrbuf));

  /* Read rest of the request. */

  rlen = cc3000_usrsock_read_rest_of_request(usrsock, common_hdr, hdrbuf,
                                             handlers[reqid].hdrlen);
  if (rlen < 0)
    {
      err = -rlen;
      goto err_out;
    }

  /* Handle request. */

  return handlers[reqid].handler_fn(usrsock, hdrbuf);

err_out:
  return cc3000_usrsock_send_response(usrsock, common_hdr, false, err);
}

static void wifi_async_callback(long event_type, char *data, uint8_t length)
{
  const char wakecmd = 'W';

  switch (event_type)
    {
    case HCI_EVNT_WLAN_UNSOL_CONNECT:
      PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
      wifi->connected = true;
      PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));
      break;

    case HCI_EVNT_WLAN_UNSOL_DISCONNECT:
      PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
      wifi->connected = false;
      PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));
      write(wifi->infd, &wakecmd, 1);
      break;

    case HCI_EVNT_WLAN_UNSOL_DHCP:
      /* Notes:
       * 1) IP config parameters are received swapped
       * 2) IP config parameters are valid only if status is OK, i.e.
       *    ulCC3000DHCP becomes 1 only if status is OK, the flag is set to 1
       *    and the addresses are valid
       */

      PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
      wifi->ipaddr.s_addr = htonl(*(uint32_t *)(data + NETAPP_IPCONFIG_IP_OFFSET));
      wifi->dnsaddr.s_addr = htonl(*(uint32_t *)(data + NETAPP_IPCONFIG_DNS_OFFSET));
      PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));
      break;

    case HCI_EVNT_BSD_TCP_CLOSE_WAIT:
      {
        int wifisd;

        memcpy(&wifisd, data, sizeof(int));

        if (wifisd >= 0 && wifisd < ARRAY_SIZE(wifi->sockets))
          {
            PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
            if (wifi->sockets[wifisd].opened)
              {
                wifi->sockets[wifisd].remote_closed = true;
              }
            PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));
          }
      }
      break;

    default:
      break;
    }
}

static bool wifi_should_stop(void)
{
  bool ret;
  PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
  ret = wifi->stop;
  PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));
  return ret;
}

static bool wifi_is_connected(void)
{
  bool ret;
  PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
  ret = wifi->connected;
  PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));
  return ret;
}

static int cc3000_usrsock_getfds(struct wifi_usrsock_s *usrsock,
                                 TICC3000fd_set *readfds,
                                 TICC3000fd_set *writefds,
                                 TICC3000fd_set *exceptfds)
{
  struct wifi_socket_s *sock;
  int highest_wifisd = -1;
  int i;

  CC3000_FD_ZERO(readfds);
  CC3000_FD_ZERO(writefds);
  CC3000_FD_ZERO(exceptfds);

  for (i = 0; i < ARRAY_SIZE(usrsock->sockets); i++)
    {
      sock = &usrsock->sockets[i];

      if (!sock->active || sock->wifisd < 0)
        {
          continue;
        }

      if (sock->type == SOCK_STREAM && !sock->connected)
        {
          continue;
        }

      if (!sock->recv_avail)
        {
          conman_dbg("%d, set read + except\n", sock->wifisd);
          CC3000_FD_SET(sock->wifisd, readfds);
          CC3000_FD_SET(sock->wifisd, exceptfds);
        }

      if (!sock->send_ready)
        {
          conman_dbg("%d, set write\n", sock->wifisd);
          CC3000_FD_SET(sock->wifisd, writefds);
        }

      if (highest_wifisd < sock->wifisd)
        {
          highest_wifisd = sock->wifisd;
        }
    }

  return highest_wifisd;
}

static void cc3000_usrsock_handle_select_results(struct wifi_usrsock_s *usrsock,
                                                 TICC3000fd_set *readfds,
                                                 TICC3000fd_set *writefds,
                                                 TICC3000fd_set *exceptfds)
{
  struct wifi_socket_s *sock;
  int i;

  for (i = 0; i < ARRAY_SIZE(usrsock->sockets); i++)
    {
      uint16_t events = 0;

      sock = &usrsock->sockets[i];

      if (!sock->active || sock->wifisd < 0)
        {
          continue;
        }

      if (sock->type == SOCK_STREAM && !sock->connected)
        {
          continue;
        }

      if (!sock->recv_avail && CC3000_FD_ISSET(sock->wifisd, readfds))
        {
          conman_dbg("%d, isset read\n", sock->wifisd);

          /* Data available, inform usrsock link. */

          events |= USRSOCK_EVENT_RECVFROM_AVAIL;
          sock->recv_avail = true;
        }

      if (!sock->recv_avail && CC3000_FD_ISSET(sock->wifisd, exceptfds))
        {
          conman_dbg("%d, isset except\n", sock->wifisd);

          /* Error on socket. Lets handle this in 'recv'/'send'. */

          events |= USRSOCK_EVENT_RECVFROM_AVAIL | USRSOCK_EVENT_SENDTO_READY;
          sock->recv_avail = true;
          sock->send_ready = true;
        }

      if (!sock->send_ready && CC3000_FD_ISSET(sock->wifisd, writefds))
        {
          conman_dbg("%d, isset write\n", sock->wifisd);

          /* Output ready, inform usrsock link. */

          events |= USRSOCK_EVENT_SENDTO_READY;
          sock->send_ready = true;
        }

      if (events != 0)
        {
          (void)cc3000_usrsock_send_event(usrsock, sock, events);
        }
    }
}

static bool cc3000_start_scan(struct timespec *scan_read_abs)
{
  unsigned long aiIntervalList[16];
  int i, ret;

  for (i = 0; i < 16; i++)
    {
      aiIntervalList[i] = 2000;
    }

  /* Start scanning. */

  ret = wlan_ioctl_set_scan_params(true, 100, 100, 5, 0x1fff, -90, 0, 205,
                                   aiIntervalList);
  if (ret != 0)
    {
      conman_dbg("cc3000_start_scan returned: %d.\n", ret);
      return false;
    }

  (void)clock_gettime(CLOCK_MONOTONIC, scan_read_abs);
  scan_read_abs->tv_sec += SCAN_READ_INTERVAL;

  return true;
}

static bool cc3000_read_scan_results(void)
{
  struct cc3000_scan_results_s cc3000_scan_res = { };
  struct conman_event_wifi_scan_entry_s result_out;
  const char event_id = 'r';
  uint8_t nresults;
  int ret;
  int i;

  nresults = -1;
  i = 0;

  ret = wlan_ioctl_get_scan_results(1, (uint8_t *)&cc3000_scan_res);
  if (ret != 0)
    {
      conman_dbg("wlan_ioctl_get_scan_results returned: %d.\n", ret);

      return false;
    }

  nresults = cc3000_scan_res.num_networks_found;
  if (nresults != cc3000_scan_res.num_networks_found)
    nresults = UINT8_MAX;

  if (nresults <= 0 || !CC3000_SCAN_RESULTS_IS_VALID(&cc3000_scan_res))
    {
      /* No scan results available yet. */

      return true;
    }

  conman_dbg("number of scan results: %d.\n", nresults);

  /* Start sending sending scan results */

  if (__conman_util_block_write(wifi->main_infd, &event_id,
                                sizeof(event_id)) != OK)
    {
      return false; /* main thread pipe write problem, exit wifi thread. */
    }
  if (__conman_util_block_write(wifi->main_infd, &nresults,
                                sizeof(nresults)) != OK)
    {
      return false;
    }

  do
    {
      if (CC3000_SCAN_RESULTS_IS_VALID(&cc3000_scan_res))
        {
          int raw_rssi;

          memset(&result_out, 0, sizeof(result_out));

          raw_rssi = CC3000_SCAN_RESULTS_RSSI(&cc3000_scan_res);
          result_out.rssi = raw_rssi - 128;

          result_out.ssid_len = CC3000_SCAN_RESULTS_SSID_LEN(&cc3000_scan_res);

          if (result_out.ssid_len > 32)
            result_out.ssid_len = 32;

          memcpy(result_out.bssid, cc3000_scan_res.bssid, 6);
          memcpy(result_out.ssid, cc3000_scan_res.ssid, result_out.ssid_len);

          if (__conman_util_block_write(wifi->main_infd, &result_out,
                                        sizeof(result_out)) != OK)
            {
              return false;
            }
        }

      ret = wlan_ioctl_get_scan_results(1, (uint8_t *)&cc3000_scan_res);
      if (ret != 0)
        {
          conman_dbg("wlan_ioctl_get_scan_results returned: %d.\n", ret);

          break;
        }

      /* Last 'wlan_ioctl_get_scan_results' call returns result with is_valid
       * flag unset. This last call is needed for flushing/reseting internal
       * scan result output logic. */

      if (++i == nresults)
        {
          break;
        }
    }
  while (CC3000_SCAN_RESULTS_IS_VALID(&cc3000_scan_res));

  /* Something went wrong for CC3000 => did not get enough results. Output
   * enough results to flush main thread. */

  memset(&result_out, 0, sizeof(result_out));
  for (; i < nresults; i++)
    {
      if (__conman_util_block_write(wifi->main_infd, &result_out,
                                    sizeof(result_out)) != OK)
        {
          return false;
        }
    }

  /* Done reading scan results. */

  return true;
}

static bool is_wifi_stopped(void *priv)
{
  return wifi_should_stop();
}

static void do_wifi_thread(void)
{
  const char connection_established_event = 'E';
  const char connection_lost_event = 'L';
  const char connection_request_failed_event = 'F';
  struct timespec scan_read = {};
  bool established = false;
  struct wifi_status_s status;
  struct wifi_usrsock_s *usrsock;
  struct pollfd pfds[2];
  int poll_timeout;
  uint32_t addr;
  uint32_t dns1;
  int ret;

  /* Initialize WiFi thread */

  PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
  conman_dbg("Initializing WiFi thread.\n");
  wifi->connected = false;
  wifi->established = false;
  wifi->scan_started = false;
  wifi->ipaddr.s_addr = 0;
  wifi->dnsaddr.s_addr = 0;
  PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));

  usrsock = calloc(1, sizeof(*usrsock));
  if (!usrsock)
    {
      conman_dbg("error allocating %d bytes.\n", sizeof(*usrsock));
      goto report_out;
    }
  usrsock->linkfd = -1;

  do
    {
      unsigned long aucDHCP = CC3000_DEFAULT_AUC_DHCP;
      unsigned long aucARP = CC3000_DEFAULT_AUC_ARP;
      unsigned long aucKeepalive = CC3000_DEFAULT_AUC_KEEPALIVE;
      unsigned long aucInactivity = 20;
      unsigned long zeroip = 0;

      if (wifi_should_stop())
        {
          goto report_out;
        }

      /* Initialize WiFi HW */

      if (!cc3000_initialize(NULL, wifi_async_callback))
        {
          conman_dbg("Could not initialize WiFi HW.\n");
          goto report_out;
        }

      if (wifi_should_stop())
        {
          goto destroy_out;
        }

      if (netapp_timeout_values(&aucDHCP, &aucARP, &aucKeepalive,
                                &aucInactivity) != 0)
        {
          conman_dbg("Could not setup networking timeouts.\n");
          goto destroy_out;
        }

      if (wifi_should_stop())
        {
          goto destroy_out;
        }

      /* Try open connection to AP. */

      if (cc3000_connect_to_accesspoint_with_stopfn(wifi->cfg.security,
                                              wifi->cfg.ssid,
                                              wifi->cfg.password,
                                              15 * 1000 /*msec*/, 0,
                                              is_wifi_stopped, NULL))
        {
          break;
        }

      if (wifi_should_stop())
        {
          goto destroy_out;
        }

      conman_dbg("Could not connect to access-point '%s'.\n", wifi->cfg.ssid);

      /* Configure CC3000 for dynamic IP address (DHCP), requires reset, saved
       * on WiFi nvmem. This is done to make sure that there is not old
       * non-dynamic configuration in WiFi nvmem. */

      netapp_dhcp(&zeroip, &zeroip, &zeroip, &zeroip);

      /* Power-off WiFi HW */

      cc3000_uninitialize();

      sleep(1);

      /* Keep retrying. */
    }
  while (true);

  /* Setup DNS resolver and inform conman main thread of established
   * connection.
   */

  PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
  conman_dbg("WiFi connection established.\n");
  wifi->established = true;
  status.connected = true;
  status.ipaddr = wifi->ipaddr;
  status.dnsaddr = wifi->dnsaddr;
  PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));

  addr = ntohl(status.ipaddr.s_addr);
  dns1 = ntohl(status.dnsaddr.s_addr);
  dns_setserver(&status.dnsaddr);

  dbg("WIFI: IP address: %d.%d.%d.%d\n", (addr >> 24) & 0xff,
      (addr >> 16) & 0xff, (addr >> 8) & 0xff, (addr >> 0) & 0xff);
  dbg("WIFI: DNS server: %d.%d.%d.%d\n", (dns1 >> 24) & 0xff,
      (dns1 >> 16) & 0xff, (dns1 >> 8) & 0xff, (dns1 >> 0) & 0xff);

  /* Open /dev/usrsock interface */

  usrsock->linkfd = open("/dev/usrsock", O_RDWR);
  if (usrsock->linkfd < 0)
    {
      conman_dbg("failed to open /dev/usrsock\n");
      goto destroy_out;
    }

  established = true;
  (void)__conman_util_block_write(wifi->main_infd,
                                  &connection_established_event,
                                  sizeof(connection_established_event));

  /* Run usrsock daemon. */

  do
    {
      TICC3000fd_set readfds, writefds, exceptfds;
      int highest_sockid;
      bool repoll;
      bool do_scan;
      int nfds;

      /* Check if there is I/O activity pending on sockets. */

      highest_sockid = cc3000_usrsock_getfds(usrsock, &readfds, &writefds,
                                             &exceptfds);
      if (highest_sockid >= 0)
        {
          struct timeval tv;
          int wifisd;

          tv.tv_sec = 0;
          tv.tv_usec = 5 * 1000;
          nfds = highest_sockid + 1;

          nfds = cc3000_select(nfds, &readfds, &writefds, &exceptfds, &tv);
          if (nfds < 0)
            {
              /* We still need to check sockets array for remote shutdown. */

              conman_dbg("cc3000_select failed: %d\n", nfds);
              nfds = 0;
            }

          PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
          for (wifisd = 0; wifisd < ARRAY_SIZE(wifi->sockets); wifisd++)
            {
              if (wifi->sockets[wifisd].opened &&
                  wifi->sockets[wifisd].remote_closed)
                {
                  if (usrsock->sockets[wifisd].active)
                    {
                      usrsock->sockets[wifisd].remote_closed = true;

                      /* Remote shutdown event, let 'recv'/'send' handle this. */

                      if (!CC3000_FD_ISSET(wifisd, &exceptfds))
                        {
                          CC3000_FD_SET(wifisd, &exceptfds);
                          nfds++;
                        }
                    }
                }
            }
          PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));

          if (nfds > 0)
            {
              cc3000_usrsock_handle_select_results(usrsock, &readfds,
                                                   &writefds, &exceptfds);
            }

          poll_timeout = 100;
        }
      else
        {
          poll_timeout = -1;
        }

      do
        {
          PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
          do_scan = wifi->do_scan;
          wifi->do_scan = false;
          PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));

          if (do_scan)
            {
              if (!cc3000_start_scan(&scan_read))
                {
                  scan_read.tv_sec = 0;
                  scan_read.tv_nsec = 0;
                }
            }

          if (scan_read.tv_sec)
            {
              struct timespec ts;

              (void)clock_gettime(CLOCK_MONOTONIC, &ts);

              /* Should we read scan results? */

              if (ts.tv_sec >= scan_read.tv_sec)
                {
                  (void)cc3000_read_scan_results();

                  scan_read.tv_sec = 0;
                  scan_read.tv_nsec = 0;
                }
              else
                {
                  int scan_timeout = (scan_read.tv_sec - ts.tv_sec) * 1000;

                  if (poll_timeout < 0 || poll_timeout > scan_timeout)
                    poll_timeout = scan_timeout;
                  if (poll_timeout <= 0)
                    poll_timeout = 1;
                }
            }

          memset(pfds, 0, sizeof(pfds));

          /* Check usrsock interface for activity. */

          pfds[0].fd = usrsock->linkfd;
          pfds[0].events = POLLIN;

          /* Check command interface for activity. */

          pfds[1].fd = wifi->outfd;
          pfds[1].events = POLLIN;

          conman_dbg("poll, pfds: %d, timeout: %d\n", 2, poll_timeout);

          repoll = false;
          ret = poll(pfds, 2, poll_timeout);
          if (ret > 0 && (pfds[0].revents & POLLIN))
            {
              cc3000_usrsock_handle_request(usrsock);
              poll_timeout = 0;
              repoll = true;
            }
          if (ret > 0 && (pfds[1].revents & POLLIN))
            {
              bool got_stop = false;
              bool got_wake = false;
              char cmd;

              do
                {
                  ret = read(wifi->outfd, &cmd, 1);
                  if (ret == 1)
                    {
                      conman_dbg("Got command '%c'.\n", cmd);

                      if (cmd == 'S')
                        {
                          /* "Stop" command. */

                          got_stop = true;
                        }
                      else if (cmd == 'W')
                        {
                          /* "Wake poll" command. */

                          got_wake = true;
                        }
                    }
                }
              while (ret == 1);

              if (got_stop)
                {
                  conman_dbg("Handle command '%c'.\n", 'S');
                  goto destroy_out;
                }
              else if (got_wake)
                {
                  conman_dbg("Handle command '%c'.\n", 'W');
                  break;
                }

              poll_timeout = 0;
              repoll = true;
            }
        }
      while (repoll);
    }
  while (!wifi_should_stop() && wifi_is_connected());

destroy_out:
  PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
  conman_dbg("Destroying WiFi thread.\n");
  wifi->established = false;
  PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));

  if (usrsock->linkfd >= 0)
    close(usrsock->linkfd);
  free(usrsock);

  cc3000_uninitialize();

report_out:
  if (established)
    {
      (void)__conman_util_block_write(wifi->main_infd, &connection_lost_event,
                                      sizeof(connection_lost_event));
    }
  else
    {
      (void)__conman_util_block_write(wifi->main_infd, &connection_request_failed_event,
                                      sizeof(connection_request_failed_event));
    }
}

static bool do_wifi_scanning_thread(void)
{
  struct timespec ts, scan_read;
  struct pollfd pfds[1];
  int poll_timeout;
  int ret;

  /* Initialize WiFi thread */

  PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
  conman_dbg("Initializing WiFi scanning thread.\n");
  wifi->connected = false;
  wifi->established = false;
  wifi->scan_started = false;
  wifi->ipaddr.s_addr = 0;
  wifi->dnsaddr.s_addr = 0;
  PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));

  /* Initialize WiFi HW */

  if (!cc3000_initialize(NULL, wifi_async_callback))
    {
      conman_dbg("Could not initialize WiFi HW.\n");
      return false;
    }

  if (wifi_should_stop())
    {
      goto destroy_out;
    }

  if (!cc3000_start_scan(&scan_read))
    {
      goto destroy_out;
    }

  do
    {
      (void)clock_gettime(CLOCK_MONOTONIC, &ts);

      /* Should we read scan results? */

      if (ts.tv_sec >= scan_read.tv_sec)
        {
          (void)cc3000_read_scan_results();

          break; /* Done scanning. */
        }

      poll_timeout = (scan_read.tv_sec - ts.tv_sec) * 1000;
      if (poll_timeout <= 0)
        {
          poll_timeout = 1;
        }

      /* Check command interface for activity. */

      memset(pfds, 0, sizeof(pfds));
      pfds[0].fd = wifi->outfd;
      pfds[0].events = POLLIN;

      conman_dbg("poll, pfds: %d, timeout: %d\n",
                 sizeof(pfds) / sizeof(pfds[0]),
                 (int)poll_timeout);

      ret = poll(pfds, sizeof(pfds) / sizeof(pfds[0]), (int)poll_timeout);
      if (ret > 0 && (pfds[0].revents & POLLIN))
        {
          bool got_stop = false;
          bool got_wake = false;
          char cmd;

          do
            {
              ret = read(wifi->outfd, &cmd, 1);
              if (ret == 1)
                {
                  conman_dbg("Got command '%c'.\n", cmd);

                  if (cmd == 'S')
                    {
                      /* "Stop" command. */

                      got_stop = true;
                    }
                  else if (cmd == 'W')
                    {
                      /* "Wake poll" command. */

                      got_wake = true;
                    }
                }
            }
          while (ret == 1);

          if (got_stop)
            {
              conman_dbg("Handle command '%c'.\n", 'S');
              goto destroy_out;
            }
          else if (got_wake)
            {
              conman_dbg("Handle command '%c'.\n", 'W');
            }
        }
    }
  while (!wifi_should_stop());

destroy_out:
  PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
  conman_dbg("Destroying WiFi thread.\n");
  wifi->established = false;
  wifi->scan_started = false;
  PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));

  cc3000_uninitialize();
  return false;
}

static void *wifi_thread(void *p)
{
  char stopped_event = 'S';

  conman_dbg("Enter.\n");

  while (!wifi_should_stop())
    {
      do_wifi_thread();
    }

  PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
  wifi->stopped = true;
  PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));
  write(wifi->main_infd, &stopped_event, 1);

  conman_dbg("Exit.\n");
  return NULL;
}

static void *wifi_scanning_only_thread(void *p)
{
  char stopped_event = 'S';

  conman_dbg("Enter.\n");

  while (!wifi_should_stop())
    {
      if (!do_wifi_scanning_thread())
        {
          break;
        }
    }

  PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
  wifi->stopped = true;
  PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));
  write(wifi->main_infd, &stopped_event, 1);

  conman_dbg("Exit.\n");
  return NULL;
}

static int start_wifi_thread(struct conman_s *conman,
                             const struct conman_wifi_connection_s *concfg,
                             bool scan_only)
{
  pthread_attr_t attr;
  int wifisec = WLAN_SEC_UNSEC;
  int ret;
  int fds[2];
  int main_fds[2];
  int flags;
  bool scanning_only = false;

  conman_dbg("Starting wifi thread...\n");

  if (wifi != NULL)
    {
      bool stop_n_restart = false;

      PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
      if (wifi->scanning_only)
        {
          /* WiFi in scanning only mode, stop thread and restart in full
           * connectivity mode. */

          stop_n_restart = true;
        }
      PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));

      if (stop_n_restart)
        {
          stop_wifi_thread(conman);
        }
      else
        {
          /* Already running */

          conman_dbg("Cannot start WiFi: %s.\n", "Already running");

          return ERROR;
        }
    }

  if (!concfg || !concfg->ssid)
    {
      if (!concfg && scan_only)
        {
          /* Start WiFi in scanning only mode. */

          scanning_only = true;
        }
      else
        {
          /* No WiFi config. */

          conman_dbg("Cannot start WiFi: %s.\n", "No WiFi config");

          return ERROR;
        }
    }
  else
    {
      if (concfg->encryption && concfg->password)
        {
          if (!strcasecmp(concfg->encryption, "wpa"))
            {
              wifisec = WLAN_SEC_WPA;
            }
          else if (!strcasecmp(concfg->encryption, "wpa2"))
            {
              wifisec = WLAN_SEC_WPA2;
            }
        }
    }

  wifi = calloc(1, sizeof(*wifi));
  if (!wifi)
    {
      conman_dbg("Cannot start WiFi: %s.\n", "Out of memory");

      return ERROR;
    }

  ret = pipe(fds);
  if (ret < 0)
    {
      conman_dbg("Cannot start WiFi: %s.\n", "Out of file-descriptors");

      free(wifi);
      wifi = NULL;
      return ERROR;
    }

  wifi->infd = fds[1];
  wifi->outfd = fds[0];

  ret = pipe(main_fds);
  if (ret < 0)
    {
      conman_dbg("Cannot start WiFi: %s.\n", "Out of file-descriptors");

      close(wifi->infd);
      close(wifi->outfd);
      free(wifi);
      wifi = NULL;
      return ERROR;
    }

  wifi->main_infd = main_fds[1];
  wifi->main_outfd = main_fds[0];

  PTH_VERIFY(pthread_mutex_init(&wifi->mutex, NULL));
  wifi->cfg.ssid = (concfg && concfg->ssid) ? concfg->ssid : "";
  wifi->cfg.security = wifisec;
  wifi->cfg.password = (concfg && concfg->password) ? concfg->password : "";
  wifi->scanning_only = scanning_only;

  flags = fcntl(wifi->outfd, F_GETFL, 0);
  if (flags == ERROR ||
      fcntl(wifi->outfd, F_SETFL, flags | O_NONBLOCK) == ERROR)
    {
      conman_dbg("Cannot make pipe non-blocking.\n");
      close(wifi->infd);
      close(wifi->outfd);
      close(wifi->main_infd);
      close(wifi->main_outfd);
      free(wifi);
      wifi = NULL;
      return ERROR;
    }

  PTH_VERIFY(pthread_attr_init(&attr));
  PTH_VERIFY(pthread_attr_setstacksize(&attr, 1024 + 128));

  /* Start pthread to handle WiFi. */

  ret = pthread_create(&wifi->tid, &attr,
                       scanning_only ? wifi_scanning_only_thread : wifi_thread,
                       NULL);
  if (ret)
    {
      conman_dbg("Can't create WiFi thread: %d\n", ret);
      pthread_attr_destroy(&attr);
      pthread_mutex_destroy(&wifi->mutex);
      close(wifi->infd);
      close(wifi->outfd);
      close(wifi->main_infd);
      close(wifi->main_outfd);
      free(wifi);
      wifi = NULL;
      return ERROR;
    }

  PTH_VERIFY(pthread_attr_destroy(&attr));
  conman_dbg("Done.\n");
  return OK;
}

static int stop_wifi_thread(struct conman_s *conman)
{
  char stopcmd = 'S';

  conman_dbg("Stopping wifi thread...\n");

  /* Not running? */

  if (wifi == NULL)
    {
      conman_dbg("Cannot stop, wifi thread not running.\n");

      return ERROR;
    }

  /* Mark thread to stop */
  PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
  wifi->stop = true;
  PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));

  /* Issue stop command */
  write(wifi->infd, &stopcmd, 1);

  /* Wait thread to exit */
  PTH_VERIFY(pthread_join(wifi->tid, NULL));
  PTH_VERIFY(pthread_mutex_destroy(&wifi->mutex));

  close(wifi->infd);
  close(wifi->outfd);
  close(wifi->main_infd);
  close(wifi->main_outfd);
  free(wifi);
  wifi = NULL;

  conman_dbg("Done.\n");

  return OK;
}

static void handle_scan_results_event(struct conman_s *conman, int fd)
{
  struct conman_event_wifi_scan_results_s *event;
  uint8_t num_results = 0;
  int last_ok;
  int i;

  /* Read number of results. */

  if (__conman_util_block_read(fd, &num_results, sizeof(num_results)) != OK)
    {
      return;
    }

  conman_dbg("num results: %d\n", num_results);

  event = calloc(1, sizeof(*event) + sizeof(event->results[0]) * num_results);
  if (!event)
    {
      conman_dbg("out of memory!\n");
      return;
    }

  event->num_results = num_results;

  /* Read results. */

  for (i = 0, last_ok = -1; i < num_results; i++)
    {
      const uint8_t *bssid;

      if (__conman_util_block_read(fd, &event->results[i],
                                   sizeof(event->results[i])) != OK)
        {
          break;
        }

      bssid = event->results[i].bssid;

      if (bssid[0] || bssid[1] || bssid[2] || bssid[3] || bssid[4] || bssid[5])
        {
          last_ok = i;
        }
    }

  event->num_results = last_ok + 1;

  __conman_send_boardcast_event(conman, CONMAN_EVENT_WIFI_SCAN, event,
            sizeof(*event) + sizeof(event->results[0]) * event->num_results);

  free(event);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __conman_cc3000_initialize
 *
 * Description:
 *   Initializes the connection manager CC3000 handling.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int __conman_cc3000_initialize(struct conman_s *conman)
{
  return OK;
}

/****************************************************************************
 * Name: __conman_cc3000_request_connection
 *
 * Description:
 *   Requests connection of type (request from the client) from the WiFi.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   type    : type of connections the client is requesting for
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int __conman_cc3000_request_connection(struct conman_s *conman,
                                       enum conman_connection_type_e type)
{
  const struct conman_wifi_connection_s *concfg =
      __conman_config_current_wifi_connection(conman);

  if (conman->connections.amount.wifi == 0)
    {
      conman_dbg("no WiFi configs available\n");
      return ERROR;
    }

  switch (type)
    {
    case CONMAN_DATA:
    case CONMAN_WIFI:
      if (conman->connections.current.wifi_refcnt++ > 0)
        {
          /* Already active. */

          conman_dbg("conn request, already active\n");

          return OK;
        }

      conman_dbg("conn request, starting...\n");

      return start_wifi_thread(conman, concfg, NULL);

    case CONMAN_NONE:
      if (conman->connections.current.wifi_refcnt == 0)
        {
          conman_dbg("BUG BUG, power off, refcount == 0\n");

          if (wifi)
            {
              conman_dbg("BUG BUG BUG, power off, refcount == 0, and wifi != NULL\n");
            }
          else
            {
              return OK;
            }
        }

      if (--conman->connections.current.wifi_refcnt > 0)
        {
          /* WiFi still in-use by other instance, do not destroy
           * connection. */

          conman_dbg("do not power off, refcount == %d\n",
                     conman->connections.current.wifi_refcnt);

          return OK;
        }

      conman_dbg("power off, destroying...\n");

      return stop_wifi_thread(conman);

    default:
      return ERROR;
    }
}

/****************************************************************************
 * Name: __conman_cc3000_get_status_connection
 *
 * Description:
 *  Get status for the current wifi connection.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   status  : pointer to status structure
 *
 * Returned Value:
 *   OK    : no errors
 *   ERROR : failure
 *
 ****************************************************************************/

int __conman_cc3000_get_status_connection(struct conman_s *conman,
                                          struct conman_status_s *status)
{
  memset(&status->info.wifi, 0, sizeof(status->info.wifi));

  if (wifi)
    {
      PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));

      if (wifi->stop)
        {
          status->status = CONMAN_STATUS_DESTROYING;
        }
      else if (wifi->established)
        {
          status->status = CONMAN_STATUS_ESTABLISHED;

          status->info.wifi.ipaddr = wifi->ipaddr;

          snprintf(status->info.wifi.ssid_name,
                   sizeof(status->info.wifi.ssid_name),
                   "%s", wifi->cfg.ssid);
        }
      else
        {
          status->status = CONMAN_STATUS_ESTABLISHING;
        }

      PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));
    }
  else
    {
      status->status = CONMAN_STATUS_OFF;
    }

  status->conn_type = CONMAN_WIFI;
  status->info_type = CONMAN_INFO_WIFI;

  return OK;
}

/****************************************************************************
 * Name: __conman_cc3000_is_destroying
 *
 * Description:
 *  Check if destroying previous connection
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *
 * Returned Value:
 *   true if still destroying
 *
 ****************************************************************************/

bool __conman_cc3000_is_destroying(struct conman_s *conman)
{
  return false;
}

/****************************************************************************
 * Name: __conman_cc3000_wifi_scan
 *
 * Description:
 *  Initiate WiFi scanning
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *
 * Returned Value:
 *   OK: scanning started successfully
 *   ERROR: could not start wifi scanning
 *
 ****************************************************************************/

int __conman_cc3000_wifi_scan(struct conman_s *conman)
{
  if (wifi)
    {
      bool stopped;

      PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));
      stopped = wifi->stopped;
      PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));

      if (stopped)
        {
          /* Already stopped, perform clean-up. */

          stop_wifi_thread(conman);
        }
    }

  if (!wifi)
    {
      /* WiFi not connected, start CC3000 in scanning mode. */

      return start_wifi_thread(conman, NULL, true);
    }
  else
    {
      char wakecmd = 'W';

      PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));

      /* WiFi connected, do scanning in parallel to connection. */

      wifi->do_scan = true;

      /* Issue wake command */

      write(wifi->infd, &wakecmd, 1);

      PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));

      return OK;
    }
}

/****************************************************************************
 * Name: __conman_cc3000_get_max_pollfds
 ****************************************************************************/

unsigned int __conman_cc3000_get_max_pollfds(struct conman_s *conman)
{
  return 1; /* WiFi thread to main thread IPC pipe. */
}

/****************************************************************************
 * Name: __conman_cc3000_setup_pollfds
 ****************************************************************************/

void __conman_cc3000_setup_pollfds(struct conman_s *conman,
                                   struct pollfd *pfds, int maxfds,
                                   int *fds_pos, int *min_timeout)
{
  if (!wifi)
    {
      return;
    }

  DEBUGASSERT(*fds_pos < maxfds);

  PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));

  if (wifi->main_outfd >= 0)
    {
      pfds[*fds_pos].fd = wifi->main_outfd;
      pfds[*fds_pos].events = POLLIN;
      pfds[*fds_pos].revents = 0;
      (*fds_pos)++;
    }

  PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));
}

/****************************************************************************
 * Name: __conman_cc3000_handle_pollfds
 ****************************************************************************/

void __conman_cc3000_handle_pollfds(struct conman_s *conman,
                                    struct pollfd *pfds)
{
  char event;
  ssize_t ret;
  bool do_process = false;

  if (!wifi)
    {
      return;
    }

  PTH_VERIFY(pthread_mutex_lock(&wifi->mutex));

  if (pfds->fd == wifi->main_outfd)
    {
      do_process = true;
    }

  PTH_VERIFY(pthread_mutex_unlock(&wifi->mutex));

  if (!do_process)
    {
      return;
    }

  /* Read event from WiFi thread. */

  ret = read(pfds->fd, &event, 1);
  if (ret == 1)
    {
      conman_dbg("Got event '%c'.\n", event);

      switch (event)
        {
          /* Thread 's'topped event. */
          case 'S':
            {
              /* Perform clean-up. */

              stop_wifi_thread(conman);
              return;
            }

          /* Scan 'r'esults event. */
          case 'r':
            {
              handle_scan_results_event(conman, pfds->fd);
              return;
            }

          /* Connection 'E'stablished event. */
          case 'E':
            {
              __conman_send_boardcast_event(conman,
                                            CONMAN_EVENT_CONNECTION_ESTABLISHED,
                                            NULL, 0);
              return;
            }

          /* Connection 'L'ost event. */
          case 'L':
            {
              __conman_send_boardcast_event(conman,
                                            CONMAN_EVENT_LOST_CONNECTION,
                                            NULL, 0);
              return;
            }

          /* Connection request 'F'ailed event. */
          case 'F':
            {
              __conman_send_boardcast_event(conman,
                                            CONMAN_EVENT_CONNECTION_REQUEST_FAILED,
                                            NULL, 0);
              return;
            }
        }
    }
}
