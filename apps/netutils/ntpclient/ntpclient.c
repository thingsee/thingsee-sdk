/****************************************************************************
 * netutils/ntpclient/ntpclient.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <stdbool.h>
#include <stdint.h>

#include <sys/socket.h>
#include <sys/time.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <apps/netutils/ntpclient.h>
#include <apps/netutils/dnsclient.h>

#include <nuttx/clock.h>

#include "ntpv3.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_HAVE_LONG_LONG
# error "64-bit integer support required for NTP client"
#endif
#ifndef CONFIG_NETUTILS_NTPCLIENT
# error "DNS client required for NTP client"
#endif

/* NTP Time is seconds since 1900. Convert to Unix time which is seconds
 * since 1970
 */

#define NTP2UNIX_TRANLSLATION 2208988800u

#define NTP_VERSION_V3       3
#define NTP_VERSION_V4       4
#define NTP_VERSION          NTP_VERSION_V4

#define MAX_SERVER_SELECTION_RETRIES 3

#ifndef CONFIG_NETUTILS_NTPCLIENT_NUM_SAMPLES
# define CONFIG_NETUTILS_NTPCLIENT_NUM_SAMPLES 5
#elif CONFIG_NETUTILS_NTPCLIENT_NUM_SAMPLES < 1
# error "NTP sample number below 1, invalid configuration"
#endif

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#ifndef CONFIG_NETUTILS_NTPCLIENT_SERVERHOSTNAME
# ifdef CONFIG_NETUTILS_NTPCLIENT_SERVERIP
/* Old config support */
#  warning "NTP server hostname not defined, using deprecated server IP address setting"
#  define CONFIG_NETUTILS_NTPCLIENT_SERVERHOSTNAME \
          inet_ntoa(CONFIG_NETUTILS_NTPCLIENT_SERVERIP)
# else
#  error "NTP server hostname not defined"
# endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This enumeration describes the state of the NTP daemon */

enum ntpc_daemon_e
{
  NTP_NOT_RUNNING = 0,
  NTP_STARTED,
  NTP_RUNNING,
  NTP_STOP_REQUESTED,
  NTP_STOPPED
};

/* This type describes the state of the NTP client daemon.  Only once
 * instance of the NTP daemon is permitted in this implementation.
 */

struct ntpc_daemon_s
{
  volatile uint8_t state; /* See enum ntpc_daemon_e */
  sem_t interlock;        /* Used to synchronize start and stop events */
  pid_t pid;              /* Task ID of the NTP daemon */
  sq_queue_t kod_list;    /* KoD excluded server addresses */
};

/* NTP offset. */

struct ntp_sample_s
{
  int64_t offset;
  int64_t delay;
  in_addr_t srv_addr;
} packet_struct;

/* Server address list. */

struct ntp_servers_s
{
  in_addr_t list[CONFIG_NETUTILS_NTPCLIENT_NUM_SAMPLES];
  size_t num;
  size_t pos;
  char *hostlist_str;
  char *hostlist_saveptr;
  char *hostnext;
};

/* KoD exclusion list. */

struct ntp_kod_exclude_s
{
  sq_entry_t node;
  in_addr_t addr;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This type describes the state of the NTP client daemon.  Only one
 * instance of the NTP daemon is permitted in this implementation.  This
 * limitation is due only to this global data structure.
 */

static struct ntpc_daemon_s g_ntpc_daemon;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sample_cmp
 ****************************************************************************/

static int sample_cmp(const void *_a, const void *_b)
{
  const struct ntp_sample_s *a = _a;
  const struct ntp_sample_s *b = _b;
  int64_t diff = a->offset - b->offset;

  if (diff < 0)
    return -1;
  else if (diff > 0)
    return 1;
  else
    return 0;
}

/****************************************************************************
 * Name: int64abs
 ****************************************************************************/

static inline int64_t int64abs(int64_t value)
{
  return value >= 0 ? value : -value;
}

/****************************************************************************
 * Name: ntpc_get_compile_timestamp
 ****************************************************************************/

static time_t ntpc_get_compile_timestamp(void)
{
  struct tm tm = {};
  int year, day, month;
  bool unknown = true;
  time_t tim;
#ifdef __DATE__
  const char *pmonth;
  const char *pyear;
  const char *pday;

  /*
   * Compile date. Format: "MMM DD YYYY", where MMM is month in three letter
   * format, DD is day of month (left padded with space if less than ten) and
   * YYYY is year. "??? ?? ????" if unknown.
   */

  pmonth = __DATE__;
  pday = pmonth + 4;
  pyear = pmonth + 7;

  year = (pyear[0] - '0') * 1000
         + (pyear[1] - '0') * 100
         + (pyear[2] - '0') * 10
         + (pyear[3] - '0') * 1;

  day = (pday[1] - '0');
  if (pday[0] != ' ')
    day += (pday[0] - '0') * 10;

  unknown = false;
  switch (pmonth[0])
    {
    default:
      unknown = true;
      break;
    case 'J':
      if (pmonth[1] == 'a') /* Jan */
        month = 1;
      else if (pmonth[2] == 'n') /* Jun */
        month = 6;
      else /* Jul */
        month = 7;
      break;
    case 'F': /* Feb */
      month = 2;
      break;
    case 'M':
      if (pmonth[2] == 'r') /* Mar */
        month = 3;
      else /* May */
        month = 5;
      break;
    case 'A':
      if (pmonth[1] == 'p') /* Apr */
        month = 4;
      else /* Aug */
        month = 8;
      break;
    case 'S': /* Sep */
      month = 9;
      break;
    case 'O': /* Oct */
      month = 10;
      break;
    case 'N': /* Nov */
      month = 11;
      break;
    case 'D': /* Dec */
      month = 12;
      break;
    }
#endif

  if (unknown)
    {
      month = 8;
      day = 18;
      year = 2015;
    }

  /* Convert date to timestamp. */

  tm.tm_hour = 0;
  tm.tm_min = 0;
  tm.tm_sec = 0;
  tm.tm_mday = day;
  tm.tm_mon = month - 1;
  tm.tm_year = year - 1900;

  tim = mktime(&tm);

  /* Reduce by one day to discount timezones. */

  tim -= 24 * 60 * 60;

  return tim;
}

/****************************************************************************
 * Name: ntpc_getuint32
 *
 * Description:
 *   Return the big-endian, 4-byte value in network (big-endian) order.
 *
 ****************************************************************************/

static inline uint32_t ntpc_getuint32(FAR const uint8_t *ptr)
{
  /* Network order is big-endian; host order is irrelevant */

  return (uint32_t)ptr[3] |          /* MS byte appears first in data stream */
         ((uint32_t)ptr[2] << 8) |
         ((uint32_t)ptr[1] << 16) |
         ((uint32_t)ptr[0] << 24);
}

/****************************************************************************
 * Name: ntpc_getuint64
 *
 * Description:
 *   Return the big-endian, 8-byte value in network (big-endian) order.
 *
 ****************************************************************************/

static inline uint64_t ntpc_getuint64(FAR const uint8_t *ptr)
{
  return ((uint64_t)ntpc_getuint32(ptr) << 32) | ntpc_getuint32(&ptr[4]);
}

/****************************************************************************
 * Name: ntpc_setuint32
 *
 * Description:
 *   Write 4-byte value to buffer in network (big-endian) order.
 *
 ****************************************************************************/

static inline void ntpc_setuint32(FAR uint8_t *ptr, uint32_t value)
{
  ptr[3] = (uint8_t)value;
  ptr[2] = (uint8_t)(value >> 8);
  ptr[1] = (uint8_t)(value >> 16);
  ptr[0] = (uint8_t)(value >> 24);
}

/****************************************************************************
 * Name: ntpc_setuint64
 *
 * Description:
 *   Write 8-byte value to buffer in network (big-endian) order.
 *
 ****************************************************************************/

static inline void ntpc_setuint64(FAR uint8_t *ptr, uint64_t value)
{
  ntpc_setuint32(ptr + 0, (uint32_t)(value >> 32));
  ntpc_setuint32(ptr + 4, (uint32_t)value);
}

/****************************************************************************
 * Name: ntp_secpart
 ****************************************************************************/

static uint32_t ntp_secpart(uint64_t time)
{
  /* NTP timestamps are represented as a 64-bit fixed-point number, in
   * seconds relative to 0000 UT on 1 January 1900.  The integer part is
   * in the first 32 bits and the fraction part in the last 32 bits, as
   * shown in the following diagram.
   *
   *    0                   1                   2                   3
   *    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
   *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   *   |                         Integer Part                          |
   *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   *   |                         Fraction Part                         |
   *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   */

  /* Get seconds part. */

  return time >> 32;
}

/****************************************************************************
 * Name: ntp_nsecpart
 ****************************************************************************/

static uint32_t ntp_nsecpart(uint64_t time)
{
  /* Get fraction part converted to nanoseconds. */

  return ((time & 0xFFFFFFFFU) * NSEC_PER_SEC) >> 32;
}

/****************************************************************************
 * Name: timespec2ntp
 *
 * Convert UNIX timespec timestamp to NTP 64-bit fixed point timestamp.
 *
 ****************************************************************************/

static uint64_t timespec2ntp(const struct timespec *ts)
{
  uint64_t ntp_time;

  /* Set fraction part. */

  ntp_time = ((uint64_t)(ts->tv_nsec) << 32) / NSEC_PER_SEC;

  DEBUGASSERT((ntp_time >> 32) == 0);

  /* Set seconds part. */

  ntp_time += (uint64_t)(ts->tv_sec) << 32;

  return ntp_time;
}

/****************************************************************************
 * Name: ntp_gettime
 *
 * Get time in NTP epoch, stored in fixed point uint64_t format (upper 32-bit
 * seconds, lower 32-bit fraction)
 ****************************************************************************/

static uint64_t ntp_localtime(void)
{
  int err = errno;
  struct timespec currts = {};

  /* Get current clock in NTP epoch. */

  (void)clock_gettime(CLOCK_REALTIME, &currts);
  currts.tv_sec += NTP2UNIX_TRANLSLATION;

  /* Restore errno. */

  errno = err;

  return timespec2ntp(&currts);
}

/****************************************************************************
 * Name: ntpc_calculate_offset
 *
 * Description:
 *   Calculate NTP time offset and round-trip delay
 *
 ****************************************************************************/

static void ntpc_calculate_offset(int64_t *offset, int64_t *delay,
                                  uint64_t local_xmittime,
                                  uint64_t local_recvtime,
                                  FAR const uint8_t *remote_recv,
                                  FAR const uint8_t *remote_xmit)
{
  uint64_t remote_recvtime;
  uint64_t remote_xmittime;

  /* Two timestamps from server, when request was received, and response
   * send. */

  remote_recvtime = ntpc_getuint64(remote_recv);
  remote_xmittime = ntpc_getuint64(remote_xmit);

  /* Calculate offset of local time compared to remote time.
   * See: https://www.eecis.udel.edu/~mills/time.html
   *      http://nicolas.aimon.fr/2014/12/05/timesync/ */

  *offset = (int64_t)((remote_recvtime - local_xmittime) +
                     (remote_xmittime - local_recvtime)) / 2;

  /* Calculate roundtrip delay. */

  *delay = (local_recvtime - local_xmittime) -
          (remote_xmittime - remote_recvtime);
}

/****************************************************************************
 * Name: ntpc_settime
 *
 * Description:
 *   Given the NTP time offset, adjust the system time
 *
 ****************************************************************************/

static void ntpc_settime(int64_t offset)
{
  struct timespec tp;

  /* Get the system time */

  (void)clock_gettime(CLOCK_REALTIME, &tp);

  /* Apply offset */

  tp.tv_sec  += ntp_secpart(offset);
  tp.tv_nsec += ntp_nsecpart(offset);
  while (tp.tv_nsec >= NSEC_PER_SEC)
    {
      tp.tv_nsec -= NSEC_PER_SEC;
      tp.tv_sec++;
    }

  /* Set the system time */

  (void)clock_settime(CLOCK_REALTIME, &tp);

  svdbg("Set time to %u.%03d seconds (offset: %s%u.%03u).\n",
        tp.tv_sec, tp.tv_nsec / NSEC_PER_MSEC,
        offset < 0 ? "-" : "",
        ntp_secpart(int64abs(offset)),
        ntp_nsecpart(int64abs(offset)) / NSEC_PER_MSEC);
}


/****************************************************************************
 * Name: ntp_address_in_kod_list
 *
 * Description: Check if address is in KoD KoD exclusion list.
 *
 ****************************************************************************/

static bool ntp_address_in_kod_list(in_addr_t server_addr)
{
  struct ntp_kod_exclude_s *entry;

  entry = (void *)sq_peek(&g_ntpc_daemon.kod_list);
  while (entry)
    {
      if (entry->addr == server_addr)
        return true;

      entry = (void *)sq_next(&entry->node);
    }

  return false;
}

/****************************************************************************
 * Name: ntp_is_kiss_of_death
 *
 * Description: Check if this is KoD response from the server. If it is,
 * add server to KoD exclusion list and return 'true'.
 *
 ****************************************************************************/

static bool ntp_is_kiss_of_death(const struct ntp_datagram_s *recv,
                                 in_addr_t server_addr)
{
  /* KoD only specified for v4. */

  if (GETVN(recv->lvm) != NTP_VERSION_V4)
    {
      if (recv->stratum == 0)
        {
          /* Stratum 0 is unspecified on v3, so ignore packet. */

          return true;
        }
      else
        {
          return false;
        }
    }

  /* KoD message if stratum == 0. */

  if (recv->stratum != 0)
    {
      return false;
    }

  /* KoD message received. */

  /* Check if we need to add server to access exclusion list. */

  if (strncmp((char *)recv->refid, "DENY", 4) == 0 ||
      strncmp((char *)recv->refid, "RSTR", 4) == 0 ||
      strncmp((char *)recv->refid, "RATE", 4) == 0)
    {
      struct ntp_kod_exclude_s *entry;

      entry = calloc(1, sizeof(*entry));
      if (entry)
        {
          entry->addr = server_addr;
          sq_addlast(&entry->node, &g_ntpc_daemon.kod_list);
        }
    }

  return true;
}

/****************************************************************************
 * Name: ntpc_verify_recvd_ntp_datagram
 ****************************************************************************/

static bool ntpc_verify_recvd_ntp_datagram(const struct ntp_datagram_s *xmit,
                                           const struct ntp_datagram_s *recv,
                                           size_t nbytes,
                                           const struct sockaddr_in *xmitaddr,
                                           const struct sockaddr_in *recvaddr,
                                           size_t recvaddrlen)
{
  time_t buildtime;
  time_t seconds;

  if (recvaddrlen != sizeof(struct sockaddr_in) ||
      xmitaddr->sin_addr.s_addr != recvaddr->sin_addr.s_addr ||
      xmitaddr->sin_port != recvaddr->sin_port ||
      xmitaddr->sin_family != recvaddr->sin_family)
    {
      svdbg("response from wrong peer\n");

      return false;
    }

  if (nbytes < NTP_DATAGRAM_MINSIZE)
    {
      /* Too short. */

      svdbg("too short response\n");

      return false;
    }

  if (GETVN(recv->lvm) != NTP_VERSION_V3 && GETVN(recv->lvm) != NTP_VERSION_V4)
    {
      /* Wrong version. */

      svdbg("wrong version: %d\n", GETVN(recv->lvm));

      return false;
    }

  if (GETMODE(recv->lvm) != 4)
    {
      /* Response not in server mode. */

      svdbg("wrong mode: %d\n", GETMODE(recv->lvm));

      return false;
    }

  if (GETMODE(recv->lvm) != 4)
    {
      /* Response not in server mode. */

      svdbg("wrong mode: %d\n", GETMODE(recv->lvm));

      return false;
    }

  if (ntp_is_kiss_of_death(recv, xmitaddr->sin_addr.s_addr))
    {
      /* KoD, Kiss-o'-Death. Ignore response. */

      svdbg("kiss-of-death response.\n");

      return false;
    }

  if (GETLI(recv->lvm) == 3)
    {
      /* Clock not synchronized. */

      svdbg("LI: not synchronized\n");

      return false;
    }

  if (memcmp(recv->origtimestamp, xmit->xmittimestamp, 8) != 0)
    {
      /* "The Originate Timestamp in the server reply should match the
       * Transmit Timestamp used in the client request." */

      svdbg("recv->origtimestamp <=> xmit->xmittimestamp mismatch.\n");

      return false;
    }

  if (ntpc_getuint32(recv->reftimestamp) == 0)
    {
      /* Invalid timestamp. */

      svdbg("invalid reftimestamp, 0x%08x.\n",
            ntpc_getuint32(recv->reftimestamp));

      return false;
    }

  if (ntpc_getuint32(recv->recvtimestamp) == 0)
    {
      /* Invalid timestamp. */

      svdbg("invalid recvtimestamp, 0x%08x.\n",
            ntpc_getuint32(recv->recvtimestamp));

      return false;
    }

  if (ntpc_getuint32(recv->xmittimestamp) == 0)
    {
      /* Invalid timestamp. */

      svdbg("invalid xmittimestamp, 0x%08x.\n",
            ntpc_getuint32(recv->xmittimestamp));

      return false;
    }

  if ((int64_t)(ntpc_getuint64(recv->xmittimestamp) -
                ntpc_getuint64(recv->recvtimestamp)) < 0)
    {
      /* Remote received our request after sending response? */

      svdbg("invalid xmittimestamp & recvtimestamp pair.\n");

      return false;
    }

  buildtime = ntpc_get_compile_timestamp();
  seconds = ntpc_getuint32(recv->recvtimestamp);
  if (seconds > NTP2UNIX_TRANLSLATION)
    {
      seconds -= NTP2UNIX_TRANLSLATION;
    }

  if (seconds < buildtime)
    {
      /* Invalid timestamp. */

      svdbg("invalid recvtimestamp, 0x%08x.\n",
            ntpc_getuint32(recv->recvtimestamp));

      return false;
    }

  return true;
}

/****************************************************************************
 * Name: ntpc_create_dgram_socket
 ****************************************************************************/

static int ntpc_create_dgram_socket(void)
{
  struct timeval tv;
  int ret;
  int sd;
  int err;

  /* Create a datagram socket  */

  sd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sd < 0)
    {
      err = errno;
      ndbg("ERROR: socket failed: %d\n", err);
      errno = err;
      return sd;
    }

  /* Setup a send timeout on the socket */

  tv.tv_sec  = 5;
  tv.tv_usec = 0;

  ret = setsockopt(sd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(struct timeval));
  if (ret < 0)
    {
      err = errno;
      ndbg("ERROR: setsockopt(SO_SNDTIMEO) failed: %d\n", errno);
      goto err_close;
    }

  /* Setup a receive timeout on the socket */

  tv.tv_sec  = 5;
  tv.tv_usec = 0;

  ret = setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval));
  if (ret < 0)
    {
      err = errno;
      ndbg("ERROR: setsockopt(SO_RCVTIMEO) failed: %d\n", errno);
      goto err_close;
    }

  return sd;

err_close:
  close(sd);
  errno = err;
  return ret;
}

/****************************************************************************
 * Name: ntp_get_next_hostip
 ****************************************************************************/

static int ntp_get_next_hostip(struct ntp_servers_s *srvs, in_addr_t *addr)
{
  int ret;

  if (srvs->pos >= srvs->num)
    {
      char *hostname;

      srvs->pos = 0;
      srvs->num = 0;

      /* Get next hostname. */

      if (srvs->hostnext == NULL)
        {
          if (!srvs->hostlist_str)
            {
              /* Allocate hostname list buffer */

              srvs->hostlist_str =
                  strdup(CONFIG_NETUTILS_NTPCLIENT_SERVERHOSTNAME);
              if (!srvs->hostlist_str)
                {
                  return ERROR;
                }
            }
          else
            {
              /* Reset hostname list buffer */

              strcpy(srvs->hostlist_str,
                     CONFIG_NETUTILS_NTPCLIENT_SERVERHOSTNAME);
            }

          srvs->hostlist_saveptr = NULL;
#ifndef __clang_analyzer__ /* Silence false 'possible memory leak'. */
          srvs->hostnext =
              strtok_r(srvs->hostlist_str, ";", &srvs->hostlist_saveptr);
#endif
        }

      hostname = srvs->hostnext;
      srvs->hostnext = strtok_r(NULL, ";", &srvs->hostlist_saveptr);

      if (!hostname)
        {
          /* Invalid configuration. */

          errno = EINVAL;
          return ERROR;
        }

      /* Refresh DNS for new IP-addresses. */

      ret = dns_gethostip_multi(hostname, srvs->list, ARRAY_SIZE(srvs->list));
      if (ret <= 0)
        {
          return ERROR;
        }

      srvs->num = ret;
    }

  *addr = srvs->list[srvs->pos++];

  return OK;
}

/****************************************************************************
 * Name: ntpc_get_ntp_sample
 ****************************************************************************/

static int ntpc_get_ntp_sample(struct ntp_servers_s *srvs,
                               struct ntp_sample_s *samples, int curr_idx)
{
  struct ntp_sample_s *sample = &samples[curr_idx];
  uint64_t xmit_time, recv_time;
  struct sockaddr_in server;
  struct sockaddr_in recvaddr;
  struct ntp_datagram_s xmit;
  struct ntp_datagram_s recv;
  socklen_t socklen;
  ssize_t nbytes;
  int errval;
  int retry = 0;
  int nsamples = curr_idx;
  bool addr_ok;
  int ret;
  int sd = -1;
  int i;

  /* Setup or sockaddr_in struct with information about the server we are
   * going to ask the the time from.
   */

  memset(&server, 0, sizeof(struct sockaddr_in));
  server.sin_family      = AF_INET;
  server.sin_port        = HTONS(CONFIG_NETUTILS_NTPCLIENT_PORTNO);

  do
    {
      addr_ok = true;

      ret = ntp_get_next_hostip(srvs, &server.sin_addr.s_addr);
      if (ret < 0)
        {
          errval = errno;

          ndbg("ERROR: sendto() failed: %d\n", errval);

          goto sock_error;
        }

      /* Make sure that server not in exclusion list. */

      if (ntp_address_in_kod_list(server.sin_addr.s_addr))
        {
          if (retry < MAX_SERVER_SELECTION_RETRIES)
            {
              svdbg("on KoD list. retry DNS.\n");

              retry++;
              addr_ok = false;
              continue;
            }
          else
            {
              errval = -EALREADY;
              goto sock_error;
            }
        }

      /* Make sure that this sample is from new server. */

      for (i = 0; i < nsamples; i++)
        {
          if (server.sin_addr.s_addr == samples[i].srv_addr)
            {
              /* Already have sample from this server, retry DNS. */

              svdbg("retry DNS.\n");

              if (retry < MAX_SERVER_SELECTION_RETRIES)
                {
                  retry++;
                  addr_ok = false;
                  break;
                }
              else
                {
                  errval = -EALREADY;
                  goto sock_error;
                }
            }
        }
    }
  while (!addr_ok);

  /* Open socket. */

  sd = ntpc_create_dgram_socket();
  if (sd < 0)
    {
      errval = errno;

      ndbg("ERROR: ntpc_create_dgram_socket() failed: %d\n", errval);

      goto sock_error;
    }

  /* Format the transmit datagram */

  memset(&xmit, 0, sizeof(xmit));
  xmit.lvm = MKLVM(0, NTP_VERSION, 3);

  svdbg("Sending a NTPv%d packet\n", NTP_VERSION);

  xmit_time = ntp_localtime();
  ntpc_setuint64(xmit.xmittimestamp, xmit_time);
  ret = sendto(sd, &xmit, sizeof(struct ntp_datagram_s),
               0, (FAR struct sockaddr *)&server,
               sizeof(struct sockaddr_in));
  if (ret < 0)
    {
      errval = errno;

      ndbg("ERROR: sendto() failed: %d\n", errval);

      goto sock_error;
    }

  /* Attempt to receive a packet (with a timeout that was set up via
   * setsockopt() above)
   */

  socklen = sizeof(struct sockaddr_in);
  nbytes = recvfrom(sd, (void *)&recv, sizeof(struct ntp_datagram_s),
                    0, (FAR struct sockaddr *)&recvaddr, &socklen);
  recv_time = ntp_localtime();

  /* Check if the received message was long enough to be a valid NTP
   * datagram.
   */

  if (nbytes > 0 && ntpc_verify_recvd_ntp_datagram(
                          &xmit, &recv, nbytes, &server, &recvaddr, socklen))
    {
      close(sd);
      sd = -1;

      svdbg("Calculate offset\n");

      memset(sample, 0, sizeof(sample));

      sample->srv_addr = server.sin_addr.s_addr;

      ntpc_calculate_offset(&sample->offset, &sample->delay,
                            xmit_time, recv_time, recv.recvtimestamp,
                            recv.xmittimestamp);

      return OK;
    }
  else
    {
      /* Check for errors.  Short datagrams are handled as error. */

      errval = errno;

      if (nbytes >= 0)
        {
          errval = EMSGSIZE;
        }
      else
        {
          ndbg("ERROR: recvfrom() failed: %d\n", errval);
        }

      goto sock_error;
    }

sock_error:
  if (sd >= 0)
    {
      close(sd);
      sd = -1;
    }

  errno = errval;
  return ERROR;
}

/****************************************************************************
 * Name: ntpc_daemon
 *
 * Description:
 *   This the the NTP client daemon.  This is a *very* minimal
 *   implementation! An NTP request is and the system clock is set when the
 *   response is received
 *
 ****************************************************************************/

static int ntpc_daemon(int argc, char **argv)
{
  struct ntp_sample_s samples[CONFIG_NETUTILS_NTPCLIENT_NUM_SAMPLES];
  struct ntp_servers_s srvs;
  int exitcode = EXIT_SUCCESS;
  int retries = 0;
  int nsamples;
  int ret;

  srvs.num = 0;
  srvs.pos = 0;
  srvs.hostlist_saveptr = NULL;
  srvs.hostlist_str = NULL;
  srvs.hostnext = NULL;

  /* Indicate that we have started */

  g_ntpc_daemon.state = NTP_RUNNING;
  sem_post(&g_ntpc_daemon.interlock);

  /* Here we do the communication with the NTP server. We collect set of
   * NTP samples (hopefully from different servers when using DNS) and
   * select median time-offset of samples. This is to filter out misconfigured
   * server giving wrong timestamps.
   *
   * NOTE that the scheduler is locked whenever this loop runs.  That
   * assures both:  (1) that there are no asynchronous stop requests and
   * (2) that we are not suspended while in critical moments when we about
   * to set the new time.  This sounds harsh, but this function is suspended
   * most of the time either: (1) sending a datagram, (2) receiving a datagram,
   * or (3) waiting for the next poll cycle.
   */

  sched_lock();
  while (g_ntpc_daemon.state != NTP_STOP_REQUESTED)
    {
      int errval = 0;
      int i;

      /* Collect samples. */

      for (nsamples = 0, i = 0; i < CONFIG_NETUTILS_NTPCLIENT_NUM_SAMPLES; i++)
        {
          /* Get next sample. */

          ret = ntpc_get_ntp_sample(&srvs, samples, nsamples);
          if (ret < 0)
            {
              errval = errno;
            }
          else
            {
              ++nsamples;
            }
        }

      /* Analyse samples. */

      if (nsamples > 0)
        {
          int64_t offset;

          /* Select median offset of samples. */

          qsort(samples, nsamples, sizeof(*samples), sample_cmp);

          for (i = 0; i < nsamples; i++)
            {
              svdbg("NTP sample[%d]: offset: %s%u.%03u sec, round-trip delay: %s%u.%03u sec\n",
                    i,
                    samples[i].offset < 0 ? "-" : "",
                    ntp_secpart(int64abs(samples[i].offset)),
                    ntp_nsecpart(int64abs(samples[i].offset)) / NSEC_PER_MSEC,
                    samples[i].delay < 0 ? "-" : "",
                    ntp_secpart(int64abs(samples[i].delay)),
                    ntp_nsecpart(int64abs(samples[i].delay)) / NSEC_PER_MSEC);
            }

          if ((nsamples % 2) == 1)
            {
              offset = samples[nsamples / 2].offset;
            }
          else
            {
              int64_t offset1 = samples[nsamples / 2].offset;
              int64_t offset2 = samples[nsamples / 2 - 1].offset;

              /* Average of two middle offsets. */

              if (offset1 > 0 && offset2 > 0)
                {
                  offset = ((uint64_t)offset1 + (uint64_t)offset2) / 2;
                }
              else if (offset1 < 0 && offset2 < 0)
                {
                  offset1 = -offset1;
                  offset2 = -offset2;

                  offset = ((uint64_t)offset1 + (uint64_t)offset2) / 2;

                  offset = -offset;
                }
              else
                {
                  offset = (offset1 + offset2) / 2;
                }
            }

          /* Adjust system time. */

          ntpc_settime(offset);

#ifndef CONFIG_NETUTILS_NTPCLIENT_STAY_ON
          /* Configured to exit at success. */

          exitcode = EXIT_SUCCESS;
          break;

#else

          /* A full implementation of an NTP client would require much more.  I
           * think we can skip most of that here.
           */

          if (g_ntpc_daemon.state == NTP_RUNNING)
            {
              svdbg("Waiting for %d seconds\n",
                    CONFIG_NETUTILS_NTPCLIENT_POLLDELAYSEC);

              (void)sleep(CONFIG_NETUTILS_NTPCLIENT_POLLDELAYSEC);
              retries = 0;
            }

          continue;
#endif
        }

      /* Exceeded maximum retries? */

      if (retries++ >= CONFIG_NETUTILS_NTPCLIENT_RETRIES)
        {
          ndbg("ERROR: too many retries: %d\n", retries - 1);
          exitcode = EXIT_FAILURE;
          break;
        }

      /* Is this error a signal? If not, sleep before retry. */

      if (errval != EINTR)
        {
          sleep(1);
        }

      /* Keep retrying. */
    }

  /* The NTP client is terminating */

  sched_unlock();

  g_ntpc_daemon.state = NTP_STOPPED;
  sem_post(&g_ntpc_daemon.interlock);
  free(srvs.hostlist_str);
  return exitcode;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: ntpc_start
 *
 * Description:
 *   Start the NTP daemon
 *
 * Returned Value:
 *   On success, the non-negative task ID of the NTPC daemon is returned;
 *   On failure, a negated errno value is returned.
 *
 ****************************************************************************/

int ntpc_start(void)
{
  /* Is the NTP in a non-running state? */

  sched_lock();
  if (g_ntpc_daemon.state == NTP_NOT_RUNNING ||
      g_ntpc_daemon.state == NTP_STOPPED)
    {
      /* Is this the first time that the NTP daemon has been started? */

      if (g_ntpc_daemon.state == NTP_NOT_RUNNING)
        {
          /* Yes... then we will need to initialize the state structure */

          sem_init(&g_ntpc_daemon.interlock, 0, 0);
        }

      /* Start the NTP daemon */

      g_ntpc_daemon.state = NTP_STARTED;
      g_ntpc_daemon.pid =
        task_create("NTP daemon", CONFIG_NETUTILS_NTPCLIENT_SERVERPRIO,
                    CONFIG_NETUTILS_NTPCLIENT_STACKSIZE, ntpc_daemon,
                    NULL);

      /* Handle failures to start the NTP daemon */

      if (g_ntpc_daemon.pid < 0)
        {
          int errval = errno;
          DEBUGASSERT(errval > 0);

          g_ntpc_daemon.state = NTP_STOPPED;
          ndbg("ERROR: Failed to start the NTP daemon\n", errval);
          sched_unlock();
          return -errval;
        }

      /* Wait for any daemon state change */

      do
        {
          (void)sem_wait(&g_ntpc_daemon.interlock);
        }
      while (g_ntpc_daemon.state == NTP_STARTED);
    }

  sched_unlock();
  return g_ntpc_daemon.pid;
}

/****************************************************************************
 * Name: ntpc_stop
 *
 * Description:
 *   Stop the NTP daemon
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.  The current
 *   implementation only returns success.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_SIGNALS
int ntpc_stop(void)
{
  int ret;

  /* Is the NTP in a running state? */

  sched_lock();
  if (g_ntpc_daemon.state == NTP_STARTED ||
      g_ntpc_daemon.state == NTP_RUNNING)
    {
      /* Yes.. request that the daemon stop. */

      g_ntpc_daemon.state = NTP_STOP_REQUESTED;

      /* Wait for any daemon state change */

      do
        {
          /* Signal the NTP client */

          ret = kill(g_ntpc_daemon.pid,
                     CONFIG_NETUTILS_NTPCLIENT_SIGWAKEUP);

          if (ret < 0)
            {
              ndbg("ERROR: kill pid %d failed: %d\n",
                   g_ntpc_daemon.pid, errno);
              break;
            }

          /* Wait for the NTP client to respond to the stop request */

          (void)sem_wait(&g_ntpc_daemon.interlock);
        }
      while (g_ntpc_daemon.state == NTP_STOP_REQUESTED);
    }

  sched_unlock();
  return OK;
}
#endif
