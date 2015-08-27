/****************************************************************************
 * apps/netutils/dnsclient/dns_socket.c
 * DNS host name to IP address resolver.
 *
 * The uIP DNS resolver functions are used to lookup a hostname and
 * map it to a numerical IP address. It maintains a list of resolved
 * hostnames that can be queried. New hostnames can be resolved using the
 * dns_whois() function.
 *
 *   Copyright (C) 2007, 2009, 2012, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based heavily on portions of uIP:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2002-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/socket.h>
#include <sys/time.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <apps/netutils/dnsclient.h>
#include <apps/netutils/netlib.h>

#include <nuttx/random.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NETUTILS_DNSCLIENT_RECV_TIMEOUT
#  define CONFIG_NETUTILS_DNSCLIENT_RECV_TIMEOUT 30
#endif

#ifndef CONFIG_NETUTILS_DNSCLIENT_RETRIES
#  define CONFIG_NETUTILS_DNSCLIENT_RETRIES 3
#endif

#ifndef CONFIG_NETUTILS_DNSCLIENT_ENTRIES
#  define RESOLV_ENTRIES 4
#else /* CONFIG_NETUTILS_DNSCLIENT_ENTRIES */
#  define RESOLV_ENTRIES CONFIG_NETUTILS_DNSCLIENT_ENTRIES
#endif /* CONFIG_NETUTILS_DNSCLIENT_ENTRIES */

#ifndef NULL
#  define NULL (void *)0
#endif /* NULL */

/* The maximum number of retries when asking for a name */

#define MAX_RETRIES 8

#define DNS_FLAG1_RESPONSE        0x80
#define DNS_FLAG1_OPCODE_STATUS   0x10
#define DNS_FLAG1_OPCODE_INVERSE  0x08
#define DNS_FLAG1_OPCODE_STANDARD 0x00
#define DNS_FLAG1_AUTHORATIVE     0x04
#define DNS_FLAG1_TRUNC           0x02
#define DNS_FLAG1_RD              0x01
#define DNS_FLAG2_RA              0x80
#define DNS_FLAG2_ERR_MASK        0x0f
#define DNS_FLAG2_ERR_NONE        0x00
#define DNS_FLAG2_ERR_NAME        0x03

#define SEND_BUFFER_SIZE 64

#ifdef CONFIG_NETUTILS_DNSCLIENT_MAXRESPONSE
#  define RECV_BUFFER_SIZE CONFIG_NETUTILS_DNSCLIENT_MAXRESPONSE
#else
#  define RECV_BUFFER_SIZE 96
#endif

#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
#  define ADDRLEN sizeof(struct sockaddr_in6)
#else
#  define ADDRLEN sizeof(struct sockaddr_in)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The DNS message header */

struct dns_hdr
{
  uint16_t id;
  uint8_t  flags1;
  uint8_t  flags2;
  uint16_t numquestions;
  uint16_t numanswers;
  uint16_t numauthrr;
  uint16_t numextrarr;
};

/* The DNS answer message structure */

struct dns_answer
{
  /* DNS answer record starts with either a domain name or a pointer
   * to a name already present somewhere in the packet.
   */

  uint16_t type;
  uint16_t class;
  uint16_t ttl[2];
  uint16_t len;
};

/* The DNS question message structure */

struct dns_question
{
  uint16_t type;
  uint16_t class;
};

struct namemap
{
  uint8_t state;
  uint8_t tmr;
  uint8_t retries;
  uint8_t seqno;
  uint8_t err;
  char name[32];
#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
  struct in6_addr ipaddr;
#else
  struct in_addr ipaddr;
#endif
};

/* Query info to check response against. */

struct dns_queue_info_s
{
  uint16_t id;
#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
  struct in6_addr srv_ip;
#else
  struct in_addr srv_ip;
#endif
  uint16_t srv_port;
  void *qname;
  size_t qnamelen;
  struct dns_question que;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint16_t g_seqno;
#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
static struct sockaddr_in6 g_dnsserver;
#else
static struct sockaddr_in g_dnsserver;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: crc8_in16
 ****************************************************************************/

static uint8_t crc8_in16(uint16_t data)
{
  static const uint8_t crc_table[16] =
  {
    0x00, 0x1c, 0x38, 0x24, 0x70, 0x6c, 0x48, 0x54,
    0xe0, 0xfc, 0xd8, 0xc4, 0x90, 0x8c, 0xa8, 0xb4
  };
  uint8_t crc;

  crc = (data >> 0) & 0xff;
  crc = (crc >> 4) ^ crc_table[crc & 15];
  crc = (crc >> 4) ^ crc_table[crc & 15];

  crc ^= (data >> 8) & 0xff;
  crc = (crc >> 4) ^ crc_table[crc & 15];
  crc = (crc >> 4) ^ crc_table[crc & 15];

  return crc;
}

/****************************************************************************
 * Name: transform_16bit
 ****************************************************************************/

static uint16_t transform_16bit(uint16_t ctr)
{
  static struct
  {
    bool init;
    uint16_t rand[3];
  } g;
  uint8_t left = ctr & 0xff;
  uint8_t right = (ctr >> 8) & 0xff;

  /* Generate ID bit more random fashion. */

  if (!g.init)
    {
      speckrandom_buf(&g.rand, sizeof(g.rand));
      g.init = true;
    }

  /* Bijective transformation uint16_t -> uint16_t */

  left  += crc8_in16(g.rand[0] - (right << 1));
  right += crc8_in16(g.rand[1] - (left << 2));
  left  += crc8_in16(g.rand[2] - (right << 3));

  return (right + (left << 8));
}

/****************************************************************************
 * Name: buf_left
 *
 * Description:
 *   Calculate how many bytes buffer has left.
 *
 ****************************************************************************/

static ssize_t buf_left(FAR void *vpos, FAR const void *vbase, ssize_t baselen)
{
  FAR unsigned char *pos = vpos;
  FAR const unsigned char *base = vbase;

  return baselen - (ssize_t)(pos - base);
}

/****************************************************************************
 * Name: dns_parse_name
 *
 * Description:
 *   Walk through a compact encoded DNS name and return the end of it.
 *
 ****************************************************************************/

static FAR unsigned char *dns_parse_name(FAR unsigned char *query,
                                         FAR const unsigned char *base,
                                         ssize_t baselen)
{
  unsigned char n;

  do
    {
      if (buf_left(query, base, baselen) < 1)
        {
          return query;
        }

      n = *query++;

      while (n > 0)
        {
          ++query;
          --n;
        }

      if (buf_left(query, base, baselen) < 1)
        {
          return query;
        }
    }
  while (*query != 0);

  return query + 1;
}

/****************************************************************************
 * Name: dns_send_query
 *
 * Description:
 *   Runs through the list of names to see if there are any that have
 *   not yet been queried and, if so, sends out a query.
 *
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
static int dns_send_query(int sockfd, FAR const char *name,
                          FAR struct sockaddr_in6 *addr,
                          FAR unsigned char *buffer, size_t buflen,
                          struct dns_queue_info_s *qinfo)
#else
static int dns_send_query(int sockfd, FAR const char *name,
                          FAR struct sockaddr_in *addr,
                          FAR unsigned char *buffer, size_t buflen,
                          struct dns_queue_info_s *qinfo)
#endif
{
  FAR struct dns_hdr *hdr;
  FAR char *query;
  FAR char *namestart;
  FAR char *nameend;
  FAR char *nptr;
  FAR const char *nameptr;
  uint16_t seqno;
  struct dns_question que;
  size_t namebuflen;
  FAR void *namebuf;
  ssize_t wlen;
  int n;

  qinfo->qname = NULL;

  seqno = transform_16bit(g_seqno++);

  hdr               = (FAR struct dns_hdr*)buffer;
  memset(hdr, 0, sizeof(*hdr));
  hdr->id           = htons(seqno);
  hdr->flags1       = DNS_FLAG1_RD;
  hdr->numquestions = HTONS(1);
  query             = (FAR char *)buffer + sizeof(*hdr);

  /* Convert hostname into suitable query format. */

  nameptr = name - 1;
  namestart = query;
  do
   {
     if (buf_left(query, buffer, buflen) < 1)
       {
         /* Too long name! */

         errno = EMSGSIZE;
         return ERROR;
       }

     nameptr++;
     nptr = query++;
     for (n = 0; *nameptr != '.' && *nameptr != 0; nameptr++)
       {
         if (buf_left(query, buffer, buflen) < 1)
           {
             /* Too long name! */

             errno = EMSGSIZE;
             return ERROR;
           }

         *query++ = *nameptr;
         n++;
       }

     *nptr = n;
   }
  while (*nameptr != 0);

  if (buf_left(query, buffer, buflen) < 1)
    {
      /* Too long name! */

      errno = EMSGSIZE;
      return ERROR;
    }

  *query++ = '\0';
  nameend = query;

  /* Add question type and class (IPv4 type and Internet class). */

  que.type = HTONS(1);
  que.class = HTONS(1);

  if (buf_left(query, buffer, buflen) < 4)
    {
      /* Too long name! */

      errno = EMSGSIZE;
      return ERROR;
    }

  memcpy(query, &que, sizeof(que));
  query += sizeof(struct dns_question);

  /* Store query info for verifying response. */

  namebuflen = (nameend - namestart);
  namebuf = malloc(namebuflen);
  if (namebuf == NULL)
    {
      /* Out of memory. */

      errno = ENOMEM;
      return ERROR;
    }

  memcpy(namebuf, namestart, namebuflen);

  qinfo->id = hdr->id;
  qinfo->srv_ip = addr->sin_addr;
  qinfo->srv_port = addr->sin_port;
  qinfo->qname = namebuf;
  qinfo->qnamelen = namebuflen;
  qinfo->que = que;

  /* Send the request. */

  wlen = sendto(sockfd, buffer, query - (FAR char *)buffer,
                0, (struct sockaddr*)addr, ADDRLEN);
  if (wlen < 0)
    {
      int err = errno;
      free(qinfo->qname);
      qinfo->qname = NULL;
      errno = err;
    }

  return wlen;
}

/****************************************************************************
 * Name: dns_recv_response
 *
 * Description:
 *   Called when new UDP data arrives
 *
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
#  error "Not implemented"
#else
static int dns_recv_response(int sockfd, FAR struct sockaddr_in *addr,
                             FAR unsigned char *buffer, size_t buflen,
                             struct dns_queue_info_s *qinfo)
#endif
{
  struct sockaddr_in recvaddr;
  FAR unsigned char *nameptr;
  FAR unsigned char *namestart;
  FAR unsigned char *nameend;
  FAR struct dns_answer *ans;
  FAR struct dns_hdr *hdr;
  uint8_t nquestions;
  uint8_t nanswers;
  ssize_t ret;
  socklen_t raddrlen;

  /* Receive the response */

  raddrlen = sizeof(recvaddr);
  ret = recvfrom(sockfd, buffer, buflen, 0, (void *)&recvaddr, &raddrlen);
  if (ret < 0)
    {
      return ret;
    }

  if (memcmp(&recvaddr.sin_addr, &qinfo->srv_ip, sizeof(recvaddr.sin_addr)))
    {
      /* Not response from DNS server. */

      ndbg("packet from wrong address\n");
      errno = EBADMSG;
      return ERROR;
    }

  if (recvaddr.sin_port != qinfo->srv_port)
    {
      /* Not response from DNS server. */

      ndbg("packet from wrong port\n");
      errno = EBADMSG;
      return ERROR;
    }

  buflen = ret;

  if (buflen < sizeof(*hdr))
    {
      ndbg("too short DNS response (len: %d, expect at least: %d)\n",
           buflen, sizeof(*hdr));

      errno = EBADMSG;
      return ERROR;
    }

  hdr = (FAR struct dns_hdr *)buffer;

  ndbg("ID %d\n", htons(hdr->id));
  ndbg("Query %d\n", hdr->flags1 & DNS_FLAG1_RESPONSE);
  ndbg("Error %d\n", hdr->flags2 & DNS_FLAG2_ERR_MASK);
  ndbg("Num questions %d, answers %d, authrr %d, extrarr %d\n",
       htons(hdr->numquestions), htons(hdr->numanswers),
       htons(hdr->numauthrr), htons(hdr->numextrarr));

  /* Check for error. If so, call callback to inform */

  if ((hdr->flags2 & DNS_FLAG2_ERR_MASK) != 0)
    {
      errno = EHOSTUNREACH;
      return ERROR;
    }

  /* Check for matching ID. */

  if (hdr->id != qinfo->id)
    {
      ndbg("wrong DNS response ID (expected %d, got %d).\n",
           qinfo->id, hdr->id);

      errno = EBADMSG;
      return ERROR;
    }

  /* We only care about the question(s) and the answers. The authrr
   * and the extrarr are simply discarded.
   */

  nquestions = htons(hdr->numquestions);
  nanswers   = htons(hdr->numanswers);

  if (nquestions != 1)
    {
      ndbg("wrong number of questions (expected %d, got %d).\n", 1, nquestions);

      errno = EBADMSG;
      return ERROR;
    }

#ifdef CONFIG_DEBUG_NET
  {
    unsigned char *oldptr = nameptr;
    int d = 64;

    nameptr = (unsigned char *)buffer + sizeof(*hdr);
    nameptr = dns_parse_name(nameptr, buffer, buflen);

    if (buf_left(nameptr, buffer, buflen) >= 4)
      {
        nameptr += sizeof(struct dns_question);

        for (;;)
          {
            ndbg("%02X %02X %02X %02X %02X %02X %02X %02X \n",
                 nameptr[0],nameptr[1],nameptr[2],nameptr[3],
                 nameptr[4],nameptr[5],nameptr[6],nameptr[7]);

            nameptr += 8;
            d -= 8;
            if (d < 0)
              {
                break;
              }
          }
      }

    nameptr = oldptr;
  }
#endif

  /* Check that the name in response matches against the name in the question.
   */

  namestart = (unsigned char *)buffer + sizeof(*hdr);
  nameend = dns_parse_name(namestart, buffer, buflen);

  if ((nameend - namestart) != qinfo->qnamelen)
    {
      ndbg("invalid DNS response, response name different length than query.\n");

      errno = EBADMSG;
      return ERROR;
    }

  if (memcmp(namestart, qinfo->qname, qinfo->qnamelen))
    {
      ndbg("invalid DNS response, response name mismatch query.\n");

      errno = EBADMSG;
      return ERROR;
    }

  nameptr = nameend + sizeof(struct dns_question);
  if (buf_left(nameptr, buffer, buflen) <= 0)
    {
      ndbg("malformed DNS response, no answers section.\n");

      errno = EBADMSG;
      return ERROR;
    }

  /* Check that question type in request matches what was asked. */

  if (memcmp((struct dns_question *)nameend, &qinfo->que,
             sizeof(struct dns_question)))
    {
      ndbg("invalid DNS response, question section mismatch.\n");

      errno = EBADMSG;
      return ERROR;
    }

  /* Go through answers. */

  for (; nanswers > 0; nanswers--)
    {
      if (buf_left(nameptr, buffer, buflen) < 1)
        {
          ndbg("malformed/truncated DNS response.\n");

          errno = EBADMSG;
          break;
        }

      /* The first byte in the answer resource record determines if it
       * is a compressed record or a normal one.
       */

      if (*nameptr & 0xc0)
        {
          /* Compressed name. */

          nameptr += 2;
          ndbg("Compressed answer\n");
        }
      else
        {
          /* Not compressed name. */

          nameptr = dns_parse_name(nameptr, buffer, buflen);
        }

      if (buf_left(nameptr, buffer, buflen) < sizeof(*ans))
        {
          ndbg("malformed/truncated DNS response.\n");

          errno = EBADMSG;
          break;
        }

      ans = (struct dns_answer *)nameptr;
      ndbg("Answer: type %x, class %x, ttl %x, length %x \n", /* 0x%08X\n", */
           htons(ans->type), htons(ans->class),
           (htons(ans->ttl[0]) << 16) | htons(ans->ttl[1]),
           htons(ans->len) /* , ans->ipaddr.s_addr */);

      /* Check for IP address type and Internet class. Others are discarded. */

      if (ans->type  == HTONS(1) &&
          ans->class == HTONS(1) &&
          ans->len   == HTONS(4))
        {
          in_addr_t a_addr;

          nameptr += sizeof(*ans);

          if (buf_left(nameptr, buffer, buflen) < sizeof(in_addr_t))
            {
              ndbg("malformed/truncated DNS response.\n");

              errno = EBADMSG;
              break;
            }

          a_addr = *(FAR in_addr_t *)nameptr;

          ndbg("IP address %d.%d.%d.%d\n",
               (a_addr       ) & 0xff,
               (a_addr >> 8  ) & 0xff,
               (a_addr >> 16 ) & 0xff,
               (a_addr >> 24 ) & 0xff);

          /* TODO: we should really check that this IP address is the one
           * we want.
           */

          addr->sin_addr.s_addr = a_addr;
          return OK;
        }
#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
      else if (ans->type  == HTONS(1) &&
               ans->class == HTONS(28) &&
               ans->len   == HTONS(16))
        {
          /* TODO: IPv6 */

          nameptr += sizeof(*ans) + 16;
        }
#endif
      else
        {
          nameptr += sizeof(*ans) + htons(ans->len);
        }
    }

  errno = EHOSTUNREACH;
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_bind_sock
 *
 * Description:
 *   Initialize the DNS resolver using the caller provided socket.
 *
 ****************************************************************************/

int dns_bind_sock(FAR int *sockfd)
{
  struct timeval tv;
  int ret;
  int err;

  /* If the socket is already open, then close it now */

  if (*sockfd >= 0)
    {
      dns_free_sock(sockfd);
    }

  /* Create a new socket */

  *sockfd = socket(PF_INET, SOCK_DGRAM, 0);
  if (*sockfd < 0)
    {
      err = errno;
      ndbg("ERROR: socket() failed: %d\n", errno);
      errno = err;
      return ERROR;
    }

  /* Set up a receive timeout */

  tv.tv_sec  = CONFIG_NETUTILS_DNSCLIENT_RECV_TIMEOUT;
  tv.tv_usec = 0;

  ret = setsockopt(*sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv,
                   sizeof(struct timeval));

  if (ret < 0)
    {
      err = errno;
      ndbg("ERROR: setsockopt() failed: %d\n", errno);
      close(*sockfd);
      *sockfd = -1;
      errno = err;
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: dns_free_sock
 *
 * Description:
 *   Release the DNS resolver by closing the socket.
 *
 ****************************************************************************/

int dns_free_sock(FAR int *sockfd)
{
  if (*sockfd >= 0)
    {
      close(*sockfd);
      *sockfd = -1;
    }

  return OK;
}

/****************************************************************************
 * Name: dns_query_sock
 *
 * Description:
 *   Using the DNS resolver socket (sockfd), look up the the 'hostname', and
 *   return its IP address in 'ipaddr'
 *
 * Returned Value:
 *   Returns zero (OK) if the query was successful.
 *
 ****************************************************************************/

int dns_query_sock(int sockfd, FAR const char *hostname, FAR in_addr_t *ipaddr)
{
#ifdef CONFIG_HAVE_GETHOSTBYNAME

  FAR struct hostent *he;

  nvdbg("Getting address of %s\n", hostname);
  he = gethostbyname(hostname);
  if (!he)
    {
      ndbg("gethostbyname failed: %d\n", h_errno);
      return ERROR;
    }

  nvdbg("Using IP address %04x%04x\n",
       (uint16_t)he->h_addr[1], (uint16_t)he->h_addr[0]);

  memcpy(ipaddr, he->h_addr, sizeof(in_addr_t));
  return OK;

#else

# ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
  struct sockaddr_in6 addr;
# else
  struct sockaddr_in addr;
# endif

  /* First check if the host is an IP address. */

  if (!netlib_ipaddrconv(hostname, (uint8_t*)ipaddr))
    {
      /* 'host' does not point to a valid address string.  Try to resolve
       *  the host name to an IP address.
       */

      if (dns_whois_socket(sockfd, hostname, &addr) < 0)
        {
          /* Needs to set the errno here */

          return ERROR;
        }

      /* Save the host address -- Needs fixed for IPv6 */

      *ipaddr = addr.sin_addr.s_addr;
  }

  return OK;
#endif
}

/****************************************************************************
 * Name: dns_setserver
 *
 * Description:
 *   Configure which DNS server to use for queries
 *
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
void dns_setserver(FAR const struct in6_addr *dnsserver)
#else
void dns_setserver(FAR const struct in_addr *dnsserver)
#endif
{
#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
  g_dnsserver.sin6_family = AF_INET6;
  g_dnsserver.sin6_port   = HTONS(53);

  memcpy(&g_dnsserver.sin6_addr, dnsserver, ADDRLEN);
#else
  g_dnsserver.sin_family  = AF_INET;
  g_dnsserver.sin_port    = HTONS(53);

  g_dnsserver.sin_addr.s_addr = dnsserver->s_addr;
#endif
}

/****************************************************************************
 * Name: dns_getserver
 *
 * Description:
 *   Obtain the currently configured DNS server.
 *
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
void dns_getserver(FAR struct in6_addr *dnsserver)
#else
void dns_getserver(FAR struct in_addr *dnsserver)
#endif
{
#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
  memcpy(dnsserver, &g_dnsserver.sin6_addr, ADDRLEN);
#else
  dnsserver->s_addr = g_dnsserver.sin_addr.s_addr;
#endif
}

/****************************************************************************
 * Name: dns_whois_socket
 *
 * Description:
 *   Get the binding for 'name' using the DNS server accessed via 'sockfd'
 *
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
int dns_whois_socket(int sockfd, FAR const char *name,
                     FAR struct sockaddr_in6 *addr)
#else
int dns_whois_socket(int sockfd, FAR const char *name,
                     FAR struct sockaddr_in *addr)
#endif
{
  FAR unsigned char *buffer;
  size_t buflen = RECV_BUFFER_SIZE;
  int retries;
  int ret;
  int err;

  if (buflen < SEND_BUFFER_SIZE)
    buflen = SEND_BUFFER_SIZE;

  buffer = malloc(buflen);
  if (!buffer)
    {
      return ERROR;
    }

  /* Loop while receive timeout errors occur and there are remaining retries */

  for (retries = 0; retries < CONFIG_NETUTILS_DNSCLIENT_RETRIES; retries++)
    {
      struct dns_queue_info_s qinfo = {};

      ret = dns_send_query(sockfd, name, &g_dnsserver, buffer, buflen,
                           &qinfo);
      if (ret < 0)
        {
          err = errno;
          free(qinfo.qname);
          qinfo.qname = NULL;
          free(buffer);
          errno = err;
          return ERROR;
        }

      ret = dns_recv_response(sockfd, addr, buffer, buflen, &qinfo);
      if (ret >= 0)
        {
          /* Response received successfully */

          free(qinfo.qname);
          qinfo.qname = NULL;
          free(buffer);
          return OK;
        }
      else if (errno != EAGAIN)
        {
          /* Some failure other than receive timeout occurred */

          err = errno;
          free(qinfo.qname);
          qinfo.qname = NULL;
          free(buffer);
          errno = err;
          return ERROR;
        }

      free(qinfo.qname);
      qinfo.qname = NULL;
    }

  free(buffer);
  errno = EHOSTUNREACH;
  return ERROR;
}
