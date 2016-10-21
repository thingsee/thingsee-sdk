/****************************************************************************
 * apps/include/netutils/dnsclient.h
 * DNS resolver code header file.
 * Author Adam Dunkels <adam@dunkels.com>
 *
 *   Copyright (C) 2007-2009, 2011-2012, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Inspired by/based on uIP logic by Adam Dunkels:
 *
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

#ifndef __APPS_INCLUDE_NETUTILS_DNSCLIENT_H
#define __APPS_INCLUDE_NETUTILS_DNSCLIENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/netconfig.h>

#include <netinet/in.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* If both IPv4 and IPv6 are enabled, the DNS client can support only one or
 * the other.
 */

#if !defined(CONFIG_NETUTILS_DNSCLIENT_IPv4) && \
    !defined(CONFIG_NETUTILS_DNSCLIENT_IPv6)
#  ifdef CONFIG_NET_IPv6
#     define CONFIG_NETUTILS_DNSCLIENT_IPv6 1
#  else
#     define CONFIG_NETUTILS_DNSCLIENT_IPv4 1
#  endif
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: dns_bind_sock
 *
 * Description:
 *   Initialize the DNS resolver using the caller provided socket.
 *
 ****************************************************************************/

int dns_bind_sock(FAR int *sockfd);

/****************************************************************************
 * Name: dns_bind
 *
 * Description:
 *   Initialize the DNS resolver using an internal, share-able socket.
 *
 ****************************************************************************/

int dns_bind(void);

/****************************************************************************
 * Name: dns_free_sock
 *
 * Description:
 *   Release the DNS resolver by closing the socket.
 *
 ****************************************************************************/

int dns_free_sock(FAR int *sockfd);

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

int dns_query_sock(int sockfd, FAR const char *hostname, FAR in_addr_t *ipaddr);

/****************************************************************************
 * Name: dns_query
 *
 * Description:
 *   Using the DNS resolver socket (sockfd), look up the the 'hostname', and
 *   return its IP addresses in 'ipaddr' array.
 *
 * Returned Value:
 *   Returns number of addresses read if the query was successful.
 *
 ****************************************************************************/

int dns_query_sock_multi(int sockfd, FAR const char *hostname,
                         FAR in_addr_t *ipaddr, size_t nipaddr);

/****************************************************************************
 * Name: dns_query
 *
 * Description:
 *   Using the internal DNS resolver socket, look up the the 'hostname', and
 *   return its IP address in 'ipaddr'
 *
 * Returned Value:
 *   Returns zero (OK) if the query was successful.
 *
 ****************************************************************************/

int dns_query(FAR const char *hostname, FAR in_addr_t *ipaddr);

/****************************************************************************
 * Name: dns_query
 *
 * Description:
 *   Using the internal DNS resolver socket, look up the the 'hostname', and
 *   return its IP addresses in 'ipaddr' array.
 *
 * Returned Value:
 *   Returns number of addresses read if the query was successful.
 *
 ****************************************************************************/

int dns_query_multi(FAR const char *hostname, FAR in_addr_t *ipaddr,
                    size_t nipaddr);

/****************************************************************************
 * Name: dns_setservers
 *
 * Description:
 *   Configure which DNS servers to use for queries
 *
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
int dns_setservers(FAR const struct in6_addr *dnsserver1,
                   FAR const struct in6_addr *dnsserver2,
                   FAR const struct in6_addr *dnsserver3);
#else
int dns_setservers(FAR const struct in_addr *dnsserver1,
                   FAR const struct in_addr *dnsserver2,
                   FAR const struct in_addr *dnsserver3);
#endif

/****************************************************************************
 * Name: dns_setserver
 *
 * Description:
 *   Configure which DNS server to use for queries
 *
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
void dns_setserver(FAR const struct in6_addr *dnsserver);
#else
void dns_setserver(FAR const struct in_addr *dnsserver);
#endif

/****************************************************************************
 * Name: dns_getserver_sockaddr
 *
 * Description:
 *   Obtain the currently configured DNS server.
 *
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
int dns_getserver_sockaddr(FAR struct sockaddr_in6 *dnsserver);
#else
int dns_getserver_sockaddr(FAR struct sockaddr_in *dnsserver);
#endif

/****************************************************************************
 * Name: dns_getserver
 *
 * Description:
 *   Obtain the currently configured DNS server.
 *
 ****************************************************************************/

#ifdef CONFIG_NETUTILS_DNSCLIENT_IPv6
int dns_getserver(FAR struct in6_addr *dnsserver);
#else
int dns_getserver(FAR struct in_addr *dnsserver);
#endif

/****************************************************************************
 * Name: dns_whois_socket
 *
 * Description:
 *   Get the binding for 'name' using the DNS server accessed via 'sockfd'
 *
 ****************************************************************************/

int dns_whois_socket_multi(int sockfd, FAR const char *name,
                           FAR in_addr_t *addr, size_t naddr);

/****************************************************************************
 * Name: dns_whois_socket
 *
 * Description:
 *   Get the binding for 'name' using the DNS server accessed via 'sockfd'.
 *
 ****************************************************************************/

int dns_whois_socket(int sockfd, FAR const char *name,
                     FAR struct sockaddr_in *addr);

/****************************************************************************
 * Name: dns_whois
 *
 * Description:
 *   Get the binding for 'name' using the DNS server accessed via the DNS
 *   resolvers internal socket.
 *
 ****************************************************************************/

int dns_whois(FAR const char *name, FAR struct sockaddr_in *addr);

/****************************************************************************
 * Name: dns_gethostip
 *
 * Descriptions:
 *   Combines the operations of dns_bind_sock(), dns_query_sock(), and
 *   dns_free_sock() to obtain the the IP address ('ipaddr') associated with
 *   the 'hostname' in one operation.
 *
 ****************************************************************************/

int dns_gethostip(FAR const char *hostname, FAR in_addr_t *ipaddr);

/****************************************************************************
 * Name: dns_gethostip_multi
 *
 * Descriptions:
 *   Combines the operations of dns_bind_sock(), dns_query_sock_multi(), and
 *   dns_free_sock() to obtain the the IP addresses ('ipaddr') associated with
 *   the 'hostname' in one operation. Allows fetching multiple IP-addresses
 *   from single DNS query response. Returns number of addresses read.
 *
 ****************************************************************************/

int dns_gethostip_multi(FAR const char *hostname, FAR in_addr_t *ipaddr,
                        size_t nipaddr);

/****************************************************************************
 * Name: dns_clear_lookup_failed_count
 ****************************************************************************/

void dns_clear_lookup_failed_count(void);

/****************************************************************************
 * Name: dns_increase_lookup_failed_count
 ****************************************************************************/

void dns_increase_lookup_failed_count(void);

/****************************************************************************
 * Name: dns_get_lookup_failed_count
 ****************************************************************************/

unsigned int dns_get_lookup_failed_count(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __APPS_INCLUDE_NETUTILS_DNSCLIENT_H */
