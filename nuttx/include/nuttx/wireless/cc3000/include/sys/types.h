/*****************************************************************************
 *  types.h  - CC3000 Host Driver Implementation.
 *  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_CC3000_INCLUDE_SYS_TYPES_H
#define __INCLUDE_NUTTX_WIRELESS_CC3000_INCLUDE_SYS_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

#ifdef  __cplusplus
extern "C" {
#endif

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/*--------- Address Families --------*/

#define  CC3000_AF_INET                2
#define  CC3000_AF_INET6               23

/*------------ Socket Types ------------*/

#define  CC3000_SOCK_STREAM     1
#define  CC3000_SOCK_DGRAM      2
#define  CC3000_SOCK_RAW        3           /* Raw sockets allow new IPv4
                                             * protocols to be implemented in
                                             * user space. A raw socket receives
                                             * or sends the raw datagram not
                                             * including link level headers */
#define  CC3000_SOCK_RDM        4
#define  CC3000_SOCK_SEQPACKET  5

/*----------- Socket Protocol ----------*/

#define CC3000_IPPROTO_IP              0                        /* Dummy for IP */
#define CC3000_IPPROTO_ICMP            1                        /* Control message protocol */
#define CC3000_IPPROTO_IPV4            CC3000_IPPROTO_IP        /* IP inside IP */
#define CC3000_IPPROTO_TCP             6                        /* TCP */
#define CC3000_IPPROTO_UDP             17                       /* User datagram protocol */
#define CC3000_IPPROTO_IPV6            41                       /* IPv6 in IPv6 */
#define CC3000_IPPROTO_NONE            59                       /* No next header */
#define CC3000_IPPROTO_TX_TEST_RAW     150                      /* Raw 802.11 Tx Test packet */
#define CC3000_IPPROTO_RAW             255                      /* Raw IP packet */
#define CC3000_IPPROTO_MAX             256

/*----------- Socket return codes  -----------*/

#define CC3000_SOC_ERROR               (-1)        /* Error  */
#define CC3000_SOC_IN_PROGRESS         (-2)        /* Socket in progress */

/*----------- Socket Options -----------*/
#define  CC3000_SOL_SOCKET              0xffff     /* Socket level */
#define  CC3000_SOCKOPT_RECV_NONBLOCK   0          /* recv non block mode, set SOCK_ON or
                                                    * SOCK_OFF (default block mode) */
#define  CC3000_SOCKOPT_RECV_TIMEOUT    1          /* optname to configure recv and recvfromtimeout */
#define  CC3000_SOCKOPT_ACCEPT_NONBLOCK 2          /* accept non block mode, set SOCK_ON or SOCK_OFF
                                                    * (default block mode) */
#define  CC3000_SOCK_ON                 0          /* socket non-blocking mode  is enabled */
#define  CC3000_SOCK_OFF                1          /* socket blocking mode is enabled */

#define  CC3000_TCP_NODELAY             0x0001
#define  CC3000_TCP_BSDURGENT           0x7000

#define  CC3000_MAX_PACKET_SIZE         1500
#define  CC3000_MAX_LISTEN_QUEUE        4

#define  CC3000_IOCTL_SOCKET_EVENTMASK

#define  CC3000_ASIC_ADDR_LEN           8

#define CC3000_NO_QUERY_RECIVED        -3

/* Access macros for 'TICC3000fd_set' */

#define CC3000_FD_SETSIZE        32

#define __TICC3000_NFDBITS       (8 * sizeof (__TICC3000fd_mask))
#define __TICC3000_FDELT(d)      ((d) / __TICC3000_NFDBITS)
#define __TICC3000_FDMASK(d)     ((__TICC3000fd_mask) 1 << ((d) % __TICC3000_NFDBITS))

#define __TICC3000_FDS_BITS(set) ((set)->fds_bits)

/* We don't use `memset' because this would require a prototype and
 *   the array isn't too big.
 */

#define CC3000_FD_ZERO(set) \
  do { \
    unsigned int __i; \
    TICC3000fd_set *__arr = (set); \
    for (__i = 0; __i < sizeof (TICC3000fd_set) / sizeof (__TICC3000fd_mask); ++__i) \
      __TICC3000_FDS_BITS (__arr)[__i] = 0; \
  } while (0)
#define CC3000_FD_SET(d, set)   (__TICC3000_FDS_BITS (set)[__TICC3000_FDELT (d)] |= __TICC3000_FDMASK (d))
#define CC3000_FD_CLR(d, set)   (__TICC3000_FDS_BITS (set)[__TICC3000_FDELT (d)] &= ~__TICC3000_FDMASK (d))
#define CC3000_FD_ISSET(d, set) (__TICC3000_FDS_BITS (set)[__TICC3000_FDELT (d)] & __TICC3000_FDMASK (d))

/*****************************************************************************
 * Public Types
 *****************************************************************************/

/* The fd_set member is required to be an array of longs. */

typedef long int __TICC3000fd_mask;

/* fd_set for select and pselect. */

typedef struct
{
  __TICC3000fd_mask fds_bits[CC3000_FD_SETSIZE / __TICC3000_NFDBITS];
} TICC3000fd_set;

#ifdef  __cplusplus
}
#endif // __cplusplus

#endif // __INCLUDE_NUTTX_WIRELESS_CC3000_INCLUDE_SYS_TYPES_H
