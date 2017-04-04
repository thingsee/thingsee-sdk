#ifndef _LIBCOAP_CONFIG_H_NUTTX_
#define _LIBCOAP_CONFIG_H_NUTTX_

#include <nuttx/config.h>
#include <debug.h>

#define WITH_NUTTX 1

#ifdef CONFIG_LIBCOAP_CUSTOM_IO
#  define CUSTOM_COAP_NETWORK_ENDPOINT          1
#  define CUSTOM_COAP_NETWORK_SEND              1
#  define CUSTOM_COAP_NETWORK_READ              1
#endif

#undef NDEBUG
#ifdef CONFIG_LIBCOAP_DISABLE_DEBUG
#  define NDEBUG                                1
#endif

#define coap_log(level, ...) dbg(__VA_ARGS__)

#define PACKAGE_NAME "libcoap"
#define PACKAGE_STRING "libcoap"
#define PACKAGE_VERSION "4.1.2-beta"

/* Provide custom network layer for libcoap with nuttx integration. */
//#define CUSTOM_COAP_NETWORK_ENDPOINT 1
//#define CUSTOM_COAP_NETWORK_SEND 1
//#define CUSTOM_COAP_NETWORK_READ 1

/* Timeout for CoAP response (larger than default 2 seconds needed for mobile
 * environment, extra ~1-3 seconds wake-up for modem from idle network state).
 */
#define COAP_DEFAULT_RESPONSE_TIMEOUT 5

/* PDU size large enough for 600 byte payload + 64 byte coap header. */
#define COAP_MAX_PDU_SIZE 664
#define COAP_MAX_BLOCK_SZX 5

/* Define to 1 if you have the <arpa/inet.h> header file. */
#define HAVE_ARPA_INET_H 1

/* Define to 1 if you have the <assert.h> header file. */
#define HAVE_ASSERT_H 1

/* Define to 1 if you have the `getaddrinfo' function. */
#undef HAVE_GETADDRINFO

/* Define to 1 if you have the <inttypes.h> header file. */
#define HAVE_INTTYPES_H 1

/* Define to 1 if you have the `coap' library (-lcoap). */
#undef HAVE_LIBCOAP

/* Define to 1 if you have the <limits.h> header file. */
#define HAVE_LIMITS_H 1

/* Define to 1 if your system has a GNU libc compatible `malloc' function, and
   to 0 otherwise. */
#define HAVE_MALLOC 1

/* Define to 1 if you have the <memory.h> header file. */
#define HAVE_MEMORY_H 1

/* Define to 1 if you have the `memset' function. */
#define HAVE_MEMSET 1

/* Define to 1 if you have the <netdb.h> header file. */
#undef HAVE_NETDB_H

/* Define to 1 if you have the <netinet/in.h> header file. */
#define HAVE_NETINET_IN_H 1

/* Define to 1 if you have the `select' function. */
#define HAVE_SELECT 1

/* Define to 1 if you have the `socket' function. */
#define HAVE_SOCKET 1

/* Define to 1 if you have the <stdint.h> header file. */
#define HAVE_STDINT_H 1

/* Define to 1 if you have the <stdlib.h> header file. */
#define HAVE_STDLIB_H 1

/* Define to 1 if you have the `strcasecmp' function. */
#define HAVE_STRCASECMP 1

/* Define to 1 if you have the <strings.h> header file. */
#define HAVE_STRINGS_H 1

/* Define to 1 if you have the <string.h> header file. */
#define HAVE_STRING_H 1

/* Define to 1 if you have the `strnlen' function. */
#define HAVE_STRNLEN 1

/* Define to 1 if you have the `fls' function. */
#define HAVE_FLS 1

/* Define to 1 if you have the `strrchr' function. */
#define HAVE_STRRCHR 1

/* Define to 1 if you have the <syslog.h> header file. */
#define HAVE_SYSLOG_H 1

/* Define to 1 if you have the <sys/socket.h> header file. */
#define HAVE_SYS_SOCKET_H 1

/* Define to 1 if you have the <sys/stat.h> header file. */
#define HAVE_SYS_STAT_H 1

/* Define to 1 if you have the <sys/time.h> header file. */
#define HAVE_SYS_TIME_H 1

/* Define to 1 if you have the <sys/types.h> header file. */
#define HAVE_SYS_TYPES_H 1

/* Define to 1 if you have the <sys/unistd.h> header file. */
#define HAVE_SYS_UNISTD_H 1

/* Define to 1 if you have the <time.h> header file. */
#define HAVE_TIME_H 1

/* Define to 1 if you have the <unistd.h> header file. */
#define HAVE_UNISTD_H 1

/* Define to 1 if you have the ANSI C header files. */
#undef STDC_HEADERS

#endif /* _LIBCOAP_CONFIG_H_NUTTX_ */
