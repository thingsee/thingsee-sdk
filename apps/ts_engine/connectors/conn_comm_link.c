/****************************************************************************
 * apps/ts_engine/connectors/conn_comm_link.c
 *
 * Copyright (C) 2015 Haltian Ltd. All rights reserved.
 * Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>

#include "con_dbg.h"
#include "conn_comm_util.h"

#ifdef CONFIG_THINGSEE_HTTPS_PROTOCOL
# ifdef CONFIG_MBEDTLS
#  include "mbedtls/net.h"
#  include "mbedtls/ssl.h"
#  include "mbedtls/entropy.h"
#  include "mbedtls/ctr_drbg.h"
#  include "mbedtls/hmac_drbg.h"
#  include "mbedtls/platform.h"
#  include "mbedtls/ssl_internal.h"
#  include "mbedtls/asn1.h"
#  include "mbedtls/bignum.h"
#  include "mbedtls/dhm.h"
# endif
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_THINGSEE_HTTPS_PROTOCOL_SESSION_TIMEOUT
#  define CONFIG_THINGSEE_HTTPS_PROTOCOL_SESSION_TIMEOUT (24 * 60 * 60)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct conn_link_s
{
#ifdef CONFIG_THINGSEE_HTTPS_PROTOCOL
#ifdef CONFIG_MBEDTLS
  mbedtls_ssl_context *ssl;
  mbedtls_net_context net;
#endif
#endif
  int sock;
  uint32_t send_timeout_ms;
  uint32_t recv_timeout_ms;
};

#ifdef CONFIG_THINGSEE_HTTPS_PROTOCOL
struct https_data_s
{
  bool initialized;
#ifdef CONFIG_MBEDTLS
  bool ssl_inbuf_in_use;

  struct {
    mbedtls_entropy_context entropy;
#ifdef CONFIG_MBEDTLS_ENABLE_CTR_DRBG
    mbedtls_ctr_drbg_context drbg;
#else
    mbedtls_hmac_drbg_context drbg;
#endif
    mbedtls_ssl_config conf;
    mbedtls_ssl_session saved_session;
  } *mbedtls;

  /* Preallocate SSL input buffer, as it is quite large (16KiB) and our heap
   * might be too fragmented to allocate it. */

  uint8_t ssl_inbuf[MBEDTLS_SSL_BUFFER_LEN];
#endif
};
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_THINGSEE_HTTPS_PROTOCOL
static struct https_data_s g_https_data;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_THINGSEE_HTTPS_PROTOCOL
#ifdef CONFIG_MBEDTLS

#undef MBEDTLS_MEMDBG

static void *conn_link_mbedtls_calloc(size_t count, size_t size)
{
  if (count == 1 && size == sizeof(g_https_data.ssl_inbuf) &&
      !g_https_data.ssl_inbuf_in_use)
    {
      g_https_data.ssl_inbuf_in_use = true;

      memset(g_https_data.ssl_inbuf, 0, sizeof(g_https_data.ssl_inbuf));

      return g_https_data.ssl_inbuf;
    }

#ifndef MBEDTLS_MEMDBG
  return calloc(count, size);
#else
  uint8_t *mem = malloc(count * size + 5);

  *(uint32_t *)mem = (count * size) | (0xABU << 24);
  mem[count * size + 4] = 0xBA;

  memset(mem + 4, 0, count * size);

  return mem + 4;
#endif
}

static void conn_link_mbedtls_free(void *ptr)
{
  if (!ptr)
    return;

  if (ptr == &g_https_data.ssl_inbuf)
    {
      DEBUGASSERT(g_https_data.ssl_inbuf_in_use);

      g_https_data.ssl_inbuf_in_use = false;

      return;
    }

#ifndef MBEDTLS_MEMDBG
  free(ptr);
#else
  uint8_t *mem = ptr;
  size_t size;

  mem -= 4;

  DEBUGASSERT(*(uint32_t *)mem != 0xf4eeeeedU); /* double free */

  DEBUGASSERT(((*(uint32_t *)mem >> 24) & 0xFF) == 0xABU); /* mem underrun */

  size = *(uint32_t *)mem & 0xFFFFFFU;

  DEBUGASSERT(mem[size + 4] == 0xBA); /* mem overrun */

  *(uint32_t *)mem = 0xf4eeeeedU;

  free(mem);
#endif
}

static int conn_link_mbedtls_initialize(void)
{
  if (g_https_data.initialized)
    return 0;

  g_https_data.mbedtls = calloc(1, sizeof(*g_https_data.mbedtls));
  if (!g_https_data.mbedtls)
    return -1;

  mbedtls_platform_set_calloc_free(conn_link_mbedtls_calloc,
                                   conn_link_mbedtls_free);

  mbedtls_ssl_config_init(&g_https_data.mbedtls->conf);
  mbedtls_ssl_session_init(&g_https_data.mbedtls->saved_session);
  mbedtls_entropy_init(&g_https_data.mbedtls->entropy);

#ifdef CONFIG_MBEDTLS_ENABLE_CTR_DRBG
  mbedtls_ctr_drbg_init(&g_https_data.mbedtls->drbg);

  if (mbedtls_ctr_drbg_seed(&g_https_data.mbedtls->drbg,
                            mbedtls_entropy_func,
                            &g_https_data.mbedtls->entropy,
                            (const void *)"sfx", 3) != 0)
    {
      goto err_free;
    }
#else
  const mbedtls_md_info_t *md_info = NULL;

#ifdef MBEDTLS_SHA1_C
  md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA1);
#elif defined(MBEDTLS_SHA256_C)
  md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
#elif defined(MBEDTLS_SHA512_C)
  md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA512);
#endif

  DEBUGASSERT(md_info != NULL);

  mbedtls_hmac_drbg_init(&g_https_data.mbedtls->drbg);

  if (mbedtls_hmac_drbg_seed(&g_https_data.mbedtls->drbg,
                             md_info,
                             mbedtls_entropy_func,
                             &g_https_data.mbedtls->entropy,
                             (const void *)"sfx", 3) != 0)
    {
      goto err_free;
    }
#endif

  if (mbedtls_ssl_config_defaults(&g_https_data.mbedtls->conf,
                                  MBEDTLS_SSL_IS_CLIENT,
                                  MBEDTLS_SSL_TRANSPORT_STREAM,
                                  MBEDTLS_SSL_PRESET_DEFAULT) != 0)
    {
      goto err_free;
    }

#ifdef CONFIG_MBEDTLS_ENABLE_CTR_DRBG
  mbedtls_ssl_conf_rng(&g_https_data.mbedtls->conf,
                       mbedtls_ctr_drbg_random,
                       &g_https_data.mbedtls->drbg);
#else
  mbedtls_ssl_conf_rng(&g_https_data.mbedtls->conf,
                       mbedtls_hmac_drbg_random,
                       &g_https_data.mbedtls->drbg);
#endif

  mbedtls_ssl_conf_authmode(&g_https_data.mbedtls->conf,
                            MBEDTLS_SSL_VERIFY_NONE);

#ifdef CONFIG_MBEDTLS_MAX_FRAGMENT
  mbedtls_ssl_conf_max_frag_len(&g_https_data.mbedtls->conf,
                                MBEDTLS_SSL_MAX_FRAG_LEN_512);
#endif

#ifdef CONFIG_MBEDTLS_TRUNCATED_HMAC
  mbedtls_ssl_conf_truncated_hmac(&g_https_data.mbedtls->conf,
                                  MBEDTLS_SSL_TRUNC_HMAC_ENABLED);
#endif

#ifdef CONFIG_MBEDTLS_SESSION_TICKET
  /* Use SSL out-fragment buffer of at least 384 bytes with session tickets,
   * preferably at least 512 bytes. */
  mbedtls_ssl_conf_session_tickets(&g_https_data.mbedtls->conf,
                                   MBEDTLS_SSL_SESSION_TICKETS_ENABLED);
#endif

  g_https_data.initialized = true;
  return 0;

err_free:
#ifdef CONFIG_MBEDTLS_ENABLE_CTR_DRBG
  mbedtls_ctr_drbg_free(&g_https_data.mbedtls->drbg);
#else
  mbedtls_hmac_drbg_free(&g_https_data.mbedtls->drbg);
#endif
  mbedtls_ssl_session_free(&g_https_data.mbedtls->saved_session);
  mbedtls_ssl_config_free(&g_https_data.mbedtls->conf);

  free(g_https_data.mbedtls);
  g_https_data.mbedtls = NULL;

  mbedtls_platform_set_calloc_free(calloc, free);
  g_https_data.initialized = false;
  return -1;
}

static void conn_link_mbedtls_cleanup(void)
{
  DEBUGASSERT(g_https_data.mbedtls);
  DEBUGASSERT(g_https_data.initialized);

#ifdef CONFIG_MBEDTLS_ENABLE_CTR_DRBG
  mbedtls_ctr_drbg_free(&g_https_data.mbedtls->drbg);
#else
  mbedtls_hmac_drbg_free(&g_https_data.mbedtls->drbg);
#endif
  mbedtls_ssl_session_free(&g_https_data.mbedtls->saved_session);
  mbedtls_ssl_config_free(&g_https_data.mbedtls->conf);

  free(g_https_data.mbedtls);
  g_https_data.mbedtls = NULL;

  mbedtls_platform_set_calloc_free(calloc, free);
  g_https_data.initialized = false;
  DEBUGASSERT(!g_https_data.ssl_inbuf_in_use);
}
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void conn_link_init_and_check(struct conn_link_s *link, size_t structsize)
{
  DEBUGASSERT(structsize >= sizeof(*link));

  memset(link, 0, sizeof(*link));

  link->sock = -1;

#ifdef CONFIG_THINGSEE_HTTPS_PROTOCOL
#ifdef CONFIG_MBEDTLS
  mbedtls_net_init(&link->net);
#endif
#endif
}

int conn_link_open(struct conn_link_s *link, int *sock, bool use_ssl,
                   uint32_t recv_timeout_ms, uint32_t send_timeout_ms)
{
  DEBUGASSERT(link->sock < 0);
#ifdef CONFIG_THINGSEE_HTTPS_PROTOCOL
  DEBUGASSERT(link->ssl == NULL);
#ifdef CONFIG_MBEDTLS
  DEBUGASSERT(link->net.fd < 0);
#endif
#endif

  link->send_timeout_ms = send_timeout_ms;
  link->recv_timeout_ms = recv_timeout_ms;

  if (!use_ssl)
    {
      link->sock = *sock;
      *sock = -1;
      return OK;
    }
  else
    {
#ifdef CONFIG_THINGSEE_HTTPS_PROTOCOL
#ifdef CONFIG_MBEDTLS
      int ret;
      struct timeval tv;

      if (!g_https_data.initialized)
        {
          con_dbg_save_pos();

          if (conn_link_mbedtls_initialize() < 0)
            return ERROR;
        }

      mbedtls_ssl_conf_read_timeout(&g_https_data.mbedtls->conf,
                                    link->recv_timeout_ms);

      /* Set up a send timeout */

      tv.tv_sec = link->send_timeout_ms / 1000;
      tv.tv_usec = (link->send_timeout_ms % 1000) * 1000;

      ret = setsockopt(*sock, SOL_SOCKET, SO_SNDTIMEO, &tv,
                       sizeof(struct timeval));
      if (ret < 0)
        {
          con_dbg("Setting SO_SNDTIMEO failed, errno: %d\n", errno);
          /* Return here. Socket is more than likely broken */
          return ERROR;
        }
      else
        {
          http_con_dbg("SO_SNDTIMEO := %d msecs\n", link->send_timeout_ms);
        }

      /* Set up a receive timeout */

      tv.tv_sec = link->recv_timeout_ms / 1000;
      tv.tv_usec = (link->recv_timeout_ms % 1000) * 1000;

      ret = setsockopt(*sock, SOL_SOCKET, SO_RCVTIMEO, &tv,
                       sizeof(struct timeval));
      if (ret < 0)
        {
          con_dbg("Setting SO_RCVTIMEO failed, errno: %d\n", errno);
          /* Return if ERROR occurred */
          return ERROR;
        }
      else
        {
          http_con_dbg("SO_RCVTIMEO := %d msecs\n", link->recv_timeout_ms);
        }

      con_dbg_save_pos();

      link->ssl = malloc(sizeof(*link->ssl));
      if (!link->ssl)
        return -1;

      mbedtls_ssl_init(link->ssl);

      ret = mbedtls_ssl_setup(link->ssl, &g_https_data.mbedtls->conf);
      if (ret != 0)
        {
          con_dbg("mbedtls_ssl_setup error: -0x%x\n", -ret);
          goto err_close_tls;
        }

      link->net.fd = *sock;
      link->sock = link->net.fd;
      *sock = -1;

      ret = mbedtls_ssl_set_session(link->ssl,
                                    &g_https_data.mbedtls->saved_session);
      if (ret != 0)
        {
          con_dbg( "mbedtls_ssl_set_session error: -0x%x\n\n", -ret);
        }

      mbedtls_ssl_set_bio(link->ssl, &link->net, mbedtls_net_send,
                          mbedtls_net_recv, mbedtls_net_recv_timeout);

      ret = mbedtls_ssl_handshake(link->ssl);
      if (ret != 0)
        {
          int lowlevel_err = -(-ret & 0x007F);
          int highlevel_err = -(-ret & 0x7F80);

          con_dbg("mbedtls_ssl_handshake error: -0x%x\n", -ret);

          /* Check out-of-memory cases. If we have run out memory or memory
           * has become too fragmented, device will be stuck in bad state.
           * In this case, it will be better to reset.
           * TODO: Should we add controlled reset? */

          DEBUGASSERT(lowlevel_err != MBEDTLS_ERR_MPI_ALLOC_FAILED);
          DEBUGASSERT(lowlevel_err != MBEDTLS_ERR_ASN1_ALLOC_FAILED);
          DEBUGASSERT(highlevel_err != MBEDTLS_ERR_X509_ALLOC_FAILED);
          DEBUGASSERT(highlevel_err != MBEDTLS_ERR_DHM_ALLOC_FAILED);
          DEBUGASSERT(highlevel_err != MBEDTLS_ERR_PK_ALLOC_FAILED);
          DEBUGASSERT(highlevel_err != MBEDTLS_ERR_ECP_ALLOC_FAILED);
          DEBUGASSERT(highlevel_err != MBEDTLS_ERR_CIPHER_ALLOC_FAILED);
          DEBUGASSERT(highlevel_err != MBEDTLS_ERR_SSL_ALLOC_FAILED);

          goto err_close_tls;
        }

      con_dbg("%c[38;5;%d48;5;%dm"
                   "HTTPS: TLS version is '%s'."
                   "%c[0m\n", 0x1B, 0, 1,
                   mbedtls_ssl_get_version(link->ssl), 0x1B);
      con_dbg("%c[38;5;%d48;5;%dm"
                   "HTTPS: TLS cipher suite is '%s'."
                   "%c[0m\n", 0x1B, 0, 1,
                   mbedtls_ssl_get_ciphersuite(link->ssl), 0x1B);

      ret = mbedtls_ssl_get_session(link->ssl,
                                    &g_https_data.mbedtls->saved_session);
      if (ret != 0)
        {
          con_dbg( "mbedtls_ssl_get_session error: -0x%x\n\n", -ret);
        }

      return OK;

err_close_tls:
      con_dbg_save_pos();
      mbedtls_ssl_free(link->ssl);
      free(link->ssl);
      link->ssl = NULL;
      return ERROR;

#endif
#endif
      return ERROR;
    }
}

void conn_link_close(struct conn_link_s *link)
{
#ifdef CONFIG_THINGSEE_HTTPS_PROTOCOL
#ifdef CONFIG_MBEDTLS
  if (link->net.fd >= 0)
    {
      close(link->net.fd);
      link->net.fd = -1;
      link->sock = -1;
    }

  if (link->ssl)
    {
      mbedtls_ssl_free(link->ssl);
      free(link->ssl);
      link->ssl = NULL;
    }

#endif
#endif

  if (link->sock >= 0)
    {
      close(link->sock);
      link->sock = -1;
    }
}

int conn_link_write(struct conn_link_s *link, const char **pbuf, const size_t *plen)
{
  struct timeval tv;
  int ret;
  size_t len;
  int bytes_written = 0;

  /* Set up a send timeout */

  tv.tv_sec = link->send_timeout_ms / 1000;
  tv.tv_usec = (link->send_timeout_ms % 1000) * 1000;

  ret = setsockopt(link->sock, SOL_SOCKET, SO_SNDTIMEO, &tv,
                   sizeof(struct timeval));
  if (ret < 0)
    {
      con_dbg("Setting SO_SNDTIMEO failed, errno: %d\n", errno);
      /* Return here. Socket is more than likely broken */
      return ERROR;
    }
  else
    {
      http_con_dbg("SO_SNDTIMEO := %d msecs\n", link->send_timeout_ms);
    }

  http_con_dbg("Send data...\n");

#ifdef CONFIG_THINGSEE_HTTPS_PROTOCOL
  if (link->ssl != NULL)
    {
      /* Fully write buffer. */

      while (*pbuf)
        {
          const char *buf = *pbuf;
          len = *plen;
          bytes_written += len;
          ++pbuf;
          ++plen;

          do {
              ssize_t nwritten;

#ifdef CONFIG_MBEDTLS
              nwritten = mbedtls_ssl_write(link->ssl, (const void *)buf, len);
              ret = nwritten;
              http_con_dbg("mbedtls_ssl_write, ret=%d\n", nwritten);
              if (nwritten <= 0)
                {
                  con_dbg("mbedtls_ssl_write error: -0x%x\n", -ret);
                  return ERROR;
                }
#endif

              buf += nwritten;
              len -= nwritten;
          } while (len);
        }
      return bytes_written;
    }
  else
#endif /*CONFIG_THINGSEE_HTTPS_PROTOCOL*/
    {
      /* Fully write buffer. */

      while (*pbuf)
        {
          const char *buf = *pbuf;
          len = *plen;
          bytes_written += len;
          ++pbuf;
          ++plen;

          do {
              ssize_t nwritten;

              nwritten = send(link->sock, buf, len, 0);
              http_con_dbg("send, ret=%d\n", nwritten);
              if (nwritten < 0)
                return ERROR;

              buf += nwritten;
              len -= nwritten;
          } while (len);
        }
    }

  return bytes_written;
}

int conn_link_read(struct conn_link_s *link, unsigned char *buf, int len)
{
  struct timeval tv;
  int ret;

  /* Set up a receive timeout */

  tv.tv_sec = link->recv_timeout_ms / 1000;
  tv.tv_usec = (link->recv_timeout_ms % 1000) * 1000;

  ret = setsockopt(link->sock, SOL_SOCKET, SO_RCVTIMEO, &tv,
                   sizeof(struct timeval));
  if (ret < 0)
    {
      con_dbg("Setting SO_RCVTIMEO failed, errno: %d\n", errno);
      /* Return if ERROR occurred */
      return ERROR;
    }
  else
    {
      http_con_dbg("SO_RCVTIMEO := %d msecs\n", link->recv_timeout_ms);
    }

#ifdef CONFIG_THINGSEE_HTTPS_PROTOCOL
  if (link->ssl != NULL)
    {
#ifdef CONFIG_MBEDTLS
      ret = mbedtls_ssl_read(link->ssl, (void *)buf, len);
      if (ret <= 0)
        {
          con_dbg("mbedtls_ssl_read error: -0x%x\n", -ret);
          return ERROR;
        }
#endif

      return ret;
    }
  else
#endif /*CONFIG_THINGSEE_HTTPS_PROTOCOL*/
    {
      return recv(link->sock, buf, (size_t) len, 0);
    }
}

void conn_link_cleanup(void)
{
#ifdef CONFIG_THINGSEE_HTTPS_PROTOCOL
#ifdef CONFIG_MBEDTLS
  if (g_https_data.mbedtls)
    {
      conn_link_mbedtls_cleanup();
    }
#endif
#endif
}
