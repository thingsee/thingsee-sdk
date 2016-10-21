/****************************************************************************
 * apps/ts_engine/connectors/conn_comm_link.h
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

#ifndef __APPS_TS_ENGINE_CONNECTORS_CONN_COMM_LINK_H
#define __APPS_TS_ENGINE_CONNECTORS_CONN_COMM_LINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define conn_link_init(ptr) \
  conn_link_init_and_check((ptr), sizeof(*(ptr)))

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct conn_link_s
{
#ifdef CONFIG_THINGSEE_HTTPS_PROTOCOL
#ifdef CONFIG_MBEDTLS
  void *__priv_ssl;
  int __priv_net;
#endif
#endif
  int __priv_sock;
  uint32_t __priv_send_timeout_ms;
  uint32_t __priv_recv_timeout_ms;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void conn_link_verify_size(size_t ext_conn_link_size);

void conn_link_init_and_check(struct conn_link_s *link, size_t struct_size);

int conn_link_open(struct conn_link_s *link, int *sock, bool use_ssl,
                   uint32_t recv_timeout_ms, uint32_t send_timeout_ms);

void conn_link_close(struct conn_link_s *link);

int conn_link_write(struct conn_link_s *link, const char **pbuf,
                    const size_t *plen);

int conn_link_read(struct conn_link_s *link, unsigned char *buf, int len);

void conn_link_cleanup(void);

#endif /* __APPS_TS_ENGINE_CONNECTORS_CONN_COMM_LINK_H */
