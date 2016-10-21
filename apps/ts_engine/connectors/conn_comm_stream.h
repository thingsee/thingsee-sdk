/****************************************************************************
 * apps/ts_engine/connectors/conn_comm_stream.h
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

#ifndef __APPS_TS_ENGINE_CONNECTORS_CONN_COMM_STREAM_H
#define __APPS_TS_ENGINE_CONNECTORS_CONN_COMM_STREAM_H

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

#define CONN_STREAM_BUF_SIZE 128

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct conn_content_stream_s
{
  char buf[CONN_STREAM_BUF_SIZE];
  size_t max_content_len;
  size_t content_pos;
  size_t buf_pos;
  size_t buf_len;

  /* Connection link pointer, abstraction to support different protocol
   * libraries. */

  void *conn_link;

  /* Link read function. */

  ssize_t (*link_read)(void *conn_link, void *buf, size_t buflen);

  /* Closing & freeing link. */

  void (*link_close)(void *conn_link);
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void conn_content_stream_init(struct conn_content_stream_s *stream);
char conn_content_stream_getc(void *priv);
void conn_content_stream_close(struct conn_content_stream_s *stream);

/* Helper function for old connectors that expect full content to be
 * passed to them as null-terminated string. */

struct conn_network_task_s *
conn_stream_to_string_and_process (struct conn_network_task_s *task,
                                   int status_code, size_t content_len,
                                   char (*stream_getc)(void *priv),
                                   void *stream_priv);

#endif /* __APPS_TS_ENGINE_CONNECTORS_CONN_COMM_H */
