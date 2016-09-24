/****************************************************************************
 * apps/ts_engine/connectors/conn_comm_stream.c
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
#include <assert.h>
#include <debug.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>

#include "connector.h"
#include "conn_comm.h"
#include "con_dbg.h"
#include "conn_comm_util.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

char conn_content_stream_getc(void *priv)
{
  struct conn_content_stream_s *stream = priv;
  size_t currlen;
  ssize_t rlen;

  if (stream->buf_pos < stream->buf_len)
    {
      /* Read from buffer. */

      return stream->buf[stream->buf_pos++];
    }

  if (stream->content_pos >= stream->max_content_len ||
      !stream->conn_link || !stream->link_read)
    {
      /* End of content reached. Read as zero. */

      return '\0';
    }

  /* Get more data from link to buffer. */

  currlen = stream->max_content_len - stream->content_pos;
  if (currlen > sizeof(stream->buf))
    {
      currlen = sizeof(stream->buf);
    }

  rlen = stream->link_read(stream->conn_link, stream->buf, currlen);
  if (rlen <= 0)
    {
      /* EOF or error, assume link is closed. */

      stream->buf_pos = 0;
      stream->buf_len = 0;
      stream->content_pos = stream->max_content_len;
      return '\0';
    }

  stream->content_pos += rlen;
  stream->buf_pos = 0;
  stream->buf_len = rlen;

  if (stream->content_pos >= stream->max_content_len)
    {
      /* Fully read content, close link. */

      stream->link_close(stream->conn_link);
      stream->conn_link = NULL;
    }

  return stream->buf[stream->buf_pos++];
}

void conn_content_stream_init(struct conn_content_stream_s *stream)
{
  stream->max_content_len = 0;
  stream->content_pos = 0;
  stream->buf_pos = 0;
  stream->buf_len = 0;
  stream->conn_link = NULL;
  stream->link_read = NULL;
  stream->link_close = NULL;
}

void conn_content_stream_close(struct conn_content_stream_s *stream)
{
  if (stream->link_close)
    {
      stream->link_close(stream->conn_link);
    }

  stream->max_content_len = 0;
  stream->content_pos = 0;
  stream->buf_pos = 0;
  stream->buf_len = 0;
  stream->conn_link = NULL;
  stream->link_read = NULL;
  stream->link_close = NULL;
}

/* Helper function for old connectors that expect full content to be
 * passed to them as null-terminated string. */

struct conn_network_task_s *
conn_stream_to_string_and_process (struct conn_network_task_s *task,
                                   int status_code, size_t content_len,
                                   char (*stream_getc)(void *priv),
                                   void *stream_priv)
{
  struct conn_network_task_s *ret;
  char *content;
  size_t i;

  DEBUGASSERT(task && task->process);

  /* Extract string from stream. */

  content = malloc(content_len + 1);
  if (!content)
    {
      return NULL;
    }

  for (i = 0; i < content_len; i++)
    {
      content[i] = stream_getc(stream_priv);
    }
  content[content_len] = '\0';

  http_con_dbg("Content: [%s]\n", content);

  /* Call old style process function. */

  ret = task->process(task->context, status_code, content);

  free(content);

  return ret;
}
