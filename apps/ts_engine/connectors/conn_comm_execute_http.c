/****************************************************************************
 * apps/ts_engine/connectors/conn_comm_execute_http.c
 *
 * Copyright (C) 2015 Haltian Ltd. All rights reserved.
 * Authors: Timo Voutilainen <timo.voutilainen@haltian.com>
 *          Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#include <sys/types.h>
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
#include <pthread.h>
#include <errno.h>
#include <netinet/in.h>

#include <apps/system/conman.h>

#include "connector.h"
#include "conn_comm.h"
#include "conn_comm_link.h"
#include "conn_comm_util.h"
#include "conn_comm_execute_http.h"
#include "con_dbg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TASK_MESSAGE_QUEUE_MAX        10
#define NETWORK_ERROR                 (ERROR - 1)

#define CONNECTION_SEND_TIMEOUT       (20 * 1000) /* msecs */
#define CONNECTION_RECV_TIMEOUT       (30 * 1000) /* msecs */

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

void execute_http_request_close_link(void *priv)
{
  struct conn_link_s *conn_link = priv;

  if (conn_link)
    {
      conn_link_close(conn_link);
    }
}

ssize_t execute_http_request_read_link(void *priv, void *buf, size_t buflen)
{
  struct conn_link_s *conn_link = priv;

  if (!conn_link)
    {
      return -1;
    }

  return conn_link_read(conn_link, (unsigned char *)buf, buflen);
}

static bool parse_line_int(const char *line, size_t linelen,
                           const char *hdr, size_t hdrlen,
                           int *val)
{
  if (linelen >= hdrlen && !strncasecmp(hdr, line, hdrlen))
    {
      const char *endp = line + hdrlen;
      long l;

      l = strtol(endp, (char **)&endp, 10);

      if (endp > line + hdrlen)
        {
          *val = l;
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int conn_execute_http_request(struct con_str *con,
                              struct conn_network_task_s *task,
                              int *pstatus_code,
                              struct conn_link_s *conn_link,
                              char *hdr,
                              size_t hdrlen,
                              char *pdata,
                              size_t datalen,
                              struct conn_content_stream_s *content)
{
  const char *bufs[3] = { hdr, pdata, NULL };
  const size_t lens[3] = { hdrlen, datalen, 0 };
  const char **pbuf = bufs;
  const size_t *plen = lens;
  char linebuf[CONN_STREAM_BUF_SIZE];
  char *inbuf;
  size_t sizeof_inbuf;
  size_t linepos;
  size_t len;
  char prev_ch;
  int linenum;
  int num;
  int sock;
  int ret;
  int contentlen;
  size_t pos;
  const char *host;
  struct sockaddr_in volatile_cache;
  struct sockaddr_in *current_srv_ip4addr;
  bool use_ssl;
  conn_workflow_context_s *context = task->context;
  struct conn_http_params_s params = {};
  uint16_t port;

  if (!task->http.get_conn_params ||
      !task->http.get_conn_params(context, &params))
    {
      params.host = con->host;
      params.port = con->port;
      params.tls = con->https;
      params.ipaddr_cache = &con->srv_ip4addr;
    }

  port = params.port;
  host = params.host;
  if (params.ipaddr_cache == NULL)
    {
      memset(&volatile_cache, 0, sizeof(volatile_cache));
      params.ipaddr_cache = &volatile_cache;
    }
  current_srv_ip4addr = params.ipaddr_cache;

  switch (params.tls)
    {
    case CONN_COMM_TLS_AUTO:
      use_ssl = (port == 443);
      break;
    case CONN_COMM_TLS_ENABLE:
      use_ssl = true;
      break;
    case CONN_COMM_TLS_DISABLE:
      use_ssl = false;
      break;
    default:
      DEBUGASSERT(false);
      break;
    }

  DEBUGASSERT(hdr && pstatus_code && content && conn_link);

  *pstatus_code = ERROR;

  http_con_dbg("%s:\n%s%s", use_ssl ? "HTTPS" : "HTTP", hdr, pdata);

  /* Fetch server IP address. */

  if (current_srv_ip4addr->sin_addr.s_addr == 0)
    {
      con_dbg_save_pos();

      con->network_ready = false;
      if (conn_comm_get_server_address(current_srv_ip4addr, host) != OK)
        return NETWORK_ERROR;

      con->network_ready = true;
    }

  con_dbg_save_pos();

  /* Open HTTP connection to server. */

  http_con_dbg("Open socket...\n");
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
    return NETWORK_ERROR;

  con_dbg_save_pos();

  http_con_dbg("Connect to port %d ...\n", port);
  current_srv_ip4addr->sin_port = htons(port);
  ret = connect(sock, (struct sockaddr *)current_srv_ip4addr, sizeof(*current_srv_ip4addr));
  if (ret < 0)
    {
      /* Could not connect to server. Try updating server IP address on
       * next try. */

      memset(current_srv_ip4addr, 0, sizeof(*current_srv_ip4addr));
      goto err_close;
    }

  con_dbg_save_pos();

  /* Initialize connection (http / https). */

  if (conn_link_open(conn_link, &sock, use_ssl, CONNECTION_RECV_TIMEOUT,
                     CONNECTION_SEND_TIMEOUT) < 0)
    {
      goto err_close;
    }

  con_dbg_save_pos();

  /* Send data to the server */

  if (conn_link_write(conn_link, pbuf, plen) < 0)
    {
      goto err_close;
    }

  http_con_dbg("Wait response...\n");

  /* Read HTTP header. */

  linepos = 0;
  prev_ch = 0;
  linenum = 0;
  contentlen = 0;
  len = 0;
  inbuf = content->buf;
  sizeof_inbuf = sizeof(content->buf);
  do
    {
      con_dbg_save_pos();

      ret = conn_link_read(conn_link, (unsigned char *)inbuf, sizeof_inbuf);
      http_con_dbg("conn_link_read, ret=%d\n", ret);
      if (ret <= 0)
        goto invalid_response;

      con_dbg_save_pos();

      for (pos = 0; pos < ret; pos++)
        {
          char cur_ch = inbuf[pos];

          if (linepos < sizeof(linebuf))
            linebuf[linepos++] = cur_ch;
          else
            linebuf[sizeof(linebuf) - 1] = '\0';

          if (prev_ch == '\r' && cur_ch == '\n')
            {
              /* EOL found! */

              DEBUGASSERT(linepos >= 2);

              if (linepos <= sizeof(linebuf))
                linebuf[linepos - 2] = '\0';
              linepos -= 2;

              http_con_dbg("Response line from server: [%s]\n", linebuf);

              if (linenum == 0)
                {
                  /* First line, "HTTP/1.1 <code> " */

                  num = sscanf(linebuf, "HTTP/1.1 %d ", pstatus_code);
                  if (num != 1)
                    {
                      /* Not HTTP response. */

                      goto invalid_response;
                    }
                }
              else if (linenum > 0)
                {
                  /* HTTP Headers, wait for empty line or "Content-Length: <len>" */

                  if (linepos == 0)
                    {
                      /* Empty line, start of content. */

                      /* inbuf probably has part of content data already. */

                      pos++;
                      if (pos < ret)
                        {
                          linepos = 0;
                          len = ret - pos;
                          memmove(inbuf, &inbuf[pos], len);
                        }
                      else
                        {
                          linepos = 0;
                          len = 0;
                        }

                      http_con_dbg("Handle content data?\n");
                      goto handle_content;
                    }

                  static const char content_len[] = "CONTENT-LENGTH:";
                  const size_t content_len_len = sizeof(content_len) - 1;

                  if (parse_line_int(linebuf, linepos, content_len,
                                     content_len_len, &contentlen)
                      && contentlen >= 0)
                    {
                      /* Got content length! */

                      http_con_dbg("HTTP Content length = %d!\n", contentlen);
                    }
                }

              linepos = 0;
              linenum++;
            }

          prev_ch = cur_ch;
        }
    }
  while (ret >= 0);

  /* Should not get here. */

  DEBUGASSERT(false);

  /* Handle content from server. */

handle_content:

  con_dbg_save_pos();

  /* Prepare content stream. */

  content->buf_pos = 0;
  content->buf_len = len;
  content->content_pos = len;
  content->max_content_len = contentlen;
  content->link_close = execute_http_request_close_link;
  content->link_read = execute_http_request_read_link;
  content->conn_link = conn_link;

  DEBUGASSERT(sock < 0);
  http_con_dbg("Done!\n");
  return OK;

invalid_response:
  http_con_dbg("Invalid HTTP response!\n");
  conn_link_close(conn_link);
  if (sock >= 0)
    close(sock);
  http_con_dbg("Socket closed!\n");
  return ERROR;

err_close:
  conn_link_close(conn_link);
  if (sock >= 0)
    close(sock);
  http_con_dbg("Socket closed!\n");
  return NETWORK_ERROR;
}
