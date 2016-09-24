/****************************************************************************
 * modules/connector.c
 *
 * Copyright (C) 2016 Haltian Ltd.
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
 * Authors:
 *   Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <apps/../ts_engine/connectors/conn_comm.h>

#include <example_app_dbg.h>
#include <example_app_main.h>
#include <example_app_connector.h>

struct context_priv_s
{
  char *host;
  char *path;
  uint16_t port;
  enum conn_comm_tls_e tls;

  size_t payload_len;

  char *url_data;
  char *method;
  char *extra_headers;
};

static struct
{
  bool initialized:1;

  struct con_str conn;
} g_connector =
  {
    .initialized = false,
  };

static bool http_get_connection_params(conn_workflow_context_s *context,
                                       struct conn_http_params_s *params)
{
  /* !!Called in conn_comm thread context!! */

  struct context_priv_s *context_priv = context->context_priv;

  params->host = context_priv->host;
  params->port = context_priv->port;
  params->tls = context_priv->tls;
  params->ipaddr_cache = NULL;
  return true;
}

static int http_request_construct(conn_workflow_context_s *context,
                                  char **outhdr, char **outdata)
{
  /* !!Called in conn_comm thread context!! */

  struct context_priv_s *context_priv = context->context_priv;
  int datalen = 0, hdrlen = 0;

  /* Construct HTTP data */

  *outdata = context->payload;
  datalen = context_priv->payload_len;

  hdrlen = asprintf(outhdr,
      "%s %s HTTP/1.1\r\n"
      "Host: %s\r\n"
      "User-Agent: %s\r\n"
      "Accept: */*\r\n"
      "Connection: close\r\n"
      "Content-Length: %d\r\n"
      "%s"
      "\r\n",
      context_priv->method,
      context_priv->path,
      context_priv->host,
      "sdk_example_app 0.0",
      datalen,
      context_priv->extra_headers);

  if (hdrlen >= 0)
    {
      exapp_dbg("SENDING DATA:\n%s%s\n", *outhdr, *outdata ? *outdata : "");
    }

  return (hdrlen >= 0) ? OK : ERROR;
}

static struct conn_network_task_s *http_response_process_stream(
    conn_workflow_context_s *context, int status_code, size_t content_len,
    char (*stream_getc)(void *priv), void *stream_priv)
{
  /* !!Called in conn_comm thread context!! */

  exapp_dbg("HTTP response, status code: %d\n", status_code);
  exapp_dbg("HTTP response, payload len: %d\n", (int)content_len);

  exapp_dbg("HTTP response, payload: ");
  while (content_len--)
    {
      char c = stream_getc(stream_priv);

      putc(c, stdout);
    }
  putc('\n', stdout);

  return NULL; /* No more chained network tasks. */
}

static int parse_url_parts(struct context_priv_s *priv)
{
  char *copy;
  int port = -1;
  char *proto;
  char *host;
  char *pc, *pc2;
  char *path;

  /* proto://host:port/path */

  copy = priv->url_data;

  proto = copy;

  pc = strstr(proto, "://");
  if (!pc)
    {
      return ERROR;
    }

  *pc = 0;
  pc += 3;

  if (strcmp(proto, "http") != 0 && strcmp(proto, "https") != 0)
    {
      return ERROR;
    }

  host = pc;
  pc = strstr(host, "/");
  if (pc)
    {
      pc2 = strstr(host, ":");
      if (pc2 && pc2 < pc)
        {
          *pc2 = 0;

          if (sscanf(pc2 + 1, "%d", &port) != 1)
            {
              port = -1;
            }
        }
      else
        {
          memmove(host - 1, host, pc - host);
          *(pc - 1) = 0;
          host--;
        }
    }
  else
    {
      return ERROR;
    }

  path = pc;

  if (port < 0)
    {
      port = strcmp(proto, "http") == 0 ? 80 : 443;
    }

  priv->host = host;
  priv->port = port;
  priv->path = path;
  priv->tls = (strcmp(proto, "http") == 0)
               ? CONN_COMM_TLS_DISABLE : CONN_COMM_TLS_ENABLE;
  return OK;
}

static int send_callback(int result, const void *priv)
{
  return OK;
}

int __exapp_connector_send_https(const char *url,
                                 const char *method,
                                 const char *extra_headers,
                                 void *payload, size_t payload_len,
                                 void *priv)
{
  struct conn_network_task_s *send_task = NULL;
  conn_workflow_context_s *context = NULL;
  struct context_priv_s *context_priv;
  int ret = OK;
  size_t url_len = strlen(url) + 1;
  size_t method_len = (method ? strlen(method) : 0) + 1;
  size_t extra_headers_len = (extra_headers ? strlen(extra_headers) : 0) + 1;

  if (!method)
    {
      method = "POST";
      method_len = strlen(method) + 1;
    }

  if (!extra_headers)
    {
      extra_headers = "";
      extra_headers_len = 1;
    }

  context = calloc(1, sizeof(conn_workflow_context_s)
                      + sizeof(struct context_priv_s)
                      + url_len
                      + method_len
                      + extra_headers_len);
  if (context)
    {
      context_priv = (void *)((uint8_t *)context + sizeof(*context));
      context->context_priv = context_priv;

      context_priv->url_data      = (char *)context_priv + sizeof(*context_priv);
      context_priv->method        = context_priv->url_data + url_len;
      context_priv->extra_headers = context_priv->method + method_len;

      memcpy(context_priv->url_data, url, url_len);
      memcpy(context_priv->method, method, method_len);
      memcpy(context_priv->extra_headers, extra_headers, extra_headers_len);

      context->payload = payload;
      context_priv->payload_len = payload_len;
      context->cb = send_callback;
      context->priv = priv;

      if (parse_url_parts(context_priv) < 0)
        {
          free(context);
          return ERROR;
        }
    }

  send_task = conn_create_network_task_stream(
      "HTTP(S) REST",
      context,
      http_request_construct,
      http_response_process_stream);

  if (send_task)
    {
      send_task->http.get_conn_params = http_get_connection_params;

      ret = conn_network_give_new_conn_task(send_task);
      if (ret != OK)
        {
          conn_destroy_task(send_task);
          send_task = NULL;
        }
    }
  else
    {
      ret = ERROR;
      exapp_dbg("Could not start a new task for sending data to connector\n");
    }

  if (!send_task)
    {
      conn_complete_task_workflow(context, ret);
      context = NULL;
    }

  return ret;
}

int __exapp_connector_init(void)
{
  int ret = OK;

  if (g_connector.initialized)
    {
      return ERROR; /* already initialized */
    }

  ret = conn_init(&g_connector.conn);
  if (ret != OK)
    {
      return ret;
    }

  return OK;
}

int __exapp_connector_uninit(void)
{
  if (!g_connector.initialized)
    {
      return ERROR;
    }

  conn_uninit();

  memset(&g_connector, 0, sizeof(g_connector));
  g_connector.initialized = false;

  return OK;
}
