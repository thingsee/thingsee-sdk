/****************************************************************************
 * apps/ts_engine/connectors/conn_comm.h
 *
 * Copyright (C) 2015 Haltian Ltd. All rights reserved.
 * Author: Timo Voutilainen <timo.voutilainen@haltian.com>
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

#ifndef __APPS_TS_ENGINE_CONNECTORS_CONN_COMM_H
#define __APPS_TS_ENGINE_CONNECTORS_CONN_COMM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <apps/netutils/cJSON.h>
#include <mqueue.h>

#include "connector.h"
#include "conn_comm_stream.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TODO: remove these after connectors have been updated to use
 * 'conn_comm_execute_http.h'. */

#define HTTP_USER_AGENT            "tsone/0.1"
#define DEFAULT_PORT               80

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum conn_protocol_type_e
{
  CON_PROTOCOL_HTTP = 0,
  CON_PROTOCOL_HTTPS,
  CON_PROTOCOL_MQTT,
 /* Add here your protocol */
} conn_protocol_type_t;

enum conn_comm_tls_e
{
  CONN_COMM_TLS_AUTO = 0,
  CONN_COMM_TLS_ENABLE,
  CONN_COMM_TLS_DISABLE,
};

typedef struct con_str
{
  char *host;
  uint16_t port;
  enum conn_comm_tls_e https;
  pthread_t thread;
  int mq_task_count:8;
  bool processing_task:1;
  bool network_ready:1; /* obsolete, write-accessed only */
  bool thread_joining:1;
  mqd_t task_mq;
  pthread_mutex_t mutex;
  struct sockaddr_in srv_ip4addr;
  bool stopping;
} con_str_t;

typedef enum
{
  NETWORK_TASK_STOP,
  NETWORK_TASK_REQUEST,
} e_network_task_type;

typedef struct
{
  char *payload;
  int process_status;
  struct sockaddr_in srv_ip4addr;
  send_cb_t cb;
  const void *priv; /* priv to give send_cb */
  void *context_priv;
} conn_workflow_context_s;

struct conn_http_params_s
{
  const char *host;
  uint16_t port;
  enum conn_comm_tls_e tls;
  struct sockaddr_in *ipaddr_cache;
};

typedef int (*conn_request_construct_t)
    (conn_workflow_context_s *context, char **outhdr, char **outdata);

typedef struct conn_network_task_s* (*conn_response_process_t)
    (conn_workflow_context_s *context, int status_code, const char *content);

typedef struct conn_network_task_s* (*conn_response_process_stream_t)
    (conn_workflow_context_s *context, int status_code,
     size_t content_len, char (*stream_getc)(void *priv), void *stream_priv);

typedef bool (*conn_get_http_params_t)
    (conn_workflow_context_s *context, struct conn_http_params_s *params);

/* Better not instantiate this structure directly,
   use function 'conn_create_network_task' instead */
struct conn_network_task_s
{
  const char *title;
  conn_workflow_context_s *context;
  conn_request_construct_t construct; /* Constructs the network request */
  conn_response_process_t process; /* Processes the server response and returns the next task */
  conn_response_process_stream_t process_stream; /* Processes the server response and returns the next task */
  union
  {
    /* Protocol specific helpers. */
    struct
    {
      conn_get_http_params_t get_conn_params;
    } http;
  };
};

typedef struct
{
  e_network_task_type type;

  union {
    struct conn_network_task_s *conn;
  };
} network_task_s;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void conn_set_protocol_type(conn_protocol_type_t type);
void conn_free_pointer(void **ptr);
int conn_init_boolean_from_json(cJSON **container, char *id, bool *dest);
int conn_init_param_from_json(cJSON **container, char *id, char ** dest);
void conn_destroy_task(struct conn_network_task_s *task);
int conn_network_give_new_conn_task(struct conn_network_task_s *conn_task);
struct conn_network_task_s* conn_create_network_task(
    const char *title,
    conn_workflow_context_s *context,
    conn_request_construct_t construct,
    conn_response_process_t process);
struct conn_network_task_s* conn_create_network_task_stream(
    const char *title,
    conn_workflow_context_s *context,
    conn_request_construct_t construct,
    conn_response_process_stream_t process_stream);
void conn_complete_task_workflow(conn_workflow_context_s *context, int err);
int conn_init(con_str_t *conn);
int conn_uninit(void);
void conn_empty_message_queue(void);
#ifdef CONFIG_THINGSEE_ENGINE
int conn_update_profile(char * const profile);
#endif

#endif /* __APPS_TS_ENGINE_CONNECTORS_CONN_COMM_H */
