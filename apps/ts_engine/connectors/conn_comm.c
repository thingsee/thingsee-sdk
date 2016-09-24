/****************************************************************************
 * apps/ts_engine/connectors/conn_comm.c
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
#include "conn_comm_execute_mqtt.h"
#include "con_dbg.h"
#include "../engine/client.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TASK_MESSAGE_QUEUE_MAX        10
#define NETWORK_ERROR                 (ERROR - 1)

#ifndef CONFIG_CONNECTOR_SIGWAKEUP
#  define CONFIG_CONNECTOR_SIGWAKEUP 20
#endif

#if CONFIG_CONNECTOR_SIGWAKEUP > MAX_SIGNO
#  error "CONFIG_CONNECTOR_SIGWAKEUP invalid"
#endif

#define CONNECTION_RETRY_DELAY_SEC    10
#define CONNECTION_RETRIES            12

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void *conn_network_thread(void *param);
static int ts_network_give_new_task(network_task_s *task);
static bool conn_network_is_deepsleep_allowed(void * const priv);
void conn_ping_main_thread(void) __attribute__((weak));
void conn_destroy_task(struct conn_network_task_s *task);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static con_str_t *con = NULL;
static conn_protocol_type_t protocol_type;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if 0
static void mem_dbg(void)
{
#ifdef CONFIG_THINGSEE_CONNECTORS_DEBUG
  struct mallinfo mem;

  mem = mallinfo();

  con_dbg ("             total       used       free    largest\n");
  con_dbg ("Mem:   %11d%11d%11d%11d\n", mem.arena, mem.uordblks, mem.fordblks,
      mem.mxordblk);
#endif
}
#endif

static bool conn_network_is_deepsleep_allowed(void * const priv)
{
  bool queued;
  bool processing;

  pthread_mutex_lock(&con->mutex);
  queued = con->mq_task_count > 0;
  processing = con->processing_task;
  pthread_mutex_unlock(&con->mutex);

  return !(queued || processing);
}

static int request_connection(uint32_t *connid, bool onoff)
{
  struct conman_client_s client;
  struct conman_status_s status;
  int retries = 0;
  int ret;

  con_dbg_save_pos();

  ret = conman_client_init(&client);
  if (ret < 0)
    {
      con_dbg("conman_client_init failed\n");
      return ERROR;
    }

  if (!onoff)
    {
      DEBUGASSERT(*connid >= 0);

      con_dbg_save_pos();

      ret = conman_client_destroy_connection(&client, *connid);
      if (ret < 0)
        {
          con_dbg("conman_client_destroy_connection failed\n");
        }
      goto out;
    }

  con_dbg_save_pos();

  ret = conman_client_request_connection(&client, CONMAN_DATA, connid);
  if (ret < 0)
    {
      con_dbg("conman_client_request_connection failed\n");
      goto out;
    }

  while (true)
    {
      struct timespec sleeptime;

      con_dbg_save_pos();

      ret = conman_client_get_connection_status(&client, &status);
      if (ret < 0)
        {
          con_dbg("conman_client_get_connection_status failed\n");
          goto out;
        }

      if (status.status == CONMAN_STATUS_ESTABLISHED)
        {
          break;
        }

      if (++retries == CONNECTION_RETRIES)
        {
          ret = ERROR;
          break;
        }

      sleeptime.tv_sec  = CONNECTION_RETRY_DELAY_SEC;
      sleeptime.tv_nsec = 0;

      ret = nanosleep(&sleeptime, NULL);
      if (ret < 0)
        {
          /* Interrupted. */

          con_dbg_save_pos();

          pthread_mutex_lock(&con->mutex);
          ret = !con->thread_joining ? OK : ERROR;
          pthread_mutex_unlock(&con->mutex);

          if (ret == ERROR)
            {
              break;
            }
        }
    }

out:

  con_dbg_save_pos();

  conman_client_uninit(&client);

  return ret;
}

static int execute_task_conn_request(struct conn_network_task_s *task)
{
  char *hdr = NULL;
  char *data = NULL;
  size_t hdrlen = 0;
  size_t datalen = 0;
  int ret = ERROR;
  struct conn_network_task_s *next_task = NULL;

  DEBUGASSERT(task);

  con_dbg("%s\n", task->title);

  con_dbg_save_pos();

  ret = task->construct(task->context, &hdr, &data);
  if (ret == OK)
    {
      con_dbg_save_pos();

      hdrlen = (hdr != NULL) ? strlen(hdr) : 0;
      datalen = (data != NULL) ? strlen(data) : 0;
      if (protocol_type != CON_PROTOCOL_MQTT && hdrlen <= 0)
        ret = ERROR;
    }

  if (ret == OK)
    {
      int status_code = INT_MIN;
      struct conn_link_s conn_link;
      struct conn_content_stream_s content;

      con_dbg("Protocol type: %d\n", protocol_type);

      conn_link_init(&conn_link);
      conn_content_stream_init(&content);

      con_dbg_save_pos();

      switch (protocol_type)
        {
          case CON_PROTOCOL_HTTP:
            ret = conn_execute_http_request(con, task, &status_code,
                                            &conn_link, hdr, hdrlen,
                                            data, datalen, &content);
            break;
          case CON_PROTOCOL_MQTT:
            ret = conn_execute_mqtt_request(data, datalen, task, &status_code);
            break;
          default:
            /* Do nothing about a wrong protocol type */

            break;
        }

      if (ret < 0)
        {
          status_code = ret;
        }

      if (status_code != INT_MIN)
        {
          con_dbg_save_pos();

          if (task->process_stream)
            {
              next_task = task->process_stream(task->context, status_code,
                                               content.max_content_len,
                                               conn_content_stream_getc,
                                               &content);
            }
          else
            {
              next_task = conn_stream_to_string_and_process(
                                                task, status_code,
                                                content.max_content_len,
                                                conn_content_stream_getc,
                                                &content);
            }

          if (next_task != NULL)
            {
              con_dbg_save_pos();
              ret = conn_network_give_new_conn_task(next_task);
            }
        }

      conn_content_stream_close(&content);
      conn_free_pointer((void**)&hdr);
    }

  if (next_task == NULL) /* We have reached the end of the workflow */
    {
      con_dbg_save_pos();

      /* Note, 'data' will be freed in conn_complete_task_workflow() */

      conn_complete_task_workflow(task->context, ret);
      data = NULL;
    }
  else if (ret == 0 && !task->context->payload)
    {
      con_dbg_save_pos();

      conn_free_pointer((void**)&data);
    }

  con_dbg_save_pos();

  conn_destroy_task(task);

  return ret;
}

static void *conn_network_thread(void *param)
{
  network_task_s task;
  bool stopped = false;
  bool do_ping;
  uint32_t connid = -1;

  UNUSED(param);

  conn_task_start();

  do
    {
      int ret;
      int task_count;

      con_dbg_save_pos();

      conn_task_done();

      /* Do we have a new task?. Note, blocks until new task available */

      if (mq_receive(con->task_mq, (void *)&task, sizeof(task), 0) != sizeof(task))
        {
          con_dbg("Failed to get task (%d)!!! \n", get_errno());
          continue;
        }

      con_dbg_save_pos();

      conn_task_start();

      pthread_mutex_lock(&con->mutex);
      con->mq_task_count = (con->mq_task_count < 2) ? 0 : con->mq_task_count - 1;
      con->processing_task = true;
      pthread_mutex_unlock(&con->mutex);

      switch (task.type)
      {
      case NETWORK_TASK_STOP:
        con_dbg("Ending network task\n");
        stopped = true;
        break;
      case NETWORK_TASK_REQUEST:
        if (protocol_type == CON_PROTOCOL_HTTP && connid == -1)
          {
            con_dbg_save_pos();

            ret = request_connection(&connid, true);
            if (ret < 0)
              {
                con_dbg("request_connection on failed, skipping task\n");

                task.conn->context->process_status = NETWORK_ERROR;
                conn_complete_task_workflow(task.conn->context, ret);
                conn_destroy_task(task.conn);
                break;
              }
          }

        con_dbg_save_pos();

        ret = execute_task_conn_request(task.conn);
        if (ret != OK)
          {
            con_dbg("Task handling error: %d\n", ret);
            if (ret == NETWORK_ERROR) {
                con_dbg("Network Error...\n", ret);
            }
            sleep(1);
          }

        con_dbg_save_pos();

        pthread_mutex_lock(&con->mutex);
        task_count = con->mq_task_count;
        con_dbg("Task count: %d\n", task_count);
        pthread_mutex_unlock(&con->mutex);

        if (task_count == 0 && protocol_type == CON_PROTOCOL_HTTP)
          {
            con_dbg_save_pos();

            ret = request_connection(&connid, false);
            if (ret < 0)
              {
                con_dbg("request_connection id: %d off failed\n", connid);
              }
            connid = -1;
          }
        break;
      }

      con_dbg_save_pos();

      pthread_mutex_lock(&con->mutex);
      con->processing_task = false;
      do_ping = (task.type == NETWORK_TASK_REQUEST && con->mq_task_count == 0);
      pthread_mutex_unlock(&con->mutex);
      if (do_ping)
        {
          con_dbg_save_pos();

          /* Ping engine thread, to allow re-evaluation of deep-sleepiness. */

          con_dbg("Ping ts_engine for deep-sleep...\n");

          conn_ping_main_thread();
        }
    }
  while (!stopped);

  con_dbg_save_pos();

  conn_task_done();

  return NULL;
}

static int ts_network_give_new_task(network_task_s *task)
{
  FAR const struct timespec ts = { 0, 0 };
  DEBUGASSERT(task);

  if (mq_timedsend(con->task_mq, (void *)task, sizeof(*task), 0, &ts) < 0)
    {
      return ERROR;
    }

  pthread_mutex_lock(&con->mutex);
  if (task->type == NETWORK_TASK_STOP)
    {
      con->stopping = true;
    }
  con->mq_task_count++;
  DEBUGASSERT(con->mq_task_count < 255);
  pthread_mutex_unlock(&con->mutex);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void conn_set_protocol_type(conn_protocol_type_t type)
{
  /* TODO: This must be fixed for multi-connector handling. Make this part
   *       of con_str_t. */

  protocol_type = type;
}

void conn_free_pointer(void **ptr)
{
  if (!ptr)
    return;

  free(*ptr);
  *ptr = NULL;
}

int conn_init_boolean_from_json(cJSON **container, char *id, bool *dest)
{
  cJSON *obj;
  int type;

  DEBUGASSERT(container && *container && id && dest);

  obj = cJSON_GetObjectItem(*container, id);
  if(!obj)
    {
      con_dbg("Failed to find %s\n", id);
      return ERROR;
    }

  type = cJSON_type(obj);
  if (type != cJSON_False && type != cJSON_True)
    {
      con_dbg("Illegal type for boolean: %d\n", type);
      return ERROR;
    }

  *dest = cJSON_boolean(obj);

  return OK;
}

int conn_init_param_from_json(cJSON **container, char *id, char ** dest)
{
  char *tmp;
  cJSON *obj;
  int type;
  const char *valuestring;
  size_t valuestring_len;

  DEBUGASSERT(container && *container && id && dest);

  obj = cJSON_GetObjectItem(*container, id);
  if(!obj)
    {
      con_dbg("Failed to find %s\n", id);
      return ERROR;
    }

  type = cJSON_type(obj);
  if (type != cJSON_String)
    {
      con_dbg("Illegal type for string: %d\n", type);
      return ERROR;
    }

  valuestring = cJSON_string(obj);
  valuestring_len = strlen(valuestring);
  tmp = (char*) malloc(valuestring_len + 1);
  if (!tmp)
    {
      return ERROR;
    }

  memcpy(tmp, valuestring, valuestring_len + 1);

  *dest = tmp;

  return OK;
}

void conn_destroy_task(struct conn_network_task_s *task)
{
  conn_free_pointer((void**)&task);
}

struct conn_network_task_s* conn_create_network_task(
    const char *title,
    conn_workflow_context_s *context,
    conn_request_construct_t construct,
    conn_response_process_t process)
{
  size_t size = sizeof(struct conn_network_task_s);
  struct conn_network_task_s *task = (struct conn_network_task_s*)calloc(1, size);
  if (task)
    {
      task->title = title;
      task->context = context;
      task->construct = construct;
      task->process = process;
      task->process_stream = NULL;
      task->http.get_conn_params = NULL;
    }

  return task;
}

struct conn_network_task_s* conn_create_network_task_stream(
    const char *title,
    conn_workflow_context_s *context,
    conn_request_construct_t construct,
    conn_response_process_stream_t process_stream)
{
  struct conn_network_task_s *task =
      conn_create_network_task(title, context, construct, NULL);

  if (task)
    {
      task->process = NULL;
      task->process_stream = process_stream;
    }

  return task;
}

void conn_complete_task_workflow(conn_workflow_context_s *context, int err)
{
  if (context)
    {
      if (context->cb)
        {
          con_dbg_save_pos();
          pthread_mutex_lock(&con->mutex);
          if (!con->stopping)
            {
              pthread_mutex_unlock(&con->mutex);
              if (context->process_status)
                {
                  err = context->process_status;
                }
              con_dbg_save_pos();
              context->cb(err, context->priv);
            }
          else
            {
              pthread_mutex_unlock(&con->mutex);
            }
        }

      conn_free_pointer((void**)&context->payload);
      conn_free_pointer((void**)&context);
    }
}

int conn_network_give_new_conn_task(struct conn_network_task_s *conn_task)
{
  network_task_s task = {};

  DEBUGASSERT(conn_task);

  task.type = NETWORK_TASK_REQUEST;
  task.conn = conn_task;

  if (ts_network_give_new_task(&task) != OK)
    {
      con_dbg("Could not create a new task.\n"
          "Is the task queue full? TASK_MESSAGE_QUEUE_MAX = %d\n", (int)(TASK_MESSAGE_QUEUE_MAX));
      return ERROR;
    }

  return OK;
}

int conn_init(con_str_t *conn)
{
  int ret;
  pthread_attr_t attr;
  struct mq_attr mattr = {
      .mq_maxmsg = TASK_MESSAGE_QUEUE_MAX,
      .mq_msgsize = sizeof(network_task_s),
      .mq_curmsgs = 0,
      .mq_flags = 0
  };

  if (con != NULL)
    {
      con_dbg("Already initialized!\n");
      return ERROR;
    }

  protocol_type = CON_PROTOCOL_HTTP;
  con = conn;

  /* Note, blocking mode used for task_mq */

  con->task_mq = mq_open("ts_network_mq", O_RDWR | O_CREAT, 0666, &mattr);
  if (con->task_mq < 0)
    return ERROR;

  pthread_mutex_init(&con->mutex, NULL);

  pthread_attr_init(&attr);
  attr.stacksize = 1024 * 3;

  /* Start pthread to handle network. */

  ret = pthread_create(&con->thread,
      &attr,
      conn_network_thread,
      NULL);
  if (ret)
    {
      con_dbg("Can't create network thread: %d\n", ret);
      pthread_attr_destroy(&attr);
      pthread_mutex_destroy(&con->mutex);
      return ERROR;
    }
  pthread_attr_destroy(&attr);

  /* Setup deep-sleep hook */

  ret = ts_core_deepsleep_hook_add(conn_network_is_deepsleep_allowed, NULL);
  DEBUGASSERT(ret == OK);

  return ret;
}

void conn_empty_message_queue(void)
{
  network_task_s task = {};

  pthread_mutex_lock(&con->mutex);

  while (!con->stopping && con->mq_task_count > 0 && mq_receive(con->task_mq, (void *)&task, sizeof(task), 0) >= sizeof(task))
    {
      con_dbg("Purging task %d\n", con->mq_task_count);
      switch (task.type)
      {
      case NETWORK_TASK_REQUEST:
        pthread_mutex_unlock(&con->mutex);
        conn_complete_task_workflow(task.conn->context, ERROR);
        pthread_mutex_lock(&con->mutex);
        conn_destroy_task(task.conn);
        break;
      default:
        break;
      }
      con->mq_task_count--;
    }
  pthread_mutex_unlock(&con->mutex);
}

int conn_uninit(void)
{
  int ret = OK;
  network_task_s task = {};

  ts_core_deepsleep_hook_remove(conn_network_is_deepsleep_allowed);

  conn_empty_message_queue();

  con_dbg("Purging tasks finished\n");

  task.type = NETWORK_TASK_STOP;

  if (ts_network_give_new_task(&task) != OK)
    {
      con_dbg("Failed to create Stop task!\n");
      return ERROR;
    }

  /* Kill is to wake-up thread from blocked IO. */

  pthread_mutex_lock(&con->mutex);
  con->thread_joining = true;
  pthread_mutex_unlock(&con->mutex);
  pthread_kill(con->thread, CONFIG_CONNECTOR_SIGWAKEUP);

  pthread_join(con->thread, NULL);

  con->network_ready = false;
  con->thread_joining = false;
  con->processing_task = false;
  mq_close(con->task_mq);
  pthread_mutex_destroy(&con->mutex);

  con = NULL;

  conn_link_cleanup();

  return ret;
}

#ifdef CONFIG_THINGSEE_ENGINE

/* TODO: This is wrong place for this code. Pass function pointer or bulk
 *       of functions to connector, move application specific code out from
 *       here (to application? to ts_connector?). */

void conn_ping_main_thread(void)
{
  struct ts_engine_client client;
  int ret;
  bool would_block;

  pthread_mutex_lock(&con->mutex);
  would_block = con->thread_joining;
  pthread_mutex_unlock(&con->mutex);
  if (would_block)
    {
      con_dbg("cannot issue message to engine, engine waiting conn_comm.\n");
      return;
    }

  ret = ts_engine_client_init(&client);
  if (ret < 0)
    {
      con_dbg("ts_engine_client_init failed\n");
      return;
    }

  ret = ts_engine_client_ping(&client);
  if (ret < 0)
    {
      con_dbg("ts_engine_client_ping failed\n");
      goto errout;
    }

  ts_engine_client_uninit(&client);
  return;

errout:
  ts_engine_client_uninit(&client);
  return;
}

#else

void conn_ping_main_thread(void)
{
  /* Do nothing. */
}

#endif

#ifdef CONFIG_THINGSEE_ENGINE

/* TODO: Move application specific code out from here.
 * TODO: Connector is running in same task as main/application thread.
 *       We can pass cJSON object directly instead of doing
 *       load-full-content-string + parse-to-json + print-json-to-string +
 *       copy-string-to-pipe + read-string-from-pipe-to-string + parse-to-json
 *       exercise. Or, since this is Thingsee, write temporary file and pass
 *       filename.
 */

int conn_update_profile(char * const profile)
{
  struct ts_engine_client client;
  int ret;
  bool would_block;

  pthread_mutex_lock(&con->mutex);
  would_block = con->thread_joining;
  pthread_mutex_unlock(&con->mutex);

  if (would_block)
    {
      con_dbg("cannot issue message to engine, engine waiting conn_comm.\n");
      return ERROR;
    }

  ret = ts_engine_client_init(&client);
  if (ret < 0)
    {
      con_dbg("ts_engine_client_init failed\n");
      return ERROR;
    }

  ret = ts_engine_client_pause(&client);
  if (ret < 0)
    {
      con_dbg("ts_engine_client_pause failed\n");
      goto errout;
    }

  ret = ts_engine_client_write_profile_shm(&client, profile, strlen(profile) + 1);
  if (ret < 0)
    {
      con_dbg("ts_engine_client_write_profile failed\n");
      goto errout;
    }
  else
    {
      if (client.resp.hdr.status == OK)
        {
          con_dbg("profile accepted by engine, emptying request queue\n");
          conn_empty_message_queue();
        }
      else
        {
          con_dbg("profile not ok: %d\n", client.resp.hdr.status);

          ret = ts_engine_client_continue(&client);
          if (ret < 0)
            {
              con_dbg("ts_engine_client_continue failed\n");
            }
          goto errout;
        }
    }

  ret = ts_engine_client_reload_profile(&client);
  if (ret < 0)
    {
      con_dbg("ts_engine_client_reload_profile failed\n");
      goto errout;
    }

  ts_engine_client_uninit(&client);

  return OK;

errout:

  ts_engine_client_uninit(&client);

  return ERROR;
}
#endif
