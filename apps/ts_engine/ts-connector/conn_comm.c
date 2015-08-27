/****************************************************************************
 * apps/ts_engine/ts-connector/conn_comm.c
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

#include <arpa/inet.h>
#include <netinet/in.h>

#include <apps/netutils/dnsclient.h>
#include <apps/system/conman.h>

#include "connector.h"
#include "conn_comm.h"
#include "con_dbg.h"
#include "../engine/client.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define MAX_ADDRESS_RESOLV_ATTEMPTS   5
#define TASK_MESSAGE_QUEUE_MAX        10
#define HTTP_MSG_BUF_SIZE             256
#define NETWORK_ERROR                 (ERROR - 1)

#ifdef CONFIG_THINGSEE_CONNECTORS_PROTOCOL_DEBUG
#  define http_con_dbg(...)           con_dbg("<http> " __VA_ARGS__)
#else
#  define http_con_dbg(...)           ((void)0)
#endif

#ifndef CONFIG_CONNECTOR_SIGWAKEUP
#  define CONFIG_CONNECTOR_SIGWAKEUP 20
#endif

#if CONFIG_CONNECTOR_SIGWAKEUP > MAX_SIGNO
#  error "CONFIG_CONNECTOR_SIGWAKEUP invalid"
#endif

#define CONNECTION_RETRY_DELAY_SEC    10
#define CONNECTION_RETRIES            12

#define CONNECTION_SEND_TIMEOUT       20 /* secs */
#define CONNECTION_RECV_TIMEOUT       30 /* secs */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void *conn_network_thread(void *param);
static int ts_network_give_new_task(network_task_s *task);
static bool conn_network_is_deepsleep_allowed(void * const priv);
static void conn_ping_main_thread(void);
static int execute_http_request(struct sockaddr_in *srv_addr, uint16_t port, char *hdr,
    size_t hdrlen, char *pdata,
    size_t datalen, int *pstatus_code, char **pcontent,
    conn_workflow_context_s *context);
void conn_destroy_task(struct conn_network_task_s *task);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static con_str_t *con = NULL;

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

  ret = conman_client_init(&client);
  if (ret < 0)
    {
      con_dbg("conman_client_init failed\n");
      return ERROR;
    }

  if (!onoff)
    {
      DEBUGASSERT(*connid >= 0);

      ret = conman_client_destroy_connection(&client, *connid);
      if (ret < 0)
        {
          con_dbg("conman_client_destroy_connection failed\n");
        }
      goto out;
    }

  ret = conman_client_request_connection(&client, CONMAN_DATA, connid);
  if (ret < 0)
    {
      con_dbg("conman_client_request_connection failed\n");
      goto out;
    }

  while (true)
    {
      struct timespec sleeptime;

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

  conman_client_uninit(&client);

  return ret;
}

static int execute_task_conn_request(struct conn_network_task_s *task)
{
  char *hdr = NULL;
  char *data = NULL;
  size_t hdrlen = 0;
  size_t datalen = 0;
  int ret;
  struct conn_network_task_s *next_task = NULL;

  DEBUGASSERT(task);

  con_dbg("%s\n", task->title);

  ret = task->construct(task->context, &hdr, &data);
  if (ret == OK)
    {
      hdrlen = (hdr != NULL) ? strlen(hdr) : 0;
      datalen = (data != NULL) ? strlen(data) : 0;
      if (hdrlen <= 0)
        ret = ERROR;
    }

  if (ret == OK)
    {
      int status_code;
      char *content = NULL;

      ret = execute_http_request(&con->srv_ip4addr, con->port,
          hdr, hdrlen, data, datalen, &status_code, &content,
          task->context);
      if (ret == OK)
        {
          next_task = task->process(task->context, status_code, content);
          if (next_task != NULL)
            ret = conn_network_give_new_conn_task(next_task);
        }
      conn_free_pointer((void**)&content);
      conn_free_pointer((void**)&hdr);
    }

  if (next_task == NULL) /* We have reached the end of the workflow */
    {
      /* Note, 'data' will be freed in conn_complete_task_workflow() */
      conn_complete_task_workflow(task->context, ret);
      data = NULL;
    }
  else if (ret == 0 && !task->context->payload)
    {
      conn_free_pointer((void**)&data);
    }

  conn_destroy_task(task);

  return ret;
}

static void *conn_network_thread(void *param)
{
  network_task_s task;
  bool stopped = false;
  bool do_ping;
  uint32_t connid = -1;

  (void)param;

  do
    {
      int ret;
      int task_count;

      /* Do we have a new task?. Note, blocks until new task available */
      if (mq_receive(con->task_mq, (void *)&task, sizeof(task), 0) != sizeof(task))
        {
          con_dbg("Failed to get task (%d)!!! \n", get_errno());
          continue;
        }

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
        if (connid == -1)
          {
            ret = request_connection(&connid, true);
            if (ret < 0)
              {
                con_dbg("request_connection on failed, skipping task\n");
                conn_destroy_task(task.conn);
                break;
              }
          }

        ret = execute_task_conn_request(task.conn);
        if (ret != OK)
          {
            con_dbg("Task handling error: %d\n", ret);
            if (ret == NETWORK_ERROR) {
                con_dbg("Network Error...\n", ret);
            }
            sleep(1);
          }

        pthread_mutex_lock(&con->mutex);
        task_count = con->mq_task_count;
        con_dbg("Task count: %d\n", task_count);
        pthread_mutex_unlock(&con->mutex);

        if (task_count == 0)
          {
            ret = request_connection(&connid, false);
            if (ret < 0)
              {
                con_dbg("request_connection id: %d off failed\n", connid);
              }
            connid = -1;
          }
        break;
      }

      pthread_mutex_lock(&con->mutex);
      con->processing_task = false;
      do_ping = (task.type == NETWORK_TASK_REQUEST && con->mq_task_count == 0);
      pthread_mutex_unlock(&con->mutex);
      if (do_ping)
        {
          /* Ping engine thread, to allow re-evaluation of deep-sleepiness. */

          con_dbg("Ping ts_engine for deep-sleep...\n");

          conn_ping_main_thread();
        }
    }
  while (!stopped);

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

static int get_server_address(struct sockaddr_in *addr, const char *hostname)
{
  int ret;

  DEBUGASSERT(addr && hostname);

  con_dbg("Getting IP address for '%s'\n", hostname);

  ret = dns_gethostip(hostname, &addr->sin_addr.s_addr);
  if (ret != OK)
    {
      con_dbg("Failed to get address for '%s'\n", hostname);
      return ret;
    }

  addr->sin_family = AF_INET;

  con_dbg("Got address %d.%d.%d.%d for '%s'\n",
      addr->sin_addr.s_addr & 0xff,
      (addr->sin_addr.s_addr >> 8) & 0xff,
      (addr->sin_addr.s_addr >> 16) & 0xff,
      addr->sin_addr.s_addr >> 24,
      hostname);
  return OK;
}

static int execute_http_request(struct sockaddr_in *srv_addr, uint16_t port, char *hdr,
    size_t hdrlen, char *pdata,
    size_t datalen, int *pstatus_code, char **pcontent,
    conn_workflow_context_s *context)
{
  const char *bufs[3] = { hdr, pdata, NULL };
  const size_t lens[3] = { hdrlen, datalen, 0 };
  const char **pbuf = bufs;
  const size_t *plen = lens;
  char inbuf[HTTP_MSG_BUF_SIZE];
  char linebuf[HTTP_MSG_BUF_SIZE];
  size_t linepos;
  size_t len;
  char prev_ch;
  int linenum;
  int num;
  int sock;
  int ret;
  int contentlen;
  int allocated_contentlen;
  size_t pos;
  char *host;
  struct timeval tv;

  DEBUGASSERT(srv_addr && hdr && pdata && pstatus_code && pcontent);

  *pstatus_code = ERROR;

  http_con_dbg("HTTP:\n%s%s", hdr, pdata);

  if (context->url.host)
    {
      con->srv_ip4addr.sin_addr.s_addr = 0;
      host = context->url.host;
      port = context->url.port;
    }
  else
    {
      host = con->host;
    }

  if (srv_addr == &con->srv_ip4addr)
    {
      /* Fetch server IP address. */
      if (con->srv_ip4addr.sin_addr.s_addr == 0)
        {
          con->network_ready = false;
          if (get_server_address(&con->srv_ip4addr, host) != OK)
            return NETWORK_ERROR;

          con->network_ready = true;
        }
    }

  /* Open HTTP connection to server. */
  http_con_dbg("Open socket...\n");
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
    return NETWORK_ERROR;

  http_con_dbg("Connect to port %d ...\n", port);
  srv_addr->sin_port = htons(port);
  ret = connect(sock, (struct sockaddr *)srv_addr, sizeof(*srv_addr));
  if (ret < 0)
    {
      if (srv_addr == &con->srv_ip4addr)
        {
          /* Could not connect to server. Try updating server IP address on
           * next try. */
          memset(&con->srv_ip4addr, 0, sizeof(con->srv_ip4addr));
        }
      goto err_close;
    }

  /* Set up a send timeout */
  tv.tv_sec = CONNECTION_SEND_TIMEOUT;
  tv.tv_usec = 0;
  ret = setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv,
                   sizeof(struct timeval));
  if (ret < 0)
    {
      con_dbg("Setting SO_SNDTIMEO failed, errno: %d\n", errno);
    }
  else
    {
      http_con_dbg("SO_SNDTIMEO := %d secs\n", CONNECTION_SEND_TIMEOUT);
    }

  http_con_dbg("Send HTTP...\n");
  /* Write buffers in 64 byte chunks. */
  while (*pbuf) {
      const char *buf = *pbuf;
      len = *plen;
      ++pbuf;
      ++plen;

      do {
          ssize_t nwritten;

          nwritten = send(sock, buf, len, 0);
          http_con_dbg("send, ret=%d\n", nwritten);
          if (nwritten < 0)
            goto err_close;

          buf += nwritten;
          len -= nwritten;
      } while (len);
  }

  http_con_dbg("Wait response...\n");

  /* Set up a receive timeout */
  tv.tv_sec = CONNECTION_RECV_TIMEOUT;
  tv.tv_usec = 0;
  ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv,
                   sizeof(struct timeval));
  if (ret < 0)
    {
      con_dbg("Setting SO_RCVTIMEO failed, errno: %d\n", errno);
    }
  else
    {
      http_con_dbg("SO_RCVTIMEO := %d secs\n", CONNECTION_RECV_TIMEOUT);
    }

  /* Read HTTP header. */
  linepos = 0;
  prev_ch = 0;
  linenum = 0;
  contentlen = 0;
  len = 0;
  *pcontent = NULL;
  allocated_contentlen = 0;
  do {
      ret = recv(sock, inbuf, sizeof(inbuf), 0);
      http_con_dbg("recv, ret=%d\n", ret);
      if (ret <= 0)
        goto invalid_response;

      for (pos = 0; pos < ret; pos++) {
          char cur_ch = inbuf[pos];

          if (linepos < sizeof(linebuf))
            linebuf[linepos++] = cur_ch;
          else
            linebuf[sizeof(linebuf) - 1] = '\0';

          if (prev_ch == '\r' && cur_ch == '\n') {
              /* EOL line found! */
              DEBUGASSERT(linepos >= 2);

              if (linepos <= sizeof(linebuf))
                linebuf[linepos - 2] = '\0';
              linepos -= 2;

              http_con_dbg("Response line from server: [%s]\n", linebuf);

              if (linenum == 0) {
                  /* First line, "HTTP/1.1 <code> " */
                  num = sscanf(linebuf, "HTTP/1.1 %d ", pstatus_code);
                  if (num != 1) {
                      /* Not HTTP response. */
                      goto invalid_response;
                  }
              } else if (linenum > 0) {
                  /* HTTP Headers, wait for empty line or "Content-Length: <len>" */
                  if (linepos == 0) {
                      /* Empty line, start of content. */

                      /* inbuf probably has part of content data already. */
                      pos++;
                      if (pos < ret) {
                          linepos = 0;
                          len = ret - pos;
                          memmove(inbuf, &inbuf[pos], len);
                      } else {
                          linepos = 0;
                          len = 0;
                      }

                      http_con_dbg("Handle content data?\n");
                      goto handle_content;
                  }

                  num = sscanf(linebuf, "Content-Length: %d", &contentlen);
                  if (num == 1) {
                      /* Got content length! */
                      http_con_dbg("HTTP Content length = %d!\n", contentlen);
                      if (contentlen > 0)
                        {
                          allocated_contentlen = contentlen + 1;
                          *pcontent = (char*)malloc(allocated_contentlen);
                          if (*pcontent)
                            (*pcontent)[0] = '\0';
                          else
                            {
                              http_con_dbg("Could not allocate '%d' bytes to receive server content!\n", allocated_contentlen);
                            }
                        }
                  }
              }

              linepos = 0;
              linenum++;
          }

          prev_ch = cur_ch;
      }
  } while (ret >= 0);

  /* Should not get here. */
  DEBUGASSERT(false);

  /* Handle content from server. */
handle_content:
  do {
      size_t curlen;

      for (pos = 0; pos < len; pos++) {
          char cur_ch = inbuf[pos];

          if (linepos == sizeof(linebuf) - 1) {
              linebuf[sizeof(linebuf) - 1] = '\0';

              http_con_dbg("Content: [%s]\n", linebuf);
              if (*pcontent)
                {
                  if ((strlen(*pcontent) + strlen(linebuf) < allocated_contentlen))
                    strcat(*pcontent, linebuf);
                  else
                    {
                      http_con_dbg("Not enough room left to concatenate content: (%d + %d >= %d)\n", strlen(*pcontent), strlen(linebuf), allocated_contentlen);
                    }
                }

              linepos = 0;
          }

          linebuf[linepos++] = cur_ch;
      }

      contentlen -= len;
      if (contentlen <= 0)
        break;

      curlen = contentlen;
      if (curlen > sizeof(inbuf))
        curlen = sizeof(inbuf);

      /* Read more content. */
      ret = recv(sock, inbuf, curlen, 0);
      http_con_dbg("recv, ret=%d\n", ret);
      if (ret <= 0)
        break;

      len = ret;
  } while (len > 0);

  if (linepos) {
      linebuf[linepos] = '\0';
      http_con_dbg("Content: [%s]\n", linebuf);
      if (*pcontent)
        {
          if ((strlen(*pcontent) + strlen(linebuf) < allocated_contentlen))
            strcat(*pcontent, linebuf);
          else
            {
              http_con_dbg("Not enough room left to concatenate content: (%d + %d >= %d)\n", strlen(*pcontent), strlen(linebuf), allocated_contentlen);
            }
        }
  }

  /* done:*/
  http_con_dbg("Done!\n");
  close(sock);
  http_con_dbg("Socket closed!\n");
  return OK;

invalid_response:
  http_con_dbg("Invalid HTTP response!\n");
  close(sock);
  http_con_dbg("Socket closed!\n");
  return ERROR;

err_close:
  close(sock);
  http_con_dbg("Socket closed!\n");
  return NETWORK_ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

  DEBUGASSERT(container && *container && id && dest);

  obj = cJSON_GetObjectItem(*container, id);
  if(!obj)
    {
      con_dbg("Failed to find %s\n", id);
      return ERROR;
    }

  if (obj->type != cJSON_False && obj->type != cJSON_True)
    {
      con_dbg("Illegal type for boolean: %d\n", obj->type);
      return ERROR;
    }

  *dest = (obj->valueint == 1) ? true : false;

  return OK;
}

int conn_init_param_from_json(cJSON **container, char *id, char ** dest)
{
  char *tmp;
  cJSON *obj;

  DEBUGASSERT(container && *container && id && dest);

  obj = cJSON_GetObjectItem(*container, id);
  if(!obj)
    {
      con_dbg("Failed to find %s\n", id);
      return ERROR;
    }

  if (obj->type != cJSON_String)
    {
      con_dbg("Illegal type for string: %d\n", obj->type);
      return ERROR;
    }
  tmp = (char*) malloc(strlen(obj->valuestring) + 1);
  if (!tmp)
    {
      return ERROR;
    }

  memcpy(tmp, obj->valuestring, strlen(obj->valuestring) + 1);

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
  struct conn_network_task_s *task = (struct conn_network_task_s*)malloc(size);
  if (task)
    {
      task->title = title;
      task->context = context;
      task->construct = construct;
      task->process = process;
    }

  return task;
}

void conn_complete_task_workflow(conn_workflow_context_s *context, int err)
{
  if (context)
    {
      if (context->cb)
        {
          pthread_mutex_lock(&con->mutex);
          if (!con->stopping)
            {
              pthread_mutex_unlock(&con->mutex);
              context->cb(err, context->priv);
            }
          else
            {
              pthread_mutex_unlock(&con->mutex);
            }
        }

      conn_free_pointer((void**)&context->payload);
      conn_free_pointer((void**)&context->url.host);
      conn_free_pointer((void**)&context->url.api);
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

  return ret;
}

#ifdef CONFIG_THINGSEE_ENGINE

static void conn_ping_main_thread(void)
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

static void conn_ping_main_thread(void)
{
  /* Do nothing. */
}

#endif

#ifdef CONFIG_THINGSEE_ENGINE
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
