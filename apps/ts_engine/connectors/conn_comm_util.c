/****************************************************************************
 * apps/ts_engine/connectors/conn_comm_util.c
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
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#include <apps/netutils/dnsclient.h>

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

#ifdef CONFIG_THINGSEE_CONNECTORS_DEBUG
static struct {
  pthread_mutex_t mutex;
  int head;
  struct {
    const char *func;
    int line;
    time_t timestamp;
  } queue[20];
} g_conn_comm_dbg =
{
  .mutex = PTHREAD_MUTEX_INITIALIZER
};
#endif

static struct {
  pthread_mutex_t mutex;
  time_t start_time;
  bool busy;
} g_conn_task_monitor =
{
  .mutex = PTHREAD_MUTEX_INITIALIZER
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int conn_comm_get_server_address(struct sockaddr_in *addr,
                                 const char *hostname)
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

#ifdef CONFIG_THINGSEE_CONNECTORS_DEBUG
void con_dbg_push_location(const char *func, int line)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);

  pthread_mutex_lock(&g_conn_comm_dbg.mutex);
  if (++g_conn_comm_dbg.head >= sizeof(g_conn_comm_dbg.queue) /
                                sizeof(g_conn_comm_dbg.queue[0]))
    {
      g_conn_comm_dbg.head = 0;
    }
  g_conn_comm_dbg.queue[g_conn_comm_dbg.head].func = func;
  g_conn_comm_dbg.queue[g_conn_comm_dbg.head].line = line;
  g_conn_comm_dbg.queue[g_conn_comm_dbg.head].timestamp = ts.tv_sec;
  pthread_mutex_unlock(&g_conn_comm_dbg.mutex);
}
#endif

void conn_dbg_print_thread_track(void)
{
#ifdef CONFIG_THINGSEE_CONNECTORS_DEBUG
  int count = sizeof(g_conn_comm_dbg.queue) / sizeof(g_conn_comm_dbg.queue[0]);
  struct timespec ts;
  int pos;
  int num;

  clock_gettime(CLOCK_MONOTONIC, &ts);

  /* Print connector traces */

  pthread_mutex_lock(&g_conn_comm_dbg.mutex);

  dbg("conn_comm thread trace:\n");

  for (num = 0, pos = g_conn_comm_dbg.head; count > 0; count--, num++, pos--)
    {
      if (g_conn_comm_dbg.queue[pos].func == NULL)
        {
          break;
        }

      dbg(" %2d: <%s>:%d    (age: %d secs)\n",
          num, g_conn_comm_dbg.queue[pos].func,
          g_conn_comm_dbg.queue[pos].line,
          ts.tv_sec - g_conn_comm_dbg.queue[pos].timestamp);

      if (pos == 0)
        {
          pos = sizeof(g_conn_comm_dbg.queue) /
                sizeof(g_conn_comm_dbg.queue[0]);
        }
    }

  pthread_mutex_unlock(&g_conn_comm_dbg.mutex);
#endif
}

void conn_task_start(void)
{
  struct timespec ts;

  pthread_mutex_lock(&g_conn_task_monitor.mutex);

  clock_gettime(CLOCK_MONOTONIC, &ts);
  g_conn_task_monitor.busy = true;
  g_conn_task_monitor.start_time = ts.tv_sec;

  pthread_mutex_unlock(&g_conn_task_monitor.mutex);
}

void conn_task_done(void)
{
  pthread_mutex_lock(&g_conn_task_monitor.mutex);

  g_conn_task_monitor.busy = false;

  pthread_mutex_unlock(&g_conn_task_monitor.mutex);
}

int conn_task_processing_secs(void)
{
  struct timespec ts;
  int secs;

  pthread_mutex_lock(&g_conn_task_monitor.mutex);

  if (g_conn_task_monitor.busy)
    {
      clock_gettime(CLOCK_MONOTONIC, &ts);

      secs = ts.tv_sec - g_conn_task_monitor.start_time;
    }
  else
    {
      secs = -1;
    }

  pthread_mutex_unlock(&g_conn_task_monitor.mutex);

  return secs;
}
