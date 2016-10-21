/****************************************************************************
 * apps/ts_engine/engine/shutdown.c
 *
 * Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
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
#include <nuttx/arch.h>

#include <pthread.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <sys/types.h>
#include <stdlib.h>
#include <debug.h>
#include <assert.h>
#include <sys/stat.h>
#include <string.h>
#include <sys/wait.h>

#include <apps/thingsee/ts_core.h>
#ifndef CONFIG_ARCH_SIM
#include <apps/thingsee/modules/ts_emmc.h>
#include <apps/system/conman.h>
#include <arch/board/board.h>
#include <arch/board/board-reset.h>
#endif

#include "eng_dbg.h"
#include "parse.h"
#include "main.h"
#include "execute.h"
#include "shutdown.h"

#define MAX_CONMAN_STOP_TIME 5

#ifndef STDIN_FILENO
#  define STDIN_FILENO 0
#endif
#ifndef STDOUT_FILENO
#  define STDOUT_FILENO 1
#endif
#ifndef STDERR_FILENO
#  define STDERR_FILENO 2
#endif
#ifndef SIGKILL
#  define SIGKILL 9
#endif

#ifdef CONFIG_SYSTEM_CONMAN

struct
{
  void (*done_cb)(void *priv);
  void *done_priv;
  int time_count;
  pthread_t guard_threadid;
  pthread_mutex_t mutex;
} g_conman_shutdown;

static void *guard_stop_conman(void *priv)
{
  struct timespec rqtp = {};
  int ret;

  rqtp.tv_sec  = MAX_CONMAN_STOP_TIME * 2;
  rqtp.tv_nsec = 0;

  ret = nanosleep(&rqtp, NULL);
  if (ret < 0)
    {
      /* Sleep cancelled. */

      return NULL;
    }

  ret = pthread_mutex_trylock(&g_conman_shutdown.mutex);
  if (ret != 0)
    {
      /* Already shutting down. */

      return NULL;
    }

  eng_dbg(">> Connection manager stopping stuck, forcing shutdown.\n");

  if (g_conman_shutdown.done_cb)
    {
      g_conman_shutdown.done_cb(g_conman_shutdown.done_priv);
    }

  pthread_mutex_unlock(&g_conman_shutdown.mutex);

  return NULL;
}

static int conman_shutdown_timer(int timerid, void *priv)
{
  const unsigned int max_time_msec = MAX_CONMAN_STOP_TIME * 1000;
  const unsigned int time_step_msec = 50;
  struct conman_client_s *conman_client = priv;
  struct conman_status_s stat;
  int ret;

  ret = conman_client_get_connection_status(conman_client, &stat);
  if (ret != OK)
    {
      eng_dbg("conman_client_get_connection_status failed\n");
      goto done;
    }

  if (stat.conn_type == CONMAN_NONE && !stat.destroying_prev)
    {
      eng_dbg(">>> Connections destroyed.\n");
      goto done;
    }

  if (g_conman_shutdown.time_count >= max_time_msec)
    {
      /* Bailout after 5 seconds. */

      eng_dbg(">>> Could not destroy connections, doing forced shutdown.\n");

      goto done;
    }

  if ((g_conman_shutdown.time_count / 1000) !=
      ((g_conman_shutdown.time_count + time_step_msec) / 1000))
    {
      eng_dbg(">>> Waiting connections to be destroyed... %d/%d\n",
              (g_conman_shutdown.time_count + time_step_msec) / 1000,
              max_time_msec / 1000);
    }

  /* Reregister timer. */

  g_conman_shutdown.time_count += time_step_msec;
  ret = ts_core_timer_setup(TS_TIMER_TYPE_TIMEOUT, time_step_msec,
                            conman_shutdown_timer,
                            conman_client);
  DEBUGASSERT(ret >= 0);
  return OK;

done:
  conman_client_uninit(conman_client);
  free(conman_client);
  pthread_mutex_lock(&g_conman_shutdown.mutex);
  g_conman_shutdown.done_cb(g_conman_shutdown.done_priv);
  pthread_mutex_unlock(&g_conman_shutdown.mutex);
  return OK;
}

static bool no_deepsleep(void *priv)
{
  return false;
}

static void stop_conman(void (*stopped_callback)(void *priv), void *priv)
{
  struct conman_client_s *conman_client;
  pthread_attr_t attr;
  int ret;

  eng_dbg("> Stopping connection manager...\n");

  ret = pthread_mutex_init(&g_conman_shutdown.mutex, NULL);
  if (ret < 0)
    {
      eng_dbg("pthread_mutex_init failed, err: %d\n", ret);
      stopped_callback(priv);
      return;
    }

  /* Launch thread that will sleep 10 seconds and commence with shutdown
   * procedure. In case mainloop is not stuck, that thread is cancelled
   * before it will wake. If mainloop is stuck, that thread will ensure
   * shutdown.
   */

  pthread_attr_init(&attr);
  attr.stacksize = 1024 * 1.5;
  ret = pthread_create(&g_conman_shutdown.guard_threadid, &attr,
                       guard_stop_conman, priv);
  if (ret < 0)
    {
      eng_dbg("pthread_create guard_stop_conman failed, err: %d\n", ret);
      goto err_done;
    }

  conman_client = malloc(sizeof(*conman_client));
  if (!conman_client)
    {
      eng_dbg("malloc failed\n");
      goto err_done;
    }

  g_conman_shutdown.done_cb = stopped_callback;
  g_conman_shutdown.done_priv = priv;
  g_conman_shutdown.time_count = 0;

  ret = conman_client_init(conman_client);
  if (ret != OK)
    {
      eng_dbg("conman_client_init failed\n");
      goto err_done;
    }
  else
    {
      eng_dbg(">> Destroy all connections...\n");

      ret = conman_client_destroy_connection(conman_client, CONMAN_CONNID_ALL);
      if (ret != OK)
        {
          eng_dbg("conman_client_destroy_connection(CONMAN_CONNID_ALL) failed\n");
        }

      ts_core_deepsleep_hook_add(no_deepsleep, NULL);

      conman_shutdown_timer(-1, conman_client);
    }

  return;

err_done:
  pthread_mutex_lock(&g_conman_shutdown.mutex);
  stopped_callback(priv);
  pthread_mutex_unlock(&g_conman_shutdown.mutex);
}
#else
static void
stop_conman(void (*stopped_callback)(void *priv), void *priv)
{
  stopped_callback(priv);
}
#endif

static void iter_kill_task(struct tcb_s *tcb, void *arg)
{
  if (tcb->pid == 0 || tcb->pid == getpid())
    {
      /* Do not kill idle or current task. */

      return;
    }

  kill(tcb->pid, SIGKILL);
  (*(int *)arg)++;
}

static void do_system_shutdown(enum ts_engine_shutdown_type_e type)
{
  int i;
  int tries = 10;
  int start_count = 0;
  int curr_count;
  int ret;

  /* Shutdown sequence:
   *  1. Kill all other tasks.
   *  2. Close file descriptors greater than 2 for current task.
   *  3. Turn off display.
   *  4. Unmount filesystem.
   *  5. Go to new shutdown state (reset/standby).
   */

  /* Kill all tasks, except current and idle (pid: 0).
   * NOTE: NuttX does not implement SIGKILL handling, each task need to do
   * this for themselves.
   */

  dbg("Killing other tasks...\n");
  sched_foreach(iter_kill_task, (void *)&start_count);
  do
    {
      usleep(10 * 1000);
      curr_count = 0;
      sched_foreach(iter_kill_task, (void *)&curr_count);
      if (curr_count == 0)
        break;
    }
  while (tries--);
  dbg("...killed %d tasks. Unkillable count: %d.\n",
      start_count - curr_count, curr_count);

  /* Close file-descriptors above stdin/out/err. */

  dbg("Closing file-descriptors...\n");
  for (i = STDERR_FILENO + 1, curr_count = 0;
       i < CONFIG_NFILE_DESCRIPTORS + CONFIG_NSOCKET_DESCRIPTORS; i++)
    {
      ret = close(i);
      if (ret == OK)
        {
          curr_count++;
        }
    }
  dbg("...closed %d descriptors.\n", curr_count);

#ifndef CONFIG_ARCH_SIM
  /* Turn screen off (digital logic part of LCD needs to be turned off,
   * so must use board_lcdoff before standby). */

#ifdef CONFIG_THINGSEE_DISPLAY_MODULE
  dbg("Display off...\n");
  board_lcdoff();
#endif

  /* Unmount emmc. */

#ifdef CONFIG_THINGSEE_EMMC_MODULE
  dbg("Unmount SDcard...\n");
  emmc_handle_power_off(true);
#endif

  /* Go power-off mode. */

  if (type == TS_ENGINE_SYSTEM_SHUTDOWN_TYPE_STANDBY)
    {
      board_go_to_standby();
    }
  else if (type == TS_ENGINE_SYSTEM_SHUTDOWN_TYPE_DFU)
    {
      board_reset_to_system_bootloader();
    }
  else
    {
      board_systemreset();
    }
#endif

  while (1)
    {
      exit(0);
    }
}

static void engine_stopped_cb(void *priv)
{
  eng_dbg ("ts_engine exit\n");

  /* Engine stopped to 'safe' state. Now exit application to close file
   * descriptors (and drive sensor HW to powered-off state). Components
   * launching new pthreads must register atexit handler to  stop those
   * threads.
   */

  do_system_shutdown((enum ts_engine_shutdown_type_e) (intptr_t) priv);
  while (1);
}

void ts_engine_do_system_shutdown(struct ts_engine_app * const app,
                                  enum ts_engine_shutdown_type_e type)
{
  int ret;
  bool loop_goon = true;

  /* Stop engine. */

  ret = ts_engine_stop(app, true, true);
  if (ret < 0)
    {
      eng_dbg ("ts_engine_stop failed\n");
    }

  /* Start turning off conman. */

  stop_conman(engine_stopped_cb, (void *) (intptr_t) type);

  /* If mainloop is already running, just exit. */

  if (app->mainloop_started)
    {
      eng_dbg ("Mainloop already started.\n");

      return;
    }

  /* Otherwise, start running mainloop now. */

  eng_dbg ("Starting Thingsee main event loop for shutdown\n");

  app->mainloop_started = true;
  ts_core_mainloop(&loop_goon);
  while (1);
}
