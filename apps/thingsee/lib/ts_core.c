/****************************************************************************
 * apps/thingsee/lib/ts_core.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Authors: Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *            Sami Pelkonen <sami.pelkonen@haltian.com>
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <queue.h>
#include <debug.h>
#include <nuttx/clock.h>
#include <arch/board/board.h>
#include <arch/board/board-deepsleep.h>
#include <arch/board/board-reset.h>

#include <apps/thingsee/ts_core.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

//#define TS_CORE_PERF_DEBUG

#ifdef TS_CORE_PERF_DEBUG
#  define TS_CORE_PERF_FILES_LIMIT      90
#  define TS_CORE_PERF_TIMERS_LIMIT     90
#endif

#ifdef CONFIG_THINGSEE_DEEPSLEEP_DEBUG
#  define TS_CORE_DEEPSLEEP_DEBUG 1
#else
//#define TS_CORE_DEEPSLEEP_DEBUG
#endif

/* Default sleep time when there are no file descriptors / timers active */

#define CORE_DEFAULT_SLEEP_MS           1000

/* Maximum non-deep-sleep sleep time */

#define TS_CORE_MAX_POLL_TIMEOUT_MSEC   2500

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* File descriptor entry */

struct file_s
{
  /* Single linked list entry */

  sq_entry_t entry;

  /* Monitored file descriptor */

  int fd:20;

  /* File descriptor active */

  bool active:1;

  /* File descriptor poll type */

  pollevent_t events;

  /* File descriptor callback function */

  ts_fd_callback_t callback;

  /* Private data for callback function */

  void * priv;

  /* Name of function that registered this entry. */

  const char *reg_func_name;
};

/* Timer flags */

struct timer_flags_s
{
  bool active:1;                                /* Timer is active */
  bool is_new:1;                                /* Timer is new and not yet active */
  enum ts_timer_type_e type:4;                  /* Timer type */
};

/* Timer entry */

struct timer_s
{
  /* Single linked list entry */

  sq_entry_t entry;

  /* Timer ID */

  uint32_t id;

  /* Timer flags */

  struct timer_flags_s flags;

  union
  {
    /* Members for interval and timeout timers. */

    struct
    {
      /* System tick count when timer expires */

      uint32_t expires;

      /* Timer interval / timeout in system ticks */

      uint32_t ticks;

      /* Timer callback function */

      ts_timer_callback_t callback;
    };

    /* Members for date/RTC timers. */

    struct
    {
      /* Absolute time when date timer expires */

      struct timespec date_expires;

      /* Date timer callback function */

      ts_timer_date_callback_t date_callback;
    };
  };

  /* Private data for callback function */

  void * priv;
};

/* Deep-sleep hook entry */

struct deepsleep_hook_s
{
  /* Single linked list entry */

  sq_entry_t entry;

  /* Hook function */

  ts_deepsleep_hook_t hook;

  /* Private data for hook function */

  void * priv;

  /* Name of function that registered this entry. */

  const char *reg_func_name;
};

/* Thingsee core internal structure */

struct ts_core_s
{
  /* File descriptor queue */

  sq_queue_t files;

  /* Timer queue */

  sq_queue_t timers;

  /* Deep-sleep hook queue */

  sq_queue_t deepsleep_hooks;

  /* Deep-sleep watchdog timer id */

  int deepsleep_watchdog_timer;

#ifdef TS_CORE_PERF_DEBUG
  struct {
    uint32_t start_ticks;
    uint32_t prev_ticks;

    uint32_t poll;
    uint32_t files;
    uint32_t nfiles;
    uint32_t timers;
    uint32_t ntimers;
    uint32_t sleep;
    uint32_t total;
  } elapsed;
#endif /* TS_CORE_PERFORMANCE_DEBUG */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Thingsee core internals */

static struct ts_core_s g_ts_core;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ts_core_get_timer_id
 *
 * Description:
 *   Get next available timer ID
 *
 * Input Parameters:
 *   ts_core     - Pointer to Thingsee core library structure
 *
 * Returned Value:
 *   Timer ID (>= 0) when timer was created successfully
 *  -1 (ERROR) means the function was executed unsuccessfully
 *
 ****************************************************************************/
static int ts_core_get_timer_id(struct ts_core_s * const ts_core)
{
  struct timer_s * timer;
  int timer_id = TS_CORE_FIRST_TIMER_ID;

  DEBUGASSERT(ts_core != NULL);

  timer = (struct timer_s *)sq_peek(&ts_core->timers);

  /* Search timer queue for available timer ID */

  while (timer)
    {
      if (timer->flags.active && timer->id >= timer_id)
        {
          /* Update next available timer ID */

          timer_id = timer->id + 1;
        }

      /* Move to next timer in queue */

      timer = (struct timer_s *)sq_next(&timer->entry);
    }

  return timer_id;
}

/****************************************************************************
 * Name: ts_core_gc_timers
 *
 * Description:
 *   Garbage collect timer queue
 *
 * Input Parameters:
 *   ts_core     - Pointer to Thingsee core library structure
 *
 ****************************************************************************/
static void ts_core_gc_timers(struct ts_core_s * const ts_core)
{
  struct timer_s * timer;

  DEBUGASSERT(ts_core != NULL);

  timer = (struct timer_s *)sq_peek(&ts_core->timers);
  while (timer)
    {
      /* Check if timer is new. */

      if (timer->flags.is_new)
        {
          /* Timer was added on this iteration of main-event loop, now remove
           * the 'is_new' mark so that timer becomes active for next round.
           */

          timer->flags.is_new = false;
        }

      /* Check if timer has been marked as deleted */

      if (!timer->flags.active)
        {
          struct timer_s * deleted = timer;

          /* Move to next timer in queue */

          timer = (struct timer_s *)sq_next(&timer->entry);

          /* Remove deleted timer from queue */

          sq_rem(&deleted->entry, &ts_core->timers);

          /* Free memory associated with timer */

          free(deleted);
        }
      else
        {
          /* Move to next timer in queue */

          timer = (struct timer_s *)sq_next(&timer->entry);
        }
    }
}

/****************************************************************************
 * Name: ts_core_timer_get_timeout
 *
 * Description:
 *   Get timeout to next timer
 *
 * Input Parameters:
 *   ts_core     - Pointer to Thingsee core internals
 *
 * Returned Value:
 *   Timeout in milliseconds to next timer event
 *  -1 when there is no registered timers
 *
 ****************************************************************************/
static int ts_core_timer_get_timeout(struct ts_core_s * const ts_core)
{
  struct timer_s * timer = (struct timer_s *)sq_peek(&ts_core->timers);
  uint32_t const current_tickcount = clock_systimer();
  int timeout_ms = -1;
  int ret;

  if (timer)
    {
      uint32_t next_to_expire = UINT32_MAX;
      struct timespec curr_ts;

      /* Get current time. */

      ret = clock_gettime(CLOCK_MONOTONIC, &curr_ts);
      DEBUGASSERT(ret != ERROR);

      while (timer)
        {
          /* Check that timer is active */

          if (timer->flags.active && !timer->flags.is_new &&
              timer->flags.type != TS_TIMER_TYPE_DATE)
            {
              /* Get lowest tick count */

              if (timer->expires < next_to_expire)
                next_to_expire = timer->expires;
            }
          else if (timer->flags.active && !timer->flags.is_new &&
                   timer->flags.type == TS_TIMER_TYPE_DATE)
            {
              int64_t msec_diff;
              uint32_t ticks_diff;

              /* Get difference between current time and expire time. */

              if (curr_ts.tv_sec > timer->date_expires.tv_sec)
                {
                  /* Already expired. */

                  next_to_expire = current_tickcount;
                }
              else
                {
                  msec_diff = timer->date_expires.tv_sec - curr_ts.tv_sec;
                  msec_diff *= 1000;

                  /* 'timespec->tv_nsec' is signed long. */

                  msec_diff += (timer->date_expires.tv_nsec - curr_ts.tv_nsec) /
                               (1000 * 1000);

                  if (msec_diff <= 0)
                    {
                      /* Already expired, current_tickcount is lowest possible
                       * tick count. */

                      next_to_expire = current_tickcount;
                    }
                  else
                    {
                      if (msec_diff > UINT32_MAX)
                        msec_diff = UINT32_MAX;

                      ticks_diff = MSEC2TICK((uint32_t)msec_diff);

                      /* Get lowest tick count */

                      if (current_tickcount + ticks_diff < next_to_expire)
                        next_to_expire = current_tickcount + ticks_diff;
                    }
                }
            }

          /* Move to next timer in queue */

          timer = (struct timer_s *)sq_next(&timer->entry);
        }

      if (next_to_expire > current_tickcount)
        {
          /* Set timeout to next timer event in milliseconds */

          timeout_ms = TICK2MSEC(next_to_expire - current_tickcount);
        }
      else
        {
          /* Timer has expired */

          timeout_ms = 0;
        }
    }

  return timeout_ms;
}

/****************************************************************************
 * Name: ts_core_timer_get_date_timeout
 *
 * Description:
 *   Get timeout to next date timer
 *
 * Input Parameters:
 *   ts_core     - Pointer to Thingsee core internals
 *
 * Returned Value:
 *   Timeout in seconds to next date timer event
 *  -1 when there is no registered date timers
 *
 ****************************************************************************/
static int ts_core_timer_get_date_timeout(struct ts_core_s * const ts_core)
{
  struct timer_s * timer = (struct timer_s *)sq_peek(&ts_core->timers);
  int timeout_sec = -1;
  uint32_t smallest = UINT32_MAX;
  struct timespec curr_ts;
  int ret;

  if (!timer)
    return timeout_sec;

  /* Get current time. */

  ret = clock_gettime(CLOCK_MONOTONIC, &curr_ts);
  DEBUGASSERT(ret != ERROR);

  while (timer)
    {
      /* Check that timer is active */

      if (timer->flags.active && !timer->flags.is_new &&
          timer->flags.type == TS_TIMER_TYPE_DATE)
        {
          time_t secs;
          int64_t nsecs;

          if (timer->date_expires.tv_sec < curr_ts.tv_sec)
            {
              /* Already expired (and active). */

              return 0;
            }

          /* Get time to expiring. */

          nsecs = (long)timer->date_expires.tv_sec - (long)curr_ts.tv_sec;
          nsecs *= (1000 * 1000 * 1000);
          nsecs += (long)timer->date_expires.tv_nsec - (long)curr_ts.tv_nsec;

          if (nsecs <= 0)
            {
              /* Already expired (and active). */

              return 0;
            }

          secs = nsecs / (1000 * 1000 * 1000);

          /* Add extra second so that we will deep-sleep slightly pass the
           * timer expiry time. This is because deep-sleep RTC wake-up
           * works in one second quantity and we want to avoid busy looping
           * for date timers.
           */

          secs += (timer->date_expires.tv_nsec != curr_ts.tv_nsec);

          if (secs < smallest)
            smallest = secs;
        }

      /* Move to next timer in queue */

      timer = (struct timer_s *)sq_next(&timer->entry);
    }

  return smallest;
}
/****************************************************************************
 * Name: ts_core_process_timers
 *
 * Description:
 *   Get timeout to next timer event in milliseconds
 *
 ****************************************************************************/
static int ts_core_process_timers(struct ts_core_s * const ts_core)
{
  struct timer_s * timer;

  DEBUGASSERT(ts_core != NULL);

  /* Get first expired timer from queue */

  timer = (struct timer_s *)sq_peek(&ts_core->timers);
  while (timer)
    {
      /* Check if systick based timer is active and has expired */

      if (timer->flags.active && !timer->flags.is_new &&
          timer->flags.type != TS_TIMER_TYPE_DATE &&
          timer->expires < clock_systimer())
        {
          int ret;

#ifdef TS_CORE_PERF_DEBUG
          ts_core->elapsed.prev_ticks = clock_systimer();
#endif
          /* Execute timer callback */

          ret = timer->callback(timer->id, timer->priv);

#ifdef TS_CORE_PERF_DEBUG
          ts_core->elapsed.timers += TICK2MSEC(clock_systimer() -
                                               ts_core->elapsed.prev_ticks);
          ts_core->elapsed.ntimers++;
#endif

          if (ret < 0)
            dbg("Timer %d callback returned %d\n", timer->id, ret);

          /* Check if timer was one shot timeout timer */

          if (timer->flags.type == TS_TIMER_TYPE_TIMEOUT)
            {
              /* Deactivate timer */

              timer->flags.active = false;
            }
          else if (timer->flags.type == TS_TIMER_TYPE_INTERVAL)
            {
              uint32_t const current_tickcount = clock_systimer();

              /* Set next timeout event */

              timer->expires += timer->ticks;

              /* Check if expiry time has already passed */

              if (timer->expires < current_tickcount)
                {
                  /* Mark timer to expire right away */

                  timer->expires = current_tickcount;
                }
            }
        }

      /* Check if RTC based timer is active */

      if (timer->flags.active && !timer->flags.is_new &&
          timer->flags.type == TS_TIMER_TYPE_DATE)
        {
          struct timespec curr_ts;
          int ret;

          /* Get current time. */

          ret = clock_gettime(CLOCK_MONOTONIC, &curr_ts);
          DEBUGASSERT(ret != ERROR);

          /* Check if timer has expired. */

          if (timer->date_expires.tv_sec < curr_ts.tv_sec ||
              (timer->date_expires.tv_sec == curr_ts.tv_sec &&
               timer->date_expires.tv_nsec < curr_ts.tv_nsec))
            {
#ifdef TS_CORE_PERF_DEBUG
              ts_core->elapsed.prev_ticks = clock_systimer();
#endif
              /* Execute timer callback */

              ret = timer->date_callback(timer->id, &timer->date_expires,
                                         timer->priv);

#ifdef TS_CORE_PERF_DEBUG
              ts_core->elapsed.timers += TICK2MSEC(clock_systimer() -
                                                   ts_core->elapsed.prev_ticks);
              ts_core->elapsed.ntimers++;
#endif

              if (ret < 0)
                dbg("Date timer %d callback returned %d\n", timer->id, ret);

              /* Deactivate timer */

              timer->flags.active = false;
            }
        }

      /* Get next timer from queue */

      timer = (struct timer_s *)sq_next(&timer->entry);
    }

  return OK;
}

/****************************************************************************
 * Name: ts_core_gc_files
 *
 * Description:
 *   Garbage collect files queue
 *
 * Input Parameters:
 *   ts_core     - Pointer to Thingsee core library structure
 *
 ****************************************************************************/
static void ts_core_gc_files(struct ts_core_s * const ts_core)
{
  struct file_s * file;

  DEBUGASSERT(ts_core != NULL);

  file = (struct file_s *)sq_peek(&ts_core->files);
  while (file)
    {
      /* Check if file has been marked for garbage collection */

      if (!file->active)
        {
          struct file_s * deleted = file;

          /* Move to next file in queue */

          file = (struct file_s *)sq_next(&file->entry);

          /* Remove deleted file from queue */

          sq_rem(&deleted->entry, &ts_core->files);

          /* Free memory associated with file */

          free(deleted);
        }
      else
        {
          /* Move to next file in queue */

          file = (struct file_s *)sq_next(&file->entry);
        }
    }
}

#ifdef CONFIG_THINGSEE_DEEPSLEEP_WATCHDOG
/****************************************************************************
 * Name: no_deepsleep_cb
 *
 * Description:
 *   Callback function, called when have not been in deepsleep for
 *   CONFIG_THINGSEE_DEEPSLEEP_WATCHDOG_TIMEOUT seconds.
 *
 * Input Parameters:
 *   ts_core     - Pointer to Thingsee core library structure
 *
 ****************************************************************************/
static int no_deepsleep_cb(const int timer_id, void * const priv)
{
  /* Reset device. */

  dbg("!!! Device has not been in deep-sleep for %d seconds !!!\n",
      CONFIG_THINGSEE_DEEPSLEEP_WATCHDOG_TIMEOUT);
  dbg("!!! Reseting device !!!\n");

  sleep(1);

  board_systemreset();
  return OK;
}

/****************************************************************************
 * Name: reset_deepsleep_watchdog
 *
 * Description:
 *   Reset deepsleep watchdog, which monitors that device goes in to STOP mode
 *
 * Input Parameters:
 *   ts_core     - Pointer to Thingsee core library structure
 *
 ****************************************************************************/
static void reset_deepsleep_watchdog(struct ts_core_s * const ts_core)
{
  if (ts_core->deepsleep_watchdog_timer != -1)
    {
      /* Remove previously registered timer. */

      ts_core_timer_stop(ts_core->deepsleep_watchdog_timer);
      ts_core->deepsleep_watchdog_timer = -1;
    }

  /* Setup timer. */
  ts_core->deepsleep_watchdog_timer = ts_core_timer_setup(
      TS_TIMER_TYPE_TIMEOUT, CONFIG_THINGSEE_DEEPSLEEP_WATCHDOG_TIMEOUT * 1000,
      no_deepsleep_cb, ts_core);
}
#endif // CONFIG_THINGSEE_DEEPSLEEP_WATCHDOG

/****************************************************************************
 * Name: ts_core_deepsleep
 *
 * Description:
 *   Perform deep-sleep if allowed.
 *
 * Input Parameters:
 *   ts_core     - Pointer to Thingsee core library structure
 *
 ****************************************************************************/
static void ts_core_deepsleep(struct ts_core_s * const ts_core)
{
  struct deepsleep_hook_s * entry;
  struct file_s * file;
  int timeout_secs;

  sched_lock();

  entry = (struct deepsleep_hook_s *)sq_peek(&ts_core->deepsleep_hooks);
  while (entry)
    {
      if (entry->hook)
        {
          if (!entry->hook(entry->priv))
            {
              /* Hook prevented deep-sleep. */
#ifdef TS_CORE_DEEPSLEEP_DEBUG
              dbg("Hook %p(%p)<%s> prevented deep-sleep.\n", entry->hook,
                  entry->priv, entry->reg_func_name);
#endif
              goto out;
            }
        }

      /* Move to next hook */

      entry = (struct deepsleep_hook_s *)sq_next(&entry->entry);
    }

  /* Check all file-descriptors if there is work left to do. */

  file = (struct file_s *)sq_peek(&ts_core->files);
  while (file)
    {
      if (file->active)
        {
          struct pollfd fds = {};
          int ret;

          /* Check with poll if file has work left. */

          fds.fd = file->fd;
          fds.events = file->events;

          ret = poll(&fds, 1, 0);

          if (ret > 0 && fds.revents > 0)
            {
#ifdef TS_CORE_DEEPSLEEP_DEBUG
              dbg("Poll %p(%p)<%s> prevented deep-sleep.\n", file->callback,
                  file->priv, file->reg_func_name);
#endif
              goto out;
            }
        }

      /* Move to next file in queue */

      file = (struct file_s *)sq_next(&file->entry);
    }

  /* Deep-sleep ok for all hooks. */

  timeout_secs = ts_core_timer_get_date_timeout(ts_core);

  if (timeout_secs == 0)
    {
#ifdef TS_CORE_DEEPSLEEP_DEBUG
      dbg("Deep-sleep: %s.\n", "Timer already expired, skipping deep-sleep");
#endif
      goto out; /* Timer already expired. */
    }

  if (timeout_secs < 0)
    {
      timeout_secs = 0; /* No timers active, sleep to interrupt. */

#ifdef TS_CORE_DEEPSLEEP_DEBUG
      dbg("Deep-sleep: %s.\n", "No timers active, sleep to interrupt");
#endif
    }
  else
    {
      if (timeout_secs > BOARD_DEEPSLEEP_MAX_SECS)
        {
          timeout_secs = BOARD_DEEPSLEEP_MAX_SECS;
        }

#ifdef TS_CORE_DEEPSLEEP_DEBUG
      dbg("Deep-sleep: Sleeping %d seconds.\n", timeout_secs);
#endif
    }

#ifndef CONFIG_ARCH_SIM
  /* Go to deep-sleep mode. */

  board_deepsleep_with_stopmode(timeout_secs);
#endif

#ifdef CONFIG_THINGSEE_DEEPSLEEP_WATCHDOG
  /* Reset deep-sleep timer. */

  reset_deepsleep_watchdog(ts_core);
#endif // CONFIG_THINGSEE_DEEPSLEEP_WATCHDOG

out:
  sched_unlock();
  return;
}

/****************************************************************************
 * Name: ts_core_process
 *
 * Description:
 *   Process one Thingsee core mainloop iteration. Poll registered file descriptors and
 *   handle timers
 *
 ****************************************************************************/
static int ts_core_process(struct ts_core_s *const ts_core)
{
  static struct pollfd fds[CONFIG_NFILE_DESCRIPTORS];
  static struct file_s * files[CONFIG_NFILE_DESCRIPTORS];
  struct file_s * file;
  int timeout_ms = -1;
  int nfds = 0;
  int ret;
  int i;

  file = (struct file_s *)sq_peek(&ts_core->files);

  while (file)
    {
      if (file->active)
        {
          /* Check that we do not exceed system defined maximum limit for monitored files */

          DEBUGASSERT(nfds < CONFIG_NFILE_DESCRIPTORS);

          files[nfds] = file;
          fds[nfds].fd = file->fd;
          fds[nfds++].events = file->events;
        }

      /* Move to next file in queue */

      file = (struct file_s *)sq_next(&file->entry);
    }

  /* Deep-sleep handling */

  ts_core_deepsleep(ts_core);

  /* Get timeout to next timer */

  timeout_ms = ts_core_timer_get_timeout(ts_core);

  /* Limit poll/normal timer length so that we re-evaluate deep-sleepiness
   * periodically. */

  if (timeout_ms > TS_CORE_MAX_POLL_TIMEOUT_MSEC)
    {
      timeout_ms = TS_CORE_MAX_POLL_TIMEOUT_MSEC;
    }

  if (nfds)
    {
#ifdef TS_CORE_PERF_DEBUG
      ts_core->elapsed.prev_ticks = clock_systimer();
#endif

      /* Poll file descriptors with timeout */

      ret = poll(fds, nfds, timeout_ms);

#ifdef TS_CORE_PERF_DEBUG
      ts_core->elapsed.poll = TICK2MSEC(clock_systimer() -
                                        ts_core->elapsed.prev_ticks);
#endif

      if (ret < 0)
        {
          dbg("poll returned errno %d\n", get_errno());

          return ret;
        }

      if (ret > 0)
        {
          for (i = 0; i < nfds; i++)
            {
              if (fds[i].revents != 0 && files[i]->callback != NULL && files[i]->active)
                {
#ifdef TS_CORE_PERF_DEBUG
                  ts_core->elapsed.prev_ticks = clock_systimer();
#endif
                  /* Execute file descriptor callback */

                  ret = files[i]->callback(&fds[i], files[i]->priv);

#ifdef TS_CORE_PERF_DEBUG
                  ts_core->elapsed.files += TICK2MSEC(clock_systimer() -
                                                      ts_core->elapsed.prev_ticks);
                  ts_core->elapsed.nfiles++;
#endif

                  if (ret < 0)
                    dbg("Callback for file %d returned %d.\n", fds[i].fd, ret);
                }
            }
        }

      /* Garbage collect files queue */

      ts_core_gc_files(ts_core);
    }
  else
    {
      /* Setup default sleep time if there was no timers or file descriptors to poll */

      if (timeout_ms < 0)
        timeout_ms = CORE_DEFAULT_SLEEP_MS;

      /* In Simulator system time won't progress unless we sleep */

#ifdef CONFIG_ARCH_SIM
      if (!timeout_ms)
        {
          timeout_ms = 1;
        }
#endif

      /* Sleep given time */

      usleep(timeout_ms * USEC_PER_MSEC);
    }

  /* Process timers */

  ts_core_process_timers(ts_core);

  /* Garbage collect timer queue */

  ts_core_gc_timers(ts_core);

  return OK;
}

/****************************************************************************
 * Name: __ts_core_timer_setup
 *
 * Description:
 *   Setup timer interval / one shot timeout timer
 *
 * Input Parameters:
 *   type        - Timer type (TS_TIMER_TYPE_INTERVAL, TS_TIMER_TYPE_TIMEOUT, TS_TIMER_TYPE_DATE)
 *   timeval     - Timer timeout in milliseconds or as absolute time (TS_TIMER_TYPE_DATE)
 *   callback    - Pointer to timer callback function
 *   priv        - Private data for timer callback
 *
 * Returned Value:
 *   Timer ID (>= 0) when timer was created successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
static int __ts_core_timer_setup(const ts_timer_type_t type,
                                 const void * timeval,
                                 const void * callback,
                                 void * const priv)
{
  struct ts_core_s * ts_core = &g_ts_core;
  struct timer_s * timer;

  /* Check input parameters */

  if (type >= TS_TIMER_TYPE_MAX || !callback)
    {
      set_errno(EINVAL);

      return ERROR;
    }

  /* Allocate memory for new timer */

  timer = malloc(sizeof(struct timer_s));
  if (!timer)
    return ERROR;

  /* Setup interval timer */

  timer->id = ts_core_get_timer_id(ts_core);

  /* Initialize timer flags */

  memset(&timer->flags, 0, sizeof(timer->flags));
  timer->flags.active = true;
  timer->flags.is_new = true;
  timer->flags.type = type;

  if (type == TS_TIMER_TYPE_DATE)
    {
      timer->date_expires = *(const struct timespec *)timeval;
      timer->date_callback = callback;
    }
  else
    {
      uint32_t timeout_ms = *(const uint32_t *)timeval;
      timer->ticks = MSEC2TICK(timeout_ms);
      timer->expires = clock_systimer() + timer->ticks;
      timer->callback = callback;
    }
  timer->priv = priv;

  /* Add timer to queue */

  sq_addlast(&timer->entry, &ts_core->timers);

  return timer->id;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ts_core_initialize
 *
 * Description:
 *   Initialize Thingsee core
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ts_core_initialize(void)
{
  struct ts_core_s * const ts_core = &g_ts_core;

  ts_core->deepsleep_watchdog_timer = -1;

  /* Initialize file descriptor queue */

  sq_init(&ts_core->files);

  /* Initialize timer queue */

  sq_init(&ts_core->timers);

  return OK;
}

/****************************************************************************
 * Name: ts_core_deinitialize
 *
 * Description:
 *   De-initialize Thingsee core
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ts_core_deinitialize(void)
{
  struct ts_core_s * ts_core = &g_ts_core;
  struct file_s * file;
  struct timer_s * timer;

  /* Unregister file descriptors */

  file = (struct file_s *)sq_peek(&ts_core->files);
  while (file)
    {
      struct file_s * deleted = file;

      /* Move to next file in queue */
      file = (struct file_s *)sq_next(&file->entry);

      /* Remove file from queue */

      sq_rem(&deleted->entry, &ts_core->files);

      /* Free memory associated with file */

      free(deleted);
    }

  /* Free timers */

  timer = (struct timer_s *)sq_peek(&ts_core->timers);
  while (timer)
    {
      struct timer_s * deleted = timer;

      /* Move to next timer in queue */
      timer = (struct timer_s *)sq_next(&timer->entry);

      /* Remove timer from queue */

      sq_rem(&deleted->entry, &ts_core->timers);

      /* Free memory associated with timer */

      free(deleted);
    }

  ts_core->deepsleep_watchdog_timer = -1;

  return OK;
}

/****************************************************************************
 * Name: ts_core_fd_register
 *
 * Description:
 *   Register file descriptor for monitoring
 *
 * Input Parameters:
 *   fd          - File descriptor to monitor
 *   events      - File descriptor poll event type (POLLIN, POLLOUT ...)
 *   callback    - Pointer to file descriptor callback function
 *   priv        - Private data for file descriptor callback
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int __ts_core_fd_register(const int fd, const pollevent_t events,
                          const ts_fd_callback_t callback,
                          void * const priv, const char *reg_func_name)
{
  struct ts_core_s * ts_core = &g_ts_core;
  struct file_s * file;

  /* Check input parameters */

  if (fd < 0 || !events || !callback)
    {
      set_errno(EINVAL);

      return ERROR;
    }

  file = (struct file_s *)sq_peek(&ts_core->files);
  while (file)
    {
      /* Check if file descriptor is already registered */

      if (file->fd == fd)
        {
          /* File descriptor is already registered */

          if (file->active)
            {
              set_errno(EEXIST);

              return ERROR;
            }

          /* We used to 'reuse' entry here. We must not do this, as
           * new 'fd' might not be for same device instance as the original
           * 'file->fd'. Setting 'active' could activate callback too early
           * if poll result handling loop is being currently processed. */
        }

      /* Move to next file */

      file = (struct file_s *)sq_next(&file->entry);
    }

  /* Allocate memory for file */

  file = calloc(1, sizeof(struct file_s));
  if (!file)
    {
      set_errno(ENOMEM);

      return ERROR;
    }

  /* Setup file descriptor for monitoring */

  file->fd = fd;
  file->active = true;
  file->events = events;
  file->callback = callback;
  file->priv = priv;
  file->reg_func_name = reg_func_name;

  /* Add file to file descriptor queue */

  sq_addlast(&file->entry, &ts_core->files);

  return OK;
}

/****************************************************************************
 * Name: ts_core_fd_unregister
 *
 * Description:
 *   Unregister file descriptor from monitoring
 *
 * Input Parameters:
 *   fd          - File descriptor that is currently monitored
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ts_core_fd_unregister(const int fd)
{
  struct ts_core_s * ts_core = &g_ts_core;
  struct file_s * file;

  /* Check input parameters */

  if (fd < 0)
    {
      set_errno(EINVAL);

      return ERROR;
    }

  file = (struct file_s *)sq_peek(&ts_core->files);
  while (file)
    {
      /* Check if file is active and file descriptor matches */

      if (file->active && file->fd == fd)
        {
          /* Mark file for garbage collection */

          file->active = false;

          return OK;
        }

      /* Move to next file */

      file = (struct file_s *)sq_next(&file->entry);
    }

  set_errno(EBADF);

  return ERROR;
}

/****************************************************************************
 * Name: ts_core_timer_setup
 *
 * Description:
 *   Setup timer interval / one shot timeout timer
 *
 * Input Parameters:
 *   type        - Timer type (TS_TIMER_TYPE_INTERVAL, TS_TIMER_TYPE_TIMEOUT)
 *   timeout_ms  - Timer timeout in milliseconds
 *   callback    - Pointer to timer callback function
 *   priv        - Private data for timer callback
 *
 * Returned Value:
 *   Timer ID (>= 0) when timer was created successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ts_core_timer_setup(const ts_timer_type_t type, const uint32_t timeout_ms, const ts_timer_callback_t callback, void * const priv)
{
  /* Check input parameters */

  if (type == TS_TIMER_TYPE_DATE)
    {
      set_errno(EINVAL);

      return ERROR;
    }

  return __ts_core_timer_setup(type, &timeout_ms, callback, priv);
}

/****************************************************************************
 * Name: ts_core_timer_setup_date
 *
 * Description:
 *   Setup RTC based date/time timer
 *
 * Input Parameters:
 *   date        - Timer timeout as absolute time
 *   callback    - Pointer to timer callback function
 *   priv        - Private data for timer callback
 *
 * Returned Value:
 *   Timer ID (>= 0) when timer was created successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ts_core_timer_setup_date(const struct timespec *date, const ts_timer_date_callback_t callback, void * const priv)
{
  return __ts_core_timer_setup(TS_TIMER_TYPE_DATE, date, callback, priv);
}

/****************************************************************************
 * Name: ts_core_timer_stop
 *
 * Description:
 *   Stop interval timer
 *
 * Input Parameters:
 *   timer_id    - Timer ID
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int __ts_core_timer_stop(const int timer_id)
{
  struct ts_core_s * ts_core = &g_ts_core;
  struct timer_s * timer;

  /* Check input parameters */

  if (timer_id < 0)
    {
      set_errno(EINVAL);

      return ERROR;
    }

  timer = (struct timer_s *)sq_peek(&ts_core->timers);
  while (timer)
    {
      if (timer->flags.active && timer->id == timer_id)
        {
          /* Deactivate timer */

          timer->flags.active = false;

          return OK;
        }

      /* Move to next timer */

      timer = (struct timer_s *)sq_next(&timer->entry);
    }

  set_errno(EINVAL);

  return ERROR;
}

/****************************************************************************
 * Name: __ts_core_deepsleep_hook_add
 *
 * Description:
 *   Add deep-sleep hook. Deep-sleep hook functions are called before entering
 *   deep-sleep. If hook returns 'false', deep-sleep is prevented. If all hooks
 *   return 'true', deep-sleep is entered.
 *
 * Input Parameters:
 *   hookfn     - Pointer to deep-sleep hook function
 *   priv       - Private data for timer callback
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int __ts_core_deepsleep_hook_add(const ts_deepsleep_hook_t hookfn,
                                 void * const priv, const char *reg_func_name)
{
  struct ts_core_s * ts_core = &g_ts_core;
  struct deepsleep_hook_s * entry;

  /* Check input parameters */

  if (hookfn == NULL)
    {
      set_errno(EINVAL);

      return ERROR;
    }

  /* Allocate memory for new timer */

  entry = zalloc(sizeof(*entry));
  if (!entry)
    return ERROR;

  /* Setup list entry */

  entry->hook = hookfn;
  entry->priv = priv;
  entry->reg_func_name = reg_func_name;

  /* Add timer to queue */

  sq_addlast(&entry->entry, &ts_core->deepsleep_hooks);

  return OK;
}

/****************************************************************************
 * Name: ts_core_deepsleep_hook_remove
 *
 * Description:
 *   Remove deep-sleep hook
 *
 * Input Parameters:
 *   hookfn - Hook function
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ts_core_deepsleep_hook_remove(ts_deepsleep_hook_t hookfn)
{
  struct ts_core_s * ts_core = &g_ts_core;
  struct deepsleep_hook_s * entry;

  /* Check input parameters */

  if (hookfn == NULL)
    {
      set_errno(EINVAL);

      return ERROR;
    }

  entry = (struct deepsleep_hook_s *)sq_peek(&ts_core->deepsleep_hooks);
  while (entry)
    {
      if (entry->hook == hookfn)
        {
          /* Remove from list and free entry */

          sq_rem(&entry->entry, &ts_core->deepsleep_hooks);
          free(entry);

          return OK;
        }

      /* Move to next hook */

      entry = (struct deepsleep_hook_s *)sq_next(&entry->entry);
    }

  set_errno(ENOENT);

  return ERROR;
}

/****************************************************************************
 * Name: ts_core_is_date_before_compile_date
 *
 * Description:
 *   Check if date for given timestamp is older than the compile date of binary.
 *
 * Input Parameters:
 *   ctime  - Timestamp
 *
 * Returned Value:
 *   true - Timestamp date is older than build date
 *   false - Timestamp date is same or newer than build date
 *
 ****************************************************************************/
bool ts_core_is_date_before_compile_date(time_t ctime)
{
  struct tm tm;
  const char *pmonth;
  const char *pyear;
  const char *pday;
  int year, day, month;

  /*
   * Compile date. Format: "MMM DD YYYY", where MMM is month in three letter
   * format, DD is day of month (left padded with space if less than ten) and
   * YYYY is year.
   */

  pmonth = __DATE__;
  pday = pmonth + 4;
  pyear = pmonth + 7;

  year = atoi(pyear);
  if (pday[0] == ' ')
    day = atoi(pday + 1);
  else
    day = atoi(pday);

  switch (pmonth[0])
    {
    default:
      DEBUGASSERT(false);
      break;
    case 'J':
      if (pmonth[1] == 'a') /* Jan */
        month = 1;
      else if (pmonth[2] == 'n') /* Jun */
        month = 6;
      else /* Jul */
        month = 7;
      break;
    case 'F': /* Feb */
      month = 2;
      break;
    case 'M':
      if (pmonth[2] == 'r') /* Mar */
        month = 3;
      else /* May */
        month = 5;
      break;
    case 'A':
      if (pmonth[1] == 'p') /* Apr */
        month = 4;
      else /* Aug */
        month = 8;
      break;
    case 'S': /* Sep */
      month = 9;
      break;
    case 'O': /* Oct */
      month = 10;
      break;
    case 'N': /* Nov */
      month = 11;
      break;
    case 'D': /* Dec */
      month = 12;
      break;
    }

  /* Convert timestamp to date. */

  gmtime_r(&ctime, &tm);

  if ((tm.tm_year + 1900) < year)
    return true; /* Current year older than compile year. */
  if ((tm.tm_year + 1900) > year)
    return false;  /* Current year newer than compile year. */

  if ((tm.tm_mon + 1) < month)
    return true; /* Current month older than compile month. */
  if ((tm.tm_mon + 1) > month)
    return false;  /* Current month newer than compile month. */

  if ((tm.tm_mday) < day)
    return true; /* Current day older than compile day. */
  if ((tm.tm_mday) > day)
    return false;  /* Current day newer than compile day. */

  return false; /* Current day same as compile day. */
}

/****************************************************************************
 * Name: ts_core_mainloop
 *
 * Description:
 *   Run Thingsee core main event loop
 *
 * Input parameters:
 *   goon - set to false to escape the otherwise endless loop
 *
 ****************************************************************************/
void ts_core_mainloop(volatile bool *goon)
{
  struct ts_core_s * ts_core = &g_ts_core;

  /* Clear 'is_new' mark from timers registered before starting mainloop. */

  ts_core_gc_timers(ts_core);

  while (*goon)
    {
#ifdef TS_CORE_PERF_DEBUG
      memset(&ts_core->elapsed, 0, sizeof(ts_core->elapsed));
      ts_core->elapsed.start_ticks = clock_systimer();
#endif

#ifndef CONFIG_ARCH_SIM
      /* First check that we still have enough battery left. */

      board_pwr_checkvbat(true);
#endif

      /* Process one iteration of Thingsee core main event loop */

      ts_core_process(ts_core);

#ifdef TS_CORE_PERF_DEBUG
      ts_core->elapsed.total = TICK2MSEC(clock_systimer() -
                                         ts_core->elapsed.start_ticks);

      if (ts_core->elapsed.files >= TS_CORE_PERF_FILES_LIMIT ||
          ts_core->elapsed.timers >= TS_CORE_PERF_TIMERS_LIMIT)
        {
          printf("poll: %d, files: %d,%d, timers: %d,%d, sleep %d, total: %d ms\n",
              ts_core->elapsed.poll,
              ts_core->elapsed.files, ts_core->elapsed.nfiles,
              ts_core->elapsed.timers, ts_core->elapsed.ntimers,
              ts_core->elapsed.sleep,
              ts_core->elapsed.total);
        }
#endif
    }
}

/****************************************************************************
 * Name: ts_core_timer_selftest_cb
 *
 * Description:
 *   Thingsee core timer callback during selftests
 *
 ****************************************************************************/
static int ts_core_timer_selftest_cb(const int timer_id, void * const priv)
{
  bool * cb_ok = (bool *)priv;

  DEBUGASSERT(cb_ok);

  *cb_ok = true;

  return OK;
}

/****************************************************************************
 * Name: ts_core_timer_selftest
 *
 * Description:
 *   Run Thingsee core timer selftests
 *
 ****************************************************************************/
static void ts_core_timer_selftest(void)
{
  struct ts_core_s * ts_core = &g_ts_core;
  struct timer_s * timer;
  int timeout_id;
  bool timeout_cb_ok = false;
  int interval_id;
  bool interval_cb_ok = false;

  /* Check that there are no timers registered */

  DEBUGASSERT(sq_peek(&ts_core->timers) == NULL);

  /* Setup timeout timer */

  timeout_id = ts_core_timer_setup(TS_TIMER_TYPE_TIMEOUT,
                                   1 * MSEC_PER_TICK,
                                   ts_core_timer_selftest_cb,
                                   &timeout_cb_ok);
  DEBUGASSERT(timeout_id == TS_CORE_FIRST_TIMER_ID + 0);

  /* Setup interval timer */

  interval_id = ts_core_timer_setup(TS_TIMER_TYPE_INTERVAL,
                                    1 * MSEC_PER_TICK,
                                    ts_core_timer_selftest_cb,
                                    &interval_cb_ok);
  DEBUGASSERT(interval_id == TS_CORE_FIRST_TIMER_ID + 1);

  /* Check timeout timer */

  timer = (struct timer_s *)sq_peek(&ts_core->timers);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.active == true);
  DEBUGASSERT(timer->flags.is_new == true);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_TIMEOUT);
  /* Not possible to check expires field exact value since system tick counter
   * is running */

  /* Check interval timer */

  timer = (struct timer_s *)sq_next(&timer->entry);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.active == true);
  DEBUGASSERT(timer->flags.is_new == true);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_INTERVAL);
  /* Not possible to check expires field exact value since system tick counter
   * is running */

  /* Clear 'is_new' mark. */

  ts_core_gc_timers(ts_core);

  /* Check timeout timer */

  timer = (struct timer_s *)sq_peek(&ts_core->timers);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.active == true);
  DEBUGASSERT(timer->flags.is_new == false);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_TIMEOUT);
  /* Not possible to check expires field exact value since system tick counter
   * is running */

  /* Check interval timer */

  timer = (struct timer_s *)sq_next(&timer->entry);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.active == true);
  DEBUGASSERT(timer->flags.is_new == false);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_INTERVAL);
  /* Not possible to check expires field exact value since system tick counter
   * is running */

  /* Wait for timers to expire */

  usleep(3 * USEC_PER_MSEC);

  /* Execute callbacks */

  ts_core_process_timers(ts_core);

  /* Check that callbacks where called */

  DEBUGASSERT(timeout_cb_ok == true);
  DEBUGASSERT(interval_cb_ok == true);

  /* Check timeout timer */

  timer = (struct timer_s *)sq_peek(&ts_core->timers);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.active == false);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_TIMEOUT);

  /* Check interval timer */

  timer = (struct timer_s *)sq_next(&timer->entry);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.active == true);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_INTERVAL);

  /* Garbage collect non-active timers */

  ts_core_gc_timers(ts_core);

  /* Try to stop timeout timer, which has already been removed */

  DEBUGASSERT(ts_core_timer_stop(timeout_id) == ERROR);

  /* Stop interval timer */

  DEBUGASSERT(ts_core_timer_stop(interval_id) == OK);

  /* Garbage collect non-active timers */

  ts_core_gc_timers(ts_core);

  /* Check that there are no timers registered */

  DEBUGASSERT(sq_peek(&ts_core->timers) == NULL);
}

/****************************************************************************
 * Name: ts_core_fd_selftest_cb
 *
 * Description:
 *   Thingsee core file callback during selftests
 *
 ****************************************************************************/
static int ts_core_fd_selftest_cb(const struct pollfd * const pfd, void * const priv)
{
  return OK;
}

/****************************************************************************
 * Name: ts_core_fd_selftest
 *
 * Description:
 *   Run Thingsee core file descriptor selftests
 *
 ****************************************************************************/
static void ts_core_fd_selftest(void)
{
  struct ts_core_s * ts_core = &g_ts_core;
  struct file_s * file;

  /* Check that there are no files registered */

  DEBUGASSERT(sq_peek(&ts_core->files) == NULL);

  /* Register fd 1 poll in callback */

  DEBUGASSERT(ts_core_fd_register(1, POLLIN, ts_core_fd_selftest_cb, ts_core) == OK);

  /* Register fd 2 poll out callback */

  DEBUGASSERT(ts_core_fd_register(2, POLLOUT, ts_core_fd_selftest_cb, ts_core) == OK);

  /* Check that files where registered correctly */

  file = (struct file_s *)sq_peek(&ts_core->files);
  DEBUGASSERT(file);
  DEBUGASSERT(file->active == true);
  DEBUGASSERT(file->events == POLLIN);
  DEBUGASSERT(file->fd == 1);

  file = (struct file_s *)sq_next(&file->entry);
  DEBUGASSERT(file);
  DEBUGASSERT(file->active == true);
  DEBUGASSERT(file->events == POLLOUT);
  DEBUGASSERT(file->fd == 2);

  /* Unregister fd 2 */

  DEBUGASSERT(ts_core_fd_unregister(2) == OK);

  /* Unregister fd 1 */

  DEBUGASSERT(ts_core_fd_unregister(1) == OK);

  /* Check that files where unregistered correctly */

  file = (struct file_s *)sq_peek(&ts_core->files);
  DEBUGASSERT(file);
  DEBUGASSERT(file->active == false);
  DEBUGASSERT(file->events == POLLIN);
  DEBUGASSERT(file->fd == 1);

  file = (struct file_s *)sq_next(&file->entry);
  DEBUGASSERT(file);
  DEBUGASSERT(file->active == false);
  DEBUGASSERT(file->events == POLLOUT);
  DEBUGASSERT(file->fd == 2);

  /* Garbage collect files */

  ts_core_gc_files(ts_core);

  /* Check that there are no files registered */

  DEBUGASSERT(sq_peek(&ts_core->files) == NULL);
}

/****************************************************************************
 * Name: ts_core_selftest
 *
 * Description:
 *   Run Thingsee core selftests. Thingsee core must be initialized with
 *   ts_core_initialize before calling this function. Also this function
 *   assumes that there are no registered files or timers setup.
 *
 ****************************************************************************/
void ts_core_selftest(void)
{
  ts_core_timer_selftest();
  ts_core_fd_selftest();
}
