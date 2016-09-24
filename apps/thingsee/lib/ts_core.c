/****************************************************************************
 * apps/thingsee/lib/ts_core.c
 *
 *   Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
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

//#define TS_CORE_TIMERS_DEBUG

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

/* Number of file descriptors handled by ts_core (default: maximum fds
 * configured for NuttX). */

#define TS_CORE_NFILES (CONFIG_NFILE_DESCRIPTORS + CONFIG_NSOCKET_DESCRIPTORS)

/* Default sleep time when there are no file descriptors / timers active */

#define CORE_DEFAULT_SLEEP_MS           1000

/* Maximum non-deep-sleep sleep time */

#if defined(CONFIG_THINGSEE_CORE_MAX_POLL_TIMEOUT_MSEC) && \
    CONFIG_THINGSEE_CORE_MAX_POLL_TIMEOUT_MSEC > 0
#  define TS_CORE_MAX_POLL_TIMEOUT_MSEC \
            CONFIG_THINGSEE_CORE_MAX_POLL_TIMEOUT_MSEC
#else
#  define TS_CORE_MAX_POLL_TIMEOUT_MSEC 2500
#endif

/* Minimum non-deep-sleep sleep time */

#define TS_CORE_MIN_POLL_TIMEOUT_MSEC   100

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#ifdef TS_CORE_PERF_DEBUG
#  define perf_dbg_start_ticks(ts_core) \
          ((void)(ts_core->elapsed.prev_ticks = clock_systimer()))

#  define perf_dbg_add_elapsed_ticks(ts_core, type) \
          ((void)({ \
            ts_core->elapsed.type += \
              TICK2MSEC(clock_systimer() - ts_core->elapsed.prev_ticks); \
            ts_core->elapsed.n##type++; \
          }))
#else
#  define perf_dbg_start_ticks(ts_core) ((void)0)
#  define perf_dbg_add_elapsed_ticks(ts_core, type) ((void)0)
#endif

#ifdef TS_CORE_DEEPSLEEP_DEBUG
#  define deepsleep_dbg(...) dbg(__VA_ARGS__)
#else
#  define deepsleep_dbg(...) ((void)0)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* File descriptor entry */

struct file_s
{
  /* File descriptor callback function */

  ts_fd_callback_t callback;

  /* Private data for callback function */

  void *priv;

  /* Name of function that registered this entry. */

  const char *reg_func_name;
};

/* Timer flags */

struct timer_flags_s
{
  enum ts_timer_type_e type:4;    /* Timer type */
  bool done:1;                    /* Timer done, and in done_timers sq */
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

  /* Members for date and RTC based interval/timeout timers. */

  struct
  {
    /* Absolute time when date timer expires */

    struct timespec date_expires;

    /* Interval length */

    uint32_t interval_ms;

    union
    {
      /* Interval/timeout timer callback function */

      ts_timer_callback_t callback;

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
  /* File descriptor lists */

  unsigned int nfiles;
  unsigned int nfiles_deleted;
  struct file_s files[TS_CORE_NFILES];
  struct pollfd pollfds[TS_CORE_NFILES];

  /* Timer queues */

  sq_queue_t timers;
  sq_queue_t new_timers;
  sq_queue_t done_timers;
  size_t heap_size;

  /* Deep-sleep hook queue */

  sq_queue_t deepsleep_hooks;

  /* Deep-sleep watchdog timer id */

  int deepsleep_watchdog_timer;

#ifdef TS_CORE_PERF_DEBUG
  struct {
    systime_t start_ticks;
    systime_t prev_ticks;

    uint32_t poll;
    uint32_t npoll;
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
 * Name: timespec_cmp
 ****************************************************************************/
static int timespec_cmp(const struct timespec *a, const struct timespec *b)
{
  int64_t a_nsec = (int64_t)a->tv_sec * (1000 * 1000 * 1000) + a->tv_nsec;
  int64_t b_nsec = (int64_t)b->tv_sec * (1000 * 1000 * 1000) + b->tv_nsec;

  if (a_nsec > b_nsec)
    return 1;
  if (a_nsec < b_nsec)
    return -1;
  return 0;
}

/****************************************************************************
 * Name: timespec_add
 ****************************************************************************/
static struct timespec *timespec_add(struct timespec *ts,
                                     const struct timespec *add)
{
  ts->tv_sec += add->tv_sec;
  ts->tv_nsec += add->tv_nsec;
  while (ts->tv_nsec >= 1000 * 1000 * 1000)
    {
      ts->tv_nsec -= 1000 * 1000 * 1000;
      ts->tv_sec += 1;
    }
  while (ts->tv_nsec < 0)
    {
      ts->tv_nsec += 1000 * 1000 * 1000;
      ts->tv_sec -= 1;
    }

  return ts;
}

/****************************************************************************
 * Name: timespec_add_msec
 ****************************************************************************/
static struct timespec *timespec_add_msec(struct timespec *ts,
                                          long msec)
{
  struct timespec add;

  add.tv_sec = msec / 1000;
  add.tv_nsec = (msec - add.tv_sec * 1000) * (1000 * 1000);

  return timespec_add(ts, &add);
}

/****************************************************************************
 * Name: timespec_to_msec
 ****************************************************************************/
static int64_t timespec_to_msec(const struct timespec *ts)
{
  return (int64_t)ts->tv_sec * 1000 + ts->tv_nsec / (1000 * 1000);
}

/****************************************************************************
 * Name: purge_timers
 ****************************************************************************/
static void purge_timers(sq_queue_t *queue)
{
  struct timer_s * timer;

  while ((timer = (struct timer_s *)sq_remfirst(queue)) != NULL)
    {
      /* Free memory associated with timer */

      free(timer);
    }
}

/****************************************************************************
 * Name: execute_timer_callback
 ****************************************************************************/
static int execute_timer_callback(struct ts_core_s * const ts_core,
                                  struct timer_s *timer)
{
  enum ts_timer_type_e type = timer->flags.type;
  int ret;

  ret = (type == TS_TIMER_TYPE_DATE) ?
          timer->date_callback(timer->id, &timer->date_expires, timer->priv) :
          timer->callback(timer->id, timer->priv);

  if (timer->flags.done)
    {
      return ret; /* Timer deleted itself in callback. */
    }

  /* Executed timer is always first of active queue (unless deleted itself). */

  DEBUGASSERT(timer == (struct timer_s *)sq_peek(&ts_core->timers));

  if (type == TS_TIMER_TYPE_INTERVAL)
    {
      /* Set next timeout event */

      timespec_add_msec(&timer->date_expires, timer->interval_ms);

      /* Move to new queue for reinsertion to active queue. */

      sq_remfirst(&ts_core->timers);
      sq_addlast(&timer->entry, &ts_core->new_timers);
    }
  else
    {
      /* Deactivate timer, move to done queue. */

      sq_remfirst(&ts_core->timers);
      sq_addlast(&timer->entry, &ts_core->done_timers);

      timer->flags.done = true;
    }

  return ret;
}

/****************************************************************************
 * Name: ts_core_get_timer_id
 *
 * Description:
 *   Get unused timer ID
 *
 * Input Parameters:
 *   ts_core     - Pointer to Thingsee core library structure
 *   new_timer   - New timer object for which to create ID
 *
 * Returned Value:
 *   Timer ID (>= 0) when timer was created successfully
 *  -1 (ERROR) means the function was executed unsuccessfully
 *
 ****************************************************************************/
static int ts_core_get_timer_id(struct ts_core_s * const ts_core,
                                struct timer_s * new_timer)
{
  /* Convert timer object position in heap to timer_id. */

  if (ts_core->heap_size <= 0)
    {
      struct mallinfo ml = mallinfo();

      ts_core->heap_size = ml.arena;
      DEBUGASSERT(ts_core->heap_size > 0);
      DEBUGASSERT(ts_core->heap_size < INT_MAX);
    }

  return ((uintptr_t)new_timer % (ts_core->heap_size)) + TS_CORE_FIRST_TIMER_ID;
}

/****************************************************************************
 * Name: dbg_timers
 ****************************************************************************/
static void dbg_timers(const char *qname, sq_queue_t *timer_queue)
{
#ifdef TS_CORE_TIMERS_DEBUG
  struct timer_s *timer = (struct timer_s *)sq_peek(timer_queue);
  struct timespec curr_ts;
  int64_t curr_msec;
  int i = 0;

  clock_gettime(CLOCK_MONOTONIC, &curr_ts);
  curr_msec = timespec_to_msec(&curr_ts);

  while (timer)
    {
      int64_t timer_msec = timespec_to_msec(&timer->date_expires);

      dbg("[%s] %d: id=%d: expiry in %lld msec, type %d\n",
          qname, i, timer->id, (timer_msec - curr_msec), timer->flags.type);

      i++;

      timer = (struct timer_s *)sq_next(&timer->entry);
    }
#endif
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
  struct timer_s *new_timer, *next_new_timer;

  DEBUGASSERT(ts_core != NULL);

  /* Insert new timers to active queue. */

  new_timer = (struct timer_s *)sq_peek(&ts_core->new_timers);
  while (new_timer)
    {
      struct timer_s *prev, *active;

      next_new_timer = (struct timer_s *)sq_next(&new_timer->entry);

      DEBUGASSERT(new_timer == (struct timer_s *)sq_peek(&ts_core->new_timers));
      sq_remfirst(&ts_core->new_timers);

      /* Ordered insert to active timer queue. */

      active = (struct timer_s *)sq_peek(&ts_core->timers);
      if (!active)
        {
          sq_addfirst(&new_timer->entry, &ts_core->timers);
        }
      else
        {
          if (timespec_cmp(&new_timer->date_expires,
                           &active->date_expires) < 0)
            {
              sq_addfirst(&new_timer->entry, &ts_core->timers);
            }
          else
            {
              while (true)
                {
                  prev = active;
                  active = (struct timer_s *)sq_next(&active->entry);

                  if (!active || timespec_cmp(&new_timer->date_expires,
                                              &active->date_expires) < 0)
                    {
                      break;
                    }
                }

              sq_addafter(&prev->entry, &new_timer->entry,
                          &ts_core->timers);
            }
        }

      new_timer = next_new_timer;
    }

  DEBUGASSERT(sq_peek(&ts_core->new_timers) == NULL);

  /* Free completed timers. */

  dbg_timers("gc done", &ts_core->done_timers);
  purge_timers(&ts_core->done_timers);
  DEBUGASSERT(sq_peek(&ts_core->done_timers) == NULL);

  /* Debug print. */

  dbg_timers("gc active", &ts_core->timers);
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
  struct timespec curr_ts;
  int timeout_ms = -1;
  int64_t curr_msec;
  int64_t timer_msec;
  int ret;

  if (!timer)
    return timeout_ms;

  /* Get current time. */

  ret = clock_gettime(CLOCK_MONOTONIC, &curr_ts);
  DEBUGASSERT(ret != ERROR);

  curr_msec = timespec_to_msec(&curr_ts);

  DEBUGASSERT(timer->flags.type >= 0);
  DEBUGASSERT(timer->flags.type < TS_TIMER_TYPE_MAX);

  /* Get difference between current time and expire time. */

  timer_msec = timespec_to_msec(&timer->date_expires);

  if (timer_msec <= curr_msec)
    {
      /* Already expired. */

      timeout_ms = 0;
    }
  else
    {
      /* First timer in active timer queue expires first. */

      if (timer_msec - curr_msec <= INT_MAX)
        timeout_ms = timer_msec - curr_msec;
      else
        timeout_ms = INT_MAX;
    }

  /* Debug print. */

  dbg_timers("timeout active", &ts_core->timers);

  return timeout_ms;
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
  struct timespec curr_ts;
  struct timer_s * timer;
  int64_t curr_msec;
  int64_t timer_msec;

  DEBUGASSERT(ts_core != NULL);

  /* Debug print. */

  dbg_timers("pre active ", &ts_core->timers);

  /* Get next timer from queue, use peek since timers queue might have
   * been modified (entries removed) in timer callback. */

  while ((timer = (struct timer_s *)sq_peek(&ts_core->timers)) != NULL)
    {
      int ret;

      DEBUGASSERT(timer->flags.type >= 0);
      DEBUGASSERT(timer->flags.type < TS_TIMER_TYPE_MAX);

      /* Get current time. */

      ret = clock_gettime(CLOCK_MONOTONIC, &curr_ts);
      DEBUGASSERT(ret != ERROR);

      curr_msec = timespec_to_msec(&curr_ts);

      /* Check if timer has expired. */

      timer_msec = timespec_to_msec(&timer->date_expires);

      if (timer_msec <= curr_msec)
        {
          perf_dbg_start_ticks(ts_core);

          /* Execute timer callback */

          ret = execute_timer_callback(ts_core, timer);

          perf_dbg_add_elapsed_ticks(ts_core, timers);

          if (ret < 0)
            dbg("RTC timer %d callback returned %d\n", timer->id, ret);
        }
      else
        {
          /* Not expired timer, so following timers have not expired either. */

          break;
        }
    }

  /* Debug print. */

  dbg_timers("post active", &ts_core->timers);
  dbg_timers("post new   ", &ts_core->new_timers);
  dbg_timers("post done  ", &ts_core->done_timers);

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
  int i;
  int num_after;

  DEBUGASSERT(ts_core != NULL);

  if (ts_core->nfiles_deleted <= 0)
    return;

  DEBUGASSERT(ts_core->nfiles > 0 && ts_core->nfiles_deleted <= ts_core->nfiles);

  for (i = 0; i < ts_core->nfiles && ts_core->nfiles_deleted > 0; i++)
    {
      if (ts_core->pollfds[i].fd >= 0)
        {
          /* Skip active file. */

          continue;
        }

      num_after = ts_core->nfiles - i - 1;
      if (num_after > 0)
        {
          /* Move tail of list forward. */

          memmove(&ts_core->pollfds[i], &ts_core->pollfds[i + 1],
                  num_after * sizeof(ts_core->pollfds[i]));
          memmove(&ts_core->files[i], &ts_core->files[i + 1],
                  num_after * sizeof(ts_core->files[i]));
        }

      ts_core->nfiles--;
      ts_core->nfiles_deleted--;
      i--;
    }

  DEBUGASSERT(ts_core->nfiles_deleted == 0);
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
#endif /* CONFIG_THINGSEE_DEEPSLEEP_WATCHDOG */

/****************************************************************************
 * Name: ts_core_deepsleep
 *
 * Description:
 *   Perform deep-sleep if allowed.
 *
 * Input Parameters:
 *   ts_core     - Pointer to Thingsee core library structure
 *
 * Return Value:
 *   Time deep-slept in milliseconds.
 *
 ****************************************************************************/
#ifndef CONFIG_THINGSEE_DEEPSLEEP_DISABLED
static int ts_core_deepsleep(struct ts_core_s * const ts_core)
{
  struct deepsleep_hook_s *entry;
  struct file_s *file;
  struct pollfd *pfd;
  int timeout_msecs;
  int time_deepslept = 0;

  sched_lock();

  entry = (struct deepsleep_hook_s *)sq_peek(&ts_core->deepsleep_hooks);
  while (entry)
    {
      if (entry->hook)
        {
          if (!entry->hook(entry->priv))
            {
              /* Hook prevented deep-sleep. */

              deepsleep_dbg("Hook %p(%p)<%s> prevented deep-sleep.\n",
                            entry->hook, entry->priv, entry->reg_func_name);

              goto out;
            }
        }

      /* Move to next hook */

      entry = (struct deepsleep_hook_s *)sq_next(&entry->entry);
    }

  /* Check all file-descriptors if there is work left to do. */

  if (ts_core->nfiles > 0)
    {
      int i, ret;

      /* Check with poll if any file has work left. */

      ret = poll(ts_core->pollfds, ts_core->nfiles, 0);

      for (i = 0; ret > 0 && i < ts_core->nfiles; i++)
        {
          file = &ts_core->files[i];
          pfd = &ts_core->pollfds[i];

          ret -= (pfd->revents != 0);

          if (pfd->revents != 0)
            {
              deepsleep_dbg("Poll %p(%p)<%s> prevented deep-sleep.\n",
                            file->callback, file->priv, file->reg_func_name);
              (void)file;

              goto out;
            }
        }
    }

  /* Deep-sleep ok for all hooks. */

  timeout_msecs = ts_core_timer_get_timeout(ts_core);

  if (timeout_msecs == 0)
    {
      deepsleep_dbg("Deep-sleep: %s.\n", "Timer already expired, skipping deep-sleep");

      goto out; /* Timer already expired. */
    }
#ifndef BOARD_HAS_SUBSECOND_DEEPSLEEP
  else if (timeout_msecs > 0 && timeout_msecs < 1000)
    {
      deepsleep_dbg("Deep-sleep: %s.\n", "Timer expiring soon, skipping deep-sleep");

      goto out;
    }
#endif

  if (timeout_msecs < 0)
    {
      timeout_msecs = 0; /* No timers active, sleep to interrupt. */

      deepsleep_dbg("Deep-sleep: %s.\n", "No timers active, sleep to interrupt");
    }
  else
    {
      if (timeout_msecs > BOARD_DEEPSLEEP_MAX_SECS * 1000)
        {
          timeout_msecs = BOARD_DEEPSLEEP_MAX_SECS * 1000;
        }

      deepsleep_dbg("Deep-sleep: Sleeping %.3f seconds.\n", timeout_msecs / 1000.0);
    }

#ifndef CONFIG_ARCH_SIM
  struct timespec ts_entry, ts_exit;

  /* Go to deep-sleep mode. */

  (void)clock_gettime(CLOCK_MONOTONIC, &ts_entry);
#ifdef BOARD_HAS_SUBSECOND_DEEPSLEEP
  board_deepsleep_with_stopmode_msecs(timeout_msecs);
#else
  board_deepsleep_with_stopmode(timeout_msecs / 1000);
#endif
  (void)clock_gettime(CLOCK_MONOTONIC, &ts_exit);

  /* How long deep-slept? */

  time_deepslept = ts_exit.tv_sec - ts_entry.tv_sec;
  if (time_deepslept < INT_MAX / MSEC_PER_SEC)
    {
      time_deepslept *= MSEC_PER_SEC;
      time_deepslept += (ts_exit.tv_nsec - ts_entry.tv_nsec) / NSEC_PER_MSEC;
      DEBUGASSERT(time_deepslept >= 0);
    }
  else
    {
      time_deepslept = INT_MAX;
    }
#endif

#ifdef CONFIG_THINGSEE_DEEPSLEEP_WATCHDOG
  /* Reset deep-sleep timer. */

  reset_deepsleep_watchdog(ts_core);
#endif // CONFIG_THINGSEE_DEEPSLEEP_WATCHDOG

out:
  sched_unlock();
  return time_deepslept;
}
#else
static int ts_core_deepsleep(struct ts_core_s * const ts_core)
{
  return 0;
}
#endif /* CONFIG_THINGSEE_DEEPSLEEP_DISABLED */

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
  struct file_s *file;
  struct pollfd *pfd;
  int timeout_ms = -1;
  int time_deepslept = 0;
  int nrevents;
  int ret;
  int i;

  /* Deep-sleep handling */

  time_deepslept = ts_core_deepsleep(ts_core);

  /* Get timeout to next timer */

  timeout_ms = ts_core_timer_get_timeout(ts_core);

  /* Limit poll/normal timer length so that we re-evaluate deep-sleepiness
   * periodically. */

  if (timeout_ms > TS_CORE_MAX_POLL_TIMEOUT_MSEC || timeout_ms == -1)
    {
      timeout_ms = TS_CORE_MAX_POLL_TIMEOUT_MSEC - time_deepslept;
      if (timeout_ms < TS_CORE_MIN_POLL_TIMEOUT_MSEC)
        timeout_ms = TS_CORE_MIN_POLL_TIMEOUT_MSEC;
    }

  if (timeout_ms)
    {
      deepsleep_dbg("poll-timeout: waiting poll event for %d milliseconds.\n", timeout_ms);
    }

  if (ts_core->nfiles)
    {
      perf_dbg_start_ticks(ts_core);

      /* Poll file descriptors with timeout */

      ret = poll(ts_core->pollfds, ts_core->nfiles, timeout_ms);

      perf_dbg_add_elapsed_ticks(ts_core, poll);

      if (ret < 0)
        {
          dbg("poll returned errno %d\n", get_errno());

          return ret;
        }

      nrevents = ret;

      for (i = 0; nrevents > 0 && i < ts_core->nfiles; i++)
        {
          file = &ts_core->files[i];
          pfd = &ts_core->pollfds[i];

          nrevents -= (pfd->revents != 0);

          if (pfd->revents != 0 && file->callback != NULL && pfd->fd >= 0)
            {
              int fd = pfd->fd;

              perf_dbg_start_ticks(ts_core);

              /* Execute file descriptor callback */

              ret = file->callback(pfd, file->priv);

              perf_dbg_add_elapsed_ticks(ts_core, files);

              if (ret < 0)
                dbg("Callback for file %d returned %d.\n", fd, ret);
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

  timer = calloc(1, sizeof(struct timer_s));
  if (!timer)
    return ERROR;

  /* Setup interval timer */

  timer->id = ts_core_get_timer_id(ts_core, timer);
  timer->flags.type = type;

  if (type == TS_TIMER_TYPE_DATE)
    {
      timer->date_expires = *(const struct timespec *)timeval;
      timer->date_callback = callback;
    }
  else
    {
      uint32_t timeout_ms = *(const uint32_t *)timeval;

      (void)clock_gettime(CLOCK_MONOTONIC, &timer->date_expires);
      timespec_add_msec(&timer->date_expires, timeout_ms);

      timer->callback = callback;
      timer->interval_ms = timeout_ms;
    }
  timer->priv = priv;

  /* Add timer to new timers queue */

  sq_addlast(&timer->entry, &ts_core->new_timers);

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

  /* Initialize file descriptor list */

  ts_core->nfiles = 0;
  ts_core->nfiles_deleted = 0;

  /* Initialize timer queues */

  sq_init(&ts_core->timers);
  sq_init(&ts_core->new_timers);
  sq_init(&ts_core->done_timers);

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
  struct ts_core_s *ts_core = &g_ts_core;
  struct deepsleep_hook_s *hook;

  /* Unregister file descriptors */

  memset(ts_core->pollfds, 0, sizeof(ts_core->pollfds));
  memset(ts_core->files, 0, sizeof(ts_core->files));
  ts_core->nfiles = 0;
  ts_core->nfiles_deleted = 0;

  /* Unregister deepsleep hooks */

  /* Remove hook from queue */

  while ((hook = (struct deepsleep_hook_s *)sq_remfirst(
                   &ts_core->deepsleep_hooks)) != NULL)
    {
      /* Free memory associated with hook */

      free(hook);
    }

  /* Free timers */

  purge_timers(&ts_core->timers);
  purge_timers(&ts_core->new_timers);
  purge_timers(&ts_core->done_timers);

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
  struct ts_core_s *ts_core = &g_ts_core;
  struct file_s *file;
  struct pollfd *pfd;
  int file_idx;

  /* Check input parameters */

  if (fd < 0 || !events || !callback)
    {
      set_errno(EINVAL);

      return ERROR;
    }

  for (file_idx = 0; file_idx < ts_core->nfiles; file_idx++)
    {
      file = &ts_core->files[file_idx];
      pfd = &ts_core->pollfds[file_idx];

      /* Check if file descriptor is already registered */

      if (pfd->fd == fd)
        {
          /* File descriptor is already registered */

          set_errno(EEXIST);

          return ERROR;
        }

      /* Check if file entry is freed */

      if (pfd->fd < 0)
        {
          DEBUGASSERT(ts_core->nfiles_deleted > 0);

          /* Reuse this file entry. */

          break;
        }
    }

  if (file_idx >= TS_CORE_NFILES)
    {
      set_errno(ENOMEM);

      return ERROR;
    }

  if (file_idx == ts_core->nfiles)
    {
      /* Not 'reused', increase count. */

      ts_core->nfiles++;
    }
  else
    {
      /* Reused deleted entry. */

      ts_core->nfiles_deleted--;
    }

  file = &ts_core->files[file_idx];
  pfd = &ts_core->pollfds[file_idx];

  /* Initialize memory for file */

  memset(file, 0, sizeof(*file));
  memset(pfd, 0, sizeof(*pfd));

  /* Setup file descriptor for monitoring */

  pfd->fd = fd;
  pfd->events = events;
  file->callback = callback;
  file->priv = priv;
  file->reg_func_name = reg_func_name;

  return OK;
}

/****************************************************************************
 * Name: ts_core_fd_set_poll_events
 *
 * Description:
 *   Update new poll events for file descriptor.
 *   Call this only from callback
 *
 * Input Parameters:
 *   fd          - File descriptor that is currently monitored
 *   events      - new poll events
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ts_core_fd_set_poll_events(const int fd, const pollevent_t events)
{
  struct ts_core_s *ts_core = &g_ts_core;
  struct pollfd *pfd;
  int i;

  /* Check input parameters */

  if (fd < 0)
    {
      set_errno(EINVAL);

      return ERROR;
    }

  for (i = 0; i < ts_core->nfiles; i++)
    {
      pfd = &ts_core->pollfds[i];

      /* Check if file descriptor matches */

      if (pfd->fd == fd)
        {
          /* Update poll events */

          pfd->events = events;

          return OK;
        }
    }

  set_errno(EBADF);

  return ERROR;
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
  struct ts_core_s *ts_core = &g_ts_core;
  struct pollfd *pfd;
  int i;

  /* Check input parameters */

  if (fd < 0)
    {
      set_errno(EINVAL);

      return ERROR;
    }

  for (i = 0; i < ts_core->nfiles; i++)
    {
      pfd = &ts_core->pollfds[i];

      /* Check if file descriptor matches */

      if (pfd->fd == fd)
        {
          /* Mark file for garbage collection */

          pfd->fd = -1;
          pfd->revents = 0;
          ts_core->nfiles_deleted++;

          return OK;
        }
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
  int i;
  sq_queue_t *timer_queues[] =
    {
      &ts_core->timers,
      &ts_core->new_timers,
      NULL,
    };

  /* Check input parameters */

  if (timer_id < 0)
    {
      set_errno(EINVAL);

      return ERROR;
    }

  for (i = 0; timer_queues[i]; i++)
    {
      struct timer_s * prev = NULL;

      timer = (struct timer_s *)sq_peek(timer_queues[i]);
      while (timer)
        {
          if (timer->id == timer_id)
            {
              /* Remove timer from active/new queue */

              if (prev)
                sq_remafter(&prev->entry, timer_queues[i]);
              else
                sq_remfirst(timer_queues[i]);

              /* Add timer to completed/'to-be-freed' queue */

              sq_addlast(&timer->entry, &ts_core->done_timers);
              timer->flags.done = true;

              return OK;
            }

          /* Move to next timer */

          prev = timer;
          timer = (struct timer_s *)sq_next(&timer->entry);
        }
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
  struct deepsleep_hook_s * prev = NULL;

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

          if (prev)
            sq_remafter(&prev->entry, &ts_core->deepsleep_hooks);
          else
            sq_remfirst(&ts_core->deepsleep_hooks);

          free(entry);

          return OK;
        }

      /* Move to next hook */

      prev = entry;
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

  /* Move timers registered before starting mainloop from new queue to active
   * queue.*/

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
 * Name: ts_core_timer_selftest_date_cb
 *
 * Description:
 *   Thingsee core timer callback during selftests
 *
 ****************************************************************************/
static int ts_core_timer_selftest_date_cb(const int timer_id,
                                          const struct timespec *date,
                                          void * const priv)
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
  struct timespec curr_ts;
  struct ts_core_s * ts_core = &g_ts_core;
  struct timer_s * timer;
  int timeout_id;
  bool timeout_cb_ok = false;
  int interval_id;
  bool interval_cb_ok = false;
  int date_id;
  bool date_cb_ok = false;

  /* Check that there are no timers registered */

  DEBUGASSERT(sq_peek(&ts_core->timers) == NULL);
  DEBUGASSERT(sq_peek(&ts_core->new_timers) == NULL);

  /* Setup timeout timer */

  timeout_id = ts_core_timer_setup(TS_TIMER_TYPE_TIMEOUT,
                                   1 * MSEC_PER_TICK,
                                   ts_core_timer_selftest_cb,
                                   &timeout_cb_ok);
  DEBUGASSERT(ts_core->heap_size > 0);
  DEBUGASSERT(timeout_id >= TS_CORE_FIRST_TIMER_ID);
  DEBUGASSERT(timeout_id < TS_CORE_FIRST_TIMER_ID + ts_core->heap_size);

  /* Setup interval timer */

  interval_id = ts_core_timer_setup(TS_TIMER_TYPE_INTERVAL,
                                    2 * MSEC_PER_TICK,
                                    ts_core_timer_selftest_cb,
                                    &interval_cb_ok);
  DEBUGASSERT(interval_id >= TS_CORE_FIRST_TIMER_ID);
  DEBUGASSERT(interval_id < TS_CORE_FIRST_TIMER_ID + ts_core->heap_size);
  DEBUGASSERT(interval_id != timeout_id);

  /* Setup date timer */

  (void)clock_gettime(CLOCK_MONOTONIC, &curr_ts);
  timespec_add_msec(&curr_ts, 2 * MSEC_PER_TICK);
  date_id = ts_core_timer_setup_date(&curr_ts,
                                     ts_core_timer_selftest_date_cb,
                                     &date_cb_ok);
  DEBUGASSERT(date_id >= TS_CORE_FIRST_TIMER_ID);
  DEBUGASSERT(date_id < TS_CORE_FIRST_TIMER_ID + ts_core->heap_size);
  DEBUGASSERT(date_id != timeout_id);
  DEBUGASSERT(date_id != interval_id);

  /* Check timeout timer */

  timer = (struct timer_s *)sq_peek(&ts_core->new_timers);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_TIMEOUT);

  /* Check interval timer */

  timer = (struct timer_s *)sq_next(&timer->entry);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_INTERVAL);

  /* Check date timer */

  timer = (struct timer_s *)sq_next(&timer->entry);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_DATE);

  timer = (struct timer_s *)sq_next(&timer->entry);
  DEBUGASSERT(!timer);

  /* Clear 'is_new' mark. */

  ts_core_gc_timers(ts_core);

  timer = (struct timer_s *)sq_peek(&ts_core->new_timers);
  DEBUGASSERT(!timer);

  /* Check timeout timer */

  timer = (struct timer_s *)sq_peek(&ts_core->timers);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_TIMEOUT);

  /* Check interval timer */

  timer = (struct timer_s *)sq_next(&timer->entry);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_INTERVAL);

  /* Check date timer */

  timer = (struct timer_s *)sq_next(&timer->entry);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_DATE);

  timer = (struct timer_s *)sq_next(&timer->entry);
  DEBUGASSERT(!timer);

  /* Wait for timers to expire */

  usleep(4 * USEC_PER_TICK);

  /* Execute callbacks */

  ts_core_process_timers(ts_core);

  /* Check that callbacks where called */

  DEBUGASSERT(timeout_cb_ok == true);
  DEBUGASSERT(interval_cb_ok == true);
  DEBUGASSERT(date_cb_ok == true);

  /* Check interval timer (should be in new timers queue for reinsertion) */

  timer = (struct timer_s *)sq_peek(&ts_core->new_timers);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_INTERVAL);

  timer = (struct timer_s *)sq_next(&timer->entry);
  DEBUGASSERT(!timer);

  /* Check timeout timer (done) */

  timer = (struct timer_s *)sq_peek(&ts_core->done_timers);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_TIMEOUT);

  /* Check date timer (done) */

  timer = (struct timer_s *)sq_next(&timer->entry);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_DATE);

  timer = (struct timer_s *)sq_next(&timer->entry);
  DEBUGASSERT(!timer);

  timer = (struct timer_s *)sq_peek(&ts_core->timers);
  DEBUGASSERT(!timer);

  /* Garbage collect non-active timers */

  ts_core_gc_timers(ts_core);

  timer = (struct timer_s *)sq_peek(&ts_core->new_timers);
  DEBUGASSERT(!timer);
  timer = (struct timer_s *)sq_peek(&ts_core->done_timers);
  DEBUGASSERT(!timer);
  timer = (struct timer_s *)sq_peek(&ts_core->timers);
  DEBUGASSERT(timer);
  DEBUGASSERT(timer->flags.type == TS_TIMER_TYPE_INTERVAL);

  /* Try to stop timeout timer, which has already been removed */

  DEBUGASSERT(ts_core_timer_stop(timeout_id) == ERROR);

  /* Stop interval timer */

  DEBUGASSERT(ts_core_timer_stop(interval_id) == OK);

  /* Try to stop date timer, which has already been removed */

  DEBUGASSERT(ts_core_timer_stop(date_id) == ERROR);

  /* Garbage collect non-active timers */

  ts_core_gc_timers(ts_core);

  /* Check that there are no timers registered */

  DEBUGASSERT(sq_peek(&ts_core->timers) == NULL);
  DEBUGASSERT(sq_peek(&ts_core->new_timers) == NULL);
  DEBUGASSERT(sq_peek(&ts_core->done_timers) == NULL);
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
  struct ts_core_s *ts_core = &g_ts_core;
  struct file_s *file;
  struct pollfd *pfd;

  /* Check that there are no files registered */

  DEBUGASSERT(ts_core->nfiles == 0);
  DEBUGASSERT(ts_core->nfiles_deleted == 0);

  /* Register fd 1 poll in callback */

  DEBUGASSERT(ts_core_fd_register(1, POLLIN, ts_core_fd_selftest_cb,
                                  (void*)(uintptr_t)10) == OK);

  /* Register fd 2 poll out callback */

  DEBUGASSERT(ts_core_fd_register(2, POLLOUT, ts_core_fd_selftest_cb,
                                  (void*)(uintptr_t)20) == OK);

  /* Register fd 3 poll out|in callback */

  DEBUGASSERT(ts_core_fd_register(3, POLLOUT|POLLIN, ts_core_fd_selftest_cb,
                                  (void*)(uintptr_t)30) == OK);

  /* Check that files where registered correctly */

  DEBUGASSERT(ts_core->nfiles == 3);
  DEBUGASSERT(ts_core->nfiles_deleted == 0);

  file = &ts_core->files[0];
  pfd = &ts_core->pollfds[0];
  DEBUGASSERT(file->callback == ts_core_fd_selftest_cb);
  DEBUGASSERT(file->priv == (void*)(uintptr_t)10);
  DEBUGASSERT(pfd->events == POLLIN);
  DEBUGASSERT(pfd->fd == 1);

  file = &ts_core->files[1];
  pfd = &ts_core->pollfds[1];
  DEBUGASSERT(file->callback == ts_core_fd_selftest_cb);
  DEBUGASSERT(file->priv == (void*)(uintptr_t)20);
  DEBUGASSERT(pfd->events == POLLOUT);
  DEBUGASSERT(pfd->fd == 2);

  file = &ts_core->files[2];
  pfd = &ts_core->pollfds[2];
  DEBUGASSERT(file->callback == ts_core_fd_selftest_cb);
  DEBUGASSERT(file->priv == (void*)(uintptr_t)30);
  DEBUGASSERT(pfd->events == (POLLOUT|POLLIN));
  DEBUGASSERT(pfd->fd == 3);

  /* Unregister fd 3 */

  DEBUGASSERT(ts_core_fd_unregister(3) == OK);

  /* Unregister fd 2 */

  DEBUGASSERT(ts_core_fd_unregister(2) == OK);

  /* Check that files where unregistered correctly */

  DEBUGASSERT(ts_core->nfiles == 3);
  DEBUGASSERT(ts_core->nfiles_deleted == 2);

  file = &ts_core->files[0];
  pfd = &ts_core->pollfds[0];
  DEBUGASSERT(file->callback == ts_core_fd_selftest_cb);
  DEBUGASSERT(file->priv == (void*)(uintptr_t)10);
  DEBUGASSERT(pfd->events == POLLIN);
  DEBUGASSERT(pfd->fd == 1);

  file = &ts_core->files[1];
  pfd = &ts_core->pollfds[1];
  DEBUGASSERT(file->callback == ts_core_fd_selftest_cb);
  DEBUGASSERT(file->priv == (void*)(uintptr_t)20);
  DEBUGASSERT(pfd->events == POLLOUT);
  DEBUGASSERT(pfd->fd == -1);

  file = &ts_core->files[2];
  pfd = &ts_core->pollfds[2];
  DEBUGASSERT(file->callback == ts_core_fd_selftest_cb);
  DEBUGASSERT(file->priv == (void*)(uintptr_t)30);
  DEBUGASSERT(pfd->events == (POLLOUT|POLLIN));
  DEBUGASSERT(pfd->fd == -1);

  /* Garbage collect files */

  ts_core_gc_files(ts_core);

  DEBUGASSERT(ts_core->nfiles == 1);
  DEBUGASSERT(ts_core->nfiles_deleted == 0);

  file = &ts_core->files[0];
  pfd = &ts_core->pollfds[0];
  DEBUGASSERT(file->callback == ts_core_fd_selftest_cb);
  DEBUGASSERT(file->priv == (void*)(uintptr_t)10);
  DEBUGASSERT(pfd->events == POLLIN);
  DEBUGASSERT(pfd->fd == 1);

  /* Unregister fd 1 */

  DEBUGASSERT(ts_core_fd_unregister(1) == OK);

  DEBUGASSERT(ts_core->nfiles == 1);
  DEBUGASSERT(ts_core->nfiles_deleted == 1);

  file = &ts_core->files[0];
  pfd = &ts_core->pollfds[0];
  DEBUGASSERT(file->callback == ts_core_fd_selftest_cb);
  DEBUGASSERT(file->priv == (void*)(uintptr_t)10);
  DEBUGASSERT(pfd->events == POLLIN);
  DEBUGASSERT(pfd->fd == -1);

  /* Garbage collect files */

  ts_core_gc_files(ts_core);

  /* Check that there are no files registered */

  DEBUGASSERT(ts_core->nfiles == 0);
  DEBUGASSERT(ts_core->nfiles_deleted == 0);
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
