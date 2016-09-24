/****************************************************************************
 * apps/system/ubgps/ubgps_poll.c
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
 *   Author: Harri Luhtala <harri.luhtala@haltian.com>
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
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <stdlib.h>

#include <apps/system/ubgps.h>

#include "ubgps_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef UBGPS_DEBUG_TIMERS
 #define dbg_poll(...) dbg(__VA_ARGS__)
#else
  #define dbg_poll(...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* GPS timer structure. */

struct gps_priv_timer_s
{
  sq_entry_t entry;
  struct timespec alarm;
  ubgps_timer_fn_t callback;
  void *cb_priv;
  uint16_t id;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void gps_handle_expired_timer(struct ubgps_s *gps,
                                       struct gps_priv_timer_s *timer)
{
  ubgps_timer_fn_t callback;
  void *cb_priv;
  int ret;
  int id;

  DEBUGASSERT(gps && timer);

  /* Expired, clean-up entry and call callback. */

  callback = timer->callback;
  cb_priv = timer->cb_priv;
  id = timer->id;

  dbg_poll("id:%d, cb:%p, priv:%p\n", id, callback, cb_priv);

  timer->callback = NULL;
  timer->cb_priv = NULL;

  sq_rem(&timer->entry, &gps->timers);
  free(timer);

  ret = callback(id, cb_priv);
  if (ret == ERROR)
    {
      dbg("Timer callback returned error!\n");
    }

  /* Run garbage collector. */

  __ubgps_gc_callbacks(gps);
}

static int gps_poll_event(struct ubgps_s *gps, struct pollfd *pfd)
{
  struct gps_priv_timer_s *timer, *next;
  struct timespec curr_ts;
  int ret;

  if (pfd == NULL || pfd->fd != gps->fd)
    {
      errno = EINVAL;
      return ERROR;
    }

  /* Handle internal timers. */

  timer = (void *)sq_peek(&gps->timers);
  if (timer)
    {
      clock_gettime(CLOCK_MONOTONIC, &curr_ts);
    }
  while (timer)
    {
      struct timespec *alarm_ts = &timer->alarm;
      int64_t msec_diff;

      next = (void *)sq_next(&timer->entry);

      DEBUGASSERT(timer->callback != NULL);

      msec_diff = alarm_ts->tv_sec;
      msec_diff -= curr_ts.tv_sec;
      msec_diff *= 1000;

      /* 'timespec->tv_nsec' is signed long. */

      msec_diff += (alarm_ts->tv_nsec - curr_ts.tv_nsec) / (1000 * 1000);

      if (msec_diff <= 0)
        {
          gps_handle_expired_timer(gps, timer);
          timer = (void *)sq_peek(&gps->timers);
        }
      else
        {
          timer = next;
        }
    }

  /* Check if poll activated. */

  if (pfd->revents & POLLIN)
    {
      /* Handle POLLIN event. */

      ret = ubgps_receiver(pfd, gps);
      if (ret != OK)
        {
          dbg("POLLIN handling failed, %d - %d\n", ret, errno);
          return ERROR;
        }

      pfd->revents &= ~POLLIN;
    }

  /* TODO: Handle POLLOUT event. */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubgps_set_timer
 *
 * Description:
 *   Setup timer with callback.
 *
 ****************************************************************************/

int __ubgps_set_timer(struct ubgps_s *gps,
                      uint32_t timeout_msec, ubgps_timer_fn_t timer_cb,
                      void *cb_priv)
{
  struct timespec alarm;
  struct gps_priv_timer_s *timer;
  struct gps_event_new_timer_s event;

  DEBUGASSERT(gps && timer_cb);

  dbg_poll("timeout:%u, cb:%p, priv:%p\n", timeout_msec, timer_cb, cb_priv);

  timer = calloc(1, sizeof(*timer));
  if (!timer)
    {
      return ERROR;
    }

  clock_gettime(CLOCK_MONOTONIC, &alarm);

  while (timeout_msec >= 1000)
    {
      timeout_msec -= 1000;
      alarm.tv_sec++;
    }
  alarm.tv_nsec += timeout_msec * (1000 * 1000);

  timer->id = ++gps->timer_id_cnt;
  timer->callback = timer_cb;
  timer->alarm = alarm;
  timer->cb_priv = cb_priv;

  sq_addlast(&timer->entry, &gps->timers);

  event.super.id = GPS_EVENT_NEW_TIMER;
  ubgps_publish_event(gps, (struct gps_event_s *)&event);

  return timer->id;
}

/****************************************************************************
 * Name: ubgps_remove_timer
 *
 * Description:
 *   Remove timer.
 *
 ****************************************************************************/

void __ubgps_remove_timer(struct ubgps_s *gps, uint16_t id)
{
  struct gps_priv_timer_s *timer;

  DEBUGASSERT(gps);

  dbg_poll("id:%u\n", id);

  timer = (void *)sq_peek(&gps->timers);
  while (timer)
    {
      if (timer->id == id)
        {
          sq_rem(&timer->entry, &gps->timers);
          free(timer);
          return;
        }

      timer = (void *)sq_next(&timer->entry);
    }
}

/****************************************************************************
 * Name: ubgps_setup_poll
 *
 * Description:
 *   Setup pollfd structure and timeout value for ubgps library
 *
 * Input Parameters:
 *   None.
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int ubgps_poll_setup(struct ubgps_s *gps, struct pollfd *pfd,
                       int *timeout)
{
  struct gps_priv_timer_s *timer, *next;
  struct timespec curr_ts;
  int shortest_msec;
  int ntimers;

  DEBUGASSERT(gps);

  if (pfd == NULL || timeout == NULL)
    {
      errno = EINVAL;
      return ERROR;
    }

  /* Get poll 'timeout' based on internal timers. */

  ntimers = 0;
  shortest_msec = INT_MAX;

  timer = (void *)sq_peek(&gps->timers);
  if (timer)
    {
      clock_gettime(CLOCK_MONOTONIC, &curr_ts);
    }
  while (timer)
    {
      struct timespec *alarm_ts = &timer->alarm;
      int64_t msec_diff;

      next = (void *)sq_next(&timer->entry);

      DEBUGASSERT(timer->callback != NULL);

      msec_diff = alarm_ts->tv_sec;
      msec_diff -= curr_ts.tv_sec;
      msec_diff *= 1000;

      /* 'timespec->tv_nsec' is signed long. */

      msec_diff += (alarm_ts->tv_nsec - curr_ts.tv_nsec) / (1000 * 1000);

      if (msec_diff <= 0)
        {
          /* Already expired. */

          shortest_msec = 0;
          ntimers++;

          break;
        }

      if (msec_diff > INT_MAX)
        msec_diff = INT_MAX;

      if ((int)msec_diff < shortest_msec)
        shortest_msec = msec_diff;

      ntimers++;
      timer = next;
    }

  *timeout = (ntimers > 0) ? shortest_msec : -1;

  memset(pfd, 0, sizeof(*pfd));

  pfd->fd = gps->fd;
  pfd->events = POLLIN;
  pfd->revents = 0;

  /* TODO: POLLOUT handling for outgoing buffer. */
  return OK;
}

/****************************************************************************
 * Name: ubgps_poll_event
 *
 * Description:
 *   Indicate poll event for ubgps library. Library will handle current
 *   pending I/O and internal state changes.
 *
 * Input Parameters:
 *   pfd: Poll structure for ubgps, setup with ubgps_poll_setup
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int ubgps_poll_event(struct ubgps_s *gps, struct pollfd *pfd)
{
  DEBUGASSERT(gps);

  return gps_poll_event(gps, pfd);
}

/****************************************************************************
 * Name: ubgps_poll_timedout
 *
 * Description:
 *   Indicate that poll timed out. This allows library to handle internal
 *   timed state changes.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int ubgps_poll_timedout(struct ubgps_s *gps)
{
  struct pollfd pollfd = { };

  DEBUGASSERT(gps);

  pollfd.fd = gps->fd;
  pollfd.revents = 0;

  return gps_poll_event(gps, &pollfd);
}
