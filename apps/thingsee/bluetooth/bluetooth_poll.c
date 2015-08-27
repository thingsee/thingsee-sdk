/****************************************************************************
 * apps/thingsee/bluetooth/bluetooth_poll.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
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
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <stdlib.h>
#include <apps/thingsee/modules/ts_bluetooth.h>

#include "btle.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* BT LE timer structure. */

struct btle_priv_timer_s
{
  sq_entry_t entry;
  struct timespec alarm;
  btle_timer_fn_t callback;
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

static void bluetooth_handle_expired_timer(struct btle_s *btle,
                                           struct btle_priv_timer_s *timer)
{
  btle_timer_fn_t callback;
  void *cb_priv;
  int ret;
  int id;

  /* Expired, clean-up entry and call callback. */

  callback = timer->callback;
  cb_priv = timer->cb_priv;
  id = timer->id;

  timer->callback = NULL;
  timer->cb_priv = NULL;

  sq_rem(&timer->entry, &btle->timers);
  free(timer);

  ret = callback(id, cb_priv);
  if (ret == ERROR)
    {
      dbg("Timer callback returned error!\n");
    }
}

static int __poll_event(struct btle_s *btle, struct pollfd *pfd)
{
  struct btle_priv_timer_s *timer, *next;
  struct timespec curr_ts;
  int ret;

  if (pfd == NULL || pfd->fd != btle->fd)
    {
      errno = EINVAL;
      return ERROR;
    }

  /* Handle internal timers. */

  timer = (void *)sq_peek(&btle->timers);
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
          bluetooth_handle_expired_timer(btle, timer);
          timer = (void *)sq_peek(&btle->timers);
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

      ret = bluetooth_receiver(pfd, btle);
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
 * Name: bluetooth_set_timer
 *
 * Description:
 *   Setup timer with callback.
 *
 ****************************************************************************/

int bluetooth_set_timer(struct btle_s *btle,
                        unsigned int timeout_msec, btle_timer_fn_t timer_cb,
                        void *cb_priv)
{
  struct timespec alarm;
  struct btle_priv_timer_s *timer;

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

  timer->id = ++btle->timer_id_cnt;
  timer->callback = timer_cb;
  timer->alarm = alarm;
  timer->cb_priv = cb_priv;

  sq_addlast(&timer->entry, &btle->timers);

  return timer->id;
}

/****************************************************************************
 * Name: bluetooth_reset_timer_timeout
 *
 * Description:
 *   Reset existing timer timeout.
 *
 ****************************************************************************/

int bluetooth_reset_timer_timeout(struct btle_s *btle,
                                  uint16_t id,
                                  unsigned int timeout_msec)
{
  struct btle_priv_timer_s *timer;
  struct timespec alarm;

  timer = (void *)sq_peek(&btle->timers);
  while (timer)
    {
      if (timer->id == id)
        {
          break;
        }

      timer = (void *)sq_next(&timer->entry);
    }

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
  timer->alarm = alarm;

  return timer->id;
}


/****************************************************************************
 * Name: bluetooth_remove_timer
 *
 * Description:
 *   Remove timer.
 *
 ****************************************************************************/

void bluetooth_remove_timer(struct btle_s *btle, uint16_t id)
{
  struct btle_priv_timer_s *timer;

  timer = (void *)sq_peek(&btle->timers);
  while (timer)
    {
      if (timer->id == id)
        {
          sq_rem(&timer->entry, &btle->timers);
          free(timer);
          return;
        }

      timer = (void *)sq_next(&timer->entry);
    }
}

/****************************************************************************
 * Name: bluetooth_poll_setup
 *
 * Description:
 *   Setup pollfd structure and timeout value
 *
 * Input Parameters:
 *   None.
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int bluetooth_poll_setup(struct btle_s *btle, struct pollfd *pfd,
                         int *timeout)
{
  struct btle_priv_timer_s *timer, *next;
  struct timespec curr_ts;
  int shortest_msec;
  int ntimers;
  DEBUGASSERT(btle);

  if (pfd == NULL || timeout == NULL)
    {
      errno = EINVAL;
      return ERROR;
    }

  /* Get poll 'timeout' based on internal timers. */

  ntimers = 0;
  shortest_msec = INT_MAX;

  timer = (void *)sq_peek(&btle->timers);
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

  pfd->fd = btle->fd;
  pfd->events = POLLIN;
  pfd->revents = 0;

  return OK;
}

/****************************************************************************
 * Name: bluetooth_poll_event
 *
 * Description:
 *   Indicate poll event for btle library. Library will handle current
 *   pending I/O and internal state changes.
 *
 * Input Parameters:
 *   pfd: Poll structure for btle, setup with btle_poll_setup
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int bluetooth_poll_event(struct btle_s *btle, struct pollfd *pfd)
{
  DEBUGASSERT(btle);

  return __poll_event(btle, pfd);
}

/****************************************************************************
 * Name: bluetooth_poll_timedout
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

int bluetooth_poll_timedout(struct btle_s *btle)
{
  struct pollfd pollfd = { };

  DEBUGASSERT(btle);

  pollfd.fd = btle->fd;
  pollfd.revents = 0;

  return __poll_event(btle, &pollfd);
}
