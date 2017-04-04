/****************************************************************************
 * apps/system/ubmodem/ubmodem_poll.c
 *
 *   Copyright (C) 2014-2017 Haltian Ltd. All rights reserved.
 *   Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#include <apps/system/ubmodem.h>

#include "ubmodem_command.h"
#include "ubmodem_parser.h"
#include "ubmodem_internal.h"
#include "ubmodem_usrsock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Modem timer structure. */

struct modem_priv_timer_s
{
  sq_entry_t entry;
  struct timespec alarm;
  ubmodem_timer_fn_t callback;
  void *cb_priv;
#ifdef CONFIG_UBMODEM_DEBUG_VERBOSE
  const char *regname;
#endif
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

/****************************************************************************
 * Name: modem_parse_response
 *
 * Description:
 *   Modem file descriptor poll callback, called when more data is available
 *   from modem.
 *
 * Input Parameters:
 *   pfd  : pollfd structure
 *   arg  : private data
 *
 ****************************************************************************/

static int modem_parse_response(struct ubmodem_s *modem)
{
  int serial_fd = modem->serial_fd;
  struct at_parser_s *parser = &modem->parser;
  ssize_t nread;

  DEBUGASSERT(modem->initialized);

  /* Read serial device in non-blocking manner */

  if (__ubmodem_set_nonblocking(modem, true) != OK)
    return ERROR;

  do
    {
      nread = read(serial_fd, parser->readbuf, sizeof(parser->readbuf));
      if (nread < 0)
        {
          if (errno == EAGAIN)
            return OK;

          return ERROR;
        }

      /* Give trace output if active. */

      if (modem->active_events & UBMODEM_EVENT_FLAG_TRACE_DATA_FROM_MODEM)
        __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_TRACE_DATA_FROM_MODEM,
                                parser->readbuf, nread);

      /* Feed bytes to parser. */

      __ubparse_buffer(parser, parser->readbuf, nread);
    }
  while(true);

  return OK;
}

static void modem_handle_expired_timer(struct ubmodem_s *modem,
                                       struct modem_priv_timer_s *timer)
{
  ubmodem_timer_fn_t callback;
  void *cb_priv;
  int ret;
  int id;

  ubdbg("timer [%d]:%08x:<%s> expired.\n",
        timer->id, timer->callback, timer->regname);

  /* Expired, clean-up entry and call callback. */

  callback = timer->callback;
  cb_priv = timer->cb_priv;
  id = timer->id;

  timer->callback = NULL;
  timer->cb_priv = NULL;

  sq_rem(&timer->entry, &modem->timers);
  free(timer);

  ret = callback(modem, id, cb_priv);
  if (ret == ERROR)
    {
      ubdbg("Timer callback returned error!\n");
    }
}

static int modem_poll_event(struct ubmodem_s *modem, struct pollfd *pfd,
                            int numpfds)
{
  struct modem_priv_timer_s *timer;
  struct timespec curr_ts;
  int ret;

  if (pfd == NULL && numpfds > 0)
    {
      errno = EINVAL;
      return ERROR;
    }

  /* Handle internal timers. */

  timer = (void *)sq_peek(&modem->timers);
  if (timer)
    {
      clock_gettime(CLOCK_MONOTONIC, &curr_ts);

      /* Add nanoseconds to entropy pool. */

      __ubmodem_seed_urandom(&curr_ts.tv_nsec, sizeof(curr_ts.tv_nsec));
    }
  while (timer)
    {
      struct timespec *alarm_ts = &timer->alarm;
      int64_t msec_diff;

      DEBUGASSERT(timer->callback != NULL);

      msec_diff = alarm_ts->tv_sec;
      msec_diff -= curr_ts.tv_sec;
      msec_diff *= 1000;

      /* 'timespec->tv_nsec' is signed long. */

      msec_diff += (alarm_ts->tv_nsec - curr_ts.tv_nsec) / (1000 * 1000);

      if (msec_diff <= 0)
        {
          modem_handle_expired_timer(modem, timer);

          /* Timers list might had been modified by timer callback, restart
           * iterating to avoid accessing freed memory.
           */

          timer = (void *)sq_peek(&modem->timers);
        }
      else
        {
          timer = (void *)sq_next(&timer->entry);
        }
    }

  while (numpfds > 0)
    {
      /* Check if poll activated. */

      if (pfd->revents & POLLIN)
        {
          /* Handle POLLIN event. */

          if (pfd->fd == modem->serial_fd)
            {
              ret = modem_parse_response(modem);
            }
#ifdef CONFIG_UBMODEM_USRSOCK
          else if (pfd->fd == modem->sockets.usrsockfd &&
                   modem->sockets.poll_off_count == 0)
            {
              ret = __ubmodem_usrsock_handle_request(modem);
            }
#endif
          else
            {
              ret = OK; /* fd might have been freed in response handler. */
            }

          if (ret != OK)
            {
              dbg("POLLIN handling failed, %d - %d\n", ret, errno);
              return ERROR;
            }

          pfd->revents &= ~POLLIN;
        }

      /* TODO: Handle POLLOUT event. */

      pfd++;
      numpfds--;
    }

  return OK;
}

static int modem_get_unused_timer_id(struct ubmodem_s *modem)
{
  int candidate;
  int first = -1;
  bool used;

  do
    {
      struct modem_priv_timer_s *timer;

      candidate = ++modem->timer_id_cnt;

      if (first >= 0)
        {
          DEBUGASSERT(first != candidate); /* Assert that we never reach first
                                            * candidate again (2¹⁶ timers would
                                            * be registered in that case). */
        }
      else
        {
          first = candidate;
        }

      used = false;
      timer = (void *)sq_peek(&modem->timers);
      while (timer)
        {
          if (timer->id == candidate)
            {
              /* ID in use. */

              used = true;
              break;
            }

          timer = (void *)sq_next(&timer->entry);
        }
    }
  while (used);

  return candidate;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_set_timer
 *
 * Description:
 *   Setup timer with callback.
 *
 ****************************************************************************/

int ___ubmodem_set_timer(struct ubmodem_s *modem,
                        unsigned int timeout_msec, ubmodem_timer_fn_t timer_cb,
                        void *cb_priv, const char *regname)
{
  struct timespec alarm;
  struct modem_priv_timer_s *timer;
  unsigned int orig_msec = timeout_msec;

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

  timer->id = modem_get_unused_timer_id(modem);
  timer->callback = timer_cb;
  timer->alarm = alarm;
  timer->cb_priv = cb_priv;
#ifdef CONFIG_UBMODEM_DEBUG_VERBOSE
  timer->regname = regname;
#endif

  sq_addlast(&timer->entry, &modem->timers);

  ubdbg("set timer [%d]:%08x:<%s>, %d msec.\n",
        timer->id, timer->callback, timer->regname, orig_msec);
  (void)orig_msec;

  return timer->id;
}

/****************************************************************************
 * Name: ubmodem_remove_timer
 *
 * Description:
 *   Remove timer.
 *
 ****************************************************************************/

void __ubmodem_remove_timer(struct ubmodem_s *modem, uint16_t id)
{
  struct modem_priv_timer_s *timer;

  timer = (void *)sq_peek(&modem->timers);
  while (timer)
    {
      if (timer->id == id)
        {
          ubdbg("remove timer [%d]:%08x:<%s>.\n",
                timer->id, timer->callback, timer->regname);

          sq_rem(&timer->entry, &modem->timers);
          free(timer);
          return;
        }

      timer = (void *)sq_next(&timer->entry);
    }
}

/****************************************************************************
 * Name: ubmodem_setup_poll
 *
 * Description:
 *   Setup pollfd structure and timeout value for ubmodem library
 *
 * Input Parameters:
 *   None.
 *
 * Returned Values:
 *   Number of setup pollfds.
 *
 ****************************************************************************/

int ubmodem_pollfds_setup(struct ubmodem_s *modem, struct pollfd *pfd,
                          int numpfds, int *timeout)
{
  struct modem_priv_timer_s *timer, *next;
  struct timespec curr_ts;
  int shortest_msec;
  int ntimers;
  int pos;

  DEBUGASSERT(modem);

  if (pfd == NULL || timeout == NULL)
    {
      errno = EINVAL;
      return ERROR;
    }

  /* Get poll 'timeout' based on internal timers. */

  ntimers = 0;
  shortest_msec = INT_MAX;

  timer = (void *)sq_peek(&modem->timers);
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

  pos = 0;

  if (pos < numpfds)
    {
      memset(&pfd[pos], 0, sizeof(pfd[pos]));
      pfd[pos].fd = modem->serial_fd;
      pfd[pos].events = POLLIN;
      pfd[pos].revents = 0;
      pos++;
    }

#ifdef CONFIG_UBMODEM_USRSOCK
  if (pos < numpfds && modem->sockets.usrsockfd > -1)
    {
      if (modem->sockets.poll_off_count == 0)
        {
          memset(&pfd[pos], 0, sizeof(pfd[pos]));
          pfd[pos].fd = modem->sockets.usrsockfd;
          pfd[pos].events = POLLIN;
          pfd[pos].revents = 0;
          pos++;
        }
    }
#endif

  /* TODO: POLLOUT handling for outgoing buffer. */

  return pos;
}

/****************************************************************************
 * Name: ubmodem_setup_poll
 *
 * Description:
 *   Setup pollfd structure and timeout value for ubmodem library
 *
 * Input Parameters:
 *   None.
 *
 * Returned Values:
 *   Number of setup pollfds.
 *
 ****************************************************************************/

int ubmodem_poll_setup(struct ubmodem_s *modem, struct pollfd *pfd,
                       int *timeout)
{
  int ret;

  ret = ubmodem_pollfds_setup(modem, pfd, 1, timeout);
  if (ret < 0)
    return ret;

  return 0;
}

/****************************************************************************
 * Name: ubmodem_poll_event
 *
 * Description:
 *   Indicate poll event for ubmodem library. Library will handle current
 *   pending I/O and internal state changes.
 *
 * Input Parameters:
 *   pfd: Poll structure for ubmodem, setup with ubmodem_poll_setup
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int ubmodem_poll_event(struct ubmodem_s *modem, struct pollfd *pfd)
{
  DEBUGASSERT(modem);

  return modem_poll_event(modem, pfd, 1);
}

/****************************************************************************
 * Name: ubmodem_poll_timedout
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

int ubmodem_poll_timedout(struct ubmodem_s *modem)
{
  DEBUGASSERT(modem);

  return modem_poll_event(modem, NULL, 0);
}

/****************************************************************************
 * Name: ubmodem_pollfds_event
 *
 * Description:
 *   Indicate poll event for ubmodem library. Library will handle current
 *   pending I/O and internal state changes.
 *
 * Input Parameters:
 *   pfd: Poll structure array for ubmodem, setup with ubmodem_poll_setup
 *   numfds: Number of poll structures
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int ubmodem_pollfds_event(struct ubmodem_s *modem, struct pollfd *pfd,
                          int numfds)
{
  DEBUGASSERT(modem);

  return modem_poll_event(modem, pfd, numfds);
}

/****************************************************************************
 * Name: ubmodem_poll_max_fds
 *
 * Description:
 *   Maximum number of pollfds ubmodem can setup in ubmodem_poll_setup.
 ****************************************************************************/

int ubmodem_poll_max_fds(struct ubmodem_s *modem)
{
  int max_count = 1;

#ifdef CONFIG_UBMODEM_USRSOCK
  max_count += 1;
#endif

  return max_count;
}
