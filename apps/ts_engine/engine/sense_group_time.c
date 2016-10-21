/****************************************************************************
 * apps/ts_engine/engine/sense_group_time.c
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
 *   Pekka Ervasti <pekka.ervasti@haltian.com>
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include "eng_dbg.h"
#include "parse.h"
#include "execute.h"
#include "sense.h"
#include "util.h"

static int timer_callback_date(const int timer_id, const struct timespec *date,
    void * const priv)
{
  struct ts_cause *cause = priv;

  return handle_cause(cause);
}

int sense_time_init(struct ts_cause *cause)
{
  struct timespec *ts = &cause->dyn.time;
  double thr;

  if (!__ts_engine_find_threshold(cause, isOneOf, &thr, NULL))
    {
      eng_dbg("No threshold set, aborting...");
      return ERROR;
    }

  switch (cause->dyn.sense_info->sId)
    {
    case SENSE_ID_UNIX_TIME:
      {

        if (cause->conf.threshold.relative)
          {
            clock_gettime(CLOCK_MONOTONIC, ts);
            ts->tv_sec += (uint32_t)thr;
            cause->dyn.sense_value.value.valueuint32 = (uint32_t)thr;
          }
        else
          {
            struct timespec realtime, monotonic;

            clock_gettime(CLOCK_REALTIME, &realtime);
            clock_gettime(CLOCK_MONOTONIC, &monotonic);

            ts->tv_sec = monotonic.tv_sec + ((uint32_t)thr - realtime.tv_sec);
            ts->tv_nsec = 0;

            cause->dyn.sense_value.value.valueuint32 = (uint32_t)thr;
          }

        cause->dyn.timer_id = ts_core_timer_setup_date(ts, timer_callback_date, cause);
        if (cause->dyn.timer_id < 0)
          {
            eng_dbg("ts_core_timer_setup_date failed\n");
            return ERROR;
          }
      }
    break;
    }

  return OK;
}
