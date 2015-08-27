/****************************************************************************
 * libc/syslog/lib_extrafmt.c
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
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
#include <syslog.h>
#include <debug.h>
#include <pthread.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

typedef int (*vsyslog_fn_t)(int priority, FAR const char *format, va_list ap);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Constant Data
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int xvsyslog_extrafmt(int prio, vsyslog_fn_t xvsyslog,
                             const char *fmt, va_list vargs)
{
  struct timespec _ts;

  clock_gettime(CLOCK_REALTIME, &_ts);
  if (_ts.tv_nsec / NSEC_PER_MSEC == 1000)
    {
      _ts.tv_sec += 1;
      _ts.tv_nsec = 0;
    }

  return xvsyslog(prio, fmt, vargs);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int syslog_extrafmt(int prio, const char *fmt, ...)
{
  va_list vargs;
  int ret;

  pthread_mutex_lock(&g_mutex);
  va_start(vargs, fmt);
  ret = xvsyslog_extrafmt(prio, vsyslog, fmt, vargs);
  va_end(vargs);
  pthread_mutex_unlock(&g_mutex);

  return ret;
}

int lowsyslog_extrafmt(int prio, const char *fmt, ...)
{
  va_list vargs;
  int ret;

  va_start(vargs, fmt);
  ret = xvsyslog_extrafmt(prio, lowvsyslog, fmt, vargs);
  va_end(vargs);

  return ret;
}
