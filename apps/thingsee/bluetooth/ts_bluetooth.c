/****************************************************************************
 * apps/thingsee/bluetooth/ts_bluetooth.c
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

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <fcntl.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <debug.h>
#include <queue.h>

#include <arch/board/board-bt.h>
#include <apps/thingsee/ts_core.h>
#include <apps/thingsee/modules/ts_bluetooth.h>

#include "btle.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ts_bluetooth_timer_callback(const int timer_id, void * const priv);
static int ts_bluetooth_poll_callback(const struct pollfd * const inpfd,
                                void * const priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct btle_s *ts_bluetooth;

struct {
  int fd;
  int timerid;
} ts_core_bluetooth = {
  .fd = -1,
  .timerid = -1,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: ts_bluetooth_core_reconfigure
 *
 * Description:
 *   Reconfigure bluetooth poll/timer events.
 *****************************************************************************/

static int ts_bluetooth_core_reconfigure(void)
{
  struct pollfd pfd = {};
  int timeout_ms;
  int ret;

  /* Get new poll setup. */

  ret = bluetooth_poll_setup(ts_bluetooth, &pfd, &timeout_ms);
  if (ret < 0)
    {
      return ret;
    }

  /* Reconfigure poll events. */

  if (ts_core_bluetooth.fd >= 0)
    ts_core_fd_unregister(ts_core_bluetooth.fd);
  if (pfd.fd >= 0)
    {
      ret = ts_core_fd_register(pfd.fd, pfd.events, ts_bluetooth_poll_callback,
                                NULL);
      DEBUGASSERT(ret >= 0);
    }

  ts_core_bluetooth.fd = pfd.fd;

  /* Reconfigure timers. */

  if (ts_core_bluetooth.timerid >= 0)
    ts_core_timer_stop(ts_core_bluetooth.timerid);
  if (timeout_ms >= 0)
    {
      ret = ts_core_timer_setup(TS_TIMER_TYPE_TIMEOUT, timeout_ms,
                                ts_bluetooth_timer_callback, NULL);
      DEBUGASSERT(ret >= 0);
      ts_core_bluetooth.timerid = ret;
    }
  else
    {
      ts_core_bluetooth.timerid = -1;
    }

  return OK;
}

/*****************************************************************************
 * Name: ts_bluetooth_deepsleep_callback
 *
 * Description:
 *   Deep-sleep hook for bluetooth
 *****************************************************************************/

static bool ts_bluetooth_deepsleep_callback(void * const priv)
{
  /* Allow deep-sleep if not advertising or connected  */

  return (ts_bluetooth_get_state() < BTLE_STATE_ADVERTISING);
}

/*****************************************************************************
 * Name: ts_bluetooth_uninit
 *
 * Description:
 *   Uninit for bluetooth
 *****************************************************************************/

static void ts_bluetooth_uninit(void)
{
  if (ts_core_bluetooth.fd >= 0)
    {
      if(ts_core_fd_unregister(ts_core_bluetooth.fd))
          dbg("core fd unregister failed\n");

      ts_core_bluetooth.fd = -1;
    }

  if (ts_core_bluetooth.timerid >= 0)
    {
      ts_core_timer_stop(ts_core_bluetooth.timerid);
      ts_core_bluetooth.timerid = -1;
    }

  ts_core_deepsleep_hook_remove(ts_bluetooth_deepsleep_callback);

  bluetooth_uninitialize(ts_bluetooth);
  ts_bluetooth = NULL;
}

/*****************************************************************************
 * Name: ts_bluetooth_timer_callback
 *
 * Description:
 *   Timer event handling for bluetooth.
 *****************************************************************************/

static int ts_bluetooth_timer_callback(const int timer_id, void * const priv)
{
  if (ts_core_bluetooth.timerid < 0)
    return OK;

  DEBUGASSERT(ts_core_bluetooth.timerid == timer_id);

  /* This is one-shot timer and have been uninitialized by core. */

  ts_core_bluetooth.timerid = -1;

  /* Process timer. */

  (void)bluetooth_poll_timedout(ts_bluetooth);

  /* Reconfigure poll/timer events. */

  return ts_bluetooth_core_reconfigure();
}

/*****************************************************************************
 * Name: ts_bluetooth_poll_callback
 *
 * Description:
 *   Poll event handling for bluetooth.
 *****************************************************************************/

static int ts_bluetooth_poll_callback(const struct pollfd * const inpfd,
                                void * const priv)
{
  struct pollfd pfd;

  if (ts_core_bluetooth.fd < 0)
    return OK;

  DEBUGASSERT(ts_core_bluetooth.fd == inpfd->fd);

  /* Process event. */

  pfd = *inpfd;
  (void)bluetooth_poll_event(ts_bluetooth, &pfd);

  /* Uninit requested while processing I/O */

  if (ts_bluetooth->uninit_pending)
    {
      ts_bluetooth_uninit();
      return OK;
    }

  /* Reconfigure poll/timer events. */

  return ts_bluetooth_core_reconfigure();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ts_bluetooth_initialize
 *
 * Description:
 *   Initialization for bluetooth module
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_bluetooth_initialize(void)
{
  int ret;

  if (ts_bluetooth)
    {
      dbg("already initialized\n");
      return OK; /* already initialized. */
    }

  ts_bluetooth = bluetooth_initialize();
  if (!ts_bluetooth)
    return ERROR;

  ret = ts_bluetooth_core_reconfigure();
  if (ret < 0)
    return ret;

  return ts_core_deepsleep_hook_add(ts_bluetooth_deepsleep_callback, NULL);
}

/****************************************************************************
 * Name: ts_bluetooth_uninitialize
 *
 * Description:
 *   Uninitialization for bluetooth module
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
void ts_bluetooth_uninitialize(void)
{
  int flags;

  if (!ts_bluetooth)
    return; /* already uninitialized. */

  /* Check whether there is pending I/O */

  flags = fcntl(ts_bluetooth->fd, F_GETFL, 0);
  if (flags & POLLIN)
    {
      /* I/O pending, perform delayed uninit */

      ts_bluetooth->uninit_pending = true;
      return;
    }
  else
    {
      ts_bluetooth_uninit();
    }
}

/****************************************************************************
 * Name: ts_bluetooth_callback_register
 *
 * Description:
 *   Register callback function for BT events and data retrieval
 *
 * Input Parameters:
 *   event_mask  - Mask of subscribed events
 *   callback    - Callback function
 *   priv        - Pointer to private data
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_bluetooth_callback_register(uint32_t const event_mask,
                                   bluetooth_callback_t callback,
                                   void * const priv)
{
  return bluetooth_callback_register(event_mask, callback, priv);
}

/****************************************************************************
 * Name: ts_bluetooth_callback_unregister
 *
 * Description:
 *   Unregister callback from BT module
 *
 * Input Parameters:
 *   callback    - Callback function
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_bluetooth_callback_unregister(bluetooth_callback_t callback)
{
  return bluetooth_callback_unregister(callback);
}

/****************************************************************************
 * Name: ts_bluetooth_get_state
 *
 * Description:
 *   Get current BT state
 *
 * Returned Values:
 *   BT state
 *
 ****************************************************************************/
int ts_bluetooth_get_state(void)
{
  return bluetooth_get_state();
}

/****************************************************************************
 * Name: ts_bluetooth_get_statename
 *
 * Description:
 *   Get BT state name
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
char const * const ts_bluetooth_get_statename(bluetooth_state_t const bt_state)
{
  return bluetooth_get_conn_state_name(bt_state);
}
