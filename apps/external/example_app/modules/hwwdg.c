/****************************************************************************
 * modules/hwwdg.c
 *
 * Copyright (C) 2015-2016 Haltian Ltd.
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
 *   Juha Niskanen <juha.niskanen@haltian.com>
 *   Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <sys/types.h>
#include <stdio.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>
#include <nuttx/timers/watchdog.h>

#include <apps/thingsee/ts_core.h>

#include <example_app_dbg.h>
#include <example_app_hwwdg.h>

static int g_hwwdg_fd = -1;

static int hwwdg_read_from_dev(int fd)
{
  int bytes;
  char buf[1];

  bytes = read(fd, buf, 1);
  if (bytes != 1)
    {
      exapp_dbg("read from watchdog failed: bytes=%d, errno=%d\n", bytes, errno);
      return ERROR;
    }
  else if (*buf != 'K')
    {
      /* Cannot happen */
      exapp_dbg("wolf ate your watchdog: %d\n", (int)*buf);
      return ERROR;
    }
  return OK;
}

static int hwwdg_receiver(const struct pollfd * const pfd, void * const priv)
{
  DEBUGASSERT(pfd);

  exapp_dbg("Kick dog!\n");

  return hwwdg_read_from_dev(g_hwwdg_fd);
}

int exapp_hwwdg_initialize(void)
{
  int ret;

  if (g_hwwdg_fd >= 0)
    {
      return OK; /* already open. */
    }

  ret = up_wdginitialize();
  if (ret < 0)
    {
      exapp_dbg("Could not initialize watchdog HW.\n");
      return ERROR;
    }

  g_hwwdg_fd = open("/dev/hwwdg0", O_RDONLY);
  if (g_hwwdg_fd < 0)
    {
      exapp_dbg("Cannot open watchdog, errno=%d\n", errno);
      return ERROR;
    }

  ret = ts_core_fd_register(g_hwwdg_fd, POLLIN, hwwdg_receiver, NULL);
  if (ret < 0)
    {
      exapp_dbg("ts_core_fd_register failed: %d\n", errno);
      goto errout_close;
    }

  return OK;

errout_close:
  close(g_hwwdg_fd);
  g_hwwdg_fd = -1;
  return ERROR;
}
