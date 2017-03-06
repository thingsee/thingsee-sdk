/****************************************************************************
 * apps/thingsee/engine/sense_group_hw_keys.c
 *
 * Copyright (C) 2014-2016 Haltian Ltd. All rights reserved.
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

#include <fcntl.h>
#include <unistd.h>

#include <debug.h>
#include <assert.h>

#include <arch/board/board.h>

#include "eng_dbg.h"
#include "parse.h"
#include "sense.h"
#include "execute.h"

#define BUTTONS_DEVNAME "/dev/buttons0"

static int g_power_key_fd = -1;

static bool
power_button_pressed(int fd)
{
  uint8_t button_flags;
  bool pressed;
  ssize_t ret;

  ret = read (fd, &button_flags, sizeof(button_flags));
  if (ret != sizeof(button_flags))
    return OK;

  pressed = !!(button_flags & BOARD_BUTTON_POWERKEY_BIT);

  eng_dbg ("POWER KEY: %s\n", pressed ? "PRESSED!" : "Not pressed.");

  return pressed;
}

static int
button_handler (const struct pollfd * const pfd, void * const arg)
{
  struct ts_cause *cause = arg;

  cause->dyn.sense_value.value.valueint32 = power_button_pressed(cause->dyn.fd);

  (void)handle_cause_event (cause, NULL);

  return OK;
}

int
sense_power_button_pressed_irq_init(struct ts_cause *cause)
{
  int ret;

  cause->dyn.fd = open (BUTTONS_DEVNAME, O_RDONLY);
  if (cause->dyn.fd < 0)
    {
      /* Could not open button input. */

      return ERROR;
    }

  /* Register button poll callback. */

  eng_dbg ("registering button callback fd: %d\n", cause->dyn.fd);

  ret = ts_core_fd_register (cause->dyn.fd, POLLIN, &button_handler, cause);
  DEBUGASSERT(ret == OK);

  return OK;
}

int
sense_power_button_pressed_active_init(struct ts_cause *cause)
{
  g_power_key_fd = open (BUTTONS_DEVNAME, O_RDONLY);
  if (g_power_key_fd < 0)
    {
      return ERROR;
    }

  return OK;
}

int
sense_power_button_pressed_active_uninit(struct ts_cause *cause)
{
  close(g_power_key_fd);
  return OK;
}

int
sense_power_button_pressed_active_read(struct ts_cause *cause)
{
  cause->dyn.sense_value.value.valueint32 = power_button_pressed(g_power_key_fd);

  (void)handle_cause_event (cause, NULL);

  return OK;
}
