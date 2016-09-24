/****************************************************************************
 * modules/charger.c
 *
 * Copyright (C) 2016 Haltian Ltd.
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

#ifdef CONFIG_THINGSEE_CHARGER_MODULE
#  include <thingsee/charger/bq24251_module.h>
#endif

#include <example_app_dbg.h>
#include <example_app_main.h>
#include <example_app_charger.h>

int exapp_charger_initialize(void)
{
#ifdef CONFIG_THINGSEE_CHARGER_MODULE
  int ret;

  exapp_dbg("Initializing charging module\n");

  ret = bq24251_init_module(
           &(const struct bq24251_chgr_cbks_s) {
              .charger_event = NULL,
              .notify_usb_connect = NULL
           } );
  if (ret < 0)
    {
      exapp_dbg("bq24251_init_module failed\n");
      return ERROR;
    }
#endif

  return OK;
}
