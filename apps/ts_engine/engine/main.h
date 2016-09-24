/****************************************************************************
 * apps/ts_engine/engine/main.h
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

#ifndef __TS_ENGINE_MAIN_H__
#define __TS_ENGINE_MAIN_H__

#include <apps/ts_engine/ts_engine.h>

#define CONFIG_THINGSEE_ENGINE_ASSERT_COUNT_RESET_SECS (1 * 60)

struct ts_engine_app
{
  struct ts_engine *engine;
  int fdin;
  int fdout;
  bool charger_connected:1;
  bool mainloop_started:1;

  struct
  {
      backend_update_cb_t cb;
      bool profile_updated:1;
  } backend_update;

  struct send_log *handle;
};

extern pid_t g_engine_pid;

const char *
cloud_properties_read (bool *ref);

bool charger_connected(void);

#endif
