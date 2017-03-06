/****************************************************************************
 * apps/ts_engine/engine/log.h
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

#ifndef __APPS_TS_ENGINE_ENGINE_LOG_H__
#define __APPS_TS_ENGINE_ENGINE_LOG_H__

#include "parse.h"

#define EVENT_LOG_FILENAME      "EVENTS.LOG"
#define CAUSE_LOG_FILENAME      "CAUSES.LOG"
#define SEND_LOG_FILENAME       "SEND.LOG"

#define LOG_DIRECTORY           TS_EMMC_MOUNT_PATH

#define EVENT_LOG_ABS_FILENAME  LOG_DIRECTORY "/" EVENT_LOG_FILENAME
#define CAUSE_LOG_ABS_FILENAME  LOG_DIRECTORY "/" CAUSE_LOG_FILENAME
#define SEND_LOG_ABS_FILENAME   LOG_DIRECTORY "/" SEND_LOG_FILENAME

enum logtypes
{
  LOG_EVENTS,
  LOG_CAUSES,
  LOG_SENDS,
  NUMBER_OF_LOGS
};

struct send_log;

bool __ts_engine_log_have_logs(void);
int __ts_engine_log_start(struct send_log **handle, bool multisend,
                          struct url * const url);
int __ts_engine_log_process(void * const priv, bool retry);
int __ts_engine_log_stop(struct send_log *handle);
void __ts_engine_log_payload(struct ts_payload *payload,
                             enum logtypes type);
void __ts_engine_log_remove(void);
void __ts_engine_log_close_all(void);

#endif
