/****************************************************************************
 * apps/ts_engine/engine/time_from_file.c
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

#include "time_from_file.h"

#include <nuttx/config.h>

#include <fcntl.h>
#include <sys/types.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdlib.h>

#include <apps/netutils/cJSON.h>
#include <apps/thingsee/modules/ts_emmc.h>

#include "eng_dbg.h"
#include "util.h"

#define TS_ENGINE_TIME_FILE TS_EMMC_MOUNT_PATH "/time.jsn"

int __ts_engine_time_update_from_file(void)
{
  int ret;
  const char *buf;
  cJSON *json_root;
  cJSON *json_unix;
  struct timespec ts;

  buf = __ts_engine_sdcard_read(TS_ENGINE_TIME_FILE);
  if (!buf)
    {
      eng_dbg("No time file\n");
      return OK;
    }

  ret = unlink(TS_ENGINE_TIME_FILE);
  if (ret < 0)
    {
      eng_dbg("unlink %s failed\n", TS_ENGINE_TIME_FILE);
    }

  json_root = cJSON_Parse(buf);

  free((void *)buf);

  if (!json_root)
    {
      eng_dbg("%s parse failed\n", TS_ENGINE_TIME_FILE);
      return ERROR;
    }

  json_unix = cJSON_GetObjectItem(json_root, "unix");
  if (!json_unix)
    {
      eng_dbg("unix object not found\n");
      cJSON_Delete(json_root);
      return ERROR;
    }

  ts.tv_sec = cJSON_int(json_unix);
  ts.tv_nsec = 0;

  eng_dbg("setting time to %u\n", ts.tv_sec);

  clock_settime(CLOCK_REALTIME, &ts);

  cJSON_Delete(json_root);

  return OK;
}
