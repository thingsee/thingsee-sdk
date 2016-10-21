/****************************************************************************
 * apps/thingsee/engine/sense_sim.c
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
#include <nuttx/arch.h>

#include <time.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include <debug.h>
#include <assert.h>

#include <apps/netutils/cJSON.h>
#include <apps/thingsee/ts_core.h>

#include "eng_dbg.h"
#include "parse.h"
#include "sense.h"
#include "execute.h"

#define PROFILE_JSON_FILENAME		"profile.json"
#define VALUES_JSON_FILENAME		"values.json"

static cJSON *json = NULL;

static int
get_input_values_for_sense(struct ts_sense *sense)
{
  char *file_str;
  char sense_id[32];
  size_t len;
  cJSON *sense_json;
  int i;
  double *mem;

  if (!json)
    {
      file_str = up_mmap (VALUES_JSON_FILENAME, &len);
      if (!file_str)
	{
	  eng_dbg ("%s mmap failed\n", VALUES_JSON_FILENAME);
	  return ERROR;
	}

      json = cJSON_Parse (file_str);
      if (!json)
	{
	  eng_dbg ("%s json parse failed\n", VALUES_JSON_FILENAME);
	  return ERROR;
	}
    }

  snprintf(sense_id, 32, "0x%08x", sense->sense_value.sense_id);

  sense_json = cJSON_GetObjectItem(json, sense_id);
  if (!sense_json)
    {
      eng_dbg ("no values for %s\n", sense_id);
      return ERROR;
    }

  sense->input.max = cJSON_GetArraySize(sense_json);

  mem = (double *)malloc (sense->input.max * sizeof(double));
  if (!mem)
    {
      eng_dbg ("malloc failed\n");
      return ERROR;
    }

  sense->input.data = mem;

  for (i = 0; i < sense->input.max; i++)
    {
      *mem++ = cJSON_double(cJSON_GetArrayItem(sense_json, i));
    }

  return OK;
}

static double
get_fake_sensor_value (struct ts_sense *sense)
{
  double value;
  int ret;

  if (!sense->input.data)
    {
      ret = get_input_values_for_sense(sense);
      if (ret != OK)
	{
	  eng_dbg ("get_input_values_for_sense failed\n");
	  return -1.0;
	}
    }

  value = sense->input.data[sense->input.i];

  sense->input.i++;

  if (sense->input.i > sense->input.max)
    {
      sense->input.i = 0;
    }

  return value;
}

static int
fake_value_timer_callback (const int timer_id, void * const priv)
{
  struct ts_sense *sense = (struct ts_sense *) priv;

  eng_dbg ("sense_id: %d timer_id %d\n", sense->sense_value.sense_id, timer_id);

  ts_core_timer_stop(timer_id);

  switch (sense->sense_value.value.valuetype)
    {
    case VALUEDOUBLE:
      sense->sense_value.value.valuedouble = get_fake_sensor_value (sense);
      break;
    case VALUEINT16:
      sense->sense_value.value.valueint16 = (int16_t) get_fake_sensor_value (sense);
      break;
    case VALUEINT32:
      sense->sense_value.value.valueint32 = (int32_t) get_fake_sensor_value (sense);
      break;
    case VALUESTRING:
      sense->sense_value.value.valuestring = "fake";
      break;
    }

  handle_sense_event (sense);

  return 0;
}

int
engine_sense_get_value_sim (struct ts_sense *sense)
{
  int ret;

  ret = ts_core_timer_setup (TS_TIMER_TYPE_TIMEOUT, 50,
			     fake_value_timer_callback, sense);
  if (ret < 0)
    {
      eng_dbg ("ts_core_timer_setup failed\n");
    }

  return ret;
}

char *
engine_sense_sim_get_profile (size_t *length)
{
  char *profile_str;
  size_t len;

  eng_dbg ("mmap profile input file\n");

  profile_str = up_mmap (PROFILE_JSON_FILENAME, &len);
  if (!profile_str)
    {
      eng_dbg ("%s mmap failed\n", PROFILE_JSON_FILENAME);
      return NULL;
    }

  *length = len;

  return profile_str;
}

