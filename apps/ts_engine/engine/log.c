/****************************************************************************
 * apps/ts_engine/engine/log.c
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

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stddef.h>
#include <sys/stat.h>

#include <apps/thingsee/modules/ts_emmc.h>
#include <apps/netutils/cJSON.h>

#include "eng_dbg.h"
#include "log.h"
#include "parse.h"
#include "execute.h"
#include "util.h"
#include "sense.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define SENSE_NAME_MAX_LEN      32

#define ENTRIES_PER_REQUEST     20
#define CHARS_IN_ONE_SEND       2048
#define RETRY_DELAY             30
#define BAILOUT_ERROUR_COUNT    10

struct send_log;

struct send_log
{
  int fds[NUMBER_OF_LOGS];
  struct send_log **handle;
  struct url *url;
  int entries_per_request;
  off_t offset;
  int errcount;
};

static struct send_log g_send_log =
  {
  .fds = { -1, -1, -1 }
  };

static const char *g_filenames_str[NUMBER_OF_LOGS] = {
    [LOG_EVENTS] = EVENT_LOG_ABS_FILENAME,
    [LOG_CAUSES] = CAUSE_LOG_ABS_FILENAME,
    [LOG_SENDS]  = SEND_LOG_ABS_FILENAME
};

static void free_payload(struct ts_payload *payload)
{
  int i;

  if (payload->state.pId)
    {
      free((void *) payload->state.pId);
    }

  for (i = 0; i < payload->number_of_senses; i++)
    {
      free((void *) payload->senses[i].name);
      if (payload->senses[i].value.valuetype == VALUEARRAY)
        {
          free(payload->senses[i].value.valuearray.items);
        }
    }

  free(payload);
}

static char *serialize_payload(struct ts_payload *payload)
{
  char valuebuf[VALUE_STR_MAX_LEN];
  char *entry;
  int i;
  size_t buflen = 512;
  int n = 0;
  int saved;
  void *realloc_tmp;

  entry = malloc(buflen);
  if (!entry)
    {
      eng_dbg("malloc %d failed\n", buflen);
      return NULL;
    }

  n += snprintf(&entry[n], buflen - n, "%s,%d,%d,%d,%d,%d", payload->state.pId,
                payload->state.puId, payload->state.stId, payload->state.evId,
                payload->state.ts.tv_sec, payload->state.ts.tv_nsec);

  for (i = 0; i < payload->number_of_senses; i++)
    {
      __value_serialize(valuebuf, VALUE_STR_MAX_LEN, &payload->senses[i].value);

      saved = n;

      retry:

      n += snprintf(&entry[n], buflen - n, ";%s,0x%08x,%s,%d,%d",
                    payload->senses[i].name, payload->senses[i].sId,
                    valuebuf,
                    payload->senses[i].ts.tv_sec,
                    payload->senses[i].ts.tv_nsec);

      if (n == (buflen - 1))
        {
          buflen += 64;

          realloc_tmp = realloc(entry, buflen);
          if (!realloc_tmp)
            {
              eng_dbg("realloc %d failed\n", buflen);
              free(entry);
              return NULL;
            }
          entry = realloc_tmp;

          n = saved;
          goto retry;
        }
    }

  if (n + 2 >= buflen)
    {
      realloc_tmp = realloc(entry, buflen);
      if (!realloc_tmp)
        {
          eng_dbg("realloc %d failed\n", buflen);
          free(entry);
          return NULL;
        }
      entry = realloc_tmp;
    }

  entry[n++] = '\n';
  entry[n++] = '\0';

  return entry;
}

static struct ts_payload *deserialize_payload(struct send_log *send_log,
                                              char *str)
{
  int ret;
  char *state_info;
  char *cause;
  static char id_buf[32];
  static char value_buf[32];
  struct ts_payload *payload;
  int causes = 0;
  void *realloc_tmp;
  char *saveptr;

  state_info = strtok_r(str, ";", &saveptr);
  if (!state_info)
    {
      state_info = str;
    }

  payload = malloc(sizeof(*payload));
  if (!payload)
    {
      eng_dbg("malloc failed\n");
      return NULL;
    }

  memset(payload, 0, sizeof(*payload));

  ret = sscanf(state_info, "%s,%d,%d,%d,%d,%d", id_buf, &payload->state.puId,
               &payload->state.stId, &payload->state.evId,
               &payload->state.ts.tv_sec, &payload->state.ts.tv_nsec);
  if (ret != 6)
    {
      eng_dbg("state parse failed: %d\n", ret);
      goto errout;
    }

  payload->state.pId = strdup(id_buf);
  if (!payload->state.pId)
    {
      eng_dbg("strdup failed\n");
      goto errout;
    }

  cause = strtok_r(NULL, ";", &saveptr);

  while (cause)
    {
      realloc_tmp = realloc(
          payload,
          sizeof(*payload) + (causes + 1) * sizeof(struct ts_sense_value));
      if (!realloc_tmp)
        {
          eng_dbg("realloc failed\n");
          goto errout;
        }
      payload = realloc_tmp;

      payload->senses[causes].name = malloc(SENSE_NAME_MAX_LEN);
      if (!payload->senses[causes].name)
        {
          eng_dbg("malloc for name failed\n");
          goto errout;
        }

      payload->number_of_senses++;

      ret = sscanf(cause, "%s,%s,%s,%d,%d", payload->senses[causes].name,
                   id_buf, value_buf,
                   &payload->senses[causes].ts.tv_sec,
                   &payload->senses[causes].ts.tv_nsec);
      if (ret != 5)
        {
          eng_dbg("cause %d parse failed\n", causes);eng_dbg("bad cause was: %s\n", cause);
          goto errout;
        }

      payload->senses[causes].sId = strtol(id_buf, NULL, 16);

      ret = __value_deserialize(value_buf, &payload->senses[causes].value);
      if (ret < 0)
        {
          eng_dbg("__value_deserialize failed\n");
          goto errout;
        }

      cause = strtok_r(NULL, ";", &saveptr);
      causes++;
    }

  return payload;

  errout:

  free_payload(payload);

  return NULL;
}

static int readline(int fd, char **buf)
{
  int n = 0;
  int len = 64;
  char *ptr;
  int ret;
  void *realloc_tmp;

  *buf = malloc(len);
  if (!*buf)
    {
      eng_dbg("malloc failed\n");
      return ERROR;
    }

  ptr = *buf;

  do
    {
      ret = read(fd, &ptr[n], 1);
      if (ret < 0)
        {
          eng_dbg("read failed\n");
          free(*buf);
          return ERROR;
        }

      n += ret;

      if (ret == 0 || ptr[n - 1] == '\n')
        {
          break;
        }

      if (n + 2 > len)
        {
          len += 64;
          realloc_tmp = realloc(*buf, len);
          if (!realloc_tmp)
            {
              eng_dbg("realloc %d failed\n", len);
              free(*buf);
              return ERROR;
            }
          *buf = realloc_tmp;

          ptr = *buf;
        }
    } while (true);

  if (n == 0)
    {
      free(*buf);
    }
  else
    {
      ptr[n] = '\0';
    }

  return n;
}

static void finish(struct send_log *send_log, bool remove_send_file)
{
  int ret;

  close(send_log->fds[LOG_SENDS]);
  send_log->fds[LOG_SENDS] = -1;
  *(send_log->handle) = NULL;

  if (!remove_send_file)
    {
      return;
    }

  eng_dbg("Done with sending: removing %s\n", g_filenames_str[LOG_SENDS]);

  ret = unlink(g_filenames_str[LOG_SENDS]);
  if (ret < 0)
    {
      eng_dbg("unlink %s failed\n", g_filenames_str[LOG_SENDS]);
    }
}

static int retry_timer_cb(const int timer_id, void * const priv)
{
  return __ts_engine_log_process(priv, true);
}

static void retry_with_delay(struct send_log *send_log, int delay)
{
  int id;

  id = ts_core_timer_setup(TS_TIMER_TYPE_TIMEOUT, delay, retry_timer_cb,
                           send_log);
  if (id < 0)
    {
      eng_dbg("ts_core_timer_setup failed\n");
      finish(send_log, false);
    }
}

int __ts_engine_log_process(void * const priv, bool retry)
{
  struct send_log *send_log = priv;
  char *line;
  int ret;
  int i, j;
  struct ts_payload *payloads[ENTRIES_PER_REQUEST];
  int len = 0;

  if (send_log->fds[LOG_SENDS] == -1)
    {
      eng_dbg("not sending\n");
      return ERROR;
    }

  if (retry)
    {
      lseek(send_log->fds[LOG_SENDS], send_log->offset, SEEK_SET);

      send_log->errcount++;
      if (send_log->errcount >= BAILOUT_ERROUR_COUNT)
        {
          eng_dbg("Too many fails, ending send...\n");
          finish(send_log, false);
          return ERROR;
        }
    }
  else
    {
      send_log->offset = lseek(send_log->fds[LOG_SENDS], 0, SEEK_CUR);
      send_log->errcount = 0;
    }

  eng_dbg("file: %s offset: %d\n", g_filenames_str[LOG_SENDS], send_log->offset);

  for (i = 0; i < send_log->entries_per_request; i++)
    {
      ret = readline(send_log->fds[LOG_SENDS], &line);
      if (ret < 0)
        {
          eng_dbg("readline failed\n");
          finish(send_log, false);
          return ERROR;
        }

      if (ret == 0)
        {
          if (i == 0)
            {
              finish(send_log, true);
              return OK;
            }
          break; /* send the last items */
        }

      len += ret;
      if (len > CHARS_IN_ONE_SEND)
        {
          free(line);

          if (i == 0)
            {
              eng_dbg("too long line, skipping\n");
              i--;
              continue;
            }
          break;
        }

      payloads[i] = deserialize_payload(send_log, line);

      free(line);

      if (!payloads[i])
        {
          eng_dbg("deserialize_payload failed\n");
          i--;
        }
    }

  if (i > 1)
    {
      ret = __ts_engine_multisend(payloads, i, send_log->url, send_log);
    }
  else
    {
      ret = __ts_engine_send(payloads[0], send_log->url, send_log);
    }

  if (ret < 0)
    {
      eng_dbg("__ts_engine_[multi]send failed\n");
      retry_with_delay(send_log, RETRY_DELAY);
    }

  for (j = 0; j < i; j++)
    {
      free_payload(payloads[j]);
    }

  return OK;
}

static void append_log(struct send_log *send_log, enum logtypes type)
{
  struct stat stats;
  int ret;

  if (send_log->fds[type] != -1)
    {
      close(send_log->fds[type]);
      send_log->fds[type] = -1;
    }

  ret = stat(g_filenames_str[type], &stats);
  if (ret < 0)
    {
      return;
    }

  ret = __ts_engine_util_cp(g_filenames_str[type], g_filenames_str[LOG_SENDS]);
  if (ret < 0)
    {
      eng_dbg("__ts_engine_util_cp %s to %s failed\n", g_filenames_str[type], g_filenames_str[LOG_SENDS]);
    }
  else
    {
      eng_dbg("copied %s to %s\n", g_filenames_str[type], g_filenames_str[LOG_SENDS]);

      ret = unlink(g_filenames_str[type]);
      if (ret < 0)
        {
          eng_dbg("unlink %s failed\n", g_filenames_str[type]);
        }
      else
        {
          eng_dbg("removed %s\n", g_filenames_str[type]);
        }
    }
}

bool __ts_engine_log_have_logs(void)
{
  struct stat stats;
  int ret;
  int i;

  for (i = 0; i < NUMBER_OF_LOGS; i++)
    {
      if (g_send_log.fds[i] >= 0)
        {
          /* Log open => so something have been written to it. */

          eng_dbg("log file: %s, open\n", g_filenames_str[i]);

          return true;
        }

      ret = stat(g_filenames_str[i], &stats);
      if (ret >= 0 && stats.st_size > 0)
        {
          eng_dbg("log file: %s, size: %ld\n", g_filenames_str[i],
                  (long)stats.st_size);

          return true;
        }
    }

  return false;
}

int __ts_engine_log_start(struct send_log **handle, bool multisend,
                          struct url * const url)
{
  struct send_log *send_log = &g_send_log;
  struct stat stats;
  int ret;

  append_log(send_log, LOG_EVENTS);
  append_log(send_log, LOG_CAUSES);

  if (send_log->fds[LOG_SENDS] != -1)
    {
      eng_dbg("already sending\n");
      return OK;
    }

  ret = stat(g_filenames_str[LOG_SENDS], &stats);
  if (ret < 0)
    {
      eng_dbg("no logs to send\n");
      return OK;
    }

  send_log->fds[LOG_SENDS] = open(g_filenames_str[LOG_SENDS], O_RDONLY);
  if (send_log->fds[LOG_SENDS] < 0)
    {
      eng_dbg("open %s failed\n", g_filenames_str[LOG_SENDS]);
      return ERROR;
    }

  send_log->url = url;
  send_log->handle = handle;
  *handle = send_log;

  if (multisend)
    {
      send_log->entries_per_request = ENTRIES_PER_REQUEST;
    }
  else
    {
      send_log->entries_per_request = 1;
    }

  eng_dbg("start sending %s\n", g_filenames_str[LOG_SENDS]);

  return __ts_engine_log_process(send_log, false);
}

int __ts_engine_log_stop(struct send_log *handle)
{
  struct send_log *send_log = handle;

  if (send_log->fds[LOG_SENDS] == -1)
    {
      eng_dbg("send_log not active\n");
      return ERROR;
    }

  finish(send_log, false);

  return OK;
}

void __ts_engine_log_payload(struct ts_payload *payload,
                             enum logtypes type)
{
  char *entry;
  int ret;

  entry = serialize_payload(payload);
  if (!entry)
    {
      eng_dbg("serialize_payload failed\n");
      return;
    }

  eng_dbg("%s", entry);

  if (g_send_log.fds[type] == -1)
    {
      g_send_log.fds[type] = open(g_filenames_str[type], O_WRONLY | O_CREAT | O_APPEND);
      if (g_send_log.fds[type] < 0)
        {
          eng_dbg("open %s failed\n", g_filenames_str[type]);
          goto out;
        }
    }

  ret = __ts_engine_full_write(g_send_log.fds[type], entry, strlen(entry));
  if (ret < 0)
    {
      eng_dbg ("__ts_engine_full_write failed\n");
      goto out;
    }

out:

  free(entry);

  /* keep log open */
}

void __ts_engine_log_close_all(void)
{
  int i;

  for (i = 0; i < NUMBER_OF_LOGS; i++)
    {
      if (g_send_log.fds[i] >= 0)
        {
          close(g_send_log.fds[i]);
          g_send_log.fds[i] = -1;
        }
    }
}

void __ts_engine_log_remove(void)
{
  struct stat stats;
  int i;
  int ret;

  for (i = 0; i < ARRAY_SIZE(g_filenames_str); i++)
    {
      ret = stat(g_filenames_str[i], &stats);
      if (ret < 0)
        {
          continue;
        }

      ret = unlink(g_filenames_str[i]);
      if (ret < 0)
        {
          eng_dbg("unlink %s failed\n", g_filenames_str[i]);
        }
      else
        {
          eng_dbg("removed %s\n", g_filenames_str[i]);
        }
    }
}
