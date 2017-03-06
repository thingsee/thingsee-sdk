/****************************************************************************
 * apps/ts_engine/engine/util.c
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
#include <nuttx/arch.h>

#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <limits.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>

#ifndef CONFIG_ARCH_SIM
#include <apps/ts_engine/watchdog.h>
#endif

#include "eng_dbg.h"
#include "parse.h"
#include "util.h"
#include "eng_error.h"

#define EEPROM_MAGIC                            "TSENGINE"
#define EEPROM_MAGIC_SIZE                       8

#define SDCARD_STRIP_NULL

static const char g_eeprom_magic[] = EEPROM_MAGIC;

struct ts_eeprom
{
  char magic[EEPROM_MAGIC_SIZE];
  char data[];
};

#ifndef CONFIG_ARCH_SIM
static int squeezestr(char * const str, size_t len)
{
  int i, j;

  for (i = 0, j = 0; i < len && str[i] != '\0'; i++)
    {
      if (str[i] != ' ' && str[i] != '\n' && str[i] != '\r')
        {
          str[j++] = str[i];
        }
    }

  str[j] = '\0';

  return j;
}

bool __ts_engine_sdcard_inserted(void)
{
  DIR *dir;
  bool inserted;

  dir = opendir(TS_EMMC_MOUNT_PATH);
  if (dir)
    {
      inserted = true;
    }
  else
    {
      inserted = false;
    }

  closedir(dir);

  return inserted;
}

cJSON *__ts_engine_sdcard_read_json(const char * const filename)
{
  int fd;
  cJSON *obj;

  eng_dbg ("filename: %s\n", filename);

  fd = open(filename, O_RDONLY);
  if (fd < 0)
    {
      perror("open");
      return NULL;
    }

  obj = cJSON_Parse_fd(fd, INT_MAX, NULL);
  close(fd);

  return obj;
}

const char *__ts_engine_sdcard_read(const char * const filename)
{
  int fd;
  size_t len = 0;
  int ret;
  char buf[64];
  char *data = NULL;

  eng_dbg ("filename: %s\n", filename);

  fd = open(filename, O_RDONLY);
  if (fd < 0)
    {
      perror("open");
      return NULL;
    }

  /* currently only way to determine the file size? */

  do
    {
      ret = read(fd, buf, 64);
      if (ret < 0)
        {
          perror("read");
          goto err;
        }

      len += ret;
    } while (ret > 0);

#ifdef SDCARD_STRIP_NULL
  data = malloc(len + 1); /* add space for trailing NULL */
#else
  data = malloc(len);
#endif
  if (!data)
    {
      perror("malloc");
      goto err;
    }

  ret = lseek(fd, SEEK_SET, 0);
  if (ret < 0)
    {
      perror("lseek");
      goto err2;
    }

  ret = __ts_engine_full_read(fd, data, len);
  if (ret < 0)
    {
      eng_dbg ("full_read failed\n");
      goto err2;
    }

  close(fd);

#ifdef SDCARD_STRIP_NULL
  data[len] = '\0';
#endif

  return data;

  err2: free(data);
  err: close(fd);

  return NULL;
}

int __ts_engine_sdcard_write(const char * const filename,
    const char * const data, size_t len, bool append)
{
  int fd;
  int ret;
  int oflags;

  eng_dbg ("filename: %s\n", filename);

  oflags = O_WRONLY | O_CREAT;

  if (append)
    {
      oflags |= O_APPEND;
    }
  else
    {
      oflags |= O_TRUNC;
    }

  fd = open(filename, oflags);
  if (fd < 0)
    {
      perror("open");
      return -TS_ENGINE_ERROR_SYSTEM;
    }

#ifdef SDCARD_STRIP_NULL
  len--; /* strip trailing NULL */
#endif

  ret = __ts_engine_full_write(fd, data, len);
  if (ret < 0)
    {
      eng_dbg ("full_write failed\n");
      return ret;
    }

  close(fd);

  return OK;
}

const char *__ts_engine_eeprom_read(enum board_eeprom_section_e section)
{
  const struct ts_eeprom *eeprom;
  size_t len;

  eeprom = board_eeprom_get_section(section, &len);

  if (memcmp(eeprom->magic, g_eeprom_magic, EEPROM_MAGIC_SIZE) != 0)
    {
      return NULL;
    }

  return eeprom->data;
}

int __ts_engine_eeprom_write(enum board_eeprom_section_e section,
    char * const data, size_t len)
{
  const struct ts_eeprom *eeprom;
  size_t size;
  int ret;

  len = squeezestr(data, len);

  eeprom = board_eeprom_get_section(section, &size);

  if (memcmp(eeprom->data, data, len) == 0)
    {
      eng_dbg ("Data not changing, do not write\n");
      return OK;
    }

  eng_dbg ("write data\n");

  ret = board_eeprom_write_section(section, offsetof(struct ts_eeprom, data),
      data, len);
  if (ret < 0)
    {
      eng_dbg ("board_eeprom_write_section failed\n");
      return -TS_ENGINE_ERROR_EEPROM;
    }

  if (memcmp(eeprom->data, g_eeprom_magic, EEPROM_MAGIC_SIZE) == 0)
    {
      eng_dbg ("magic already written\n");
      return OK;
    }

  eng_dbg ("write magic\n");

  ret = board_eeprom_write_section(section, offsetof(struct ts_eeprom, magic),
      g_eeprom_magic, EEPROM_MAGIC_SIZE);
  if (ret < 0)
    {
      eng_dbg ("board_eeprom_write_section failed\n");
      return -TS_ENGINE_ERROR_EEPROM;
    }

  return OK;
}
#endif

bool __ts_engine_find_threshold(struct ts_cause *cause,
    enum ts_threshold_t thr_type, double *value, bool *valuebool)
{
  struct ts_threshold *threshold;

  threshold = (struct ts_threshold *) sq_peek(&cause->conf.thresholds);

  while (threshold)
    {
      if (threshold->conf.type == thr_type)
        {
          switch (thr_type)
          {
            case isAny:
              *valuebool = threshold->conf.value.valuebool;
              break;
            case isOneOf:
              /* TODO: all items ? */
              *value = threshold->conf.value.valuearray.items[0].valuedouble;
              break;
            default:
              *value = threshold->conf.value.valuedouble;
              break;
          }

          return true;
        }

      threshold = (struct ts_threshold *) sq_next(&threshold->entry);
    }

  return false;
}

int __ts_engine_full_read(int fd, const void *buf, size_t count)
{
  int ret;
  int done = 0;
  char *addr = (char *) buf;

  while ((count - done) > 0)
    {
      ret = read(fd, addr + done, count - done);
      if (ret < 0)
        {
          perror("read");
          return -TS_ENGINE_ERROR_SYSTEM;
        }

      done += ret;
    }

  return count;
}

int __ts_engine_full_write(int fd, const void *buf, size_t count)
{
  int ret;
  int done = 0;
  char *addr = (char *) buf;

  while ((count - done) > 0)
    {
      ret = write(fd, addr + done, (count - done > 512 ? 512 : count - done));
      if (ret < 0)
        {
          perror("write");
          return -TS_ENGINE_ERROR_SYSTEM;
        }

      done += ret;
    }

  return count;
}

static const char * const __ts_engine_util_get_filename(
    const char * const dirname, const char * const basename, bool first)
{
  struct dirent *dirent;
  DIR *dir;
  size_t basename_len = strlen(basename);
  int index;
  int index_max = 0;
  char *filename;
  int ret;
  bool found = false;

  dir = opendir(dirname);
  if (dir == NULL)
    {
      printf("Cannot open directory '%s'\n", dirname);
      return NULL;
    }

  while ((dirent = readdir(dir)) != NULL)
    {
      if (strlen(dirent->d_name) <= basename_len)
        {
          continue;
        }

      if (dirent->d_name[basename_len] != '.')
        {
          continue;
        }

      found = true;

      index = atoi(&dirent->d_name[basename_len + 1]);

      if (first)
        {
          index_max = index;
          break;
        }

      index++;

      if (index > index_max)
        {
          index_max = index;
        }
    }

  if (!found && first)
    {
      return NULL;
    }

  ret = asprintf(&filename, "%s/%s.%d", dirname, basename, index_max);
  if (ret < 0)
    {
      eng_dbg("asprintf failed\n");
      return NULL;
    }

  closedir(dir);

  return filename;
}

const char * const __ts_engine_util_find_log(const char * const dirname,
    const char * const basename)
{
  return __ts_engine_util_get_filename(dirname, basename, true);
}

const char * const __ts_engine_util_gen_next_log(const char * const dirname,
    const char * const basename)
{
  return __ts_engine_util_get_filename(dirname, basename, false);
}

int __ts_engine_util_cp(const char * const src, const char * const dst)
{
  int fdsrc;
  int fddst;
  char *buf;
  int n;
  int ret;
  int oflags;
  int count;

  buf = malloc(1024);
  if (!buf)
    {
      eng_dbg("malloc failed\n");
      return ERROR;
    }

  fdsrc = open(src, O_RDONLY);
  if (fdsrc < 0)
    {
      eng_dbg("open %s failed\n", src);
      ret = ERROR;
      goto errout_free;
    }

  oflags = O_WRONLY | O_CREAT | O_APPEND;

  fddst = open(dst, oflags);
  if (fddst < 0)
    {
      eng_dbg("open %s failed\n", dst);
      ret = ERROR;
      goto errout_close;
    }

  count = 0;
  do
    {
      n = read(fdsrc, buf, sizeof(buf));
      if (n < 0)
        {
          eng_dbg("read failed\n");
          ret = ERROR;
          goto errout_close_both;
        }

#ifndef CONFIG_ARCH_SIM
      /* Kick watchdog every 50kb */

      if (count % 50 == 0)
        {
          ts_watchdog_kick();
        }
#endif
      count++; /* For debugging */

      ret = __ts_engine_full_write(fddst, buf, n);
      if (ret < 0)
        {
          eng_dbg("__ts_engine_full_write failed\n");
          ret = ERROR;
          goto errout_close_both;
        }
    } while (n > 0);

  errout_close_both: close(fddst);
  errout_close: close(fdsrc);
  errout_free: free(buf);

  return ret;
}
