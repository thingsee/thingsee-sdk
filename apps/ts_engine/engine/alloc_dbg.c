/****************************************************************************
 * apps/ts_engine/engine/alloc_dbg.c
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

#include <sys/types.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include <queue.h>

#include "eng_dbg.h"

#undef malloc
#undef zalloc
#undef calloc
#undef realloc
#undef free
#undef strdup

struct alloc_entry
{
  sq_entry_t entry;
  void *addr;
  size_t len;
  char *file;
  int line;
};

struct alloc
{
  sq_queue_t allocs;
  bool on;
};

static struct alloc g_alloc =
  {
  .on = false
  };

void alloc_dbg_start(void)
{
  sq_init(&g_alloc.allocs);
  g_alloc.on = true;
}

static void add_entry(void * const addr, size_t len, char *file, int line)
{
  struct alloc_entry *entry;

  entry = malloc(sizeof(*entry));
  if (!entry)
    {
      eng_dbg("malloc for entry failed\n");
      return;
    }

  entry->addr = addr;
  entry->len = len;
  entry->file = file;
  entry->line = line;

  sq_addlast(&entry->entry, &g_alloc.allocs);
}

void *alloc_dbg_alloc(size_t size, char *file, int line, bool zero)
{
  void *p;

  if (zero)
    {
      p = calloc(size, 1);
    }
  else
    {
      p = malloc(size);
    }

  if (size == 0)
    {
      eng_dbg("WARN: malloc(0) %s %d\n", file, line);
    }

  if (!g_alloc.on || !p)
    {
      return p;
    }

  add_entry(p, size, file, line);

  return p;
}

void *alloc_dbg_realloc(void *p, size_t newsize, char *file, int line)
{
  void *addr;
  struct alloc_entry *entry, *old;

  if (!g_alloc.on)
    {
      addr = realloc(p, newsize);
      goto out;
    }

  /* Check that old pointer is valid and get pointer to its entry. */

  old = NULL;
  if (p)
    {
      entry = (struct alloc_entry *) sq_peek(&g_alloc.allocs);

      while (entry)
        {
          if (entry->addr == p)
            {
              old = entry;
              break;
            }

          entry = (struct alloc_entry *) sq_next(&entry->entry);
        }

      if (!old)
        {
          eng_dbg("BUG: realloc(%p) %s %d not registered\n", (uintptr_t)p, file, line);
        }
    }

  addr = realloc(p, newsize);
  if (addr || newsize == 0)
    {
      /* Remove old entry, if any. */

      if (old)
        {
          sq_rem(&old->entry, &g_alloc.allocs);
          free(old);
        }
    }
  if (addr)
    {
      add_entry(addr, newsize, file, line);
    }

out:
  if (newsize == 0)
    {
      eng_dbg("WARN: realloc(p, 0) %s %d\n", file, line);
    }
  return addr;
}

char *alloc_dbg_strdup(char *str, char *file, int line)
{
  char *addr;

  if (!str)
    {
      eng_dbg("WARN: strdup(0) %s %d\n", file, line);
    }

  addr = strdup(str);

  if (!g_alloc.on || !addr)
    {
      return addr;
    }

  add_entry(addr, strlen(addr) + 1, file, line);

  return addr;
}

int alloc_dbg_asprintf(char *file, int line, FAR char **ptr, FAR const char *fmt, ...)
{
  int ret;
  int len = 0;
  va_list ap;

  va_start(ap, fmt);
  ret = avsprintf(ptr, fmt, ap);
  if (ret < 0)
    {
      return len;
    }
  len += ret;
  va_end(ap);

  if (!g_alloc.on || !len)
    {
      return len;
    }

  add_entry(*ptr, len, file, line);

  return len;
}

void alloc_dbg_free(void *addr, char *file, int line)
{
  struct alloc_entry *entry;

  if (!g_alloc.on)
    {
      free(addr);
      return;
    }

  entry = (struct alloc_entry *) sq_peek(&g_alloc.allocs);

  while (entry)
    {
      if (entry->addr == addr)
        {
          sq_rem(&entry->entry, &g_alloc.allocs);
          free(entry);
          free(addr);
          return;
        }

      entry = (struct alloc_entry *) sq_next(&entry->entry);
    }

  eng_dbg("BUG: free(%p) %s %d not registered\n", (uintptr_t)addr, file, line);
  free(addr); /* This corrupts heap if it is really a bogus pointer. */
}

void alloc_dbg_stop(void)
{
  struct alloc_entry *entry, *del;

  if (!g_alloc.on)
    {
      return;
    }

  g_alloc.on = false;

  entry = (struct alloc_entry *) sq_peek(&g_alloc.allocs);

  while (entry)
    {
      eng_dbg("unfreed: %p %u %s %d\n", entry->addr, entry->len, entry->file, entry->line);

      del = entry;
      entry = (struct alloc_entry *) sq_next(&entry->entry);
      free(del);
    }
}
