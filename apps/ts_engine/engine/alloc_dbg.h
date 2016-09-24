/****************************************************************************
 * apps/ts_engine/engine/alloc_dbg.h
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

#ifndef __APPS_TS_ENGINE_ENGINE_ALLOC_DBG_H__
#define __APPS_TS_ENGINE_ENGINE_ALLOC_DBG_H__

#if 0
#define malloc(x) alloc_dbg_alloc((x), __FILE__, __LINE__, false);
#define calloc(x,y) alloc_dbg_alloc(((x)*(y)), __FILE__, __LINE__, true);
#define zalloc(x) alloc_dbg_alloc((x), __FILE__, __LINE__, true);
#define realloc(ptr,x) alloc_dbg_realloc((ptr), (x), __FILE__, __LINE__);
#define strdup(x) alloc_dbg_strdup((x), __FILE__, __LINE__);
#define asprintf(ptr, fmt, ...) alloc_dbg_asprintf(__FILE__, __LINE__, (ptr), (fmt), __VA_ARGS__)

#define free(x) alloc_dbg_free((x), __FILE__, __LINE__)

void alloc_dbg_start(void);
void *alloc_dbg_alloc(size_t size, char *file, int line, bool zero);
void *alloc_dbg_realloc(void *p, size_t newsize, char *file, int line);
char *alloc_dbg_strdup(char *str, char *file, int line);
int alloc_dbg_asprintf(char *file, int line, FAR char **ptr, FAR const char *fmt, ...);
void alloc_dbg_free(void *p, char *file, int line);
void alloc_dbg_stop(void);
#endif

#endif
