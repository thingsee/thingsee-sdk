/****************************************************************************
 * apps/ts_engine/engine/debug.h
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

#ifndef __TS_ENGINE_DEBUG_H__
#define __TS_ENGINE_DEBUG_H__

#include <nuttx/config.h>
#include <debug.h>

#include "alloc_dbg.h"

#define ESC     0x1B

#define BLACK   0
#define RED     1
#define GREEN   2
#define YELLOW  3
#define BLUE    4
#define MAGENTA 5
#define CYAN    6
#define WHITE   7

#ifdef CONFIG_THINGSEE_UI
#  include "../../thingsee/ui/thingsee_ui.h" /* TODO */
#else
#  define thingsee_UI_sense_event(x, ...) ((void)0)
#endif

#undef eng_dbg
#undef eng_lldbg
#undef eng_dispdbg
#ifdef CONFIG_THINGSEE_ENGINE_DBG
#  define eng_dbg(x, ...)	dbg(x, ##__VA_ARGS__)
#  define eng_color_dbg(fc,bc,x, ...)	dbg("%c[38;5;%d48;5;%dm" x "%c[0m", ESC, fc, bc, ##__VA_ARGS__, ESC)
#  define eng_dispdbg(x, ...)	do { dbg(x "\n", ##__VA_ARGS__); thingsee_UI_sense_event(x, ##__VA_ARGS__); } while (0)
#  define eng_lldbg(x, ...)	lldbg(x, ##__VA_ARGS__)
#else
#  define eng_dbg(x, ...) ((void)0)
#  define eng_color_dbg(fc,bc,x, ...) ((void)0)
#  define eng_dispdbg(x, ...) thingsee_UI_sense_event(x, ##__VA_ARGS__)
#  define eng_lldbg(x, ...) ((void)0)
#endif

#endif
