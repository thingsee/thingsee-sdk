/****************************************************************************
 * include/example_app_dbg.h
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

#ifndef __EXAMPLE_APP_INCLUDE_DBG_H__
#define __EXAMPLE_APP_INCLUDE_DBG_H__

#include <nuttx/config.h>
#include <debug.h>

#undef exapp_dbg
#undef exapp_lldbg
#ifdef CONFIG_SDK_EXAMPLE_APP_DBG
#  define exapp_dbg(x, ...)	dbg(x, ##__VA_ARGS__)
#  define exapp_lldbg(x, ...)	lldbg(x, ##__VA_ARGS__)
#else
#  define exapp_dbg(x, ...) ((void)0)
#  define exapp_lldbg(x, ...) ((void)0)
#endif

#endif /* __EXAMPLE_APP_INCLUDE_DBG_H__ */
