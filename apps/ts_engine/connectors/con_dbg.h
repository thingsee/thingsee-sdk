/****************************************************************************
 * apps/ts_engine/connectors/con_dbg.h
 *
 * Copyright (C) 2015 Haltian Ltd. All rights reserved.
 * Author: Pekka Ervasti <pekka.ervasti@haltian.com>
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
 ****************************************************************************/

#ifndef __APPS_TS_ENGINE_CONNECTORS_CON_DBG_H
#define __APPS_TS_ENGINE_CONNECTORS_CON_DBG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef con_dbg
#undef con_lldbg
#ifdef CONFIG_THINGSEE_CONNECTORS_DEBUG
#  define con_dbg(x, ...)       dbg(x, ##__VA_ARGS__)
#  define con_lldbg(x, ...)     lldbg(x, ##__VA_ARGS__)
#else
#  define con_dbg(x, ...)
#  define con_lldbg(x, ...)
#endif

#ifdef CONFIG_THINGSEE_CONNECTORS_PROTOCOL_DEBUG
#  define http_con_dbg(...)     con_dbg("<http> " __VA_ARGS__)
#else
#  define http_con_dbg(...)     ((void)0)
#endif

#ifdef CONFIG_THINGSEE_CONNECTORS_DEBUG
#  define con_dbg_save_pos()  con_dbg_push_location(__func__, __LINE__)
#else
#  define con_dbg_save_pos()  ((void)0)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* __APPS_TS_ENGINE_CONNECTORS_CON_DBG_H */
