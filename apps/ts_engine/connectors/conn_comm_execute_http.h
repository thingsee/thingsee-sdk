/****************************************************************************
 * apps/ts_engine/connectors/conn_comm_execute_http.h
 *
 * Copyright (C) 2015 Haltian Ltd. All rights reserved.
 * Authors: Timo Voutilainen <timo.voutilainen@haltian.com>
 *          Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#ifndef __APPS_TS_ENGINE_CONNECTORS_CONN_COMM_EXECUTE_HTTP_H
#define __APPS_TS_ENGINE_CONNECTORS_CONN_COMM_EXECUTE_HTTP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <arpa/inet.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HTTP_DEFAULT_USER_AGENT            "tsone/0.3"
#define HTTP_DEFAULT_PORT                  80

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct con_str;
struct conn_network_task_s;
struct conn_link_s;
struct conn_content_stream_s;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int conn_execute_http_request(struct con_str *con,
                              struct conn_network_task_s *task,
                              int *pstatus_code,
                              struct conn_link_s *conn_link,
                              char *hdr,
                              size_t hdrlen,
                              char *pdata,
                              size_t datalen,
                              struct conn_content_stream_s *content);

#endif /* __APPS_TS_ENGINE_CONNECTORS_CONN_COMM_EXECUTE_HTTP_H */
