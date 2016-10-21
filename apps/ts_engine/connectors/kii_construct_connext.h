/****************************************************************************
 * apps/ts_engine/connectors/kii_construct_connext.h
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

#ifndef __APPS_TS_ENGINE_CONNECTORS_KII_CONSTRUCT_CONNEXT_H
#define __APPS_TS_ENGINE_CONNECTORS_KII_CONSTRUCT_CONNEXT_H

struct ts_context {
  con_str_t con;
  bool kii_waiting_access_token:1;
  bool kii_active:1;
  kii_cloud_params_s cloud_params;
  pthread_mutex_t mutex;
};

conn_workflow_context_s *kii_create_workflow_context(struct ts_payload *payload,
                                                     send_cb_t cb, const void *priv);
int kii_post_data_construct(conn_workflow_context_s *context,
                            char **outhdr, char **outdata);
struct conn_network_task_s *kii_post_data_process_stream(
    conn_workflow_context_s *context, int status_code, size_t content_len,
    char (*stream_getc)(void *priv), void *stream_priv) weak_function;
struct conn_network_task_s *kii_post_data_process(
    conn_workflow_context_s *context, int status_code, const char *content) weak_function;

#endif /* __APPS_TS_ENGINE_CONNECTORS_KII_CONSTRUCT_CONNEXT_H */
