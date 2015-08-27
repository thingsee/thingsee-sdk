/****************************************************************************
 * apps/ts_engine/meshblu-connector/connector.h
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

#ifndef __APPS_TS_ENGINE_CONNECTORS_CONNECTOR_H
#define __APPS_TS_ENGINE_CONNECTORS_CONNECTOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <time.h>

#include <apps/thingsee/ts_core.h>

#include "engine/value.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ts_sense_value
{
  sense_id_t sId;
  const char *name;
  struct ts_value value;
  struct timespec ts;
};

struct ts_ids
{
  const char *pId;
  int puId;
  int stId;
  int evId;
  struct timespec ts;
};

struct ts_payload
{
  struct ts_ids state;
  int number_of_senses;
  struct ts_sense_value senses[];
};

struct url
{
    char *host;
    uint16_t port;
    char *api;
};

typedef bool
(*allow_deepsleep_t) (void);

typedef int
(*init_t) (const char * const connectors);

typedef int
(*uninit_t) (void);

typedef int
(*reg_cb_t) (const void *priv);

typedef int
(*reg_t) (const char *uuid, reg_cb_t cb,
	  const void *priv);

typedef int
(*unreg_cb_t) (const void *priv);

typedef int
(*unreg_t) (const char *uuid, unreg_cb_t cb, const void *priv);

typedef int
(*send_cb_t) (int result, const void *priv);

typedef int
(*send_t) (struct ts_payload *payload, send_cb_t cb, const void *priv);

typedef int
(*multisend_t) (struct ts_payload **payload, int number_of_payloads, send_cb_t cb,
    const void *priv);

typedef int
(*send_url_t) (struct ts_payload *payload, send_cb_t cb, const struct url * const url, const void *priv);

typedef int
(*multisend_url_t) (struct ts_payload **payload, int number_of_payloads,
    send_cb_t cb, const struct url * const url, const void *priv);

struct ts_connector
{
  uint32_t id;
  char *name;

  ts_deepsleep_hook_t allow_deepsleep;

  init_t init;
  uninit_t uninit;

  reg_t reg;
  unreg_t unreg;

  send_t send;
  multisend_t multisend;

  send_url_t send_url;
  multisend_url_t multisend_url;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct ts_connector *ts_connectors[];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t count_ts_engine_connectors(void);

#endif /* __APPS_TS_ENGINE_CONNECTORS_CONNECTOR_H */
