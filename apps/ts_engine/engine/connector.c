/****************************************************************************
 * apps/thingsee/engine/connector.c
 *
 * Copyright (C) 2014-2016 Haltian Ltd. All rights reserved.
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
 *   Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <debug.h>
#include <string.h>
#include <assert.h>

#include <apps/thingsee/ts_core.h>
#include <apps/system/conman.h>

#include "eng_dbg.h"
#include "parse.h"
#include "cloud_property.h"
#include "main.h"
#include "execute.h"

#include "../connectors/connector.h"

#ifndef CONFIG_THINGSEE_ENGINE_CONNECTION_CUTOFF_TIME
/* TODO: This should be configurable in "cloud.jsn" !!! */
# define CONFIG_THINGSEE_ENGINE_CONNECTION_CUTOFF_TIME 30
#endif

struct ts_connector_state
{
  const struct ts_connector *con;
  uint32_t con_idx;
  int timerid;
  uint32_t conman_connid;
  bool requested:1;
};

struct ts_connector_state g_state =
  {
    .con = NULL,
    .con_idx = -1,
    .timerid = -1,
    .requested = false
  };

static int cancel_connection_cb(const int timer_id, void * const priv)
{
  struct ts_connector_state *state = priv;
  int ret = OK;

  if (!state->requested)
    {
      return OK;
    }

  if (timer_id == -1 && state->timerid >= 0)
    {
      ts_core_timer_stop(state->timerid);
    }

#ifdef CONFIG_SYSTEM_CONMAN
  struct conman_client_s conman_client;

  ret = conman_client_init(&conman_client);
  if (ret != OK)
    {
      eng_dbg("conman_client_init failed\n");
      ret = ERROR;
    }
  else
    {
      ret = conman_client_destroy_connection(&conman_client, state->conman_connid);
      if (ret != OK)
        {
          eng_dbg("conman_client_destroy_connection failed\n");
          ret = ERROR;
        }

      state->conman_connid = CONMAN_CONNID_CLEAR;
      conman_client_uninit(&conman_client);
    }
#endif

  state->requested = false;
  state->timerid = -1;
  return ret;
}


static int request_connection(void)
{
  int ret = OK;

  if (g_state.timerid >= 0)
    {
      ts_core_timer_stop(g_state.timerid);
    }

  g_state.timerid = ts_core_timer_setup(TS_TIMER_TYPE_TIMEOUT,
                                        CONFIG_THINGSEE_ENGINE_CONNECTION_CUTOFF_TIME * 1000,
                                        cancel_connection_cb, &g_state);

  if (g_state.requested)
    {
      return OK;
    }

#ifdef CONFIG_SYSTEM_CONMAN
  struct conman_client_s conman_client;

  ret = conman_client_init(&conman_client);
  if (ret != OK)
    {
      eng_dbg("conman_client_init failed\n");
      ret = ERROR;
    }
  else
    {
      if (g_state.conman_connid != CONMAN_CONNID_CLEAR)
        dbg("engine->conman_connid != CONMAN_CONNID_CLEAR !!!\n");

      ret = conman_client_request_connection(&conman_client, CONMAN_DATA, &g_state.conman_connid);
      if (ret != OK)
        {
          eng_dbg("conman_client_request_connection failed\n");
          ret = ERROR;
        }

      conman_client_uninit(&conman_client);
    }
#endif

  g_state.requested = true;
  return ret;
}

static char *
get_connector_name_by_index(const char *connectors, uint32_t connector_idx)
{
  cJSON *root, *arrayitem, *connname;
  char *connector_name;

  root = cJSON_Parse(connectors);
  if (!root)
    {
      eng_dbg("Failed to parse cloud params\n");
      return NULL;
    }

  arrayitem = cJSON_GetArrayItem(root, connector_idx);
  if (!arrayitem)
    {
      cJSON_Delete(root);
      return NULL;
    }

  connname = cJSON_GetObjectItem(arrayitem, "connectorName");
  if (!connname || cJSON_type(connname) != cJSON_String)
    {
      eng_dbg("Found connector without connector name!\n");
      connector_name = strdup("");
    }
  else
    {
      connector_name = strdup(cJSON_string(connname));
    }

  cJSON_Delete(root);
  return connector_name;
}

static const struct ts_connector *
find_connector_by_name (const char *connector_name)
{
  int i;

  for (i = 0; ts_connectors[i] != NULL; i++)
    {
      if (ts_connectors[i]->name == connector_name ||
          strcasecmp(ts_connectors[i]->name, connector_name) == 0)
        {
          return ts_connectors[i];
        }
    }

  return NULL;
}

int
__ts_engine_cancel_connection(void)
{
  return cancel_connection_cb(-1, &g_state);
}

int
ts_engine_select_connector (const uint32_t connector_idx, const struct ts_connector **con)
{
  int ret;
  const char *connectors;
  char *connector_name;

  *con = NULL;

  if (g_state.con)
    {
      if (connector_idx != -1 && g_state.con_idx == connector_idx)
        {
          ret = request_connection();
          if (ret < 0)
            {
              eng_dbg("request_connection failed\n");
            }
          *con = g_state.con;
          return OK;
        }

      if (g_state.con->allow_deepsleep)
        {
          ret = ts_core_deepsleep_hook_remove (
              g_state.con->allow_deepsleep);
          if (ret != OK)
            {
              eng_dbg ("deepsleep hook unregister failed\n");
            }
        }

      /* Uninitialize current */

      if (g_state.con->uninit)
        {
          ret = g_state.con->uninit ();
          if (ret != OK)
            {
              return ERROR;
            }
        }

      g_state.con = NULL;
      g_state.con_idx = -1;
    }

  if (connector_idx == -1)
    {
      eng_dbg ("connector uninitialized\n");
      return OK;
    }

  connectors = cloud_property_connectors ();
  if (!connectors)
    {
      eng_dbg("cloud_property_connectors failed\n");
      return ERROR;
    }

  connector_name = get_connector_name_by_index(connectors, connector_idx);
  if (!connector_name)
    {
      eng_dbg ("connector_idx %d does not exist\n", connector_idx);
      free ((void *)connectors);
      return ERROR;
    }

  eng_dbg("connector_idx %d => connector_name '%s'\n",
          connector_idx, connector_name);

  g_state.con = find_connector_by_name (connector_name);
  if (!g_state.con)
    {
      eng_dbg ("connector_name %d does not exist\n", connector_name);
      free(connector_name);
      free ((void *)connectors);
      return ERROR;
    }

  free(connector_name);
  g_state.con_idx = connector_idx;

  /* Initialize */

  if (g_state.con->init)
    {
      ret = g_state.con->init (connectors);
      if (ret != OK)
	{
          eng_dbg("init failed\n");
          g_state.con = NULL;
          g_state.con_idx = -1;
          free ((void *)connectors);
          return ERROR;
	}

      ret = request_connection();
      if (ret < 0)
        {
          eng_dbg("__ts_engine_request_connection failed\n");
        }
    }

  free ((void *)connectors);

  /* Register deepsleep callback */

  if (g_state.con->allow_deepsleep)
    {
      ts_core_deepsleep_hook_add (g_state.con->allow_deepsleep,
                                  (void *)g_state.con);
    }

  *con = g_state.con;
  return OK;
}
