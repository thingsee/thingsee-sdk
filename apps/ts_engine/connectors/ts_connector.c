/****************************************************************************
 * apps/ts_engine/connectors/ts_connector.c
 *
 * Copyright (C) 2015 Haltian Ltd.
 * Authors:
 *   Timo Voutilainen <timo.voutilainen@haltian.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <assert.h>
#include <debug.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <apps/netutils/cJSON.h>

#include "connector.h"
#include "connector_ids.h"
#include "con_dbg.h"
#include "conn_comm.h"
#include "conn_comm_execute_http.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct
{
  uint32_t connector_id;
  char *connector_name;
  char *protocol;
  char *host;
  uint16_t port;
  char *device_auth_uuid;
  char *device_auth_token;
  char *api;
}tsc_cloud_params_s;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool tsc_allow_deepsleep(void * const priv);
static int tsc_init(const char * const connectors);
static int tsc_uninit(void);
static int tsc_send(struct ts_payload *payload, send_cb_t cb, const void *priv);
static int tsc_multisend(struct ts_payload **payload, int number_of_payloads,
                         send_cb_t cb, const void *priv);
static int tsc_multisend_url(struct ts_payload **payload, int number_of_payloads,
                             send_cb_t cb, struct url * const url, const void *priv);
static int tsc_send_url(struct ts_payload *payload, send_cb_t cb,
                        struct url * const url,
                        const void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct {
  con_str_t con;
  tsc_cloud_params_s cloud_params;
} ts_context;

struct tsc_context_priv_s
{
  struct url *url;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct ts_connector tsc =
{
  .id = TS_CONNECTOR_ID,
  .name = "TSC",
  .allow_deepsleep = tsc_allow_deepsleep,
  .init = tsc_init,
  .uninit = tsc_uninit,
  .reg = NULL,
  .unreg = NULL,
  .send = tsc_send,
  .multisend = tsc_multisend,
  .send_url = tsc_send_url,
  .multisend_url = tsc_multisend_url
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool tsc_get_http_params(conn_workflow_context_s *context,
                                struct conn_http_params_s *params)
{
  struct tsc_context_priv_s *context_priv = context->context_priv;
  struct url *context_url = context_priv->url;

  if (!context_url)
    return false; /* use default http params for connector. */

  params->host = context_url->host;
  params->port = context_url->port;
  params->tls = ts_context.con.https;
  params->ipaddr_cache = &context_url->srv_ip4addr;
  return true;
}

static int tsc_post_data_construct(conn_workflow_context_s *context,
                                   char **outhdr, char **outdata)
{
  int datalen = 0, hdrlen = 0;
  char *header = NULL;
  char *tmp;
  int i;
  int ret;

  /* Construct HTTP data */
  *outdata = context->payload;
  datalen = (*outdata != NULL) ? strlen(*outdata) : 0;

  if (datalen > 0)
    {
      struct tsc_context_priv_s *context_priv = context->context_priv;
      struct url *context_url = context_priv->url;

      /* Construct HTTP header */

      if (context_url)
        {
          if (context_url->http_header.valuearray.number_of_items > 0)
            {
              for (i = 0; i < context_url->http_header.valuearray.number_of_items; i++)
                {
                  if (context_url->http_header.valuearray.items[i].valuetype != VALUESTRING)
                    {
                      con_dbg("Invalid valuetype=%d\n", context_url->http_header.valuearray.items[i].valuetype);
                      free(header);
                      return ERROR;
                    }
                  tmp = header ? header : strdup("");

                  ret = asprintf(&header, "%s%s\r\n", tmp, context_url->http_header.valuearray.items[i].valuestring);
                  free(tmp);
                  if (ret < 0)
                    {
                      con_dbg("asprintf failed\n");
                      return ERROR;
                    }
                }
            }
          else
            {
              header = strdup("");
              if (!header)
                {
                  con_dbg("strdup failed\n");
                  return ERROR;
                }
            }
        }
      else /* default ts backend token authentication */
        {
          ret = asprintf(&header, "Authorization: Bearer %s\r\n", ts_context.cloud_params.device_auth_token);
          if (ret < 0)
            {
              return ERROR;
            }
        }

      hdrlen = asprintf(outhdr,
          "POST /%s%s HTTP/1.1\r\n"
          "connectorId: %d\r\n"
          "connectorName: %s\r\n"
          "deviceAuthUuid: %s\r\n"
          "User-Agent: %s\r\n"
          "Host: %s\r\n"
          "Accept: */*\r\n"
          "%s" /* auth */
          "Connection: close\r\n"
          "Content-Length: %d\r\n"
          "Content-Type: application/json\r\n"
          "\r\n",
          (context_url ? (context_url->api ? context_url->api : "") : ts_context.cloud_params.api),
          (context_url ? "" : "/events"),
          ts_context.cloud_params.connector_id,
          ts_context.cloud_params.connector_name,
          ts_context.cloud_params.device_auth_uuid,
          HTTP_DEFAULT_USER_AGENT,
          (context_url ? context_url->host : ts_context.con.host),
          header,
          strlen(*outdata)
      );

      free(header);
    }

  if (datalen >= 0 && hdrlen >= 0)
    {
      con_dbg("SENDINGDATA:\n%s%s\n", *outhdr, *outdata);
    }

  return (datalen >= 0 && hdrlen >= 0) ? OK : ERROR;
}

struct conn_network_task_s* tsc_system_actions(const char * const content)
{
  cJSON *root;
  cJSON *actions;
  cJSON *update;
  int ret;

  root = cJSON_Parse(content);
  if (!root)
    {
      con_dbg("content not json\n");
      return NULL;
    }

  actions = cJSON_GetObjectItem(root, "systemActions");
  if (!actions)
    {
      con_dbg("No system actions\n");
      goto out;
    }

  update = cJSON_GetObjectItem(actions, "updateProfile");
  if (update)
    {
      char *profile;

      con_dbg("New profile found\n");

      profile = cJSON_PrintUnformatted(update);
      cJSON_Delete(root);
      ret = conn_update_profile(profile);
      if (ret < 0)
        {
          con_dbg("conn_update_profile failed\n");

          /* TODO: inform backend if profile update fails */
        }

      free ((void *)profile);
      return NULL;
    }

out:

  cJSON_Delete(root);

  return NULL;
}

static struct conn_network_task_s *tsc_post_data_process(
  conn_workflow_context_s *context,
  int status_code, const char *content)
{
  con_dbg("\n\nCode:%d\n%s\n\n#######################\n", status_code, content);

  if (status_code == 200)
    {
      con_dbg("Data successfully sent to TSC\n");

      return tsc_system_actions(content);
    }
  else
    {
      con_dbg("Unhandled status code: %d!\n", status_code);
    }

  return NULL;
}

static struct conn_network_task_s *tsc_post_data_create(conn_workflow_context_s *context)
{
  struct conn_network_task_s *task = conn_create_network_task(
          "Posting data",
          context,
          tsc_post_data_construct,
          tsc_post_data_process);

  if (task)
    {
      task->http.get_conn_params = tsc_get_http_params;
    }

  return task;
}

static bool tsc_allow_deepsleep(void * const priv)
{
  /* Deep-sleep blocking is handled in conn_comm. */
  return true;
}

static int tsc_parse_cloud_params(const char * const connectors)
{
  cJSON *root, *arrayitem;
  cJSON *connid, *port;
  uint16_t i;
  int ret = ERROR;
  uint32_t connector_id;

  DEBUGASSERT(connectors);

  root = cJSON_Parse(connectors);
  if (!root)
    {
      con_dbg("Failed to parse cloud params\n");
      return ERROR;
    }

  /* Find our connector parameters from array */
  for (i = 0; i < cJSON_GetArraySize(root); i++)
    {
      arrayitem = cJSON_GetArrayItem(root, i);
      if (!arrayitem)
        {
          /* No more connectors */
          break;
        }
      connid = cJSON_GetObjectItem(arrayitem, "connectorId");
      if (!connid)
        {
          /* This Connector does not seem to have connector ID ! */
          con_dbg("Found connector without connector ID!\n");
          continue;
        }
      if (cJSON_type(connid) == cJSON_Number)
        {
          connector_id = cJSON_int(connid);
        }
      else
        {
          con_dbg("Found connector with bad connector ID!\n");
          continue;
        }
      /* TODO: Do we need to loop through connectors array
         Or do we only get the data valid to us (one connector data)?
         if (connector_id == TS_CONNECTOR_ID)
         if (connector_id == 1) */
        {
          /* This is our connector data - make sure at least mandatory items can be found*/
          ts_context.cloud_params.connector_id = connector_id;
          ret = OK;

          if ((conn_init_param_from_json(&arrayitem, "connectorName", &ts_context.cloud_params.connector_name) != OK ||
              conn_init_param_from_json(&arrayitem, "protocol", &ts_context.cloud_params.protocol) != OK ||
              conn_init_param_from_json(&arrayitem, "host", &ts_context.con.host) != OK ||
              conn_init_param_from_json(&arrayitem, "deviceAuthUuid", &ts_context.cloud_params.device_auth_uuid) != OK ||
              conn_init_param_from_json(&arrayitem, "deviceAuthToken", &ts_context.cloud_params.device_auth_token) != OK ||
              conn_init_param_from_json(&arrayitem, "api", &ts_context.cloud_params.api) != OK))
            {
              ret = ERROR;
              con_dbg("Mandatory item missing in cloud params!\n");
              goto out;
            }
          ts_context.con.port = HTTP_DEFAULT_PORT;
          port = cJSON_GetObjectItem(arrayitem, "port");
          if (port)
            {
              if (cJSON_type(port) == cJSON_Number)
                {
                  ts_context.con.port = cJSON_int(port);
                }
              else
                {
                  con_dbg("Failed to read port - using default (%d)!\n", HTTP_DEFAULT_PORT);
                }
            }
        } /* Our connector data */
    } /* For */

  out:

  cJSON_Delete(root);
  return ret;
}

static int tsc_init(const char * const connectors)
{
  int ret = OK;

  memset(&ts_context, 0, sizeof(ts_context));
  ts_context.con.network_ready = false;

  ret = conn_init(&ts_context.con);
  if (ret != OK)
    {
      return ret;
    }

  if (tsc_parse_cloud_params(connectors) != OK)
    {
      con_dbg("Failed to parse cloud connection parameters\n");
      return ERROR;
    }

  return OK;
}

static int tsc_uninit(void)
{
  conn_uninit();

  conn_free_pointer((void **)&ts_context.cloud_params.connector_name);
  conn_free_pointer((void **)&ts_context.cloud_params.device_auth_token);
  conn_free_pointer((void **)&ts_context.cloud_params.device_auth_uuid);
  conn_free_pointer((void **)&ts_context.cloud_params.protocol);
  conn_free_pointer((void **)&ts_context.con.host);
  conn_free_pointer((void **)&ts_context.cloud_params.api);

  return OK;
}

static conn_workflow_context_s *tsc_create_workflow_context(struct ts_payload **payload, int number_of_payloads,
                                                            send_cb_t cb, struct url * const url, const void *priv)
{
  cJSON *root, *pload, *engine, *senses;
  conn_workflow_context_s *context = NULL;
  const char * TS =   "ts";
  uint64_t timestamp_msecs;

  root = cJSON_CreateArray();
  if (root)
    {
      while (number_of_payloads--)
        {
          pload = cJSON_CreateObject();
          if (pload)
            {
              engine = cJSON_CreateObject();
              if (engine)
                {
                  const char * PID =  "pId";
                  const char * PUID = "puId";
                  const char * STID = "stId";
                  const char * EVID = "evId";

                  cJSON_AddStringToObject(engine, PID, (*payload)->state.pId ? (*payload)->state.pId : "(null)");

                  cJSON_AddNumberToObject(engine, PUID, (*payload)->state.puId);

                  cJSON_AddNumberToObject(engine, STID, (*payload)->state.stId);

                  cJSON_AddNumberToObject(engine, EVID, (*payload)->state.evId);

                  timestamp_msecs = (uint64_t)(*payload)->state.ts.tv_sec * 1000;
                  timestamp_msecs += (*payload)->state.ts.tv_nsec / (1000 * 1000);
                  cJSON_AddNumberToObject(engine, TS, timestamp_msecs);

                  cJSON_AddItemToObject(pload, "engine", engine);
                } /* Engine */

              if ((*payload)->number_of_senses > 0)
                {
                  senses = cJSON_CreateArray();
                  if (senses)
                    {
                      int i;
                      for (i = 0; i < (*payload)->number_of_senses; i++)
                        {
                          char sId[11]; /* SenseID format is 0xAABBCCDD -> 10 chars */
                          const char * VAL = "val";

                          cJSON *sense = cJSON_CreateObject();
                          if (!sense)
                            break;

                          sprintf(sId, "0x%08x", (*payload)->senses[i].sId);
                          cJSON_AddStringToObject(sense, "sId", sId);

                          __value_to_json(sense, VAL, &(*payload)->senses[i].value);

                          timestamp_msecs = (uint64_t)(*payload)->state.ts.tv_sec * 1000;
                          timestamp_msecs += (*payload)->state.ts.tv_nsec / (1000 * 1000);
                          cJSON_AddNumberToObject(sense, TS, timestamp_msecs);
                          cJSON_AddItemToArray(senses, sense);
                        }

                      cJSON_AddItemToObject(pload, "senses", senses);
                    }
                }
              cJSON_AddItemToArray(root, pload);
            }
          payload++;
        }
      context = (conn_workflow_context_s*)calloc(1, sizeof(conn_workflow_context_s) + sizeof(struct tsc_context_priv_s));
      if (context)
        {
          struct tsc_context_priv_s *context_priv;

          context->payload = cJSON_PrintUnformatted(root);
          context->cb = cb;
          context->priv = priv;

          context->context_priv = (void *)((uint8_t*)context + sizeof(*context));
          context_priv = context->context_priv;
          context_priv->url = url;
        }
    }

  cJSON_Delete(root);

  return context;
}

static int tsc_multisend_url(struct ts_payload **payload, int number_of_payloads,
                             send_cb_t cb, struct url * const url, const void *priv)
{
  int ret = OK;
  struct conn_network_task_s *send_task = NULL;
  conn_workflow_context_s *context = NULL;

  DEBUGASSERT(payload && priv);

  con_dbg("Sending data\n");
  context = tsc_create_workflow_context(payload, number_of_payloads, cb, url, priv);
  if (!context)
    {
      return ERROR;
    }
  send_task = tsc_post_data_create(context);

  if (send_task)
    {
      ret = conn_network_give_new_conn_task(send_task);
      if (ret != OK)
        {
          conn_destroy_task(send_task);
          send_task = NULL;
        }
    }
  else
    {
      ret = ERROR;
      con_dbg("Could not start a new task for sending data to TSC\n");
    }

  if (!send_task)
    {
      conn_complete_task_workflow(context, ret);
      context = NULL;
    }

  return ret;
}

static int tsc_send_url(struct ts_payload *payload, send_cb_t cb,
                        struct url * const url,
                        const void *priv)
{
  return tsc_multisend_url(&payload, 1, cb, url, priv);
}

static int tsc_multisend(struct ts_payload **payload, int number_of_payloads,
                         send_cb_t cb, const void *priv)
{
  return tsc_multisend_url(payload, number_of_payloads, cb, NULL, priv);
}

static int tsc_send(struct ts_payload *payload, send_cb_t cb, const void *priv)
{
  return tsc_multisend(&payload, 1, cb, priv);
}
