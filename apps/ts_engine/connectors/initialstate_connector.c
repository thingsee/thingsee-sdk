/****************************************************************************
 * apps/ts_engine/connectors/initialstate_connector.c
 *
 * Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
 * Author: Timo Voutilainen <timo.voutilainen@haltian.com>
 * Author: Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
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

#include <apps/netutils/base64.h>
#include <apps/netutils/cJSON.h>

#include "connector.h"
#include "connector_ids.h"
#include "con_dbg.h"
#include "conn_comm.h"

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
  char *api;
  char *protocol;
  char *bucket_key;
  char *device_auth_token;
  char *accept_version;
  bool use_human_readable_senseid;
  bool is_bucket_created;
}inst_cloud_params_s;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool inst_allow_deepsleep(void * const priv);
static int inst_init(const char * const connectors);
static int inst_uninit(void);
static int inst_send(struct ts_payload *payload, send_cb_t cb, const void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct {
  con_str_t con;
  inst_cloud_params_s cloud_params;
} ts_context;

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct ts_connector initialstate =
{
  .id = INITIALSTATE_CONNECTOR_ID,
  .name = "Initial State Cloud",
  .allow_deepsleep = inst_allow_deepsleep,
  .init = inst_init,
  .uninit = inst_uninit,
  .reg = NULL,
  .unreg = NULL,
  .send = inst_send
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int inst_post_bucket_construct(conn_workflow_context_s *context,
                                      char **outhdr, char **outdata)
{
  int datalen = -1, hdrlen = -1;

  /* Construct HTTP data */
  *outdata = context->payload;
  datalen = (*outdata != NULL) ? strlen(*outdata) : 0;

  if (datalen > 0)
    {
      /* Construct HTTP header */
      hdrlen = asprintf(outhdr,
          "POST /api/buckets HTTP/1.1\r\n"
          "User-Agent: %s\r\n"
          "Host: %s\r\n"
          "Accept: */*\r\n"
          "Content-Type: application/json\r\n"
          "X-IS-AccessKey: %s\r\n"
          "Accept-Version: %s\r\n"
          "Content-Length: %d\r\n"
          "\r\n",
          HTTP_USER_AGENT,
          ts_context.con.host,
          ts_context.cloud_params.device_auth_token,
          ts_context.cloud_params.accept_version,
          strlen(*outdata)
      );
    }

  if (datalen >= 0 && hdrlen >= 0)
  con_dbg("SENDINGDATA:\n%s%s\n", *outhdr, *outdata);

  return (datalen >= 0 && hdrlen >= 0) ? OK : ERROR;
}

static int inst_post_data_construct(conn_workflow_context_s *context,
                                   char **outhdr, char **outdata)
{
  int datalen = -1, hdrlen = -1;

  /* Construct HTTP data */
  *outdata = context->payload;
  datalen = (*outdata != NULL) ? strlen(*outdata) : 0;

  if (datalen > 0)
    {
      /* Construct HTTP header */
      hdrlen = asprintf(outhdr,
          "POST /api/events HTTP/1.1\r\n"
          "User-Agent: %s\r\n"
          "Host: %s\r\n"
          "Accept: */*\r\n"
          "Content-Type: application/json\r\n"
          "X-IS-AccessKey: %s\r\n"
          "X-IS-BucketKey: %s\r\n"
          "Accept-Version: %s\r\n"
          "Content-Length: %d\r\n"
          "\r\n",
          HTTP_USER_AGENT,
          ts_context.con.host,
          ts_context.cloud_params.device_auth_token,
          ts_context.cloud_params.bucket_key,
          ts_context.cloud_params.accept_version,
          strlen(*outdata)
      );
    }

  if (datalen >= 0 && hdrlen >= 0)
  con_dbg("SENDINGDATA:\n%s%s\n", *outhdr, *outdata);

  return (datalen >= 0 && hdrlen >= 0) ? OK : ERROR;
}

static struct conn_network_task_s* inst_post_data_process(
  conn_workflow_context_s *context,
  int status_code, const char *content)
{
  con_dbg("\n\nCode:%d\n%s\n\n#######################\n", status_code, content);

  if (status_code == 200 || status_code == 201 || status_code == 204)
    {
      /* Say, we got a 20* response. Bucket is created or data in
       * Set bucket created flag to true */

      ts_context.cloud_params.is_bucket_created = true;
      con_dbg("Data successfully sent to cloud\n");
    }
  else
    {
      con_dbg("Unhandled status code: %d!\n", status_code);
    }

  return NULL;
}

static struct conn_network_task_s* inst_post_data_create(conn_workflow_context_s *context, conn_request_construct_t construct_data)
{
  return conn_create_network_task(
          "Posting data",
          context,
          construct_data,
          inst_post_data_process);
}

static bool inst_allow_deepsleep(void * const priv)
{
  /* Deep-sleep blocking is handled in conn_comm. */
  return true;
}

static int inst_parse_cloud_params(const char * const connectors)
{
  cJSON *root, *arrayitem, *service, *auth;
  cJSON *port;
  uint32_t connid = 1;
  int ret = ERROR;
  bool use_human_sid = true;

  DEBUGASSERT(connectors);

  root = cJSON_Parse(connectors);
  if (!root)
    {
      con_dbg("Failed to parse cloud params\n");
      return ERROR;
    }

  /* Find our connector parameters from array */

  /* Find our connector parameters from array */
  arrayitem = conn_init_find_connector_settings(root, initialstate.name, &connid);
  if (!arrayitem)
    {
      goto out;
    }

  /* This is our connector data - make sure at least mandatory items can be found*/
  ts_context.cloud_params.connector_id = connid;
  ret = OK;

  /* Look for "auth" */
  auth = cJSON_GetObjectItem(arrayitem, "auth");
  if (!auth)
    {
      con_dbg("Failed to find \"auth\" from cloud params!\n");
      ret = ERROR;
      goto out;
    }

  /* Look for "service" */
  service = cJSON_GetObjectItem(arrayitem, "service");
  if (!service)
    {
      con_dbg("Failed to find \"service\" from cloud params!\n");
      ret = ERROR;
      goto out;
    }
  if ((conn_init_param_from_json(&arrayitem, "connectorName", &ts_context.cloud_params.connector_name) != OK ||
       conn_init_param_from_json(&arrayitem, "api", &ts_context.cloud_params.api) != OK ||
       conn_init_param_from_json(&arrayitem, "protocol", &ts_context.cloud_params.protocol) != OK ||
       conn_init_param_from_json(&arrayitem, "host", &ts_context.con.host) != OK ||
       conn_init_param_from_json(&arrayitem, "bucket_key", &ts_context.cloud_params.bucket_key) != OK ||
       conn_init_param_from_json(&auth, "deviceAuthToken", &ts_context.cloud_params.device_auth_token) != OK ||
       conn_init_param_from_json(&arrayitem, "acceptVersion", &ts_context.cloud_params.accept_version) != OK))
    {
      ret = ERROR;
      con_dbg("Mandatory item missing in cloud params!\n");
      goto out;
    }

  /* Optional settings */
  (void)conn_init_boolean_from_json(&service, "useHumanSenseID", &use_human_sid);
  ts_context.cloud_params.use_human_readable_senseid = use_human_sid;

  ts_context.con.port = DEFAULT_PORT;
  port = cJSON_GetObjectItem(arrayitem, "port");
  if (port)
    {
      if (cJSON_type(port) == cJSON_Number)
        {
          ts_context.con.port = cJSON_int(port);
        }
      else
        {
          con_dbg("Failed to read port - using default (%d)!\n", DEFAULT_PORT);
        }
    }

out:
  cJSON_Delete(root);
  return ret;
}

static void inst_free_reserved(void)
{
  conn_free_pointer((void **)&ts_context.cloud_params.connector_name);
  conn_free_pointer((void **)&ts_context.cloud_params.api);
  conn_free_pointer((void **)&ts_context.cloud_params.protocol);
  conn_free_pointer((void **)&ts_context.con.host);
  conn_free_pointer((void **)&ts_context.cloud_params.bucket_key);
  conn_free_pointer((void **)&ts_context.cloud_params.device_auth_token);
  conn_free_pointer((void **)&ts_context.cloud_params.accept_version);
}

static int inst_init(const char * const connectors)
{
  int ret = OK;

  memset(&ts_context, 0, sizeof(ts_context));
  ts_context.con.network_ready = false;

  ret = conn_init(&ts_context.con);
  if (ret != OK)
    {
      return ret;
    }

  if (inst_parse_cloud_params(connectors) != OK)
    {
      con_dbg("Failed to parse cloud connection parameters\n");
      inst_free_reserved();
      return ERROR;
    }

  /* Always false in initialization. Even if the bucket already exists
   * The server will response with 204, if bucket exists. It's not a
   * deadly error for us */

  ts_context.cloud_params.is_bucket_created = false;

  return OK;
}

static int inst_uninit(void)
{
  conn_uninit();
  inst_free_reserved();

  return OK;
}

static conn_workflow_context_s *inst_create_bucket_context(send_cb_t cb, const void *priv)
{
  cJSON *root;
  conn_workflow_context_s *context = NULL;
  const char *KEY = "bucketKey";
  const char *NAME = "bucketName";

  root = cJSON_CreateObject();
  if (root)
    {
      /* Name of the bucket is the same as its key
       * Basically we are talking about the same
       * thing, but one is for human and another one is for machine */

      cJSON_AddStringToObject(root, KEY, ts_context.cloud_params.bucket_key);
      cJSON_AddStringToObject(root, NAME, ts_context.cloud_params.bucket_key);

      context = (conn_workflow_context_s*)calloc(1, sizeof(conn_workflow_context_s));
      if (context)
        {
          context->payload = cJSON_PrintUnformatted(root);
          context->cb = cb;
          context->priv = priv;
        }
      else
        {
          con_dbg("Failed to allocate context\n");
        }
      cJSON_Delete(root);

      con_dbg("Bucket data string is created\n");
    }

  return context;
}

static conn_workflow_context_s *inst_create_workflow_context(struct ts_payload *payload, send_cb_t cb, const void *priv)
{
  cJSON *root, *varvals;
  conn_workflow_context_s *context = NULL;
  const char *KEY = "key";
  const char *VALUE = "value";
  char *varid = NULL;

  con_dbg("Parsing payload: Amount of senses: %d\n", payload->number_of_senses);

  root = cJSON_CreateArray(); /* To hold all data */

  if (root)
    {
      int i;

      for (i = 0; i < payload->number_of_senses; i++)
        {
          int len = 0;

          varvals = cJSON_CreateObject(); /* To hold sensor's name and value */
          if (varvals)
            {
              if (ts_context.cloud_params.use_human_readable_senseid)
                {
                  varid = (char*)payload->senses[i].name;
                }
              else
                {
                  len = asprintf(&varid, "0x%08x", payload->senses[i].sId);
                  if (len < 0)
                    {
                      con_dbg("Failed to allocate sense name\n");
                      break;
                    }
                }

              con_dbg("Handling sense: 0x%08x, %s\n", payload->senses[i].sId, varid);
              if (payload->senses[i].name == NULL)
                {
                  con_dbg("Sense is missing the name - ignoring sense!\n");
                  continue;
                }

              cJSON_AddStringToObject(varvals, KEY, varid);

              switch (payload->senses[i].value.valuetype)
              {
              case VALUEDOUBLE:
                cJSON_AddNumberToObject(varvals, VALUE, payload->senses[i].value.valuedouble);
                break;
              case VALUEINT16:
                cJSON_AddNumberToObject(varvals, VALUE, payload->senses[i].value.valueint16);
                break;
              case VALUEINT32:
                cJSON_AddNumberToObject(varvals, VALUE, payload->senses[i].value.valueint32);
                break;
              case VALUEUINT16:
                cJSON_AddNumberToObject(varvals, VALUE, payload->senses[i].value.valueuint16);
                break;
              case VALUEUINT32:
                cJSON_AddNumberToObject(varvals, VALUE, payload->senses[i].value.valueuint32);
                break;
              case VALUEBOOL:
                cJSON_AddNumberToObject(varvals, VALUE, payload->senses[i].value.valuebool);
                break;
              case VALUESTRING:
                cJSON_AddStringToObject(varvals, VALUE, payload->senses[i].value.valuestring);
                break;
              default:
                break;
              }

              cJSON_AddItemToArray(root, varvals);
              if (!ts_context.cloud_params.use_human_readable_senseid)
                {
                  conn_free_pointer((void**)&varid);
                }
            }
        }
    }
      context = (conn_workflow_context_s*)calloc(1, sizeof(conn_workflow_context_s));
      if (context)
        {
          context->payload = cJSON_PrintUnformatted(root);
          context->cb = cb;
          context->priv = priv;
        }
      else
        {
          con_dbg("Failed to allocate context\n");
        }
      cJSON_Delete(root);

  con_dbg("Parsing payload done\n");
  return context;
}

static int inst_send(struct ts_payload *payload, send_cb_t cb, const void *priv)
{
  int ret = OK;
  struct conn_network_task_s *send_task = NULL;
  conn_workflow_context_s *context = NULL;

  DEBUGASSERT(payload && priv);

  con_dbg("Sending data\n");

  /* On the very first call we will attempt to create a new bucket */

  if (ts_context.cloud_params.is_bucket_created)
    {
      context = inst_create_workflow_context(payload, cb, priv);
      if (!context)
        {
          return ERROR;
        }
      send_task = inst_post_data_create(context, inst_post_data_construct);
    }
  else
    {
      context = inst_create_bucket_context(cb, priv);
      if (!context)
        {
          return ERROR;
        }
      send_task = inst_post_data_create(context, inst_post_bucket_construct);
    }

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
      con_dbg("Could not start a new task for sending data to cloud\n");
    }

  if (!send_task)
    {
      conn_complete_task_workflow(context, ret);
      context = NULL;
    }

  return ret;
}
