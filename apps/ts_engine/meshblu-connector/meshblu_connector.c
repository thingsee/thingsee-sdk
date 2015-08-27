/****************************************************************************
 * apps/ts_engine/meshblu-connector/meshblu_connector.c
 *
 * Copyright (C) 2015 Haltian Ltd. All rights reserved.
 * Author: Timo Voutilainen <timo.voutilainen@haltian.com>
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
#include <float.h>
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
  char *device_type;
  bool send_as_data;
  bool use_human_readable_senseid;
  cJSON *message_target;
}meshblu_cloud_params_s;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool meshblu_allow_deepsleep(void * const priv);
static int meshblu_init(const char * const connectors);
static int meshblu_uninit(void);
static int meshblu_send(struct ts_payload *payload, send_cb_t cb, const void *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct {
  con_str_t con;
  meshblu_cloud_params_s cloud_params;
} ts_context;

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct ts_connector meshblu =
{
  .id = MESHBLU_CONNECTOR_ID,
  .name = "MESHBLU",
  .allow_deepsleep = meshblu_allow_deepsleep,
  .init = meshblu_init,
  .uninit = meshblu_uninit,
  .reg = NULL,
  .unreg = NULL,
  .send = meshblu_send
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int meshblu_post_message_construct(conn_workflow_context_s *context,
                                   char **outhdr, char **outdata)
{
  int datalen = 0, hdrlen = 0;

  /* Construct HTTP data */
  *outdata = context->payload;
  datalen = (*outdata != NULL) ? strlen(*outdata) : 0;

  if (datalen > 0)
    {
      /* Construct HTTP header */
      hdrlen = asprintf(outhdr,
          "POST /messages HTTP/1.1\r\n"
          "meshblu_auth_uuid: %s\r\n"
          "meshblu_auth_token: %s\r\n"
          "User-Agent: %s\r\n"
          "Host: %s\r\n"
          "Accept: */*\r\n"
          "Connection: close\r\n"
          "Content-Length: %d\r\n"
          "Content-Type: application/x-www-form-urlencoded\r\n"
          "\r\n",
          ts_context.cloud_params.device_auth_uuid,
          ts_context.cloud_params.device_auth_token,
          HTTP_USER_AGENT,
          ts_context.con.host,
          strlen(*outdata)
      );
    }
  if (datalen >= 0 && hdrlen >= 0)
  con_dbg("SENDINGMESSAGE:\n%s%s\n", *outhdr, *outdata);

  return (datalen >= 0 && hdrlen >= 0) ? OK : ERROR;
}

static int meshblu_post_data_construct(conn_workflow_context_s *context,
                                   char **outhdr, char **outdata)
{
  int datalen = 0, hdrlen = 0;

  /* Construct HTTP data */
  *outdata = context->payload;
  datalen = (*outdata != NULL) ? strlen(*outdata) : 0;

  if (datalen > 0)
    {
      /* Construct HTTP header */
      hdrlen = asprintf(outhdr,
          "POST /data/%s HTTP/1.1\r\n"
          "meshblu_auth_uuid: %s\r\n"
          "meshblu_auth_token: %s\r\n"
          "User-Agent: %s\r\n"
          "Host: %s\r\n"
          "Accept: */*\r\n"
          "Connection: close\r\n"
          "Content-Length: %d\r\n"
          "Content-Type: application/x-www-form-urlencoded\r\n"
          "\r\n",
          ts_context.cloud_params.device_auth_uuid,
          ts_context.cloud_params.device_auth_uuid,
          ts_context.cloud_params.device_auth_token,
          HTTP_USER_AGENT,
          ts_context.con.host,
          strlen(*outdata)
      );
    }
  if (datalen >= 0 && hdrlen >= 0)
  con_dbg("SENDINGDATA:\n%s%s\n", *outhdr, *outdata);

  return (datalen >= 0 && hdrlen >= 0) ? OK : ERROR;
}

static struct conn_network_task_s* meshblu_post_process(
  conn_workflow_context_s *context,
  int status_code, const char *content)
{
  con_dbg("\n\nCode:%d\n%s\n\n#######################\n", status_code, content);

  if (status_code == 200)
    {
      con_dbg("Data successfully sent to MESHBLU\n");
    }
  else
    {
      con_dbg("Unhandled status code: %d!\n", status_code);
    }

  return NULL;
}

static struct conn_network_task_s* meshblu_post_message_create(conn_workflow_context_s *context)
{
  return conn_create_network_task(
          "Posting message",
          context,
          meshblu_post_message_construct,
          meshblu_post_process);
}

static struct conn_network_task_s* meshblu_post_data_create(conn_workflow_context_s *context)
{
  return conn_create_network_task(
          "Posting data",
          context,
          meshblu_post_data_construct,
          meshblu_post_process);
}

static bool meshblu_allow_deepsleep(void * const priv)
{
  /* Don't allow deep sleep */
  return false;
}

static int meshblu_parse_cloud_params(const char * const connectors)
{
  cJSON *root, *arrayitem, *auth, *service, *connid, *port, *msgtarget;
  uint16_t i;
  int ret = ERROR;
  uint32_t connector_id;
  bool sendd = false;
  bool use_human_sid = true;
  char *temp = NULL;

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
      if (connid->type == cJSON_Number)
        {
          connector_id = connid->valueint;
        }
      else
        {
          con_dbg("Found connector with bad connector ID!\n");
          continue;
        }
      /* TODO: Do we need to loop through connectors array
         Or do we only get the data valid to us (one connector data)?
         if (connector_id == MESHBLU_CONNECTOR_ID) */
      {
        /* This is our connector data - make sure at least mandatory items can be found*/
        ts_context.cloud_params.connector_id = connector_id;
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
            conn_init_param_from_json(&arrayitem, "protocol", &ts_context.cloud_params.protocol) != OK ||
            conn_init_param_from_json(&arrayitem, "host", &ts_context.con.host) != OK ||
            conn_init_param_from_json(&arrayitem, "api", &ts_context.cloud_params.api) != OK ||
            conn_init_param_from_json(&auth, "deviceAuthUuid", &ts_context.cloud_params.device_auth_uuid) != OK ||
            conn_init_param_from_json(&auth, "deviceAuthToken", &ts_context.cloud_params.device_auth_token) != OK ||
            conn_init_param_from_json(&service, "type", &ts_context.cloud_params.device_type) != OK ))
          {
            ret = ERROR;
            con_dbg("Mandatory item missing in cloud params!\n");
            goto out;
          }

        /* Optional settings */
        (void)conn_init_boolean_from_json(&service, "sendAsData", &sendd);
        (void)conn_init_boolean_from_json(&service, "useHumanSenseID", &use_human_sid);
        ts_context.cloud_params.send_as_data = sendd;
        ts_context.cloud_params.use_human_readable_senseid = use_human_sid;

        msgtarget = cJSON_GetObjectItem(service, "messageTarget");
        if (msgtarget)
          {
            /* We are expecting either an array if UUIDs or just one */
            temp = cJSON_PrintUnformatted(msgtarget);
            ts_context.cloud_params.message_target = cJSON_Parse(temp);
            conn_free_pointer((void**)&temp);
          }
        if (!ts_context.cloud_params.message_target)
          {
            con_dbg("No valid 'messageTarget' found -> default to '*'\n");
            ts_context.cloud_params.message_target = cJSON_CreateString("*");
            if (!ts_context.cloud_params.message_target)
              {
                ret = ERROR;
                con_dbg("Failed to allocate message target string\n");
                goto out;
              }
          }

        ts_context.con.port = DEFAULT_PORT;
        port = cJSON_GetObjectItem(arrayitem, "port");
        if (port)
          {
            if (port->type == cJSON_Number)
              {
                ts_context.con.port = port->valueint;
              }
            else
              {
                con_dbg("Failed to read port - using default (%d)!\n", DEFAULT_PORT);
              }
          }
        break; /* Stop after handling our connector's data */
      } /* Our connector data */
    } /* For */

  out:

  cJSON_Delete(root);
  return ret;
}

static void meshblu_free_reserved(void)
{
  conn_free_pointer((void **)&ts_context.cloud_params.connector_name);
  conn_free_pointer((void **)&ts_context.cloud_params.device_auth_token);
  conn_free_pointer((void **)&ts_context.cloud_params.device_auth_uuid);
  conn_free_pointer((void **)&ts_context.cloud_params.protocol);
  conn_free_pointer((void **)&ts_context.con.host);
  conn_free_pointer((void **)&ts_context.cloud_params.api);
  conn_free_pointer((void **)&ts_context.cloud_params.device_type);
  conn_free_pointer((void **)&ts_context.cloud_params.message_target);
}

static int meshblu_init(const char * const connectors)
{
  int ret = OK;

  memset(&ts_context, 0, sizeof(ts_context));
  ts_context.con.network_ready = false;

  ret = conn_init(&ts_context.con);
  if (ret != OK)
    {
      return ret;
    }

  if (meshblu_parse_cloud_params(connectors) != OK)
    {
      con_dbg("Failed to parse cloud connection parameters\n");
      meshblu_free_reserved();
      return ERROR;
    }

  return OK;
}

static int meshblu_uninit(void)
{
  conn_uninit();
  meshblu_free_reserved();

  return OK;
}

static conn_workflow_context_s *meshblu_create_data_workflow_context(struct ts_payload *payload, send_cb_t cb, const void *priv)
{
  conn_workflow_context_s *context = NULL;
  char *data_tmp = NULL;
  char *data = NULL;
  char *varid = NULL;
  /* Need this one to properly handle float numbers
   * and reduce the amount of allocated memory and sent data
   * to the server */
  char double_buffer[32];
  int float_len = 0;
  int i;

  con_dbg("Parsing payload: Amount of senses: %d\n", payload->number_of_senses);

  for (i = 0; i < payload->number_of_senses; i++)
    {
      char len;
      char separator[2];
      char *dataitem = NULL;

      snprintf(separator, sizeof(separator), "&");
      con_dbg("Handling sense: 0x%08x (%s)\n", payload->senses[i].sId, payload->senses[i].name);
      if (payload->senses[i].name == NULL)
        {
          con_dbg("Sense is missing the name - ignoring sense!\n");
          continue;
        }

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
              conn_free_pointer((void**)&data);
              break;
            }
        }

      if (i == payload->number_of_senses - 1)
        {
          /* This is the last sense */
          snprintf(separator, sizeof(separator), "");
        }

      switch (payload->senses[i].value.valuetype)
      {
      case VALUEDOUBLE:
        float_len = snprintf(double_buffer, sizeof(double_buffer), "%.10f", payload->senses[i].value.valuedouble);
        if (strchr(double_buffer, '.'))
          {
            /* Remove trailing zeros. */

            float_len -= 1;

            while (float_len > 0)
              {
                if (double_buffer[--float_len] != '0')
                  break;

                double_buffer[float_len] = '\0';
              }

            /* Remove trailing dot. */

            if (float_len > 0 && double_buffer[float_len] == '.')
              double_buffer[float_len] = '\0';
          }
        len = asprintf(&dataitem, "%s=%s%s", varid, double_buffer, separator);
        break;
      case VALUEINT16:
        len = asprintf(&dataitem, "%s=%d%s", varid, payload->senses[i].value.valueint16, separator);
        break;
      case VALUEINT32:
        len = asprintf(&dataitem, "%s=%d%s", varid, payload->senses[i].value.valueint32, separator);
        break;
      case VALUEUINT16:
        len = asprintf(&dataitem, "%s=%u%s", varid, payload->senses[i].value.valueuint16, separator);
        break;
      case VALUEUINT32:
        len = asprintf(&dataitem, "%s=%u%s", varid, payload->senses[i].value.valueuint32, separator);
        break;
      case VALUEBOOL:
        len = asprintf(&dataitem, "%s=%d%s", varid, payload->senses[i].value.valuebool, separator);
        break;
      case VALUESTRING:
        len = asprintf(&dataitem, "%s=%s%s", varid, payload->senses[i].value.valuestring, separator);
        break;
      default:
        con_dbg("Unsupported value type: %d\n", payload->senses[i].value.valuetype);
        len = -1;
        break;
      }
      if (!ts_context.cloud_params.use_human_readable_senseid)
        {
          conn_free_pointer((void**)&varid);
        }
      if (len < 0)
        {
          con_dbg("Could not allocate for key value\n");
          conn_free_pointer((void**)&data);
          break;
        }

      /* Add this key val to data */

      if (data == NULL)
        {
          data = dataitem;
        }
      else
        {
          len = asprintf(&data_tmp, "%s%s", data, dataitem);
          conn_free_pointer((void**)&dataitem);
          if (len < 0)
            {
              con_dbg("Could not allocate for key value\n");
              conn_free_pointer((void**)&data);
              break;
            }
          conn_free_pointer((void**)&data);
          data = data_tmp;
        }

    } /* End of one sense item loop */

    if (!data || strlen(data) == 0)
      {
        con_dbg("Failed to create data workflow context\n");
        return context;
      }

    context = (conn_workflow_context_s*)calloc(1, sizeof(conn_workflow_context_s));
    if (context)
      {
        context->payload = data;
        context->cb = cb;
        context->priv = priv;
      }
    else
      {
        con_dbg("Failed to allocate context\n");
      }

  con_dbg("Parsing payload done\n");
  return context;
}

static conn_workflow_context_s *meshblu_create_message_workflow_context(struct ts_payload *payload, send_cb_t cb, const void *priv)
{
  cJSON *root, *devices, *payld, *senses;
  conn_workflow_context_s *context = NULL;
  char *varid = NULL;
  char *temp = NULL;

  con_dbg("Parsing payload: Amount of senses: %d\n", payload->number_of_senses);

  root = cJSON_CreateObject();
  if (root)
    {
      temp = cJSON_PrintUnformatted(ts_context.cloud_params.message_target);
      devices = cJSON_Parse(temp);
      conn_free_pointer((void**)&temp);
      if (devices)
        {
          cJSON_AddItemToObject(root, "devices", devices);
          payld = cJSON_CreateObject();
          if (payld)
            {
              senses = cJSON_CreateArray();
              if (senses)
                {
                  int i;
                  for (i = 0; i < payload->number_of_senses; i++)
                    {
                      int len = 0;

                      con_dbg("Handling sense: 0x%08x\n", payload->senses[i].sId);
                      if (payload->senses[i].name == NULL)
                        {
                          con_dbg("Sense is missing the name - ignoring sense!\n");
                          continue;
                        }

                      cJSON *sense = cJSON_CreateObject();
                      if (!sense)
                        break;

                      if (ts_context.cloud_params.use_human_readable_senseid)
                        {
                          varid = (char *)payload->senses[i].name;
                        }
                      else
                        {
                          len = asprintf(&varid, "0x%08x", payload->senses[i].sId);
                          if (len < 0)
                            {
                              con_dbg("Failed to allocate sense name\n");
                              break;;
                            }
                        }

                      switch (payload->senses[i].value.valuetype)
                      {
                      case VALUEDOUBLE:
                        cJSON_AddNumberToObject(sense, varid, payload->senses[i].value.valuedouble);
                        break;
                      case VALUEINT16:
                        cJSON_AddNumberToObject(sense, varid, payload->senses[i].value.valueint16);
                        break;
                      case VALUEINT32:
                        cJSON_AddNumberToObject(sense, varid, payload->senses[i].value.valueint32);
                        break;
                      case VALUEUINT16:
                        cJSON_AddNumberToObject(sense, varid, payload->senses[i].value.valueuint16);
                        break;
                      case VALUEUINT32:
                        cJSON_AddNumberToObject(sense, varid, payload->senses[i].value.valueuint32);
                        break;
                      case VALUEBOOL:
                        cJSON_AddNumberToObject(sense, varid, payload->senses[i].value.valuebool);
                        break;
                      case VALUESTRING:
                        cJSON_AddStringToObject(sense, varid, payload->senses[i].value.valuestring);
                        break;
                      default:
                        con_dbg("Unsupported value type: %d\n", payload->senses[i].value.valuetype);
                        break;
                      }
                      if (!ts_context.cloud_params.use_human_readable_senseid)
                        {
                          conn_free_pointer((void**)&varid);
                        }
                      cJSON_AddItemToArray(senses, sense);
                    }
                  cJSON_AddItemToObject(payld, "senses", senses);
                  cJSON_AddItemToObject(root, "payload", payld);
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
    }

  con_dbg("Parsing payload done\n");
  return context;
}

static int meshblu_send(struct ts_payload *payload, send_cb_t cb, const void *priv)
{
  int ret = OK;
  struct conn_network_task_s *send_task = NULL;
  conn_workflow_context_s *context = NULL;

  DEBUGASSERT(payload && priv);

  if (ts_context.cloud_params.send_as_data)
    {
      con_dbg("Sending data\n");
      context = meshblu_create_data_workflow_context(payload, cb, priv);
      if (!context)
        {
          return ERROR;
        }
      send_task = meshblu_post_data_create(context);
    }
  else
    {
      con_dbg("Sending message\n");
      context = meshblu_create_message_workflow_context(payload, cb, priv);
      if (!context)
        {
          return ERROR;
        }
      send_task = meshblu_post_message_create(context);
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
      con_dbg("Could not start a new task for sending data to MESHBLU\n");
    }

  if (!send_task)
    {
      conn_complete_task_workflow(context, ret);
      context = NULL;
    }

  return ret;
}
