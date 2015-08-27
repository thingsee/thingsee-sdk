/****************************************************************************
 * apps/ts_engine/kii-connector/kii_construct_connext_stub.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "con_dbg.h"
#include "connector.h"

#include "conn_comm.h"
#include "kii_connext.h"
#include "kii_construct_connext.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct ts_context ts_context;
struct conn_network_task_s* kii_get_access_token_create(conn_workflow_context_s *context);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

struct conn_network_task_s* kii_get_access_token_create(conn_workflow_context_s *context);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

conn_workflow_context_s *kii_create_workflow_context(struct ts_payload *payload, send_cb_t cb, const void *priv)
{
  cJSON *root, *senses;
  conn_workflow_context_s *context = NULL;

  root = cJSON_CreateObject();
  if (root)
    {
      senses = cJSON_CreateArray();
      if (senses)
        {
          int i;
          for (i = 0; i < payload->number_of_senses; i++)
            {
              char sId[11]; /* SenseID format is 0xAABBCCDD -> 10 chars */
              const char * VAL = "val";

              cJSON *sense = cJSON_CreateObject();
              if (!sense)
                break;

              sprintf(sId, "0x%08x", payload->senses[i].sId);
              cJSON_AddStringToObject(sense, "sId", sId);

              switch (payload->senses[i].value.valuetype)
                {
                case VALUEDOUBLE:
                  cJSON_AddNumberToObject(sense, VAL, payload->senses[i].value.valuedouble);
                  break;
                case VALUEINT16:
                  cJSON_AddNumberToObject(sense, VAL, payload->senses[i].value.valueint16);
                  break;
                case VALUEINT32:
                  cJSON_AddNumberToObject(sense, VAL, payload->senses[i].value.valueint32);
                  break;
                case VALUEUINT16:
                  cJSON_AddNumberToObject(sense, VAL, payload->senses[i].value.valueuint16);
                  break;
                case VALUEUINT32:
                  cJSON_AddNumberToObject(sense, VAL, payload->senses[i].value.valueuint32);
                  break;
                case VALUEBOOL:
                  cJSON_AddNumberToObject(sense, VAL, payload->senses[i].value.valuebool);
                  break;
                case VALUESTRING:
                  cJSON_AddStringToObject(sense, VAL, payload->senses[i].value.valuestring);
                  break;
                default:
                  break;
                }

              cJSON_AddItemToArray(senses, sense);
            }

          cJSON_AddItemToObject(root, "senses", senses);
        }

      context = (conn_workflow_context_s*)calloc(1, sizeof(conn_workflow_context_s));
      if (context)
        {
          context->payload = cJSON_PrintUnformatted(root);
          context->cb = cb;
          context->priv = priv;
        }
      cJSON_Delete(root);
    }

  return context;
}

int kii_post_data_construct(conn_workflow_context_s *context,
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
                        "POST /api/apps/%s/things/VENDOR_THING_ID:%s/buckets/%s/objects HTTP/1.1\r\n"
                        "User-Agent: %s\r\n"
                        "Host: %s\r\n"
                        "Accept: */*\r\n"
                        "x-kii-appid: %s\r\n"
                        "x-kii-appkey: %s\r\n"
                        "Authorization: Bearer %s\r\n"
                        "Connection: close\r\n"
                        "Content-Length: %d\r\n"
                        "Content-Type: application/json\r\n"
                        "\r\n",
                        ts_context.cloud_params.app_id,
                        ts_context.cloud_params.vendor_thing_id,
                        ts_context.cloud_params.bucket,
                        HTTP_USER_AGENT,
                        ts_context.con.host,
                        ts_context.cloud_params.app_id,
                        ts_context.cloud_params.app_key,
                        ts_context.cloud_params.access_token,
                        strlen(*outdata)
                       );
    }
  if (datalen >= 0 && hdrlen >= 0)
    con_dbg("Header:\n%s\nPayload:\n%s\n", *outhdr, *outdata);

  return (datalen >= 0 && hdrlen >= 0) ? OK : ERROR;
}

struct conn_network_task_s* kii_post_data_process(
    conn_workflow_context_s *context,
    int status_code, const char *content)
{
  con_dbg("\n\nCode:%d\n%s\n\n#######################\n", status_code, content);
  if (status_code == 403) /* Is Access Token invalid (expired) ? */
    {
      /* Existing messages have expired token */
      /* Get a new Access Token */
      return kii_get_access_token_create(context);
    }
  else if (status_code == 201)
    {
      con_dbg("Data successfully sent to KII\n");
    }
  else
    {
      con_dbg("Unhandled status code: %d!\n", status_code);
    }

  return NULL;
}
