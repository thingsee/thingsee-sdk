/****************************************************************************
 * apps/ts_engine/kii-connector/kii_connector.c
 *
 * Copyright (C) 2015 Haltian Ltd. All rights reserved.
 * Author: Sila Kayo <sila.kayo@haltian.com>
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
#include "kii_connext.h"
#include "kii_construct_connext.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool kii_allow_deepsleep(void * const priv);
static int kii_init(const char * const connectors);
static int kii_uninit(void);
static int kii_send(struct ts_payload *payload, send_cb_t cb, const void *priv);
struct conn_network_task_s* kii_get_access_token_create(conn_workflow_context_s *context);
static struct conn_network_task_s* kii_post_data_create(conn_workflow_context_s *context);
static struct conn_network_task_s* kii_register_device_create(conn_workflow_context_s *context);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct ts_connector kii =
{
    .id = KII_CONNECTOR_ID,
    .name = "KII",
    .allow_deepsleep = kii_allow_deepsleep,
    .init = kii_init,
    .uninit = kii_uninit,
    .reg = NULL,
    .unreg = NULL,
    .send = kii_send
};

struct ts_context ts_context;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void kii_invalidate_access_token(void)
{
  conn_free_pointer((void**)&ts_context.cloud_params.access_token);

  con_dbg("deleting cached KII Access Token\n");
}

static void kii_invalidate_thingid(void)
{
  conn_free_pointer((void**)&ts_context.cloud_params.thing_id);
  ts_context.cloud_params.thing_id = NULL;

  con_dbg("deleting cached KII Thing id\n");
}

static void kii_save_access_token(const char *access_token)
{
  kii_invalidate_access_token();
  ts_context.kii_waiting_access_token = false;

  if (access_token)
    {
      ts_context.cloud_params.access_token = (char*)malloc(strlen(access_token) + 1);
      if (ts_context.cloud_params.access_token)
        strcpy(ts_context.cloud_params.access_token, access_token);
    }

  con_dbg("KII Access Token is '%s'\n", ts_context.cloud_params.access_token);
}

static void kii_save_thingid(const char *thingid)
{
  kii_invalidate_thingid();

  if (thingid)
    {
      ts_context.cloud_params.thing_id = (char*)malloc(strlen(thingid) + 1);
      if (ts_context.cloud_params.thing_id)
        strcpy(ts_context.cloud_params.thing_id, thingid);
    }

  con_dbg("KII ThingId is '%s'\n", ts_context.cloud_params.thing_id);
}

static struct conn_network_task_s* kii_post_data_create(conn_workflow_context_s *context)
{
  return conn_create_network_task(
      "Posting data",
      context,
      kii_post_data_construct,
      kii_post_data_process);
}

static int kii_register_device_construct(conn_workflow_context_s *context,
    char **outhdr, char **outdata)
{
  int datalen = 0, hdrlen = 0;

  /* Construct HTTP data */
  datalen = asprintf(outdata,
      "{"
      "\"_thingType\":\"%s\","
      "\"_vendor\":\"%s\","
      "\"_vendorThingID\":\"%s\","
      "\"_password\":\"%s\""
      "}",
      ts_context.cloud_params.thing_type,
      ts_context.cloud_params.kii_vendor,
      ts_context.cloud_params.vendor_thing_id,
      ts_context.cloud_params.password
  );

  if (datalen > 0)
    {
      /* Construct HTTP header */
      hdrlen = asprintf(outhdr,
          "POST /api/apps/%s/things HTTP/1.1\r\n"
          "User-Agent: %s\r\n"
          "Host: %s\r\n"
          "Accept: */*\r\n"
          "x-kii-appid: %s\r\n"
          "x-kii-appkey: %s\r\n"
          "Connection: close\r\n"
          "Content-Length: %d\r\n"
          "Content-Type: application/vnd.kii.ThingRegistrationAndAuthorizationRequest+json\r\n"
          "\r\n",
          ts_context.cloud_params.app_id,
          HTTP_USER_AGENT,
          ts_context.con.host,
          ts_context.cloud_params.app_id,
          ts_context.cloud_params.app_key,
          strlen(*outdata)
      );
      if (hdrlen < 0)
        {
          conn_free_pointer((void**)outdata);
        }
    }
  if (datalen >= 0 && hdrlen >= 0)
    con_dbg("Regheader:\n%s\nPayload:\n%s\n", *outhdr, *outdata);

  return (datalen >= 0 && hdrlen >= 0) ? OK : ERROR;
}

static struct conn_network_task_s* kii_register_device_process(
    conn_workflow_context_s *context,
    int status_code, const char *content)
{
  con_dbg("\n\nCode:%d\n%s\n\n#######################\n", status_code, content);
  if (status_code == 201)
    {
      if (content)
        {
          cJSON *root = cJSON_Parse(content);
          if (root)
            {
              pthread_mutex_lock(&ts_context.mutex);

              cJSON *json_access_token = cJSON_GetObjectItem(root, "_accessToken");
              if (json_access_token)
                kii_save_access_token(json_access_token->valuestring);

              cJSON *json_thing_id = cJSON_GetObjectItem(root, "_id");
              if (json_thing_id)
                kii_save_thingid(json_thing_id->valuestring);

              pthread_mutex_unlock(&ts_context.mutex);

              cJSON_Delete(root);
            }
        }
    }
  else if (status_code == 409)
    {
      con_dbg("The device is already registered\n");
      /* Get a new Access Token */
      return kii_get_access_token_create(context);
    }
  else
    {
      con_dbg("Unhandled status code: %d!\n", status_code);
    }

  return NULL;
}

static struct conn_network_task_s* kii_register_device_create(conn_workflow_context_s *context)
{
  return conn_create_network_task(
      "Registering the device",
      context,
      kii_register_device_construct,
      kii_register_device_process);
}

static int kii_get_access_token_construct(conn_workflow_context_s *context,
    char **outhdr, char **outdata)
{
  int datalen = 0, hdrlen = 0;

  /* Construct HTTP data */
  datalen = asprintf(outdata,
      "{"
      "\"username\":\"VENDOR_THING_ID:%s\","
      "\"password\":\"%s\""
      "}",
      ts_context.cloud_params.vendor_thing_id,
      ts_context.cloud_params.password
  );

  if (datalen > 0)
    {
      /* Construct HTTP header */
      hdrlen = asprintf(outhdr,
          "POST /api/oauth2/token HTTP/1.1\r\n"
          "User-Agent: %s\r\n"
          "Host: %s\r\n"
          "Accept: */*\r\n"
          "x-kii-appid: %s\r\n"
          "x-kii-appkey: %s\r\n"
          "Connection: close\r\n"
          "Content-Length: %d\r\n"
          "Content-Type: application/vnd.kii.OauthTokenRequest+json\r\n"
          "\r\n",
          HTTP_USER_AGENT,
          ts_context.con.host,
          ts_context.cloud_params.app_id,
          ts_context.cloud_params.app_key,
          strlen(*outdata)
      );
      if (hdrlen < 0)
        {
          conn_free_pointer((void**)outdata);
        }
    }
  if (datalen >= 0 && hdrlen >= 0)
    con_dbg("AccessTokenHeader:\n%s\nPayload:\n%s\n", *outhdr, *outdata);

  return (datalen >= 0 && hdrlen >= 0) ? OK : ERROR;
}

static struct conn_network_task_s* kii_get_access_token_process(
    conn_workflow_context_s *context,
    int status_code, const char *content)
{
  con_dbg("\n\nCode:%d\n%s\n\n#######################\n", status_code, content);
  if (status_code == 400) /* Device not registered? */
    {
      /* Register the device first */
      return kii_register_device_create(context);
    }
  else if (status_code == 200)
    {
      if (content)
        {
          cJSON *root = cJSON_Parse(content);
          if (root)
            {
              cJSON *json_access_token = cJSON_GetObjectItem(root, "access_token");
              if (json_access_token)
                kii_save_access_token(json_access_token->valuestring);

              cJSON *json_thing_id = cJSON_GetObjectItem(root, "id");
              if (json_thing_id)
                kii_save_thingid(json_thing_id->valuestring);
              cJSON_Delete(root);
            }
        }
    }
  else
    {
      con_dbg("Unhandled status code: %d!\n", status_code);
    }

  return NULL;
}

struct conn_network_task_s* kii_get_access_token_create(conn_workflow_context_s *context)
{
  /* Current messages have expired tokens - delete from queue */
  conn_empty_message_queue();

  pthread_mutex_lock(&ts_context.mutex);

  kii_invalidate_access_token();

  if (ts_context.kii_active)
    {
      ts_context.kii_waiting_access_token = true;

      con_dbg("Requesting new access token\n");
      pthread_mutex_unlock(&ts_context.mutex);
      return conn_create_network_task(
          "Getting an Access Token",
          context,
          kii_get_access_token_construct,
          kii_get_access_token_process);
    }
  else
    {
      con_dbg("Ignored Requesting new access token due to shutting down\n");
      pthread_mutex_unlock(&ts_context.mutex);
      return NULL;
    }
}

static bool kii_allow_deepsleep(void * const priv)
{
  /* Don't allow deep sleep */
  return false;
}

static int kii_parse_cloud_params(const char * const connectors)
{
  cJSON *root, *arrayitem, *auth, *service, *connid, *port;
  uint16_t i;
  int ret = ERROR;
  uint32_t connector_id;

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
         if (connector_id == KII_CONNECTOR_ID) */
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
        if (!auth)
          {
            con_dbg("Failed to find \"service\" from cloud params!\n");
            ret = ERROR;
            goto out;
          }
        if ((conn_init_param_from_json(&arrayitem, "connectorName", &ts_context.cloud_params.connector_name) != OK ||
            conn_init_param_from_json(&arrayitem, "protocol", &ts_context.cloud_params.protocol) != OK ||
            conn_init_param_from_json(&arrayitem, "host", &ts_context.con.host) != OK ||
            conn_init_param_from_json(&auth, "appId", &ts_context.cloud_params.app_id) != OK ||
            conn_init_param_from_json(&auth, "appKey", &ts_context.cloud_params.app_key) != OK ||
            conn_init_param_from_json(&auth, "accessToken", &ts_context.cloud_params.access_token) != OK ||
            conn_init_param_from_json(&service, "thingType", &ts_context.cloud_params.thing_type) != OK ||
            conn_init_param_from_json(&service, "thingId", &ts_context.cloud_params.thing_id) != OK ||
            conn_init_param_from_json(&service, "vendorThingId", &ts_context.cloud_params.vendor_thing_id) != OK ||
            conn_init_param_from_json(&service, "password", &ts_context.cloud_params.password) != OK ||
            conn_init_param_from_json(&service, "vendor", &ts_context.cloud_params.kii_vendor) != OK ||
            conn_init_param_from_json(&service, "bucketName", &ts_context.cloud_params.bucket) != OK))
          {
            ret = ERROR;
            con_dbg("Mandatory item missing in cloud params!\n");
            goto out;
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

static void kii_free_reserved(void)
{
  conn_free_pointer((void**)&ts_context.cloud_params.connector_name);
  conn_free_pointer((void**)&ts_context.cloud_params.protocol);
  conn_free_pointer((void**)&ts_context.con.host);
  conn_free_pointer((void**)&ts_context.cloud_params.app_id);
  conn_free_pointer((void**)&ts_context.cloud_params.app_key);
  conn_free_pointer((void**)&ts_context.cloud_params.access_token);
  conn_free_pointer((void**)&ts_context.cloud_params.thing_type);
  conn_free_pointer((void**)&ts_context.cloud_params.thing_id);
  conn_free_pointer((void**)&ts_context.cloud_params.vendor_thing_id);
  conn_free_pointer((void**)&ts_context.cloud_params.password);
  conn_free_pointer((void**)&ts_context.cloud_params.kii_vendor);
  conn_free_pointer((void**)&ts_context.cloud_params.bucket);
}

static int kii_init(const char * const connectors)
{
  int ret = OK;

  memset(&ts_context, 0, sizeof(ts_context));
  ts_context.con.network_ready = false;

  kii_invalidate_access_token();

  ret = conn_init(&ts_context.con);
  if (ret != OK)
    {
      return ret;
    }

  /* Check if we have external configuration available */
  if (kii_get_configuration(&ts_context.cloud_params, &ts_context.con) != KII_EXT_OK)
    {
      kii_free_reserved();
      /* Otherwise load configuration from connector data */
      if (kii_parse_cloud_params(connectors) != OK)
        {
          con_dbg("Failed to parse cloud connection parameters\n");
          kii_free_reserved();
          return ERROR;
        }
    }

  ts_context.kii_active = true;
  pthread_mutex_init(&ts_context.mutex, NULL);

  return OK;
}

static int kii_uninit(void)
{
  pthread_mutex_lock(&ts_context.mutex);
  ts_context.kii_active = false;
  kii_invalidate_access_token();
  pthread_mutex_unlock(&ts_context.mutex);

  conn_uninit();

  kii_free_reserved();

  pthread_mutex_destroy(&ts_context.mutex);

  return OK;
}


static kii_get_ext_status_t kii_update_vendor_thing_id(void)
{
  kii_get_ext_status_t ret;
  char *vendor_thing_id = NULL;

  ret = kii_get_vendor_thing_id(&vendor_thing_id);
  if (ret == KII_EXT_OK && vendor_thing_id)
    {
      if ((ts_context.cloud_params.vendor_thing_id == NULL) ||
          (strcmp(ts_context.cloud_params.vendor_thing_id, vendor_thing_id) != 0))
        {
          int len = -1;

          conn_free_pointer((void**)&ts_context.cloud_params.vendor_thing_id);
          len = asprintf(&ts_context.cloud_params.vendor_thing_id, "%s", vendor_thing_id);
          if (len <= 0)
            {
              con_dbg("Failed to allocate new vendor thing id\n");
              return KII_EXT_WAITING;
            }
          con_dbg("Setting new vendor thing id: %s\n", ts_context.cloud_params.vendor_thing_id);
        }
    }
  return ret;
}

static int kii_send(struct ts_payload *payload, send_cb_t cb, const void *priv)
{
  int ret = OK;
  struct conn_network_task_s *send_task = NULL;
  conn_workflow_context_s *context = NULL;


  DEBUGASSERT(payload && priv);
  con_dbg("Received payload\n");

  pthread_mutex_lock(&ts_context.mutex);
  if (!ts_context.kii_active)
    {
      pthread_mutex_unlock(&ts_context.mutex);
      return ERROR;
    }

  if (kii_update_vendor_thing_id() == KII_EXT_WAITING)
    {
      con_dbg("New vendor thing id not yet available\n");
      pthread_mutex_unlock(&ts_context.mutex);
      return ERROR;
    }

  pthread_mutex_unlock(&ts_context.mutex);

  if (!ts_context.cloud_params.access_token)
    {
      if (ts_context.kii_waiting_access_token) {
          ret = ERROR;
          con_dbg("Still waiting for a new access token\n");
      } else {
          con_dbg("Requesting a new access token\n");
          context = kii_create_workflow_context(payload, cb, priv);
          send_task = kii_get_access_token_create(context);
      }
    }
  else
    {
      con_dbg("Sending data\n");
      context = kii_create_workflow_context(payload, cb, priv);
      send_task = kii_post_data_create(context);
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
      con_dbg("Could not start a new task for sending data to KII\n");
    }

  if (!send_task)
    {
      conn_complete_task_workflow(context, ret);
      context = NULL;
    }

  return ret;
}
