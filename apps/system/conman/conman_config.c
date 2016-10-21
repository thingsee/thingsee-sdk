/****************************************************************************
 * apps/system/conman/conman_config.c
 *
 *   Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
 *   Author: Pekka Ervasti <pekka.ervasti@haltian.com>
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

#include <string.h>
#include <stdlib.h>
#include <poll.h>

#include <apps/netutils/cJSON.h>

#include "conman_dbg.h"
#include "conman_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define JSON_STRING_INIT()          cJSON *json_entry
#define JSON_STRDUP(str,obj,label)  do { \
                                      json_entry = cJSON_GetObjectItem(obj, label); \
                                      str = (json_entry ? \
                                           strdup(cJSON_string(json_entry)) : NULL); \
                                    } while(0)

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct conman_ubmodem_config_offsets_s
{
char *label;
int offset;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct conman_ubmodem_config_offsets_s g_conman_ubmodem_config_offsets[] =
  {
    {
    "modem.apn_name", offset_of(struct conman_cellular_connection_s,
        access_point_name)
    },
    {
    "modem.apn_ipaddr", offset_of(struct conman_cellular_connection_s,
        access_point_ipaddr)
    },
    {
    "modem.apn_user", offset_of(struct conman_cellular_connection_s, user_name)
    },
    {
    "modem.apn_password", offset_of(struct conman_cellular_connection_s,
        password)
    },
    {
    "modem.pin", offset_of(struct conman_cellular_connection_s, pin)
    },
    {
    NULL, 0
    },
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: current_cellular_connection
 *
 * Description:
 *   Returns pointer to the current cellular connection configuration.
 *
 * Input Parameters:
 *   conman : connection manager handle
 *
 * Returned Value:
 *   Pointer to the current cellular connection configuration or NULL if
 *   none available or current id points to non existent configuration.
 *
 ****************************************************************************/

static const struct conman_cellular_connection_s *current_cellular_connection(
    struct conman_s *conman)
{
  struct conman_connection_entry_s *entry;
  const struct conman_cellular_connection_s *retval = NULL;

  entry = (struct conman_connection_entry_s *) sq_peek(
      &conman->connections.cellular);

  while (entry)
    {
      if (entry->connection.cellular->id
          == conman->connections.current.cellular)
        {
          retval = entry->connection.cellular;
          goto out;
        }
      entry = (struct conman_connection_entry_s *) sq_next(&entry->entry);
    }

out:

  return retval;
}

/****************************************************************************
 * Name: current_wifi_connection
 *
 * Description:
 *   Returns pointer to the current wifi connection configuration.
 *
 * Input Parameters:
 *   conman : connection manager handle
 *
 * Returned Value:
 *   Pointer to the current wifi connection configuration or NULL if
 *   none available or current id points to non existent configuration.
 *
 ****************************************************************************/

static const struct conman_wifi_connection_s *current_wifi_connection(
    struct conman_s *conman)
{
  struct conman_connection_entry_s *entry;
  const struct conman_wifi_connection_s *retval = NULL;

  entry = (struct conman_connection_entry_s *) sq_peek(
      &conman->connections.wifi);

  while (entry)
    {
      if (entry->connection.wifi->id
          == conman->connections.current.wifi)
        {
          retval = entry->connection.wifi;
          goto out;
        }
      entry = (struct conman_connection_entry_s *) sq_next(&entry->entry);
    }

out:

  return retval;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __conman_config_init
 *
 * Description:
 *   Initializes the connection manager internal data structure.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *
 ****************************************************************************/

void __conman_config_init(struct conman_s *conman)
{
  sq_init(&conman->connections.cellular);
  sq_init(&conman->connections.wifi);

  conman->connections.amount.cellular = 0;
  conman->connections.amount.wifi = 0;

  conman->connections.current.cellular = -1;
  conman->connections.current.wifi = -1;

  conman->connections.current.cellular_refcnt = 0;
  conman->connections.current.wifi_refcnt = 0;

  sq_init(&conman->connections.current.connids);
  conman->connections.current.connid_next = CONMAN_CONNID_MIN;
}

/****************************************************************************
 * Name: __conman_config_set_connections
 *
 * Description:
 *   Parses the configuration json data received from the client and
 *   initializes the internal representation of the same data.
 *
 * Input Parameters:
 *   conman  : connection manager handle
 *   config  : json format connection configuration
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   Anything else that one might need to know to use this function.
 *
 ****************************************************************************/

int __conman_config_set_connections(struct conman_s *conman, const char *config)
{
  struct conman_connection_entry_s *entry;
  struct conman_cellular_connection_s *cellu;
  struct conman_wifi_connection_s *wifi;
  cJSON *json_config;
  cJSON *json_connections;
  cJSON *json_connection;
  cJSON *cons, *con;
  int number_of_connections;
  int n;
  int i;
  int j;
  int err = CONMAN_RESP_OK;
  JSON_STRING_INIT();

  json_config = cJSON_Parse(config);
  if (!json_config)
    {
      conman_dbg("cJSON_Parse(properties) failed\n");
      return CONMAN_RESP_PARSE_FAIL;
    }

  json_connections = cJSON_GetObjectItem(json_config, "connections");
  if (!json_connections)
    {
      conman_dbg("no connections\n");
      err = CONMAN_RESP_NO_CONNECTIONS;
      goto errout_free;
    }

  number_of_connections = cJSON_GetArraySize(json_connections);

  for (j = 0; j < number_of_connections; j++)
    {
      json_connection = cJSON_GetArrayItem(json_connections, j);

      cons = cJSON_GetObjectItem(json_connection, "cellularConnections");
      if (cons)
        {
          n = cJSON_GetArraySize(cons);

          for (i = 0; i < n; i++)
            {
              cellu = calloc(sizeof(*cellu), 1);
              if (!cellu)
                {
                  conman_dbg("malloc failed\n");
                  err = CONMAN_RESP_OOM;
                  goto errout_free;
                }

              con = cJSON_GetArrayItem(cons, i);

              cellu->id = cJSON_int(cJSON_GetObjectItem(con, "connectionId"));

              JSON_STRDUP(cellu->access_point_name, con, "accessPointName");
              JSON_STRDUP(cellu->access_point_ipaddr, con, "accessPointIPAddr");
              JSON_STRDUP(cellu->proxy_address, con, "proxyAddress");
              JSON_STRDUP(cellu->proxy_port, con, "proxyPort");
              JSON_STRDUP(cellu->user_name, con, "userName");
              JSON_STRDUP(cellu->password, con, "password");
              JSON_STRDUP(cellu->mobile_country_code, con, "mobileCountryCode");
              JSON_STRDUP(cellu->mobile_network_code, con, "mobileNetworkCode");
              JSON_STRDUP(cellu->auth_type, con, "authType");
              JSON_STRDUP(cellu->pin, con, "pinCode");
              conman_dbg("PIN: %s\n", cellu->pin);

              entry = malloc(sizeof(*entry));
              if (!entry)
                {
                  conman_dbg("malloc failed\n");
                  free(cellu);
                  err = CONMAN_RESP_OOM;
                  goto errout_free;
                }

              entry->connection.cellular = cellu;
              sq_addlast(&entry->entry, &conman->connections.cellular);

              conman->connections.amount.cellular++;
              conman->connections.current.cellular = cellu->id; /* TODO */

              conman_dbg("APN: %s\n", cellu->access_point_name);
            }
        }

      cons = cJSON_GetObjectItem(json_connection, "wifiConnections");
      if (cons)
        {
          n = cJSON_GetArraySize(cons);

          for (i = 0; i < n; i++)
            {
              wifi = calloc(sizeof(*wifi), 1);
              if (!wifi)
                {
                  conman_dbg("malloc failed\n");
                  err = CONMAN_RESP_OOM;
                  goto errout_free;
                }

              con = cJSON_GetArrayItem(cons, i);

              wifi->id = cJSON_int(cJSON_GetObjectItem(con, "connectionId"));

              JSON_STRDUP(wifi->ssid, con, "ssid");
              JSON_STRDUP(wifi->password, con, "password");
              JSON_STRDUP(wifi->encryption, con, "encryption");

              entry = malloc(sizeof(*entry));
              if (!entry)
                {
                  conman_dbg("malloc failed\n");
                  free(wifi);
                  err = CONMAN_RESP_OOM;
                  goto errout_free;
                }

              entry->connection.wifi = wifi;
              sq_addlast(&entry->entry, &conman->connections.wifi);

              conman->connections.amount.wifi++;
              conman->connections.current.wifi = wifi->id; /* TODO */

              conman_dbg("SSID: %s\n", wifi->ssid);
            }
        }
    }

  free(json_config);

  return OK;

  errout_free:

  free(json_config);
  return err;
}

/****************************************************************************
 * Name: __conman_config_modem_config_cb
 *
 * Description:
 *   Callback function for modem library to query cellular configuration.
 *
 * Input Parameters:
 *   variable : variable for which we want a value
 *   buf      : buffer to store the return value
 *   buflen   : length of buf
 *   priv     : modem library clients (conman) private data
 *
 * Returned Value:
 *   true if value for variable is found, else false
 *
 ****************************************************************************/

bool __conman_config_modem_config_cb(struct ubmodem_s *modem,
    const char *variable, char *buf, size_t buflen, void *priv)
{
  struct conman_s *conman;
  const struct conman_ubmodem_config_offsets_s *offset;
  const struct conman_cellular_connection_s *connection;

  conman_dbg("\n");

  conman = (struct conman_s *) priv;

  connection = current_cellular_connection(conman);
  if (!connection)
    {
      conman_dbg("no current connection\n");
      return false;
    }

  offset = g_conman_ubmodem_config_offsets;

  while (offset->label)
    {
      if (strncmp(offset->label, variable, buflen) == 0)
        {
          char **str;

          str = (char **) ((char *) connection + offset->offset);

          if (*str)
            {
              strncpy(buf, *str, buflen);
              conman_dbg("%s: %s\n", variable, buf);
              return true;
            }
        }

      offset++;
    }

  conman_dbg("variable %s not found\n", variable);

  return false;
}

/****************************************************************************
 * Name: __conman_config_current_wifi_connection
 *
 * Description:
 *   Returns pointer to the current wifi connection configuration.
 *
 * Input Parameters:
 *   conman : connection manager handle
 *
 * Returned Value:
 *   Pointer to the current wifi connection configuration or NULL if
 *   none available or current id points to non existent configuration.
 *
 ****************************************************************************/

const struct conman_wifi_connection_s *
__conman_config_current_wifi_connection(struct conman_s *conman)
{
  return current_wifi_connection(conman);
}
