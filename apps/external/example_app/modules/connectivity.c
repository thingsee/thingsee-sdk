/****************************************************************************
 * modules/connectivity.c
 *
 * Copyright (C) 2015-2016 Haltian Ltd.
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

#include <time.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <apps/netutils/cJSON.h>
#include <apps/system/conman.h>
#include <apps/thingsee/ts_core.h>
#include <arch/board/board-reset.h>

#include <example_app_dbg.h>
#include <example_app_main.h>
#include <example_app_connectivity.h>

#define CONNECTION_RETRY_DELAY_SEC      1
#define CONNECTION_RETRY_DELAY_MAX_LOG2 7 /* log2(128 secs) */

#define CONMAN_WAKEUP_INTERVAL          2000

#ifndef MIN
#  define MIN(x,y) ((x) < (y) ? (x) : (y))
#endif

struct conman_event_priv_s
{
  enum conman_event_type_e event;
  size_t payloadlen;
  uint8_t payload[];
} packed_struct;

static void initial_connectivity_wait_handler(void);

static uint32_t g_connid = UINT32_MAX;
static int32_t g_contimer_id = -1;
static int g_retry_id = -1;
static struct conman_client_s g_conman_client;
static bool g_conn_request_waiting;

static int create_conman_config(char **config)
{
  cJSON *root = NULL, *connections = NULL, *connection = NULL,
      *links = NULL, *link = NULL;

  root = cJSON_CreateObject();
  if (!root)
    {
      exapp_dbg("cJSON_CreateObject failed\n");
      return ERROR;
    }

  connections = cJSON_CreateArray();
  if (!connections)
    {
      exapp_dbg("cJSON_CreateArray failed\n");
      goto errout_free;
    }

  connection = cJSON_CreateObject();
  if (!connection)
    {
      exapp_dbg("cJSON_CreateObject failed\n");
      goto errout_free;
    }

  links = cJSON_CreateArray();
  if (!links)
    {
      exapp_dbg("cJSON_CreateArray failed\n");
      goto errout_free;
    }

  link = cJSON_CreateObject();
  if (!link)
    {
      exapp_dbg("cJSON_CreateObject failed\n");
      goto errout_free;
    }

#ifdef CONFIG_SDK_EXAMPLE_APP_CONNECTION_MODEM
  cJSON_AddNumberToObject(link, "connectionId", 0);
  cJSON_AddStringToObject(link, "pinCode", CONFIG_SDK_EXAMPLE_APP_CONNECTION_SIM_PIN);
  cJSON_AddStringToObject(link, "accessPointName", CONFIG_SDK_EXAMPLE_APP_CONNECTION_SIM_APN_NAME);
  cJSON_AddStringToObject(link, "userName", CONFIG_SDK_EXAMPLE_APP_CONNECTION_SIM_APN_USER);
  cJSON_AddStringToObject(link, "password", CONFIG_SDK_EXAMPLE_APP_CONNECTION_SIM_APN_PASSWD);

  cJSON_AddItemToArray(links, link);
  cJSON_AddItemToObject(connection, "cellularConnections", links);
#elif defined CONFIG_SDK_EXAMPLE_APP_CONNECTION_WIFI
  cJSON_AddNumberToObject(link, "connectionId", 0);
  cJSON_AddStringToObject(link, "ssid", CONFIG_SDK_EXAMPLE_APP_CONNECTION_WIFI_SSID);
  cJSON_AddStringToObject(link, "password", CONFIG_SDK_EXAMPLE_APP_CONNECTION_WIFI_PSK);
  cJSON_AddStringToObject(link, "encryption", "wpa2");

  cJSON_AddItemToArray(links, link);
  cJSON_AddItemToObject(connection, "wifiConnections", links);
#else
# error "Connectivity misconfigured"
#endif

  cJSON_AddItemToArray(connections, connection);
  cJSON_AddItemToObject(root, "connections", connections);

  *config = cJSON_PrintUnformatted(root);

  cJSON_Delete(root); /* Deletes the whole tree */

  return OK;

errout_free:

  cJSON_Delete(root);
  cJSON_Delete(connections);
  cJSON_Delete(connection);
  cJSON_Delete(links);
  cJSON_Delete(link);

  return ERROR;
}

static int conman_client_pollfd(const struct pollfd * const pfd,
                                void * const priv)
{
  int ret;

  exapp_dbg("\n");

  DEBUGASSERT(pfd->fd == g_conman_client.sd);

  ret = conman_client_handle_events(&g_conman_client);
  if (ret < 0)
    {
      exapp_dbg("conman_client_handle_events failed\n");
      ret = ERROR;
    }

  if (g_conn_request_waiting)
    {
      initial_connectivity_wait_handler();
    }

  return OK;
}

static int retry_lost_connection(const int timer_id,
                                 const struct timespec *date,
                                 void * const priv)
{
  int ret;

  g_retry_id = -1;

  ret = __exapp_connectivity_poke_connection();
  DEBUGASSERT(ret == OK);

  return OK;
}

static cJSON *parse_cell_env(void *payload, size_t payloadlen)
{
  struct conman_event_cell_environment_s *cenv = payload;
  char buf[32];
  cJSON *root;
  cJSON *obj = NULL;
  cJSON *array = NULL;
  unsigned int i;

  if (payloadlen < sizeof(*cenv))
    return NULL;
  if (payloadlen != sizeof(*cenv)
                    + cenv->num_neighbors * sizeof(cenv->neighbors[0]))
    return NULL;

  root = cJSON_CreateObject();
  if (!root)
    return NULL;

  if (cenv->have_serving)
    {
      obj = cJSON_CreateNamedObject("serving");
      if (!obj)
        goto err_free;

      switch (cenv->serving.type)
        {
          case CONMAN_CELL_ENVIRONMENT_TYPE_GSM:
            cJSON_AddStringToObject(obj, "type", "GSM");
            break;
          case CONMAN_CELL_ENVIRONMENT_TYPE_UMTS:
            cJSON_AddStringToObject(obj, "type", "UMTS");
            break;
          default:
            cJSON_AddStringToObject(obj, "type", "unknown");
            break;
        }

      cJSON_AddNumberToObject(obj, "mcc", cenv->serving.mcc);
      cJSON_AddNumberToObject(obj, "mnc", cenv->serving.mnc);
      cJSON_AddNumberToObject(obj, "lac", cenv->serving.lac);

      snprintf(buf, sizeof(buf), "%X", cenv->serving.cell_id);
      cJSON_AddStringToObject(obj, "cellid", buf);

      if (cenv->serving.signal_dbm >= -256 && cenv->serving.signal_dbm < 0)
        cJSON_AddNumberToObject(obj, "signal_dbm", cenv->serving.signal_dbm);

      if (cenv->serving.type == CONMAN_CELL_ENVIRONMENT_TYPE_UMTS &&
          cenv->serving.sc != 0xffff)
        cJSON_AddNumberToObject(obj, "sc", cenv->serving.sc);

      if (cenv->serving.type == CONMAN_CELL_ENVIRONMENT_TYPE_GSM &&
          cenv->serving.bsic != 0xff)
        cJSON_AddNumberToObject(obj, "bsic", cenv->serving.bsic);

      if (cenv->serving.type == CONMAN_CELL_ENVIRONMENT_TYPE_GSM &&
          cenv->serving.arfcn != 0xffff)
        cJSON_AddNumberToObject(obj, "arfcn", cenv->serving.arfcn);

      cJSON_PackChild(obj);
      cJSON_AddNamedItemToObject(root, obj);
      obj = NULL;
    }

  if (cenv->have_signal_qual)
    {
      obj = cJSON_CreateNamedObject("signal_qual");
      if (!obj)
        goto err_free;

      if (cenv->signal_qual.rssi >= -256 && cenv->signal_qual.rssi < 0)
        cJSON_AddNumberToObject(obj, "rssi", cenv->signal_qual.rssi);

      if (cenv->signal_qual.qual >= 0)
        cJSON_AddNumberToObject(obj, "qual", cenv->signal_qual.qual);

      cJSON_PackChild(obj);
      cJSON_AddNamedItemToObject(root, obj);
      obj = NULL;
    }

  if (cenv->num_neighbors > 0)
    {
      array = cJSON_CreateNamedArray("neighbors");
      if (!array)
        goto err_free;

      for (i = 0; i < cenv->num_neighbors; i++)
        {
          obj = cJSON_CreateObject();
          if (!obj)
            goto err_free;

          switch (cenv->neighbors[i].type)
            {
              case CONMAN_CELL_ENVIRONMENT_TYPE_GSM:
                cJSON_AddStringToObject(obj, "type", "GSM");
                break;
              case CONMAN_CELL_ENVIRONMENT_TYPE_UMTS:
                cJSON_AddStringToObject(obj, "type", "UMTS");
                break;
              default:
                cJSON_AddStringToObject(obj, "type", "unknown");
                break;
            }

          if (cenv->neighbors[i].have_cellid)
            {
              snprintf(buf, sizeof(buf), "%X", cenv->neighbors[i].cell_id);
              cJSON_AddStringToObject(obj, "cellid", buf);
            }

          if (cenv->neighbors[i].have_mcc_mnc_lac)
            {
              cJSON_AddNumberToObject(obj, "mcc", cenv->neighbors[i].mcc);
              cJSON_AddNumberToObject(obj, "mnc", cenv->neighbors[i].mnc);
              cJSON_AddNumberToObject(obj, "lac", cenv->neighbors[i].lac);
            }

          if (cenv->neighbors[i].have_signal_dbm)
            {
              cJSON_AddNumberToObject(obj, "signal_dbm", cenv->neighbors[i].signal_dbm);
            }

          if (cenv->neighbors[i].have_bsic)
            {
              cJSON_AddNumberToObject(obj, "bsic", cenv->neighbors[i].bsic);
            }

          if (cenv->neighbors[i].have_arfcn)
            {
              cJSON_AddNumberToObject(obj, "arfcn", cenv->neighbors[i].arfcn);
            }

          if (cenv->neighbors[i].have_sc)
            {
              cJSON_AddNumberToObject(obj, "sc", cenv->neighbors[i].sc);
            }

          cJSON_PackChild(obj);
          cJSON_AddItemToArray(array, obj);
          obj = NULL;
        }

      cJSON_PackChild(array);
      cJSON_AddNamedItemToObject(root, array);
      array = NULL;
    }

  cJSON_PackChild(root);
  return root;

err_free:
  cJSON_Delete(obj);
  cJSON_Delete(array);
  cJSON_Delete(root);
  return NULL;
}

static cJSON *parse_wifi_scan(void *payload, size_t payloadlen)
{
  struct conman_event_wifi_scan_results_s *wscan = payload;
  char buf[33];
  cJSON *obj = NULL;
  cJSON *array;
  int i;

  if (payloadlen < sizeof(*wscan))
    return NULL;
  if (payloadlen != sizeof(*wscan)
                    + wscan->num_results * sizeof(wscan->results[0]))
    return NULL;

  array = cJSON_CreateArray();
  if (!array)
    goto err_free;

  for (i = 0; i < wscan->num_results; i++)
    {
      const uint8_t *bssid = wscan->results[i].bssid;

      obj = cJSON_CreateObject();
      if (!obj)
        goto err_free;

      snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
               bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
      cJSON_AddStringToObject(obj, "bssid", buf);

      snprintf(buf, sizeof(buf), "%.*s", wscan->results[i].ssid_len,
               wscan->results[i].ssid);
      cJSON_AddStringToObject(obj, "ssid", buf);

      cJSON_AddNumberToObject(obj, "rssi", wscan->results[i].rssi);

      cJSON_PackChild(obj);
      cJSON_AddItemToArray(array, obj);
      obj = NULL;
    }

  cJSON_PackChild(array);
  return array;

err_free:
  cJSON_Delete(obj);
  cJSON_Delete(array);
  return NULL;
}

static void handle_conman_events(enum conman_event_type_e event,
                                 void *payload, size_t payloadlen)
{
  /* Called from ts_core timer. */

  switch (event)
    {
      case CONMAN_EVENT_CELL_ENVIRONMENT:
        {
          cJSON *cell_env;

          exapp_dbg("%s event.\n", "CONMAN_EVENT_CELL_ENVIRONMENT");

          cell_env = parse_cell_env(payload, payloadlen);
          if (cell_env)
            {
              char *str = cJSON_Print(cell_env);

              exapp_dbg("\n%s\n", str);

              free(str);

              __exapp_main_report_cell_environment_results(cell_env);
            }
        }
        break;

      case CONMAN_EVENT_WIFI_SCAN:
        {
          cJSON *wifi_env;

          exapp_dbg("%s event.\n", "CONMAN_EVENT_WIFI_SCAN");

          wifi_env = parse_wifi_scan(payload, payloadlen);
          if (wifi_env)
            {
              char *str = cJSON_Print(wifi_env);

              exapp_dbg("\n%s\n", str);

              free(str);

              __exapp_main_report_wifi_scan_results(wifi_env);
            }
        }
        break;

      case CONMAN_EVENT_LOST_CONNECTION:
      case CONMAN_EVENT_CONNECTION_REQUEST_FAILED:
        {
          exapp_dbg("%s event.\n",
                 event == CONMAN_EVENT_LOST_CONNECTION ?
                     "CONMAN_EVENT_LOST_CONNECTION" :
                     "CONMAN_EVENT_CONNECTION_REQUEST_FAILED");

          if (g_retry_id < 0)
            {
              struct timespec ts;

              /* Lost connection. Poke conman to retry connection. */

              clock_gettime(CLOCK_MONOTONIC, &ts);
              ts.tv_sec += 1;

              g_retry_id = ts_core_timer_setup_date(&ts, retry_lost_connection,
                                                    NULL);
              DEBUGASSERT(g_retry_id >= 0);
            }
        }
        break;

      case CONMAN_EVENT_HW_ERROR_TOO_LOW_VOLTAGE:
        {
          exapp_dbg("%s event.\n", "CONMAN_EVENT_HW_ERROR_TOO_LOW_VOLTAGE");

          /* Modem cannot start-up, too low voltage level (battery
           * below ~3.3-3.35V). */

          if (g_conn_request_waiting)
            {
              /* At initial connection creation stage, main event loop not
               * started/running yet. Choose quick path to stand-by. */

              board_go_to_standby();
            }
        }
        break;

      default:
        break;
    }
}

static int handle_deferred_conman_event(const int timer_id,
                                        const struct timespec *date,
                                        void * const arg)
{
  struct conman_event_priv_s *priv = arg;

  handle_conman_events(priv->event, priv->payload, priv->payloadlen);

  free(priv);

  return OK;
}

static void conman_events_cb(struct conman_client_s *client,
                             enum conman_event_type_e event,
                             void *payload, size_t payloadlen, void *arg)
{
  struct conman_event_priv_s *priv;
  struct timespec ts;
  int id;

  /* Called from conman_client_handle_events() or conman command response
   * handler (if event received while waiting for command response).
   */

  DEBUGASSERT(client == &g_conman_client);

  /* Events from conman command repsonse handler might be in
   * main.c:switch_state context. We must not handle events in this
   * context, so defer event handling with ts_core timer. */

  priv = calloc(1, sizeof(*priv) + payloadlen);
  if (!priv)
    {
      exapp_dbg("could not allocate %d bytes\n", sizeof(*priv) + payloadlen);
      return;
    }

  priv->event = event;
  priv->payloadlen = payloadlen;
  if (payloadlen)
    {
      memcpy(priv->payload, payload, payloadlen);
    }

  clock_gettime(CLOCK_MONOTONIC, &ts);

  id = ts_core_timer_setup_date(&ts, handle_deferred_conman_event, priv);
  DEBUGASSERT(id >= 0);
}

int __exapp_connectivity_request(void)
{
  int ret;

  if (g_connid != UINT32_MAX)
    {
      if (g_contimer_id >= 0)
        {
          ts_core_timer_stop(g_contimer_id);
          g_contimer_id = -1;
        }
      return OK;
    }

  ret = conman_client_request_connection(&g_conman_client, CONMAN_DATA, &g_connid);
  if (ret < 0)
    {
      exapp_dbg("conman_client_request_connection failed\n");
    }

  return ret;
}

int __exapp_connectivity_initial_request(void)
{
  int ret;

  g_conn_request_waiting = true;

  ret = __exapp_connectivity_request();
  if (ret < 0)
    {
      g_conn_request_waiting = false;
      return ret;
    }

  initial_connectivity_wait_handler();

  return OK;
}

static void initial_connectivity_wait_handler(void)
{
  struct conman_status_s status;
  int ret;

  ret = conman_client_get_connection_status(&g_conman_client, &status);
  if (ret < 0)
    {
      exapp_dbg("conman_client_get_connection_status failed\n");
      return;
    }

  exapp_dbg("status: %d\n", status.status);

  if (status.status == CONMAN_STATUS_ESTABLISHED)
    {
      g_conn_request_waiting = false;

      __exapp_main_initial_connectivity_reached();

      return;
    }

  if (status.status == CONMAN_STATUS_OFF)
    {
      if (conman_client_destroy_connection(&g_conman_client, g_connid) < 0)
        {
          exapp_dbg("conman_client_destroy_connection failed\n");
        }

      ret = conman_client_request_connection(&g_conman_client, CONMAN_DATA, &g_connid);
      if (ret < 0)
        {
          exapp_dbg("conman_client_request_connection failed\n");
          return;
        }
    }
  else if (g_retry_id >= 0)
    {
      /* If g_retry_id is not -1, then connection failed. Poke connection. */

      __exapp_connectivity_poke_connection();
    }

  if (g_retry_id >= 0)
    {
      ts_core_timer_stop(g_retry_id);
      g_retry_id = -1;
    }

  exapp_dbg("wait conman event\n");
}

int __exapp_connectivity_poke_connection(void)
{
  uint32_t new_connid = CONMAN_CONNID_CLEAR;
  int ret;

  exapp_dbg("\n");

  /* Open 'new' connection. */

  ret = conman_client_request_connection(&g_conman_client, CONMAN_DATA, &new_connid);
  if (ret < 0)
    {
      exapp_dbg("conman_client_request_connection failed\n");
      ret = ERROR;
      goto out;
    }

  if (g_connid != UINT32_MAX)
    {
      /* Free previous instance. */

      ret = conman_client_destroy_connection(&g_conman_client, g_connid);
      if (ret < 0)
        {
          exapp_dbg("conman_client_destroy_connection failed\n");
          ret = ERROR;
          goto out;
        }

      g_connid = UINT32_MAX;
    }

  /* Store active connection ID. */

  g_connid = new_connid;
  ret = OK;

out:
  return ret;
}

static int con_timer_cb(const int timer_id,
                        const struct timespec *date, void * const priv)
{
  int ret;

  g_contimer_id = -1;

  if (g_connid == UINT32_MAX)
    {
      return OK;
    }

  ret = conman_client_destroy_connection(&g_conman_client, g_connid);
  if (ret < 0)
    {
      exapp_dbg("conman_client_destroy_connection failed\n");
      return ERROR;
    }

  g_connid = UINT32_MAX;

  return OK;
}

int __exapp_connectivity_destroy(uint32_t delay_s)
{
  struct timespec ts;

  if (g_connid == UINT32_MAX)
    {
      return OK;
    }

  if (delay_s >= 0)
    {
      if (g_contimer_id >= 0)
        {
          ts_core_timer_stop(g_contimer_id);
        }

      clock_gettime(CLOCK_MONOTONIC, &ts);
      ts.tv_sec += delay_s;
      g_contimer_id = ts_core_timer_setup_date(&ts, con_timer_cb, NULL);
    }
  else
    {
      con_timer_cb(-1, &ts, NULL);
    }

  return OK;
}

int __exapp_connectivity_init(void)
{
  int ret = OK;
  char *config;

  ret = create_conman_config(&config);
  if (ret < 0)
    {
      exapp_dbg("create_config failed\n");
      return ERROR;
    }

  ret = conman_client_init_events(&g_conman_client, conman_events_cb, NULL);
  if (ret != OK)
    {
      exapp_dbg("conman_client_init_events failed\n");
      return ERROR;
    }

  ret = conman_client_set_connections_config(&g_conman_client, config);
  free(config);
  if (ret != OK)
    {
      exapp_dbg("conman_client_set_connections_config failed\n");
      return ERROR;
    }

  ret = ts_core_fd_register(g_conman_client.sd, POLLIN, conman_client_pollfd,
                            NULL);
  if (ret != OK)
    {
      exapp_dbg("ts_core_fd_register failed\n");
      return ERROR;
    }

  return ret;
}

int __exapp_connectivity_uninit(void)
{
  int ret;

  ret = ts_core_fd_unregister(g_conman_client.sd);
  if (ret < 0)
    {
      exapp_dbg("ts_core_fd_unregister failed\n");
    }

  if (g_connid != UINT32_MAX)
    {
      ret = con_timer_cb(-1, NULL, NULL);
      if (ret < 0)
        {
          exapp_dbg("con_timer_cb failed\n");
        }
    }

  exapp_dbg(">> Destroy all connections...\n");

  ret = conman_client_destroy_connection(&g_conman_client, CONMAN_CONNID_ALL);
  if (ret != OK)
    {
      exapp_dbg("conman_client_destroy_connection(CONMAN_CONNID_ALL) failed\n");
    }

  exapp_dbg(">> Done...\n");

  conman_client_uninit(&g_conman_client);

  return ret;
}

int __exapp_connectivity_send_sms(const struct exapp_sms_s * const sms)
{
  int ret;

  ret = conman_client_send_sms(&g_conman_client, sms->number, sms->content);
  if (ret != OK)
    {
      exapp_dbg("conman_client_send_sms failed\n");
    }

  return ret;
}

int __exapp_connectivity_get_imei(char **imei)
{
  struct conman_status_s connection_status;
  int ret;

  ret = conman_client_get_connection_status(&g_conman_client, &connection_status);
  if (ret != OK)
    {
      exapp_dbg("conman_client_get_connection_status failed\n");
    }
  else
    {
      *imei = strdup(connection_status.info.cellu.imei);
    }

  return ret;
}

int __exapp_connectivity_request_cell_environment(void)
{
  int ret;

  /* Request cell environment information. Data is received with conman
   * event (see: handle_conman_events/CONMAN_EVENT_CELL_ENVIRONMENT). */

  ret = conman_client_request_cell_environment(&g_conman_client);
  if (ret != OK)
    {
      exapp_dbg("conman_client_request_cell_environment failed\n");
    }

  return ret;
}

int __exapp_connectivity_wifi_scan(void)
{
  int ret;

  /* Request cell environment information. Data is received with conman
   * event (see: handle_conman_events/CONMAN_EVENT_WIFI_SCAN).
   *
   * WiFi will scan for <scan_max_secs> and then power-off. If <scan_max_secs>
   * is negative, wifi will keep scanning until this command is reissued with
   * <scan_max_secs> being zero or more. Giving <scan_max_secs> zero will
   * stop scanning immediately.
   */

  ret = conman_client_wifi_scan(&g_conman_client);
  if (ret != OK)
    {
      exapp_dbg("conman_client_wifi_scan failed\n");
    }

  return ret;
}
