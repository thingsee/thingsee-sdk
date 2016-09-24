/****************************************************************************
 * app/example_app_main.c
 *
 * Copyright (C) 2016 Haltian Ltd.
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
 *   Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/version.h>
#include <nuttx/sensors/lis2dh.h>
#include <arch/board/board.h>
#include <arch/board/board-reset.h>
#include <arch/board/board-battery.h>
#include <arch/board/board-eeprom.h>

#include <stdio.h>
#include <stddef.h>
#include <sys/types.h>
#include <stdlib.h>
#include <debug.h>
#include <assert.h>
#include <sys/stat.h>
#include <string.h>
#include <fixedmath.h>
#include <errno.h>

#include <apps/thingsee/ts_core.h>
#ifdef CONFIG_NETUTILS_NTPCLIENT
#include <apps/netutils/ntpclient.h>
#endif

#include <example_app_dbg.h>
#include <example_app_main.h>
#include <example_app_hwwdg.h>
#include <example_app_charger.h>
#include <example_app_connectivity.h>
#include <example_app_connector.h>
#include <example_app_accelerometer.h>
#include <example_app_gps.h>


static struct accelerometer_config accel_config =
{
  .impact_threshold = CONFIG_SDK_EXAMPLE_APP_ACCEL_IMPACT_THRESHOLD / 1000.0,
  .restart_time = CONFIG_SDK_EXAMPLE_APP_ACCEL_RESTART_TIME,
};

static struct gps_config_s gps_config =
{
  .rate = 10000,
};

static struct example_app_s g_example_app;


static void __exapp_do_radio_environment_query(void);
static void __exapp_flush_radio_environment_to_cloud(bool force);


int __exapp_main_report_cell_environment_results(cJSON *cell_env)
{
  if (g_example_app.data.cell_environment)
    {
      cJSON_Delete(g_example_app.data.cell_environment);
      g_example_app.data.cell_environment = NULL;
    }

  g_example_app.data.cell_environment = cell_env;

  /* Check if all data ready and send to cloud if is. */

  __exapp_flush_radio_environment_to_cloud(false);

  return OK;
}

int __exapp_main_report_wifi_scan_results(cJSON *wifi_networks)
{
  if (g_example_app.data.wifi_scan)
    {
      cJSON_Delete(g_example_app.data.wifi_scan);
      g_example_app.data.wifi_scan = NULL;
    }

  g_example_app.data.wifi_scan = wifi_networks;

  /* Check if all data ready and send to cloud if is. */

  __exapp_flush_radio_environment_to_cloud(false);

  return OK;
}

static char *format_headers(const char *raw_headers)
{
  char *headers;
  char *pc;
  size_t len = strlen(raw_headers);
  size_t pos;

  headers = strdup(raw_headers);
  while (headers)
    {
      pc = strstr(headers, "\\r");
      if (pc)
        {
          *pc = '\r';
        }
      else
        {
          pc = strstr(headers, "\\n");
          if (pc)
            {
              *pc = '\n';
            }
        }

      if (!pc)
        {
          break;
        }

      pos = pc - headers;
      memmove(pc + 1, pc + 2, len - (pos + 2) + 1);
      len -= 1;
    }

  return headers;
}

static void exapp_prepare_config_headers(void)
{
  if (!g_example_app.config_headers)
    {
      /* "\\n" => "\n" and "\\r" => "\r" */

      g_example_app.config_headers = format_headers(
          CONFIG_SDK_EXAMPLE_APP_HTTP_CLOUD_EXTRA_HEADER
          "Content-type: json/text\\r\\n");

      if (!g_example_app.config_headers)
        {
          g_example_app.config_headers = "";
        }
    }
}

static void __exapp_flush_radio_environment_to_cloud(bool force)
{
  size_t payload_len;
  char *payload;
  cJSON *root;

  if (!force)
    {
      if (!g_example_app.data.cell_environment)
        return; /* No cell-env data yet */
      if (!g_example_app.data.wifi_scan)
        return; /* No wifi-scan data yet */
    }
  else
    {
      if (!g_example_app.data.cell_environment &&
          !g_example_app.data.wifi_scan)
        return; /* No data */
    }

  /* Create master JSON object. */

  root = cJSON_CreateObject();
  if (!root)
    {
      return; /* OOM */
    }

  /* Add child objects to master JSON. */

  if (g_example_app.data.cell_environment)
    {
      cJSON_AddItemToObject(root, "cellular_networks",
                            g_example_app.data.cell_environment);
      g_example_app.data.cell_environment = NULL;
    }

  if (g_example_app.data.wifi_scan)
    {
      cJSON_AddItemToObject(root, "wifi_networks",
                            g_example_app.data.wifi_scan);
      g_example_app.data.wifi_scan = NULL;
    }

  /* Convert JSON object to cstring. */

  payload = cJSON_PrintUnformatted(root);
  if (!payload)
    {
      /* OOM */
      cJSON_Delete(root);
      return;
    }

  cJSON_Delete(root);
  payload_len = strlen(payload);

  exapp_prepare_config_headers();

  /* Send payload to cloud over HTTPS/HTTP REST. Connector takes ownership
   * of malloced 'payload' pointer if returned success. */

  if (__exapp_connector_send_https(CONFIG_SDK_EXAMPLE_APP_HTTP_CLOUD_URL,
                                   CONFIG_SDK_EXAMPLE_APP_HTTP_CLOUD_METHOD,
                                   g_example_app.config_headers,
                                   payload, payload_len,
                                   NULL) != OK)
    {
      /* Failed, free payload. */
      free(payload);
    }
}

static int radio_env_timer_cb(const int timer_id, const struct timespec *date,
                              void * const priv)
{
  g_example_app.radio_env_timer = -1; /* one-shot */

  /* Send collected radio environment data. */

  __exapp_flush_radio_environment_to_cloud(true);

  return OK;
}

static void __exapp_do_radio_environment_query(void)
{
  struct timespec ts;
  int ret;

  if (g_example_app.radio_env_timer >= 0)
    {
      return; /* Already waiting for results. */
    }

  /* Send previous radio environment data. */

  __exapp_flush_radio_environment_to_cloud(true);

  /* Request cell environment information. */

  ret = __exapp_connectivity_request_cell_environment();
  if (ret != OK)
    {
      exapp_dbg("cellular environment information could not be requested!\n");
    }

  /* Launch WiFi scan. */

  ret = __exapp_connectivity_wifi_scan();
  if (ret != OK)
    {
      exapp_dbg("WiFi scanning could not be started!\n");
    }

  /* Start radio enviroment query interval timer.  */

  (void)clock_gettime(CLOCK_MONOTONIC, &ts);
  ts.tv_sec += CONFIG_SDK_EXAMPLE_APP_RADIO_ENVIRONMENT_INTERVAL;

  g_example_app.radio_env_timer =
      ts_core_timer_setup_date(&ts, radio_env_timer_cb, NULL);
  DEBUGASSERT(g_example_app.radio_env_timer >= 0);
}

static void accel_event_cb(struct lis2dh_result *result, void *priv)
{
  static const struct {
    unsigned int bit;
    const char *name;
  } int_source_names[] = {
    { ST_LIS2DH_INT_SR_XHIGH, "x high" },
    { ST_LIS2DH_INT_SR_XLOW, "x low" },
    { ST_LIS2DH_INT_SR_YHIGH, "y high" },
    { ST_LIS2DH_INT_SR_YLOW, "y low" },
    { ST_LIS2DH_INT_SR_ZHIGH, "z high" },
    { ST_LIS2DH_INT_SR_ZLOW, "z low" },
    { 0, NULL }
  };
  const uint8_t int_sources[2] = {
    result->header.int1_source,
    result->header.int2_source
  };
  int i, j;

  exapp_dbg("x: %.3f\n", ((double)result->measurements[0].x) / 1000);
  exapp_dbg("y: %.3f\n", ((double)result->measurements[0].y) / 1000);
  exapp_dbg("z: %.3f\n", ((double)result->measurements[0].z) / 1000);
  exapp_dbg("int1_occurred: 0x%08x\n", result->header.int1_occurred);
  exapp_dbg("int2_occurred: 0x%08x\n", result->header.int2_occurred);
  exapp_dbg("int1_source: 0x%08x\n", result->header.int1_source);
  exapp_dbg("int2_source: 0x%08x\n", result->header.int2_source);

  for (i = 0; i < 2; i++)
    {
      for (j = 0; int_source_names[j].bit; j++)
        {
          if (int_sources[i] & int_source_names[j].bit)
            {
              exapp_dbg("%d: %s\n", i + 1, int_source_names[j].name);
            }
        }
    }

  /* Launch cell-environment & wifi scan. */

  __exapp_do_radio_environment_query();
}

static void exapp_gps_location_to_cloud(cJSON *root)
{
  size_t payload_len;
  char *payload;

  /* Convert JSON object to cstring. */

  payload = cJSON_PrintUnformatted(root);
  if (!payload)
    {
      /* OOM */
      cJSON_Delete(root);
      return;
    }

  cJSON_Delete(root);
  payload_len = strlen(payload);

  exapp_prepare_config_headers();

  /* Send payload to cloud over HTTPS/HTTP REST. Connector takes ownership
   * of malloced 'payload' pointer if returned success. */

  if (__exapp_connector_send_https(CONFIG_SDK_EXAMPLE_APP_HTTP_CLOUD_URL,
                                   CONFIG_SDK_EXAMPLE_APP_HTTP_CLOUD_METHOD,
                                   g_example_app.config_headers,
                                   payload, payload_len,
                                   NULL) != OK)
    {
      /* Failed, free payload. */
      free(payload);
    }
}

static cJSON *exapp_parse_location(struct location_s *location)
{
  cJSON *obj = NULL;

  obj = cJSON_CreateObject();
  if (!obj)
    goto err_free;

  cJSON_AddNumberToObject(obj, "time", location->time);

  cJSON_AddNumberToObject(obj, "lat", location->latitude);
  cJSON_AddNumberToObject(obj, "long", location->longitude);
  cJSON_AddNumberToObject(obj, "alt", location->altitude);

  cJSON_AddNumberToObject(obj, "acc", location->accuracy);

  cJSON_AddNumberToObject(obj, "spd", location->speed);
  cJSON_AddNumberToObject(obj, "head", location->heading);

  return obj;

err_free:
  cJSON_Delete(obj);
  return NULL;
}

static int exapp_gps_callback(struct location_s *location, void *priv)
{
  cJSON *location_json;

  if (location && location->valid)
    {
      /* GPS fix acquired. */

      location_json = exapp_parse_location(location);
      if (location_json)
        {
          exapp_gps_location_to_cloud(location_json);
        }

      g_example_app.gps_got_fix = true;
    }
  else
    {
      /* GPS fix timeout. */

      g_example_app.gps_got_fix = false;
    }

  g_example_app.gps_started = false;
  exapp_gps_uninit();

  return OK;
}

static int exapp_gps_relaunch_timer_cb(const int timer_id,
                                       const struct timespec *date,
                                       void * const priv)
{
  struct timespec ts;

  g_example_app.gps_relaunch_timer = -1; /* one-shot */

  /* Start GPS module */

  if (!g_example_app.gps_started)
    {
      if (exapp_gps_init(&gps_config, exapp_gps_callback, NULL,
                         !g_example_app.gps_got_fix) == OK)
        {
          g_example_app.gps_started = true;
        }
    }

  /* Re-register timer. */

  (void)clock_gettime(CLOCK_MONOTONIC, &ts);
  ts.tv_sec += CONFIG_SDK_EXAMPLE_APP_GPS_LOCATION_INTERVAL;

  g_example_app.gps_relaunch_timer =
      ts_core_timer_setup_date(&ts, exapp_gps_relaunch_timer_cb, NULL);
  DEBUGASSERT(g_example_app.gps_relaunch_timer >= 0);

  return OK;
}

void __exapp_main_initial_connectivity_reached(void)
{
  char *imei;
  int ret;

  /* This function is called after __exapp_connectivity_initial_request()
   * succeeds opening network connection. */

  if (__exapp_connectivity_get_imei(&imei) == OK)
    {
      exapp_dbg("Modem IMEI: %s\n", imei);
      free(imei);
    }

#ifdef CONFIG_NETUTILS_NTPCLIENT
  /* At this phase, we have initial connection open and we can fetch NTP
   * time. */

  (void)ntpc_start();
#endif

  /* Initialize HTTPS-REST connector module. */

  ret = __exapp_connector_init();
  DEBUGASSERT(ret != ERROR);

  /* Start accelerometer. Do radio environment query when detect movement. */

  ret = exapp_accelerometer_init(&accel_config, accel_event_cb, NULL);
  DEBUGASSERT(ret != ERROR);

  /* Start GPS timer. Get GPS location every 5 minutes. */

  (void)exapp_gps_relaunch_timer_cb(-1, NULL, NULL);
}

static int reset_assert_count(const int timer_id,
                              const struct timespec *date, void * const priv)
{
  /* Reset assert counter. */

  board_reset_assert_count();

  exapp_dbg ("assert count reset.\n");

  return OK;
}

static void __exapp_start_reset_assert_count_timer(void)
{
  struct timespec ts = {};
  int id;
  int ret;

  /* Delayed reset assert boot counter in MCU backup register.
   * Controls entry to USB-DFU mode (allow device recovery after boot-assert
   * loops). */

  ret = clock_gettime(CLOCK_MONOTONIC, &ts);
  DEBUGASSERT(ret >= 0);

  /* Reset assert counter after device has been on for sometime. */

  ts.tv_sec += CONFIG_SDK_EXAMPLE_APP_ASSERT_COUNT_RESET_SECS;

  /* Launch timer */

  id = ts_core_timer_setup_date(&ts, reset_assert_count, NULL);
  DEBUGASSERT(id >= 0);
}

static int __exapp_init_system(struct example_app_s *app)
{
  int ret;

  exapp_dbg("Initializing Thingsee core library\n");

  ret = ts_core_initialize();
  DEBUGASSERT(ret == OK);

  /* Print build version */

  exapp_dbg("FW build version: %s\n", CONFIG_VERSION_BUILD);

  /* Start hardware watchdog driver. */

  ret = exapp_hwwdg_initialize();
  DEBUGASSERT(ret == OK);

  /* Start charger module. */

  ret = exapp_charger_initialize();
  DEBUGASSERT(ret == OK);

  /* Reset assert boot counter in MCU backup register. Controls entry to
   * USB-DFU mode (allow device recovery after boot-assert loop). */

  __exapp_start_reset_assert_count_timer();

  return OK;
}

int example_app_main(int argc, char *argv[])
{
  int ret;
  bool loop_go_on = true;

  memset(&g_example_app, 0, sizeof(g_example_app));
  g_example_app.radio_env_timer = -1;
  g_example_app.gps_relaunch_timer = -1;

  /* Initialize system and setup required background modules (ie. charging,
   * watchdog). */

  ret = __exapp_init_system(&g_example_app);
  if (ret != OK)
    {
      exapp_dbg("init_system failed\n");

      /* Should not happen. Debug assert? Power-off? */

      return EXIT_FAILURE;
    }

  /* Initialize connectivity module (ie. configure conman and connector). */

  ret =__exapp_connectivity_init();
  if (ret != OK)
    {
      exapp_dbg("__exapp_connectivity_init failed\n");

      return EXIT_FAILURE;
    }

#if 1
  /* Launch initial connection for NTP, etc. */

  ret = __exapp_connectivity_initial_request();
  if (ret != OK)
    {
      exapp_dbg("__exapp_connectivity_initial_request failed\n");

      return EXIT_FAILURE;
    }
#endif

  exapp_dbg("Starting main event loop\n");

  ts_core_mainloop(&loop_go_on);

  /* This is never reached. */

  return EXIT_SUCCESS;
}
