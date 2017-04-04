/****************************************************************************
 * apps/ts_engine/engine/main.c
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
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/version.h>

#include <fcntl.h>
#include <stdio.h>
#include <stddef.h>
#include <sys/types.h>
#include <stdlib.h>
#include <debug.h>
#include <assert.h>
#include <sys/stat.h>
#include <string.h>

#include <apps/netutils/cJSON.h>
#ifdef CONFIG_NETUTILS_NTPCLIENT
#include <apps/netutils/ntpclient.h>
#endif
#include <apps/thingsee/ts_core.h>

#ifndef CONFIG_ARCH_SIM
#include <apps/system/conman.h>
#include "../../thingsee/charger/bq24251_module.h"
#include "../../system/display/oled_display.h" /* TODO */
#endif

#include "eng_dbg.h"
#include "parse.h"
#include "main.h"
#include "execute.h"
#include "connector.h"
#include "sense.h"
#include "device_property.h"
#include "cloud_property.h"
#include "util.h"
#include "client.h"
#include "eng_error.h"
#include "shutdown.h"
#include "log.h"
#include "time_from_file.h"
#include "system_config.h"

#ifndef CONFIG_ARCH_SIM
#include <apps/ts_engine/watchdog.h>
#endif

#ifdef CONFIG_ARCH_SIM
#include "sense_sim.h"
#endif

#include <arch/board/board.h>
#include <arch/board/board-reset.h>
#include <arch/board/board-eeprom.h>
#ifdef CONFIG_SYSTEM_PROPERTIES_INT
#  include "../../thingsee/system_properties/sys_prop.h"
#endif

#ifndef CONFIG_ARCH_SIM

#define SDCARD_PROFILE_FILENAME          TS_EMMC_MOUNT_PATH "/profile.jsn"

#ifdef CONFIG_THINGSEE_UI
static const char g_mass_storage_str[] = "Mass Storage";
#endif

#endif


pid_t g_engine_pid;
static struct ts_engine_app g_app_s;


extern const unsigned char profile_jsn_tmp[];

#ifdef CONFIG_THINGSEE_UI
static void
engine_system_shutdown(const char *reset_type)
{
  enum ts_engine_shutdown_type_e type;

  if (strcmp(reset_type, "standby") == 0)
    {
      type = TS_ENGINE_SYSTEM_SHUTDOWN_TYPE_STANDBY;
    }
  else if (strcmp(reset_type, "reset") == 0)
    {
      type = TS_ENGINE_SYSTEM_SHUTDOWN_TYPE_RESET;
    }
  else if (strcmp(reset_type, "dfu") == 0)
    {
      type = TS_ENGINE_SYSTEM_SHUTDOWN_TYPE_DFU;
    }
  else
    {
      DEBUGASSERT(false);
    }

  ts_engine_do_system_shutdown(&g_app_s, type);

}
#endif

int ts_engine_stop(struct ts_engine_app * const app, bool free, bool uninit_connector)
{
  int ret;

  if (!app->engine)
    {
      eng_dbg("engine not started\n");
      return -TS_ENGINE_ERROR_NOT_STARTED;
    }

  ret = profile_stop(app->engine, free, uninit_connector);
  if (ret < 0)
    {
      eng_dbg("profile_stop failed\n");
    }

  if (app->handle)
    {
      (void)__ts_engine_log_stop(app->handle);
    }

  __ts_engine_log_close_all();

  if (free)
    {
      profile_free(app->engine->profile);
      app->engine = NULL;
      eng_dbg("profile freed\n");
    }

  eng_dbg("engine %s\n", (free ? "stopped" : "paused"));

  return OK;
}

int ts_engine_continue(const struct ts_engine_app *app)
{
  int ret;

  if (!app->engine)
    {
      eng_dbg("engine not started\n");
      return -TS_ENGINE_ERROR_NOT_STARTED;
    }

  ret = profile_continue(app->engine);
  if (ret < 0)
    {
      eng_dbg("profile_continue failed\n");
      return ret;
    }

  eng_dbg("engine continued\n");

  return OK;
}

static void seed_urandom(const void *buf, size_t buflen)
{
  int fd;

  fd = open("/dev/urandom", O_WRONLY);
  if (fd < 0)
    fd = open("/dev/random", O_WRONLY);

  if (fd >= 0)
    {
      (void)write(fd, buf, buflen);
      close(fd);
    }
}

static int
profile_write(char * const profile_json, size_t len)
{
  struct ts_profile *profile;
  int errcode;
  int ret;

  /* Parse the profile for validity */

  profile = profile_parse(profile_json, &errcode);
  if (!profile)
    {
      eng_dbg("profile_parse failed: %d\n", errcode);
      return errcode;
    }

  profile_free(profile);

  ret = __ts_engine_sdcard_write(SDCARD_PROFILE_FILENAME, profile_json, len, false);
  if (ret == OK)
    {
      return OK;
    }

#ifndef CONFIG_THINGSEE_ENGINE_EEPROM
  return ERROR;
#else
  return __ts_engine_eeprom_write(BOARD_EEPROM_SECTION_ENGINE_PROFILE, profile_json, len);
#endif
}

static const char *
profile_read(bool *ref)
{
#ifdef CONFIG_ARCH_SIM
  size_t len;

  *ref = true;
  return engine_sense_sim_get_profile(&len);
#else
  const char *profile;

  profile = __ts_engine_sdcard_read(SDCARD_PROFILE_FILENAME);
  if (profile)
    {
      *ref = false;
      return profile;
    }

#ifndef CONFIG_THINGSEE_ENGINE_EEPROM
  profile = NULL;
#else
  *ref = true;
  profile = __ts_engine_eeprom_read(BOARD_EEPROM_SECTION_ENGINE_PROFILE);
#endif

  if (!profile)
    {
      /* No profile found, use builtin default */

      *ref = true;
      profile = (const char *)profile_jsn_tmp;
    }

  return profile;
#endif
}

static int
engine_exec_profile(struct ts_engine_app * const app, const char *profile_json)
{
  struct ts_profile *profile;
  int errcode;

  eng_dbg("Parsing profile\n");

  profile = profile_parse(profile_json, &errcode);
  if (!profile)
    {
      eng_dbg("profile_parse failed: %d\n", errcode);
      return errcode;
    }

  eng_dbg("Starting profile\n");

  app->engine = profile_main(app, profile);
  if (!app->engine)
    {
      eng_dbg("profile_main failed\n");
      profile_free(profile);
      return -TS_ENGINE_ERROR_RUNTIME;
    }

  return OK;
}

static int ts_engine_start(struct ts_engine_app * const app)
{
  const char * profile_json;
  bool ref;
  int ret;

  profile_json = profile_read(&ref);
  if (!profile_json)
    {
      eng_dbg("profile_read failed\n");
      return -TS_ENGINE_ERROR_NO_PROFILE;
    }

  ret = engine_exec_profile(app, profile_json);
  if (ret < 0)
    {
      eng_dbg("engine_exec_profile failed\n");
    }

  if (!ref)
    {
      free((void *) profile_json);
    }

  eng_dbg("engine started\n");

  return ret;
}

static int
engine_ctrl_cb(const struct pollfd * const pfd, void * const arg)
{
  struct ts_engine_app *app = arg;
  struct ts_engine_client_req req = {};
  struct ts_engine_client_resp resp = {};
  char *datain = NULL;
  const char *dataout = NULL;
  bool refdataout = true;
  int ret;

  ret = __ts_engine_full_read(app->fdin, &req, sizeof(req));
  if (ret != sizeof(req))
    {
      eng_dbg("__ts_engine_full_read(req) failed\n");
      return ERROR;
    }

  resp.hdr.id = req.hdr.id;
  resp.hdr.status = -TS_ENGINE_ERROR_NONE;
  resp.hdr.length = 0;

  if (req.hdr.length)
    {
      datain = malloc(req.hdr.length);
      if (!datain)
        {
          resp.hdr.status = -TS_ENGINE_ERROR_OOM;
          goto out;
        }

      ret = __ts_engine_full_read(app->fdin, datain, req.hdr.length);
      if (ret != req.hdr.length)
        {
          eng_dbg("__ts_engine_full_read(datain) failed\n");
          resp.hdr.status = -TS_ENGINE_ERROR_READ;
          goto out;
        }
    }

  switch (req.hdr.id)
    {
    case TS_ENGINE_READ_DEVICE_PROPERTY:
      {
        ret = device_property_get(&dataout);
        if (ret != OK)
          {
            eng_dbg("device_property_get failed\n");
            resp.hdr.status = ret;
            goto out;
          }

        refdataout = false;

        resp.hdr.length = strlen(dataout) + 1; /* include NULL termination */
      }
      break;

    case TS_ENGINE_READ_CLOUD_PROPERTY:
      {
        dataout = cloud_property_read(&refdataout);
        if (!dataout)
          {
            eng_dbg("cloud_properties_read failed\n");
            resp.hdr.status = -TS_ENGINE_ERROR_NO_CLOUD_PROPERTIES;
            goto out;
          }

         resp.hdr.length = strlen(dataout) + 1;
      }
      break;

    case TS_ENGINE_WRITE_CLOUD_PROPERTY:
      {
        ret = cloud_property_write(datain, req.hdr.length);
        if (ret != OK)
          {
            eng_dbg("cloud_properties_write failed\n");
            resp.hdr.status = -TS_ENGINE_ERROR_STORAGE;
          }
      }
      break;

    case TS_ENGINE_READ_PROFILE:
      {
        dataout = profile_read(&refdataout);
        if (!dataout)
          {
            eng_dbg("profile_read failed\n");
            resp.hdr.status = -TS_ENGINE_ERROR_NO_PROFILE;
            goto out;
          }

        resp.hdr.length = strlen(dataout) + 1;
      }
      break;

    case TS_ENGINE_WRITE_PROFILE:
      {
        ret = profile_write(datain, req.hdr.length);
        if (ret != OK)
          {
            eng_dbg("profile_write failed\n");
            resp.hdr.status = ret;
          }
      }
      break;

    case TS_ENGINE_WRITE_PROFILE_SHM:
      {
        struct ts_engine_shm_obj *shm = (struct ts_engine_shm_obj *)datain;

        ret = profile_write(shm->addr, shm->len);
        if (ret != OK)
          {
            eng_dbg("profile_write failed\n");
            resp.hdr.status = ret;
          }
        app->backend_update.profile_updated = true;
      }
      break;

    case TS_ENGINE_RELOAD_PROFILE:
      {
        ret = ts_engine_stop(app, true, false);
        if (ret < 0)
          {
            eng_dbg("ts_engine_stop failed\n");
          }

        __ts_engine_log_remove();

        resp.hdr.status = ts_engine_start(app);
      }
      break;

    case TS_ENGINE_STOP:
      {
        ret = ts_engine_stop(app, true, true);
        if (ret < 0)
          {
            eng_dbg("ts_engine_stop failed\n");
            resp.hdr.status = ret;
            goto out;
          }
      }
      break;

    case TS_ENGINE_PAUSE:
      {
        ret = ts_engine_stop(app, false, false);
        if (ret < 0)
          {
            eng_dbg("ts_engine_stop failed\n");
            resp.hdr.status = ret;
            goto out;
          }
      }
      break;

    case TS_ENGINE_CONTINUE:
      {
        ret = ts_engine_continue(app);
        if (ret < 0)
          {
            eng_dbg("ts_engine_continue failed\n");
            resp.hdr.status = ret;
            goto out;
          }
      }
      break;

    case TS_ENGINE_PING:
      {
        eng_dbg("ts_engine... Ping! Pong!\n");
        ret = OK;
      }
      break;

    case TS_ENGINE_CONNECTOR_RESULT_SHM:
      {
        struct ts_engine_shm_obj *shm = (struct ts_engine_shm_obj *)datain;
        struct ts_engine_connector_cb_info *info = shm->addr;

        if (app->handle && app->handle == info->priv)
          {
            __ts_engine_log_process(app->handle, (info->result != OK));
          }
#if CONFIG_THINGSEE_ENGINE_PING
        else if (!strncmp(info->priv, "ping", 4))
          {
            __ts_engine_ping_result(app, info->priv, info->result);
          }
#endif
      }
      break;

    default:
      eng_dbg("unknown req: %d\n", req.hdr.id);
    }

  out:

  if (datain)
    {
      free(datain);
    }

  ret = __ts_engine_full_write(app->fdout, &resp, sizeof(resp));
  DEBUGASSERT(ret == sizeof(resp));

  if (dataout)
    {
      ret = __ts_engine_full_write(app->fdout, dataout, resp.hdr.length);
      DEBUGASSERT(ret == resp.hdr.length);

      if (!refdataout)
        {
          free((void *) dataout);
        }
    }

  return OK;
}

enum porttype_e
{
  TYPE_NONE,
  TYPE_DCP,
};

#ifdef CONFIG_THINGSEE_CHARGER_MODULE
static
__attribute__((unused))
void
engine_system_reset(void)
{
  ts_engine_do_system_shutdown(&g_app_s, TS_ENGINE_SYSTEM_SHUTDOWN_TYPE_RESET);
}

static void
ts_handle_usb_change(const char *porttype)
{
  static enum porttype_e previous_porttype = TYPE_NONE;
  int ret;

  g_app_s.charger_connected = (porttype != NULL);

#ifdef CONFIG_THINGSEE_UI
  thingsee_UI_set_charger_state(g_app_s.charger_connected);
#endif

  /* DCP check. Is DCP connected? */
  if ((porttype != NULL && strcmp(porttype, "DCP") == 0))
    {
      previous_porttype = TYPE_DCP;
      return;
    }
  /* Was DCP connected previously? */
  else if (previous_porttype == TYPE_DCP)
    {
      previous_porttype = TYPE_NONE;

      /* If no DCP anymore, exit */
      if (porttype == NULL)
          return;
    }

  if (porttype != NULL && (strcmp(porttype, "SDP") == 0 ||
                           strcmp(porttype, "CDP") == 0))
    {
      /* Got PC USB connection. */

#ifdef CONFIG_THINGSEE_UI
      thingsee_UI_set_PC_USB_connected(true);
#endif

      if (!__ts_engine_sdcard_inserted())
        {
          eng_dbg("no sdcard -> no mass storage\n");
          return;
        }

      ret = ts_engine_stop(&g_app_s, true, true);
      if (ret < 0)
        {
          eng_dbg("ts_engine_stop failed\n");
        }

#ifdef CONFIG_SYSTEM_CONMAN
      struct conman_client_s conman_client;

      ret = conman_client_init(&conman_client);
      if (ret != OK)
        {
          eng_dbg("conman_client_init failed\n");
        }
      else
        {
          ret = conman_client_destroy_connection(&conman_client, CONMAN_CONNID_ALL);
          if (ret != OK)
            {
              eng_dbg("conman_client_destroy_connection(CONMAN_CONNID_ALL) failed\n");
            }

          conman_client_uninit(&conman_client);
        }
#endif

#ifdef CONFIG_THINGSEE_EMMC_MODULE
      emmc_switch_to_usbmsc_mode();
#endif
#ifdef CONFIG_THINGSEE_UI
      thingsee_UI_set_purpose_and_state(g_mass_storage_str, "");
#endif
    }
  else
    {
      /* Lost PC USB connection. */

#ifdef CONFIG_THINGSEE_UI
      thingsee_UI_set_PC_USB_connected(false);
#endif
#ifdef CONFIG_THINGSEE_EMMC_MODULE
      emmc_switch_to_filesystem_mode(engine_system_reset);
#endif
    }
}
#endif

static int generate_device_property(void)
{
  const char *device_property;
  const char *compare;
  int ret;

  ret = device_property_get(&device_property);
  if (ret < 0)
    {
      eng_dbg("device_property_get failed\n");
      return ret;
    }

  /* Add it to entropy pool. */

  seed_urandom(device_property, strlen(device_property));

  compare = __ts_engine_sdcard_read(SDCARD_DEVICE_PROPERTY_FILENAME);
  if (compare)
    {
      if (memcmp(compare, device_property, strlen(device_property)) == 0)
        {
          eng_dbg("%s already created, skipping...\n", SDCARD_DEVICE_PROPERTY_FILENAME);
          ret = OK;
          goto out;
        }
    }

  ret = __ts_engine_sdcard_write(SDCARD_DEVICE_PROPERTY_FILENAME, device_property, strlen(device_property) + 1, false);
  if (ret < 0)
    {
      eng_dbg("sdcard_write failed\n");
    }

out:

  free((void*)device_property);

  if (compare)
    {
      free((void*)compare);
    }

  return ret;
}

#ifdef CONFIG_THINGSEE_CHARGER_MODULE
static const struct bq24251_chgr_cbks_s *
ts_get_charger_callbacks(void)
{
  static const struct bq24251_chgr_cbks_s callbacks =
  {
    .charger_event = NULL,
    .notify_usb_connect = ts_handle_usb_change,
  };

  return &callbacks;
}
#endif

static void
print_crash_log(void)
{
  size_t size;
  const char *log =
      board_eeprom_get_section(BOARD_EEPROM_SECTION_ENGINE_ASSERT_CRASH_DATA,
                               &size);
  char null = '\0';

  if (!log || size == 0)
    {
      return;
    }

  if (log[0] != '!')
    {
      return;
    }

  /* Print crash reason. */

  dbg("== SOFTWARE WAS RESETED BY ASSERT ==\n");
  dbg("== Assert was at: %s\n", log + 1);

  /* Clear log. */

  board_eeprom_write_section(BOARD_EEPROM_SECTION_ENGINE_ASSERT_CRASH_DATA, 0,
                             &null, 1);
}

#ifndef CONFIG_ARCH_SIM

static int reset_assert_count(const int timer_id,
                              const struct timespec *date, void * const priv)
{
  /* Reset assert counter. */

  board_reset_assert_count();

  eng_dbg("assert count reset.\n");

  return OK;
}

static void start_reset_assert_count_timer(void)
{
  struct timespec ts = {};
  int id;
  int ret;

  ret = clock_gettime(CLOCK_MONOTONIC, &ts);
  DEBUGASSERT(ret >= 0);

  /* Reset assert counter after device has been on for sometime. */

  ts.tv_sec += CONFIG_THINGSEE_ENGINE_ASSERT_COUNT_RESET_SECS;

  /* Launch timer */

  id = ts_core_timer_setup_date(&ts, reset_assert_count, NULL);
  DEBUGASSERT(id >= 0);
}

#endif /* CONFIG_ARCH_SIM */

static int
init_system(struct ts_engine_app *app)
{
  int ret;

#if defined(CONFIG_THINGSEE_EMMC_MODULE) && !defined(CONFIG_ARCH_SIM)

  /* Open MMC/SDcard. */

  ret = emmc_init_module();
  if (ret < 0)
    {
      eng_dbg("emmc_init_module failed\n");
      return ret;
    }

#endif

  dbg("Initializing Thingsee core library\n");

  ret = ts_core_initialize();
  DEBUGASSERT(ret == OK);

  /* Print build version */

  dbg("Thingsee SW build version: %s\n", CONFIG_VERSION_BUILD);

  /* Add something SW version dependent to entropy pool. */

  seed_urandom(CONFIG_VERSION_BUILD, sizeof(CONFIG_VERSION_BUILD));

  /* Print crash log. */

  print_crash_log();

  /* Load system configuration parameters. */

  system_config_load();

#ifndef CONFIG_ARCH_SIM

#ifdef CONFIG_THINGSEE_CHARGER_MODULE
  eng_dbg("Initializing charging module\n");

  bq24251_set_charging_allowed(system_config_charging_allowed());

  ret = bq24251_init_module(ts_get_charger_callbacks());
  if (ret < 0)
    {
      eng_dbg("bq24251_init_module failed\n");
      return ERROR;
    }

#endif
  eng_dbg("Initializing display\n");

#ifdef CONFIG_THINGSEE_DISPLAY_MODULE
  ret = oled_start_module();
  DEBUGASSERT(ret == OK);
#endif

#ifdef CONFIG_THINGSEE_UI
  start_thingsee_UI(engine_system_shutdown, system_config_force_power_on(), app);
#endif

  ts_watchdog_initialize();
  start_reset_assert_count_timer();

#endif /* CONFIG_ARCH_SIM */

#ifdef CONFIG_SYSTEM_PROPERTIES_INT

  if (sys_prop_check_flag(SYS_PROP_INFO_FLAG_SLEEP))
    {
      sys_prop_clear_flag(SYS_PROP_INFO_FLAG_SLEEP);

      board_lcdoff();
      emmc_handle_power_off(true);

      eng_dbg("First boot mode\n");

      board_go_to_standby();

      /* Never reached */
   }

#endif /* CONFIG_SYSTEM_PROPERTIES_INT */

  eng_dbg("Initializing sense lookup\n");

  ret = initialize_sense_lookup();
  if (ret != OK)
    {
      eng_dbg("initialize_sense_lookup failed\n");
      return ERROR;
    }

  ret = __ts_engine_client_init(&app->fdin, &app->fdout);
  if (ret != OK)
    {
      eng_dbg("__ts_engine_client_init failed\n");
      return ERROR;
    }

  ret = ts_core_fd_register(app->fdin, POLLIN, engine_ctrl_cb, app);
  if (ret != OK)
    {
      eng_dbg("fdin register failed\n");
      return ERROR;
    }

  ret = generate_device_property();
  if (ret < 0)
    {
      eng_dbg("generate_device_property failed\n");
    }

  ret = __ts_engine_time_update_from_file();
  if (ret < 0)
    {
      eng_dbg("__ts_engine_time_update_from_file failed\n");
    }

  return OK;
}

int
ts_engine_main(int argc, char *argv[])
{
  int ret;
  const char *properties;
  bool refproperties;
  struct ts_engine_app *app = &g_app_s;
  bool loop_goon = true;

  memset(app, 0, sizeof(*app));

  g_engine_pid = getpid();

  //alloc_dbg_start();

  ret = init_system(app);
  if (ret != OK)
    {
      eng_dbg("init_system failed\n");
      return EXIT_FAILURE;
    }

  properties = cloud_property_read(&refproperties);
  if (properties)
    {
#ifdef CONFIG_SYSTEM_CONMAN
      struct conman_client_s conman_client;

      ret = conman_client_init(&conman_client);
      if (ret != OK)
        {
          eng_dbg("conman_client_init failed\n");
        }
      else
        {
          ret = conman_client_set_connections_config(&conman_client, properties);
          if (ret != OK)
            {
              eng_dbg("conman_client_set_connections_config failed\n");
            }

          conman_client_uninit(&conman_client);
        }
#endif
      if (!refproperties)
        {
          free((void *) properties);
        }
    }

  ret = ts_engine_start(app);
  if (ret < 0)
    {
      eng_dbg("ts_engine_start failed\n");
    }

  eng_dbg("Starting Thingsee main event loop\n");

#ifdef CONFIG_NETUTILS_NTPCLIENT
  ret = ntpc_start();
#endif

  g_app_s.mainloop_started = true;
  ts_core_mainloop(&loop_goon);

  /* This is never reached */
  return EXIT_SUCCESS;
}

bool charger_connected(void)
{
  return g_app_s.charger_connected;
}
