/****************************************************************************
 * examples/ubmodem/ubmodem_main.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *    Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <debug.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <arch/board/board-modem.h>
#include <apps/system/ubmodem.h>
#include <apps/netutils/dnsclient.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define UBMODEM_EXAMPLE_DEBUG 1

#ifndef CONFIG_EXAMPLES_UBMODEM_DAEMONPRIO
#  define CONFIG_EXAMPLES_UBMODEM_DAEMONPRIO SCHED_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_EXAMPLES_UBMODEM_DAEMONSTACKSIZE
#  define CONFIG_EXAMPLES_UBMODEM_DAEMONSTACKSIZE 2048
#endif

#ifndef CONFIG_EXAMPLES_UBMODEM_SIMPIN
#  define CONFIG_EXAMPLES_UBMODEM_SIMPIN "1234"
#endif

#ifndef CONFIG_EXAMPLES_UBMODEM_APN_NAME
#  define CONFIG_EXAMPLES_UBMODEM_APN_NAME "internet"
#endif

#ifndef CONFIG_EXAMPLES_UBMODEM_APN_USER
#  define CONFIG_EXAMPLES_UBMODEM_APN_USER NULL
#endif

#ifndef CONFIG_EXAMPLES_UBMODEM_APN_PASSWD
#  define CONFIG_EXAMPLES_UBMODEM_APN_PASSWD NULL
#endif

#ifndef CONFIG_EXAMPLES_UBMODEM_APN_IPADDR
#  define CONFIG_EXAMPLES_UBMODEM_APN_IPADDR "0.0.0.0"
#endif

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int ubmodem_hw_init(void *priv, bool *is_vcc_off);
static int ubmodem_hw_deinit(void *priv, int serial_fd);
static bool ubmodem_hw_vcc_set(void *priv, bool on);
static uint32_t ubmodem_hw_poweron_pin_set(void *priv, bool on);
static uint32_t ubmodem_hw_reset_pin_set(void *priv, bool on);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ubmodem_hw_ops_s ubmodem_hwops =
{
  .initialize       = ubmodem_hw_init,
  .deinitialize     = ubmodem_hw_deinit,
  .vcc_set          = ubmodem_hw_vcc_set,
  .reset_pin_set    = ubmodem_hw_reset_pin_set,
  .poweron_pin_set  = ubmodem_hw_poweron_pin_set
};

static struct
{
  pid_t pid;

  bool stop_daemon:1;
  bool in_poweroff:1;
  bool powered_off:1;
} g_ubmodemd =
{
  .pid = -1,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int ubmodem_hw_init(void *priv, bool *is_vcc_off)
{
  return board_modem_initialize(is_vcc_off);
}

static int ubmodem_hw_deinit(void *priv, int serial_fd)
{
  return board_modem_deinitialize(serial_fd);
}

static bool ubmodem_hw_vcc_set(void *priv, bool on)
{
  return board_modem_vcc_set(on);
}

static uint32_t ubmodem_hw_poweron_pin_set(void *priv, bool on)
{
  return board_modem_poweron_pin_set(on);
}

static uint32_t ubmodem_hw_reset_pin_set(void *priv, bool on)
{
  return board_modem_reset_pin_set(on);
}

static void event_ip_address(struct ubmodem_s *modem,
                             enum ubmodem_event_flags_e event,
                             const void *event_data, size_t datalen,
                             void *priv)
{
  const struct ubmodem_event_ip_address_s *ipcfg = event_data;
  uint32_t addr = ntohl(ipcfg->ipaddr.s_addr);
  uint32_t dns1 = ntohl(ipcfg->dns1.s_addr);
  uint32_t dns2 = ntohl(ipcfg->dns2.s_addr);

  printf("%s(): GPRS IP address: %d.%d.%d.%d\n", __func__,
         (addr >> 24) & 0xff,
         (addr >> 16) & 0xff,
         (addr >> 8) & 0xff,
         (addr >> 0) & 0xff);
  printf("%s(): Primary DNS server: %d.%d.%d.%d\n", __func__,
         (dns1 >> 24) & 0xff,
         (dns1 >> 16) & 0xff,
         (dns1 >> 8) & 0xff,
         (dns1 >> 0) & 0xff);
  printf("%s(): Secondary DNS server: %d.%d.%d.%d\n", __func__,
         (dns2 >> 24) & 0xff,
         (dns2 >> 16) & 0xff,
         (dns2 >> 8) & 0xff,
         (dns2 >> 0) & 0xff);

  dns_setserver(&ipcfg->dns1);
}

static void event_target_level(struct ubmodem_s *modem,
                               enum ubmodem_event_flags_e event,
                               const void *event_data, size_t datalen,
                               void *priv)
{
  const struct ubmodem_event_target_level_reached_s *data = event_data;

  printf("%s(): target level reached: %d\n", __func__, data->new_level);

  if (data->new_level == UBMODEM_LEVEL_POWERED_OFF)
    {
      if (g_ubmodemd.in_poweroff)
        {
          g_ubmodemd.powered_off = true;
          g_ubmodemd.stop_daemon = true;
        }
    }
  else if (data->new_level == UBMODEM_LEVEL_GPRS)
    {
      /* Timeout 30 seconds, target accuracy 1000 meters. */

      ubmodem_start_cell_locate(modem, 30, 1000);
    }
}

static void event_failed_target_level(struct ubmodem_s *modem,
                                      enum ubmodem_event_flags_e event,
                                      const void *event_data, size_t datalen,
                                      void *priv)
{
  const struct ubmodem_event_transition_failed_s *data = event_data;

  printf("%s(): current level: %d, target level: %d, reason: %s\n", __func__,
      data->current_level, data->target_level, data->reason);

  if (data->target_level == UBMODEM_LEVEL_GPRS &&
      data->current_level < UBMODEM_LEVEL_GPRS)
    {
      printf("Failed to open GPRS connection. Powering off modem...\n");

      ubmodem_request_level(modem, UBMODEM_LEVEL_POWERED_OFF);

      g_ubmodemd.in_poweroff = true;
    }
  else if (data->target_level == UBMODEM_LEVEL_GPRS &&
           data->current_level == UBMODEM_LEVEL_GPRS)
    {
      printf("GPRS connection lost. Attempt reconnect...\n");

      ubmodem_request_level(modem, UBMODEM_LEVEL_GPRS);
    }
}

static void event_cell_location(struct ubmodem_s *modem,
                                enum ubmodem_event_flags_e event,
                                const void *event_data, size_t datalen,
                                void *priv)
{
  const struct ubmodem_event_cell_location_s *data = event_data;

  printf("%s(): Location: Latitude: %.7f deg\n", __func__, data->location.latitude);
  printf("%s(): Location: Longitude: %.7f deg\n", __func__, data->location.longitude);
  printf("%s(): Location: Altitude: %d meters\n", __func__, data->location.altitude);
  printf("%s(): Location: Accuracy: %d meters\n", __func__, data->location.accuracy);
  printf("%s(): Date: %04d-%02d-%02d\n", __func__, data->date.year, data->date.month, data->date.day);
  printf("%s(): Time: %02d:%02d:%02d\n", __func__, data->date.hour, data->date.min, data->date.sec);
}

static void event_cell_environment(struct ubmodem_s *modem,
                                   enum ubmodem_event_flags_e event,
                                   const void *event_data, size_t datalen,
                                   void *priv)
{
  const struct ubmodem_event_cell_environment_s *data = event_data;
  unsigned int i;

  if (data->have_signal_qual)
    {
      int qual = data->signal_qual.qual;
      int qual_scaled = (7 - data->signal_qual.qual) * 100 / 7;

      if (qual < 0)
        qual_scaled = -1;

      printf("%s(): Signal quality: RSSI=%d dBm, QUAL=%d (%d %%)\n",
             __func__, data->signal_qual.rssi, qual, qual_scaled);
    }

  if (data->have_serving)
    {
      printf("%s(): Serving cell: type=%s, MCC=%d, MNC=%d, LAC=%x, CellID=%x, SIGNAL_dBm=%d",
             __func__, data->serving.rat == UBMODEM_RAT_GSM ? "GSM" : "UMTS",
             data->serving.mcc, data->serving.mnc, data->serving.lac,
             data->serving.cell_id, data->serving.signal_dbm);

      printf(data->serving.rat == UBMODEM_RAT_UMTS ? ", SC=%d\n" : "\n", data->serving.sc);
    }

  printf("%s(): Neighbor cells (num=%d):\n", __func__, data->num_neighbors);

  for (i = 0; i < data->num_neighbors; i++)
    {
      printf("%s(): type=%s", __func__, (data->neighbors[i].rat == UBMODEM_RAT_GSM) ? "GSM" : "UMTS");
      if (data->neighbors[i].have_mcc_mnc_lac)
        printf(", MCC=%d, MNC=%d, LAC=%x",
               data->neighbors[i].mcc, data->neighbors[i].mnc,
               data->neighbors[i].lac);
      if (data->neighbors[i].have_cellid)
        printf(", CellID=%x", data->neighbors[i].cell_id);
      if (data->neighbors[i].have_sc)
        printf(", SC=%d", data->neighbors[i].sc);
      if (data->neighbors[i].have_signal_dbm)
        printf(", SIGNAL_dBm=%d", data->neighbors[i].signal_dbm);
      printf("\n");
    }
}

#ifdef UBMODEM_EXAMPLE_DEBUG
static void event_new_level(struct ubmodem_s *modem,
                            enum ubmodem_event_flags_e event,
                            const void *event_data, size_t datalen,
                            void *priv)
{
  const struct ubmodem_event_new_level_s *data = event_data;

  printf("%s(): old level: %d => new level: %d\n", __func__, data->old_level,
         data->new_level);
}

static void event_state_change(struct ubmodem_s *modem,
                               enum ubmodem_event_flags_e event,
                               const void *event_data, size_t datalen,
                               void *priv)
{
  /* Library internal state change, listener for debugging. */

  printf("%s(): old state: %d => new state: %d\n", __func__,
      ((const int *)event_data)[1],
      ((const int *)event_data)[0]);
}

static void event_data_to_modem(struct ubmodem_s *modem,
                                enum ubmodem_event_flags_e event,
                                const void *event_data, size_t datalen,
                                void *priv)
{
  const char *buf = event_data;
  size_t i;

  /* Data listener for debugging. */

  printf("%s(): %s: [", __func__, "    to modem");

  for (i = 0; i < datalen; i++)
    {
      char cur = buf[i];

      printf(isprint(cur) ? "%c" : "\\x%02X", cur);
    }

  printf("]\n");
}

static void event_data_from_modem(struct ubmodem_s *modem,
                                  enum ubmodem_event_flags_e event,
                                  const void *event_data, size_t datalen,
                                  void *priv)
{
  const char *buf = event_data;
  size_t i;

  /* Data listener for debugging. */

  printf("%s(): %s: [", __func__, "from modem");

  for (i = 0; i < datalen; i++)
    {
      char cur = buf[i];

      printf(isprint(cur) ? "%c" : "\\x%02X", cur);
    }

  printf("]\n");
}
#endif

static bool get_modem_config(struct ubmodem_s *modem,
                             const char *name, char *buf,
                             size_t buflen, void *priv)
{
  static const struct
  {
    const char *name;
    const char *value;
  } config[] =
    {
      { "modem.pin", CONFIG_EXAMPLES_UBMODEM_SIMPIN },
      { "modem.apn_name", CONFIG_EXAMPLES_UBMODEM_APN_NAME },
      { "modem.apn_user", CONFIG_EXAMPLES_UBMODEM_APN_USER },
      { "modem.apn_password", CONFIG_EXAMPLES_UBMODEM_APN_PASSWD },
      { "modem.apn_ipaddr", CONFIG_EXAMPLES_UBMODEM_APN_IPADDR },
    };
  int i;

  printf("get_modem_config: looking value for '%s'.\n", name);

  for (i = 0; i < ARRAY_SIZE(config); i++)
    {
      if (strcmp(name, config[i].name) != 0)
        continue;

      if (config[i].value == NULL || config[i].value[0] == '\0')
        return false;

      snprintf(buf, buflen, config[i].value);

      return true;
    }

  return false;
}

static int ubmodemd_daemon(int argc, char *argv[])
{
  struct ubmodem_s *modem;
  struct pollfd *pfds;
  int maxfds;
  int timeout;
  int nfds;
  int ret;
  int time_cell_info_prev = 0;

  g_ubmodemd.in_poweroff = false;
  g_ubmodemd.stop_daemon = false;

  printf("ubmodemd_daemon: Initializing modem...\n");

  /* Initialize modem library. */

  modem = ubmodem_initialize(&ubmodem_hwops, NULL);
  if (!modem)
    {
      printf("ubmodemd_daemon: Failed to initialize.\n");
      ret = -1;
      goto out;
    }

  /* Set callback used for fetching SIM-PIN and APN settings. */

  ubmodem_set_config_callback(modem, get_modem_config, NULL);

  /* Register event listeners. */

  ubmodem_register_event_listener(modem, UBMODEM_EVENT_FLAG_IP_ADDRESS,
                                  event_ip_address,
                                  NULL);
  ubmodem_register_event_listener(modem, UBMODEM_EVENT_FLAG_TARGET_LEVEL_REACHED,
                                  event_target_level,
                                  NULL);
  ubmodem_register_event_listener(modem, UBMODEM_EVENT_FLAG_FAILED_LEVEL_TRANSITION,
                                  event_failed_target_level,
                                  NULL);
  ubmodem_register_event_listener(modem, UBMODEM_EVENT_FLAG_CELL_LOCATION,
                                  event_cell_location,
                                  NULL);
  ubmodem_register_event_listener(modem, UBMODEM_EVENT_FLAG_CELL_ENVIRONMENT,
                                  event_cell_environment,
                                  NULL);
#ifdef UBMODEM_EXAMPLE_DEBUG
  ubmodem_register_event_listener(modem, UBMODEM_EVENT_FLAG_NEW_LEVEL,
                                  event_new_level,
                                  NULL);
  ubmodem_register_event_listener(modem, UBMODEM_EVENT_FLAG_TRACE_STATE_CHANGE,
                                  event_state_change,
                                  NULL);
  ubmodem_register_event_listener(modem, UBMODEM_EVENT_FLAG_TRACE_DATA_TO_MODEM,
                                  event_data_to_modem,
                                  NULL);
  ubmodem_register_event_listener(modem, UBMODEM_EVENT_FLAG_TRACE_DATA_FROM_MODEM,
                                  event_data_from_modem,
                                  NULL);
#endif

  printf("ubmodemd_daemon: Try GPRS connection.\n");

  /* Request GPRS connection for modem. */

  ubmodem_request_level(modem, UBMODEM_LEVEL_GPRS);

  /* Setup modem event processing. */

  maxfds = ubmodem_poll_max_fds(modem);
  pfds = calloc(maxfds, sizeof(*pfds));
  if (!pfds)
    {
      printf("ubmodemd_daemon: Out-of-memory, tried to allocate %lld.\n",
             (long long int)maxfds * sizeof(*pfds));

      ret = -ENOMEM;
      goto out;
    }

  /* Process modem events. */

  do
    {
      /* Check if daemon should be stopped. */

      if (g_ubmodemd.stop_daemon && !g_ubmodemd.in_poweroff)
        {
          /* Request power-off for modem. */

          ubmodem_request_level(modem, UBMODEM_LEVEL_POWERED_OFF);

          g_ubmodemd.in_poweroff = true;
        }

      /* Setup pollfds for modem and get timeout for timers. */

      nfds = ubmodem_pollfds_setup(modem, pfds, maxfds, &timeout);
      if (nfds < 0)
        {
          printf("ubmodemd_daemon: ubmodem_pollfds_setup failed, %d\n", nfds);
          continue;
        }

      /* Wait for poll event or timeout. */

      ret = poll(pfds, nfds, timeout < 5000 ? timeout : 5000);
      if (ret < 0)
        {
          printf("ubmodemd_daemon: poll failed, %d\n", ret);
          continue;
        }

      /* Handle events and/or timeout. */

      ret = ubmodem_pollfds_event(modem, ret ? pfds : NULL, ret ? nfds : 0);
      if (ret < 0)
        {
          printf("ubmodemd_daemon: ubmodem_pollfds_event failed, %d\n", ret);
          continue;
        }

      /* Trigger cell-environment request */

      if (time(0) - time_cell_info_prev >= 15)
        {
          time_cell_info_prev = time(0);

          ret = ubmodem_request_cell_environment(modem);
          if (ret < 0)
            {
              printf("ubmodem_request_cell_environment: %d\n", ret);
            }
        }
    }
  while (!(g_ubmodemd.stop_daemon && g_ubmodemd.powered_off));

  /* Uninitialize modem library. */

  ubmodem_uninitialize(modem);

  ret = 0;

out:
  printf("ubmodem daemon stopped!\n");

  g_ubmodemd.pid = -1;

  return ret;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * ubmodem_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int ubmodem_main(int argc, char *argv[])
#endif
{
  unsigned int priority;
  unsigned int stacksize;
  pid_t pid = -1;

  if (argc > 1 && strcmp(argv[1], "stop") == 0)
    {
      printf("ubmodem_main: Stopping the ubmodem daemon...\n");

      if (g_ubmodemd.pid < 0)
        {
          printf("ubmodem_main: the ubmodem daemon not running.\n");

          return -1;
        }

      g_ubmodemd.stop_daemon = true;
      kill(g_ubmodemd.pid, SIGUSR1);

      return 0;
    }

  printf("ubmodem_main: Starting the ubmodem daemon\n");

  if (g_ubmodemd.pid < 0)
    {
      priority  = CONFIG_EXAMPLES_UBMODEM_DAEMONPRIO;
      stacksize = CONFIG_EXAMPLES_UBMODEM_DAEMONSTACKSIZE;

      pid = task_create("ubmodemd", priority, stacksize, ubmodemd_daemon, NULL);
      if (pid < 0)
        {
          printf("ubmodem_main: Starting the ubmodem daemon failed!\n");
        }

      g_ubmodemd.pid = pid;
    }
  else
    {
      printf("ubmodem_main: ubmodem daemon already started\n");
    }

  printf("ubmodem_main: Exiting\n");

  return pid;
}
