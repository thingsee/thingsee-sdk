/****************************************************************************
 * apps/ts_engine/engine/sense_hts221.c
 *
 * Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
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

#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdlib.h>

#include <nuttx/sensors/hts221.h>

#include "eng_dbg.h"
#include "parse.h"
#include "execute.h"
#include "sense.h"

#define HTS221_DEVPATH                "/dev/hts221"

static const hts221_settings_t settings =
  {
  .temp_resol = HTS221_AVGT4, .humid_resol = HTS221_AVGH8, .odr =
      HTS221_ODR_ONESHOT, .is_bdu = true, .is_data_rdy = true, .is_high_edge =
  true, .is_open_drain = false, .is_boot = false
  };

struct client
{
  sq_entry_t entry;
  struct ts_cause *cause;
  bool active :1;
};

struct hts221
{
  int fd;
  sq_queue_t clients;
  hts221_conv_data_t values;
  unsigned int refcount;
};

static struct hts221 g_hts221 =
  {
  .fd = -1, .refcount = 0,
  };

static int publish_measurements(struct hts221 *hts221)
{
  struct client *client;
  struct ts_cause *cause;

  client = (struct client *) sq_peek(&hts221->clients);

  while (client)
    {
      cause = client->cause;

      switch (cause->dyn.sense_info->sId & 0xffffff00)
        {
        case SENSE_ID_TEMPERATURE:
          {
            cause->dyn.sense_value.value.valuedouble =
                hts221->values.temperature
                    / (double) HTS221_TEMPERATURE_PRECISION;
          }
        break;
        case SENSE_ID_HUMIDITY:
          {
            cause->dyn.sense_value.value.valuedouble = hts221->values.humidity
                / (double) HTS221_HUMIDITY_PRECISION;
          }
        break;
        }

      if (handle_cause(cause) == true)
        {
          break;
        }

      client = (struct client *) sq_next(&client->entry);
    }

  return OK;
}

static int sense_hts221_read_measurement(struct hts221 *hts221)
{
  struct hts221_status_t hwstatus;
  int ret;

  ret = ioctl(hts221->fd, HTS221_IOC_CHECK_STATUS_REG,
      (unsigned int) &hwstatus);
  if (ret < 0)
    {
      eng_dbg("Conversion not ready\n");
      return ERROR;
    }

  if (!hwstatus.is_humid_ready || !hwstatus.is_temp_ready)
    {
      eng_dbg("Conversion not ready\n");
      return ERROR;
    }

  ret = ioctl(hts221->fd, HTS221_IOC_READ_CONVERT_DATA,
      (unsigned int) &hts221->values);
  if (ret < 0)
    {
      eng_dbg("Failed to read humidity & temperature.\n");
    }

  ret = ts_core_fd_unregister(hts221->fd);
  if (ret < 0)
    {
      eng_dbg("ts_core_fd_unregister failed\n");
    }

  close(hts221->fd);
  hts221->fd = -1;

  publish_measurements(hts221);

  return OK;
}

static int sense_hts221_callback(const struct pollfd * const pfd,
    void * const priv)
{
  struct hts221 *hts221 = priv;

  (void) sense_hts221_read_measurement(hts221);

  return OK;
}

static int sense_hts221_start_measurement(struct hts221 *hts221)
{
  int ret;

  hts221->fd = open(HTS221_DEVPATH, O_RDONLY);
  if (hts221->fd < 0)
    {
      eng_dbg("Failed to open %s (%d).\n", HTS221_DEVPATH, get_errno());
      return ERROR;
    }

  ret = ioctl(hts221->fd, HTS221_IOC_CFGR, (unsigned int) &settings);
  if (ret < 0)
    {
      eng_dbg("Failed to configure hts221\n");
      goto errout_close;
    }

  ret = ioctl(hts221->fd, HTS221_IOC_START_CONVERSION, 0);
  if (ret < 0)
    {
      eng_dbg("Failed to start conversion\n");
      goto errout_close;
    }

  ret = ts_core_fd_register(hts221->fd, POLLIN, sense_hts221_callback, hts221);
  if (ret < 0)
    {
      eng_dbg("ts_core_fd_register failed\n");
      return ERROR;
    }

  return OK;

  errout_close: close(hts221->fd);
  hts221->fd = -1;

  return ERROR;
}

int sense_hts221_active_init(struct ts_cause *cause)
{
  struct hts221 *hts221 = &g_hts221;
  struct client *client;

  if (hts221->refcount == 0)
    {
      sq_init(&hts221->clients);
    }

  client = (struct client *) sq_peek(&hts221->clients);

  while (client)
    {
      if (client->cause == cause)
        {
          eng_dbg("already registered client\n");
          return OK;
        }

      client = (struct client *) sq_next(&client->entry);
    }

  client = malloc(sizeof(*client));
  if (!client)
    {
      eng_dbg("malloc failed\n");
      return ERROR;
    }

  client->cause = cause;

  sq_addlast(&client->entry, &hts221->clients);
  hts221->refcount++;

  return OK;
}

int sense_hts221_active_uninit(struct ts_cause *cause)
{
  struct hts221 *hts221 = &g_hts221;
  struct client *client;

  client = (struct client *) sq_peek(&hts221->clients);

  while (client)
    {
      if (client->cause == cause)
        {
          sq_rem(&client->entry, &hts221->clients);
          free(client);
          hts221->refcount--;
          return OK;
        }

      client = (struct client *) sq_next(&client->entry);
    }

  return ERROR;
}

int sense_hts221_active_read(struct ts_cause *cause)
{
  struct hts221 *hts221 = &g_hts221;
  int ret;

  if (hts221->fd == -1)
    {
      ret = sense_hts221_start_measurement(hts221);
      if (ret < 0)
        {
          eng_dbg("sense_hts221_start_measurement failed\n");
          return ERROR;
        }
    }

  return OK;
}
