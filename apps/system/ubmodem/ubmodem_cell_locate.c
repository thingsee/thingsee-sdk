/****************************************************************************
 * apps/system/ubmodem/ubmodem_cell_locate.c
 *
 *   Copyright (C) 2014-2016 Haltian Ltd. All rights reserved.
 *   Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#include <sys/types.h>
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <apps/system/ubmodem.h>

#include "ubmodem_internal.h"
#include "ubmodem_hw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CELLLOCATE_GUARD_TIMEOUT_SECS 10

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct cell_locate_priv_s
{
  int timeout;
  int target_accuracy;
};

struct cell_locate_aid_priv_s
{
  time_t time;
  double latitude;
  double longitude;
  int altitude;
  struct
  {
    unsigned int major;
    unsigned int minor;
    unsigned int orientation;
  } accuracy;
  unsigned int speed;
  unsigned int direction;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_UBMODEM_DISABLE_CELLLOCATE

static const struct at_cmd_def_s cmd_ATpULOCCELL =
{
  .name         = "+ULOCCELL",
  .resp_format  = NULL,
  .resp_num     = 0,
};

static const struct at_cmd_def_s cmd_ATpULOC =
{
  .name         = "+ULOC",
  .resp_format  =
      (const uint8_t[]){
        RESP_FMT_INT8,
        RESP_FMT_INT8,
        RESP_FMT_INT8,
        RESP_FMT_INT16,
        RESP_FMT_INT32,
        RESP_FMT_INT8,
      },
  .resp_num     = 6,
  .timeout_dsec = MODEM_CMD_LOCATION_TIMEOUT,
};

static const struct at_cmd_def_s urc_ATpUULOC =
{
  /* response_type = 0 */
  .name         = "+UULOC",
  .resp_format  =
      (const uint8_t[]){
        RESP_FMT_STRING,
        RESP_FMT_STRING,
        RESP_FMT_STRING,
        RESP_FMT_STRING,
        RESP_FMT_INT32,
        RESP_FMT_INT32,
      },
  .resp_num     = 6,
};

#endif

#ifndef CONFIG_UBMODEM_DISABLE_AID_CELLLOCATE

static const struct at_cmd_def_s cmd_ATpULOCAID =
{
  .name         = "+ULOCAID",
  .resp_format  = NULL,
  .resp_num     = 0,
  .timeout_dsec = MODEM_CMD_LOCATION_TIMEOUT,
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_UBMODEM_DISABLE_CELLLOCATE

static void send_celllocate_failed_event(struct ubmodem_s *modem)
{
  struct ubmodem_event_cell_location_s data = {};

  /* Report failed CellLocation to application. */

  data.location.valid = false;
  data.date.valid = false;
  __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_CELL_LOCATION, &data,
                          sizeof(data));
}

static void urc_cell_locate_handler(struct ubmodem_s *modem,
                                    const struct at_cmd_def_s *cmd,
                                    const struct at_resp_info_s *info,
                                    const uint8_t *stream, size_t stream_len,
                                    void *priv)
{
  struct ubmodem_event_cell_location_s data = {};
  int ret, dummy;
  const char *date = ""; /* "dd/mm/yyyy" */
  const char *time = ""; /* "HH:MM:SS.sss" */
  const char *latitude = ""; /* degrees, "dd.ddddd" */
  const char *longitude = ""; /* degrees, "dd.ddddd" */
  int32_t altitude = 0; /* meters from sea level */
  int32_t accuracy = 999999; /* meters of accuracy */

  __ubmodem_cell_locate_cleanup(modem, true);

  /*
   * Response handler for +UULOC URC.
   */

  if (info->status != RESP_STATUS_URC)
    return;

  if (__ubmodem_stream_get_string(&stream, &stream_len, &date, NULL))
    {
      if (__ubmodem_stream_get_string(&stream, &stream_len, &time, NULL))
        {
          if (__ubmodem_stream_get_string(&stream, &stream_len, &latitude, NULL))
            {
              if (__ubmodem_stream_get_string(&stream, &stream_len, &longitude, NULL))
                {
                  if (__ubmodem_stream_get_int32(&stream, &stream_len, &altitude))
                    {
                      (void)__ubmodem_stream_get_int32(&stream, &stream_len, &accuracy);
                    }
                }
            }
        }
    }

  ubdbg("\"%s\",\"%s\",%s,%s,%d,%d\n", date, time, latitude, longitude,
        altitude, accuracy);

  data.date.valid = true;

  /* Parse date. */

  ret = sscanf(date, "%d/%d/%d", &data.date.day, &data.date.month,
               &data.date.year);
  if (ret != 3)
    data.date.valid = false;
  else if (data.date.day < 1 || data.date.day > 31)
    data.date.valid = false;
  else if (data.date.month < 1 || data.date.month > 12)
    data.date.valid = false;
  else if (data.date.year < 2015 || data.date.year > 2999)
    data.date.valid = false;

  /* Parse time. */

  ret = sscanf(time, "%d:%d:%d.%d", &data.date.hour, &data.date.min,
               &data.date.sec, &dummy);
  if (ret != 4)
    data.date.valid = false;
  else if (data.date.hour < 0 || data.date.hour > 23)
    data.date.valid = false;
  else if (data.date.min < 0 || data.date.min > 59)
    data.date.valid = false;
  else if (data.date.sec < 0 || data.date.sec > 59)
    data.date.valid = false;

  /* Parse latitude. */

  data.location.latitude = strtod(latitude, NULL);

  /* Parse longitude. */

  data.location.longitude = strtod(longitude, NULL);

  data.location.accuracy = accuracy;
  data.location.altitude = altitude;

  if ((data.location.latitude < 0.0000000001 &&
       data.location.latitude > -0.0000000001) ||
      (data.location.longitude < 0.0000000001 &&
       data.location.longitude > -0.0000000001))
    {
      data.location.valid = false;
    }
  else
    {
      data.location.valid = true;
    }

  /* Report CellLocation to application. */

  __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_CELL_LOCATION, &data,
                          sizeof(data));
}

static int uuloc_timeout_handler(struct ubmodem_s *modem,
                                 const int timer_id,
                                 void * const arg)
{
  modem->cell_locate_timer_id = -1;

  __ubmodem_cell_locate_cleanup(modem, false);

  return OK;
}

static void set_ATpULOC_handler(struct ubmodem_s *modem,
                                const struct at_cmd_def_s *cmd,
                                const struct at_resp_info_s *info,
                                const uint8_t *resp_stream,
                                size_t stream_len, void *priv)
{
  uintptr_t uuloc_timeout = (uintptr_t)priv;

  /*
   * Response handler for +ULOC command.
   */

  /* Update main state machine. */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);

  modem->cell_locate_timer_id =
    __ubmodem_set_timer(modem,
                        (CELLLOCATE_GUARD_TIMEOUT_SECS + uuloc_timeout) * 1000,
                        uuloc_timeout_handler, modem);
  DEBUGASSERT(modem->cell_locate_timer_id >= 0);

  if (info->status == RESP_STATUS_TIMEOUT)
    {
      int ret;

      /* Timeout for ULOC command can mean only one thing: Modem is stuck in
       * bad state. */

      /* Prepare task for bringing modem alive from stuck state. */

      ret = __ubmodem_recover_stuck_hardware(modem);
      MODEM_DEBUGASSERT(modem, ret != ERROR);
      return;
    }
}

static int start_cell_locate(struct ubmodem_s *modem, void *privptr)
{
  struct cell_locate_priv_s priv = *(struct cell_locate_priv_s *)privptr;
  int err;

  free(privptr);

  if (modem->level < UBMODEM_LEVEL_GPRS)
    {
      send_celllocate_failed_event(modem);

      return ERROR;
    }

  if (!modem->cell_locate_urc_registered)
    {
      modem->cell_locate_timer_id = -1;
      modem->cell_locate_urc_registered = true;
      __ubparser_register_response_handler(&modem->parser, &urc_ATpUULOC,
                                       urc_cell_locate_handler, modem, true);
    }

  /* Initiate CellLocate, parameters:
   *
   *  1st: mode = 2 (single shot)
   *  2nd: sensor = 2 (CellLocate location information)
   *  3rd: response_type = 0 (standard response)
   *  4th: timeout (1 - 999 seconds)
   *  5th: target accuracy (1 - 999999 meters)
   */

  if (priv.timeout < 1)
    priv.timeout = 1;
  else if (priv.timeout > 999)
    priv.timeout = 999;

  if (priv.target_accuracy < 1)
    priv.target_accuracy = 1;
  else if (priv.target_accuracy > 999999)
    priv.target_accuracy = 999999;

  /* Launch CellLocate. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpULOC, set_ATpULOC_handler,
                       (void *)(uintptr_t)priv.timeout, "=2,2,0,%d,%d",
                       priv.timeout, priv.target_accuracy);
  MODEM_DEBUGASSERT(modem, err == OK);

  /* New CellLocate, inform power-management of low activity increase. */

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_LOW, true);

  return OK;
}

static void config_enable_deepscan(struct ubmodem_s *modem)
{
  int err;

  err = __ubmodem_send_cmd(modem, &cmd_ATpULOCCELL,
                           __ubmodem_cmdprompt_generic_config_handler,
                           (void *)&cmd_ATpULOCCELL, "=1");
  MODEM_DEBUGASSERT(modem, err == OK);
}
#endif

#ifndef CONFIG_UBMODEM_DISABLE_AID_CELLLOCATE
static void set_ATpULOCAID_handler(struct ubmodem_s *modem,
                                   const struct at_cmd_def_s *cmd,
                                   const struct at_resp_info_s *info,
                                   const uint8_t *resp_stream,
                                   size_t stream_len, void *priv)
{
  /*
   * Response handler for +ULOCAID command.
   */

  if (modem->support_cell_locate_aid == UBMODEM_CELL_LOCATE_SUPPORT_UNKNOWN)
    {
      /* Check if +ULOCAID is supported by modem, "+CME ERROR: 100" means
       * unknown command. */

      if (info->status == RESP_STATUS_CME_ERROR)
        {
          if (info->errorcode == 100)
            {
              modem->support_cell_locate_aid = UBMODEM_CELL_LOCATE_SUPPORT_NOK;
            }
        }
      else if (info->status == RESP_STATUS_OK)
        {
          modem->support_cell_locate_aid = UBMODEM_CELL_LOCATE_SUPPORT_OK;
        }
    }

  /* Update main state machine. */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);

  if (info->status == RESP_STATUS_TIMEOUT)
    {
      int ret;

      /* Timeout for ULOCAID command can mean only one thing: Modem is stuck in
       * bad state. */

      /* Prepare task for bringing modem alive from stuck state. */

      ret = __ubmodem_recover_stuck_hardware(modem);
      MODEM_DEBUGASSERT(modem, ret != ERROR);
      return;
    }
}

static int aid_cell_locate(struct ubmodem_s *modem, void *privptr)
{
  struct cell_locate_aid_priv_s priv = *(struct cell_locate_aid_priv_s *)privptr;
  struct tm tm;
  int err;

  free(privptr);

  if (modem->level < UBMODEM_LEVEL_CMD_PROMPT)
    {
      return ERROR;
    }

  if (modem->support_cell_locate_aid == UBMODEM_CELL_LOCATE_SUPPORT_NOK)
    {
      return ERROR;
    }

  if (gmtime_r(&priv.time, &tm) == NULL)
    {
      return ERROR;
    }

  tm.tm_year += 1900;
  tm.tm_mon += 1;
  if (tm.tm_sec > 59)
    tm.tm_sec = 59;

  /* Add aiding parameters for CellLocate. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpULOCAID, set_ATpULOCAID_handler, NULL,
      "=0,\"%02u/%02u/%04u\",\"%02u:%02u:%02u.000\",\"%.6f\",\"%.6f\",%u,%u,%u,%u,%u",
      tm.tm_mday, tm.tm_mon, tm.tm_year, tm.tm_hour, tm.tm_min, tm.tm_sec,
      priv.latitude, priv.longitude, priv.accuracy.major, priv.accuracy.minor,
      priv.accuracy.orientation, priv.speed, priv.direction);
  MODEM_DEBUGASSERT(modem, err == OK);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubmodem_start_cell_locate
 *
 * Description:
 *   Ask modem to perform CellLocate work. Modem must be in GPRS level.
 *
 * Input Parameters:
 *   modem           : Modem structure
 *   locate_timeout  : Timeout period (in seconds)
 *   allowed_accuracy: Target accuracy (in meters)
 *
 * Returned valued:
 *   ERROR if failed, OK on success.
 *
 ****************************************************************************/

#ifndef CONFIG_UBMODEM_DISABLE_CELLLOCATE
int ubmodem_start_cell_locate(struct ubmodem_s *modem, int locate_timeout,
                              int target_accuracy)
{
  struct cell_locate_priv_s *priv;

  DEBUGASSERT(modem);

  /* GPRS connectivity required for UDP/TCP connection. */

  if (modem->level < UBMODEM_LEVEL_GPRS)
    {
      errno = ENONET;
      return ERROR;
    }

  if (modem->cell_locate_urc_registered)
    {
      errno = EALREADY;
      return ERROR;
    }

  priv = calloc(1, sizeof(*priv));
  if (!priv)
    {
      return ERROR;
    }

  priv->timeout = locate_timeout;
  priv->target_accuracy = target_accuracy;

  /* Add modem task. */

  return __ubmodem_add_task(modem, start_cell_locate, priv);
}
#endif

/****************************************************************************
 * Name: __ubmodem_cell_locate_cleanup
 *
 * Description:
 *   Clean-up and free CellLocate state.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *   got_celllocate_event : True if CellLocate succeeded. Generate
 *                          CellLocate failed event if this is set to 'false'.
 *
 ****************************************************************************/

void __ubmodem_cell_locate_cleanup(struct ubmodem_s *modem,
                                   bool got_celllocate_event)
{
#ifndef CONFIG_UBMODEM_DISABLE_CELLLOCATE
  if (modem->cell_locate_timer_id >= 0)
    {
      __ubmodem_remove_timer(modem, modem->cell_locate_timer_id);
      modem->cell_locate_timer_id = -1;
    }

  if (!modem->cell_locate_urc_registered)
    {
      return;
    }

  if (!got_celllocate_event)
    {
      send_celllocate_failed_event(modem);
    }

  __ubparser_unregister_response_handler(&modem->parser, urc_ATpUULOC.name);
  modem->cell_locate_urc_registered = false;

  /* CellLocate completed, inform power-management of low activity decrease. */

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_LOW, false);
#endif
}

/****************************************************************************
 * Name: ubmodem_cell_locate_give_location_aid
 *
 * Description:
 *   Give modem external location information to aid Cell-Locate (+ULOCAID).
 *   Modem will send this information to u-blox CellLocate servers and
 *   potentially improve CellLocate database over time.
 *
 * Input Parameters:
 *   modem    : Modem structure
 *   time     : UTC timestamp of aided location
 *   latitude : Estimated latitude in degrees
 *   longitude: Estimated longitude in degrees
 *   altitude : Estimated altitude in meters
 *   accuracy : Estimated accuracy in meters
 *   speed    : Estimated speed in meters per second
 *   direction: Estimated direction of speed in degrees (north zero, clockwise)
 *
 * Returned valued:
 *   ERROR if failed, OK on success.
 *
 ****************************************************************************/

#ifndef CONFIG_UBMODEM_DISABLE_AID_CELLLOCATE
int ubmodem_cell_locate_give_location_aid(struct ubmodem_s *modem,
                                          time_t time, double latitude,
                                          double longitude, int altitude,
                                          unsigned int accuracy,
                                          unsigned int speed,
                                          unsigned int direction)
{
  struct cell_locate_aid_priv_s *priv;

  DEBUGASSERT(modem);

  if (modem->level < UBMODEM_LEVEL_CMD_PROMPT)
    {
      errno = ENONET;
      return ERROR;
    }

  if (modem->support_cell_locate_aid == UBMODEM_CELL_LOCATE_SUPPORT_NOK)
    {
      errno = ENOSYS;
      return ERROR;
    }

  priv = calloc(1, sizeof(*priv));
  if (!priv)
    {
      return ERROR;
    }

  priv->time = time;
  priv->latitude = latitude;
  priv->longitude = longitude;
  priv->altitude = altitude;
  priv->accuracy.major = accuracy;
  priv->accuracy.minor = accuracy;
  priv->accuracy.orientation = 0;
  priv->speed = speed;
  priv->direction = direction;

  /* Add modem task. */

  return __ubmodem_add_task(modem, aid_cell_locate, priv);
}
#endif

/****************************************************************************
 * Name: __ubmodem_celllocate_get_config_cmds
 *
 * Description:
 *   Get configuration command list for CellLocate.
 *
 * Input Parameters:
 *   modem    : Modem data
 *   ncmds    : Number of commands in array
 *
 * Return value:
 *   Pointer to command array
 *
 ****************************************************************************/

const ubmodem_send_config_func_t *
__ubmodem_celllocate_get_config_cmds(struct ubmodem_s *modem, size_t *ncmds)
{
#ifndef CONFIG_UBMODEM_DISABLE_CELLLOCATE
  static const ubmodem_send_config_func_t commands[] =
  {
   config_enable_deepscan,
  };

  *ncmds = ARRAY_SIZE(commands);

  return commands;
#else
  return NULL;
#endif
}
