/****************************************************************************
 * apps/system/ubmodem/ubmodem_cell_locate.c
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void urc_cell_locate_handler(struct ubmodem_s *modem,
                                    const struct at_cmd_def_s *cmd,
                                    const struct at_resp_info_s *info,
                                    const uint8_t *stream, size_t stream_len,
                                    void *priv)
{
  struct ubmodem_event_cell_location_s data = {};
  int ret, dummy;
  const char *date; /* "dd/mm/yyyy" */
  const char *time; /* "HH:MM:SS.sss" */
  const char *latitude; /* degrees, "dd.ddddd" */
  const char *longitude; /* degrees, "dd.ddddd" */
  int32_t altitude; /* meters from sea level */
  int32_t accuracy; /* meters of accuracy */

  /*
   * Response handler for +UULOC URC.
   */

  if (info->status != RESP_STATUS_URC)
    return;

  if (!__ubmodem_stream_get_string(&stream, &stream_len, &date, NULL))
    return;

  if (!__ubmodem_stream_get_string(&stream, &stream_len, &time, NULL))
    return;

  if (!__ubmodem_stream_get_string(&stream, &stream_len, &latitude, NULL))
    return;

  if (!__ubmodem_stream_get_string(&stream, &stream_len, &longitude, NULL))
    return;

  if (!__ubmodem_stream_get_int32(&stream, &stream_len, &altitude))
    return;

  if (!__ubmodem_stream_get_int32(&stream, &stream_len, &accuracy))
    return;

  /* Parse date. */

  ret = sscanf(date, "%d/%d/%d", &data.date.day, &data.date.month,
               &data.date.year);
  if (ret != 3)
    return;
  if (data.date.day < 1 || data.date.day > 31)
    return;
  if (data.date.month < 1 || data.date.month > 12)
    return;
  if (data.date.year < 2014 || data.date.year > 2999)
    return;

  /* Parse time. */

  ret = sscanf(time, "%d:%d:%d.%d", &data.date.hour, &data.date.min,
               &data.date.sec, &dummy);
  if (ret != 4)
    return;
  if (data.date.hour < 0 || data.date.hour > 23)
    return;
  if (data.date.min < 0 || data.date.min > 59)
    return;
  if (data.date.sec < 0 || data.date.sec > 59)
    return;

  /* Parse latitude. */

  data.location.latitude = strtod(latitude, NULL);
  if (data.location.latitude < 0.0000000001 &&
      data.location.latitude > -0.0000000001)
    return;

  /* Parse longitude. */

  data.location.longitude = strtod(longitude, NULL);
  if (data.location.longitude < 0.0000000001 &&
      data.location.longitude > -0.0000000001)
    return;

  data.location.accuracy = accuracy;
  data.location.altitude = altitude;

  /* Report CellLocation to application. */

  __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_CELL_LOCATION, &data,
                      sizeof(data));
}

static void set_ATpULOC_handler(struct ubmodem_s *modem,
                                const struct at_cmd_def_s *cmd,
                                const struct at_resp_info_s *info,
                                const uint8_t *resp_stream,
                                size_t stream_len, void *priv)
{
  /*
   * Response handler for +ULOC command.
   */

  /* Update main state machine. */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);
}

static int start_cell_locate(struct ubmodem_s *modem, void *priv)
{
  int err;

  if (modem->level < UBMODEM_LEVEL_GPRS)
    {
      return ERROR;
    }

  if (!modem->cell_locate_urc_registered)
    {
      modem->cell_locate_urc_registered = true;
      __ubparser_register_response_handler(&modem->parser, &urc_ATpUULOC,
                                       urc_cell_locate_handler, modem, true);
    }

  /* Initiate CellLocate, timeout 1 sec, allowed inaccuracy 99999 meters. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpULOC, set_ATpULOC_handler,
                       modem, "%s", "=2,2,0,30,20000");
  MODEM_DEBUGASSERT(modem, err == OK);

  return OK;
}

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
 *   None.
 *
 * Returned valued:
 *   ERROR if failed, OK on success.
 *
 ****************************************************************************/

int ubmodem_start_cell_locate(struct ubmodem_s *modem)
{
  DEBUGASSERT(modem);

  /* GPRS connectivity required for TCP connection. */

  if (modem->level < UBMODEM_LEVEL_GPRS)
    {
      errno = ENONET;
      return ERROR;
    }

  /* Add modem task. */

  return __ubmodem_add_task(modem, start_cell_locate, NULL);
}

/****************************************************************************
 * Name: __ubmodem_has_cell_locate_work
 *
 * Description:
 *   Check if CellLocate is active
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 * Return value:
 *   true if there is work waiting.
 *
 ****************************************************************************/

bool __ubmodem_has_cell_locate_work(struct ubmodem_s *modem)
{
  return modem->cell_locate_urc_registered &&
         modem->level >= UBMODEM_LEVEL_GPRS;
}

/****************************************************************************
 * Name: __ubmodem_cell_locate_cleanup
 *
 * Description:
 *   Clean-up and free CellLocate state.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 ****************************************************************************/

void __ubmodem_cell_locate_cleanup(struct ubmodem_s *modem)
{
  if (!modem->cell_locate_urc_registered)
    return;

  __ubparser_unregister_response_handler(&modem->parser, urc_ATpUULOC.name);
  modem->cell_locate_urc_registered = false;
}
