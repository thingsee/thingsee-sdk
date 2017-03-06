/****************************************************************************
 * apps/system/ubmodem/ubmodem_cell_environment.c
 *
 *   Copyright (C) 2016-2017 Haltian Ltd. All rights reserved.
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

#ifndef CONFIG_UBMODEM_DISABLE_CELL_ENVIRONMENT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_NEIGHBORS UBMODEM_MAX_CELL_ENVIRONMENT_NEIGHBORS

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

enum cged_parser_state_e
{
  CGED_INIT = 0,
  CGED_START,
  CGED_SERVICE_CELL,
  CGED_NEIGHBOR_CELL,
  CGED_DONE,
};

struct cell_env_priv_s
{
  enum cged_parser_state_e state;
  enum cged_parser_state_e cged_section;
  unsigned int neighbor_count;
  struct ubmodem_event_cell_environment_s data;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at_cmd_def_s cmd_ATpCGED =
{
  .name               = "+CGED",
  .resp_format        = (const uint8_t[]){ RESP_FMT_STRING_TO_EOL },
  .resp_num           = 1,
  .flag_multiple_resp = true,
  .flag_plain         = true,
};

static const struct at_cmd_def_s cmd_ATpUCELLINFO =
{
  .name               = "+UCELLINFO",
  .resp_format        = (const uint8_t[]){
                            RESP_FMT_INT8,
                            RESP_FMT_INT8,
                            RESP_FMT_STRING,
                            RESP_FMT_STRING,
                            RESP_FMT_STRING,
                            RESP_FMT_STRING,
                            RESP_FMT_STRING,
                            RESP_FMT_STRING,
                            RESP_FMT_STRING,
                            RESP_FMT_STRING_TO_EOL
                        },
  .resp_num           = 10,
  .timeout_dsec       = 7 * 10,
  .flag_multiple_resp = true,
  .flag_plain         = false,
};

static const struct at_cmd_def_s cmd_ATpCSQ =
{
  .name               = "+CSQ",
  .resp_format        = (const uint8_t[]){ RESP_FMT_INT8, RESP_FMT_INT8 },
  .resp_num           = 2,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void parse_cged_line(struct ubmodem_s *modem,
                            struct cell_env_priv_s *env_priv,
                            const char *line)
{
  static const char g_neighbour_cell_[] = "Neighbour Cell ";
  static const char g_service_cell_0[] = "Service-Cell:";
  static const char g_service_cell_1[] = "Service Cell:";
  static const char g_mcc_[] = "MCC:";
  static const char g_arfcn_[] = "Arfcn:";
  int nscan;

  switch (env_priv->state)
    {
    case CGED_INIT:
      env_priv->state = CGED_START;
      env_priv->cged_section = CGED_INIT;
      /* no break */

    case CGED_START:
      {
        if (strncmp(line, g_service_cell_0, sizeof(g_service_cell_0) - 1) == 0 ||
            strncmp(line, g_service_cell_1, sizeof(g_service_cell_1) - 1) == 0)
          {
            /* Next line should start with "MCC:" and have service cell
             * information. */

            env_priv->state = CGED_SERVICE_CELL;
            env_priv->cged_section = CGED_SERVICE_CELL;
          }
        else if (strncmp(line, g_neighbour_cell_, sizeof(g_neighbour_cell_) - 1) == 0)
          {
            /* Next line should start with "MCC:" and have neighbor cell
             * information. */

            env_priv->state = CGED_NEIGHBOR_CELL;
            env_priv->cged_section = CGED_NEIGHBOR_CELL;
          }
        else if (env_priv->cged_section == CGED_SERVICE_CELL &&
                 strncmp(line, g_arfcn_, sizeof(g_arfcn_) - 1) == 0)
          {
            const char *str_rxlevserv = strstr(line, "RxLevServ:");
            unsigned int rxlev;
            unsigned int arfcn;

            if (sscanf(line, "Arfcn: %d", &arfcn) == 1)
              {
                env_priv->data.serving.arfcn = arfcn;
              }

            if (sscanf(str_rxlevserv, "RxLevServ: %d", &rxlev) == 1)
              {
                if (rxlev >= 0 && rxlev <= 63)
                  {
                    env_priv->data.serving.signal_dbm = -110 + rxlev;
                  }
              }
          }
      }
      break;

    case CGED_SERVICE_CELL:
    case CGED_NEIGHBOR_CELL:
      {
        if (strncmp(line, g_mcc_, sizeof(g_mcc_) - 1) == 0)
          {
            int scans[5] = {};

            nscan = sscanf(line,
                           "MCC: %d, MNC: %d, LAC: %x, CI: %x, BSIC: %x",
                           &scans[0], &scans[1], &scans[2], &scans[3],
                           &scans[4]);
            if (nscan == 5)
              {
                if (env_priv->state == CGED_SERVICE_CELL)
                  {
                    if (!env_priv->data.have_serving)
                      {
                        env_priv->data.have_serving = true;
                        env_priv->data.serving.rat = UBMODEM_RAT_GSM;
                        env_priv->data.serving.mcc = scans[0];
                        env_priv->data.serving.mnc = scans[1];
                        env_priv->data.serving.lac = scans[2];
                        env_priv->data.serving.cell_id = scans[3];
                        env_priv->data.serving.bsic = scans[4];
                        env_priv->data.serving.signal_dbm = INT16_MIN;
                        env_priv->data.serving.arfcn = 0xffff;
                      }
                  }
                else if (env_priv->state == CGED_NEIGHBOR_CELL)
                  {
                    const char *str_rxlev = strstr(line, "RxLev:");
                    unsigned int rxlev = 0xffffffff;
                    const char *str_arfcn = strstr(line, "Arfcn:");
                    unsigned int arfcn = 0xffff;
                    int tmp;

                    if (str_rxlev &&
                        sscanf(str_rxlev, "RxLev: %d", &tmp) == 1)
                      {
                        rxlev = tmp;
                      }

                    if (str_arfcn &&
                        sscanf(str_arfcn, "Arfcn: %d", &tmp) == 1)
                      {
                        arfcn = tmp;
                      }

                    if (env_priv->data.num_neighbors < MAX_NEIGHBORS)
                      {
                        struct ubmodem_event_cell_env_neighbor_s *neighbor =
                          &env_priv->data.neighbors[env_priv->data.num_neighbors];

                        memset(neighbor, 0, sizeof(*neighbor));

                        neighbor->rat = UBMODEM_RAT_GSM;
                        neighbor->have_mcc_mnc_lac =
                            (scans[0] >= 0 && scans[0] <= 999) &&
                            (scans[1] >= 0 && scans[1] <= 999) &&
                            (scans[2] >= 0 && scans[2] < 0xffff);
                        neighbor->have_cellid = !(scans[3] == 0xffff);
                        neighbor->have_bsic =
                            (scans[4] >= 0 && scans[4] <= 0x3f);
                        neighbor->have_arfcn =
                            (arfcn >= 0 && arfcn < 0xffff);
                        neighbor->mcc = scans[0];
                        neighbor->mnc = scans[1];
                        neighbor->lac = scans[2];
                        neighbor->cell_id = scans[3];
                        neighbor->bsic = scans[4];
                        neighbor->arfcn = arfcn;
                        neighbor->signal_dbm = INT16_MIN;

                        if (rxlev >= 0 && rxlev <= 63)
                          {
                            neighbor->have_signal_dbm = true;
                            neighbor->signal_dbm = -110 + rxlev;
                          }

                        if (neighbor->have_mcc_mnc_lac ||
                            neighbor->have_cellid)
                          {
                            env_priv->data.num_neighbors++;
                          }
                      }
                  }
              }
          }

        if (env_priv->data.num_neighbors >= MAX_NEIGHBORS &&
            env_priv->data.have_serving)
          {
            env_priv->state = CGED_DONE;
          }
        else
          {
            env_priv->state = CGED_START;
          }
      }
      break;

    case CGED_DONE:
      break;
    }
}

static void ATpCGED_handler(struct ubmodem_s *modem,
                            const struct at_cmd_def_s *cmd,
                            const struct at_resp_info_s *info,
                            const uint8_t *stream,
                            size_t stream_len, void *priv)
{
  struct cell_env_priv_s *env_priv = priv;
  uint16_t linelen = 0;
  const char *line = NULL;

  /*
   * Response handler for +CGED=0 command.
   */

  if (info->status == RESP_STATUS_LINE)
    {
      if (!__ubmodem_stream_get_string(&stream, &stream_len, &line, &linelen))
        return;

      if (linelen)
        parse_cged_line(modem, env_priv, line);
    }
  else
    {
      /* Other response type received, command completed. */

      if (env_priv->data.have_serving || env_priv->data.num_neighbors ||
          env_priv->data.have_signal_qual)
        {
          struct ubmodem_event_cell_environment_s *event = &env_priv->data;
          size_t event_size;

          event_size = sizeof(*event);
          event_size += event->num_neighbors * sizeof(event->neighbors[0]);

          /* Report cell environment to application. */

          __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_CELL_ENVIRONMENT,
                                  event, event_size);
        }

      free(env_priv);

      /* Update main state machine. */

      __ubmodem_change_state(modem, MODEM_STATE_WAITING);
    }
}

static void ATpUCELLINFO_handler(struct ubmodem_s *modem,
                                 const struct at_cmd_def_s *cmd,
                                 const struct at_resp_info_s *info,
                                 const uint8_t *stream,
                                 size_t stream_len, void *priv)
{
  struct cell_env_priv_s *env_priv = priv;

  /*
   * Response handler for +UCELLINFO? command.
   */

  if (info->status == RESP_STATUS_LINE)
    {
      int8_t mode = -1;
      int8_t type = -1;
      int32_t mcc = -1;
      int32_t mnc = -1;
      const char *str_mcc;
      const char *str_mnc;
      const char *str_lac;
      const char *str_cell_id;
      const char *str_rxlev;
      bool serving;
      bool mcc_mnc_lac_valid = true;
      enum ubmodem_rat_type_e rat;
      unsigned int lac = 0xffffffff;
      unsigned int cell_id = 0xffffffff;
      unsigned int rxlev = 0xffffffff;
      unsigned int sc = 0xffffffff;
      unsigned int rscp_lev = 0xffffffff;

      if (!__ubmodem_stream_get_int8(&stream, &stream_len, &mode) || mode != 0)
        return;

      if (!__ubmodem_stream_get_int8(&stream, &stream_len, &type))
        return;

      serving = (type == 0) || (type == 2);

      if (!__ubmodem_stream_get_string(&stream, &stream_len, &str_mcc, NULL))
        return;

      if (!__ubmodem_stream_get_string(&stream, &stream_len, &str_mnc, NULL))
        return;

      /* Modem reports report FFFF for MCC/MNC if not avail, otherwise base 10 number. */

      mcc = (strtoul(str_mcc, NULL, 16) == 0xffff) ? 0xffff : strtoul(str_mcc, NULL, 10);
      if (mcc < 0 || mcc > 999)
        {
          mcc = 0xffff;
          mcc_mnc_lac_valid = false;
        }

      mnc = (strtoul(str_mnc, NULL, 16) == 0xffff) ? 0xffff : strtoul(str_mnc, NULL, 10);
      if (mnc < 0 || mnc > 999)
        {
          mnc = 0xffff;
          mcc_mnc_lac_valid = false;
        }

      if (!__ubmodem_stream_get_string(&stream, &stream_len, &str_lac, NULL))
        return;

      if (!__ubmodem_stream_get_string(&stream, &stream_len, &str_cell_id, NULL))
        return;

      if (type <= 1)
        {
          rat = UBMODEM_RAT_GSM;

          if (!__ubmodem_stream_get_string(&stream, &stream_len, &str_rxlev, NULL))
            return;

          rxlev = strtoul(str_rxlev, NULL, 10);
        }
      else if (type <= 4)
        {
          const char *str_sc = NULL;
          const char *str_dl_freq;
          const char *str_rscp_lev = NULL;

          rat = UBMODEM_RAT_UMTS;

          if (!__ubmodem_stream_get_string(&stream, &stream_len, &str_sc, NULL))
            return;

          if (!__ubmodem_stream_get_string(&stream, &stream_len, &str_dl_freq, NULL))
            return;

          if (!__ubmodem_stream_get_string(&stream, &stream_len, &str_rscp_lev, NULL))
            return;

          sc = strtoul(str_sc, NULL, 16);
          rscp_lev = strtoul(str_rscp_lev, NULL, 10);
        }
      else
        {
          return; /* Unknown type. */
        }

      lac = strtoul(str_lac, NULL, 16);
      if (lac < 0 || lac > 0xffff)
        {
          lac = 0xffff;
          mcc_mnc_lac_valid = false;
        }

      cell_id = strtoul(str_cell_id, NULL, 16);
      if (rat == UBMODEM_RAT_GSM)
        {
          if (cell_id < 0 || cell_id >= 0xffff)
            {
              cell_id = 0xffff;
            }
        }
      else
        {
          if (cell_id < 0 || cell_id >= 0xfffffff || cell_id == 0xffff)
            {
              cell_id = 0xfffffff;
            }
        }

      if (serving && !env_priv->data.have_serving)
        {
          /* Serving cell information. */

          env_priv->data.have_serving = true;
          env_priv->data.serving.rat = rat;
          env_priv->data.serving.mcc = mcc;
          env_priv->data.serving.mnc = mnc;
          env_priv->data.serving.lac = lac;
          env_priv->data.serving.cell_id = cell_id;
          env_priv->data.serving.signal_dbm = INT16_MIN;

          if (rat == UBMODEM_RAT_UMTS)
            {
              env_priv->data.serving.sc = sc;

              if (rscp_lev >= 0 && rscp_lev <= 91)
                {
                  env_priv->data.serving.signal_dbm = -115 + rscp_lev;
                }
            }
          else
            {
              if (rxlev >= 0 && rxlev <= 63)
                {
                  env_priv->data.serving.signal_dbm = -110 + rxlev;
                }
            }
        }
      else
        {
          /* Neighbor cell information. */

          if (env_priv->data.num_neighbors < MAX_NEIGHBORS)
            {
              struct ubmodem_event_cell_env_neighbor_s *neighbor =
                &env_priv->data.neighbors[env_priv->data.num_neighbors];

              memset(neighbor, 0, sizeof(*neighbor));

              neighbor->rat = rat;
              neighbor->have_mcc_mnc_lac = mcc_mnc_lac_valid;

              if (rat == UBMODEM_RAT_GSM)
                neighbor->have_cellid = (cell_id < 0xffff);
              else
                neighbor->have_cellid = (cell_id < 0xfffffff);

              neighbor->have_sc = (sc <= 511);
              neighbor->mcc = mcc;
              neighbor->mnc = mnc;
              neighbor->lac = lac;
              neighbor->cell_id = cell_id;
              neighbor->sc = sc;
              neighbor->signal_dbm = INT16_MIN;

              if (rat == UBMODEM_RAT_UMTS)
                {
                  if (rscp_lev >= 0 && rscp_lev <= 91)
                    {
                      neighbor->have_signal_dbm = true;
                      neighbor->signal_dbm = -115 + rscp_lev;
                    }
                }
              else
                {
                  if (rxlev >= 0 && rxlev <= 63)
                    {
                      neighbor->have_signal_dbm = true;
                      neighbor->signal_dbm = -110 + rxlev;
                    }
                }

              if (neighbor->have_mcc_mnc_lac ||
                  neighbor->have_cellid ||
                  neighbor->have_sc)
                {
                  env_priv->data.num_neighbors++;
                }
            }
        }
    }
  else
    {
      /* Other response type received, command completed. */

      if (env_priv->data.have_serving || env_priv->data.num_neighbors ||
          env_priv->data.have_signal_qual)
        {
          struct ubmodem_event_cell_environment_s *event = &env_priv->data;
          size_t event_size;

          event_size = sizeof(*event);
          event_size += event->num_neighbors * sizeof(event->neighbors[0]);

          /* Report cell environment to application. */

          __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_CELL_ENVIRONMENT,
                                  event, event_size);
        }

      free(env_priv);

      /* Update main state machine. */

      __ubmodem_change_state(modem, MODEM_STATE_WAITING);
    }
}

static void ATpCSQ_handler(struct ubmodem_s *modem,
                           const struct at_cmd_def_s *cmd,
                           const struct at_resp_info_s *info,
                           const uint8_t *stream,
                           size_t stream_len, void *priv)
{
  struct cell_env_priv_s *env_priv = priv;
  int8_t rssi_val = -1;
  int8_t qual_val = -1;
  int16_t rssi;
  int8_t qual;
  int err;

  /*
   * Response handler for +CSQ command.
   */

  if (info->status == RESP_STATUS_OK)
    {
      if (__ubmodem_stream_get_int8(&stream, &stream_len, &rssi_val) &&
          __ubmodem_stream_get_int8(&stream, &stream_len, &qual_val))
        {
          /* GSM & UMTS, 0 <= rssi <= 31 */

          if (rssi_val < 0 || rssi_val > 31)
            rssi = INT16_MIN;
          else
            rssi = -113 + rssi_val * 2;

          if (qual_val < 0 || qual_val > 7)
            qual = INT8_MIN;
          else
            qual = qual_val;

          env_priv->data.have_signal_qual = (rssi > INT16_MIN || qual > INT8_MIN);
          env_priv->data.signal_qual.rssi = rssi;
          env_priv->data.signal_qual.qual = qual;
        }
    }

  if (__ubmodem_is_model_sara_g(modem))
    {
      /* Get cell information through +CGED. */

      err = __ubmodem_send_cmd(modem, &cmd_ATpCGED, ATpCGED_handler,
                               env_priv, "=0");
      MODEM_DEBUGASSERT(modem, err == OK);
    }
  else
    {
      /* Get cell information with +UCELLINFO. */

      err = __ubmodem_send_cmd(modem, &cmd_ATpUCELLINFO, ATpUCELLINFO_handler,
                               env_priv, "?");
      MODEM_DEBUGASSERT(modem, err == OK);
    }
}

static int request_cell_environment(struct ubmodem_s *modem, void *priv)
{
  struct cell_env_priv_s *env_priv = priv;
  int err;

  if (modem->level < UBMODEM_LEVEL_CMD_PROMPT)
    {
      free(priv);
      return ERROR;
    }

  err = __ubmodem_send_cmd(modem, &cmd_ATpCSQ, ATpCSQ_handler,
                           env_priv, "");
  MODEM_DEBUGASSERT(modem, err == OK);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubmodem_request_cell_environment
 *
 * Description:
 *   Ask cell environment information from modem
 *
 * Input Parameters:
 *   modem           : Modem structure
 *
 * Returned valued:
 *   ERROR if failed, OK on success.
 *
 ****************************************************************************/

int ubmodem_request_cell_environment(struct ubmodem_s *modem)
{
  struct cell_env_priv_s *env_priv;

  DEBUGASSERT(modem);

  if (modem->level < UBMODEM_LEVEL_CMD_PROMPT)
    {
      errno = ENODEV;
      return ERROR;
    }

  env_priv = calloc(1, sizeof(*env_priv)
                       + MAX_NEIGHBORS * sizeof(env_priv->data.neighbors[0]));
  if (!env_priv)
    {
      return ERROR;
    }

  env_priv->state = CGED_INIT;

  /* Add modem task. */

  return __ubmodem_add_task(modem, request_cell_environment, env_priv);
}

#endif /* !CONFIG_UBMODEM_DISABLE_CELL_ENVIRONMENT */
