/****************************************************************************
 * apps/system/ubgps/ubgps_state.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Sami Pelkonen <sami.pelkonen@haltian.com>
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

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <debug.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <nuttx/random.h>
#include <apps/thingsee/ts_core.h>
#include <arch/board/board-gps.h>

#include "ubgps_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_UBGPS_DEBUG
 #define dbg_sm(...) dbg(__VA_ARGS__)
#else
  #define dbg_sm(...)
#endif

#define GPS_POWERUP_DELAY             500

#define GPS_INIT_RETRY_COUNT          5
#define GPS_RETRY_DELAY               50

#define GPS_INITIAL_BAUD_RATE         9600
#define GPS_BAUD_RATE                 115200

/* Timeout in ms to poll aiding data status */

#define AID_STATUS_POLL_DELAY         30000

/* Minimum time period in seconds between subsequent aiding update attempts */

#define AID_RECHECK_DELAY             (1*60*60)

/* Aiding validity time in seconds for 1 day assistance data. */

#define AID_VALIDITY_TIME_1D          (24*60*60)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum gps_init_phase_e
{
  INIT_PHASE_CFG_PRT = 0,
  INIT_PHASE_CFG_PRT_BAUD,
#ifdef CONFIG_SYSTEM_UBGPS_LNA_CONTROL
  INIT_PHASE_CFG_ANT_LNA,
#endif
  INIT_PHASE_SBAS_MODE,
  INIT_PHASE_RECEIVER_MODE,
  INIT_PHASE_PM_MODE,
  INIT_PHASE_AID_SET_TIME,
  INIT_PHASE_AID_ALPSRV_ENABLE,
  INIT_PHASE_NAV_PVT_RATE,
  INIT_PHASE_NAVIGATION_RATE,
  INIT_PHASE_DONE,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ubgps_sm_poweroff(struct ubgps_s * const gps, struct sm_event_s const * const event);
static int ubgps_sm_initialization(struct ubgps_s * const gps, struct sm_event_s const * const event);
static int ubgps_sm_cold_start(struct ubgps_s * const gps, struct sm_event_s const * const event);
static int ubgps_sm_global(struct ubgps_s * const gps, struct sm_event_s const * const event);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* GPS state machine states */

const gps_sm_t gps_sm[__GPS_STATE_MAX] =
{
  [GPS_STATE_POWER_OFF] = ubgps_sm_poweroff,
  [GPS_STATE_INITIALIZATION] = ubgps_sm_initialization,
  [GPS_STATE_COLD_START] = ubgps_sm_cold_start,
  [GPS_STATE_SEARCHING_FIX] = ubgps_sm_global,
  [GPS_STATE_FIX_ACQUIRED] = ubgps_sm_global,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubgps_sm_poweroff
 *
 * Description:
 *   State machine for GPS power off
 *
 ****************************************************************************/
static int ubgps_sm_poweroff(struct ubgps_s * const gps, struct sm_event_s const * const event)
{
  DEBUGASSERT(gps && event);

  /* Check event */

  switch (event->id)
    {
      case SM_EVENT_ENTRY:
        {
          dbg_sm("SM_EVENT_ENTRY\n");
          if (gps->state.powered)
            {
              /* Power down GPS chip */

              board_gps_power(false);

              /* Mark GPS power down */

              gps->state.powered = false;

              /* Reset UBX receiver */

              ubx_reset(gps);
            }

          /* Report target reached status */

          ubgps_report_target_state(gps, false);

          /* Reset PSM accuracy filter */

          gps->filt_location.horizontal_accuracy = 0;

          /* Make sure that timers are not running */

          if (gps->state.location_timer_id >= 0)
            {
              __ubgps_remove_timer(gps, gps->state.location_timer_id);
              gps->state.location_timer_id = -1;
            }

#ifdef CONFIG_UBGPS_ASSIST_UPDATER
          /* Stop aiding updater and timers */

          if (gps->state.aid_timer_id >= 0)
            {
              __ubgps_remove_timer(gps, gps->state.aid_timer_id);
              gps->state.aid_timer_id = -1;
            }

          if (gps->assist)
            {
              if (ubgps_aid_updater_stop(gps->assist))
                {
                  dbg("Aiding updater stop failed\n");
                }
            }
#endif
          /* Free GPS assistance data if PSM is not active */

          if (gps->assist && gps->state.psm_timer_id < 0)
            {
              if (gps->assist->alp_file)
                free(gps->assist->alp_file);

              free(gps->assist);
              gps->assist = NULL;
            }

          return OK;
        }

      case SM_EVENT_TARGET_STATE:
        {
          struct sm_event_target_state_s * const target =
              (struct sm_event_target_state_s *)event;

          dbg_sm("SM_EVENT_TARGET_STATE -> %u\n", target->target_state);

          /* Stop PSM timer */

          if (gps->state.psm_timer_id > 0)
            {
              ts_core_timer_stop(gps->state.psm_timer_id);
              gps->state.psm_timer_id = -1;
            }

          if (target->target_state != GPS_STATE_POWER_OFF)
            {
              /* Setup target state transition timeout */

              ubgps_setup_timeout(gps, target->timeout);

              /* Set GPS target state */

              gps->state.target_state = target->target_state;

              /* Set next GPS state */

              ubgps_set_new_state(gps, GPS_STATE_INITIALIZATION);
            }
          else
            {
              /* Make current state the target state */

              /* Free GPS assistance data */

              if (gps->assist)
                {
                  if (gps->assist->alp_file)
                    free(gps->assist->alp_file);

                  free(gps->assist);
                  gps->assist = NULL;
                }

              gps->state.target_state = gps->state.current_state;

              /* Report target reached */

              ubgps_report_target_state(gps, false);
            }

          return OK;
        }

      case SM_EVENT_PSM_STATE:
        {
          struct sm_event_psm_event_s const * const psm_event =
              (struct sm_event_psm_event_s *)event;
          dbg_sm("SM_EVENT_PSM_STATE -> %u\n", psm_event->enable);

          if (!psm_event->enable)
            {
              gps->state.target_state = GPS_STATE_FIX_ACQUIRED;
              ubgps_set_new_state(gps, GPS_STATE_INITIALIZATION);
            }

          return OK;
        }

      case SM_EVENT_EXIT:
        {
          dbg_sm("SM_EVENT_EXIT\n");

          if (!gps->state.powered)
            {
              struct termios uart;
              int ret;

              /* Configure initial baud rate to GPS UART */

              ret = ioctl(gps->fd, TCGETS, (unsigned long)&uart);
              DEBUGASSERT(ret == OK);

              uart.c_cflag = CS8;
              uart.c_speed = GPS_INITIAL_BAUD_RATE;

              ret = ioctl(gps->fd, TCSETS, (unsigned long)&uart);
              DEBUGASSERT(ret == OK);

              /* Power up GPS chip */

              board_gps_power(true);

              /* Mark GPS power up */

              gps->state.powered = true;
            }

          return OK;
        }

      default:
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: ubgps_sm_global
 *
 * Description:
 *   Global handler for gps_sm_searching_fix and gps_sm_fix_acquired states
 *
 ****************************************************************************/
static int ubgps_sm_global(struct ubgps_s * const gps, struct sm_event_s const * const event)
{
  DEBUGASSERT(gps && event);

  /* Check event */

  switch (event->id)
    {
      case SM_EVENT_ENTRY:
        {
          dbg_sm("SM_EVENT_ENTRY\n");
          if (gps->state.current_state == gps->state.target_state)
            {
              /* Report target reached */

              ubgps_report_target_state(gps, false);
            }

          if (gps->assist)
            {
              dbg_sm("start polling aiding status\n");

              (void)ubgps_send_aid_alp_poll(gps);
            }

          return OK;
        }

      case SM_EVENT_TARGET_STATE:
        {
          struct sm_event_target_state_s const * const target =
              (struct sm_event_target_state_s *)event;

          dbg_sm("SM_EVENT_TARGET_STATE\n");

          if (target->target_state != gps->state.current_state)
            {
              /* Setup target state transition timeout */

              ubgps_setup_timeout(gps, target->timeout);

              /* Set GPS target state */

              gps->state.target_state = target->target_state;

              /* Check current and target state */

              if (gps->state.target_state == GPS_STATE_FIX_ACQUIRED &&
                  gps->state.current_state == GPS_STATE_COLD_START)
                {
                  /* Move to GPS initialization for GPS aiding parameters */

                  ubgps_set_new_state(gps, GPS_STATE_INITIALIZATION);
                }
              else if (gps->state.target_state == GPS_STATE_FIX_ACQUIRED &&
                  gps->state.current_state < GPS_STATE_FIX_ACQUIRED)
                {
                  /* Move to search for GPS fix */

                  ubgps_set_new_state(gps, GPS_STATE_SEARCHING_FIX);
                }
              else
                {
                  /* Move to requested target state */

                  ubgps_set_new_state(gps, gps->state.target_state);
                }
            }
          else
            {
              /* Make current state the target state */

              gps->state.target_state = gps->state.current_state;

              /* Report target reached */

              ubgps_report_target_state(gps, false);
            }

          return OK;
        }

      case SM_EVENT_TIMEOUT:
        {
          struct sm_event_timeout_s const * const timeout =
              (struct sm_event_timeout_s *)event;
          dbg_sm("SM_EVENT_TIMEOUT\n");

          if (timeout->timer_id == gps->state.target_timer_id)
            {
              /* Report target state timeout */

              ubgps_report_target_state(gps, true);
            }
          else if (timeout->timer_id == gps->state.aid_timer_id)
            {
              /* poll whether ALP data has been expired */

              return ubgps_send_aid_alp_poll(gps);
            }
#ifdef CONFIG_UBGPS_PSM_MODE
          else if (timeout->timer_id == gps->state.location_timer_id)
            {
              struct gps_event_location_s levent;

              /* Publish the most accurate location event received so far. */

              dbg_sm("location filter timeout, accuracy: %um\n",
                  gps->filt_location.horizontal_accuracy / 1000);

              levent.super.id = GPS_EVENT_LOCATION;
              levent.time = &gps->time;
              levent.location = &gps->filt_location;
              ubgps_publish_event(gps, (struct gps_event_s *)&levent);

              /* Reset filtered location event. */

              gps->filt_location.horizontal_accuracy = 0;

              /* Construct power save mode event */

              if (gps->state.navigation_rate >= CONFIG_UBGPS_PSM_MODE_THRESHOLD)
                {
                  struct sm_event_psm_event_s psm;

                  psm.super.id = SM_EVENT_PSM_STATE;
                  psm.enable = true;
                  ubgps_sm_process(gps, (struct sm_event_s *)&psm);
                }
            }
#endif /* CONFIG_UBGPS_PSM_MODE */

          return OK;
        }

      case SM_EVENT_UBX_MESSAGE:
        {
          struct sm_event_ubx_msg_s const * const ubx_msg =
              (struct sm_event_ubx_msg_s *)event;
          struct ubx_msg_s const * const msg = ubx_msg->msg;

          if (msg->class_id == UBX_CLASS_NAV && msg->msg_id == UBX_NAV_PVT)
            {
              return ubgps_handle_nav_pvt(gps, msg);
            }
          else if (msg->class_id == UBX_CLASS_AID && msg->msg_id == UBX_AID_ALPSRV)
            {
              return ubgps_handle_aid_alpsrv(gps, msg);
            }
          else if (msg->class_id == UBX_CLASS_AID && msg->msg_id == UBX_AID_ALP)
            {
              return ubgps_handle_aid_alp(gps, msg);
            }

          dbg_sm("Unhandled UBX message, class:0x%02x, msg:0x%02x, len:%d.\n",
              ubx_msg->msg->class_id, ubx_msg->msg->msg_id, ubx_msg->msg->length);

          return OK;
        }

      case SM_EVENT_AID_STATUS:
        {
#ifdef CONFIG_UBGPS_ASSIST_UPDATER
          struct sm_event_aid_s const * const aid = (struct sm_event_aid_s *)event;
          struct timespec ts = {};
          uint32_t timeout = 0;
          dbg_sm("SM_EVENT_AID_STATUS\n");

          /* Check whether aiding has been enabled */

          if (!gps->assist)
            {
              return OK;
            }

          if (!gps->assist->alp_file)
            {
              /* Aiding file does not exist, start updater */

              ubgps_aid_updater_start(gps->assist);
              timeout = AID_STATUS_POLL_DELAY;
            }
          else if (aid->age < 0)
            {
              /* Status of aiding data not yet resolved */

              timeout = AID_STATUS_POLL_DELAY;
            }
          else if (aid->age < AID_VALIDITY_TIME_1D )
            {
              /* Aiding still valid, setup recheck timeout */

              timeout = AID_RECHECK_DELAY*1000;
            }
          else
            {
              /* Aiding data needs to be updated */

              clock_gettime(CLOCK_MONOTONIC, &ts);

              /* But first add nanoseconds to entropy pool. */

              add_time_randomness(ts.tv_nsec);

              if (gps->assist->update_time == 0 ||
                 (ts.tv_sec - gps->assist->update_time) > AID_RECHECK_DELAY)
                {
                  /* The first update attempt or retry after timeout */

                  dbg_sm("start aiding update, previous update: %ds\n",
                    ts.tv_sec - gps->assist->update_time);

                  gps->assist->update_time = ts.tv_sec;
                  ubgps_aid_updater_start(gps->assist);
                  timeout = AID_STATUS_POLL_DELAY;
                }
              else
                {
                  /* Set timeout for the next update attempt */

                  timeout = AID_RECHECK_DELAY*1000;
                }
            }

            if (gps->state.aid_timer_id >= 0)
              {
                __ubgps_remove_timer(gps, gps->state.aid_timer_id);
              }

            gps->state.aid_timer_id = __ubgps_set_timer(gps,
                                                       timeout,
                                                       ubgps_timeout,
                                                       gps);
#endif /* CONFIG_UBGPS_ASSIST_UPDATER */
          return OK;
        }

#ifdef CONFIG_UBGPS_PSM_MODE
      case SM_EVENT_PSM_STATE:
        {
          struct sm_event_psm_event_s const * const psm_event =
              (struct sm_event_psm_event_s *)event;
          dbg_sm("SM_EVENT_PSM_STATE -> %u\n", psm_event->enable);

          if (psm_event->enable)
            {
              /* Start SW controlled power save mode (PSM). */

              if (gps->state.navigation_rate >= CONFIG_UBGPS_PSM_MODE_THRESHOLD)
                {
                  struct timespec ts = {};

                  clock_gettime(CLOCK_MONOTONIC, &ts);
                  ts.tv_sec += gps->state.navigation_rate/1000;
                  gps->state.psm_timer_id = ts_core_timer_setup_date(&ts,
                      ubgps_psm_timer_cb, gps);

                  dbg_sm("gps->state.psm_timer_id:%d, set POWER_OFF\n",
                      gps->state.psm_timer_id);
                  ubgps_set_new_state(gps, GPS_STATE_POWER_OFF);
                }
            }

          return OK;
        }
#endif /* CONFIG_UBGPS_PSM_MODE */

      default:
        return OK;
    }
}

/****************************************************************************
 * Name: ubgps_init_process_phase
 *
 * Description:
 *   Send UBX messages according to initialization phase
 *
 ****************************************************************************/
static int ubgps_init_process_phase(struct ubgps_s * const gps, bool next)
{
  bool waiting_for_response = false;
  int status = OK;
  static bool init_retry_done = false;

  DEBUGASSERT(gps);

  /* Moving to next initialization phase */

  if (next)
    {
      /* Reset retry counter */

      gps->state.init_count = 0;

      /* Move to next phase */

      gps->state.init_phase++;
    }

  if (gps->state.init_count > 0)
    ubx_reset(gps);

  if (gps->state.init_count++ == (GPS_INIT_RETRY_COUNT + 1))
    {
      dbg("GPS initialization failed at phase %d, retries %d.\n",
          gps->state.init_phase, GPS_INIT_RETRY_COUNT);

      if (init_retry_done)
        {
          /* Fail back to power down state */

          ubgps_set_new_state(gps, GPS_STATE_POWER_OFF);

          return ERROR;
        }
      else
        {
          /* Workaround for rarely happening initialization failures. Try to
             recover by toggling supply voltage and starting init from the
             beginning. */

          struct termios uart;
          int ret;

          dbg_sm("Retry init from the beginning\n");

          init_retry_done = true;
          gps->state.init_phase = 0;

          /* Toggle supply voltage */

          board_gps_power(false);
          usleep(5000);
          board_gps_power(true);

          /* Configure initial baud rate to GPS UART */

          ret = ioctl(gps->fd, TCGETS, (unsigned long)&uart);
          if (ret != OK)
            {
              ubgps_set_new_state(gps, GPS_STATE_POWER_OFF);
              return ERROR;
            }

          uart.c_cflag = CS8;
          uart.c_speed = GPS_INITIAL_BAUD_RATE;

          ret = ioctl(gps->fd, TCSETS, (unsigned long)&uart);
          if (ret != OK)
            {
              ubgps_set_new_state(gps, GPS_STATE_POWER_OFF);
              return ERROR;
            }

          usleep(GPS_POWERUP_DELAY*1000);
        }
    }

  while (status == OK && !waiting_for_response)
    {
      switch (gps->state.init_phase)
        {
          case INIT_PHASE_CFG_PRT:
            {
              dbg_sm("INIT_PHASE_CFG_PRT\n");

              /* Re-configure UART port with given settings */

              status = ubgps_send_cfg_prt(gps, GPS_BAUD_RATE);

              if (status == OK)
                {
                  size_t count = 0;

                  /* Wait for UART TX buffer to empty */

                  while (!board_gps_tx_buffer_empty(gps->fd) && count++ < (1 << 16));

                  /* Reset receiver and don't wait for acknowledgment */

                  ubx_reset(gps);

                  /* Sleep that baud rate configuration takes effect */

                  usleep(50000);

                  /* Move to next state */

                  gps->state.init_phase++;
                }

              break;
            }

          case INIT_PHASE_CFG_PRT_BAUD:
            {
              struct termios uart;
              dbg_sm("INIT_PHASE_CFG_PRT_BAUD\n");

              /* Set UART baud rate */

              status = ioctl(gps->fd, TCGETS, (unsigned long)&uart);
              if(status != OK)
                break;

              uart.c_cflag = CS8;
              uart.c_speed = GPS_BAUD_RATE;

              status = ioctl(gps->fd, TCSETS, (unsigned long)&uart);
              if(status != OK)
                break;

              /* Re-configure UART port with given settings */

              status = ubgps_send_cfg_prt(gps, GPS_BAUD_RATE);

              if (status == OK)
                {
                  /* Exit and wait for response */

                  waiting_for_response = true;
                }

              break;
            }

#ifdef CONFIG_SYSTEM_UBGPS_LNA_CONTROL
          case INIT_PHASE_CFG_ANT_LNA:
            {
              dbg_sm("INIT_PHASE_CFG_ANT_LNA\n");

              if (gps->state.target_state_pending &&
                  gps->state.target_state == GPS_STATE_COLD_START)
                {
                  /* Move to next state */

                  gps->state.init_phase++;

                  break;
                }

              /* Send CFG-ANT message. Setup LNA pin and reconfigure. */

              status = ubgps_send_cfg_ant(gps, 0,
                (CONFIG_SYSTEM_UBGPS_LNA_PIN << ANT_PIN_LNA_CTRL_SHIFT) |
                ANT_PIN_RECONFIG);

              if (status == OK)
                {
                  /* Exit and wait for response */

                  waiting_for_response = true;
                }

              break;
            }
#endif /* CONFIG_SYSTEM_UBGPS_LNA_CONTROL */

          case INIT_PHASE_SBAS_MODE:
            {
              dbg_sm("INIT_PHASE_SBAS_MODE\n");

              if (gps->state.target_state_pending &&
                  gps->state.target_state == GPS_STATE_COLD_START)
                {
                  /* Move to next state */

                  gps->state.init_phase++;

                  break;
                }

              /* Send CFG-SBAS message. Disable SBAS subsystem when PSM is enabled */

              status = ubgps_send_cfg_sbas(gps, SBAS_ENABLED);

              if (status == OK)
                {
                  /* Exit and wait for response */

                  waiting_for_response = true;
                }

              break;
            }

          case INIT_PHASE_RECEIVER_MODE:
            {
              dbg_sm("INIT_PHASE_RECEIVER_MODE\n");

              if (gps->state.target_state_pending &&
                  gps->state.target_state == GPS_STATE_COLD_START)
                {
                  /* Move to next state */

                  gps->state.init_phase++;

                  break;
                }

              /* Send CFG-RXM message */

              status = ubgps_send_cfg_rxm(gps, RXM_CONTINOUS);

              if (status == OK)
                {
                  /* Exit and wait for response */

                  waiting_for_response = true;
                }

              break;
            }

          case INIT_PHASE_PM_MODE:
            {
              dbg_sm("INIT_PHASE_PM_MODE\n");

              if (gps->state.target_state_pending &&
                  gps->state.target_state == GPS_STATE_COLD_START)
                {
                  /* Move to next state */

                  gps->state.init_phase++;

                  break;
                }

              /* Send CFG-PM2 message */

              status = ubgps_send_cfg_pm2(gps,
                  PM2_MODE_CYCLIC | PM2_NO_FIX_NO_OFF,
                  gps->state.navigation_rate, 10000);

              if (status == OK)
                {
                  /* Exit and wait for response */

                  waiting_for_response = true;
                }

              break;
            }

          case INIT_PHASE_AID_SET_TIME:
            {
              dbg_sm("INIT_PHASE_AID_SET_TIME\n");
              /* Check if need to setup GPS aiding */

              if (gps->state.target_state_pending &&
                  gps->state.target_state == GPS_STATE_COLD_START)
                {
                  /* Move to next state */

                  gps->state.init_phase++;

                  break;
                }

              /* Send AID-INI message */

              status = ubgps_send_aid_ini(gps);

              if (status == OK)
                {
                  /* Move to next state */

                  gps->state.init_phase++;
                }

              break;
            }

          case INIT_PHASE_AID_ALPSRV_ENABLE:
            {
              dbg_sm("INIT_PHASE_AID_ALPSRV_ENABLE\n");

              /* Check if need to setup GPS aiding */

              if (gps->state.target_state_pending &&
                  gps->state.target_state == GPS_STATE_COLD_START)
                {
                  /* Move to next state */

                  gps->state.init_phase++;

                  break;
                }

              /* Check if AssistNow Offline is enabled */

              if (gps->assist)
                {
                  /* Enable AID-ALPSRV message */

                  status = ubgps_send_cfg_msg(gps,
                                            UBX_CLASS_AID,
                                            UBX_AID_ALPSRV,
                                            1);

                  if (status == OK)
                    {
                      /* Exit and wait for response */

                      waiting_for_response = true;
                    }
                }
              else
                {
                  /* Move to next state */

                  gps->state.init_phase++;
                }

              break;
            }

          case INIT_PHASE_NAV_PVT_RATE:
            {
              dbg_sm("INIT_PHASE_NAV_PVT_RATE\n");

              /* Check if NAV-PVT setup is needed */

              if (gps->state.target_state_pending &&
                  gps->state.target_state == GPS_STATE_COLD_START)
                {
                  /* Move to next state */

                  gps->state.init_phase++;

                  break;
                }

              /* Set NAV-PVT message rate */

              status = ubgps_send_cfg_msg(gps,
                                        UBX_CLASS_NAV,
                                        UBX_NAV_PVT,
                                        1);

              if (status == OK)
                {
                  /* Exit and wait for response */

                  waiting_for_response = true;
                }

              break;
            }

          case INIT_PHASE_NAVIGATION_RATE:
            {
              dbg_sm("INIT_PHASE_NAVIGATION_RATE\n");

              /* Check if navigation rate setup is needed */

              if (gps->state.target_state_pending &&
                  gps->state.target_state == GPS_STATE_COLD_START)
                {
                  /* Move to next state */

                  gps->state.init_phase++;

                  break;
                }

              /* Set GPS navigation rate */

#ifdef CONFIG_UBGPS_PSM_MODE
              if (gps->state.navigation_rate >= CONFIG_UBGPS_PSM_MODE_THRESHOLD*1000)
                {
                  /* Use default navigation rate for SW controlled PSM */

                  status = ubgps_send_cfg_rate(gps, DEFAULT_NAVIGATION_RATE);
                }
              else
                {
                  status = ubgps_send_cfg_rate(gps, gps->state.navigation_rate);
                }
#else
              status = ubgps_send_cfg_rate(gps, gps->state.navigation_rate);
#endif

              if (status == OK)
                {
                  /* Exit and wait for response */

                  waiting_for_response = true;
                }

              break;
            }

          case INIT_PHASE_DONE:
            {
              dbg_sm("INIT_PHASE_DONE\n");

              if (gps->state.target_state == GPS_STATE_INITIALIZATION)
                {
                  /* Report target state reached */

                  ubgps_report_target_state(gps, false);

                  /* Move to search for GPS fix */

                  ubgps_set_new_state(gps, GPS_STATE_SEARCHING_FIX);
                }
              else if (gps->state.target_state == GPS_STATE_FIX_ACQUIRED)
                {
                  /* Move to search for GPS fix */

                  ubgps_set_new_state(gps, GPS_STATE_SEARCHING_FIX);
                }
              else
                {
                  /* Move to requested target state */

                  ubgps_set_new_state(gps, gps->state.target_state);
                }

              /* Reset init retry flag */

              init_retry_done = false;

              return OK;
            }
        }
    }

  if (status != OK)
    {
      dbg_sm("Status fail for init phase:%u\n", gps->state.init_phase);

      /* Start initialization retry timer */

      gps->state.init_timer_id = __ubgps_set_timer(gps,
                                                  GPS_RETRY_DELAY,
                                                  ubgps_timeout,
                                                  gps);

      if (gps->state.init_timer_id < 0)
        {
          /* Fail back to power down state */

          ubgps_set_new_state(gps, GPS_STATE_POWER_OFF);

          return ERROR;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ubgps_sm_initialization
 *
 * Description:
 *   State machine for GPS initialization
 *
 ****************************************************************************/
static int ubgps_sm_initialization(struct ubgps_s * const gps,
                                   struct sm_event_s const * const event)
{
  DEBUGASSERT(gps && event);

  /* Check event */

  switch (event->id)
    {
      case SM_EVENT_ENTRY:
        {
          dbg_sm("SM_EVENT_ENTRY\n");

          /* Start initialization timer */

          gps->state.init_timer_id = __ubgps_set_timer(gps,
                                                       GPS_POWERUP_DELAY,
                                                       ubgps_timeout,
                                                       gps);

          if (gps->state.init_timer_id < 0)
            {
              dbg("Error while setting up initialization timer.\n");

              /* Fail back to power down state */

              ubgps_set_new_state(gps, GPS_STATE_POWER_OFF);

              return ERROR;
            }

          /* Initialize GPS location data */

          memset(&gps->location, 0, sizeof(gps->location));

          /* Reset initialization phase & retry counter */

          gps->state.init_phase = 0;
          gps->state.init_count = 0;

          return OK;
        }

      case SM_EVENT_TIMEOUT:
        {
          struct sm_event_timeout_s const * const timeout =
              (struct sm_event_timeout_s *)event;

          dbg_sm("SM_EVENT_TIMEOUT\n");

          if (timeout->timer_id == gps->state.target_timer_id)
            {
              /* Report target state timeout */

              ubgps_report_target_state(gps, true);
            }
          else if (timeout->timer_id == gps->state.init_timer_id)
            {
              /* Clear initialization timer ID */

              gps->state.init_timer_id = -1;

              /* Continue initialization process */

              return ubgps_init_process_phase(gps, false);
            }

          return OK;
        }

      case SM_EVENT_UBX_STATUS:
        {
          struct sm_event_ubx_status_s const * const ubx =
              (struct sm_event_ubx_status_s *)event;

          dbg_sm("SM_EVENT_UBX_STATUS - class:%02X, msg:%02X\n",
              ubx->class_id, ubx->msg_id);

          if (ubx->class_id == UBX_CLASS_CFG &&
             (ubx->msg_id == UBX_CFG_PRT || ubx->msg_id == UBX_CFG_MSG ||
              ubx->msg_id == UBX_CFG_RATE || ubx->msg_id == UBX_CFG_SBAS ||
              ubx->msg_id == UBX_CFG_PM2 || ubx->msg_id == UBX_CFG_RXM ||
              ubx->msg_id == UBX_CFG_ANT))
            {
              if (ubx->status != UBX_STATUS_ACK)
                {
                  /* Retry initialization phase */

                  dbg_sm("SM_EVENT_UBX_STATUS - NACK\n");

                  return ubgps_init_process_phase(gps, false);
                }

              /* Move to next phase */

              return ubgps_init_process_phase(gps, true);
            }

          dbg_sm("Unhandled SM_EVENT_UBX_STATUS. Class 0x%02x, message 0x%02x\n",
              ubx->class_id, ubx->msg_id);

          return OK;
        }

      case SM_EVENT_UBX_MESSAGE:
        {
          struct sm_event_ubx_msg_s const * const ubx_msg =
              (struct sm_event_ubx_msg_s *)event;
          struct ubx_msg_s const * const msg = ubx_msg->msg;
          dbg_sm("SM_EVENT_UBX_MESSAGE\n");

          if (msg->class_id == UBX_CLASS_NAV && msg->msg_id == UBX_NAV_PVT)
            {
              dbg_sm("Unexpected NAV message in initialization phase???\n");

              return ubgps_handle_nav_pvt(gps, msg);
            }
          else if (msg->class_id == UBX_CLASS_AID && msg->msg_id == UBX_AID_ALPSRV)
            {
              return ubgps_handle_aid_alpsrv(gps, msg);
            }

          return OK;
        }

      case SM_EVENT_TARGET_STATE:
        {
          struct sm_event_target_state_s const * const target =
              (struct sm_event_target_state_s *)event;

          dbg_sm("SM_EVENT_TARGET_STATE\n");

          if (target->target_state != GPS_STATE_INITIALIZATION)
            {
              /* Setup target state transition timeout */

              ubgps_setup_timeout(gps, target->timeout);

              /* Set GPS target state */

              gps->state.target_state = target->target_state;

              if (target->target_state == GPS_STATE_POWER_OFF)
                {
                  /* Set next GPS state */

                  ubgps_set_new_state(gps, GPS_STATE_POWER_OFF);
                }
            }
          else if (gps->state.init_phase == INIT_PHASE_DONE)
            {
              /* Make current state the target state */

              gps->state.target_state = gps->state.current_state;

              /* Report target reached */

              ubgps_report_target_state(gps, false);
            }

          return OK;
        }

      case SM_EVENT_EXIT:
        {
          dbg_sm("SM_EVENT_EXIT\n");

          /* Stop pending initialization timer */

          if (gps->state.init_timer_id >= 0)
            {
              __ubgps_remove_timer(gps, gps->state.init_timer_id);

              gps->state.init_timer_id = -1;
            }

          return OK;
        }

      default:
        return OK;
    }
}

/****************************************************************************
 * Name: ubgps_sm_cold_start
 *
 * Description:
 *   State machine for GPS cold start
 *
 ****************************************************************************/
static int ubgps_sm_cold_start(struct ubgps_s * const gps, struct sm_event_s const * const event)
{
  DEBUGASSERT(gps && event);

  /* Check event */

  switch (event->id)
    {
      case SM_EVENT_ENTRY:
        {
          int status;
          dbg_sm("SM_EVENT_ENTRY\n");

          dbg("Performing GPS receiver cold start.\n");

          /* Reset receiver */

          status = ubgps_send_cfg_rst(gps);

          if (status != OK)
            {
              /* Fail back to power down state */

              ubgps_set_new_state(gps, GPS_STATE_POWER_OFF);
            }

          return OK;
        }

      case SM_EVENT_UBX_STATUS:
        {
          struct sm_event_ubx_status_s const * const ubx =
              (struct sm_event_ubx_status_s *)event;

          dbg_sm("SM_EVENT_UBX_STATUS\n");

          if (ubx->class_id == UBX_CLASS_CFG && ubx->msg_id == UBX_CFG_RST)
            {
              if (ubx->status != UBX_STATUS_ACK)
                {
                  /* Fail back to power down state */

                  ubgps_set_new_state(gps, GPS_STATE_POWER_OFF);
                }
              else
                {
                  /* Report target reached */

                  ubgps_report_target_state(gps, false);

                  /* Set target state only if not already set by application */

                  if (!gps->state.target_state_pending)
                    {
                      /* Set new target state */

                      gps->state.target_state = GPS_STATE_FIX_ACQUIRED;

                      /* Move to GPS initialization in order to use GPS aiding
                       * parameters if set */

                      ubgps_set_new_state(gps, GPS_STATE_INITIALIZATION);
                    }
                }

              return OK;
            }

          dbg_sm("Unhandled SM_EVENT_UBX_STATUS. Class 0x%02x, message 0x%02x\n",
              ubx->class_id, ubx->msg_id);

          return OK;
        }

      default:
        break;
    }

  /* Pass to global state handler */

  return ubgps_sm_global(gps, event);
}
