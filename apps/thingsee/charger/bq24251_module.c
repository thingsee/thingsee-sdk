/****************************************************************************
 * apps/thingsee/charger/bq24251_module.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Authors: Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
 *            Juha Niskanen <juha.niskanen@haltian.com>
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <debug.h>
#include <poll.h>
#include <string.h>
#include <apps/thingsee/ts_core.h>

#include <nuttx/power/bq24251.h>
#include <arch/board/board.h>
#include <arch/board/board-battery.h>

#include "bq24251_module.h"

#define CHARGER_DEV_PATH			"/dev/charger0"
#define CHARGER_DEV_DELAY_SEC		2

#undef bq24251_dbg
#ifdef CONFIG_THINGSEE_CHARGER_MODULE_DBG
#  define bq24251_dbg(x, ...)   dbg(x, ##__VA_ARGS__)
#  define bq24251_dbg_perror(x) dbg("%s: %d\n", x, errno)
#else
#  define bq24251_dbg(x, ...)
#  define bq24251_dbg_perror(x)
#endif

enum bq24251_plugged_state_e
{
  PLUGGED_STATE_UNKNOWN = 0,
  PLUGGED_STATE_TRUE,
  PLUGGED_STATE_FALSE,
};

typedef struct bq24251_charger_dev_t
  {
    int file;
    int timer;
    enum bq24251_plugged_state_e plugged;
    unsigned char det_attempts;
    bool is_dead_fault;
    bq24251_state_t state;
    struct
    {
      bool charging_allowed;
    } config;
  } bq24251_charger_dev_t;


static void bq24251_charger_event(enum bq24251_chgr_event_e event);
static void bq24251_notify_usb_connect(const char *porttype);
static int bq24251_setup_detect_timer(void);
static int bq24251_stop_detect_timer(void);
static int bq24251_check_status_bits(void);

/* Global variables for this unit */

static bq24251_charger_dev_t g_charger = {
  .file = -1,
  .timer = -1,
  .plugged = PLUGGED_STATE_UNKNOWN,
  .det_attempts = 0,
  .is_dead_fault = false,
  .state = BQ24251_READY_ST,
  .config.charging_allowed = true,
};

static bq24251_chgr_cbks_t g_progress_cbks = {
  .charger_event = NULL,
  .notify_usb_connect = NULL,
};

/****************************************************************************
 * Name: bq24251_charger_event
 ****************************************************************************/

static void bq24251_charger_event(enum bq24251_chgr_event_e event)
{
  switch (event)
    {
    case BQ24251_CHRG_EVENT_CHARGING:
      bq24251_dbg("Charging in progress\n");
      break;
    case BQ24251_CHRG_EVENT_DONE:
      bq24251_dbg("Charging done\n");
      break;
    case BQ24251_CHRG_EVENT_FAULT:
      bq24251_dbg("Charging fault\n");
      break;
    case BQ24251_CHRG_EVENT_WARN:
      bq24251_dbg("Charger warning?\n");
      break;
    case BQ24251_CHRG_EVENT_NO_CHARGER:
      bq24251_dbg("Charger not presented\n");
      break;
    default:
      bq24251_dbg("Unknown charger event\n");
      break;
    }

  if (g_progress_cbks.charger_event)
    g_progress_cbks.charger_event(event);
}

/****************************************************************************
 * Name: bq24251_notify_usb_connect
 ****************************************************************************/

static void bq24251_notify_usb_connect(const char *porttype)
{
  if (!porttype)
    {
      dbg("USB disconnected\n");
    }
  else
    {
      dbg("USB connected, port type: \"%s\"\n", porttype);
    }

  /* Inform baselayer SW about event. */

  board_pwr_charger_connected(porttype);

  /* Inform application SW about event. */

  if (g_progress_cbks.notify_usb_connect)
    g_progress_cbks.notify_usb_connect(porttype);
}

/****************************************************************************
 * Name: bq24251_timer_callback
 *
 * Description:
 *   Charger timer call-back
 *
 * Input Parameters:
 *   timer_id    - timer's id
 *   priv 		 - private arguments could be passed if needed
 *
 * Returned Value:
 * 	0 on success and -1 on ERROR
 *
 ****************************************************************************/
static int bq24251_timer_callback(const int timer_id,
                                  const struct timespec *date, void *const priv)
{
  if (g_charger.timer < 0)
    return OK;

  g_charger.timer = -1;
  bq24251_setup_detect_timer();

  bq24251_dbg("...\n");

  (void)bq24251_check_status_bits();

  return OK;
}

/****************************************************************************
 * Name: bq24251_charger_stop_charging
 *
 * Description:
 *   Charger stops charger IC from doing bad things
 *
 * Input Parameters:
 *   is_dead_fault    - if fault is dangerous for TS device, turn device off.
 *
 * Returned Value:
 * 	0 on success and -1 on ERROR
 *
 ****************************************************************************/
static int bq24251_charger_stop_charging(bool is_dead_fault, bool force_ce)
{
  int ret = OK;
  bq24251_data_t data = {
    .enable_ce = false,
    .enable_hz = false,
  };

  if (g_charger.file < 0)
    return ERROR;

  if (is_dead_fault || force_ce)
    {
      data.enable_ce = !is_dead_fault;
      ret = ioctl(g_charger.file, BQ24251_IOC_SET_CHARGE_ENABLE,
                  (unsigned int)&data);
      if (ret < 0)
        {
          bq24251_dbg_perror("Failed to disable charger");
          goto fail;
        }

      g_charger.is_dead_fault = is_dead_fault;
    }

  ret = ioctl(g_charger.file, BQ24251_IOC_SET_HZ, (unsigned int)&data);
  if (ret < 0)
    {
      bq24251_dbg_perror("Failed to set HZ");
      goto fail;
    }

  ret = ioctl(g_charger.file, BQ24251_IOC_DISABLE_TIMERS, 0);
  if (ret < 0)
    {
      bq24251_dbg_perror("Failed to disable timers");
      goto fail;
    }

  if (is_dead_fault)
    {
      ret = bq24251_turnoff_dev();
      if (ret < 0)
        goto fail;
    }

fail:
  return ret;
}

/****************************************************************************
 * Name: bq24251_charger_read_charge_enable
 *
 * Description:
 *   Read charger enabled flag from charger chip
 *
 * Input Parameters:
 *   ce    - output
 *
 * Returned Value:
 *      0 on success and -1 on ERROR
 *
 ****************************************************************************/
static int bq24251_charger_check_charge_enable(bool *ce)
{
  int ret;

  if (g_charger.file < 0 || !ce)
    return ERROR;

  ret = ioctl(g_charger.file, BQ24251_IOC_GET_CHARGE_ENABLE, (unsigned int)ce);
  if (ret < 0)
    {
      bq24251_dbg_perror("Failed to read CE");
    }

  return ret;
}
/****************************************************************************
 * Name: bq24251_charger_type_detection
 *
 * Description:
 *   Detects type of the charger
 *
 * Input Parameters:
 *
 * Returned Value:
 * 	0 on success and -1 on ERROR
 *
 ****************************************************************************/
static int bq24251_charger_type_detection(enum bq24251_state_t status)
{
  int ret = OK;
  bq24251_data_t data = {
    .chrg_type = -1
  };
  const char *porttype = NULL;

  if (g_charger.det_attempts == 0)
    {
      ret = ioctl(g_charger.file, BQ24251_IOC_START_USBDET, 0);
      if (ret < 0)
        {
          ret = -errno;
          bq24251_dbg_perror("Cannot start USB detection");
          goto fail;
        }
    }

  ret = ioctl(g_charger.file, BQ24251_IOC_CHECK_USBDET, (unsigned int)&data);
  if (ret < 0)
    {
      ret = -errno;
      bq24251_dbg_perror("Cannot detect charger");
      goto fail;
    }

  bq24251_stop_detect_timer();

  if (data.chrg_type == BQ24251_CHRG_DCP)
    {
      /*
       * DCP is reported after USB detection is completed and there is no
       * charger actually connected. Use status to determinate if we are
       * connected or not.
       */
      if (status == BQ24251_READY_ST)
        {
          /* Expecting BQ24251_PROGR_ST or BQ24251_DONE_ST when connected
           * to DCP. Discard detection if we get READY_ST. */

          bq24251_dbg("%s\n",
                      "USB detection gave DCP but with charger ready status."
                      " Ignoring result of USB detection.");
          ret = -ENOLINK;
          goto fail;
        }
    }

  /* Parse port type and notify upper SW layers. */

  switch (data.chrg_type)
    {
    case BQ24251_CHRG_DCP:
      porttype = "DCP";
      break;
    case BQ24251_CHRG_CDP:
      porttype = "CDP";
      break;
    case BQ24251_CHRG_SDP:
      porttype = "SDP";
      break;
    case BQ24251_CHRG_TT:
      porttype = "nonstd";
      break;
    default:
      DEBUGASSERT(false);
      break;
    }

  bq24251_notify_usb_connect(porttype);

  /* Set charging limits and currents. */

#ifdef HAVE_BOARD_GET_BATTERY_SAFE_LIMITS
  data.ilim = board_get_battery_safe_ilim(data.chrg_type);
  data.chrg_current = board_get_battery_safe_charger_current(data.chrg_type);

  /* Increase charge termination threshold for easier
     detection of charging ending: */
  data.term_current = BQ24251_ITERM_100MA;
#else /* Thingsee */
  switch (data.chrg_type)
    {
    case BQ24251_CHRG_DCP:
      data.ilim = BQ24251_ILIM_CHRG_1500MA;
      data.chrg_current = BQ24251_CHRG_CURRENT_2000;
      break;
    case BQ24251_CHRG_CDP:
      data.ilim = BQ24251_ILIM_CHRG_1500MA;
      data.chrg_current = BQ24251_CHRG_CURRENT_1500;
      break;
    case BQ24251_CHRG_SDP:
      data.ilim = BQ24251_ILIM_USB20_500MA;
      data.chrg_current = BQ24251_CHRG_CURRENT_500;
      break;
    case BQ24251_CHRG_TT:
      data.ilim = BQ24251_ILIM_USB20_500MA;
      data.chrg_current = BQ24251_CHRG_CURRENT_500;
      break;
    }
  data.term_current = BQ24251_ITERM_DEFAULT;
#endif

  ret = ioctl(g_charger.file, BQ24251_IOC_SET_ILIM, (unsigned int)&data);
  if (ret < 0)
    {
      ret = -errno;
      bq24251_dbg_perror("Cannot set ilim for charger");
      goto fail;
    }

  ret = ioctl(g_charger.file, BQ24251_IOC_SET_CHRG_CURRENT, (unsigned int)&data);
  if (ret < 0)
    {
      ret = -errno;
      bq24251_dbg_perror("Cannot set charger current");
      goto fail;
    }

  if (!g_charger.config.charging_allowed)
    {
      /* Enabled charging, but config has charging disabled. */

      data.enable_ce = false;
      ret = ioctl(g_charger.file, BQ24251_IOC_SET_CHARGE_ENABLE,
                  (unsigned int)&data);
      if (ret < 0)
        {
          ret = -errno;
          bq24251_dbg_perror("Failed to disable charger");
          goto fail;
        }
    }

fail:
  return ret;
}

/****************************************************************************
 * Name: bq24251_charger_work_handler
 *
 * Description:
 *   Checks charger work progress
 *
 * Input Parameters:
 * 	work_status		- pointer to data structure
 *
 * Returned Value:
 * 	0 on success and -1 on ERROR
 *
 ****************************************************************************/
static int bq24251_charger_work_handler(bq24251_data_t * work_status)
{
  int ret = OK;

  if (work_status->sts->state != BQ24251_DONE_ST)
    {
      if (g_charger.plugged != PLUGGED_STATE_UNKNOWN)
        {
          bq24251_charger_event(BQ24251_CHRG_EVENT_CHARGING);
        }
    }
  else
    {
      bq24251_charger_event(BQ24251_CHRG_EVENT_DONE);
      bq24251_charger_stop_charging(false, false);
    }

  bq24251_dbg("g_charger.det_attempts = %d\n",
                (int)g_charger.det_attempts);

  if (g_charger.det_attempts > 3)
    {
      g_charger.plugged = PLUGGED_STATE_FALSE;
      bq24251_stop_detect_timer();
      return ERROR;
    }

  if (g_charger.plugged != PLUGGED_STATE_TRUE) /* FALSE or UNKNOWN */
    {
      ret = ioctl(g_charger.file, BQ24251_IOC_INIT, 0);
      if (ret < 0)
        {
          bq24251_dbg_perror("Cannot reinit charger");
          goto fail;
        }

      g_charger.is_dead_fault = false;

      ret = bq24251_charger_type_detection(work_status->sts->state);

      /* Returns -EAGAIN in case detection is still in progress. */
      if (ret < 0)
        {
          if (ret == -EAGAIN)
            {
              g_charger.det_attempts++;
              bq24251_setup_detect_timer();
            }
          else
            {
              goto fail;
            }
        }
      else
        g_charger.det_attempts = 0;
    }

  if (ret == OK)
    {
      if (g_charger.plugged == PLUGGED_STATE_UNKNOWN)
        {
          if (work_status->sts->state != BQ24251_DONE_ST)
            {
              bq24251_charger_event(BQ24251_CHRG_EVENT_CHARGING);
            }
        }

      g_charger.plugged = PLUGGED_STATE_TRUE;
    }

fail:
  return ret;
}

/****************************************************************************
 * Name: bq24251_charger_fault_handler
 *
 * Description:
 *   Checks charger faults
 *
 * Input Parameters:
 * 	status_fault		- pointer to data structure
 *
 * Returned Value:
 * 	0 on success and -1 on ERROR
 *
 ****************************************************************************/
static int bq24251_charger_fault_handler(bq24251_data_t * status_fault)
{
  int ret = OK;

  switch (status_fault->sts->fault)
    {
#define X(x) case x: bq24251_dbg("fault: %s\n", #x); break;
      X(BQ24251_INPUT_OVP);
      X(BQ24251_THERMAL_SHUTDOWN);
      X(BQ24251_BATTERY_OVP);
      X(BQ24251_INPUT_UVLO);
      X(BQ24251_LDO_SHORT);
      X(BQ24251_SLEEP);
      X(BQ24251_BATTERY_TS_FAULT);
      X(BQ24251_TIMER_FAULT);
      X(BQ24251_NO_BATTERY_CONNECTED);
      X(BQ24251_ISET_SHORT);
    default:
      break;
#undef X
    }

  /* TODO: go through the entire list of faults.
   * For now only the first fault will be checked.
   */
  switch (status_fault->sts->fault)
    {
    case BQ24251_INPUT_OVP:
    case BQ24251_THERMAL_SHUTDOWN:
    case BQ24251_BATTERY_OVP:
      bq24251_dbg("Dead fault occurred\n");
      bq24251_charger_stop_charging(true, false);
      bq24251_charger_event(BQ24251_CHRG_EVENT_FAULT);
      break;
    case BQ24251_INPUT_UVLO:
    case BQ24251_LDO_SHORT:
    case BQ24251_SLEEP:
      if (g_charger.plugged != PLUGGED_STATE_FALSE) /* TRUE or UNKNOWN */
        {
          bq24251_dbg("Charger disconnected\n");
          bq24251_charger_stop_charging(false, true);
          bq24251_charger_event(BQ24251_CHRG_EVENT_NO_CHARGER);
          bq24251_notify_usb_connect(NULL);
          g_charger.plugged = PLUGGED_STATE_FALSE;
          g_charger.det_attempts = 0;
        }
      else if (g_charger.det_attempts > 0)
        {
          /* poll might be running if we are in middle of USB detection cycle. */
          bq24251_stop_detect_timer();
          bq24251_charger_event(BQ24251_CHRG_EVENT_FAULT);
        }
      break;
    case BQ24251_BATTERY_TS_FAULT:
    case BQ24251_TIMER_FAULT:
    case BQ24251_NO_BATTERY_CONNECTED:
    case BQ24251_ISET_SHORT:
      bq24251_dbg("Device is still alive\n");
      bq24251_charger_stop_charging(false, false);
      bq24251_charger_event(BQ24251_CHRG_EVENT_FAULT);
      g_charger.plugged = PLUGGED_STATE_FALSE;
      g_charger.det_attempts = 0;
      break;
    default:
      bq24251_dbg("Unknown fault detected\n");
      break;
    }

  return ret;
}

/****************************************************************************
 * Name: bq24251_check_status_bits
 *
 * Description:
 *   Checks if charger can be run normally
 *
 * Input Parameters:
 *
 * Returned Value:
 * 	0 on success and -1 on ERROR
 *
 ****************************************************************************/
static int bq24251_check_status_bits(void)
{
  int ret = OK;
  int fault_count = 0;
  int prev_fault = -1;
  bq24251_chrg_status_t sts = {
    .fault = false,
    .state = BQ24251_READY_ST
  };
  bq24251_data_t data = {
    .sts = &sts
  };

  if (g_charger.file < 0)
    {
      /* Module initialized yet. */

      return ERROR;
    }

  while (fault_count++ < BQ24251_MAX_CHG_FAULT)
    {
      ret = ioctl(g_charger.file, BQ24251_IOC_CHK_CHRG_STS, (unsigned int)&data);
      if (ret < 0)
        {
          ret = -errno;
          bq24251_dbg_perror("Cannot check charger status");

          /* If we cannot check status, and error is not transient we might as well
           * attempt to stop charging for safety. */
          if (!(ret == -EAGAIN || ret == -EINVAL))
            bq24251_charger_stop_charging(false, false);
          return ERROR;
        }

      if (data.sts->fault != BQ24251_NORMAL && prev_fault == data.sts->fault)
        {
          break;
        }

      bq24251_dbg("Check status bit state: %d Fault member: %d Fault count: %d Is plugged: %s\n",
                    data.sts->state, data.sts->fault, fault_count,
                    (g_charger.plugged == PLUGGED_STATE_FALSE ? "false" :
                     (g_charger.plugged == PLUGGED_STATE_TRUE ? "true" : "unknown")));

      g_charger.state = data.sts->state;

      if (data.sts->fault != BQ24251_NORMAL)
        {
          prev_fault = data.sts->fault;

          bq24251_charger_fault_handler(&data);
        }
      else
        {
          ret = bq24251_charger_work_handler(&data);
          if (ret == -ENOLINK)
            {
              if (g_charger.plugged == PLUGGED_STATE_UNKNOWN)
                {
                  /* Not connected, missed SLEEP fault because reboot.
                   * Generate SLEEP. */

                  data.sts->fault = BQ24251_SLEEP;

                  bq24251_charger_fault_handler(&data);
                }
            }
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: bq24251_setup_detect_timer
 *
 * Description:
 *   Initializes timer
 *
 * Returned Value:
 * 	0 on success and -1 on ERROR
 *
 ****************************************************************************/
static int bq24251_setup_detect_timer(void)
{
  struct timespec waketime;

  if (g_charger.timer >= 0)
    {
      /* Timer is running already. Most likely previous USB detection attempt.
       * We consider this not a bug. */
      dbg
        ("Attempted to start charger status poll, but timer is already active.\n");
      return OK;
    }

  clock_gettime(CLOCK_MONOTONIC, &waketime);
  waketime.tv_sec += CHARGER_DEV_DELAY_SEC;

  g_charger.timer = ts_core_timer_setup_date(&waketime,
                                             bq24251_timer_callback, NULL);
  if (g_charger.timer < 0)
    {
      bq24251_dbg_perror("Cannot get timer for charger");
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: bq24251_stop_detect_timer
 *
 * Description:
 *   Removes timer
 *
 * Input Parameters:
 *
 * Returned Value:
 * 	0 on success and -1 on ERROR
 *
 ****************************************************************************/
static int bq24251_stop_detect_timer(void)
{
  int ret = OK;

  if (g_charger.timer >= 0)
    {
      ret = ts_core_timer_stop(g_charger.timer);
      if (ret < 0)
        {
          bq24251_dbg_perror("Charger status poll timer removed");
          return ERROR;
        }
      g_charger.timer = -1;
      g_charger.det_attempts = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: bq24251_charger_callback
 *
 * Description:
 *   Will be called when charger is connected or disconnected
 *
 * Input Parameters:
 * 	pfd 		- pointer to poll structure
 * 	priv		- private data
 *
 * Returned Value:
 * 	0 on success and -1 on ERROR
 *
 ****************************************************************************/
static int bq24251_charger_callback(const struct pollfd *const pfd,
                                    void *const priv)
{
  bq24251_dbg("...\n");

  (void)bq24251_check_status_bits();

  return OK;
}

/****************************************************************************
 * Name: bq24251_init_module
 *
 * Description:
 *   Initializes charger module
 *
 * Input Parameters:
 * 	bq24251_cbks 		- callbacks to be used
 * 	when something happens with charger
 *
 * Returned Value:
 * 	0 on success and -1 on ERROR
 *
 ****************************************************************************/
int bq24251_init_module(const bq24251_chgr_cbks_t * bq24251_cbks)
{
  int ret = OK;

  if (bq24251_cbks)
    {
      if (bq24251_cbks->charger_event)
        g_progress_cbks.charger_event = bq24251_cbks->charger_event;
      if (bq24251_cbks->notify_usb_connect)
        g_progress_cbks.notify_usb_connect = bq24251_cbks->notify_usb_connect;
    }

  g_charger.timer = -1;
  g_charger.is_dead_fault = false;

  g_charger.file = open(CHARGER_DEV_PATH, O_RDWR);
  if (g_charger.file < 0)
    {
      bq24251_dbg_perror("Cannot open charger to read/write");
      return ERROR;
    }

  /* Register charger poll callback. */

  ret = ts_core_fd_register(g_charger.file, POLLIN, bq24251_charger_callback,
                            NULL);
  DEBUGASSERT(ret == OK);

  /* Check if charger plugged in already */
  bq24251_check_status_bits();

  return ret;
}

/****************************************************************************
 * Name: bq24251_turnoff_dev
 *
 * Description:
 *   Powers OFF system. 1 uA is used
 *
 * Input Parameters:
 *
 * Returned Value:
 * 	0 on success and -1 on ERROR
 *
 ****************************************************************************/
int bq24251_turnoff_dev(void)
{
  int ret = OK;

  if (g_charger.file < 0)
    {
      set_errno(ENODEV);
      return ERROR;
    }

  ret = ioctl(g_charger.file, BQ24251_IOC_SYSOFF, 0);
  if (ret < 0)
    bq24251_dbg_perror("Cannot set SYSOFF bit");

  return ret;
}

/****************************************************************************
 * Name: bq24251_set_charging_allowed
 *
 * Description:
 *   Setup for if charging allowed or disallow
 *
 * Input Parameters:
 *   allowed:  'true' if allowed, 'false' if disallowed
 *
 ****************************************************************************/
void bq24251_set_charging_allowed(bool allowed)
{
  bool curr_ce = false;

  if (g_charger.config.charging_allowed == allowed)
    return;

  g_charger.config.charging_allowed = allowed;

  if (g_charger.file < 0)
    return;

  /* Check state. */

  if (!g_charger.is_dead_fault &&
      bq24251_charger_check_charge_enable(&curr_ce) >= 0 &&
      curr_ce != allowed)
    {
      const bq24251_data_t data = { .enable_ce = allowed };

      /* Disable or restore charging. */

      (void)ioctl(g_charger.file, BQ24251_IOC_SET_CHARGE_ENABLE,
                  (unsigned int)&data);
    }

  /* Setup and check state. */

  (void)bq24251_check_status_bits();
}

/****************************************************************************
 * Name: bq24251_is_charging
 *
 * Description:
 *   Check charging status.
 *
 * Returned Value:
 *   'true' when charging.
 *
 ****************************************************************************/
bool bq24251_is_charging(void)
{
  /* Well, it would be nice to check status bits before
   * saying that charging in progress. Also HW hack was
   * removed starting from build 1.5 and STAT-pin is not
   *  relabel in current system at all */

  bq24251_check_status_bits();

  return (g_charger.state == BQ24251_PROGR_ST);
}
