/****************************************************************************
 * apps/thingsee/charger/bq24251_module.h
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
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

#ifndef BQ24251_MODULE_H_
#define BQ24251_MODULE_H_

enum bq24251_chgr_state_e
  {
    BQ24251_CHRG_NORMAL = 0,
    BQ24251_CHRG_WARN,
    BQ24251_CHRG_FAULT,
    BQ24251_CHRG_NOT_IN_PLACE,
    BQ24251_CHRG_FULL,
    BQ24251_CHRG_SLEEP
  };

enum bq24251_chgr_event_e
  {
    BQ24251_CHRG_EVENT_CHARGING = 0,
    BQ24251_CHRG_EVENT_DONE,
    BQ24251_CHRG_EVENT_FAULT,
    BQ24251_CHRG_EVENT_WARN,
    BQ24251_CHRG_EVENT_NO_CHARGER,
  };

typedef struct bq24251_chgr_cbks_s
  {
    void (*charger_event) (enum bq24251_chgr_event_e event);
    void (*notify_usb_connect) (const char *porttype);
  } bq24251_chgr_cbks_t;

#ifdef CONFIG_THINGSEE_CHARGER_MODULE

int bq24251_init_module(const bq24251_chgr_cbks_t * bq24251_cbks);
void bq24251_set_charging_allowed(bool allowed);
int bq24251_turnoff_dev(void);
bool bq24251_is_charging(void);

#else

static inline int bq24251_init_module(bq24251_chgr_cbks_t * bq24251_cbks)
{
  return 0;
}

static inline int bq24251_turnoff_dev(void)
{
  return 0;
}

static inline bool bq24251_is_charging(void)
{
  return false;
}

#endif

#endif                                 /* BQ24251_MODULE_H_ */
