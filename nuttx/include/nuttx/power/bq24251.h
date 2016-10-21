/****************************************************************************
 * include/nuttx/power/bq24251.c
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
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

#ifndef BQ24253_H_
#define BQ24253_H_

#include <nuttx/i2c.h>

typedef enum bq24251_fault_t
  {
    BQ24251_NORMAL = 0,
    BQ24251_INPUT_OVP,
    BQ24251_INPUT_UVLO,
    BQ24251_SLEEP,
    BQ24251_BATTERY_TS_FAULT,
    BQ24251_BATTERY_OVP,
    BQ24251_THERMAL_SHUTDOWN,
    BQ24251_TIMER_FAULT,
    BQ24251_NO_BATTERY_CONNECTED,
    BQ24251_ISET_SHORT,
    BQ24251_LDO_SHORT,
    BQ24251_MAX_CHG_FAULT
  } bq24251_fault_t;

typedef enum bq24251_current_t
  {
    BQ24251_ILIM_USB20_100MA = 0,
    BQ24251_ILIM_USB30_150MA,
    BQ24251_ILIM_USB20_500MA,
    BQ24251_ILIM_USB30_900MA,
    BQ24251_ILIM_CHRG_1500MA,
    BQ24251_ILIM_CHRG_2000MA,
    BQ24251_ILIM_EXT_ILIM,
    BQ24251_ILIM_NO_LIMIT
  } bq24251_current_t;

typedef enum bq24251_current_term_limit_t
  {
    BQ24251_ITERM_DEFAULT = 0,
    BQ24251_ITERM_25MA = (1 << 0),
    BQ24251_ITERM_50MA = (1 << 1),
    BQ24251_ITERM_100MA = (1 << 2)
  } bq24251_current_term_limit_t;

typedef enum bq24251_ioc_t
  {
    BQ24251_IOC_INIT,
    BQ24251_IOC_CHK_CHRG_STS,
    BQ24251_IOC_START_USBDET,
    BQ24251_IOC_CHECK_USBDET,
    BQ24251_IOC_SET_ILIM,
    BQ24251_IOC_SET_CHRG_CURRENT,
    BQ24251_IOC_SET_HZ,
    BQ24251_IOC_SET_TERM,
    BQ24251_IOC_SYSOFF,
    BQ24251_IOC_DISABLE_TIMERS,
    BQ24251_IOC_FORCE_BAT_DETECTION,
    BQ24251_IOC_SET_CHARGE_ENABLE
  } bq24251_ioc_t;

typedef enum bq24251_chrg_type_t
  {
    BQ24251_CHRG_DCP = 0,
    BQ24251_CHRG_CDP,
    BQ24251_CHRG_SDP,
    BQ24251_CHRG_TT
  } bq24251_chrg_type_t;

typedef enum bq24251_state_t
  {
    BQ24251_READY_ST = 0,
    BQ24251_PROGR_ST,
    BQ24251_DONE_ST,
    BQ24251_FAULT_ST
  } bq24251_state_t;

/* TO DO: ADD more charger current values */

typedef enum bq24251_charger_current_t
  {
    BQ24251_CHRG_CURRENT_500 = 0x0,
    BQ24251_CHRG_CURRENT_1500 = 0xA0,
    BQ24251_CHRG_CURRENT_2000 = 0xF0,
    BQ24251_CHRG_CURRENT_EXTERNAL = 0xF8
  } bq24251_charger_current_t;

typedef struct bq24251_chrg_status_t
  {
    uint8_t fault;
    bq24251_fault_t *faults;
    bq24251_state_t state;
  } bq24251_chrg_status_t;

typedef struct bq24251_data_t
  {
    bq24251_chrg_status_t *sts;
    bq24251_current_t ilim;
    bq24251_chrg_type_t chrg_type;
    bq24251_charger_current_t chrg_current;
    bq24251_current_term_limit_t term_current;
    bool enable_hz:1;
    bool enable_ce:1;
    bool enable_term:1;
  } bq24251_data_t;

typedef struct bq24251_config_t
  {
    int irq;
    int (*irq_attach) (FAR struct bq24251_config_t * state, xcpt_t isr_stat);
    void (*irq_enable) (FAR struct bq24251_config_t * state, bool enable);
    void (*irq_clear) (FAR struct bq24251_config_t * state);
  } bq24251_config_t;

int bq24251_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                     uint8_t addr, bq24251_config_t * config);

#endif                                 /* BQ24253_H_ */
