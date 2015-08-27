/****************************************************************************
 * include/nuttx/sensors/max44009.h
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
 *   Authors: Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
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

#ifndef MAX44009_H_
#define MAX44009_H_

#include <nuttx/i2c.h>

/* IOCTL commands */

typedef enum max44009_ioctl_cmds_t
  {
    MAX44009_IOC_INIT = 0x0,
    MAX44009_IOC_READ_INTERRUPT_STATUS,
    MAX44009_IOC_SET_INTERRUPT,
    MAX44009_IOC_SET_CONT,
    MAX44009_IOC_SET_MANUAL,
    MAX44009_IOC_SET_CDR,
    MAX44009_IOC_SET_INTEGR_TIME,
    MAX44009_IOC_DO_SELFCALIB,
    MAX44009_IOC_SET_THRES_TIMER,
    MAX44009_IOC_READ_RAW_DATA,
    MAX44009_IOC_DO_SELFTEST
  } max44009_ioctl_cmds_t;

/* Integration time, ms */

typedef enum max44009_integration_time_t
  {
    MAX44009_INTEGR_TIME_800 = 0x0,     /* Preferred mode for boosting
                                         * low-light sensitivity */
    MAX44009_INTEGR_TIME_400,
    MAX44009_INTEGR_TIME_200,
    MAX44009_INTEGR_TIME_100,   /* Preferred mode for high-brightness
                                 * applications */
    MAX44009_INTEGR_TIME_50,    /* Manual mode only */
    MAX44009_INTEGR_TIME_25,    /* Manual mode only */
    MAX44009_INTEGR_TIME_12_5,  /* Manual mode only */
    MAX44009_INTEGR_TIME_6_25   /* Manual mode only */
  } max44009_integration_time_t;

/* Interrupt configuration data structure */

typedef struct max44009_config_t
  {
    int irq;
    int (*irq_attach) (FAR struct max44009_config_t * state, xcpt_t isr);
    void (*irq_enable) (FAR struct max44009_config_t * state, bool enable);
    void (*irq_clear) (FAR struct max44009_config_t * state);
  } max44009_config_t;

/* Configuration structure for MAX44009 */

typedef struct max44009_init_ops_t
  {
    bool is_cont;               /* Needs more power, if it's in continuous
                                 * mode. This one is usefull in test-mode for
                                 * instance */
    bool is_manual;             /* Timer's settings must be specified manually */
    bool is_cdr;                /* Current division ratio: false - All of the
                                 * photodiode current goes to the ADC, true -
                                 * 1/8 (must be used in high-brightness
                                 * situations) */
    max44009_integration_time_t integr_time;    /* Integration time */
    uint8_t upper_threshold;    /* Upper threshold high-byte */
    uint8_t lower_threshold;    /* Lower threshold high-byte */
    uint8_t threshold_timer;    /* 0 - interrupt will be triggered as soon as
                                 * the light level exceeds either threshold */
  } max44009_init_ops_t;

/* Data transfer structure */

typedef struct max44009_data_t
  {
    uint8_t int_sts;            /* interrupt status */
    bool is_interrupt;          /* Activates interrupt if set to true */
    bool is_cont;               /* Same as in init structure. Added for my own
                                 * convenience */
    bool is_manual;             /* Same as in init structure. Added for my own
                                 * convenience */
    bool is_cdr;                /* Same as in init structure. Added for my own
                                 * convenience */
    max44009_integration_time_t integr_time;    /* Same as in init structure.
                                                 * Added for my own convenience
                                                 */
    uint8_t threshold_timer;    /* Same as in init structure. Added for my own
                                 * convenience */
    uint16_t raw_value;         /* sensor's raw-data */
    uint8_t test_value;         /* test value for self-test. Must be
                                 * provided!!!! */
  } max44009_data_t;

int max44009_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                      uint8_t addr, max44009_config_t * config);

#endif /* MAX44009_H_ */
