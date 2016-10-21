/****************************************************************************
 * apps/include/thingsee/modules/ts_accel.h
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
 *   Author: Timo Voutilainen <timo.voutilainen@haltian.com>
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

#ifndef __APPS_INCLUDE_THINGSEE_TS_ACCEL_H
#define __APPS_INCLUDE_THINGSEE_TS_ACCEL_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Accelerometer public events */

enum accel_event_id_e
{
  /* Impact event */

  ACCEL_EVENT_IMPACT  =               (1 << 0),

  /* Tilt event */

  ACCEL_EVENT_TILT =                  (1 << 1),

};

/* Accel public event base */

struct accel_event_s
{
  /* Accel event ID */

  enum accel_event_id_e id;
};

/* Accel impact event */

struct accel_event_impact
{
  /* Event base class */

  struct accel_event_s super;

  /* Impact value */

  int16_t impact_value;
};

/* Accel tilt event */

struct accel_event_tilt
{
  /* Event base class */

  struct accel_event_s super;

  /* Accel Tilt */

  int16_t tilt_value;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ts_accel_callback_t
 *
 * Description:
 *   Accelerometer callback function prototype
 *
 * Input Parameters:
 *   e           - Pointer to event
 *   priv        - Pointer to private data
 *
 ****************************************************************************/
typedef void (*accel_callback_t)(void const * const e,
                               void * const priv);

/****************************************************************************
 * Name: ts_accel_initialize
 *
 * Description:
 *   Initialize accelerometer module daemon
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_accel_initialize(void);

/****************************************************************************
 * Name: ts_accel_callback_register
 *
 * Description:
 *   Register callback function for accelerometer events and data retrieval
 *
 * Input Parameters:
 *   callback    - Callback function
 *   priv        - Pointer to private data
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_accel_callback_register(uint32_t const event_mask, accel_callback_t callback, void * const priv);

/****************************************************************************
 * Name: ts_accel_callback_unregister
 *
 * Description:
 *   Unregister callback from accelerometer module
 *
 * Input Parameters:
 *   callback    - Callback function
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_accel_callback_unregister(accel_callback_t callback);

/****************************************************************************
 * Name: ts_accel_get_axes
 *
 * Description:
 *   Get latest accelerometer values
 *
 * Input Parameters:
 *   x  - pointer to x value
 *   y  - pointer to y value
 *   z  - pointer to z value
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_accel_get_axes(int16_t *x, int16_t *y, int16_t *z);

/****************************************************************************
 * Name: ts_accel_get_angle
 *
 * Description:
 *   Get latest z -axis angle based on accelerometer values
 *
 * Input Parameters:
 *   angle - pointer to angle value
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_accel_get_angle(int16_t * angle);

/****************************************************************************
 * Name: ts_accel_get_max_angle
 *
 * Description:
 *   Get max z -axis angle based on accelerometer values
 *
 * Input Parameters:
 *   angle - pointer to angle value
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int ts_accel_get_max_angle(int16_t * angle);

/****************************************************************************
 * Name: ts_accel_reset_max_angle
 *
 * Description:
 *   Reset maximum angle value to 0
 *
 ****************************************************************************/
void ts_accel_reset_max_angle(void);

/****************************************************************************
 * Name: ts_accel_get_impact_value
 *
 * Description:
 *   Get Impact value
 *
 * Input Parameters:
 *   value  - pointer to impact g value
 *
 * Returned Values:
 *   ERROR in case there is no impact value otherwise OK
 *
 ****************************************************************************/
int ts_accel_get_impact_value(double *value);

/****************************************************************************
 * Name: ts_accel_get_max_impact_value
 *
 * Description:
 *   Get maximum Impact value since it was previously reset
 *
 * Input Parameters:
 *   value  - pointer to impact g value
 *
 * Returned Values:
 *   ERROR in case there is no max impact value otherwise OK
 *
 ****************************************************************************/
int ts_accel_get_max_impact_value(double *value);

/****************************************************************************
 * Name: ts_accel_reset_max_impact_value
 *
 * Description:
 *   Reset maximum Impact value to invalid
 *
 ****************************************************************************/
void ts_accel_reset_max_impact_value(void);

/****************************************************************************
 * Name: udpate_accel_status
 *
 * Description:
 *   Update accelerometer status
 *
 * Input Parameters:
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/
int update_accel_status(void);

#endif /* __APPS_INCLUDE_THINGSEE_TS_ACCEL_H */
