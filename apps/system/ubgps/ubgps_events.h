/****************************************************************************
 * apps/system/ubgps/ubgps_events.h
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

#ifndef __THINGSEE_GPS_EVENTS_H
#define __THINGSEE_GPS_EVENTS_H

#include <apps/thingsee/modules/ts_gps.h>

/****************************************************************************
 * Public Defines
 ****************************************************************************/

/* GPS state machine events */

enum sm_event_id_e
{
  SM_EVENT_ENTRY = 0,
  SM_EVENT_EXIT,
  SM_EVENT_TIMEOUT,
  SM_EVENT_TARGET_STATE,
  SM_EVENT_UBX_MESSAGE,
  SM_EVENT_UBX_STATUS,
  SM_EVENT_AID_STATUS,
  SM_EVENT_PSM_STATE,
};
typedef enum sm_event_id_e sm_event_id_t;

/* UBX status */

enum ubx_status_e
{
  UBX_STATUS_ACK = 0,
  UBX_STATUS_NAK,
  UBX_STATUS_TIMEOUT,
};
typedef enum ubx_status_e ubx_status_t;

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GPS internal event base */

struct sm_event_s
{
  /* GPS state machine event ID */

  sm_event_id_t id;
};


/* Timeout event */

struct sm_event_timeout_s
{
  /* Event base class */

  struct sm_event_s super;

  /* Timer ID */

  int timer_id;
};


/* Target state event */

struct sm_event_target_state_s
{
  /* Event base class */

  struct sm_event_s super;

  /* Target state */

  gps_state_t target_state;

  /* Timeout in seconds */

  uint32_t timeout;
};


/* UBX message event */

struct sm_event_ubx_msg_s
{
  /* Event base class */

  struct sm_event_s super;

  /* UBX message */

  struct ubx_msg_s const * msg;
};


/* UBX status event */

struct sm_event_ubx_status_s
{
  /* Event base class */

  struct sm_event_s super;

  /* UBX class ID */

  uint8_t class_id;

  /* UBX message ID */

  uint8_t msg_id;

  /* UBX status */

  ubx_status_t status;
};


/* Aiding event */

struct sm_event_aid_s
{
  /* Event base class */

  struct sm_event_s super;

  /* Aiding data age */

  long int age;
};

/* Power save mode event */

struct sm_event_psm_event_s
{
  /* Event base class */

  struct sm_event_s super;

  /* PSM state */

  bool enable;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct ubgps_s;
struct sm_event_s;

/* GPS state machine function prototype */

typedef int (*gps_sm_t)(struct ubgps_s * const gps, struct sm_event_s const * const event);

extern const gps_sm_t gps_sm[__GPS_STATE_MAX];

#endif /* __THINGSEE_GPS_EVENTS_H */
