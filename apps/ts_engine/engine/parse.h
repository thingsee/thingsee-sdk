/****************************************************************************
 * apps/thingsee/engine/parse.h
 *
 * Copyright (C) 2014-2016 Haltian Ltd. All rights reserved.
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
 * Authors:
 *   Pekka Ervasti <pekka.ervasti@haltian.com>
 *
 ****************************************************************************/

#ifndef __TS_ENGINE_PARSE_H__
#define __TS_ENGINE_PARSE_H__

#include <nuttx/config.h>
#include <stdbool.h>
#include <queue.h>

#include "connectors/connector.h"
#include "parse_labels.h"

#include <apps/ts_engine/ts_engine.h>

#define FIRST_ORDER_ID				1

typedef int state_id_t;
typedef int purpose_id_t;
typedef int event_id_t;
typedef int connector_id_t;
typedef int order_id_t;
typedef double threshold_t;
typedef int rate_t;
typedef int count_t;
typedef int bool_t;
typedef uint64_t timestamp_t;

struct ts_threshold;
struct ts_value;

typedef bool (*check_threshold_t)(struct ts_threshold *threshold, struct ts_value *value);

enum ts_threshold_t
{
  isOneOf, isGt, isLt, isNotIn, isAny, isInsideGeo,
};

struct ts_global_state
{
  sq_entry_t entry;
  struct ts_state *state;
};


/* list object to store currently active senses */

struct ts_active_sense

{
  sq_entry_t entry;
  struct ts_sense *sense;
};

struct ts_state;
struct ts_event;

enum ts_engine_state
{
  ENGINE_STOPPED,
  ENGINE_RUNNING,
  ENGINE_PAUSED
};

struct ts_sms_limit
{
  uint32_t min_interval_minutes;
  uint32_t max_per_day;
};

struct ts_engine
{
  struct ts_engine_app *app;
  struct ts_profile *profile;
  struct ts_purpose *current_purpose;
  struct ts_state *current_state;
  struct ts_sms_limit sms_limit;
  sq_queue_t global_states;
  enum ts_engine_state state;
#if defined(CONFIG_SYSTEM_CONMAN) && !defined(CONFIG_ARCH_SIM)
  uint32_t conman_connid;
#endif
};

struct ts_profile
{
  struct ts_engine *parent;

  struct
  {
    char * const apiVersion;
    char * const pId;
    char * const name;
    purpose_id_t initPuId;
    sq_queue_t purposes;
  } conf;
};

struct ts_purpose
{
  /* Housekeeping */

  sq_entry_t entry;
  struct ts_profile *parent;

  /* Data */

  struct
  {
    purpose_id_t puId;
    char *name;
    state_id_t initStId;
    sq_queue_t states;
  } conf;
};

struct ts_state
{
  /* Housekeeping */

  sq_entry_t entry;
  struct ts_purpose *parent;

  /* Data */

  struct
  {
    state_id_t stId;
    char *name;
    bool_t isGlobal;
    sq_queue_t events;
  } conf;

  struct
  {
    sq_queue_t active_causes;
  } dyn;
};

struct ts_phone_number
{
  sq_entry_t entry;

  char *phoneNumber;
};

struct ts_sms_action
{
  char *text;
  char *phoneNumber;
};

struct ts_cloud_action
{
  bool sendEvent;
  bool sendLog;
  bool sendPush;
  struct url url;
};

struct ts_engine_action
{
  state_id_t gotoStId;
  purpose_id_t gotoPuId;
};

struct ts_display_action
{
  char *showText;
};

struct ts_actions
{
  struct ts_sms_action sms;
  struct ts_cloud_action cloud;
  struct ts_engine_action engine;
  struct ts_display_action display;
};

struct ts_event
{
  /* Housekeeping */

  sq_entry_t entry;
  struct ts_state *parent;

  /* Data */

  struct
  {
    event_id_t evId;
    char *name;
    bool_t eventLog;
    struct ts_actions actions;
    int number_of_causes;
    sq_queue_t causes;
  } conf;

  struct
  {
    order_id_t expected_order_id;
  } dyn;
};

#ifdef CONFIG_ARCH_SIM
struct ts_sim_input
{
  double *data;
  int i;
  int max;
};
#endif

struct ts_measurement
{
  bool log;
  int interval;
  int count;
  bool send;
};

struct ts_threshold_params
{
  int count;
  bool negate;
  bool relative;
  int decimalAcc;
};

struct ts_cause
{
  /* Housekeeping */

  sq_entry_t entry;
  sq_entry_t active_entry;
  struct ts_event *parent;

  /* Data */

  struct
  {
    order_id_t orderId;
    bool_t senseLog;
    struct ts_measurement measurement;
    struct ts_threshold_params threshold;
    sq_queue_t thresholds;
  } conf;

  struct
  {
    int timer_id;
    struct timespec time; /* used for date timers */
    int fd;
    bool_t triggered;
    bool measure_bias:1;
    count_t measurement_counter;
    count_t threshold_counter;

    struct ts_sense_value sense_value;
    struct ts_sense_value sense_bias;
    const struct ts_sense_info *sense_info;

#ifdef CONFIG_ARCH_SIM
    struct ts_sim_input input;
#endif
    void *priv;
  } dyn;
};

struct ts_threshold
{
  /* Housekeeping */

  sq_entry_t entry;
  struct ts_cause *parent;

  /* Data */

  struct
  {
    struct ts_value value;
    enum ts_threshold_t type;
    check_threshold_t check_threshold;
  } conf;
};

void
profile_free (struct ts_profile *profile);

struct ts_profile *
profile_parse (const char *profile_str, int *errcode);

#endif
