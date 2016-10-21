/****************************************************************************
 * apps/system/ubmodem/ubmodem_main.c
 *
 *   Copyright (C) 2014-2016 Haltian Ltd. All rights reserved.
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

#include <apps/system/ubmodem.h>

#include "ubmodem_internal.h"
#include "ubmodem_hw.h"
#include "ubmodem_usrsock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

struct modem_event_listener_entry_s
{
  struct sq_entry_s list;
  ubmodem_event_func_t callback_fn;
  void *callback_priv;
  uint32_t event_types;
};

/* Modem task structure. */

struct modem_priv_task_s
{
  sq_entry_t entry;
  ubmodem_task_fn_t func;
  void *priv;
#ifdef CONFIG_UBMODEM_DEBUG_VERBOSE
  const char *regname;
#endif
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Transition/path matrix between different functionality levels. */

static const int8_t
  level_transition_matrix[__UBMODEM_LEVEL_MAX][__UBMODEM_LEVEL_MAX] =
{
  /* Current: */
  [UBMODEM_LEVEL_POWERED_OFF] =
    {
      /* Target:                         Next state on path to target: */
      [UBMODEM_LEVEL_POWERED_OFF]        = MODEM_STATE_WAITING,
      [UBMODEM_LEVEL_POWERED_ON]         = MODEM_STATE_DO_POWER_ON,
      [UBMODEM_LEVEL_CMD_PROMPT]         = MODEM_STATE_DO_POWER_ON,
      [UBMODEM_LEVEL_SIM_ENABLED]        = MODEM_STATE_DO_POWER_ON,
      [UBMODEM_LEVEL_NETWORK]            = MODEM_STATE_DO_POWER_ON,
      [UBMODEM_LEVEL_GPRS]               = MODEM_STATE_DO_POWER_ON,
      [UBMODEM_LEVEL_STUCK_HARDWARE]     = MODEM_STATE_FAKE_STUCK_HARDWARE,
    },
  [UBMODEM_LEVEL_POWERED_ON] =
    {
      [UBMODEM_LEVEL_POWERED_OFF]        = MODEM_STATE_SETUP_CMD_PROMPT,
      [UBMODEM_LEVEL_POWERED_ON]         = MODEM_STATE_WAITING,
      [UBMODEM_LEVEL_CMD_PROMPT]         = MODEM_STATE_SETUP_CMD_PROMPT,
      [UBMODEM_LEVEL_SIM_ENABLED]        = MODEM_STATE_SETUP_CMD_PROMPT,
      [UBMODEM_LEVEL_NETWORK]            = MODEM_STATE_SETUP_CMD_PROMPT,
      [UBMODEM_LEVEL_GPRS]               = MODEM_STATE_SETUP_CMD_PROMPT,
      [UBMODEM_LEVEL_STUCK_HARDWARE]     = MODEM_STATE_SETUP_CMD_PROMPT,
    },
  [UBMODEM_LEVEL_CMD_PROMPT] =
    {
      [UBMODEM_LEVEL_POWERED_OFF]        = MODEM_STATE_DO_POWER_OFF,
      [UBMODEM_LEVEL_POWERED_ON]         = MODEM_STATE_DO_POWER_OFF,
      [UBMODEM_LEVEL_CMD_PROMPT]         = MODEM_STATE_WAITING,
      [UBMODEM_LEVEL_SIM_ENABLED]        = MODEM_STATE_SETUP_SIM,
      [UBMODEM_LEVEL_NETWORK]            = MODEM_STATE_SETUP_SIM,
      [UBMODEM_LEVEL_GPRS]               = MODEM_STATE_SETUP_SIM,
      [UBMODEM_LEVEL_STUCK_HARDWARE]     = MODEM_STATE_FAKE_STUCK_HARDWARE,
    },
  [UBMODEM_LEVEL_SIM_ENABLED] =
    {
      [UBMODEM_LEVEL_POWERED_OFF]        = MODEM_STATE_DO_POWER_OFF,
      [UBMODEM_LEVEL_POWERED_ON]         = MODEM_STATE_DO_POWER_OFF,
      [UBMODEM_LEVEL_CMD_PROMPT]         = MODEM_STATE_DO_POWER_OFF,
      [UBMODEM_LEVEL_SIM_ENABLED]        = MODEM_STATE_WAITING,
      [UBMODEM_LEVEL_NETWORK]            = MODEM_STATE_SETUP_NETWORK,
      [UBMODEM_LEVEL_GPRS]               = MODEM_STATE_SETUP_NETWORK,
      [UBMODEM_LEVEL_STUCK_HARDWARE]     = MODEM_STATE_FAKE_STUCK_HARDWARE,
    },
  [UBMODEM_LEVEL_NETWORK] =
    {
      [UBMODEM_LEVEL_POWERED_OFF]        = MODEM_STATE_DO_POWER_OFF,
      [UBMODEM_LEVEL_POWERED_ON]         = MODEM_STATE_DO_POWER_OFF,
      [UBMODEM_LEVEL_CMD_PROMPT]         = MODEM_STATE_DO_POWER_OFF,
      [UBMODEM_LEVEL_SIM_ENABLED]        = MODEM_STATE_DO_POWER_OFF,
      [UBMODEM_LEVEL_NETWORK]            = MODEM_STATE_WAITING,
      [UBMODEM_LEVEL_GPRS]               = MODEM_STATE_OPEN_GPRS_CONNECTION,
      [UBMODEM_LEVEL_STUCK_HARDWARE]     = MODEM_STATE_FAKE_STUCK_HARDWARE,
    },
  [UBMODEM_LEVEL_GPRS] =
    {
      [UBMODEM_LEVEL_POWERED_OFF]        = MODEM_STATE_DO_POWER_OFF,
      [UBMODEM_LEVEL_POWERED_ON]         = MODEM_STATE_DO_POWER_OFF,
      [UBMODEM_LEVEL_CMD_PROMPT]         = MODEM_STATE_DO_POWER_OFF,
      [UBMODEM_LEVEL_SIM_ENABLED]        = MODEM_STATE_DO_POWER_OFF,
      [UBMODEM_LEVEL_NETWORK]            = MODEM_STATE_CLOSE_GPRS_CONNECTION,
      [UBMODEM_LEVEL_GPRS]               = MODEM_STATE_WAITING,
      [UBMODEM_LEVEL_STUCK_HARDWARE]     = MODEM_STATE_DISCONNECT_NETWORK,
    },
  [UBMODEM_LEVEL_STUCK_HARDWARE] =
    {
     [UBMODEM_LEVEL_POWERED_OFF]        = MODEM_STATE_DO_RESET,
     [UBMODEM_LEVEL_POWERED_ON]         = MODEM_STATE_DO_RESET,
     [UBMODEM_LEVEL_CMD_PROMPT]         = MODEM_STATE_DO_RESET,
     [UBMODEM_LEVEL_SIM_ENABLED]        = MODEM_STATE_DO_RESET,
     [UBMODEM_LEVEL_NETWORK]            = MODEM_STATE_DO_RESET,
     [UBMODEM_LEVEL_GPRS]               = MODEM_STATE_DO_RESET,
     [UBMODEM_LEVEL_STUCK_HARDWARE]     = MODEM_STATE_FAKE_STUCK_HARDWARE,
    },
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: modem_has_task_work
 *
 * Description:
 *   Check if there is tasks queued.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 * Return value:
 *   true if there is tasks waiting to do.
 *
 ****************************************************************************/

static bool modem_has_task_work(struct ubmodem_s *modem)
{
  /* Check if there is task to do. */

  if (sq_peek(&modem->tasks) != NULL)
    {
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: modem_do_task_work
 *
 * Description:
 *   Perform task work.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *
 ****************************************************************************/

static void modem_do_task_work(struct ubmodem_s *modem)
{
  struct modem_priv_task_s *task;
  int ret;

  /* Pick first task item. */

  task = (void *)sq_remfirst(&modem->tasks);
  if (!task)
    {
      /* No active items. */

      __ubmodem_change_state(modem, MODEM_STATE_WAITING);
      return;
    }

  ubdbg("executing task %p:<%s>.\n", task->func, task->regname);

  ret = task->func(modem, task->priv);
  if (ret != OK)
    {
      /* No work started, return to waiting state. */

      __ubmodem_change_state(modem, MODEM_STATE_WAITING);
    }

  free(task);
}

/****************************************************************************
 * Name: modem_do_state_machine
 *
 * Description:
 *   Main modem state machine
 *
 ****************************************************************************/

static void modem_do_state_machine(struct ubmodem_s *modem)
{
  enum modem_state_e curr_state;

  MODEM_DEBUGASSERT(modem, !modem->in_state_machine);

  modem->in_state_machine = true;

  do
    {
      curr_state = modem->state;

      switch (curr_state)
      {
      case MODEM_STATE_IN_SUBSTATE:
        /*
         * Sub-state machine active, don't do anything for main state machine.
         */

        break;

      case MODEM_STATE_WAITING:
        /*
         * Check if we need to initiate transition from one level to
         * another.
         */
        if (modem->intermediate_level < __UBMODEM_LEVEL_MAX &&
                         modem->intermediate_level != modem->level)
          {
            /* Get next state step for transition. */

            __ubmodem_change_state(modem,
              level_transition_matrix[modem->level][modem->intermediate_level]);
          }
        else if (modem->level != modem->target_level)
          {
            /* Get next state step for transition. */

            __ubmodem_change_state(modem,
                level_transition_matrix[modem->level][modem->target_level]);
          }
        else if (modem_has_task_work(modem))
          {
            /* There is task work queued */

            __ubmodem_change_state(modem, MODEM_STATE_DO_TASK_WORK);
          }
        else if (modem->level >= UBMODEM_LEVEL_GPRS)
          {
#ifdef CONFIG_UBMODEM_USRSOCK
            if (__ubmodem_has_usrsock_work(modem))
              {
                /* There is socket work queued for GPRS. */

                __ubmodem_change_state(modem, MODEM_STATE_DO_SOCKET_WORK);
              }
#endif
          }

        break;

      case MODEM_STATE_DO_TASK_WORK:
        /*
         * Task sub-state machine.
         */

        modem_do_task_work(modem);

        break;


      case MODEM_STATE_DO_SOCKET_WORK:
        /*
         * Socket work sub-state machine.
         */

#ifdef CONFIG_UBMODEM_USRSOCK
        __ubmodem_do_usrsock_work(modem);
#else
        __ubmodem_change_state(modem, MODEM_STATE_WAITING);
#endif

        break;

      case MODEM_STATE_DO_POWER_ON:
        /*
         * Power on the modem
         */

        __ubmodem_change_state(modem, MODEM_STATE_IN_SUBSTATE);
        __ubmodem_substate_start_do_power_on(modem);

        break;

      case MODEM_STATE_DO_RESET:
        /*
         * Do reset on the modem
         */

        __ubmodem_change_state(modem, MODEM_STATE_IN_SUBSTATE);
        __ubmodem_substate_start_do_reset(modem);

        break;

      case MODEM_STATE_DO_POWER_OFF:
        /*
         * Start "power off" sequence
         */

        __ubmodem_change_state(modem, MODEM_STATE_IN_SUBSTATE);
        __ubmodem_substate_start_do_power_off(modem);

        break;

      case MODEM_STATE_SETUP_CMD_PROMPT:
        /*
         * Start "setup command prompt" sequence
         */

        __ubmodem_change_state(modem, MODEM_STATE_IN_SUBSTATE);
        __ubmodem_substate_start_setup_cmd_prompt(modem);

        break;

      case MODEM_STATE_SETUP_SIM:
        /*
         * Start "setup SIM / PIN" sequence
         */

        __ubmodem_change_state(modem, MODEM_STATE_IN_SUBSTATE);
        __ubmodem_substate_start_setup_sim(modem);

        break;

      case MODEM_STATE_SETUP_NETWORK:
        /*
         * Start "setup GSM network" sequence
         */

        __ubmodem_change_state(modem, MODEM_STATE_IN_SUBSTATE);
        __ubmodem_substate_start_setup_network(modem);

        break;

      case MODEM_STATE_DISCONNECT_NETWORK:
        /*
         * Start "disconnect GSM network" sequence
         */

        __ubmodem_change_state(modem, MODEM_STATE_IN_SUBSTATE);
        __ubmodem_substate_start_disconnect_network(modem);

        break;

      case MODEM_STATE_OPEN_GPRS_CONNECTION:
        /*
         * Start "open GPRS connection" sequence
         */

        __ubmodem_change_state(modem, MODEM_STATE_IN_SUBSTATE);
        __ubmodem_substate_start_open_gprs(modem);

        break;

      case MODEM_STATE_CLOSE_GPRS_CONNECTION:
        /*
         * Start "close GPRS connection" sequence
         */

        __ubmodem_change_state(modem, MODEM_STATE_IN_SUBSTATE);
        __ubmodem_substate_start_close_gprs(modem);

        break;

      case MODEM_STATE_FAKE_STUCK_HARDWARE:
        /*
         * Fake the stuck hardware state.
         */

        __ubmodem_change_state(modem, MODEM_STATE_IN_SUBSTATE);
        __ubmodem_reached_level(modem, UBMODEM_LEVEL_STUCK_HARDWARE);

        break;

      default:

        printf("WARNING: Unknown modem state: %d\n", modem->state);

        break;
      }
    }
  while (curr_state != modem->state);

  modem->in_state_machine = false;
}

/****************************************************************************
 * Name: wake_timer_handler
 *
 * Description:
 *   Timer handler for waking-up main state machine.
 *
 ****************************************************************************/

static int wake_timer_handler(struct ubmodem_s *modem, const int timer_id,
                              void * const arg)
{
  MODEM_DEBUGASSERT(modem, !modem->in_state_machine);
  MODEM_DEBUGASSERT(modem, modem->wake_timer_active);

  modem->wake_timer_active = false;

  modem_do_state_machine(modem);

  /* Wake timer done, decrease HIGH activity. */

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, false);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_publish_event_slow
 *
 * Description:
 *   Publish modem event to callback functions (slow-path).
 *
 * Input Parameters:
 *   modem    : Modem data
 *   type     : Event type
 *   data     : Data buffer for event
 *   datalen  : Size of buffer
 *
 ****************************************************************************/

void __ubmodem_publish_event_slow(struct ubmodem_s *modem,
                           enum ubmodem_event_flags_e type, const void *data,
                           size_t datalen)
{
  struct modem_event_listener_entry_s *item, *next;

  modem->publishing_event++;

  /* Walk-through listeners and call ones interested in this event type. */

  item = (void *)sq_peek(&modem->event_listeners);
  while (item)
    {
      next = (void *)sq_next(&item->list);

      /* Check and call event function. */

      if (item->callback_fn && (type & item->event_types) != 0)
        item->callback_fn(modem, type, data, datalen, item->callback_priv);

      item = next;
    }

  modem->publishing_event--;

  /* Perform clean-up. */

  if (modem->publishing_event != 0)
    return;

  item = (void *)sq_peek(&modem->event_listeners);
  while (item)
    {
      next = (void *)sq_next(&item->list);

      if (!item->callback_fn)
        {
          sq_rem(&item->list, &modem->event_listeners);
          free(item);
        }

      item = next;
    }
}

/****************************************************************************
 * Name: ubmodem_register_event_listener
 *
 * Description:
 *   Register modem event listener.
 *
 * Input Parameters:
 *   event_types   : Event type flags this callback listens for.
 *   event_callback: Callback function for the event.
 *   callback_priv : Private data for callback.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

int ubmodem_register_event_listener(struct ubmodem_s *modem,
                                     uint32_t event_types,
                                     ubmodem_event_func_t event_callback,
                                     void *callback_priv)
{
  struct modem_event_listener_entry_s *item, *next;
  bool done = false;
  uint32_t active_events = 0;

  DEBUGASSERT(modem);
  MODEM_DEBUGASSERT(modem, event_types != 0);
  MODEM_DEBUGASSERT(modem, event_callback);

  /* Check if already registered. */

  item = (void *)sq_peek(&modem->event_listeners);
  while (item)
    {
      next = (void *)sq_next(&item->list);

      if (event_callback == item->callback_fn &&
          callback_priv == item->callback_priv)
        {
          /* Update event_types for callback. */

          item->event_types = event_types;

          done = true;
        }

      /* Gather active event types */

      if (event_callback != item->callback_fn)
        active_events |= item->event_types;

      item = next;
    }

  /* Update active events */

  modem->active_events = active_events;

  if (done)
    return OK;

  /* Allocate and setup new entry. */

  item = calloc(sizeof(*item), 1);
  if (!item)
    return ERROR;

  item->callback_fn = event_callback;
  item->callback_priv = callback_priv;
  item->event_types = event_types;

  modem->active_events |= event_types;

  sq_addlast(&item->list, &modem->event_listeners);

  return OK;
}

/****************************************************************************
 * Name: ubmodem_unregister_event_listener
 *
 * Description:
 *   Unregister modem event listener.
 *
 * Input Parameters:
 *   event_callback: Callback function for the event.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

int ubmodem_unregister_event_listener(struct ubmodem_s *modem,
                                       ubmodem_event_func_t event_callback)
{
  struct modem_event_listener_entry_s *item, *next;
  uint32_t active_events = 0;

  DEBUGASSERT(modem);
  MODEM_DEBUGASSERT(modem, event_callback);

  /* Check if already registered. */

  item = (void *)sq_peek(&modem->event_listeners);
  while (item)
    {
      next = (void *)sq_next(&item->list);

      if (event_callback == item->callback_fn)
        {
          /* Reset callback function */

          item->callback_fn = NULL;
          item->event_types = 0;
        }

      /* Gather active event types */

      if (event_callback != item->callback_fn)
        active_events |= item->event_types;

      /* Clean-up */

      if (modem->publishing_event == 0 && item->callback_fn == NULL)
        {
          sq_rem(&item->list, &modem->event_listeners);
          free(item);
        }

      item = next;
    }

  /* Update active events */

  modem->active_events = active_events;

  return OK;
}

/****************************************************************************
 * Name: ubmodem_request_level
 *
 * Description:
 *   Request functionality level from modem.
 *
 * Input Parameters:
 *   level   : Functional level requested for modem.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void ubmodem_request_level(struct ubmodem_s *modem,
                            enum ubmodem_func_level_e level)
{
  DEBUGASSERT(modem);

  /* Change the target level. */

  if (modem->force_level < __UBMODEM_LEVEL_MAX)
    modem->target_level = modem->force_level;
  else
    modem->target_level = level;

  /* Early exit for final power-off sequence. */

  __ubmodem_cancel_full_poweroff(modem);

  /* If modem does not have active work, initiate state machine. */

  if (modem->state == MODEM_STATE_WAITING && !modem->in_state_machine)
    modem_do_state_machine(modem);
}

/****************************************************************************
 * Name: __ubmodem_retry_current_level
 *
 * Description:
 *   Attempt to go to new level through intermediate level,
 *   current => intermediate => current.
 *
 * Input Parameters:
 *   level   : Functional level requested for modem.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void __ubmodem_retry_current_level(struct ubmodem_s *modem,
                                   enum ubmodem_func_level_e intermediate_level)
{
  DEBUGASSERT(modem);

  /* Set the intermediate level. */

  modem->intermediate_level = intermediate_level;

  /* If modem does not have active work, initiate state machine. */

  if (modem->state == MODEM_STATE_WAITING && !modem->in_state_machine)
    modem_do_state_machine(modem);
}

/****************************************************************************
 * Name: __ubmodem_level_transition_failed
 *
 * Description:
 *   Inform that modem could not reach targeted functionality level
 *
 * Input Parameters:
 *   modem    : Modem data
 *   new_level: New functionality level
 *
 ****************************************************************************/

void __ubmodem_level_transition_failed(struct ubmodem_s *modem,
                                   const char *reason_fmt, ...)
{
  char reason[64];
  struct ubmodem_event_transition_failed_s event_data =
  {
    .current_level = modem->level,
    .target_level  = modem->target_level,
    .reason        = reason,
  };
  va_list vaargs;

  if (modem->target_level == UBMODEM_LEVEL_POWERED_OFF)
    {
      /* If we fail power-off, mark HW as stuck and go to power-off through hw-reset. */

      __ubmodem_reached_level(modem, UBMODEM_LEVEL_STUCK_HARDWARE);
      return;
    }

  /* Generate reason string. */

  va_start(vaargs, reason_fmt);
  vsnprintf(reason, sizeof(reason), reason_fmt, vaargs);
  va_end(vaargs);

  /* Don't continue transition, stay at current level. */

  modem->target_level = modem->level;

  /* Go to waiting state */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);

  /* Publish error event */

  __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_FAILED_LEVEL_TRANSITION,
                      &event_data, sizeof(event_data));

  if (modem->force_level < __UBMODEM_LEVEL_MAX &&
      modem->target_level != modem->force_level)
    {
      ubmodem_request_level(modem, modem->force_level);
    }
}

/****************************************************************************
 * Name: __ubmodem_reached_level
 *
 * Description:
 *   Inform that modem state has reached new functionality level
 *
 * Input Parameters:
 *   modem    : Modem data
 *   new_level: New functionality level
 *
 ****************************************************************************/

void __ubmodem_reached_level(struct ubmodem_s *modem,
                         enum ubmodem_func_level_e new_level)
{
  enum ubmodem_func_level_e current_target = modem->target_level;
  struct ubmodem_event_new_level_s event_new_level =
  {
    .new_level = new_level,
    .old_level = modem->level,
  };

  if (new_level == modem->level)
    {
      /* Same level reached? Error? */

      MODEM_DEBUGASSERT(modem, false);

      return;
    }

  if (modem->intermediate_level == new_level)
    {
      /* Reached intermediate level. */

      modem->intermediate_level = __UBMODEM_LEVEL_MAX;
    }

  /* Update level */

  modem->level = new_level;

  /* Publish status event */

  __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_NEW_LEVEL, &event_new_level,
                      sizeof(event_new_level));

  /* Target reached? Publish another event */

  if (new_level == current_target)
    {
      struct ubmodem_event_target_level_reached_s event_target_reached =
      {
        .new_level = new_level,
      };

      __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_TARGET_LEVEL_REACHED,
                          &event_target_reached, sizeof(event_target_reached));
    }

  /* Go to waiting state */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);
}

/****************************************************************************
 * Name: __ubmodem_change_state
 *
 * Description:
 *   Set new state for main state machine.
 *
 * Input Parameters:
 *   modem    : Modem data
 *   new_state: New state
 *
 ****************************************************************************/

void __ubmodem_change_state(struct ubmodem_s *modem,
                        enum modem_state_e new_state)
{
  int states[2] = { new_state, modem->state }; /* New and old state. */
  bool was_waiting = modem->state == MODEM_STATE_WAITING;

  /* Moving from sub-state to other, call cleanup function is setup. */

  if (modem->substate_cleanup_fn && modem->state == MODEM_STATE_IN_SUBSTATE)
    {
      modem->substate_cleanup_fn(modem);
      modem->substate_cleanup_fn = NULL;
    }

  /* Publish event */

  __ubmodem_publish_event(modem, UBMODEM_EVENT_FLAG_TRACE_STATE_CHANGE, &states,
      sizeof(states));

  /* Inform power-management of activity change. */

  if (was_waiting && new_state != MODEM_STATE_WAITING)
    {
      /* From no-activity (waiting/idle) to low-activity (transition, task,
       * socket work).*/

      ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_LOW, true);
    }
  else if (!was_waiting && new_state == MODEM_STATE_WAITING)
    {
      /* From low-activity (non-waiting state) to no-activity (waiting/idle).
       */

      ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_LOW, false);
    }

  /* Update state */

  modem->state = new_state;

  /* Wake-up state machine. */

  __ubmodem_wake_waiting_state(modem, was_waiting);
}

/****************************************************************************
 * Name: __ubmodem_wake_waiting_state
 *
 * Description:
 *   Wake up modem main state machine from waiting state.
 *
 * Input Parameters:
 *   modem      : Modem data
 *   force_wake : Forced wake-up even if module not marked to waiting state.
 *
 ****************************************************************************/

void __ubmodem_wake_waiting_state(struct ubmodem_s *modem,
                              bool force_wake)
{
  int err;

  /* Run state machine if not in active state */

  if (modem->in_state_machine)
    return;

  if (modem->state != MODEM_STATE_WAITING && !force_wake)
    return;

  if (modem->wake_timer_active)
    return;

  /* Launching fast wake timer, increase HIGH activity. */

  ubmodem_pm_set_activity(modem, UBMODEM_PM_ACTIVITY_HIGH, true);

  /* Launch wake timer. */

  modem->wake_timer_active = true;
  err = __ubmodem_set_timer(modem, 1, &wake_timer_handler, modem);
  if (err == ERROR)
    {
      /* Error here? Add assert? Or just try bailout? */

      (void)wake_timer_handler(modem, -1, modem);
    }
}

/****************************************************************************
 * Name: ubmodem_is_powered_off
 *
 * Description:
 *   Check if modem in powered-off state and internal state machine is not
 *   active.
 *
 * Input Parameters:
 *   None.
 *
 ****************************************************************************/

bool ubmodem_is_powered_off(struct ubmodem_s *modem)
{
  DEBUGASSERT(modem);

  /* Internal state must be in waiting state, target level and current level
   * must be POWERED_OFF. */

  if (modem->state != MODEM_STATE_WAITING)
    return false;
  if (modem->level != UBMODEM_LEVEL_POWERED_OFF)
    return false;
  if (modem->target_level != UBMODEM_LEVEL_POWERED_OFF)
    return false;

  return true;
}

/****************************************************************************
 * Name: ubmodem_get_func_level
 *
 * Description:
 *   Read current functionality level of modem
 *
 ****************************************************************************/

enum ubmodem_func_level_e ubmodem_get_func_level(struct ubmodem_s *modem)
{
  DEBUGASSERT(modem);
  return modem->level;
}

/****************************************************************************
 * Name: ubmodem_force_func_level
 *
 * Description:
 *   Allow forcing modem level to some value (used for debugging).
 *   Library will keep up trying to reach this level and ignore any other
 *   level requests.
 *
 ****************************************************************************/

void ubmodem_force_func_level(struct ubmodem_s *modem, int level)
{
  if (level < 0 || level >= __UBMODEM_LEVEL_MAX)
    {
      modem->force_level = __UBMODEM_LEVEL_MAX;
    }
  else
    modem->force_level = level;

  ubmodem_request_level(modem, modem->target_level);
}

/****************************************************************************
 * Name: __ubmodem_add_task
 *
 * Description:
 *   Queue task for modem. Tasks are run when modem state machine would
 *   otherwise enter waiting state.
 *
 * Input Parameters:
 *   modem : Modem private structure.
 *   func  : Task function.
 *   priv  : Private data for task function.
 *
 * Return value:
 *   OK if queued successfully, ERROR if out of memory.
 *
 ****************************************************************************/

int ___ubmodem_add_task(struct ubmodem_s *modem, ubmodem_task_fn_t func,
                        void *priv, const char *regname)
{
  struct modem_priv_task_s *task;

  task = calloc(1, sizeof(*task));
  if (!task)
    {
      return ERROR;
    }

  /* Add new task. */

  task->func = func;
  task->priv = priv;
#ifdef CONFIG_UBMODEM_DEBUG_VERBOSE
  task->regname = regname;
#endif

  sq_addlast(&task->entry, &modem->tasks);

  ubdbg("add task to queue %p:<%s>.\n", task->func, task->regname);

  /* Wake-up main state machine. */

  __ubmodem_wake_waiting_state(modem, false);

  return OK;
}
