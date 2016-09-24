/****************************************************************************
 * apps/include/thingsee/ts_core.h
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

#ifndef __APPS_INCLUDE_THINGSEE_TS_CORE_H
#define __APPS_INCLUDE_THINGSEE_TS_CORE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <poll.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Minimum timer id value, set to one to detect stopping of uninitialized
 * timers. */

#define TS_CORE_FIRST_TIMER_ID 1

/* Preprocessor magic to convert __LINE__ to string. */

#ifndef STRINGIFY
#  define STRINGIFY(x) #x
#endif
#ifndef TOSTRING
#  define TOSTRING(x) STRINGIFY(x)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* File descriptor callback */

typedef int (*ts_fd_callback_t)(const struct pollfd * const pfd, void * const priv);

/* Timer type */

enum ts_timer_type_e {
  TS_TIMER_TYPE_DATE = 0,       /* Timer type is date timer (using RTC, allow deep-sleep) */
  TS_TIMER_TYPE_INTERVAL,       /* Timer type is interval (using RTC, allow deep-sleep) */
  TS_TIMER_TYPE_TIMEOUT,        /* Timer type is one shot timeout (using RTC, allow deep-sleep) */
  TS_TIMER_TYPE_MAX,
};
typedef enum ts_timer_type_e ts_timer_type_t;

/* Timer callback */

typedef int (*ts_timer_callback_t)(const int timer_id, void * const priv);

/* Date timer callback */

typedef int (*ts_timer_date_callback_t)(const int timer_id,
    const struct timespec *date, void * const priv);

/* Deep-sleep hook function */

typedef bool (*ts_deepsleep_hook_t)(void * const priv);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ts_core_initialize
 *
 * Description:
 *   Initialize Thingsee core
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ts_core_initialize(void);

/****************************************************************************
 * Name: ts_core_deinitialize
 *
 * Description:
 *   De-initialize Thingsee core
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ts_core_deinitialize(void);

/****************************************************************************
 * Name: ts_core_fd_register
 *
 * Description:
 *   Register file descriptor for monitoring
 *
 * Input Parameters:
 *   fd          - File descriptor to monitor
 *   events      - File descriptor poll event type (POLLIN, POLLOUT ...)
 *   callback    - Pointer to file descriptor callback function
 *   priv        - Private data for file descriptor callback
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
#define ts_core_fd_register(fd, events, callback, priv) \
        __ts_core_fd_register(fd, events, callback, priv, \
                              __FILE__ ":" TOSTRING(__LINE__))

int __ts_core_fd_register(const int fd, const pollevent_t events,
                          const ts_fd_callback_t callback, void * const priv,
                          const char *reg_func_name);

/****************************************************************************
 * Name: ts_core_fd_set_poll_events
 *
 * Description:
 *   Update new poll events for file descriptor.
 *   Call this only from callback
 *
 * Input Parameters:
 *   fd          - File descriptor that is currently monitored
 *   events      - new poll events
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/

int ts_core_fd_set_poll_events(const int fd, const pollevent_t events);

/****************************************************************************
 * Name: ts_core_fd_unregister
 *
 * Description:
 *   Unregister file descriptor from monitoring
 *
 * Input Parameters:
 *   fd          - File descriptor that is currently monitored
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ts_core_fd_unregister(const int fd);

/****************************************************************************
 * Name: ts_core_timer_setup
 *
 * Description:
 *   Setup timer interval / one shot timeout timer
 *
 * Input Parameters:
 *   type        - Timer type (TS_TIMER_TYPE_INTERVAL, TS_TIMER_TYPE_TIMEOUT)
 *   timeout_ms  - Timer timeout in milliseconds
 *   callback    - Pointer to timer callback function
 *   priv        - Private data for timer callback
 *
 * Returned Value:
 *   Timer ID (>= 0) when timer was created successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ts_core_timer_setup(const ts_timer_type_t type, const uint32_t timeout_ms, const ts_timer_callback_t callback, void * const priv);

/****************************************************************************
 * Name: ts_core_timer_setup_date
 *
 * Description:
 *   Setup RTC based date/time timer
 *
 * Input Parameters:
 *   date        - Timer timeout as absolute time
 *   callback    - Pointer to timer callback function
 *   priv        - Private data for timer callback
 *
 * Returned Value:
 *   Timer ID (>= 0) when timer was created successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ts_core_timer_setup_date(const struct timespec *date, const ts_timer_date_callback_t callback, void * const priv);

/****************************************************************************
 * Name: ts_core_timer_stop
 *
 * Description:
 *   Stop interval timer
 *
 * Input Parameters:
 *   timer_id    - Timer ID
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
#define ts_core_timer_stop(timer_id) ({ \
    char __fail_compile_if_timer_id_type_is_unsigned[ \
      (!(((typeof(timer_id))-1) >= 0) * 2 - 1) \
    ]; \
    DEBUGASSERT(timer_id >= TS_CORE_FIRST_TIMER_ID); \
    __ts_core_timer_stop(timer_id); \
  })

int __ts_core_timer_stop(const int timer_id);

/****************************************************************************
 * Name: ts_core_deepsleep_hook_add
 *
 * Description:
 *   Add deep-sleep hook. Deep-sleep hook functions are called before entering
 *   deep-sleep. If hook returns 'false', deep-sleep is prevented. If all hooks
 *   return 'true', deep-sleep is entered.
 *
 * Input Parameters:
 *   hookfn     - Pointer to deep-sleep hook function
 *   priv       - Private data for timer callback
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
#define ts_core_deepsleep_hook_add(hookfn, priv) \
        __ts_core_deepsleep_hook_add(hookfn, priv, \
                                     __FILE__ ":" TOSTRING(__LINE__))

int __ts_core_deepsleep_hook_add(const ts_deepsleep_hook_t hookfn,
                                 void * const priv, const char *reg_func_name);

/****************************************************************************
 * Name: ts_core_deepsleep_hook_remove
 *
 * Description:
 *   Remove deep-sleep hook
 *
 * Input Parameters:
 *   hookfn - Hook function
 *
 * Returned Value:
 *   0 (OK) means the function was executed successfully
 *  -1 (ERROR) means the function was executed unsuccessfully. Check value of
 *  errno for more details.
 *
 ****************************************************************************/
int ts_core_deepsleep_hook_remove(ts_deepsleep_hook_t hookfn);

/****************************************************************************
 * Name: ts_core_is_date_before_compile_date
 *
 * Description:
 *   Check if date for given timestamp is older than the compile date of binary.
 *
 * Input Parameters:
 *   ctime  - Timestamp
 *
 * Returned Value:
 *   true - Timestamp date is older than build date
 *   false - Timestamp date is same or newer than build date
 *
 ****************************************************************************/
bool ts_core_is_date_before_compile_date(time_t ctime);

/****************************************************************************
 * Name: ts_core_mainloop
 *
 * Description:
 *   Run Thingsee core main event loop
 *
 * Input parameters:
 *   goon - set to false to escape the otherwise endless loop
 *
 ****************************************************************************/
void ts_core_mainloop(volatile bool *goon);

/****************************************************************************
 * Name: ts_core_selftest
 *
 * Description:
 *   Run Thingsee core selftests. Thingsee core must be initialized with
 *   ts_core_initialize before calling this function. Also this function
 *   assumes that there are no registered files or timers setup.
 *
 ****************************************************************************/
void ts_core_selftest(void);

#endif /* __APPS_INCLUDE_THINGSEE_TS_CORE_H */
