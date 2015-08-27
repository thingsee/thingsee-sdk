/****************************************************************************
 * apps/system/ubmodem/ubmodem_initialize.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
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
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <apps/system/ubmodem.h>

#include "ubmodem_command.h"
#include "ubmodem_parser.h"
#include "ubmodem_internal.h"
#include "ubmodem_hw.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: do_modem_start_selftests
 *
 * Description:
 *   Run various modem module selftests/unit-tests
 *
 ****************************************************************************/

static void do_modem_selftest(struct ubmodem_s *modem)
{
  static bool done = false;

  if (done)
    return;

  __ubmodem_parser_selftest();

  done = true;
  memset(modem, 0, sizeof(*modem));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubmodem_initialize
 *
 * Description:
 *   Initialization for modem library
 *
 * Input Parameters:
 *   hw_ops           : Modem HW control function structure
 *   hw_ops_priv      : Private data pointer for HW control functions
 *
 * Returned Values:
 *   Not NULL means the function was executed successfully
 *   NULL means the function was executed unsuccessfully
 *
 ****************************************************************************/

struct ubmodem_s *ubmodem_initialize(const struct ubmodem_hw_ops_s *hw_ops,
                                     void *hw_ops_priv)
{
  struct ubmodem_s *modem;
  bool is_vcc_off;

  modem = calloc(1, sizeof(*modem));
  if (!modem)
    return NULL;

  /* Run self-tests at start */

  do_modem_selftest(modem);

  sq_init(&modem->event_listeners);
  sq_init(&modem->timers);
#ifdef CONFIG_UBMODEM_USRSOCK
  sq_init(&modem->sockets.list);
  sq_init(&modem->sockets.removed_list);

  modem->sockets.usrsockfd = -1;
#endif

  modem->hw.ops = hw_ops;
  modem->hw.priv = hw_ops_priv;

  /* Open modem serial-port. */

  modem->serial_fd = ubmodem_hw_initialize(modem, &is_vcc_off);
  if (modem->serial_fd == ERROR)
    {
      free(modem);
      return NULL;
    }

  modem->creg_timer_id = -1;
  modem->poweroff_timer = -1;

  modem->force_level = __UBMODEM_LEVEL_MAX;
  modem->intermediate_level = __UBMODEM_LEVEL_MAX;

  modem->is_vcc_off = is_vcc_off;
  if (!modem->is_vcc_off)
    {
      /* After board initialization, modem is in POWERED_ON functional level. */

      modem->level = UBMODEM_LEVEL_POWERED_ON;
      modem->is_powered_off = false;
    }
  else
    {
      /* After board initialization, modem is in POWERED_OFF functional level. */

      modem->level = UBMODEM_LEVEL_POWERED_OFF;
      modem->is_powered_off = true;
    }

  /* Set initial state for modem main state machine. */

  modem->state = MODEM_STATE_WAITING;

  /* Modem is now initialized for initial setup. */

  modem->initialized = true;

  /* Start initial setup for AT command prompt. */

  ubmodem_request_level(modem, UBMODEM_LEVEL_POWERED_OFF);

  return modem;
}

/****************************************************************************
 * Name: ubmodem_uninitialize
 *
 * Description:
 *   Uninitialization for modem library
 *
 * Input Parameters:
 *   modem:  Pointer for modem library object from ubmodem_initialize
 *
 * Returned Values:
 *   None.
 *
 ****************************************************************************/

void ubmodem_uninitialize(struct ubmodem_s *modem)
{
  int ret;

  DEBUGASSERT(sq_peek(&modem->event_listeners) == NULL);
#ifdef CONFIG_UBMODEM_USRSOCK
  DEBUGASSERT(sq_peek(&modem->sockets.list) == NULL);
  DEBUGASSERT(sq_peek(&modem->sockets.removed_list) == NULL);
#endif
  DEBUGASSERT(sq_peek(&modem->timers) == NULL);
  DEBUGASSERT(sq_peek(&modem->tasks) == NULL);

  ret = ubmodem_hw_deinitialize(modem, modem->serial_fd);
  if (ret != OK)
    {
      dbg("Failed to deinitialize modem HW.\n");
    }
  free(modem);
}
