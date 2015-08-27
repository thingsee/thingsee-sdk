/****************************************************************************
 * apps/system/ubmodem/ubmodem_voice_audio.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
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
#include <errno.h>

#include <apps/system/ubmodem.h>

#include "ubmodem_internal.h"
#include "ubmodem_voice.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes.
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct at_cmd_def_s cmd_ATpUGPIOW =
{
  .name         = "+UGPIOW",
  .resp_format  = NULL,
  .resp_num     = 0,
};

static const struct at_cmd_def_s cmd_ATpCMUT =
{
  .name         = "+CMUT",
  .resp_format  = NULL,
  .resp_num     = 0,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void simple_cmd_handler(struct ubmodem_s *modem,
                               const struct at_cmd_def_s *cmd,
                               const struct at_resp_info_s *info,
                               const uint8_t *resp_stream, size_t stream_len,
                               void *priv)
{
  /* Update main state machine. */

  __ubmodem_change_state(modem, MODEM_STATE_WAITING);
}

#ifdef UBMODEM_AUDIO_OUT_CTRL_GPIO

static int start_task_audio_setup_out_gpio(struct ubmodem_s *modem,
                                           int gpio, int high)
{
  int err;

  if (modem->level < UBMODEM_LEVEL_CMD_PROMPT)
    {
      return ERROR;
    }

  /* Setup audio OUT gpio. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpUGPIOW, simple_cmd_handler, modem,
                           "=%d,%d", gpio, high);
  MODEM_DEBUGASSERT(modem, err == OK);

  return OK;
}
#endif

#ifdef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_1
static int start_task_audio_setup_out_gpio1(struct ubmodem_s *modem, void *priv)
{
  bool out_on = (uintptr_t)priv;

#ifdef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_1_HIGH_OFF
  out_on = !out_on;
#endif

  return start_task_audio_setup_out_gpio(modem,
                                         CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_1,
                                         out_on);
}
#endif

#ifdef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_2
static int start_task_audio_setup_out_gpio2(struct ubmodem_s *modem, void *priv)
{
  bool out_on = (uintptr_t)priv;

#ifdef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_2_HIGH_OFF
  out_on = !out_on;
#endif

  return start_task_audio_setup_out_gpio(modem,
                                         CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_2,
                                         out_on);
}
#endif

static int start_task_audio_setup_in_mic(struct ubmodem_s *modem, void *priv)
{
  bool in_on = (uintptr_t)priv;
  bool mute = !in_on;
  int err;

  if (modem->level < UBMODEM_LEVEL_CMD_PROMPT)
    {
      return ERROR;
    }

  /* Setup mic muting. */

  err = __ubmodem_send_cmd(modem, &cmd_ATpCMUT, simple_cmd_handler, modem,
                           "=%d", mute);
  MODEM_DEBUGASSERT(modem, err == OK);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubmodem_audio_setup
 *
 * Description:
 *   Audio setup, enable/disable OUT/IN.
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void ubmodem_audio_setup(struct ubmodem_s *modem, bool out_on, bool in_on)
{
  DEBUGASSERT(modem);

  if (modem->level < UBMODEM_LEVEL_CMD_PROMPT)
    {
      return;
    }

#ifdef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_1
  (void)__ubmodem_add_task(modem, start_task_audio_setup_out_gpio1,
                           (void *)(uintptr_t)out_on);
#endif
#ifdef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_2
  (void)__ubmodem_add_task(modem, start_task_audio_setup_out_gpio2,
                           (void *)(uintptr_t)out_on);
#endif

  (void)__ubmodem_add_task(modem, start_task_audio_setup_in_mic,
                           (void *)(uintptr_t)in_on);
}
