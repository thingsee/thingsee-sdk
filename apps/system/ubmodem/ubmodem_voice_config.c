/****************************************************************************
 * apps/system/ubmodem/ubmodem_voice_config.c
 *
 *   Copyright (C) 2015-2016 Haltian Ltd. All rights reserved.
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

#ifdef UBMODEM_AUDIO_OUT_CTRL_GPIO

static const struct at_cmd_def_s cmd_ATpUGPIOC =
{
  .name         = "+UGPIOC",
  .resp_format  = NULL,
  .resp_num     = 0,
  .timeout_dsec = 10 * 10,
};

#endif /*UBMODEM_AUDIO_OUT_CTRL_GPIO*/

static const struct at_cmd_def_s cmd_ATpCRC =
{
  .name         = "+CRC",
  .resp_format  = NULL,
  .resp_num     = 0,
};

static const struct at_cmd_def_s cmd_ATpUCALLSTAT =
{
  .name         = "+UCALLSTAT",
  .resp_format  = NULL,
  .resp_num     = 0,
};

static const struct at_cmd_def_s cmd_ATpCLIP =
{
  .name         = "+CLIP",
  .resp_format  = NULL,
  .resp_num     = 0,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef UBMODEM_AUDIO_OUT_CTRL_GPIO

static void config_audio_out_ctrl_gpio(struct ubmodem_s *modem, int pin,
                                       bool set)
{
  int err;

  err = __ubmodem_send_cmd(modem, &cmd_ATpUGPIOC,
                           __ubmodem_cmdprompt_generic_config_handler,
                           (void *)&cmd_ATpUGPIOC, "=%d,0,%d", pin, set);
  MODEM_DEBUGASSERT(modem, err == OK);
}

#endif /*UBMODEM_AUDIO_OUT_CTRL_GPIO*/

#ifdef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_1

static void config_audio_out_ctrl_gpio_1(struct ubmodem_s *modem)
{
  bool high_off;

#ifdef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_1_HIGH_OFF
  high_off = true;
#else
  high_off = false;
#endif

  config_audio_out_ctrl_gpio(modem, CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_1,
                             high_off);
}

#endif /*CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_1*/

#ifdef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_2

static void config_audio_out_ctrl_gpio_2(struct ubmodem_s *modem)
{
  bool high_off;

#ifdef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_2_HIGH_OFF
  high_off = true;
#else
  high_off = false;
#endif

  config_audio_out_ctrl_gpio(modem, CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_2,
                             high_off);
}

#endif /*CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_2*/

static void config_enable_urc_common(struct ubmodem_s *modem,
                                     const struct at_cmd_def_s *cmd)
{
  int err;

  __ubmodem_voice_control_setup(modem);

  err = __ubmodem_send_cmd(modem, cmd,
                           __ubmodem_cmdprompt_generic_config_handler,
                           (void *)cmd, "=1");
  MODEM_DEBUGASSERT(modem, err == OK);
}

static void config_enable_cring_urc(struct ubmodem_s *modem)
{
  config_enable_urc_common(modem, &cmd_ATpCRC);
}

static void config_enable_ucallstat_urc(struct ubmodem_s *modem)
{
  config_enable_urc_common(modem, &cmd_ATpUCALLSTAT);
}

static void config_enable_clip_urc(struct ubmodem_s *modem)
{
  config_enable_urc_common(modem, &cmd_ATpCLIP);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_voice_get_config_commands
 *
 * Description:
 *   Get configuration command list for voice-call setup.
 *
 * Input Parameters:
 *   modem    : Modem data
 *   ncmds    : Number of commands in array
 *
 * Return value:
 *   Pointer to command array
 *
 ****************************************************************************/

const ubmodem_send_config_func_t *
__ubmodem_voice_get_config_commands(struct ubmodem_s *modem, size_t *ncmds)
{
  static const ubmodem_send_config_func_t commands[] =
  {
#ifdef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_1
     config_audio_out_ctrl_gpio_1,
#endif
#ifdef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_2
     config_audio_out_ctrl_gpio_2,
#endif
     config_enable_cring_urc,
     config_enable_ucallstat_urc,
     config_enable_clip_urc,
  };

  *ncmds = ARRAY_SIZE(commands);

  return commands;
}
