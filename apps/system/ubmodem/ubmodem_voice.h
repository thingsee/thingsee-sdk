/****************************************************************************
 * apps/system/ubmodem/ubmodem_voice.h
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

#ifndef __SYSTEM_UBMODEM_UBMODEM_VOICE_H_
#define __SYSTEM_UBMODEM_UBMODEM_VOICE_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#include <apps/system/ubmodem.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_1
#  if CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_1 < 0
#    undef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_1
#  endif
#endif

#ifdef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_2
#  if CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_2 < 0
#    undef CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_2
#  endif
#endif

#undef UBMODEM_AUDIO_OUT_CTRL_GPIO
#if defined(CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_1) || \
    defined(CONFIG_UBMODEM_AUDIO_OUT_CTRL_GPIO_2)
#  define UBMODEM_AUDIO_OUT_CTRL_GPIO 1
#endif

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Public Data exports
 ****************************************************************************/

/****************************************************************************
 * Public Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: __ubmodem_audio_presetup
 *
 * Description:
  *  Additional audio setup, enable/disable OUT/IN before main setup routine
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_audio_presetup(struct ubmodem_s *modem, bool out_on,
                              bool in_on);

/****************************************************************************
 * Name: __ubmodem_audio_postsetup
 *
 * Description:
 *  Additional audio setup, enable/disable OUT/IN after main setup routine
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_audio_postsetup(struct ubmodem_s *modem, bool out_on,
                               bool in_on);

/****************************************************************************
 * Name: __ubmodem_audio_cleanup_additional
 *
 * Description:
 *  Additional audio setup cleanup
 *
 * Input Parameters:
 *   modem    : Modem data
 *
 ****************************************************************************/

void __ubmodem_audio_cleanup_additional(struct ubmodem_s *modem);

#endif /* __SYSTEM_UBMODEM_UBMODEM_VOICE_H_ */
