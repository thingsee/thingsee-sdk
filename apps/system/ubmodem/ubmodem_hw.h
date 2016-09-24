/****************************************************************************
 * apps/system/ubmodem/ubmodem_hw.h
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

#ifndef __SYSTEM_UBMODEM_UBMODEM_HW_H_
#define __SYSTEM_UBMODEM_UBMODEM_HW_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#include <apps/system/ubmodem.h>

#include "ubmodem_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Inline Functions
 ****************************************************************************/

static inline int ubmodem_hw_initialize(struct ubmodem_s *modem,
                                        bool *is_vcc_off)
{
  if (!modem->hw.ops || !modem->hw.ops->initialize)
    return ERROR;

  return modem->hw.ops->initialize(modem->hw.priv, is_vcc_off);
}

static inline int ubmodem_hw_deinitialize(struct ubmodem_s *modem,
                                          int serial_fd)
{
  if (!modem->hw.ops || !modem->hw.ops->deinitialize)
    return ERROR;

  return modem->hw.ops->deinitialize(modem->hw.priv, serial_fd);
}

static inline bool ubmodem_hw_vcc_set(struct ubmodem_s *modem, bool on)
{
  if (!modem->hw.ops || !modem->hw.ops->vcc_set)
    return false;

  return modem->hw.ops->vcc_set(modem->hw.priv, on);
}

static inline uint32_t ubmodem_hw_poweron_pin_set(struct ubmodem_s *modem,
                                                  bool set)
{
  if (!modem->hw.ops || !modem->hw.ops->poweron_pin_set)
    return 0;

  return modem->hw.ops->poweron_pin_set(modem->hw.priv, set);
}

static inline uint32_t ubmodem_hw_reset_pin_set(struct ubmodem_s *modem,
                                                bool set)
{
  if (!modem->hw.ops || !modem->hw.ops->reset_pin_set)
    return 0;

  return modem->hw.ops->reset_pin_set(modem->hw.priv, set);
}

static inline void ubmodem_pm_set_activity(struct ubmodem_s *modem,
                                           enum ubmodem_hw_pm_activity_e type,
                                           bool set)
{
  if (!modem->hw.ops || !modem->hw.ops->pm_set_activity)
    return;

  modem->hw.ops->pm_set_activity(modem->hw.priv, type, set);
}

#endif /* __SYSTEM_UBMODEM_UBMODEM_HW_H_ */
