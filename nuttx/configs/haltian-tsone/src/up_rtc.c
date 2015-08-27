/****************************************************************************
 * Copyright (C) 2015 Haltian Ltd. All rights reserved.
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
 * Author: Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
 *
 ****************************************************************************/



#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "up_arch.h"



/****************************************************************************
 * Name: board_rtc_is_mass_storage_disallowed
 *
 * Description:
 *  Checks if mass-storage mode was disallowed by the user
 *
 * Input Parameters:
 *
 * Returned Values:
 *  Returns true if mass-storage mode disallowed. Otherwise false
 *
 ****************************************************************************/

bool board_rtc_is_mass_storage_disallowed(void)
{
  return !!(getreg32(STM32_RTC_BK17R));
}

/****************************************************************************
 * Name: board_rtc_block_mass_storage
 *
 * Description:
 *  Blocks mass-storage mode if needed. User must set this register
 *  right before SW-reset operation, if he/she plans to use device in
 *  mass-storage mode
 *
 * Input Parameters:
 *  block - true if user does not need mass-storage. Otherwise false.
 *
 * Returned Values:
 *
 ****************************************************************************/

void board_rtc_block_mass_storage(bool block)
{
  DEBUGASSERT((getreg32(STM32_PWR_CR) & PWR_CR_DBP) == 0);
  stm32_pwr_enablebkp(true);
  putreg32(block, STM32_RTC_BK17R);
  stm32_pwr_enablebkp(false);
}

/****************************************************************************
 * Name: board_rtc_index_to_block_reg
 *
 * Description:
 *  Convert an index to a register
 *
 * Input Parameters:
 *  index - the index value.
 *
 * Returned Values:
 *  Returns the register
 *
 ****************************************************************************/

static uint32_t board_rtc_index_to_block_reg(uint32_t index)
{
  return index ? STM32_RTC_BK16R : STM32_RTC_BK15R;
}

/****************************************************************************
 * Name: board_rtc_save_value
 *
 * Description:
 *   Persist a value
 *
 * Input Parameters:
 *  value - the value to persist.
 *  index - the index where to persist.
 *
 * Returned Values:
 *
 ****************************************************************************/

void board_rtc_save_value(uint32_t value, uint32_t index)
{
  uint32_t reg = board_rtc_index_to_block_reg(index);
  DEBUGASSERT((getreg32(STM32_PWR_CR) & PWR_CR_DBP) == 0);
  stm32_pwr_enablebkp(true);
  putreg32(value, reg);
  stm32_pwr_enablebkp(false);
}

/****************************************************************************
 * Name: board_rtc_read_value
 *
 * Description:
 *   Read a persisted value
 *
 * Input Parameters:
 *  index - the index.
 *
 * Returned Values:
 *  Returns a value
 *
 ****************************************************************************/

uint32_t board_rtc_read_value(uint32_t index)
{
  uint32_t reg = board_rtc_index_to_block_reg(index);
  return getreg32(reg);
}
