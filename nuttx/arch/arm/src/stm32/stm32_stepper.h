/************************************************************************************
 * arch/arm/src/stm32/stm32_stepper.h
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Authors: Harri Luhtala <harri.luhtala@harri.luhtala@haltian.com>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_STM32_STEPPER_H
#define __ARCH_ARM_SRC_STM32_STM32_STEPPER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Timer devices may be used for different purposes. One special purpose is to
 * generate control signal for such things as stepper motors. If CONFIG_STM32_TIMn
 * is defined then the CONFIG_STM32_TIMn_STEPPER must also be defined to indicate that
 * timer "n" is intended to be used for stepper motor control.
 */

#ifndef CONFIG_STM32_TIM2
#  undef CONFIG_STM32_TIM2_STEPPER
#endif
#ifndef CONFIG_STM32_TIM3
#  undef CONFIG_STM32_TIM3_STEPPER
#endif
#ifndef CONFIG_STM32_TIM4
#  undef CONFIG_STM32_TIM4_STEPPER
#endif
#ifndef CONFIG_STM32_TIM5
#  undef CONFIG_STM32_TIM5_STEPPER
#endif

#include <arch/board/board.h>
#include "chip/stm32_tim.h"

/* For each timer that is enabled for stepper motor control usage, we need to
 * associate control signal GPIOs, this is to be done on board level configuration:
 *
 * TIMx_P1_P - Specifies output GPIO that is used by timer for stepper motor
 *             phase 1 positive terminal signal.
 * TIMx_P1_N - Specifies output GPIO that is used by timer for stepper motor
 *             phase 1 negative terminal signal.
 * TIMx_P2_P - Specifies output GPIO that is used by timer for stepper motor
 *             phase 2 positive terminal signal.
 * TIMx_P2_N - Specifies output GPIO that is used by timer for stepper motor
 *             phase 2 negative terminal signal.
 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_stepper_initialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level stepper motor driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use. The number of valid timer
 *     IDs varies with the STM32 MCU.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half stepper driver is returned.
 *   NULL is returned on any failure.
 *
 ************************************************************************************/

EXTERN FAR struct stepper_lowerhalf_s *stm32_stepper_initialize(int timer);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32_STM32_STEPPER_H */
