/****************************************************************************
 * arch/arm/src/stm32/stm32_stepper.c
 *
 *   Copyright (C) 2015-2017 Haltian Ltd. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/motor/stepper.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "stm32_stepper.h"
#include "stm32.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing stepper control */

#ifndef CONFIG_DEBUG
#  undef CONFIG_STEPPER_DEBUG
#endif

#ifdef CONFIG_STEPPER_DEBUG
#  define stepperdbg              dbg
#  define stepperlldbg            lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define steppervdbg           vdbg
#    define stepperllvdbg         llvdbg
#    define stepper_dumpgpio(p,m) stm32_dumpgpio(p,m)
#  else
#    define steppervdbg(x...)
#    define stepperllvdbg(x...)
#    define stepper_dumpgpio(p,m)
#  endif
#else
#  define stepperdbg(x...)
#  define stepperlldbg(x...)
#  define steppervdbg(x...)
#  define stepperllvdbg(x...)
#  define stepper_dumpgpio(p,m)
#endif

#define MAX_STEP_INDEX  3

#define STEP_INDEX_CW \
  (putreg32((getreg32(priv->pattern_bkreg) + 1) % \
  (MAX_STEP_INDEX + 1), priv->pattern_bkreg))

#define STEP_INDEX_CCW \
  { \
    uint32_t __regval = getreg32(priv->pattern_bkreg); \
    putreg32(__regval == 0? MAX_STEP_INDEX: __regval - 1, priv->pattern_bkreg); \
  }

enum drive_direction_e
{
  DIR_CW = 0,
  DIR_CCW
};

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the state of one motor control channel */

struct stm32_stepper_s
{
  FAR const struct stepper_ops_s *ops;  /* Motor control operations */
  uint8_t timid;                        /* Timer ID */
  uint8_t irq;                          /* Timer update IRQ */
  enum drive_direction_e direction;     /* Drive direction */
  uint8_t gpio_port;                    /* GPIO port number */
  uint16_t min_freq;                    /* Minimum step pattern frequency */
  uint16_t max_freq;                    /* Maximum step pattern frequency */
  uint32_t pattern_bkreg;               /* Pattern index backup register */
  uint32_t position_bkreg;              /* Position index backup register */
  uint32_t step[MAX_STEP_INDEX + 1];    /* Step pattern GPIO BSRR config */
  uint32_t step_break;                  /* Break step pattern */
  uint32_t step_standby;                /* Standby step pattern */
  uint32_t frequency;                   /* Stepping frequency */
  uint32_t tim_base;                    /* The base address of the timer */
  uint32_t pclk;                        /* The frequency of the peripheral clock */
  volatile int32_t target_pos;          /* Target position index */
  FAR void *handle;                     /* Handle used for upper-half callback */
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/
/* Register access */

static uint16_t stepper_getreg(struct stm32_stepper_s *priv, int offset);
static void stepper_putreg(struct stm32_stepper_s *priv, int offset, uint16_t value);

#if defined(CONFIG_STEPPER_DEBUG) && defined(CONFIG_DEBUG_VERBOSE)
static void stepper_dumpregs(struct stm32_stepper_s *priv, FAR const char *msg);
#else
#  define stepper_dumpregs(priv,msg)
#endif

/* Timer management */

static int stepper_timer(FAR struct stm32_stepper_s *priv,
                         FAR const struct stepper_info_s *info);
static int stepper_interrupt(struct stm32_stepper_s *priv);
static int stepper_tim2interrupt(int irq, void *context);
static int stepper_tim3interrupt(int irq, void *context);
static int stepper_tim4interrupt(int irq, void *context);
static int stepper_tim5interrupt(int irq, void *context);

/* Stepper driver methods */

static int stepper_setup(FAR struct stepper_lowerhalf_s *dev);

static int stepper_shutdown(FAR struct stepper_lowerhalf_s *dev);

static int stepper_set_pos(FAR struct stepper_lowerhalf_s *dev,
                           FAR const struct stepper_info_s *info,
                           FAR void *handle);

static int stepper_get_status(FAR struct stepper_lowerhalf_s *dev,
                              FAR struct stepper_status_s *status);

static int stepper_reset_index(FAR struct stepper_lowerhalf_s *dev);

static int stepper_set_pattern_index(FAR struct stepper_lowerhalf_s *dev,
                                     unsigned int index);

static int stepper_stop(FAR struct stepper_lowerhalf_s *dev);

static int stepper_ioctl(FAR struct stepper_lowerhalf_s *dev,
                         int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half stepper motor control methods used by
   the upper half driver */

static const struct stepper_ops_s g_stepperops =
{
  .setup             = stepper_setup,
  .shutdown          = stepper_shutdown,
  .set_pos           = stepper_set_pos,
  .get_status        = stepper_get_status,
  .reset_index       = stepper_reset_index,
  .set_pattern_index = stepper_set_pattern_index,
  .stop              = stepper_stop,
  .ioctl             = stepper_ioctl,
};

#ifdef CONFIG_STM32_TIM2_STEPPER
static struct stm32_stepper_s g_stepper2dev =
{
  .ops        = &g_stepperops,

  .min_freq   = TIM2_STEPPER_MIN_FREQ,
  .max_freq   = TIM2_STEPPER_MAX_FREQ,

  .step       = {
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM2_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM2_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM2_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK &TIM2_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM2_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM2_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM2_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK &TIM2_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM2_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM2_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM2_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM2_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM2_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM2_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM2_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM2_GPIO_P2_N) >> GPIO_PIN_SHIFT ),
                },

  .step_break = GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM2_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM2_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM2_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM2_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

  .step_standby =
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM2_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM2_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM2_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM2_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

  .gpio_port  = (GPIO_PORT_MASK & TIM2_GPIO_P1_P) >> GPIO_PORT_SHIFT,

  .timid      = 2,
  .irq        = STM32_IRQ_TIM2,
  .tim_base   = STM32_TIM2_BASE,
  .pclk       = STM32_APB1_TIM2_CLKIN,

  .pattern_bkreg = TIM2_PATTERN_BKREG,
  .position_bkreg = TIM2_POSITION_BKREG,
};
#endif

#ifdef CONFIG_STM32_TIM3_STEPPER
static struct stm32_stepper_s g_stepper3dev =
{
  .ops        = &g_stepperops,

  .min_freq   = TIM3_STEPPER_MIN_FREQ,
  .max_freq   = TIM3_STEPPER_MAX_FREQ,

  .step       = {
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM3_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM3_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM3_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK &TIM3_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM3_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM3_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM3_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK &TIM3_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM3_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM3_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM3_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM3_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM3_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM3_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM3_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM3_GPIO_P2_N) >> GPIO_PIN_SHIFT ),
                },

  .step_break = GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM3_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM3_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM3_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM3_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

  .step_standby =
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM3_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM3_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM3_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM3_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

  .gpio_port =  (GPIO_PORT_MASK & TIM3_GPIO_P1_P) >> GPIO_PORT_SHIFT,

  .timid      = 3,
  .irq        = STM32_IRQ_TIM3,
  .tim_base   = STM32_TIM3_BASE,
  .pclk       = STM32_APB1_TIM3_CLKIN,

  .pattern_bkreg = TIM3_PATTERN_BKREG,
  .position_bkreg = TIM3_POSITION_BKREG,
};
#endif

#ifdef CONFIG_STM32_TIM4_STEPPER
static struct stm32_stepper_s g_stepper4dev =
{
  .ops        =  &g_stepperops,

  .min_freq   = TIM4_STEPPER_MIN_FREQ,
  .max_freq   = TIM4_STEPPER_MAX_FREQ,

  .step       = {
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM4_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM4_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM4_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK &TIM4_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM4_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM4_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM4_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK &TIM4_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM4_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM4_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM4_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM4_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM4_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM4_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM4_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM4_GPIO_P2_N) >> GPIO_PIN_SHIFT ),
                },

  .step_break = GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM4_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM4_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM4_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM4_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

  .step_standby =
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM4_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM4_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM4_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM4_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

  .gpio_port =  (GPIO_PORT_MASK & TIM4_GPIO_P1_P) >> GPIO_PORT_SHIFT,

  .timid     =  4,
  .irq       =  STM32_IRQ_TIM4,
  .tim_base  =  STM32_TIM4_BASE,
  .pclk      =  STM32_APB1_TIM4_CLKIN,

  .pattern_bkreg = TIM4_PATTERN_BKREG,
  .position_bkreg = TIM4_POSITION_BKREG,
};
#endif

#ifdef CONFIG_STM32_TIM5_STEPPER
static struct stm32_stepper_s g_stepper5dev =
{
  .ops        = &g_stepperops,

  .min_freq   = TIM5_STEPPER_MIN_FREQ,
  .max_freq   = TIM5_STEPPER_MAX_FREQ,

  .step       = {
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM5_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM5_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM5_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK &TIM5_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM5_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM5_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM5_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK &TIM5_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM5_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM5_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM5_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM5_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM5_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM5_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM5_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM5_GPIO_P2_N) >> GPIO_PIN_SHIFT ),
                },

  .step_break = GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM5_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM5_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM5_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_SET( (GPIO_PIN_MASK & TIM5_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

  .step_standby =
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM5_GPIO_P1_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM5_GPIO_P1_N) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM5_GPIO_P2_P) >> GPIO_PIN_SHIFT ) |
                GPIO_BSRR_RESET( (GPIO_PIN_MASK & TIM5_GPIO_P2_N) >> GPIO_PIN_SHIFT ),

  .gpio_port  = (GPIO_PORT_MASK & TIM5_GPIO_P1_P) >> GPIO_PORT_SHIFT,

  .timid      = 5,
  .irq        = STM32_IRQ_TIM5,
  .tim_base   = STM32_TIM5_BASE,
  .pclk       = STM32_APB1_TIM5_CLKIN,

  .pattern_bkreg = TIM5_PATTERN_BKREG,
  .position_bkreg = TIM5_POSITION_BKREG,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stepper_getreg
 *
 * Description:
 *   Read the value of an 16-bit timer register.
 *
 * Input Parameters:
 *   priv - reference to the stepper status
 *   offset - offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint16_t stepper_getreg(struct stm32_stepper_s *priv, int offset)
{
  return getreg16(priv->tim_base + offset);
}

/****************************************************************************
 * Name: stepper_getreg32
 *
 * Description:
 *   Read the value of an 32-bit timer register.
 *
 * Input Parameters:
 *   priv - reference to the stepper status
 *   offset - offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t stepper_getreg32(struct stm32_stepper_s *priv, int offset)
{
  return getreg32(priv->tim_base + offset);
}

/****************************************************************************
 * Name: stepper_putreg
 *
 * Description:
 *   Write the value for an timer register.
 *
 * Input Parameters:
 *   priv - reference to the stepper status
 *   offset - offset to the register to read
 *   value - value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stepper_putreg(struct stm32_stepper_s *priv, int offset, uint16_t value)
{
  putreg16(value, priv->tim_base + offset);
}

/****************************************************************************
 * Name: stepper_putreg32
 *
 * Description:
 *   Write the value for an 32-bit timer register.
 *
 * Input Parameters:
 *   priv - reference to the stepper status
 *   offset - offset to the register to read
 *   value - value
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stepper_putreg32(struct stm32_stepper_s *priv, int offset, uint32_t value)
{
  putreg32(value, priv->tim_base + offset);
}

/****************************************************************************
 * Name: stepper_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input parameters:
 *   priv - reference to the stepper status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_STEPPER_DEBUG) && defined(CONFIG_DEBUG_VERBOSE)
static void stepper_dumpregs(struct stm32_stepper_s *priv, FAR const char *msg)
{
  steppervdbg("%s: (id:%d, base:0x%04X)\n", msg, priv->timid, priv->tim_base);
  steppervdbg("CR1:%04x CR2:%04x SMCR:%04x DIER:%04x\n",
          stepper_getreg(priv, STM32_GTIM_CR1_OFFSET),
          stepper_getreg(priv, STM32_GTIM_CR2_OFFSET),
          stepper_getreg(priv, STM32_GTIM_SMCR_OFFSET),
          stepper_getreg(priv, STM32_GTIM_DIER_OFFSET));
  steppervdbg("SR:%04x EGR:%04x CCMR1:%04x CCMR2:%04x\n",
          stepper_getreg(priv, STM32_GTIM_SR_OFFSET),
          stepper_getreg(priv, STM32_GTIM_EGR_OFFSET),
          stepper_getreg(priv, STM32_GTIM_CCMR1_OFFSET),
          stepper_getreg(priv, STM32_GTIM_CCMR2_OFFSET));
  steppervdbg("CCER:%04x CNT:%08x PSC:%04x ARR:%04x\n",
          stepper_getreg(priv, STM32_GTIM_CCER_OFFSET),
          stepper_getreg32(priv, STM32_GTIM_CNT_OFFSET),
          stepper_getreg(priv, STM32_GTIM_PSC_OFFSET),
          stepper_getreg(priv, STM32_GTIM_ARR_OFFSET));
  steppervdbg("CCR1:%08x CCR2:%08x CCR3:%08x CCR4:%08x\n",
          stepper_getreg32(priv, STM32_GTIM_CCR1_OFFSET),
          stepper_getreg32(priv, STM32_GTIM_CCR2_OFFSET),
          stepper_getreg32(priv, STM32_GTIM_CCR3_OFFSET),
          stepper_getreg32(priv, STM32_GTIM_CCR4_OFFSET));
  steppervdbg("DCR:%04x DMAR:%04x\n",
          stepper_getreg(priv, STM32_GTIM_DCR_OFFSET),
          stepper_getreg(priv, STM32_GTIM_DMAR_OFFSET));
}
#endif

/****************************************************************************
 * Name: stepper_setup_frequency
 *
 * Description:
 *   Setup timer registers for desired step pattern frequency
 *
 * Input parameters:
 *   priv      - reference to the stepper status
 *   frequency - step pattern frequency
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void stepper_setup_frequency(FAR struct stm32_stepper_s *priv,
                                    uint32_t frequency)
{
  uint32_t prescaler;
  uint32_t timclk;
  uint32_t reload;

  /* Calculate optimal values for the timer prescaler and for the timer
   * reload register. */

  prescaler = (priv->pclk / frequency + 65534) / 65535;
  if (prescaler < 1)
    {
      prescaler = 1;
    }
  else if (prescaler > 65536)
    {
      prescaler = 65536;
    }

  timclk = priv->pclk / prescaler;

  /* Reload value defines step pattern frequency. */

  reload = timclk / frequency;
  if (reload < 1)
    {
      reload = 1;
    }
  else if (reload > 65535)
    {
      reload = 65535;
    }

  priv->frequency = frequency;

  steppervdbg("TIM%d PCLK:%d, freq:%d, TIMCLK:%d, prescaler:%d, reload:%d\n",
    priv->timid, priv->pclk, frequency, timclk, prescaler, reload);

  /* Set the reload and prescaler values */

  stepper_putreg32(priv, STM32_GTIM_ARR_OFFSET, reload);
  stepper_putreg(priv, STM32_GTIM_PSC_OFFSET, (uint16_t)(prescaler - 1));

  /* Generate an update event to reload the prescaler (all timers) */

  stepper_putreg(priv, STM32_GTIM_EGR_OFFSET, GTIM_EGR_UG);
}

/****************************************************************************
 * Name: stepper_timer
 *
 * Description:
 *   Initialize the timer resources and start step pattern output
 *
 * Input parameters:
 *   priv      - lower half stepper state
 *   frequency - step pattern frequency
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int stepper_timer(FAR struct stm32_stepper_s *priv,
                         FAR const struct stepper_info_s *info)
{
  /* Register contents */

  uint16_t cr1;
  uint16_t ccer;

  DEBUGASSERT(priv != NULL);

  steppervdbg("TIM%d, freq:%d\n", priv->timid, info->frequency);

  /* Disable all interrupts and clear all pending status */

  stepper_putreg(priv, STM32_GTIM_DIER_OFFSET, 0);
  stepper_putreg(priv, STM32_GTIM_SR_OFFSET, 0);

  cr1 = stepper_getreg(priv, STM32_GTIM_CR1_OFFSET);

  /* Disable the timer until we get it configured */

  cr1 &= ~(GTIM_CR1_CEN + GTIM_CR1_CKD_MASK);
  cr1 |= GTIM_CR1_URS;
  stepper_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

  /* Setup step pattern frequency */

  stepper_setup_frequency(priv, info->frequency);

  /* Disable channel outputs */

  ccer = stepper_getreg(priv, STM32_GTIM_CCER_OFFSET);
  ccer &= ~(GTIM_CCER_CC1E | GTIM_CCER_CC2E | GTIM_CCER_CC3E | GTIM_CCER_CC4E);

  stepper_putreg(priv, STM32_GTIM_CCER_OFFSET, ccer);

  /* Set the ARR Preload Bit */

  cr1 = stepper_getreg(priv, STM32_GTIM_CR1_OFFSET);
  cr1 |= GTIM_CR1_ARPE;
  stepper_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

  steppervdbg("enable irq:%u\n", priv->irq);
  stepper_putreg(priv, STM32_GTIM_SR_OFFSET, 0);
  stepper_putreg(priv, STM32_GTIM_DIER_OFFSET, GTIM_DIER_UIE);

  /* Start timer in one pulse mode. */

  cr1 |= GTIM_CR1_CEN + GTIM_CR1_OPM;
  stepper_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

  up_enable_irq(priv->irq);

  return OK;
}

/****************************************************************************
 * Name: stepper_update
 *
 * Description:
 *   Reconfigure parameters for timer and step pattern
 *
 * Input parameters:
 *   priv      - lower half stepper state
 *   frequency - step pattern frequency
 *
 * Returned Value:
 *   Zero on success
 *
 ****************************************************************************/
static int stepper_update(FAR struct stm32_stepper_s *priv,
                          FAR const struct stepper_info_s *info)
{
  uint16_t cr1;
  uint32_t frequency;
  float    duration;

  steppervdbg("->\n");

  if ((info->pos_type == POS_ABSOLUTE) && (priv->target_pos ==
    info->position))
    {
      /* Target already set to requested position */

      return OK;
    }

  /* Disable interrupts and timer until we get it reconfigured */

  stepper_putreg(priv, STM32_GTIM_DIER_OFFSET, 0);

  cr1 = stepper_getreg(priv, STM32_GTIM_CR1_OFFSET);
  cr1 &= ~GTIM_CR1_CEN;
  stepper_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

  if (info->pos_type == POS_ABSOLUTE)
    {
      duration = abs(info->position - getreg32(priv->position_bkreg)) /
        (float)info->frequency;
      priv->target_pos = info->position;
    }
  else
    {
      /* Relative position */

      duration = abs(info->position) / (float)info->frequency;
      priv->target_pos += info->position;
    }

  priv->direction = priv->target_pos > getreg32(priv->position_bkreg)?
    DIR_CW: DIR_CCW;

  if (duration > 0)
    {
      /* Calculate new frequency based on step pattern duration and
       * remaining steps.
       */

      frequency = (uint32_t)(abs(priv->target_pos - getreg32(
        priv->position_bkreg)) / duration);

      if (frequency < priv->min_freq)
        {
          frequency = priv->min_freq;
        }
      else if (frequency > priv->max_freq)
        {
          frequency = priv->max_freq;
        }

      /* Update step pattern frequency */

      stepper_setup_frequency(priv, frequency);
    }
  else
    {
      frequency = info->frequency;
    }

  /* Enable timer and interrupts */

  cr1 |= GTIM_CR1_CEN + GTIM_CR1_OPM;
  stepper_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);
  stepper_putreg(priv, STM32_GTIM_DIER_OFFSET, GTIM_DIER_UIE);

  steppervdbg("freq %u->%u\n", info->frequency, frequency);

  return OK;
}

/****************************************************************************
 * Name: stepper_interrupt
 *
 * Description:
 *   Handle timer interrupts.
 *
 * Input parameters:
 *   priv - lower half stepper state
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int stepper_interrupt(struct stm32_stepper_s *priv)
{
  if (getreg32(priv->position_bkreg) != priv->target_pos)
    {
      uint16_t cr1;
      uint32_t regval = stepper_getreg(priv, STM32_GTIM_SR_OFFSET);

      if (regval & GTIM_SR_UIF)
        {
          DEBUGASSERT((getreg32(STM32_PWR_CR) & PWR_CR_DBP));

          /* Update step and position indexes based on drive direction */

          if (priv->direction == DIR_CW)
            {
              putreg32(getreg32(priv->position_bkreg) + 1, priv->position_bkreg);
              STEP_INDEX_CW;
            }
          else
            {
              putreg32(getreg32(priv->position_bkreg) - 1, priv->position_bkreg);
              STEP_INDEX_CCW;
            }

          putreg32(priv->step[getreg32(priv->pattern_bkreg)],
            g_gpiobase[priv->gpio_port] + STM32_GPIO_BSRR_OFFSET);

          /* Clear the update interrupt bit */

          stepper_putreg(priv, STM32_GTIM_SR_OFFSET, regval & ~GTIM_SR_UIF);
        }

      /* Restart timer in one pulse mode */

      cr1 = stepper_getreg(priv, STM32_GTIM_CR1_OFFSET);
      cr1 |= (GTIM_CR1_CEN + GTIM_CR1_OPM);
      stepper_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);
    }
  else
    {
      /* No steps left. Disable further interrupts, set output to standby and
         perform callback. */

      (void)stepper_stop((FAR struct stepper_lowerhalf_s *)priv);
      stepperllvdbg("No steps left\n");
      stepper_expired(priv->handle);
      priv->handle = NULL;
      stm32_pwr_enablebkp(false);
    }

  return OK;
}

/****************************************************************************
 * Name: stepper_timxinterrupt
 *
 * Description:
 *   Handle timer 2 - 5 interrupts.
 *
 * Input parameters:
 *   Standard NuttX interrupt inputs
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_TIM2_STEPPER
static int stepper_tim2interrupt(int irq, void *context)
{
  return stepper_interrupt(&g_stepper2dev);
}
#endif

#ifdef CONFIG_STM32_TIM3_STEPPER
static int stepper_tim3interrupt(int irq, void *context)
{
  return stepper_interrupt(&g_stepper3dev);
}
#endif

#ifdef CONFIG_STM32_TIM4_STEPPER
static int stepper_tim4interrupt(int irq, void *context)
{
  return stepper_interrupt(&g_stepper4dev);
}
#endif

#ifdef CONFIG_STM32_TIM5_STEPPER
static int stepper_tim5interrupt(int irq, void *context)
{
  return stepper_interrupt(&g_stepper5dev);
}
#endif

/****************************************************************************
 * Name: stepper_set_apb_clock
 *
 * Description:
 *   Enable or disable APB clock for the timer peripheral
 *
 * Input parameters:
 *   dev - lower half stepper state
 *   on  - Enable clock if 'on' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static void stepper_set_apb_clock(FAR struct stm32_stepper_s *priv, bool on)
{
  uint32_t en_bit;
  uint32_t regaddr;

  /* Determine which timer to configure */

  switch (priv->timid)
    {
#ifdef CONFIG_STM32_TIM2_STEPPER
      case 2:
        regaddr  = STM32_RCC_APB1ENR;
        en_bit   = RCC_APB1ENR_TIM2EN;
        break;
#endif
#ifdef CONFIG_STM32_TIM3_STEPPER
      case 3:
        regaddr  = STM32_RCC_APB1ENR;
        en_bit   = RCC_APB1ENR_TIM3EN;
        break;
#endif
#ifdef CONFIG_STM32_TIM4_STEPPER
      case 4:
        regaddr  = STM32_RCC_APB1ENR;
        en_bit   = RCC_APB1ENR_TIM4EN;
        break;
#endif
#ifdef CONFIG_STM32_TIM5_STEPPER
      case 5:
        regaddr  = STM32_RCC_APB1ENR;
        en_bit   = RCC_APB1ENR_TIM5EN;
        break;
#endif
      default:
        assert(false);
        break;
    }

  /* Enable/disable APB 1/2 clock for timer */

  if (on)
    {
      modifyreg32(regaddr, 0, en_bit);
    }
  else
    {
      modifyreg32(regaddr, en_bit, 0);
    }
}

/****************************************************************************
 * Name: stepper_setup
 *
 * Description:
 *   This method is called when the driver is opened. The lower half driver
 *   configures and initializes the device so that it is ready for use.
 *
 * Input parameters:
 *   dev - the lower half motor control driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int stepper_setup(FAR struct stepper_lowerhalf_s *dev)
{
  FAR struct stm32_stepper_s *priv = (FAR struct stm32_stepper_s *)dev;

  steppervdbg("TIM%d\n", priv->timid);
  stepper_dumpregs(priv, "Initially");

  /* Enable APB1/2 clocking for timer. */

  stepper_set_apb_clock(priv, true);

  steppervdbg("step[0] BSRR = 0x%08X\n", priv->step[0]);
  steppervdbg("step[1] BSRR = 0x%08X\n", priv->step[1]);
  steppervdbg("step[2] BSRR = 0x%08X\n", priv->step[2]);
  steppervdbg("step[3] BSRR = 0x%08X\n", priv->step[3]);
  steppervdbg("break   BSRR = 0x%08X\n", priv->step_break);
  steppervdbg("standby BSRR = 0x%08X\n", priv->step_standby);
  stm32_pwr_enablebkp(true);
  steppervdbg("position_bkreg: %d\n", getreg32(priv->position_bkreg));
  steppervdbg("pattern_bkreg: %d\n", getreg32(priv->pattern_bkreg));
  stm32_pwr_enablebkp(false);

  return OK;
}

/****************************************************************************
 * Name: stepper_shutdown
 *
 * Description:
 *   This method is called when the driver is closed. The lower half driver
 *   stops step pattern output, free any resources and disable the timer
 *   hardware.
 *
 * Input parameters:
 *   dev - the lower half motor control driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int stepper_shutdown(FAR struct stepper_lowerhalf_s *dev)
{
  FAR struct stm32_stepper_s *priv = (FAR struct stm32_stepper_s *)dev;
  int32_t current_pos;

  steppervdbg("TIM%d\n", priv->timid);

  current_pos = getreg32(priv->position_bkreg);

  /* Make sure that the output has been stopped */

  stepper_stop(dev);

  if (current_pos != priv->target_pos)
    {
      stepperdbg("shutdown while stepping %d->%d\n", current_pos,
        priv->target_pos);
    }

  /* Disable APB1/2 clocking for timer. */

  stepper_set_apb_clock(priv, false);

  return OK;
}

/****************************************************************************
 * Name: stepper_set_pos
 *
 * Description:
 *   Drive stepper to requested position index
 *
 * Input parameters:
 *   dev - the lower half stepper state
 *   info - step pattern characteristics
 *   handle - the upper-half expiration callback function
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int stepper_set_pos(FAR struct stepper_lowerhalf_s *dev,
                           FAR const struct stepper_info_s *info,
                           FAR void *handle)
{
  FAR struct stm32_stepper_s *priv = (FAR struct stm32_stepper_s *)dev;

  steppervdbg("TIM%d pos:%d (%s), freq:%d\n", priv->timid, info->position,
    info->pos_type == POS_ABSOLUTE? "abs" : "rel", info->frequency);

  if (info->frequency < priv->min_freq || info->frequency > priv->max_freq)
    {
      stepperdbg("invalid frequency\n");
      return -EINVAL;
    }

  if (info->pos_type == POS_ABSOLUTE && (getreg32(priv->position_bkreg) ==
      info->position))
    {
      stepperdbg("current position index already set to %d\n", info->position);
      return -EINVAL;
    }

  if (info->pos_type == POS_RELATIVE && info->position == 0)
    {
      stepperdbg("Invalid relative step count: %d\n", info->position);
      return -EINVAL;
    }

  if (stepper_getreg(priv, STM32_GTIM_CR1_OFFSET) & GTIM_CR1_CEN)
    {
      /* Previous step pattern is still active, set new target position index
       * and update parameters.
       */

      return stepper_update(priv, info);
    }
  else
    {
      /* Previous state was standby, drive pre-excitation pulse as initial
       * state for step pattern (the last state of previous step sequence)
       */

      stm32_pwr_enablebkp(true);

      priv->handle = handle;

      /* Do step */

      putreg32(priv->step[getreg32(priv->pattern_bkreg)],
        g_gpiobase[priv->gpio_port] + STM32_GPIO_BSRR_OFFSET);

      /* Setup target position index and drive direction */

      if (info->pos_type == POS_ABSOLUTE)
        {
          int32_t pos = getreg32(priv->position_bkreg);
          stepperdbg("pos:%u\n", pos);
          priv->target_pos = info->position;
          priv->direction = info->position > pos?
            DIR_CW: DIR_CCW;
        }
      else
        {
          /* Relative position parameter */

          priv->target_pos += info->position;
          priv->direction = info->position > 0? DIR_CW: DIR_CCW;
        }

      return stepper_timer(priv, info);
    }
}

/****************************************************************************
 * Name: stepper_get_status
 *
 * Description:
 *   Get stepper status information
 *
 * Input parameters:
 *   dev    - the lower half stepper state
 *   status - stepper status
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int stepper_get_status(FAR struct stepper_lowerhalf_s *dev,
                              FAR struct stepper_status_s *status)
{
  FAR struct stm32_stepper_s *priv = (FAR struct stm32_stepper_s *)dev;

  DEBUGASSERT(status != NULL);
  if (!status)
    {
      return -EINVAL;
    }

  stm32_pwr_enablebkp(true);

  steppervdbg("TIM%d current_pos:%d, target_pos:%d, freq:%u, index:%u\n",
    priv->timid, getreg32(priv->position_bkreg), priv->target_pos,
    priv->frequency, getreg32(priv->pattern_bkreg));

  status->current_pos = getreg32(priv->position_bkreg);
  status->target_pos = priv->target_pos;
  status->frequency = priv->frequency;
  status->step_index = getreg32(priv->pattern_bkreg);

  stm32_pwr_enablebkp(false);

  return OK;
}

/****************************************************************************
 * Name: stepper_reset_index
 *
 * Description:
 *   Reset index to current position
 *
 * Input parameters:
 *   dev - the lower half stepper state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int stepper_reset_index(FAR struct stepper_lowerhalf_s *dev)
{
  FAR struct stm32_stepper_s *priv = (FAR struct stm32_stepper_s *)dev;
  irqstate_t flags;

  stepperllvdbg("TIM%d\n", priv->timid);

  flags = irqsave();
  stm32_pwr_enablebkp(true);

  putreg32(0, priv->position_bkreg);
  priv->target_pos = 0;

  stm32_pwr_enablebkp(false);
  irqrestore(flags);

  return OK;
}

/****************************************************************************
 * Name: stepper_set_pattern_index
 *
 * Description:
 *   Set step pattern index
 *
 * Input parameters:
 *   dev - the lower half stepper state structure
 *   index - stepper pattern index
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int stepper_set_pattern_index(FAR struct stepper_lowerhalf_s *dev,
                                     unsigned int index)
{
  FAR struct stm32_stepper_s *priv = (FAR struct stm32_stepper_s *)dev;
  irqstate_t flags;
  int ret = OK;

  steppervdbg("TIM%d\n", priv->timid);

  flags = irqsave();

  if (index > MAX_STEP_INDEX)
    {
      stepperdbg("value %u out of range\n", index);
      ret = -EINVAL;
      goto err_out;
    }

  stm32_pwr_enablebkp(true);
  putreg32(index, priv->pattern_bkreg);
  stm32_pwr_enablebkp(false);

err_out:
  irqrestore(flags);
  return ret;
}

/****************************************************************************
 * Name: stepper_stop
 *
 * Description:
 *   Stop step pattern output and reset the timer resources
 *
 * Input parameters:
 *   dev - the lower half stepper state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   This function is called to stop the step pattern output. This method is
 *   also called from the timer interrupt handler.
 *
 ****************************************************************************/

static int stepper_stop(FAR struct stepper_lowerhalf_s *dev)
{
  FAR struct stm32_stepper_s *priv = (FAR struct stm32_stepper_s *)dev;
  uint32_t resetbit;
  uint32_t regaddr;
  uint32_t regval;
  irqstate_t flags;

  stepperllvdbg("TIM%d\n", priv->timid);

  /* Disable interrupts momentary to stop any ongoing timer processing and
   * to prevent any concurrent access to the reset register.
   */

  flags = irqsave();

  /* Disable further interrupts and stop the timer */

  stepper_putreg(priv, STM32_GTIM_DIER_OFFSET, 0);
  stepper_putreg(priv, STM32_GTIM_SR_OFFSET, 0);
  stepper_putreg(priv, STM32_GTIM_CR1_OFFSET, 0);

  /* Determine which timer to reset */

  switch (priv->timid)
    {
#ifdef CONFIG_STM32_TIM2_STEPPER
      case 2:
        regaddr  = STM32_RCC_APB1RSTR;
        resetbit = RCC_APB1RSTR_TIM2RST;
        break;
#endif
#ifdef CONFIG_STM32_TIM3_STEPPER
      case 3:
        regaddr  = STM32_RCC_APB1RSTR;
        resetbit = RCC_APB1RSTR_TIM3RST;
        break;
#endif
#ifdef CONFIG_STM32_TIM4_STEPPER
      case 4:
        regaddr  = STM32_RCC_APB1RSTR;
        resetbit = RCC_APB1RSTR_TIM4RST;
        break;
#endif
#ifdef CONFIG_STM32_TIM5_STEPPER
      case 5:
        regaddr  = STM32_RCC_APB1RSTR;
        resetbit = RCC_APB1RSTR_TIM5RST;
        break;
#endif
      default:
        assert(false);
        break;
    }

  /* Reset the timer */

  regval  = getreg32(regaddr);
  regval |= resetbit;
  putreg32(regval, regaddr);

  regval &= ~resetbit;
  putreg32(regval, regaddr);
  irqrestore(flags);

  /* Disable output */

  putreg32(priv->step_break, g_gpiobase[priv->gpio_port] + STM32_GPIO_BSRR_OFFSET);

  stepperllvdbg("regaddr: %08x resetbit: %08x\n", regaddr, resetbit);

  return OK;
}

/****************************************************************************
 * Name: stepper_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input parameters:
 *   dev - the lower half stepper state structure
 *   cmd - ioctl command
 *   arg - ioctl command argument
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int stepper_ioctl(FAR struct stepper_lowerhalf_s *dev, int cmd,
                         unsigned long arg)
{
#ifdef CONFIG_STEPPER_DEBUG
  FAR struct stm32_stepper_s *priv = (FAR struct stm32_stepper_s *)dev;

  /* There are no platform-specific ioctl commands */

  stepperdbg("TIM%d\n", priv->timid);
#endif
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_stepper_initialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level motor control driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer. The number of valid timer IDs
 *   varies with the STM32 MCU.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half motor control driver is
 *   returned. NULL is returned on any failure.
 *
 ****************************************************************************/

FAR struct stepper_lowerhalf_s *stm32_stepper_initialize(int timer)
{
  FAR struct stm32_stepper_s *lower;

  steppervdbg("stm32_stepper_initialize -> TIM%d\n", timer);

  switch (timer)
    {
#ifdef CONFIG_STM32_TIM2_STEPPER
      case 2:
        lower = &g_stepper2dev;
        irq_attach(lower->irq, stepper_tim2interrupt);
        up_disable_irq(lower->irq);
        break;
#endif
#ifdef CONFIG_STM32_TIM3_STEPPER
      case 3:
        lower = &g_stepper3dev;
        irq_attach(lower->irq, stepper_tim3interrupt);
        up_disable_irq(lower->irq);
        break;
#endif
#ifdef CONFIG_STM32_TIM4_STEPPER
      case 4:
        lower = &g_stepper4dev;
        irq_attach(lower->irq, stepper_tim4interrupt);
        up_disable_irq(lower->irq);
        break;
#endif
#ifdef CONFIG_STM32_TIM5_STEPPER
      case 5:
        lower = &g_stepper5dev;
        irq_attach(lower->irq, stepper_tim5interrupt);
        up_disable_irq(lower->irq);
        break;
#endif
      default:
        stepperdbg("No such timer (%d) configured\n", timer);
        return NULL;
    }

  return (FAR struct stepper_lowerhalf_s *)lower;
}
