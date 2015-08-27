/************************************************************************************
 * configs/cloudctrl/src/up_adc.c
 * arch/arm/src/board/up_adc.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Darcy Gong <darcy.gong@gmail.com>
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/analog/adc.h>
#include <arch/board/board.h>
#include <arch/board/board-adc.h>
#include <arch/board/board-pwrctl.h>
#include <fcntl.h>
#include <unistd.h>

#include "chip.h"
#include "up_arch.h"

#include "stm32.h"
#include "stm32_pwm.h"

#include "haltian-tsone.h"
#include "up_adc.h"

#ifdef CONFIG_ADC
/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Up to 3 ADC interfaces are supported */

#if STM32_NADC < 3
#  undef CONFIG_STM32_ADC3
#endif

#if STM32_NADC < 2
#  undef CONFIG_STM32_ADC2
#endif

#if STM32_NADC < 1
#  undef CONFIG_STM32_ADC1
#endif

#if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2) || defined(CONFIG_STM32_ADC3)
#ifndef CONFIG_STM32_ADC1
#  warning "Channel information only available for ADC1"
#endif

/* The number of ADC channels in the conversion list */

#define ADC1_NCHANNELS 1

/************************************************************************************
 * Private Data
 ************************************************************************************/

static struct adc_dev_s *g_adc;

/* Identifying number of each ADC channel. The only internal signal for ADC testing
 * is the potentiometer input:
 *
 *   ADC1_IN10(PC0) Potentiometer
 *
 * External signals are also available on CON5 CN14:
 *
 *  ADC_IN8 (PB0) CON5 CN14 Pin2
 *  ADC_IN9 (PB1) CON5 CN14 Pin1
 */

#ifdef CONFIG_STM32_ADC1

static const uint8_t  g_chanlist[ADC1_NCHANNELS] = {ADC1_MEASURE_VBAT_CHANNEL};

/* Configurations of pins used by each ADC channel */

static const uint32_t g_pinlist[ADC1_NCHANNELS]  = {GPIO_VBAT_MEASURE_ADC};

#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/



/************************************************************************************
 * Public Functions
 ************************************************************************************/

void board_adc_vbat_setctrl(bool on)
{
  /* Enable/disable VBAT measurement equipment. Control pin is used to disable
   * measurement electronics when not using ADC. This is for reduced power
   * usage.*/

#ifdef CONFIG_STM32_ADC1
  static bool powered = false;

  if (on && !powered)
    {
      powered = true;
      board_pwrctl_get(PWRCTL_SWITCH_ADC_VBAT_MEASUREMENT);
    }
  else if (!on && powered)
    {
      powered = false;
      board_pwrctl_put(PWRCTL_SWITCH_ADC_VBAT_MEASUREMENT);
    }
#else
  (void)on;
#endif
}

/************************************************************************************
 * Name: adc_devinit
 *
 * Description:
 *   All STM32 architectures must provide the following interface to work with
 *   examples/adc.
 *
 ************************************************************************************/

int adc_devinit(void)
{
#ifdef CONFIG_STM32_ADC1
  static bool initialized = false;
  int ret;
  int fd;
  int i;

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* Configure the pins as analog inputs for the selected channels */

      for (i = 0; i < ADC1_NCHANNELS; i++)
        {
          stm32_configgpio(g_pinlist[i]);
        }

      /* Call stm32_adcinitialize() to get an instance of the ADC interface */

      g_adc = stm32_adcinitialize(1, g_chanlist, ADC1_NCHANNELS);
      if (g_adc == NULL)
        {
          alldbg("ERROR: Failed to get ADC interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc0" */

      ret = adc_register("/dev/adc0", g_adc);
      if (ret < 0)
        {
          alldbg("adc_register failed: %d\n", ret);
          return ret;
        }

      fd = open("/dev/adc0", O_RDWR);
      if (fd != ERROR)
       close(fd);

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
#else
  return -ENOSYS;
#endif
}

void adc_set_sample_time(struct adc_sample_time_s *adc_sample_time)
{
	stm32_adcchange_sample_time(g_adc, adc_sample_time);
}

#endif /* CONFIG_STM32_ADC1 || CONFIG_STM32_ADC2 || CONFIG_STM32_ADC3 */
#endif /* CONFIG_ADC */
