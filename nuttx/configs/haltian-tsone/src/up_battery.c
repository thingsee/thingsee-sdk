/************************************************************************************
 * configs/haltian-tsone/src/up_battery.c
 * arch/arm/src/board/up_battery.c
 *
 *   Copyright (C) 2015 Haltian ltd. All rights reserved.
 *   Authors:
 *     Pekka Ervasti <pekka.ervasti@haltian.com>
 *     Juha Niskanen <juha.niskanen@haltian.com>
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
#include <nuttx/random.h>
#include <nuttx/arch.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

#include <arch/board/board-adc.h>
#include <arch/board/board-battery.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

#ifndef MAX
#  define MAX(a,b) a > b ? a : b
#endif

/* Constants for battery capacity measurement */

#define TEMPERATURE_AREAS       3
#define V2P_CURVE_POINTS        6

#define BATTERY_CAPACITY_THINGSEE_ONE   1900

/* Constants for battery voltage measurement */

#define BATTVOLT_R1_KILOOHMS          (1000)
#define BATTVOLT_R2_KILOOHMS          BATTVOLT_R1_KILOOHMS

#define BATTVOLT_VREF_MILLIVOLTS      3000
#define BATTVOLT_ADC_MAX_VALUE        ((1 << 12) - 1)


/************************************************************************************
 * Private Types
 ************************************************************************************/

struct v2p_pair
{
  float voltage;
  uint8_t percentage;
};

struct v2p_curve
{
  int min_temperature;
  struct v2p_pair values[V2P_CURVE_POINTS];
};

/************************************************************************************
 * Private Data
 ************************************************************************************/

static const struct v2p_curve g_v2p_table[TEMPERATURE_AREAS] =
  {
    {  10, { { 3.00, 0 }, { 3.40, 5 }, { 3.60, 10 }, { 3.70, 25 }, { 3.80, 50 }, { 4.15, 100 } } },
    {  -5, { { 3.00, 0 }, { 3.30, 5 }, { 3.40, 10 }, { 3.65, 25 }, { 3.75, 50 }, { 4.10, 100 } } },
    { -20, { { 3.00, 0 }, { 3.10, 5 }, { 3.15, 10 }, { 3.35, 25 }, { 3.50, 50 }, { 3.90, 100 } } },
  };

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static const struct v2p_curve *find_v2p_curve(int temperature)
{
  int i;

  for (i = 0; i < TEMPERATURE_AREAS - 1; i++)
    {
      if (temperature >= g_v2p_table[i].min_temperature)
        {
          break;
        }
    }

  return &g_v2p_table[i];
}

static int find_low_index(const float voltage, const struct v2p_curve * const curve)
{
  int i;

  for (i = 0; i < V2P_CURVE_POINTS - 1; i++)
    {
      if (voltage >= curve->values[i].voltage && voltage <= curve->values[i + 1].voltage)
        {
          break;
        }
    }

  return i;
}

/****************************************************************************
 * Name: adc_to_millivolts
 *
 * Description:
 *   Convert raw ADC value to millivolts
 *
 * Returned Values:
 *   VBAT voltage in millivolts
 *
 ****************************************************************************/

static uint32_t adc_to_millivolts(uint32_t adc)
{
  uint64_t vbat;

  vbat = (uint64_t)adc;
  vbat *= BATTVOLT_R1_KILOOHMS + BATTVOLT_R2_KILOOHMS;
  vbat *= BATTVOLT_VREF_MILLIVOLTS;
  vbat /= BATTVOLT_R2_KILOOHMS * BATTVOLT_ADC_MAX_VALUE;

  return (uint32_t)vbat;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

uint8_t board_get_battery_level(float voltage, int temperature)
{
  const struct v2p_curve *curve;
  float k;
  int low, high;

  curve = find_v2p_curve(temperature);

  if (voltage > curve->values[V2P_CURVE_POINTS - 1].voltage)
    {
      return 100;
    }

  if (voltage < curve->values[0].voltage)
    {
      return 0;
    }

  low = find_low_index(voltage, curve);

  high = low + 1;

  k = (curve->values[high].percentage  - curve->values[low].percentage) /
      (curve->values[high].voltage - curve->values[low].voltage);

  return (uint8_t)(k * (voltage - curve->values[low].voltage) + curve->values[low].percentage);
}

uint16_t board_get_battery_capacity(void)
{
  return BATTERY_CAPACITY_THINGSEE_ONE;
}

int board_get_battery_voltage(uint32_t *voltage)
{
  ssize_t nbytes = 0;
  uint8_t sample[5] = { 0 };
  uint32_t val;

  board_adc_vbat_setctrl(true);

  int fd = open("/dev/adc0", O_RDWR);
  if (fd < 0)
    {
      lldbg("Can't open ADC converter\n");
      board_adc_vbat_setctrl(false);
      return ERROR;
    }

  nbytes = read(fd, sample, sizeof(sample));
  if (nbytes != sizeof(sample))
    {
      lldbg("Can't read samples from ADC converter\n");
      close(fd);
      board_adc_vbat_setctrl(false);
      return ERROR;
    }

  close(fd);
  board_adc_vbat_setctrl(false);

  val = sample[1] | (sample[2] << 8) | (sample[3] << 16) | (sample[4] << 24);

  /* Add the raw value to entropy pool. */

  add_sensor_randomness(val);

  /* Convert it to actual millivolts. */

  val = adc_to_millivolts(val);

  *voltage = val;

  return OK;
}

int board_get_battery_average_voltage(uint32_t *voltage)
{
  int ret;
  uint32_t val, val2;
  static uint32_t avg = 0;
  enum { NSAMPLE = 5 };

  ret = board_get_battery_voltage(&val);
  if (ret < 0)
    {
      return ret;
    }

  /* Sleep 2.3ms, then take second sample. The value is chosen so that no
     two 2G modem TX bursts, of length 577us and period 4.615ms, can occur
     during both measurements. TX burst causes worst case 170mV voltage drop,
     which is normal behavior. */

  up_udelay(2300);
  ret = board_get_battery_voltage(&val2);
  if (ret < 0)
    {
      return ret;
    }

  /* Discard lower value, it might be taken during a TX burst. */

  val = MAX(val, val2);
  if (avg == 0)
    {
      avg = val;
    }
  else
    {
      /* This is an *approximation* of true sliding average
         of last NSAMPLE values. */

      avg -= avg / NSAMPLE;
      avg += val / NSAMPLE;
    }
  *voltage = avg;
  return OK;
}
