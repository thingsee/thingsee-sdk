/****************************************************************************
 * include/nuttx/sensors/sht25.h
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
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

#ifndef SHT25_H_
#define SHT25_H_

int sht25_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                   uint8_t addr);

enum IO_OPERATIONS
  {
    SHT25_IO_HUMIDITY_OUT_START = 0,
    SHT25_IO_HUMIDITY_OUT_STOP,
    SHT25_IO_TEMPERATURE_OUT_START,
    SHT25_IO_TEMPERATURE_OUT_STOP,
    SHT25_IO_SOFT_RESET
  };

#define SHT25_TM_NO_HOLDMASTER				0xF3
#define SHT25_SOFT_RESET				0xFE
#define SHT25_RH_NO_HOLDMASTER				0xF5

#define SHT25_DIVIDER_HUMID 				10
#define SHT25_DIVIDER_TEMPER				100

/* Must be divide by SHT25_DEVIDER_TEMPER in your app code */
typedef struct sht25_temper_data_s
  {
    int32_t int_temper;
  } sht25_temper_data_t;

/* Must be divide by SHT25_DEVIDER_HUMID in your app code */
typedef struct sht25_humidity_data_s
  {
    uint32_t int_humid;
  } sht25_humidity_data_t;

#endif /* SHT25_H_ */
