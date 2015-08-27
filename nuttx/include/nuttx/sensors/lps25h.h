/****************************************************************************
 * include/nuttx/sensors/lps25h.h
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

#ifndef LPS25H_H_
#define LPS25H_H_

#define LPS25H_TEMPER_DIVIDER  1000

/* int_temper value must be divided by LPS25H_TEMPER_DIVIDER in your app code */
typedef struct lps25h_temper_data_s
  {
    int32_t int_temper;
    int16_t raw_data;
  } lps25h_temper_data_t;

typedef struct lps25h_pressure_data_s
  {
    uint32_t pressure_int_hP;
    uint32_t pressure_Pa;
    uint32_t raw_data;
  } lps25h_pressure_data_t;

typedef struct lps25h_who_am_i_data
  {
    uint8_t who_am_i;
  } lps25h_who_am_i_data;

typedef struct lps25h_config_s
  {
    /* Device characterization */

    int irq;                    /* IRQ number received by interrupt handler. */

    /* IRQ/GPIO access callbacks.  These operations all hidden behind callbacks
     * to isolate the driver from differences in GPIO interrupt handling
     * by varying boards and MCUs.
     * irq_attach - Attach the interrupt handler to the GPIO interrupt
     * irq_enable - Enable or disable the GPIO
     * interrupt clear_irq - Acknowledge/clear any pending GPIO interrupt */

    int (*irq_attach) (FAR struct lps25h_config_s * state, xcpt_t isr);
    void (*irq_enable) (FAR struct lps25h_config_s * state, bool enable);
    void (*irq_clear) (FAR struct lps25h_config_s * state);
  } lps25h_config_t;

#  define LPS25H_REF_P_XL		0x08
#  define LPS25H_REF_P_L		0x09
#  define LPS25H_REF_P_H		0x0A
#  define LPS25H_WHO_AM_I		0x0F
#  define LPS25H_RES_CONF		0x10
#  define LPS25H_CTRL_REG1		0x20
#  define LPS25H_CTRL_REG2		0x21
#  define LPS25H_CTRL_REG3		0x22
#  define LPS25H_CTRL_REG4		0x23
#  define LPS25H_INT_CFG		0x24
#  define LPS25H_INT_SOURCE		0x25
#  define LPS25H_STATUS_REG		0x27
#  define LPS25H_PRESS_POUT_XL		0x28
#  define LPS25H_PRESS_OUT_L		0x29
#  define LPS25H_PRESS_OUT_H		0x2A
#  define LPS25H_TEMP_OUT_L		0x2B
#  define LPS25H_TEMP_OUT_H		0x2C
#  define LPS25H_FIFO_CTRL		0x2E
#  define LPS25H_FIFO_STATUS		0x2F
#  define LPS25H_THS_P_L		0x30
#  define LPS25H_THS_P_H		0x31
#  define LPS25H_RPDS_L			0x39
#  define LPS25H_RPDS_H			0x3A

#  define LPS25H_AUTO_ZERO		(1 << 2)
#  define LPS25H_BDU			(1 << 2)
#  define LPS25H_DIFF_EN		(1 << 3)
#  define LPS25H_FIFO_EN		(1 << 6)
#  define LPS25H_WTM_EN			(1 << 5)
#  define LPS25H_FIFO_MEAN_DEC		(1 << 4)
#  define LPS25H_PD			(1 << 7)
#  define LPS25H_ONE_SHOT		(1 << 0)
#  define LPS25H_INT_H_L		(1 << 7)
#  define LPS25H_PP_OD			(1 << 6)

#  define LPS25H_VALID_WHO_AM_I 0xBD

int lps25h_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                    uint8_t addr, lps25h_config_t * config);

enum lps25h_ioctl_cmds_t
  {
    LPS25H_PRES_CONFIG_ON = 0,
    LPS25H_PRESSURE_OUT,
    LPS25H_TEMPERATURE_OUT,
    LPS25H_SENSOR_OFF,
    LPS25H_SENSOR_WHO_AM_I
  };

#endif /* LPS25H_H_ */
