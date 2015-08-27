/****************************************************************************
 * include/nuttx/sensors/lsm9ds1.h
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
 *   Authors: Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
 *            Juha Niskanen <juha.niskanen@haltian.com>
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

#ifndef LSM9DS1_H_
#define LSM9DS1_H_

/* IOCTL commands for 9-axels control */

typedef enum lsm9ds1_ioc_cmds
  {
    /* common to all sub-devices: */
    LSM9DS1_IOC_WHO_AM_I = 0x0,
    LSM9DS1_IOC_CALIBRATE_BIAS_GYRO,
    LSM9DS1_IOC_CALIBRATE_BIAS_MAG,
    LSM9DS1_IOC_READ_BIAS,
    LSM9DS1_IOC_WRITE_BIAS,
    /* gyro and accelerometer: */
    LSM9DS1_IOC_CONFIG_GYRO,
    LSM9DS1_IOC_READ_RAW_GYRO,
    LSM9DS1_IOC_READ_STATUS_GYRO,
    LSM9DS1_IOC_RST_FIFO_GYRO,
    LSM9DS1_IOC_SET_ACT_THS_GYRO,
    LSM9DS1_IOC_READ_RAW_XL,
    /* thermometer: */
    LSM9DS1_IOC_READ_TEMP,
    /* magnetometer: */
    LSM9DS1_IOC_CONFIG_MAG,
    LSM9DS1_IOC_READ_RAW_MAG,
    LSM9DS1_IOC_READ_STATUS_MAG,
  } lsm9ds1_ioc_cmds;

enum { LSM9DS1_TEMPERATURE_PRECISION = 100 };

/* LSM9DS1 FIFO mode configuration */

typedef enum lsm9ds1_fifo_mode_t
  {
    LSM9DS1_FIFO_BYPASS = 0x0,
    LSM9DS1_FIFO_FIFO,
    LSM9DS1_FIFO_STREAMTOFIFO = 0x3,
    LSM9DS1_FIFO_BYPASSTOSTREAM,
    LSM9DS1_FIFO_STREAM = 0x6,
  } lsm9ds1_fifo_mode_t;

/* LSM9DS1 gyro operation modes */

typedef enum lsm9ds1_gyro_state_t
  {
    LSM9DS1_GYRO_STATE_POWER_DOWN = 0x0,
    LSM9DS1_GYRO_STATE_LOW_POWER,       /* Like normal mode but gyro can enter
                                         * low-power mode. */
    LSM9DS1_GYRO_STATE_NORMAL,  /* Both gyro and accelerometer running; default
                                 * mode. */
    LSM9DS1_GYRO_STATE_XL_ONLY, /* Only accelerometer running. */
  } lsm9ds1_gyro_state_t;

/* LSM9DS1 gyro/accel output data rate */

typedef enum lsm9ds1_gyro_odr_t
  {
    LSM9DS1_GYRO_ODR_PDOWN = 0x0,
    LSM9DS1_GYRO_ODR_14_9HZ,
    LSM9DS1_GYRO_ODR_59_5HZ,
    LSM9DS1_GYRO_ODR_119HZ,
    LSM9DS1_GYRO_ODR_238HZ,
    LSM9DS1_GYRO_ODR_476HZ,
    LSM9DS1_GYRO_ODR_952HZ,
  } lsm9ds1_gyro_odr_t;

/* Gyroscope Bandwidth Mode selection */

typedef enum lsm9ds1_gyro_bwm_t
  {
    LSM9DS1_GYRO_MODE_LOW = 0x0,
    LSM9DS1_GYRO_MODE_MEDIUM,
    LSM9DS1_GYRO_MODE_HIGH,
    LSM9DS1_GYRO_MODE_ULTRAHIGH,
  } lsm9ds1_gyro_bwm_t;

/* LSM9DS1 high-pass filter cutoff frequency configuration */

typedef enum lsm9ds1_gyro_hpcf_t
  {
    LSM9DS1_GYRO_HPCF0000 = 0x0,
    LSM9DS1_GYRO_HPCF0001,
    LSM9DS1_GYRO_HPCF0010,
    LSM9DS1_GYRO_HPCF0011,
    LSM9DS1_GYRO_HPCF0100,
    LSM9DS1_GYRO_HPCF0101,
    LSM9DS1_GYRO_HPCF0110,
    LSM9DS1_GYRO_HPCF0111,
    LSM9DS1_GYRO_HPCF1000,
    LSM9DS1_GYRO_HPCF1001
  } lsm9ds1_gyro_hpcf_t;

/* LSM9DS1 Full-scale selection */

typedef enum lsm9ds1_gyro_scale_t
  {
    LSM9DS1_GYRO_SCALE_245DPS = 0x0,
    LSM9DS1_GYRO_SCALE_500DPS,
    LSM9DS1_GYRO_SCALE_NOTAVAILABLE,
    LSM9DS1_GYRO_SCALE_2000DPS
  } lsm9ds1_gyro_scale_t;

static const float lsm9ds1_gyro_resolution[4] = {
  [LSM9DS1_GYRO_SCALE_245DPS] = 245.0 / 32768.0,
  [LSM9DS1_GYRO_SCALE_500DPS] = 500.0 / 32768.0,
  [LSM9DS1_GYRO_SCALE_2000DPS] = 2000.0 / 32768.0,
};

/* LSM9DS1 accelerometer output data rate */

typedef enum lsm9ds1_xl_odr_t
  {
    LSM9DS1_XL_ODR_PDOWN = 0x0,
    LSM9DS1_XL_ODR_10HZ,
    LSM9DS1_XL_ODR_50HZ,
    LSM9DS1_XL_ODR_119HZ,
    LSM9DS1_XL_ODR_238HZ,
    LSM9DS1_XL_ODR_476HZ,
    LSM9DS1_XL_ODR_952HZ,
  } lsm9ds1_xl_odr_t;

/* Accelerometer Bandwidth Mode selection */

typedef enum lsm9ds1_xl_bwm_t
  {
    LSM9DS1_XL_MODE_408HZ = 0x0,
    LSM9DS1_XL_MODE_211HZ,
    LSM9DS1_XL_MODE_105HZ,
    LSM9DS1_XL_MODE_50HZ,
  } lsm9ds1_xl_bwm_t;

/* Accelerometer output scale */

typedef enum lsm9ds1_xl_scale_t
  {
    LSM9DS1_XL_SCALE_2G = 0x0,
    LSM9DS1_XL_SCALE_NOTAVAILABLE,
    LSM9DS1_XL_SCALE_4G,
    LSM9DS1_XL_SCALE_8G,
  } lsm9ds1_xl_scale_t;

static const float lsm9ds1_xl_resolution[4] = {
  [LSM9DS1_XL_SCALE_2G] = 2.0 / 32768.0,
  [LSM9DS1_XL_SCALE_4G] = 4.0 / 32768.0,
  [LSM9DS1_XL_SCALE_8G] = 8.0 / 32768.0,
};

/* gyro and accelerometer status after interrupt */

typedef struct lsm9ds1_gyro_status_t
  {
    uint8_t fifo_src_data;
    uint8_t int_gyro_sts_data;
    uint8_t int_xl_sts_data;
    uint8_t status_reg_data;
  } lsm9ds1_gyro_status_t;

typedef struct lsm9ds1_gyro_config_data_t
  {
    lsm9ds1_gyro_state_t state;
    lsm9ds1_gyro_odr_t odr;
    lsm9ds1_gyro_bwm_t bwm;
    lsm9ds1_gyro_hpcf_t hpcf;
    lsm9ds1_gyro_scale_t scale;
    lsm9ds1_xl_odr_t xl_odr;
    lsm9ds1_xl_bwm_t xl_bwm;
    lsm9ds1_xl_scale_t xl_scale;
    lsm9ds1_fifo_mode_t fifo_mode;
    uint8_t fifo_samples_nbr;
    bool is_reboot;             /* When set to true, reboots memory content */
    bool fifo_en;               /* When set to true, FIFO is enabled */
    bool hpen;                  /* When set to true, high-pass filter is
                                 * enabled */
    bool bdu;                   /* Block data update */
    bool latch_int;             /* Latch interrupt on INT_G */
    bool is_high;               /* Trigger an interrupt when value is high/low
                                 * than threshold */
    bool threshold_int;         /* The signal will be send to INT_G if true */
    bool watermark_int;         /* Water mark interrupt on DRDY_G */
    uint16_t x_threshold;       /* Sensitivity thresholds for Gyro */
    uint16_t y_threshold;
    uint16_t z_threshold;
    uint8_t x_threshold_xl;     /* Sensitivity thresholds for Accelerometer */
    uint8_t y_threshold_xl;
    uint8_t z_threshold_xl;
    uint8_t int_gen_dur_g;      /* 7-bits value. If zero passed, the interrupt
                                 * will be triggered immediately */
    uint8_t int_gen_dur_xl;     /* 7-bits value. If zero passed, the interrupt
                                 * will be triggered immediately */
  } lsm9ds1_gyro_config_data_t;

/* LSM9DS1 magnetometer output data rate */

typedef enum lsm9ds1_mag_odr_t
  {
    LSM9DS1_MAG_ODR_0_625HZ = 0x0,
    LSM9DS1_MAG_ODR_1_25HZ,
    LSM9DS1_MAG_ODR_2_5HZ,
    LSM9DS1_MAG_ODR_5HZ,
    LSM9DS1_MAG_ODR_10HZ,
    LSM9DS1_MAG_ODR_20HZ,
    LSM9DS1_MAG_ODR_40HZ,
    LSM9DS1_MAG_ODR_80HZ,
  } lsm9ds1_mag_odr_t;

/* Magnetometer output scale or max range in Gauss */

typedef enum lsm9ds1_mag_scale_t
  {
    LSM9DS1_MAG_SCALE_4GS = 0x0,
    LSM9DS1_MAG_SCALE_8GS,
    LSM9DS1_MAG_SCALE_12GS,
    LSM9DS1_MAG_SCALE_16GS,
  } lsm9ds1_mag_scale_t;

static const float lsm9ds1_mag_resolution[4] = {
  [LSM9DS1_MAG_SCALE_4GS] = 4.0 / 32768.0,
  [LSM9DS1_MAG_SCALE_8GS] = 8.0 / 32768.0,
  [LSM9DS1_MAG_SCALE_12GS] = 12.0 / 32768.0,
  [LSM9DS1_MAG_SCALE_16GS] = 16.0 / 32768.0,
};

/* Different axis can be in different power-saving modes */

typedef enum lsm9ds1_mag_opermode_t
  {
    LSM9DS1_MAG_MODE_LOW = 0x0,
    LSM9DS1_MAG_MODE_MEDIUM,
    LSM9DS1_MAG_MODE_HIGH,
    LSM9DS1_MAG_MODE_ULTRAHIGH,
  } lsm9ds1_mag_opermode_t;

/* Magnetometer status data after interrupt */

typedef struct lsm9ds1_mag_status_t
  {
    uint8_t sts_reg_m;
    uint8_t int_src_reg_m;
  } lsm9ds1_mag_status_t;

typedef struct lsm9ds1_mag_config_data_t
  {
    bool mag_int_en;            /* Magnetic interrupt enabled */
    bool mag_temp_comp_en;      /* Use temperature compensation */
    bool mag_bdu;               /* Block data update */
    bool is_reboot;             /* When set to true, reboots memory content */
    lsm9ds1_mag_odr_t mag_odr;  /* Magnetic Output Data Rate */
    lsm9ds1_mag_scale_t mag_scale;
    lsm9ds1_mag_opermode_t mag_xy_opermode;
    lsm9ds1_mag_opermode_t mag_z_opermode;
    uint16_t mag_threshold;     /* Magnetic interrupt threshold */
    int16_t *mag_offset_xyz;    /* Environmental offset compensation. First is
                                 * X, last is Z */
  } lsm9ds1_mag_config_data_t;

/* Interrupt configuration data structure */

typedef struct lsm9ds1_config_t
  {
    int irq;
    int (*irq_attach) (FAR struct lsm9ds1_config_t * state, xcpt_t isr);
    void (*irq_enable) (FAR struct lsm9ds1_config_t * state, bool enable);
    void (*irq_clear) (FAR struct lsm9ds1_config_t * state);
    int (*set_power) (FAR struct lsm9ds1_config_t *state, bool on);
  } lsm9ds1_config_t;

/* Who am I data structure */

struct lsm9ds1_who_am_i_t
  {
    uint8_t acc_gyro;
    uint8_t magn;
  } packed_struct;
typedef struct lsm9ds1_who_am_i_t lsm9ds1_who_am_i_t;

/* raw output data in dps/g/gauss */

struct lsm9ds1_raw_data_t
  {
    uint16_t out_x;
    uint16_t out_y;
    uint16_t out_z;
  } packed_struct;
typedef struct lsm9ds1_raw_data_t lsm9ds1_raw_data_t;

struct lsm9ds1_sensor_bias_t
  {
    int16_t gyro_bias[3];
    int16_t xl_bias[3];
    int16_t mag_bias[3];
  } packed_struct;
typedef struct lsm9ds1_sensor_bias_t lsm9ds1_sensor_bias_t;

int lsm9ds1_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                     uint8_t * addr, lsm9ds1_config_t * config);

#endif /* LSM9DS1_H_ */
