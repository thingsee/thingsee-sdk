/****************************************************************************
 * include/nuttx/sensors/lsm9ds0.h
 *
 *   Copyright (C) 2014 Haltian Ltd. All rights reserved.
 *   Authors: Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
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

#ifndef LSM9DS0_H_
#define LSM9DS0_H_

/* IOCTL commands for 9-axels control */

typedef enum lsm9ds0_ioc_cmds
  {
    LSM9DS0_IOC_WHO_AM_I = 0x0,
    LSM9DS0_IOC_CONFIG_GYRO,
    LSM9DS0_IOC_READ_RAW_GYRO,
    LSM9DS0_IOC_READ_STS_BITS_GYRO,
    LSM9DS0_IOC_RST_FIFO_GYRO,
    LSM9DS0_IOC_CONFIG_ACC_MAGN,
    LSM9DS0_IOC_READ_RAW_ACC_MAGN,
    LSM9DS0_IOC_READ_STS_BITS_ACC_MAGN,
    LSM9DS0_IOC_RST_FIFO_ACC_MAGN
  } lsm9ds0_ioc_cmds;

/* LSM9DS0 gyro state */

typedef enum lsm9ds0_gyro_state_t
  {
    LSM9DS0_GYRO_STATE_POWER_DOWN = 0x0,
    LSM9DS0_GYRO_STATE_SLEEP,
    LSM9DS0_GYRO_STATE_NORMAL
  } lsm9ds0_gyro_state_t;

/* LSM9DS0 output data rate */

typedef enum lsm9ds0_gyro_odr_t
  {
    LSM9DS0_GYRO_ODR_95HZ = 0x0,
    LSM9DS0_GYRO_ODR_190HZ,
    LSM9DS0_GYRO_ODR_380HZ,
    LSM9DS0_GYRO_ODR_760HZ
  } lsm9ds0_gyro_odr_t;

/* LSM9DS0 cutoff values (generic). See BW-bit's values in documentation */

typedef enum lsm9ds0_gyro_cutoff_t
  {
    LSM9DS0_GYRO_CUTOFF_00 = 0x0,
    LSM9DS0_GYRO_CUTOFF_01,
    LSM9DS0_GYRO_CUTOFF_10,
    LSM9DS0_GYRO_CUTOFF_11
  } lsm9ds0_gyro_cutoff_t;

/* LSM9DS0 gyro-axels mask */

typedef enum lsm9ds0_gyro_axels_mask
  {
    LSM9DS0_GYRO_AXEL_YEN = 0x01,
    LSM9DS0_GYRO_AXEL_XEN,
    LSM9DS0_GYRO_AXEL_ZEN = 0x04
  } lsm9ds0_gyro_axels_mask;

/* LSM9DS0 magnetometer-axels interrupt mask */

typedef enum lsm9ds0_magn_int_axels_mask
  {
    LSM9DS0_MAGN_AXEL_ZIEN = 0x20,
    LSM9DS0_MAGN_AXEL_YIEN = 0x40,
    LSM9DS0_MAGN_AXEL_XIEN = 0x80
  } lsm9ds0_magn_int_axels_mask;

/* LSM9DS0 acc-axels mask */

typedef enum lsm9ds0_acc_axels_mask
  {
    LSM9DS0_ACC_AXEL_XEN = 0x01,
    LSM9DS0_ACC_AXEL_YEN,
    LSM9DS0_ACC_AXEL_ZEN = 0x04
  } lsm9ds0_acc_axels_mask;

/* LSM9DS0 high-pass filter mode */

typedef enum lsm9ds0_gyro_hpm_t
  {
    LSM9DS0_GYRO_HPM_NORMAL_MODE_RST_RD = 0x0,
    LSM9DS0_GYRO_HPM_REF_SIG_FILT,
    LSM9DS0_GYRO_HPM_NORMAL_MODE,
    LSM9DS0_GYRO_HPM_AUTO_RST
  } lsm9ds0_gyro_hpm_t;

/* LSM9DS0 high-pass filter cutoff frequency configuration */

typedef enum lsm9ds0_gyro_hpcf_t
  {
    LSM9DS0_GYRO_HPCF0000 = 0x0,
    LSM9DS0_GYRO_HPCF0001,
    LSM9DS0_GYRO_HPCF0010,
    LSM9DS0_GYRO_HPCF0011,
    LSM9DS0_GYRO_HPCF0100,
    LSM9DS0_GYRO_HPCF0101,
    LSM9DS0_GYRO_HPCF0110,
    LSM9DS0_GYRO_HPCF0111,
    LSM9DS0_GYRO_HPCF1000,
    LSM9DS0_GYRO_HPCF1001
  } lsm9ds0_gyro_hpcf_t;

/* LSM9DS0 Full-scale selection */

typedef enum lsm9ds0_gyro_fs_t
  {
    LSM9DS0_GYRO_FS_245DPS = 0x0,
    LSM9DS0_GYRO_FS_500DPS,
    LSM9DS0_GYRO_FS_2000DPS
  } lsm9ds0_gyro_fs_t;

/* LSM9DS0 acceleration data rate configuration */

typedef enum lsm9ds0_acc_aodr_t
  {
    LSM9DS0_ACC_AODR_PDOWN = 0x0,
    LSM9DS0_ACC_AODR_3_125HZ,
    LSM9DS0_ACC_AODR_6_25HZ,
    LSM9DS0_ACC_AODR_12_5HZ,
    LSM9DS0_ACC_AODR_25HZ,
    LSM9DS0_ACC_AODR_50HZ,
    LSM9DS0_ACC_AODR_100HZ,
    LSM9DS0_ACC_AODR_200HZ,
    LSM9DS0_ACC_AODR_400HZ,
    LSM9DS0_ACC_AODR_800HZ,
    LSM9DS0_ACC_AODR_1600HZ,
  } lsm9ds0_acc_aodr_t;

/* LSM9DS0 self-test mode selection. */

typedef enum lsm9ds0_gyro_st_t
  {
    LSM9DS0_GYRO_ST_NORMAL_MODE = 0x0,
    LSM9DS0_GYRO_ST_0,
    LSM9DS0_GYRO_ST_1 = 0x03
  } lsm9ds0_gyro_st_t;

/* LSM9DS0 FIFO mode configuration */

typedef enum lsm9ds0_fifo_mode_t
  {
    LSM9DS0_FIFO_BYPASS = 0x0,
    LSM9DS0_FIFO_FIFO,
    LSM9DS0_FIFO_STREAM,
    LSM9DS0_FIFO_STREAMTOFIFO,
    LSM9DS0_FIFO_BYPASSTOSTREAM
  } lsm9ds0_fifo_mode_t;

/* LSM9DS0 accelerometer anti-alias filter bandwidth */

typedef enum lsm9ds0_acc_anti_alias_bw_t
  {
    LSM9DS0_ACC_ABW_773HZ = 0x0,
    LSM9DS0_ACC_ABW_194HZ,
    LSM9DS0_ACC_ABW_362HZ,
    LSM9DS0_ACC_ABW_50HZ,
  } lsm9ds0_acc_anti_alias_bw_t;

/* LSM9DS0 acceleration full-scale selection */

typedef enum lsm9ds0_acc_full_scale_t
  {
    LSM9DS0_ACC_AFS_2G = 0x0,
    LSM9DS0_ACC_AFS_4G,
    LSM9DS0_ACC_AFS_6G,
    LSM9DS0_ACC_AFS_8G,
    LSM9DS0_ACC_AFS_16G,
  } lsm9ds0_acc_full_scale_t;

/* LSM9DS0 acc/magn self-test configuration */

typedef enum lsm9ds0_xm_self_tst_t
  {
    LSM9DS0_XM_STST_NORMAL_MODE = 0x0,
    LSM9DS0_XM_STST_POS_SIGN_TST,
    LSM9DS0_XM_STST_NEG_SIGN_TST,
  } lsm9ds0_xm_self_tst_t;

/* LSM9DS0 magnetic data rate configuration */

typedef enum lsm9ds0_magn_odr_t
  {
    LSM9DS0_M_ODR_3_125HZ = 0x0,
    LSM9DS0_M_ODR_6_25HZ,
    LSM9DS0_M_ODR_12_5HZ,
    LSM9DS0_M_ODR_25HZ,
    LSM9DS0_M_ODR_50HZ,
    LSM9DS0_M_ODR_100HZ         /* Only for accelerometer with ODR > 50 Hz */
  } lsm9ds0_magn_odr_t;

/* LSM9DS0 magnetic full-scale selection */

typedef enum lsm9ds0_magn_mfs_t
  {
    LSM9DS0_MFS_2GAUSS = 0x0,
    LSM9DS0_MFS_4GAUSS,
    LSM9DS0_MFS_8GAUSS,
    LSM9DS0_MFS_12GAUSS
  } lsm9ds0_magn_mfs_t;

/* LSM9DS0 high-pass filter mode selection for acceleration data */

typedef enum lsm9ds0_acc_ahpm_t
  {
    LSM9DS0_AHPM_NORMAL_MODE_RST = 0x0,
    LSM9DS0_AHPM_REF_SIGNAL,
    LSM9DS0_AHPM_NORMAL_MODE,
    LSM9DS0_AHPM_AUTO_RST
  } lsm9ds0_acc_ahpm_t;

/* LSM9DS0 magnetic sensor mode selection */

typedef enum lsm9ds0_md1_t
  {
    LSM9DS0_MD1_CONT_CONV = 0x0,
    LSM9DS0_MD1_SINGLE_CONV,
    LSM9DS0_MD1_PDOWN
  } lsm9ds0_md1_t;

/* LSM9DS0 magnetic sensor mode selection */

/* Interrupt configuration data structure */

typedef struct lsm9ds0_config_t
  {
    int irq;
    int (*irq_attach) (FAR struct lsm9ds0_config_t * state, xcpt_t isr);
    void (*irq_enable) (FAR struct lsm9ds0_config_t * state, bool enable);
    void (*irq_clear) (FAR struct lsm9ds0_config_t * state);
  } lsm9ds0_config_t;

/* Who am I data structure */

typedef struct lsm9ds0_who_am_i_t
  {
    uint8_t acc_magn;
    uint8_t gyro;
  } lsm9ds0_who_am_i_t;

/* Gyro's configuration data */

typedef struct lsm9ds0_gyro_config_data_t
  {
    lsm9ds0_gyro_axels_mask axels_mask;
    lsm9ds0_gyro_cutoff_t cutoff;
    lsm9ds0_gyro_odr_t odr;
    lsm9ds0_gyro_state_t state;
    lsm9ds0_gyro_hpm_t hpm;
    lsm9ds0_gyro_hpcf_t hpcf;
    lsm9ds0_gyro_fs_t fs;
    lsm9ds0_fifo_mode_t fifo_mode;
    lsm9ds0_gyro_st_t st;
    uint8_t fifo_samples_nbr;
    bool is_reboot;             /* When set to true, reboots memory content */
    bool fifo_en;               /* When set to true, FIFO is enabled */
    bool hpen;                  /* When set to true, high-pass filter is enabled */
    bool bdu;                   /* Block data update */
    bool latch_int;             /* Latch interrupt on INT_G */
    bool is_high;               /* Trigger an interrupt when value is high/low
                                 * than threshold */
    bool threshold_int;         /* The signal will be send to INT_G if true */
    bool watermark_int;         /* Water mark interrupt on DRDY_G */
    uint16_t x_threshold;
    uint16_t y_threshold;
    uint16_t z_threshold;
    uint8_t interrupt_duration; /* 7-bits value. If zero passed, the interrupt
                                 * will be triggered immediately */
  } lsm9ds0_gyro_config_data_t;

/* Acc/magn configuration data */

typedef struct lsm9ds0_xm_config_data_t
  {
    lsm9ds0_magn_int_axels_mask int_maxels_mask;
    bool int_high_active;       /* If true the interrupt will be a high-active
                                 * one */
    bool int_latch;             /* true to latch interrupt before unblocking it */
    bool int_magn_en;           /* Enable interrupt generation for magnetic data */
    bool four_D;                /* 4D detection on acceleration data is enabled
                                 * when 6D bit is set */
    uint16_t magn_threshold;    /* Magnetic interrupt threshold */
    uint16_t *magn_offset_xyz;  /* First value is X, last is Z */
    bool reboot;                /* Reboots memory content */
    bool fifo_en;               /* Enable or disable fifo */
    bool wtm_en;                /* Enable/disable water-mark interrupt. Works
                                 * for CTRL_REG4_XM as well */
    bool bdu;                   /* Block data update till MSB and LSB is read */
    lsm9ds0_acc_axels_mask acc_ax_mask; /* Enable/Disable axles */
    lsm9ds0_acc_aodr_t aodr;    /* Acceleration data rate configuration */
    lsm9ds0_acc_anti_alias_bw_t acc_aa_bw;      /* Acceleration anti-alias
                                                 * filter bandwidth */
    lsm9ds0_acc_full_scale_t acc_fullscale;     /* Acceleration full-scale
                                                 * selection */
    lsm9ds0_xm_self_tst_t s_test_mode;  /* Self-test mode configuration */
    bool p_int_magn;            /* Magnetic interrupt generator on INT1_XM pin */
    bool p_int_inertial1;       /* Inertial interrupt generator 1 on INT1_XM pin */
    bool p_int_inertial2;       /* Inertial interrupt generator 2 on INT1_XM pin */
    bool temp_en;               /* Temperature sensor enable */
    bool magn_high_res;         /* Magnetic resolution selection (false: low
                                 * resolution, true: high resolution) */
    lsm9ds0_magn_odr_t magn_odr;        /* Magnetic data rate configuration */
    bool lir1;                  /* Latch interrupt request on INT1_SRC
                                 * register, with INT1_SRC register cleared by
                                 * reading INT1_SRC itself */
    bool lir2;                  /* Latch interrupt request on INT2_SRC
                                 * register, with INT2_SRC register cleared by
                                 * reading INT2_SRC itself */
    lsm9ds0_magn_mfs_t mfs;     /* Magnetic full-scale selection */
    lsm9ds0_acc_ahpm_t acc_ahpm;        /* High-pass filter mode selection */
    lsm9ds0_md1_t md1;          /* Magnetic sensor mode selection */
    lsm9ds0_fifo_mode_t fifo_mode;      /* FIFO mode selection */
    uint8_t fifo_samples_nbr;   /* FIFO water-mark level, 5-bits */
    bool six_D1;                /* 6-direction movement recognition */
    bool aoi1;                  /* And/OR interrupt. See specification (page
                                 * 63) for more information */
    bool six_D2;                /* 6-direction movement recognition */
    bool aoi2;                  /* And/OR interrupt. See specification (page
                                 * 63) for more information */
    bool int1_gen_high;         /* Enable interrupt 1 generation on high/low
                                 * event */
    uint8_t int1_gen_ths;       /* Interrupt 1 threshold */
    uint8_t int1_gen_duration;  /* Sets the minimum duration of the Interrupt 1
                                 * event to be recognized */
    bool int2_gen_high;         /* Enable interrupt 2 generation on high/low
                                 * event */
    uint8_t int2_gen_ths;       /* Interrupt 2 threshold */
    uint8_t int2_gen_duration;  /* Sets the minimum duration of the Interrupt 2
                                 * event to be recognized */
  } lsm9ds0_xm_config_data_t;

/* Acc/magn raw data */

typedef struct lsm9ds0_acc_magn_raw_data_t
  {
    uint16_t *magn_xyz;         /* Magnetometer's raw data. Format x, y, z */
    uint16_t *acc_xyz;          /* Accelerometer's raw data. Format x, y, z */
  } lsm9ds0_acc_magn_raw_data_t;

/* Acc/magn status data after interrupt */

typedef struct lsm9ds0_acc_magn_sts_data_t
  {
    uint8_t sts_reg_m;
    uint8_t int_src_reg_m;
    uint8_t sts_reg_a;
    uint8_t int_gen_1_src;
    uint8_t int_gen_2_src;
  } lsm9ds0_acc_magn_sts_data_t;

/* Gyro's raw output data in dps */

typedef struct lsm9ds0_gyro_raw_data_t
  {
    uint16_t out_x_g;
    uint16_t out_y_g;
    uint16_t out_z_g;
  } lsm9ds0_gyro_raw_data_t;

/* Gyro's interrupt information */

typedef struct lsm9ds0_gyro_int_info_t
  {
    uint8_t fifo_src_data;
    uint8_t int1_sts_data;
    uint8_t status_reg_data;
  } lsm9ds0_gyro_int_info_t;

int lsm9ds0_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                     uint8_t * addr, lsm9ds0_config_t * config);

#endif /* LSM9DS0_H_ */
