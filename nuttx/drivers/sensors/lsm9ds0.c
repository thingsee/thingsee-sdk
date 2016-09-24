/****************************************************************************
 * drivers/sensors/lsm9ds0.c
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

#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/i2c.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <nuttx/kmalloc.h>

#include <nuttx/sensors/lsm9ds0.h>

#ifdef CONFIG_TRACES_GAM9AXEL_SENS
#  define lsm9ds0_dbg(x, ...)   dbg(x, ##__VA_ARGS__)
#  define lsm9ds0_lldbg(x, ...) lldbg(x, ##__VA_ARGS__)
#else
#  define lsm9ds0_dbg(x, ...)
#  define lsm9ds0_lldbg(x, ...)
#endif

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define LSM9DS0_WHO_AM_I_G				0x0F
#define LSM9DS0_CTRL_REG1_G				0x20
#define LSM9DS0_CTRL_REG2_G				0x21
#define LSM9DS0_CTRL_REG3_G				0x22
#define LSM9DS0_CTRL_REG4_G				0x23
#define LSM9DS0_CTRL_REG5_G				0x24
#define LSM9DS0_REFERENCE_G				0x25
#define LSM9DS0_STATUS_REG_G				0x27
#define LSM9DS0_OUT_X_L_G				0x28
#define LSM9DS0_OUT_X_H_G				0x29
#define LSM9DS0_OUT_Y_L_G				0x2A
#define LSM9DS0_OUT_Y_H_G				0x2B
#define LSM9DS0_OUT_Z_L_G				0x2C
#define LSM9DS0_OUT_Z_H_G				0x2D
#define LSM9DS0_FIFO_CTRL_REG_G				0x2E
#define LSM9DS0_FIFO_SRC_REG_G				0x2F
#define LSM9DS0_INT1_CFG_G				0x30
#define LSM9DS0_INT1_SRC_G				0x31
#define LSM9DS0_INT1_TSH_XH_G				0x32
#define LSM9DS0_INT1_TSH_XL_G				0x33
#define LSM9DS0_INT1_TSH_YH_G				0x34
#define LSM9DS0_INT1_TSH_YL_G				0x35
#define LSM9DS0_INT1_TSH_ZH_G				0x36
#define LSM9DS0_INT1_TSH_ZL_G				0x37
#define LSM9DS0_INT1_DURATION_G				0x38
#define LSM9DS0_OUT_TEMP_L_XM				0x05
#define LSM9DS0_OUT_TEMP_H_XM				0x06
#define LSM9DS0_STATUS_REG_M				0x07
#define LSM9DS0_OUT_X_L_M				0x08
#define LSM9DS0_OUT_X_H_M				0x09
#define LSM9DS0_OUT_Y_L_M				0x0A
#define LSM9DS0_OUT_Y_H_M				0x0B
#define LSM9DS0_OUT_Z_L_M				0x0C
#define LSM9DS0_OUT_Z_H_M				0x0D
#define LSM9DS0_WHO_AM_I_XM				0x0F
#define LSM9DS0_INT_CTRL_REG_M				0x12
#define LSM9DS0_INT_SRC_REG_M				0x13
#define LSM9DS0_INT_THS_L_M				0x14
#define LSM9DS0_INT_THS_H_M				0x15
#define LSM9DS0_OFFSET_X_L_M				0x16
#define LSM9DS0_OFFSET_X_H_M				0x17
#define LSM9DS0_OFFSET_Y_L_M				0x18
#define LSM9DS0_OFFSET_Y_H_M				0x19
#define LSM9DS0_OFFSET_Z_L_M				0x1A
#define LSM9DS0_OFFSET_Z_H_M				0x1B
#define LSM9DS0_REFERENCE_X				0x1C
#define LSM9DS0_REFERENCE_Y				0x1D
#define LSM9DS0_REFERENCE_Z				0x1E
#define LSM9DS0_CTRL_REG0_XM				0x1F
#define LSM9DS0_CTRL_REG1_XM				0x20
#define LSM9DS0_CTRL_REG2_XM				0x21
#define LSM9DS0_CTRL_REG3_XM				0x22
#define LSM9DS0_CTRL_REG4_XM				0x23
#define LSM9DS0_CTRL_REG5_XM				0x24
#define LSM9DS0_CTRL_REG6_XM				0x25
#define LSM9DS0_CTRL_REG7_XM				0x26
#define LSM9DS0_STATUS_REG_A				0x27
#define LSM9DS0_OUT_X_L_A				0x28
#define LSM9DS0_OUT_X_H_A				0x29
#define LSM9DS0_OUT_Y_L_A				0x2A
#define LSM9DS0_OUT_Y_H_A				0x2B
#define LSM9DS0_OUT_Z_L_A				0x2C
#define LSM9DS0_OUT_Z_H_A				0x2D
#define LSM9DS0_FIFO_CTRL_REG				0x2E
#define LSM9DS0_FIFO_SRC_REG				0x2F
#define LSM9DS0_INT_GEN_1_REG				0x30
#define LSM9DS0_INT_GEN_1_SRC				0x31
#define LSM9DS0_INT_GEN_1_THS				0x32
#define LSM9DS0_INT_GEN_1_DURATION			0x33
#define LSM9DS0_INT_GEN_2_REG				0x34
#define LSM9DS0_INT_GEN_2_SRC				0x35
#define LSM9DS0_INT_GEN_2_THS				0x36
#define LSM9DS0_INT_GEN_2_DURATION			0x37
#define LSM9DS0_CLICK_CFG				0x38
#define LSM9DS0_CLICK_SRC				0x39
#define LSM9DS0_CLICK_THS				0x3A
#define LSM9DS0_TIME_LIMIT				0x3B
#define LSM9DS0_TIME_LATENCY				0x3C
#define LSM9DS0_TIME_WINDOW				0x3D
#define LSM9DS0_ACT_THS					0x3E
#define LSM9DS0_ACT_DUR					0x3F

/****************************************************************************
* Private Function Prototypes
*****************************************************************************/

static int lsm9ds0_open(FAR struct file *filep);
static int lsm9ds0_close(FAR struct file *filep);
static ssize_t lsm9ds0_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t lsm9ds0_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int lsm9ds0_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int lsm9ds0_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);
#endif

/****************************************************************************
* Private Data
****************************************************************************/

typedef enum lsm9ds0_addr_type_t
  {
    LSM9DS0_ADDR_TYPE_ACC_MAGN = 0x0,
    LSM9DS0_ADDR_TYPE_GYRO
  } lsm9ds0_addr_type_t;

struct lsm9ds0_dev_t
  {
    struct i2c_dev_s *i2c;
    uint8_t acc_addr;
    uint8_t gyro_addr;
    lsm9ds0_config_t *config;
    sem_t devsem;
    sem_t st_sem;
    bool int_pending;
#ifndef CONFIG_DISABLE_POLL
    struct pollfd *fds[CONFIG_GAM9AXEL_SENS_NPOLLWAITERS];
#endif
  };

static const struct file_operations g_9axelsops = {
  lsm9ds0_open,
  lsm9ds0_close,
  lsm9ds0_read,
  lsm9ds0_write,
  0,
  lsm9ds0_ioctl,
#ifndef CONFIG_DISABLE_POLL
  lsm9ds0_poll
#endif
};

static struct lsm9ds0_dev_t *g_9axel_data;

static int lsm9ds0_write_reg8(FAR struct lsm9ds0_dev_t *dev, uint8_t * command,
                              lsm9ds0_addr_type_t addr_type)
{
  int ret = OK;
  uint8_t addr = 0;

  if (addr_type == LSM9DS0_ADDR_TYPE_ACC_MAGN)
    {
      addr = dev->acc_addr;
    }
  else
    {
      addr = dev->gyro_addr;
    }

  ret = I2C_SETADDRESS(dev->i2c, addr, 7);

  if (ret < 0)
    {
      return ERROR;
    }

  ret = I2C_WRITE(dev->i2c, command, 2);
  if (ret < 0)
    {
      return ERROR;
    }

  return OK;
}

static int lsm9ds0_read_reg8(struct lsm9ds0_dev_t *dev, uint8_t * regaddr,
                             uint8_t * value, lsm9ds0_addr_type_t addr_type)
{
  int ret = OK;
  uint8_t addr = 0;

  if (addr_type == LSM9DS0_ADDR_TYPE_ACC_MAGN)
    {
      addr = dev->acc_addr;
    }
  else
    {
      addr = dev->gyro_addr;
    }

  ret = I2C_SETADDRESS(dev->i2c, addr, 7);
  if (ret < 0)
    {
      perror("Can't set address");
      return ERROR;
    }

  ret = I2C_WRITE(dev->i2c, regaddr, 1);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = I2C_READ(dev->i2c, value, 1);
  if (ret < 0)
    {
      return ERROR;
    }

  return OK;
}

#ifdef CONFIG_TRACES_GAM9AXEL_SENS
static int lsm9ds0_debug_sens_regs_gyro(FAR struct lsm9ds0_dev_t *priv)
{
  int ret = OK;
  uint8_t regaddr = 0;
  uint8_t value = 0;

  lsm9ds0_lldbg("============================ REGS ============================\n");

  regaddr = LSM9DS0_CTRL_REG1_G;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("CTRL_REG1: 0x%02X\n", value);

  regaddr = LSM9DS0_CTRL_REG2_G;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("CTRL_REG2: 0x%02X\n", value);

  regaddr = LSM9DS0_CTRL_REG3_G;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("CTRL_REG3: 0x%02X\n", value);

  regaddr = LSM9DS0_CTRL_REG4_G;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("CTRL_REG4: 0x%02X\n", value);

  regaddr = LSM9DS0_CTRL_REG5_G;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("CTRL_REG5: 0x%02X\n", value);

  regaddr = LSM9DS0_FIFO_CTRL_REG_G;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("FIFO_CTRL_REG: 0x%02X\n", value);

  regaddr = LSM9DS0_INT1_CFG_G;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("INT1_CFG_G: 0x%02X\n", value);

  regaddr = LSM9DS0_INT1_DURATION_G;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("INT1_DURATION_G: 0x%02X\n", value);

  lsm9ds0_lldbg("==============================================================\n");

  return ret;
}
#endif

static int lsm9ds0_ask_who_am_i(FAR struct lsm9ds0_dev_t *priv,
                                lsm9ds0_who_am_i_t * who_am_i)
{
  int ret = OK;
  uint8_t regaddr = 0;

  regaddr = LSM9DS0_WHO_AM_I_XM;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &who_am_i->acc_magn,
                          LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot read Acc's who_am_i");
      return ERROR;
    }

  regaddr = LSM9DS0_WHO_AM_I_G;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &who_am_i->gyro, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot read Gyro's who_am_i");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_gyro_normal_state(FAR struct lsm9ds0_dev_t *priv,
                                         lsm9ds0_gyro_config_data_t *
                                         config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG1_G;
  uint8_t value = 0x08;
  uint8_t cmd[2] = { 0 };

  value |= config_data->axels_mask |
           (config_data->odr << 6) | (config_data->cutoff << 4);
  lsm9ds0_lldbg("Value to write normal mode: 0x%02X\n", value);

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot setup ctrl_reg1_g");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_gyro_powerdown_state(FAR struct lsm9ds0_dev_t *priv,
                                            lsm9ds0_gyro_config_data_t *
                                            config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG1_G;
  uint8_t value = 0x0;
  uint8_t cmd[2] = { 0 };

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot setup ctrl_reg1_g");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_gyro_sleep_state(FAR struct lsm9ds0_dev_t *priv,
                                        lsm9ds0_gyro_config_data_t *
                                        config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG1_G;
  uint8_t value = 0x08;
  uint8_t cmd[2] = { 0 };

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot setup ctrl_reg1_g");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_ctrl_reg1_g(FAR struct lsm9ds0_dev_t *priv,
                                   lsm9ds0_gyro_config_data_t * config_data)
{
  int ret = OK;

  switch (config_data->state)
    {
    case LSM9DS0_GYRO_STATE_NORMAL:
      ret = lsm9ds0_set_gyro_normal_state(priv, config_data);
      break;
    case LSM9DS0_GYRO_STATE_POWER_DOWN:
      ret = lsm9ds0_set_gyro_powerdown_state(priv, config_data);
      break;
    case LSM9DS0_GYRO_STATE_SLEEP:
      ret = lsm9ds0_set_gyro_sleep_state(priv, config_data);
      break;
    default:
      break;
    }

  return ret;
}

static int lsm9ds0_set_ctrl_reg2_g(FAR struct lsm9ds0_dev_t *priv,
                                   lsm9ds0_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG2_G;
  uint8_t value = 0;
  uint8_t value_mask = 0x3F;
  uint8_t cmd[2] = { 0 };

  value = (config_data->hpm | config_data->hpcf) & value_mask;
  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot set ctrl_reg2");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_ctrl_reg3_g(FAR struct lsm9ds0_dev_t *priv,
                                   lsm9ds0_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG3_G;
  uint8_t value = 0;            /* Water mark interrupt is ON by default */
  uint8_t cmd[2] = { 0 };

  value = (config_data->watermark_int ? (1 << 2) : 0);
  value |= (config_data->threshold_int ? (1 << 7) : 0);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot set ctrl_reg3");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_ctrl_reg4_g(FAR struct lsm9ds0_dev_t *priv,
                                   lsm9ds0_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG4_G;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = (config_data->bdu ? (1 << 7) : 0);
  value |= (config_data->fs << 4);
  value |= (config_data->st << 1);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot set ctrl_reg4");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_ctrl_reg5_g(FAR struct lsm9ds0_dev_t *priv,
                                   lsm9ds0_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG5_G;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = (config_data->fifo_en ? (1 << 6) : 0);
  value |= (config_data->hpen ? (1 << 4) : 0);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot set ctrl_reg5");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_fifo_ctrl_reg_g(FAR struct lsm9ds0_dev_t *priv,
                                       lsm9ds0_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_FIFO_CTRL_REG_G;
  uint8_t value = config_data->fifo_mode << 5 | config_data->fifo_samples_nbr;
  uint8_t cmd[2] = { 0 };

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot set fifo_ctrl_reg");
      return ERROR;
    }

  return ret;
}

#ifdef CONFIG_TRACES_GAM9AXEL_SENS
static int lsm9ds0_debug_sens_regs_xm(FAR struct lsm9ds0_dev_t *priv)
{
  int ret = OK;
  uint8_t regaddr = 0;
  uint8_t value = 0;

  lsm9ds0_lldbg("============================ REGS ============================\n");

  regaddr = LSM9DS0_INT_CTRL_REG_M;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("INT_CTRL_REG_M: 0x%02X\n", value);

  regaddr = LSM9DS0_CTRL_REG0_XM;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("CTRL_REG0_XM: 0x%02X\n", value);

  regaddr = LSM9DS0_CTRL_REG1_XM;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("CTRL_REG1_XM: 0x%02X\n", value);

  regaddr = LSM9DS0_CTRL_REG2_XM;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("CTRL_REG2_XM: 0x%02X\n", value);

  regaddr = LSM9DS0_CTRL_REG3_XM;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("CTRL_REG3_XM: 0x%02X\n", value);

  regaddr = LSM9DS0_CTRL_REG4_XM;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("CTRL_REG4_XM: 0x%02X\n", value);

  regaddr = LSM9DS0_CTRL_REG5_XM;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("CTRL_REG5_XM: 0x%02X\n", value);

  regaddr = LSM9DS0_CTRL_REG6_XM;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("CTRL_REG6_XM: 0x%02X\n", value);

  regaddr = LSM9DS0_CTRL_REG7_XM;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("CTRL_REG7_XM: 0x%02X\n", value);

  regaddr = LSM9DS0_FIFO_CTRL_REG;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      return ERROR;
    }
  lsm9ds0_lldbg("FIFO_CTRL_REG: 0x%02X\n", value);

  lsm9ds0_lldbg("==============================================================\n");

  return ret;
}
#endif

static int lsm9ds0_set_int_ctrl_reg_m(FAR struct lsm9ds0_dev_t *priv,
                                      lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_INT_CTRL_REG_M;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = config_data->int_maxels_mask;
  value |= (config_data->int_high_active ? (1 << 3) : 0);
  value |= (config_data->int_latch ? (1 << 2) : 0);
  value |= ((config_data->four_D &&
           (config_data->six_D1 || config_data->six_D2)) ? (1 << 1) : 0);
  value |= (config_data->int_magn_en ? (1 << 0) : 0);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set int_ctrl_reg");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_int_ths_m(FAR struct lsm9ds0_dev_t *priv,
                                 lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_INT_THS_L_M;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = (uint8_t) (config_data->magn_threshold & 0xFF);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set int_ths_l_m");
      return ERROR;
    }

  regaddr = LSM9DS0_INT_THS_H_M;
  value = (uint8_t) (config_data->magn_threshold >> 8);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set int_ths_h_m");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_offset_regs_m(FAR struct lsm9ds0_dev_t *priv,
                                     lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_OFFSET_X_L_M;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };
  int i = 0;

  if (config_data->magn_offset_xyz)
    {
      for (i = 0; i < 3; i++)
        {
          value = (uint8_t) (config_data->magn_offset_xyz[i] & 0xFF);

          cmd[0] = regaddr;
          cmd[1] = value;

          ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
          if (ret < 0)
            {
              perror("Cannot set offset_reg_l_m");
              return ERROR;
            }

          regaddr++;
          value = (uint8_t) (config_data->magn_offset_xyz[i] >> 8);

          cmd[0] = regaddr;
          cmd[1] = value;

          ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
          if (ret < 0)
            {
              perror("Cannot set offset_reg_h_m");
              return ERROR;
            }

          regaddr++;
        }
    }

  return ret;
}

static int lsm9ds0_set_ctrl_reg0_xm(FAR struct lsm9ds0_dev_t *priv,
                                    lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG0_XM;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = (config_data->reboot ? (1 << 7) : 0);
  value |= (config_data->fifo_en ? (1 << 6) : 0);
  value |= (config_data->wtm_en ? (1 << 5) : 0);

  cmd[0] = regaddr;
  cmd[1] = (value & 0xE7);

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set ctrl_reg0_xm");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_ctrl_reg1_xm(FAR struct lsm9ds0_dev_t *priv,
                                    lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG1_XM;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = config_data->acc_ax_mask;
  value |= (config_data->bdu ? (1 << 3) : 0);
  value |= config_data->aodr << 4;

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set ctrl_reg1_xm");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_ctrl_reg2_xm(FAR struct lsm9ds0_dev_t *priv,
                                    lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG2_XM;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  /* Self-test mode will be turned on separately */

  value = config_data->acc_aa_bw << 6;
  value |= config_data->acc_fullscale << 3;
  value |= (config_data->s_test_mode << 1);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set ctrl_reg2_xm");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_ctrl_reg3_xm(FAR struct lsm9ds0_dev_t *priv,
                                    lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG3_XM;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = (config_data->p_int_inertial1 ? (1 << 5) : 0);
  value |= (config_data->p_int_inertial1 ? (1 << 4) : 0);
  value |= (config_data->p_int_magn ? (1 << 3) : 0);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set ctrl_reg3_xm");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_ctrl_reg4_xm(FAR struct lsm9ds0_dev_t *priv,
                                    lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG4_XM;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = (config_data->wtm_en ? (1 << 0) : 0);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set ctrl_reg4_xm");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_ctrl_reg5_xm(FAR struct lsm9ds0_dev_t *priv,
                                    lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG5_XM;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = (config_data->temp_en ? (1 << 7) : 0);
  value |= (config_data->magn_high_res ? (0x03 << 5) : 0);
  value |= config_data->magn_odr << 2;
  value |= (config_data->lir2 ? (1 << 1) : 0);
  value |= (config_data->lir1 ? (1 << 0) : 0);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set ctrl_reg5_xm");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_ctrl_reg6_xm(FAR struct lsm9ds0_dev_t *priv,
                                    lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG6_XM;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = (config_data->mfs << 5) & 0x60;

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set ctrl_reg6_xm");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_ctrl_reg7_xm(FAR struct lsm9ds0_dev_t *priv,
                                    lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_CTRL_REG7_XM;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = config_data->acc_ahpm << 6;
  value |= config_data->md1 & 0xE7;

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set ctrl_reg7_xm");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_fifo_ctrl_reg_xm(FAR struct lsm9ds0_dev_t *priv,
                                        lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_FIFO_CTRL_REG;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = config_data->fifo_mode << 5;
  value |= config_data->fifo_samples_nbr;

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set fifo_ctrl_reg");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_int_gen_reg_xm(FAR struct lsm9ds0_dev_t *priv,
                                  lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_INT_GEN_1_REG;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = (config_data->aoi1 ? (1 << 7) : 0);
  value |= (config_data->six_D1 ? (1 << 6) : 0);
  value |= (config_data->int1_gen_high ? 0x2A : 0x15);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set int_gen_1_reg");
      return ERROR;
    }

  regaddr = LSM9DS0_INT_GEN_2_REG;
  value = (config_data->aoi2 ? (1 << 7) : 0);
  value |= (config_data->six_D2 ? (1 << 6) : 0);
  value |= (config_data->int2_gen_high ? 0x2A : 0x15);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set int_gen_2_reg");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_int_gen_ths_xm(FAR struct lsm9ds0_dev_t *priv,
                                  lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_INT_GEN_1_THS;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = config_data->int1_gen_ths & 0x7F;

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set int_gen_1_ths");
      return ERROR;
    }

  regaddr = LSM9DS0_INT_GEN_2_THS;
  value = config_data->int2_gen_ths & 0x7F;

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set int_gen_2_ths");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_int_gen_duration_xm(FAR struct lsm9ds0_dev_t *priv,
                                       lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_INT_GEN_1_DURATION;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = config_data->int1_gen_duration & 0x7F;

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set int_gen_1_duration");
      return ERROR;
    }

  regaddr = LSM9DS0_INT_GEN_2_DURATION;
  value = config_data->int2_gen_duration & 0x7F;

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot set int_gen_2_duration");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_config_acc_magn(FAR struct lsm9ds0_dev_t *priv,
                                   lsm9ds0_xm_config_data_t * config_data)
{
  int ret = OK;

  ret = lsm9ds0_set_int_ctrl_reg_m(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_set_int_ths_m(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_set_offset_regs_m(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_set_ctrl_reg0_xm(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_set_ctrl_reg1_xm(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_set_ctrl_reg2_xm(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_set_ctrl_reg3_xm(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_set_ctrl_reg4_xm(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_set_ctrl_reg5_xm(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_set_ctrl_reg6_xm(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_set_ctrl_reg7_xm(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_set_fifo_ctrl_reg_xm(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_int_gen_reg_xm(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_int_gen_ths_xm(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_int_gen_duration_xm(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

#ifdef CONFIG_TRACES_GAM9AXEL_SENS
  lsm9ds0_debug_sens_regs_xm(priv);
#endif

  return ret;
}

static int lsm9ds0_acc_magn_read_raw_data(FAR struct lsm9ds0_dev_t *priv,
                                          lsm9ds0_acc_magn_raw_data_t *
                                          raw_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_OUT_X_L_M;
  uint8_t value = 0;
  int i = 0;

  for (i = 0; i < 3; i++)
    {
      ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
      if (ret < 0)
        {
          perror("Cannot read out_*_l_m");
          return ERROR;
        }

      raw_data->magn_xyz[i] = value;

      regaddr++;

      ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
      if (ret < 0)
        {
          perror("Cannot read out_*_h_m");
          return ERROR;
        }

      raw_data->magn_xyz[i] |= (value << 8);

      regaddr++;
    }

  regaddr = LSM9DS0_OUT_X_L_A;

  for (i = 0; i < 3; i++)
    {
      ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
      if (ret < 0)
        {
          perror("Cannot read out_*_l_a");
          return ERROR;
        }

      raw_data->acc_xyz[i] = value;

      regaddr++;

      ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
      if (ret < 0)
        {
          perror("Cannot read out_*_h_a");
          return ERROR;
        }

      raw_data->acc_xyz[i] |= (value << 8);

      regaddr++;
    }

  return ret;
}

static int lsm9ds0_acc_magn_read_status_data(FAR struct lsm9ds0_dev_t *priv,
                                             lsm9ds0_acc_magn_sts_data_t *
                                             sts_data)
{
  int ret = OK;
  uint8_t regaddr = 0;
  uint8_t value = 0;

#if 1
  regaddr = LSM9DS0_STATUS_REG_M;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot read status_reg_m");
      return ERROR;
    }
  sts_data->sts_reg_m = value;
#endif
  regaddr = LSM9DS0_INT_SRC_REG_M;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot read int_src_reg_m");
      return ERROR;
    }
  sts_data->int_src_reg_m = value;
#if 1
  regaddr = LSM9DS0_STATUS_REG_A;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot read status_reg_a");
      return ERROR;
    }
  sts_data->sts_reg_a = value;
#endif
  regaddr = LSM9DS0_INT_GEN_1_SRC;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot read int_gen_1_src");
      return ERROR;
    }
  sts_data->int_gen_1_src = value;

  regaddr = LSM9DS0_INT_GEN_2_SRC;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_ACC_MAGN);
  if (ret < 0)
    {
      perror("Cannot read int_gen_2_src");
      return ERROR;
    }
  sts_data->int_gen_2_src = value;

  return ret;
}

static int lsm9ds0_set_int1_cfg_g(FAR struct lsm9ds0_dev_t *priv,
                                  lsm9ds0_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_INT1_CFG_G;
  uint8_t value = 0;
  uint8_t cmd[2] = { 0 };

  value = (config_data->latch_int ? (1 << 6) : 0);
  value |= (config_data->is_high ? 0x2A : 0x15);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot set int1_cfg");
      return ERROR;
    }

  regaddr = LSM9DS0_INT1_TSH_XH_G;
  value = (uint8_t) ((config_data->x_threshold >> 8) & 0x7F);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot set int1_ths_xh_g");
      return ERROR;
    }

  regaddr = LSM9DS0_INT1_TSH_XL_G;
  value = (uint8_t) (config_data->x_threshold & 0xFF);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot set int1_ths_xl_g");
      return ERROR;
    }

  regaddr = LSM9DS0_INT1_TSH_YH_G;
  value = (uint8_t) ((config_data->y_threshold >> 8) & 0x7F);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot set int1_ths_yh_g");
      return ERROR;
    }

  regaddr = LSM9DS0_INT1_TSH_YL_G;
  value = (uint8_t) (config_data->y_threshold & 0xFF);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot set int1_ths_yl_g");
      return ERROR;
    }

  regaddr = LSM9DS0_INT1_TSH_ZH_G;
  value = (uint8_t) ((config_data->z_threshold >> 8) & 0x7F);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot set int1_ths_zh_g");
      return ERROR;
    }

  regaddr = LSM9DS0_INT1_TSH_ZL_G;
  value = (uint8_t) (config_data->z_threshold & 0xFF);

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot set int1_ths_zl_g");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_set_int1_duration_g(FAR struct lsm9ds0_dev_t *priv,
                                       lsm9ds0_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_INT1_DURATION_G;
  uint8_t value = 0x0;          /* Generate interrupt right away */
  uint8_t cmd[2] = { 0 };

  if (config_data->interrupt_duration)
    {
      value = (1 << 7);         /* if duration is greater than 0, the wait bit
                                 * will be asserted */
    }
  value |= config_data->interrupt_duration;

  cmd[0] = regaddr;
  cmd[1] = value;

  ret = lsm9ds0_write_reg8(priv, cmd, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot set int1_duration");
      return ERROR;
    }

  return ret;
}

static int lsm9ds0_config_gyro(FAR struct lsm9ds0_dev_t *priv,
                               lsm9ds0_gyro_config_data_t * config_data)
{
  int ret = OK;

  ret = lsm9ds0_set_ctrl_reg1_g(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_set_ctrl_reg2_g(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  /* config_data variable left for future compatibility */

  ret = lsm9ds0_set_ctrl_reg3_g(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_set_ctrl_reg4_g(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  ret = lsm9ds0_set_ctrl_reg5_g(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

  if (config_data->fifo_en)
    {
      ret = lsm9ds0_set_fifo_ctrl_reg_g(priv, config_data);
      if (ret < 0)
        {
          return ERROR;
        }
    }

  if (config_data->threshold_int)
    {
      ret = lsm9ds0_set_int1_cfg_g(priv, config_data);
      if (ret < 0)
        {
          return ERROR;
        }
    }

  ret = lsm9ds0_set_int1_duration_g(priv, config_data);
  if (ret < 0)
    {
      return ERROR;
    }

#ifdef CONFIG_TRACES_GAM9AXEL_SENS
  ret = lsm9ds0_debug_sens_regs_gyro(priv);
  if (ret < 0)
    {
      return ERROR;
    }
#endif

  return ret;
}

static int lsm9ds0_read_raw_gyro(FAR struct lsm9ds0_dev_t *priv,
                                 lsm9ds0_gyro_raw_data_t * raw_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_OUT_X_L_G;
  uint8_t bits[6] = { 0 };
  int i = 0;

  for (i = 0; i < 6; i++)
    {
      ret = lsm9ds0_read_reg8(priv, &regaddr, &bits[i], LSM9DS0_ADDR_TYPE_GYRO);
      if (ret < 0)
        {
          perror("Cannot read axes information");
          return ERROR;
        }
      regaddr++;
    }

  raw_data->out_x_g = bits[0] | (bits[1] << 8);
  raw_data->out_y_g = bits[2] | (bits[3] << 8);
  raw_data->out_z_g = bits[4] | (bits[5] << 8);

  return ret;
}

static int lsm9ds0_read_sts_bits_gyro(FAR struct lsm9ds0_dev_t *priv,
                                      lsm9ds0_gyro_int_info_t * int_info)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS0_FIFO_SRC_REG_G;
  uint8_t value = 0;

  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot read fifo_src_reg_g");
      return ERROR;
    }
  int_info->fifo_src_data = value;

  regaddr = LSM9DS0_INT1_SRC_G;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot read int1_src_g");
      return ERROR;
    }
  int_info->int1_sts_data = value;

  regaddr = LSM9DS0_STATUS_REG_G;
  ret = lsm9ds0_read_reg8(priv, &regaddr, &value, LSM9DS0_ADDR_TYPE_GYRO);
  if (ret < 0)
    {
      perror("Cannot read status_reg_g");
      return ERROR;
    }
  int_info->status_reg_data = value;

  return ret;
}

static int lsm9ds0_open(FAR struct file *filep)
{
  int ret = OK;

  lsm9ds0_lldbg("Sensor is powered on\n");

  return ret;
}

static int lsm9ds0_close(FAR struct file *filep)
{
  lsm9ds0_lldbg("CLOSED\n");
  return OK;
}

static ssize_t lsm9ds0_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  ssize_t length = 0;
  return length;
}

static ssize_t lsm9ds0_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  ssize_t length = 0;
  return length;
}

static int lsm9ds0_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm9ds0_dev_t *priv = inode->i_private;
  int ret = 0;

  switch (cmd)
    {
    case LSM9DS0_IOC_WHO_AM_I:
      ret = lsm9ds0_ask_who_am_i(priv, (lsm9ds0_who_am_i_t *) arg);
      break;
    case LSM9DS0_IOC_CONFIG_GYRO:
      ret = lsm9ds0_config_gyro(priv, (lsm9ds0_gyro_config_data_t *) arg);
      break;
    case LSM9DS0_IOC_READ_RAW_GYRO:
      ret = lsm9ds0_read_raw_gyro(priv, (lsm9ds0_gyro_raw_data_t *) arg);
      break;
    case LSM9DS0_IOC_READ_STS_BITS_GYRO:
      ret = lsm9ds0_read_sts_bits_gyro(priv, (lsm9ds0_gyro_int_info_t *) arg);
      break;
    case LSM9DS0_IOC_RST_FIFO_GYRO:
      ret = lsm9ds0_set_fifo_ctrl_reg_g(priv, (lsm9ds0_gyro_config_data_t *) arg);
      break;
    case LSM9DS0_IOC_CONFIG_ACC_MAGN:
      ret = lsm9ds0_config_acc_magn(priv, (lsm9ds0_xm_config_data_t *) arg);
      break;
    case LSM9DS0_IOC_READ_RAW_ACC_MAGN:
      ret = lsm9ds0_acc_magn_read_raw_data(priv,
                                          (lsm9ds0_acc_magn_raw_data_t *) arg);
      break;
    case LSM9DS0_IOC_READ_STS_BITS_ACC_MAGN:
      ret = lsm9ds0_acc_magn_read_status_data(priv,
                                             (lsm9ds0_acc_magn_sts_data_t *) arg);
      break;
    case LSM9DS0_IOC_RST_FIFO_ACC_MAGN:
      ret = lsm9ds0_set_fifo_ctrl_reg_xm(priv, (lsm9ds0_xm_config_data_t *) arg);
      break;
    default:
      ret = -EINVAL;
      break;
    }

  return ret;
}

static void lsm9ds0_notify(FAR struct lsm9ds0_dev_t *priv)
{
  DEBUGASSERT(priv != NULL);

#ifndef CONFIG_DISABLE_POLL
  int i;
#endif

  /* If there are threads waiting on poll() for LIS2DH data to become
   * available, * then wake them up now.  NOTE: we wake up all waiting threads
   * because we * do not know that they are going to do.  If they all try to
   * read the data, * then some make end up blocking after all. */
#ifndef CONFIG_DISABLE_POLL
  for (i = 0; i < CONFIG_GAM9AXEL_SENS_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          lsm9ds0_lldbg("Report events: %02x\n", fds->revents);
          sem_post(fds->sem);
          priv->int_pending = false;
        }
    }
#endif
}

#ifndef CONFIG_DISABLE_POLL
static int lsm9ds0_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode *inode;
  FAR struct lsm9ds0_dev_t *priv;
  int ret = OK;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct lsm9ds0_dev_t *)inode->i_private;

  if (ret < 0)
    {
      ret = -EINTR;
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */
      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          return ret;
        }

      /* This is a request to set up the poll.  Find an available slot for the
       * poll structure reference */
      for (i = 0; i < CONFIG_GAM9AXEL_SENS_NPOLLWAITERS; i++)
        {
          /* Find an available slot */
          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */
              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_GAM9AXEL_SENS_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          return ret;
        }
      if (priv->int_pending)
        {
          lsm9ds0_notify(priv);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */
      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */
      *slot = NULL;
      fds->priv = NULL;
    }

  return ret;
}
#endif

static int lsm9ds0_int_handler(int irq, FAR void *context)
{
  g_9axel_data->int_pending = true;
  lsm9ds0_notify(g_9axel_data);
  lsm9ds0_lldbg("LSM9DS0 interrupt\n");

  return OK;
}

int lsm9ds0_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                     uint8_t * addr, lsm9ds0_config_t * config)
{
  int ret = 0;
  struct lsm9ds0_dev_t *priv;

  priv = (struct lsm9ds0_dev_t *)kmm_zalloc(sizeof(struct lsm9ds0_dev_t));

  if (!priv)
    {
      perror("Memory cannot be allocated for LSM9DS0 sensor");
      return -ENOMEM;
    }

  g_9axel_data = priv;
  priv->acc_addr = addr[0];
  priv->gyro_addr = addr[1];
  priv->i2c = i2c;
  priv->config = config;
  sem_init(&priv->devsem, 0, 0);
  sem_init(&priv->st_sem, 0, 1);

  ret = register_driver(devpath, &g_9axelsops, 0666, priv);

  lsm9ds0_dbg("Registered with %d\n", ret);

  if (ret < 0)
    {
      kmm_free(priv);
      perror("Error occurred during the driver registering");
      return ERROR;
    }

  priv->config->irq_clear(priv->config);
  priv->config->irq_attach(priv->config, lsm9ds0_int_handler);
  priv->config->irq_enable(priv->config, true);

  return OK;
}
