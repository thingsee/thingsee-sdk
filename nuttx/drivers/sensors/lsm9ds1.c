/****************************************************************************
 * drivers/sensors/lsm9ds1.c
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

#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/i2c.h>
#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <nuttx/kmalloc.h>
#include <nuttx/random.h>

#include <nuttx/sensors/lsm9ds1.h>

#ifdef CONFIG_TRACES_LSM9DS1_SENS
#  define lsm9ds1_dbg(x, ...)   dbg(x, ##__VA_ARGS__)
#  define lsm9ds1_lldbg(x, ...) lldbg(x, ##__VA_ARGS__)
#else
#  define lsm9ds1_dbg(x, ...)
#  define lsm9ds1_lldbg(x, ...)
#endif

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Device registers */

#define LSM9DS1_ACT_THS 			0x04
#define LSM9DS1_ACT_DUR				0x05
#define LSM9DS1_INT_GEN_CFG_XL			0x06
#define LSM9DS1_INT_GEN_THS_X_XL 		0x07
#define LSM9DS1_INT_GEN_THS_Y_XL 		0x08
#define LSM9DS1_INT_GEN_THS_Z_XL 		0x09
#define LSM9DS1_INT_GEN_DUR_XL 			0x0A
#define LSM9DS1_REFERENCE_G 			0x0B
#define LSM9DS1_INT1_CTRL 			0x0C
#define LSM9DS1_INT2_CTRL 			0x0D
#define LSM9DS1_WHO_AM_I_AG			0x0F
#define LSM9DS1_CTRL_REG1_G 			0x10
#define LSM9DS1_CTRL_REG2_G 			0x11
#define LSM9DS1_CTRL_REG3_G 			0x12
#define LSM9DS1_ORIENT_CFG_G 			0x13
#define LSM9DS1_INT_GEN_SRC_G 			0x14
#define LSM9DS1_OUT_TEMP_L 			0x15
#define LSM9DS1_OUT_TEMP_H 			0x16
/* #define LSM9DS1_STATUS_REG 			0x17 */ /* Unused: duplicate of reg. 0x27 */
#define LSM9DS1_OUT_X_L_G 			0x18
#define LSM9DS1_OUT_X_H_G 			0x19
#define LSM9DS1_OUT_Y_L_G 			0x1A
#define LSM9DS1_OUT_Y_H_G 			0x1B
#define LSM9DS1_OUT_Z_L_G 			0x1C
#define LSM9DS1_OUT_Z_H_G 			0x1D
#define LSM9DS1_CTRL_REG4 			0x1E
#define LSM9DS1_CTRL_REG5_XL 			0x1F
#define LSM9DS1_CTRL_REG6_XL 			0x20
#define LSM9DS1_CTRL_REG7_XL 			0x21
#define LSM9DS1_CTRL_REG8 			0x22
#define LSM9DS1_CTRL_REG9 			0x23
#define LSM9DS1_CTRL_REG10 			0x24
#define LSM9DS1_INT_GEN_SRC_XL 			0x26
#define LSM9DS1_STATUS_REG 			0x27
#define LSM9DS1_OUT_X_L_XL 			0x28
#define LSM9DS1_OUT_X_H_XL 			0x29
#define LSM9DS1_OUT_Y_L_XL 			0x2A
#define LSM9DS1_OUT_Y_H_XL 			0x2B
#define LSM9DS1_OUT_Z_L_XL 			0x2C
#define LSM9DS1_OUT_Z_H_XL 			0x2D
#define LSM9DS1_FIFO_CTRL 			0x2E
#define LSM9DS1_FIFO_SRC 			0x2F
#define LSM9DS1_INT_GEN_CFG_G 			0x30
#define LSM9DS1_INT_GEN_THS_XH_G 		0x31
#define LSM9DS1_INT_GEN_THS_XL_G  		0x32
#define LSM9DS1_INT_GEN_THS_YH_G 		0x33
#define LSM9DS1_INT_GEN_THS_YL_G  		0x34
#define LSM9DS1_INT_GEN_THS_ZH_G 		0x35
#define LSM9DS1_INT_GEN_THS_ZL_G  		0x36
#define LSM9DS1_INT_GEN_DUR_G   		0x37
#define LSM9DS1_OFFSET_X_REG_L_M  		0x05
#define LSM9DS1_OFFSET_X_REG_H_M   		0x06
#define LSM9DS1_OFFSET_Y_REG_L_M  		0x07
#define LSM9DS1_OFFSET_Y_REG_H_M   		0x08
#define LSM9DS1_OFFSET_Z_REG_L_M  		0x09
#define LSM9DS1_OFFSET_Z_REG_H_M   		0x0A
#define LSM9DS1_WHO_AM_I_M    			0x0F
#define LSM9DS1_CTRL_REG1_M 			0x20
#define LSM9DS1_CTRL_REG2_M 			0x21
#define LSM9DS1_CTRL_REG3_M 			0x22
#define LSM9DS1_CTRL_REG4_M 			0x23
#define LSM9DS1_CTRL_REG5_M 			0x24
#define LSM9DS1_STATUS_REG_M 			0x27
#define LSM9DS1_OUT_X_L_M 			0x28
#define LSM9DS1_OUT_X_H_M 			0x29
#define LSM9DS1_OUT_Y_L_M 			0x2A
#define LSM9DS1_OUT_Y_H_M 			0x2B
#define LSM9DS1_OUT_Z_L_M 			0x2C
#define LSM9DS1_OUT_Z_H_M 			0x2D
#define LSM9DS1_INT_CFG_M  			0x30
#define LSM9DS1_INT_SRC_M   			0x31
#define LSM9DS1_INT_THS_L_M    			0x32
#define LSM9DS1_INT_THS_H_M    			0x33

/* Other macros */

#define LSM9DS1_I2C_RETRIES  10

/****************************************************************************
* Private Function Prototypes
*****************************************************************************/

static int lsm9ds1_open(FAR struct file *filep);
static int lsm9ds1_close(FAR struct file *filep);
static ssize_t lsm9ds1_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t lsm9ds1_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int lsm9ds1_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int lsm9ds1_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);
#endif

/****************************************************************************
* Private Data
****************************************************************************/

typedef enum lsm9ds1_addr_type_t
  {
    LSM9DS1_ADDR_TYPE_ACC_GYRO = 0x0,
    LSM9DS1_ADDR_TYPE_MAGN
  } lsm9ds1_addr_type_t;

struct lsm9ds1_dev_t
  {
    struct i2c_dev_s *i2c;
    uint8_t acc_gyro_addr;
    uint8_t magn_addr;
    lsm9ds1_config_t *config;
    sem_t devsem;
    uint8_t cref;
    volatile bool int_pending;
    lsm9ds1_sensor_bias_t bias; /* Biases are calculated once during device open() */
#ifndef CONFIG_DISABLE_POLL
    struct pollfd *fds[CONFIG_LSM9DS1_SENS_NPOLLWAITERS];
#endif
  };

static const struct file_operations g_lsm9ds1 = {
  lsm9ds1_open,
  lsm9ds1_close,
  lsm9ds1_read,
  lsm9ds1_write,
  0,
  lsm9ds1_ioctl,
#ifndef CONFIG_DISABLE_POLL
  lsm9ds1_poll
#endif
};

static struct lsm9ds1_dev_t *g_lsm9ds1_data;

static int lsm9ds1_write_reg8(FAR struct lsm9ds1_dev_t *dev, uint8_t * command,
                              lsm9ds1_addr_type_t addr_type)
{
  struct i2c_msg_s msgv[2] =
  { {
    .addr   = (addr_type == LSM9DS1_ADDR_TYPE_ACC_GYRO) ? dev->acc_gyro_addr : dev->magn_addr,
    .flags  = 0,
    .buffer = &command[0],
    .length = 1
  }, {
    .addr   = (addr_type == LSM9DS1_ADDR_TYPE_ACC_GYRO) ? dev->acc_gyro_addr : dev->magn_addr,
    .flags  = I2C_M_NORESTART,
    .buffer = &command[1],
    .length = 1
  } };
  int ret = -EIO;
  int retries;

  for (retries = 0; retries < LSM9DS1_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(dev->i2c, msgv, 2);
      if (ret >= 0)
        {
          return 0;
        }
    }

  return ret;
}

static int lsm9ds1_read_reg8(struct lsm9ds1_dev_t *dev, uint8_t * regaddr,
                             uint8_t * value, lsm9ds1_addr_type_t addr_type)
{
  struct i2c_msg_s msgv[2] =
  { {
    .addr   = (addr_type == LSM9DS1_ADDR_TYPE_ACC_GYRO) ? dev->acc_gyro_addr : dev->magn_addr,
    .flags  = 0,
    .buffer = regaddr,
    .length = 1
  }, {
    .addr   = (addr_type == LSM9DS1_ADDR_TYPE_ACC_GYRO) ? dev->acc_gyro_addr : dev->magn_addr,
    .flags  = I2C_M_READ,
    .buffer = value,
    .length = 1
  } };
  int ret = -EIO;
  int retries;

  for (retries = 0; retries < LSM9DS1_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(dev->i2c, msgv, 2);
      if (ret >= 0)
        {
          return 0;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */
#ifdef CONFIG_I2C_RESET
          ret = up_i2creset(dev->i2c);
          if (ret < 0)
            {
              lsm9ds1_dbg("up_i2creset failed: %d\n", ret);
              return ret;
            }
#endif
          continue;
        }
    }

  return ret;
}

#ifdef CONFIG_TRACES_LSM9DS1_SENS
static int lsm9ds1_debug_regs_gyro(FAR struct lsm9ds1_dev_t *priv)
{
  int ret = OK;
  uint8_t regaddr;
  uint8_t value = 0;

  lsm9ds1_lldbg("============================ REGS ============================\n");

  regaddr = LSM9DS1_CTRL_REG1_G;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("CTRL_REG1_G: 0x%02X\n", value);

  regaddr = LSM9DS1_CTRL_REG2_G;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("CTRL_REG2_G: 0x%02X\n", value);

  regaddr = LSM9DS1_CTRL_REG3_G;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("CTRL_REG3_G: 0x%02X\n", value);

  regaddr = LSM9DS1_CTRL_REG4;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("CTRL_REG4: 0x%02X\n", value);

  regaddr = LSM9DS1_CTRL_REG5_XL;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("CTRL_REG5_XL: 0x%02X\n", value);

  regaddr = LSM9DS1_CTRL_REG6_XL;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("CTRL_REG6_XL: 0x%02X\n", value);

  regaddr = LSM9DS1_CTRL_REG7_XL;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("CTRL_REG7_XL: 0x%02X\n", value);

  regaddr = LSM9DS1_CTRL_REG8;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("CTRL_REG8: 0x%02X\n", value);

  regaddr = LSM9DS1_CTRL_REG9;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("CTRL_REG9: 0x%02X\n", value);

  regaddr = LSM9DS1_FIFO_CTRL;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("FIFO_CTRL: 0x%02X\n", value);

  regaddr = LSM9DS1_INT_GEN_CFG_G;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("INT_GEN_CFG_G: 0x%02X\n", value);

  regaddr = LSM9DS1_INT_GEN_DUR_G;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("INT_GEN_DUR_G: 0x%02X\n", value);

  regaddr = LSM9DS1_INT_GEN_CFG_XL;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("INT_GEN_CFG_XL: 0x%02X\n", value);

  regaddr = LSM9DS1_INT_GEN_DUR_XL;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("INT_GEN_DUR_XL: 0x%02X\n", value);

  lsm9ds1_lldbg("==============================================================\n");

  return ret;
}
#endif

static int lsm9ds1_ask_who_am_i(FAR struct lsm9ds1_dev_t *priv,
                                lsm9ds1_who_am_i_t * who_am_i)
{
  int ret = OK;
  uint8_t regaddr;

  regaddr = LSM9DS1_WHO_AM_I_AG;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &who_am_i->acc_gyro,
                          LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot probe acc/gyro\n");
      return ret;
    }

  regaddr = LSM9DS1_WHO_AM_I_M;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &who_am_i->magn, LSM9DS1_ADDR_TYPE_MAGN);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot probe magnetometer\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_read_raw_triple(FAR struct lsm9ds1_dev_t *priv,
                                   lsm9ds1_raw_data_t * raw_data,
                                   uint8_t regaddr,
                                   lsm9ds1_addr_type_t addr_type)
{
  int ret = OK;
  uint8_t bits[6] = { 0 };
  int i;

  for (i = 0; i < 6; i++)
    {
      ret = lsm9ds1_read_reg8(priv, &regaddr, &bits[i], addr_type);
      if (ret < 0)
        {
          lsm9ds1_lldbg("Cannot read register-pair triple\n");
          return ret;
        }
      regaddr++;
    }

  raw_data->out_x = bits[0] | (bits[1] << 8);
  raw_data->out_y = bits[2] | (bits[3] << 8);
  raw_data->out_z = bits[4] | (bits[5] << 8);

  /* Add something to entropy pool. */

  add_sensor_randomness(((uint32_t)raw_data->out_x << 16) | (raw_data->out_y ^ raw_data->out_z));

  return ret;
}

static int lsm9ds1_read_int16_triple(FAR struct lsm9ds1_dev_t *priv,
                                     int16_t * raw_data, uint8_t regaddr,
                                     lsm9ds1_addr_type_t addr_type)
{
  int ret = OK;
  uint8_t bits[6] = { 0 };
  int i;

  for (i = 0; i < 6; i++)
    {
      ret = lsm9ds1_read_reg8(priv, &regaddr, &bits[i], addr_type);
      if (ret < 0)
        {
          lsm9ds1_lldbg("Cannot read register-pair triple. errno = %d\n", errno);
          return ret;
        }
      regaddr++;
    }

  raw_data[0] = (int16_t) (bits[0] | ((int16_t) bits[1] << 8));
  raw_data[1] = (int16_t) (bits[2] | ((int16_t) bits[3] << 8));
  raw_data[2] = (int16_t) (bits[4] | ((int16_t) bits[5] << 8));

  return ret;
}

static int lsm9ds1_write_int16_triple(FAR struct lsm9ds1_dev_t *priv,
                                      int16_t * raw_data, uint8_t regaddr,
                                      lsm9ds1_addr_type_t addr_type)
{
  int ret = OK;
  uint8_t bits[6];
  uint8_t cmd[2];
  int i;

  bits[0] = raw_data[0] & 0xFF;
  bits[1] = (raw_data[0] >> 8) & 0xFF;
  bits[2] = raw_data[1] & 0xFF;
  bits[3] = (raw_data[1] >> 8) & 0xFF;
  bits[4] = raw_data[2] & 0xFF;
  bits[5] = (raw_data[2] >> 8) & 0xFF;

  for (i = 0; i < 6; i++)
    {
      cmd[0] = regaddr;
      cmd[1] = bits[i];
      ret = lsm9ds1_write_reg8(priv, cmd, addr_type);
      if (ret < 0)
        {
          lsm9ds1_lldbg("Cannot write register-pair triple\n");
          return ret;
        }
      regaddr++;
    }

  return ret;
}

static inline int
lsm9ds1_read_raw_gyro(FAR struct lsm9ds1_dev_t *priv,
                      lsm9ds1_raw_data_t * raw_data)
{
  return lsm9ds1_read_raw_triple(priv, raw_data, LSM9DS1_OUT_X_L_G,
                                 LSM9DS1_ADDR_TYPE_ACC_GYRO);
}

static inline int
lsm9ds1_read_raw_xl(FAR struct lsm9ds1_dev_t *priv,
                    lsm9ds1_raw_data_t * raw_data)
{
  return lsm9ds1_read_raw_triple(priv, raw_data, LSM9DS1_OUT_X_L_XL,
                                 LSM9DS1_ADDR_TYPE_ACC_GYRO);
}

static inline int
lsm9ds1_read_raw_mag(FAR struct lsm9ds1_dev_t *priv,
                     lsm9ds1_raw_data_t * raw_data)
{
  return lsm9ds1_read_raw_triple(priv, raw_data, LSM9DS1_OUT_X_L_M,
                                 LSM9DS1_ADDR_TYPE_MAGN);
}

static int lsm9ds1_read_raw_temp(FAR struct lsm9ds1_dev_t *priv,
                                 int16_t * raw_data)
{
  int ret = OK;
  uint8_t bits[2] = { 0 };
  uint8_t regaddr = LSM9DS1_OUT_TEMP_L;
  int i;

  for (i = 0; i < 2; i++)
    {
      ret =
        lsm9ds1_read_reg8(priv, &regaddr, &bits[i], LSM9DS1_ADDR_TYPE_ACC_GYRO);
      if (ret < 0)
        {
          lsm9ds1_lldbg("Cannot read temperature registers\n");
          return ret;
        }
      regaddr++;
    }

  *raw_data = bits[0] | ((int16_t) bits[1] << 8);
  return ret;
}

static int lsm9ds1_read_status_gyro(FAR struct lsm9ds1_dev_t *priv,
                                    lsm9ds1_gyro_status_t * int_info)
{
  int ret = OK;
  uint8_t regaddr;
  uint8_t value;

  regaddr = LSM9DS1_FIFO_SRC;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot read LSM9DS1_FIFO_SRC\n");
      return ret;
    }
  int_info->fifo_src_data = value;

  regaddr = LSM9DS1_INT_GEN_SRC_G;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot read LSM9DS1_INT_GEN_SRC_G\n");
      return ret;
    }
  int_info->int_gyro_sts_data = value;

  regaddr = LSM9DS1_INT_GEN_SRC_XL;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot read LSM9DS1_INT_GEN_SRC_XL\n");
      return ret;
    }
  int_info->int_xl_sts_data = value;

  regaddr = LSM9DS1_STATUS_REG;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot read LSM9DS1_STATUS_REG\n");
      return ret;
    }
  int_info->status_reg_data = value;

  return ret;
}

static int lsm9ds1_set_ctrl_reg1_g(FAR struct lsm9ds1_dev_t *priv,
                                   lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_CTRL_REG1_G;
  uint8_t value;
  uint8_t cmd[2];

  if (config_data->state == LSM9DS1_GYRO_STATE_POWER_DOWN)
    value = 0;
  else
    value = (config_data->odr << 5) | (config_data->scale << 3) | config_data->bwm;

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_CTRL_REG1_G\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_ctrl_reg2_g(FAR struct lsm9ds1_dev_t *priv,
                                   lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;

  /* Output selection for filters: not used. */
  return ret;
}

static int lsm9ds1_set_ctrl_reg3_g(FAR struct lsm9ds1_dev_t *priv,
                                   lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_CTRL_REG3_G;
  uint8_t value;
  uint8_t cmd[2];

  /* Low power mode enable */
  value = (config_data->state == LSM9DS1_GYRO_STATE_LOW_POWER) ? (1 << 7) : 0;
  /* High-pass filter enable */
  value |= (config_data->hpen ? (1 << 6) : 0);
  /* High-pass filter cut-off frequency */
  value |= (config_data->hpcf & 0x0F);

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_CTRL_REG3_G\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_ctrl_reg4(FAR struct lsm9ds1_dev_t *priv,
                                 lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_CTRL_REG4;
  uint8_t value;
  uint8_t cmd[2];

  /* Is XL interrupt latched */
  value = config_data->latch_int ? (1 << 1) : 0;
  value |= 0x38;

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_CTRL_REG4\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_ctrl_reg5_xl(FAR struct lsm9ds1_dev_t *priv,
                                    lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;
  /* Accelerometer decimation and output enable: not used */
  return ret;
}

static int lsm9ds1_set_ctrl_reg6_xl(FAR struct lsm9ds1_dev_t *priv,
                                    lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_CTRL_REG6_XL;
  uint8_t value;
  uint8_t cmd[2];

  value = (config_data->xl_odr << 5) | (config_data->xl_scale << 3);
  /* TODO: Currently xl_bwm is ignored and we use HW default selected by xl_odr */

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_CTRL_REG6_XL\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_ctrl_reg7_xl(FAR struct lsm9ds1_dev_t *priv,
                                    lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;
  /* TODO: Only makes sense for accelerometer-only mode? */
  return ret;
}

static int lsm9ds1_set_ctrl_reg8(FAR struct lsm9ds1_dev_t *priv,
                                 lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_CTRL_REG8;
  uint8_t value;
  uint8_t cmd[2];

  value = (config_data->is_reboot ? (1 << 7) : 0);
  value |= (config_data->bdu ? (1 << 6) : 0);
  value |= (1 << 2);

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_CTRL_REG8\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_ctrl_reg9(FAR struct lsm9ds1_dev_t *priv,
                                 lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_CTRL_REG9;
  uint8_t value = (config_data->fifo_en ? (1 << 1) : 0);
  uint8_t cmd[2];

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_CTRL_REG9\n");
      return ret;
    }

  return ret;
}

/* Type of selftest to perform */

typedef enum lsm9ds1_self_test_t
  {
    LSM9DS1_ST_NONE = 0x0,
    LSM9DS1_ST_XL,
    LSM9DS1_ST_G = 0x4,
    LSM9DS1_ST_BOTH,
  } lsm9ds1_self_test_t;

static int lsm9ds1_set_ctrl_reg10(FAR struct lsm9ds1_dev_t *priv,
                                  lsm9ds1_self_test_t st)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_CTRL_REG10;
  uint8_t value = st;
  uint8_t cmd[2];

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_CTRL_REG10\n");
      return ret;
    }

  return ret;
}

static inline int lsm9ds1_selftest_enable(FAR struct lsm9ds1_dev_t *priv)
{
  return lsm9ds1_set_ctrl_reg10(priv, LSM9DS1_ST_BOTH);
}

static inline int lsm9ds1_selftest_disable(FAR struct lsm9ds1_dev_t *priv)
{
  return lsm9ds1_set_ctrl_reg10(priv, LSM9DS1_ST_NONE);
}

static int lsm9ds1_set_fifo_ctrl_reg(FAR struct lsm9ds1_dev_t *priv,
                                     lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_FIFO_CTRL;
  uint8_t value;
  uint8_t cmd[2];

  value = (config_data->fifo_samples_nbr < 32) ? config_data->fifo_samples_nbr : 31;
  value |= config_data->fifo_mode << 5;

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_FIFO_CTRL\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_act_ths(FAR struct lsm9ds1_dev_t *priv, uint8_t value)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_ACT_THS;
  uint8_t cmd[2];
  uint8_t oldvalue;

  ret = lsm9ds1_read_reg8(priv, &regaddr, &oldvalue, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot read LSM9DS1_ACT_THS\n");
      return ret;
    }

  cmd[0] = regaddr;
  cmd[1] = (oldvalue & ~0x7F) | (value & 0x7F);
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_ACT_THS\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_int_cfg_g(FAR struct lsm9ds1_dev_t *priv,
                                 lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_INT_GEN_CFG_G;
  uint8_t value;
  uint8_t cmd[2];

  value = config_data->latch_int ? (1 << 6) : 0;
  value |= (config_data->is_high ? 0x2A : 0x15);

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_INT_GEN_CFG_G\n");
      return ret;
    }

  regaddr = LSM9DS1_INT_GEN_THS_XH_G;
  value = (uint8_t) ((config_data->x_threshold >> 8) & 0x7F);

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_INT_GEN_THS\n");
      return ret;
    }

  regaddr = LSM9DS1_INT_GEN_THS_XL_G;
  value = (uint8_t) (config_data->x_threshold & 0xFF);

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_INT_GEN_THS\n");
      return ret;
    }

  regaddr = LSM9DS1_INT_GEN_THS_YH_G;
  value = (uint8_t) ((config_data->y_threshold >> 8) & 0x7F);

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_INT_GEN_THS\n");
      return ret;
    }

  regaddr = LSM9DS1_INT_GEN_THS_YL_G;
  value = (uint8_t) (config_data->y_threshold & 0xFF);

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_INT_GEN_THS\n");
      return ret;
    }

  regaddr = LSM9DS1_INT_GEN_THS_ZH_G;
  value = (uint8_t) ((config_data->z_threshold >> 8) & 0x7F);

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_INT_GEN_THS\n");
      return ret;
    }

  regaddr = LSM9DS1_INT_GEN_THS_ZL_G;
  value = (uint8_t) (config_data->z_threshold & 0xFF);

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_INT_GEN_THS\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_int_cfg_xl(FAR struct lsm9ds1_dev_t *priv,
                                  lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_INT_GEN_CFG_XL;
  uint8_t value;
  uint8_t cmd[2];

  value = config_data->is_high ? 0x2A : 0x15;

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_INT_GEN_CFG_XL\n");
      return ret;
    }

  regaddr = LSM9DS1_INT_GEN_THS_X_XL;
  value = config_data->x_threshold_xl;

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_INT_GEN_THS\n");
      return ret;
    }

  regaddr = LSM9DS1_INT_GEN_THS_Y_XL;
  value = config_data->y_threshold_xl;

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_INT_GEN_THS\n");
      return ret;
    }

  regaddr = LSM9DS1_INT_GEN_THS_Z_XL;
  value = config_data->z_threshold_xl;

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_INT_GEN_THS\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_int_gen_dur_g(FAR struct lsm9ds1_dev_t *priv,
                                     lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_INT_GEN_DUR_G;
  uint8_t value = 0;            /* Generate interrupt right away */
  uint8_t cmd[2];

  if (config_data->int_gen_dur_g)
    {
      /* if duration is greater than 0, the wait bit will be asserted */
      value = (1 << 7) | config_data->int_gen_dur_g;
    }

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_INT_GEN_DUR_G\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_int_gen_dur_xl(FAR struct lsm9ds1_dev_t *priv,
                                      lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_INT_GEN_DUR_XL;
  uint8_t value = 0;            /* Generate interrupt right away */
  uint8_t cmd[2];

  if (config_data->int_gen_dur_xl)
    {
      /* if duration is greater than 0, the wait bit will be asserted */
      value = (1 << 7) | config_data->int_gen_dur_xl;
    }

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_INT_GEN_DUR_XL\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_config_gyro(FAR struct lsm9ds1_dev_t *priv,
                               lsm9ds1_gyro_config_data_t * config_data)
{
  int ret = OK;

  ret = lsm9ds1_set_ctrl_reg1_g(priv, config_data);
  if (ret < 0)
    {
      return ret;
    }

  ret = lsm9ds1_set_ctrl_reg2_g(priv, config_data);
  if (ret < 0)
    {
      return ret;
    }

  ret = lsm9ds1_set_ctrl_reg3_g(priv, config_data);
  if (ret < 0)
    {
      return ret;
    }

  ret = lsm9ds1_set_ctrl_reg4(priv, config_data);
  if (ret < 0)
    {
      return ret;
    }

  if (config_data->state == LSM9DS1_GYRO_STATE_XL_ONLY)
    {
      ret = lsm9ds1_set_ctrl_reg5_xl(priv, config_data);
      if (ret < 0)
        {
          return ret;
        }

      ret = lsm9ds1_set_ctrl_reg6_xl(priv, config_data);
      if (ret < 0)
        {
          return ret;
        }

      ret = lsm9ds1_set_ctrl_reg7_xl(priv, config_data);
      if (ret < 0)
        {
          return ret;
        }
    }

  ret = lsm9ds1_set_ctrl_reg8(priv, config_data);
  if (ret < 0)
    {
      return ret;
    }

  if (config_data->fifo_en)
    {
      ret = lsm9ds1_set_fifo_ctrl_reg(priv, config_data);
      if (ret < 0)
        {
          return ret;
        }
      ret = lsm9ds1_set_ctrl_reg9(priv, config_data);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (config_data->threshold_int)
    {
      ret = lsm9ds1_set_int_cfg_g(priv, config_data);
      if (ret < 0)
        {
          return ret;
        }
      ret = lsm9ds1_set_int_cfg_xl(priv, config_data);
      if (ret < 0)
        {
          return ret;
        }
    }

  ret = lsm9ds1_set_int_gen_dur_g(priv, config_data);
  if (ret < 0)
    {
      return ret;
    }

  ret = lsm9ds1_set_int_gen_dur_xl(priv, config_data);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_TRACES_LSM9DS1_SENS
  ret = lsm9ds1_debug_regs_gyro(priv);
  if (ret < 0)
    {
      return ret;
    }
#endif

  return ret;
}

/* Calibrate both sub-devices.
 *
 * Note that calibration config is *not* cleared or returned
 * to old values when calibration is done, so user is adviced
 * to configure the device again afterwards.
 */

/* Calibration config for testing the gyroscope + accelerometer. */

static const lsm9ds1_gyro_config_data_t gyro_calibration_config = {
  .state = LSM9DS1_GYRO_STATE_NORMAL,
  .bdu = true,                  /* We need to wait for samples */
  .odr = LSM9DS1_GYRO_ODR_238HZ,
  .bwm = LSM9DS1_GYRO_MODE_HIGH,
  .scale = LSM9DS1_GYRO_SCALE_245DPS,
  .xl_scale = LSM9DS1_XL_SCALE_2G,      /* HW default */
  .fifo_mode = LSM9DS1_FIFO_STREAM,
  .fifo_samples_nbr = 31,
  .fifo_en = true,
};

static const lsm9ds1_gyro_config_data_t gyro_reset_fifo_config = {
  .fifo_mode = LSM9DS1_FIFO_BYPASS,
  .fifo_samples_nbr = 0
};

/* Calibration routine for detecting gyro and accelerator biases. */
static int lsm9ds1_calibrate_gyro(FAR struct lsm9ds1_dev_t *priv)
{
  int ret = OK;
  int i, samples;
  uint8_t regaddr;
  uint8_t value = 0;
  int16_t gyro_sample[3] = { 0, 0, 0 };
  int32_t gyro_bias[3] = { 0, 0, 0 };
  int16_t xl_sample[3] = { 0, 0, 0 };
  int32_t xl_bias[3] = { 0, 0, 0 };

  ret = lsm9ds1_config_gyro(priv,
                            (lsm9ds1_gyro_config_data_t *) &
                            gyro_calibration_config);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set calibration config\n");
      return ret;
    }

  lsm9ds1_lldbg("Gyro calibration: do not move the device\n");
  usleep(900 * 1000);

  /* Read number of stored samples from FIFO. */

  regaddr = LSM9DS1_FIFO_SRC;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot read LSM9DS1_FIFO_SRC\n");
      return ret;
    }
  samples = value & 0x2F;

  for (i = 0; i < samples; i++)
    {
      ret = lsm9ds1_read_int16_triple(priv, gyro_sample, LSM9DS1_OUT_X_L_G,
                                  LSM9DS1_ADDR_TYPE_ACC_GYRO);
      if (ret < 0)
        {
          lsm9ds1_lldbg("Calibration cannot read gyro samples\n");
          return ret;
        }

      ret = lsm9ds1_read_int16_triple(priv, xl_sample, LSM9DS1_OUT_X_L_XL,
                                  LSM9DS1_ADDR_TYPE_ACC_GYRO);
      if (ret < 0)
        {
          lsm9ds1_lldbg("Calibration cannot read xl samples\n");
          return ret;
        }

      /* Sum individual values to get accumulated 32-bit bias values. */

      gyro_bias[0] += gyro_sample[0];
      gyro_bias[1] += gyro_sample[1];
      gyro_bias[2] += gyro_sample[2];
      xl_bias[0] += xl_sample[0];
      xl_bias[1] += xl_sample[1];
      xl_bias[2] += xl_sample[2];
    }

  if (samples > 0)
    {
      gyro_bias[0] /= samples;
      gyro_bias[1] /= samples;
      gyro_bias[2] /= samples;
      xl_bias[0] /= samples;
      xl_bias[1] /= samples;
      xl_bias[2] /= samples;
    }

  /* Accelerometer measures Earth's gravity so the xl_bias[2] should show 1g
   * upward force. We need to remove that from final bias values. */
  enum
    {
      ONE_GEE = 16383,          /* (2^15-1)/2, half of LSM9DS1_XL_SCALE_2G range */
      LOW_GEE = ONE_GEE / 4,    /* Fail limit, somewhat arbitrary */
    };

  if (abs(xl_bias[0]) > LOW_GEE || abs(xl_bias[1]) > LOW_GEE)
    {
      lsm9ds1_lldbg("FAIL: Calibration: device is not horizontal!\n");
      return -EAGAIN;
    }
  else if (abs(xl_bias[2]) < LOW_GEE)
    {
      lsm9ds1_lldbg("FAIL: Calibration: device is in free-fall or in space!\n");
      return -EAGAIN;
    }
  else
    {
      /* Device can be upside-down. */
      if (xl_bias[2] < 0)
        xl_bias[2] += ONE_GEE;
      else if (xl_bias[2] > 0)
        xl_bias[2] -= ONE_GEE;
    }

  lsm9ds1_lldbg("Calibration done:\n");
  lsm9ds1_lldbg("\tgyro bias = [%d, %d, %d]\n",
                gyro_bias[0], gyro_bias[1], gyro_bias[2]);
  lsm9ds1_lldbg("\txl bias   = [%d, %d, %d]\n",
                xl_bias[0], xl_bias[1], xl_bias[2]);

  /* Store bias values to device */

  priv->bias.gyro_bias[0] = (int16_t) gyro_bias[0];
  priv->bias.gyro_bias[1] = (int16_t) gyro_bias[1];
  priv->bias.gyro_bias[2] = (int16_t) gyro_bias[2];
  priv->bias.xl_bias[0] = (int16_t) xl_bias[0];
  priv->bias.xl_bias[1] = (int16_t) xl_bias[1];
  priv->bias.xl_bias[2] = (int16_t) xl_bias[2];

  /* Reset fifo and set it to bypass mode */
  ret = lsm9ds1_set_fifo_ctrl_reg(priv,
                                  (lsm9ds1_gyro_config_data_t *) &
                                  gyro_reset_fifo_config);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot reset FIFO after calibration\n");
      return ret;
    }

  return ret;
}

/******** Magnetic sensor ****************************************************************/

#ifdef CONFIG_TRACES_LSM9DS1_SENS
static int lsm9ds1_debug_regs_mag(FAR struct lsm9ds1_dev_t *priv)
{
  int ret = OK;
  uint8_t regaddr;
  uint8_t value = 0;

  lsm9ds1_lldbg("============================ REGS ============================\n");

  regaddr = LSM9DS1_CTRL_REG1_M;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_MAGN);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("CTRL_REG1_M: 0x%02X\n", value);

  regaddr = LSM9DS1_CTRL_REG2_M;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_MAGN);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("CTRL_REG2_M: 0x%02X\n", value);

  regaddr = LSM9DS1_CTRL_REG3_M;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_MAGN);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("CTRL_REG3_M: 0x%02X\n", value);

  regaddr = LSM9DS1_CTRL_REG4_M;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_MAGN);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("CTRL_REG4_M: 0x%02X\n", value);

  regaddr = LSM9DS1_CTRL_REG5_M;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_MAGN);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("CTRL_REG5_M: 0x%02X\n", value);

  regaddr = LSM9DS1_INT_CFG_M;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_MAGN);
  if (ret < 0)
    {
      return ret;
    }
  lsm9ds1_lldbg("INT_CFG_M: 0x%02X\n", value);

  lsm9ds1_lldbg("==============================================================\n");

  return ret;
}
#endif

static int lsm9ds1_set_ctrl_reg1_m(FAR struct lsm9ds1_dev_t *priv,
                                   lsm9ds1_mag_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_CTRL_REG1_M;
  uint8_t value;
  uint8_t cmd[2];

  value = config_data->mag_odr << 2;
  value |= config_data->mag_xy_opermode << 5;
  value |= (config_data->mag_temp_comp_en ? (1 << 7) : 0);

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_MAGN);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_CTRL_REG1_M\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_ctrl_reg2_m(FAR struct lsm9ds1_dev_t *priv,
                                   lsm9ds1_mag_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_CTRL_REG2_M;
  uint8_t value;
  uint8_t cmd[2];

  value = config_data->mag_scale << 5;
  value |= (config_data->is_reboot ? (1 << 3) : 0);

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_MAGN);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_CTRL_REG2_M\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_ctrl_reg3_m(FAR struct lsm9ds1_dev_t *priv,
                                   bool enable)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_CTRL_REG3_M;
  uint8_t cmd[2];

  cmd[0] = regaddr;
  cmd[1] = enable ? 0 : 0x3;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_MAGN);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_CTRL_REG3_M\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_ctrl_reg4_m(FAR struct lsm9ds1_dev_t *priv,
                                   lsm9ds1_mag_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_CTRL_REG4_M;
  uint8_t value;
  uint8_t cmd[2];

  value = config_data->mag_z_opermode << 2;

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_MAGN);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_CTRL_REG4_M\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_ctrl_reg5_m(FAR struct lsm9ds1_dev_t *priv,
                                   lsm9ds1_mag_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_CTRL_REG5_M;
  uint8_t value;
  uint8_t cmd[2];

  value = config_data->mag_bdu ? (1 << 6) : 0;

  cmd[0] = regaddr;
  cmd[1] = value;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_MAGN);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_CTRL_REG5_M\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_set_int_cfg_m(FAR struct lsm9ds1_dev_t *priv,
                                 lsm9ds1_mag_config_data_t * config_data)
{
  int ret = OK;
  uint8_t regaddr = LSM9DS1_INT_CFG_M;
  uint8_t cmd[2];

  cmd[0] = regaddr;
  cmd[1] = 0;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_MAGN);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set LSM9DS1_INT_CFG_M\n");
      return ret;
    }

  return ret;
}

static int lsm9ds1_powerdown(FAR struct lsm9ds1_dev_t *priv)
{
  int ret;
  uint8_t regaddr;
  uint8_t value = 0;
  uint8_t cmd[2];

  /* Power-down XL first, in case we were in accelerometer-only mode. */

  regaddr = LSM9DS1_CTRL_REG6_XL;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot read LSM9DS1_CTRL_REG6_XL\n");
      return ret;
    }

  cmd[0] = regaddr;
  cmd[1] = value & 0x1f;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot write LSM9DS1_CTRL_REG6_XL\n");
      return ret;
    }

  /* Power-down Gyro. */

  regaddr = LSM9DS1_CTRL_REG1_G;
  ret = lsm9ds1_read_reg8(priv, &regaddr, &value, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot read LSM9DS1_CTRL_REG1_G\n");
      return ret;
    }

  cmd[0] = regaddr;
  cmd[1] = value & 0x1b;
  ret = lsm9ds1_write_reg8(priv, cmd, LSM9DS1_ADDR_TYPE_ACC_GYRO);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot write LSM9DS1_CTRL_REG1_G\n");
      return ret;
    }

  /* Power-down Magnetometer. */

  ret = lsm9ds1_set_ctrl_reg3_m(priv, false);
  return ret;
}

/* Default magnetometer config if not provided by user. */
static const lsm9ds1_mag_config_data_t mag_default_config = {
  .mag_temp_comp_en = true,
  .mag_bdu = true,
  .mag_odr = LSM9DS1_MAG_ODR_10HZ,
  .mag_scale = LSM9DS1_MAG_SCALE_4GS,
  .mag_xy_opermode = LSM9DS1_MAG_MODE_HIGH,
  .mag_z_opermode = LSM9DS1_MAG_MODE_HIGH,
};

static int lsm9ds1_config_mag(FAR struct lsm9ds1_dev_t *priv,
                              lsm9ds1_mag_config_data_t * config_data)
{
  int ret = OK;

  /* Use reasonable default configuration so user does not have to setup it
   * every time. */
  if (config_data == NULL)
    config_data = (lsm9ds1_mag_config_data_t *) &mag_default_config;

  ret = lsm9ds1_set_ctrl_reg1_m(priv, config_data);
  if (ret < 0)
    {
      return ret;
    }

  ret = lsm9ds1_set_ctrl_reg2_m(priv, config_data);
  if (ret < 0)
    {
      return ret;
    }

  ret = lsm9ds1_set_ctrl_reg3_m(priv, true);
  if (ret < 0)
    {
      return ret;
    }

  ret = lsm9ds1_set_ctrl_reg4_m(priv, config_data);
  if (ret < 0)
    {
      return ret;
    }

  ret = lsm9ds1_set_ctrl_reg5_m(priv, config_data);
  if (ret < 0)
    {
      return ret;
    }

  ret = lsm9ds1_set_int_cfg_m(priv, config_data);
  if (ret < 0)
    {
      return ret;
    }

#ifdef CONFIG_TRACES_LSM9DS1_SENS
  ret = lsm9ds1_debug_regs_mag(priv);
  if (ret < 0)
    {
      return ret;
    }
#endif

  return ret;
}

static int lsm9ds1_set_mag_offset(FAR struct lsm9ds1_dev_t *priv)
{
  int ret;

  ret = lsm9ds1_write_int16_triple(priv, priv->bias.mag_bias, LSM9DS1_OFFSET_X_REG_L_M,
                                   LSM9DS1_ADDR_TYPE_MAGN);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot write magnetometer offset regs.\n");
      return ret;
    }

  return ret;
}

/* Calibration config for testing the magnetometer. */

static const lsm9ds1_mag_config_data_t mag_calibration_config = {
  .mag_temp_comp_en = true,
  .mag_bdu = true,              /* We need to wait for samples */
  .mag_odr = LSM9DS1_MAG_ODR_10HZ,
  .mag_scale = LSM9DS1_MAG_SCALE_4GS,
  .mag_xy_opermode = LSM9DS1_MAG_MODE_HIGH,
  .mag_z_opermode = LSM9DS1_MAG_MODE_LOW,
};

/* Calibration routine for setting the magnetometer's offset registers. */
static int lsm9ds1_calibrate_mag(FAR struct lsm9ds1_dev_t *priv)
{
  int ret = OK;
  int i, j;
  enum { NSAMPLES = 128 };
  int16_t mag_bias[3] = { 0, 0, 0 };
  int16_t mag_sample[3] = { 0, 0, 0 };
  int16_t mag_max[3] = { SHRT_MIN, SHRT_MIN, SHRT_MIN };
  int16_t mag_min[3] = { SHRT_MAX, SHRT_MAX, SHRT_MAX };

  /* Clear magnetometer biases in offset registers. */
  priv->bias.mag_bias[0] = 0;
  priv->bias.mag_bias[1] = 0;
  priv->bias.mag_bias[2] = 0;
  ret = lsm9ds1_set_mag_offset(priv);

  ret = lsm9ds1_config_mag(priv, (lsm9ds1_mag_config_data_t *) &mag_calibration_config);
  if (ret < 0)
    {
      lsm9ds1_lldbg("Cannot set calibration config\n");
      return ret;
    }

  lsm9ds1_lldbg("Magnetometer calibration: wave device in a figure eight until done\n");
  usleep(2000 * 1000);

  for (i = 0; i < NSAMPLES; i++)
    {
      ret = lsm9ds1_read_int16_triple(priv, mag_sample, LSM9DS1_OUT_X_L_M,
                                      LSM9DS1_ADDR_TYPE_MAGN);
      if (ret < 0)
        {
          lsm9ds1_lldbg("Calibration cannot read samples\n");
          return ret;
        }

      for (j = 0; j < 3; j++)
        {
          if (mag_sample[j] > mag_max[j])
            mag_max[j] = mag_sample[j];
          if (mag_sample[j] < mag_min[j])
            mag_min[j] = mag_sample[j];
        }

      usleep(105 * 1000);       /* At 10Hz ODR, new data is available every 100 ms. */
    }

  /* Bias is just the average. */
  mag_bias[0] = ((int32_t) mag_max[0] + mag_min[0]) / 2;
  mag_bias[1] = ((int32_t) mag_max[1] + mag_min[1]) / 2;
  mag_bias[2] = ((int32_t) mag_max[2] + mag_min[2]) / 2;

  lsm9ds1_lldbg("Calibration done:\n");
  lsm9ds1_lldbg("\tmax = [%hd, %hd, %hd], min = [%hd, %hd, %hd]\n",
                mag_max[0], mag_max[1], mag_max[2], mag_min[0], mag_min[1], mag_min[2]);
  lsm9ds1_lldbg("\tbias = [%hd, %hd, %hd]\n", mag_bias[0], mag_bias[1], mag_bias[2]);

  /* Store bias values to device */

  priv->bias.mag_bias[0] = mag_bias[0];
  priv->bias.mag_bias[1] = mag_bias[1];
  priv->bias.mag_bias[2] = mag_bias[2];

  /* Write biases to offset registers. */
  ret = lsm9ds1_set_mag_offset(priv);

  return ret;
}

static int lsm9ds1_open(FAR struct file *filep)
{
  int ret = OK;
  unsigned int use_count;
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm9ds1_dev_t *priv = inode->i_private;

  while (sem_wait(&priv->devsem) != 0)
    {
      assert(errno == EINTR);
    }

  use_count = priv->cref + 1;
  if (use_count == 1)
    {
      /* First user, do power on. */

      ret = priv->config->set_power(priv->config, true);
      if (ret < 0)
        {
          goto out_sem;
        }

      /* Let chip to power up before I2C operations. */

      usleep(100 * 1000);

      /* Try to reset I2C bus as power switch seems to mess it. */
#ifdef CONFIG_I2C_RESET
      ret = up_i2creset(priv->i2c);
      if (ret < 0)
        {
          lsm9ds1_lldbg("up_i2creset failed: %d\n", ret);
        }
#endif

      /* Enable interrupt */

      priv->config->irq_enable(priv->config, true);

      priv->cref = use_count;
    }
  else
    {
      DEBUGASSERT(use_count < UINT8_MAX && use_count > priv->cref);

      priv->cref = use_count;
    }

#ifdef CONFIG_LSM9DS1_SENS_ALWAYS_CALIBRATE
  ret = lsm9ds1_calibrate(priv);
#endif
  lsm9ds1_lldbg("Sensor is powered on\n");

out_sem:
  sem_post(&priv->devsem);

  return ret;
}

static int lsm9ds1_close(FAR struct file *filep)
{
  int use_count;
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm9ds1_dev_t *priv = inode->i_private;

  while (sem_wait(&priv->devsem) != 0)
    {
      assert(errno == EINTR);
    }

  use_count = priv->cref - 1;
  if (use_count == 0)
    {
      /* Disable interrupt */

      priv->config->irq_enable(priv->config, false);

      /* Set chip in low-power mode. This is needed because
         board might share power switch among multiple devices. */

      lsm9ds1_powerdown(priv);

      /* Last user, do power off. */

      (void)priv->config->set_power(priv->config, false);

      priv->cref = use_count;
    }
  else
    {
      DEBUGASSERT(use_count > 0);

      priv->cref = use_count;
    }

  lsm9ds1_lldbg("Sensor is powered off\n");

  sem_post(&priv->devsem);

  return OK;
}

static ssize_t lsm9ds1_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  int ret = OK;
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm9ds1_dev_t *priv = inode->i_private;
  size_t nsamples;
  int i;
  int16_t sample[3];
  int16_t *ptr;
  irqstate_t flags;

  if (buffer == NULL || buflen < 1)
    return -EINVAL;

  ptr = (int16_t *) buffer;
  nsamples = buflen / (3 * sizeof(sample));

  while (sem_wait(&priv->devsem) != 0)
    {
      assert(errno == EINTR);
    }

  for (i = 0; i < nsamples; i++)
    {
      ret = lsm9ds1_read_int16_triple(priv, sample, LSM9DS1_OUT_X_L_G,
                                      LSM9DS1_ADDR_TYPE_ACC_GYRO);
      if (ret < 0)
        {
          lsm9ds1_lldbg("Cannot read samples 1\n");
          goto outsem;
        }
      *ptr++ = sample[0];
      *ptr++ = sample[1];
      *ptr++ = sample[2];
      ret = lsm9ds1_read_int16_triple(priv, sample, LSM9DS1_OUT_X_L_XL,
                                      LSM9DS1_ADDR_TYPE_ACC_GYRO);
      if (ret < 0)
        {
          lsm9ds1_lldbg("Cannot read samples 2\n");
          goto outsem;
        }
      *ptr++ = sample[0];
      *ptr++ = sample[1];
      *ptr++ = sample[2];
      ret = lsm9ds1_read_int16_triple(priv, sample, LSM9DS1_OUT_X_L_M,
                                      LSM9DS1_ADDR_TYPE_MAGN);
      if (ret < 0)
        {
          lsm9ds1_lldbg("Cannot read samples 3\n");
          goto outsem;
        }
      *ptr++ = sample[0];
      *ptr++ = sample[1];
      *ptr++ = sample[2];

      if (i >= 1)
        {
          /* TODO: sleep should be calibrated from current odr, but then again
           * current user-space only reads one sample at a time. */
          usleep(1000);
        }
    }

  flags = irqsave();
  priv->int_pending = false;
  irqrestore(flags);

outsem:
  sem_post(&priv->devsem);
  return (char *)ptr - buffer;
}

static ssize_t lsm9ds1_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  ssize_t length = 0;
  return length;
}

static int lsm9ds1_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lsm9ds1_dev_t *priv = inode->i_private;
  int ret = 0;

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      return -EINTR;
    }

  switch (cmd)
    {
    case LSM9DS1_IOC_WHO_AM_I:
      ret = lsm9ds1_ask_who_am_i(priv, (lsm9ds1_who_am_i_t *) arg);
      break;
    case LSM9DS1_IOC_CALIBRATE_BIAS_GYRO:
        {
          ret = lsm9ds1_calibrate_gyro(priv);
          if (ret < 0)
            {
              goto out_sem;
            }
        }
      *(lsm9ds1_sensor_bias_t *) arg = priv->bias;
      break;
    case LSM9DS1_IOC_CALIBRATE_BIAS_MAG:
        {
          ret = lsm9ds1_calibrate_mag(priv);
          if (ret < 0)
            {
              goto out_sem;
            }
        }
      *(lsm9ds1_sensor_bias_t *) arg = priv->bias;
      break;
    case LSM9DS1_IOC_READ_BIAS:
      *(lsm9ds1_sensor_bias_t *) arg = priv->bias;
      break;
    case LSM9DS1_IOC_WRITE_BIAS:
      priv->bias = *(lsm9ds1_sensor_bias_t *) arg;
      ret = lsm9ds1_set_mag_offset(priv);
      if (ret < 0)
        {
          goto out_sem;
        }
      break;
    case LSM9DS1_IOC_CONFIG_GYRO:
      ret = lsm9ds1_config_gyro(priv, (lsm9ds1_gyro_config_data_t *) arg);
      break;
    case LSM9DS1_IOC_READ_RAW_GYRO:
      ret = lsm9ds1_read_raw_gyro(priv, (lsm9ds1_raw_data_t *) arg);
      break;
    case LSM9DS1_IOC_READ_STATUS_GYRO:
      ret = lsm9ds1_read_status_gyro(priv, (lsm9ds1_gyro_status_t *) arg);
      break;
    case LSM9DS1_IOC_RST_FIFO_GYRO:
      ret = lsm9ds1_set_fifo_ctrl_reg(priv, (lsm9ds1_gyro_config_data_t *) arg);
      break;
    case LSM9DS1_IOC_SET_ACT_THS_GYRO:
      ret = lsm9ds1_set_act_ths(priv, (uint8_t) arg);
      break;
    case LSM9DS1_IOC_READ_RAW_XL:
      ret = lsm9ds1_read_raw_xl(priv, (lsm9ds1_raw_data_t *) arg);
      break;
    case LSM9DS1_IOC_READ_TEMP:
      ret = lsm9ds1_read_raw_temp(priv, (int16_t *) arg);
      break;
    case LSM9DS1_IOC_CONFIG_MAG:
      ret = lsm9ds1_config_mag(priv, (lsm9ds1_mag_config_data_t *) arg);
      break;
    case LSM9DS1_IOC_READ_RAW_MAG:
      ret = lsm9ds1_read_raw_mag(priv, (lsm9ds1_raw_data_t *) arg);
      break;
    case LSM9DS1_IOC_READ_STATUS_MAG:
    default:
      ret = -EINVAL;
      break;
    }

out_sem:
  sem_post(&priv->devsem);

  return ret;
}

#ifndef CONFIG_DISABLE_POLL
static void lsm9ds1_notify(FAR struct lsm9ds1_dev_t *priv)
{
  DEBUGASSERT(priv != NULL);

  int i;

  /* If there are threads waiting on poll() for data to become available, *
   * then wake them up now.  NOTE: we wake up all waiting threads because we *
   * do not know that they are going to do.  If they all try to read the data,
   * * then some make end up blocking after all. */
  for (i = 0; i < CONFIG_LSM9DS1_SENS_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          sem_post(fds->sem);
        }
    }
}

static int lsm9ds1_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode *inode;
  FAR struct lsm9ds1_dev_t *priv;
  int ret = OK;
  int i;
  irqstate_t flags;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct lsm9ds1_dev_t *)inode->i_private;

  ret = sem_wait(&priv->devsem);
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
          goto out;
        }

      /* This is a request to set up the poll.  Find an available slot for the
       * poll structure reference */

      for (i = 0; i < CONFIG_LSM9DS1_SENS_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_LSM9DS1_SENS_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }
      flags = irqsave();
      if (priv->int_pending)
        {
          lsm9ds1_notify(priv);
        }
      irqrestore(flags);
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

out:
  sem_post(&priv->devsem);
  return ret;
}
#endif /* !CONFIG_DISABLE_POLL */

static int lsm9ds1_int_handler(int irq, FAR void *context)
{
  irqstate_t flags;

  flags = irqsave();
  g_lsm9ds1_data->int_pending = true;
#ifndef CONFIG_DISABLE_POLL
  lsm9ds1_notify(g_lsm9ds1_data);
#endif
  irqrestore(flags);

  return OK;
}

int lsm9ds1_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                     uint8_t * addr, lsm9ds1_config_t * config)
{
  int ret = 0;
  struct lsm9ds1_dev_t *priv;

  priv = (struct lsm9ds1_dev_t *)kmm_zalloc(sizeof(struct lsm9ds1_dev_t));

  if (!priv)
    {
      lsm9ds1_dbg("Memory cannot be allocated for LSM9DS1 sensor\n");
      return -ENOMEM;
    }

  g_lsm9ds1_data = priv;
  priv->acc_gyro_addr = addr[0];
  priv->magn_addr = addr[1];
  priv->i2c = i2c;
  priv->config = config;
  sem_init(&priv->devsem, 0, 1);

  ret = register_driver(devpath, &g_lsm9ds1, 0666, priv);

  lsm9ds1_dbg("Registered with %d\n", ret);

  if (ret < 0)
    {
      kmm_free(priv);
      lsm9ds1_dbg("Error occurred during the driver registering\n");
      return ERROR;
    }

  priv->config->irq_clear(priv->config);
  priv->config->irq_attach(priv->config, lsm9ds1_int_handler);
  priv->config->irq_enable(priv->config, false);

  return OK;
}
