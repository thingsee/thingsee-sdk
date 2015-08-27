/****************************************************************************
 * drivers/sensors/sht25.c
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

#include <stdio.h>
#include <debug.h>
#include <stdlib.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/i2c.h>
#include <sys/types.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>
#include <errno.h>
#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>

#include "nuttx/sensors/sht25.h"

#ifdef CONFIG_DEBUG_HUMIDITY_DEV_SHT25
#  define sht25_dbg(x, ...)	dbg(x, ##__VA_ARGS__)
#else
#  define sht25_dbg(x, ...)
#endif

/************************************************************************************
* Private Function Prototypes
************************************************************************************/

static int sht25_open(FAR struct file *filep);
static int sht25_close(FAR struct file *filep);
static ssize_t sht25_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t sht25_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static int sht25_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
* Private Data
****************************************************************************/

struct sht25_dev_t
  {
    struct i2c_dev_s *i2c;
    uint8_t addr;
  };

static const struct file_operations g_humidityops = {
  sht25_open,
  sht25_close,
  sht25_read,
  sht25_write,
  0,
  sht25_ioctl,
#ifndef CONFIG_DISABLE_POLL
  0
#endif
};

static struct sht25_dev_t *humid_data;

static int32_t sht25_write_reg8(struct sht25_dev_t *dev,
                                const uint8_t * command)
{
  int32_t ret = 0;

  ret = I2C_SETADDRESS(dev->i2c, dev->addr, 7);

  if (ret < 0)
    {
      sht25_dbg("Can't set address\n");
      return ret;
    }
  ret = I2C_WRITE(dev->i2c, command, 1);

  if (ret < 0)
    {
      sht25_dbg("Can't write data\n");
      return ret;
    }

  return OK;
}

static int32_t sht25_start_read_reg(struct sht25_dev_t *dev, uint8_t * command)
{
  int32_t ret = 0;
  ret = I2C_SETADDRESS(dev->i2c, dev->addr, 7);

  if (ret < 0)
    {
      perror("Can't set address");
      return ret;
    }

  ret = I2C_WRITE(dev->i2c, command, 1);
  if (ret < 0)
    {
      perror("Can't write data");
      return ret;
    }

  return OK;
}

static int32_t sht25_stop_read_reg(struct sht25_dev_t *dev, uint8_t * value)
{
  int32_t ret = 0;

  ret = I2C_READ(dev->i2c, value, 3);

  if (ret < 0)
    {
      perror("Can't read data");
      return ret;
    }

  return OK;
}

static int32_t sht25_start_read_temperature(struct sht25_dev_t *dev)
{
  uint8_t command = SHT25_TM_NO_HOLDMASTER;

  return sht25_start_read_reg(dev, &command);
}

static int32_t sht25_stop_read_temperature(struct sht25_dev_t *dev,
                                           sht25_temper_data_t * m_temper)
{
  int32_t ret = OK;
  int32_t final_temper = 0;
  uint8_t tmp_tbl[3] = { 0 };
  const int32_t divider = 0x10000;

  ret = sht25_stop_read_reg(dev, tmp_tbl);

  final_temper = (int32_t) (((int16_t) tmp_tbl[0] << 8) | tmp_tbl[1]);
  m_temper->int_temper = -4685 + 17572 * final_temper / divider;

  sht25_dbg("Temperature: %d\n", m_temper->int_temper);

  return ret;
}

static int32_t sht25_start_read_relative_humidity(struct sht25_dev_t *dev)
{
  uint8_t command = SHT25_RH_NO_HOLDMASTER;

  return sht25_start_read_reg(dev, &command);
}

static int32_t sht25_stop_read_relative_humidity(struct sht25_dev_t *dev,
                                                 sht25_humidity_data_t *
                                                 m_humidity)
{
  int32_t ret = OK;
  uint32_t final_humidity = 0;
  uint8_t tmp_tbl[3] = { 0 };
  const int32_t divider = 0x10000;

  ret = sht25_stop_read_reg(dev, tmp_tbl);

  final_humidity = (uint32_t) (((uint16_t) tmp_tbl[0] << 8) | tmp_tbl[1]);

  m_humidity->int_humid = -60 + 1250 * final_humidity / divider;

  sht25_dbg("Humidity: %d\n", m_humidity->int_humid);

  return ret;
}

static int32_t sht25_soft_reset(struct sht25_dev_t *dev)
{
  int32_t ret = OK;
  uint8_t com = SHT25_SOFT_RESET;

  ret = sht25_write_reg8(dev, &com);
  sht25_dbg("Reset done with error code: %d\n", (ret < 0 ? ret : 0));

  return ret;
}

static int sht25_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sht25_dev_t *priv = inode->i_private;
  int ret = OK;

  sht25_dbg("Sensor is powered on\n");
  /* Device must be reseted to its initial state. Workaround on error #6 */
  sht25_soft_reset(priv);
  return ret;
}

static int sht25_close(FAR struct file *filep)
{
  sht25_dbg("CLOSED\n");
  return OK;
}

static ssize_t sht25_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  ssize_t length = 0;
  return length;
}

static ssize_t sht25_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  ssize_t length = 0;
  return length;
}

static int sht25_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct sht25_dev_t *priv = inode->i_private;
  int32_t ret = 0;

  switch (cmd)
    {
    case SHT25_IO_HUMIDITY_OUT_START:
      ret = sht25_start_read_relative_humidity(priv);
      break;
    case SHT25_IO_HUMIDITY_OUT_STOP:
      ret =
        sht25_stop_read_relative_humidity(priv, (sht25_humidity_data_t *) arg);
      break;
    case SHT25_IO_TEMPERATURE_OUT_START:
      ret = sht25_start_read_temperature(priv);
      break;
    case SHT25_IO_TEMPERATURE_OUT_STOP:
      ret = sht25_stop_read_temperature(priv, (sht25_temper_data_t *) arg);
      break;
    case SHT25_IO_SOFT_RESET:
      ret = sht25_soft_reset(priv);
      break;
    default:
      break;
    }

  return ret;
}

int sht25_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                   uint8_t addr)
{
  int ret = 0;
  struct sht25_dev_t *priv;

  priv = (struct sht25_dev_t *)kmm_zalloc(sizeof(struct sht25_dev_t));

  if (!priv)
    {
      perror("Memory cannot be allocated for Humidity sensor");
      return -ENOMEM;
    }

  humid_data = priv;
  priv->addr = addr;
  priv->i2c = i2c;

  ret = register_driver(devpath, &g_humidityops, 0666, priv);

  sht25_dbg("Registered with %d\n", ret);

  if (ret < 0)
    {
      kmm_free(priv);
      perror("Error occurred during the driver registering\n");
      return ERROR;
    }

  return OK;
}
