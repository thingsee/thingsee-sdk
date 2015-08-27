/****************************************************************************
 * drivers/sensors/max44009.c
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
#include <errno.h>
#include <nuttx/kmalloc.h>
#include <nuttx/random.h>

#include <nuttx/sensors/max44009.h>

#ifdef CONFIG_TRACES_MAX44009_SENSOR
#  define max44009_dbg(x, ...)      dbg(x, ##__VA_ARGS__)
#  define max44009_lldbg(x, ...)    lldbg(x, ##__VA_ARGS__)
#else
#  define max44009_dbg(x, ...)
#  define max44009_lldbg(x, ...)
#endif

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Sensor's registers */

#define MAX44009_INT_STS            0x0
#define MAX44009_INT_EN             0x01
#define MAX44009_CONFIG             0x02
#define MAX44009_LUX_HBYTE          0x03
#define MAX44009_LUX_LBYTE          0x04
#define MAX44009_UP_THRESH_BYTE     0x05
#define MAX44009_LOW_THRESH_BYTE    0x06
#define MAX44009_THRESH_TIMER       0x07

#define MAX44009_I2C_RETRIES        10

/****************************************************************************
* Private Function Prototypes
*****************************************************************************/

static int max44009_open(FAR struct file *filep);
static int max44009_close(FAR struct file *filep);
static ssize_t max44009_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t max44009_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);
static int max44009_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int max44009_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup);
#endif

/****************************************************************************
* Private Data
****************************************************************************/

struct max44009_dev_t
  {
    struct i2c_dev_s *i2c;
    uint8_t addr;
    max44009_config_t *config;
    sem_t dev_sem;
    bool int_pending;
#ifndef CONFIG_DISABLE_POLL
    struct pollfd *fds[CONFIG_MAX44009_SENSOR_NPOLLWAITERS];
#endif
  };

static const struct file_operations g_alsops = {
  max44009_open,
  max44009_close,
  max44009_read,
  max44009_write,
  0,
  max44009_ioctl,
#ifndef CONFIG_DISABLE_POLL
  max44009_poll
#endif
};

static struct max44009_dev_t *g_als_data;

static int max44009_do_transfer(FAR struct max44009_dev_t *dev,
                                FAR struct i2c_msg_s *msgv,
                                size_t nmsg)
{
  int ret = -EIO;
  int retries;

  for (retries = 0; retries < MAX44009_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(dev->i2c, msgv, nmsg);
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
              max44009_dbg("up_i2creset failed: %d\n", ret);
              return ret;
            }
#endif
          continue;
        }
    }

  max44009_dbg("xfer failed: %d\n", ret);
  return ret;
}

static int32_t max44009_write_reg8(FAR struct max44009_dev_t *dev,
                                   const uint8_t * command)
{
  struct i2c_msg_s msgv[2] =
  { {
    .addr   = dev->addr,
    .flags  = 0,
    .buffer = (void *)&command[0],
    .length = 1
  }, {
    .addr   = dev->addr,
    .flags  = I2C_M_NORESTART,
    .buffer = (void *)&command[1],
    .length = 1
  } };

  return max44009_do_transfer(dev, msgv, 2);
}

static int32_t max44009_read_reg8(struct max44009_dev_t *dev, uint8_t * command,
                                  uint8_t * value)
{
  struct i2c_msg_s msgv[2] =
  { {
    .addr   = dev->addr,
    .flags  = 0,
    .buffer = command,
    .length = 1
  }, {
    .addr   = dev->addr,
    .flags  = I2C_M_READ,
    .buffer = value,
    .length = 1
  } };

  return max44009_do_transfer(dev, msgv, 2);
}

static int max44009_open(FAR struct file *filep)
{
  int ret = OK;

  max44009_dbg("Sensor is powered on\n");

  return ret;
}

static int max44009_close(FAR struct file *filep)
{
  max44009_dbg("CLOSED\n");
  return OK;
}

static ssize_t max44009_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  ssize_t length = 0;
  return length;
}

static ssize_t max44009_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  ssize_t length = 0;
  return length;
}

static int max44009_set_interrupt_bit(FAR struct max44009_dev_t *priv,
                                      bool is_activated)
{
  int ret = OK;
  uint8_t value[2] = { 0 };

  value[0] = (uint8_t) MAX44009_INT_EN;
  value[1] = (uint8_t) (is_activated ? (1 << 0) : 0);

  ret = max44009_write_reg8(priv, value);
  if (ret < 0)
    {
      max44009_dbg("Cannot set interrupt bit\n");
    }

  return ret;
}

static int max44009_set_manual_mode(FAR struct max44009_dev_t *priv,
                                    bool is_manual)
{
  int ret = OK;
  uint8_t value = 0;
  const uint8_t manual_bit = (1 << 6);
  uint8_t cmd[2] = { 0 };
  cmd[0] = (uint8_t) MAX44009_CONFIG;

  ret = max44009_read_reg8(priv, &(cmd[0]), &value);
  if (ret < 0)
    {
      max44009_dbg("Cannot read config register\n");
      goto fail;
    }

  if (is_manual)
    {
      value |= manual_bit;
    }
  else
    {
      value &= ~manual_bit;
    }

  cmd[1] = value;
  ret = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Cannot set manual bit in config register\n");
    }

fail:
  return ret;
}

static int max44009_set_continuous_mode(FAR struct max44009_dev_t *priv,
                                        bool is_cont)
{
  int ret = OK;
  uint8_t value = 0;
  const uint8_t cont_bit = (1 << 7);
  uint8_t cmd[2] = { 0 };
  cmd[0] = (uint8_t) MAX44009_CONFIG;

  ret = max44009_read_reg8(priv, &(cmd[0]), &value);
  if (ret < 0)
    {
      max44009_dbg("Cannot read config register\n");
      goto fail;
    }

  if (is_cont)
    {
      value |= cont_bit;
    }
  else
    {
      value &= ~cont_bit;
    }

  cmd[1] = value;
  ret = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Cannot set cont bit in config register\n");
    }

fail:
  return ret;
}

static int max44009_set_current_div_ratio(FAR struct max44009_dev_t *priv,
                                          bool is_cdr)
{
  int ret = OK;
  uint8_t value = 0;
  const uint8_t cdr_bit = (1 << 3);
  uint8_t cmd[2] = { 0 };
  cmd[0] = (uint8_t) MAX44009_CONFIG;

  ret = max44009_read_reg8(priv, &(cmd[0]), &value);
  if (ret < 0)
    {
      max44009_dbg("Cannot read config register\n");
      goto fail;
    }

  if (is_cdr)
    {
      value |= cdr_bit;
    }
  else
    {
      value &= ~cdr_bit;
    }

  cmd[1] = value;
  ret = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Cannot set cdr bit in config register\n");
    }

fail:
  return ret;
}

static int max44009_set_integration_time(FAR struct max44009_dev_t *priv,
                                         max44009_integration_time_t
                                         integration_time)
{
  int ret = OK;
  uint8_t value = 0;
  const uint8_t tim_bits = (uint8_t) (integration_time << 0);
  const uint8_t tim_mask = 0x07;
  uint8_t cmd[2] = { 0 };
  cmd[0] = (uint8_t) MAX44009_CONFIG;

  ret = max44009_read_reg8(priv, &(cmd[0]), &value);
  if (ret < 0)
    {
      max44009_dbg("Cannot read config register\n");
      goto fail;
    }

  value &= ~tim_mask;
  value |= tim_bits;

  cmd[1] = value;
  ret = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Cannot set tim bits in config register\n");
    }

fail:
  return ret;
}

static int max44009_set_upper_threshold(FAR struct max44009_dev_t *priv,
                                        uint8_t upper_threshold)
{
  int ret;
  uint8_t cmd[2];

  cmd[0] = (uint8_t) MAX44009_UP_THRESH_BYTE;
  cmd[1] = upper_threshold;

  ret = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Upper threshold cannot be set\n");
    }

  return ret;
}

static int max44009_set_lower_threshold(FAR struct max44009_dev_t *priv,
                                        uint8_t lower_threshold)
{
  int ret;
  uint8_t cmd[2];

  cmd[0] = (uint8_t) MAX44009_LOW_THRESH_BYTE;
  cmd[1] = lower_threshold;

  ret = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Lower threshold cannot be set\n");
    }

  return ret;
}

static int max44009_do_self_calibration(FAR struct max44009_dev_t *priv,
                                        max44009_init_ops_t * settings)
{
  int ret;

  ret = max44009_set_upper_threshold(priv, settings->upper_threshold);
  if (ret < 0)
    return ret;

  ret = max44009_set_lower_threshold(priv, settings->lower_threshold);

  return ret;
}

static int max44009_set_threshold_timer(FAR struct max44009_dev_t *priv,
                                        uint8_t threshold_timer)
{
  int ret;
  uint8_t cmd[2];

  cmd[0] = (uint8_t) MAX44009_THRESH_TIMER;
  cmd[1] = threshold_timer;

  ret = max44009_write_reg8(priv, cmd);
  if (ret < 0)
    {
      max44009_dbg("Threshold timer cannot be set\n");
    }

  return ret;
}

static int max44009_selftest(FAR struct max44009_dev_t *priv,
                             max44009_data_t * data)
{
  int ret = OK;
  uint8_t reg_addr = MAX44009_THRESH_TIMER;
  uint8_t value = 0;

  ret = max44009_set_threshold_timer(priv, data->test_value);
  if (ret < 0)
    {
      max44009_dbg("Cannot write test-value\n");
      goto fail;
    }

  ret = max44009_read_reg8(priv, &reg_addr, &value);
  if (ret < 0)
    {
      max44009_dbg("Cannot read written value\n");
      goto fail;
    }

  if (value != data->test_value)
    {
      max44009_dbg("Test failed\n");
      ret = -EIO;
    }

fail:
  return ret;
}

static int max44009_init_device(FAR struct max44009_dev_t *priv,
                                max44009_init_ops_t * settings)
{
  int ret;

  /* By default interrupt is activated. However, you can
   * deactivate it later. */

  ret = max44009_set_interrupt_bit(priv, true);
  if (ret < 0)
    {
      max44009_dbg("Cannot init interrupt\n");
      goto fail;
    }

  ret = max44009_set_manual_mode(priv, settings->is_manual);
  if (ret < 0)
    {
      max44009_dbg("Cannot init manual mode\n");
      goto fail;
    }

  ret = max44009_set_continuous_mode(priv, settings->is_cont);
  if (ret < 0)
    {
      max44009_dbg("Cannot init cont mode\n");
      goto fail;
    }

  if (settings->is_manual)
    {
      ret = max44009_set_current_div_ratio(priv, settings->is_cdr);
      if (ret < 0)
        {
          max44009_dbg("Cannot init cdr mode\n");
          goto fail;
        }

      ret = max44009_set_integration_time(priv, settings->integr_time);
      if (ret < 0)
        {
          max44009_dbg("Cannot init tim mode\n");
          goto fail;
        }
    }

  ret = max44009_set_threshold_timer(priv, settings->threshold_timer);
  if (ret < 0)
    {
      max44009_dbg("Cannot init threshold timer\n");
    }

fail:
  return ret;
}

static int max44009_read_int_status(FAR struct max44009_dev_t *priv,
                                    max44009_data_t * data)
{
  int ret;
  uint8_t reg_addr = (uint8_t) MAX44009_INT_STS;

  ret = max44009_read_reg8(priv, &reg_addr, &data->int_sts);
  if (ret < 0)
    {
      max44009_dbg("Cannot read interrupt status register\n");
    }

  return ret;
}

static int max44009_read_raw_data(FAR struct max44009_dev_t *priv,
                                  max44009_data_t * data)
{
  int ret = OK;
  uint8_t lvalue;
  uint8_t hvalue;
  uint8_t reg_addr;

  reg_addr = MAX44009_LUX_HBYTE;
  ret = max44009_read_reg8(priv, &reg_addr, &hvalue);
  if (ret < 0)
    {
      max44009_dbg("Cannot read high bits from lux register\n");
      return ret;
    }

  /* LUX HBYTE has (starting with MSB): E3.E2.E1.E0.M7.M6.M5.M4
   * LUX LBYTE has                    : --.--.--.--.M3.M2.M1.M0
   *
   * E[3..0] = Exponent, M[7..0]: Mantissa.
   *
   * Lux can be calculated as (full resolution): (M[7..0] << E[3..0]) * 0.045.
   *
   * Lux can also be calculated using only HBYTE:
   *     (M[7..4] << E[3..0]) * 0.72
   *       == (M[7..4] << E[3..0]) * 2^4 * 0.045
   *       == (M[7..4] << E[3..0]) * (1 << 4) * 0.045
   *       == (M[7..4] << (E[3..0] + 4)) * 0.045
   */

  reg_addr = MAX44009_LUX_LBYTE;
  ret = max44009_read_reg8(priv, &reg_addr, &lvalue);
  if (ret < 0)
    {
      max44009_dbg("Cannot read low bits from lux register\n");
      return ret;
    }

  /* Merge HBYTE and LBYTE to 16-bit integer:
   *   --.--.--.--.E3.E2.E1.E0.M7.M6.M5.M4.M3.M2.M1.M0 */

  data->raw_value = (hvalue << 4) | (lvalue & 0xF);

  /* Add raw value to entropy pool. */

  add_sensor_randomness(data->raw_value);

  return ret;
}

static int max44009_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct max44009_dev_t *priv = inode->i_private;
  int32_t ret = 0;

  switch (cmd)
    {
    case MAX44009_IOC_INIT:
      ret = max44009_init_device(priv, (max44009_init_ops_t *) arg);
      break;
    case MAX44009_IOC_READ_INTERRUPT_STATUS:
      ret = max44009_read_int_status(priv, (max44009_data_t *) arg);
      break;
    case MAX44009_IOC_SET_INTERRUPT:
      ret = max44009_set_interrupt_bit(priv,
                                       ((max44009_data_t *) arg)->is_interrupt);
      break;
    case MAX44009_IOC_SET_CONT:
      ret =
        max44009_set_continuous_mode(priv, ((max44009_data_t *) arg)->is_cont);
      break;
    case MAX44009_IOC_SET_MANUAL:
      ret =
        max44009_set_manual_mode(priv, ((max44009_data_t *) arg)->is_manual);
      break;
    case MAX44009_IOC_SET_CDR:
      ret =
        max44009_set_current_div_ratio(priv, ((max44009_data_t *) arg)->is_cdr);
      break;
    case MAX44009_IOC_SET_INTEGR_TIME:
      ret =
        max44009_set_integration_time(priv,
                                      ((max44009_data_t *) arg)->integr_time);
      break;
    case MAX44009_IOC_DO_SELFCALIB:
      ret = max44009_do_self_calibration(priv, (max44009_init_ops_t *) arg);
      break;
    case MAX44009_IOC_SET_THRES_TIMER:
      ret =
        max44009_set_threshold_timer(priv,
                                     ((max44009_data_t *) arg)->threshold_timer);
      break;
    case MAX44009_IOC_READ_RAW_DATA:
      ret = max44009_read_raw_data(priv, (max44009_data_t *) arg);
      break;
    case MAX44009_IOC_DO_SELFTEST:
      ret = max44009_selftest(priv, (max44009_data_t *) arg);
      break;
    default:
      ret = -EINVAL;
      break;
    }

  return ret;
}

#ifndef CONFIG_DISABLE_POLL
static void max44009_notify(FAR struct max44009_dev_t *priv)
{
  DEBUGASSERT(priv != NULL);

  int i;

  /* If there are threads waiting on poll() for data to become available, *
   * then wake them up now.  NOTE: we wake up all waiting threads because we *
   * do not know that they are going to do.  If they all try to read the data,
   * * then some make end up blocking after all. */

  for (i = 0; i < CONFIG_MAX44009_SENSOR_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          max44009_lldbg("Report events: %02x\n", fds->revents);
          sem_post(fds->sem);
          priv->int_pending = false;
        }
    }
}

static int max44009_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  FAR struct inode *inode;
  FAR struct max44009_dev_t *priv;
  int ret = OK;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct max44009_dev_t *)inode->i_private;

  ret = sem_wait(&priv->dev_sem);
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

      for (i = 0; i < CONFIG_MAX44009_SENSOR_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_MAX44009_SENSOR_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }
      if (priv->int_pending)
        {
          max44009_notify(priv);
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
out:
  sem_post(&priv->dev_sem);
  return ret;
}
#endif /* !CONFIG_DISABLE_POLL */

static int max44009_int_handler(int irq, FAR void *context)
{
  g_als_data->int_pending = true;
#ifndef CONFIG_DISABLE_POLL
  max44009_notify(g_als_data);
#endif
  max44009_lldbg("MAX44009 interrupt\n");

  return OK;
}

int max44009_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                      uint8_t addr, max44009_config_t * config)
{
  int ret = 0;
  struct max44009_dev_t *priv;

  priv = (struct max44009_dev_t *)kmm_zalloc(sizeof(struct max44009_dev_t));

  if (!priv)
    {
      max44009_dbg("Memory cannot be allocated for ALS sensor\n");
      return -ENOMEM;
    }

  g_als_data = priv;
  priv->addr = addr;
  priv->i2c = i2c;
  priv->config = config;
  sem_init(&priv->dev_sem, 0, 1);

  ret = register_driver(devpath, &g_alsops, 0666, priv);

  max44009_dbg("Registered with %d\n", ret);

  if (ret < 0)
    {
      kmm_free(priv);
      max44009_dbg("Error occurred during the driver registering\n");
      return ERROR;
    }

  priv->config->irq_clear(priv->config);
  priv->config->irq_attach(priv->config, max44009_int_handler);
  priv->config->irq_enable(priv->config, true);

  return OK;
}
