/****************************************************************************
 * drivers/power/bq24251.c
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

#include <nuttx/config.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>

#include "nuttx/power/bq24251.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_BQ24251_CHARGER
#  define bq24251_dbg(x, ...)	dbg(x, ##__VA_ARGS__)
#  define bq24251_lldbg(x, ...) lldbg(x, ##__VA_ARGS__)
#else
#  define bq24251_dbg(x, ...)
#  define bq24251_lldbg(x, ...)
#endif

#define BQ24251_I2C_RETRIES             10

/* Register#1 */

#define BQ24251_WD_FAULT		(1 << 7)
#define BQ24251_WD_EN			(1 << 6)
#define BQ24251_STAT_SHIFT		4
#define BQ24251_STAT_MASK		(0x03 << BQ24251_STAT_SHIFT)
#define BQ24251_READY 		(BQ24251_READY_ST << BQ24251_STAT_SHIFT)
#define BQ24251_PROGR			(BQ24251_PROGR_ST << BQ24251_STAT_SHIFT)
#define BQ24251_DONE			(BQ24251_DONE_ST << BQ24251_STAT_SHIFT)
#define BQ24251_FAULT			(BQ24251_FAULT_ST << BQ24251_STAT_SHIFT)

#define BQ24251_FAULT_SHIFT		0
#define BQ24251_FAULT_MASK		(0x0F << BQ24251_FAULT_SHIFT)

/* Register#2 */

#define BQ24251_RESET			(1 << 7)
#define BQ24251_ILIM_SHIFT		4
#define BQ24251_ILIM_MASK		(0x07 << BQ24251_ILIM_SHIFT)
#define BQ24251_EN_STAT			(1 << 3)
#define BQ24251_EN_TERM			(1 << 2)
#define BQ24251_CE			(1 << 1)
#define BQ24251_HZ_MODE			(1 << 0)
#define BQ24251_CLEAR_RST_MASK		0x7F

/* Register#3 */

#define BQ24251_USB_DET_SHIFT		0
#define BQ24251_USB_DET_MASK		(0x03 << BQ24251_USB_DET_SHIFT)

/* Register#4 */

#define BQ24251_ICHG_4			(1 << 7)
#define BQ24251_ICHG_3			(1 << 6)
#define BQ24251_ICHG_2			(1 << 5)
#define BQ24251_ICHG_1			(1 << 4)
#define BQ24251_ICHG_0			(1 << 3)
#define BQ24251_ITERM_2			(1 << 2)
#define BQ24251_ITERM_1			(1 << 1)
#define BQ24251_ITERM_0			(1 << 0)

/* Register#5 */

#define BQ24251_LOOP_SHIFT		6
#define BQ24251_LOOP_MASK		(1 << BQ24251_LOOP_SHIFT)
#define BQ24251_LOW_CHG			(1 << 5)
#define BQ24251_DPDM_EN			(1 << 4)
#define BQ24251_CE_STATUS		(1 << 3)

/* Register#6 */

#define BQ24251_SYSOFF			(1 << 4)

#define REG1					0x0
#define REG2					0x01
#define REG3					0x02
#define REG4					0x03
#define REG5					0x04
#define REG6					0x05
#define REG7					0x06

/************************************************************************************
* Private Function Prototypes
************************************************************************************/

static int bq24251_open(FAR struct file *filep);
static int bq24251_close(FAR struct file *filep);
static ssize_t bq24251_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t bq24251_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int bq24251_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int bq24251_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);
#endif

/****************************************************************************
* Private Data
****************************************************************************/

struct bq24251_dev_t
  {
    struct i2c_dev_s *i2c;
    uint8_t addr;
    bq24251_config_t *config;
    sem_t devsem;
    sem_t st_sem;
    bool stat_pending;
#ifndef CONFIG_DISABLE_POLL
    struct pollfd *fds[CONFIG_BQ24251_NPOLLWAITERS];
#endif
  };

static const struct file_operations g_chargerops = {
  bq24251_open,
  bq24251_close,
  bq24251_read,
  bq24251_write,
  0,
  bq24251_ioctl,
#ifndef CONFIG_DISABLE_POLL
  bq24251_poll
#endif
};

static struct bq24251_dev_t *g_charger_data;

#ifdef CONFIG_DEBUG_BQ24251_CHARGER
static int bq24251_dump_regs(FAR struct bq24251_dev_t *priv);
#  define bq24251_dump_regs(priv) bq24251_dump_regs(priv)
#else
#  define bq24251_dump_regs(priv)
#endif

static int bq24251_do_transfer(FAR struct bq24251_dev_t *dev,
                               FAR struct i2c_msg_s *msgv,
                               size_t nmsg)
{
  int ret = -EIO;
  int retries;

  for (retries = 0; retries < BQ24251_I2C_RETRIES; retries++)
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
              bq24251_dbg("up_i2creset failed: %d\n", ret);
              return ret;
            }
#endif
          continue;
        }
    }

  bq24251_dbg("xfer failed: %d\n", ret);
  return ret;
}

static int bq24251_write_reg8(struct bq24251_dev_t *dev, uint8_t reg_addr,
                              const uint8_t value)
{
  struct i2c_msg_s msgv[2] =
  { {
    .addr   = dev->addr,
    .flags  = 0,
    .buffer = &reg_addr,
    .length = 1
  }, {
    .addr   = dev->addr,
    .flags  = I2C_M_NORESTART,
    .buffer = (void *)&value,
    .length = 1
  } };

  return bq24251_do_transfer(dev, msgv, 2);
}

static int bq24251_read_reg8(struct bq24251_dev_t *dev, uint8_t reg_addr,
                             uint8_t * value)
{
  struct i2c_msg_s msgv[2] =
  { {
    .addr   = dev->addr,
    .flags  = 0,
    .buffer = &reg_addr,
    .length = 1
  }, {
    .addr   = dev->addr,
    .flags  = I2C_M_READ,
    .buffer = value,
    .length = 1
  } };
  int ret;

  ret = bq24251_do_transfer(dev, msgv, 2);
  if (ret < 0)
    return ret;

  /* Fix to separate data reads */

  usleep(1);

  return OK;
}

#ifdef CONFIG_DEBUG_BQ24251_CHARGER
static int (bq24251_dump_regs) (FAR struct bq24251_dev_t * priv)
{
  int ret = OK;
  uint8_t value = 0;

  ret = bq24251_read_reg8(priv, REG1, &value);
  bq24251_lldbg("REG#1: 0x%08X\n", value);
  ret |= bq24251_read_reg8(priv, REG2, &value);
  bq24251_lldbg("REG#2: 0x%08X\n", value);
  ret |= bq24251_read_reg8(priv, REG3, &value);
  bq24251_lldbg("REG#3: 0x%08X\n", value);
  ret |= bq24251_read_reg8(priv, REG4, &value);
  bq24251_lldbg("REG#4: 0x%08X\n", value);
  ret |= bq24251_read_reg8(priv, REG5, &value);
  bq24251_lldbg("REG#5: 0x%08X\n", value);
  ret |= bq24251_read_reg8(priv, REG6, &value);
  bq24251_lldbg("REG#6: 0x%08X\n", value);
  ret |= bq24251_read_reg8(priv, REG7, &value);
  bq24251_lldbg("REG#7: 0x%08X\n", value);

  return ret;
}
#endif

static int bq24251_reset_chrg(FAR struct bq24251_dev_t *priv)
{
  int ret = OK;

  ret = bq24251_write_reg8(priv, REG2, BQ24251_RESET);
  if (ret)
    {
      bq24251_lldbg("REG#2 cannot write i2c reset\n");
      return ret;
    }

  bq24251_lldbg("Reset done\n");

  return ret;
}

static int bq24251_enable_chrg(FAR struct bq24251_dev_t *priv, bool enable)
{
  int ret = OK;
  uint8_t value = 0;

  ret = bq24251_read_reg8(priv, REG2, &value);
  if (ret < 0)
    goto fail;

  value &= BQ24251_CLEAR_RST_MASK;
  if (enable)
    {
      value &= ~BQ24251_CE;
    }
  else
    {
      value |= BQ24251_CE;
    }

  ret = bq24251_write_reg8(priv, REG2, value);

fail:
  return ret;
}

static int bq24251_set_chrg_current(FAR struct bq24251_dev_t *priv,
                                    bq24251_charger_current_t value)
{
  int ret = OK;
  uint8_t tmp = 0;

  ret = bq24251_read_reg8(priv, REG4, &tmp);
  if (ret)
    {
      bq24251_lldbg("REG#4 cannot read i2c ilim\n");
      return ret;
    }

  tmp &= ~BQ24251_CHRG_CURRENT_EXTERNAL;
  tmp |= (0xFF & value);        /* Remove garbage if presented */
  bq24251_lldbg("REG#4 value: 0x%02X\n", tmp);

  ret = bq24251_write_reg8(priv, REG4, tmp);

  if (ret)
    {
      bq24251_lldbg("REG#4 cannot write i2c ilim\n");
    }
#ifdef CONFIG_DEBUG_BQ24251_CHARGER
  uint8_t dbg_val;
  bq24251_read_reg8(priv, REG4, &dbg_val);
  bq24251_lldbg("REG#4: 0x%08X\n", dbg_val);
#endif

  return ret;
}

static int bq24251_set_ilim(FAR struct bq24251_dev_t *priv,
                            bq24251_current_t value)
{
  int ret = OK;
  uint8_t tmp = 0;

  ret = bq24251_read_reg8(priv, REG2, &tmp);
  if (ret)
    {
      bq24251_lldbg("REG#2 cannot read i2c ilim\n");
      return ret;
    }

  tmp &= 0x0F;
  tmp |= (((uint8_t) value) << BQ24251_ILIM_SHIFT);

  bq24251_lldbg("REG#2 ILIM 0x%08X\n", tmp);

  ret = bq24251_write_reg8(priv, REG2, tmp);
  if (ret)
    {
      bq24251_lldbg("REG#2 cannot write i2c ilim\n");
      return ret;
    }

  bq24251_dump_regs(priv);

  return ret;
}

static int bq24251_chk_chrg_sts(FAR struct bq24251_dev_t *priv,
                                bq24251_chrg_status_t * status)
{
  int ret = OK;
  uint8_t value;

  ret = sem_wait(&priv->st_sem);
  if (ret < 0)
    return ret;

  ret = bq24251_read_reg8(priv, REG1, &value);  /* Read register for status */
  if (ret < 0)
    {
      bq24251_lldbg("Cannot read REG#1\n");
      goto fail;
    }

  if ((value & BQ24251_FAULT) == BQ24251_FAULT)
    {
      status->fault = value & 0x0F;
      bq24251_lldbg("Fault occurred: 0x%02X\n", value & 0x0F);
    }
  else
    {
      status->fault = BQ24251_NORMAL & 0x0F;
    }

  /* Just to be sure there is no garbage masking is used */
  status->state = (bq24251_state_t) ((value >> BQ24251_STAT_SHIFT) & 0x3);
  priv->stat_pending = false;

  bq24251_lldbg("state: %d is_fault: %d fault: %d wd_en: %d wd_fault: %d\n",
      ((value >> BQ24251_STAT_SHIFT) & 0x3),
      (value & BQ24251_FAULT) == BQ24251_FAULT,
      value & 0x0F,
      (value & BQ24251_WD_EN) == BQ24251_WD_EN,
      (value & BQ24251_WD_FAULT) == BQ24251_WD_FAULT);

fail:
  sem_post(&priv->st_sem);
  return ret;
}

static int bq24251_start_usbdet(FAR struct bq24251_dev_t *priv)
{
  int ret;
  uint8_t tmp_value = 0;

  ret = bq24251_read_reg8(priv, REG5, &tmp_value);
  if (ret < 0)
    {
      bq24251_lldbg("Cannot read REG#5\n");
      return ret;
    }

  bq24251_lldbg("Read value REG5: 0x%08X\n", tmp_value);

  /* Force D+/D- detection. */

  tmp_value |= BQ24251_DPDM_EN;

  ret = bq24251_write_reg8(priv, REG5, tmp_value);
  if (ret < 0)
    {
      bq24251_lldbg("Cannot write REG#5\n");
      return ret;
    }

  return ret;
}

static int bq24251_check_usbdet(FAR struct bq24251_dev_t *priv,
                                bq24251_chrg_type_t * value)
{
  int ret;
  uint8_t tmp_value = 0;

  ret = bq24251_read_reg8(priv, REG5, &tmp_value);
  if (ret < 0)
    {
      bq24251_lldbg("Cannot write REG#5\n");
      goto fail;
    }

  if (tmp_value & BQ24251_DPDM_EN)
    {
      bq24251_lldbg("USB detection still on-going, ret => EAGAIN\n");
      ret = -EAGAIN;
      goto fail;
    }

  ret = bq24251_read_reg8(priv, REG3, &tmp_value);
  if (ret < 0)
    {
      bq24251_lldbg("Cannot read REG#3\n");
      goto fail;
    }
  bq24251_lldbg("Read value REG3: 0x%08X\n", tmp_value);

  *value = (bq24251_chrg_type_t) (tmp_value & 0x3);

fail:
  return ret;
}

static int bq24251_enable_stat(FAR struct bq24251_dev_t *priv, bool enable)
{
  int ret = OK;
  uint8_t value = 0;

  ret = bq24251_read_reg8(priv, REG2, &value);
  if (ret < 0)
    goto fail;

  value &= BQ24251_CLEAR_RST_MASK;
  if (enable)
    {
      value |= BQ24251_EN_STAT;
    }
  else
    {
      value &= ~BQ24251_EN_STAT;
    }

  ret = bq24251_write_reg8(priv, REG2, value);

fail:
  return ret;
}

static int bq24251_enable_term(FAR struct bq24251_dev_t *priv, bool enable)
{
  int ret = OK;
  uint8_t value = 0;

  ret = bq24251_read_reg8(priv, REG2, &value);
  if (ret < 0)
    goto fail;

  value &= BQ24251_CLEAR_RST_MASK;
  if (enable)
    {
      value |= BQ24251_EN_TERM;
    }
  else
    {
      value &= ~BQ24251_EN_TERM;
    }

  ret = bq24251_write_reg8(priv, REG2, value);

fail:
  return ret;
}

static int bq24251_set_low_chg(FAR struct bq24251_dev_t *priv, bool enable)
{
  int ret = OK;
  uint8_t value = 0;

  ret = bq24251_read_reg8(priv, REG5, &value);
  if (ret < 0)
    goto fail;

  if (enable)
    {
      value |= BQ24251_LOW_CHG;
    }
  else
    {
      value &= ~BQ24251_LOW_CHG;
    }

  ret = bq24251_write_reg8(priv, REG5, value);

fail:
  return ret;
}

static int bq24251_disable_timers(FAR struct bq24251_dev_t *priv)
{
  int ret = OK;
  uint8_t value = 0;

  ret = bq24251_read_reg8(priv, REG6, &value);
  if (ret < 0)
    {
      bq24251_lldbg("Warning: Cannot disable timers: %d\n", ret);
      goto fail;
    }

  value |= 0x60;
  ret = bq24251_write_reg8(priv, REG6, value);
  if (ret < 0)
    {
      bq24251_lldbg("Warning: Cannot disable timers: %d\n", ret);
      goto fail;
    }

  ret = bq24251_read_reg8(priv, REG6, &value);
  if (ret < 0)
    goto fail;

  bq24251_lldbg("After timer disable: REG#6: 0x%02X\n", value);

fail:
  return ret;
}

static int bq24251_enable_wd(FAR struct bq24251_dev_t *priv, bool enable)
{
  int ret = OK;
  uint8_t value = 0;

  ret = bq24251_read_reg8(priv, REG1, &value);
  if (ret < 0)
    goto fail;

  if (enable)
    {
      value |= BQ24251_WD_EN;
    }
  else
    {
      value &= ~BQ24251_WD_EN;
    }

  ret = bq24251_write_reg8(priv, REG1, value);

fail:
  return ret;
}

static int bq24251_set_hzmode(FAR struct bq24251_dev_t *priv, bool enable)
{
  int ret = OK;
  uint8_t value = 0;

  ret = bq24251_read_reg8(priv, REG2, &value);
  if (ret < 0)
    goto fail;

  value &= BQ24251_CLEAR_RST_MASK;
  if (enable)
    {
      value |= BQ24251_HZ_MODE;
    }
  else
    {
      value &= ~BQ24251_HZ_MODE;
    }

  ret = bq24251_write_reg8(priv, REG2, value);

fail:
  return ret;
}

static int bq24251_set_bat_det(FAR struct bq24251_dev_t *priv)
{
  int ret;
  uint8_t value;

  value = 0xE8;
  ret = bq24251_write_reg8(priv, REG7, value);

  return ret;
}

static int bq24251_init_chrg(FAR struct bq24251_dev_t *priv)
{
  int ret = OK;

  bq24251_reset_chrg(priv);
  bq24251_enable_chrg(priv, true);
  bq24251_enable_stat(priv, true);
  bq24251_enable_term(priv, true);
  bq24251_set_hzmode(priv, false);
  bq24251_set_low_chg(priv, false);
  bq24251_enable_wd(priv, false);

  bq24251_dump_regs(priv);

  bq24251_lldbg("Charger initialization done\n");

  return ret;
}

static int bq24251_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bq24251_dev_t *priv = inode->i_private;
  int ret = OK;

  priv->stat_pending = true;

  bq24251_dbg("Charger is powered on\n");

  return ret;
}

static int bq24251_sysoff(FAR struct bq24251_dev_t *priv)
{
  int ret = OK;
  uint8_t value = 0;

  ret = bq24251_read_reg8(priv, REG6, &value);
  bq24251_dbg("REG6 read value: 0x%08X\n", value);
  value |= BQ24251_SYSOFF;
  ret |= bq24251_write_reg8(priv, REG6, value);

  return ret;
}

static void bq24251_notify(FAR struct bq24251_dev_t *priv, pollevent_t revents)
{
#ifndef CONFIG_DISABLE_POLL
  int i;

  DEBUGASSERT(priv != NULL);

  for (i = 0; i < CONFIG_BQ24251_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= revents;
          bq24251_lldbg("Report events: %02x\n", fds->revents);
          sem_post(fds->sem);
        }
    }
#endif
}

static int bq24251_close(FAR struct file *filep)
{
  bq24251_dbg("CLOSED\n");
  return OK;
}

static ssize_t bq24251_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  ssize_t length = 0;
  return length;
}

static ssize_t bq24251_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  ssize_t length = 0;
  return length;
}

static int bq24251_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct bq24251_dev_t *priv = inode->i_private;
  int32_t ret = OK;
  bq24251_ioc_t ioc_cmd = (bq24251_ioc_t) cmd;

  switch (ioc_cmd)
    {
    case BQ24251_IOC_RESET:
      ret = bq24251_reset_chrg(priv);
      break;
    case BQ24251_IOC_INIT:
      ret = bq24251_init_chrg(priv);
      break;
    case BQ24251_IOC_START_USBDET:
      ret = bq24251_start_usbdet(priv);
      break;
    case BQ24251_IOC_CHECK_USBDET:
      ret = bq24251_check_usbdet(priv, &((bq24251_data_t *) arg)->chrg_type);
      break;
    case BQ24251_IOC_CHK_CHRG_STS:
      ret = bq24251_chk_chrg_sts(priv, ((bq24251_data_t *) arg)->sts);
      break;
    case BQ24251_IOC_SET_ILIM:
      ret = bq24251_set_ilim(priv, ((bq24251_data_t *) arg)->ilim);
      break;
    case BQ24251_IOC_SET_CHRG_CURRENT:
      ret =
        bq24251_set_chrg_current(priv, ((bq24251_data_t *) arg)->chrg_current);
      break;
    case BQ24251_IOC_SET_HZ:
      ret = bq24251_set_hzmode(priv, ((bq24251_data_t *) arg)->enable_hz);
      break;
    case BQ24251_IOC_SET_TERM:
      ret = bq24251_enable_term(priv, ((bq24251_data_t *) arg)->enable_term);
      break;
    case BQ24251_IOC_SYSOFF:
      ret = bq24251_sysoff(priv);
      break;
    case BQ24251_IOC_DISABLE_TIMERS:
      ret = bq24251_disable_timers(priv);
      break;
    case BQ24251_IOC_FORCE_BAT_DETECTION:
      ret = bq24251_set_bat_det(priv);
      break;
    case BQ24251_IOC_SET_CHARGE_ENABLE:
      bq24251_enable_chrg(priv, ((bq24251_data_t *) arg)->enable_ce);
      break;
    default:
      break;
    }

  return ret;
}

#ifndef CONFIG_DISABLE_POLL
static int bq24251_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode *inode;
  FAR struct bq24251_dev_t *priv;
  int ret = OK;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct bq24251_dev_t *)inode->i_private;

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
      for (i = 0; i < CONFIG_BQ24251_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_BQ24251_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
        }
      else if (priv->stat_pending)
        {
          bq24251_notify(priv, POLLIN);
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
  sem_post(&priv->devsem);
  return ret;
}
#endif

static int bq24251_int_handler_stat(int irq, FAR void *context)
{
  g_charger_data->stat_pending = true;

  bq24251_notify(g_charger_data, POLLIN);
  bq24251_lldbg("Charger interrupt stat\n");

  return OK;
}

int bq24251_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                     uint8_t addr, bq24251_config_t * config)
{
  int ret = 0;
  struct bq24251_dev_t *priv;

  priv = (struct bq24251_dev_t *)kmm_zalloc(sizeof(struct bq24251_dev_t));

  if (!priv)
    {
      bq24251_dbg("Memory cannot be allocated for driver\n");
      return -ENOMEM;
    }

  g_charger_data = priv;
  priv->addr = addr;
  priv->i2c = i2c;
  priv->config = config;
  sem_init(&priv->devsem, 0, 1);
  sem_init(&priv->st_sem, 0, 1);

  ret = register_driver(devpath, &g_chargerops, 0666, priv);

  priv->config->irq_clear(priv->config);
  priv->config->irq_attach(priv->config, bq24251_int_handler_stat);

  bq24251_dbg("Registered with %d\n", ret);

  if (ret < 0)
    {
      kmm_free(priv);
      bq24251_dbg("Error occurred during the driver registering\n");
      return ERROR;
    }

  priv->config->irq_enable(priv->config, true);

  return ret;
}
