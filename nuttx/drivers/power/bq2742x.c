/****************************************************************************
 * drivers/power/bq2742x.c
 * Lower-half battery fuel gauge driver for TI BQ2742X.
 *
 *   Copyright (C) 2017 Haltian Ltd. All rights reserved.
 *   Author: Harri Luhtala <harri.luhtala@haltian.com>
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
#include <nuttx/power/battery.h>
#include <nuttx/power/bq2742x.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_BQ2742X_DEBUG
#  define bq2742x_dbg(x, ...)   dbg(x, ##__VA_ARGS__)
#  define bq2742x_lldbg(x, ...) lldbg(x, ##__VA_ARGS__)
#else
#  define bq2742x_dbg(x, ...)
#  define bq2742x_lldbg(x, ...)
#endif

#define BQ2742X_I2C_WAIT        66  /* microseconds */
#define BQ2742X_I2C_RETRIES     10

#define UPD_MODE_MAX_RETRY      5

#define BQ2742X_UNSEAL_KEY      0x8000

/* Standard commands and associated subcommands */

#define BQ2742X_CMD_CNTL        0x00
#define SUB_CMD_STATUS          0x0000
#define SUB_CMD_DEVICE_TYPE     0x0001
#define SUB_CMD_FW_VER          0x0002
#define SUB_CMD_DM_CODE         0x0004
#define SUB_CMD_PREV_MACWR      0x0007
#define SUB_CMD_CHEM_ID         0x0008
#define SUB_CMD_BAT_INSERT      0x000C
#define SUB_CMD_BAT_REMOVE      0x000D
#define SUB_CMD_SET_CFG_UPD     0x0013
#define SUB_CMD_SMOOTH_SYNC     0x0019
#define SUB_CMD_SHUTDOWN_EN     0x001B
#define SUB_CMD_SHUTDOWN        0x001C
#define SUB_CMD_SEALED          0x0020
#define SUB_CMD_PULSE_INT       0x0023
#define SUB_CMD_CHEM_A          0x0030
#define SUB_CMD_CHEM_B          0x0031
#define SUB_CMD_CHEM_C          0x0032
#define SUB_CMD_RESET           0x0041
#define SUB_CMD_SOFT_RST        0x0042
#define BQ2742X_CMD_TEMP        0x02
#define BQ2742X_CMD_VOLT        0x04
#define BQ2742X_CMD_FLAGS       0x06
#define BQ2742X_CMD_NAC         0x08
#define BQ2742X_CMD_FAC         0x0A
#define BQ2742X_CMD_RM          0x0C
#define BQ2742X_CMD_FCC         0x0E
#define BQ2742X_CMD_AVG_CUR     0x10
#define BQ2742X_CMD_AVG_PWR     0x18
#define BQ2742X_CMD_SOC         0x1C
#define BQ2742X_CMD_INT_TEMP    0x1E
#define BQ2742X_CMD_SOH         0x20
#define BQ2742X_CMD_RCU         0x28
#define BQ2742X_CMD_RCF         0x2A
#define BQ2742X_CMD_FCCU        0x2C
#define BQ2742X_CMD_FCCF        0x2E
#define BQ2742X_CMD_SOCU        0x30

/* Extended commands */

#define BQ2742X_DATA_CLASS      0x3E
#define BQ2742X_DATA_BLOCK      0x3F
#define BQ2742X_REMAINDER       0x40
#define BQ2742X_CHECKSUM        0x60
#define BQ2742X_DATA_CONTROL    0x61

/* Data subclass IDs */

#define DC_SAFETY               2
#define DC_SAFETY_LEN           6
#define DC_CHARGE_TERM          36
#define DC_CHARGE_TERM_LEN      9
#define DC_DISCHARGE            49
#define DC_DISCHARGE_LEN        4
#define DC_REGISTER             64
#define DC_REGISTER_LEN         5
#define DC_IT_CFG               80
#define DC_IT_CFG_LEN           85
#define DC_CUR_THRESH           81
#define DC_CUR_THRESH_LEN       14
#define DC_STATE                82
#define DC_STATE_LEN            31
#define DC_RA0_RAM              89
#define DC_RA0_RAM_LEN          30
#define DC_DATA                 104
#define DC_DATA_LEN             22
#define DC_CC_CAL               105
#define DC_CC_CAL_LEN           12
#define DC_CURRENT              107
#define DC_CURRENT_LEN          1
#define DC_CHEM_DATA            109
#define DC_CHEM_DATA_LEN        10
#define DC_CODES                112
#define DC_CODES_LEN            4

/* Bit definitions for status subcmd */

#define STATUS_SHUTDOWN_EN      (1 << 15)
#define STATUS_WDRESET          (1 << 14)
#define STATUS_SS               (1 << 13)
#define STATUS_CALMODE          (1 << 12)
#define STATUS_CCA              (1 << 11)
#define STATUS_BCA              (1 << 10)
#define STATUS_QMAX_UP          (1 << 9)
#define STATUS_RES_UP           (1 << 8)
#define STATUS_INITCOMP         (1 << 7)
#define STATUS_SLEEP            (1 << 4)
#define STATUS_LDMD             (1 << 3)
#define STATUS_RUP_DIS          (1 << 2)
#define STATUS_VOK              (1 << 1)
#define STATUS_CHEM_CHANGE      (1 << 0)

/* Bit definitions for flags command */

#define FLAG_OT                 (1 << 15)
#define FLAG_UT                 (1 << 14)
#define FLAG_FC                 (1 << 9)
#define FLAG_CHG                (1 << 8)
#define FLAG_OCVTAKEN           (1 << 7)
#define FLAG_DOD_CORRECT        (1 << 6)
#define FLAG_ITPOR              (1 << 5)
#define FLAG_CFGUPMODE          (1 << 4)
#define FLAG_BAT_DET            (1 << 3)
#define FLAG_SOC1               (1 << 2)
#define FLAG_SOCF               (1 << 1)
#define FLAG_DSG                (1 << 0)

/* Bit definitions for SOH command */

#define SOH_NOT_VALID           0
#define SOH_INSTANT_READY       1
#define SOH_INITIAL_READY       2
#define SOH_VALUE_READY         3

/****************************************************************************
* Private Function Prototypes
 ****************************************************************************/

static int bq2742x_state(struct battery_dev_s *dev, int *status);
static int bq2742x_online(struct battery_dev_s *dev, bool *status);
static int bq2742x_voltage(struct battery_dev_s *dev, b16_t *value);
static int bq2742x_current(struct battery_dev_s *dev, b16_t *value);
static int bq2742x_power(struct battery_dev_s *dev, b16_t *value);
static int bq2742x_capacity(struct battery_dev_s *dev, b16_t *value);
static int bq2742x_capacity_full(struct battery_dev_s *dev, b16_t *value);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct bq2742x_dev_s
{
  /* The common part of the battery driver visible to the upper-half driver */

  FAR const struct battery_operations_s *ops;
  sem_t batsem;
#ifndef CONFIG_DISABLE_POLL
  struct pollfd *fds[CONFIG_BATTERY_NPOLLWAITERS];
  volatile bool int_pending;
#endif

  /* Data fields specific to the lower half BQ2742X driver */

  FAR struct i2c_dev_s *i2c;
  uint8_t addr;
  struct bq2742x_config_s *config;
};

static const struct battery_operations_s bq2742x_ops =
{
  bq2742x_state,
  bq2742x_online,
  bq2742x_voltage,
  bq2742x_current,
  bq2742x_power,
  bq2742x_capacity,
  bq2742x_capacity_full
};

static struct bq2742x_dev_s *g_gauge_data;

static int bq2742x_do_transfer(FAR struct bq2742x_dev_s *dev,
                               FAR struct i2c_msg_s *msgv,
                               size_t nmsg)
{
  int ret = -EIO;
  int retries;

  for (retries = 0; retries < BQ2742X_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(dev->i2c, msgv, nmsg);
      if (ret >= 0)
        {
          usleep(BQ2742X_I2C_WAIT);
          return 0;
        }
      else
        {
          /* Some error. Try to reset I2C bus and keep trying. */
#ifdef CONFIG_I2C_RESET
          if (retries == BQ2742X_I2C_RETRIES - 1)
            break;

          ret = up_i2creset(dev->i2c);
          if (ret < 0)
            {
              bq2742x_dbg("up_i2creset failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  bq2742x_dbg("xfer failed: %d\n", ret);
  return ret;
}

static uint8_t bq2742x_write(struct bq2742x_dev_s *dev, uint8_t reg_addr,
                             uint8_t *buf, uint8_t len)
{
  uint8_t ret;

  struct i2c_msg_s msgv[2] = {
  {
    .addr   = dev->addr,
    .flags  = 0,
    .buffer = &reg_addr,
    .length = 1
  },
  {
    .addr   = dev->addr,
    .flags  = I2C_M_NORESTART,
    .buffer = buf,
    .length = len
  }};

  ret = bq2742x_do_transfer(dev, msgv, 2);
  if (!ret)
    return len;

  return 0;
}

static int bq2742x_write_cmd(struct bq2742x_dev_s *dev, uint8_t reg_addr,
                             uint16_t value)
{
  uint8_t ret;
  uint8_t tx_buf[2] = {value & 0xFF, value >> 8};

  if ((ret = bq2742x_write(dev, reg_addr, tx_buf, sizeof(tx_buf))) !=
    sizeof(tx_buf))
    {
      bq2742x_dbg("failed, ret: %d, cmd: %02X\n", ret, reg_addr);
      return -1;
    }

  return OK;
}

static uint8_t bq2742x_read(struct bq2742x_dev_s *dev, uint8_t reg_addr,
                            uint8_t *buf, uint8_t len)
{
  int ret;
  struct i2c_msg_s msgv[2] = {
  {
    .addr   = dev->addr,
    .flags  = 0,
    .buffer = &reg_addr,
    .length = 1
  },
  {
    .addr   = dev->addr,
    .flags  = I2C_M_READ,
    .buffer = buf,
    .length = len
  }};

  ret = bq2742x_do_transfer(dev, msgv, 2);
  if (!ret)
    return len;

  return 0;
}

static int bq2742x_read_cmd(struct bq2742x_dev_s *dev, uint8_t reg_addr,
                            uint16_t *value)
{
  uint8_t ret;
  uint8_t rx_buf[2];

  if ((ret = bq2742x_read(dev, reg_addr, rx_buf, sizeof(rx_buf))) !=
    sizeof(rx_buf))
    {
      bq2742x_dbg("failed, ret: %d, cmd: %02X\n", ret, reg_addr);
      return -EIO;
    }

  *value = ((uint16_t)rx_buf[1] << 8) | ((uint16_t)rx_buf[0]);
  return OK;
}

static int bq2742x_subcmd(struct bq2742x_dev_s *dev, uint16_t subcmd,
                          uint16_t *value)
{
  int ret;
  uint8_t wr_lsb[2] = {BQ2742X_CMD_CNTL, (uint8_t)(subcmd & 0xFF)};
  uint8_t wr_msb[2] = {BQ2742X_CMD_CNTL + 1, (uint8_t)((subcmd >> 8) & 0xFF)};
  uint8_t rx_buf[2];

  /* One byte write commands used here to ensure proper operation at bus
   * frequency 100 kHz >.
   */

  struct i2c_msg_s msg_lsb = {
    .addr   = dev->addr,
    .flags  = 0,
    .buffer = wr_lsb,
    .length = 2
  };

  struct i2c_msg_s msg_msb = {
    .addr   = dev->addr,
    .flags  = 0,
    .buffer = wr_msb,
    .length = 2
  };

  ret = bq2742x_do_transfer(dev, &msg_lsb, 1);
  if (ret < 0)
    {
      bq2742x_dbg("subcmd: %u, wr_lsb failed: %d\n", subcmd, ret);
      return ret;
    }

  ret = bq2742x_do_transfer(dev, &msg_msb, 1);
  if (ret < 0)
    {
      bq2742x_dbg("subcmd: %u, wr_msb failed: %d\n", subcmd, ret);
      return ret;
    }

  if (!value)
    return OK;

  if (bq2742x_read(dev, BQ2742X_CMD_CNTL, rx_buf, sizeof(rx_buf)) != sizeof(rx_buf))
    return -1;

  *value = rx_buf[1] << 8 | rx_buf[0];
  return OK;
}

static int bq2742x_is_sealed(FAR struct bq2742x_dev_s *priv, bool *sealed_state)
{
  uint16_t value;

  if (bq2742x_subcmd(priv, SUB_CMD_STATUS, &value))
    return -EIO;

  *sealed_state = (value & STATUS_SS);
  return OK;
}

static int bq2742x_unseal(FAR struct bq2742x_dev_s *priv)
{
  uint8_t i;
  int ret;

  for (i=0; i < 2; i++)
    {
      if ((ret = bq2742x_write_cmd(priv, BQ2742X_CMD_CNTL, BQ2742X_UNSEAL_KEY)))
        {
          bq2742x_dbg("unseal failed: %d\n", ret);
          return -1;
        }
    }

  return OK;
}

static int bq2742x_seal(FAR struct bq2742x_dev_s *priv)
{
  if (bq2742x_subcmd(priv, SUB_CMD_STATUS, NULL))
    return -EIO;

  return OK;
}

#ifdef CONFIG_BQ2742X_DEBUG
static int bq2742x_dump_regs(FAR struct bq2742x_dev_s *priv)
{
  int ret;
  uint8_t i;
  uint16_t value;
  const uint8_t cmds[] = {
    BQ2742X_CMD_TEMP, BQ2742X_CMD_VOLT, BQ2742X_CMD_FLAGS, BQ2742X_CMD_NAC,
    BQ2742X_CMD_FAC, BQ2742X_CMD_RM, BQ2742X_CMD_FCC, BQ2742X_CMD_AVG_CUR,
    BQ2742X_CMD_AVG_PWR, BQ2742X_CMD_SOC, BQ2742X_CMD_INT_TEMP, BQ2742X_CMD_SOH,
    BQ2742X_CMD_RCU, BQ2742X_CMD_RCF, BQ2742X_CMD_FCCU, BQ2742X_CMD_FCCF,
    BQ2742X_CMD_SOCU
    };

  const uint8_t subcmds[] = {
    SUB_CMD_STATUS, SUB_CMD_DEVICE_TYPE, SUB_CMD_FW_VER, SUB_CMD_DM_CODE,
    SUB_CMD_PREV_MACWR, SUB_CMD_CHEM_ID
    };

  (void)bq2742x_unseal(priv);

  for(i=0; i < sizeof(cmds); i++)
    {
      ret = bq2742x_read_cmd(priv, cmds[i], &value);
      bq2742x_dbg("0x%02X: 0x%04X %s\n", cmds[i], value, ret? "Failed":"");
    }

  for(i=0; i < sizeof(subcmds); i++)
    {
      ret = bq2742x_subcmd(priv, subcmds[i], &value);
      bq2742x_dbg("0x%04X: 0x%04X %s\n", subcmds[i], value, ret? "Failed":"");
    }

  return ret;
}
#endif /* CONFIG_BQ2742X_DEBUG */

#ifndef CONFIG_DISABLE_POLL
static void bq2742x_notify(FAR struct bq2742x_dev_s *priv)
{
  int i;

  DEBUGASSERT(priv != NULL);

  for (i = 0; i < CONFIG_BATTERY_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          bq2742x_lldbg("Report events: %02x\n", fds->revents);
          sem_post(fds->sem);
        }
    }
}
#endif

static int bq2742x_int_handler(int irq, FAR void *context)
{
  irqstate_t flags;

  flags = irqsave();
#ifndef CONFIG_DISABLE_POLL
  bq2742x_notify(g_gauge_data);
#endif
  irqrestore(flags);

  bq2742x_lldbg("interrupt\n");

  return OK;
}

static int bq2742x_enter_update_mode(FAR struct bq2742x_dev_s *dev)
{
  int ret;
  uint16_t flags = 0;
  uint16_t attempts = 0;

  if ((ret = bq2742x_subcmd(dev, SUB_CMD_SET_CFG_UPD, NULL)))
    return ret;

  do
    {
      (void)bq2742x_read_cmd(dev, BQ2742X_CMD_FLAGS, &flags);
      if (!(flags & FLAG_CFGUPMODE))
        usleep(500000);
    } while (!(flags & FLAG_CFGUPMODE) && (attempts++ < UPD_MODE_MAX_RETRY));

  return (attempts >= UPD_MODE_MAX_RETRY? -EIO: OK);
}

static int bq2742x_exit_update_mode(FAR struct bq2742x_dev_s *dev,
                                     uint16_t subcmd)
{
  int ret;
  uint16_t flags = FLAG_CFGUPMODE;
  uint16_t attempts = 0;

  if ((ret = bq2742x_subcmd(dev, subcmd, NULL)))
    return ret;

  do
    {
      (void)bq2742x_read_cmd(dev, BQ2742X_CMD_FLAGS, &flags);
      if (flags & FLAG_CFGUPMODE)
        usleep(500000);
    } while ((flags & FLAG_CFGUPMODE) && (attempts++ < UPD_MODE_MAX_RETRY));

  return (attempts >= UPD_MODE_MAX_RETRY? -EIO: OK);
}

static int bq2742x_read_data_class(FAR struct bq2742x_dev_s *dev,
                                   uint8_t dataclass, uint8_t *data,
                                   uint8_t len)
{
  uint8_t remainder = len;
  uint8_t datablock = 0;

  if (len < 1)
    return -1;

  do
    {
      len = remainder;
      if (len > 32)
        {
          remainder = len - 32;
          len = 32;
        }
      else
        {
          remainder = 0;
        }

      (void)bq2742x_write_cmd(dev, BQ2742X_DATA_CLASS,
        (datablock << 8) | dataclass);

      if (bq2742x_read(dev, BQ2742X_DATA_BLOCK, data, len) != len)
        return -EIO;

      data += len;
      datablock++;
    } while (remainder > 0);

  return OK;
}

static uint8_t bq2742x_check_sum(uint8_t *data, uint8_t len)
{
  uint8_t i, checksum = 0x00;

  for (i = 0; i < len; i++)
    checksum += data[i];

  return (0xFF - checksum);
}

static int bq2742x_write_data_class(FAR struct bq2742x_dev_s *dev,
                                    uint8_t dataclass, uint8_t *data,
                                    uint8_t len)
{
  uint8_t remainder = len;
  uint8_t checksum[2] = {0};
  uint8_t datablock = 0;
  uint16_t nData;

  if (len < 1)
    return 0;

  do
    {
      len = remainder;
      if (len < 32)
        {
          remainder = len - 32;
          len = 32;
        }
      else
        {
          remainder = 0;
        }

      nData = (datablock << 8) | dataclass;

      (void)bq2742x_write_cmd(dev, BQ2742X_DATA_CLASS, nData);

      if (bq2742x_write(dev, BQ2742X_DATA_BLOCK, data, len) != len)
        return -EIO;

      checksum[0] = bq2742x_check_sum(data, len);
      bq2742x_write(dev, BQ2742X_CHECKSUM, checksum, 1);

      usleep(10000);

      (void)bq2742x_write_cmd(dev, BQ2742X_DATA_CLASS, nData);
      bq2742x_read(dev, BQ2742X_CHECKSUM, checksum + 1, 1);
      if (checksum[0] != checksum[1])
        return -EIO;

      data += len;
      datablock++;
  } while (remainder > 0);

  return OK;
}

/****************************************************************************
 * Name: flashstream_execute
 *
 * Description:
 *   Parse and execute FlashStream configuration file.
 *
 *   Flash Stream relatad code based on Texas Instruments application report,
 *   Gauge Communication - slua801, January 2017.
 *
 * Parameters:
 *   dev - battery device
 *   pfs - pointer to null terminated FlashStream buffer.
 *
 * Returned Value:
 *   A pointer to the last processed FlashStream buffer character.
 *   Succesfully processed buffer would return pFS + len. In case of failure,
 *   pointer to failed buffer location is returned.
 *
 ****************************************************************************/

static const char *flashstream_execute(FAR struct bq2742x_dev_s *dev,
                                       const char * pfs, uint16_t len)
{
  int16_t datalen, n, m = 0;
  char buf[16], data[32];
  const char *pend = pfs + len;
  char *perr;
  uint8_t reg = 0;
  bool writecmd = false;

  do
  {
    switch (*pfs)
      {
        case ';':
          break;

        /* W - write, C - read and compare */
        case 'W':
        case 'C':
          writecmd = (*pfs == 'W');
          pfs++;
          if ((*pfs) != ':')
            return pfs;

          pfs++;

          n = 0;
          while ((pend - pfs > 2) && (n < sizeof(data) + 2) && (*pfs != '\n'))
            {
              buf[0] = *(pfs++);
              buf[1] = *(pfs++);
              buf[2] = 0;

              m = strtoul(buf, &perr, 16);
              if (*perr)
                return (pfs - 2);

              if (n == 1)
                reg = m;
              if (n > 1)
                data[n - 2] = m;
              n++;
            }

          if (n < 3)
            return pfs;

          datalen = n - 2;

          if (writecmd)
            {
              bq2742x_write(dev, reg, (uint8_t*)data, datalen);
            }
          else
            {
              uint8_t gauge_data[datalen];
              bq2742x_read(dev, reg, gauge_data, datalen);
              if (memcmp(data, gauge_data, datalen))
                {
                  bq2742x_dbg("data[0]:%02X verify failed, len: %d\n",
                    *data, datalen);
                  return pfs;
                }
            }
          break;

        /* X - wait given number of milliseconds */
        case 'X':
          pfs++;
          if ((*pfs) != ':')
            return pfs;

          pfs++;
          n = 0;
          while ((pfs != pend) && (*pfs != '\n') &&(n <sizeof(buf) - 1))
            {
              buf[n++] = *pfs;
              pfs++;
            }

          buf[n] = 0;
          n = atoi(buf);
          usleep(n * 1000);
          break;

        default:
          return pfs;
      }

    while ((pfs != pend) && (*pfs != '\n'))
      pfs++;

    if (pfs != pend)
      pfs++;

  } while (pfs != pend);

  return pfs;
}

/****************************************************************************
 * Name: bq2742x_config
 *
 * Description:
 *   Configure settings from FlashStream
 *
 * Parameters:
 *   dev - battery device
 *   config - pointer to FlashStream configuration buffer.
 *
 ****************************************************************************/

static int bq2742x_config(struct bq2742x_dev_s *priv, const void *config)
{
  char* flashstream = (char*)config;
  char* pch, *buf = NULL;
  const char *pRet;
  size_t len;
  int ret = 0;
  uint16_t spaces = 0, value;
  bool sealed_state = true;

  len = strlen(flashstream);
  bq2742x_dbg("flashstream len: %u\n", len);

  if (!len)
    return -EINVAL;

  /* Include trailing null in buffer length */

  len += 1;

  /* Count number of space characters in flash stream */

  pch = strchr(flashstream, ' ');
  while (pch != NULL)
    {
      pch = strchr(pch + 1, ' ');
      spaces++;
    }

  /* Strip spaces out from the flashstream buffer */

  if (spaces)
    {
      int src, dst = 0;

      if (!(buf = malloc(len - spaces)))
        return -ENOMEM;

      bq2742x_dbg("stripped flashstream len: %d\n", len - spaces);

      for (src = 0; src < len; src++)
        {
          if (flashstream[src] == ' ')
            continue;

          buf[dst++] = flashstream[src];
          bq2742x_dbg("%02X\n", buf[dst-1]);
        }

      len = dst;
      flashstream = buf;
    }

  if (bq2742x_subcmd(priv, SUB_CMD_FW_VER, &value))
    goto err_out;

  bq2742x_dbg("FW_VER: 0x%04X\n", value);

  bq2742x_is_sealed(priv, &sealed_state);

  if (sealed_state)
    {
      if ((ret = bq2742x_unseal(priv)))
        {
          bq2742x_dbg("failed to unseal\n");
          goto err_out;
        }
    }

  if ((ret = bq2742x_subcmd(priv, SUB_CMD_STATUS, &value)))
    goto err_seal;

  bq2742x_dbg("status: 0x%04X\n", value);

  if ((ret = bq2742x_enter_update_mode(priv)))
    {
      bq2742x_dbg("failed to enter in update mode\n");
      goto err_seal;
    }

  if ((pRet = flashstream_execute(priv, flashstream, len)) !=
    (flashstream + len))
    {
      bq2742x_dbg("flashstream execute failed at offset: %u\n",
        pRet - flashstream);
    }

  if ((ret = bq2742x_exit_update_mode(priv, SUB_CMD_SOFT_RST)))
    {
      bq2742x_dbg("failed to exit from update mode\n");
    }

err_seal:
  if (sealed_state)
    {
      bq2742x_seal(priv);
    }

err_out:
  if (buf)
    free (buf);

  return ret;
}

/****************************************************************************
 * Name: bq2742x_state
 *
 * Description:
 *   The current state of battery. Battery gauge flags decoding to state
 *   is affected by BQ2742X data memory configuration, such as FC set/clear
 *   threshold values.
 *
 ****************************************************************************/

static int bq2742x_state(struct battery_dev_s *dev, int *state)
{
  FAR struct bq2742x_dev_s *priv = (FAR struct bq2742x_dev_s *)dev;
  uint16_t flags, status;

  if (bq2742x_read_cmd(priv, BQ2742X_CMD_FLAGS, &flags))
    {
      return -EIO;
    }
  bq2742x_dbg("flags: %04X\n", flags);

  if (bq2742x_subcmd(priv, SUB_CMD_STATUS, &status))
    {
      return -EIO;
    }
  bq2742x_dbg("status: %04X\n", status);

  if (flags & FLAG_CFGUPMODE || !(status & STATUS_INITCOMP))
    {
      bq2742x_dbg("unknown\n");
      *state = BATTERY_UNKNOWN;
    }
  else if (status & STATUS_SLEEP)
    {
      bq2742x_dbg("idle\n");
      *state = BATTERY_IDLE;
    }
  else if (flags & FLAG_DSG)
    {
      bq2742x_dbg("discharging\n");
      *state = BATTERY_DISCHARGING;
    }
  else if (flags & FLAG_FC)
    {
      bq2742x_dbg("full\n");
      *state = BATTERY_FULL;
    }
  else
    {
      bq2742x_dbg("charging\n");
      *state = BATTERY_CHARGING;
    }

  return OK;
}

/****************************************************************************
 * Name: bq2742x_online
 *
 * Description:
 *   Battery is online when fuel gauge has detected battery insertion.
 *
 ****************************************************************************/

static int bq2742x_online(struct battery_dev_s *dev, bool *online)
{
  FAR struct bq2742x_dev_s *priv = (FAR struct bq2742x_dev_s *)dev;
  uint16_t flags;

  if (bq2742x_read_cmd(priv, BQ2742X_CMD_FLAGS, &flags))
    {
      return -EIO;
    }

  *online = (flags & FLAG_BAT_DET);

  return OK;
}

/****************************************************************************
 * Name: bq2742x_voltage
 *
 * Description:
 *   Current battery voltage in mV.
 *
 ****************************************************************************/

static int bq2742x_voltage(struct battery_dev_s *dev, b16_t *value)
{
  FAR struct bq2742x_dev_s *priv = (FAR struct bq2742x_dev_s *)dev;
  return bq2742x_read_cmd(priv, BQ2742X_CMD_VOLT, (uint16_t*)value);
}

/****************************************************************************
 * Name: bq2742x_current
 *
 * Description:
 *   Current battery current in mA. Negative value indicates discharging and
 *   positive value charging.
 *
 ****************************************************************************/

static int bq2742x_current(struct battery_dev_s *dev, b16_t *value)
{
  FAR struct bq2742x_dev_s *priv = (FAR struct bq2742x_dev_s *)dev;
  return bq2742x_read_cmd(priv, BQ2742X_CMD_AVG_CUR, (uint16_t*)value);
}

/****************************************************************************
 * Name: bq2742x_power
 *
 * Description:
 *   Current battery power in mW. Negative value indicates discharging and
 *   positive value charging.
 *
 ****************************************************************************/

static int bq2742x_power(struct battery_dev_s *dev, b16_t *value)
{
  FAR struct bq2742x_dev_s *priv = (FAR struct bq2742x_dev_s *)dev;
  return bq2742x_read_cmd(priv, BQ2742X_CMD_AVG_PWR, (uint16_t*)value);
}

/****************************************************************************
 * Name: bq2742x_capacity
 *
 * Description:
 *   Battery state of charge in percentage.
 *
 ****************************************************************************/

static int bq2742x_capacity(struct battery_dev_s *dev, b16_t *value)
{
  FAR struct bq2742x_dev_s *priv = (FAR struct bq2742x_dev_s *)dev;
  return bq2742x_read_cmd(priv, BQ2742X_CMD_SOC, (uint16_t*)value);
}

/****************************************************************************
 * Name: bq2742x_capacity_full
 *
 * Description:
 *   Battery full capacity in mAh.
 *
 ****************************************************************************/

static int bq2742x_capacity_full(struct battery_dev_s *dev, b16_t *value)
{
  FAR struct bq2742x_dev_s *priv = (FAR struct bq2742x_dev_s *)dev;
  return bq2742x_read_cmd(priv, BQ2742X_CMD_FCCU, (uint16_t*)value);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct battery_dev_s *bq2742x_initialize(FAR struct i2c_dev_s *i2c,
                                             uint8_t addr,
                                             struct bq2742x_config_s * config)
{
  FAR struct bq2742x_dev_s *priv;

  /* Initialize the BQ2742X device structure */

  priv = (FAR struct bq2742x_dev_s *)kmm_zalloc(sizeof(struct bq2742x_dev_s));
  if (!priv)
    {
      bq2742x_dbg("mem alloc failed\n");
      return NULL;
    }

  g_gauge_data = priv;
  sem_init(&priv->batsem, 0, 1);
  priv->ops = &bq2742x_ops;
  priv->i2c = i2c;
  priv->addr = addr;
  priv->config = config;

  priv->config->irq_clear(priv->config);
  priv->config->irq_attach(priv->config, bq2742x_int_handler);
  priv->config->irq_enable(priv->config, true);

  if (priv->config->flashstream)
    {
      if (bq2742x_config(priv, priv->config->flashstream))
        bq2742x_dbg("flashstream load failed\n");
    }

  return (FAR struct battery_dev_s *)priv;
}
