/****************************************************************************
 * drivers/input/lp5521.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>
#include <arpa/inet.h> /* HTONS */

#include <nuttx/arch.h>
#include <nuttx/i2c.h>
#include <nuttx/kmalloc.h>

#include <nuttx/led/lp5521.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LP5521_NPOLLWAITERS
#  define CONFIG_LP5521_NPOLLWAITERS 2
#endif

#ifdef CONFIG_LED_LP5521_DEBUG
#  define lp5521_dbg(x, ...)   dbg(x, ##__VA_ARGS__)
#  define lp5521_lldbg(x, ...) lldbg(x, ##__VA_ARGS__)
#else
#  define lp5521_dbg(x, ...)   ((void)0)
#  define lp5521_lldbg(x, ...) ((void)0)
#endif

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/* Register macros */

#define LP5521_REG_ENABLE                  0x00
#define LP5521_REG_OP_MODE                 0x01
#define LP5521_REG_R_PWM                   0x02
#define LP5521_REG_G_PWM                   0x03
#define LP5521_REG_B_PWM                   0x04
#define LP5521_REG_R_CURRENT               0x05
#define LP5521_REG_G_CURRENT               0x06
#define LP5521_REG_B_CURRENT               0x07
#define LP5521_REG_CONFIG                  0x08
#define LP5521_REG_R_PC                    0x09
#define LP5521_REG_G_PC                    0x0A
#define LP5521_REG_B_PC                    0x0B
#define LP5521_REG_STATUS                  0x0C
#define LP5521_REG_RESET                   0x0D
#define LP5521_REG_GPO                     0x0E

#define LP5521_REG_PROG_MEM_R_BASE         0x10
#define LP5521_REG_PROG_MEM_G_BASE         0x30
#define LP5521_REG_PROG_MEM_B_BASE         0x50

#define LP5521_REG_PROG_MEM_LEN            (16 * 2)

/* Register bits/masks */

#define LP5521_ENABLE_HOLD              0x0
#define LP5521_ENABLE_STEP              0x1
#define LP5521_ENABLE_RUN               0x2
#define LP5521_ENABLE_EXEC_ONE          0x3
#define LP5521_ENABLE_LOG_EN            (1 << 7)
#define LP5521_ENABLE_CHIP_EN           (1 << 6)
#define LP5521_ENABLE_R_EXEC_SHIFT      4
#define LP5521_ENABLE_G_EXEC_SHIFT      2
#define LP5521_ENABLE_B_EXEC_SHIFT      0
#define LP5521_ENABLE_R_EXEC_MASK       (0x3 << LP5521_ENABLE_R_EXEC_SHIFT)
#define LP5521_ENABLE_G_EXEC_MASK       (0x3 << LP5521_ENABLE_G_EXEC_SHIFT)
#define LP5521_ENABLE_B_EXEC_MASK       (0x3 << LP5521_ENABLE_B_EXEC_SHIFT)

#define LP5521_CONFIG_PWM_HF            (1 << 7)
#define LP5521_CONFIG_PWRSAVE_EN        (1 << 6)
#define LP5521_CONFIG_CP_MODE_OFF       0x0
#define LP5521_CONFIG_CP_MODE_1X        0x1
#define LP5521_CONFIG_CP_MODE_15X       0x2
#define LP5521_CONFIG_CP_MODE_AUTO      0x3
#define LP5521_CONFIG_CP_MODE_SHIFT     (3)
#define LP5521_CONFIG_CP_MODE_MASK      (0x3 << LP5521_CONFIG_CP_MODE_SHIFT)
#define LP5521_CONFIG_R_TO_BATT         (1 << 2)
#define LP5521_CONFIG_CLK_DET_EN        (1 << 1)
#define LP5521_CONFIG_INT_CLK_EN        (1 << 0)

#define LP5521_OP_MODE_DISABLED         0x0
#define LP5521_OP_MODE_LOAD_PROGRAM     0x1
#define LP5521_OP_MODE_RUN_PROGRAM      0x2
#define LP5521_OP_MODE_DIRECT_CONTROL   0x3
#define LP5521_OP_MODE_R_MODE_SHIFT     (4)
#define LP5521_OP_MODE_G_MODE_SHIFT     (2)
#define LP5521_OP_MODE_B_MODE_SHIFT     (0)
#define LP5521_OP_MODE_R_MODE_MASK      (0x3 << LP5521_OP_MODE_R_MODE_SHIFT)
#define LP5521_OP_MODE_G_MODE_MASK      (0x3 << LP5521_OP_MODE_G_MODE_SHIFT)
#define LP5521_OP_MODE_B_MODE_MASK      (0x3 << LP5521_OP_MODE_B_MODE_SHIFT)

#define LP5521_STATUS_EXT_CLK_USED      (1 << 3)
#define LP5521_STATUS_R_INT             (1 << 2)
#define LP5521_STATUS_G_INT             (1 << 1)
#define LP5521_STATUS_B_INT             (1 << 0)

#define LP5521_RESET_MASK               0xFF

/* Completion times for device commands */

/* Program command limits */

#define LP5521_MASTER_CLOCK_HZ                  32768U

#define LP5521_CMD_RAMP_WAIT_PRESCALE0_CLKDIV   16U
#define LP5521_CMD_RAMP_WAIT_PRESCALE1_CLKDIV   512U
#define LP5521_CMD_RAMP_WAIT_MIN_STEPTIME       1U
#define LP5521_CMD_RAMP_WAIT_MAX_STEPTIME       63U
#define LP5521_CMD_RAMP_WAIT_MIN_NUM_STEPS      1U
#define LP5521_CMD_RAMP_WAIT_MAX_NUM_STEPS      128U
#define LP5521_CMD_WAIT_NUM_STEPS               1U

#define LP5521_CMD_BRANCH_MIN_LOOPS             1U
#define LP5521_CMD_BRANCH_MAX_LOOPS             63U
#define LP5521_CMD_BRANCH_INFINITE_LOOP         0U

#define LP5521_CMD_RAMP_WAIT_MAX_USECS(prescale, num_steps) \
  ((uint32_t)((uint64_t)(num_steps) * 1000U * 1000U * (prescale) * \
      LP5521_CMD_RAMP_WAIT_MAX_STEPTIME / LP5521_MASTER_CLOCK_HZ))

#define LP5521_CMD_RAMP_WAIT_MIN_USECS(prescale) \
  ((uint32_t)((uint64_t)1000U * 1000U * (prescale) * \
      LP5521_CMD_RAMP_WAIT_MIN_STEPTIME / LP5521_MASTER_CLOCK_HZ))

#define LP5521_CMD_WAIT_MAX_USECS  \
    LP5521_CMD_RAMP_WAIT_MAX_USECS(LP5521_CMD_RAMP_WAIT_PRESCALE1_CLKDIV, \
                                   LP5521_CMD_WAIT_NUM_STEPS)

/* Other macros */

#define LP5521_I2C_RETRIES              5

#define LP5521_DEFAULT_CURRENT_VALUE    0xAF

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct lp5521_dev_s
{
  sem_t devsem;
  uint8_t cref;
  bool powered;

  /* I2C bus and address for device. */

  uint8_t addr;
  FAR struct i2c_dev_s *i2c;

  /* Configuration for device. */

  FAR const struct lp5521_board_s *board;
  FAR const struct lp5521_conf_s *config;

  /* Interrupt and poll handling */

  int int_pending;

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *fds[CONFIG_LP5521_NPOLLWAITERS];
#endif
};

/****************************************************************************
* Private Function Prototypes
*****************************************************************************/

static int lp5521_open(FAR struct file *filep);
static int lp5521_close(FAR struct file *filep);
static ssize_t lp5521_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t lp5521_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);

#ifndef CONFIG_DISABLE_POLL
static int lp5521_poll(struct file *filep, struct pollfd *fds, bool setup);
static void lp5521_notify(struct lp5521_dev_s *priv);
#endif

/****************************************************************************
* Private Data
****************************************************************************/

static FAR struct lp5521_dev_s *g_lp5521_dev;

static const struct file_operations g_lp5521_fileops = {
  lp5521_open,
  lp5521_close,
  lp5521_read,
  lp5521_write,
  0,
  0,
#ifndef CONFIG_DISABLE_POLL
  lp5521_poll
#endif
};

/****************************************************************************
* Private Functions
****************************************************************************/

static void lp5521_irq_clear(FAR struct lp5521_dev_s *dev)
{
  if (dev->board->irq_clear)
    {
      dev->board->irq_clear(dev->board);
    }
}

static int lp5521_irq_attach(FAR struct lp5521_dev_s *dev,
                             int (*handler)(void *priv), void *priv)
{
  if (dev->board->irq_attach)
    {
      return dev->board->irq_attach(dev->board, handler, priv);
    }

  return OK;
}

static void lp5521_irq_enable(FAR struct lp5521_dev_s *dev, bool enable)
{
  if (dev->board->irq_enable)
    {
      dev->board->irq_enable(dev->board, enable);
    }
}

static unsigned int lp5521_get_write_guardtime(uint8_t start_reg, size_t nregs)
{
  unsigned int curr_reg;
  unsigned int curr_guard;
  unsigned int pos;
  unsigned int max_guard = 0;

  for (pos = 0; pos < nregs; pos++)
    {
      curr_reg = start_reg + pos;

      /* Some registers are synchronized to 32kHz clock and need delay between
       * writes to the same register. */

      switch (curr_reg)
        {
          default:
            curr_guard = 0;
            break;
          case LP5521_REG_ENABLE:
            curr_guard = 488 /* µsec */;
            break;
          case LP5521_REG_OP_MODE:
          case LP5521_REG_R_PC:
          case LP5521_REG_G_PC:
          case LP5521_REG_B_PC:
            curr_guard = 153 /* µsec */;
            break;
        }

      if (curr_guard > max_guard)
        {
          max_guard = curr_guard;
        }
    }

  return max_guard;
}

static int lp5521_i2c_write(FAR struct lp5521_dev_s *dev, uint8_t reg,
                             const uint8_t *buf, size_t buflen)
{
  struct i2c_msg_s msgv[2] =
  { {
    .addr   = dev->addr,
    .flags  = 0,
    .buffer = &reg,
    .length = 1
  }, {
    .addr   = dev->addr,
    .flags  = I2C_M_NORESTART,
    .buffer = (void *)buf,
    .length = buflen
  } };
  int ret = -EIO;
  int retries;
  unsigned int delay_write_guardtime;

  /* LP5521 will respond with NACK to address when in low-power mode. Host
   * needs to retry address selection multiple times to get LP5521 to wake-up.
   */

  for (retries = 0; retries < LP5521_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(dev->i2c, msgv, 2);
      if (ret == -ENXIO)
        {
          /* -ENXIO is returned when getting NACK from response.
           * Keep trying. */

          continue;
        }

      if (ret >= 0)
        {
          /* Success! */

          delay_write_guardtime = lp5521_get_write_guardtime(reg, buflen);
          if (delay_write_guardtime)
            usleep(delay_write_guardtime);

          return 0;
        }
    }

  /* Failed to read sensor. */

  return ret;
}

static int lp5521_i2c_read(FAR struct lp5521_dev_s *dev, uint8_t reg,
                            uint8_t *buf, size_t buflen)
{
  struct i2c_msg_s msgv[2] =
  { {
    .addr   = dev->addr,
    .flags  = 0,
    .buffer = &reg,
    .length = 1
  }, {
    .addr   = dev->addr,
    .flags  = I2C_M_READ,
    .buffer = buf,
    .length = buflen
  } };
  int ret = -EIO;
  int retries;

  /* LP5521 will respond with NACK to address when in low-power mode. Host
   * needs to retry address selection multiple times to get LP5521 to wake-up.
   */

  for (retries = 0; retries < LP5521_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(dev->i2c, msgv, 2);
      if (ret == -ENXIO)
        {
          /* -ENXIO is returned when getting NACK from response.
           * Keep trying.*/

          continue;
        }
      else if (ret >= 0)
        {
          /* Success! */

          return 0;
        }
      else
        {
#ifdef CONFIG_I2C_RESET
          if (retries == LP5521_I2C_RETRIES - 1)
            break;

          /* Some other error. Try to reset I2C bus and keep trying. */

          ret = up_i2creset(dev->i2c);
          if (ret < 0)
            {
              lp5521_dbg("up_i2creset failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  /* Failed to read sensor. */

  return ret;
}

static int lp5521_startup_enable(FAR struct lp5521_dev_s *dev)
{
  uint8_t val;
  int ret;

  ret = lp5521_i2c_read(dev, LP5521_REG_ENABLE, &val, 1);
  if (ret < 0)
    {
      lp5521_dbg("%s read failed.\n", "LP5521_REG_ENABLE");
      return ret;
    }

  lp5521_dbg("before: LP5521_REG_ENABLE: %02x\n", val);

  val = LP5521_ENABLE_CHIP_EN;
  ret = lp5521_i2c_write(dev, LP5521_REG_ENABLE, &val, 1);
  if (ret < 0)
    {
      lp5521_dbg("%s:CHIP_EN write failed.\n", "LP5521_REG_ENABLE");
      return ret;
    }

  ret = lp5521_i2c_read(dev, LP5521_REG_ENABLE, &val, 1);
  if (ret < 0)
    {
      lp5521_dbg("%s read failed.\n", "LP5521_REG_ENABLE");
      return ret;
    }

  lp5521_dbg("after:  LP5521_REG_ENABLE: %02x\n", val);

  if (!(val & LP5521_ENABLE_CHIP_EN))
    {
      /* Could not enable? */

      return -ENXIO;
    }

  return 0;
}

static int lp5521_disable(FAR struct lp5521_dev_s *dev)
{
  uint8_t val;
  int ret;

  ret = lp5521_i2c_read(dev, LP5521_REG_ENABLE, &val, 1);
  if (ret < 0)
    {
      lp5521_dbg("%s read failed.\n", "LP5521_REG_ENABLE");
      return ret;
    }

  val &= ~LP5521_ENABLE_CHIP_EN;

  ret = lp5521_i2c_write(dev, LP5521_REG_ENABLE, &val, 1);
  if (ret < 0)
    {
      lp5521_dbg("%s write failed.\n", "LP5521_REG_ENABLE:~CHIP_EN");
      return ret;
    }

  return 0;
}

static int lp5521_reset(FAR struct lp5521_dev_s *dev)
{
  uint8_t val = LP5521_RESET_MASK;
  int ret;

  ret = lp5521_i2c_read(dev, LP5521_REG_ENABLE, &val, 1);
  if (ret < 0)
    {
      lp5521_dbg("%s read failed.\n", "LP5521_REG_ENABLE");
      return ret;
    }

  lp5521_dbg("before: LP5521_REG_ENABLE: %02x\n", val);

  /* Do reset. Note that chip does not reply with ACK after writing reset mask
   * to reset register. */

  val = LP5521_RESET_MASK;
  (void)lp5521_i2c_write(dev, LP5521_REG_RESET, &val, 1);
  usleep(10 * 1000);

  /* Check that chip has been reseted and not running. */

  ret = lp5521_i2c_read(dev, LP5521_REG_ENABLE, &val, 1);
  if (ret < 0)
    {
      lp5521_dbg("%s read failed.\n", "LP5521_REG_ENABLE");
      return ret;
    }

  lp5521_dbg("after:  LP5521_REG_ENABLE: %02x\n", val);

  if ((val & LP5521_ENABLE_CHIP_EN))
    {
      /* Should not be enable after reset. */

      return -ENXIO;
    }

  return 0;
}

static int lp5521_probe_device(FAR struct lp5521_dev_s *dev)
{
  uint8_t val[3] = {};
  int ret;

  /* Read LP5521_REG_R_CURRENT to LP5521_REG_G_CURRENT. */

  ret = lp5521_i2c_read(dev, LP5521_REG_R_CURRENT, &val[0], 3);
  if (ret < 0)
    {
      lp5521_dbg("%s read failed.\n", "LP5521_REG_RGB_CURRENT");
      return ret;
    }

  /* This function is called after device has been reseted. CURRENT registers
   * should have reset value of 0xAF. */

  if (val[0] == 0xAF &&
      val[1] == 0xAF &&
      val[2] == 0xAF)
    {
      return 0;
    }

  return -ENODEV;
}

static int lp5521_device_configuration(FAR struct lp5521_dev_s *dev,
                                       FAR const struct lp5521_conf_s *config)
{
  uint8_t conf_buf[LP5521_REG_B_PC + 1] = { };
  int ret;

  /* Prepare chip defaults. */

  conf_buf[LP5521_REG_ENABLE] = LP5521_ENABLE_CHIP_EN;
  conf_buf[LP5521_REG_R_CURRENT] = LP5521_DEFAULT_CURRENT_VALUE;
  conf_buf[LP5521_REG_G_CURRENT] = 0xAF;
  conf_buf[LP5521_REG_B_CURRENT] = 0xAF;

  /* Construct configuration register buffer based on 'config'. */

  if (config)
    {
      uint8_t clk_mode;
      uint8_t cp_mode;

      /* Generic configuration for LP5521. */

      if (config->logarithmic)
        conf_buf[LP5521_REG_ENABLE] |= LP5521_ENABLE_LOG_EN;

      if (config->auto_powersave)
        conf_buf[LP5521_REG_CONFIG] |= LP5521_CONFIG_PWRSAVE_EN;

      if (config->pwm_clock_558hz)
        conf_buf[LP5521_REG_CONFIG] |= LP5521_CONFIG_PWM_HF;

      if (config->r_channel_to_batt)
        conf_buf[LP5521_REG_CONFIG] |= LP5521_CONFIG_R_TO_BATT;

      switch (config->charge_pump_mode)
        {
          default:
            cp_mode = LP5521_CONFIG_CP_MODE_OFF;
            break;
          case LP5521_CHARGE_PUMP_MODE_FORCED_1X:
            cp_mode = LP5521_CONFIG_CP_MODE_1X;
            break;
          case LP5521_CHARGE_PUMP_MODE_FORCED_1_5X:
            cp_mode = LP5521_CONFIG_CP_MODE_15X;
            break;
          case LP5521_CHARGE_PUMP_MODE_AUTOMATIC_SELECTION:
            cp_mode = LP5521_CONFIG_CP_MODE_AUTO;
            break;
        }

      conf_buf[LP5521_REG_CONFIG] |= cp_mode << LP5521_CONFIG_CP_MODE_SHIFT;

      switch (config->clock_select)
        {
          default:
          case LP5521_CLOCK_EXTERNAL_CLK_32K:
            clk_mode = 0;
            break;
          case LP5521_CLOCK_INTERNAL:
            clk_mode = LP5521_CONFIG_INT_CLK_EN;
            break;
          case LP5521_CLOCK_AUTOMATIC_SELECTION:
            clk_mode = LP5521_CONFIG_CLK_DET_EN;
            break;
        }

      conf_buf[LP5521_REG_CONFIG] |= clk_mode;

      /* Per channel configuration. */

      conf_buf[LP5521_REG_R_PWM] = config->r.initial_pwm;
      conf_buf[LP5521_REG_G_PWM] = config->g.initial_pwm;
      conf_buf[LP5521_REG_B_PWM] = config->b.initial_pwm;

      if (config->r.current >= 0)
        conf_buf[LP5521_REG_R_CURRENT] =
            (config->r.current > 25500 ? 25500 : config->r.current) / 100;
      if (config->g.current >= 0)
        conf_buf[LP5521_REG_G_CURRENT] =
            (config->g.current > 25500 ? 25500 : config->g.current) / 100;
      if (config->b.current >= 0)
        conf_buf[LP5521_REG_B_CURRENT] =
            (config->b.current > 25500 ? 25500 : config->b.current) / 100;

      switch (config->r.initial_mode)
        {
          case LP5521_CHANNEL_MODE_DISABLED:
            conf_buf[LP5521_REG_OP_MODE] |=
                LP5521_OP_MODE_DISABLED << LP5521_OP_MODE_R_MODE_SHIFT;
            break;
          case LP5521_CHANNEL_MODE_DIRECT_CONTROL:
            conf_buf[LP5521_REG_OP_MODE] |=
                LP5521_OP_MODE_DIRECT_CONTROL << LP5521_OP_MODE_R_MODE_SHIFT;
            break;
          default:
            /* TODO: initial program load & run. */
            break;
        }

      switch (config->g.initial_mode)
        {
          case LP5521_CHANNEL_MODE_DISABLED:
            conf_buf[LP5521_REG_OP_MODE] |=
                LP5521_OP_MODE_DISABLED << LP5521_OP_MODE_G_MODE_SHIFT;
            break;
          case LP5521_CHANNEL_MODE_DIRECT_CONTROL:
            conf_buf[LP5521_REG_OP_MODE] |=
                LP5521_OP_MODE_DIRECT_CONTROL << LP5521_OP_MODE_G_MODE_SHIFT;
            break;
          default:
            break;
        }

      switch (config->b.initial_mode)
        {
          case LP5521_CHANNEL_MODE_DISABLED:
            conf_buf[LP5521_REG_OP_MODE] |=
                LP5521_OP_MODE_DISABLED << LP5521_OP_MODE_B_MODE_SHIFT;
            break;
          case LP5521_CHANNEL_MODE_DIRECT_CONTROL:
            conf_buf[LP5521_REG_OP_MODE] |=
                LP5521_OP_MODE_DIRECT_CONTROL << LP5521_OP_MODE_B_MODE_SHIFT;
            break;
          default:
            break;
        }
    }

  /* Write configuration. */

  ret = lp5521_i2c_write(dev, LP5521_REG_ENABLE, conf_buf, sizeof(conf_buf));
  if (ret < 0)
    {
      lp5521_dbg("configuration write failed.\n");
      return ret;
    }

  return 0;
}

static int lp5521_set_pwm(FAR struct lp5521_dev_s *dev,
                          FAR const struct lp5521_cmd_pwm_s *_pwm)
{
  struct lp5521_cmd_pwm_s pwm = *_pwm;
  int ret;
  uint8_t val;

  if ((pwm.r < -1 || pwm.r > 255) ||
      (pwm.g < -1 || pwm.g > 255) ||
      (pwm.b < -1 || pwm.b > 255))
    {
      return -EINVAL;
    }

  if (pwm.r > -1)
    {
      val = pwm.r;
      ret = lp5521_i2c_write(dev, LP5521_REG_R_PWM, &val, sizeof(val));
      if (ret < 0)
        {
          lp5521_dbg("%s:%d write failed.\n", "LP5521_REG_R_PWM", val);
          return ret;
        }

      lp5521_dbg("%c channel: %d\n", 'R', val);
    }

  if (pwm.g > -1)
    {
      val = pwm.g;
      ret = lp5521_i2c_write(dev, LP5521_REG_G_PWM, &val, sizeof(val));
      if (ret < 0)
        {
          lp5521_dbg("%s:%d write failed.\n", "LP5521_REG_G_PWM", val);
          return ret;
        }

      lp5521_dbg("%c channel: %d\n", 'G', val);
    }

  if (pwm.b > -1)
    {
      val = pwm.b;
      ret = lp5521_i2c_write(dev, LP5521_REG_B_PWM, &val, sizeof(val));
      if (ret < 0)
        {
          lp5521_dbg("%s:%d write failed.\n", "LP5521_REG_B_PWM", val);
          return ret;
        }

      lp5521_dbg("%c channel: %d\n", 'B', val);
    }

  return 0;
}

static int lp5521_set_mode(FAR struct lp5521_dev_s *dev,
                           FAR const struct lp5521_cmd_mode_s *mode)
{
  static const struct {
    uint8_t shift;
    uint8_t mask;
    uint8_t enableshift;
    uint8_t enablemask;
  } modes[3] =
  {
    {
      .shift            = LP5521_OP_MODE_R_MODE_SHIFT,
      .mask             = LP5521_OP_MODE_R_MODE_MASK,
      .enableshift      = LP5521_ENABLE_R_EXEC_SHIFT,
      .enablemask       = LP5521_ENABLE_R_EXEC_MASK,
    },
    {
      .shift            = LP5521_OP_MODE_G_MODE_SHIFT,
      .mask             = LP5521_OP_MODE_G_MODE_MASK,
      .enableshift      = LP5521_ENABLE_G_EXEC_SHIFT,
      .enablemask       = LP5521_ENABLE_G_EXEC_MASK,
    },
    {
      .shift            = LP5521_OP_MODE_B_MODE_SHIFT,
      .mask             = LP5521_OP_MODE_B_MODE_MASK,
      .enableshift      = LP5521_ENABLE_B_EXEC_SHIFT,
      .enablemask       = LP5521_ENABLE_B_EXEC_MASK,
    }
  };
  enum lp5521_channel_mode_e ms[ARRAY_SIZE(modes)] =
    { mode->r, mode->g, mode->b };
  int ret;
  uint8_t modereg = 0;
  uint8_t enablereg = 0;
  uint8_t modemask = 0;
  uint8_t newmodes = 0;
  uint8_t enablemask = 0;
  uint8_t newenables = 0;
  uint8_t stopmodes = 0;
  uint8_t stopmodesmask = 0;
  uint8_t stopenable = 0;
  uint8_t stopenablemask = 0;
  uint8_t currmode;
  int i;

  for (i = 0; i < ARRAY_SIZE(modes); i++)
    {
      switch (ms[i])
        {
        case LP5521_CHANNEL_MODE_NO_CHANGE:
        default:
          break;
        case LP5521_CHANNEL_MODE_DISABLED:
          newmodes |= LP5521_OP_MODE_DISABLED << modes[i].shift;
          modemask |= modes[i].mask;
          break;
        case LP5521_CHANNEL_MODE_DIRECT_CONTROL:
          newmodes |= LP5521_OP_MODE_DIRECT_CONTROL << modes[i].shift;
          modemask |= modes[i].mask;
          break;
        case LP5521_CHANNEL_MODE_RUN_PROGRAM:
          newmodes |= LP5521_OP_MODE_RUN_PROGRAM << modes[i].shift;
          modemask |= modes[i].mask;
          newenables |= LP5521_ENABLE_RUN << modes[i].enableshift;
          enablemask |= modes[i].enablemask;
          break;
        case LP5521_CHANNEL_MODE_LOAD_PROGRAM:
          newmodes |= LP5521_OP_MODE_LOAD_PROGRAM << modes[i].shift;
          modemask |= modes[i].mask;
          break;
        }
    }

  DEBUGASSERT((modemask & newmodes) == newmodes);
  DEBUGASSERT((enablemask & newenables) == newenables);

  if (modemask == 0)
    {
      /* No change for all channels. */

      return 0;
    }

  /* Read registers. */

  ret = lp5521_i2c_read(dev, LP5521_REG_OP_MODE, &modereg, sizeof(modereg));
  if (ret < 0)
    {
      lp5521_dbg("%s read failed.\n", "LP5521_REG_OP_MODE");
      return ret;
    }

  ret = lp5521_i2c_read(dev, LP5521_REG_ENABLE, &enablereg, sizeof(enablereg));
  if (ret < 0)
    {
      lp5521_dbg("%s read failed.\n", "LP5521_REG_ENABLE");
      return ret;
    }

  lp5521_dbg("before: OP_MODE: %08b, ENABLE: %08b\n", modereg, enablereg);

  /* Leaving RUN mode requires extra steps: set execution to hold and disable
   * operation mode. */

  for (i = 0; i < ARRAY_SIZE(modes); i++)
    {
      if (ms[i] != LP5521_CHANNEL_MODE_NO_CHANGE)
        {
          currmode = (modereg & modes[i].mask) >> modes[i].shift;
          if (currmode == LP5521_OP_MODE_RUN_PROGRAM)
            {
              stopenable     |= LP5521_ENABLE_HOLD << modes[i].enableshift;
              stopenablemask |= modes[i].enablemask;
              stopmodes     |= LP5521_OP_MODE_DISABLED << modes[i].shift;
              stopmodesmask |= modes[i].mask;
            }
        }
    }

  DEBUGASSERT((stopmodesmask & stopmodes) == stopmodes);
  DEBUGASSERT((stopenablemask & stopenable) == stopenable);

  if (stopmodesmask)
    {
      /* Modify register. */

      enablereg &= ~stopenablemask;
      enablereg |= stopenable;

      /* Write register. */

      ret = lp5521_i2c_write(dev, LP5521_REG_ENABLE, &enablereg,
                             sizeof(enablereg));
      if (ret < 0)
        {
          lp5521_dbg("%s:%x write failed.\n", "LP5521_REG_ENABLE", enablereg);
          return ret;
        }

      /* Modify register. */

      modereg &= ~stopmodesmask;
      modereg |= stopmodes;

      /* Write register. */

      ret = lp5521_i2c_write(dev, LP5521_REG_OP_MODE, &modereg,
                             sizeof(modereg));
      if (ret < 0)
        {
          lp5521_dbg("%s:%x write failed.\n", "LP5521_REG_OP_MODE", modereg);
          return ret;
        }
    }

  /* Modify register. */

  modereg &= ~modemask;
  modereg |= newmodes;

  /* Write register. */

  ret = lp5521_i2c_write(dev, LP5521_REG_OP_MODE, &modereg, sizeof(modereg));
  if (ret < 0)
    {
      lp5521_dbg("%s:%x write failed.\n", "LP5521_REG_OP_MODE", modereg);
      return ret;
    }

  if (enablemask)
    {
      /* Modify register. */

      enablereg &= ~enablemask;
      enablereg |= newenables;

      /* Write register. */

      ret = lp5521_i2c_write(dev, LP5521_REG_ENABLE, &enablereg,
                             sizeof(enablereg));
      if (ret < 0)
        {
          lp5521_dbg("%s:%x write failed.\n", "LP5521_REG_ENABLE", enablereg);
          return ret;
        }
    }

  lp5521_dbg("after:  OP_MODE: %08b, ENABLE: %08b\n", modereg, enablereg);

  return 0;
}

static int lp5521_get_status(FAR struct lp5521_dev_s *dev,
                             FAR struct lp5521_status_s *status)
{
  uint8_t regval;
  int ret;

  /* Read register. */

  ret = lp5521_i2c_read(dev, LP5521_REG_STATUS, &regval, sizeof(regval));
  if (ret < 0)
    {
      lp5521_dbg("%s read failed.\n", "LP5521_REG_STATUS");
      return ret;
    }

  status->r_int = !!(regval & LP5521_STATUS_R_INT);
  status->g_int = !!(regval & LP5521_STATUS_G_INT);
  status->b_int = !!(regval & LP5521_STATUS_B_INT);
  status->ext_clk_used = !!(regval & LP5521_STATUS_EXT_CLK_USED);

  return 0;
}

static int
lp5521_load_program(FAR struct lp5521_dev_s *dev,
                    FAR const struct lp5521_cmd_load_program_s *loadprog)
{
  struct lp5521_cmd_mode_s mode = {};
  FAR const struct lp5521_program_s *prog = &loadprog->program;
  uint8_t start_reg;
  const char *reg_name;
  int ret;

  if (prog->ncommands > LP5521_MAX_PROGRAM_COMMANDS)
    {
      return -EINVAL;
    }

  mode.type = LED_LP5521_CMD_SET_MODE;
  mode.r = LP5521_CHANNEL_MODE_NO_CHANGE;
  mode.g = LP5521_CHANNEL_MODE_NO_CHANGE;
  mode.b = LP5521_CHANNEL_MODE_NO_CHANGE;

  switch (loadprog->channel)
    {
    case LP5521_CHANNEL_R:
      mode.r = LP5521_CHANNEL_MODE_LOAD_PROGRAM;
      start_reg = LP5521_REG_PROG_MEM_R_BASE;
      reg_name = "LP5521_REG_PROG_MEM_R_BASE";
      break;

    case LP5521_CHANNEL_G:
      mode.g = LP5521_CHANNEL_MODE_LOAD_PROGRAM;
      start_reg = LP5521_REG_PROG_MEM_G_BASE;
      reg_name = "LP5521_REG_PROG_MEM_G_BASE";
      break;

    case LP5521_CHANNEL_B:
      mode.b = LP5521_CHANNEL_MODE_LOAD_PROGRAM;
      start_reg = LP5521_REG_PROG_MEM_B_BASE;
      reg_name = "LP5521_REG_PROG_MEM_B_BASE";
      break;

    default:
      DEBUGASSERT(false);
    }

  /* Prepare channel mode for loading program. */

  ret = lp5521_set_mode(dev, &mode);
  if (ret < 0)
    {
      lp5521_dbg("lp5521_set_mode failed, channel=%d.\n", loadprog->channel);
      return ret;
    }

  /* Load program. */

  ret = lp5521_i2c_write(dev, start_reg, (const void *)prog->commands,
                         sizeof(prog->commands[0]) * prog->ncommands);
  if (ret < 0)
    {
      lp5521_dbg("%s write failed.\n", reg_name);
      (void)reg_name;
      return ret;
    }

  return ret;
}

static void lp5521_power_off(FAR struct lp5521_dev_s *dev)
{
  /* Disable chip. */

  (void)lp5521_disable(dev);

  /* Power off. */

  (void)dev->board->set_enable(dev->board, false);
  (void)dev->board->set_power(dev->board, false);

  dev->int_pending = 0;
  dev->powered = false;
  lp5521_dbg("Powered OFF.\n");
}

static int lp5521_power_on(FAR struct lp5521_dev_s *dev,
                           FAR const struct lp5521_conf_s *config)
{
  int ret;

  /* Power on. */

  ret = dev->board->set_power(dev->board, true);
  if (ret < 0)
    {
      lp5521_dbg("set_power=true failed.\n");
      return ret;
    }

  /* Disable & enable device. */

  ret = dev->board->set_enable(dev->board, false);
  if (ret < 0)
    {
      lp5521_dbg("set_enable=false failed.\n");
      return ret;
    }

  usleep(1000);

  ret = dev->board->set_enable(dev->board, true);
  if (ret < 0)
    {
      lp5521_dbg("set_enable=true failed.\n");
      return ret;
    }

  usleep(1000);

  /* Reset device (in case old configuration is lurking on device). */

  ret = lp5521_reset(dev);
  if (ret < 0)
    {
      /* Could not reset. */

      (void)dev->board->set_enable(dev->board, false);
      (void)dev->board->set_power(dev->board, false);

      lp5521_dbg("lp5521_reset failed.\n");
      return ret;
    }

  /* Enable device. */

  ret = lp5521_startup_enable(dev);
  if (ret < 0)
    {
      /* Could not enable. */

      (void)dev->board->set_enable(dev->board, false);
      (void)dev->board->set_power(dev->board, false);

      lp5521_dbg("lp5521_startup_enable failed.\n");
      return ret;
    }

  /* Check that device exists on I2C. */

  ret = lp5521_probe_device(dev);
  if (ret < 0)
    {
      /* No such device. Power off the switch. */

      (void)dev->board->set_enable(dev->board, false);
      (void)dev->board->set_power(dev->board, false);

      lp5521_dbg("lp5521_probe_device failed.\n");
      return ret;
    }

  if (config)
    {
      /* Do configuration. */

      ret = lp5521_device_configuration(dev, config);
      if (ret < 0)
        {
          /* Configuration failed. Power off the switch. */

          (void)dev->board->set_enable(dev->board, false);
          (void)dev->board->set_power(dev->board, false);

          lp5521_dbg("lp5521_device_configuration failed.\n");
          return ret;
        }
    }

  dev->int_pending = 0;
  dev->powered = true;
  lp5521_dbg("Powered ON.\n");
  return 0;
}

static ssize_t lp5521_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode;
  FAR struct lp5521_dev_s *priv;
  enum lp5521_cmd_e type;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  if (buflen < sizeof(enum lp5521_cmd_e))
    {
      return -EINVAL;
    }

  while (sem_wait(&priv->devsem) != 0)
    {
      assert(errno == EINTR);
    }

  type = *(FAR const enum lp5521_cmd_e *)buffer;

  switch (type)
    {
    case LED_LP5521_CMD_SET_PWM:
      {
        FAR const struct lp5521_cmd_pwm_s *pwm =
            (FAR const struct lp5521_cmd_pwm_s *)buffer;

        if (buflen != sizeof(*pwm))
          {
            ret = -EINVAL;
            goto out;
          }

        ret = lp5521_set_pwm(priv, pwm);
      }
      break;
    case LED_LP5521_CMD_SET_MODE:
      {
        FAR const struct lp5521_cmd_mode_s *mode =
            (FAR const struct lp5521_cmd_mode_s *)buffer;

        if (buflen != sizeof(*mode))
          {
            ret = -EINVAL;
            goto out;
          }

        ret = lp5521_set_mode(priv, mode);
      }
      break;
    case LED_LP5521_CMD_RECONFIGURE:
      {
        FAR const struct lp5521_cmd_reconfig_s *reconf =
            (FAR const struct lp5521_cmd_reconfig_s *)buffer;

        if (buflen != sizeof(*reconf))
          {
            ret = -EINVAL;
            goto out;
          }

        (void)lp5521_power_off(priv);

        ret = lp5521_power_on(priv, &reconf->config);
      }
      break;
    case LED_LP5521_CMD_LOAD_PROGRAM:
      {
        FAR const struct lp5521_cmd_load_program_s *loadprog =
            (FAR const struct lp5521_cmd_load_program_s *)buffer;

        if (buflen != sizeof(*loadprog))
          {
            ret = -EINVAL;
            goto out;
          }

        ret = lp5521_load_program(priv, loadprog);
      }
      break;
    default:
      ret = -EINVAL;
      break;
    }

out:
  sem_post(&priv->devsem);

  return ret < 0 ? ret : buflen;
}

static ssize_t lp5521_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode;
  FAR struct lp5521_dev_s *priv;
  struct lp5521_status_s status = {};
  irqstate_t flags;
  int ret;

  if (buflen < sizeof(status))
    {
      return -EINVAL;
    }

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  while (sem_wait(&priv->devsem) != 0)
    {
      assert(errno == EINTR);
    }

  status.powered = priv->powered;
  if (status.powered)
    {
      /* Get IRQ status. */

      ret = lp5521_get_status(priv, &status);
      if (ret < 0)
        {
          sem_post(&priv->devsem);
          return ret;
        }
    }

  flags = irqsave();
  if (priv->int_pending)
    {
      priv->int_pending--;
#ifndef CONFIG_DISABLE_POLL
      if (priv->int_pending)
        {
          lp5521_notify(priv);
        }
#endif
    }
  irqrestore(flags);

  sem_post(&priv->devsem);

  memcpy(buffer, &status, sizeof(status));
  return sizeof(status);
}

#ifndef CONFIG_DISABLE_POLL

static int lp5521_poll(struct file *filep, struct pollfd *fds, bool setup)
{
  FAR struct inode *inode;
  FAR struct lp5521_dev_s *priv;
  irqstate_t flags;
  int ret = OK;
  int i;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      return -EINTR;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_LP5521_NPOLLWAITERS; i++)
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

      if (i >= CONFIG_LP5521_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }
      flags = irqsave();
      if (priv->int_pending)
        {
          lp5521_notify(priv);
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

static void lp5521_notify(struct lp5521_dev_s *priv)
{
  int i;

  for (i = 0; i < CONFIG_LP5521_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          sem_post(fds->sem);
        }
    }
}

#endif /* !CONFIG_DISABLE_POLL */

static int lp5521_int_handler(void *_priv)
{
  FAR struct lp5521_dev_s *priv = _priv;
  irqstate_t flags;

  flags = irqsave();

  priv->int_pending++;

#ifndef CONFIG_DISABLE_POLL
  lp5521_notify(priv);
#endif

  irqrestore(flags);

  return OK;
}

static int lp5521_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct lp5521_dev_s *priv;
  unsigned int use_count;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  while (sem_wait(&priv->devsem) != 0)
    {
      assert(errno == EINTR);
    }

  use_count = priv->cref + 1;
  if (use_count == 1)
    {
      /* First user, do power on. */

      ret = lp5521_power_on(priv, priv->config);
      if (ret < 0)
        {
          goto out_sem;
        }

      lp5521_irq_enable(priv, true);
      priv->cref = use_count;
    }
  else
    {
      DEBUGASSERT(use_count < UINT8_MAX && use_count > priv->cref);

      priv->cref = use_count;
      ret = 0;
    }

out_sem:
  sem_post(&priv->devsem);
  return ret;
}

static int lp5521_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct lp5521_dev_s *priv;
  int use_count;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  while (sem_wait(&priv->devsem) != 0)
    {
      assert(errno == EINTR);
    }

  use_count = priv->cref - 1;
  if (use_count == 0)
    {
      lp5521_irq_enable(priv, false);

      /* Last user, do power off. */

      (void)lp5521_power_off(priv);

      priv->cref = use_count;
    }
  else
    {
      DEBUGASSERT(use_count > 0);

      priv->cref = use_count;
    }

  sem_post(&priv->devsem);

  return 0;
}

static uint16_t lp5521_command_wait_ramp(unsigned int usecs, int nsteps)
{
  unsigned int prescale = 1;
  unsigned int sign = 0;
  unsigned int increment;
  unsigned int step_usecs;
  unsigned int cycle_usecs;
  unsigned int ncycles;
  uint16_t cmd;

  if (nsteps < 0)
    {
      /* Negative direction. */

      sign = 1;
      nsteps = -nsteps;
    }

  if (nsteps == 0)
    {
      /* Wait command. */

      increment = 0;
    }
  else
    {
      /* Range, 1 to 128. Note: nsteps == 1, same as wait command. */

      increment = nsteps - 1;
    }

  /* Length of one step. */

  step_usecs = usecs / (increment + 1);

  /* Select prescale / cycle-time. */

  if (step_usecs > LP5521_CMD_RAMP_WAIT_MAX_USECS(
                                    LP5521_CMD_RAMP_WAIT_PRESCALE0_CLKDIV, 1U))
    {
      prescale = 1;
      cycle_usecs = LP5521_CMD_RAMP_WAIT_MIN_USECS(
                          LP5521_CMD_RAMP_WAIT_PRESCALE1_CLKDIV);
    }
  else
    {
      prescale = 0;
      cycle_usecs = LP5521_CMD_RAMP_WAIT_MIN_USECS(
                          LP5521_CMD_RAMP_WAIT_PRESCALE0_CLKDIV);
    }

  ncycles = step_usecs / cycle_usecs;
  ncycles += (ncycles == 0);

  /* Generate ramp/wait command. */

  cmd = 0x0 << 15;
  cmd += (prescale & 1) << 14;
  cmd += (ncycles & 0x3F) << 8;
  cmd += (sign & 1) << 7;
  cmd += (increment & 0x7F) << 0;

  return HTONS(cmd);
}

static uint16_t lp5521_command_set_pwm(uint8_t pwm)
{
  uint16_t cmd;

  cmd = 0x40 << 8;
  cmd += pwm << 0;

  return HTONS(cmd);
}

static uint16_t lp5521_command_start(void)
{
  return HTONS(0x0000);
}

static  uint16_t lp5521_command_branch(unsigned int stepnum,
                                       unsigned int nloops)
{
  uint16_t cmd;

  cmd = 0x5 << 13;
  cmd += (nloops & 0x3F) << 7;
  cmd += (stepnum & 0xF) << 0;

  return HTONS(cmd);
}

static uint16_t lp5521_command_end(bool sendint, bool resetpwm)
{
  uint16_t cmd;

  cmd = 0x6 << 13;
  cmd += (!!sendint) << 12;
  cmd += (!!resetpwm) << 11;

  return HTONS(cmd);
}

static uint16_t lp5521_command_trigger(unsigned int wait_trigger,
                                       unsigned int send_trigger)
{
  uint16_t cmd;

  cmd = 0x7 << 13;
  cmd += (wait_trigger & 0x3F) << 7;
  cmd += (send_trigger & 0x3F) << 1;

  return HTONS(cmd);
}

static unsigned int lp5521_command_triggerflags(bool r, bool g, bool b,
                                                bool ext)
{
  unsigned int flags = 0;

  flags += !!r << 0;
  flags += !!g << 1;
  flags += !!b << 2;
  flags += !!ext << 5;

  return flags;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int led_lp5521_register(FAR const char *devpath,
                        FAR struct i2c_dev_s *i2c_bus,
                        uint8_t i2c_devaddr,
                        FAR const struct lp5521_board_s *board_config,
                        FAR const struct lp5521_conf_s *chip_config)
{
  struct lp5521_dev_s *priv;
  int ret = 0;

  /* Allocate device private structure. */

  g_lp5521_dev = kmm_zalloc(sizeof(struct lp5521_dev_s));
  if (!g_lp5521_dev)
    {
      lp5521_dbg("Memory cannot be allocated for lp5521\n");
      return -ENOMEM;
    }

  /* Setup device structure. */

  priv = g_lp5521_dev;
  priv->addr = i2c_devaddr;
  priv->i2c = i2c_bus;
  priv->board = board_config;
  priv->config = chip_config;

  sem_init(&priv->devsem, 0, 1);

  ret = register_driver(devpath, &g_lp5521_fileops, 0666, priv);
  if (ret < 0)
    {
      kmm_free(g_lp5521_dev);
      g_lp5521_dev = NULL;
      lp5521_dbg("Error occurred during the driver registering\n");
      return ret;
    }

  lp5521_dbg("Registered with %d\n", ret);

  /* Initialize GPIOs. */

  priv->board->set_power(priv->board, false);
  priv->board->set_enable(priv->board, false);

  /* Prepare interrupt line and handler. */

  lp5521_irq_clear(priv);
  lp5521_irq_attach(priv, lp5521_int_handler, priv);
  lp5521_irq_enable(priv, false);

  return 0;
}

int lp5521_program_wait(struct lp5521_program_s *prog, unsigned int msecs)
{
  unsigned int usecs = msecs * 1000;
  unsigned int wait_usecs;
  unsigned int short_wait_usecs = 0;
  unsigned int nloops;
  unsigned int ncmds;

  if (usecs / 1000 != msecs)
    {
      return -EINVAL;
    }

  if (msecs == 0)
    {
      return 0; /* Do nothing. */
    }
  else if ((usecs > LP5521_CMD_RAMP_WAIT_MAX_USECS(
                             LP5521_CMD_RAMP_WAIT_PRESCALE0_CLKDIV, 1U)) &&
           (usecs < LP5521_CMD_RAMP_WAIT_MIN_USECS(
                             LP5521_CMD_RAMP_WAIT_PRESCALE1_CLKDIV) * 20U))
    {
      const unsigned int prescale1_min_usecs =
          LP5521_CMD_RAMP_WAIT_MIN_USECS(LP5521_CMD_RAMP_WAIT_PRESCALE1_CLKDIV);

      /* With msec > 30 ms, we could use prescale=1 and not use loop, but then
       * wait time will increase in 15.6 ms steps, causing inaccuracy for short
       * waits. Form wait from one longer wait with prescale=1 (to utilize
       * automatic power-save) and one shorter wait with prescale=0. */

      wait_usecs = usecs / prescale1_min_usecs;
      wait_usecs *= prescale1_min_usecs;
      short_wait_usecs = usecs - wait_usecs;

      nloops = 0;
    }
  else if (usecs <= LP5521_CMD_WAIT_MAX_USECS)
    {
      /* Generate simple single command wait. */

      wait_usecs = usecs;
      nloops = 0;
    }
  else if (usecs <= (LP5521_CMD_BRANCH_MAX_LOOPS + 1) *
                     LP5521_CMD_WAIT_MAX_USECS)
    {
      /* Very long wait (> 984 ms), use loop. */

      nloops = usecs / LP5521_CMD_WAIT_MAX_USECS;
      nloops += !!(usecs % LP5521_CMD_WAIT_MAX_USECS);

      wait_usecs = usecs / nloops;
    }
  else
    {
      return -EINVAL;
    }

  /* Enough space left for these commands? */

  ncmds = prog->ncommands + 1 + (nloops > 1) + (short_wait_usecs > 0);
  if (ncmds > LP5521_MAX_PROGRAM_COMMANDS)
    {
      return -ENOBUFS;
    }

  ncmds = prog->ncommands;

  /* Insert wait command. */

  prog->commands[ncmds++] = lp5521_command_wait_ramp(wait_usecs, 0);

  if (short_wait_usecs > 0)
    {
      /* Insert additional short wait command. */

      prog->commands[ncmds++] = lp5521_command_wait_ramp(short_wait_usecs, 0);
    }

  if (nloops > 1)
    {
      /* Insert loop branch command. */

      prog->commands[ncmds++] = lp5521_command_branch(prog->ncommands,
                                                      nloops - 1);
    }

  ncmds -= prog->ncommands;
  prog->ncommands += ncmds;

  return ncmds;
}

int lp5521_program_ramp(struct lp5521_program_s *prog, unsigned int msecs,
                        int increment)
{
  unsigned int usecs = msecs * 1000;
  unsigned int increment_abs;

  if (increment < LP5521_CMD_RAMP_WAIT_MIN_NUM_STEPS ||
      increment > LP5521_CMD_RAMP_WAIT_MAX_NUM_STEPS)
    {
      return -EINVAL;
    }

  if (usecs / 1000 != msecs)
    {
      return -EINVAL;
    }

  increment_abs = increment < 0 ? -increment : increment;

  if (usecs > LP5521_CMD_RAMP_WAIT_MAX_USECS(
                         LP5521_CMD_RAMP_WAIT_PRESCALE1_CLKDIV, increment_abs))
    {
      /* Time too long for this amount of increment. TODO: generate loop
       * as is done for 'wait'
       */

      return -EINVAL;
    }

  /* Enough space left for this command? */

  if (prog->ncommands + 1 > LP5521_MAX_PROGRAM_COMMANDS)
    {
      return -ENOBUFS;
    }

  prog->commands[prog->ncommands++] = lp5521_command_wait_ramp(usecs,
                                                               increment);

  return 1;
}

int lp5521_program_set_pwm(struct lp5521_program_s *prog, uint8_t pwm)
{
  /* Enough space left for this command? */

  if (prog->ncommands + 1 > LP5521_MAX_PROGRAM_COMMANDS)
    {
      return -ENOBUFS;
    }

  prog->commands[prog->ncommands++] = lp5521_command_set_pwm(pwm);
  return 1;
}

int lp5521_program_goto_start(struct lp5521_program_s *prog)
{
  /* Enough space left for this command? */

  if (prog->ncommands + 1 > LP5521_MAX_PROGRAM_COMMANDS)
    {
      return -ENOBUFS;
    }

  prog->commands[prog->ncommands++] = lp5521_command_start();
  return 1;
}

int lp5521_program_branch(struct lp5521_program_s *prog, unsigned int jump_to,
                          unsigned int loop_count)
{
  if (jump_to >= LP5521_MAX_PROGRAM_COMMANDS)
    {
      return -EINVAL;
    }

  if ((loop_count < LP5521_CMD_BRANCH_MIN_LOOPS ||
      loop_count > LP5521_CMD_BRANCH_MAX_LOOPS) &&
      loop_count != LP5521_CMD_BRANCH_INFINITE_LOOP)
    {
      return -EINVAL;
    }

  /* Enough space left for this command? */

  if (prog->ncommands + 1 > LP5521_MAX_PROGRAM_COMMANDS)
    {
      return -ENOBUFS;
    }

  prog->commands[prog->ncommands++] =
      lp5521_command_branch(jump_to, loop_count);
  return 1;
}

int lp5521_program_end(struct lp5521_program_s *prog, bool sendint,
                       bool resetpwm)
{
  /* Enough space left for this command? */

  if (prog->ncommands + 1 > LP5521_MAX_PROGRAM_COMMANDS)
    {
      return -ENOBUFS;
    }

  prog->commands[prog->ncommands++] = lp5521_command_end(sendint, resetpwm);
  return 1;
}

int lp5521_program_trigger(struct lp5521_program_s *prog, bool wait_r,
                           bool wait_g, bool wait_b, bool wait_ext, bool send_r,
                           bool send_g, bool send_b, bool send_ext)
{
  /* Enough space left for this command? */

  if (prog->ncommands + 1 > LP5521_MAX_PROGRAM_COMMANDS)
    {
      return -ENOBUFS;
    }

  prog->commands[prog->ncommands++] =
      lp5521_command_trigger(
          lp5521_command_triggerflags(wait_r, wait_g, wait_b, wait_ext),
          lp5521_command_triggerflags(send_r, send_g, send_b, send_ext));
  return 1;
}
