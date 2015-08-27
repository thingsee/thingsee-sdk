/****************************************************************************
 * config/haltian-tsone/src/up_i2cdev_init.c
 * arch/arm/src/board/up_i2cdev_init.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *   Authors: Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *            Sami Pelkonen <sami.pelkonen@haltian.com>
 *            Timo Voutilainen <timo.voutilainen@haltian.com>
 *            Dmitry Nikolaev  <dmitry.nikolaev@haltian.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>
#include <debug.h>
#include <errno.h>

#include <unistd.h>
#include <errno.h>
#include <poll.h>
#include <fcntl.h>

#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/i2c.h>
#include <arch/board/board.h>
#include <arch/board/board-pwrctl.h>
#include <arch/board/board-device.h>
#ifdef CONFIG_LIS2DH
#include <nuttx/sensors/lis2dh.h>
#endif
#ifdef CONFIG_I2C_PRESSURE_DEV_LPS25H
#include <nuttx/sensors/lps25h.h>
#endif
#ifdef CONFIG_I2C_HUMIDITY_DEV_SHT25
#include <nuttx/sensors/sht25.h>
#endif
#ifdef CONFIG_BQ24251_CHARGER
#include <nuttx/power/bq24251.h>
#endif
#ifdef CONFIG_HTS221_HUMIDITY
#include <nuttx/sensors/hts221.h>
#endif
#ifdef CONFIG_LSM9DS1_SENS
#include <nuttx/sensors/lsm9ds1.h>
#endif
#ifdef CONFIG_INPUT_CYPRESS_MBR3108
#include <nuttx/input/cypress_mbr3108.h>
#endif
#ifdef CONFIG_MAX44009_SENSOR
#include <nuttx/sensors/max44009.h>
#endif
#ifdef CONFIG_LED_LP5521
#include <nuttx/led/lp5521.h>
#endif

#include "haltian-tsone.h"

#include "stm32.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#ifdef CONFIG_STM32_I2C3
#  define MAX_I2C_BUS 3
#elif defined CONFIG_STM32_I2C2
#  define MAX_I2C_BUS 2
#elif defined CONFIG_STM32_I2C1
#  define MAX_I2C_BUS 1
#else
#  define MAX_I2C_BUS 0
#endif

/* Default I2C buses for I2C slaves. */

#ifndef CONFIG_BOARD_I2C_BQ24251_CHARGER
#  define CONFIG_BOARD_I2C_BQ24251_CHARGER 1
#endif

#ifndef CONFIG_BOARD_I2C_HTS221_HUMIDITY
#  define CONFIG_BOARD_I2C_HTS221_HUMIDITY 1
#endif

#ifndef CONFIG_BOARD_I2C_SHT25_HUMIDITY
#  define CONFIG_BOARD_I2C_SHT25_HUMIDITY 1
#endif

#ifndef CONFIG_BOARD_I2C_LIS2DH_ACCEL
#  define CONFIG_BOARD_I2C_LIS2DH_ACCEL 1
#endif

#ifndef CONFIG_BOARD_I2C_MAX44009_SENSOR
#  define CONFIG_BOARD_I2C_MAX44009_SENSOR 1
#endif

#ifndef CONFIG_BOARD_I2C_LPS25H_PRESSURE
#  define CONFIG_BOARD_I2C_LPS25H_PRESSURE 1
#endif

#ifndef CONFIG_BOARD_I2C_LSM9DS1_SENSOR
#  define CONFIG_BOARD_I2C_LSM9DS1_SENSOR 2
#endif

#ifndef CONFIG_BOARD_I2C_CYPRESS_CAPSENSE
#  define CONFIG_BOARD_I2C_CYPRESS_CAPSENSE 2
#endif

#define CAPSENSE_POLL_INTERVAL_MSEC (100)

/* Configuration ************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_LIS2DH
struct stm32_lis2dh_config_s
{
  struct lis2dh_config_s dev;
  xcpt_t handler;
};
#endif

#ifdef CONFIG_I2C_PRESSURE_DEV_LPS25H
FAR typedef struct stm32_lps25h_config_s {
  lps25h_config_t dev;
  xcpt_t handler;
} stm32_lps25h_config_t;
#endif

#ifdef CONFIG_BQ24251_CHARGER
FAR typedef struct stm32_bq24251_config_s {
  bq24251_config_t dev;
  xcpt_t handler_stat;
} stm32_bq24251_config_t;
#endif

#ifdef CONFIG_HTS221_HUMIDITY
FAR typedef struct stm32_hts221_config_s {
  hts221_config_t dev;
  xcpt_t handler;
} stm32_hts221_config_t;
#endif

#ifdef CONFIG_INPUT_CYPRESS_MBR3108
FAR typedef struct stm32_mbr3108_config_s
{
  struct mbr3108_board_s dev;
  xcpt_t handler;
  struct
  {
    WDOG_ID wid;
    bool enabled;
  } irqwd;
} stm32_mbr3108_config_t;
#endif


#ifdef CONFIG_LSM9DS1_SENS
FAR typedef struct stm32_lsm9ds1_config_s {
  lsm9ds1_config_t dev;
  xcpt_t handler;
} stm32_lsm9ds1_config_t;
#endif

#ifdef CONFIG_MAX44009_SENSOR
FAR typedef struct stm32_max44009_config_s {
  max44009_config_t dev;
  xcpt_t handler;
} stm32_max44009_config_t;
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_BQ24251_CHARGER
  static int bq24251_attach_irq(FAR struct bq24251_config_t *state, xcpt_t handler_stat);
  static void bq24251_enable_irq(FAR bq24251_config_t *state, bool enable);
  static void bq24251_clear_irq(FAR bq24251_config_t *state);
#endif

#ifdef CONFIG_INPUT_CYPRESS_MBR3108
  static int mbr3108_attach_irq(FAR struct mbr3108_board_s *state, xcpt_t handler);
  static void mbr3108_enable_irq(FAR struct mbr3108_board_s *state, bool enable);
  static void mbr3108_clear_irq(FAR struct mbr3108_board_s *state);
  static int mbr3108_set_power(FAR struct mbr3108_board_s *state, bool on);
#endif


#ifdef CONFIG_HTS221_HUMIDITY
  static int hts221_attach_irq(FAR hts221_config_t *state, xcpt_t handler);
  static void hts221_enable_irq(FAR hts221_config_t *state, bool enable);
  static void hts221_clear_irq(FAR hts221_config_t *state);
#endif

#ifdef CONFIG_LIS2DH
  static int lis2dh_attach_irq(FAR struct lis2dh_config_s *state, xcpt_t handler);
  static void lis2dh_enable_irq(FAR struct lis2dh_config_s *state, bool enable);
  static void lis2dh_clear_irq(FAR struct lis2dh_config_s *state);
#endif

#ifdef CONFIG_LSM9DS1_SENS
  static int lsm9ds1_attach_irq(FAR lsm9ds1_config_t *state, xcpt_t handler);
  static void lsm9ds1_enable_irq(FAR lsm9ds1_config_t *state, bool enable);
  static void lsm9ds1_clear_irq(FAR lsm9ds1_config_t *state);
  static int lsm9ds1_set_power(FAR lsm9ds1_config_t *state, bool on);
#endif

#ifdef CONFIG_MAX44009_SENSOR
  static int max44009_attach_irq(FAR struct max44009_config_t *state, xcpt_t handler);
  static void max44009_enable_irq(FAR struct max44009_config_t *state, bool enable);
  static void max44009_clear_irq(FAR struct max44009_config_t *state);
#endif

#ifdef CONFIG_I2C_PRESSURE_DEV_LPS25H
  static int lps25h_attach_irq(FAR struct lps25h_config_s *state, xcpt_t handler);
  static void lps25h_enable_irq(FAR struct lps25h_config_s *state, bool enable);
  static void lps25h_clear_irq(lps25h_config_t *state);
#endif

#ifdef CONFIG_LED_LP5521
  static int lp5521_set_power(FAR const struct lp5521_board_s *state, bool on);
  static int lp5521_set_enable(FAR const struct lp5521_board_s *state, bool enable);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_LIS2DH
static struct stm32_lis2dh_config_s g_lis2dh_info =
{
  .dev.irq_attach       = lis2dh_attach_irq,
  .dev.irq_enable       = lis2dh_enable_irq,
  .dev.irq_clear        = lis2dh_clear_irq,
  .handler              = NULL,
};
#endif

#ifdef CONFIG_MAX44009_SENSOR
static struct stm32_max44009_config_s g_max44009_config =
{
  .dev.irq_attach       = max44009_attach_irq,
  .dev.irq_enable       = max44009_enable_irq,
  .dev.irq_clear        = max44009_clear_irq,
  .handler              = NULL,
};
#endif

#ifdef CONFIG_I2C_PRESSURE_DEV_LPS25H
static stm32_lps25h_config_t g_lps25h_config = {
  .dev.irq_attach = lps25h_attach_irq,
  .dev.irq_enable = lps25h_enable_irq,
  .dev.irq_clear = lps25h_clear_irq,
  .handler = NULL
};
#endif

#ifdef CONFIG_BQ24251_CHARGER
static stm32_bq24251_config_t g_bq24251_config = {
  .dev.irq_attach = bq24251_attach_irq,
  .dev.irq_enable = bq24251_enable_irq,
  .dev.irq_clear = bq24251_clear_irq
};
#endif

#ifdef CONFIG_INPUT_CYPRESS_MBR3108
static stm32_mbr3108_config_t g_mbr3108_config = {
  .dev.irq_attach = mbr3108_attach_irq,
  .dev.irq_enable = mbr3108_enable_irq,
  .dev.irq_clear = mbr3108_clear_irq,
  .dev.set_power = mbr3108_set_power,
  .handler = NULL
};
#endif


#ifdef CONFIG_HTS221_HUMIDITY
static stm32_hts221_config_t g_hts221_config = {
  .dev.irq_attach = hts221_attach_irq,
  .dev.irq_enable = hts221_enable_irq,
  .dev.irq_clear = hts221_clear_irq,
  .handler = NULL
};
#endif

#ifdef CONFIG_LSM9DS1_SENS
static stm32_lsm9ds1_config_t g_lsm9ds1_config = {
  .dev.irq_attach = lsm9ds1_attach_irq,
  .dev.irq_enable = lsm9ds1_enable_irq,
  .dev.irq_clear = lsm9ds1_clear_irq,
  .dev.set_power = lsm9ds1_set_power,
  .handler = NULL
};
#endif

#ifdef CONFIG_LED_LP5521
static const struct lp5521_board_s g_lp5521_config = {
  .set_power = lp5521_set_power,
  .set_enable = lp5521_set_enable,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_HTS221_HUMIDITY

static int hts221_init(struct i2c_dev_s *i2c)
{
  return hts221_register("/dev/hts221", i2c, 0x5F, &g_hts221_config.dev);
}

static int hts221_attach_irq(FAR struct hts221_config_t *state, xcpt_t handler)
{
  FAR struct stm32_hts221_config_s *dev = (FAR struct stm32_hts221_config_s *)state;

  /* Just save the handler for use when the interrupt is enabled */
  dev->handler = handler;

  return OK;
}

static void hts221_enable_irq(FAR struct hts221_config_t *state, bool enable)
{
  FAR struct stm32_hts221_config_s *priv = (FAR struct stm32_hts221_config_s *)state;

  DEBUGASSERT(priv->handler);
  /* Attach and enable, or detach and disable */
  if (enable)
    {
      (void)stm32_gpiosetevent(GPIO_HTS221_INT, true, false, false, priv->handler);
    }
  else
    {
      (void)stm32_gpiosetevent(GPIO_HTS221_INT, false, false, false, NULL);
    }
}

static void hts221_clear_irq(FAR struct hts221_config_t *state)
{
  /* Do nothing */
}

#endif

#ifdef CONFIG_INPUT_CYPRESS_MBR3108

static int mbr3108_init(struct i2c_dev_s *i2c)
{
  static const struct mbr3108_sensor_conf_s mbr3108_sensor_conf =
  {
    /* Configuration generated using EZ-Click tool. Enable two proximity
     * sensors and host interrupt on SPO0/BUZ pin. */

    /* Summary of EZ-Click 2.0 settings:
     *
     * Capsense sensor configuration:
     *  - Number of proximity sensors: 2
     *  - Median filter: on
     *  - IIR filter: off
     *  - Automatic threshold: off
     *  - Advanced low-pass: disabled
     *  - Scan period: 120 msec
     *
     *  - CS0/PS0, Sensor1:
     *    + Resolution: 16 bits
     *    + Prox/+ve/-ve/Finger threshold: 100/30/30/512
     *    + Wakeup: off
     *
     *  - CS1/PS1, Sensor2:
     *    + Resolution: 16 bits
     *    + Prox/+ve/-ve/Finger threshold: 200/60/60/1024
     *    + Wakeup: off
     *
     * Global configuration:
     *  - Proximity auto-reset: 20 sec
     *  - I2C address: 0x37
     *  - LED duration: 0 ms
     *  - State timeout: 10 sec
     *  - EMC noise immunity: on
     *  - Enable shield: off
     *  - Enable guard sensor: off
     *  - Enable GPIO host control: off
     *  - GPO drive mode: Open Drain Low
     *  - GPO logic level: Active low
     *  - Host interrupt pin: HI/BUZ
     *  - Buzzer, pin/freq/dura: none/4kHz/100ms
     *  - Supply voltage: 1.8-5.5 V
     *  - Debounce, CS0-CS7: 3
     *  - LEDs off
     */

    .conf_data =
      {
        0x03u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x64u, 0xC8u, 0x7Fu, 0x7Fu,
        0x7Fu, 0x7Fu, 0x7Fu, 0x7Fu, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x03u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x03u, 0x00u,
        0x05u, 0x00u, 0x00u, 0x02u, 0x00u, 0x04u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x1Eu, 0x3Cu, 0x00u,
        0x00u, 0x1Eu, 0x3Cu, 0x00u, 0x00u, 0x00u, 0x01u, 0x01u,
        0x00u, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x04u, 0x01u, 0x01u, 0x94u,
        0x00u, 0x37u, 0x06u, 0x00u, 0x00u, 0x0Au, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x47u, 0x29u
      }
  };

  return cypress_mbr3108_register("/dev/capsense0", i2c, 0x37,
                                  &g_mbr3108_config.dev,
                                  &mbr3108_sensor_conf);
}

static int mbr3108_attach_irq(FAR struct mbr3108_board_s *state, xcpt_t handler)
{
  FAR struct stm32_mbr3108_config_s *priv =
      (FAR struct stm32_mbr3108_config_s *)state;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;

  return 0;
}

static void mbr3108_irqwd(int argc, uint32_t arg1, ...)
{
  FAR struct stm32_mbr3108_config_s *priv =
      (FAR struct stm32_mbr3108_config_s *)(intptr_t)arg1;

  if (priv->handler)
    {
      priv->handler(-1, NULL);
    }

  /* Restart timer. */

  if (priv->irqwd.wid)
    {
      wd_start(priv->irqwd.wid, MSEC2TICK(CAPSENSE_POLL_INTERVAL_MSEC),
               mbr3108_irqwd, 1, priv);
    }
}

static void mbr3108_enable_irq(FAR struct mbr3108_board_s *state, bool enable)
{
  FAR struct stm32_mbr3108_config_s *priv =
      (FAR struct stm32_mbr3108_config_s *)state;

  DEBUGASSERT(priv->handler);

  if (board_get_hw_ver() >= BOARD_HWVER_B2_0)
    {
      /* Attach and enable, or detach and disable */

      if (enable)
        {
          (void)stm32_gpiosetevent(GPIO_MBR3108_INT, true, false, false,
                                   priv->handler);
        }
      else
        {
          (void)stm32_gpiosetevent(GPIO_MBR3108_INT, false, false, false, NULL);
        }
    }
  else
    {
      irqstate_t flags;

      /* Workaround for non-working IRQ line. In v1.5-v1.7, WLAN and Capsense
       * share EXTI line. To make both work simultaneously, emulate Capsense
       * interrupts with timer. */

      if (enable == priv->irqwd.enabled)
        {
          return;
        }

      flags = irqsave();

      priv->irqwd.enabled = enable;

      if (enable)
        {
          priv->irqwd.wid = wd_create();
          if (priv->irqwd.wid)
            {
              wd_start(priv->irqwd.wid, MSEC2TICK(CAPSENSE_POLL_INTERVAL_MSEC),
                       mbr3108_irqwd, 1, priv);
            }
          else
            {
              dbg("Could not create watchdog timer!\n");
            }
        }
      else
        {
          if (priv->irqwd.wid)
            {
              wd_delete(priv->irqwd.wid);
              priv->irqwd.wid = NULL;
            }
        }

      irqrestore(flags);
    }
}

static void mbr3108_clear_irq(FAR struct mbr3108_board_s *state)
{
  (void)state; /* Do nothing */
}

static int mbr3108_set_power(FAR struct mbr3108_board_s *state, bool on)
{
  static bool powered_on = false;

  (void)state;

  if (powered_on == on)
    {
      return 0;
    }

  if (on)
    {
      board_pwrctl_get(PWRCTL_SWITCH_CAPSENSE_SENSOR);
    }
  else
    {
      board_pwrctl_put(PWRCTL_SWITCH_CAPSENSE_SENSOR);
    }

  powered_on = on;

  return 0;
}

#endif

#ifdef CONFIG_MAX44009_SENSOR

static int max44009_init(struct i2c_dev_s *i2c)
{
  return max44009_register("/dev/als0", i2c, 0x4B, &g_max44009_config.dev);
}

static int max44009_attach_irq(FAR struct max44009_config_t *state, xcpt_t handler)
{
  FAR struct stm32_max44009_config_s *dev = (FAR struct stm32_max44009_config_s *)state;

  /* Just save the handler for use when the interrupt is enabled */
  dev->handler = handler;

  return OK;
}

static void max44009_enable_irq(FAR struct max44009_config_t *state, bool enable)
{
  FAR struct stm32_max44009_config_s *priv = (FAR struct stm32_max44009_config_s *)state;

  DEBUGASSERT(priv->handler);

  /* Attach and enable, or detach and disable */
  if (enable)
    {
      (void)stm32_gpiosetevent(GPIO_MAX44009_INT, false, true, false, priv->handler);
    }
  else
    {
      (void)stm32_gpiosetevent(GPIO_MAX44009_INT, false, false, false, NULL);
    }
}

static void max44009_clear_irq(FAR struct max44009_config_t *state)
{
  /* Do nothing */
}

#endif


#ifdef CONFIG_LIS2DH

static int lis2dh_attach_irq(FAR struct lis2dh_config_s *state, xcpt_t handler)
{
  FAR struct stm32_lis2dh_config_s *priv = (FAR struct stm32_lis2dh_config_s *)state;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;

  return OK;
}

static void lis2dh_enable_irq(FAR struct lis2dh_config_s *state, bool enable)
{
  FAR struct stm32_lis2dh_config_s *priv = (FAR struct stm32_lis2dh_config_s *)state;

  /* The caller should not attempt to enable interrupts if the handler
   * has not yet been 'attached'
   */

  DEBUGASSERT(priv->handler || !enable);

  /* Attach and enable, or detach and disable */

  if (enable)
    {
      (void)stm32_gpiosetevent(GPIO_LIS2DH_INT1, true, false, false, priv->handler);
      (void)stm32_gpiosetevent(GPIO_LIS2DH_INT2, true, false, false, priv->handler);
    }
  else
    {
      (void)stm32_gpiosetevent(GPIO_LIS2DH_INT1, false, false, false, NULL);
      (void)stm32_gpiosetevent(GPIO_LIS2DH_INT2, false, false, false, NULL);
    }
}

static void lis2dh_clear_irq(FAR struct lis2dh_config_s *state)
{
  /* Do nothing */
}
#endif

#ifdef CONFIG_LSM9DS1_SENS

static int lsm9ds1_init(struct i2c_dev_s *i2c)
{
  /* First address is for the acceleration and gyro sensor. The second is for magnetometer. */
  uint8_t addrs[] = { 0x6B, 0x1E };

  /* The DEN_A/G pin of the LSM9DS1 can be used for external trigger level recognition.
   * When this functionality is enabled, it is used to stamp gyroscope data based on the
   * value on this pin or to reduce ODR according to the signal on this pin. We don't use this
   * but just set the pin to zero.
   */
  stm32_configgpio(GPIO_LSM9DS1_DEN_AG);

  return lsm9ds1_register("/dev/lsm9ds1", i2c, addrs, &g_lsm9ds1_config.dev);
}

static int lsm9ds1_attach_irq(FAR struct lsm9ds1_config_t *state, xcpt_t handler)
{
  FAR struct stm32_lsm9ds1_config_s *dev = (FAR struct stm32_lsm9ds1_config_s *)state;

  /* Just save the handler for use when the interrupt is enabled */
  dev->handler = handler;

  return OK;
}

static void lsm9ds1_enable_irq(FAR struct lsm9ds1_config_t *state, bool enable)
{
  FAR struct stm32_lsm9ds1_config_s *priv = (FAR struct stm32_lsm9ds1_config_s *)state;

  DEBUGASSERT(priv->handler);
  /* Attach and enable, or detach and disable */
  if (enable)
    {
      (void)stm32_gpiosetevent(GPIO_LSM9DS1_INT1_AG, true, false, false, priv->handler);
      (void)stm32_gpiosetevent(GPIO_LSM9DS1_INT2_AG, true, false, false, priv->handler);
      (void)stm32_gpiosetevent(GPIO_LSM9DS1_INT_M, true, false, false, priv->handler);
      (void)stm32_gpiosetevent(GPIO_LSM9DS1_DRDY_M, true, false, false, priv->handler);
    }
  else
    {
      (void)stm32_gpiosetevent(GPIO_LSM9DS1_INT1_AG, false, false, false, NULL);
      (void)stm32_gpiosetevent(GPIO_LSM9DS1_INT2_AG, false, false, false, NULL);
      (void)stm32_gpiosetevent(GPIO_LSM9DS1_INT_M, false, false, false, NULL);
      (void)stm32_gpiosetevent(GPIO_LSM9DS1_DRDY_M, false, false, false, NULL);
    }
}

static void lsm9ds1_clear_irq(FAR struct lsm9ds1_config_t *state)
{
  /* Do nothing */
}

static int lsm9ds1_set_power(FAR struct lsm9ds1_config_t *state, bool on)
{
  static bool powered_on = false;

  (void)state;

  if (powered_on == on)
    {
      return 0;
    }

  if (on)
    {
      board_pwrctl_get(PWRCTL_SWITCH_9AXIS_INERTIAL_SENSOR);
    }
  else
    {
      board_pwrctl_put(PWRCTL_SWITCH_9AXIS_INERTIAL_SENSOR);
    }

  powered_on = on;

  return 0;
}

#endif

#ifdef CONFIG_I2C_PRESSURE_DEV_LPS25H

static int lps25h_attach_irq(FAR struct lps25h_config_s *state, xcpt_t handler)
{
  FAR struct stm32_lps25h_config_s *dev = (FAR struct stm32_lps25h_config_s *)state;

  /* Just save the handler for use when the interrupt is enabled */

  dev->handler = handler;
  return OK;
}

static void lps25h_enable_irq(FAR struct lps25h_config_s *state, bool enable)
{
  FAR struct stm32_lps25h_config_s *priv = (FAR struct stm32_lps25h_config_s *)state;

  /* The caller should not attempt to enable interrupts if the handler
   * has not yet been 'attached'
   */

  DEBUGASSERT(priv->handler || !enable);

  /* Attach and enable, or detach and disable */
  if (enable)
    {
      (void)stm32_gpiosetevent(GPIO_LPS25H_PRES_INT, true, false, false, priv->handler);
    }
  else
    {
      (void)stm32_gpiosetevent(GPIO_LPS25H_PRES_INT, false, false, false, NULL);
    }
}

static void lps25h_clear_irq(lps25h_config_t *state)
{
  /* Do nothing */
}

#endif

#ifdef CONFIG_I2C_PRESSURE_DEV_LPS25H

static int lps25h_i2c_pressure_sensor_init(struct i2c_dev_s *i2c)
{
  int ret;

  ret = lps25h_register("/dev/pres0", i2c, 0x5D, &g_lps25h_config.dev);

  return ret;
}
#endif

#ifdef CONFIG_I2C_HUMIDITY_DEV_SHT25

static int sht25_i2c_humidity_sensor_init(struct i2c_dev_s *i2c)
{
  int ret;

  ret = sht25_register("/dev/humid0", i2c, 0x40);
  lldbg("Initializing humidity sensor\n");

  return ret;
}

#endif

#ifdef CONFIG_BQ24251_CHARGER

static int bq24251_init(struct i2c_dev_s *i2c)
{
  /* Charger CE pin setup */

  /* TODO: Move to charger driver */

  stm32_configgpio(GPIO_BQ24251_CE);
  stm32_gpiowrite(GPIO_BQ24251_CE, false);

  return bq24251_register("/dev/charger0", i2c, 0x6A, &g_bq24251_config.dev);
}

#endif

#ifdef CONFIG_LIS2DH

static int lis2dh_init(struct i2c_dev_s * i2c)
{
  int ret;

  /* Register the accelerometer sensor */

  ret = lis2dh_register("/dev/acc0", i2c, 0x19, &g_lis2dh_info.dev);
  dbg("Registered with %d\n", ret);

  return ret;
}

#endif

#ifdef CONFIG_BQ24251_CHARGER

static int bq24251_attach_irq(FAR struct bq24251_config_t *state, xcpt_t handler_stat)
{
  FAR struct stm32_bq24251_config_s *dev = (FAR struct stm32_bq24251_config_s *)state;

  /* Just save the handler for use when the interrupt is enabled */
  dev->handler_stat = handler_stat;

  return OK;
}

static void bq24251_enable_irq(FAR struct bq24251_config_t *state, bool enable)
{
  FAR struct stm32_bq24251_config_s *priv = (FAR struct stm32_bq24251_config_s *)state;

  DEBUGASSERT(priv->handler_stat);
  /* Attach and enable, or detach and disable */
  if (enable)
    {
      (void)stm32_gpiosetevent(GPIO_BQ24251_STAT, true, true, false, priv->handler_stat);
    }
  else
    {
      (void)stm32_gpiosetevent(GPIO_BQ24251_STAT, false, false, false, NULL);
    }
}

static void bq24251_clear_irq(FAR struct bq24251_config_t *state)
{
  /* Do nothing */
}

#endif

#ifdef CONFIG_LED_LP5521

static int lp5521_set_power(FAR const struct lp5521_board_s *state, bool on)
{
  static bool g_on = false;

  if (on == g_on)
    {
      return OK;
    }

  if (on)
    {
      /* Evaluation kit connected to I2C2 (thus needs Capsense power, I2C2 at
       * pads J9003 & J9004) and takes VDD from SDCARD regulator (pad J9013).
       */

      board_pwrctl_get(PWRCTL_SWITCH_CAPSENSE_SENSOR);
      board_pwrctl_get(PWRCTL_REGULATOR_SDCARD);
    }
  else
    {
      board_pwrctl_put(PWRCTL_REGULATOR_SDCARD);
      board_pwrctl_put(PWRCTL_SWITCH_CAPSENSE_SENSOR);
    }

  g_on = on;

  return OK;
}

static int lp5521_set_enable(FAR const struct lp5521_board_s *state, bool enable)
{
  stm32_gpiowrite(GPIO_PAD_J9005, enable);
  return OK;
}

static int led_lp5521_init(struct i2c_dev_s *i2c)
{
  static const struct lp5521_conf_s config =
  {
    .r.initial_mode = LP5521_CHANNEL_MODE_DISABLED,
    .g.initial_mode = LP5521_CHANNEL_MODE_DISABLED,
    .b.initial_mode = LP5521_CHANNEL_MODE_DISABLED,
    .r.initial_pwm = 255,
    .g.initial_pwm = 255,
    .b.initial_pwm = 255,
    .r.current = 2000,
    .g.current = 2000,
    .b.current = 2000,

    .logarithmic = true,
    .auto_powersave = true,
    .pwm_clock_558hz = true,

    .clock_select = LP5521_CLOCK_INTERNAL,
    .charge_pump_mode = LP5521_CHARGE_PUMP_MODE_AUTOMATIC_SELECTION,
  };
  int ret;

  stm32_configgpio(GPIO_PAD_J9005);

  ret = led_lp5521_register("/dev/led0", i2c, 0x64 >> 1, &g_lp5521_config,
                            &config);
  if (ret < 0)
    {
      lldbg("Initializing LP5521 failed.\n");
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_i2c_devinitialize
 *
 * Description:
 *   Perform initialization for I2C buses and I2C devices
 *
 ****************************************************************************/

int up_i2c_devinitialize(void)
{
  struct i2c_dev_s *i2c_buses[MAX_I2C_BUS] = { 0 };

  /* Get instances of the I2C interfaces and reset the buses. */

#ifdef CONFIG_STM32_I2C1
  i2c_buses[0] = up_i2cinitialize(1);
  if (!i2c_buses[0])
    {
      return -ENODEV;
    }
  (void)up_i2creset(i2c_buses[0]);
#endif

#ifdef CONFIG_STM32_I2C2
  i2c_buses[1] = up_i2cinitialize(2);
  if (!i2c_buses[1])
    {
      return -ENODEV;
    }
  (void)up_i2creset(i2c_buses[1]);
#endif

/* Macro to check device I2C configuration correctness. */
#define DEBUG_CHECK_I2C(dev_bus) { \
    char \
    device_i2cbus_too_large[(dev_bus) <= MAX_I2C_BUS ? 0 : -1]; \
    char \
    device_i2cbus_too_small[(dev_bus) > 0 ? 0 : -1]; \
    (void)device_i2cbus_too_large; \
    (void)device_i2cbus_too_small; \
    DEBUGASSERT(i2c_buses[(dev_bus) - 1]); \
  }

#ifdef CONFIG_BQ24251_CHARGER
  DEBUG_CHECK_I2C(CONFIG_BOARD_I2C_BQ24251_CHARGER);
  bq24251_init(i2c_buses[CONFIG_BOARD_I2C_BQ24251_CHARGER - 1]);
#endif


#ifdef CONFIG_HTS221_HUMIDITY
  DEBUG_CHECK_I2C(CONFIG_BOARD_I2C_HTS221_HUMIDITY);
  hts221_init(i2c_buses[CONFIG_BOARD_I2C_HTS221_HUMIDITY - 1]);
#endif

#ifdef CONFIG_I2C_HUMIDITY_DEV_SHT25
  DEBUG_CHECK_I2C(CONFIG_BOARD_I2C_SHT25_HUMIDITY);
  sht25_i2c_humidity_sensor_init(i2c_buses[CONFIG_BOARD_I2C_SHT25_HUMIDITY - 1]);
#endif

#ifdef CONFIG_LIS2DH
  DEBUG_CHECK_I2C(CONFIG_BOARD_I2C_LIS2DH_ACCEL);
  lis2dh_init(i2c_buses[CONFIG_BOARD_I2C_LIS2DH_ACCEL - 1]);
#endif

#ifdef CONFIG_LSM9DS1_SENS
  DEBUG_CHECK_I2C(CONFIG_BOARD_I2C_LSM9DS1_SENSOR);
  lsm9ds1_init(i2c_buses[CONFIG_BOARD_I2C_LSM9DS1_SENSOR - 1]);
#endif

#ifdef CONFIG_INPUT_CYPRESS_MBR3108
  DEBUG_CHECK_I2C(CONFIG_BOARD_I2C_CYPRESS_CAPSENSE);
  mbr3108_init(i2c_buses[CONFIG_BOARD_I2C_CYPRESS_CAPSENSE - 1]);
#endif

#ifdef CONFIG_MAX44009_SENSOR
  DEBUG_CHECK_I2C(CONFIG_BOARD_I2C_MAX44009_SENSOR);
  max44009_init(i2c_buses[CONFIG_BOARD_I2C_MAX44009_SENSOR - 1]);
#endif

#ifdef CONFIG_I2C_PRESSURE_DEV_LPS25H
  DEBUG_CHECK_I2C(CONFIG_BOARD_I2C_LPS25H_PRESSURE);
  lps25h_i2c_pressure_sensor_init(i2c_buses[CONFIG_BOARD_I2C_LPS25H_PRESSURE - 1]);
#endif

#ifdef CONFIG_LED_LP5521
  DEBUG_CHECK_I2C(CONFIG_BOARD_I2C_LED_LP5521);
  led_lp5521_init(i2c_buses[CONFIG_BOARD_I2C_LED_LP5521 - 1]);
#endif

  /* Power off the switches that we temporarily
   * enabled in stm32_boardinitialize()
   */

  board_pwrctl_put(PWRCTL_SWITCH_CAPSENSE_SENSOR);
  board_pwrctl_put(PWRCTL_SWITCH_9AXIS_INERTIAL_SENSOR);

  return OK;
}
