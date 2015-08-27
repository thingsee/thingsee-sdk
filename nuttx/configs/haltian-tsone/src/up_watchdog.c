/************************************************************************************
 * configs/haltian-tsone/src/up_watchdog.c
 * arch/arm/src/board/up_watchdog.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>
#include <poll.h>

#include <nuttx/timers/watchdog.h>
#include <arch/board/board.h>
#include <arch/board/board-device.h>
#include <nuttx/arch.h>
#include "haltian-tsone.h"

#include "stm32_wdg.h"

#ifdef CONFIG_WATCHDOG

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Configuration *******************************************************************/
/* Wathdog hardware should be enabled */

#if !defined(CONFIG_STM32_WWDG) && !defined(CONFIG_STM32_IWDG)
#if !defined(CONFIG_BOARD_HALTIAN_HWWDG)
#  warning "One of CONFIG_STM32_WWDG or CONFIG_STM32_IWDG" \
           "or CONFIG_BOARD_HALTIAN_HWWDG must be defined"
#endif
#endif

/* Select the path to the registered watchdog timer device */

#ifndef CONFIG_STM32_WDG_DEVPATH
#  ifdef CONFIG_EXAMPLES_WATCHDOG_DEVPATH
#    define CONFIG_STM32_WDG_DEVPATH CONFIG_EXAMPLES_WATCHDOG_DEVPATH
#  else
#    define CONFIG_STM32_WDG_DEVPATH "/dev/watchdog0"
#  endif
#endif

/* Use the un-calibrated LSI frequency if we have nothing better */

#define STM32_LSI_FREQUENCY_MAX 56000

#if defined(CONFIG_STM32_IWDG) && !defined(CONFIG_STM32_LSIFREQ)
#  define CONFIG_STM32_LSIFREQ STM32_LSI_FREQUENCY_MAX
#endif

/* Debug ***************************************************************************/
/* Non-standard debug that may be enabled just for testing the watchdog timer */

//#define CONFIG_DEBUG_WATCHDOG
#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_WATCHDOG
#endif

#ifdef CONFIG_DEBUG_WATCHDOG
#  define wdgdbg                 dbg
#  define wdglldbg               lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define wdgvdbg              vdbg
#    define wdgllvdbg            llvdbg
#  else
#    define wdgvdbg(x...)
#    define wdgllvdbg(x...)
#  endif
#else
#  define wdgdbg(x...)
#  define wdglldbg(x...)
#  define wdgvdbg(x...)
#  define wdgllvdbg(x...)
#endif


#ifdef CONFIG_BOARD_HALTIAN_HWWDG

#define DEV_FORMAT   "/dev/hwwdg%d"
#define DEV_NAMELEN  16

#ifndef CONFIG_HWWDG_NPOLLWAITERS
#define CONFIG_HWWDG_NPOLLWAITERS 1
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int board_hwwdg_open(FAR struct file *filep);
static int board_hwwdg_close(FAR struct file *filep);
static ssize_t board_hwwdg_read(FAR struct file *filep, FAR char *buffer, size_t len);
#ifndef CONFIG_DISABLE_POLL
static int board_hwwdg_poll(FAR struct file *filep, struct pollfd *fds, bool setup);
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

static const struct file_operations hwwdg_fops =
{
  board_hwwdg_open,   /* open */
  board_hwwdg_close,  /* close */
  board_hwwdg_read,   /* read */
  0,                  /* write */
  0,                  /* seek */
  0,                  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  board_hwwdg_poll,   /* poll */
#endif
};

struct hwwdg_dev_s
{
  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *fds[CONFIG_HWWDG_NPOLLWAITERS];
#endif
  sem_t devsem;                /* Manages exclusive access to this structure */
  volatile int int_pending;    /* An unreported event is buffered */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct hwwdg_dev_s hwwdg_dev;

static void (*board_hwwdg_irq_tracefn)(void) = NULL;
static bool board_hwwdg_autokick = false;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static int board_hwwdg_open(FAR struct file *filep)
{
  DEBUGASSERT(filep);
  return OK;
}

static int board_hwwdg_close(FAR struct file *filep)
{
  DEBUGASSERT(filep);
  return OK;
}

static ssize_t board_hwwdg_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode *inode;
  FAR struct hwwdg_dev_s *priv;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct hwwdg_dev_s *)inode->i_private;

  if (len < 1)
    {
      return 0;
    }

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  wdglldbg("Kicking the dog\n");
  board_hwwdg_kick();
  *buffer = 'K';             /* K means kick */
  priv->int_pending = false;

  sem_post(&priv->devsem);

  return 1;
}

#ifndef CONFIG_DISABLE_POLL

static void board_hwwdg_notify(FAR struct hwwdg_dev_s *priv)
{
  int i;

  /* If there are threads waiting on poll() for device data to become available,
   * then wake them up now.  NOTE: we wake up all waiting threads because we
   * do not know that they are going to do.  If they all try to read the data,
   * then some make end up blocking after all.
   */

  for (i = 0; i < CONFIG_HWWDG_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          wdglldbg("Report events: %02x\n", fds->revents);
          sem_post(fds->sem);
        }
    }
}

static int board_hwwdg_poll(FAR struct file *filep, struct pollfd *fds, bool setup)
{
  FAR struct inode *inode;
  FAR struct hwwdg_dev_s *priv;
  int ret;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct hwwdg_dev_s *)inode->i_private;

  /* Are we setting up the poll?  Or tearing it down? */

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      /* This should only happen if the wait was cancelled by an signal */

      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto errout;
        }

      /* This is a request to set up the poll. Find an available
       * slot for the poll structure reference.
       */

      for (i = 0; i < CONFIG_HWWDG_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot. */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_HWWDG_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->int_pending)
        {
          board_hwwdg_notify(priv);
        }
    }

  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup. */

      *slot = NULL;
      fds->priv = NULL;
    }

errout:
  sem_post(&priv->devsem);
  return ret;
}

#endif /* CONFIG_DISABLE_POLL */

static void board_hwwdg_default_trace(void)
{
  wdglldbg("board_hwwdg_handler called\n");
}

static int board_hwwdg_handler(int irq, FAR void *context)
{
  if (board_hwwdg_autokick)
    {
      board_hwwdg_kick();
    }
  else
    {
      FAR struct hwwdg_dev_s *priv = &hwwdg_dev;

      priv->int_pending = true;
      board_hwwdg_notify(priv);
    }

  if (board_hwwdg_irq_tracefn != NULL)
    {
      board_hwwdg_irq_tracefn();
    }

  return OK;
}

static void board_hwwdg_enable_irq(bool enable)
{
  if (enable)
    {
      (void)stm32_gpiosetevent(GPIO_HWWDG_WAKE, true, false, true, &board_hwwdg_handler);
    }
  else
    {
      (void)stm32_gpiosetevent(GPIO_HWWDG_WAKE, false, false, false, NULL);
    }
}

void board_hwwdg_kick(void)
{
  stm32_gpiowrite(GPIO_HWWDG_DONE, 1);
  up_udelay(10); /* 2.5us for 400KHz gpio should be enough, 10us just in case. */
  stm32_gpiowrite(GPIO_HWWDG_DONE, 0);
}

static void board_hwwdg_initialize(void (*irq_tracefn)(void), bool autokick)
{
  FAR struct hwwdg_dev_s *priv = &hwwdg_dev;
  char devname[DEV_NAMELEN];
  int ret;

  sem_init(&priv->devsem, 0, 1);
  priv->int_pending = false;

  stm32_configgpio(GPIO_HWWDG_WAKE);
  stm32_configgpio(GPIO_HWWDG_DONE);

  /* Enable this before registering driver, so we always get traces. */

  board_hwwdg_irq_tracefn = irq_tracefn;
  board_hwwdg_autokick = autokick;
  board_hwwdg_enable_irq(true);

  if (!board_hwwdg_autokick)
    {
      /* Register new device node for watchdog (to transfer irq to fd/poll). */

      (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, 0);

      ret = register_driver(devname, &hwwdg_fops, 0666, priv);
      if (ret < 0)
        {
          wdglldbg("Could not register %s\n", devname);
          return;
        }

      wdglldbg("board_hwwdg_initialize succeeded\n");
    }

  board_hwwdg_kick();
}

#endif /* CONFIG_BOARD_HALTIAN_HWWDG */

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: up_wdginitialize()
 *
 * Description:
 *   Perform architecture-specific initialization of the Watchdog hardware.
 *   This interface must be provided by all configurations using
 *   apps/examples/watchdog
 *
 ****************************************************************************/

int up_wdginitialize(void)
{
  /* Register the watchdog timer device */

#ifdef CONFIG_STM32_WWDG
  stm32_wwdginitialize(CONFIG_STM32_WDG_DEVPATH);
  return OK;
#endif

#ifdef CONFIG_STM32_IWDG
  if (board_get_hw_ver() < BOARD_HWVER_B2_0)
    {
      stm32_iwdginitialize(CONFIG_STM32_WDG_DEVPATH, CONFIG_STM32_LSIFREQ);
      return OK;
    }
#endif

#ifdef CONFIG_BOARD_HALTIAN_HWWDG
  if (board_get_hw_ver() >= BOARD_HWVER_B2_0)
    {
      board_hwwdg_initialize(board_hwwdg_default_trace, false);
      return OK;
    }
#endif

  return -ENODEV;
}

/****************************************************************************
 * Name: board_wdginitialize_autokick()
 *
 * Description:
 *   Perform architecture-specific initialization of the Watchdog hardware
 *   to automatic kicking mode.
 *
 ****************************************************************************/

int board_wdginitialize_autokick(void (*irq_tracefn)(void))
{
#ifdef CONFIG_BOARD_HALTIAN_HWWDG
  if (board_get_hw_ver() >= BOARD_HWVER_B2_0)
    {
      if (irq_tracefn == NULL)
        {
          irq_tracefn = board_hwwdg_default_trace;
        }
      board_hwwdg_initialize(irq_tracefn, true);
      return OK;
    }
#endif

  return -ENODEV;
}

#endif /* CONFIG_WATCHDOG */
