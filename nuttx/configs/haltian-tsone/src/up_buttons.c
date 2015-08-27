/****************************************************************************
 * configs/haltian-tsone/src/up_buttons.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *   Authors: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#include <stdint.h>
#include <errno.h>
#include <poll.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "haltian-tsone.h"

#include "stm32.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define DEV_FORMAT   "/dev/buttons%d"
#define DEV_NAMELEN  16

#ifndef CONFIG_IRQBUTTONS_NPOLLWAITERS
#define CONFIG_IRQBUTTONS_NPOLLWAITERS 2
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct irqbuttons_dev_s
{
  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *fds[CONFIG_IRQBUTTONS_NPOLLWAITERS];
#endif

  sem_t devsem;                         /* Manages exclusive access to this structure */
  volatile int int_pending;             /* An unreported event is buffered */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int irqbuttons_open(FAR struct file *filep);
static int irqbuttons_close(FAR struct file *filep);
static ssize_t irqbuttons_read(FAR struct file *filep, FAR char *buffer, size_t len);
#ifndef CONFIG_DISABLE_POLL
static int irqbuttons_poll(FAR struct file *filep, struct pollfd *fds, bool setup);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pin configuration for each button. This array is indexed by
 * the BUTTON_* definitions in board.h
 */

static const uint32_t g_buttons[BOARD_NUM_BUTTONS] =
{
  GPIO_BTN_POWERKEY,
};

static const bool g_buttons_lowpressed[BOARD_NUM_BUTTONS] =
{
  GPIO_BTN_POWERKEY_LOW_WHEN_PRESSED,
};

static const struct file_operations irqbuttons_fops =
{
  irqbuttons_open,    /* open */
  irqbuttons_close,   /* close */
  irqbuttons_read,    /* read */
  0,                  /* write */
  0,                  /* seek */
  0,                  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  irqbuttons_poll,    /* poll */
#endif
};

struct irqbuttons_dev_s irqbuttons_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int irqbuttons_open(FAR struct file *filep)
{
  DEBUGASSERT(filep);
  return OK;
}

static int irqbuttons_close(FAR struct file *filep)
{
  DEBUGASSERT(filep);
  return OK;
}

static ssize_t irqbuttons_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode *inode;
  FAR struct irqbuttons_dev_s *priv;
  int ret;

  if (len < 1)
    {
      return 0;
    }

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct irqbuttons_dev_s *)inode->i_private;

  ret = sem_wait(&priv->devsem);
  if (ret < 0)
    {
      DEBUGASSERT(errno == EINTR);
      return -EINTR;
    }

  *buffer = board_buttons();
  priv->int_pending = false;

  sem_post(&priv->devsem);

  return 1;
}

static void irqbuttons_notify(FAR struct irqbuttons_dev_s *priv)
{
#ifndef CONFIG_DISABLE_POLL
  int i;

  /* If there are threads waiting on poll() for device data to become available,
   * then wake them up now.  NOTE: we wake up all waiting threads because we
   * do not know that they are going to do.  If they all try to read the data,
   * then some make end up blocking after all.
   */

  for (i = 0; i < CONFIG_IRQBUTTONS_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          fds->revents |= POLLIN;
          ivdbg("Report events: %02x\n", fds->revents);
          sem_post(fds->sem);
        }
    }
#endif
}

#ifndef CONFIG_DISABLE_POLL
static int irqbuttons_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup)
{
  FAR struct inode *inode;
  FAR struct irqbuttons_dev_s *priv;
  int ret;
  int i;

  ivdbg("setup: %d\n", (int)setup);
  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv  = (FAR struct irqbuttons_dev_s *)inode->i_private;

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

      /* This is a request to set up the poll.  Find an available
       * slot for the poll structure reference
       */

      for (i = 0; i < CONFIG_IRQBUTTONS_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_IRQBUTTONS_NPOLLWAITERS)
        {
          fds->priv    = NULL;
          ret          = -EBUSY;
          goto errout;
        }

      /* Should we immediately notify on any of the requested events? */

      if (priv->int_pending)
        {
          irqbuttons_notify(priv);
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      struct pollfd **slot = (struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
    }

errout:
  sem_post(&priv->devsem);
  return ret;
}
#endif

static int irqbuttons_interrupt(int irq, FAR void *context)
{
  FAR struct irqbuttons_dev_s *priv = &irqbuttons_dev;

  priv->int_pending = true;
  irqbuttons_notify(priv);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.  After
 *   that, board_buttons() may be called to collect the current state of all
 *   buttons or board_button_irq() may be called to register button interrupt
 *   handlers.
 *
 ****************************************************************************/

void board_button_initialize(void)
{
  FAR struct irqbuttons_dev_s *priv = &irqbuttons_dev;
  int i, ret;
  char devname[DEV_NAMELEN];

  /* Configure the GPIO pins as inputs.  NOTE that EXTI interrupts are
   * configured for all pins.
   */

  for (i = 0; i < BOARD_NUM_BUTTONS; i++)
    {
      stm32_configgpio(g_buttons[i]);
    }

  sem_init(&priv->devsem, 0, 1);    /* Initialize device structure semaphore */
  priv->int_pending = true;         /* Button state is available from the start */

  /* Register new device node for button (to transfer irq to fd/poll). */

  (void)snprintf(devname, DEV_NAMELEN, DEV_FORMAT, 0);

  ret = register_driver(devname, &irqbuttons_fops, 0666, priv);
  if (ret < 0)
    {
      dbg("Could not register %s\n", devname);
      return;
    }

#ifdef CONFIG_ARCH_IRQBUTTONS
  board_button_irq(BOARD_BUTTON_POWERKEY, irqbuttons_interrupt);
#endif
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint8_t board_buttons(void)
{
  uint8_t ret = 0;
  int i;

  /* Check that state of each key */

  for (i = 0; i < BOARD_NUM_BUTTONS; i++)
    {
       bool high = stm32_gpioread(g_buttons[i]);
       bool pressed;

       if (g_buttons_lowpressed[i])
         {
           /* A LOW value means that the key is pressed.
            */

           pressed = !high;
         }
       else
         {
           /* A HIGH value means that the key is pressed.
            */

           pressed = high;
         }

       /* Accumulate the set of depressed (not released) keys */

       if (pressed)
         {
            ret |= (1 << i);
         }
    }

  return ret;
}

/************************************************************************************
 * Button support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.  After
 *   that, board_buttons() may be called to collect the current state of all
 *   buttons or board_button_irq() may be called to register button interrupt
 *   handlers.
 *
 *   After board_button_initialize() has been called, board_buttons() may be called to
 *   collect the state of all buttons.  board_buttons() returns an 8-bit bit set
 *   with each bit associated with a button.  See the BUTTON_*_BIT
 *   definitions in board.h for the meaning of each bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource. See the
 *   BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.  The previous interrupt handler address is returned (so that it may
 *   restored, if so desired).
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
xcpt_t board_button_irq(int id, xcpt_t irqhandler)
{
  xcpt_t oldhandler = NULL;

  /* The following should be atomic */

  if (id >= MIN_IRQBUTTON && id <= MAX_IRQBUTTON)
    {
      oldhandler = stm32_gpiosetevent(g_buttons[id], true, true, true, irqhandler);
    }

  return oldhandler;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
