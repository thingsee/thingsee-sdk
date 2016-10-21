/****************************************************************************
 * drivers/motor/stepper.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
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

/****************************************************************************
 * Compilation Switches
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/motor/stepper.h>

#include <arch/irq.h>

#ifdef CONFIG_STEPPER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing stepper */

#ifdef CONFIG_STEPPER_DEBUG
#  define stepperdbg    dbg
#  define steppervdbg   vdbg
#  define stepperlldbg  llvdbg
#else
#  define stepperdbg(x...)
#  define steppervdbg(x...)
#  define stepperlldbg(x...)
#endif

#define DEV_NAMELEN 16
#define DEV_FORMAT "/dev/mc%d"

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct stepper_upperhalf_s
{
  uint8_t                           crefs;    /* The number of times the device
                                                 has been opened */
  volatile bool                     started;  /* True: step output is being
                                                 generated */
  volatile bool                     waiting;  /* True: Caller is waiting for
                                                 the step count to expire */
  sem_t                             exclsem;  /* Supports mutual exclusion */
  sem_t                             waitsem;  /* Used to wait for the step count
                                                 to expire */
  FAR const struct stepper_board_s  *board;   /* Board configuration */
  struct stepper_info_s             info;     /* Step pattern characteristics */
  FAR struct stepper_lowerhalf_s    *dev;     /* lower-half state */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    stepper_dump(FAR const char *msg,
                            FAR const struct stepper_info_s *info,
                            bool started);
static int     stepper_open(FAR struct file *filep);
static int     stepper_close(FAR struct file *filep);
static ssize_t stepper_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t stepper_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int     stepper_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_stepperops =
{
  stepper_open,   /* open */
  stepper_close,  /* close */
  stepper_read,   /* read */
  stepper_write,  /* write */
  0,              /* seek */
  stepper_ioctl,  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0               /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stepper_dump
 ****************************************************************************/

static void stepper_dump(FAR const char *msg,
                         FAR const struct stepper_info_s *info,
                         bool started)
{
  steppervdbg("%s: frequency: %d\n", msg, info->frequency);
  steppervdbg("position: %d (%s)\n", info->position, info->pos_type ==
    POS_RELATIVE? "rel" : "abs");
  steppervdbg("started: %d\n", started);
}

/****************************************************************************
 * Name: stepper_open
 *
 * Description:
 *   This function is called whenever the stepper device is opened.
 *
 ****************************************************************************/

static int stepper_open(FAR struct file *filep)
{
  FAR struct inode                *inode = filep->f_inode;
  FAR struct stepper_upperhalf_s  *upper = inode->i_private;
  uint8_t                         tmp;
  int                             ret;

  steppervdbg("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      ret = -get_errno();
      goto errout;
    }

  /* Increment the count of references to the device. If this is the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* Check if this is the first time that the driver has been opened. */

  if (tmp == 1)
    {
      FAR struct stepper_lowerhalf_s *lower = upper->dev;

      DEBUGASSERT(lower->ops->setup != NULL);
      steppervdbg("calling setup\n");

      ret = lower->ops->setup(lower);
      if (ret < 0)
        {
          goto errout_with_sem;
        }
    }

  /* Board level function counts power control requests and enables power
   * if this is the first time stepper device is opened.
   */

  DEBUGASSERT(upper->board->set_power != NULL);
  upper->board->set_power(upper->board, true);

  /* Save the new open count on success */

  upper->crefs = tmp;
  ret = OK;

errout_with_sem:
  sem_post(&upper->exclsem);

errout:
  return ret;
}

/****************************************************************************
 * Name: stepper_close
 *
 * Description:
 *   This function is called when the stepper device is closed.
 *
 ****************************************************************************/

static int stepper_close(FAR struct file *filep)
{
  FAR struct inode                *inode = filep->f_inode;
  FAR struct stepper_upperhalf_s  *upper = inode->i_private;
  int                             ret;

  steppervdbg("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      ret = -get_errno();
      goto errout;
    }

  /* Decrement the references to the driver. If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 1)
    {
      upper->crefs--;
    }
  else
    {
      FAR struct stepper_lowerhalf_s *lower = upper->dev;

      /* There are no more references to the driver */

      DEBUGASSERT(upper->crefs == 1)

      upper->crefs = 0;
      upper->started = false;

      /* Disable the stepper device */

      DEBUGASSERT(lower->ops->shutdown != NULL);
      steppervdbg("calling shutdown: %d\n");

      lower->ops->shutdown(lower);
    }

  /* Board level function counts stepper power control requests and
   * disables power if there is no stepper devices opened.
   */

  DEBUGASSERT(upper->board->set_power != NULL);
  upper->board->set_power(upper->board, false);

  ret = OK;

  sem_post(&upper->exclsem);

errout:
  return ret;
}

/****************************************************************************
 * Name: stepper_read
 *
 * Description:
 *   This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t stepper_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/****************************************************************************
 * Name: stepper_write
 *
 * Description:
 *   This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t stepper_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: stepper_set_pos
 *
 * Description:
 *   Handle the MCIOC_SET_POS ioctl command
 *
 ****************************************************************************/

static int stepper_set_pos(FAR struct stepper_upperhalf_s *upper,
                           unsigned int oflags)
{
  FAR struct stepper_lowerhalf_s *lower = upper->dev;
  irqstate_t flags;
  int ret = OK;

  DEBUGASSERT(lower->ops->set_pos != NULL);

  /* Disable interrupts to avoid race conditions */

  flags = irqsave();

  if (!upper->started)
    {
      upper->waiting = ((oflags & O_NONBLOCK) == 0);
      upper->started = true;

      /* Invoke the bottom half method to start step pattern output */

      ret = lower->ops->set_pos(lower, &upper->info, upper);

      irqrestore(flags);

      /* A return value of zero means that the step pattern was started
       * successfully.
       */

      if (ret == OK)
        {
          /* Should we wait for the step pattern to complete? Loop in
           * in case the wakeup from sem_wait() is a false alarm.
           */

          while (upper->waiting)
            {
              /* Wait until we are awakened by stepper_expired. When
               * stepper_expired is called, it will post the waitsem
               * and clear the waiting flag.
               */

              int tmp = sem_wait(&upper->waitsem);
              DEBUGASSERT(tmp == OK || get_errno() == EINTR);
            }
        }
      else
        {
          steppervdbg("start failed: %d\n", ret);
          upper->started = false;
          upper->waiting = false;
        }
    }
  else
    {
      /* Invoke the bottom half method to update the stepper position */

      ret = lower->ops->set_pos(lower, &upper->info, upper);

      irqrestore(flags);
    }

  return ret;
}

/****************************************************************************
 * Name: stepper_ioctl
 *
 * Description:
 *   The standard ioctl method.
 *
 ****************************************************************************/

static int stepper_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode                *inode = filep->f_inode;
  FAR struct stepper_upperhalf_s  *upper = inode->i_private;
  FAR struct stepper_lowerhalf_s  *lower = upper->dev;
  int                             ret;

  steppervdbg("cmd: %d arg: %ld\n", cmd, arg);

  /* Get exclusive access to the device structures */

  ret = sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
      /* MCIOC_SET_POS - Start step pattern output with given parameters.
       *
       *   ioctl argument: reference to struct stepper_info_s that provides
       *   position index, step mode and frequency.
       */

      case MCIOC_SET_POS:
        {
          FAR const struct stepper_info_s *info =
            (FAR const struct stepper_info_s *)((uintptr_t)arg);

          DEBUGASSERT(lower->ops->set_pos != NULL);

          /* Save step pattern characteristics */

          memcpy(&upper->info, info, sizeof(struct stepper_info_s));
          stepper_dump("MCIOC_SET_POS", &upper->info, upper->started);

          /* Generate step pattern output until given position is reached */

          ret = stepper_set_pos(upper, filep->f_oflags);
        }
        break;

      /* MCIOC_GET_STATUS - Get stepper status.
       *
       *   ioctl argument: reference to struct stepper_status_s.
       */

      case MCIOC_GET_STATUS:
        {
          FAR struct stepper_status_s *status =
            (FAR struct stepper_status_s*)((uintptr_t)arg);

          DEBUGASSERT(status != NULL);

          steppervdbg("MCIOC_GET_STATUS: started: %d\n", upper->started);

          /* Get stepper status information */

          DEBUGASSERT(lower->ops->get_status != NULL);

          ret = lower->ops->get_status(lower, status);
        }
        break;

      /* MCIOC_RESET_INDEX - Reset index to current position.
       *
       *   ioctl argument:  None
       */

      case MCIOC_RESET_INDEX:
        {
          steppervdbg("MCIOC_RESET_INDEX: started: %d\n", upper->started);
          DEBUGASSERT(lower->ops->reset_index != NULL);

          ret = lower->ops->reset_index(lower);
        }
        break;

      /* MCIOC_SET_PATTERN_INDEX - Set pattern index to given value.
       *
       *   ioctl argument:  pattern index
       */

      case MCIOC_SET_PATTERN_INDEX:
        {
          steppervdbg("MCIOC_SET_PATTERN_INDEX: index: %u, started: %d\n",
            arg, upper->started);
          DEBUGASSERT(lower->ops->set_pattern_index != NULL);

          ret = lower->ops->set_pattern_index(lower, (unsigned int)arg);
        }
        break;

      /* MCIOC_STOP - Stop the step pattern output.
       *
       *   ioctl argument:  None
       */

      case MCIOC_STOP:
        {
          steppervdbg("MCIOC_STOP: started: %d\n", upper->started);
          DEBUGASSERT(lower->ops->stop != NULL);

          if (upper->started)
            {
              ret = lower->ops->stop(lower);
              upper->started = false;

              if (upper->waiting)
                {
                  upper->waiting = false;
                }
            }
        }
        break;

      /* Any unrecognized IOCTL commands might be platform-specific ioctl
         commands */

      default:
        {
          steppervdbg("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);
          DEBUGASSERT(lower->ops->ioctl != NULL);
          ret = lower->ops->ioctl(lower, cmd, arg);
        }
        break;
    }

  sem_post(&upper->exclsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stepper_register
 *
 * Description:
 *   This function binds an instance of a "lower half" timer driver with the
 *   "upper half" stepper control device and registers that device so that can
 *   be used by application code.
 *
 * Input parameters:
 *   minor - The minor device number. Stepper motor control drivers are
 *     installed as "/dev/mc0", "/dev/mc1", etc. where the driver path
 *     differs only in the "minor" number at the end of the device name.
 *   dev - pointer to an instance of lower half timer driver. This instance
 *     is bound to the stepper driver and must persists as long as the
 *     driver persists.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stepper_register(uint8_t minor,
                     FAR struct stepper_lowerhalf_s *dev,
                     FAR const struct stepper_board_s *board_config)
{
  FAR struct stepper_upperhalf_s *upper;
  char devpath[DEV_NAMELEN];

  /* Allocate the upper-half data structure */

  upper = (FAR struct stepper_upperhalf_s *)kmm_zalloc(sizeof(
    struct stepper_upperhalf_s));
  if (!upper)
    {
      stepperdbg("Allocation failed\n");
      return -ENOMEM;
    }

  sem_init(&upper->exclsem, 0, 1);
  sem_init(&upper->waitsem, 0, 0);
  upper->dev = dev;
  upper->board = board_config;

  /* Register the stepper motor device */

  (void)snprintf(devpath, DEV_NAMELEN, DEV_FORMAT, minor);
  steppervdbg("Registering %s\n", devpath);
  return register_driver(devpath, &g_stepperops, 0666, upper);
}

/****************************************************************************
 * Name: stepper_expired
 *
 * Input parameters:
 *   handle - This is the handle that was provided to the lower-half.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

void stepper_expired(FAR void *handle)
{
  FAR struct stepper_upperhalf_s *upper =
    (FAR struct stepper_upperhalf_s *)handle;

  stepperlldbg("started:%d, waiting:%d\n", upper->started, upper->waiting);

  if (upper->started)
    {
      /* Is there a thread waiting for the step pattern to complete? */

      if (upper->waiting)
        {
          upper->waiting = false;
          sem_post(&upper->waitsem);
        }

      /* The stepper is now stopped */

      upper->started = false;
    }
}

#endif /* CONFIG_STEPPER */
