/****************************************************************************
 * include/nuttx/motor/stepper.h
 *
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

#ifndef __INCLUDE_NUTTX_STEPPER_H
#define __INCLUDE_NUTTX_STEPPER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <fixedmath.h>

#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_STEPPER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_STEPPER - Upper half stepper motor control driver support
 * CONFIG_DEBUG_STEPPER - If enabled (with CONFIG_DEBUG and, optionally,
 *   CONFIG_DEBUG_VERBOSE), this will generate output that can be used to
 *   debug the stepper motor control driver.
 */

/* IOCTL Commands ***********************************************************/
/*
 * MCIOC_SET_POS - Set motor position to given index.
 *
 *   ioctl argument:  Reference to struct stepper_info_s to define the
 *   characteristics of the step pattern.
 *
 * MCIOC_GET_STATUS - Get current status information.
 *
 *   ioctl argument:  Reference to struct stepper_status_s to return stepper
 *   status information.
 *
 * MCIOC_RESET_INDEX - Reset position index to current position.
 *
 *   ioctl argument:  None
 *
 * MCIOC_SET_PATTERN_INDEX - Set step pattern index to given value.
 *
 *   ioctl argument:  step pattern index.
 *
 * MCIOC_STOP - Stop the step pattern output signal. This command will
 *   stop the associated timer and return immediately.
 *
 *   ioctl argument:  None
 */

#define MCIOC_SET_POS           _MCIOC(1)
#define MCIOC_GET_STATUS        _MCIOC(2)
#define MCIOC_RESET_INDEX       _MCIOC(3)
#define MCIOC_SET_PATTERN_INDEX _MCIOC(4)
#define MCIOC_STOP              _MCIOC(5)

enum position_type_e
{
  POS_RELATIVE = 0,
  POS_ABSOLUTE
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Board configuration */

struct stepper_board_s
{
  int (*set_power) (FAR const struct stepper_board_s *state, bool on);
};

/* This structure describes the characteristics of the stepper pattern */

struct stepper_info_s
{
  uint32_t              frequency;  /* Frequency of the step pattern */
  int32_t               position;   /* Target position index */
  enum position_type_e  pos_type;   /* Position index type */
};

/* This structure describes the status of the stepper */

struct stepper_status_s
{
  int32_t               target_pos;   /* Target position index */
  int32_t               current_pos;  /* Current position index */
  uint32_t              frequency;    /* Stepping frequency */
  uint8_t               step_index;   /* Step pattern index */
};

/* This structure is a set a callback functions used to call from the upper-
 * half stepper driver into lower-half, platform-specific logic that
 * supports the low-level timer outputs.
 */

struct stepper_lowerhalf_s;
struct stepper_ops_s
{
  /* This method is called when the driver is opened. The lower half driver
   * should configure and initialize the device so that it is ready for use.
   * It does not generate step pattern output until the set_pos method is
   * called.
   */

  CODE int (*setup)(FAR struct stepper_lowerhalf_s *dev);

  /* This method is called when the driver is closed. The lower half driver
   * should stop step pattern output, free any resources and disable the
   * timer hardware.
   */

  CODE int (*shutdown)(FAR struct stepper_lowerhalf_s *dev);

  /* Initialize timer resources and start step pattern output. The set_pos
   * method returns an error if it cannot start the timer with the given
   * step pattern parameters.
   */

  CODE int (*set_pos)(FAR struct stepper_lowerhalf_s *dev,
                      FAR const struct stepper_info_s *info,
                      FAR void *handle);

  /* Get status of the stepper. */

  CODE int (*get_status)(FAR struct stepper_lowerhalf_s *dev,
                         FAR struct stepper_status_s *status);

  /* Reset position index. */

  CODE int (*reset_index)(FAR struct stepper_lowerhalf_s *dev);

  /* Set step pattern index. */

  CODE int (*set_pattern_index)(FAR struct stepper_lowerhalf_s *dev,
                                unsigned int index);

  /* Stop the step pattern output and reset the timer resources. */

  CODE int (*stop)(FAR struct stepper_lowerhalf_s *dev);

  /* Lower-half logic may support platform-specific ioctl commands */

  CODE int (*ioctl)(FAR struct stepper_lowerhalf_s *dev,
                    int cmd, unsigned long arg);
};

/* This structure is the generic form of state structure used by lower half
 * stepper motor control driver. This state structure is passed to the
 * stepper driver when the driver is initialized.
 */

struct stepper_lowerhalf_s
{
  /* The first field of this state structure must be a pointer to the stepper
   * motor control callback structure:
   */

  FAR const struct stepper_ops_s *ops;

  /* The custom timer state structure may include additional fields after
   * the pointer to the stepper motor control callback structure.
   */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * "Upper-Half" stepper control driver interfaces
 ****************************************************************************/
/****************************************************************************
 * Name: stepper_register
 *
 * Description:
 *   This function binds an instance of a "lower half" timer driver with the
 *   "upper half" stepper motor control device and registers that device so
 *   that can be used by application code.
 *
 * Input parameters:
 *   minor - minor number at the end of the device name, such as "mc0".
 *   dev - A pointer to an instance of lower half timer driver. This instance
 *     is bound to the stepper motor control driver.
 *   board_config - stepper board configuration
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stepper_register(uint8_t minor,
                     FAR struct stepper_lowerhalf_s *dev,
                     FAR const struct stepper_board_s *board_config);

/****************************************************************************
 * Name: stepper_expired
 *
 * Description:
 *   "upper half" driver will wait for the step pattern to expire.
 *   The sequence of expected events is as follows:
 *
 *   1. The upper half driver calls the set_pos method, providing the lower
 *      half driver with the step pattern characteristics.
 *   2. When the set_pos method returns success, the upper half driver
 *      will wait until the motorctl_expired method is called.
 *   3. When the lower half detects that the step pattern has completed
 *      it must call the motorctl_expired interface using the handle that
 *      was previously passed to the set_pos method.
 *
 * Input parameters:
 *   handle - This is the handle that was provided to the lower-half
 *     set_pos method.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

void stepper_expired(FAR void *handle);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_STEPPER */
#endif /* __INCLUDE_NUTTX_STEPPER_H */
