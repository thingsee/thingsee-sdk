/*******************************************************************************
 * include/nuttx/lcd/memlcd.h
 * Common definitions for the Sharp Memory LCD driver
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
 ******************************************************************************/

#ifndef __INCLUDE_NUTTX_MEMLCD_H
#define __INCLUDE_NUTTX_MEMLCD_H

/*******************************************************************************
 * Included Files
 ******************************************************************************/

#include <stdbool.h>
#include <sys/ioctl.h>

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/*******************************************************************************
 * Pre-processor Definitions
 ******************************************************************************/

#ifdef CONFIG_MEMLCD_CHARACTER_CONSOLE_MODE

/* Memlcd driver ioctl commands ***********************************************/

#define _MEMLCDIOCVALID(c)   (_IOC_TYPE(c)==_DIOCBASE)
#define _MEMLCDIOC(nr)       _IOC(_DIOCBASE,nr)

#define MEMLCD_IOC_GETPOWER       _MEMLCDIOC(0x00f1) /* IN:  Pointer to write-able bool variable.
                                                      * OUT: bool is set with power-on status. */
#define MEMLCD_IOC_SETPOWER       _MEMLCDIOC(0x00f2) /* IN:  'bool' for new power-on state.
                                                      * OUT: None. */
#define MEMLCD_IOC_GETDIMENSIONS  _MEMLCDIOC(0x00f3) /* IN:  Pointer to write-able struct
                                                      *      memlcd_dimensions_s in which to receive
                                                      *      receive geometry data.
                                                      * OUT: Dimensions structure is populated
                                                      *      with data for the MemLCD. */
#define MEMLCD_IOC_GETCURSOR      _MEMLCDIOC(0x00f4) /* IN:  Pointer to write-able struct
                                                      *      memlcd_cursor_s in which to receive
                                                      *      receive cursor position.
                                                      * OUT: Cursor structure is populated
                                                      *      with data for the MemLCD. */
#define MEMLCD_IOC_SETCURSOR      _MEMLCDIOC(0x00f5) /* IN:  Pointer to struct memlcd_cursor_s
                                                      *      in which to new position for cursor
                                                      *      is available.
                                                      * OUT: None. */
#define MEMLCD_IOC_DRAWPIXELLINES _MEMLCDIOC(0x00f6) /* IN:  Pointer to struct memlcd_pixellines_s
                                                      *      which contains information for drawing
                                                      *      raw pixel lines to display.
                                                      * OUT: None. */
#define MEMLCD_IOC_CLEARDISPLAY   _MEMLCDIOC(0x00f7) /* IN:  None.
                                                      * OUT: None. */

#endif

/*******************************************************************************
 * Public Types
 ******************************************************************************/

/* Dimensions for the display, reported with ioctl MEMLCD_IOC_GETDIMENSIONS. */

struct memlcd_dimensions_s
{
  /* Display raw pixel dimensions. */

  uint16_t width_pixels;
  uint16_t height_pixels;

  /* Display character console dimensions. */

  uint16_t cons_rows;
  uint16_t cons_cols;
};

/* Character console cursor position for the display, operated with ioctls
 * MEMLCD_IOC_GETCURSOR and MEMLCD_IOC_SETCURSOR. */

struct memlcd_cursor_s
{
  uint16_t rowpos;
  uint16_t colpos;
};

/* Structure for controlling raw pixel writing, used by ioctl
 * MEMLCD_IOC_DRAWPIXELLINES. */

struct memlcd_pixellines_s
{
  uint16_t first_line;
  uint16_t num_lines;

  FAR const void *pixels;
  size_t pixelbuflen;
};

/* A reference to a structure of this type must be passed to the initialization
 * method. It provides some board-specific hooks used by driver to manage the
 * timer and gpio (IRQ and DISP).
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

struct memlcd_priv_s
{
  /* IRQ/GPIO access callbacks.  These operations all hidden behind
   * callbacks to isolate the Memory LCD driver from differences in GPIO
   * interrupt handling by varying boards and MCUs.
   *
   * irqattach   - Attach the driver interrupt handler to the GPIO interrupt
   *               If isr is NULL, detach and disable it.
   *               Needed only when CONFIG_MEMLCD_EXTCOMIN_MODE_HW is not set.
   * setpolarity - Board specified method to set EXTCOMIN.
   *               Needed only when CONFIG_MEMLCD_EXTCOMIN_MODE_HW is not set.
   * dispcontrol - Enable or disable the DISP pin and EXTCOMIN interrupt.
   * setvcomfreq - Set timer frequency for EXTCOMIN.
   */

#ifndef CONFIG_MEMLCD_EXTCOMIN_MODE_HW
  int (*attachirq) (xcpt_t isr);
  void (*setpolarity) (bool pol);
#endif
  void (*dispcontrol) (bool on);
  void (*setvcomfreq) (unsigned int freq);
};

/*******************************************************************************
 * Public Data
 ******************************************************************************/

/*******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Name:  memlcd_initialize
 *
 * Description:
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 throuh CONFIG_MEMLCD_NINTERFACES-1.
 *   This allows support for multiple devices.
 *   memlcd_priv_s - Board specific structure
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for the
 *   specified LCD.  NULL is returned on any failure.
 *
 ******************************************************************************/

struct lcd_dev_s;               /* see nuttx/lcd.h */
struct spi_dev_s;               /* see nuttx/spi/spi.h */
FAR struct lcd_dev_s *memlcd_initialize(FAR struct spi_dev_s *spi,
                                        FAR struct memlcd_priv_s *priv,
                                        unsigned int devno);

#ifdef CONFIG_MEMLCD_CHARACTER_CONSOLE_MODE

/****************************************************************************
 * Name: memlcd_console_register
 *
 * Description:
 *   Register a console device on a memory LCD.  The device will be
 *   registered at /dev/memlcdconN where N is the provided minor number.
 *
 * Input Parameters:
 *   dev - A handle that will be used to access the memory LCD device.
 *   minor - The device minor number
 *
 * Return:
 *   OK on success; ERROR on failure with errno set appropriately
 *
 ****************************************************************************/

int memlcd_console_register(FAR struct lcd_dev_s *dev, int minor);

#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_MEMLCD_H */
