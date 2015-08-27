/****************************************************************************
 * config/haltian-tsone/src/up_bringup.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *   Authors:
 *     Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *     Sami Pelkonen <sami.pelkonen@haltian.com>
 *     Timo Voutilainen <timo.voutilainen@haltian.com>
 *     Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
 *     Juha Niskanen <juha.niskanen@haltian.com>
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
#include <fcntl.h>
#include <stdlib.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#ifdef CONFIG_SYSTEM_USBMSC
#include <nuttx/usb/usbmsc.h>
#endif
#ifdef CONFIG_USBDEV_COMPOSITE
#  include <../../apps/system/composite/composite.h>
#endif

#include <arch/board/board.h>
#include <arch/board/board-pwrctl.h>
#ifdef CONFIG_ADC
#include <up_adc.h>
#endif
#if CONFIG_BLUETOOTH
#  include <arch/board/board-bt.h>
#endif

#include "haltian-tsone.h"
#include "up_i2cdev_init.h"

#include "stm32.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef CONFIG_BOARD_MMCSDSLOTNO
#  define CONFIG_BOARD_MMCSDSLOTNO 0
#endif
#ifndef CONFIG_BOARD_MMCSDMINOR
#  define CONFIG_BOARD_MMCSDMINOR 0
#endif
#ifndef CONFIG_RND_CONSOLE_USBDEV
#define CONFIG_RND_CONSOLE_USBDEV "/dev/ttyACM0"
#endif
/****************************************************************************
 * Private Types
 ****************************************************************************/

struct thread_params_s
{
  int uartfd;
  int infd;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

FAR struct spi_dev_s *spi_mmcsd;
FAR struct spi_dev_s *spi_oled;

#ifdef CONFIG_USBDEV_COMPOSITE

struct composite_state_s g_composite;

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_BOARD_RND_USE_USB_CONSOLE

static void console_configstdio(int infd, int outfd, int errfd)
{
  if (infd >= 0)
      (void)fflush(stdin);
  if (outfd >= 0)
      (void)fflush(stdout);
  if (errfd >= 0)
      (void)fflush(stderr);

  /* Make sure the stdin, stdout, and stderr are closed */

  if (infd >= 0)
      (void)fclose(stdin);
  if (outfd >= 0)
      (void)fclose(stdout);
  if (errfd >= 0)
      (void)fclose(stderr);

  /* Dup the fd to create standard fd 0-2 */

  if (infd >= 0)
      (void)dup2(infd, 0);
  if (outfd >= 0)
      (void)dup2(outfd, 1);
  if (errfd >= 0)
      (void)dup2(errfd, 2);

  /* fdopen to get the stdin, stdout and stderr streams. The following logic depends
   * on the fact that the library layer will allocate FILEs in order.  And since
   * we closed stdin, stdout, and stderr above, that is what we should get.
   *
   * fd = 0 is stdin  (read-only)
   * fd = 1 is stdout (write-only, append)
   * fd = 2 is stderr (write-only, append)
   */

  if (infd >= 0)
      (void)fdopen(0, "r");
  if (outfd >= 0)
      (void)fdopen(1, "a");
  if (errfd >= 0)
      (void)fdopen(2, "a");
}

static void write_all(int fd, const char *buf, size_t len)
{
  ssize_t nwritten;

  if (fd < 0)
    return;

  do
    {
      nwritten = write(fd, buf, len);
      if (nwritten < 0)
        {
          /* Error, skip output. */
          nwritten = len;
        }

      len -= nwritten;
      buf += nwritten;
    }
  while (len > 0);
}

static void *console_output_thread(void *param)
{
  struct thread_params_s p = *(struct thread_params_s *)param;
  ssize_t nread;
  char buf[32];
  int flags;
  int usbfd = -1;

  free(param);

  while (true)
    {
      nread = read(p.infd, buf, sizeof(buf));
      if (nread > 0)
        {
          write_all(p.uartfd, buf, nread);

          if (usbfd < 0)
            {
              /* Try to open the console */

              usbfd = open(CONFIG_RND_CONSOLE_USBDEV, O_RDWR);
              if (usbfd < 0)
                {
                  /* ENOTCONN means that the USB device is not yet connected. Anything
                   * else is bad.
                   */
                  DEBUGASSERT(errno == ENOTCONN);
                }
              else
                {
                  /* Make USB non-blocking to avoid stalls. */

                  flags = fcntl(usbfd, F_GETFL, NULL);
                  if (flags >= 0)
                      (void)fcntl(usbfd, F_SETFL, flags | O_NONBLOCK);
                }
            }
          else
            {
              write_all(usbfd, buf, nread);
            }
        }
    }

  return NULL;
}

static int setup_console_output_pipe(void)
{
  pthread_attr_t attr;
  struct thread_params_s *params;
  int uartfd;
  int pfd[2] = { -1, -1 };
  int err;
  int ret;
  pthread_t thread;

  dbg("Opening UART/USB output pipe...\n");

  ret = pipe(pfd);
  if (ret < 0)
    {
      err = errno;
      goto err_out;
    }

  /* Duplicate current stdout file descriptor (UART). */

  uartfd = dup(1);
  if (uartfd < 0)
    {
      err = errno;
      goto err_close_pipe;
    }

  params = calloc(1, sizeof(*params));
  if (params == NULL)
    {
      err = errno;
      goto err_close_uart;
    }

  params->uartfd = uartfd;
  params->infd = pfd[0];

  pthread_attr_init(&attr);

  ret = pthread_create(&thread, &attr, console_output_thread, params);
  if (ret < 0)
    {
      err = -ret;
      goto err_free_params;
    }

  return pfd[1];

err_free_params:
  free(params);
err_close_uart:
  close(uartfd);
err_close_pipe:
  close(pfd[0]);
  close(pfd[1]);
err_out:
  errno = err;
  return -1;
}

static void setup_rnd_console(void)
{
  int outfd;

  dbg("Switching to use USB serial console...\n");

  outfd = setup_console_output_pipe();
  if (outfd < 0)
    {
      dbg("Failed to setup console output pipe.\n");
      return;
    }

  /* Configure standard I/O */

  console_configstdio(-1, outfd, outfd);

  /* We can close pipe output file descriptor. */

  if (outfd > 2)
    close(outfd);

  dbg("USB serial console enabled.\n");
}

#else

static void setup_rnd_console(void)
{
  /* Do nothing; use default console. */
}

#endif /*CONFIG_BOARD_RND_USE_USB_CONSOLE*/

/****************************************************************************
 * Name: up_sdinitialize
 *
 * Description:
 *   Initialize SPI-based microSD.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_SPI3
static int up_sdinitialize(void)
{
  int ret;

  /* First, turn on regulator for SDcard */

  board_pwrctl_get(PWRCTL_REGULATOR_SDCARD);

  /* First, get an instance of the SPI interface */

  lldbg("up_archinitialize: Initializing SPI port 3\n");
  spi_mmcsd = up_spiinitialize(3);
  if (!spi_mmcsd)
    {
      lldbg("up_archinitialize: Failed to initialize SPI port 3\n");
      return -ENODEV;
    }

  /* Now bind the SPI interface to the MMC/SD driver */

  ret = mmcsd_spislotinitialize(CONFIG_BOARD_MMCSDMINOR,
                                CONFIG_BOARD_MMCSDSLOTNO, spi_mmcsd);
  if (ret != OK)
    {
      lldbg("up_archinitialize: "
              "Failed to bind SPI to the MMC/SD driver: %d\n",
              ret);

      return ret;
    }

  return OK;
}
#else /* !SPI3 */
# define up_sdinitialize() ((void)0)
#endif /* SPI3*/

#ifdef CONFIG_SYSTEM_USBMSC
#ifndef CONFIG_USBMSC_COMPOSITE
int up_msconn_init(void)
{
  FAR void *handle;
  int ret;

  lldbg("mcsonn_main: Configuring with NLUNS=%d\n", CONFIG_SYSTEM_USBMSC_NLUNS);
  ret = usbmsc_configure(CONFIG_SYSTEM_USBMSC_NLUNS, &handle);
  if (ret < 0)
    {
      lldbg("mcsonn_main: usbmsc_configure failed: %d\n", -ret);
      usbmsc_uninitialize(handle);
      return ERROR;
    }

  lldbg("mcsonn_main: handle=%p\n", handle);

  lldbg("mcsonn_main: Bind LUN=0 to %s\n", CONFIG_SYSTEM_USBMSC_DEVPATH1);
  ret = usbmsc_bindlun(handle, CONFIG_SYSTEM_USBMSC_DEVPATH1, 0, 0, 0, false);
  if (ret < 0)
    {
      lldbg("mcsonn_main: usbmsc_bindlun failed for LUN 1 using %s: %d\n",
               CONFIG_SYSTEM_USBMSC_DEVPATH1, -ret);
      usbmsc_uninitialize(handle);
      return ERROR;
    }

#if CONFIG_SYSTEM_USBMSC_NLUNS > 1

  lldbg("mcsonn_main: Bind LUN=1 to %s\n", CONFIG_SYSTEM_USBMSC_DEVPATH2);
  ret = usbmsc_bindlun(handle, CONFIG_SYSTEM_USBMSC_DEVPATH2, 1, 0, 0, false);
  if (ret < 0)
    {
      lldbg("mcsonn_main: usbmsc_bindlun failed for LUN 2 using %s: %d\n",
               CONFIG_SYSTEM_USBMSC_DEVPATH2, -ret);
      usbmsc_uninitialize(handle);
      return ERROR;
    }

#if CONFIG_SYSTEM_USBMSC_NLUNS > 2

  lldbg("mcsonn_main: Bind LUN=2 to %s\n", CONFIG_SYSTEM_USBMSC_DEVPATH3);
  ret = usbmsc_bindlun(handle, CONFIG_SYSTEM_USBMSC_DEVPATH3, 2, 0, 0, false);
  if (ret < 0)
    {
      lldbg("mcsonn_main: usbmsc_bindlun failed for LUN 3 using %s: %d\n",
               CONFIG_SYSTEM_USBMSC_DEVPATH3, -ret);
      usbmsc_uninitialize(handle);
      return ERROR;
    }

#endif
#endif

  ret = usbmsc_exportluns(handle);
  if (ret < 0)
    {
      lldbg("mcsonn_main: usbmsc_exportluns failed: %d\n", -ret);
      usbmsc_uninitialize(handle);
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_SYSTEM_USBMSC */
#endif /* CONFIG_USBMSC_COMPOSITE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_NET
void up_netinitialize(void)
{
}
#endif

/****************************************************************************
 * Name: up_bringup
 *
 * Description:
 *   Perform board initialization
 *
 ****************************************************************************/

int up_bringup(void)
{
  int ret;

#if defined(CONFIG_STM32_PWR)
  board_pwr_enablepvd();
#endif

  /* Initialize ADC early, for VBAT measurements. */

  ret = adc_devinit();
  if (ret < 0)
    return ret;

  /* Check that we have enough battery left, just in case
     PVD detection missed it. */

  board_pwr_checkvbat(false);

#if defined(CONFIG_ARCH_BUTTONS)
  /* Register buttons */

  board_button_initialize();
#endif

#ifdef CONFIG_LCD_SHARP_MEMLCD
  /* Initialize Memory LCD. */

  up_memlcd_initialize();
#endif

  up_sdinitialize();

  /* Initialize USB driver */
#ifdef CONFIG_USBDEV_COMPOSITE
  g_composite.cmphandle = composite_initialize();
    if (!g_composite.cmphandle)
      {
        lldbg("composite_initialize failed\n");
        return ERROR;
      }

    close(g_composite.infd);
    close(g_composite.outfd);
#elif !defined(CONFIG_USBDEV_COMPOSITE) && defined(CONFIG_BOARD_INITIALIZE_USB_SERIAL)

#  if defined(CONFIG_PL2303)
      ret = usbdev_serialinitialize(0);
      DEBUGASSERT(ret == OK);
#  elif defined(CONFIG_CDCACM)
      ret = cdcacm_initialize(0, NULL);
      DEBUGASSERT(ret == OK);
#  endif

#endif

  ret = up_i2c_devinitialize();
  DEBUGASSERT(ret == OK);

#ifdef CONFIG_WL_CC3000
  nsh_cc3000_start();
#endif

  setup_rnd_console();
#ifdef CONFIG_STM32_LCD
  /* Initialize the SLCD and register the SLCD device as /dev/slcd */

  return stm32_slcd_initialize();
#else
  return OK;
#endif
}
