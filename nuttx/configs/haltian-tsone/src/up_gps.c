/****************************************************************************
 * configs/haltian-tsone/src/up_gps.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *   Authors: Sami Pelkonen <sami.pelkonen@haltian.com>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "chip.h"
#include "stm32_gpio.h"

#include <arch/board/board.h>
#include <arch/board/board-gps.h>
#include <arch/board/board-pwrctl.h>
#include "haltian-tsone.h"
#include "up_serialrxdma_poll.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool powered = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gps_power
 *
 * Description:
 *   Power GPS chip on / off
 *
 ****************************************************************************/

void board_gps_power(bool on)
{
  if (!powered && on)
    {
      /* Power on GPS chip */

      board_pwrctl_get(PWRCTL_REGULATOR_GPS);
      powered = true;
      up_launch_serialrxdma_poll();
    }
  else if (powered && !on)
    {
      /* Power off GPS chip */

      board_pwrctl_put(PWRCTL_REGULATOR_GPS);

      powered = false;
    }
}

/****************************************************************************
 * Name: board_gps_initialize
 *
 * Description:
 *   Initialize the u-blox GPS.
 *
 * Return:
 *   Opened GPS serial port file descriptor.
 *
 ****************************************************************************/

int board_gps_initialize(void)
{
  /* Open serial. */

  return open(GPS_SERIAL_DEVNAME, O_RDWR, 0666);
}

/****************************************************************************
 * Name: board_gps_deinitialize
 *
 * Description:
 *   Deinitialize the u-blox GPS.
 *
 ****************************************************************************/

int board_gps_deinitialize(int fd)
{
  /* Close file-descriptor. */

  return close(fd);
}

/****************************************************************************
 * Name: board_gps_tx_buffer_empty
 *
 * Description:
 *   Check whether GPS UART TX buffer is empty
 *
 ****************************************************************************/

bool board_gps_tx_buffer_empty(int fd)
{
  int bytes_free;
  int ret;

  ret = ioctl(fd, FIONWRITE, (unsigned long)&bytes_free);
  if (ret < 0)
    return true;

  /* Serial driver buffer holds N bytes, but has max capacity N-1 */

  return bytes_free == (CONFIG_USART1_TXBUFSIZE - 1);
}

