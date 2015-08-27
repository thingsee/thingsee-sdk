/****************************************************************************
 *
 * Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
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
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#include "chip.h"
#include "stm32_gpio.h"
#include "stm32_exti.h"
#include "stm32_uart.h"

#include <arch/board/board-pwrctl.h>
#include <arch/board/board.h>
#include <arch/board/board-device.h>
#include "haltian-tsone.h"
#include "up_serialrxdma_poll.h"

#include <arch/board/board-bt.h>
#include <termios.h>
#include <nuttx/serial/tioctl.h>

#ifdef CONFIG_DEBUG_BLUETOOTH
#  define bt_dbg(x, ...)	dbg(x, ##__VA_ARGS__)
#  define bt_lldbg(x, ...)	lldbg(x, ##__VA_ARGS__)
#else
#  define bt_dbg(x, ...)
#  define bt_lldbg(x, ...)
#endif

/****************************************************************************
 * Name: board_bt_init_pins
 *
 * Description:
 *   Safely initialize pins
 *
 * Input Parameters:
 *
 * Returned Values:
 *
 ****************************************************************************/

static void board_bt_init_pins(void)
{
  if (board_get_hw_ver() < BOARD_HWVER_B2_0)
    {
      stm32_configgpio(GPIO_BT_RESERVED1);
      stm32_configgpio(GPIO_BT_RESERVED2);
    }

  stm32_configgpio(GPIO_BT_RESERVED3);
  stm32_configgpio(GPIO_BT_RESET_N);
  stm32_configgpio(GPIO_REGULATOR_BLUETOOTH);


  stm32_configgpio(GPIO_BT_CTS);
  stm32_configgpio(GPIO_BT_RTS);

  stm32_gpiowrite(GPIO_BT_RESERVED3, true); /* As a backdoor disabled */
  stm32_gpiowrite(GPIO_BT_CTS, false); /* Not used currently : don't supply v */
  stm32_gpiowrite(GPIO_BT_RTS, false); /* Not used currently : don't supply v */
  stm32_gpiowrite(GPIO_BT_RESET_N, true);
}

/****************************************************************************
 * Name: board_bt_deinit_pins
 *
 * Description:
 *   Safely de-initialize pins
 *
 * Input Parameters:
 *
 * Returned Values:
 *
 ****************************************************************************/

static void board_bt_deinit_pins(void)
{
  stm32_gpiowrite(GPIO_BT_RESET_N, true);
  stm32_gpiowrite(GPIO_BT_RESET_N, false);
  stm32_gpiowrite(GPIO_REGULATOR_BLUETOOTH, false);

  if (board_get_hw_ver() < BOARD_HWVER_B2_0)
    {
      stm32_unconfiggpio(GPIO_BT_RESERVED1);
      stm32_unconfiggpio(GPIO_BT_RESERVED2);
    }

  stm32_gpiowrite(GPIO_BT_RESERVED3, false);/*Backdoor enabled -> !supplyvolt */
}

/****************************************************************************
 * Name: board_bt_power_device_up
 *
 * Description:
 *   Powers BT device ON/OFF
 *
 * Input Parameters:
 *   enable:
 *     		true -  Powers device ON if not already powered ON
 *     		false - Powers device down if not already powered down
 *
 * Returned Values:
 *
 *
 ****************************************************************************/

void board_bt_power_device_up(bool enable)
{
  static bool poweredup = false;

  if (enable && !poweredup)
    {
      board_bt_init_pins();

      board_pwrctl_get(PWRCTL_REGULATOR_BLUETOOTH);
      poweredup = true;
      up_launch_serialrxdma_poll();
    }
  else if (!enable && poweredup)
    {
      board_pwrctl_put(PWRCTL_REGULATOR_BLUETOOTH);
      poweredup = false;
      board_bt_deinit_pins();
    }
}

/****************************************************************************
 * Name: board_bt_init_dev
 *
 * Description:
 *   Opens BT's file descriptor for further use
 *
 * Input Parameters:
 *
 * Returned Values:
 * 	 On success returns file descriptor. On failure an ERROR, which, I guess,
 * 	 is equal to -1
 *
 ****************************************************************************/

int board_bt_init_dev(void)
{
	int fd = -1;
	fd = open(BLUETOOTH_SERIAL_DEVNAME, O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		perror("Cannot open bluetooth device. Check if it was powered on");
		return ERROR;
	}

	bt_lldbg("Serial port is opened: descriptor=%d\n", fd);

	return fd;
}

/****************************************************************************
 * Name: board_bt_deinit_dev
 *
 * Description:
 *   Closes BT's file descriptor
 *
 ****************************************************************************/

int board_bt_deinit_dev(int fd)
{
  /* Close file-descriptor. */

  return close(fd);
}

/****************************************************************************
 * Name: board_bt_backdoor
 *
 * Description:
 *   Enable bootloader backdoor
 *
 * Input Parameters:
 *
 * Returned Values:
 *
 ****************************************************************************/

void board_bt_backdoor(bool enable)
{
  if (enable)
    stm32_gpiowrite(GPIO_BT_RESERVED3, false);
  else
    stm32_gpiowrite(GPIO_BT_RESERVED3, true);

  usleep((1 + 10));
}
/****************************************************************************
 * Name: board_bt_hw_reset
 *
 * Description:
 *   Executes HW-reset
 *
 * Input Parameters:
 *
 * Returned Values:
 *
 ****************************************************************************/

void board_bt_hw_reset(void)
{
	/* Reset RESET_N pin and hold it low for (100000) us */
	stm32_gpiowrite(GPIO_BT_RESET_N, false);
	usleep(100000);
	stm32_gpiowrite(GPIO_BT_RESET_N, true);
}

/****************************************************************************
 * Name: board_bt_change_baud_rate
 *
 * Description:
 *   Changes baud-rate
 *
 * Input Parameters:
 * 		baud_rate - baud rate to apply
 * 		fd - file descriptor opened earlier
 *
 * Returned Values:
 * 		On success returns 0. On Error -1
 *
 ****************************************************************************/

int board_bt_change_baud_rate(unsigned int baud_rate, int fd)
{
	int ret = OK;
	struct termios uart;

	bt_lldbg("Baud-rate=%d\n", baud_rate);

	ret = ioctl(fd, TCGETS, (unsigned int)&uart);

	uart.c_speed = baud_rate;

	ret = ioctl(fd, TCSETS, (unsigned int)&uart);

	return ret;
}
