/****************************************************************************
 * examples/stepper/stepper_main.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/boardctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include <nuttx/motor/stepper.h>

#include "stepper.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEFAULT_MOTOR       0
#define DEFAULT_FREQ        200
#define DEFAULT_STEPS       20
#define DEFAULT_LOOPS       1
#define DEFAULT_DELAY       0
#define DEFAULT_REL_POS     1
#define DEFAULT_BLOCKING    0
#define DEFAULT_RESET_INDEX 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stepper_state_s
{
  uint8_t motor;
  uint8_t rel_pos;
  uint8_t blocking;
  bool reset_index;
  int32_t steps;
  uint32_t freq;
  uint32_t loops;
  uint32_t delay_ms;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stepper_state_s g_stepper;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stepper_help
 ****************************************************************************/

static void stepper_help(void)
{
  printf("%s usage: stepper -<param1><value> -<paramN><value>\n", __func__);
  printf("parameters:\n");
  printf("'m' -> motor device minor number (%u)\n", DEFAULT_MOTOR);
  printf("'r' -> relative stepping (%u)\n", DEFAULT_REL_POS);
  printf("'s' -> steps (%u)\n", DEFAULT_STEPS);
  printf("'f' -> frequency (%u)\n", DEFAULT_FREQ);
  printf("'d' -> delay [ms] (%u)\n", DEFAULT_DELAY);
  printf("'b' -> blocking mode (%u)\n", DEFAULT_BLOCKING);
  printf("'l' -> loops (%u)\n", DEFAULT_LOOPS);
  printf("'i' -> reset index (%u)\n", DEFAULT_RESET_INDEX);
}

/****************************************************************************
 * Name: arg_string
 ****************************************************************************/

static int arg_string(FAR char **arg, FAR char **value)
{
  FAR char *ptr = *arg;

  if (ptr[2] == '\0')
    {
      *value = arg[1];
      return 0;
    }
  else
    {
      *value = &ptr[2];
      return 1;
    }
}

/****************************************************************************
 * Name: arg_decimal
 ****************************************************************************/

static int arg_decimal(FAR char **arg, FAR long *value)
{
  FAR char *string;
  int ret;

  ret = arg_string(arg, &string);
  *value = strtol(string, NULL, 10);
  return ret;
}

/****************************************************************************
 * Name: parse_args
 ****************************************************************************/

static int parse_args(int argc, FAR char **argv)
{
  FAR char *ptr;
  long value;
  int index;
  int nargs;

  g_stepper.motor = DEFAULT_MOTOR;
  g_stepper.freq = DEFAULT_FREQ;
  g_stepper.steps = DEFAULT_STEPS;
  g_stepper.rel_pos = DEFAULT_REL_POS;
  g_stepper.delay_ms = DEFAULT_DELAY;
  g_stepper.blocking = DEFAULT_BLOCKING;
  g_stepper.loops = DEFAULT_LOOPS;
  g_stepper.reset_index = DEFAULT_RESET_INDEX;

  for (index = 1; index < argc; )
    {
      ptr = argv[index];
      if (ptr[0] != '-')
        {
          printf("Invalid options format: %s\n", ptr);
          exit(0);
        }

      switch (ptr[1])
        {
          /* Motor device to be controlled */

          case 'm':
            nargs = arg_decimal(&argv[index], &value);
            if (nargs == 0 || value < 0)
              {
                printf("'m' out of range, nargs:%d, value:%ld\n", nargs, value);
                return ERROR;
              }

            g_stepper.motor = (uint8_t)value;
            index += nargs;
            break;

          /* Step count */

          case 's':
            nargs = arg_decimal(&argv[index], &value);
            if (nargs == 0)
              {
                printf("'s' out of range, nargs:%d\n", nargs);
                return ERROR;
              }

            g_stepper.steps = (int32_t)value;

            index += nargs;
            break;

          /* Frequency */

          case 'f':
            nargs = arg_decimal(&argv[index], &value);
            if (nargs == 0 || value < 1)
              {
                printf("'f' out of range, nargs:%d, value:%ld\n", nargs, value);
                return ERROR;
              }

            g_stepper.freq = (uint32_t)value;

            index += nargs;
            break;

          /* Relative position index */

          case 'r':
            nargs = arg_decimal(&argv[index], &value);
            if (nargs == 0 || (value < 0 || value > 1))
              {
                printf("'r' out of range, nargs:%d, value:%ld\n", nargs, value);
                return ERROR;
              }

            g_stepper.rel_pos = (uint8_t)value;

            index += nargs;
            break;

          /* Delay between loops */

          case 'd':
            nargs = arg_decimal(&argv[index], &value);
            if (nargs == 0 || value < 0)
              {
                printf("'d' out of range, nargs:%d, value:%ld\n", nargs, value);
                return ERROR;
              }

            g_stepper.delay_ms = (uint32_t)value;
            index += nargs;
            break;

          /* Blocking / non-blocking mode */

          case 'b':
            nargs = arg_decimal(&argv[index], &value);
            if (nargs == 0 || (value < 0 || value > 1))
              {
                printf("'b' out of range, nargs:%d, value:%ld\n", nargs, value);
                return ERROR;
              }

            g_stepper.blocking = (uint8_t)value;
            index += nargs;
            break;

          /* Loop n-times */

          case 'l':
            nargs = arg_decimal(&argv[index], &value);
            if (nargs == 0 || value < 1)
              {
                printf("'l' out of range, nargs:%d, value:%ld\n", nargs, value);
                return ERROR;
              }

            g_stepper.loops = (uint32_t)value;
            index += nargs;
            break;

          /* Reset position index */

          case 'i':
            nargs = arg_decimal(&argv[index], &value);
            if (nargs == 0 || (value < 0 || value > 1))
              {
                printf("'i' out of range, nargs:%d, value:%ld\n", nargs, value);
                return ERROR;
              }

            g_stepper.reset_index = (bool)value;
            index += nargs;
            break;

          default:
            printf("Unsupported option: %s\n", ptr);
            stepper_help();
            exit(1);
        }
    }

    return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stepper_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int stepper_main(int argc, char *argv[])
#endif
{
  int fd, ret;
  struct stepper_info_s info;
  char devname[16];

  /* Parse the command line */

  if (parse_args(argc, argv) != OK)
    {
      return ERROR;
    }

  printf("motor:%u, freq:%u, steps:%d (%s), loops:%u, delay:%ums, blocking:%u, "
    "reset_index:%u\n", g_stepper.motor, g_stepper.freq, g_stepper.steps,
    g_stepper.rel_pos? "rel" : "abs", g_stepper.loops, g_stepper.delay_ms,
    g_stepper.blocking, g_stepper.reset_index);

  /* Open device */

  (void)snprintf(devname, 16, "/dev/mc%d", g_stepper.motor);

  fd = open(devname, O_RDONLY | (g_stepper.blocking? 0 : O_NONBLOCK));
  if (fd < 0)
    {
      printf("open: %s failed: %d\n", devname, errno);
      return ERROR;
    }
  else
    {
      printf("succesfully opened: %s\n", devname);
    }

  /* Enable motor and drive with given params */

  info.frequency = g_stepper.freq;
  info.position = g_stepper.steps;
  info.pos_type = g_stepper.rel_pos? POS_RELATIVE : POS_ABSOLUTE;

  while (g_stepper.loops--)
    {
      ret = ioctl(fd, MCIOC_SET_POS, (unsigned long)((uintptr_t)&info));
      if (ret < 0)
        {
          printf("ioctl(MCIOC_SET_POS) failed: %d\n", errno);
        }

      if (g_stepper.delay_ms)
        {
          usleep(g_stepper.delay_ms * 1000);
        }
    }

  /* Wait until steps are completed */

  if (!g_stepper.blocking)
    {
      struct stepper_status_s status;

      do {
        usleep(500 * 1000);

        ret = ioctl(fd, MCIOC_GET_STATUS, (unsigned long)((uintptr_t)&status));
        if (ret < 0)
          {
            printf("ioctl(MCIOC_GET_STATUS) failed: %d\n", errno);
            goto err;
          }
      } while(status.current_pos != status.target_pos);

      printf("Stepping completed - target:%d, current:%d, pattern index:%u\n",
        status.target_pos, status.current_pos, status.step_index);
    }

  if (g_stepper.reset_index)
    {
      ret = ioctl(fd, MCIOC_RESET_INDEX, 0);
      if (ret < 0)
        {
          printf("ioctl(MCIOC_RESET_INDEX) failed: %d\n", errno);
        }
    }

err:
  fflush(stdout);
  close(fd);
  return OK;
}
