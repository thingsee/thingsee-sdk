/****************************************************************************
 * examples/ubgps_nmea/ubgps_nmea_main.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *    Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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
#include <stdbool.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <poll.h>
#include <debug.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <apps/system/ubgps.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t fullwrite(int fd, const void *buf, size_t writelen)
{
  const uint8_t *writebuf = buf;
  ssize_t nwritten;
  size_t total = 0;

  do
    {
      nwritten = write(fd, writebuf, writelen);
      if (nwritten == ERROR)
        {
          int error = get_errno();
          if (error != EAGAIN)
            {
              return ERROR;
            }
          nwritten = 0;
        }
      writebuf += nwritten;
      writelen -= nwritten;
      total += nwritten;
    }
  while (writelen > 0);

  return total;
}

static void gps_event_handler(void const * const e, void * const priv)
{
  struct gps_event_s const * const event = e;
  int outfd = (intptr_t)priv;

  if (event->id == GPS_EVENT_NMEA_DATA)
    {
      struct gps_event_nmea_data_s const * const nmea = e;

      if (nmea->line)
        {
          char *buf = NULL;
          ssize_t len;

          len = asprintf(&buf, "%s\n", nmea->line);

          if (len > 0)
            (void)fullwrite(outfd, buf, len);

          if (len >= 0)
            free(buf);
        }
    }
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * gpsnmea_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int gpsnmea(int argc, FAR char *argv[])
#else
int gpsnmea_main(int argc, char *argv[])
#endif
{
  const bool show_nmea = true;
  struct pollfd *pfds;
  struct ubgps_s *gps;
  int timeout_ms;
  int outfd = 1;
  int ret;

  if (argc >= 2)
    {
      /* Open output file */

      outfd = open(argv[1], O_WRONLY | O_CREAT);
      if (outfd < 0)
        {
          fprintf(stderr, "gpsnmea: Failed to open input file %s, errno: %d\n",
                  argv[1], errno);
          exit(1);
        }
    }

  /* Initialize gps library. */

  gps = ubgps_initialize();
  if (!gps)
    {
      fprintf(stderr, "gpsnmea: Failed to initialize.\n");
      exit(1);
    }

  /* Setup GPS event processing. */

  pfds = calloc(1, sizeof(*pfds));
  if (!pfds)
    {
      fprintf(stderr, "gpsnmea: Out-of-memory, tried to allocate %lld.\n",
             (long long int)1 * sizeof(*pfds));
      exit(1);
    }

  /* Register event handler (TODO: register function does not take 'gps' object
   * pointer). */

  ubgps_callback_register(GPS_EVENT_TARGET_STATE_NOT_REACHED |
                          GPS_EVENT_TARGET_STATE_REACHED |
                          GPS_EVENT_TARGET_STATE_TIMEOUT |
                          GPS_EVENT_STATE_CHANGE |
                          GPS_EVENT_NMEA_DATA |
                          GPS_EVENT_TIME |
                          GPS_EVENT_LOCATION,
                          gps_event_handler, (void *)(intptr_t)outfd);

  /* Enable NMEA output */

  ubgps_config(GPS_CONFIG_NMEA_DATA, &show_nmea);

  /* Get GPS fix. */

  ret = ubgps_request_state(GPS_STATE_FIX_ACQUIRED, 0);
  if (ret != 0)
    {
      fprintf(stderr, "gpsnmea: ts_gps_request_state failed.\n");
      exit(1);
    }

  /* Process GPS events. */

  do
    {
      /* Setup pollfds for GPS and get timeout for timers. */

      ret = ubgps_poll_setup(gps, pfds, &timeout_ms);
      if (ret < 0)
        {
          fprintf(stderr, "gpsnmea: ubgps_poll_setup failed, %d\n", ret);
          continue;
        }

      /* Wait for poll event or timeout. */

      ret = poll(pfds, 1, timeout_ms);
      if (ret < 0)
        {
          fprintf(stderr, "gpsnmea: poll failed, %d\n", ret);
          continue;
        }

      /* Handle events and/or timeout. */

      ret = ubgps_poll_event(gps, pfds);
      if (ret < 0)
        {
          fprintf(stderr, "gpsnmea: ubgps_poll_event failed, %d\n", ret);
          continue;
        }
    }
  while (1);

  /* Uninitialize GPS library. */

  /* TODO: ubgps_uninitialize(gps); */

  return 0;
}
