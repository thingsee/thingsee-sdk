/****************************************************************************
 * apps/ts_engine/control/control_main.c
 *
 * Copyright (C) 2014-2016 Haltian Ltd. All rights reserved.
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
 * Authors:
 *   Pekka Ervasti <pekka.ervasti@haltian.com>
 *
 ****************************************************************************/
#include <nuttx/config.h>

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../engine/alloc_dbg.h"
#include "../engine/eng_dbg.h"
#include "../engine/client.h"

extern char profile_jsn_tmp[];
extern char cloud_jsn_tmp[];

static void showusage(const char *progname, int exitcode)
{
  printf("USAGE: %s [-r|w|s|l|h]\n\n", progname);
  printf("Where:\n"
      "  -r : Read profile/cloud/device\n"
      "  -w : Write profile/cloud\n"
      "  -l : (Re)load profile\n"
      "  -s : Stop profile\n"
      "  -p : Pause profile\n"
      "  -c : Continue profile\n"
      "  -h : Show this information\n");

  exit(exitcode);
}

int ts_engine_control_main(int argc, char *argv[])
{
  int option;
  int ret;
  struct ts_engine_client client;
  enum ts_engine_client_cmd cmd = 0;

  if (argc == 1)
    {
      showusage(argv[0], EXIT_SUCCESS);
    }

  while ((option = getopt(argc, argv, "w:r:lspch")) != ERROR)
    {
      switch (option)
        {
        case 'r': /* read */
          {
            if (strcmp(optarg, "profile") == 0)
              {
                cmd = TS_ENGINE_READ_PROFILE;
              }
            else if (strcmp(optarg, "cloud") == 0)
              {
                cmd = TS_ENGINE_READ_CLOUD_PROPERTY;
              }
            else if (strcmp(optarg, "device") == 0)
              {
                cmd = TS_ENGINE_READ_DEVICE_PROPERTY;
              }
          }
        break;

        case 'w': /* write */
          {
            if (strcmp(optarg, "profile") == 0)
              {
                cmd = TS_ENGINE_WRITE_PROFILE;
              }
            else if (strcmp(optarg, "cloud") == 0)
              {
                cmd = TS_ENGINE_WRITE_CLOUD_PROPERTY;
              }
          }
        break;

        case 'l': /* (re)load */
          {
            cmd = TS_ENGINE_RELOAD_PROFILE;
          }
        break;

        case 's': /* stop */
          {
            cmd = TS_ENGINE_STOP;
          }
        break;

        case 'p': /* pause */
          {
            cmd = TS_ENGINE_PAUSE;
          }
        break;

        case 'c': /* continue */
          {
            cmd = TS_ENGINE_CONTINUE;
          }
        break;

        case 'h': /*  help */
          {
            showusage(argv[0], EXIT_SUCCESS);
          }
        break;

        default:
          {
            showusage(argv[0], EXIT_FAILURE);
          }
        break;

        }
    }

  if (!cmd)
    {
      showusage(argv[0], EXIT_FAILURE);
    }

  ret = ts_engine_client_init(&client);
  if (ret < 0)
    {
      eng_dbg("ts_engine_client_init failed\n");
      return EXIT_FAILURE;
    }

  switch (cmd)
    {
    case TS_ENGINE_READ_PROFILE:
      {
        ret = ts_engine_client_read_profile(&client);
        if (ret < 0)
          {
            printf("ts_engine_client_read_profile failed\n");
            goto errout_uninit;
          }

        printf(client.payload.data);
      }
    break;

    case TS_ENGINE_READ_CLOUD_PROPERTY:
      {
        ret = ts_engine_client_read_cloud_property(&client);
        if (ret < 0)
          {
            printf("ts_engine_client_read_cloud_property failed\n");
            goto errout_uninit;
          }

        printf(client.payload.data);
      }
    break;

    case TS_ENGINE_READ_DEVICE_PROPERTY:
      {
        ret = ts_engine_client_read_device_property(&client);
        if (ret < 0)
          {
            printf("ts_engine_client_read_device_property failed\n");
            goto errout_uninit;
          }

        printf(client.payload.data);
      }
    break;

    case TS_ENGINE_WRITE_PROFILE:
      {
        ret = ts_engine_client_write_profile(&client, profile_jsn_tmp,
            strlen(profile_jsn_tmp) + 1);
        if (ret < 0)
          {
            printf("ts_engine_client_write_profile failed\n");
            goto errout_uninit;
          }
      }
    break;

    case TS_ENGINE_WRITE_CLOUD_PROPERTY:
      {
        ret = ts_engine_client_write_cloud_property(&client, cloud_jsn_tmp,
            strlen(cloud_jsn_tmp) + 1);
        if (ret < 0)
          {
            printf("ts_engine_client_write_cloud_property failed\n");
            goto errout_uninit;
          }
      }
    break;

    case TS_ENGINE_RELOAD_PROFILE:
      {
        ret = ts_engine_client_reload_profile(&client);
        if (ret < 0)
          {
            printf("ts_engine_client_reload_profile failed\n");
            goto errout_uninit;
          }
      }
    break;

    case TS_ENGINE_STOP:
      {
        ret = ts_engine_client_stop(&client);
        if (ret < 0)
          {
            printf("ts_engine_client_reload_profile failed\n");
            goto errout_uninit;
          }
      }
    break;

    case TS_ENGINE_PAUSE:
      {
        ret = ts_engine_client_pause(&client);
        if (ret < 0)
          {
            printf("ts_engine_client_pause failed\n");
            goto errout_uninit;
          }
      }
    break;

    case TS_ENGINE_CONTINUE:
      {
        ret = ts_engine_client_continue(&client);
        if (ret < 0)
          {
            printf("ts_engine_client_reload_profile failed\n");
            goto errout_uninit;
          }
      }
    break;

    case TS_ENGINE_PING:
      {
        ret = ts_engine_client_ping(&client);
        if (ret < 0)
          {
            printf("ts_engine_client_pause failed\n");
            goto errout_uninit;
          }
      }
    break;

    default:
      break;
    }

  ts_engine_client_uninit(&client);

  return OK;

  errout_uninit:

  ts_engine_client_uninit(&client);

  return ERROR;
}
