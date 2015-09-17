/****************************************************************************
 * apps/system/ubgps/gps_aiding.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Authors: Harri Luhtala <harri.luhtala@haltian.com>
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
#include <assert.h>
#include <string.h>
#include <debug.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <apps/system/conman.h>
#include <apps/netutils/dnsclient.h>

#include "ubgps_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SYSTEM_UBGPS_DEBUG
 #define aid_dbg(...) dbg(__VA_ARGS__)
#else
  #define aid_dbg(...)
#endif

/* Maximum retry attempt and delay for conman connection */

#define CONMAN_STATUS_RETRY   10
#define CONMAN_RETRY_DELAY    2

/* Maximum times socket are allowed to block */

#define AIDING_SOCKET_SEND_TIMEOUT    20 /* secs */
#define AIDING_SOCKET_RECV_TIMEOUT    30 /* secs */

/* AlmanacPlus data on sdcard */

#define ALP_FOLDER      "/media/GPS"
#define ALP_FILE        "info.txt"
#define ALP_FILE_PATH   ALP_FOLDER "/" ALP_FILE

/* u-blox AlmanacPlus files */

#define ALP_1D_NAME     "current_1d.alp"
#define ALP_2D_NAME     "current_2d.alp"
#define ALP_3D_NAME     "current_3d.alp"
#define ALP_5D_NAME     "current_5d.alp"
#define ALP_7D_NAME     "current_7d.alp"
#define ALP_10D_NAME    "current_10d.alp"
#define ALP_14D_NAME    "current_14d.alp"

/* u-blox assistance data server */

#define ALP_SERVER      "alp.u-blox.com"
#define ALP_SERVER_PORT 80

#define ALP_REQ_LINE    "GET " "/" ALP_1D_NAME " HTTP/1.1" "\r\n"
#define ALP_REQ_UA      "User-Agent: TS" "\r\n"
#define ALP_REQ_HOST    "Host: alp.u-blox.com" "\r\n"
#define ALP_REQ_END     "\r\n"

#define ALP_REQUEST     ALP_REQ_LINE ALP_REQ_UA ALP_REQ_HOST ALP_REQ_END

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum http_parser_state_e
{
  HTTP_HEADER_STATUS = 0,
  HTTP_HEADER_LEN,
  HTTP_BODY_SEPARATOR,
};
typedef enum http_parser_state_e http_parser_state_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Updater thread running state */

static volatile bool thread_running = false;

static pthread_t aid_thread;

static const char alp_uri[] = ALP_REQUEST;

static const char alp_server[] = ALP_SERVER;

struct assistance_updater_s {
  http_parser_state_t state;
  bool header_received;
  size_t content_len;
  size_t write_cnt;
  FILE* fd;
};

static struct assistance_updater_s updater;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: get_server_address
 *
 * Description:
 *   resolves ip-address for aiding data server
 *
 * Input Parameters:
 *   addr             - address structure to be filled
 *   hostname         - hostname to be resolved
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

static int get_server_address(struct sockaddr_in *addr, const char *hostname)
{
  int ret;

  DEBUGASSERT(addr && hostname);

  aid_dbg("Getting IP address for '%s'\n", hostname);

  ret = dns_gethostip(hostname, &addr->sin_addr.s_addr);
  if (ret != OK)
    {
      aid_dbg("Failed to get address for '%s' errno:%d\n", hostname, errno);
      return ret;
    }

  addr->sin_family = AF_INET;

  aid_dbg("Got address %d.%d.%d.%d for '%s'\n",
      addr->sin_addr.s_addr & 0xff,
      (addr->sin_addr.s_addr >> 8) & 0xff,
      (addr->sin_addr.s_addr >> 16) & 0xff,
      addr->sin_addr.s_addr >> 24,
      hostname);
  return OK;
}

/****************************************************************************
 * Name: http_parser_reset
 *
 * Description:
 *   reset http parser to its initial state
 *
 * Input Parameters:
 *
 ****************************************************************************/

static void http_parser_reset(void)
{
  updater.state = HTTP_HEADER_STATUS;
  updater.header_received = false;
  updater.content_len = 0;
  updater.write_cnt = 0;
  if (updater.fd)
    {
      fclose(updater.fd);
      updater.fd = NULL;
    }
}

/****************************************************************************
 * Name: memmem
 *
 * Description:
 *   Find matching sub-memory buffer from larger memory buffer.
 *
 ****************************************************************************/

static const void *memmem(const void *buf, size_t buflen,
                          const void *sub, size_t sublen)
{
  const char *cbuf = buf;

  while (buflen >= sublen)
    {
      if (memcmp(cbuf, sub, sublen) == 0)
        return cbuf;

      cbuf++;
      buflen--;
    }

  return NULL;
}

/****************************************************************************
 * Name: http_parse_response
 *
 * Description:
 *   http response parser
 *
 * Input Parameters:
 *   inbuf         - http response data buffer
 *   len           - number of bytes in the buffer. The len value is updated
 *                   with remaining byte count.
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

static int http_parse_response(char *inbuf, size_t *len,
                               struct gps_assistance_s * const assist)
{
  int status;

  if (!*len || !inbuf)
    {
      return OK;
    }

  if (!updater.header_received)
    {
      const char *pInbuf = inbuf;
      static const char substr [] = "\r\n";
      const size_t substr_len = sizeof(substr) - 1;
      const char *pch;
      size_t currlen = *len;

      aid_dbg("'%.*s'\n", currlen, inbuf);

      while( (pch=memmem(pInbuf, currlen, substr, substr_len)) )
        {
          switch (updater.state)
          {
            case HTTP_HEADER_STATUS:
              {
                static const char resp_ok[] = "HTTP/1.1 200 OK";
                const size_t resp_ok_len = sizeof(resp_ok_len) - 1;

                aid_dbg("HTTP_HEADER_STATUS\n");

                if (strncmp(pInbuf, resp_ok, resp_ok_len) == 0)
                  {
                    updater.state++;
                  }
                break;
              }

            case HTTP_HEADER_LEN:
              {
                static const char resp_content_len[] = "Content-Length: ";
                const size_t resp_content_len_len = sizeof(resp_content_len) - 1;

                aid_dbg("HTTP_HEADER_LEN\n");

                if (!strncmp(resp_content_len, pInbuf, resp_content_len_len))
                  {
                    updater.content_len = atoi(pInbuf + resp_content_len_len);
                    aid_dbg("content_len: %d\n", updater.content_len);
                    updater.state++;
                  }
                break;
              }

            case HTTP_BODY_SEPARATOR:
              {
                const char *resp_separator = substr;
                const size_t resp_separator_len = substr_len;

                aid_dbg("HTTP_BODY_SEPARATOR\n");

                if(!strncmp(resp_separator, pInbuf, resp_separator_len))
                  {
                    aid_dbg("header_received\n");
                    updater.header_received=true;
                  }
                break;
              }

            default:
                break;
          }

          pInbuf = pch + substr_len;
          currlen = *len - (pInbuf - inbuf);

          if (updater.header_received)
            {
              aid_dbg("HTTP header done.\n");
              break;
            }
        }

      /* Adjust remaining length according to handled bytes */

      *len -= (pInbuf - inbuf);

      /* Move remaining bytes at the beginning of the buffer */

      memmove(inbuf, pInbuf, *len);

      status = OK;
    }

  /* Receive aiding data */

  if (updater.header_received && *len > 0)
    {
      /* Receive aiding data */

      if (updater.fd == NULL)
        {
          updater.write_cnt = 0;
          updater.fd = fopen(ALP_FILE_PATH, "w");
          if (!updater.fd)
            {
              aid_dbg("file open failed, errno:%d\n", errno);
              return ERROR;
            }
        }

      status = fwrite(inbuf, 1, *len, updater.fd);
      if (status)
        {
          *len -= status;
          updater.write_cnt += status;
          status = OK;
        }
      else
        {
          fclose(updater.fd);
          status = ERROR;
        }

      if (updater.write_cnt >= updater.content_len)
        {
          fclose(updater.fd);

          if (!ubgps_check_alp_file_validity(ALP_FILE_PATH))
            {
              /* Received data is invalid. */
              unlink(ALP_FILE_PATH);

              assist->alp_file_id++;
              status = ERROR;
            }
          else
            {
              /* Aiding data ready to be taken in use, file_id should differ from
                 the previous one to notify aiding client about updated data */

              if (!assist->alp_file)
                {
                  /* file does not exist */

                  assist->alp_file = strdup(ALP_FILE_PATH);
                }

              assist->alp_file_id++;
              status = OK;
            }
        }
    }

  return status;
}

/****************************************************************************
 * Name: close_conman_connection
 *
 * Description:
 *   Closes conman connection
 *
 * Input Parameters:
 *
 *
 ****************************************************************************/

static void close_conman_connection(int *sock, uint32_t *connid)
{
  struct conman_client_s conman_client;

  if (*sock >= 0)
    {
      close(*sock);
      *sock = -1;
    }

  if (conman_client_init(&conman_client))
    {
      aid_dbg("client init failed\n");
      return;
    }

  if (*connid >= CONMAN_CONNID_MIN && *connid <= CONMAN_CONNID_MAX)
    {
      if (conman_client_destroy_connection(&conman_client, *connid))
        {
          aid_dbg("conman_client_destroy_connection failed\n");
        }
    }

  *connid = CONMAN_CONNID_CLEAR;

  (void)conman_client_uninit(&conman_client);
}

/****************************************************************************
 * Name: open_conman_connection
 *
 * Description:
 *   Opens conman connection
 *
 * Input Parameters:
 *   param         - conman_client_s
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

static int open_conman_connection(int *sock, uint32_t *connid)
{
  struct conman_client_s conman_client;
  struct conman_status_s conman_status;
  uint8_t retry = CONMAN_STATUS_RETRY;
  int ret = ERROR;

  *connid = CONMAN_CONNID_CLEAR;

  if (conman_client_init(&conman_client))
    {
      aid_dbg("conman init failed\n");
      return ERROR;
    }

  if (conman_client_request_connection(&conman_client, CONMAN_DATA, connid))
    {
      aid_dbg("failed to connect!\n");
      goto out_uninit;
    }

  while(retry--)
    {
      if (conman_client_get_connection_status(&conman_client, &conman_status))
        {
          aid_dbg("conman connection status failed\n");
        }
      else
        {
          if (conman_status.status == CONMAN_STATUS_ESTABLISHED)
            {
              *sock = socket(AF_INET, SOCK_STREAM, 0);
              if (*sock < 0)
                {
                  aid_dbg("socket open failed errno:%d\n", errno);
                }
              else
                {
                  aid_dbg("socket successfully opened\n");
                  ret = OK;
                  goto out_uninit;
                }
            }
        }

      /* retry connection after sleep */
      sleep(CONMAN_RETRY_DELAY);
    }

  (void)conman_client_uninit(&conman_client);
  (void)close_conman_connection(sock, connid);
  return ERROR;

out_uninit:
  (void)conman_client_uninit(&conman_client);
  return ret;
}

/****************************************************************************
 * Name: ubgps_aid_thread
 *
 * Description:
 *   Aiding thread
 *
 * Input Parameters:
 *   param         - GPS assistance structure
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

static void *ubgps_aid_thread(void *param)
{
  struct gps_assistance_s * assist = (struct gps_assistance_s *)param;
  int sock = -1;
  uint32_t connid = CONMAN_CONNID_CLEAR;
  int ret;
  const size_t inbuf_size = 768;
  char *inbuf = NULL;
  char *pBuf;
  size_t remaining_bytes = 0;
  size_t len;
  uint16_t file_id;
  struct timeval tv;

  aid_dbg("->\n");

  inbuf = malloc(inbuf_size);
  if (!inbuf)
    {
      aid_dbg("malloc(%d) failed\n", inbuf_size);
      return NULL;
    }

  /* Establish conman connection */

  if (open_conman_connection(&sock, &connid))
    {
      free(inbuf);
      return NULL;
    }

  if (assist->alp_srv_addr.sin_addr.s_addr == 0)
    {
      /* resolve u-blox ALP server IP address */

      if (get_server_address(&assist->alp_srv_addr, alp_server) != OK)
        {
          aid_dbg("DNS request failed\n");
          goto error;
        }
    }

  /* Open HTTP connection to server */

  aid_dbg("Connect to port %d\n", ALP_SERVER_PORT);
  assist->alp_srv_addr.sin_port = htons(ALP_SERVER_PORT);
  ret = connect(sock, (struct sockaddr *)&assist->alp_srv_addr,
    sizeof(assist->alp_srv_addr));
  if (ret < 0)
    {
      aid_dbg("connect failed\n");
      goto error;
    }

  aid_dbg("send request\n");

  /* Set up a send timeout */
  tv.tv_sec = AIDING_SOCKET_SEND_TIMEOUT;
  tv.tv_usec = 0;
  ret = setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv,
                   sizeof(struct timeval));
  if (ret < 0)
    {
      aid_dbg("Setting SO_SNDTIMEO failed, errno: %d\n", errno);
    }
  else
    {
      aid_dbg("SO_SNDTIMEO := %d secs\n", AIDING_SOCKET_SEND_TIMEOUT);
    }

  pBuf = (char*)alp_uri;
  aid_dbg("'%s'\n", pBuf);
  len = sizeof(alp_uri);
  aid_dbg("len:%d\n", len);

  while (len)
    {
      ssize_t written;
      written = send(sock, pBuf, len, 0);
      aid_dbg("written:%d\n", written);
      if (written < 0)
        break;
      len -= written;
      pBuf += written;
    }

  /* Update aiding data */

  aid_dbg("handle response\n");

  /* Set up a receive timeout */
  tv.tv_sec = AIDING_SOCKET_RECV_TIMEOUT;
  tv.tv_usec = 0;
  ret = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv,
                   sizeof(struct timeval));
  if (ret < 0)
    {
      aid_dbg("Setting SO_RCVTIMEO failed, errno: %d\n", errno);
    }
  else
    {
      aid_dbg("SO_RCVTIMEO := %d secs\n", AIDING_SOCKET_RECV_TIMEOUT);
    }

  pBuf = inbuf;

  file_id = assist->alp_file_id;
  pthread_mutex_lock(&g_aid_mutex);
  do {
    ssize_t read;
    pthread_mutex_unlock(&g_aid_mutex);
    read = recv(sock, pBuf, inbuf_size - remaining_bytes, 0);
    pthread_mutex_lock(&g_aid_mutex);
    if (read < 0)
      {
        /* no bytes left */

        aid_dbg("recv: read:%d errno:%d\n", read, errno);
        break;
      }
    if (read == 0)
      {
        /* remote closed connection, no bytes left */

        aid_dbg("recv: read:%d\n", read);
        break;
      }

    remaining_bytes += read;

    if (http_parse_response(inbuf, &remaining_bytes, assist))
      {
        aid_dbg("http parser error\n");
        break;
      }

    if (remaining_bytes >= inbuf_size)
      {
        aid_dbg("parser error, remaining >= buf_size\n");
        break;
      }

    /* Check whether data is updated */

    if (file_id != assist->alp_file_id)
      {
        dbg("Aiding data updated, file_id: %d\n", assist->alp_file_id);

        break;
      }

    /* set receiving offset for buffer */

    pBuf = inbuf + remaining_bytes;

  } while(read > 0);

  /* reset parser variables */

  http_parser_reset();

  thread_running = false;
  pthread_mutex_unlock(&g_aid_mutex);

error:
  close_conman_connection(&sock, &connid);
  free(inbuf);

  aid_dbg("<-\n");
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ubgps_aid_updater_start
 *
 * Description:
 *   Start GPS aiding updater
 *
 * Input Parameters:
 *   assist         - assistance structure
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int ubgps_aid_updater_start(struct gps_assistance_s * const assist)
{
  int ret;
  struct stat st;
  pthread_attr_t attr;

  aid_dbg("->\n");

  DEBUGASSERT(assist);

  ret = pthread_mutex_trylock(&g_aid_mutex);
  if (ret < 0)
    {
      aid_dbg("mutex locked => updater already running\n");
      return OK;
    }

  if (thread_running)
    {
      aid_dbg("updater already running\n");
      pthread_mutex_unlock(&g_aid_mutex);
      return OK;
    }

  thread_running = true;
  pthread_mutex_unlock(&g_aid_mutex);

  /* Check that aiding data directory exists */

  ret = stat(ALP_FOLDER, &st);

  if (ret < 0)
    {
      ret = mkdir(ALP_FOLDER, 0666);
      if (ret < 0)
        {
          aid_dbg("unable to create aid data directory\n");
          goto error;
        }
    }

  pthread_attr_init(&attr);
  attr.stacksize = 1536;

  ret = pthread_create(&aid_thread,
    &attr,
    ubgps_aid_thread,
    (pthread_addr_t)assist);

  pthread_attr_destroy(&attr);
  if (!ret)
    {
      return OK;
    }

  aid_dbg("thread create failed: %d\n", ret);
error:
  thread_running = false;
  aid_dbg("<-\n");
  return ret;
}

/****************************************************************************
 * Name: ubgps_aid_updater_stop
 *
 * Description:
 *   Stop aiding updater
 *
 * Input Parameters:
 *   assist         - assistance structure
 *
 * Returned Values:
 *   Status
 *
 ****************************************************************************/

int ubgps_aid_updater_stop(struct gps_assistance_s * const assist)
{
  aid_dbg("->\n");

  if(!assist)
    return OK;

  pthread_join(aid_thread, NULL);

  aid_dbg("<-\n");
  return OK;
}

/****************************************************************************
 * Name: ubgps_aid_get_alp_filename
 ****************************************************************************/

const char *ubgps_aid_get_alp_filename(void)
{
  return ALP_FILE_PATH;
}
