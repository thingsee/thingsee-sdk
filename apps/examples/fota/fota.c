
/****************************************************************************
 * apps/exmaples/fota/fota.c
 *
 *   Copyright (C) 2016 Haltian Ltd. All rights reserved.
 *   Author: Pekka Ervasti <pekka.ervasti@haltian.com>
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

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <string.h>
#include <sys/ioctl.h>
#include <poll.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mount.h>
#include <fcntl.h>
#include <netinet/in.h>

#include <arch/board/board-pwrctl.h>
#include <arch/board/board-reset.h>
#include <apps/netutils/dnsclient.h>
#include <apps/netutils/webclient.h>
#include <apps/system/conman.h>

#include "fota_dbg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MMC_MOUNT_PATH		"/media"
#define FOTA_PATH		MMC_MOUNT_PATH "/fota"
#define FOTA_CURRENT		FOTA_PATH "/current.txt"
#define FOTA_VERSION		FOTA_PATH "/version.txt"
#define FOTA_DOWNLOAD		FOTA_PATH "/nuttx.oci"
#define FOTA_UPDATE		"/media/update.oci"

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wget_callback
 *
 * Description:
 *   Callback for wget.  Stores the data to a local file.
 *
 * Input Parameters:
 *   **buffer - input data
 *   offset   - offset of data in the buffer
 *   datend   - end of data in the buffer
 *   *buflen  - length of buffer
 *   *arg     - private data
 *
 * Returned Value:
 *
 ****************************************************************************/

static void wget_callback(FAR char **buffer, int offset,
                          int datend, FAR int *buflen, FAR void *arg)
{
  int fd = *(int *)arg;
  int ret;

  fota_dbg("%s: offset: %d datend: %d\n", __func__, offset, datend);

  while (datend - offset > 0)
    {
      ret = write(fd, &((*buffer)[offset]), datend - offset);
      if (ret < 0)
        {
          fota_dbg("write failed, ret=%d, errno=%d\n", ret, errno);
          break;
        }

      offset += ret;
    }
}

/****************************************************************************
 * Name: request_internet_connection
 *
 * Description:
 *   Requests internet connection from the connection manager.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   ERROR on fails, or connection id
 *
 ****************************************************************************/

static int request_internet_connection(void)
{
  int ret;
  struct conman_client_s conman;
  struct conman_status_s status = { 0 };
  uint32_t connid;
  unsigned int retry = 10;

  ret = conman_client_init(&conman);
  if (ret < 0)
    {
      fota_dbg("conmand_client_init failed, ret=%d\n");
      return ERROR;
    }

  ret = conman_client_request_connection(&conman, CONMAN_DATA, &connid);
  if (ret < 0)
    {
      fota_dbg("conman_client_request_connection failed, ret=%d\n");
      goto free_conman;
    }

  do
    {
      ret = conman_client_get_connection_status(&conman, &status);
      if (ret < 0)
        {
          fota_dbg("conman_client_get_connection_status failed, ret=%d\n");
          goto free_conman;
        }

      if (status.status == CONMAN_STATUS_ESTABLISHED)
        {
          fota_dbg("Connection established\n");
          break;
        }

      fota_dbg("Waiting for internet connection\n");
      sleep(3);
    }
  while (status.status != CONMAN_STATUS_ESTABLISHED && retry--);

free_conman:
  conman_client_uninit(&conman);

  return (status.status != CONMAN_STATUS_ESTABLISHED) ? ERROR : connid;
}

/****************************************************************************
 * Name: release_internet_connection
 *
 * Description:
 *   Releases the internet connection.
 *
 * Input Parameters:
 *   connid - current connection id
 *
 * Returned Value:
 *   ERROR on fails
 *
 ****************************************************************************/

static int release_internet_connection(uint32_t connid)
{
  int ret;
  struct conman_client_s conman;
  struct conman_status_s status = { 0 };
  unsigned int retry = 10;

  ret = conman_client_init(&conman);
  if (ret < 0)
    {
      fota_dbg("conmand_client_init failed, ret=%d\n");
      return ERROR;
    }

  ret = conman_client_destroy_connection(&conman, connid);
  if (ret < 0)
    {
      fota_dbg("conman_client_destroy_connection failed\n");
      ret = ERROR;
    }

  do
    {
      ret = conman_client_get_connection_status(&conman, &status);
      if (ret < 0)
        {
          fota_dbg("conman_client_get_connection_status failed, ret=%d\n");
          goto free_conman;
        }

      if (status.status == CONMAN_STATUS_OFF)
        {
          fota_dbg("Connection shutdown\n");
          break;
        }

      fota_dbg("Waiting for internet connection to shutdown\n");
      sleep(3);
    }
  while (status.status != CONMAN_STATUS_OFF && retry--);

free_conman:
  conman_client_uninit(&conman);

  return ret;
}

/****************************************************************************
 * Name: wget_file
 *
 * Description:
 *   Receives a file from an URL and stores it to a local file.
 *
 * Input Parameters:
 *   url  - url to fetch the file from
 *   file - filename
 *
 * Returned Value:
 *   ERROR on fails
 *
 ****************************************************************************/

static int wget_file(const char *url, const char *file)
{
  int ret;
  char buf[512];
  int fd;

  fota_dbg("url: %s file: %s\n", url, file);

  /* open file for writing */

  fd = open(file, O_WRONLY | O_CREAT);
  if (fd < 0)
    {
      fota_dbg("open failed, file=%s errno=%d\n", file, errno);
      return ERROR;
    }

  /* receive file content */

  ret = wget(url, buf, sizeof(buf), wget_callback, &fd);
  if (ret < 0)
    {
      fota_dbg("wget failed, ret=%d\n", ret);
      ret = ERROR;
    }

  /* close file */

  close(fd);

  return ret;
}

/****************************************************************************
 * Name: read_version
 *
 * Description:
 *   Reads the version number from a file.
 *
 * Input Parameters:
 *   file - filename
 *
 * Returned Value:
 *   Version number
 *
 ****************************************************************************/

static int read_version(const char *file)
{
  int fd;
  int ret;
  char buf[64];
  int version;

  fd = open(file, O_RDONLY);
  if (fd < 0)
    {
      fota_dbg("open failed, file=%s errno=%d\n", file, errno);
      return ERROR;
    }

  ret = read(fd, buf, sizeof(buf));
  if (ret < 0)
    {
      fota_dbg("read failed, errno=%d\n");
      goto err;
    }

  ret = sscanf(buf, "Version: %d", &version);
  if (ret != 1)
    {
      fota_dbg("sscanf failed, errno=%d\n", errno);
      goto err;
    }

  close(fd);

  return version;

err:
  close(fd);

  return ERROR;
}

/****************************************************************************
 * Name: get_current_version
 *
 * Description:
 *   Returns the firmware version last updated by the FOTA.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Firmware version
 *
 ****************************************************************************/

static int get_current_version(void)
{
  int ret;

  ret = read_version(FOTA_CURRENT);

  /* no file -> version 0 */

  if (ret < 0)
    ret = 0;

  return ret;
}

/****************************************************************************
 * Name: get_server_address
 *
 * Description:
 *   Returns numeric IP address
 *
 * Input Parameters:
 *   *addr - pointer to store the numberic IP address
 *   *hostname -  host name to resolve
 *
 * Returned Value:
 *   ERROR on fails
 *
 ****************************************************************************/

static int get_server_address(struct sockaddr_in *addr, const char *hostname)
{
  int ret;

  DEBUGASSERT(addr && hostname);

  fota_dbg("Getting IP address for '%s'\n", hostname);

  ret = dns_gethostip(hostname, &addr->sin_addr.s_addr);
  if (ret != OK)
    {
      fota_dbg("Failed to get address for '%s'\n", hostname);
      return ret;
    }

  addr->sin_family = AF_INET;

  fota_dbg("Got address %d.%d.%d.%d for '%s'\n",
           addr->sin_addr.s_addr & 0xff,
           (addr->sin_addr.s_addr >> 8) & 0xff,
           (addr->sin_addr.s_addr >> 16) & 0xff,
           addr->sin_addr.s_addr >> 24, hostname);

  return OK;
}

/****************************************************************************
 * Name: generate_url
 *
 * Description:
 *   Generates URL to receive a file from.
 *
 * Input Parameters:
 *   file - filename
 *   url  - buffer to store the url
 *   url_max_len - maximum length for the buffer url
 *
 * Returned Value:
 *   ERROR on fails
 *
 ****************************************************************************/

static int generate_url(const char *file, char *url, size_t url_max_len)
{
  int ret;
  struct sockaddr_in addr;

  ret = get_server_address(&addr, CONFIG_SYSTEM_FOTA_HOST_ADDRESS);
  if (ret < 0)
    {
      fota_dbg("get_server_address failed, ret=%d\n", ret);
      return ERROR;
    }

  snprintf(url, url_max_len, "http://%d.%d.%d.%d/%s/%s",
           addr.sin_addr.s_addr & 0xff,
           (addr.sin_addr.s_addr >> 8) & 0xff,
           (addr.sin_addr.s_addr >> 16) & 0xff,
           addr.sin_addr.s_addr >> 24, CONFIG_SYSTEM_FOTA_HOST_PATH, file);

  return OK;
}

/****************************************************************************
 * Name: get_version_available
 *
 * Description:
 *   Receives the version.txt file from the server and then reads the version
 *   number in the file.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   version number
 *
 ****************************************************************************/

static int get_version_available(void)
{
  int ret;
  char url[128];

  ret = generate_url("version.txt", url, sizeof(url));
  if (ret < 0)
    {
      fota_dbg("generate_url failed, ret=%d\n");
      return ERROR;
    }

  ret = wget_file(url, FOTA_VERSION);
  if (ret < 0)
    {
      return ERROR;
    }

  return read_version(FOTA_VERSION);
}

/****************************************************************************
 * Name: update
 *
 * Description:
 *   Downloads an firmware update from the server and resets the device.
 *   Bootloader flashes the firmware.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int update(uint32_t connid)
{
  int ret;
  char url[128];

  ret = generate_url("nuttx.oci", url, sizeof(url));
  if (ret < 0)
    {
      fota_dbg("generate_url failed, ret=%d");
      return ERROR;
    }

  ret = wget_file(url, FOTA_DOWNLOAD);
  if (ret < 0)
    {
      fota_dbg("wget_file failed\n");
      goto err;
    }

  (void)unlink(FOTA_UPDATE);

  ret = rename(FOTA_DOWNLOAD, FOTA_UPDATE);
  if (ret < 0)
    {
      fota_dbg("rename failed, %s %s errno=%d\n",
               FOTA_DOWNLOAD, FOTA_UPDATE, errno);
      goto err;
    }

  (void)unlink(FOTA_CURRENT);

  ret = rename(FOTA_VERSION, FOTA_CURRENT);
  if (ret < 0)
    {
      fota_dbg("rename failed, %s %s errno=%d\n",
               FOTA_VERSION, FOTA_CURRENT, errno);
      goto err;
    }

  sleep(1);

  /* this should be handled gracefully at system level shutdown function or
   * command */

  ret = umount(MMC_MOUNT_PATH);
  if (ret < 0)
    {
      fota_dbg("Can't unmount disk, ret=%d, errno=%d\n", ret, errno);
    }

  board_pwrctl_put(PWRCTL_REGULATOR_SDCARD);

  (void)release_internet_connection(connid);

  fota_dbg("Resetting system\n");

  board_systemreset();

  /* never reached */

err:
  unlink(FOTA_DOWNLOAD);

  return ERROR;
}

/****************************************************************************
 * Name: check_versions
 *
 * Description:
 *   Checks current version vs version available on the fota server and
 *   initializes update process if needed.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void check_versions(void)
{
  int current_version;
  int new_version;
  uint32_t connid;
  int ret;

  /* check current version */

  current_version = get_current_version();
  if (current_version < 0)
    {
      fota_dbg("read_current_version failed\n");
      return;
    }

  /* request internet connection */

  ret = request_internet_connection();
  if (ret < 0)
    return;

  connid = ret;

  /* check new version */

  new_version = get_version_available();
  if (new_version < 0)
    {
      fota_dbg("get_version_available failed\n");
      return;
    }

  fota_dbg("Current version: %d\n", current_version);
  fota_dbg("New     version: %d\n", new_version);

  /* do we need to update */

  if (new_version > current_version)
    {
      fota_dbg("Trying to update\n");
      update(connid);
    }
  else
    {
      fota_dbg("No update\n");
    }

  (void)release_internet_connection(connid);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fota_main
 *
 * Description:
 *   fota app entry point
 *
 * Input Parameters:
 *   argc : number of arguments in the argv vector
 *   argv : arguments
 *
 * Returned Value:
 *   EXIT_FAILURE on errors
 *
 ****************************************************************************/

int fota_main(int argc, char **argv)
{
  (void)mkdir(FOTA_PATH, 0777);

  while (true)
    {
      fota_dbg("Sleeping %d hours\n", CONFIG_SYSTEM_FOTA_INTERVAL_HOURS);
      sleep(CONFIG_SYSTEM_FOTA_INTERVAL_HOURS * 60 * 60);

      fota_dbg("Checking versions\n");
      check_versions();
    }

  /* never reached */

  return EXIT_SUCCESS;
}
