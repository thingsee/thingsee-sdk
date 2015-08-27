/****************************************************************************
 * apps/thingsee/emmc/emmc.c
 *
 *   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
 *   Authors: Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
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
#include <sys/stat.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/mkfatfs.h>
#include <nuttx/usb/usbmsc.h>
#include <nuttx/mmcsd.h>
#include <sys/mount.h>
#include <nuttx/fs/dirent.h>

#include <apps/thingsee/modules/ts_emmc.h>
#include <apps/thingsee/ts_core.h>

#include <arch/board/board.h>
#include <arch/board/board-reset.h>
#include <arch/board/board-pwrctl.h>

#ifdef CONFIG_SYSLOG_RUNSTOP_DYNAMIC
#  include <nuttx/syslog/syslog.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifdef CONFIG_THINGSEE_EMMC_MODULE_DBG
#  define emmc_dbg(x, ...)    lldbg(x, ##__VA_ARGS__)
#else
#  define emmc_dbg(x, ...)
#endif

#define TO_STRING(d)         #d

#define SOURCE_URL           "/dev/mmcsd"
#define GET_SOURCE_URL(x)    SOURCE_URL TO_STRING(x)

#define FILE_SYSTEM_TYPE     "vfat"

#define LOG_DIR              "/sensors"
#define MAKE_LOG_DIR         TS_EMMC_MOUNT_PATH LOG_DIR

/* Backup firmware filename is BACKUP.OCI for bootloader 0.4.0 and later. */

#define FIRMWARE_BACKUP_FILE TS_EMMC_MOUNT_PATH "/backup.oci"

/****************************************************************************
 * Private Types
 ****************************************************************************/

static struct fat_format_s g_emmc_fmt = {
  .ff_nfats = 1,
  .ff_fattype = 32,             /* FAT32 */
  .ff_clustshift = 0xFF,
  .ff_volumelabel = "Thingsee",
  .ff_backupboot = 0,
  .ff_rootdirentries = 0,
  .ff_rsvdseccount = 0,
  .ff_hidsec = 0,
  .ff_volumeid = 0,
  .ff_nsectors = 0
};

/************************************************************************************
 * Private data
 ************************************************************************************/

#ifdef CONFIG_USBMSC
static void *g_mschandle = NULL;
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static void emmc_cleanup_firmware_update(void)
{
  struct stat buf;

  if (stat(FIRMWARE_BACKUP_FILE, &buf) == OK)
    {
      /* Firmware backup file exists, remove it to prevent reflashing of old
       * firmware on reset. */

      printf("Firmware backup file %s exists. Removing...\n",
             FIRMWARE_BACKUP_FILE);

      unlink(FIRMWARE_BACKUP_FILE);
    }
}

static int emmc_try_to_mount(void)
{
  int ret;
  int err_tmp = 0;

  ret = mount(GET_SOURCE_URL(CONFIG_BOARD_MMCSDMINOR),
              TS_EMMC_MOUNT_PATH, FILE_SYSTEM_TYPE, 0, NULL);

  if (!ret)
    {
      emmc_cleanup_firmware_update();

      return OK;
    }

  if (errno == EINVAL)
    {
      emmc_dbg("Error: %d. Formatting...\n", errno);
      ret = mkfatfs(GET_SOURCE_URL(CONFIG_BOARD_MMCSDMINOR), &g_emmc_fmt);

      if (ret < 0)
        {
          perror("Can't format");
          return ret;
        }

      ret = mount(GET_SOURCE_URL(CONFIG_BOARD_MMCSDMINOR),
                  TS_EMMC_MOUNT_PATH, FILE_SYSTEM_TYPE, 0, NULL);

    }
  else
    {
      err_tmp = errno;
      if (err_tmp != ENODEV)
        {
          perror("mount");
        }
      else
        {
          emmc_dbg("No device found. Trying reinitialize reader...\n");
        }
      return -err_tmp;
    }

  return ret;
}

static int emmc_check_if_already_mounted(void)
{
  DIR *dir = NULL;

  dir = opendir(TS_EMMC_MOUNT_PATH);
  if (!dir)
    {
      return ERROR;
    }
  else
    {
      closedir(dir);
    }

  return OK;
}

static int emmc_first_time_connected(void)
{
  int ret = 0;

  if (emmc_check_if_already_mounted())
    {
      ret = emmc_try_to_mount();
    }

  if (ret < 0)
    {
      perror("Can't mount after formating");
    }

  return ret;
}

static int emmc_create_log_dir(void)
{
  int ret;

  ret = mkdir(MAKE_LOG_DIR, 0666);
  if (ret < 0)
    {
      perror("Can't create a dir");
    }

  return ret;
}

static int emmc_try_to_create_log_dir(void)
{
  int ret = OK;
  DIR *dir = NULL;

  dir = opendir(MAKE_LOG_DIR);
  if (dir)
    {
      closedir(dir);
    }
  else if (errno == ENOENT || errno == ENOTDIR)
    {
      ret = emmc_create_log_dir();
    }
  else
    {
      perror("Directory cannot be accessed");
      ret = ERROR;
    }

  return ret;
}

static int emmc_no_dev_found_reinit(void)
{
  mmcsd_check_media(CONFIG_BOARD_MMCSDSLOTNO, 0);
  mmcsd_check_media(CONFIG_BOARD_MMCSDSLOTNO, 1);

  return emmc_try_to_mount();
}

static int emmc_unmount(void)
{
  int ret;
  DIR *dir = NULL;

  dir = opendir(TS_EMMC_MOUNT_PATH);
  if (dir)
    {
      closedir(dir);
    }
  else
    {
      emmc_dbg("Can't open target dir %s. eMMC was removed\n",
               TS_EMMC_MOUNT_PATH);
      mmcsd_check_media(CONFIG_BOARD_MMCSDSLOTNO, 0);
    }

  ret = umount(TS_EMMC_MOUNT_PATH);
  if (ret < 0)
    {
      emmc_dbg("Can't unmount disk\n");
      return ret;
    }

  return OK;
}

static int emmc_remount(void)
{
  int ret;
  const uint8_t no_card_not_ready_mask = 0x3;
  uint8_t emmc_slot_card_mask = 0;

  mmcsd_get_slot_card_status(CONFIG_BOARD_MMCSDSLOTNO, &emmc_slot_card_mask);
  if (emmc_slot_card_mask & no_card_not_ready_mask)
    {
      emmc_dbg("Can't open target dir %s. eMMC was previously removed. Reinitializing...\n",
         TS_EMMC_MOUNT_PATH);
      mmcsd_check_media(CONFIG_BOARD_MMCSDSLOTNO, 1);
    }

  ret = emmc_try_to_mount();

  if (ret == -ENODEV)
    {
      ret = emmc_no_dev_found_reinit();
      if (ret < 0)
        {
          emmc_dbg("Card is still not in its place. "
                   "If it was inserted, be sure it was properly installed\n");
          return ret;
        }
    }
  else if (ret < 0)
    {
      perror("Can't mount disk");
      return ret;
    }

  return OK;
}

#ifdef CONFIG_USBMSC
static FAR void *emmc_msconn_init(void)
{
  FAR void *handle;
  int ret;

  emmc_dbg("configuring with NLUNS=%d\n", 1);
  ret = usbmsc_configure(1, &handle);
  if (ret < 0)
    {
      emmc_dbg("mcsonn_main: usbmsc_configure failed: %d\n", -ret);
      usbmsc_uninitialize(handle);
      return NULL;
    }

  emmc_dbg("handle=%p\n", handle);

  emmc_dbg("Bind LUN=0 to %s\n", GET_SOURCE_URL(CONFIG_BOARD_MMCSDMINOR));
  ret = usbmsc_bindlun(handle, GET_SOURCE_URL(CONFIG_BOARD_MMCSDMINOR), 0, 0, 0, false);
  if (ret < 0)
    {
      emmc_dbg("usbmsc_bindlun failed for LUN 1 using %s: %d\n",
               GET_SOURCE_URL(CONFIG_BOARD_MMCSDMINOR), -ret);
      usbmsc_uninitialize(handle);
      return NULL;
    }

  ret = usbmsc_exportluns(handle);
  if (ret < 0)
    {
      emmc_dbg("mcsonn_main: usbmsc_exportluns failed: %d\n", -ret);
      usbmsc_uninitialize(handle);
      return NULL;
    }

  return handle;
}

static bool emmc_usbmsc_deepsleep_hook(void *const priv)
{
  /* Prevent deep-sleep when USB composite device is active. */

  if (g_mschandle == NULL)
    return true;                /* Allow deep-sleep. */

  return false;
}


#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

int emmc_handle_power_off(bool force)
{
  int ret;

#ifdef CONFIG_SYSLOG_RUNSTOP_DYNAMIC
  syslog_runstop(0);
#endif

  ret = emmc_unmount();
  if (ret < 0)
    {
      /* If unmount fails because card was never mounted to begin with, or we
       * are forcing unmount, then allow power off. */

      if (ret != -ENOENT && !force)
        {
          return ret;
        }
    }

  board_pwrctl_put(PWRCTL_REGULATOR_SDCARD);

  return OK;
}

int emmc_handle_power_on_second_time(void)
{
  int ret;

  board_pwrctl_get(PWRCTL_REGULATOR_SDCARD);

  ret = emmc_remount();
  if (ret >= 0)
    {
#ifdef CONFIG_SYSLOG_RUNSTOP_DYNAMIC
      syslog_runstop(1);
#endif
    }

  return ret;
}

int emmc_init_module(void)
{
  int ret = OK;

  ret = emmc_first_time_connected();
  if (!ret)
    {
      ret = emmc_try_to_create_log_dir();
    }

#if defined(CONFIG_USBMSC)
  ret = ts_core_deepsleep_hook_add(emmc_usbmsc_deepsleep_hook, NULL);
  DEBUGASSERT(ret == OK);
#endif

  return ret;
}

const char *emmc_get_log_dir(void)
{
  return MAKE_LOG_DIR;
}

void emmc_switch_to_usbmsc_mode(void)
{
#ifdef CONFIG_USBMSC

  emmc_dbg("PC USB connect (msc:%p)\n", g_mschandle);

#ifdef CONFIG_SYSLOG_RUNSTOP_DYNAMIC
  syslog_runstop(0);
#endif

  if (!g_mschandle)
    {
      /* Release SDcard. */

      emmc_unmount();

      /* Enable USB MSC device. */

      g_mschandle = emmc_msconn_init();
      if (!g_mschandle)
        {
          /* Restore SDcard. */

          emmc_remount();

#ifdef CONFIG_SYSLOG_RUNSTOP_DYNAMIC
          syslog_runstop(1);
#endif
        }
    }
#endif
}

void emmc_switch_to_filesystem_mode(void (*system_reset)(void))
{
#ifdef CONFIG_USBMSC
  emmc_dbg("USB disconnect (msc:%p)\n", g_mschandle);

  if (g_mschandle)
    {
      /* Turn-off USB MSC device. */

      usbmsc_uninitialize(g_mschandle);

#if 1
      /* Restore functionality (mount) through reset. */

      system_reset();
#else
      /* Restore SDcard. */

      emmc_remount();

#ifdef CONFIG_SYSLOG_RUNSTOP_DYNAMIC
      syslog_runstop(1);
#endif

#endif
    }

  g_mschandle = NULL;
#endif
}
