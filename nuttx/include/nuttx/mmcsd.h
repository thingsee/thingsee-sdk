/****************************************************************************
 * include/nuttx/mmcsd.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_MMCSD_H
#define __INCLUDE_NUTTX_MMCSD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: mmcsd_slotinitialize
 *
 * Description:
 *   Initialize one slot for operation using the MMC/SD interface
 *
 * Input Parameters:
 *   minor - The MMC/SD minor device number.  The MMC/SD device will be
 *     registered as /dev/mmcsdN where N is the minor number
 *   dev - And instance of an MMC/SD interface.  The MMC/SD hardware should
 *     be initialized and ready to use.
 *
 ****************************************************************************/

struct sdio_dev_s; /* See nuttx/sdio.h */
int mmcsd_slotinitialize(int minor, FAR struct sdio_dev_s *dev);

/****************************************************************************
 * Name: mmcsd_spislotinitialize
 *
 * Description:
 *   Initialize one slot for operation using the SPI MMC/SD interface
 *
 * Input Parameters:
 *   minor - The MMC/SD minor device number.  The MMC/SD device will be
 *     registered as /dev/mmcsdN where N is the minor number
 *   slotno - The slot number to use.  This is only meaningful for architectures
 *     that support multiple MMC/SD slots.  This value must be in the range
 *     {0, ..., CONFIG_MMCSD_NSLOTS}.
 *   spi - And instance of an SPI interface obtained by called
 *     up_spiinitialize() with the appropriate port number (see spi.h)
 *
 ****************************************************************************/

struct spi_dev_s; /* See nuttx/spi/spi.h */
int mmcsd_spislotinitialize(int minor, int slotno, FAR struct spi_dev_s *spi);

/****************************************************************************
 * Name: mmcsd_check_media
 ****************************************************************************/

EXTERN void mmcsd_check_media(int slotno, bool try_to_reinit);

/****************************************************************************
 * Name: mmcsd_get_slot_card_status
 ****************************************************************************/

EXTERN void mmcsd_get_slot_card_status(int slotno, uint8_t *status);

/****************************************************************************
 * Name: mmcsd_is_card_in_slot
 *
 * Description:
 *  Returns true if card is in slot. Otherwise, false
 *
 * Input Parameters:
 *  slotno - The slot number to use.
 *
 ****************************************************************************/

EXTERN bool mmcsd_is_card_in_slot(int slotno);

/****************************************************************************
 * Name: mmcsd_slot_pm_suspend
 *
 * Description:
 *   Mark MMC/SD device as suspended by power-management
 *
 * Input Parameters:
 *   slotno - The slot number to use.
 *
 ****************************************************************************/

EXTERN void mmcsd_slot_pm_suspend(int slotno);

/****************************************************************************
 * Name: mmcsd_slot_pm_resume
 *
 * Description:
 *   Resume MMC/SD device from power-management suspend
 *
 * Input Parameters:
 *   slotno - The slot number to use.
 *
 ****************************************************************************/

EXTERN void mmcsd_slot_pm_resume(int slotno);

/****************************************************************************
 * Name: mmcsd_slot_pm_allowed
 *
 * Description:
 *   Check if MMC/SD device is ready for power-management suspend
 *
 * Input Parameters:
 *   slotno - The slot number to use.
 *
 ****************************************************************************/

EXTERN bool mmcsd_slot_pm_allowed(int slotno);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_MMCSD_H */
