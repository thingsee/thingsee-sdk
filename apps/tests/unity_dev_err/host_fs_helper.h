/****************************************************************************
 * apps/examples/unity_dev_err/host_fs_helper.h
 * Helper functions to work with simulator's host filesystem.
 * Actual implementation is conditionally compiled into
 * arch/sim/src/up_simstderr.c
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *   Author: Roman Saveljev <roman.saveljev@haltian.com>
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
#ifndef __EXAMPLES_UNITY_DEV_ERR_HOST_FS_HELPER_H
#define __EXAMPLES_UNITY_DEV_ERR_HOST_FS_HELPER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: host_fs_mkstemp
 *
 * Description:
 *   Calls mkstemp. See 'man 3 mkstemp'
 *
 * Input Parameters:
 *   template - file name template
 *
 * Returned Value:
 *   Open file descriptor or -1
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
int host_fs_mkstemp(char* template);

/****************************************************************************
 * Name: host_fs_open
 *
 * Description:
 *   Opens file in the host filesystem. See 'man 2 open'
 *
 * Input Parameters:
 *   name - file name
 *   flags - file flags
 *
 * Returned Value:
 *   File descriptor or -1
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
int host_fs_open(const char *pathname, int flags);

/****************************************************************************
 * Name: host_fs_reopen_stderr
 *
 * Description:
 *   Re-opens STDERR descriptor assigned to the simulator by the host
 *
 * Input Parameters:
 *   fd - new file descriptor
 *
 * Returned Value:
 *   Duplicated old STDERR descriptor or -1
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
int host_fs_reopen_stderr(int fd);

/****************************************************************************
 * Name: host_fs_lseek
 *
 * Description:
 *   Calls lseek implementation from host. See 'man 2 lseek'
 *
 * Input Parameters:
 *   fd - file descriptor to seek
 *   offset - offset
 *   int whence - SEEK_SET, SEEK_CUR or SEEK_END
 *
 * Returned Value:
 *   New offset from the beginning of the file or -1
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
off_t host_fs_lseek(int fd, off_t offset, int whence);

/****************************************************************************
 * Name: host_fs_read
 *
 * Description:
 *   Reads file descriptor using host function. See 'man 2 read'
 *
 * Input Parameters:
 *   fd - file descriptor to read
 *   buf - receiving buffer
 *   count - maximum number of bytes allowed to read
 *
 * Returned Value:
 *   Number of bytes retrieved or -1
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
ssize_t host_fs_read(int fd, void *buf, size_t count);

/****************************************************************************
 * Name: host_fs_close
 *
 * Description:
 *   Closes file descriptor using host function. See 'man 2 close'
 *
 * Input Parameters:
 *   fd - file descriptor to read
 *
 * Returned Value:
 *   0 on success or -1
 *
 * Assumptions/Limitations:
 *   None
 *
 ****************************************************************************/
int host_fs_close(int fd);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __EXAMPLES_UNITY_DEV_ERR_HOST_FS_HELPER_H */
