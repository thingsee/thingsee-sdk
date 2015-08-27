/****************************************************************************
 * arch/sim/src/up_simstderr.c
 * Access STDERR of the simulator host process
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_stderr_write
 ****************************************************************************/

ssize_t sim_stderr_write(const char *buffer, size_t len)
{
  return write(2, buffer, len);
}

/* These are backdoor testing functions for automated test.
 * They are here, because they need to be linked against host libc.
 * Enabled when unity_dev_err application is compiled.
 */
#ifdef CONFIG_TESTS_UNITY_DEV_ERR
/****************************************************************************
 * Name: host_fs_mkstemp
 ****************************************************************************/
int host_fs_mkstemp(char* template)
{
  return mkstemp(template);
}

/****************************************************************************
 * Name: host_fs_open
 ****************************************************************************/
int host_fs_open(const char *pathname, int flags)
{
  return open(pathname, flags);
}

/****************************************************************************
 * Name: host_fs_reopen_stderr
 ****************************************************************************/
int host_fs_reopen_stderr(int fd)
{
  int old_stderr = dup(2);
  if (old_stderr >= 0)
    {
      if (dup2(fd, 2) == -1)
	{
	  close(old_stderr);
	  old_stderr = -1;
	}
    }
  return old_stderr;
}

/****************************************************************************
 * Name: host_fs_lseek
 ****************************************************************************/
off_t host_fs_lseek(int fd, off_t offset, int whence)
{
  return lseek(fd, offset, whence);
}

/****************************************************************************
 * Name: host_fs_read
 ****************************************************************************/
ssize_t host_fs_read(int fd, void *buf, size_t count)
{
  return read(fd, buf, count);
}

/****************************************************************************
 * Name: host_fs_close
 ****************************************************************************/
int host_fs_close(int fd)
{
  return close(fd);
}
#endif /* CONFIG_EXAMPLES_UNITY_DEV_ERR */
