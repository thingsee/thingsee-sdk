/****************************************************************************
 * include/nuttx/bits.h
 *
 *   Copyright (C) 2015 Jussi Kivilinna. All rights reserved.
 *   Author: Jussi Kivilinna <jussi.kivilinna@iki.fi>
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

#ifndef __INCLUDE_NUTTX_BITS_H
#define __INCLUDE_NUTTX_BITS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: clzu32
 *
 * Description:
 *   clzu32 counts leading zero bits of unsigned 32-bit integer
 ****************************************************************************/
#ifndef HAVE_BUILTIN_CLZ
unsigned int clzu32(uint32_t n);
#else
static inline int clzu32(uint32_t n)
{
  if (n == 0)
    {
      return 32;
    }

  if (sizeof(n) == sizeof(unsigned long))
    {
      return __builtin_clzl(n);
    }
  return __builtin_clz(n);
}
#endif

/****************************************************************************
 * Name: ctzu32
 *
 * Description:
 *   ctzu32 counts trailing zero bits of unsigned 32-bit integer.
 ****************************************************************************/
#ifndef HAVE_BUILTIN_CTZ
unsigned int ctzu32(uint32_t n);
#else
static inline int ctzu32(uint32_t n)
{
  if (n == 0)
    {
      return 32;
    }

  if (sizeof(n) == sizeof(unsigned long))
    {
      return __builtin_ctzl(n);
    }
  return __builtin_ctz(n);
}
#endif

#ifdef UINT64_MAX
/****************************************************************************
 * Name: clzu64
 *
 * Description:
 *   clzu64 counts leading zero bits of unsigned 64-bit integer
 ****************************************************************************/
#ifndef HAVE_BUILTIN_CLZ
unsigned int clzu64(uint64_t n);
#else
static inline int clzu64(uint64_t n)
{
  if (n == 0)
    {
      return 64;
    }

#ifdef CONFIG_HAVE_LONG_LONG
  if (sizeof(n) == sizeof(unsigned long long))
    {
      return __builtin_clzll(n);
    }
#endif

  return __builtin_clzl(n);
}
#endif

/****************************************************************************
 * Name: ctzu64
 *
 * Description:
 *   ctzu64 counts trailing zero bits of unsigned 64-bit integer
 ****************************************************************************/
#ifndef HAVE_BUILTIN_CTZ
unsigned int ctzu64(uint64_t n);
#else
static inline int ctzu64(uint64_t n)
{
  if (n == 0)
    {
      return 64;
    }

#ifdef CONFIG_HAVE_LONG_LONG
  if (sizeof(n) == sizeof(unsigned long long))
    {
      return __builtin_ctzll(n);
    }
#endif

  return __builtin_ctzl(n);
}
#endif
#endif /* UINT64_MAX */

/****************************************************************************
 * Name: clz
 *
 * Description:
 *   clz counts leading zero bits of unsigned integer
 ****************************************************************************/

static inline unsigned int clz(unsigned int n)
{
  if (sizeof(n) == sizeof(uint16_t))
    {
      return clzu32(n) - 16;
    }
  else
    {
      return clzu32(n);
    }
}

/****************************************************************************
 * Name: clzl
 *
 * Description:
 *   clzl counts leading zero bits of unsigned long integer
 ****************************************************************************/

static inline unsigned int clzl(unsigned long n)
{
#ifdef UINT64_MAX
  if (sizeof(n) == sizeof(uint64_t))
    {
      return clzu64(n);
    }
  else
#endif
    {
      return clzu32(n);
    }
}

/****************************************************************************
 * Name: clzll
 *
 * Description:
 *   clzll counts leading zero bits of unsigned long long integer
 ****************************************************************************/

#ifdef CONFIG_HAVE_LONG_LONG
static inline unsigned int clzll(unsigned long long n)
{
#ifdef UINT64_MAX
  if (sizeof(n) == sizeof(uint64_t))
    {
      return clzu64(n);
    }
  else
#endif
    {
      return clzu32(n);
    }
}
#endif

/****************************************************************************
 * Name: ctz
 *
 * Description:
 *   ctz counts trailing zero bits of unsigned integer.
 ****************************************************************************/

static inline unsigned int ctz(unsigned int n)
{
  if (sizeof(n) == sizeof(uint16_t))
    {
      unsigned int ret = ctzu32(n);
      if (ret > 16)
        return 16;
      return ret;
    }
  else
    {
      return ctzu32(n);
    }
}

/****************************************************************************
 * Name: ctzl
 *
 * Description:
 *   ctzl counts trailing zero bits of unsigned long integer.
 ****************************************************************************/

static inline unsigned int ctzl(unsigned long n)
{
#ifdef UINT64_MAX
  if (sizeof(n) == sizeof(uint64_t))
    {
      return ctzu64(n);
    }
  else
#endif
    {
      return ctzu32(n);
    }
}

/****************************************************************************
 * Name: ctzll
 *
 * Description:
 *   ctzll counts trailing zero bits of unsigned long long integer.
 ****************************************************************************/

#ifdef CONFIG_HAVE_LONG_LONG
static inline unsigned int ctzll(unsigned long long n)
{
#ifdef UINT64_MAX
  if (sizeof(n) == sizeof(uint64_t))
    {
      return ctzu64(n);
    }
  else
#endif
    {
      return ctzu32(n);
    }
}
#endif

#endif /* __INCLUDE_NUTTX_BITS_H */
