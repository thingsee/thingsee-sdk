/****************************************************************************
 * apps/system/ubmodem/tests/sms_util_test.cc
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
 **************************************************************************/

#include <limits.h>
#include "gtest/gtest.h"
extern "C" {
#include "ubmodem_pdu_util.h"
}

class DecodeUTF8 : public testing::Test
{
protected:
  void LonelyStartsTest(unsigned int numlone, int firstlone);
  void AssertQuotedInvalidCodes(const char *input);

protected:
  virtual void SetUp()
  {
  }
  virtual void TearDown()
  {
  }
};

TEST_F(DecodeUTF8, EmptyInbuf)
{
  size_t offset = 0;
  const char *inbuf = "";
  size_t buflen = strlen(inbuf);
  int codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(0, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, OneASCII)
{
  size_t offset = 0;
  const char *inbuf = "a";
  size_t buflen = strlen(inbuf);
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ('a', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, TwoASCII)
{
  size_t offset = 0;
  const char *inbuf = "ab";
  size_t buflen = strlen(inbuf);
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ('a', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ('b', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, TenASCII)
{
  size_t offset = 0;
  const char *inbuf = "0123456789";
  size_t buflen = strlen(inbuf);
  size_t i;
  int codep;

  for (i = 0; i < buflen; i++) {
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(i + 1, offset);
    ASSERT_EQ('0' + i, codep);
  }
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(10, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, TwoByteCodepoint)
{
  size_t offset = 0;
  const char *inbuf = "\xc3\x87"; /* Ç */
  size_t buflen = strlen(inbuf);
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(0x00C7, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(1, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, ThreeByteCodepoint)
{
  size_t offset = 0;
  const char *inbuf = "\xe1\xb8\x88"; /* Ḉ */
  size_t buflen = strlen(inbuf);
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(3, offset);
  ASSERT_EQ(0x1E08, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(3, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(1, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, FourByteCodepoint)
{
  size_t offset = 0;
  const char *inbuf = "\xf0\x9d\x9c\x8d"; /* 𝜍  */
  size_t buflen = strlen(inbuf);
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(0x1D70D, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(1, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, MultipleVariableByteCodepoints)
{
  size_t offset = 0;
  const char *inbuf = "\xe1\xb8\x88" "a" "\xf0\x9d\x9c\x8d" "\xc3\x87" "d"; /* 𝜍  */
  size_t buflen = strlen(inbuf);
  int codep;

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(3, offset);
  ASSERT_EQ(0x1E08, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ('a', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(8, offset);
  ASSERT_EQ(0x1D70D, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(10, offset);
  ASSERT_EQ(0x00C7, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(11, offset);
  ASSERT_EQ('d', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(11, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(5, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, BadTwoByte)
{
  size_t offset = 0;
  const char *inbuf = "\xC2\xC3";
  size_t buflen = strlen(inbuf);
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, BadFourByte)
{
  size_t offset = 0;
  const char *inbuf = "\x80\x81\x82\x83";
  size_t buflen = strlen(inbuf);
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(3, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, BadOverlong)
{
  size_t offset = 0;
  const char *inbuf = "\xC0\x43";
  size_t buflen = strlen(inbuf);
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(0x43, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, Inval_U140000)
{
  size_t offset = 0;
  const char *inbuf = "\xF5\x80\x80\x80";
  size_t buflen = strlen(inbuf);
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(3, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, Inval_U110000)
{
  size_t offset = 0;
  const char *inbuf = "\xF4\x90\x80\x80";
  size_t buflen = strlen(inbuf);
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(3, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, Inval_UD800)
{
  size_t offset = 0;
  const char *inbuf = "\xED\xA0\x80";
  size_t buflen = strlen(inbuf);
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(3, offset);
  ASSERT_EQ(-1, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(3, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

/* Following are from http://www.cl.cam.ac.uk/~mgk25/ucs/examples/UTF-8-test.txt */

/* 2  Boundary condition test cases */
/* 2.1  First possible sequence of a certain length */
/* 2.1.1 - 2.1.6 */

TEST_F(DecodeUTF8, BoundaryConditionFirstPossibleOneByte)
{
  size_t offset = 0;
  const char *inbuf = " \x00 ";
  size_t buflen = 2 + 1;
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(0, codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(3, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(3, offset);
  ASSERT_EQ(0, codep);
}

TEST_F(DecodeUTF8, BoundaryConditionFirstPossibleTwoBytes)
{
  size_t offset = 0;
  const char *inbuf = " \xC2\x80 ";
  size_t buflen = 2 + 2;
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(3, offset);
  ASSERT_EQ(0x80, codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(0, codep);
}

TEST_F(DecodeUTF8, BoundaryConditionFirstPossibleThreeBytes)
{
  size_t offset = 0;
  const char *inbuf = " \xE0\xA0\x80 ";
  size_t buflen = 2 + 3;
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(0x800, codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(5, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(5, offset);
  ASSERT_EQ(0, codep);
}

TEST_F(DecodeUTF8, BoundaryConditionFirstPossibleFourBytes)
{
  size_t offset = 0;
  const char *inbuf = " \xF0\x90\x80\x80 ";
  size_t buflen = 2 + 4;
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(5, offset);
  ASSERT_EQ(0x10000, codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
}

TEST_F(DecodeUTF8, BoundaryConditionFirstPossibleFiveBytes)
{
  size_t offset = 0;
  const char *inbuf = " \xF8\x88\x80\x80\x80 ";
  size_t buflen = 2 + 5;
  int codep;
  size_t i;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  for (i = 0; i < buflen - 2; i++)
    {
      codep = __utf8_decode(inbuf, &offset, buflen);
      ASSERT_EQ(2 + i, offset);
      ASSERT_EQ(-1, codep);
    }

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
}

TEST_F(DecodeUTF8, BoundaryConditionFirstPossibleSixBytes)
{
  size_t offset = 0;
  const char *inbuf = " \xFC\x84\x80\x80\x80\x80 ";
  size_t buflen = 2 + 6;
  int codep;
  size_t i;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  for (i = 0; i < buflen - 2; i++)
    {
      codep = __utf8_decode(inbuf, &offset, buflen);
      ASSERT_EQ(2 + i, offset);
      ASSERT_EQ(-1, codep);
    }

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
}

/* 2.2  Last possible sequence of a certain length */
/* 2.2.1 - 2.2.6 */

TEST_F(DecodeUTF8, BoundaryConditionLastPossibleOneByte)
{
  size_t offset = 0;
  const char *inbuf = " \x7f ";
  size_t buflen = 2 + 1;
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(0x7f, codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(3, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(3, offset);
  ASSERT_EQ(0, codep);
}

TEST_F(DecodeUTF8, BoundaryConditionLastPossibleTwoBytes)
{
  size_t offset = 0;
  const char *inbuf = " \xDF\xBF ";
  size_t buflen = 2 + 2;
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(3, offset);
  ASSERT_EQ(0x7ff, codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(0, codep);
}

TEST_F(DecodeUTF8, BoundaryConditionLastPossibleThreeBytes)
{
  size_t offset = 0;
  const char *inbuf = " \xEF\xBF\xBF ";
  size_t buflen = 2 + 3;
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(0xffff, codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(5, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(5, offset);
  ASSERT_EQ(0, codep);
}

TEST_F(DecodeUTF8, BoundaryConditionLastPossibleFourBytes)
{
  size_t offset = 0;
  const char *inbuf = " \xF7\xBF\xBF\xBF ";
  size_t buflen = 2 + 4;
  int codep;
  size_t i;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  for (i = 0; i < buflen - 2; i++)
    {
      codep = __utf8_decode(inbuf, &offset, buflen);
      ASSERT_EQ(2 + i, offset);
      ASSERT_EQ(-1, codep);
    }

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
}

TEST_F(DecodeUTF8, BoundaryConditionLastPossibleFiveBytes)
{
  size_t offset = 0;
  const char *inbuf = " \xFB\xBF\xBF\xBF\xBF ";
  size_t buflen = 2 + 5;
  int codep;
  size_t i;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  for (i = 0; i < buflen - 2; i++)
    {
      codep = __utf8_decode(inbuf, &offset, buflen);
      ASSERT_EQ(2 + i, offset);
      ASSERT_EQ(-1, codep);
    }

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
}

TEST_F(DecodeUTF8, BoundaryConditionLastPossibleSixBytes)
{
  size_t offset = 0;
  const char *inbuf = " \xFD\xBF\xBF\xBF\xBF\xBF ";
  size_t buflen = 2 + 6;
  int codep;
  size_t i;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  for (i = 0; i < buflen - 2; i++)
    {
      codep = __utf8_decode(inbuf, &offset, buflen);
      ASSERT_EQ(2 + i, offset);
      ASSERT_EQ(-1, codep);
    }

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
}

/* 2.3  Other boundary conditions */
/* 2.3.1 - 2.3.5 */

TEST_F(DecodeUTF8, BoundaryConditionOther_1)
{
  size_t offset = 0;
  const char *inbuf = " \xed\x9f\xbf ";
  size_t buflen = 2 + 3;
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(0xD7FF, codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
}

TEST_F(DecodeUTF8, BoundaryConditionOther_2)
{
  size_t offset = 0;
  const char *inbuf = " \xee\x80\x80 ";
  size_t buflen = 2 + 3;
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(0xE000, codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
}

TEST_F(DecodeUTF8, BoundaryConditionOther_3)
{
  size_t offset = 0;
  const char *inbuf = " \xef\xbf\xbd ";
  size_t buflen = 2 + 3;
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(0xFFFD, codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
}

TEST_F(DecodeUTF8, BoundaryConditionOther_4)
{
  size_t offset = 0;
  const char *inbuf = " \xf4\x8f\xbf\xbf ";
  size_t buflen = 2 + 4;
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(5, offset);
  ASSERT_EQ(0x10FFFF, codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
}

TEST_F(DecodeUTF8, BoundaryConditionOther_5)
{
  size_t offset = 0;
  const char *inbuf = " \xf4\x90\x80\x80 ";
  size_t buflen = 2 + 4;
  int codep;
  size_t i;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  for (i = 0; i < buflen - 2; i++)
    {
      codep = __utf8_decode(inbuf, &offset, buflen);
      ASSERT_EQ(2 + i, offset);
      ASSERT_EQ(-1, codep);
    }

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
}

/* 3  Malformed sequences */
/* 3.1  Unexpected continuation bytes*/
/* 3.1.1 - 3.1.9 */

TEST_F(DecodeUTF8, FirstContinuationByte)
{
  size_t offset = 0;
  const char *inbuf = " \x80 ";
  size_t buflen = 2 + 1;
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(-1, codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, LastContinuationByte)
{
  size_t offset = 0;
  const char *inbuf = " \xbf ";
  size_t buflen = 2 + 1;
  int codep;
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ(' ', codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(2, offset);
  ASSERT_EQ(-1, codep);

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(' ', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, TwoToSevenContinuationBytes)
{
  const char *inbufs[6] = {
    " \x80\xbf ",
    " \x80\xbf\x80 ",
    " \x80\xbf\x80\xbf ",
    " \x80\xbf\x80\xbf\x80 ",
    " \x80\xbf\x80\xbf\x80\xbf ",
    " \x80\xbf\x80\xbf\x80\xbf\x80 ",
  };
  int bytes;

  for (bytes = 0; bytes < 6; bytes++) {
    const char *inbuf = inbufs[bytes];
    size_t buflen = strlen(inbuf);
    size_t offset = 0;
    int codep;
    size_t i;

    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(1, offset);
    ASSERT_EQ(' ', codep);

    for (i = 0; i < buflen - 2; i++) {
      codep = __utf8_decode(inbuf, &offset, buflen);
      ASSERT_EQ(2 + i, offset);
      ASSERT_EQ(-1, codep);
    }

    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(buflen, offset);
    ASSERT_EQ(' ', codep);
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(buflen, offset);
    ASSERT_EQ(0, codep);
    ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
  }
}

TEST_F(DecodeUTF8, All64ContinuationBytes)
{
  char inbuf[64+2+1];
  size_t buflen = 64+2;
  size_t offset = 0;
  int codep;
  size_t i;

  inbuf[0] = '\"';
  for (i = 0; i < 32; i++)
    {
      inbuf[1 + i] = i + 0x80;
    }
  for (; i < 64; i++)
    {
      inbuf[1 + i] = i + 0xA0;
    }
  inbuf[1 + i++] = '\"';
  inbuf[1 + i] = 0;

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ('\"', codep);

  for (i = 0; i < buflen - 2; i++) {
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(2 + i, offset);
    ASSERT_EQ(-1, codep);
  }

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ('\"', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

/* 3.2  Lonely start characters */

void DecodeUTF8::LonelyStartsTest(unsigned int numlone, int firstlone)
{
  char inbuf[numlone*2+2+1];
  size_t buflen = sizeof(inbuf) - 1;
  size_t offset = 0;
  int codep;
  size_t i;

  inbuf[0] = '\"';
  for (i = 0; i < numlone; i++)
    {
      inbuf[1 + (i * 2 + 0)] = i + firstlone;
      inbuf[1 + (i * 2 + 1)] = ' ';
    }
  inbuf[1 + (i * 2 + 0)] = '\"';
  inbuf[1 + (i * 2 + 1)] = 0;

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ('\"', codep);

  for (i = 0; i < numlone; i++) {
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(1 + (i * 2) + 1, offset);
    ASSERT_EQ(-1, codep);

    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(1 + (i * 2) + 2, offset);
    ASSERT_EQ(' ', codep);
  }

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ('\"', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

/* 3.2.1  All 32 first bytes of 2-byte sequences (0xc0-0xdf),
          each followed by a space character */
TEST_F(DecodeUTF8, LonelyStartsOfTwoByteSequences)
{
  LonelyStartsTest(32, 0xC0);
}

/* 3.2.2  All 16 first bytes of 3-byte sequences (0xe0-0xef),
          each followed by a space character */
TEST_F(DecodeUTF8, LonelyStartsOfThreeByteSequences)
{
  LonelyStartsTest(16, 0xE0);
}

/* 3.2.3  All 8 first bytes of 4-byte sequences (0xf0-0xf7),
          each followed by a space character */
TEST_F(DecodeUTF8, LonelyStartsOfFourByteSequences)
{
  LonelyStartsTest(4, 0xF0);
}

/* 3.2.4  All 2 first bytes of 5-byte sequences (0xfc-0xfd),
          each followed by a space character */
TEST_F(DecodeUTF8, LonelyStartsOfFiveByteSequences)
{
  LonelyStartsTest(2, 0xFC);
}

/* 3.3  Sequences with last continuation byte missing */
/* 3.3.1-3.3.10 */

void DecodeUTF8::AssertQuotedInvalidCodes(const char *inbuf)
{
  size_t offset = 0;
  size_t buflen = strlen(inbuf);
  int codep;
  size_t i;

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ('\"', codep);

  for (i = 0; i < buflen - 2; i++) {
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(2 + i, offset);
    ASSERT_EQ(-1, codep);
  }

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ('\"', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(offset, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, TwoByteSeqWithLastContinuationByteMissing)
{
  AssertQuotedInvalidCodes("\"\xC0\"");
  AssertQuotedInvalidCodes("\"\xDF\"");
}

TEST_F(DecodeUTF8, ThreeByteSeqWithLastContinuationByteMissing)
{
  AssertQuotedInvalidCodes("\"\xE0\x80\"");
  AssertQuotedInvalidCodes("\"\xEF\xBF\"");
}

TEST_F(DecodeUTF8, FourByteSeqWithLastContinuationByteMissing)
{
  AssertQuotedInvalidCodes("\"\xF0\x80\x80\"");
  AssertQuotedInvalidCodes("\"\xF7\xBF\xBF\"");
}

TEST_F(DecodeUTF8, FiveByteSeqWithLastContinuationByteMissing)
{
  AssertQuotedInvalidCodes("\"\xF8\x80\x80\x80\"");
  AssertQuotedInvalidCodes("\"\xFB\xBF\xBF\xBF\"");
}

TEST_F(DecodeUTF8, SixByteSeqWithLastContinuationByteMissing)
{
  AssertQuotedInvalidCodes("\"\xFC\x80\x80\x80\x80\"");
  AssertQuotedInvalidCodes("\"\xFD\xBF\xBF\xBF\xBF\"");
}

/* 3.5  Impossible bytes */

TEST_F(DecodeUTF8, ImpossibleByte_0xFE)
{
  AssertQuotedInvalidCodes("\"\xFE\"");
}

TEST_F(DecodeUTF8, ImpossibleByte_0xFF)
{
  AssertQuotedInvalidCodes("\"\xFE\"");
}

TEST_F(DecodeUTF8, ImpossibleBytes_0xFEFEFFFF)
{
  AssertQuotedInvalidCodes("\"\xFE\xFE\xFF\xFF\"");
}

/* 4  Overlong sequences */

/* 4.1  Examples of an overlong ASCII character  */

TEST_F(DecodeUTF8, OverlongASCIIChar)
{
  AssertQuotedInvalidCodes("\"\xc0\xaf\"");
  AssertQuotedInvalidCodes("\"\xe0\x80\xaf\"");
  AssertQuotedInvalidCodes("\"\xf0\x80\x80\xaf\"");
  AssertQuotedInvalidCodes("\"\xf8\x80\x80\x80\xaf\"");
  AssertQuotedInvalidCodes("\"\xfc\x80\x80\x80\x80\xaf\"");
}

/* 4.2  Maximum overlong sequences  */

TEST_F(DecodeUTF8, MaximumOverlongSequences)
{
  AssertQuotedInvalidCodes("\"\xc1\xbf\"");
  AssertQuotedInvalidCodes("\"\xe0\x9f\xbf\"");
  AssertQuotedInvalidCodes("\"\xf0\x8f\xbf\xbf\"");
  AssertQuotedInvalidCodes("\"\xf8\x87\xbf\xbf\xbf\"");
  AssertQuotedInvalidCodes("\"\xfc\x83\xbf\xbf\xbf\xbf\"");
}

/* 4.2  Overlong NUL character  */

TEST_F(DecodeUTF8, OverlongNULChar)
{
  AssertQuotedInvalidCodes("\"\xc0\x80\"");
  AssertQuotedInvalidCodes("\"\xe0\x80\x80\"");
  AssertQuotedInvalidCodes("\"\xf0\x80\x80\x80\"");
  AssertQuotedInvalidCodes("\"\xf8\x80\x80\x80\x80\"");
  AssertQuotedInvalidCodes("\"\xfc\x80\x80\x80\x80\x80\"");
}

/* 5  Illegal code positions */

/* 5.1  Single UTF-16 surrogates */

TEST_F(DecodeUTF8, SingleUTF16Surrogates)
{
  AssertQuotedInvalidCodes("\"\xed\xa0\x80\"");
  AssertQuotedInvalidCodes("\"\xed\xad\xbf\"");
  AssertQuotedInvalidCodes("\"\xed\xae\x80\"");
  AssertQuotedInvalidCodes("\"\xed\xaf\xbf\"");
  AssertQuotedInvalidCodes("\"\xed\xb0\x80\"");
  AssertQuotedInvalidCodes("\"\xed\xbe\x80\"");
  AssertQuotedInvalidCodes("\"\xed\xbf\xbf\"");
}

/* 5.2  Paired UTF-16 surrogates */

TEST_F(DecodeUTF8, PairedUTF16Surrogates)
{
  AssertQuotedInvalidCodes("\"\xed\xa0\x80\xed\xb0\x80\"");
  AssertQuotedInvalidCodes("\"\xed\xa0\x80\xed\xbf\xbf\"");
  AssertQuotedInvalidCodes("\"\xed\xad\xbf\xed\xb0\x80\"");
  AssertQuotedInvalidCodes("\"\xed\xad\xbf\xed\xbf\xbf\"");
  AssertQuotedInvalidCodes("\"\xed\xae\x80\xed\xbf\xbf\"");
  AssertQuotedInvalidCodes("\"\xed\xaf\xbf\xed\xb0\x80\"");
  AssertQuotedInvalidCodes("\"\xed\xaf\xbf\xed\xbf\xbf\"");
}

/* 5.2  Other illegal code positions */

TEST_F(DecodeUTF8, OtherIllegalCodePositions)
{
  const char *inbuf = "\"\xef\xbf\xbe\"";
  size_t offset = 0;
  size_t buflen = strlen(inbuf);
  int codep;

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ('\"', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(0xfffe, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ('\"', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(3, __utf8_num_code_points(inbuf, buflen, false));

  inbuf = "\"\xef\xbf\xbf\"";
  buflen = strlen(inbuf);
  offset = 0;

  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(1, offset);
  ASSERT_EQ('\"', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(4, offset);
  ASSERT_EQ(0xffff, codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ('\"', codep);
  codep = __utf8_decode(inbuf, &offset, buflen);
  ASSERT_EQ(buflen, offset);
  ASSERT_EQ(0, codep);
  ASSERT_EQ(3, __utf8_num_code_points(inbuf, buflen, false));
}

TEST_F(DecodeUTF8, AllOneByte)
{
  size_t offset = 0;
  char inbuf[4];
  int codep;
  int i;

  for (i = 0; i <= 0x7f; i++) {
    unsigned int buflen = 0;

    inbuf[buflen++] = '_';
    inbuf[buflen++] = i;
    inbuf[buflen++] = '>';
    inbuf[buflen] = 0;
    offset = 0;

    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(1, offset);
    ASSERT_EQ('_', codep);
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(buflen - 1, offset);
    ASSERT_EQ(i, codep);
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(buflen, offset);
    ASSERT_EQ('>', codep);
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(buflen, offset);
    ASSERT_EQ(0, codep);
    ASSERT_EQ(3, __utf8_num_code_points(inbuf, buflen, false));
  }
}

/* Tests for all inputs. */

TEST_F(DecodeUTF8, AllTwoByte)
{
  size_t offset = 0;
  char inbuf[5];
  int codep;
  unsigned int i;

  for (i = 0x80; i <= 0x7ff; i++) {
    unsigned int buflen = 0;

    inbuf[buflen++] = '_';
    inbuf[buflen++] = 0xC0 | ((i >> 6) & 0x1F);
    inbuf[buflen++] = 0x80 | (i & 0x3F);
    inbuf[buflen++] = '>';
    inbuf[buflen] = 0;
    offset = 0;

    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(1, offset);
    ASSERT_EQ('_', codep);
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(buflen - 1, offset);
    ASSERT_EQ(i, codep);
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(buflen, offset);
    ASSERT_EQ('>', codep);
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(buflen, offset);
    ASSERT_EQ(0, codep);
    ASSERT_EQ(3, __utf8_num_code_points(inbuf, buflen, false));
  }
}

TEST_F(DecodeUTF8, AllThreeByte)
{
  size_t offset = 0;
  char inbuf[6];
  int codep;
  unsigned int i;

  for (i = 0x800; i <= 0xffff; i++) {
    unsigned int buflen = 0;

    inbuf[buflen++] = '\"';
    inbuf[buflen++] = 0xE0 | ((i >> 12) & 0x0F);
    inbuf[buflen++] = 0x80 | ((i >> 6) & 0x3F);
    inbuf[buflen++] = 0x80 | (i & 0x3F);
    inbuf[buflen++] = '\"';
    inbuf[buflen] = 0;
    offset = 0;

    /* UTF-16 surrogates are invalid for UTF-8 */
    if (i >= 0xd800 && i <= 0xdfff) {
        AssertQuotedInvalidCodes(inbuf);
        continue;
    }

    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(1, offset);
    ASSERT_EQ('\"', codep);
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(buflen - 1, offset);
    ASSERT_EQ(i, codep);
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(buflen, offset);
    ASSERT_EQ('\"', codep);
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(buflen, offset);
    ASSERT_EQ(0, codep);
    ASSERT_EQ(3, __utf8_num_code_points(inbuf, buflen, false));
  }
}

TEST_F(DecodeUTF8, AllFourByte)
{
  size_t offset = 0;
  char inbuf[7];
  int codep;
  unsigned int i;

  for (i = 0x10000; i <= 0x10ffff; i++) {
    unsigned int buflen = 0;

    inbuf[buflen++] = 'R';
    inbuf[buflen++] = 0xF0 | ((i >> 18) & 0x07);
    inbuf[buflen++] = 0x80 | ((i >> 12) & 0x3F);
    inbuf[buflen++] = 0x80 | ((i >> 6) & 0x3F);
    inbuf[buflen++] = 0x80 | (i & 0x3F);
    inbuf[buflen++] = 'Y';
    inbuf[buflen] = 0;
    offset = 0;

    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(1, offset);
    ASSERT_EQ('R', codep);
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(buflen - 1, offset);
    ASSERT_EQ(i, codep);
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(buflen, offset);
    ASSERT_EQ('Y', codep);
    codep = __utf8_decode(inbuf, &offset, buflen);
    ASSERT_EQ(buflen, offset);
    ASSERT_EQ(0, codep);
    ASSERT_EQ(3, __utf8_num_code_points(inbuf, buflen, false));
  }
}

class EncodeUCS2 : public testing::Test
{
protected:

protected:
  virtual void SetUp()
  {
  }
  virtual void TearDown()
  {
  }
};

TEST_F(EncodeUCS2, SimpleCodepointASCII)
{
  int i;

  ASSERT_EQ('a', __ucs2_encode('a'));

  for (i = 0; i <= 0x7f; i++)
    ASSERT_EQ(i, __ucs2_encode(i));
}

TEST_F(EncodeUCS2, All16bitCodepoints)
{
  int i;

  for (i = 0; i <= 0xffff; i++) {
    /* UTF-16 surrogates are invalid, encoded to replacement character 0xfffd. */
    if (i >= 0xd800 && i <= 0xdfff) {
        ASSERT_EQ(0xfffd, __ucs2_encode(i));
        continue;
    }

    ASSERT_EQ(i, __ucs2_encode(i));
  }
}

TEST_F(EncodeUCS2, OutsideUCS2Range)
{
  ASSERT_EQ(0xfffd, __ucs2_encode(-1));
  ASSERT_EQ(0xfffd, __ucs2_encode(INT32_MIN));
  ASSERT_EQ(0xfffd, __ucs2_encode(0xffff + 1));
  ASSERT_EQ(0xfffd, __ucs2_encode(INT32_MAX));
}

class UTF8toUCS2 : public testing::Test
{
protected:
  char *_input;
  uint16_t *_output;

protected:
  virtual void SetUp()
  {
    _input = NULL;
    _output = NULL;
  }
  virtual void TearDown()
  {
    if (_input) delete[](_input);
    if (_output) delete[](_output);
  }
};

TEST_F(UTF8toUCS2, SimpleASCIItoUCS2_MatchingBufferElementCounts)
{
  size_t i, j;
  size_t maxlen = 16;
  size_t outputlen;
  size_t inputlen;

  inputlen = maxlen;
  _input = new char[inputlen];
  ASSERT_TRUE(_input != NULL);
  outputlen = maxlen;
  _output = new uint16_t[outputlen];
  ASSERT_TRUE(_output != NULL);

  memset(_output, 0xff, outputlen * sizeof(uint16_t));
  for (i = 0; i < maxlen; i++)
    _input[i] = 'a' + i % 20;

  for (i = 0; i < maxlen; i++) {
      size_t inpos = 0;
      size_t outpos = __utf8_to_ucs2(_output, outputlen,
                                     _input, &inpos, inputlen);

      ASSERT_EQ(outputlen, inputlen);
      ASSERT_EQ(inpos, inputlen);
      ASSERT_EQ(outpos, outputlen);
      ASSERT_EQ(inpos, outpos);

      for (j = 0; j < i; j++) {
          ASSERT_EQ((uint8_t)_input[j], _output[j]);
          ASSERT_EQ((uint8_t)_input[j], (uint8_t)('a' + j % 20));
      }
  }
}

TEST_F(UTF8toUCS2, SimpleASCIItoUCS2_OutputLarger)
{
  size_t i, j;
  size_t maxlen = 16;
  size_t outputlen;
  size_t inputlen;

  inputlen = maxlen;
  _input = new char[inputlen];
  ASSERT_TRUE(_input != NULL);
  outputlen = maxlen + 1000;
  _output = new uint16_t[outputlen];
  ASSERT_TRUE(_output != NULL);

  for (i = 0; i < maxlen; i++)
    _input[i] = 'a' + i % 20;

  for (i = 0; i < maxlen; i++) {
      memset(_output, 0xff, outputlen * sizeof(uint16_t));

      size_t inpos = 0;
      size_t outpos = __utf8_to_ucs2(_output, outputlen,
                                     _input, &inpos, inputlen);

      ASSERT_EQ(inpos, outpos);
      ASSERT_EQ(inpos, inputlen);

      for (j = 0; j < i; j++) {
          ASSERT_EQ((uint8_t)_input[j], _output[j]);
      }
  }
}

TEST_F(UTF8toUCS2, SimpleASCIItoUCS2_OutputSmaller)
{
  size_t i, j;
  size_t maxlen = 16;
  size_t outputlen;
  size_t inputlen;

  inputlen = maxlen;
  _input = new char[inputlen];
  ASSERT_TRUE(_input != NULL);
  outputlen = maxlen - 10;
  _output = new uint16_t[outputlen];
  ASSERT_TRUE(_output != NULL);

  for (i = 0; i < maxlen; i++)
    _input[i] = 'a' + i % 20;

  for (i = 0; i < maxlen; i++) {
      memset(_output, 0xff, outputlen * sizeof(uint16_t));

      size_t inpos = 0;
      size_t outpos = __utf8_to_ucs2(_output, outputlen,
                                     _input, &inpos, inputlen);

      ASSERT_EQ(inpos, outpos);
      ASSERT_EQ(inpos, outputlen);

      for (j = 0; j < outpos; j++) {
          ASSERT_EQ((uint8_t)_input[j], _output[j]);
      }
  }
}

TEST_F(UTF8toUCS2, InitialOffsetPastInputLength)
{
  size_t inpos = 5;
  size_t outpos;
  uint16_t output[4];

  outpos = __utf8_to_ucs2(output, 4, "abcd", &inpos, 4);

  ASSERT_EQ(outpos, 0);
  ASSERT_EQ(inpos, 5);
}

TEST_F(UTF8toUCS2, Valid_UTF8_to_USC2_One)
{
  const char *input = "\xe1\xb8\x88";
  size_t inputlen = strlen(input);
  size_t inpos = 0;
  size_t outpos;
  static const uint16_t outexpect[] = {
    0x1E08,
  };
  size_t explen = sizeof(outexpect) / sizeof(outexpect[0]);
  uint16_t output[explen + 2];
  size_t outputlen = sizeof(output) / sizeof(output[0]);
  size_t i;

  outpos = __utf8_to_ucs2(output, outputlen, input, &inpos, inputlen);

  ASSERT_EQ(outpos, explen);
  ASSERT_EQ(inpos, inputlen);
  for (i = 0; i < outpos; i++) {
    ASSERT_EQ(output[i], outexpect[i]);
  }

  inpos = 0;
  outpos = __utf8_to_ucs2(output, 0, input, &inpos, inputlen);
  ASSERT_EQ(inpos, 0);
  ASSERT_EQ(outpos, 0);

  inpos = 0;
  outpos = __utf8_to_ucs2(output, outputlen, input, &inpos, inputlen - 1);
  ASSERT_EQ(inpos, 2);
  ASSERT_EQ(outpos, 1);
  ASSERT_EQ(output[0], 0xfffd);
}

TEST_F(UTF8toUCS2, Valid_UTF8_to_USC2_Multiple)
{
  const char *input = "\xe1\xb8\x88" "a" "\xf0\x9d\x9c\x8d" "\xc3\x87" "d";
  size_t inputlen = strlen(input);
  size_t inpos = 0;
  size_t outpos;
  static const uint16_t outexpect[] = {
    0x1E08,
    'a',
    0xFFFD, /* 0x1D70D */
    0x00C7,
    'd'
  };
  size_t explen = sizeof(outexpect) / sizeof(outexpect[0]);
  uint16_t output[explen + 2];
  size_t outputlen = sizeof(output) / sizeof(output[0]);
  size_t i;

  outpos = __utf8_to_ucs2(output, outputlen, input, &inpos, inputlen);

  ASSERT_EQ(outpos, explen);
  ASSERT_EQ(inpos, inputlen);
  for (i = 0; i < outpos; i++) {
    ASSERT_EQ(output[i], outexpect[i]);
  }

  /* Truncate input by one ASCII */
  inpos = 0;
  outpos = __utf8_to_ucs2(output, outputlen, input, &inpos, inputlen - 1);
  ASSERT_EQ(outpos, explen - 1);
  ASSERT_EQ(inpos, inputlen - 1);
  for (i = 0; i < outpos; i++) {
    ASSERT_EQ(output[i], outexpect[i]);
  }

  /* Truncate input by one ASCII and half of two byte UTF-8 (last output is
   * replacement character). */
  inpos = 0;
  outpos = __utf8_to_ucs2(output, outputlen, input, &inpos, inputlen - 2);
  ASSERT_EQ(outpos, explen - 1);
  ASSERT_EQ(inpos, inputlen - 2);
  for (i = 0; i < outpos - 1; i++) {
    ASSERT_EQ(output[i], outexpect[i]);
  }
  ASSERT_EQ(output[i], 0xfffd);

  /* Splitting to multiple output buffers. */

  inpos = 0;
  outpos = __utf8_to_ucs2(output, 2, input, &inpos, inputlen);
  ASSERT_EQ(outpos, 2);
  ASSERT_EQ(inpos, 4);
  ASSERT_EQ(output[0], outexpect[0]);
  ASSERT_EQ(output[1], outexpect[1]);
  outpos = __utf8_to_ucs2(output, 2, input, &inpos, inputlen);
  ASSERT_EQ(outpos, 2);
  ASSERT_EQ(inpos, 10);
  ASSERT_EQ(output[0], outexpect[2]);
  ASSERT_EQ(output[1], outexpect[3]);
  outpos = __utf8_to_ucs2(output, 2, input, &inpos, inputlen);
  ASSERT_EQ(outpos, 1);
  ASSERT_EQ(inpos, inputlen);
  ASSERT_EQ(output[0], outexpect[4]);
}

/* Used http://smstools3.kekekasvi.com/topic.php?id=288 to generate vectors */

class ToPdu : public testing::Test
{
protected:
  uint8_t* _pdu;

protected:
  virtual void SetUp()
  {
    _pdu = NULL;
  }
  virtual void TearDown()
  {
    if (_pdu)
      free(_pdu);
    _pdu = NULL;
  }
};

TEST_F(ToPdu, OneASCIICharacter)
{
  int size = __ubmodem_utf8_to_pdu("A", &_pdu);
  ASSERT_EQ(2, size);
  ASSERT_EQ(0x00, _pdu[0]);
  ASSERT_EQ(0x41, _pdu[1]);
}

TEST_F(ToPdu, TwoASCIICharacters)
{
  int size = __ubmodem_utf8_to_pdu("aB", &_pdu);
  ASSERT_EQ(4, size);
  ASSERT_EQ(0x00, _pdu[0]);
  ASSERT_EQ(0x61, _pdu[1]);
  ASSERT_EQ(0x00, _pdu[2]);
  ASSERT_EQ(0x42, _pdu[3]);
}

TEST_F(ToPdu, ThreeToTenASCIICharacters)
{
  int num, i;

  for (num = 3; num < 10; num++) {
    char inbuf[num + 1];
    int size;

    for (i = 0; i < num; i++)
      inbuf[i] = '0' + i;
    inbuf[i] = '\0';

    size = __ubmodem_utf8_to_pdu(inbuf, &_pdu);
    ASSERT_EQ(2 * num, size);

    for (i = 0; i < num; i++) {
      ASSERT_EQ(0x00, _pdu[2 * i + 0]);
      ASSERT_EQ(inbuf[i], _pdu[2 * i + 1]);
    }

    free(_pdu);
    _pdu = NULL;
  }
}

TEST_F(ToPdu, UTF8_EuroCharacter)
{
  int size = __ubmodem_utf8_to_pdu("\xe2\x82\xac", &_pdu);
  ASSERT_EQ(2, size);
  ASSERT_EQ(0x20, _pdu[0]);
  ASSERT_EQ(0xAC, _pdu[1]);
}

TEST_F(ToPdu, UTF8_Characters)
{
  /* Following message is:
   * "Hëĺlố Ẅǿlrḑ!\nHello World!\n"
   */
  const char *message =
      "\x48\xc3\xab\xc4\xba\x6c\xe1\xbb\x91\x20\xe1\xba\x84\xc7\xbf\x6c"
      "\x72\xe1\xb8\x91\x21\x0a\x48\x65\x6c\x6c\x6f\x20\x57\x6f\x72\x6c"
      "\x64\x21\x0a";
  int size = __ubmodem_utf8_to_pdu(message, &_pdu);
  ASSERT_EQ(26 * 2, size);
  ASSERT_EQ(0x00, _pdu[0]);
  ASSERT_EQ(0x48, _pdu[1]);
  ASSERT_EQ(0x00, _pdu[2]);
  ASSERT_EQ(0xEB, _pdu[3]);
  ASSERT_EQ(0x01, _pdu[4]);
  ASSERT_EQ(0x3A, _pdu[5]);
  ASSERT_EQ(0x00, _pdu[6]);
  ASSERT_EQ(0x6C, _pdu[7]);
  ASSERT_EQ(0x1E, _pdu[8]);
  ASSERT_EQ(0xD1, _pdu[9]);
  ASSERT_EQ(0x00, _pdu[10]);
  ASSERT_EQ(0x20, _pdu[11]);
  ASSERT_EQ(0x1E, _pdu[12]);
  ASSERT_EQ(0x84, _pdu[13]);
  ASSERT_EQ(0x01, _pdu[14]);
  ASSERT_EQ(0xFF, _pdu[15]);
  ASSERT_EQ(0x00, _pdu[16]);
  ASSERT_EQ(0x6C, _pdu[17]);
  ASSERT_EQ(0x00, _pdu[18]);
  ASSERT_EQ(0x72, _pdu[19]);
  ASSERT_EQ(0x1E, _pdu[20]);
  ASSERT_EQ(0x11, _pdu[21]);
  ASSERT_EQ(0x00, _pdu[22]);
  ASSERT_EQ(0x21, _pdu[23]);
  ASSERT_EQ(0x00, _pdu[24]);
  ASSERT_EQ(0x0A, _pdu[25]);
  ASSERT_EQ(0x00, _pdu[26]);
  ASSERT_EQ(0x48, _pdu[27]);
  ASSERT_EQ(0x00, _pdu[28]);
  ASSERT_EQ(0x65, _pdu[29]);
  ASSERT_EQ(0x00, _pdu[30]);
  ASSERT_EQ(0x6C, _pdu[31]);
  ASSERT_EQ(0x00, _pdu[32]);
  ASSERT_EQ(0x6C, _pdu[33]);
  ASSERT_EQ(0x00, _pdu[34]);
  ASSERT_EQ(0x6F, _pdu[35]);
  ASSERT_EQ(0x00, _pdu[36]);
  ASSERT_EQ(0x20, _pdu[37]);
  ASSERT_EQ(0x00, _pdu[38]);
  ASSERT_EQ(0x57, _pdu[39]);
  ASSERT_EQ(0x00, _pdu[40]);
  ASSERT_EQ(0x6F, _pdu[41]);
  ASSERT_EQ(0x00, _pdu[42]);
  ASSERT_EQ(0x72, _pdu[43]);
  ASSERT_EQ(0x00, _pdu[44]);
  ASSERT_EQ(0x6C, _pdu[45]);
  ASSERT_EQ(0x00, _pdu[46]);
  ASSERT_EQ(0x64, _pdu[47]);
  ASSERT_EQ(0x00, _pdu[48]);
  ASSERT_EQ(0x21, _pdu[49]);
  ASSERT_EQ(0x00, _pdu[50]);
  ASSERT_EQ(0x0A, _pdu[51]);
}

class PhoneNumToPdu : public testing::Test
{
protected:
  uint8_t* _pdu;

protected:
  virtual void SetUp()
  {
    _pdu = NULL;
  }
  virtual void TearDown()
  {
    if (_pdu)
      free(_pdu);
    _pdu = NULL;
  }
};

TEST_F(PhoneNumToPdu, EmptyNumber)
{
  int size = __ubmodem_address_to_pdu("", &_pdu);
  ASSERT_EQ(0, size);
}

TEST_F(PhoneNumToPdu, OneNumber)
{
  int size = __ubmodem_address_to_pdu("1", &_pdu);
  ASSERT_EQ(2, size);
  ASSERT_EQ(0x81, _pdu[0]);
  ASSERT_EQ(0xF1, _pdu[1]);

  for (int i = 0; i < 10; i++) {
      char addr[2] = { (char)(i + '0'), 0 };
      size = __ubmodem_address_to_pdu(addr, &_pdu);
      ASSERT_EQ(2, size);
      ASSERT_EQ(0x81, _pdu[0]);
      ASSERT_EQ(0xF0 + i, _pdu[1]);
      free(_pdu);
      _pdu = NULL;
  }
}

TEST_F(PhoneNumToPdu, TwoNumbers)
{
  int size = __ubmodem_address_to_pdu("56", &_pdu);
  ASSERT_EQ(2, size);
  ASSERT_EQ(0x81, _pdu[0]);
  ASSERT_EQ(0x65, _pdu[1]);

  for (int i = 0; i < 100; i++) {
      char addr[3] = { (char)((i / 10) + '0'), (char)((i % 10) + '0'), 0 };
      size = __ubmodem_address_to_pdu(addr, &_pdu);
      ASSERT_EQ(2, size);
      ASSERT_EQ(0x81, _pdu[0]);
      ASSERT_EQ((i / 10) | ((i % 10) << 4), _pdu[1]);
      free(_pdu);
      _pdu = NULL;
  }
}

TEST_F(PhoneNumToPdu, ThreeNumbers)
{
  int size = __ubmodem_address_to_pdu("369", &_pdu);
  ASSERT_EQ(3, size);
  ASSERT_EQ(0x81, _pdu[0]);
  ASSERT_EQ(0x63, _pdu[1]);
  ASSERT_EQ(0xF9, _pdu[2]);
}

TEST_F(PhoneNumToPdu, OneNumberInternational)
{
  int size = __ubmodem_address_to_pdu("+1", &_pdu);
  ASSERT_EQ(2, size);
  ASSERT_EQ(0x91, _pdu[0]);
  ASSERT_EQ(0xF1, _pdu[1]);

  for (int i = 0; i < 10; i++) {
      char addr[3] = { (char)'+', (char)(i + '0'), 0 };
      size = __ubmodem_address_to_pdu(addr, &_pdu);
      ASSERT_EQ(2, size);
      ASSERT_EQ(0x91, _pdu[0]);
      ASSERT_EQ(0xF0 + i, _pdu[1]);
      free(_pdu);
      _pdu = NULL;
  }
}

TEST_F(PhoneNumToPdu, TwoNumbersInternational)
{
  int size = __ubmodem_address_to_pdu("+56", &_pdu);
  ASSERT_EQ(2, size);
  ASSERT_EQ(0x91, _pdu[0]);
  ASSERT_EQ(0x65, _pdu[1]);

  for (int i = 0; i < 100; i++) {
      char addr[4] = {
        (char)'+',
        (char)((i / 10) + '0'),
        (char)((i % 10) + '0'),
        0
      };
      size = __ubmodem_address_to_pdu(addr, &_pdu);
      ASSERT_EQ(2, size);
      ASSERT_EQ(0x91, _pdu[0]);
      ASSERT_EQ((i / 10) | ((i % 10) << 4), _pdu[1]);
      free(_pdu);
      _pdu = NULL;
  }
}

TEST_F(PhoneNumToPdu, ThreeNumbersInternational)
{
  int size = __ubmodem_address_to_pdu("+369", &_pdu);
  ASSERT_EQ(3, size);
  ASSERT_EQ(0x91, _pdu[0]);
  ASSERT_EQ(0x63, _pdu[1]);
  ASSERT_EQ(0xF9, _pdu[2]);
}

class MessageToMultiPartPDU : public testing::Test
{
protected:
  uint8_t* _msg_pdu;
  uint8_t* _recv_pdu;
  char *_pdu;
  char *_message;

protected:
  virtual void SetUp()
  {
    _pdu = NULL;
    _msg_pdu = NULL;
    _recv_pdu = NULL;
    _message = NULL;
  }
  virtual void TearDown()
  {
    if (_pdu)
      free(_pdu);
    if (_msg_pdu)
      free(_msg_pdu);
    if (_recv_pdu)
      free(_recv_pdu);
    if (_message)
      free(_message);
    _pdu = NULL;
    _msg_pdu = NULL;
    _recv_pdu = NULL;
    _message = NULL;
  }
};

TEST_F(MessageToMultiPartPDU, SimpleMessage)
{
  struct multipdu_iter_s iter;
  int recvsize;
  int mesgsize;
  bool ret;

  recvsize = __ubmodem_address_to_pdu("+4321", &_recv_pdu);
  ASSERT_EQ(3, recvsize);

  mesgsize = __ubmodem_utf8_to_pdu("test", &_msg_pdu);
  ASSERT_EQ(8, mesgsize);

  ret = __ubmodem_pdu_multipart_iterator_init(&iter, mesgsize, 0x11);
  ASSERT_EQ(true, ret);
  ASSERT_EQ(0, iter.msg_pos);
  ASSERT_EQ(0, iter.part_pos);
  ASSERT_EQ(1, iter.part_count);
  ASSERT_EQ(0x11, iter.ref_num);

  int pdulen = __ubmodem_prepare_pdu_string(_recv_pdu, recvsize, _msg_pdu,
                                            mesgsize, &iter, &_pdu);
  int smsclen = __ubmodem_get_pdu_smsc_len(_pdu);
  ASSERT_EQ(17, pdulen / 2 - smsclen);
  ASSERT_EQ(pdulen, strlen(_pdu));
  ASSERT_STREQ("000100049134120008080074006500730074", _pdu);
  ASSERT_EQ(mesgsize, iter.msg_pos);
  ASSERT_EQ(1, iter.part_pos);
}

TEST_F(MessageToMultiPartPDU, MaximumOnePartMessage)
{
  struct multipdu_iter_s iter;
  int recvsize;
  int mesgsize;
  bool ret;

  recvsize = __ubmodem_address_to_pdu("+4321", &_recv_pdu);
  ASSERT_EQ(3, recvsize);

  mesgsize = __ubmodem_utf8_to_pdu("test0123456789test0123456789test0123456789"
                                   "test0123456789test0123456789", &_msg_pdu);
  ASSERT_EQ(70 * 2, mesgsize);

  ret = __ubmodem_pdu_multipart_iterator_init(&iter, mesgsize, 0xAA);
  ASSERT_EQ(true, ret);
  ASSERT_EQ(0, iter.msg_pos);
  ASSERT_EQ(0, iter.part_pos);
  ASSERT_EQ(1, iter.part_count);
  ASSERT_EQ(0xAA, iter.ref_num);

  int pdulen = __ubmodem_prepare_pdu_string(_recv_pdu, recvsize, _msg_pdu,
                                            mesgsize, &iter, &_pdu);
  int smsclen = __ubmodem_get_pdu_smsc_len(_pdu);
  ASSERT_EQ(149, pdulen / 2 - smsclen);
  ASSERT_EQ(pdulen, strlen(_pdu));
  ASSERT_STREQ("0001000491341200088C007400650073007400300031003200330034"
               "00350036003700380039007400650073007400300031003200330034"
               "00350036003700380039007400650073007400300031003200330034"
               "00350036003700380039007400650073007400300031003200330034"
               "00350036003700380039007400650073007400300031003200330034"
               "00350036003700380039", _pdu);
  ASSERT_EQ(mesgsize, iter.msg_pos);
  ASSERT_EQ(1, iter.part_pos);
}

TEST_F(MessageToMultiPartPDU, Multipart71Characters)
{
  struct multipdu_iter_s iter;
  int recvsize;
  int mesgsize;
  bool ret;

  recvsize = __ubmodem_address_to_pdu("+4321", &_recv_pdu);
  ASSERT_EQ(3, recvsize);

  mesgsize = __ubmodem_utf8_to_pdu("test0123456789test0123456789test0123456789"
                                   "test0123456789test0123456789!", &_msg_pdu);
  ASSERT_EQ(71 * 2, mesgsize);

  ret = __ubmodem_pdu_multipart_iterator_init(&iter, mesgsize, 0xAA);
  ASSERT_EQ(true, ret);
  ASSERT_EQ(0, iter.msg_pos);
  ASSERT_EQ(0, iter.part_pos);
  ASSERT_EQ(2, iter.part_count);
  ASSERT_EQ(0xAA, iter.ref_num);

  /* First part for multipart SMS. */
  int pdulen = __ubmodem_prepare_pdu_string(_recv_pdu, recvsize, _msg_pdu,
                                            mesgsize, &iter, &_pdu);
  int smsclen = __ubmodem_get_pdu_smsc_len(_pdu);
  ASSERT_EQ(149, pdulen / 2 - smsclen);
  ASSERT_EQ(pdulen, strlen(_pdu));
  ASSERT_STREQ("0041000491341200088C050003AA02010074006500730074003000310032"
               "003300340035003600370038003900740065007300740030003100320033"
               "003400350036003700380039007400650073007400300031003200330034"
               "003500360037003800390074006500730074003000310032003300340035"
               "003600370038003900740065007300740030003100320033003400350036",
               _pdu);
  ASSERT_EQ(67 * 2, iter.msg_pos);
  ASSERT_EQ(1, iter.part_pos);
  free(_pdu);
  _pdu = NULL;

  /* Second part for multipart SMS. */
  pdulen = __ubmodem_prepare_pdu_string(_recv_pdu, recvsize, _msg_pdu,
                                        mesgsize, &iter, &_pdu);
  smsclen = __ubmodem_get_pdu_smsc_len(_pdu);
  ASSERT_EQ(23, pdulen / 2 - smsclen);
  ASSERT_EQ(pdulen, strlen(_pdu));
  ASSERT_STREQ("0041000491341200080E050003AA02020037003800390021", _pdu);
  ASSERT_EQ(mesgsize, iter.msg_pos);
  ASSERT_EQ(2, iter.part_pos);
  free(_pdu);
  _pdu = NULL;

  /* Already completed, prepare pdu should return 0. */
  pdulen = __ubmodem_prepare_pdu_string(_recv_pdu, recvsize, _msg_pdu,
                                        mesgsize, &iter, &_pdu);
  ASSERT_EQ(0, pdulen);
  ASSERT_EQ(NULL, _pdu);
}

TEST_F(MessageToMultiPartPDU, MaximumMultipartMessage)
{
  size_t messagelen = 67 * 255;
  struct multipdu_iter_s iter;
  int recvsize;
  int mesgsize;
  int smsclen;
  int pdulen;
  bool ret;
  int count;
  int i;

  _message = (char *)malloc(messagelen + 1);
  ASSERT_NE(NULL, (intptr_t)_message);
  memset(_message, 'a', messagelen);
  _message[messagelen] = '\0';

  recvsize = __ubmodem_address_to_pdu("+4321", &_recv_pdu);
  ASSERT_EQ(3, recvsize);

  mesgsize = __ubmodem_utf8_to_pdu(_message, &_msg_pdu);
  ASSERT_EQ(messagelen * 2, mesgsize);

  ret = __ubmodem_pdu_multipart_iterator_init(&iter, mesgsize, 0xAA);
  ASSERT_EQ(true, ret);
  ASSERT_EQ(0, iter.msg_pos);
  ASSERT_EQ(0, iter.part_pos);
  ASSERT_EQ(255, iter.part_count);
  ASSERT_EQ(0xAA, iter.ref_num);

  count = 0;
  do {
      char temp[15 * 2 + 1];

      ASSERT_EQ(count, iter.part_pos);
      count++;

      /* Next part for multipart SMS. */
      pdulen = __ubmodem_prepare_pdu_string(_recv_pdu, recvsize, _msg_pdu,
                                            mesgsize, &iter, &_pdu);
      smsclen = __ubmodem_get_pdu_smsc_len(_pdu);
      ASSERT_EQ(149, pdulen / 2 - smsclen);
      ASSERT_EQ(pdulen, strlen(_pdu));
      ASSERT_EQ(count, iter.part_pos);
      ASSERT_EQ(67 * 2 * iter.part_pos, iter.msg_pos);

      memcpy(temp, _pdu, 15 * 2);
      temp[15 * 2] = '\0';
      ASSERT_STREQ("0041000491341200088C050003AAFF", temp);

      snprintf(temp, sizeof(temp), "%02X", count);
      ASSERT_EQ(temp[0], _pdu[15 * 2 + 0]);
      ASSERT_EQ(temp[1], _pdu[15 * 2 + 1]);

      for (i = 0; i < 67; i++)
        {
          memcpy(temp, &_pdu[16 * 2 + i * 4], 4);
          temp[4] = 0;
          ASSERT_STREQ("0061", temp);
        }

      free(_pdu);
      _pdu = NULL;
  } while (count < 255);

  /* Already completed, prepare pdu should return 0. */
  pdulen = __ubmodem_prepare_pdu_string(_recv_pdu, recvsize, _msg_pdu,
                                        mesgsize, &iter, &_pdu);
  ASSERT_EQ(0, pdulen);
  ASSERT_EQ(NULL, _pdu);
}

TEST_F(MessageToMultiPartPDU, OverMaximumMultipartMessage)
{
  size_t messagelen = 67 * 255 + 1;
  struct multipdu_iter_s iter;
  int recvsize;
  int mesgsize;
  bool ret;

  _message = (char *)malloc(messagelen + 1);
  ASSERT_NE(NULL, (intptr_t)_message);
  memset(_message, 'a', messagelen);
  _message[messagelen] = '\0';

  recvsize = __ubmodem_address_to_pdu("+4321", &_recv_pdu);
  ASSERT_EQ(3, recvsize);

  mesgsize = __ubmodem_utf8_to_pdu(_message, &_msg_pdu);
  ASSERT_EQ(messagelen * 2, mesgsize);

  ret = __ubmodem_pdu_multipart_iterator_init(&iter, mesgsize, 0xAA);
  ASSERT_EQ(false, ret);
}
