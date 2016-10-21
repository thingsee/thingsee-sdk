/****************************************************************************
 * thingsee/system_properties/sys_prop.h
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
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

#ifndef SYS_PROP_H_
#define SYS_PROP_H_


#define SYS_PROP_INFO_FLAG_SLEEP    (1 << 4)  /* 00010000 0x10 */



/****************************************************************************
 * Name: sys_prop_check_flag
 *
 * Description:
 *  Reads flag status from EEPROM
 *
 * Input Parameters:
 *  flag - flag to be checked
 *
 * Returned Values:
 *  true if flag is set. Otherwise false
 *
 ****************************************************************************/

bool sys_prop_check_flag(uint32_t flag);

/****************************************************************************
 * Name: sys_prop_clear_flag
 *
 * Description:
 *  Clears flag in EEPROM
 *
 * Input Parameters:
 *  flag - flag to be cleared
 *
 * Returned Values:
 *  true if flag is set. Otherwise false
 *
 ****************************************************************************/

int sys_prop_clear_flag(uint32_t flag);

#endif /* SYS_PROP_H_ */
