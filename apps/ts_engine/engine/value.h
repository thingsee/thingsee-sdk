/****************************************************************************
 * apps/thingsee/engine/value.h
 *
 *   Copyright (C) 2015 Haltian Ltd. All rights reserved.
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

#ifndef __APPS_TS_ENGINE_ENGINE_VALUE_H__
#define __APPS_TS_ENGINE_ENGINE_VALUE_H__

#include <stdint.h>
#include <apps/netutils/cJSON.h>

#define VALUE_STR_MAX_LEN       128

enum ts_valuetype_t
{
    VALUEDOUBLE,
    VALUEUINT16,
    VALUEUINT32,
    VALUEINT16,
    VALUEINT32,
    VALUESTRING,
    VALUEARRAY,
    VALUEBOOL,
    VALUEHEXSTRING,
    VALUEARRAY_FIRSTSTRING
};

typedef uint32_t sense_id_t;

struct ts_value;

struct ts_value_array
{
    int number_of_items;
    struct ts_value *items;
};

struct ts_value
{
    enum ts_valuetype_t valuetype;

    union
    {
        double valuedouble;
        uint16_t valueuint16;
        uint32_t valueuint32;
        int16_t valueint16;
        int32_t valueint32;
        bool valuebool;
        char *valuestring;
        struct ts_value_array valuearray;
    };
};

int __value_serialize(char *str, size_t size,
                      const struct ts_value * const value);
int __value_deserialize(char * value_str,
                        struct ts_value * const value);
int __value_to_json(cJSON *obj, const char * const label,
                       const struct ts_value * const value);
#endif
