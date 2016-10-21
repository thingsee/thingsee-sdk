/****************************************************************************
 * apps/netutils/json/cJSON_internal.h
 *
 * This file is a part of NuttX:
 *
 *   Copyright (c) 2011 Gregory Nutt. All rights reserved.
 *   Copyright (c) 2015 Haltian Ltd.
 *     Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
 *
 * And derives from the cJSON Project which has an MIT license:
 *
 *   Copyright (c) 2009 Dave Gamble
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 ****************************************************************************/

#ifndef __APPS_NETUTILS_JSON_CJSON_INTERNAL_H
#define __APPS_NETUTILS_JSON_CJSON_INTERNAL_H

#include <apps/netutils/cJSON.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Internal data storage formats: */

#define JSON_DATA_NULL          0
#define JSON_DATA_FALSE         1
#define JSON_DATA_TRUE          2
#define JSON_DATA_NUM_INT8      3
#define JSON_DATA_NUM_INT16     4
#define JSON_DATA_NUM_INT32     5
#define JSON_DATA_NUM_FLOAT     6
#define JSON_DATA_NUM_DOUBLE    7
#define JSON_DATA_STR_SHORT     8
#define JSON_DATA_STR_LONG      9
#define JSON_DATA_ARRAY         10
#define JSON_DATA_OBJECT        11
#define JSON_DATA_BUFFER        12

#define JSON_DATA_MASK          0xF

/* Next object format: */

/* Next object available as pointer. */
#define JSON_NEXT_PTR          (0 << 4)

/* Packed format: No next object. */
#define JSON_NEXT_NULL         (1 << 4)

/* Packed format: Next object appended after this object. */
#define JSON_NEXT_MEM          (2 << 4)

#define JSON_NEXT_MASK         (0x3 << 4)

/* Object name format: */

#define JSON_NAME_SHORT        (0 << 6)   /* Length field uint8_t. */
#define JSON_NAME_LONG         (1 << 6)   /* Length field uint16_t. */
#define JSON_NAME_EMPTY        (2 << 6)   /* No name (""). */

#define JSON_NAME_MASK         (0x3 << 6)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Common cJSON structure: */

struct cJSON
{
  /* The internal storage format info of the item, as defined above. */

  uint8_t info;

  /* Data section structure, defined by 'info'.
   *
   * Data section has following fields: [next][data][name].
   *
   * [next] field length:
   *   - JSON_NEXT_NULL, JSON_NEXT_MEM: Zero bytes. These are used for packed
   *     storage format.
   *   - JSON_NEXT_PTR: Four bytes (struct cJSON_DataNext)
   *
   * [data] field length:
   *   - JSON_DATA_NULL, JSON_DATA_FALSE, JSON_DATA_TRUE: Zero bytes.
   *   - JSON_DATA_NUM_INT8: One byte (struct cJSON_DataInt8).
   *   - JSON_DATA_NUM_INT16: Two bytes (struct cJSON_DataInt16).
   *   - JSON_DATA_NUM_INT32: Four bytes (struct cJSON_DataInt32).
   *   - JSON_DATA_NUM_FLOAT: Four bytes (struct cJSON_DataFloat).
   *   - JSON_DATA_NUM_DOUBLE: Eight bytes (struct cJSON_DataDouble).
   *   - JSON_DATA_STR_SHORT: Variable, (string buffer length) + 1 bytes. Field
   *                          type is 'struct cJSON_DataStringShort' with
   *                          uint8_t for string length.
   *   - JSON_DATA_STR_LONG: Variable, (string buffer length) + 2 bytes. Field
   *                         type is 'struct cJSON_DataStringLong' with
   *                         uint16_t for string length.
   *   - JSON_DATA_NUM_ARRAY: Four bytes (struct cJSON_DataArray).
   *   - JSON_DATA_NUM_OBJECT: Four bytes (struct cJSON_DataArray).
   *
   * [name] field length:
   *   - JSON_NAME_EMPTY: Zero bytes.
   *   - JSON_NAME_SHORT: Variable, (string buffer length) + 1 bytes. Field
   *                      type is 'struct cJSON_DataStringShort' with uint8_t
   *                      for string length.
   *   - JSON_NAME_LONG: Variable, (string buffer length) + 2 bytes. Field
   *                     type is 'struct cJSON_DataStringShort' with uint16_t
   *                     for string length.
   */

  uint8_t data[];
} packed_struct;

/* Next field structure. */

struct cJSON_DataNext
{
  /* Next allow you to walk array/object chains. */

  struct cJSON *next;
} packed_struct;

/* JSON_DATA_STR_SHORT & JSON_NAME_SHORT field structure. */

struct cJSON_DataStringShort
{
  uint8_t string_size;
  char valuestring[];
} packed_struct;

/* JSON_DATA_STR_LONG & JSON_NAME_LONG & JSON_DATA_BUFFER field structure. */

struct cJSON_DataStringLong
{
  uint16_t string_size;
  char valuestring[];
} packed_struct;

/* JSON_DATA_INT8 field structure. */

struct cJSON_DataInt8
{
  int8_t valueint8;
} packed_struct;

/* JSON_DATA_INT16 field structure. */

struct cJSON_DataInt16
{
  int16_t valueint16;
} packed_struct;

/* JSON_DATA_INT32 field structure. */

struct cJSON_DataInt32
{
  int32_t valueint32;
} packed_struct;

/* JSON_DATA_FLOAT field structure. */

struct cJSON_DataFloat
{
  float valuefloat;
} packed_struct;

/* JSON_DATA_DOUBLE field structure. */

struct cJSON_DataDouble
{
  double valuedouble;
} packed_struct;

/* JSON_DATA_ARRAY & JSON_DATA_OBJECT field: */

struct cJSON_DataArray
{
  /* An array or object item will have a child pointer pointing to a chain
   * of the items in the array/object.
   */

  struct cJSON *child;
} packed_struct;

/****************************************************************************
 * Public Inline Functions
 ****************************************************************************/

/* Internal storage format to public cJSON type mapping. */
static inline __attribute__((always_inline)) enum cJSON_type_e
cJSON_InfoToType(uint8_t info)
{
  switch (info & JSON_DATA_MASK)
    {
    case JSON_DATA_NULL:
      return cJSON_NULL;

    case JSON_DATA_FALSE:
      return cJSON_False;

    case JSON_DATA_TRUE:
      return cJSON_True;

    case JSON_DATA_NUM_INT8:
    case JSON_DATA_NUM_INT16:
    case JSON_DATA_NUM_INT32:
    case JSON_DATA_NUM_FLOAT:
    case JSON_DATA_NUM_DOUBLE:
      return cJSON_Number;

    case JSON_DATA_STR_SHORT:
    case JSON_DATA_STR_LONG:
      return cJSON_String;

    case JSON_DATA_ARRAY:
      return cJSON_Array;

    case JSON_DATA_OBJECT:
      return cJSON_Object;

    case JSON_DATA_BUFFER:
      return cJSON_Buffer;

    default:
      DEBUGASSERT(false);
      return cJSON_NULL;
    }
}

/* Get length of data field. */
static inline __attribute__((always_inline)) size_t
cJSON_DataFieldLength(uint8_t data_type, void *field)
{
  switch (data_type)
    {
    case JSON_DATA_NULL:
    case JSON_DATA_FALSE:
    case JSON_DATA_TRUE:
      return 0;

    case JSON_DATA_NUM_INT8:
      return sizeof(struct cJSON_DataInt8);

    case JSON_DATA_NUM_INT16:
      return sizeof(struct cJSON_DataInt16);

    case JSON_DATA_NUM_INT32:
      return sizeof(struct cJSON_DataInt32);

    case JSON_DATA_NUM_FLOAT:
      return sizeof(struct cJSON_DataFloat);

    case JSON_DATA_NUM_DOUBLE:
      return sizeof(struct cJSON_DataDouble);

    case JSON_DATA_STR_SHORT:
      {
        struct cJSON_DataStringShort *ss = field;

        return sizeof(*ss) + ss->string_size;
      }

    case JSON_DATA_BUFFER:
    case JSON_DATA_STR_LONG:
      {
        struct cJSON_DataStringLong *sl = field;

        return sizeof(*sl) + sl->string_size;
      }

    case JSON_DATA_ARRAY:
      return sizeof(struct cJSON_DataArray);

    case JSON_DATA_OBJECT:
      return sizeof(struct cJSON_DataArray);

    default:
      DEBUGASSERT(false);
      return 0;
    }
}

/* Get length of data field. */
static inline __attribute__((always_inline)) size_t
cJSON_NameFieldLength(uint8_t name_type, void *field)
{
  switch (name_type)
    {
    case JSON_NAME_EMPTY:
      return 0;

    case JSON_NAME_SHORT:
      {
        struct cJSON_DataStringShort *ss = field;

        return sizeof(*ss) + ss->string_size;
      }

    case JSON_NAME_LONG:
      {
        struct cJSON_DataStringLong *sl = field;

        return sizeof(*sl) + sl->string_size;
      }

    default:
      DEBUGASSERT(false);
      return 0;
    }
}

/* Get data section fields and returns object length. */
static inline __attribute((always_inline)) size_t
cJSON_GetFieldPointers(cJSON *item, struct cJSON_DataNext **next_field,
                       uint8_t *next_field_type, void **data_field,
                       uint8_t *data_field_type, void **name_field,
                       uint8_t *name_field_type)
{
  uint8_t next_type = item->info & JSON_NEXT_MASK;
  uint8_t data_type = item->info & JSON_DATA_MASK;
  uint8_t name_type = item->info & JSON_NAME_MASK;
  struct cJSON_DataNext *next = NULL;
  uint8_t *pos = item->data;
  void *data = NULL;
  void *name = NULL;
  size_t datalen;
  size_t namelen;

  if (next_type == JSON_NEXT_PTR)
    {
      next = (void *)pos;
      pos += sizeof(*next);
    }

  datalen = cJSON_DataFieldLength(data_type, pos);
  if (datalen > 0)
    {
      data = (void *)pos;
      pos += datalen;
    }

  namelen = cJSON_NameFieldLength(name_type, pos);
  if (namelen > 0)
    {
      name = (void *)pos;
      pos += namelen;
    }

  if (next_field)
    *next_field = next;
  if (next_field_type)
    *next_field_type = next_type;

  if (data_field)
    *data_field = data;
  if (data_field_type)
    *data_field_type = data_type;

  if (name_field)
    *name_field = name;
  if (name_field_type)
    *name_field_type = name_type;

  return pos - (uint8_t *)item;
}

#endif /* __APPS_NETUTILS_JSON_CJSON_INTERNAL_H */
