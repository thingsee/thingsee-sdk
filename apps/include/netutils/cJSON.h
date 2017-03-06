/****************************************************************************
 * apps/netutils/json/cJSON.c
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Ported by: Darcy Gong
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

#ifndef __APPS_INCLUDE_NETUTILS_JSON_H
#define __APPS_INCLUDE_NETUTILS_JSON_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define cJSON_CreateNull()    cJSON_CreateNamedNull(NULL)
#define cJSON_CreateTrue()    cJSON_CreateNamedTrue(NULL)
#define cJSON_CreateFalse()   cJSON_CreateNamedFalse(NULL)
#define cJSON_CreateBool(b)   cJSON_CreateNamedBool(NULL, b)
#define cJSON_CreateNumber(n) cJSON_CreateNamedNumber(NULL, n)
#define cJSON_CreateString(s) cJSON_CreateNamedString(NULL, s)
#define cJSON_CreateArray()   cJSON_CreateNamedArray(NULL)
#define cJSON_CreateObject()  cJSON_CreateNamedObject(NULL)
#define cJSON_CreateBuffer(b,l) cJSON_CreateNamedBuffer(NULL, b, l)

#define cJSON_AddNullToObject(object,name) \
  cJSON_AddNamedItemToObject(object, cJSON_CreateNamedNull(name))
#define cJSON_AddTrueToObject(object,name) \
  cJSON_AddNamedItemToObject(object, cJSON_CreateNamedTrue(name))
#define cJSON_AddFalseToObject(object,name) \
  cJSON_AddNamedItemToObject(object, cJSON_CreateNamedFalse(name))
#define cJSON_AddBoolToObject(object,name,b) \
  cJSON_AddNamedItemToObject(object, cJSON_CreateNamedBool(name, b))
#define cJSON_AddNumberToObject(object,name,n) \
  cJSON_AddNamedItemToObject(object, cJSON_CreateNamedNumber(name, n))
#define cJSON_AddStringToObject(object,name,s) \
  cJSON_AddNamedItemToObject(object, cJSON_CreateNamedString(name, s))
#define cJSON_AddBufferToObject(object,name,b,l) \
  cJSON_AddNamedItemToObject(object, cJSON_CreateNamedBuffer(name, b, l))

#define cJSON_AddNamedItemToObject(object,item) \
    cJSON_AddItemToArray(object, item)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Enumeration of public cJSON object types */

enum cJSON_type_e
{
  cJSON_NULL = 0,
  cJSON_False,
  cJSON_True,
  cJSON_Number,
  cJSON_String,
  cJSON_Array,
  cJSON_Object,
  cJSON_Buffer /* for CBOR byte-string compatibility. */
};

/* cJSON_Buffer structure returned with cJSON_buffer(). */

struct cJSON_buffer_s
{
  void *ptr;
  size_t len;
};

/* Forward declaration of the cJSON structure */

typedef struct cJSON cJSON;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Return type of cJSON object. */

enum cJSON_type_e cJSON_type(const cJSON *item);

/* Return name of cJSON object. */

const char *cJSON_name(const cJSON *item);

/* Return string value of cJSON object. */

const char *cJSON_string(const cJSON *item);

/* Return buffer value of cJSON object. */

struct cJSON_buffer_s cJSON_buffer(const cJSON *item);

/* Return integer value of cJSON object. */

int cJSON_int(const cJSON *item);

/* Return floating-point value of cJSON object. */

double cJSON_double(const cJSON *item);

/* Return boolean value of cJSON object. */

bool cJSON_boolean(const cJSON *item);

/* Return child object of cJSON object. */

cJSON *cJSON_child(cJSON *item);

/* Return next object of cJSON object. */

cJSON *cJSON_next(cJSON *item);

/* Supply a block of JSON, and this returns a cJSON object you can
 * interrogate. Call cJSON_Delete when finished.
 */

cJSON *cJSON_Parse(const char *value);

/* Supply a stream of JSON, and this returns a cJSON object you can
 * interrogate. Call cJSON_Delete when finished.
 */

cJSON *cJSON_Parse_Stream(char (*getc_fn)(void *priv), void *priv);

/* Render a cJSON entity to text for transfer/storage. Free the char* when
 * finished.
 */

/* Parse an object from file-descriptor - create a new root, and populate. */

cJSON *cJSON_Parse_fd(int fd, ssize_t max_readlen, size_t *nread);

char *cJSON_Print(cJSON *item);

/* Render a cJSON entity to text for transfer/storage without any
 * formatting. Free the char* when finished.
 */

char *cJSON_PrintUnformatted(cJSON *item);

/* Render a cJSON entity to text for transfer/storage with or without
 * formatting.
 */

void cJSON_Print_Stream(cJSON *item, bool formatted,
                        void (*putc_fn)(char c, void *priv), void *putc_priv);

/* Render a cJSON item/entity/structure to buffer. */

size_t cJSON_Print_Buf(cJSON *item, bool formatted, char *buf, size_t buflen);

/* Get allocation size of cJSON object. */

size_t cJSON_GetAllocMemSize(cJSON *item);

/* Delete a cJSON entity and all subentities. */

void cJSON_Delete(cJSON *c);

/* Returns the number of items in an array (or object). */

int cJSON_GetArraySize(cJSON *array);

/* Retrieve item number "item" from array "array". Returns NULL if
 * unsuccessful.
 */

cJSON *cJSON_GetArrayItem(cJSON *array, int item);

/* Get item "string" from object. Case insensitive. */

cJSON *cJSON_GetObjectItem(cJSON *object, const char *string);

/* For analysing failed parses. This returns a pointer to the parse error.
 * You'll probably need to look a few chars back to make sense of it.
 * Defined when cJSON_Parse() returns 0. 0 when cJSON_Parse() succeeds.
 */

const char *cJSON_GetErrorPtr(void);

/* Set the name of object. */

cJSON *cJSON_SetItemName(cJSON *item, const char *name);

/* These calls create a cJSON item of the appropriate type. */

cJSON *cJSON_CreateNamedNull(const char *name);
cJSON *cJSON_CreateNamedTrue(const char *name);
cJSON *cJSON_CreateNamedFalse(const char *name);
cJSON *cJSON_CreateNamedBool(const char *name, bool b);
cJSON *cJSON_CreateNamedNumber(const char *name, double num);
cJSON *cJSON_CreateNamedString(const char *name, const char *string);
cJSON *cJSON_CreateNamedArray(const char *name);
cJSON *cJSON_CreateNamedObject(const char *name);
cJSON *cJSON_CreateNamedBuffer(const char *name, const void *buf, size_t len);

/* These utilities create an Array of count items. */

cJSON *cJSON_CreateIntArray(const int *numbers, int count);
cJSON *cJSON_CreateFloatArray(const float *numbers, int count);
cJSON *cJSON_CreateDoubleArray(const double *numbers, int count);
cJSON *cJSON_CreateStringArray(const char **strings, int count);

/* Append item to the specified array/object. */

bool cJSON_AddItemToArray(cJSON *array, cJSON *item);
bool cJSON_AddItemToObject(cJSON *object, const char *string, cJSON *item);

/* Remove/Detach items from Arrays/Objects. */

cJSON *cJSON_DetachItemFromArray(cJSON *array, int which);
void cJSON_DeleteItemFromArray(cJSON *array, int which);
cJSON *cJSON_DetachItemFromObject(cJSON *object, const char *string);
void cJSON_DeleteItemFromObject(cJSON *object, const char *string);

/* Update array items. */

bool cJSON_ReplaceItemInArray(cJSON *array, int which, cJSON *newitem);
bool cJSON_ReplaceItemInObject(cJSON *object, const char *string, cJSON *newitem);

/* Unpack and pack arrays/objects for reduced memory usage. */

bool cJSON_IsPacked(cJSON *item);
cJSON *cJSON_UnpackArray(cJSON *item);
cJSON *cJSON_PackArray(cJSON *item);
void cJSON_PackChild(cJSON *item);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_INCLUDE_NETUTILS_JSON_H */
