/****************************************************************************
 *
 * Copyright (C) 2015 Intel Corporation
 * Copyright (C) 2016 Haltian Ltd.
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

#include <nuttx/config.h>
#include <apps/netutils/cbor.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

enum Mode
{
  PrintCborDump = 0,
};

static uint8_t *buffer = NULL;

static int do_exit(int retval)
{
  free(buffer);
  buffer = NULL;
  exit(retval);
  return 0;
}

static void xrealloc_buffer(size_t size, const char *fname)
{
  void *new;
  new = realloc(buffer, size);
  if (new == NULL)
    {
      fprintf(stderr, "%s: %s\n", fname, strerror(errno));
      do_exit(EXIT_FAILURE);
    }
  buffer = new;
}

static void printerror(CborError err, const char *fname)
{
  fprintf(stderr, "%s: %s\n", fname, cbor_error_string(err));
  do_exit(EXIT_FAILURE);
}

static void dumpFile(FILE *in, const char *fname, int mode)
{
  static const size_t chunklen = 256;
  size_t bufsize = 0;

  size_t buflen = 0;
  do
    {
      if (bufsize == buflen)
        xrealloc_buffer(bufsize += chunklen, fname);

      size_t n = fread(buffer + buflen, 1, bufsize - buflen, in);
      buflen += n;
      if (n == 0)
        {
          if (!ferror(in))
            continue;
          fprintf(stderr, "%s: %s\n", fname, strerror(errno));
          do_exit(EXIT_FAILURE);
        }
    }
  while (!feof(in));

  CborParser parser;
  CborValue value;
  CborError err = cbor_parser_init(buffer, buflen, 0, &parser, &value);
  if (!err)
    {
      err = cbor_value_to_pretty_advance(stdout, &value);
      if (!err)
        puts("");
    }
  if (!err && value.ptr != buffer + buflen)
    err = CborErrorGarbageAtEnd;
  if (err)
    printerror(err, fname);
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int cbordump_main(int argc, char *argv[])
#endif
{
  int mode = PrintCborDump;
  int c;
  while ((c = getopt(argc, argv, "ch")) != -1)
    {
      switch (c)
        {
        case 'c':
          mode = PrintCborDump;
          break;

        case '?':
          fprintf(stderr, "Unknown option -%c.\n", optopt);
          // fall through
          // no break
        case 'h':
          puts(
              "Usage: cbordump [OPTION]... [FILE]...\n"
              "Interprets FILEs as CBOR binary data and dumps the content to stdout.\n"
              "\n"
              "Options:\n"
              " -c       Print a CBOR dump (see RFC 7049) (default)\n"
              " -h       Print this help output and exit\n"
              "");
          return do_exit(c == '?' ? EXIT_FAILURE : EXIT_SUCCESS);
        }
    }

  char **fname = argv + optind;
  if (!*fname)
    {
      dumpFile(stdin, "-", mode);
    }
  else
    {
      for (; *fname; ++fname)
        {
          FILE *in = fopen(*fname, "rb");
          if (!in)
            {
              perror("open");
              return EXIT_FAILURE;
            }

          dumpFile(in, *fname, mode);
          fclose(in);
        }
    }

  return do_exit(EXIT_SUCCESS);
}
