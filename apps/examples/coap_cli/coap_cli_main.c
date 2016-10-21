/* coap-client -- simple CoAP client for NuttX
 *
 * This file is part of the CoAP library libcoap. Please see
 * README for terms of use.
 *
 * Copyright (C) 2010--2014 Olaf Bergmann <bergmann@tzi.org>
 * Copyright (C) 2016 Haltian Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   o Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   o Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Port to NuttX by Jussi Kivilinna <jussi.kivilinna@haltian.com>
 */

#include <nuttx/config.h>

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>

#include <apps/netutils/libcoap/coap.h>
#include <apps/netutils/dnsclient.h>

typedef unsigned char method_t;

#define FLAGS_BLOCK 0x01

struct coap_cli_priv_s {
  int flags;

  str the_token;

  coap_list_t *optlist;

  /* Request URI.
   * TODO: associate the resources with transaction id and make it expireable */
  coap_uri_t uri;
  str proxy;
  unsigned short proxy_port;

  /* reading is done when this flag is set */
  int ready;

  str output_file; /* output file name */
  FILE *file; /* output file stream */

  str payload; /* optional payload to send */

  unsigned char msgtype; /* usually, requests are sent confirmable */

  method_t method; /* the method we are using in our requests */

  coap_block_t block;

  unsigned int wait_seconds; /* default timeout in seconds */
  coap_tick_t max_wait; /* global timeout (changed by set_timeout()) */

  unsigned int obs_seconds; /* default observe time */
  coap_tick_t obs_wait; /* timeout for current subscription */
};

static unsigned char g_token_data[9];

static const struct coap_cli_priv_s g_coap_cli_defaults =
{
  .flags = 0,
  .the_token = { 0, g_token_data },
  .optlist = NULL,
  .proxy = { 0, NULL },
  .proxy_port = COAP_DEFAULT_PORT,
  .ready = 0,
  .output_file = { 0, NULL }, /* output file name */
  .file = NULL, /* output file stream */
  .payload = { 0, NULL }, /* optional payload to send */
  .msgtype = COAP_MESSAGE_CON, /* usually, requests are sent confirmable */
  .method = 1, /* the method we are using in our requests */

  .block = { .num = 0, .m = 0, .szx = 6 },

  .wait_seconds = 90, /* default timeout in seconds */

  .obs_seconds = 30, /* default observe time */
  .obs_wait = 0, /* timeout for current subscription */
};

static struct coap_cli_priv_s g;

#define min(a,b) ((a) < (b) ? (a) : (b))

static inline void set_timeout(coap_tick_t *timer, const unsigned int seconds)
{
  coap_ticks(timer);
  *timer += seconds * COAP_TICKS_PER_SECOND;
}

static int append_to_output(const unsigned char *data, size_t len)
{
  size_t written;

  if (!g.file)
    {
      if (!g.output_file.s || (g.output_file.length && g.output_file.s[0] == '-'))
        g.file = stdout;
      else
        {
          if (!(g.file = fopen((char *) g.output_file.s, "w")))
            {
              perror("fopen");
              return -1;
            }
        }
    }

  do
    {
      written = fwrite(data, 1, len, g.file);
      len -= written;
      data += written;
    }
  while (written && len);
  fflush(g.file);

  return 0;
}

static void close_output(void)
{
  if (g.file)
    {

      /* add a newline before closing in case were writing to stdout */
      if (!g.output_file.s || (g.output_file.length && g.output_file.s[0] == '-'))
        fwrite("\n", 1, 1, g.file);

      fflush(g.file);
      fclose(g.file);
    }
}

static coap_pdu_t *
new_ack(coap_context_t *ctx, coap_queue_t *node)
{
  coap_pdu_t *pdu = coap_new_pdu();

  if (pdu)
    {
      pdu->hdr->type = COAP_MESSAGE_ACK;
      pdu->hdr->code = 0;
      pdu->hdr->id = node->pdu->hdr->id;
    }

  return pdu;
}

static coap_pdu_t *
new_response(coap_context_t *ctx, coap_queue_t *node, unsigned int code)
{
  coap_pdu_t *pdu = new_ack(ctx, node);

  if (pdu)
    pdu->hdr->code = code;

  return pdu;
}

static coap_pdu_t *
coap_new_request(coap_context_t *ctx, method_t m, coap_list_t *options,
                 unsigned char *data, size_t length)
{
  coap_pdu_t *pdu;
  coap_list_t *opt;

  if (!(pdu = coap_new_pdu()))
    return NULL;

  pdu->hdr->type = g.msgtype;
  pdu->hdr->id = coap_new_message_id(ctx);
  pdu->hdr->code = m;

  pdu->hdr->token_length = g.the_token.length;
  if (!coap_add_token(pdu, g.the_token.length, g.the_token.s))
    {
      debug("cannot add token to request\n");
    }

  coap_show_pdu(pdu);

  for (opt = options; opt; opt = opt->next)
    {
      coap_add_option(pdu, COAP_OPTION_KEY(*(coap_option * )opt->data),
                      COAP_OPTION_LENGTH(*(coap_option * )opt->data),
                      COAP_OPTION_DATA(*(coap_option * )opt->data));
    }

  if (length)
    {
      if ((g.flags & FLAGS_BLOCK) == 0)
        coap_add_data(pdu, length, data);
      else
        coap_add_block(pdu, length, data, g.block.num, g.block.szx);
    }

  return pdu;
}

static coap_tid_t clear_obs(coap_context_t *ctx,
                     const coap_endpoint_t *local_interface,
                     const coap_address_t *remote)
{
  coap_pdu_t *pdu;
  coap_list_t *option;
  coap_tid_t tid = COAP_INVALID_TID;
  unsigned char buf[2];

  /* create bare PDU w/o any option  */
  pdu = coap_pdu_init(g.msgtype, COAP_REQUEST_GET, coap_new_message_id(ctx),
  COAP_MAX_PDU_SIZE);

  if (!pdu)
    {
      return tid;
    }

  if (!coap_add_token(pdu, g.the_token.length, g.the_token.s))
    {
      coap_log(LOG_CRIT, "cannot add token");
      goto error;
    }

  for (option = g.optlist; option; option = option->next)
    {
      if (COAP_OPTION_KEY(*(coap_option *)option->data) == COAP_OPTION_URI_HOST)
        {
          if (!coap_add_option(
              pdu, COAP_OPTION_KEY(*(coap_option * )option->data),
              COAP_OPTION_LENGTH(*(coap_option * )option->data),
              COAP_OPTION_DATA(*(coap_option * )option->data)))
            {
              goto error;
            }
          break;
        }
    }

  if (!coap_add_option(pdu, COAP_OPTION_OBSERVE,
                       coap_encode_var_bytes(buf, COAP_OBSERVE_CANCEL), buf))
    {
      coap_log(LOG_CRIT, "cannot add option Observe: %u", COAP_OBSERVE_CANCEL);
      goto error;
    }

  for (option = g.optlist; option; option = option->next)
    {
      switch (COAP_OPTION_KEY(*(coap_option * )option->data))
        {
        case COAP_OPTION_URI_PORT:
        case COAP_OPTION_URI_PATH:
        case COAP_OPTION_URI_QUERY:
          if (!coap_add_option(
              pdu, COAP_OPTION_KEY(*(coap_option * )option->data),
              COAP_OPTION_LENGTH(*(coap_option * )option->data),
              COAP_OPTION_DATA(*(coap_option * )option->data)))
            {
              goto error;
            }
          break;
        default:
          ;
        }
    }

  coap_show_pdu(pdu);

  if (pdu->hdr->type == COAP_MESSAGE_CON)
    tid = coap_send_confirmed(ctx, local_interface, remote, pdu);
  else
    tid = coap_send(ctx, local_interface, remote, pdu);

  if (tid == COAP_INVALID_TID)
    {
      debug("clear_obs: error sending new request");
      coap_delete_pdu(pdu);
    }
  else if (pdu->hdr->type != COAP_MESSAGE_CON)
    coap_delete_pdu(pdu);

  return tid;
  error:

  coap_delete_pdu(pdu);
  return tid;
}

static int resolve_address(const str *server, struct sockaddr *dst)
{
  struct sockaddr_in *dst_inet = (void *)dst;
  in_addr_t addrs[5];
  char addrstr[128];
  unsigned int ridx;
  int naddr;

  if (server->length)
    {
      snprintf(addrstr, sizeof(addrstr), "%.*s", server->length, server->s);
    }
  else
    {
      snprintf(addrstr, sizeof(addrstr), "%s", "127.0.0.1");
    }

  naddr = dns_gethostip_multi(addrstr, addrs, sizeof(addrs) / sizeof(addrs[0]));
  if (naddr <= 0)
    return -1;

  speckrandom_buf(&ridx, sizeof(ridx));

  dst_inet->sin_family = AF_INET;
  dst_inet->sin_port = 0;
  dst_inet->sin_addr.s_addr = addrs[ridx % naddr];

  return sizeof(*dst_inet);
}

#define HANDLE_BLOCK1(Pdu)						\
  ((method == COAP_REQUEST_PUT || method == COAP_REQUEST_POST) &&	\
   ((flags & FLAGS_BLOCK) == 0) &&					\
   ((Pdu)->hdr->code == COAP_RESPONSE_CODE(201) ||			\
    (Pdu)->hdr->code == COAP_RESPONSE_CODE(204)))

static inline int check_token(coap_pdu_t *received)
{
  return received->hdr->token_length == g.the_token.length
      && memcmp(received->hdr->token, g.the_token.s, g.the_token.length) == 0;
}

static void message_handler(struct coap_context_t *ctx,
                     const coap_endpoint_t *local_interface,
                     const coap_address_t *remote, coap_pdu_t *sent,
                     coap_pdu_t *received, const coap_tid_t id)
{

  coap_pdu_t *pdu = NULL;
  coap_opt_t *block_opt;
  coap_opt_iterator_t opt_iter;
  unsigned char buf[4];
  coap_list_t *option;
  size_t len;
  unsigned char *databuf;
  coap_tid_t tid;

#ifndef NDEBUG
  if (LOG_DEBUG <= coap_get_log_level())
    {
      debug("** process incoming %d.%02d response:\n",
            (received->hdr->code >> 5), received->hdr->code & 0x1F);
      coap_show_pdu(received);
    }
#endif

  /* check if this is a response to our original request */
  if (!check_token(received))
    {
      /* drop if this was just some message, or send RST in case of notification */
      if (!sent && (received->hdr->type == COAP_MESSAGE_CON
          || received->hdr->type == COAP_MESSAGE_NON))
        coap_send_rst(ctx, local_interface, remote, received);
      return;
    }

  if (received->hdr->type == COAP_MESSAGE_RST)
    {
      info("got RST\n");
      return;
    }

  /* output the received data, if any */
  if (COAP_RESPONSE_CLASS(received->hdr->code) == 2)
    {
      /* set obs timer if we have successfully subscribed a resource */
      if (sent && coap_check_option(received, COAP_OPTION_SUBSCRIPTION,
                                    &opt_iter))
        {
          debug("observation relationship established, set timeout to %d\n",
                g.obs_seconds);
          set_timeout(&g.obs_wait, g.obs_seconds);
        }

      /* Got some data, check if block option is set. Behavior is undefined if
       * both, Block1 and Block2 are present. */
      block_opt = coap_check_option(received, COAP_OPTION_BLOCK2, &opt_iter);
      if (block_opt)
        { /* handle Block2 */
          unsigned short blktype = opt_iter.type;

          /* TODO: check if we are looking at the correct block number */
          if (coap_get_data(received, &len, &databuf))
            append_to_output(databuf, len);

          if (COAP_OPT_BLOCK_MORE(block_opt))
            {
              /* more bit is set */
              debug("found the M bit, block size is %u, block nr. %u\n",
                    COAP_OPT_BLOCK_SZX(block_opt),
                    coap_opt_block_num(block_opt));

              /* create pdu with request for next block */
              pdu = coap_new_request(ctx, g.method, NULL, NULL, 0); /* first, create bare PDU w/o any option  */
              if (pdu)
                {
                  /* add URI components from optlist */
                  for (option = g.optlist; option; option = option->next)
                    {
                      switch (COAP_OPTION_KEY(*(coap_option * )option->data))
                        {
                        case COAP_OPTION_URI_HOST:
                        case COAP_OPTION_URI_PORT:
                        case COAP_OPTION_URI_PATH:
                        case COAP_OPTION_URI_QUERY:
                          coap_add_option(
                              pdu,
                              COAP_OPTION_KEY(*(coap_option * )option->data),
                              COAP_OPTION_LENGTH(*(coap_option * )option->data),
                              COAP_OPTION_DATA(*(coap_option * )option->data));
                          break;
                        default:
                          ; /* skip other options */
                        }
                    }

                  /* finally add updated block option from response, clear M bit */
                  /* blocknr = (blocknr & 0xfffffff7) + 0x10; */
                  debug("query block %d\n",
                        (coap_opt_block_num(block_opt) + 1));
                  coap_add_option(
                      pdu,
                      blktype,
                      coap_encode_var_bytes(
                          buf,
                          ((coap_opt_block_num(block_opt) + 1) << 4) | COAP_OPT_BLOCK_SZX(
                              block_opt)),
                      buf);

                  if (pdu->hdr->type == COAP_MESSAGE_CON)
                    tid = coap_send_confirmed(ctx, local_interface, remote,
                                              pdu);
                  else
                    tid = coap_send(ctx, local_interface, remote, pdu);

                  if (tid == COAP_INVALID_TID)
                    {
                      debug("message_handler: error sending new request");
                      coap_delete_pdu(pdu);
                    }
                  else
                    {
                      set_timeout(&g.max_wait, g.wait_seconds);
                      if (pdu->hdr->type != COAP_MESSAGE_CON)
                        coap_delete_pdu(pdu);
                    }

                  return;
                }
            }
        }
      else
        { /* no Block2 option */
          block_opt = coap_check_option(received, COAP_OPTION_BLOCK1,
                                        &opt_iter);

          if (block_opt)
            { /* handle Block1 */
              g.block.szx = COAP_OPT_BLOCK_SZX(block_opt);
              g.block.num = coap_opt_block_num(block_opt);

              debug("found Block1, block size is %u, block nr. %u\n", g.block.szx,
                     g.block.num);

              if (g.payload.length <= (g.block.num + 1) * (1 << (g.block.szx + 4)))
                {
                  debug("upload ready\n");
                  g.ready = 1;
                  return;
                }

              /* create pdu with request for next block */
              pdu = coap_new_request(ctx, g.method, NULL, NULL, 0); /* first, create bare PDU w/o any option  */
              if (pdu)
                {

                  /* add URI components from optlist */
                  for (option = g.optlist; option; option = option->next)
                    {
                      switch (COAP_OPTION_KEY(*(coap_option * )option->data))
                        {
                        case COAP_OPTION_URI_HOST:
                        case COAP_OPTION_URI_PORT:
                        case COAP_OPTION_URI_PATH:
                        case COAP_OPTION_CONTENT_FORMAT:
                        case COAP_OPTION_URI_QUERY:
                          coap_add_option(
                              pdu,
                              COAP_OPTION_KEY(*(coap_option * )option->data),
                              COAP_OPTION_LENGTH(*(coap_option * )option->data),
                              COAP_OPTION_DATA(*(coap_option * )option->data));
                          break;
                        default:
                          ; /* skip other options */
                        }
                    }

                  /* finally add updated block option from response, clear M bit */
                  /* blocknr = (blocknr & 0xfffffff7) + 0x10; */
                  g.block.num++;
                  g.block.m = ((g.block.num + 1) * (1 << (g.block.szx + 4))
                      < g.payload.length);

                  debug("send block %d\n", g.block.num);
                  coap_add_option(
                      pdu,
                      COAP_OPTION_BLOCK1,
                      coap_encode_var_bytes(
                          buf, (g.block.num << 4) | (g.block.m << 3) | g.block.szx),
                      buf);

                  coap_add_block(pdu, g.payload.length, g.payload.s, g.block.num,
                                 g.block.szx);
                  coap_show_pdu(pdu);
                  if (pdu->hdr->type == COAP_MESSAGE_CON)
                    tid = coap_send_confirmed(ctx, local_interface, remote,
                                              pdu);
                  else
                    tid = coap_send(ctx, local_interface, remote, pdu);

                  if (tid == COAP_INVALID_TID)
                    {
                      debug("message_handler: error sending new request");
                      coap_delete_pdu(pdu);
                    }
                  else
                    {
                      set_timeout(&g.max_wait, g.wait_seconds);
                      if (pdu->hdr->type != COAP_MESSAGE_CON)
                        coap_delete_pdu(pdu);
                    }

                  return;
                }
            }
          else
            {
              /* There is no block option set, just read the data and we are done. */
              if (coap_get_data(received, &len, &databuf))
                append_to_output(databuf, len);
            }
        }
    }
  else
    { /* no 2.05 */

      /* check if an error was signaled and output payload if so */
      if (COAP_RESPONSE_CLASS(received->hdr->code) >= 4)
        {
          fprintf(stderr, "%d.%02d", (received->hdr->code >> 5),
                  received->hdr->code & 0x1F);
          if (coap_get_data(received, &len, &databuf))
            {
              fprintf(stderr, " ");
              while (len--)
                fprintf(stderr, "%c", *databuf++);
            }
          fprintf(stderr, "\n");
        }

    }

  /* finally send new request, if needed */
  if (pdu && coap_send(ctx, local_interface, remote, pdu) == COAP_INVALID_TID)
    {
      debug("message_handler: error sending response");
    }
  coap_delete_pdu(pdu);

  /* our job is done, we can exit at any time */
  g.ready = coap_check_option(received, COAP_OPTION_SUBSCRIPTION,
                              &opt_iter) == NULL;
}

static void usage(const char *program, const char *version)
{
  const char *p;

  p = strrchr(program, '/');
  if (p)
    program = ++p;

  fprintf(
      stderr,
      "%s v%s -- a small CoAP implementation\n"
      "(c) 2010-2014 Olaf Bergmann <bergmann@tzi.org>\n\n"
      "usage: %s [-A type...] [-t type] [-b [num,]size] [-B seconds] [-e text]\n"
      "\t\t[-m method] [-N] [-o file] [-P addr[:port]]\n"
      "\t\t[-s duration] [-O num,text] [-T string] [-v num] URI\n\n"
      "\tURI can be an absolute or relative coap URI,\n"
      "\t-A type...\taccepted media types as comma-separated list of\n"
      "\t\t\tsymbolic or numeric values\n"
      "\t-t type\t\tcontent type for given resource for PUT/POST\n"
      "\t-b [num,]size\tblock size to be used in GET/PUT/POST requests\n"
      "\t       \t\t(value must be a multiple of 16 not larger than 1024)\n"
      "\t       \t\tIf num is present, the request chain will start at\n"
      "\t       \t\tblock num\n"
      "\t-B seconds\tbreak operation after waiting given seconds\n"
      "\t\t\t(default is %d)\n"
      "\t-e text\t\tinclude text as payload (use percent-encoding for\n"
      "\t\t\tnon-ASCII characters)\n"
      "\t-f file\t\tfile to send with PUT/POST (use '-' for STDIN)\n"
      "\t-m method\trequest method (get|put|post|delete), default is 'get'\n"
      "\t-N\t\tsend NON-confirmable message\n"
      "\t-o file\t\toutput received data to this file (use '-' for STDOUT)\n"
      "\t-s duration\tsubscribe for given duration [s]\n"
      "\t-v num\t\tverbosity level (default: 3)\n"
      "\t-O num,text\tadd option num with contents text to request\n"
      "\t-P addr[:port]\tuse proxy (automatically adds Proxy-Uri option to\n"
      "\t\t\trequest)\n"
      "\t-T token\tinclude specified token\n"
      "\n"
      "examples:\n"
      "\t%s -m get coap://[::1]/\n"
      "\t%s -m get coap://[::1]/.well-known/core\n"
      "\t%s -m get -T cafe coap://[::1]/time\n"
      "\techo 1000 | %s -m put -T cafe coap://[::1]/time -f -\n",
      program, version, program, g.wait_seconds,
      program, program, program, program);
}

static int order_opts(void *a, void *b)
{
  if (!a || !b)
    return a < b ? -1 : 1;

  if (COAP_OPTION_KEY(*(coap_option *)a) < COAP_OPTION_KEY(*(coap_option * )b))
    return -1;

  return COAP_OPTION_KEY(*(coap_option * )a)
      == COAP_OPTION_KEY(*(coap_option * )b);
}

static coap_list_t *
new_option_node(unsigned short key, unsigned int length, unsigned char *data)
{
  coap_option *option;
  coap_list_t *node;

  option = coap_malloc(sizeof(coap_option) + length);
  if (!option)
    goto error;

  COAP_OPTION_KEY(*option) = key;
  COAP_OPTION_LENGTH(*option) = length;
  memcpy(COAP_OPTION_DATA(*option), data, length);

  /* we can pass NULL here as delete function since option is released automatically  */
  node = coap_new_listnode(option, NULL);

  if (node)
    return node;

error:
  perror("new_option_node: malloc");
  coap_free(option);
  return NULL;
}

typedef struct
{
  unsigned char code;
  char *media_type;
} content_type_t;

static void cmdline_content_type(char *arg, unsigned short key)
{
  static const content_type_t content_types[] =
    { { 0, "plain" },
      { 0, "text/plain" },
      { 40, "link" },
      { 40, "link-format" },
      { 40, "application/link-format" },
      { 41, "xml" },
      { 42, "binary" },
      { 42, "octet-stream" },
      { 42, "application/octet-stream" },
      { 47, "exi" },
      { 47, "application/exi" },
      { 50, "json" },
      { 50, "application/json" },
      { 255, NULL }
    };
  coap_list_t *node;
  unsigned char i, value[10];
  int valcnt = 0;
  unsigned char buf[2];
  char *p, *q = arg;

  while (q && *q)
    {
      p = strchr(q, ',');

      if (isdigit(*q))
        {
          if (p)
            *p = '\0';
          value[valcnt++] = atoi(q);
        }
      else
        {
          for (i = 0;
              content_types[i].media_type && strncmp(
                  q, content_types[i].media_type, p ? p - q : strlen(q))
                                             != 0; ++i)
            {
              /* */
            }

          if (content_types[i].media_type)
            {
              value[valcnt] = content_types[i].code;
              valcnt++;
            }
          else
            {
              warn("W: unknown content-type '%s'\n", arg);
            }
        }

      if (!p || key == COAP_OPTION_CONTENT_TYPE)
        break;

      q = p + 1;
    }

  for (i = 0; i < valcnt; ++i)
    {
      node = new_option_node(key, coap_encode_var_bytes(buf, value[i]), buf);
      if (node)
        coap_insert(&g.optlist, node, order_opts);
    }
}

static void cmdline_uri(char *arg)
{
  unsigned char portbuf[2];
#define BUFSIZE 40
  unsigned char _buf[BUFSIZE];
  unsigned char *buf = _buf;
  size_t buflen;
  int res;

  if (g.proxy.length)
    { /* create Proxy-Uri from argument */
      size_t len = strlen(arg);
      while (len > 270)
        {
          coap_insert(
              &g.optlist,
              new_option_node(COAP_OPTION_PROXY_URI, 270,
                              (unsigned char *) arg),
              order_opts);
          len -= 270;
          arg += 270;
        }

      coap_insert(
          &g.optlist,
          new_option_node(COAP_OPTION_PROXY_URI, len, (unsigned char *) arg),
          order_opts);
    }
  else
    { /* split arg into Uri-* options */
      coap_split_uri((unsigned char *) arg, strlen(arg), &g.uri);

      if (g.uri.port != COAP_DEFAULT_PORT)
        {
          coap_insert(
              &g.optlist,
              new_option_node(COAP_OPTION_URI_PORT,
                              coap_encode_var_bytes(portbuf, g.uri.port),
                              portbuf),
              order_opts);
        }

      if (g.uri.path.length)
        {
          buflen = BUFSIZE;
          res = coap_split_path(g.uri.path.s, g.uri.path.length, buf, &buflen);

          while (res--)
            {
              coap_insert(
                  &g.optlist,
                  new_option_node(COAP_OPTION_URI_PATH, COAP_OPT_LENGTH(buf),
                                  COAP_OPT_VALUE(buf)),
                  order_opts);

              buf += COAP_OPT_SIZE(buf);
            }
        }

      if (g.uri.query.length)
        {
          buflen = BUFSIZE;
          buf = _buf;
          res = coap_split_query(g.uri.query.s, g.uri.query.length, buf, &buflen);

          while (res--)
            {
              coap_insert(
                  &g.optlist,
                  new_option_node(COAP_OPTION_URI_QUERY, COAP_OPT_LENGTH(buf),
                                  COAP_OPT_VALUE(buf)),
                  order_opts);

              buf += COAP_OPT_SIZE(buf);
            }
        }
    }
}

static int cmdline_blocksize(char *arg)
{
  unsigned short size;

again:
  size = 0;
  while (*arg && *arg != ',')
    size = size * 10 + (*arg++ - '0');

  if (*arg == ',')
    {
      arg++;
      g.block.num = size;
      goto again;
    }

  if (size)
    g.block.szx = (coap_fls(size >> 4) - 1) & 0x07;

  g.flags |= FLAGS_BLOCK;
  return 1;
}

/* Called after processing the options from the commandline to set
 * Block1 or Block2 depending on method. */
static void set_blocksize(void)
{
  unsigned char buf[4]; /* hack: temporarily take encoded bytes */
  unsigned short opt;
  unsigned int opt_length;

  if (g.method != COAP_REQUEST_DELETE)
    {
      opt =
          g.method == COAP_REQUEST_GET ? COAP_OPTION_BLOCK2 : COAP_OPTION_BLOCK1;

      g.block.m = (opt == COAP_OPTION_BLOCK1)
          && ((1 << (g.block.szx + 4)) < g.payload.length);

      opt_length = coap_encode_var_bytes(
          buf, (g.block.num << 4 | g.block.m << 3 | g.block.szx));

      coap_insert(&g.optlist, new_option_node(opt, opt_length, buf), order_opts);
    }
}

static void cmdline_subscribe(char *arg)
{
  g.obs_seconds = atoi(optarg);
  coap_insert(&g.optlist, new_option_node(COAP_OPTION_SUBSCRIPTION, 0, NULL),
              order_opts);
}

static int cmdline_proxy(char *arg)
{
  char *proxy_port_str = strrchr((const char *) arg, ':'); /* explicit port ? */
  if (proxy_port_str)
    {
      char *ipv6_delimiter = strrchr((const char *) arg, ']');
      if (!ipv6_delimiter)
        {
          if (proxy_port_str == strchr((const char *) arg, ':'))
            {
              /* host:port format - host not in ipv6 hexadecimal string format */
              *proxy_port_str++ = '\0'; /* split */
              g.proxy_port = atoi(proxy_port_str);
            }
        }
      else
        {
          arg = strchr((const char *) arg, '[');
          if (!arg)
            return 0;
          arg++;
          *ipv6_delimiter = '\0'; /* split */
          if (ipv6_delimiter + 1 == proxy_port_str++)
            {
              /* [ipv6 address]:port */
              g.proxy_port = atoi(proxy_port_str);
            }
        }
    }

  g.proxy.length = strlen(arg);
  if ((g.proxy.s = coap_malloc(g.proxy.length + 1)) == NULL)
    {
      g.proxy.length = 0;
      return 0;
    }

  memcpy(g.proxy.s, arg, g.proxy.length + 1);
  return 1;
}

static inline void cmdline_token(char *arg)
{
  snprintf((void*)g_token_data, sizeof(g_token_data), "%s", arg);
  g.the_token.s = g_token_data;
  g.the_token.length = strlen((void*)g_token_data);
}

static void cmdline_option(char *arg)
{
  unsigned int num = 0;

  while (*arg && *arg != ',')
    {
      num = num * 10 + (*arg - '0');
      ++arg;
    }
  if (*arg == ',')
    ++arg;

  coap_insert(&g.optlist,
              new_option_node(num, strlen(arg), (unsigned char *) arg),
              order_opts);
}

/* FIXME This is ugly accesses to libcoap internals. */
extern int coap_check_segment(const unsigned char *s, size_t length);
extern void coap_decode_segment(const unsigned char *seg, size_t length,
                                unsigned char *buf);

static int cmdline_input(char *text, str *buf)
{
  int len;
  len = coap_check_segment((unsigned char *) text, strlen(text));

  if (len < 0)
    return 0;

  buf->s = (unsigned char *) coap_malloc(len);
  if (!buf->s)
    return 0;

  buf->length = len;
  coap_decode_segment((unsigned char *) text, strlen(text), buf->s);
  return 1;
}

static int cmdline_input_from_file(char *filename, str *buf)
{
  FILE *inputfile = NULL;
  ssize_t len;
  int result = 1;
  struct stat statbuf;

  if (!filename || !buf)
    return 0;

  if (filename[0] == '-' && !filename[1])
    { /* read from stdin */
      buf->length = 20000;
      buf->s = (unsigned char *) coap_malloc(buf->length);
      if (!buf->s)
        return 0;

      inputfile = stdin;
    }
  else
    {
      /* read from specified input file */
      if (stat(filename, &statbuf) < 0)
        {
          perror("cmdline_input_from_file: stat");
          return 0;
        }

      buf->length = statbuf.st_size;
      buf->s = (unsigned char *) coap_malloc(buf->length);
      if (!buf->s)
        return 0;

      inputfile = fopen(filename, "r");
      if (!inputfile)
        {
          perror("cmdline_input_from_file: fopen");
          coap_free(buf->s);
          return 0;
        }
    }

  len = fread(buf->s, 1, buf->length, inputfile);

  if (len < buf->length)
    {
      if (ferror(inputfile) != 0)
        {
          perror("cmdline_input_from_file: fread");
          coap_free(buf->s);
          buf->length = 0;
          buf->s = NULL;
          result = 0;
        }
      else
        {
          buf->length = len;
        }
    }

  if (inputfile != stdin)
    fclose(inputfile);

  return result;
}

static method_t cmdline_method(char *arg)
{
  static const char *methods[] = { 0, "get", "post", "put", "delete", 0 };
  unsigned char i;

  for (i = 1; methods[i] && strcasecmp(arg, methods[i]) != 0; ++i)
    ;

  return i; /* note that we do not prevent illegal methods */
}

static coap_context_t *
get_context(void)
{
  coap_address_t addr;

  coap_address_init(&addr);
  addr.size = sizeof(addr.addr.sin);
  addr.addr.sin.sin_family = AF_INET;
  addr.addr.sin.sin_port = 0;
  addr.addr.sin.sin_addr.s_addr = INADDR_ANY;

  return coap_new_context(&addr);
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int coap_cli_main(int argc, char *argv[])
#endif
{
  coap_context_t *ctx = NULL;
  coap_address_t dst;
  char addr[INET6_ADDRSTRLEN] = {};
  void *addrptr = NULL;
  int result;
  coap_tick_t now;
  coap_queue_t *nextpdu;
  coap_pdu_t *pdu;
  str server = {};
  unsigned short port = COAP_DEFAULT_PORT;
  int opt, res;
  coap_log_t log_level = LOG_WARNING;
  coap_tid_t tid = COAP_INVALID_TID;
  struct pollfd pfd;
  int retval = 0;

  g = g_coap_cli_defaults;

  while ((opt = getopt(argc, argv, "Nb:e:f:m:s:t:o:v:A:B:O:P:T:")) != -1)
    {
      switch (opt)
        {
        case 'b':
          cmdline_blocksize(optarg);
          break;
        case 'B':
          g.wait_seconds = atoi(optarg);
          break;
        case 'e':
          if (!cmdline_input(optarg, &g.payload))
            g.payload.length = 0;
          break;
        case 'f':
          if (!cmdline_input_from_file(optarg, &g.payload))
            g.payload.length = 0;
          break;
        case 'm':
          g.method = cmdline_method(optarg);
          break;
        case 'N':
          g.msgtype = COAP_MESSAGE_NON;
          break;
        case 's':
          cmdline_subscribe(optarg);
          break;
        case 'o':
          g.output_file.length = strlen(optarg);
          g.output_file.s = (unsigned char *) coap_malloc(g.output_file.length + 1);

          if (!g.output_file.s)
            {
              fprintf(stderr, "cannot set output file: insufficient memory\n");
              retval = -1;
              goto out;
            }
          else
            {
              /* copy filename including trailing zero */
              memcpy(g.output_file.s, optarg, g.output_file.length + 1);
            }
          break;
        case 'A':
          cmdline_content_type(optarg, COAP_OPTION_ACCEPT);
          break;
        case 't':
          cmdline_content_type(optarg, COAP_OPTION_CONTENT_TYPE);
          break;
        case 'O':
          cmdline_option(optarg);
          break;
        case 'P':
          if (!cmdline_proxy(optarg))
            {
              fprintf(stderr, "error specifying proxy address\n");
              retval = -1;
              goto out;
            }
          break;
        case 'T':
          cmdline_token(optarg);
          break;
        case 'v':
          log_level = strtol(optarg, NULL, 10);
          break;
        default:
          usage(argv[0], PACKAGE_VERSION);
          retval = 1;
          goto out;
        }
    }

  coap_set_log_level(log_level);

  if (optind < argc)
    cmdline_uri(argv[optind]);
  else
    {
      usage(argv[0], PACKAGE_VERSION);
      retval = 1;
      goto out;
    }

  if (g.proxy.length)
    {
      server = g.proxy;
      port = g.proxy_port;
    }
  else
    {
      server = g.uri.host;
      port = g.uri.port;
    }

  /* resolve destination address where server should be sent */
  res = resolve_address(&server, &dst.addr.sa);

  if (res < 0)
    {
      fprintf(stderr, "failed to resolve address\n");
      retval = -1;
      goto out;
    }

  dst.size = res;
  dst.addr.sin.sin_port = htons(port);

  /* add Uri-Host if server address differs from uri.host */

  switch (dst.addr.sa.sa_family)
    {
    case AF_INET:
      addrptr = &dst.addr.sin.sin_addr;

      /* create context for IPv4 */
      ctx = get_context();
      break;
#if 0
      case AF_INET6:
      addrptr = &dst.addr.sin6.sin6_addr;

      /* create context for IPv6 */
      ctx = get_context();
      break;
#endif
    default:
      ;
    }

  if (!ctx)
    {
      coap_log(LOG_EMERG, "cannot create context\n");
      retval = -1;
      goto out;
    }

  coap_register_option(ctx, COAP_OPTION_BLOCK2);
  coap_register_response_handler(ctx, message_handler);

  /* construct CoAP message */

  if (!g.proxy.length && addrptr
      && (inet_ntop(dst.addr.sa.sa_family, addrptr, addr, sizeof(addr)) != 0)
      && (strlen(addr) != g.uri.host.length || memcmp(addr, g.uri.host.s,
                                                      g.uri.host.length)
                                             != 0))
    {
      /* add Uri-Host */

      coap_insert(
          &g.optlist,
          new_option_node(COAP_OPTION_URI_HOST, g.uri.host.length, g.uri.host.s),
          order_opts);
    }

  /* set block option if requested at commandline */
  if (g.flags & FLAGS_BLOCK)
    set_blocksize();

  if (!(pdu = coap_new_request(ctx, g.method, g.optlist, g.payload.s, g.payload.length)))
    {
      retval = -1;
      goto out;
    }

#ifndef NDEBUG
  if (LOG_DEBUG <= coap_get_log_level())
    {
      debug("sending CoAP request:\n");
      coap_show_pdu(pdu);
    }
#endif

  if (pdu->hdr->type == COAP_MESSAGE_CON)
    tid = coap_send_confirmed(ctx, ctx->endpoint, &dst, pdu);
  else
    tid = coap_send(ctx, ctx->endpoint, &dst, pdu);

  if (pdu->hdr->type != COAP_MESSAGE_CON || tid == COAP_INVALID_TID)
    coap_delete_pdu(pdu);

  set_timeout(&g.max_wait, g.wait_seconds);
  debug("timeout is set to %d seconds\n", g.wait_seconds);

  memset(&pfd, 0, sizeof(pfd));
  pfd.fd = ctx->sockfd;

  while (!(g.ready && coap_can_exit(ctx)))
    {
      unsigned int timeout_msec = -1;

      pfd.fd = ctx->sockfd;
      pfd.events = POLLIN;
      pfd.revents = 0;

      nextpdu = coap_peek_next(ctx);

      coap_ticks(&now);
      while (nextpdu && nextpdu->t <= now - ctx->sendqueue_basetime)
        {
          coap_retransmit(ctx, coap_pop_next(ctx));
          nextpdu = coap_peek_next(ctx);
        }

      if (nextpdu && nextpdu->t
          < min(g.obs_wait ? g.obs_wait : g.max_wait, g.max_wait) - now)
        {
          /* set timeout if there is a pdu to send */
          timeout_msec = nextpdu->t * 1000 / COAP_TICKS_PER_SECOND;
        }
      else
        {
          /* check if obs_wait fires before max_wait */
          if (g.obs_wait && g.obs_wait < g.max_wait)
            {
              timeout_msec = (g.obs_wait - now) * 1000 / COAP_TICKS_PER_SECOND;
            }
          else
            {
              timeout_msec = (g.max_wait - now) * 1000 / COAP_TICKS_PER_SECOND;
            }
        }

      result = poll(&pfd, 1, timeout_msec);

      if (result < 0)
        { /* error */
          perror("poll");
        }
      else if (result > 0)
        { /* read from socket */
          if (pfd.revents & ~POLLOUT)
            {
              coap_read(ctx); /* read received data */
              /* coap_dispatch( ctx );	/\* and dispatch PDUs from receivequeue *\/ */
            }
        }
      else
        { /* timeout */
          coap_ticks(&now);
          if (g.max_wait <= now)
            {
              info("timeout\n");
              break;
            }
          if (g.obs_wait && g.obs_wait <= now)
            {
              debug("clear observation relationship\n");
              clear_obs(ctx, ctx->endpoint, &dst); /* FIXME: handle error case COAP_TID_INVALID */

              /* make sure that the obs timer does not fire again */
              g.obs_wait = 0;
              g.obs_seconds = 0;
            }
        }
    }

  retval = 0;

out:
  close_output();

  coap_free_context(ctx);

  coap_delete_list(g.optlist);
  g.optlist = NULL;

  coap_free(g.output_file.s);
  g.output_file.s = NULL;

  coap_free(g.payload.s);
  g.payload.s = NULL;

  coap_free(g.proxy.s);
  g.proxy.s = NULL;

  return retval;
}
