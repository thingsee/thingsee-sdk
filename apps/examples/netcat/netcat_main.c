/* $OpenBSD: netcat.c,v 1.130 2015/07/26 19:12:28 chl Exp $ */
/*
 * Copyright (c) 2001 Eric Jackson <ericj@monkey.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD$
 */

/*
 * Re-written nc(1) for OpenBSD. Original implementation by
 * *Hobbit* <hobbit@avian.org>.
 */

/*
 * Port to NuttX by Jussi Kivilinna <jussi.kivilinna@haltian.com>
 */

#include <nuttx/config.h>
#include <nuttx/random.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>

#include <netinet/in.h>
#include <netinet/ip.h>
#include <arpa/inet.h>

#include <errno.h>
#include <getopt.h>
#include <fcntl.h>
#include <limits.h>
#include <netdb.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <apps/netutils/dnsclient.h>

#ifndef SUN_LEN
#define SUN_LEN(su) \
        (sizeof(*(su)) - sizeof((su)->sun_path) + strlen((su)->sun_path))
#endif

#define PORT_MAX                65535
#define PORT_MAX_LEN            6
#define UNIX_DG_TMP_SOCKET_SIZE 19

#define POLL_STDIN              0
#define POLL_NETOUT             1
#define POLL_NETIN              2
#define POLL_STDOUT             3
#define BUFSIZE                 1024


/* Command Line Options */
static int dflag; /* detached, no stdin */
static unsigned int iflag; /* Interval Flag */
static int kflag; /* More than one connect */
static int lflag; /* Bind to local port */
static int Nflag; /* shutdown() network socket */
static int nflag; /* Don't do name look up */
static char *pflag; /* Localport flag */
static int rflag; /* Random ports flag */
static char *sflag; /* Source Address */
static int uflag; /* UDP - Default to TCP */
static int vflag; /* Verbosity */
static int zflag; /* Port Scan Flag */
static int Dflag; /* sodebug */
static int Iflag; /* TCP receive buffer size */
static int Oflag; /* TCP send buffer size */
static char *fflag; /* Input file */

static int timeout = -1;
static int family = AF_UNSPEC;
static char *portlist[5 + 1];
static char *unix_dg_tmp_socket;


static void reset_state(void)
{
  dflag = 0;
  iflag = 0;
  kflag = 0;
  lflag = 0;
  Nflag = 0;
  nflag = 0;
  pflag = 0;
  rflag = 0;
  sflag = 0;
  uflag = 0;
  vflag = 0;
  zflag = 0;
  Dflag = 0;
  Iflag = 0;
  Oflag = 0;
  fflag = 0;
  timeout = -1;
  family = AF_UNSPEC;
  unix_dg_tmp_socket = NULL;
}


static void build_ports(char *);
static void help(void);
static int local_listen(char *, char *, struct addrinfo);
static void readwrite(int);
static int remote_connect(const char *, const char *, struct addrinfo);
static int timeout_connect(int, const struct sockaddr *, socklen_t);
static int udptest(int);
static int unix_bind(char *);
static int unix_connect(char *);
static int unix_listen(char *);
static void set_common_sockopts(int, int);
static void report_connect(const struct sockaddr *, socklen_t);
static void usage(int);
static ssize_t drainbuf(int, unsigned char *, size_t *);
static ssize_t fillbuf(int, unsigned char *, size_t *);

static void err(int eval, const char *fmt, ...)
{
  int errn = errno;
  va_list va;

  if (fmt)
    {
      va_start(va, fmt);
      vdprintf(2, fmt, va);
      va_end(va);
    }

  dprintf(2, " error=%d,\"%s\"\n", errn, strerror(errn));
  exit(eval);
}

static void errx(int eval, const char *fmt, ...)
{
  va_list va;

  if (fmt)
    {
      va_start(va, fmt);
      vdprintf(2, fmt, va);
      va_end(va);
    }

  exit(eval);
}

static long long strtonum(const char *num, long long min, long long max,
                          const char **errstr)
{
  long long val;

  *errstr = NULL;

  val = strtoll(num, NULL, 10);
  if (val < min)
    *errstr = "too small";
  else if (val > max)
    *errstr = "too large";

  return val;
}

#define SHUT_WR 0
#define SHUT_RD 1
static void shutdown(int fd, int how)
{
  /* do nothing */
}

static size_t strlcpy(char *dst, const char *src, size_t siz)
{
  register char *d = dst;
  register const char *s = src;
  register size_t n = siz;

  /* Copy as many bytes as will fit */
  if (n != 0 && --n != 0)
    {
      do
        {
          if ((*d++ = *s++) == 0)
            break;
        }
      while (--n != 0);
    }

  /* Not enough room in dst, add NUL and traverse rest of src */
  if (n == 0)
    {
      if (siz != 0)
        *d = '\0'; /* NUL-terminate dst */
      while (*s++)
        ;
    }

  return (s - src - 1); /* count does not include NUL */
}

int nc_main(int argc, char *argv[])
{
  int ch, s, ret, i;
  char *host, *uport;
  struct addrinfo hints;
  socklen_t len;
  struct sockaddr_storage cliaddr;
  const char *errstr;

  reset_state();
  ret = 1;
  s = 0;
  host = NULL;
  uport = NULL;

  while ((ch = getopt(argc, argv, "46Ddhf:I:i:klNnoO:p:rs:UuV:vw:z")) != -1)
    {
      switch (ch)
        {
        case '4':
          family = AF_INET;
          break;
        case '6':
          family = AF_INET6;
          break;
        case 'U':
          family = AF_UNIX;
          break;
        case 'd':
          dflag = 1;
          break;
        case 'h':
          help();
          break;
        case 'f':
          fflag = optarg;
          break;
        case 'i':
          iflag = strtonum(optarg, 0, UINT_MAX, &errstr);
          if (errstr)
            errx(1, "interval %s: %s", errstr, optarg);
          break;
        case 'k':
          kflag = 1;
          break;
        case 'l':
          lflag = 1;
          break;
        case 'N':
          Nflag = 1;
          break;
        case 'n':
          nflag = 1;
          break;
        case 'o':
          fprintf(stderr, "option -o is deprecated.\n");
          break;
        case 'p':
          pflag = optarg;
          break;
        case 'r':
          rflag = 1;
          break;
        case 's':
          sflag = optarg;
          break;
        case 'u':
          uflag = 1;
          break;
        case 'v':
          vflag = 1;
          break;
        case 'w':
          timeout = strtonum(optarg, 0, INT_MAX / 1000, &errstr);
          if (errstr)
            errx(1, "timeout %s: %s", errstr, optarg);
          timeout *= 1000;
          break;
          break;
        case 'z':
          zflag = 1;
          break;
        case 'D':
          Dflag = 1;
          break;
        case 'I':
          Iflag = strtonum(optarg, 1, 65536 << 14, &errstr);
          if (errstr != NULL)
            errx(1, "TCP receive window %s: %s", errstr, optarg);
          break;
        case 'O':
          Oflag = strtonum(optarg, 1, 65536 << 14, &errstr);
          if (errstr != NULL)
            {
              if (strcmp(errstr, "invalid") != 0)
                errx(1, "TCP send window %s: %s", errstr, optarg);
            }
          break;
        default:
          usage(1);
        }
    }
  argc -= optind;
  argv += optind;

  /* Cruft to make sure options are clean, and used properly. */
  if (argv[0] && !argv[1] && family == AF_UNIX)
    {
      host = argv[0];
      uport = NULL;
    }
  else if (argv[0] && !argv[1])
    {
      if (!lflag)
        usage(1);
      uport = argv[0];
      host = NULL;
    }
  else if (argv[0] && argv[1])
    {
      host = argv[0];
      uport = argv[1];
    }
  else
    usage(1);

  if (lflag && sflag)
    errx(1, "cannot use -s and -l");
  if (lflag && pflag)
    errx(1, "cannot use -p and -l");
  if (lflag && zflag)
    errx(1, "cannot use -z and -l");
  if (!lflag && kflag)
    errx(1, "must use -l with -k");

  /* open file for input */
  if (fflag)
    {
      int infd;

      infd = open(fflag, O_RDONLY);
      if (infd < 0)
        errx(1, "could not open %s for input", fflag);

      (void)fflush(stdin);
      (void)fclose(stdin);
      (void)dup2(infd, 0);
      (void)fdopen(0, "r");
    }

  /* Get name of temporary socket for unix datagram client */
  if ((family == AF_UNIX) && uflag && !lflag)
    {
      if (sflag)
        {
          unix_dg_tmp_socket = sflag;
        }
#if 0
      else
        {
          strlcpy(unix_dg_tmp_socket_buf, "/tmp/nc.XXXXXXXXXX",
          UNIX_DG_TMP_SOCKET_SIZE);
          if (mktemp(unix_dg_tmp_socket_buf) == NULL)
            err(1, "mktemp");
          unix_dg_tmp_socket = unix_dg_tmp_socket_buf;
        }
#endif
    }

  /* Initialize addrinfo structure. */
  if (family != AF_UNIX)
    {
      memset(&hints, 0, sizeof(struct addrinfo));
      hints.ai_family = family;
      hints.ai_socktype = uflag ? SOCK_DGRAM : SOCK_STREAM;
      hints.ai_protocol = uflag ? IPPROTO_UDP : IPPROTO_TCP;
      if (nflag)
        hints.ai_flags |= AI_NUMERICHOST;
    }

  if (lflag)
    {
      int connfd;
      ret = 0;

      if (family == AF_UNIX)
        {
          if (uflag)
            s = unix_bind(host);
          else
            s = unix_listen(host);
        }

      /* Allow only one connection at a time, but stay alive. */
      for (;;)
        {
          if (family != AF_UNIX)
            s = local_listen(host, uport, hints);
          if (s < 0)
            err(1, NULL);
          /*
           * For UDP and -k, don't connect the socket, let it
           * receive datagrams from multiple socket pairs.
           */
          if (uflag && kflag)
            readwrite(s);
          /*
           * For UDP and not -k, we will use recvfrom() initially
           * to wait for a caller, then use the regular functions
           * to talk to the caller.
           */
          else if (uflag && !kflag)
            {
              int rv, plen;
              char *buf;
              struct sockaddr_storage z;

              buf = malloc(4096);
              if (!buf)
                errx(1, "malloc failed");

              len = 4096;
              plen = 2048;
              rv = recvfrom(s, buf, plen, MSG_PEEK, (struct sockaddr *) &z,
                            &len);
              if (rv < 0)
                err(1, "recvfrom");

              rv = connect(s, (struct sockaddr *) &z, len);
              if (rv < 0)
                err(1, "connect");

              if (vflag)
                report_connect((struct sockaddr *) &z, len);

              readwrite(s);

              free(buf);
            }
          else
            {
              len = sizeof(cliaddr);
              connfd = accept(s, (struct sockaddr *) &cliaddr, &len);
              if (connfd == -1)
                {
                  /* For now, all errnos are fatal */
                  err(1, "accept");
                }
              if (vflag)
                report_connect((struct sockaddr *) &cliaddr, len);

              readwrite(connfd);
              close(connfd);
            }

          if (family != AF_UNIX)
            close(s);
          else if (uflag)
            {
              if (connect(s, NULL, 0) < 0)
                err(1, "connect");
            }

          if (!kflag)
            break;
        }
    }
  else if (family == AF_UNIX)
    {
      ret = 0;

      if ((s = unix_connect(host)) > 0 && !zflag)
        {
          readwrite(s);
          close(s);
        }
      else
        ret = 1;

      if (uflag)
        unlink(unix_dg_tmp_socket);
      exit(ret);

    }
  else
    {
      /* Construct the portlist[] array. */
      build_ports(uport);

      /* Cycle through portlist, connecting to each port. */
      for (i = 0; portlist[i] != NULL; i++)
        {
          if (s)
            close(s);

          s = remote_connect(host, portlist[i], hints);
          if (s < 0)
            continue;

          ret = 0;
          if (vflag || zflag)
            {
              /* For UDP, make sure we are connected. */
              if (uflag)
                {
                  if (udptest(s) == -1)
                    {
                      ret = 1;
                      continue;
                    }
                }

              fprintf(stderr, "Connection to %s %s port [%s/%s] "
                      "succeeded!\n",
                      host, portlist[i], uflag ? "udp" : "tcp",
                      nflag ? "*" : portlist[i]);
            }
          else if (!zflag)
            readwrite(s);
        }
    }

  if (s)
    close(s);

  for (i = 0; i < sizeof(portlist) / sizeof(portlist[0]); i++)
    {
      free(portlist[i]);
      portlist[i] = NULL;
    }

  exit(ret);
}

/*
 * unix_bind()
 * Returns a unix socket bound to the given path
 */
static int unix_bind(char *path)
{
  struct sockaddr_un sun;
  int s;

  /* Create unix domain socket. */
  if ((s = socket(AF_UNIX, uflag ? SOCK_DGRAM : SOCK_STREAM, 0)) < 0)
    return (-1);

  memset(&sun, 0, sizeof(struct sockaddr_un));
  sun.sun_family = AF_UNIX;

  if (strlcpy(sun.sun_path, path, sizeof(sun.sun_path)) >= sizeof(sun.sun_path))
    {
      close(s);
      errno = ENAMETOOLONG;
      return (-1);
    }

  if (bind(s, (struct sockaddr *) &sun, SUN_LEN(&sun)) < 0)
    {
      close(s);
      return (-1);
    }
  return (s);
}

/*
 * unix_connect()
 * Returns a socket connected to a local unix socket. Returns -1 on failure.
 */
static int unix_connect(char *path)
{
  struct sockaddr_un sun;
  int s;

  if (uflag)
    {
      if ((s = unix_bind(unix_dg_tmp_socket)) < 0)
        return (-1);
    }
  else
    {
      if ((s = socket(AF_UNIX, SOCK_STREAM, 0)) < 0)
        return (-1);
    }
  (void) fcntl(s, F_SETFD, FD_CLOEXEC);

  memset(&sun, 0, sizeof(struct sockaddr_un));
  sun.sun_family = AF_UNIX;

  if (strlcpy(sun.sun_path, path, sizeof(sun.sun_path)) >= sizeof(sun.sun_path))
    {
      close(s);
      errno = ENAMETOOLONG;
      return (-1);
    }
  if (connect(s, (struct sockaddr *) &sun, SUN_LEN(&sun)) < 0)
    {
      close(s);
      return (-1);
    }
  return (s);

}

/*
 * unix_listen()
 * Create a unix domain socket, and listen on it.
 */
static int unix_listen(char *path)
{
  int s;
  if ((s = unix_bind(path)) < 0)
    return (-1);

  if (listen(s, 5) < 0)
    {
      close(s);
      return (-1);
    }
  return (s);
}

/*
 * remote_connect()
 * Returns a socket connected to a remote host. Properly binds to a local
 * port or source address if needed. Returns -1 on failure.
 */
static int remote_connect(const char *host, const char *port, struct addrinfo hints)
{
  in_addr_t addrs[5];
  uint16_t portn;
  int naddr, i;
  int fd;

  portn = htons(atoi(port));
  naddr = dns_gethostip_multi(host, addrs, sizeof(addrs) / sizeof(addrs[0]));
  if (naddr <= 0)
    errx(1, "dns_gethostip_multi failed");

  for (i = 0; i < naddr; i++)
    {
      struct sockaddr_in addr;

      fd = socket(AF_INET, hints.ai_socktype, hints.ai_protocol);
      if (fd < 0)
        {
          continue;
        }

      addr.sin_family = AF_INET;
      addr.sin_port = portn;
      addr.sin_addr.s_addr = addrs[i];
      if (timeout_connect(fd, (const void *) &addr, sizeof(addr)) == 0)
        {
          break;
        }

      close(fd);
      fd = -1;
    }

  return (fd);
}

static int timeout_connect(int s, const struct sockaddr *name, socklen_t namelen)
{
  struct pollfd pfd;
  socklen_t optlen;
  int flags = 0, optval;
  int ret;

  if (timeout != -1)
    {
      flags = fcntl(s, F_GETFL, 0);
      if (fcntl(s, F_SETFL, flags | O_NONBLOCK) == -1)
        err(1, "set non-blocking mode");
    }

  if ((ret = connect(s, name, namelen)) != 0 && errno == EINPROGRESS)
    {
      pfd.fd = s;
      pfd.events = POLLOUT;
      if ((ret = poll(&pfd, 1, timeout)) == 1)
        {
          optlen = sizeof(optval);
          if ((ret = getsockopt(s, SOL_SOCKET, SO_ERROR, &optval, &optlen)) == 0)
            {
              errno = optval;
              ret = optval == 0 ? 0 : -1;
            }
        }
      else if (ret == 0)
        {
          errno = ETIMEDOUT;
          ret = -1;
        }
      else
        err(1, "poll failed");
    }

  if (timeout != -1 && fcntl(s, F_SETFL, flags) == -1)
    err(1, "restoring flags");

  return (ret);
}

/*
 * local_listen()
 * Returns a socket listening on a local port, binds to specified source
 * address. Returns -1 on failure.
 */
static int local_listen(char *host, char *port, struct addrinfo hints)
{
#if 1
  return -1;
#else
  struct addrinfo *res, *res0;
  int s, ret, x = 1;
  int error;

  /* Allow nodename to be null. */
  hints.ai_flags |= AI_PASSIVE;

  /*
   * In the case of binding to a wildcard address
   * default to binding to an ipv4 address.
   */
  if (host == NULL && hints.ai_family == AF_UNSPEC)
    hints.ai_family = AF_INET;

  if ((error = getaddrinfo(host, port, &hints, &res)))
    errx(1, "getaddrinfo: %s", gai_strerror(error));

  res0 = res;
  do
    {
      if ((s = socket(res0->ai_family, res0->ai_socktype, res0->ai_protocol)) < 0)
        continue;

      ret = setsockopt(s, SOL_SOCKET, SO_REUSEPORT, &x, sizeof(x));
      if (ret == -1)
        err(1, NULL);
      if (FreeBSD_Oflag)
        {
          if (setsockopt(s, IPPROTO_TCP, TCP_NOOPT, &FreeBSD_Oflag,
                         sizeof(FreeBSD_Oflag))
              == -1)
            err(1, "disable TCP options");
        }

      set_common_sockopts(s, res0->ai_family);

      if (bind(s, (struct sockaddr *) res0->ai_addr, res0->ai_addrlen) == 0)
        break;

      close(s);
      s = -1;
    }
  while ((res0 = res0->ai_next) != NULL);

  if (!uflag && s != -1)
    {
      if (listen(s, 1) < 0)
        err(1, "listen");
    }

  freeaddrinfo(res);

  return (s);
#endif
}

/*
 * readwrite()
 * Loop that polls on the network file descriptor and stdin.
 */
static void readwrite(int net_fd)
{
  struct pollfd pfd[4];
  int stdin_fd = 0;
  int stdout_fd = 1;
  unsigned char *netinbuf;
  size_t netinbufpos = 0;
  unsigned char *stdinbuf;
  size_t stdinbufpos = 0;
  int n, num_fds;
  ssize_t ret;

  netinbuf = malloc(BUFSIZE);
  if (!netinbuf)
    errx(1, "malloc failed");

  stdinbuf = malloc(BUFSIZE);
  if (!stdinbuf)
    errx(1, "malloc failed");

  /* don't read from stdin if requested */
  if (dflag)
    stdin_fd = -1;

  /* stdin */
  pfd[POLL_STDIN].fd = stdin_fd;
  pfd[POLL_STDIN].events = POLLIN;

  /* network out */
  pfd[POLL_NETOUT].fd = net_fd;
  pfd[POLL_NETOUT].events = 0;

  /* network in */
  pfd[POLL_NETIN].fd = net_fd;
  pfd[POLL_NETIN].events = POLLIN;

  /* stdout */
  pfd[POLL_STDOUT].fd = stdout_fd;
  pfd[POLL_STDOUT].events = 0;

  while (1)
    {
      /* both inputs are gone, buffers are empty, we are done */
      if (pfd[POLL_STDIN].fd == -1 && pfd[POLL_NETIN].fd == -1
          && stdinbufpos == 0 && netinbufpos == 0)
        {
          close(net_fd);
          goto freebufs;
        }
      /* both outputs are gone, we can't continue */
      if (pfd[POLL_NETOUT].fd == -1 && pfd[POLL_STDOUT].fd == -1)
        {
          close(net_fd);
          goto freebufs;
        }
      /* listen and net in gone, queues empty, done */
      if (lflag && pfd[POLL_NETIN].fd == -1 && stdinbufpos == 0
          && netinbufpos == 0)
        {
          close(net_fd);
          goto freebufs;
        }

      /* help says -i is for "wait between lines sent". We read and
       * write arbitrary amounts of data, and we don't want to start
       * scanning for newlines, so this is as good as it gets */
      if (iflag)
        sleep(iflag);

      /* poll */
      num_fds = poll(pfd, 4, timeout);

      /* treat poll errors */
      if (num_fds == -1)
        {
          close(net_fd);
          err(1, "polling error");
        }

      /* timeout happened */
      if (num_fds == 0)
        goto freebufs;

      /* treat socket error conditions */
      for (n = 0; n < 4; n++)
        {
          if (pfd[n].revents & (POLLERR | POLLNVAL))
            {
              pfd[n].fd = -1;
            }
        }
      /* reading is possible after HUP */
      if ((pfd[POLL_STDIN].events & POLLIN) && (pfd[POLL_STDIN].revents & POLLHUP)
          && !(pfd[POLL_STDIN].revents & POLLIN))
        pfd[POLL_STDIN].fd = -1;

      if ((pfd[POLL_NETIN].events & POLLIN) && (pfd[POLL_NETIN].revents & POLLHUP)
          && !(pfd[POLL_NETIN].revents & POLLIN))
        pfd[POLL_NETIN].fd = -1;

      if (pfd[POLL_NETOUT].revents & POLLHUP)
        {
          if (Nflag)
            shutdown(pfd[POLL_NETOUT].fd, SHUT_WR);
          pfd[POLL_NETOUT].fd = -1;
        }
      /* if HUP, stop watching stdout */
      if (pfd[POLL_STDOUT].revents & POLLHUP)
        pfd[POLL_STDOUT].fd = -1;
      /* if no net out, stop watching stdin */
      if (pfd[POLL_NETOUT].fd == -1)
        pfd[POLL_STDIN].fd = -1;
      /* if no stdout, stop watching net in */
      if (pfd[POLL_STDOUT].fd == -1)
        {
          if (pfd[POLL_NETIN].fd != -1)
            shutdown(pfd[POLL_NETIN].fd, SHUT_RD);
          pfd[POLL_NETIN].fd = -1;
        }

      /* try to read from stdin */
      if ((pfd[POLL_STDIN].revents & POLLIN) && stdinbufpos < BUFSIZE)
        {
          ret = fillbuf(pfd[POLL_STDIN].fd, stdinbuf, &stdinbufpos);
          /* error or eof on stdin - remove from pfd */
          if (ret == 0 || ret == -1)
            pfd[POLL_STDIN].fd = -1;
          /* read something - poll net out */
          if (stdinbufpos > 0)
            pfd[POLL_NETOUT].events = POLLOUT;
          /* filled buffer - remove self from polling */
          if (stdinbufpos == BUFSIZE)
            pfd[POLL_STDIN].events = 0;
        }
      /* try to write to network */
      if ((pfd[POLL_NETOUT].revents & POLLOUT) && stdinbufpos > 0)
        {
          ret = drainbuf(pfd[POLL_NETOUT].fd, stdinbuf, &stdinbufpos);
          if (ret == -1)
            pfd[POLL_NETOUT].fd = -1;
          /* buffer empty - remove self from polling */
          if (stdinbufpos == 0)
            pfd[POLL_NETOUT].events = 0;
          /* buffer no longer full - poll stdin again */
          if (stdinbufpos < BUFSIZE)
            pfd[POLL_STDIN].events = POLLIN;
        }
      /* try to read from network */
      if ((pfd[POLL_NETIN].revents & POLLIN) && netinbufpos < BUFSIZE)
        {
          ret = fillbuf(pfd[POLL_NETIN].fd, netinbuf, &netinbufpos);
          if (ret == -1)
            pfd[POLL_NETIN].fd = -1;
          /* eof on net in - remove from pfd */
          if (ret == 0)
            {
              shutdown(pfd[POLL_NETIN].fd, SHUT_RD);
              pfd[POLL_NETIN].fd = -1;
            }
          /* read something - poll stdout */
          if (netinbufpos > 0)
            pfd[POLL_STDOUT].events = POLLOUT;
          /* filled buffer - remove self from polling */
          if (netinbufpos == BUFSIZE)
            pfd[POLL_NETIN].events = 0;
        }
      /* try to write to stdout */
      if ((pfd[POLL_STDOUT].revents & POLLOUT) && netinbufpos > 0)
        {
          ret = drainbuf(pfd[POLL_STDOUT].fd, netinbuf, &netinbufpos);
          if (ret == -1)
            pfd[POLL_STDOUT].fd = -1;
          /* buffer empty - remove self from polling */
          if (netinbufpos == 0)
            pfd[POLL_STDOUT].events = 0;
          /* buffer no longer full - poll net in again */
          if (netinbufpos < BUFSIZE)
            pfd[POLL_NETIN].events = POLLIN;
        }

      /* stdin gone and queue empty? */
      if (pfd[POLL_STDIN].fd == -1 && stdinbufpos == 0)
        {
          if (pfd[POLL_NETOUT].fd != -1 && Nflag)
            shutdown(pfd[POLL_NETOUT].fd, SHUT_WR);
          pfd[POLL_NETOUT].fd = -1;
        }
      /* net in gone and queue empty? */
      if (pfd[POLL_NETIN].fd == -1 && netinbufpos == 0)
        {
          pfd[POLL_STDOUT].fd = -1;
        }
    }
freebufs:
  free(netinbuf);
  free(stdinbuf);
}

static ssize_t drainbuf(int fd, unsigned char *buf, size_t *bufpos)
{
  ssize_t n;
  ssize_t adjust;

  n = write(fd, buf, *bufpos);
  /* don't treat EAGAIN, EINTR as error */
  if (n == -1 && (errno == EAGAIN || errno == EINTR))
    n = -2;
  if (n <= 0)
    return n;
  /* adjust buffer */
  adjust = *bufpos - n;
  if (adjust > 0)
    memmove(buf, buf + n, adjust);
  *bufpos -= n;
  return n;
}

static ssize_t fillbuf(int fd, unsigned char *buf, size_t *bufpos)
{
  int nyield = 0;
  size_t total = 0;

  do
    {
      size_t num = BUFSIZE - *bufpos;
      ssize_t n;

      n = read(fd, buf + *bufpos, num);
      /* don't treat EAGAIN, EINTR as error */
      if (n == -1 && (errno == EAGAIN || errno == EINTR))
        n = -2;
      if (n <= 0)
        {
          if (total == 0)
            return n;
          else
            return total;
        }
      *bufpos += n;
      total += n;
      if (*bufpos == BUFSIZE)
        return total;
      if (++nyield >= 2)
        return total;
      usleep(1); /* Yield processing time to other process to get more input. */
    }
  while (1);
}

/*
 * build_ports()
 * Build an array of ports in portlist[], listing each port
 * that we should try to connect to.
 */
static void build_ports(char *p)
{
  const char *errstr;
  char *n;
  int hi, lo, cp, i;
  int x = 0;

  for (i = 0; i < sizeof(portlist) / sizeof(portlist[0]); i++)
    {
      free(portlist[i]);
      portlist[i] = NULL;
    }

  if ((n = strchr(p, '-')) != NULL)
    {
      *n = '\0';
      n++;

      /* Make sure the ports are in order: lowest->highest. */
      hi = strtonum(n, 1, PORT_MAX, &errstr);
      if (errstr)
        errx(1, "port number %s: %s", errstr, n);
      lo = strtonum(p, 1, PORT_MAX, &errstr);
      if (errstr)
        errx(1, "port number %s: %s", errstr, p);

      if (lo > hi)
        {
          cp = hi;
          hi = lo;
          lo = cp;
        }

      /* Load ports sequentially. */
      for (cp = lo; cp <= hi; cp++)
        {
          portlist[x] = calloc(1, PORT_MAX_LEN);
          if (portlist[x] == NULL)
            err(1, NULL);
          snprintf(portlist[x], PORT_MAX_LEN, "%d", cp);
          x++;
        }

      /* Randomly swap ports. */
      if (rflag)
        {
          int y;
          char *c;

          for (x = 0; x <= (hi - lo); x++)
            {
              unsigned int rnd;
              getrandom(&rnd, sizeof(rnd));
              y = (rnd & 0xFFFF) % (hi - lo);
              c = portlist[x];
              portlist[x] = portlist[y];
              portlist[y] = c;
            }
        }
    }
  else
    {
      hi = strtonum(p, 1, PORT_MAX, &errstr);
      if (errstr)
        errx(1, "port number %s: %s", errstr, p);
      portlist[0] = strdup(p);
      if (portlist[0] == NULL)
        err(1, NULL);
    }
}

/*
 * udptest()
 * Do a few writes to see if the UDP port is there.
 * Fails once PF state table is full.
 */
static int udptest(int s)
{
  int i, ret;

  for (i = 0; i <= 3; i++)
    {
      if (write(s, "X", 1) == 1)
        ret = 1;
      else
        ret = -1;
    }
  return (ret);
}

static void set_common_sockopts(int s, int af)
{
  int x = 1;

  if (Dflag)
    {
      if (setsockopt(s, SOL_SOCKET, SO_DEBUG, &x, sizeof(x)) == -1)
        err(1, NULL);
    }
  if (Iflag)
    {
      if (setsockopt(s, SOL_SOCKET, SO_RCVBUF, &Iflag, sizeof(Iflag)) == -1)
        err(1, "set TCP receive buffer size");
    }
  if (Oflag)
    {
      if (setsockopt(s, SOL_SOCKET, SO_SNDBUF, &Oflag, sizeof(Oflag)) == -1)
        err(1, "set TCP send buffer size");
    }
}

static void report_connect(const struct sockaddr *sa, socklen_t salen)
{
#if 0
  char remote_host[NI_MAXHOST];
  char remote_port[NI_MAXSERV];
  int herr;
  int flags = NI_NUMERICSERV;

  if (nflag)
    flags |= NI_NUMERICHOST;

  if ((herr = getnameinfo(sa, salen, remote_host, sizeof(remote_host),
                          remote_port, sizeof(remote_port), flags))
      != 0)
    {
      if (herr == EAI_SYSTEM)
        err(1, "getnameinfo");
      else
        errx(1, "getnameinfo: %s", gai_strerror(herr));
    }

  fprintf(stderr, "Connection from %s %s "
          "received!\n",
          remote_host, remote_port);
#endif
}

static void help(void)
{
  usage(0);
  fprintf(stderr, "\tCommand Summary:\n\
  \t-4            Use IPv4\n\
  \t-6            Use IPv6\n\
  \t-D            Enable the debug socket option\n\
  \t-d            Detach from stdin\n\
  \t-f file       Input file\n");
  fprintf(stderr, "\
  \t-h            This help text\n\
  \t-I length     TCP receive buffer length\n\
  \t-i secs\t     Delay interval for lines sent, ports scanned\n\
  \t-k            Keep inbound sockets open for multiple connects\n\
  \t-l            Listen mode, for inbound connects\n\
  \t-N            Shutdown the network socket after EOF on stdin\n\
  \t-n            Suppress name/port resolutions\n\
  \t--no-tcpopt   Disable TCP options\n\
  \t-O length     TCP send buffer length\n\
  \t-p port\t     Specify local port for remote connects\n\
  \t-r            Randomize remote ports\n\
  \t-s addr\t     Local source address\n\
  \t-U            Use UNIX domain socket\n\
  \t-u            UDP mode\n\
  \t-v            Verbose\n\
  \t-w secs\t     Timeout for connects and final net reads\n\
  \t-z            Zero-I/O mode [used for scanning]\n\
  Port numbers can be individual or ranges: lo-hi [inclusive]\n");
  exit(1);
}

static void usage(int ret)
{
  fprintf(
      stderr,
      "usage: nc [-46DdhklNnrUuvz] [-I length] [-i interval] [-O length]\n"
      "\t  [-p source_port] [-s source]\n"
      "\t  [-w timeout] [-f input_file] \n"
      "\t  [destination] [port]\n");
  if (ret)
    exit(1);
}
