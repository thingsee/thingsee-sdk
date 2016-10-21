############################################################################
# apps/netutils/mbedtls/library/Makefile
#
#   Copyright (C) 2015 Haltian Ltd.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

# mbedTLS NuttX integration

CSRCS_CRYPTO = \
    aes.c \
    aesni.c \
    arc4.c \
    asn1parse.c \
    asn1write.c \
    base64.c \
    bignum.c \
    blowfish.c \
    camellia.c \
    ccm.c \
    cipher.c \
    cipher_wrap.c \
    ctr_drbg.c \
    des.c \
    dhm.c \
    ecdh.c \
    ecdsa.c \
    ecjpake.c \
    ecp.c \
    ecp_curves.c \
    entropy.c \
    entropy_poll.c \
    error.c \
    gcm.c \
    havege.c \
    hmac_drbg.c \
    md.c \
    md2.c \
    md4.c \
    md5.c \
    md_wrap.c \
    memory_buffer_alloc.c \
    oid.c \
    padlock.c \
    pem.c \
    pk.c \
    pk_wrap.c \
    pkcs12.c \
    pkcs5.c \
    pkparse.c \
    pkwrite.c \
    platform.c \
    ripemd160.c \
    rsa.c \
    sha1.c \
    sha256.c \
    sha512.c \
    threading.c \
    timing.c \
    version.c \
    version_features.c \
    xtea.c

CSRCS_X509 = \
    certs.c \
    pkcs11.c \
    x509.c \
    x509_create.c \
    x509_crl.c \
    x509_crt.c \
    x509_csr.c \
    x509write_crt.c \
    x509write_csr.c

CSRCS_TLS = \
    debug.c \
    net.c \
    ssl_cache.c \
    ssl_ciphersuites.c \
    ssl_cli.c \
    ssl_cookie.c \
    ssl_srv.c \
    ssl_ticket.c \
    ssl_tls.c

ASRCS  =
CSRCS  = $(CSRCS_CRYPTO) $(CSRCS_X509) $(CSRCS_TLS)

CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" \
                  $(APPDIR)$(DELIM)include$(DELIM)netutils$(DELIM)ssl}

AOBJS		= $(ASRCS:.S=_mbedtls$(OBJEXT))
COBJS		= $(CSRCS:.c=_mbedtls$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BIN		= ..\..\..\libapps$(LIBEXT)
else
ifeq ($(WINTOOL),y)
  BIN		= ..\\..\\..\\libapps$(LIBEXT)
else
  BIN		= ../../../libapps$(LIBEXT)
endif
endif

ROOTDEPPATH	= --dep-path .

# Common build

VPATH		=

all: .built
.PHONY: clean depend distclean

$(AOBJS): %_mbedtls$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %_mbedtls$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

.built: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
	@touch .built

context:

.depend: Makefile $(SRCS)
	@$(MKDEP) --obj-path "." --obj-suffix "_mbedtls$(OBJEXT)" $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	$(call DELFILE, .built)
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
