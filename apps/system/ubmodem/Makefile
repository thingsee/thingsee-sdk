############################################################################
# apps/system/ubmodem/Makefile
#
#   Copyright (C) 2014-2015 Haltian Ltd. All rights reserved.
#   Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

ASRCS		=
CSRCS		= ubmodem_main.c ubmodem_initialize.c ubmodem_command.c \
			  ubmodem_parser.c ubmodem_util.c ubmodem_substate_cmdprompt.c \
			  ubmodem_substate_sim.c ubmodem_substate_poweron.c \
			  ubmodem_substate_poweroff.c ubmodem_substate_network.c \
			  ubmodem_substate_gprs.c ubmodem_cell_locate.c ubmodem_poll.c \
			  ubmodem_info.c ubmodem_cmd_task.c ubmodem_filesystem.c \
			  ubmodem_cell_environment.c

ifeq ($(CONFIG_UBMODEM_SMS_ENABLED),y)
	CSRCS	+= ubmodem_sms_send.c
	CSRCS	+= ubmodem_pdu_util.c
endif

ifeq ($(CONFIG_UBMODEM_FTP_ENABLED),y)
	CSRCS	+= ubmodem_ftp_download.c
endif

ifeq ($(CONFIG_UBMODEM_VOICE),y)
	CSRCS	+= ubmodem_voice_config.c
	CSRCS	+= ubmodem_voice_control.c
	CSRCS	+= ubmodem_voice_audio.c
	CSRCS	+= ubmodem_voice_audio_additional.c
	CSRCS	+= ubmodem_voice_config_additional.c
endif

ifeq ($(CONFIG_UBMODEM_USRSOCK),y)
	CSRCS	+= ubmodem_usrsock.c
	CSRCS	+= ubmodem_usrsock_socket.c
	CSRCS	+= ubmodem_usrsock_close.c
	CSRCS	+= ubmodem_usrsock_connect.c
	CSRCS	+= ubmodem_usrsock_sendto.c
	CSRCS	+= ubmodem_usrsock_recvfrom.c
	CSRCS	+= ubmodem_usrsock_setgetsock.c
	CSRCS	+= ubmodem_usrsock_bind.c
endif

CSRCS		+= $(wildcard ubmodem_external_*.c)

AOBJS		= $(ASRCS:.S=$(OBJEXT))
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BIN		= ..\..\libapps$(LIBEXT)
else
ifeq ($(WINTOOL),y)
  BIN		= ..\\..\\libapps$(LIBEXT)
else
  BIN		= ../../libapps$(LIBEXT)
endif
endif

ROOTDEPPATH	= --dep-path .

# Common build

VPATH		=

all: .built
.PHONY: context depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

.built: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
	$(Q) touch .built

install:

context:

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, .built)
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
