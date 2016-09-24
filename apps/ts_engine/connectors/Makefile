############################################################################
# apps/thingsee/connectors/Makefile
#
#   Copyright (C) 2014 Haltian Ltd.
#   Author: Pekka Ervasti <pekka.ervasti@haltian.com>
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

CFLAGS += -I../..

ifeq ($(CONFIG_THINGSEE_MQTT_PROTOCOL),y)
CFLAGS += -I$(APPDIR)/netutils/mqtt/MQTTClient-C/src -I$(APPDIR)/netutils/mqtt/MQTTPacket/src \
		  -I$(APPDIR)/netutils/mqtt/MQTTClient-C/src/nuttx
endif

ifeq ($(CONFIG_THINGSEE_HTTPS_PROTOCOL),y)
CFLAGS += -I $(APPDIR)/include/netutils/ssl
endif

# Thingsee connectors

ASRCS  =
CSRCS  =

ifneq ($(CONFIG_ONLY_CONNECTOR_CORE_LIBRARY),y)
# Concrete connector implementations are overlaid to this folder in the workarea
CONNECTOR_SRCS  += $(wildcard *_connector.c)
CONNECTOR_EXT_STUB_SRCS  += $(wildcard *_connext_stub.c)
CONNECTOR_EXT_SRCS = $(foreach EXT_STUB_SRC,$(CONNECTOR_EXT_STUB_SRCS), \
			$(if $(filter "$(wildcard $(subst _stub.c,.c,$(EXT_STUB_SRC)))",""), \
				$(EXT_STUB_SRC), \
				$(subst _stub.c,.c,$(EXT_STUB_SRC))))
CSRCS += $(CONNECTOR_SRCS) $(CONNECTOR_EXT_SRCS) connectors.c connector_util.c
endif

CSRCS += conn_comm.c conn_comm_stream.c conn_comm_link.c conn_comm_util.c
CSRCS += conn_comm_execute_http.c conn_comm_execute_mqtt.c

AOBJS		= $(ASRCS:.S=$(OBJEXT))
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

CFLAGS += -I..

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
.PHONY: clean depend distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

.built: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
	@touch .built

context:

.depend: Makefile $(SRCS)
	@$(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	$(call DELFILE, .built)
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
