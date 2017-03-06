############################################################################
# apps/thingsee/engine/Makefile
#
#   Copyright (C) 2014-2016 Haltian Ltd.
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

# Thingsee engine

ASRCS  =
CSRCS  = main.c parse.c parse_labels.c execute.c sense.c util.c client.c
CSRCS += shutdown.c
CSRCS += log.c
CSRCS += alloc_dbg.c
CSRCS += time_from_file.c
CSRCS += value.c
CSRCS += geofence.c
CSRCS += system_config.c
CSRCS += $(wildcard *button.c)

ifeq ($(CONFIG_ARCH_SIM),y)
  CSRCS += sense_sim.c
else
  CSRCS += sense_group_hw_keys.c
  CSRCS += sense_group_acceleration.c
  CSRCS += sense_group_orientation.c
  CSRCS += sense_group_location.c
  CSRCS += sense_group_energy.c
  CSRCS += sense_group_time.c
  CSRCS += sense_ambient_light.c
  CSRCS += sense_hts221.c
  CSRCS += sense_lps25h.c
  CSRCS += device_property.c
  CSRCS += cloud_property.c
  CSRCS += hwwdg.c
  CSRCS += watchdog.c

  CSRCS += ../../thingsee/nineaxls/lsm9ds1_module.c # FIXME
  CSRCS += ../../thingsee/nineaxls/ts_nineaxls_fusion.c # FIXME
  CSRCS += ../../thingsee/nineaxls/ts_nineaxls_quaternion.c # FIXME

  ifeq ($(CONFIG_THINGSEE_UI), y)
    CSRCS += ../../thingsee/ui/thingsee_ui.c # FIXME
    CSRCS += ../../thingsee/ui/ui_bitmaps.c # FIXME
    CSRCS += ../../thingsee/ui/ui_battery.c # FIXME
    CFLAGS += -I../../thingsee/ui # FIXME
  endif
  
  CSRCS += ../../system/ubgps/ubgps.c # FIXME
  CSRCS += ../../system/ubgps/ubgps_internal.c # FIXME
  CSRCS += ../../system/ubgps/ubgps_state.c # FIXME
  CSRCS += ../../system/ubgps/ubx.c # FIXME
endif

ifeq ($(CONFIG_THINGSEE_CONNECTORS),y)
  CSRCS  += connector.c
endif

JSONS = profile.jsn

JSONCSRCS	= $(JSONS:.jsn=.c)
CSRCS		+= $(JSONCSRCS)

CFLAGS += -I..

AOBJS		= $(ASRCS:.S=$(OBJEXT))
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS) $(JSONS)
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

# built-in application info

APPNAME = ts_engine
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = (4 * 1024)

# Common build

VPATH		=

all: .built
.PHONY: clean depend distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(JSONCSRCS): %.c: %.jsn
	cat $< | tr -d ' \n\r' > $<.tmp && /bin/echo -n -e \\x00 >> $<.tmp && xxd -i $<.tmp $@ && rm $<.tmp
	sed -e 's/unsigned/const unsigned/g' $@ > $@.tmp && mv $@.tmp $@

.built: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
	@touch .built

ifeq ($(CONFIG_NSH_BUILTIN_APPS),y)
$(BUILTIN_REGISTRY)$(DELIM)$(APPNAME)_main.bdat: $(DEPCONFIG) Makefile
	$(call REGISTER,$(APPNAME),$(PRIORITY),$(STACKSIZE),$(APPNAME)_main)

context: $(BUILTIN_REGISTRY)$(DELIM)$(APPNAME)_main.bdat
else
context:
endif

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
