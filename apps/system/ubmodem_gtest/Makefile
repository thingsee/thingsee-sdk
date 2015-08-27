-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

HOSTOBJEXT ?= .hobj

HOSTCSRCS := ../ubmodem/ubmodem_pdu_util.c
HOSTCXXSRCS := platform.cc sms_util_test.cc

HOSTCOBJS		= $(HOSTCSRCS:.c=$(HOSTOBJEXT))
HOSTCXXOBJS		= $(HOSTCXXSRCS:.cc=$(HOSTOBJEXT))

HOSTSRCS		= $(HOSTCSRCS) $(HOSTCXXSRCS)
HOSTOBJS		= $(HOSTCOBJS) $(HOSTCXXOBJS)

HOSTCFLAGS += -I../ubmodem
HOSTCXXFLAGS += -pthread -I../ubmodem
HOSTLDFLAGS += -pthread

HOST_BIN := ubmodem_ut
INSTALLED_HOST_BIN := $(TOPDIR)/../tests/apps/$(HOST_BIN)

ROOTDEPPATH	= --dep-path .

.PHONY: depend clean distclean all context

$(HOSTCOBJS): %$(HOSTOBJEXT): %.c
	$(call HOSTCOMPILE, $<, $@)

$(HOSTCXXOBJS): %$(HOSTOBJEXT): %.cc
	$(call HOSTCOMPILEXX, $<, $@)

context:

depend : .depend

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(ROOTDEPPATH) "$(HOSTCC)" -- $(HOSTCFLAGS) -- $(HOSTCSRCS) >Make.dep
	$(Q) $(MKDEP) $(ROOTDEPPATH) "$(HOSTCXX)" -- $(HOSTCXXFLAGS) -- $(HOSTCXXSRCS) >>Make.dep
	$(Q) touch $@

all: $(INSTALLED_HOST_BIN)

$(INSTALLED_HOST_BIN) : $(HOST_BIN)
	$(Q) install $< $@

$(HOST_BIN) : $(HOSTOBJS)
	@echo "LD: $(HOST_BIN)"
	$(Q) $(HOSTCXX) $(HOSTLDFLAGS) $^ -o $@ -lgtest -lgtest_main

clean:
	$(call DELFILE, $(HOST_BIN))
	$(call DELFILE, $(HOSTOBJS))
	$(call DELFILE, $(INSTALLED_HOST_BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
