-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

HOSTOBJEXT ?= .hobj

HOSTCSRCS := ../json/cJSON.c
HOSTCXXSRCS := cJSON_test.cc empty_arrays.cc empty_objects.cc

HOSTCOBJS		= $(HOSTCSRCS:.c=$(HOSTOBJEXT))
HOSTCXXOBJS		= $(HOSTCXXSRCS:.cc=$(HOSTOBJEXT))

HOSTSRCS		= $(HOSTCSRCS) $(HOSTCXXSRCS)
HOSTOBJS		= $(HOSTCOBJS) $(HOSTCXXOBJS)

HOSTINCLUDES += -I../json
HOSTCFLAGS += -isystem $(TOPDIR)/include
HOSTCXXFLAGS += -pthread -I$(TOPDIR)/include/apps/netutils
HOSTLDFLAGS += -pthread

HOST_BIN := json_ut
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
