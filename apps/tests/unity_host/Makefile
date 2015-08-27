-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

HOSTOBJEXT ?= .hobj

HOSTSRCS = \
	unity_fixture_Test.c unity_fixture_TestRunner.c unity_output_Spy.c \
	unity.c unity_fixture.c main/AllTests.c main.c
HOSTOBJS = $(HOSTSRCS:.c=$(HOSTOBJEXT))

HOSTCFLAGS += -pthread -isystem $(TOPDIR)/include
HOSTLDFLAGS += -pthread

HOST_BIN := unity_host
INSTALLED_BIN := $(TOPDIR)/../tests/apps/$(HOST_BIN)

ROOTDEPPATH	= --dep-path .

.PHONY: depend clean distclean all context install

$(HOSTOBJS): %$(HOSTOBJEXT): %.c
	$(call HOSTCOMPILE, $<, $@)

install:

context:

depend : .depend

.depend: Makefile $(HOSTSRCS)
	$(Q) $(MKDEP) $(ROOTDEPPATH) "$(HOSTCC)" -- $(HOSTCFLAGS) -- $(HOSTSRCS) >Make.dep
	$(Q) touch $@

$(HOST_BIN) : $(HOSTOBJS)
	$(Q) $(HOSTCC) $(HOSTLDFLAGS) $^ -o $@

$(INSTALLED_BIN) : $(HOST_BIN)
	$(Q) install $< $@

all: $(INSTALLED_BIN)

clean:
	$(call DELFILE, $(HOST_BIN))
	$(call DELFILE, $(INSTALLED_BIN))
	$(call DELFILE, $(HOSTOBJS))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
