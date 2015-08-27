############################################################################
# apps/ts_engine/Makefile
#
#   Copyright (C) 2015 Haltian ltd. All rights reserved.
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

-include $(TOPDIR)/.config	# Current configuration

# Sub-directories

SUBDIRS = engine

ifeq ($(CONFIG_THINGSEE_CONNECTOR_TS),y)
SUBDIRS += ts-connector
CNTXTDIRS += ts-connector
endif

ifeq ($(CONFIG_THINGSEE_CONNECTOR_KII),y)
SUBDIRS += kii-connector
CNTXTDIRS += kii-connector
endif

ifeq ($(CONFIG_THINGSEE_CONNECTOR_MESHBLU),y)
SUBDIRS += meshblu-connector
CNTXTDIRS += meshblu-connector
endif

# Create the list of installed runtime modules (INSTALLED_DIRS)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
define ADD_DIRECTORY
  INSTALLED_DIRS  += $(if $(wildcard .\$1\Makefile),$1,)
endef
else
define ADD_DIRECTORY
  INSTALLED_DIRS  += $(if $(wildcard ./$1/Makefile),$1,)
endef
endif

$(foreach DIR, $(SUBDIRS), $(eval $(call ADD_DIRECTORY,$(DIR))))

all: nothing
.PHONY: nothing context depend clean distclean

define SDIR_template
$(1)_$(2):
	$(Q) $(MAKE) -C $(1) $(2) TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)"
endef

$(foreach SDIR, $(INSTALLED_DIRS), $(eval $(call SDIR_template,$(SDIR),depend)))
$(foreach SDIR, $(INSTALLED_DIRS), $(eval $(call SDIR_template,$(SDIR),clean)))
$(foreach SDIR, $(INSTALLED_DIRS), $(eval $(call SDIR_template,$(SDIR),distclean)))

nothing:

install:

context:

depend: $(foreach SDIR, $(INSTALLED_DIRS), $(SDIR)_depend)

clean: $(foreach SDIR, $(INSTALLED_DIRS), $(SDIR)_clean)

distclean: $(foreach SDIR, $(INSTALLED_DIRS), $(SDIR)_distclean)
