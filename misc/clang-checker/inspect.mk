# Every part except the top level Makefile expects
# this variable to be assigned. Should not let them
# down.
TOPDIR := $(abspath ../../nuttx)

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs

# Creates a rule, which can print every variable
# Credits go to http://blog.jgc.org/2015/04/the-one-line-you-should-add-to-every.html
print-% :
	@echo $($*)
