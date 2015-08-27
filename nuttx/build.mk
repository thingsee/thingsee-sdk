LOCAL_PATH := $(call my-dir)
-include $(LOCAL_PATH)/.config

# We expect to have nuttx (ELF) and nuttx.dfu from the build
# ELF always gets built, but DFU can be activated in menuconfig
SYMLINKS := nuttx System.map .version .config

ifndef CONFIG_ARCH_SIM
SYMLINKS += nuttx.dfu
endif

include $(BUILD_OVERLAY)

include $(call first-makefiles-under,$(LOCAL_PATH))
