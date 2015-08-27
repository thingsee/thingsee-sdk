LOCAL_PATH := $(call my-dir)

SYMLINKS := dfu/flashing.py smoke/primitive.py smoke/get_user_entrypoint.mk
 
include $(BUILD_OVERLAY)

include $(call all-makefiles-under,$(LOCAL_PATH))
