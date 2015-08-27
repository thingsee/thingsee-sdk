LOCAL_PATH := $(call my-dir)

SYMLINKS := $(shell cd $(LOCAL_PATH) && find -type f)

include $(BUILD_OVERLAY)
