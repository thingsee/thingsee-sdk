LOCAL_PATH := $(call my-dir)

SYMLINKS_DST := lib
SYMLINKS := $(shell cd $(LOCAL_PATH) && find -not -type d)

include $(BUILD_OVERLAY)
