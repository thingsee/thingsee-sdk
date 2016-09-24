LOCAL_PATH := $(call my-dir)

SYMLINKS_DST := smoke
SYMLINKS := $(shell cd $(LOCAL_PATH) && find -not -type d)

include $(BUILD_OVERLAY)
