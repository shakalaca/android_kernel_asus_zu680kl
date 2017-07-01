LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
DLKM_DIR := device/qcom/common/dlkm
LOCAL_MODULE := texfat.ko
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_PATH := $(TARGET_ROOT_OUT_SBIN)
include $(DLKM_DIR)/AndroidKernelModule.mk
