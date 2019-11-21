#
# $ Copyright Broadcom Corporation $
#

NAME := Lib_fdk_aac_dec

FDK-AAC-DEC_LIBRARY_NAME :=fdk_aac_dec.$(RTOS).$(HOST_ARCH).$(BUILD_TYPE).a

#$(NAME)_CFLAGS := 

GLOBAL_INCLUDES += \
		   ./libSYS/include \
		   ./libFDK/include \
		   ./libPCMutils/include \
		   ./libMpegTPDec/include \
		   ./libSBRdec/include \
		   ./libAACdec/include 

$(NAME)_ALWAYS_OPTIMISE:=1

ifdef USE_FDK_AAC_STEREO
$(info fdk_aac_dec - WARNING - build type is STEREO only.)
GLOBAL_DEFINES     += CY_FDK_AAC_STEREO
endif

ifneq ($(wildcard $(CURDIR)$(FDK-AAC-DEC_LIBRARY_NAME)),)
# Build from prebuilt (SDK default)
$(info Using PREBUILT:  $(FDK-AAC-DEC_LIBRARY_NAME))
$(NAME)_PREBUILT_LIBRARY := $(FDK-AAC-DEC_LIBRARY_NAME)

else

# Build from source (Broadcom internal)
$(info Building SRC:  $(FDK-AAC-DEC_LIBRARY_NAME))
include $(CURDIR)fdk_aac_dec_src.mk
endif 


