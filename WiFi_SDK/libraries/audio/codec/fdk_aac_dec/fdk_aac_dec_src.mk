#
# $ Copyright Broadcom Corporation $
#

#
# fdk-sublibs folders
#
LIBMPEGTPDEC_DIR := libMpegTPDec
LIBMPEGTPENC_DIR := libMpegTPEnc

LIBAACDEC_DIR := libAACdec
LIBAACENC_DIR := libAACenc

LIBSBRDEC_DIR := libSBRdec
LIBSBRENC_DIR := libSBRenc

LIBFDK_DIR := libFDK
LIBSYS_DIR := libSYS
LIBPCM_DIR := libPCMutils



# ################################################################
#
# libSYS definition
#
# ################################################################

#LIBSYS_INCS := \
#	$(LIBSYS_DIR)/include/audio.h \
#	$(LIBSYS_DIR)/include/cmdl_parser.h \
#	$(LIBSYS_DIR)/include/conv_string.h \
#	$(LIBSYS_DIR)/include/FDK_audio.h \
#	$(LIBSYS_DIR)/include/genericStds.h \
#	$(LIBSYS_DIR)/include/machine_type.h \
#	$(LIBSYS_DIR)/include/wav_file.h

LIBSYS_INCLUDES := \
	$(LIBSYS_DIR)/include/

LIBSYS_SRCS := \
	$(LIBSYS_DIR)/src/cmdl_parser.cpp \
	$(LIBSYS_DIR)/src/conv_string.cpp \
	$(LIBSYS_DIR)/src/genericStds.cpp \
	$(LIBSYS_DIR)/src/wav_file.cpp \
	$(LIBSYS_DIR)/src/linux/audio_linux.cpp \
	$(LIBSYS_DIR)/src/linux/coresup_linux.cpp \
	$(LIBSYS_DIR)/src/linux/FDK_stackload_linux.cpp \
	$(LIBSYS_DIR)/src/linux/genericStds_linux.cpp \
	$(LIBSYS_DIR)/src/linux/uart_linux.cpp


# ################################################################
#
# libFDK framework definitions
# 
# ################################################################

#LIBFDK_INCS := \
#	$(LIBFDK_DIR)/include/abs.h \
#	$(LIBFDK_DIR)/include/autocorr2nd.h \
#	$(LIBFDK_DIR)/include/clz.h \
#	$(LIBFDK_DIR)/include/cplx_mul.h \
#	$(LIBFDK_DIR)/include/dct.h \
#	$(LIBFDK_DIR)/include/FDK_archdef.h \
#	$(LIBFDK_DIR)/include/FDK_bitbuffer.h \
#	$(LIBFDK_DIR)/include/FDK_bitstream.h \
#	$(LIBFDK_DIR)/include/FDK_core.h \
#	$(LIBFDK_DIR)/include/FDK_crc.h \
#	$(LIBFDK_DIR)/include/FDK_hybrid.h \
#	$(LIBFDK_DIR)/include/FDK_tools_rom.h \
#	$(LIBFDK_DIR)/include/FDK_trigFcts.h \
#	$(LIBFDK_DIR)/include/fft.h \
#	$(LIBFDK_DIR)/include/fft_rad2.h \
#	$(LIBFDK_DIR)/include/fixmadd.h \
#	$(LIBFDK_DIR)/include/fixminmax.h \
#	$(LIBFDK_DIR)/include/fixmul.h \
#	$(LIBFDK_DIR)/include/fixpoint_math.h \
#	$(LIBFDK_DIR)/include/mdct.h \
#	$(LIBFDK_DIR)/include/qmf.h \
#	$(LIBFDK_DIR)/include/scale.h \
#	$(LIBFDK_DIR)/include/scramble.h

LIBFDK_INCLUDES := \
	$(LIBFDK_DIR)/include \
	$(LIBFDK_DIR)/include/arm

LIBFDK_SRCS := \
	$(LIBFDK_DIR)/src/FDK_bitbuffer.cpp \
	$(LIBFDK_DIR)/src/FDK_core.cpp \
	$(LIBFDK_DIR)/src/FDK_crc.cpp \
	$(LIBFDK_DIR)/src/FDK_hybrid.cpp \
	$(LIBFDK_DIR)/src/FDK_tools_rom.cpp \
	$(LIBFDK_DIR)/src/FDK_trigFcts.cpp \
	$(LIBFDK_DIR)/src/fft.cpp \
	$(LIBFDK_DIR)/src/fixpoint_math.cpp \
	$(LIBFDK_DIR)/src/mdct.cpp \
	$(LIBFDK_DIR)/src/autocorr2nd.cpp \
	$(LIBFDK_DIR)/src/dct.cpp \
	$(LIBFDK_DIR)/src/fft_rad2.cpp \
	$(LIBFDK_DIR)/src/qmf.cpp \
	$(LIBFDK_DIR)/src/scale.cpp


# ################################################################
#
# lib PCM Utils definitions
#
# ################################################################

#LIBPCM_INCS := \
#	$(LIBPCM_DIR)/include/pcmutils_lib.h


LIBPCM_INCLUDES := \
	$(LIBPCM_DIR)/include

LIBPCM_SRCS := \
	$(LIBPCM_DIR)/src/pcmutils_lib.cpp \
	$(LIBPCM_DIR)/src/limiter.cpp

# ################################################################
#
# lib Mpeg Transport decoder definitions
#
# ################################################################

#LIBMPEGTPDEC_INCS := \
#	$(LIBMPEGTPDEC_DIR)/include/mpegFileRead.h \
#	$(LIBMPEGTPDEC_DIR)/include/tp_data.h \
#	$(LIBMPEGTPDEC_DIR)/include/tpdec_lib.h \
#	$(LIBMPEGTPDEC_DIR)/src/mpegFileFormat.h \
#	$(LIBMPEGTPDEC_DIR)/src/tpdec_adif.h \
#	$(LIBMPEGTPDEC_DIR)/src/tpdec_adts.h \
#	$(LIBMPEGTPDEC_DIR)/src/tpdec_latm.h 

LIBMPEGTPDEC_INCLUDES := \
	$(LIBMPEGTPDEC_DIR)/include

LIBMPEGTPDEC_SRCS := \
	$(LIBMPEGTPDEC_DIR)/src/tpdec_adif.cpp \
	$(LIBMPEGTPDEC_DIR)/src/tpdec_adts.cpp \
	$(LIBMPEGTPDEC_DIR)/src/tpdec_asc.cpp \
	$(LIBMPEGTPDEC_DIR)/src/tpdec_latm.cpp \
	$(LIBMPEGTPDEC_DIR)/src/tpdec_drm.cpp \
	$(LIBMPEGTPDEC_DIR)/src/tpdec_lib.cpp


# ################################################################
#
# libSBR decoder definition
#
# ################################################################

#LIBSBRDEC_INCS := \
#	$(LIBSBRDEC_DIR)/include/sbrdecoder.h \
#	$(LIBSBRDEC_DIR)/src/env_calc.h \
#	$(LIBSBRDEC_DIR)/src/env_dec.h \
#	$(LIBSBRDEC_DIR)/src/env_extr.h \
#	$(LIBSBRDEC_DIR)/src/huff_dec.h \
#	$(LIBSBRDEC_DIR)/src/lpp_tran.h \
#	$(LIBSBRDEC_DIR)/src/psbitdec.h \
#	$(LIBSBRDEC_DIR)/src/psdec.h \
#	$(LIBSBRDEC_DIR)/src/psdec_hybrid.h \
#	$(LIBSBRDEC_DIR)/src/sbr_crc.h \
#	$(LIBSBRDEC_DIR)/src/sbr_deb.h \
#	$(LIBSBRDEC_DIR)/src/sbrdec_drc.h \
#	$(LIBSBRDEC_DIR)/src/sbrdec_freq_sca.h \
#	$(LIBSBRDEC_DIR)/src/sbr_dec.h \
#	$(LIBSBRDEC_DIR)/src/sbr_ram.h \
#	$(LIBSBRDEC_DIR)/src/sbr_rom.h \
#	$(LIBSBRDEC_DIR)/src/sbr_scale.h \
#	$(LIBSBRDEC_DIR)/src/transcendent.h

LIBSBRDEC_INCLUDES := \
	$(LIBSBRDEC_DIR)/include \
	$(LIBSBRDEC_DIR)/src

LIBSBRDEC_SRCS := \
	$(LIBSBRDEC_DIR)/src/env_calc.cpp \
	$(LIBSBRDEC_DIR)/src/env_dec.cpp \
	$(LIBSBRDEC_DIR)/src/env_extr.cpp \
	$(LIBSBRDEC_DIR)/src/huff_dec.cpp \
	$(LIBSBRDEC_DIR)/src/lpp_tran.cpp \
	$(LIBSBRDEC_DIR)/src/psbitdec.cpp \
	$(LIBSBRDEC_DIR)/src/psdec.cpp \
	$(LIBSBRDEC_DIR)/src/psdec_hybrid.cpp \
	$(LIBSBRDEC_DIR)/src/sbr_crc.cpp \
	$(LIBSBRDEC_DIR)/src/sbr_deb.cpp \
	$(LIBSBRDEC_DIR)/src/sbr_dec.cpp \
	$(LIBSBRDEC_DIR)/src/sbrdec_drc.cpp \
	$(LIBSBRDEC_DIR)/src/sbrdec_freq_sca.cpp \
	$(LIBSBRDEC_DIR)/src/sbrdecoder.cpp \
	$(LIBSBRDEC_DIR)/src/sbr_ram.cpp \
	$(LIBSBRDEC_DIR)/src/sbr_rom.cpp


# ################################################################
#
# libAAC decoder definitions
#
# ################################################################

#LIBAACDEC_INCS := \
#	$(LIBAACDEC_DIR)/include/aacdecoder_lib.h \
#	$(LIBAACDEC_DIR)/src/aacdec_drc.h \
#	$(LIBAACDEC_DIR)/src/aacdec_drc_types.h \
#	$(LIBAACDEC_DIR)/src/aacdec_hcr_bit.h \
#	$(LIBAACDEC_DIR)/src/aacdec_hcr.h \
#	$(LIBAACDEC_DIR)/src/aacdec_hcrs.h \
#	$(LIBAACDEC_DIR)/src/aacdec_hcr_types.h \
#	$(LIBAACDEC_DIR)/src/aacdec_pns.h \
#	$(LIBAACDEC_DIR)/src/aacdec_tns.h \
#	$(LIBAACDEC_DIR)/src/aac_ram.h \
#	$(LIBAACDEC_DIR)/src/aac_rom.h \
#	$(LIBAACDEC_DIR)/src/block.h \
#	$(LIBAACDEC_DIR)/src/channel.h \
#	$(LIBAACDEC_DIR)/src/conceal.h \
#	$(LIBAACDEC_DIR)/src/conceal_types.h \
#	$(LIBAACDEC_DIR)/src/debug.h \
#	$(LIBAACDEC_DIR)/src/ldfiltbank.h \
#	$(LIBAACDEC_DIR)/src/overlapadd.h \
#	$(LIBAACDEC_DIR)/src/pulsedata.h \
#	$(LIBAACDEC_DIR)/src/rvlcbit.h \
#	$(LIBAACDEC_DIR)/src/rvlcconceal.h \
#	$(LIBAACDEC_DIR)/src/rvlc.h \
#	$(LIBAACDEC_DIR)/src/rvlc_info.h \
#	$(LIBAACDEC_DIR)/src/stereo.h

LIBAACDEC_INCLUDES := \
	$(LIBAACDEC_DIR)/include

LIBAACDEC_SRCS := \
	$(LIBAACDEC_DIR)/src/aacdec_drc.cpp \
	$(LIBAACDEC_DIR)/src/aacdec_hcr.cpp \
	$(LIBAACDEC_DIR)/src/aacdecoder.cpp \
	$(LIBAACDEC_DIR)/src/aacdec_pns.cpp \
	$(LIBAACDEC_DIR)/src/aac_ram.cpp \
	$(LIBAACDEC_DIR)/src/block.cpp \
	$(LIBAACDEC_DIR)/src/channelinfo.cpp \
	$(LIBAACDEC_DIR)/src/ldfiltbank.cpp \
	$(LIBAACDEC_DIR)/src/rvlcbit.cpp \
	$(LIBAACDEC_DIR)/src/rvlc.cpp \
	$(LIBAACDEC_DIR)/src/aacdec_hcr_bit.cpp \
	$(LIBAACDEC_DIR)/src/aacdec_hcrs.cpp \
	$(LIBAACDEC_DIR)/src/aacdecoder_lib.cpp \
	$(LIBAACDEC_DIR)/src/aacdec_tns.cpp \
	$(LIBAACDEC_DIR)/src/aac_rom.cpp \
	$(LIBAACDEC_DIR)/src/channel.cpp \
	$(LIBAACDEC_DIR)/src/conceal.cpp \
	$(LIBAACDEC_DIR)/src/pulsedata.cpp \
	$(LIBAACDEC_DIR)/src/rvlcconceal.cpp \
	$(LIBAACDEC_DIR)/src/stereo.cpp


KEEP_LIST:= \
	fdk_aac_dec*.a \
	fdk_aac_dec*.mk
#	audiopcm_api.h \
#	audiopcm_tables.h \



#
# library build
# 
$(info fdk_aac_dec build type is $(BUILD_TYPE).)

$(NAME)_SOURCES += \
		$(LIBSYS_SRCS) \
		$(LIBFDK_SRCS) \
		$(LIBPCM_SRCS) \
		$(LIBMPEGTPDEC_SRCS) \
		$(LIBSBRDEC_SRCS) \
		$(LIBAACDEC_SRCS)

$(NAME)_INCLUDES += \
		$(CURDIR)/$(LIBSYS_INCLUDES) \
		$(CURDIR)/$(LIBFDK_INCLUDES) \
		$(CURDIR)/$(LIBPCM_INCLUDES) \
		$(CURDIR)/$(LIBMPEGTPDEC_INCLUDES) \
		$(CURDIR)/$(LIBSBRDEC_INCLUDES) \
		$(CURDIR)/$(LIBAACDEC_INCLUDES)


$(NAME)_DEFINES := WICED


$(NAME)_CFLAGS := -O2
$(NAME)_CFLAGS += -std=gnu99 

#
# debug/release flags
#
ifeq ($(BUILD_TYPE),debug)
$(info ************************************************************** )
$(info ==> FDK-AAC: Enabling debug build.)
$(info ************************************************************** )


$(NAME)_DEFINES += DEBUG=1

else

$(NAME)_DEFINES += NDEBUG=1
$(NAME)_CFLAGS  += \
		-funroll-loops \
		-ftree-vectorize
endif


#
# FDK-AAC needs to relax some compiler flags to compile on WICED
# 

$(NAME)_CXXFLAGS += \
		-Wno-sign-compare \
		-Wno-unused-variable \
		-Wno-unused-value \
		-Wno-unused-but-set-variable \
		-Wno-unused-function \
		-Wno-attributes \
		-Wno-uninitialized \
		-Wno-maybe-uninitialized \
		-Wno-sequence-point \
		-Wno-unused-label \
		-Wno-narrowing 

#test.fdk_aac_dec-BCM943907WAE_1.B1-debug
