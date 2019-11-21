#
# Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 # Cypress Semiconductor Corporation. All Rights Reserved.
 # This software, including source code, documentation and related
 # materials ("Software"), is owned by Cypress Semiconductor Corporation
 # or one of its subsidiaries ("Cypress") and is protected by and subject to
 # worldwide patent protection (United States and foreign),
 # United States copyright laws and international treaty provisions.
 # Therefore, you may use this Software only as provided in the license
 # agreement accompanying the software package from which you
 # obtained this Software ("EULA").
 # If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 # non-transferable license to copy, modify, and compile the Software
 # source code solely for use in connection with Cypress's
 # integrated circuit products. Any reproduction, modification, translation,
 # compilation, or representation of this Software except as specified
 # above is prohibited without the express written permission of Cypress.
 #
 # Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 # EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 # reserves the right to make changes to the Software without notice. Cypress
 # does not assume any liability arising out of the application or use of the
 # Software or any product or circuit described in the Software. Cypress does
 # not authorize its products for use in any products where a malfunction or
 # failure of the Cypress product may reasonably be expected to result in
 # significant property damage, injury or death ("High Risk Product"). By
 # including Cypress's product in a High Risk Product, the manufacturer
 # of such system or application assumes all risk of such use and in doing
 # so agrees to indemnify Cypress against all liability.
#

NAME := Lib_FLAC

OGG_SRCS_C = \
	libFLAC/ogg_helper.c \
	libFLAC/ogg_mapping.c	\
	libFLAC/ogg_decoder_aspect.c

LIBFLAC_SRC_C = \
	libFLAC/bitmath.c \
	libFLAC/bitreader.c \
	libFLAC/bitwriter.c \
	libFLAC/cpu.c \
	libFLAC/crc.c \
	libFLAC/fixed.c \
	libFLAC/fixed_intrin_sse2.c \
	libFLAC/fixed_intrin_ssse3.c \
	libFLAC/float.c \
	libFLAC/format.c \
	libFLAC/lpc.c \
	libFLAC/lpc_intrin_sse.c \
	libFLAC/lpc_intrin_sse2.c \
	libFLAC/lpc_intrin_sse41.c \
	libFLAC/lpc_intrin_avx2.c \
	libFLAC/md5.c \
	libFLAC/memory.c \
	libFLAC/metadata_iterators.c \
	libFLAC/metadata_object.c \
	libFLAC/window.c

LIBFLAC_DECODER_SRC_C = \
	libFLAC/stream_decoder.c

LIBFLAC_ENCODER_SRC_C = \
	libFLAC/stream_encoder.c \
	libFLAC/stream_encoder_framing.c \
	libFLAC/stream_encoder_intrin_avx2.c \
	libFLAC/stream_encoder_intrin_sse2.c \
	libFLAC/stream_encoder_intrin_ssse3.c

SRCS_C = \
	$(LIBFLAC_SRC_C)		\
	$(LIBFLAC_DECODER_SRC_C)		\
	$(LIBFLAC_ENCODER_SRC_C)		\
	$(OGG_SRCS)

$(NAME)_SOURCES := 	$(SRCS_C)


$(NAME)_INCLUDES :=  .
$(NAME)_INCLUDES += include/private
$(NAME)_INCLUDES += include/protected

GLOBAL_INCLUDES += include
GLOBAL_INCLUDES += include/FLAC/
