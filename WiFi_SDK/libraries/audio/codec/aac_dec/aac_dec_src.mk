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
$(NAME)_COMPONENTS += audio/codec/fdk_aac_dec

ifdef WICED_ENABLE_TRACEX
$(info ************************************************************** )
$(info ==> AAC_DEC: Enabling TraceX lib)
$(info ************************************************************** )
$(NAME)_COMPONENTS += test/TraceX
endif # WICED_ENABLE_TRACEX

$(NAME)_INCLUDES := ../fdk_aac_dec/libAACdec/include/

$(NAME)_HEADERS := \
			aacdec.h \
			aacdec_core.h \
			aacdec_types.h \
			globals.h \
			logger.h \
			adtsparse.h \
			adtsparse_core.h \
			adtsparse_types.h \
			mp4parse.h \
			mp4parse_core.h \
			mp4parse_types.h \
			sysclk.h \
			sysclk_wiced.h \
			tsp.h \
			tsp_wiced.h



$(NAME)_SOURCES := \
			aacdec.c \
			adtsparse.c \
			mp4parse.c


KEEP_LIST := \
			aac_dec*.a \
			aac_dec*.mk \
			aacdec.h

#
# library build
#
$(info aacdec build type is $(BUILD_TYPE).)

$(NAME)_DEFINES := WICED


$(NAME)_CFLAGS := -O2
$(NAME)_CFLAGS += -std=gnu99 \
                  -D_FILE_OFFSET_BITS=64 \
                  -D_LARGEFILE_SOURCE=1 \
                  -D_LARGEFILE64_SOURCE=1 \
		  -Wno-unused-function


#
# debug/release flags
#
ifeq ($(BUILD_TYPE),debug)
#$(info ************************************************************** )
#$(info ==> AAC_DEC: Enabling debug build.)
#$(info ************************************************************** )


$(NAME)_DEFINES += DEBUG=1

else

$(NAME)_DEFINES += NDEBUG=1
$(NAME)_CFLAGS  += \
		-funroll-loops \
		-ftree-vectorize
endif
