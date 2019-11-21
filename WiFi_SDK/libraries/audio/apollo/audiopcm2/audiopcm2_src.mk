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
$(NAME)_COMPONENTS += audio/apollo/audioplc2
$(NAME)_COMPONENTS += audio/codec/wavpack/shortblocks

$(NAME)_HEADERS := \
					globals.h \
					audiopcm_types.h \
					audiopcm_core.h \
					audiopcm_tables.h \
					audiopcm_api.h \
					clk.h \
					clk_core.h \
					dsp.h \
					dsp_core.h \
					dsp_tools.h \
					events.h \
					logger.h \
					output.h \
					output_core.h \
					rtp.h \
					rtp_core.h \
					sysclk.h \
					tsp.h \
					watchdog.h \
					watchdog_core.h \
					concealmet.h \
					concealment_core.h \
					comp.h \
					comp_core.h \
					comp_memreader.h \
					comp_types.h \
					version.h

$(NAME)_SOURCES := \
					audiopcm.c \
					clk.c \
					concealment.c \
					dsp.c \
					dsp_tools.c \
					events.c \
					output.c \
					rtp.c \
					watchdog.c \
					sysclk.c \
					comp.c \
					comp_memreader.c \
					logger.c

KEEP_LIST:= \
					audiopcm*.a \
					audiopcm*.mk \
					audiopcm_api.h \
					audiopcm_tables.h \
					events.h \
					globals.h \
					audiopcm_types.h


#$(NAME)_CFLAGS += $(COMPILER_SPECIFIC_PEDANTIC_CFLAGS)


$(NAME)_CFLAGS += \
					-std=gnu99 \
					-D_FILE_OFFSET_BITS=64 \
					-D_LARGEFILE_SOURCE=1 \
					-D_LARGEFILE64_SOURCE=1 \
					-D ARM_MATH=1 \
					-O2

$(NAME)_DEFINES := WICED

$(NAME)_INCLUDES := \
					. \
					../../codec/wavpack/shortblocks \
					../../codec/wavpack/shortblocks/include \
					../audioplc2/src \
					../audioplc2/src/lcplc \
					../audioplc2/src/music \
					../audioplc2/src/splib
