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

$(NAME)_HEADERS := \
		src/audioPLCerrs.h\
		src/AudioStreamingConcealment.h\
		src/AudioStreamingConcealmentDefs.h\
		src/lcplc/autocor.h\
		src/lcplc/estpitch.h\
		src/lcplc/lcplc.h\
		src/lcplc/levi.h\
		src/lcplc/smdpitch.h\
		src/music/audioPLC.h\
		src/music/audioProcessing.h\
		src/samplePLC/aswin192.h\
		src/samplePLC/aswin192_fix.h\
		src/samplePLC/aswin44.h\
		src/samplePLC/aswin44_fix.h\
		src/samplePLC/aswin48.h\
		src/samplePLC/aswin48_fix.h\
		src/samplePLC/aswin96.h\
		src/samplePLC/aswin96_fix.h\
		src/samplePLC/Fautocor.h\
		src/samplePLC/levinson.h\
		src/samplePLC/matrix.h\
		src/samplePLC/samplePLC.h\
		src/samplePLC/samplePLCdefs.h\
		src/samplePLC/samplePLCtables.h\
		src/splib/Bopslib.h\
		src/splib/fixmath.h\
		src/typedef.h

$(NAME)_SOURCES := \
		src/splib/Bopslib.c\
		src/splib/fixmath.c\
		src/lcplc/autocor.c\
		src/lcplc/configs.c\
		src/lcplc/estpitch.c\
		src/lcplc/lcplc.c\
		src/lcplc/levi.c\
		src/lcplc/smdpitch.c\
		src/lcplc/tables.c\
		src/music/audioPLC.c\
		src/music/audioProcessing.c\
		src/samplePLC/Fautocor.c\
		src/samplePLC/levinson.c\
		src/samplePLC/matrix.c\
		src/samplePLC/samplePLC.c\
		src/samplePLC/samplePLCtables.c\
		src/AudioStreamingConcealment.c

$(NAME)_INCLUDES := src\
                src/splib\
                src/samplePLC\
                src/lcplc\
                src/music

KEEP_LIST:= \
		audioplc*.a \
		audioplc*.mk \
		src/audioPLCerrs.h\
		src/AudioStreamingConcealment.h\
		src/AudioStreamingConcealmentDefs.h\
		src/music/audioPLC.h\
		src/music/audioProcessing.h\
                src/lcplc/lcplc.h\
		src/music/audioPLC.h \
		src/samplePLC/samplePLC.h\
		src/samplePLC/samplePLCdefs.h\
		src/samplePLC/samplePLCtables.h \
		src/splib/Bopslib.h\
		src/typedef.h

$(NAME)_CFLAGS += \
		-std=gnu99 \
		-D_FILE_OFFSET_BITS=64 \
		-D_LARGEFILE_SOURCE=1 \
		-D_LARGEFILE64_SOURCE=1 \
		-D ARM_MATH=1

$(NAME)_DEFINES := WICED
$(NAME)_DEFINES += DEBUG_FPRINTF_ENABLED=0

GLOBAL_DEFINES  += USE_LINEAR_INTERPOLATION
