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

#
# Over The Air Image creation tool
#
# Takes a number of components and builds an OTA Package
# TODO: Optional SHA/other encryption
# TODO: Optional compression of components
#

NAME := mk_wiced_ota2_image


# Determine what host platform is being used
UNAME = $(shell uname)
ifeq ($(findstring MINGW,$(UNAME)),MINGW)
HOST:=WIN32
CFLAGS += -I/mingw/include
CFLAGS += -Wlogical-op -Wdouble-promotion
EXECUTABLE_EXTENSION:=.exe
COMMON_TOOLS_DIR:= ../../../../tools/common/Win32/
WICED_COMMON_TOOLS_32_DIR := ../../../../tools/common/Win32/
else # UNAME
ifeq ($(findstring Linux,$(UNAME)),Linux)
HOST:=LINUX
EXECUTABLE_EXTENSION:=
COMMON_TOOLS_DIR:= /usr/sbin/
WICED_COMMON_TOOLS_32_DIR := ../../../../tools/common/Linux32/
# Flags to enable fseeko64/ftello64 on Linux
CFLAGS += -D_FILE_OFFSET_BITS=64 -D_LARGEFILE64_SOURCE -D_USE_LARGEFILE64
else #UNAME
# MAC here
HOST:=OSX
CFLAGS +=
EXECUTABLE_EXTENSION:=
COMMON_TOOLS_DIR:=/usr/bin/
WICED_COMMON_TOOLS_32_DIR := ../../../../tools/common/OSX/
endif
endif

CC:=gcc
CP:=cp

# Add some includes and define
CFLAGS += -D$(HOST) -DOTA2_BUILDER_UTILITY
CFLAGS += -I.
CFLAGS += -I..
CFLAGS += -I../../../../include
CFLAGS += -I../../../../WICED/WWD/include
CFLAGS += -I../../../../WICED/security/BESL/include
CFLAGS += -I../../../../WICED/platform/include
CFLAGS += -I../../../../WICED/platform/GCC
CFLAGS += -I../../../../libraries/filesystems/wicedfs/src
CFLAGS += -I../../../../libraries/filesystems/ota2


# Enable Debug, disable optimisation
CFLAGS += -g -O0

# Add pedantic warning flags
CFLAGS += -Wall -Werror -W -Wstrict-prototypes -Wshadow  -Wwrite-strings -std=gnu99 -pedantic -Wno-format -Wconversion -Wextra -Wdeclaration-after-statement -Wconversion -Waddress -Wold-style-definition -Wmissing-prototypes -Wmissing-declarations -Wmissing-field-initializers -Wswitch-enum -Wswitch-default -Wuninitialized -Wunknown-pragmas -Wfloat-equal  -Wundef

# Macro to convert a C filename into an Object filename
# $(1) is the C source filename
GET_OBJ_FN = $(patsubst %.c,obj/%.o,$(notdir $(1)))


# Macro to create target to compile C source files
# $(1) is the C source filename, $(2) number of bits
define CREATE_C_TARGET

v$(2)_$(call GET_OBJ_FN,$(1)): $(1)
	mkdir -pv $$(@D)
	$(CC) -c $(CFLAGS) $(CFLAGS_$(2)) -o $$@ $$^

$(eval OBJS_$(2) += v$(2)_$(call GET_OBJ_FN,$(1)))

endef


# List of source files
SOURCES         := mk_wiced_ota2_image.c \
				   wiced_ota2_image_create.c \
				   ../../../utilities/crc/crc.c

# Create Executable names
EXE32_NAME          := $(NAME)32$(EXECUTABLE_EXTENSION)

# Main target - clean first, then build
all: clean $(EXE32_NAME)

# Create the targets for compiling C files
$(foreach SRC, $(SOURCES), $(eval $(call CREATE_C_TARGET,$(SRC),32)))

# Final link targets
$(EXE32_NAME): $(OBJS_32)
	$(CC) $(LDFLAGS) -o $@ $^
	$(CP) $@ $(COMMON_TOOLS_DIR)$@
	$(CP) -f $@ $(WICED_COMMON_TOOLS_32_DIR)$@

.PHONY: all clean

clean:
	 rm -rfv $(EXE32_NAME) $(OBJS) v32_obj
