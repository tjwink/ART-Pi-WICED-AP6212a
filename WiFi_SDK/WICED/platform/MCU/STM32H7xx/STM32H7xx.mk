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

NAME = STM32H7xx

# Host architecture is ARM Cortex M7
HOST_ARCH := ARM_CM7

# Host MCU alias for OpenOCD
HOST_OPENOCD := stm32h7x

GLOBAL_INCLUDES := . \
                   .. \
                   ../.. \
                   ../../include \
                   ../../$(HOST_ARCH) \
                   ../../$(HOST_ARCH)/CMSIS \
                   peripherals \
                   WAF

# Global defines
GLOBAL_DEFINES  := USE_STDPERIPH_DRIVER \
                   _STM3x_ \
                   _STM32x_

ifeq (1, $(OTA2_SUPPORT))
EXTERNAL_DCT := 1
endif

GLOBAL_DEFINES += STM32H7 STM32H753xx
GLOBAL_DEFINES += WICED_PLATFORM_MASKS_BUS_IRQ

# Low power : Stop mode
# GLOBAL_DEFINES += PLATFORM_SUPPORTS_LOW_POWER_MODES

# Global flags
GLOBAL_CFLAGS   += $$(CPU_CFLAGS)    $$(ENDIAN_CFLAGS_LITTLE)
GLOBAL_CXXFLAGS += $$(CPU_CXXFLAGS)  $$(ENDIAN_CXXFLAGS_LITTLE)
GLOBAL_ASMFLAGS += $$(CPU_ASMFLAGS)  $$(ENDIAN_ASMFLAGS_LITTLE)
GLOBAL_LDFLAGS  += $$(CPU_LDFLAGS)   $$(ENDIAN_LDFLAGS_LITTLE)

ifeq ($(TOOLCHAIN_NAME),GCC)
GLOBAL_LDFLAGS  += -nostartfiles
GLOBAL_LDFLAGS  += -Wl,--defsym,__STACKSIZE__=$$($(RTOS)_START_STACK)
GLOBAL_LDFLAGS  += -L ./WICED/platform/MCU/$(NAME)/$(TOOLCHAIN_NAME) \
                   -L ./WICED/platform/MCU/$(NAME)/$(TOOLCHAIN_NAME)/$(HOST_MCU_VARIANT)
else
ifeq ($(TOOLCHAIN_NAME),IAR)
DEFAULT_GLOBAL_HEAPSIZE ?= 0x8000
GLOBAL_LDFLAGS  += --config_def __STACKSIZE__=$$($(RTOS)_START_STACK)
GLOBAL_LDFLAGS  += --config_def __HEAPSIZE__=$$(DEFAULT_GLOBAL_HEAPSIZE)
GLOBAL_LDFLAGS  += --config_search ./WICED/platform/MCU/$(NAME)/$(TOOLCHAIN_NAME)/$(HOST_MCU_VARIANT)
endif
endif

# Currently, ARM_CM7 supports only FreeRTOS
VALID_OSNS_COMBOS := NoOS FreeRTOS-LwIP

# Components
$(NAME)_COMPONENTS += $(TOOLCHAIN_NAME)
$(NAME)_COMPONENTS += MCU/STM32H7xx/peripherals
$(NAME)_COMPONENTS += utilities/ring_buffer

# Source files
$(NAME)_SOURCES := ../../$(HOST_ARCH)/crt0_$(TOOLCHAIN_NAME).c \
                   ../../$(HOST_ARCH)/hardfault_handler.c \
                   ../../$(HOST_ARCH)/host_cm7.c \
                   ../platform_resource.c \
                   ../platform_stdio.c \
                   ../wiced_platform_common.c \
                   ../wwd_platform_separate_mcu.c \
                   ../wwd_resources.c \
                   ../wiced_apps_common.c	\
                   ../wiced_waf_common.c	\
                   ../platform_nsclock.c \
                   platform_vector_table.c \
                   platform_init.c \
                   platform_unhandled_isr.c \
                   platform_filesystem.c \
                   WAF/waf_platform.c

#for DCT with crc checking
$(NAME)_COMPONENTS  += utilities/crc

ifdef PLATFORM_SUPPORTS_BUTTONS
$(NAME)_SOURCES += ../platform_button.c
endif

ifeq ($(EXTERNAL_DCT), 1)
$(NAME)_SOURCES +=  ../wiced_dct_external_common.c ../wiced_dct_external_ota2.c ../wiced_dct_update.c

else
$(NAME)_SOURCES +=  ../wiced_dct_internal_common.c ../wiced_dct_update.c
endif

ifndef NO_WIFI
$(NAME)_SOURCES += WWD/wwd_platform.c \
                   WWD/wwd_$(BUS).c
endif

# These need to be forced into the final ELF since they are not referenced otherwise
$(NAME)_LINK_FILES := ../../$(HOST_ARCH)/crt0_$(TOOLCHAIN_NAME).o \
                      ../../$(HOST_ARCH)/hardfault_handler.o \
                      platform_vector_table.o

$(NAME)_CFLAGS = $(COMPILER_SPECIFIC_PEDANTIC_CFLAGS)

# Add maximum and default watchdog timeouts to definitions. Warning: Do not change MAX_WATCHDOG_TIMEOUT_SECONDS
MAX_WATCHDOG_TIMEOUT_SECONDS = 22
GLOBAL_DEFINES += MAX_WATCHDOG_TIMEOUT_SECONDS=$(MAX_WATCHDOG_TIMEOUT_SECONDS)

# DCT linker script
ifeq (1, $(OTA2_SUPPORT))
DCT_LINK_SCRIPT += $(TOOLCHAIN_NAME)/$(HOST_MCU_VARIANT)/ota2_dct$(LINK_SCRIPT_SUFFIX)
else
DCT_LINK_SCRIPT += $(TOOLCHAIN_NAME)/$(HOST_MCU_VARIANT)/dct$(LINK_SCRIPT_SUFFIX)
endif

ifeq ($(APP),bootloader)
BUILDING_BOOTLOADER = 1
endif
ifeq ($(APP),ota2_bootloader)
BUILDING_BOOTLOADER = 1
endif

ifeq ($(BUILDING_BOOTLOADER),1)
####################################################################################
# Building bootloader
####################################################################################
ifeq (1, $(OTA2_SUPPORT))
DEFAULT_LINK_SCRIPT := $(TOOLCHAIN_NAME)/ota2_bootloader$(LINK_SCRIPT_SUFFIX)
else
DEFAULT_LINK_SCRIPT += $(TOOLCHAIN_NAME)/bootloader$(LINK_SCRIPT_SUFFIX)
endif
else
ifneq ($(filter sflash_write, $(APP)),)
####################################################################################
# Building sflash_write
####################################################################################

PRE_APP_BUILDS      += bootloader
DEFAULT_LINK_SCRIPT := $(TOOLCHAIN_NAME)/app_ram$(LINK_SCRIPT_SUFFIX)
GLOBAL_INCLUDES     += WAF ../../../../../apps/waf/bootloader/
GLOBAL_DEFINES      += __JTAG_FLASH_WRITER_DATA_BUFFER_SIZE__=16384
ifeq ($(TOOLCHAIN_NAME),IAR)
GLOBAL_LDFLAGS      += --config_def __JTAG_FLASH_WRITER_DATA_BUFFER_SIZE__=16384
endif

else
ifeq ($(USES_BOOTLOADER_OTA),1)
####################################################################################
# Building standard application to run with bootloader
####################################################################################
ifeq (1, $(OTA2_SUPPORT))
DEFAULT_LINK_SCRIPT := $(TOOLCHAIN_NAME)/ota2_app_with_bootloader$(LINK_SCRIPT_SUFFIX)
else
PRE_APP_BUILDS      += bootloader
DEFAULT_LINK_SCRIPT := $(TOOLCHAIN_NAME)/app_with_bootloader$(LINK_SCRIPT_SUFFIX)
GLOBAL_INCLUDES     += WAF ../../../../../apps/waf/bootloader/
endif

else
####################################################################################
# Building a WWD application (standalone app without bootloader and DCT)
####################################################################################

DEFAULT_LINK_SCRIPT := $(TOOLCHAIN_NAME)/app_ram$(LINK_SCRIPT_SUFFIX)
GLOBAL_DEFINES      += WICED_DISABLE_BOOTLOADER

endif # USES_BOOTLOADER_OTA = 1
endif # APP= sflash_write
endif # APP=bootloader

