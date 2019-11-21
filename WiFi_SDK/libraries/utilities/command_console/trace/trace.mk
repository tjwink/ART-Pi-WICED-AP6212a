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
NAME := Lib_command_console_trace

$(NAME)_SOURCES += trace.c \
                   trace_hook.c

#==============================================================================
# Configuration
#==============================================================================
TRACE_BUFFERED       := 1
TRACE_BUFFERED_PRINT := 1
TRACE_GPIO           := 1


ifneq ($(wildcard $(CURDIR)trace/$(RTOS)_trace.h),)
# Make sure to include this trace_macros.h first (to ensure the trace macros are defined accordingly)
GLOBAL_CFLAGS   += -include $(CURDIR)trace/$(RTOS)_trace.h
GLOBAL_CXXFLAGS += -include $(CURDIR)trace/$(RTOS)_trace.h

ifeq (FreeRTOS, $(RTOS))
# Needed to store useful tracing information
GLOBAL_DEFINES  += configUSE_TRACE_FACILITY=1
endif

#==============================================================================
# Defines and sources
#==============================================================================
ifeq (1, $(TRACE_BUFFERED))
ifneq ($(wildcard $(CURDIR)trace/gpio/$(HOST_MCU_FAMILY)_gpio_trace.c),)
GLOBAL_CFLAGS   += -include $(CURDIR)trace/clocktimer/ARM_CM3_DWT.h
GLOBAL_CXXFLAGS += -include $(CURDIR)trace/clocktimer/ARM_CM3_DWT.h

$(NAME)_DEFINES += TRACE_ENABLE_BUFFERED
$(NAME)_SOURCES += buffered/buffered_trace.c
$(NAME)_SOURCES += clocktimer/$(HOST_ARCH)_DWT.c

ifeq (1, $(TRACE_BUFFERED_PRINT))
$(NAME)_DEFINES += TRACE_ENABLE_BUFFERED_PRINT
$(NAME)_SOURCES += buffered/print/buffered_trace_print.c
endif

ifeq (1, $(TRACE_GPIO))
ifneq ($(wildcard $(CURDIR)trace/gpio/$(HOST_MCU_FAMILY)_gpio_trace.c),)
$(NAME)_DEFINES += TRACE_ENABLE_GPIO
$(NAME)_SOURCES += gpio/gpio_trace.c
# Note that we need to use delayed variable expansion here.
# See: http://www.bell-labs.com/project/nmake/newsletters/issue025.html
$(NAME)_SOURCES += gpio/$(HOST_MCU_FAMILY)_gpio_trace.c
else
$(info GPIO tracing not implemented for MCU $(HOST_MCU_FAMILY) - GPIO tracing will be disabled )
endif
endif

else
$(info Clocktimer tracing not implemented for Host Architecture $(HOST_ARCH) - Buffered tracing will be disabled )
endif
endif

else
$(info Tracing not implemented for RTOS $(RTOS) - tracing will be disabled)
endif