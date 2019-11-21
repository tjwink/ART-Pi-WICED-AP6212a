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

NAME := WWD_ThreadX_Interface

GLOBAL_INCLUDES := .

$(NAME)_SOURCES  := wwd_rtos.c

ifneq ($(filter $(HOST_ARCH), ARM_CM3 ARM_CM4),)
$(NAME)_SOURCES  += CM3_CM4/low_level_init.c
GLOBAL_INCLUDES  += CM3_CM4
else
ifneq ($(filter $(HOST_ARCH), ARM_CR4),)
$(NAME)_SOURCES    += CR4/low_level_init.S
$(NAME)_SOURCES    += CR4/timer_isr.c
$(NAME)_LINK_FILES += CR4/timer_isr.o
GLOBAL_INCLUDES    += CR4
else
$(error No ThreadX low_level_init function for architecture $(HOST_ARCH))
endif


$(NAME)_CFLAGS  = $(COMPILER_SPECIFIC_PEDANTIC_CFLAGS)

endif

$(NAME)_CHECK_HEADERS := \
                         wwd_rtos.h

