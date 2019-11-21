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

NAME = STM32F4xx_Peripheral_Libraries

GLOBAL_INCLUDES :=  . \
                    inc \
                    ../../../$(HOST_ARCH)/CMSIS

$(NAME)_SOURCES := \
                   src/misc.c \
                   src/stm32f4xx_adc.c \
                   src/stm32f4xx_can.c \
                   src/stm32f4xx_crc.c \
                   src/stm32f4xx_dac.c \
                   src/stm32f4xx_dbgmcu.c \
                   src/stm32f4xx_dma.c \
                   src/stm32f4xx_exti.c \
                   src/stm32f4xx_flash.c \
                   src/stm32f4xx_gpio.c \
                   src/stm32f4xx_rng.c \
                   src/stm32f4xx_i2c.c \
                   src/stm32f4xx_iwdg.c \
                   src/stm32f4xx_pwr.c \
                   src/stm32f4xx_rcc.c \
                   src/stm32f4xx_rtc.c \
                   src/stm32f4xx_sdio.c \
                   src/stm32f4xx_spi.c \
                   src/stm32f4xx_syscfg.c \
                   src/stm32f4xx_tim.c \
                   src/stm32f4xx_usart.c \
                   src/stm32f4xx_wwdg.c

# Add FSMC for supported STM32F4xx MCUs
ifneq ($(filter $(HOST_MCU_VARIANT), STM32F415 STM32F417),)
$(NAME)_SOURCES += src/stm32f4xx_fsmc.c
endif 

# Add FMC for supported STM32F4xx MCUs
ifneq ($(filter $(HOST_MCU_VARIANT), STM32F429 STM32F437),)
$(NAME)_SOURCES += src/stm32f4xx_fmc.c
$(NAME)_SOURCES += src/stm32f4xx_dma2d.c \
                   src/stm32f4xx_dcmi.c \
                   src/stm32f4xx_ltdc.c
endif

