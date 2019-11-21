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

NAME = STM32H7xx_Peripheral_Libraries

GLOBAL_INCLUDES :=  . \
                    inc \
                    cmsis/include \
                    ../../../$(HOST_ARCH)/CMSIS

GLOBAL_DEFINES += STM32H7 STM32H753xx
GLOBAL_DEFINES += USE_HAL_DRIVER

$(NAME)_SOURCES := \
                   src/stm32h7xx_hal.c \
                   src/stm32h7xx_hal_cortex.c \
                   src/stm32h7xx_hal_gpio.c \
                   src/stm32h7xx_hal_pwr.c \
                   src/stm32h7xx_hal_pwr_ex.c \
                   src/stm32h7xx_hal_rcc.c \
                   src/stm32h7xx_hal_rcc_ex.c \
                   src/stm32h7xx_hal_flash.c \
                   src/stm32h7xx_hal_flash_ex.c \
                   src/stm32h7xx_hal_uart.c \
                   src/stm32h7xx_hal_uart_ex.c \
                   src/stm32h7xx_hal_usart.c \
                   system_stm32h7xx.c \
                   src/stm32h7xx_hal_dma.c \
                   src/stm32h7xx_hal_iwdg.c \
                   src/stm32h7xx_hal_i2c.c \
                   src/stm32h7xx_hal_i2c_ex.c \
                   src/stm32h7xx_hal_sd.c \
                   src/stm32h7xx_ll_sdmmc.c \
                   src/stm32h7xx_hal_rtc.c \
                   src/stm32h7xx_hal_rtc_ex.c \
                   src/stm32h7xx_hal_tim.c \
                   src/stm32h7xx_hal_tim_ex.c \
                   src/stm32h7xx_hal_mdma.c \
                   src/stm32h7xx_hal_qspi.c

