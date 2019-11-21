/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 * Defines STM32H7xx default unhandled ISR and default mappings to unhandled ISR
 */
#include <stdint.h>
#include "platform_cmsis.h"
#include "platform_isr_interface.h"
#include "platform_assert.h"
#include "platform_constants.h"
#include "platform_isr.h"
#include "wwd_rtos.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

extern void UnhandledInterrupt( void );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

PLATFORM_DEFINE_ISR( UnhandledInterrupt )
{
    uint32_t active_interrupt_vector = (uint32_t) ( SCB->ICSR & 0x3fU );

    /* This variable tells you which interrupt vector is currently active */
    (void)active_interrupt_vector;
    WICED_TRIGGER_BREAKPOINT( );

    while( 1 )
    {
    }
}

// Insert after UsageFault
// PLATFORM_SET_DEFAULT_ISR( SVC_Handler,  UnhandledInterrupt )

// Insert after DebugMon_Handler
// PLATFORM_SET_DEFAULT_ISR( PendSV_Handler,  UnhandledInterrupt )
// PLATFORM_SET_DEFAULT_ISR( SysTick_Handler,  UnhandledInterrupt )

/******************************************************
 *          Default IRQ Handler Declarations
 ******************************************************/

PLATFORM_SET_DEFAULT_ISR( NMI_Handler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( HardFault_Handler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( MemManage_Handler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( BusFault_Handler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( UsageFault_Handler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DebugMon_Handler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( WWDG_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( PVD_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TAMP_STAMP_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( RTC_WKUP_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( FLASH_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( RCC_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( EXTI0_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( EXTI1_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( EXTI2_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( EXTI3_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( EXTI4_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream0_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream1_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream2_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream3_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream4_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream5_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream6_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( ADC_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN1_TX_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN1_RX0_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN1_RX1_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN1_SCE_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( EXTI9_5_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM1_BRK_TIM9_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM1_UP_TIM10_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM1_TRG_COM_TIM11_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM1_CC_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM2_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM3_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM4_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( I2C1_EV_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( I2C1_ER_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( I2C2_EV_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( I2C2_ER_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( SPI1_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( SPI2_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( USART1_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( USART2_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( USART3_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( EXTI15_10_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( RTC_Alarm_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( OTG_FS_WKUP_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM8_BRK_TIM12_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM8_UP_TIM13_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM8_TRG_COM_TIM14_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM8_CC_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream7_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( FMC_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( SDIO_irq,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM5_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( SPI3_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( UART4_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( UART5_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM6_DAC_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM7_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream0_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream1_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream2_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream3_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream4_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( ETH_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( ETH_WKUP_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN2_TX_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN2_RX0_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN2_RX1_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN2_SCE_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( OTG_FS_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream5_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream6_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream7_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( USART6_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( I2C3_EV_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( I2C3_ER_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( OTG_HS_EP1_OUT_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( OTG_HS_EP1_IN_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( OTG_HS_WKUP_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( OTG_HS_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DCMI_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( HASH_RNG_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( FPU_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( UART7_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( UART8_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( SPI4_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( SPI5_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( SPI6_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( SAI1_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( LTDC_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( LTDC_ER_IRQHandler,  UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2D_IRQHandler,  UnhandledInterrupt )
