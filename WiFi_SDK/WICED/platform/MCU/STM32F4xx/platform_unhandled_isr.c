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
 * Defines STM32F4xx default unhandled ISR and default mappings to unhandled ISR
 */
#include <stdint.h>
#include "platform_cmsis.h"
#include "platform_assert.h"
#include "platform_constants.h"
#include "platform_isr.h"
#include "platform_isr_interface.h"
#include "wwd_rtos.h"
#include "platform_peripheral.h"

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

    /* reset the processor immeditly if not debug */
    platform_mcu_reset( );

    while( 1 )
    {
    }
}

/******************************************************
 *          Default IRQ Handler Declarations
 ******************************************************/

PLATFORM_SET_DEFAULT_ISR( NMIException           , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( HardFaultException     , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( MemManageException     , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( BusFaultException      , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( UsageFaultException    , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DebugMonitor           , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( WWDG_irq               , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( PVD_irq                , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TAMP_STAMP_irq         , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( RTC_WKUP_irq           , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( FLASH_irq              , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( RCC_irq                , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( EXTI0_irq              , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( EXTI1_irq              , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( EXTI2_irq              , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( EXTI3_irq              , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( EXTI4_irq              , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream0_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream1_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream2_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream3_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream4_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream5_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream6_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( ADC_irq                , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN1_TX_irq            , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN1_RX0_irq           , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN1_RX1_irq           , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN1_SCE_irq           , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( EXTI9_5_irq            , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM1_BRK_TIM9_irq      , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM1_UP_TIM10_irq      , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM1_TRG_COM_TIM11_irq , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM1_CC_irq            , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM2_irq               , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM3_irq               , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM4_irq               , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( I2C1_EV_irq            , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( I2C1_ER_irq            , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( I2C2_EV_irq            , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( I2C2_ER_irq            , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( SPI1_irq               , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( SPI2_irq               , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( USART1_irq             , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( USART2_irq             , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( USART3_irq             , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( EXTI15_10_irq          , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( RTC_Alarm_irq          , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( OTG_FS_WKUP_irq        , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM8_BRK_TIM12_irq     , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM8_UP_TIM13_irq      , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM8_TRG_COM_TIM14_irq , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM8_CC_irq            , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA1_Stream7_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( FSMC_irq               , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( SDIO_irq               , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM5_irq               , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( SPI3_irq               , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( UART4_irq              , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( UART5_irq              , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM6_DAC_irq           , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( TIM7_irq               , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream0_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream1_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream2_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream3_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream4_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( ETH_irq                , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( ETH_WKUP_irq           , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN2_TX_irq            , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN2_RX0_irq           , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN2_RX1_irq           , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CAN2_SCE_irq           , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( OTG_FS_irq             , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream5_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream6_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DMA2_Stream7_irq       , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( USART6_irq             , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( I2C3_EV_irq            , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( I2C3_ER_irq            , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( OTG_HS_EP1_OUT_irq     , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( OTG_HS_EP1_IN_irq      , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( OTG_HS_WKUP_irq        , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( OTG_HS_irq             , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( DCMI_irq               , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( CRYP_irq               , UnhandledInterrupt )
PLATFORM_SET_DEFAULT_ISR( HASH_RNG_irq           , UnhandledInterrupt )

