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
 * STM32H7xx vector table
 */
#include <stdint.h>
#include "platform_cmsis.h"
#include "platform_isr_interface.h"
#include "platform_assert.h"
#include "platform_constants.h"
#include "platform_isr.h"
#include "wwd_rtos_isr.h"

#if ( defined ( DEBUG ) && defined ( DEBUG_HARDFAULT ) )
#define WICED_TRIGGER_BREAKPOINT( ) do { __asm__("bkpt");  } while (0)
#endif
/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#ifndef SVC_irq
#error SVC_irq not defined - this will probably cause RTOS to fail to run
#endif

#ifndef PENDSV_irq
#error PENDSV_irq not defined - this will probably cause RTOS to fail to run
#endif

#ifndef SYSTICK_irq
#error SYSTICK_irq not defined - this will probably cause RTOS to fail to run
#endif

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
extern void reset_handler     ( void );


/******************************************************
 *               Variable Definitions
 ******************************************************/

/* Pointer to stack location */
extern void* link_stack_end;

PLATFORM_DEFINE_INTERRUPT_VECTOR_TABLE_ARRAY( interrupt_vector_table, PLATFORM_INTERRUPT_VECTOR_TABLE_HAS_VARIABLE_SIZE ) =
{
    (uint32_t) &link_stack_end           , // Initial stack location
    (uint32_t) reset_handler             , //Reset Handler
    (uint32_t) NMI_Handler               , //NMI Handler
    (uint32_t) my_HardFault_Handler         , //Hard Fault Handler
    (uint32_t) MemManage_Handler         , //MPU Fault Handler
    (uint32_t) BusFault_Handler          , //Bus Fault Handler
    (uint32_t) UsageFault_Handler        , //Usage Fault Handler
    (uint32_t) 0                         , //Reserved
    (uint32_t) 0                         , //Reserved
    (uint32_t) 0                         , //Reserved
    (uint32_t) 0                         , //Reserved
    (uint32_t) SVC_irq                   , //SVCall Handler
    (uint32_t) DebugMon_Handler          , //Debug Monitor Handler
    (uint32_t) 0                         , //Reserved
    (uint32_t) PENDSV_irq                , //PendSV Handler
    (uint32_t) stm32h7_systick           , //SysTick Handler
    (uint32_t) WWDG_IRQHandler                   , //Window WatchDog
    (uint32_t) PVD_IRQHandler                    , //PVD through EXTI Line detection
    (uint32_t) TAMP_STAMP_IRQHandler             , //Tamper and TimeStamps through the EXTI line
    (uint32_t) RTC_WKUP_IRQHandler               , //RTC Wakeup through the EXTI line
    (uint32_t) FLASH_IRQHandler                  , //FLASH
    (uint32_t) RCC_IRQHandler                    , //RCC
    (uint32_t) EXTI0_IRQHandler                  , //EXTI Line0
    (uint32_t) EXTI1_IRQHandler                  , //EXTI Line1
    (uint32_t) EXTI2_IRQHandler                  , //EXTI Line2
    (uint32_t) EXTI3_IRQHandler                  , //EXTI Line3
    (uint32_t) EXTI4_IRQHandler                  , //EXTI Line4
    (uint32_t) DMA1_Stream0_IRQHandler           , //DMA1 Stream 0
    (uint32_t) DMA1_Stream1_IRQHandler           , //DMA1 Stream 1
    (uint32_t) DMA1_Stream2_IRQHandler           , //DMA1 Stream 2
    (uint32_t) DMA1_Stream3_IRQHandler           , //DMA1 Stream 3
    (uint32_t) DMA1_Stream4_IRQHandler           , //DMA1 Stream 4
    (uint32_t) DMA1_Stream5_IRQHandler           , //DMA1 Stream 5
    (uint32_t) DMA1_Stream6_IRQHandler           , //DMA1 Stream 6
    (uint32_t) ADC_IRQHandler                    , //ADC1, ADC2 and ADC3s
    (uint32_t) CAN1_TX_IRQHandler                , //CAN1 TX
    (uint32_t) CAN1_RX0_IRQHandler               , //CAN1 RX0
    (uint32_t) CAN1_RX1_IRQHandler               , //CAN1 RX1
    (uint32_t) CAN1_SCE_IRQHandler               , //CAN1 SCE
    (uint32_t) EXTI9_5_IRQHandler                , //External Line[9:5]s
    (uint32_t) TIM1_BRK_TIM9_IRQHandler          , //TIM1 Break and TIM9
    (uint32_t) TIM1_UP_TIM10_IRQHandler          , //TIM1 Update and TIM10
    (uint32_t) TIM1_TRG_COM_TIM11_IRQHandler     , //TIM1 Trigger and Commutation and TIM11
    (uint32_t) TIM1_CC_IRQHandler                , //TIM1 Capture Compare
    (uint32_t) TIM2_IRQHandler                   , //TIM2
    (uint32_t) TIM3_IRQHandler                   , //TIM3
    (uint32_t) TIM4_IRQHandler                   , //TIM4
    (uint32_t) I2C1_EV_IRQHandler                , //I2C1 Event
    (uint32_t) I2C1_ER_IRQHandler                , //I2C1 Error
    (uint32_t) I2C2_EV_IRQHandler                , //I2C2 Event
    (uint32_t) I2C2_ER_IRQHandler                , //I2C2 Error
    (uint32_t) SPI1_IRQHandler                   , //SPI1
    (uint32_t) SPI2_IRQHandler                   , //SPI2
    (uint32_t) USART1_IRQHandler                 , //USART1
    (uint32_t) USART2_IRQHandler                 , //USART2
    (uint32_t) USART3_IRQHandler                 , //USART3
    (uint32_t) EXTI15_10_IRQHandler              , //External Line[15:10]s
    (uint32_t) RTC_Alarm_IRQHandler              , //RTC Alarm (A and B) through EXTI Line
    (uint32_t) OTG_FS_WKUP_IRQHandler            , //USB OTG FS Wakeup through EXTI line
    (uint32_t) TIM8_BRK_TIM12_IRQHandler         , //TIM8 Break and TIM12
    (uint32_t) TIM8_UP_TIM13_IRQHandler          , //TIM8 Update and TIM13
    (uint32_t) TIM8_TRG_COM_TIM14_IRQHandler     , //TIM8 Trigger and Commutation and TIM14
    (uint32_t) TIM8_CC_IRQHandler                , //TIM8 Capture Compare
    (uint32_t) DMA1_Stream7_IRQHandler           , //DMA1 Stream7
    (uint32_t) FMC_IRQHandler                    , //FMC
    (uint32_t) SDIO_irq                          , //SDIO
    (uint32_t) TIM5_IRQHandler                   , //TIM5
    (uint32_t) SPI3_IRQHandler                   , //SPI3
    (uint32_t) UART4_IRQHandler                  , //UART4
    (uint32_t) UART5_IRQHandler                  , //UART5
    (uint32_t) TIM6_DAC_IRQHandler               , //TIM6 and DAC1&2 underrun errors
    (uint32_t) TIM7_IRQHandler                   , //TIM7
    (uint32_t) DMA2_Stream0_IRQHandler           , //DMA2 Stream 0
    (uint32_t) DMA2_Stream1_IRQHandler           , //DMA2 Stream 1
    (uint32_t) DMA2_Stream2_IRQHandler           , //DMA2 Stream 2
    (uint32_t) DMA2_Stream3_IRQHandler           , //DMA2 Stream 3
    (uint32_t) DMA2_Stream4_IRQHandler           , //DMA2 Stream 4
    (uint32_t) ETH_IRQHandler                    , //Ethernet
    (uint32_t) ETH_WKUP_IRQHandler               , //Ethernet Wakeup through EXTI line
    (uint32_t) CAN2_TX_IRQHandler                , //CAN2 TX
    (uint32_t) CAN2_RX0_IRQHandler               , //CAN2 RX0
    (uint32_t) CAN2_RX1_IRQHandler               , //CAN2 RX1
    (uint32_t) CAN2_SCE_IRQHandler               , //CAN2 SCE
    (uint32_t) OTG_FS_IRQHandler                 , //USB OTG FS
    (uint32_t) DMA2_Stream5_IRQHandler           , //DMA2 Stream 5
    (uint32_t) DMA2_Stream6_IRQHandler           , //DMA2 Stream 6
    (uint32_t) DMA2_Stream7_IRQHandler           , //DMA2 Stream 7
    (uint32_t) USART6_IRQHandler                 , //USART6
    (uint32_t) I2C3_EV_IRQHandler                , //I2C3 event
    (uint32_t) I2C3_ER_IRQHandler                , //I2C3 error
    (uint32_t) OTG_HS_EP1_OUT_IRQHandler         , //USB OTG HS End Point 1 Out
    (uint32_t) OTG_HS_EP1_IN_IRQHandler          , //USB OTG HS End Point 1 In
    (uint32_t) OTG_HS_WKUP_IRQHandler            , //USB OTG HS Wakeup through EXTI
    (uint32_t) OTG_HS_IRQHandler                 , //USB OTG HS
    (uint32_t) DCMI_IRQHandler                   , //DCMI
    (uint32_t) 0                                 , //Reserved
    (uint32_t) HASH_RNG_IRQHandler               , //Hash and Rng
    (uint32_t) FPU_IRQHandler                    , //FPU
    (uint32_t) UART7_IRQHandler                  , //UART7
    (uint32_t) UART8_IRQHandler                  , //UART8
    (uint32_t) SPI4_IRQHandler                   , //SPI4
    (uint32_t) SPI5_IRQHandler                   , //SPI5
    (uint32_t) SPI6_IRQHandler                   , //SPI6
    (uint32_t) SAI1_IRQHandler                   , //SAI1
    (uint32_t) LTDC_IRQHandler                   , //LTDC
    (uint32_t) LTDC_ER_IRQHandler                , //LTDC error
    (uint32_t) DMA2D_IRQHandler                  , //DMA2D
};

/******************************************************
 *               Function Definitions
 ******************************************************/

WWD_RTOS_DEFINE_ISR(stm32h7_systick)
{
    HAL_IncTick();
    SYSTICK_irq();
}

WWD_RTOS_DEFINE_ISR(my_HardFault_Handler)
{
#if ( defined ( DEBUG ) && defined ( DEBUG_HARDFAULT ) )
   __ASM("MRS R0, MSP" );
   __ASM("MRS R1, PSP" );
   __ASM("MOV R2, LR" );
   __ASM("B HardFaultException_handler");
#endif
    while(1);
}
