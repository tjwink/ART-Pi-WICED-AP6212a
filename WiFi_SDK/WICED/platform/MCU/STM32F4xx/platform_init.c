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
 * Define default STM32F4xx initialisation functions
 */
#include "platform_init.h"
#include "platform_isr.h"
#include "platform_peripheral.h"
#include "platform_sleep.h"
#include "platform_config.h"
#include "platform_toolchain.h"
#include "platform/wwd_platform_interface.h"


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

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/


void platform_mcu_reset( void )
{
    NVIC_SystemReset( );

    /* Loop forever */
    while ( 1 )
    {
    }
}

/* STM32F2 common clock initialisation function
 * This brings up enough clocks to allow the processor to run quickly while initialising memory.
 * Other platform specific clock init can be done in init_platform() or init_architecture()
 */
WEAK void platform_init_system_clocks( void )
{
#if 0   /* If this is enabled, the LSE PA8 output will be disabled and never comes up again */
    RCC_DeInit( );
#endif /* if 0 */

    /* Configure Clocks */

    RCC_HSEConfig( HSE_SOURCE );
    RCC_WaitForHSEStartUp( );

    RCC_HCLKConfig( AHB_CLOCK_DIVIDER );
    RCC_PCLK2Config( APB2_CLOCK_DIVIDER );
    RCC_PCLK1Config( APB1_CLOCK_DIVIDER );

    /* Enable the PLL */
    FLASH_SetLatency( INT_FLASH_WAIT_STATE );
    FLASH_PrefetchBufferCmd( ENABLE );

    /* Use the clock configuration utility from ST to calculate these values
     * http://www.st.com/web/catalog/tools/FM147/CL1794/SC961/SS1743/LN1897
     */
    /* NOTE: The CPU Clock Frequency is independently defined in <WICED-SDK>/WICED/platform/<platform>/<platform>.mk */
#if defined(STM32F410xx) || defined(STM32F412xG) || defined(STM32F446xx) || defined(STM32F469_479xx)
    RCC_PLLConfig( PLL_SOURCE, PLL_M_CONSTANT, PLL_N_CONSTANT, PLL_P_CONSTANT, PLL_Q_CONSTANT, PLL_R_CONSTANT);
#else
    RCC_PLLConfig( PLL_SOURCE, PLL_M_CONSTANT, PLL_N_CONSTANT, PLL_P_CONSTANT, PLL_Q_CONSTANT );

#endif

    RCC_PLLCmd( ENABLE );

    while ( RCC_GetFlagStatus( RCC_FLAG_PLLRDY ) == RESET )
    {
    }
    RCC_SYSCLKConfig( SYSTEM_CLOCK_SOURCE );

    while ( RCC_GetSYSCLKSource( ) != 0x08 )
    {
    }

    /* Configure HCLK clock as SysTick clock source. */
    SysTick_CLKSourceConfig( SYSTICK_CLOCK_SOURCE );

}

WEAK void platform_init_memory( void )
{

}

void platform_init_mcu_infrastructure( void )
{
    uint8_t i;

#ifdef INTERRUPT_VECTORS_IN_RAM
    SCB->VTOR = 0x20000000; /* Change the vector table to point to start of SRAM */
#endif /* ifdef INTERRUPT_VECTORS_IN_RAM */

    /* Initialise watchdog */
    //platform_watchdog_init( );

    /* Initialise interrupt priorities */
    for ( i = 0; i < 96; i++ )
    {
        NVIC_SetPriority( (IRQn_Type) i, 0xf );
    }
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
    platform_init_rtos_irq_priorities();

    platform_init_peripheral_irq_priorities();

    /* Initialise GPIO IRQ manager */
    platform_gpio_irq_manager_init();

#ifndef WICED_DISABLE_MCU_POWERSAVE
    /* Initialise MCU powersave */
//    platform_mcu_powersave_init( );
//    platform_mcu_powersave_disable( );

//    /* Initialise RTC */
//    platform_rtc_init( );
#endif /* ifndef WICED_DISABLE_MCU_POWERSAVE */
}

void platform_init_connectivity_module( void )
{
    /* Ensure 802.11 device is in reset. */
    host_platform_init( );
}

WEAK void platform_init_external_devices( void )
{

}
