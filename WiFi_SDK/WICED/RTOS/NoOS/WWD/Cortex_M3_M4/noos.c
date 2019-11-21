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
 *  Implementation of wiced_rtos.c for the case where no RTOS is used
 *
 *  This is a special implementation of the Wiced RTOS
 *  abstraction layer, which is designed to make Wiced work even with
 *  no RTOS.
 *  It provides Wiced with alternates to the functions for threads,
 *  semaphores and time functions.
 *
 *  Semaphores are simple integers, since accesses to an integer will be atomic.
 *
 */

#include "wwd_rtos.h"
#include "wwd_rtos_isr.h"
#include <stdint.h>
#include "platform_config.h"
//#include "platform_cmsis.h"
#include "platform_init.h"

volatile uint32_t noos_total_time = 0;

#define CYCLES_PER_SYSTICK  ( ( CPU_CLOCK_HZ / SYSTICK_FREQUENCY ) - 1 )

void NoOS_setup_timing( void )
{
    /* Setup the system handler priorities */
    SCB->SHP[11] = (uint8_t) 0x40; /* SysTick system handler priority */


    /* Setup the system tick */
    SysTick->LOAD = (uint32_t) ( CYCLES_PER_SYSTICK );
    SysTick->CTRL = (uint32_t) ( SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_CLKSOURCE_Msk );  /* clock source is processor clock - AHB */
}

void NoOS_stop_timing( void )
{
    /* Setup the system handler priorities */
    SCB->SHP[11] = (uint8_t) 0x40; /* SysTick system handler priority */


    /* Setup the system tick */
    SysTick->CTRL &= (uint32_t) ~( SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk );
}

/* NoOS interrupt priority setup. Called by platform_init_mcu_infrastructure() */
void platform_init_rtos_irq_priorities( void )
{
    /* Setup the system handler priorities */
    NVIC_SetPriority( MemoryManagement_IRQn,  0 ); /* Mem Manage system handler priority    */
    NVIC_SetPriority( BusFault_IRQn        ,  0 ); /* Bus Fault system handler priority     */
    NVIC_SetPriority( UsageFault_IRQn      ,  0 ); /* Usage Fault system handler priority   */
    NVIC_SetPriority( SVCall_IRQn          ,  0 ); /* SVCall system handler priority        */
    NVIC_SetPriority( DebugMonitor_IRQn    ,  0 ); /* Debug Monitor system handler priority */
    NVIC_SetPriority( PendSV_IRQn          ,  0 ); /* PendSV system handler priority        */
    NVIC_SetPriority( SysTick_IRQn         , 15 ); /* SysTick system handler priority       */
}

/* NoOS SysTick handler */
WWD_RTOS_DEFINE_ISR( NoOS_systick_irq )
{
    noos_total_time++;
}
