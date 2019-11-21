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
#include "platform_init.h"
#include "platform_cmsis.h"
#include "FreeRTOSConfig.h"

/** @file
 *
 */

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

/* FreeRTOS interrupt priority setup. Called by platform_init_mcu_infrastructure() */
void platform_init_rtos_irq_priorities( void )
{
    /* Setup the system handler priorities */
    NVIC_SetPriority( MemoryManagement_IRQn, 0                                    ); /* Mem Manage system handler priority    */
    NVIC_SetPriority( BusFault_IRQn        , 0                                    ); /* Bus Fault system handler priority     */
    NVIC_SetPriority( UsageFault_IRQn      , 0                                    ); /* Usage Fault system handler priority   */
    NVIC_SetPriority( SVCall_IRQn          , 0                                    ); /* SVCall system handler priority        */
    NVIC_SetPriority( DebugMonitor_IRQn    , 0                                    ); /* Debug Monitor system handler priority */
    NVIC_SetPriority( PendSV_IRQn          , configKERNEL_INTERRUPT_PRIORITY >> 4 ); /* PendSV system handler priority        */
    NVIC_SetPriority( SysTick_IRQn         , configKERNEL_INTERRUPT_PRIORITY >> 4 ); /* SysTick system handler priority       */
}
