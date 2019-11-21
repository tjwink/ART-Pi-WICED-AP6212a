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
 */

#include "platform_cmsis.h"
#include "platform_constants.h"
#include "platform_peripheral.h"
#include "platform_stdio.h"
#include "platform_isr.h"
#include "platform_isr_interface.h"
#include "platform_assert.h"
#include "wwd_assert.h"
#include "wwd_rtos.h"
#include "wiced_defaults.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#ifndef MAX_WATCHDOG_TIMEOUT_SECONDS
#define MAX_WATCHDOG_TIMEOUT_SECONDS    (22)
#endif

#ifdef APPLICATION_WATCHDOG_TIMEOUT_SECONDS
#define WATCHDOG_TIMEOUT                (APPLICATION_WATCHDOG_TIMEOUT_SECONDS)
#else
#define WATCHDOG_TIMEOUT                (MAX_WATCHDOG_TIMEOUT_SECONDS)
#endif /* APPLICATION_WATCHDOG_TIMEOUT_SECONDS */

#if (defined(APPLICATION_WATCHDOG_TIMEOUT_SECONDS) && (APPLICATION_WATCHDOG_TIMEOUT_SECONDS > MAX_WATCHDOG_TIMEOUT_SECONDS))
#error APPLICATION_WATCHDOG_TIMEOUT_SECONDS must NOT be larger than 22 seconds
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

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

platform_result_t platform_watchdog_init( void )
{
#ifndef WICED_DISABLE_WATCHDOG
    IWDG_HandleTypeDef IwdgHandle = { 0 };
    IwdgHandle.Instance = WATCHDOG_INSTANCE;
    /* Counter Reload Value = (LsiFreq(Hz) * Timeout(sec)) / (prescaler) */
    IwdgHandle.Init.Prescaler = WATCHDOG_PRESCALER;
    IwdgHandle.Init.Reload = ( LSI_FREQ_HZ * MAX_WATCHDOG_TIMEOUT_SECONDS ) / ( PRESCALAR );
    IwdgHandle.Init.Window = ( LSI_FREQ_HZ * MAX_WATCHDOG_TIMEOUT_SECONDS ) / ( PRESCALAR );

    if ( HAL_IWDG_Init( &IwdgHandle ) != HAL_OK )
    {
        /* Initialization Error */
        return PLATFORM_ERROR;
    }

    return PLATFORM_SUCCESS;
#else
    return PLATFORM_FEATURE_DISABLED;
#endif
}

platform_result_t platform_watchdog_kick( void )
{
#ifndef WICED_DISABLE_WATCHDOG
    IWDG_HandleTypeDef IwdgHandle = { 0 };
    IwdgHandle.Instance = WATCHDOG_INSTANCE;
    /* Counter Reload Value = (LsiFreq(Hz) * Timeout(sec)) / (prescaler) */
    IwdgHandle.Init.Prescaler = WATCHDOG_PRESCALER;
    IwdgHandle.Init.Reload = ( LSI_FREQ_HZ * MAX_WATCHDOG_TIMEOUT_SECONDS ) / ( PRESCALAR );
    IwdgHandle.Init.Window = ( LSI_FREQ_HZ * MAX_WATCHDOG_TIMEOUT_SECONDS ) / ( PRESCALAR );

    if ( HAL_IWDG_Refresh( &IwdgHandle ) != HAL_OK )
    {
        /* Refresh Error */
        return PLATFORM_ERROR;
    }

    return PLATFORM_SUCCESS;
#else
    return PLATFORM_FEATURE_DISABLED;
#endif
}

wiced_bool_t platform_watchdog_check_last_reset( void )
{
#ifndef WICED_DISABLE_WATCHDOG
    if ( __HAL_RCC_GET_FLAG( WATCHDOG_RESET_FLAG ) != RESET )
    {
        /* Clear the flag and return */
        __HAL_RCC_CLEAR_RESET_FLAGS( );
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
#else
    return WICED_FALSE;
#endif
}
