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

#include "wiced.h"
#include "wiced_duktape.h"
#include "platform_config.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define LOG_LABEL           "duk:mod:time"
#define LOG_DEBUG_ENABLE    0

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

static duk_ret_t time_module_mdelay( duk_context* ctx )
{
    uint32_t ms;

    ms = duk_require_uint( ctx, 0 );

    LOGI( "Delaying for %lu ms", ms );

    wiced_rtos_delay_milliseconds( ms );

    return 0;
}

static duk_ret_t time_module_get_utc_time( duk_context* ctx )
{
    wiced_utc_time_t time = 0;

    if ( wiced_time_get_utc_time( &time ) != WICED_SUCCESS )
    {
        LOGE( "Failed to get UTC time" );

        /* Return 'undefined' */
        return 0;
    }

    LOGD( "Got UTC time of %lu s", time );

    duk_push_int( ctx, time ); /* -> [time] */

    return 1;
}

static duk_ret_t time_module_set_utc_time( duk_context* ctx )
{
    wiced_utc_time_ms_t time;

    time = duk_require_uint( ctx, 0 );
    time *= 1000;

    LOGD( "Setting UTC time to %llu s", time / 1000 );
    if ( wiced_time_set_utc_time_ms( &time ) != WICED_SUCCESS )
    {
        LOGE( "Failed to set UTC time" );
    }

    return 0;
}

static const duk_function_list_entry time_module_funcs[] =
{
      /* Name */        /* Function */              /* nargs */
    { "mDelay",         time_module_mdelay,         1 },
    { "getUtcTime",     time_module_get_utc_time,   0 },
    { "setUtcTime",     time_module_set_utc_time,   1 },
    { NULL, NULL, 0 }
};

duk_ret_t wiced_duktape_module_time_register( duk_context* ctx )
{
    duk_push_object( ctx ); /* -> [obj] */
    duk_put_function_list( ctx, -1, time_module_funcs ); /* -> [obj] */

    return 1;
}
