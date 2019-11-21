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
#include "wiced_time.h"
#include "platform_config.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define LOG_LABEL                       "duk:obj:time"
#define LOG_DEBUG_ENABLE                0

/*
 * Object properties
 */

/* Macro for defining internal properties by using the "\xff" prefix */
#define INTERNAL_PROPERTY(name)         ("\xff" name)

#define INTERNAL_PROPERTY_TIMER         INTERNAL_PROPERTY("timer")
#define INTERNAL_PROPERTY_TIMER_CB      INTERNAL_PROPERTY("timer_cb")

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

static duk_ret_t time_object_mdelay( duk_context* ctx )
{
    uint32_t ms;

    ms = duk_require_uint( ctx, 0 );

    LOGI( "Delaying for %lu ms", ms );

    wiced_rtos_delay_milliseconds( ms );

    return 0;
}

static duk_ret_t time_object_get_utc_time( duk_context* ctx )
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

static duk_ret_t time_object_set_utc_time( duk_context* ctx )
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

static wiced_result_t time_object_timer_callback( void* this )
{
    duk_context*            ctx;
    wiced_timed_event_t*    timer;
    wiced_duktape_state_t   state;

    LOGD( "Timer callback ('this'=%p)", this );

    wiced_assert( "Bad args", this != NULL );

    ctx = wiced_duktape_get_control( &state );
    if ( ctx == NULL )
    {
        LOGW( "Duktape heap destroyed- ignoring timer callback" );
        return WICED_SUCCESS;
    }

    duk_push_heapptr( ctx, this ); /* -> [this] */
    if ( duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_TIMER ) != 1 )
    {
        /* -> [this undefined] */

        duk_pop_2( ctx ); /* -> [] */
        wiced_duktape_put_control( &state );

        LOGW( "Timer does not exist- ignoring timer callback" );
        return WICED_SUCCESS;
    }
    /* -> [this timer] */

    timer = duk_get_pointer( ctx, -1 );
    duk_pop( ctx ); /* -> [this] */

    LOGD( "Stopping timer (%p) to prevent more triggers", timer );
    if ( wiced_rtos_deregister_timed_event( timer ) != WICED_SUCCESS )
    {
        LOGW( "Failed to stop timer (%p)- not sure what's going to happen",
              timer );
    }

    if ( duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_TIMER_CB ) != 1 )
    {
        /* -> [this undefined] */

        duk_pop_2( ctx ); /* -> [] */
        wiced_duktape_put_control( &state );

        LOGW( "Timer callback does not exist- ignoring timer callback" );
        return WICED_SUCCESS;
    }
    /* -> [this timer_cb] */

    duk_dup( ctx, -2 ); /* -> [this timer_cb this] */

    /* Remove callback to free up the timer */
    duk_del_prop_string( ctx, -1, INTERNAL_PROPERTY_TIMER_CB );
    duk_remove( ctx, -3 ); /* -> [timer_cb this] */

    LOGD( "Calling timer (%p) callback", timer );
    if ( duk_pcall_method( ctx, 0 /* nargs */ ) == DUK_EXEC_ERROR )
    {
        /* -> [err] */

        LOGE( "Failed to call timer (%p) callback (%s)", timer,
              duk_safe_to_string( ctx, -1 ));
    }
    /* -> [retval/err] */

    duk_pop( ctx ); /* -> [] */

    wiced_duktape_put_control( &state );

    return WICED_SUCCESS;
}

static duk_ret_t time_object_finalizer( duk_context* ctx )
{
    wiced_timed_event_t*    timer;

    /* Arguments:
     * 0 = 'this'
     * 1 = heap destruction flag    (boolean)
     */

    if ( !duk_get_boolean( ctx, 1 ))
    {
        /* Free resources only when heap is destroyed */
        return 0;
    }

    LOGD( "Finalizer called ('this'=%p)", duk_get_heapptr( ctx, 0 ));

    duk_dup( ctx, 0 ); /* -> [this] */

    if ( duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_TIMER ))
    {
        /* -> [this timer] */

        timer = duk_get_pointer( ctx, -1 );
        duk_pop( ctx ); /* -> [this] */

        LOGD( "Stopping any running timer (%p)", timer );
        wiced_rtos_deregister_timed_event( timer );

        LOGD( "Freeing timer (%p) memory", timer );
        free( timer );
    }
    else
    {
        /* -> [this undefined] */

        duk_pop( ctx ); /* -> [this] */
        LOGD( "No timer defined- why did we get called?" );
    }

    duk_pop( ctx ); /* -> [] */

    return 0;
}

static duk_ret_t time_object_set_timer( duk_context* ctx )
{
    wiced_timed_event_t*    timer;

    /* Arguments:
     * 0 = timeout in ms    (required)
     * 1 = callback         (required)
     */

    duk_require_int( ctx, 0 );
    duk_require_function( ctx, 1 );

    LOGD ("Setting timer for %d ms", duk_get_int( ctx, 0 ));

    duk_push_this( ctx ); /* -> [this] */

    /* We only support one timer */
    if ( duk_has_prop_string( ctx, -1, INTERNAL_PROPERTY_TIMER_CB ))
    {
        duk_pop( ctx ); /* -> [] */

        LOGE( "A timer has already been set" );
        return DUK_RET_ERROR;
    }

    if ( duk_has_prop_string( ctx, -1, INTERNAL_PROPERTY_TIMER ))
    {
        duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_TIMER );
        /* -> [this timer] */
        timer = duk_get_pointer(  ctx, -1 );
        duk_pop( ctx ); /* -> [this] */
    }
    else
    {
        LOGD( "Allocating memory for timer" );
        timer = calloc(1, sizeof( wiced_timed_event_t ));
        if ( timer == NULL )
        {
            LOGE( "Failed to allocate memory for timer" );
            return DUK_RET_ERROR;
        }

        duk_push_pointer( ctx, timer ); /* -> [this timer] */
        duk_put_prop_string( ctx, -2, INTERNAL_PROPERTY_TIMER ); /* -> [this] */

        /* Set the finalizer for 'this' to free timer memory */
        duk_push_c_function( ctx, time_object_finalizer, 2 );
        /* -> [this finalizer] */
        duk_set_finalizer( ctx, -2 ); /* -> [this] */
    }

    /* Save the callback to 'this' */
    duk_dup( ctx, 1 ); /* -> [this timer_cb] */
    duk_put_prop_string( ctx, -2, INTERNAL_PROPERTY_TIMER_CB ); /* -> [this] */

    LOGD( "Starting timer (%p)", timer );
    if ( wiced_rtos_register_timed_event( timer, WICED_DUKTAPE_WORKER_THREAD,
                                          time_object_timer_callback,
                                          duk_get_int( ctx, 0 ),
                                          duk_get_heapptr( ctx, -1 )) !=
         WICED_SUCCESS )
    {
        /* Remove callback from 'this' */
        duk_del_prop_string( ctx, -1, INTERNAL_PROPERTY_TIMER_CB );
        duk_pop( ctx ); /* -> [] */

        LOGE( "Failed to start timer (%p)", timer );
        return DUK_RET_ERROR;
    }

    duk_pop( ctx ); /* -> [] */

    return 0;
}

static duk_ret_t time_object_clear_timer( duk_context* ctx )
{
    wiced_timed_event_t*    timer;

    /* Arguments:
     * None
     */

    duk_push_this( ctx ); /* -> [this] */

    if ( duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_TIMER ))
    {
        /* -> [this timer] */

        timer = duk_get_pointer( ctx, -1 );
        duk_pop( ctx ); /* -> [this] */

        LOGD( "Stopping timer (%p)", timer );
        if ( wiced_rtos_deregister_timed_event( timer ) != WICED_SUCCESS )
        {
            LOGW( "Failed to stop timer (%p)", timer );
        }

        if ( duk_has_prop_string( ctx, -1, INTERNAL_PROPERTY_TIMER_CB ))
        {
            LOGD( "Removing timer callback" );
            duk_del_prop_string( ctx, -1, INTERNAL_PROPERTY_TIMER_CB );
        }
    }
    else
    {
        /* -> [this undefined] */

        duk_pop( ctx ); /* -> [this] */
        LOGD( "No timer defined" );
    }

    duk_pop( ctx ); /* -> [] */

    return 0;
}

static const duk_function_list_entry time_object_funcs[] =
{
      /* Name */        /* Function */              /* nargs */
    { "mDelay",         time_object_mdelay,         1 },
    { "getUtcTime",     time_object_get_utc_time,   0 },
    { "setUtcTime",     time_object_set_utc_time,   1 },
    { "setTimer",       time_object_set_timer,      2 },
    { "clearTimer",     time_object_clear_timer,    0 },
    { NULL, NULL, 0 }
};

static duk_ret_t time_object_constructor( duk_context* ctx )
{
    if ( !duk_is_constructor_call( ctx ))
    {
        LOGD( "Called as a normal function (non-constructor)" );

        /* Reject non-constructor call */
        return DUK_RET_TYPE_ERROR;
    }

    duk_push_this( ctx ); /* -> [this] */
    LOGD( "Constructor called ('this'=%p)", duk_get_heapptr( ctx, -1 ));
    duk_pop( ctx ); /* -> [] */

    return 0;
}

void wiced_duktape_object_time_init( duk_context* ctx )
{
    duk_push_c_function( ctx, time_object_constructor, 0 ); /* -> [func] */

    duk_push_object( ctx ); /* -> [func obj] */
    duk_put_function_list( ctx, -1, time_object_funcs );

    duk_put_prop_string( ctx, -2, "prototype" ); /* -> [func] */

    duk_put_global_string( ctx, "time" ); /* -> [] */
}
