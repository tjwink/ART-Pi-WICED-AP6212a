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
 *
 */

#include "wiced_rtos.h"
#include "wiced_time.h"
#include "wiced_defaults.h"
#include "wiced_low_power.h"
#include "tx_api.h"
#include "RTOS/wwd_rtos_interface.h"
#include "wiced_utilities.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "internal/wiced_internal_api.h"
#include "RTOS/wiced_rtos_common.h"
#include "tx_thread.h"
#include "TraceX.h"

#ifdef TX_ENABLE_EVENT_TRACE
#include "platform_config.h"  /* pick up platform specific tracex buffer location */
#endif /* ifdef TX_ENABLE_EVENT_TRACE */

/******************************************************
 *                      Macros
 ******************************************************/

#define WORKER_THREAD_MONITOR_UPDATE(worker, delay)    do { worker->monitor_info.last_update = host_rtos_get_time(); worker->monitor_info.longest_delay = delay; } while(0)

#define TX_TIMEOUT(timeout_ms)   ((timeout_ms != WICED_NEVER_TIMEOUT) ? ((ULONG)(timeout_ms / ms_to_tick_ratio)) : TX_WAIT_FOREVER )

/******************************************************
 *                    Constants
 ******************************************************/

#ifndef APPLICATION_STACK_SIZE
#define APPLICATION_STACK_SIZE   WICED_DEFAULT_APPLICATION_STACK_SIZE
#endif

#define SYSTEM_MONITOR_THREAD_STACK_SIZE   512


/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef VOID (*native_thread_t)(ULONG);
typedef void (*native_timer_handler_t)(ULONG);

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    event_handler_t function;
    void*           arg;
} wiced_event_message_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static void application_thread_main    ( ULONG thread_input );
static void application_thread_cleanup ( TX_THREAD* thread_ptr, UINT condition );

/******************************************************
 *               Variable Definitions
 ******************************************************/

const uint32_t        ms_to_tick_ratio = 1000 / SYSTICK_FREQUENCY;
wiced_worker_thread_t wiced_hardware_io_worker_thread;
wiced_worker_thread_t wiced_networking_worker_thread;
#ifndef WICED_DISABLE_WATCHDOG
static wiced_thread_t system_monitor_thread_handle;
#endif /* WICED_DISABLE_WATCHDOG */
static ULONG          WICED_DEEP_SLEEP_SAVED_VAR( before_deep_sleep_time );

/******************************************************
 *               Function Definitions
 ******************************************************/

/**
 *  Main function - starts ThreadX
 *  Called from the crt0 _start function
 *
 */
#ifndef ALTERNATE_MAIN
int main( void )
{
#if defined ( __IAR_SYSTEMS_ICC__ )
/* IAR allows init functions in __low_level_init(), but it is run before global
 * variables have been initialised, so the following init still needs to be done
 * When using GCC, this is done in crt0_GCC.c
 */
    extern void platform_init_mcu_infrastructure( void );
    extern void platform_init_external_devices( void );

    platform_init_mcu_infrastructure( );
    platform_init_external_devices( );

#endif /* #elif defined ( __IAR_SYSTEMS_ICC__ ) */    /* Enter the ThreadX kernel.  */
    tx_kernel_enter( );
    return 0;
}
#endif /* ifndef ALTERNATE_MAIN */


/**
 *  Application Define function - creates and starts the application thread
 *  Called by ThreadX whilst it is initialising
 *
 *  @param first_unused_memory: unused parameter - required to match prototype
 */
void tx_application_define( void *first_unused_memory )
{
    TX_THREAD* app_thread_handle;
    char*      app_thread_stack;
    UINT       result;

    UNUSED_PARAMETER(first_unused_memory);

    /* Restore system clock taking into account time spent in deep-sleep */
    if ( WICED_DEEP_SLEEP_IS_WARMBOOT_HANDLE( ) )
    {
        tx_time_set( before_deep_sleep_time + wiced_deep_sleep_ticks_since_enter( ) );
    }

    /* Create the application thread.  */
    app_thread_handle = (TX_THREAD*)MALLOC_OBJECT("app thread", TX_THREAD);
    app_thread_stack  = (char*)malloc_named("app stack", APPLICATION_STACK_SIZE);

    result = tx_thread_create( app_thread_handle, (char*)"app thread", application_thread_main, 0, app_thread_stack, APPLICATION_STACK_SIZE, WICED_APPLICATION_PRIORITY, WICED_APPLICATION_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START );

    if ( TX_SUCCESS != result )
    {
        free ( app_thread_handle );
        free ( app_thread_stack );
        app_thread_handle = NULL;
        app_thread_stack  = NULL;
    }
    else
    {
        ( void )tx_thread_entry_exit_notify( app_thread_handle, application_thread_cleanup );
    }
}

void application_thread_cleanup( TX_THREAD *thread_ptr, UINT condition )
{
    /* Determine if the thread was exited. */
    if ( thread_ptr && condition == TX_THREAD_EXIT )
    {
        malloc_transfer_to_curr_thread(thread_ptr->tx_thread_stack_start);
        malloc_transfer_to_curr_thread(thread_ptr);
        tx_thread_terminate( thread_ptr );
        malloc_leak_check(thread_ptr, LEAK_CHECK_THREAD);
        tx_thread_delete( thread_ptr );
        free( thread_ptr->tx_thread_stack_start );
        free( thread_ptr );
    }
}

void application_thread_main( ULONG thread_input )
{
    UNUSED_PARAMETER( thread_input);

#ifndef WICED_DISABLE_WATCHDOG
    /* Start the watchdog kicking thread */
    wiced_rtos_create_thread( &system_monitor_thread_handle, RTOS_HIGHEST_PRIORITY, "system monitor", (wiced_thread_function_t)system_monitor_thread_main, SYSTEM_MONITOR_THREAD_STACK_SIZE, NULL );
#endif /* WICED_DISABLE_WATCHDOG */

#ifdef TX_ENABLE_EVENT_TRACE
    {
        wiced_result_t        result;
        wiced_tracex_config_t tracex_config;

        tracex_config.loop_rec = WICED_FALSE;
        tracex_config.filter = 0;
        tracex_config.tcp_server.enable = WICED_FALSE;
        tracex_config.tcp_server.port = WICED_TRACEX_TCP_SERVER_PORT;
        tracex_config.tcp_server.max_data_len = WICED_TRACEX_TCP_MAX_PACKET_LENGTH;
        tracex_config.tcp_server.timeout = WICED_TRACEX_TCP_CONNECT_TIMEOUT;
        tracex_config.tcp_server.num_retries = WICED_TRACEX_TCP_CONNECTION_NUM_RETRIES;
        SET_IPV4_ADDRESS(tracex_config.tcp_server.ip, WICED_TRACEX_TCP_SERVER_IP);
        tracex_config.buf.addr = WICED_TRACEX_BUFFER_ADDRESS;
        tracex_config.buf.size = WICED_TRACEX_BUFFER_SIZE;
        tracex_config.buf.obj_cnt = WICED_TRACEX_OBJECT_COUNT;

        result = wiced_tracex_enable(&tracex_config);
        wiced_assert( "TraceX enable failed", result == WICED_SUCCESS );
        REFERENCE_DEBUG_ONLY_VARIABLE( result );
    }
#endif /* TX_ENABLE_EVENT_TRACE */

    application_start( );
    malloc_leak_check(NULL, LEAK_CHECK_THREAD);
}

wiced_result_t wiced_rtos_create_thread( wiced_thread_t* thread, uint8_t priority, const char* name, wiced_thread_function_t function, uint32_t stack_size, void* arg )
{
    wiced_result_t result;

    thread->stack = malloc_named( "stack", stack_size );
    if (thread->stack == NULL)
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }
    malloc_transfer_to_curr_thread( thread->stack );

    result = (wiced_result_t) host_rtos_create_thread_with_arg( WICED_GET_THREAD_HANDLE( thread ), function, name, thread->stack, stack_size, priority, (uint32_t)arg );

    if ( result != WICED_WWD_SUCCESS )
    {
        free( thread->stack );
        thread->stack = NULL;
    }

    return result;
}

wiced_result_t wiced_rtos_create_thread_with_stack( wiced_thread_t* thread, uint8_t priority, const char* name, wiced_thread_function_t function, void* stack, uint32_t stack_size, void* arg )
{
    wiced_result_t result;

    memset(thread, 0, sizeof(wiced_thread_t));
    result =  (wiced_result_t) host_rtos_create_thread_with_arg( WICED_GET_THREAD_HANDLE( thread ), function, name, stack, stack_size, priority, (uint32_t)arg );

    return result;
}

wiced_result_t wiced_rtos_delete_thread( wiced_thread_t* thread )
{
    wiced_result_t result =  (wiced_result_t) host_rtos_finish_thread( WICED_GET_THREAD_HANDLE( thread ) );

    if ( result != WICED_WWD_SUCCESS )
    {
        return result;
    }

    if ( thread->stack != NULL )
    {
        malloc_transfer_to_curr_thread( thread->stack );
    }

    malloc_leak_check( &thread->handle, LEAK_CHECK_THREAD );

    result =  (wiced_result_t) host_rtos_delete_terminated_thread( WICED_GET_THREAD_HANDLE( thread ) );

    if ( result == WICED_WWD_SUCCESS )
    {
        if ( thread->stack != NULL )
        {
            free( thread->stack );
            thread->stack = NULL;
        }
    }

    return result;
}


wiced_result_t wiced_rtos_is_current_thread( wiced_thread_t* thread )
{
    if ( tx_thread_identify( ) == &thread->handle )
    {
        return WICED_SUCCESS;
    }
    else
    {
        return WICED_ERROR;
    }
}


wiced_result_t wiced_rtos_check_stack( void )
{
    /* TODO: Add stack checking here. */

    return WICED_SUCCESS;
}

wiced_result_t wiced_rtos_thread_yield ( void )
{
    tx_thread_relinquish();
    return WICED_SUCCESS;
}

wiced_result_t wiced_rtos_thread_force_awake( wiced_thread_t* thread )
{
    if ( tx_thread_wait_abort( &thread->handle ) == TX_SUCCESS )
    {
        return WICED_SUCCESS;
    }
    else
    {
        return WICED_ERROR;
    }
}


wiced_result_t wiced_time_get_time( wiced_time_t* time_ptr )
{
    *time_ptr = (wiced_time_t) ( tx_time_get( ) * ms_to_tick_ratio );
    return WICED_SUCCESS;
}


wiced_result_t wiced_time_set_time( const wiced_time_t* time_ptr )
{
    tx_time_set( ( *time_ptr ) / ms_to_tick_ratio );
    return WICED_SUCCESS;
}


wiced_result_t wiced_rtos_init_mutex( wiced_mutex_t* mutex )
{
    /* Mutex uses priority inheritance */
    if ( tx_mutex_create( mutex, (CHAR*) "", TX_INHERIT ) != TX_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}


wiced_result_t wiced_rtos_lock_mutex( wiced_mutex_t* mutex )
{
    if ( tx_mutex_get( mutex, TX_WAIT_FOREVER ) != TX_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}


wiced_result_t wiced_rtos_unlock_mutex( wiced_mutex_t* mutex )
{
    if ( tx_mutex_put( mutex ) != TX_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}


wiced_result_t wiced_rtos_deinit_mutex( wiced_mutex_t* mutex )
{
    if ( tx_mutex_delete( mutex ) != TX_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}


wiced_result_t wiced_rtos_init_queue( wiced_queue_t* queue, const char* name, uint32_t message_size, uint32_t number_of_messages )
{
    uint32_t queue_size = message_size * number_of_messages;
    wiced_result_t result;

    UNUSED_PARAMETER( name );

    if ( ( message_size % 4 ) > 0 )
    {
        return WICED_ERROR;
    }

    queue->buffer = (void*) malloc_named( "queue", queue_size );
    if ( queue->buffer == NULL )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }
    malloc_transfer_to_curr_thread( queue->buffer );

    result =  (wiced_result_t) host_rtos_init_queue( WICED_GET_QUEUE_HANDLE( queue ), queue->buffer, queue_size, message_size );

    if ( result != WICED_WWD_SUCCESS )
    {
        free( queue->buffer );
        queue->buffer = NULL;
    }

    return result;
}


wiced_result_t wiced_rtos_deinit_queue( wiced_queue_t* queue )
{
    wiced_result_t result;

    malloc_transfer_to_curr_thread( queue->buffer );

    result =  (wiced_result_t) host_rtos_deinit_queue( WICED_GET_QUEUE_HANDLE( queue ) );

    if ( result == WICED_WWD_SUCCESS )
    {
        free( queue->buffer );
        queue->buffer = NULL;
    }

    return result;
}

wiced_result_t wiced_rtos_get_queue_occupancy( wiced_queue_t* queue, uint32_t *count )
{
    *count = queue->handle.tx_queue_enqueued;
    return WICED_SUCCESS;
}

wiced_result_t wiced_rtos_is_queue_empty( wiced_queue_t* queue )
{
    return ( queue->handle.tx_queue_enqueued == 0 ) ? WICED_SUCCESS : WICED_ERROR;
}

wiced_result_t wiced_rtos_is_queue_full( wiced_queue_t* queue )
{
    return ( queue->handle.tx_queue_available_storage == 0 ) ? WICED_SUCCESS : WICED_ERROR;
}

wiced_result_t wiced_rtos_init_timer( wiced_timer_t* timer, uint32_t time_ms, timer_handler_t function, void* arg )
{
    if ( tx_timer_create( timer, (char*)"", (native_timer_handler_t)function, (ULONG)arg, time_ms / ms_to_tick_ratio, time_ms / ms_to_tick_ratio, 0 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}


wiced_result_t wiced_rtos_start_timer( wiced_timer_t* timer )
{
    if ( tx_timer_change( timer, timer->tx_timer_internal.tx_timer_internal_re_initialize_ticks, timer->tx_timer_internal.tx_timer_internal_re_initialize_ticks ) != TX_SUCCESS )
    {
        return WICED_ERROR;
    }

    if ( tx_timer_activate( timer ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}


wiced_result_t wiced_rtos_stop_timer( wiced_timer_t* timer )
{
    if ( tx_timer_deactivate( timer ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}


wiced_result_t wiced_rtos_deinit_timer( wiced_timer_t* timer )
{
    if ( tx_timer_delete( timer ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}


wiced_result_t wiced_rtos_is_timer_running( wiced_timer_t* timer )
{
    return ( (void*) timer->tx_timer_internal.tx_timer_internal_list_head != TX_NULL ) ? WICED_SUCCESS : WICED_ERROR;
}


wiced_result_t wiced_rtos_init_event_flags( wiced_event_flags_t* event_flags )
{
    if ( tx_event_flags_create( event_flags, (CHAR*)"Event flags" ) != TX_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}


wiced_result_t wiced_rtos_wait_for_event_flags( wiced_event_flags_t* event_flags, uint32_t flags_to_wait_for, uint32_t* flags_set, wiced_bool_t clear_set_flags, wiced_event_flags_wait_option_t wait_option, uint32_t timeout_ms )
{
    UINT option = 0;

    option |= ( clear_set_flags == WICED_TRUE )      ? ( 1 << 0 ) : 0;
    option |= ( wait_option == WAIT_FOR_ALL_EVENTS ) ? ( 1 << 1 ) : 0;

    if ( tx_event_flags_get( event_flags, flags_to_wait_for, option, (ULONG*)flags_set, TX_TIMEOUT( timeout_ms ) ) != TX_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}


wiced_result_t wiced_rtos_set_event_flags( wiced_event_flags_t* event_flags, uint32_t flags_to_set )
{
    if ( tx_event_flags_set( event_flags, flags_to_set, TX_OR ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}


wiced_result_t wiced_rtos_deinit_event_flags( wiced_event_flags_t* event_flags )
{
    if ( tx_event_flags_delete( event_flags ) != TX_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

WICED_SLEEP_EVENT_HANDLER( sleep_rtos_event_handler )
{
    if ( event == WICED_SLEEP_EVENT_ENTER )
    {
        /* Save current time before entering deep-sleep */
        before_deep_sleep_time = tx_time_get( );
    }
}
