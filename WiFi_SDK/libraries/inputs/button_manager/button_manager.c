/**
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
 **/
#include "string.h"
#include "wiced_time.h"
#include "wiced_rtos.h"
#include "wiced_utilities.h"
#include "button_manager.h"
#include "platform_button.h"

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
 *               Function Declarations
 ******************************************************/

static void button_state_change_callback( platform_button_t id, wiced_bool_t new_state );
static wiced_result_t button_pressed_event_handler ( void* arg );
static wiced_result_t button_released_event_handler( void* arg );
static wiced_result_t deferred_button_timer_handler( void* arg );

static wiced_bool_t button_check_event_mask ( button_manager_button_t* button, uint16_t new_event );
static void button_check_for_double_click( button_manager_button_t* button,  button_manager_event_t* new_event );
static button_manager_event_t button_deduce_duration_event( uint32_t current_interval );
static button_manager_button_t* get_button( platform_button_t id );
static void timer_handler( void* arg );

/******************************************************
 *               Variables Definitions
 ******************************************************/

static button_manager_t* button_manager;

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t button_manager_init( button_manager_t* manager, const wiced_button_manager_configuration_t* configuration, wiced_worker_thread_t* thread, button_manager_button_t* buttons, uint32_t number_of_buttons )
{
    uint32_t a;

    memset( manager, 0, sizeof( *manager ) );

    manager->configuration = configuration;
    manager->worker_thread = thread;
    manager->buttons = buttons;
    manager->number_of_buttons = number_of_buttons;

    button_manager = manager;

    for ( a = 0; a < number_of_buttons; ++a )
    {
        platform_button_init( buttons[a].configuration->button );
        platform_button_enable( buttons[a].configuration->button );
        buttons[a].current_state = BUTTON_STATE_RELEASED;
    }

    platform_button_register_state_change_callback( button_state_change_callback );

    wiced_rtos_init_timer( &button_manager->timer, 100, timer_handler, manager );
    return WICED_SUCCESS;
}

wiced_result_t button_manager_deinit( button_manager_t* manager )
{
    uint32_t a;
    for ( a = 0; a < manager->number_of_buttons; ++a )
    {
        platform_button_disable( manager->buttons[a].configuration->button );
        platform_button_deinit( manager->buttons[a].configuration->button );
    }
    button_manager = NULL;
    return WICED_SUCCESS;
}

static void timer_handler( void* arg )
{
    button_manager_t* manager = (button_manager_t*)arg;

    wiced_time_get_time( &manager->timer_timestamp );
    wiced_rtos_send_asynchronous_event( button_manager->worker_thread, deferred_button_timer_handler, (void *) manager );
}

static wiced_result_t deferred_button_timer_handler( void* arg )
{
    uint32_t                 a;
    button_manager_t*        manager = (button_manager_t*) arg;
    button_manager_button_t* button;
    button_manager_event_t   new_held_event = 0;
    uint32_t                 duration;

    for ( a = 0; a < manager->number_of_buttons; ++a )
    {
        button = &manager->buttons[ a ];
        duration = manager->timer_timestamp - button->pressed_timestamp;

        if( button->current_state == BUTTON_STATE_RELEASED )
        {
            continue;
        }

        /* deduce the event depending on the duration */
        new_held_event = button_deduce_duration_event ( duration );

        /* timers should be mainly interested in duration-specific events; let release_handler only report Click events to the application */
        if ( new_held_event == BUTTON_CLICK_EVENT )
        {
            continue;
        }

        if( button_check_event_mask( button, new_held_event ) )
        {
            if ( button->last_sent_event != new_held_event )
            {
                button_manager->configuration->event_handler( button, new_held_event, button->current_state );
                button->last_sent_event = new_held_event;
            }
        }
    }
    return WICED_SUCCESS;
}

static void button_state_change_callback( platform_button_t id, wiced_bool_t new_state )
{
    button_manager_button_t* button = get_button( id );

    if ( button == NULL || button_manager == NULL || button_manager->worker_thread == NULL )
    {
        return;
    }

    if ( new_state == 1 )
    {
        wiced_time_get_time( &button->pressed_timestamp );
        wiced_rtos_start_timer( &button_manager->timer );
        wiced_rtos_send_asynchronous_event( button_manager->worker_thread, button_pressed_event_handler, (void *) button );
    }
    else
    {
        wiced_time_get_time( &button->released_timestamp );
        wiced_rtos_send_asynchronous_event( button_manager->worker_thread, button_released_event_handler, (void *) button );
    }
}

static wiced_result_t button_pressed_event_handler( void* arg )
{
    button_manager_button_t* button = (button_manager_button_t*)arg;

    if ( button->current_state == BUTTON_STATE_HELD )
    {
        return WICED_SUCCESS;
    }

    /* Button is pressed; update the state so that timer-handlers know it */
    button->current_state = BUTTON_STATE_HELD;

    return WICED_SUCCESS;
}

static wiced_result_t button_released_event_handler( void* arg )
{
    button_manager_button_t* button = (button_manager_button_t*)arg;
    button_manager_event_t  new_release_event = 0;
    uint32_t duration;

    if ( button->current_state == BUTTON_STATE_RELEASED )
    {
        return WICED_SUCCESS;
    }

    button->current_state = BUTTON_STATE_RELEASED;

    duration = button->released_timestamp - button->pressed_timestamp;

    /* If release event comes before debounce duration, ignore it */
    if ( duration <= button_manager->configuration->debounce_duration )
    {
        return WICED_SUCCESS;
    }

    /* deduce the event depending on the duration */
    new_release_event = button_deduce_duration_event( duration );

    /* Check if this Release is from 2nd click of a double-click event */
    button_check_for_double_click( button, &new_release_event );

    /* As the new state is Release and application has asked for this kind of event, send it irrespective of whether timer-hanlder
     * had sent it previously */
    if ( button_check_event_mask( button, new_release_event ) )
    {
        button_manager->configuration->event_handler( button, new_release_event, button->current_state );
    }

    /* reset the button's last-sent so that a new press/held after this release is handled properly */
    button->last_sent_event = 0;

    return WICED_SUCCESS;
}

static void button_check_for_double_click( button_manager_button_t* button,  button_manager_event_t* new_event )
{
    if( !button_check_event_mask( button, BUTTON_DOUBLE_CLICK_EVENT ) || *new_event != BUTTON_CLICK_EVENT )
    {
        return;
    }
    /* figure out the time-difference in two-releases */
    if ( (button->released_timestamp - button->last_released_timestamp) <= button_manager->configuration->double_click_interval  )
    {
        /* morph it as DOUBLE_CLICK */
        *new_event = BUTTON_DOUBLE_CLICK_EVENT;
    }

    button->last_released_timestamp = button->released_timestamp;

    return;

}

static wiced_bool_t button_check_event_mask ( button_manager_button_t* button, uint16_t new_event )
{
    if ( !button )
    {
        return WICED_FALSE;
    }

    return ( ( new_event & button->configuration->button_event_mask ) ? WICED_TRUE : WICED_FALSE );
}

static button_manager_event_t button_deduce_duration_event( uint32_t current_interval )
{
    button_manager_event_t  new_event = 0;

    if( current_interval > button_manager->configuration->debounce_duration && current_interval <= button_manager->configuration->short_hold_duration )
    {
        new_event = BUTTON_CLICK_EVENT;
    }

    else if( current_interval > button_manager->configuration->short_hold_duration && current_interval <= button_manager->configuration->medium_hold_duration )
    {
        new_event = BUTTON_SHORT_DURATION_EVENT;
    }

    else if( current_interval > button_manager->configuration->medium_hold_duration && current_interval <= button_manager->configuration->long_hold_duration )
    {
        new_event = BUTTON_MEDIUM_DURATION_EVENT;
    }

    else if( current_interval > button_manager->configuration->long_hold_duration && current_interval <= button_manager->configuration->very_long_hold_duration )
    {
        new_event = BUTTON_LONG_DURATION_EVENT;
    }
    else if( current_interval > button_manager->configuration->very_long_hold_duration )
    {
        new_event = BUTTON_VERY_LONG_DURATION_EVENT;
    }

    return new_event;
}

static button_manager_button_t* get_button( platform_button_t id )
{
    uint8_t a;

    for ( a = 0; a < button_manager->number_of_buttons; ++a )
    {
        if ( button_manager->buttons[ a ].configuration->button == id )
        {
            return &button_manager->buttons[ a ];
        }
    }

    return NULL;
}
