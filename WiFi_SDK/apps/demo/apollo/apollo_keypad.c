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

/** @file Apollo audio keypad handling.
 */

#include "apollo_keypad.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    ACTION_CONFIG_GATT_LAUNCH = 0,  /* Launch BTLE GATT configuration process */
    ACTION_CONFIG_VOLUME_UP,
    ACTION_CONFIG_VOLUME_DOWN,
} app_action_t;

typedef enum
{
    CONFIG_GATT_BUTTON = 0,
    VOLUME_UP_BUTTON,
    VOLUME_DOWN_BUTTON,
} application_button_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

static void app_button_event_handler( const button_manager_button_t* button, button_manager_event_t event, button_manager_button_state_t state );

/******************************************************
 *               Variables Definitions
 ******************************************************/

static const wiced_button_manager_configuration_t button_manager_configuration =
{
    .short_hold_duration     = 500  * MILLISECONDS,
    .debounce_duration       = 150  * MILLISECONDS,

    .event_handler           = app_button_event_handler,
};

/* Static button configuration */
static const wiced_button_configuration_t button_configurations[] =
{
#if (WICED_PLATFORM_BUTTON_COUNT > 0)
    [ CONFIG_GATT_BUTTON ] = { PLATFORM_BUTTON_1, BUTTON_CLICK_EVENT , ACTION_CONFIG_GATT_LAUNCH },
#endif
#if (WICED_PLATFORM_BUTTON_COUNT > 1)
    [ VOLUME_UP_BUTTON   ] = { PLATFORM_BUTTON_3, BUTTON_CLICK_EVENT , ACTION_CONFIG_VOLUME_UP   },
#endif
#if (WICED_PLATFORM_BUTTON_COUNT > 2)
    [ VOLUME_DOWN_BUTTON ] = { PLATFORM_BUTTON_2, BUTTON_CLICK_EVENT , ACTION_CONFIG_VOLUME_DOWN },
#endif
};

/* Button objects for the button manager */
static button_manager_button_t buttons[] =
{
#if (WICED_PLATFORM_BUTTON_COUNT > 0)
    [ CONFIG_GATT_BUTTON ] = { &button_configurations[ CONFIG_GATT_BUTTON ] },
#endif
#if (WICED_PLATFORM_BUTTON_COUNT > 1)
    [ VOLUME_UP_BUTTON   ] = { &button_configurations[ VOLUME_UP_BUTTON   ] },
#endif
#if (WICED_PLATFORM_BUTTON_COUNT > 2)
    [ VOLUME_DOWN_BUTTON ] = { &button_configurations[ VOLUME_DOWN_BUTTON ] },
#endif
};

/******************************************************
 *               Function Definitions
 ******************************************************/

static void app_button_event_handler( const button_manager_button_t* button, button_manager_event_t event, button_manager_button_state_t state )
{
    if (( g_apollo != NULL ) && ( event == BUTTON_CLICK_EVENT ))
    {
        uint32_t apollo_event = APOLLO_EVENT_NONE;

        switch (button->configuration->application_event)
        {
            case ACTION_CONFIG_GATT_LAUNCH:
                if ( g_apollo->button_gatt_launch_was_pressed != WICED_TRUE )
                {
                    g_apollo->button_gatt_launch_was_pressed = WICED_TRUE;
                }
                break;

            case ACTION_CONFIG_VOLUME_UP:
                apollo_event = APOLLO_EVENT_VOLUME_UP;
                break;

            case ACTION_CONFIG_VOLUME_DOWN:
                apollo_event = APOLLO_EVENT_VOLUME_DOWN;
                break;

            default:
                break;
        }

        if (apollo_event != APOLLO_EVENT_NONE)
        {
            wiced_rtos_set_event_flags(&g_apollo->events, apollo_event);
        }
    }

    return;
}


wiced_result_t apollo_button_handler_init(apollo_app_t* apollo)
{
    wiced_result_t result;

    result = wiced_rtos_create_worker_thread( &apollo->button_worker_thread, WICED_DEFAULT_WORKER_PRIORITY, BUTTON_WORKER_STACK_SIZE, BUTTON_WORKER_QUEUE_SIZE );
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_rtos_create_worker_thread() failed !\r\n") );
    result = button_manager_init( &apollo->button_manager, &button_manager_configuration, &apollo->button_worker_thread, buttons, ARRAY_SIZE( buttons ) );
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "button_manager_init() failed !\r\n") );

 _exit:
    return result;
}


void apollo_button_handler_deinit(apollo_app_t* apollo)
{
    button_manager_deinit( &apollo->button_manager );
    wiced_rtos_delete_worker_thread( &apollo->button_worker_thread );
}
