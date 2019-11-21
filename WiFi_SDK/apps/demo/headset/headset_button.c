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

/*
 * @headset_button.c : Application's Button-list entries; What button does what...
 */

#include "wiced.h"
#include "headset.h"
#include "headset_button.h"
#include "headset_wlan.h"
#include "hashtable.h"

#include "button_manager.h"
#include "wiced_log.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define BUTTON_EVENT_ID(app_button, event, state)      ( ((uint32_t)app_button << 24) + ((uint32_t)event<<1) + state )

/******************************************************
 *                    Constants
 ******************************************************/

/* This Application's MMI has been designed to use 6 logical buttons and *should* be one-to-one mapped with physical buttons
 * provided from the Platform. In case, the platform provides lesser number of physical buttons, some of the MMI functionality
 * will get skipped.
 */
#define BUTTON_WORKER_STACK_SIZE            ( 2048 )
#define BUTTON_WORKER_QUEUE_SIZE            ( 8 )

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef wiced_result_t  (*button_event_function_t)(void);

typedef struct
{
    /* Logical action-enum */
    app_service_action_t    action;

    /* Primary routine which will be executed to service above action */
    button_event_function_t    primary;
    /* Secondary routine which need to be executed only when either primary routine has not been defined
     * or when Primary routines fails to service the action */
    button_event_function_t    secondary;
} button_action_function_table_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_result_t app_multi_func_short_release(void);
static wiced_result_t app_multi_func_long_release(void);
static wiced_result_t service_multi_functions_short_release(void);
static wiced_result_t service_multi_functions_long_release(void);
static wiced_result_t service_media_backward(void);
static wiced_result_t service_media_forward(void);
static wiced_result_t service_media_fast_forward_held(void);
static wiced_result_t service_media_fast_forward_release(void);
static wiced_result_t service_media_fast_rewind_held(void);
static wiced_result_t service_media_fast_rewind_release(void);
static wiced_result_t service_pause_play(void);
static wiced_result_t service_media_stop(void);
static wiced_result_t service_volume_up(void);
static wiced_result_t service_volume_down(void);
static wiced_result_t service_wlan_connect(void);
static wiced_result_t service_wlan_mode_switch(void);
static wiced_result_t service_EQ_setting(void);
static wiced_result_t app_pause_play(void);
static void app_service_action_arbitrator( app_service_action_t action );
static void app_button_event_handler( const button_manager_button_t* button, button_manager_event_t event, button_manager_button_state_t state );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const wiced_button_manager_configuration_t button_manager_configuration =
{
    .short_hold_duration     = 500  * MILLISECONDS,
    .medium_hold_duration    = 700  * MILLISECONDS,
    .long_hold_duration      = 1000  * MILLISECONDS,
    .very_long_hold_duration = 1500  * MILLISECONDS,
    .debounce_duration       = 150   * MILLISECONDS, /* typically a click takes around ~150-200 ms */

    .event_handler = app_button_event_handler,
};

/* Static button configuration */
static const wiced_button_configuration_t button_configurations[] =
{
#if ( WICED_PLATFORM_BUTTON_COUNT > 0 )
    [ BACK_BUTTON           ]     = { PLATFORM_BUTTON_1, BUTTON_CLICK_EVENT | BUTTON_VERY_LONG_DURATION_EVENT , 0 },
#endif
#if ( WICED_PLATFORM_BUTTON_COUNT > 1 )
    [ VOLUME_DOWN_BUTTON    ]     = { PLATFORM_BUTTON_2, BUTTON_CLICK_EVENT | BUTTON_LONG_DURATION_EVENT, 0 },
#endif
#if ( WICED_PLATFORM_BUTTON_COUNT > 2 )
    /* To register a very long duration event is required to get a next long event even we don't need very long event */
    [ VOLUME_UP_BUTTON      ]     = { PLATFORM_BUTTON_3, BUTTON_CLICK_EVENT | BUTTON_LONG_DURATION_EVENT | BUTTON_VERY_LONG_DURATION_EVENT, 0 },
#endif
#if ( WICED_PLATFORM_BUTTON_COUNT > 3 )
    [ PLAY_PAUSE_BUTTON     ]     = { PLATFORM_BUTTON_4, BUTTON_CLICK_EVENT | BUTTON_LONG_DURATION_EVENT | BUTTON_VERY_LONG_DURATION_EVENT, 0 },
#endif
#if ( WICED_PLATFORM_BUTTON_COUNT > 4 )
    [ FORWARD_BUTTON        ]     = { PLATFORM_BUTTON_6, BUTTON_CLICK_EVENT | BUTTON_VERY_LONG_DURATION_EVENT, 0 },
#endif
#if ( WICED_PLATFORM_BUTTON_COUNT > 5 )
    [ MULTI_FUNCTION_BUTTON ]     = { PLATFORM_BUTTON_5, BUTTON_CLICK_EVENT | BUTTON_LONG_DURATION_EVENT | BUTTON_VERY_LONG_DURATION_EVENT, 0 },
#endif
};

/* Button objects for the button manager */
button_manager_button_t buttons[] =
{
#if ( WICED_PLATFORM_BUTTON_COUNT > 0 )
    [ BACK_BUTTON ]             = { &button_configurations[ BACK_BUTTON ]        },
#endif
#if ( WICED_PLATFORM_BUTTON_COUNT > 1 )
    [ FORWARD_BUTTON ]          = { &button_configurations[ FORWARD_BUTTON ]     },
#endif
#if ( WICED_PLATFORM_BUTTON_COUNT > 2 )
    [ VOLUME_UP_BUTTON ]        = { &button_configurations[ VOLUME_UP_BUTTON ]   },
#endif
#if ( WICED_PLATFORM_BUTTON_COUNT > 3 )
    [ VOLUME_DOWN_BUTTON ]      = { &button_configurations[ VOLUME_DOWN_BUTTON ] },
#endif
#if ( WICED_PLATFORM_BUTTON_COUNT > 4 )
    [ PLAY_PAUSE_BUTTON ]       = { &button_configurations[ PLAY_PAUSE_BUTTON ]  },
#endif
#if ( WICED_PLATFORM_BUTTON_COUNT > 5 )
    [ MULTI_FUNCTION_BUTTON ]   = { &button_configurations[ MULTI_FUNCTION_BUTTON ]  },
#endif
};

static button_manager_t button_manager;
wiced_worker_thread_t   button_worker_thread;

/* A table for deciding how a particular action will be handled...
 * Table must be in same order as app_service_action_t enum variables */
static const button_action_function_table_t action_table[] =
{
    { NO_ACTION,                                NULL,                                    NULL },
    { ACTION_MULTI_FUNCTION_SHORT_RELEASE,      service_multi_functions_short_release,   app_multi_func_short_release },
    { ACTION_MULTI_FUNCTION_LONG_RELEASE,       service_multi_functions_long_release,    app_multi_func_long_release },
    { ACTION_PAUSE_PLAY,                        service_pause_play,                      app_pause_play },
    { ACTION_STOP,                              service_media_stop,                      NULL },
    { ACTION_FORWARD,                           service_media_forward,                   NULL },
    { ACTION_BACKWARD,                          service_media_backward,                  NULL },
    { ACTION_FAST_FORWARD_HELD,                 service_media_fast_forward_held,         NULL },
    { ACTION_FAST_FORWARD_RELEASE,              service_media_fast_forward_release,      NULL },
    { ACTION_FAST_REWIND_HELD,                  service_media_fast_rewind_held,          NULL },
    { ACTION_FAST_REWIND_RELEASE,               service_media_fast_rewind_release,       NULL },
    { ACTION_VOLUME_UP,                         service_volume_up,                       NULL },
    { ACTION_VOLUME_DOWN,                       service_volume_down,                     NULL },
    { ACTION_WLAN_INIT,                         NULL,                                    NULL },
    { ACTION_WLAN_DEINIT,                       NULL,                                    NULL },
    { ACTION_WLAN_CONNECT,                      service_wlan_connect,                    NULL },
    { ACTION_WLAN_DISCONNECT,                   NULL,                                    NULL },
    { ACTION_WLAN_MODE_SWITCH,                  service_wlan_mode_switch,                NULL },
    { ACTION_EQ_SETTING,                        service_EQ_setting,                      NULL },
};

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t headset_init_button_interface(void)
{
    wiced_result_t result = WICED_SUCCESS;

    HEADSET_CHECK_RESULT(wiced_rtos_create_worker_thread(&button_worker_thread,
                                                         WICED_DEFAULT_WORKER_PRIORITY,
                                                         BUTTON_WORKER_STACK_SIZE,
                                                         BUTTON_WORKER_QUEUE_SIZE));

    HEADSET_CHECK_RESULT(button_manager_init(&button_manager,
                                             &button_manager_configuration,
                                             &button_worker_thread,
                                             buttons,
                                             ARRAY_SIZE(buttons)));
_exit:
    return result;
}

static wiced_bool_t is_service_valid(wiced_app_service_t*  service)
{
    return ( (service != NULL && service->button_handler != NULL) ? WICED_TRUE: WICED_FALSE );
}

static wiced_result_t service_volume_up(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): Invalid Service\n", __func__);
        return result;
    }
    result = service->button_handler(ACTION_VOLUME_UP);

    return result;
}

static wiced_result_t service_volume_down(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_VOLUME_DOWN);
    return result;
}

static wiced_result_t service_media_backward(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_BACKWARD);
    return result;
}

static wiced_result_t service_wlan_connect(void)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_app_service_t*  service = NULL;

    service = wiced_get_entry(SERVICE_WLAN);

    if (service == NULL ||
        !is_service_valid(service) ||
        service->button_handler == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): Failed to get service entry\n", __func__);
        return WICED_ERROR;
    }

    /* Toggle */
    if (app_get_service_state(SERVICE_WLAN) != SERVICE_DISABLED)
    {
        /* Stop WLAN */
        result = service->button_handler(ACTION_WLAN_DISCONNECT);
    }
    else
    {
        /* Start WLAN */
        result = service->button_handler(ACTION_WLAN_CONNECT);
    }
    return result;
}

static wiced_result_t service_wlan_mode_switch(void)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_app_service_t*  service = NULL;

    service = wiced_get_entry(SERVICE_WLAN);

    if (service == NULL ||
        !is_service_valid(service) ||
        service->button_handler == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): Failed to get service entry\n", __func__);
        return WICED_ERROR;
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "\n*** Switching WLAN mode... ***\n");
    result = service->button_handler(ACTION_WLAN_MODE_SWITCH);

    return result;
}

static wiced_result_t service_EQ_setting(void)
{
    wiced_result_t result = WICED_ERROR;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "service_EQ_setting\n");

    /* TODO: audio EQ setting */
    return result;
}

static wiced_result_t service_media_forward(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_FORWARD);
    return result;
}

static wiced_result_t service_media_fast_forward_held(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_FAST_FORWARD_HELD);
    return result;
}

static wiced_result_t service_media_fast_forward_release(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_FAST_FORWARD_RELEASE);
    return result;
}

static wiced_result_t service_media_fast_rewind_held(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_FAST_REWIND_HELD);
    return result;
}

static wiced_result_t service_media_fast_rewind_release(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_FAST_REWIND_RELEASE);
    return result;
}

static wiced_result_t service_pause_play(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();

    if( !is_service_valid(service) )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): Invalid Service\n", __func__);
        return WICED_UNSUPPORTED;
    }

    result = service->button_handler(ACTION_PAUSE_PLAY);

    return result;
}

static wiced_result_t service_media_stop(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): Invalid Service\n", __func__);
        return  result;
    }

    result = service->button_handler(ACTION_STOP);
    return result;
}

static wiced_result_t service_multi_functions_short_release(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_MULTI_FUNCTION_SHORT_RELEASE);
    return result;
}

static wiced_result_t service_multi_functions_long_release(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    service = get_app_current_service();
    if( !is_service_valid(service) )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): Invalid Service\n", __func__);
        return result;
    }

    result = service->button_handler(ACTION_MULTI_FUNCTION_LONG_RELEASE);
    return result;
}

static wiced_result_t app_multi_func_short_release(void)
{
    /* TODO: if HFP is connected -- always do the dial-last-number */
    return WICED_SUCCESS;
}

static wiced_result_t app_multi_func_long_release(void)
{
    /* TODO: WICED_BT_HFP_HF_STATE_SLC_CONNECTED */
    return WICED_SUCCESS;
}

static wiced_result_t app_pause_play(void)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t*  service = NULL;
    /* FIXME : Here we can figure out which was last played audio service and
     * and send the play event back to that service only. For time being,
     * We are routing it to A2DP only*/
    service = wiced_get_entry(SERVICE_BT_A2DP);
    if ( service == NULL )
    {
        return WICED_UNSUPPORTED;
    }
    if (service->state == SERVICE_DISABLED)
    {
        return WICED_ERROR;
    }
    if(service->button_handler)
    {
        result = service->button_handler(ACTION_PAUSE_PLAY);
    }

    return result;
}

static void app_service_action_arbitrator( app_service_action_t action )
{
    wiced_result_t result = WICED_ERROR;

    if ( action == NO_ACTION )
    {
        return;
    }

    if ( action_table[action].primary )
    {
        result = action_table[action].primary();
    }

    if ( result != WICED_SUCCESS && action_table[action].secondary )
    {
        result = action_table[action].secondary();
    }

    return;
}

/**
 *
 * Button events thread callback
 *
 * @param button
 * @param state
 * @return
 */
static void app_button_event_handler( const button_manager_button_t* button, button_manager_event_t event, button_manager_button_state_t state )
{
    uint32_t button_event_id = BUTTON_EVENT_ID( ARRAY_POSITION(buttons, button), event, state );

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Button-%d [event: %x state:%x]\n", button->configuration->button, (unsigned int)event, (unsigned int)state);

    app_service_action_t new_action = NO_ACTION;

    switch ( button_event_id )
    {
        case BUTTON_EVENT_ID( MULTI_FUNCTION_BUTTON, BUTTON_CLICK_EVENT,               BUTTON_STATE_RELEASED ): new_action = ACTION_MULTI_FUNCTION_SHORT_RELEASE; break;
        case BUTTON_EVENT_ID( MULTI_FUNCTION_BUTTON, BUTTON_LONG_DURATION_EVENT,       BUTTON_STATE_HELD ):     new_action = ACTION_MULTI_FUNCTION_LONG_RELEASE; break;

        case BUTTON_EVENT_ID( BACK_BUTTON,           BUTTON_CLICK_EVENT,               BUTTON_STATE_RELEASED ): new_action = ACTION_BACKWARD; break;
        case BUTTON_EVENT_ID( BACK_BUTTON,           BUTTON_VERY_LONG_DURATION_EVENT,  BUTTON_STATE_HELD ):     new_action = ACTION_FAST_REWIND_HELD; break;
        case BUTTON_EVENT_ID( BACK_BUTTON,           BUTTON_VERY_LONG_DURATION_EVENT,  BUTTON_STATE_RELEASED ): new_action = ACTION_FAST_REWIND_RELEASE; break;

        case BUTTON_EVENT_ID( FORWARD_BUTTON,        BUTTON_CLICK_EVENT,               BUTTON_STATE_RELEASED ): new_action = ACTION_FORWARD; break;
        case BUTTON_EVENT_ID( FORWARD_BUTTON,        BUTTON_VERY_LONG_DURATION_EVENT,  BUTTON_STATE_HELD ):     new_action = ACTION_FAST_FORWARD_HELD; break;
        case BUTTON_EVENT_ID( FORWARD_BUTTON,        BUTTON_VERY_LONG_DURATION_EVENT,  BUTTON_STATE_RELEASED ): new_action = ACTION_FAST_FORWARD_RELEASE; break;

        case BUTTON_EVENT_ID( PLAY_PAUSE_BUTTON,     BUTTON_CLICK_EVENT,               BUTTON_STATE_RELEASED ): new_action = ACTION_PAUSE_PLAY; break;
        case BUTTON_EVENT_ID( PLAY_PAUSE_BUTTON,     BUTTON_LONG_DURATION_EVENT,       BUTTON_STATE_HELD ):     new_action = ACTION_EQ_SETTING; break;

        case BUTTON_EVENT_ID( VOLUME_UP_BUTTON,      BUTTON_CLICK_EVENT,               BUTTON_STATE_RELEASED ): new_action = ACTION_VOLUME_UP; break;
        case BUTTON_EVENT_ID( VOLUME_UP_BUTTON,      BUTTON_LONG_DURATION_EVENT,       BUTTON_STATE_HELD )    : new_action = ACTION_WLAN_CONNECT; break;
        case BUTTON_EVENT_ID( VOLUME_DOWN_BUTTON,    BUTTON_CLICK_EVENT,               BUTTON_STATE_RELEASED ): new_action = ACTION_VOLUME_DOWN; break;
        case BUTTON_EVENT_ID( VOLUME_DOWN_BUTTON,    BUTTON_LONG_DURATION_EVENT,       BUTTON_STATE_HELD )    : new_action = ACTION_WLAN_MODE_SWITCH; break;

        default: wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Button unhandled button_event_id: %x\n", (unsigned int)button_event_id); return;
    }

    app_service_action_arbitrator( new_action );

    return;
}
