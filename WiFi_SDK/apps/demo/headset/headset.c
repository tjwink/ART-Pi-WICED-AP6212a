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

/** @file headset.c
 *
 * Headset Application
 * This application demonstrates a wireless Headset.
 *
 * How to Start WiFi Headset
 *  - To prepare a UPnP Server/Controller
 *  - Do modify WiFi connection information in wifi_config_dct.h
 *  - Build and Download this application into the WICED board
 *  - Do press & hold vol+ button after the board is booted up
 *   : RED led is turned on
 *  - To check if the WICED board is connected to the UPnP server
 *   : RED led is turned off and then GREEN led is turend on
 *  - Enjoy WiFi Headset
 *
 * How to Switch the WiFi mode (AP, STA and P2P)
 *  - Do press & hold vol- button after the board is booted up.
 *   : GREEN + RED leds are turned on to inform that the swithcing is reserved.
 *  - The WLAN mode will be toggled from a default mode in DCT to +1.
 *
 * LED Behavior
 *  - GREEN      : WLAN is connected
 *  - RED        : WLAN is ready to connect; waiting for connection
 *  - GREEN + RED: WLAN is waiting for mode switching; switching is reserved
 *
 * Parameters for low power consumption
 *  - Turn on WLAN powersave in headset_dct.h (NO_POWERSAVE_MODE -> PM2_POWERSAVE_MODE).
 *  - Select lower MCU clock frequency in headset_dct.h (320MHz -> 120MHz).
 *  - Do increase the TCP RX Window in include/wiced_default.h (to 15K or 20K).
 *  - Turn on MCU powersave in headset.mk and select sleep mode in headset_dct.h.
 *
 * Have some choppiness?
 *  - Do increase the http buffer and low threshold value in headset_dct.h.
 *  - Do increase the audio period size in headset_dct.h (256 -> 512).
 *   : Do increase the device period size in include/wiced_audio.h (1024 -> 2048).
 *   : You should check your available memory
 *
 * Features demonstrated
 *  - Wireless Connection : BT, WiFi(Direct, AP/STA)
 *  - Discovery Protocol  : UPnP
 *  - Supoorting Codecs   : AAC, FLAC, MP3, WAV
 *  - Button interfaces   : 6 buttons are mapped to control the Headset
 *
 */
#include <unistd.h>

#include "wiced.h"
#include "headset.h"
#include "headset_wlan.h"
#include "hashtable.h"
#include "platform_audio.h"
#include "wiced_log.h"
#ifdef HEADSET_CONS_SUPPORT
#include "command_console.h"
#include "dct/command_console_dct.h"
#include "wifi/command_console_wifi.h"
#include "p2p/command_console_p2p.h"
#include "platform/command_console_platform.h"
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/* Console */
#ifdef HEADSET_CONS_SUPPORT
#define HEADSET_CONS_MAX_LINE_LENGTH               ( 128 )
#define HEADSET_CONS_MAX_HISTORY_LENGTH            ( 20 )
#endif

/* Main Loop */
#define HEADSET_WORKER_THREAD_STACK_SIZE           ( 4096 )
#define HEADSET_QUEUE_MAX_ENTRIES                  ( 10 )

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
static void headset_deinit                                       (headset_context_t* headset_context);
static void headset_app_reboot                                   (headset_context_t* headset_context);
static void headset_worker_thread                                (uint32_t arg);
static wiced_result_t headset_init                               (headset_context_t** _headset_context);
static wiced_result_t headset_init_mcu_powersave                 (headset_context_t* headset_context);
static int headset_log_output_handler                            (WICED_LOG_LEVEL_T level, char *logmsg);

/******************************************************
 *               Variable Definitions
 ******************************************************/
#ifdef HEADSET_CONS_SUPPORT
static char cons_line_buffer[HEADSET_CONS_MAX_LINE_LENGTH];
static char cons_history_buffer[HEADSET_CONS_MAX_LINE_LENGTH * HEADSET_CONS_MAX_HISTORY_LENGTH];
static const command_t cons_commands[] =
{
    WIFI_COMMANDS
    P2P_COMMANDS
    PLATFORM_COMMANDS
    DCT_CONSOLE_COMMANDS
    CMD_TABLE_END
};
#endif

static uint8_t headset_worker_thread_stack[HEADSET_WORKER_THREAD_STACK_SIZE] __attribute__((section (".ccm")));
static headset_context_t* g_headset_context;

/******************************************************
 *               Function Definitions
 ******************************************************/
/*************************************************************
 * Headset logging subsystem
 *
 */
static int headset_log_output_handler(WICED_LOG_LEVEL_T level, char *logmsg)
{
    write(STDOUT_FILENO, logmsg, strlen(logmsg));

    return 0;
}

/*************************************************************
 * Headset main worker thread
 *
 */
static void headset_worker_thread( uint32_t args )
{
    app_queue_element_t message;
    wiced_result_t result;
    headset_context_t* headset_context = (headset_context_t *) args;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Headset Worker thread started...\n");

    while( WICED_TRUE )
    {
        result = wiced_rtos_pop_from_queue(&headset_context->queue, &message, WICED_NEVER_TIMEOUT );
        if( result != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error receiving event from app-queue\n");
            continue;
        }

        if( message.function )
        {
            message.function(message.arg);
        }

        if( message.wait_for_event_complete == WICED_TRUE )
        {
            headset_worker_wait_for_event_semaphore( message.event );
        }
    }
}

/*************************************************************
 * Push a new work to be processed by worker thread
 *
 */
void headset_worker_push_work( app_event_t event, wiced_result_t (*function)(void* arg), void* func_arg, wiced_bool_t wait )
{
    app_queue_element_t message;

    wiced_rtos_lock_mutex(&g_headset_context->lock);
    message.event = event;
    message.function = function;
    message.arg = func_arg;
    message.wait_for_event_complete = wait;
    wiced_rtos_push_to_queue(&g_headset_context->queue, &message, WICED_NO_WAIT);
    wiced_rtos_unlock_mutex(&g_headset_context->lock);
}

/*************************************************************
 * Send a signal to worker thread to wake the thread up
 *
 */
void headset_worker_signal_for_event_semaphore( app_event_t event, wiced_result_t completed )
{
    wiced_rtos_lock_mutex(&g_headset_context->lock);
    g_headset_context->event_mask = event;
    g_headset_context->event_result = completed;
    wiced_rtos_unlock_mutex(&g_headset_context->lock);

    wiced_rtos_set_semaphore(&g_headset_context->queue_semaphore);
}

/*************************************************************
 * Wait for a signal
 *
 * @return WICED_SUCCESS for success
 */
wiced_result_t headset_worker_wait_for_event_semaphore( app_event_t event )
{
    wiced_result_t result = WICED_SUCCESS;

    while( WICED_TRUE )
    {
        g_headset_context->event_wait |= event;
        wiced_rtos_get_semaphore( &g_headset_context->queue_semaphore, WICED_NEVER_TIMEOUT );
        if( g_headset_context->event_mask == event )
        {
            g_headset_context->event_wait &= ~event;
            result = g_headset_context->event_result;
            break;
        }
    }
    return result;
}

/*************************************************************
 * Check for pending event
 *
 * @return WICED_TRUE for having a pending event
 */
wiced_bool_t headset_worker_check_pending_event_semaphore( app_event_t event )
{
    if ( g_headset_context->event_wait & event )
    {
        return WICED_TRUE;
    }

    return WICED_FALSE;
}

/*************************************************************
 * Get a currnet service, WIFI/BT/...
 *
 * @return a current service
 */
wiced_app_service_t* get_app_current_service( void )
{
    wiced_app_service_t*  current_service;

    current_service = g_headset_context->current_service;
    return ( current_service );
}

/*************************************************************
 * Set a state to service
 *
 * @return WICED_SUCCESS for success
 */
wiced_result_t app_set_service_state(service_type_t service_type, service_state_t state)
{
    wiced_app_service_t* service;

    if(service_type == SERVICE_NONE)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): new service type is NONE. Can't proceed\n", __func__);
        return WICED_ERROR;
    }

    wiced_rtos_lock_mutex( &g_headset_context->lock );
    service = wiced_get_entry(service_type);
    if (service == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): new service is NULL. Can't proceed\n", __func__);
        wiced_rtos_unlock_mutex( &g_headset_context->lock );
        return WICED_ERROR;
    }

    service->state = state;
    wiced_rtos_unlock_mutex( &g_headset_context->lock );

    return WICED_SUCCESS;
}

/*************************************************************
 * Get a state for specified service
 *
 * @return a service state
 */
service_state_t app_get_service_state(service_type_t service_type)
{
    wiced_app_service_t* service;

    if(service_type == SERVICE_NONE)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): service type is NONE. Can't proceed\n", __func__);
        return WICED_ERROR;
    }

    service = wiced_get_entry(service_type);
    if (service == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"%s(): service is NULL. Can't proceed\n", __func__);
        return WICED_ERROR;
    }

    return service->state;
}

/*************************************************************
 * Disable a service
 *
 * @return WICED_SUCCESS for success
 */
wiced_result_t app_disable_service(service_type_t service_type)
{
    wiced_result_t result;
    /*if this is the current service, then reset the "current service"*/
    result = app_reset_current_service(service_type);
    result = app_set_service_state(service_type, SERVICE_DISABLED);

    return result;
}

/*************************************************************
 * Set a current service to specified one
 *
 * @return WICED_SUCCESS for success
 */
wiced_result_t app_set_current_service(service_type_t service_type)
{
    int i = 0;
    wiced_app_service_cell_t* cell = NULL;
    wiced_app_service_t* new_service;
    wiced_app_service_t* current;
    wiced_result_t result = WICED_ERROR;

    if(service_type == SERVICE_NONE)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): service type is NONE. Can't proceed\n", __func__);
        return result;
    }

    wiced_rtos_lock_mutex( &g_headset_context->lock );
    /* Get the service pointer to new service */
    new_service = wiced_get_entry(service_type);

    /* Get the current service pointer */
    current = get_app_current_service();

    if (new_service == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): new service is NULL. Can't proceed\n", __func__);
        goto SET_UNLOCK_AND_END;
    }

    if( current != NULL )
    {
        /* A lower priority service 'should' never interrupt a higher-priority service because
         * currently active higher priority service 'should' have erstwhile prevented them.
         */
        if ( current->priority > new_service->priority )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "New service[%d] lower priority than current [%d]..Aborting !!\n", new_service->priority, current->priority);
            goto SET_UNLOCK_AND_END;
        }
    }

    g_headset_context->current_service = new_service;

    for( i = 1; i <= new_service->priority; i++)
    {
        for( cell = (wiced_service_array[i]); cell!= NULL; cell = cell->next )
        {
            /* if same service, skip... */
            if( cell->service.type == new_service->type )
            {
                continue;
            }

            /* prevent the services which are not in prevented/preempted state already */
            if(cell->service.prevent_service &&
                (cell->service.state != SERVICE_PREEMPTED || cell->service.state != SERVICE_PREVENTED))
            {
                cell->service.prevent_service(&(cell->service));
            }
        }
    }
    result = WICED_SUCCESS;

SET_UNLOCK_AND_END:
    wiced_rtos_unlock_mutex( &g_headset_context->lock );
    return result;
}

/*************************************************************
 * Reset the current service
 *
 * @return WICED_SUCCESS for success
 */
wiced_result_t app_reset_current_service(service_type_t service_type)
{
    uint8_t i = 0;
    wiced_app_service_cell_t* cell = NULL;
    wiced_app_service_t *old_service;
    wiced_app_service_t *current;

    if(service_type == SERVICE_NONE)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): new service type is NONE. Can't proceed\n", __func__);
        return WICED_ERROR;
    }

    wiced_rtos_lock_mutex( &g_headset_context->lock );

    old_service = wiced_get_entry(service_type);
    current = get_app_current_service();
    if(old_service == NULL || current == NULL || (old_service->type != current->type))
    {
        wiced_rtos_unlock_mutex( &g_headset_context->lock );
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s(): service is NULL. Can't proceed\n", __func__);
        return WICED_ERROR;
    }

    old_service->state = SERVICE_IDLE;
    g_headset_context->current_service = NULL;
    /* Iterate through all low-priority services */
    for(i = old_service->priority; i >= 1; i--)
    {
        cell = (wiced_service_array[i]);

        for( cell = wiced_service_array[i]; cell!= NULL; cell = cell->next )
        {
            /* if same service, skip... */
            if( cell->service.type == service_type )
                continue;

            /* if some service had been preempted from playing */
            if( cell->service.state == SERVICE_PREEMPTED )
            {
                /* allow the 'preempted' one and make sure that others remain 'prevented' state */
                if( cell->service.allow_service)
                {
                    cell->service.allow_service( &(cell->service) );
                }
                wiced_rtos_unlock_mutex( &g_headset_context->lock );
                return WICED_SUCCESS;
            }

            /* keep allowing whatever would have been prevented */
            if( cell->service.allow_service )
                cell->service.allow_service( &(cell->service) );
        }
    }

    wiced_rtos_unlock_mutex( &g_headset_context->lock );
    return WICED_SUCCESS;
}

/*************************************************************
 * Init Headset MCU Powersave
 *
 * @return WICED_SUCCESS for success
 */
static wiced_result_t headset_init_mcu_powersave(headset_context_t* headset_context)
{
    wiced_result_t result = WICED_SUCCESS;
    powersave_dct_t powersave_dct = HEADSET_DCT_POWERSAVE_PARAMS(headset_context);
#if PLATFORM_BCM4390X_APPS_CPU_FREQ_24_AND_48_MHZ_ENABLED
    /* Init MCU Clock frequence */
    if (powersave_dct.mcu_clock_freq < PLATFORM_CPU_CLOCK_FREQUENCY_320_MHZ &&
        powersave_dct.mcu_clock_freq >= PLATFORM_CPU_CLOCK_FREQUENCY_24_MHZ)
    {
        platform_tick_stop();
        platform_cpu_clock_init(powersave_dct.mcu_clock_freq);
        platform_tick_start();
    }
#endif /* PLATFORM_BCM4390X_APPS_CPU_FREQ_24_AND_48_MHZ_ENABLED */

    /* Init MCU Powersave Tick mode */
    if (powersave_dct.mcu_sleep_tick_mode < PLATFORM_TICK_POWERSAVE_MODE_MAX &&
        powersave_dct.mcu_sleep_tick_mode >= PLATFORM_TICK_POWERSAVE_MODE_TICKLESS_ALWAYS)
    {
        platform_mcu_powersave_set_tick_mode(powersave_dct.mcu_sleep_tick_mode);
    }

    /* Init MCU Powersave mode */
    if (powersave_dct.mcu_sleep_mode < PLATFORM_MCU_POWERSAVE_MODE_MAX &&
        powersave_dct.mcu_sleep_mode >= PLATFORM_MCU_POWERSAVE_MODE_DEEP_SLEEP)
    {
        platform_mcu_powersave_set_mode(powersave_dct.mcu_sleep_mode);
    }

    wiced_platform_mcu_enable_powersave();

    return result;
}

/*************************************************************
 * Init Headset functions
 *
 * @return WICED_SUCCESS for success
 */
static wiced_result_t headset_init(headset_context_t** _headset_context)
{
    wiced_result_t result = WICED_SUCCESS;
    headset_context_t* headset_context = NULL;

    /* Initialise the device */
    HEADSET_CHECK_RESULT(wiced_init());

    /* Init the logging system
     * can printout audio library logs from LOG_INFO */
    HEADSET_CHECK_RESULT(wiced_log_init(WICED_LOG_INFO, headset_log_output_handler, NULL));

    /* Init headset app data */
    headset_context = calloc_named("headset_context", 1, sizeof(headset_context_t));

    if (headset_context == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocation headset context\n");
        return WICED_ERROR;
    }

    /* Init handle: service state */
    headset_context->current_service = NULL;
    headset_context->play_button_state = WICED_FALSE;
    headset_context->a2dp_media_state = WICED_FALSE;

    /* Init handle: lock */
    HEADSET_CHECK_RESULT(wiced_rtos_init_mutex(&headset_context->lock));
    HEADSET_CHECK_RESULT(wiced_rtos_init_semaphore(&headset_context->queue_semaphore));

    /* Init handle: queue */
    HEADSET_CHECK_RESULT(wiced_rtos_init_queue(&headset_context->queue,
                                               "headset_worker_queue",
                                               sizeof(app_queue_element_t),
                                               HEADSET_QUEUE_MAX_ENTRIES));


    /* Init handle; read DCT */
    HEADSET_CHECK_RESULT(headset_config_init(&headset_context->dct_tables));

    /* Save WLAN interface */
    headset_context->wlan_interface = HEADSET_DCT_WLAN_INTERFACE(headset_context);

    /* Save WLAN powersave mode */
    headset_context->wlan_powersave_mode = HEADSET_DCT_POWERSAVE_PARAMS(headset_context).wlan_pm_mode;

    /* Save MCU sleep mode */
    headset_context->mcu_sleep_mode = HEADSET_DCT_POWERSAVE_PARAMS(headset_context).mcu_sleep_mode;

    /* Init audio */
    HEADSET_CHECK_RESULT(platform_init_audio());

    /* Init UPnP AV renderer */
    HEADSET_CHECK_RESULT(headset_upnpavrender_init(&headset_context->upnpavrender_context,
                                                   &HEADSET_DCT_AUDIO_PARAMS(headset_context),
                                                   &HEADSET_DCT_POWERSAVE_PARAMS(headset_context),
                                                   headset_context->wlan_interface));

    /* Init hashtable */
    wiced_init_hashtable();

    /* Create worker thread */
    HEADSET_CHECK_RESULT(wiced_rtos_create_thread_with_stack(&headset_context->thread_handle,
                                                             WICED_DEFAULT_WORKER_PRIORITY,
                                                             "headset_worker_thread",
                                                             headset_worker_thread,
                                                             headset_worker_thread_stack,
                                                             HEADSET_WORKER_THREAD_STACK_SIZE,
                                                             headset_context));

    /* Init MCU Powersave */
    HEADSET_CHECK_RESULT(headset_init_mcu_powersave(headset_context));

    /* Init LEDs */
    HEADSET_LED_INIT
    HEADSET_LED_OFF

    /* return headset context */
    *_headset_context = headset_context;

    /* Add console */
#ifdef HEADSET_CONS_SUPPORT
    command_console_init(STDIO_UART,
                         HEADSET_CONS_MAX_LINE_LENGTH,
                         cons_line_buffer,
                         HEADSET_CONS_MAX_HISTORY_LENGTH,
                         cons_history_buffer,
                         " ");
    console_add_cmd_table(cons_commands);
#endif

_exit:
    return result;
}

/*************************************************************
 * De-initialize headset functions
 *
 */
static void headset_deinit(headset_context_t* headset_context)
{
    wiced_result_t result = WICED_SUCCESS;
    /* TODO: Deinit */
#ifdef HEADSET_CONS_SUPPORT
    HEADSET_CHECK_RESULT(command_console_deinit());
#endif
    /* UPnP AV renderer */
    HEADSET_CHECK_RESULT(headset_upnpavrender_deinit(headset_context->upnpavrender_context));
    /* WLAN service */
    HEADSET_CHECK_RESULT(headset_wlan_application_stop(HEADSET_DCT_WLAN_INTERFACE(headset_context)));
    /* Config */
    HEADSET_CHECK_RESULT(headset_config_deinit(&headset_context->dct_tables));
    /* Audio */
    HEADSET_CHECK_RESULT(platform_deinit_audio());
    /* Worker thread */
    HEADSET_CHECK_RESULT(wiced_rtos_delete_thread(&headset_context->thread_handle));
    /* Lock */
    HEADSET_CHECK_RESULT(wiced_rtos_deinit_mutex(&headset_context->lock));
    HEADSET_CHECK_RESULT(wiced_rtos_deinit_semaphore(&headset_context->queue_semaphore));
    /* Queue */
    HEADSET_CHECK_RESULT(wiced_rtos_deinit_queue(&headset_context->queue));
    /* Context */
    if (headset_context != NULL)
    {
        free(headset_context);
        headset_context = NULL;
    }
    /* Logging subsystem */
    wiced_log_shutdown();
_exit:
    return;
}

/*************************************************************
 * Reboot system to get out from urgent case
 *
 * @return WICED_SUCCESS for success
 */
static void headset_app_reboot(headset_context_t* headset_context)
{
    headset_deinit(headset_context);
    wiced_core_deinit();
    wiced_rtos_delay_milliseconds(WICED_APP_REBOOT_DELAY_MSECS);
    wiced_framework_reboot();
}

/* main() */
void application_start(void)
{
    wiced_result_t result = WICED_SUCCESS;
    headset_context_t* headset_context = NULL;
    headset_wlan_context_t headset_wlan_context = {0};

    /* Init Headset */
    HEADSET_CHECK_RESULT(headset_init(&headset_context));

    /* Save headset context as a global variable */
    g_headset_context = headset_context;

    /* Dump DCT */
    HEADSET_CHECK_RESULT(headset_config_dump(&headset_context->dct_tables));

    /* TODO: Start BT application */

    /* Start WLAN application */
    headset_wlan_context.wlan_interface = headset_context->wlan_interface;
    headset_wlan_context.wlan_interface_toggle = headset_context->wlan_interface;
    headset_wlan_context.wlan_powersave_mode = headset_context->wlan_powersave_mode;
    headset_wlan_context.upnpavrender_context = headset_context->upnpavrender_context;
    HEADSET_CHECK_RESULT(headset_wlan_application_start(&headset_wlan_context));

    /* Init Button interface */
    HEADSET_CHECK_RESULT(headset_init_button_interface());

    /* TODO: Need a loop for other task ? otherwise, just get out here */

    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Headset demo is started ...\n");
    return;
_exit:
    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Something wrong !!! System will be reboot\n");
    headset_app_reboot(headset_context);
}
