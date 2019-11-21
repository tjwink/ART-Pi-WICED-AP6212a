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
 * WiFi Connection Manger Application
 *
 * This application snippet demonstrates how to use WiFi 1:1 connections.
 *
 * Features demonstrated
 *  - WPS Registrar
 *  - WPS Enrollee
 *  - P2P Group Owner
 *  - P2P Group Client
 *
 * Application Instructions
 *   1. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *   2. WPS Registrar:
 *      Press Pause/Play button on the 4390X WICED device
 *      Using a WPS capable Wi-Fi Device, press the WPS button on
 *      the Device to start a WPS setup session
 *   3. WPS Enrollee:
 *      Press Multi-Func button on the 4390X WICED device
 *      Using a WPS capable Wi-Fi Access Point, press the WPS button on
 *      the AP to start a WPS setup session
 *      If holding the Multi-Func long duration (over 4000ms),
 *      WICED will try to join with stored AP list in WIFI DCT
 *   4. P2P Group Owner:
 *      Press Forward button on the 4390X WICED device
 *      Using a P2P capable Wi-Fi Device, press the search button on
 *      the Device and press the 'WICED_P2P' to start P2P setup session
 *   5. P2P Group Client:
 *      Press Backward button on the 4390X WICED device
 *      Using a P2P capable Wi-Fi Device, press the search button on
 *      the Device and press the 'WICED_P2P' to start P2P setup session
 *   6. Connection progress is printed to the console
 *
 * The WPS Enrollee runs for up to 2 minutes before either successfully
 * connecting to the AP or timing out.
 *
 */
#include "wiced.h"
#include "app_button_interface.h"
#include "connection_manager.h"

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
    CONNECTION_ALL_EVENTS                      = -1,

    /* BUTTON EVENTs */
    CONNECTION_EVENT_WPS_REGISTRAR             = (1 << 0),
    CONNECTION_EVENT_WPS_ENROLLEE              = (1 << 1),
    CONNECTION_EVENT_WPS_ENROLLEE_REINVOKE     = (1 << 2),
    CONNECTION_EVENT_P2P_GROUP_OWNER           = (1 << 3),
    CONNECTION_EVENT_P2P_GROUP_CLIENT          = (1 << 4),
    CONNECTION_EVENT_P2P_GO_NEGOTIATION        = (1 << 5),
} CONNECTION_EVENTS_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/
/* Default P2P device name */
#define WIFI_CM_P2P_DEVNAME   "snip.CM.P2P"

/* Default GO intent value */
#define WIFI_CM_P2P_GO_INTENT 10

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static void app_main_loop(void);
static void app_p2p_result_callback(connection_p2p_result_t result);
static wiced_result_t button_event_handler(app_service_action_t action);

/******************************************************
 *               Variable Definitions
 ******************************************************/
static wiced_event_flags_t connection_events;

/******************************************************
 *               Function Definitions
 ******************************************************/

static void app_main_loop(void)
{
    wiced_result_t      result;
    uint32_t            events;
    connection_status_t existing_connection;
    connection_manager_context_t cm_context;
    uint8_t p2p_devname_len;

    /* Get default settings */
    connection_get_settings(&cm_context);

    /* Override P2P Device Name */
    p2p_devname_len = strlen(cm_context.p2p_details.wps_device_details.device_name);
    memset(cm_context.p2p_details.wps_device_details.device_name, 0x0, p2p_devname_len);
    strncpy(cm_context.p2p_details.wps_device_details.device_name, WIFI_CM_P2P_DEVNAME, p2p_devname_len);

    /* Set default settings */
    connection_set_settings(&cm_context);

    while(WICED_TRUE)
    {
        result = wiced_rtos_wait_for_event_flags(&connection_events, CONNECTION_ALL_EVENTS, &events,
                                                 WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if (result != WICED_SUCCESS)
        {
            continue;
        }

        /* Kill existing connections first */
        existing_connection = connection_get_status();
        if (connection_kill(existing_connection) != WICED_SUCCESS)
        {
            WPRINT_APP_INFO(( "Failed to stop existing connection %08X\n", existing_connection ));
        }

        /* To handle a connection event */
        if (events & CONNECTION_EVENT_WPS_REGISTRAR)
        {
            result = connection_launch(CONNECTION_WPS_REGISTRAR);

            if (result == WICED_SUCCESS)
            {
                WPRINT_APP_INFO( ("WPS Registrar connected\n") );
            }
            else
            {
                WPRINT_APP_INFO( ("WPS Registrar failed to connect\n") );
            }
        }

        if (events & CONNECTION_EVENT_WPS_ENROLLEE_REINVOKE)
        {
            result = connection_launch(CONNECTION_WPS_ENROLLEE_REINVOKE);

            if (result == WICED_SUCCESS)
            {
                WPRINT_APP_INFO( ("WPS Enrollee connected\n") );
            }
            else
            {
                WPRINT_APP_INFO( ("WPS Enrollee failed to connect\n") );
            }
        }

        if (events & CONNECTION_EVENT_WPS_ENROLLEE)
        {
            WPRINT_APP_INFO(("Trying to connect to stored AP first, and then starting WPS enrollee\n"));
            result = connection_launch(CONNECTION_WPS_ENROLLEE);

            if (result == WICED_SUCCESS)
            {
                WPRINT_APP_INFO( ("WPS Enrollee connected\n") );
            }
            else
            {
                WPRINT_APP_INFO( ("WPS Enrollee failed to connect\n") );
            }
        }

        if (events & CONNECTION_EVENT_P2P_GROUP_OWNER)
        {
            /* Register a result callback for p2p connection */
            connection_register_p2p_result_callback(app_p2p_result_callback);
            connection_launch(CONNECTION_P2P_GO);
        }

        if (events & CONNECTION_EVENT_P2P_GROUP_CLIENT)
        {
            /* Register a result callback for p2p connection */
            connection_register_p2p_result_callback(app_p2p_result_callback);
            connection_launch(CONNECTION_P2P_GC);
        }

        if (events & CONNECTION_EVENT_P2P_GO_NEGOTIATION)
        {
            /* Register a result callback for p2p connection */
            connection_register_p2p_result_callback(app_p2p_result_callback);

            /* Get default settings */
            connection_get_settings(&cm_context);

            /* Override P2P GO Intent */
            cm_context.p2p_details.group_owner_intent = WIFI_CM_P2P_GO_INTENT;

            /* Set default settings */
            connection_set_settings(&cm_context);

            /* Start P2P */
            connection_launch(CONNECTION_P2P_GO_NEGOTIATION);
        }
    }
}

static void app_p2p_result_callback(connection_p2p_result_t result)
{
    switch(result)
    {
        case CONNECTION_P2P_CONNECTED:
            WPRINT_APP_INFO( ("P2P connected\n") );
            break;
        case CONNECTION_P2P_DISCONNECTED:
            WPRINT_APP_INFO( ("P2P disconnected\n") );
            break;
        case CONNECTION_P2P_FAILED:
            WPRINT_APP_INFO( ("P2P failed to connect\n") );
            break;
        default:
            WPRINT_APP_INFO( ("Unknown P2P result ! : %02X\n", result) );
            break;
    }
}

static wiced_result_t button_event_handler(app_service_action_t action)
{
    wiced_result_t result = WICED_ERROR;

    switch( action )
    {
        case ACTION_PAUSE_PLAY:
            result = wiced_rtos_set_event_flags(&connection_events, CONNECTION_EVENT_WPS_REGISTRAR);
            break;
        case ACTION_FORWARD:
            result = wiced_rtos_set_event_flags(&connection_events, CONNECTION_EVENT_P2P_GROUP_OWNER);
            break;
        case ACTION_BACKWARD_SHORT_RELEASE:
            result = wiced_rtos_set_event_flags(&connection_events, CONNECTION_EVENT_P2P_GROUP_CLIENT);
            break;
        case ACTION_VOLUME_UP:
            result = wiced_rtos_set_event_flags(&connection_events, CONNECTION_EVENT_P2P_GO_NEGOTIATION);
            break;
        case ACTION_MULTI_FUNCTION_LONG_RELEASE:
            result = wiced_rtos_set_event_flags(&connection_events, CONNECTION_EVENT_WPS_ENROLLEE);
            break;
        case ACTION_MULTI_FUNCTION_SHORT_RELEASE:
            result = wiced_rtos_set_event_flags(&connection_events, CONNECTION_EVENT_WPS_ENROLLEE_REINVOKE);
            break;
        default:
            break;
    }

    return result;
}

void application_start( )
{
    /* Initialise the device */
    wiced_init( );

    WPRINT_APP_INFO( ( "\n\nWiFi Connection Manager. Press the key to test WiFi 1:1 connection\n" ) );
    WPRINT_APP_INFO( ( "Play/Pause            > Launch WPS Registrar\n" ) );
    WPRINT_APP_INFO( ( "Multi-Func short      > Launch WPS Enrollee\n" ) );
    WPRINT_APP_INFO( ( "Multi-Func long       > Launch WPS Enrollee but just join if stored AP is exist\n" ) );
    WPRINT_APP_INFO( ( "Forward               > Launch P2P Group Owner\n" ) );
    WPRINT_APP_INFO( ( "Backward              > Launch P2P Group Client\n" ) );
    WPRINT_APP_INFO( ( "Volume +              > Launch P2P GO Negotiation\n" ) );

    /* Init button interface; See app_button_interface.c */
    app_init_button_interface(button_event_handler);

    /* Init events */
    wiced_rtos_init_event_flags(&connection_events);

    /* Start a main loop */
    app_main_loop();
}

