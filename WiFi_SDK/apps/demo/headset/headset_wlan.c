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

/** @headset_wlan.c
 */
#include "wiced.h"
#include "headset.h"
#include "headset_dct.h"
#include "headset_wlan.h"
#include "hashtable.h"

#include "connection_manager.h"
#include "wiced_log.h"
#include "upnp_av_render.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define HEADSET_AUDIO_VOLUME_INTERVAL        (5)
#define HEADSET_P2P_DEVICE_NAME              "WICED_HEADSET_P2P"

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
static wiced_result_t headset_wlan_init_service       (void* arg);
static wiced_result_t headset_wlan_deinit_service     (void* arg);
static wiced_result_t headset_wlan_connect            (wiced_interface_t wlan_iface, uint8_t wlan_powersave_mode);
static wiced_result_t headset_wlan_disconnect         (wiced_interface_t wlan_iface);
static wiced_result_t headset_wlan_powersave_enable   (wiced_interface_t wlan_iface, uint8_t wlan_powersave_mode);
static wiced_result_t headset_wlan_add_service        (headset_wlan_context_t* headset_wlan_context);
static wiced_result_t headset_wlan_prevent_service    (wiced_app_service_t* service);
static wiced_result_t headset_wlan_allow_service      (wiced_app_service_t* service);
static wiced_result_t headset_wlan_button_handler     (app_service_action_t action);
static wiced_result_t headset_wlan_connect_service    (void* arg);
static wiced_result_t headset_wlan_disconnect_service (void* arg);
static wiced_result_t headset_wlan_mode_switch_service(void* arg);
static void           headset_wlan_p2p_connection_cb  (connection_p2p_result_t result);
static void           headset_wlan_link_down_cb       (void);
static void           headset_wlan_preinit_p2p        (void);

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

/*************************************************************
 * Init WLAN service
 *
 * @return WICED_SUCCESS for success
 */
static wiced_result_t headset_wlan_init_service(void* arg)
{
    wiced_result_t result = WICED_SUCCESS;

    /* Gets an argument pointer for future use, but not used for now */
    UNUSED_PARAMETER(arg);

    /* Init WICED WLAN */
    HEADSET_CHECK_RESULT(wiced_wlan_connectivity_init());

    /* To skip IV boundary checking while reordering the AMPDU */
    wwd_wifi_set_iovar_value(IOVAR_STR_APIVTW_OVERRIDE, WICED_TRUE, WWD_STA_INTERFACE);

    /* TODO: Init other WLAN functions here */


_exit:
    if (result == WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Success to init Headset WLAN!\n");
    }
    else
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to init Headset WLAN!\n");
    }
    return result;
}

/*************************************************************
 * Deinit WLAN service
 *
 * @return WICED_SUCCESS for success
 */
static wiced_result_t headset_wlan_deinit_service(void* arg)
{
    wiced_result_t result = WICED_SUCCESS;

    /* Gets an argument pointer for future use, but not used for now */
    UNUSED_PARAMETER(arg);

    /* Deinit WICED WLAN */
    HEADSET_CHECK_RESULT(wiced_wlan_connectivity_deinit());

    /* TODO: DeInit other WLAN functions here */


_exit:
    if (result == WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Success to Deinit Headset WLAN!\n");
    }
    else
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to Deinit Headset WLAN!\n");
    }
    return result;
}

/*************************************************************
 * WiFi connection request
 *
 * @return WICED_SUCCESS for success
 */
wiced_result_t headset_wlan_connect(wiced_interface_t wlan_iface, uint8_t wlan_powersave_mode)
{
    wiced_result_t result = WICED_SUCCESS;

    /* Make a pending state until the disconnection is done */
    HEADSET_CHECK_RESULT(app_set_current_service(SERVICE_WLAN));

    /* Turn RED LED on to indicate the headset is trying to connect to wifi */
    HEADSET_LED_WLAN_CONNECTING

    /* for P2P interface */
    if (wlan_iface == WICED_P2P_INTERFACE)
    {
        /* Start WiFi direct group client */
        HEADSET_CHECK_RESULT(connection_launch(CONNECTION_P2P_GC));
        /* Register a result callback for p2p connection
         * The p2p thread will return a connection result to this callback */
        connection_register_p2p_result_callback(headset_wlan_p2p_connection_cb);
    }
    /* for STA interface */
    else if(wlan_iface == WICED_STA_INTERFACE)
    {
        /* Register a callback for link event */
        HEADSET_CHECK_RESULT(wiced_network_register_link_callback(NULL, headset_wlan_link_down_cb, WICED_STA_INTERFACE));

        /* Bring up the wlan network, blocked until the DHCP is done */
        HEADSET_CHECK_RESULT(wiced_network_up(wlan_iface, WICED_USE_EXTERNAL_DHCP_SERVER, NULL));
    }
    /* for AP iterface */
    else if (wlan_iface == WICED_AP_INTERFACE)
    {
        wiced_ip_setting_t headset_ip_settings =
        {
            INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
            INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS( 255,255,255,  0 ) ),
            INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
        };

        HEADSET_CHECK_RESULT(wiced_network_up(wlan_iface, WICED_USE_INTERNAL_DHCP_SERVER, &headset_ip_settings));
    }
    /* Unknown interface */
    else
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Invalid WLAN interface!: %d\n", wlan_iface);
        result = WICED_ERROR;
    }

_exit:
    return result;
}

/*************************************************************
 * WiFi Direct disconnection request
 *
 * @return WICED_SUCCESS for success
 */
wiced_result_t headset_wlan_disconnect(wiced_interface_t wlan_iface)
{
    wiced_result_t result = WICED_SUCCESS;

    if (wlan_iface == WICED_P2P_INTERFACE)
    {
        HEADSET_CHECK_RESULT(connection_kill(CONNECTION_P2P_GC));
        connection_register_p2p_result_callback(NULL);
    }
    else
    {
        HEADSET_CHECK_RESULT(wiced_network_down(wlan_iface));
        HEADSET_CHECK_RESULT(wiced_network_deregister_link_callback(NULL, headset_wlan_link_down_cb, WICED_STA_INTERFACE));
    }

_exit:
    return result;
}

/*************************************************************
 * WLAN powersave enable
 *
 */
static wiced_result_t headset_wlan_powersave_enable(wiced_interface_t wlan_iface, uint8_t wlan_powersave_mode)
{
    wiced_result_t result = WICED_SUCCESS;

    if (wlan_powersave_mode == PM1_POWERSAVE_MODE)
    {
        /* PM 1 */
        HEADSET_CHECK_RESULT(wiced_wifi_enable_powersave_interface(wlan_iface));
    }
    else if (wlan_powersave_mode == PM2_POWERSAVE_MODE)
    {
        /* PM 2, default return-to-sleep 200ms */
        HEADSET_CHECK_RESULT(wiced_wifi_enable_powersave_with_throughput_interface(200, wlan_iface));
    }
    else
    {
        /* Disable Powersave */
        HEADSET_CHECK_RESULT(wiced_wifi_disable_powersave_interface(wlan_iface));
    }

_exit:
    return result;
}

/*************************************************************
 * Add WLAN service with callbacks
 *
 * @return WICED_SUCCESS for success
 */
static wiced_result_t headset_wlan_add_service(headset_wlan_context_t* headset_wlan_context)
{
    wiced_result_t result = WICED_ERROR;
    wiced_app_service_t cell;

    cell.priority                   = SERVICE_WLAN_PRIORITY;
    cell.type                       = SERVICE_WLAN;
    cell.state                      = SERVICE_DISABLED;

    cell.prevent_service            = headset_wlan_prevent_service;
    cell.allow_service              = headset_wlan_allow_service;
    cell.button_handler             = headset_wlan_button_handler;
    cell.arg1                       = (void*) headset_wlan_context;

    result = wiced_add_entry(&cell);

    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to add WLAN Service entry [Error:%d]\n", result);
        return result;
    }

    return result;
}

/*************************************************************
 * WLAN connect service callback
 *
 */
static wiced_result_t headset_wlan_connect_service(void* arg)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_app_service_t* service = NULL;
    headset_wlan_context_t* headset_wlan_context = NULL;
    headset_upnpavrender_context_t* upnpavrender_context = NULL;
    wiced_interface_t wlan_iface = WICED_STA_INTERFACE;
    uint8_t wlan_powersave_mode = NO_POWERSAVE_MODE;

    service = (wiced_app_service_t *) arg;

    if( service == NULL ||
        service->type != SERVICE_WLAN ||
        service->arg1 == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Invalid service\n");
        return WICED_ERROR;
    }

    /* Get WLAN context */
    headset_wlan_context = (headset_wlan_context_t *) service->arg1;

    /* Get a toggled interface which is from user input */
    wlan_iface = headset_wlan_context->wlan_interface_toggle;

    /* Replace the original interface with new one.
     * The first original value is from DCT(wifi_config_dct.h).
     * It'll be used for destroying the previous connection.
     */
    headset_wlan_context->wlan_interface = wlan_iface;

    /* Get powersave parameter */
    wlan_powersave_mode = headset_wlan_context->wlan_powersave_mode;

    /* Get upnpavrender context */
    upnpavrender_context = headset_wlan_context->upnpavrender_context;

    /* Start wlan connection */
    HEADSET_CHECK_RESULT(headset_wlan_connect(wlan_iface, wlan_powersave_mode));

    /* Check P2P connection status */
    if (wlan_iface == WICED_P2P_INTERFACE)
    {
        /* Wait an asynchronous connection event from p2p thread */
        HEADSET_CHECK_RESULT(headset_worker_wait_for_event_semaphore(WICED_WLAN_P2P_ASYNC_RESULT))
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Headset WLAN connected! : %02X\n", result);

    /* Start UPnP AV renderer */
    HEADSET_CHECK_RESULT(headset_upnpavrender_start(upnpavrender_context));

    /* Enable WLAN Poewrsave */
    if (wlan_iface != WICED_AP_INTERFACE)
    {
        HEADSET_CHECK_RESULT(headset_wlan_powersave_enable(wlan_iface, wlan_powersave_mode));
    }

    /* Disable MCU powersave to ready for wifi streaming */
    if (platform_mcu_powersave_is_permitted() == WICED_TRUE)
    {
        wiced_platform_mcu_disable_powersave();
    }

    /* Set a service state to Enabled */
    HEADSET_CHECK_RESULT(app_set_service_state(SERVICE_WLAN, SERVICE_ENABLED));

    /* Notify that the WLAN is connected */
    HEADSET_LED_WLAN_CONNECTED

_exit:
    if (result != WICED_SUCCESS)
    {
        app_set_service_state(SERVICE_WLAN, SERVICE_STOPPING);
        headset_wlan_disconnect_service(arg);
    }
    return result;
}

/*************************************************************
 * WLAN disconnect service callback
 *
 */
static wiced_result_t headset_wlan_disconnect_service(void* arg)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_app_service_t* service = NULL;
    headset_wlan_context_t* headset_wlan_context = NULL;
    headset_upnpavrender_context_t* upnpavrender_context = NULL;
    wiced_interface_t wlan_iface = WICED_STA_INTERFACE;

    service = (wiced_app_service_t *) arg;

    if( service == NULL ||
        service->type != SERVICE_WLAN ||
        service->arg1 == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Invalid service\n");
        return WICED_ERROR;
    }

    /* Get WLAN context */
    headset_wlan_context = (headset_wlan_context_t *) service->arg1;

    /* Save WLAN parameters */
    wlan_iface = headset_wlan_context->wlan_interface;
    upnpavrender_context = headset_wlan_context->upnpavrender_context;

    /* Stop UPnP AV renderer */
    if (headset_upnpavrender_stop(upnpavrender_context) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to stop upnpavrender\n");
    }

    /* Stop wlan connection */
    if (headset_wlan_disconnect(wlan_iface) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to stop WLAN\n");
    }

    /* Enable MCU powersave. Should check BT status later */
    if (platform_mcu_powersave_is_permitted() == WICED_FALSE)
    {
        wiced_platform_mcu_enable_powersave();
    }

    /* Disable WLAN service */
    if (app_disable_service(SERVICE_WLAN) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to disable WLAN service\n");
    }

    /* Turn off the LEDs */
    HEADSET_LED_OFF

    return result;
}


/*************************************************************
 * WLAN mode switching service callback
 *
 */
static wiced_result_t headset_wlan_mode_switch_service(void* arg)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_app_service_t* service = NULL;
    headset_wlan_context_t* headset_wlan_context = NULL;
    headset_upnpavrender_context_t* upnpavrender_context = NULL;

    service = (wiced_app_service_t *) arg;

    if( service == NULL ||
        service->type != SERVICE_WLAN ||
        service->arg1 == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Invalid service\n");
        return WICED_ERROR;
    }

    /* Get WLAN context */
    headset_wlan_context = (headset_wlan_context_t *) service->arg1;

    /* Get upnpavrender context */
    upnpavrender_context = headset_wlan_context->upnpavrender_context;

    /* Check if need to down the existing connection */
    if (app_get_service_state(SERVICE_WLAN) == SERVICE_PENDING ||
        app_get_service_state(SERVICE_WLAN) == SERVICE_STOPPING)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Something is pending! Couldn't switch the wlan mode now! Try again.\n");
        return WICED_ERROR;
    }
    else if (app_get_service_state(SERVICE_WLAN) == SERVICE_ENABLED)
    {
        /* Destroy previous connection */
        HEADSET_CHECK_RESULT(app_set_service_state(SERVICE_WLAN, SERVICE_STOPPING));
        HEADSET_CHECK_RESULT(headset_wlan_disconnect_service(arg));
    }

    /* Toggle the WLAN mode and then start the WLAN
     * If the default interface is P2P in wifi DCT,
     * the toggle interface will be STA->AP->P2P->STA->AP->P2P...
     */
    if (headset_wlan_context->wlan_interface_toggle >= WICED_P2P_INTERFACE)
    {
        headset_wlan_context->wlan_interface_toggle = WICED_STA_INTERFACE;
    }
    else
    {
        /* Move to next interface */
        headset_wlan_context->wlan_interface_toggle += 1;
    }

    /* Change the interface in upnpavrender as well */
    upnpavrender_context->service_params.interface = headset_wlan_context->wlan_interface_toggle;
    upnpavrender_context->service_params.audio_client_params.interface = headset_wlan_context->wlan_interface_toggle;

    /* Mark the state to PENDING */
    HEADSET_CHECK_RESULT(app_set_service_state(SERVICE_WLAN, SERVICE_PENDING));

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "\n\n*** WLAN is switching to '%s' ***\n\n",
                  headset_wlan_context->wlan_interface_toggle == WICED_AP_INTERFACE ? "AP" :
                  headset_wlan_context->wlan_interface_toggle == WICED_STA_INTERFACE ? "STA" :
                  headset_wlan_context->wlan_interface_toggle == WICED_P2P_INTERFACE ? "P2P" : "Unknown");


    /* Start WLAN with changed interface */
    HEADSET_CHECK_RESULT(headset_wlan_connect_service(arg));

_exit:
    return result;
}

/*************************************************************
 * WLAN prevent service callback
 *
 */
static wiced_result_t headset_wlan_prevent_service(wiced_app_service_t *service)
{
    wiced_result_t result = WICED_SUCCESS;

    if( service == NULL || service->type != SERVICE_WLAN)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "[WLAN] Invalid argument\n");
        return WICED_ERROR;
    }

    if( service->state == SERVICE_PLAYING_AUDIO )
    {
        /* TODO: Service is prevented while playing audio */
    }

    HEADSET_CHECK_RESULT(app_set_service_state(SERVICE_WLAN, SERVICE_PREVENTED));

_exit:
    return result;
}

/*************************************************************
 * WLAN allow service callback
 *
 */
static wiced_result_t headset_wlan_allow_service(wiced_app_service_t *service)
{
    wiced_result_t result = WICED_SUCCESS;

    if( service == NULL || service->type != SERVICE_WLAN )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "[WLAN] Invalid argument\n");
        return WICED_ERROR;
    }

    if( service->state == SERVICE_PREEMPTED || service->state == SERVICE_PREVENTED )
    {
        /* TODO: Servie is allowed from prevent */
    }

    HEADSET_CHECK_RESULT(app_set_current_service(SERVICE_WLAN));
    HEADSET_CHECK_RESULT(app_set_service_state(SERVICE_WLAN, SERVICE_IDLE));

_exit:
    return result;
}

/*************************************************************
 * WLAN STA Link down handler
 *
 */
static void headset_wlan_link_down_cb(void)
{
    wiced_app_service_t* service = NULL;

    service = wiced_get_entry(SERVICE_WLAN);

    if (service == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to get service entry\n");
        return;
    }

    if (app_get_service_state(SERVICE_WLAN) == SERVICE_STOPPING)
    {
        return;
    }

    app_set_service_state(SERVICE_WLAN, SERVICE_STOPPING);
    headset_worker_push_work(WICED_WLAN_DISCONNECT, headset_wlan_disconnect_service, (void *)service, WICED_FALSE);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Headset WLAN disconnected!\n");
}

/*************************************************************
 * WLAN P2P connection events handler
 *
 */
static void headset_wlan_p2p_connection_cb(connection_p2p_result_t result)
{
    wiced_app_service_t* service = NULL;

    service = wiced_get_entry(SERVICE_WLAN);

    if (service == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to get service entry\n");
        return;
    }

    switch(result)
    {
        case CONNECTION_P2P_CONNECTED:
            /* To notify the connection result to caller of headset_wlan_connect */
            if (headset_worker_check_pending_event_semaphore(WICED_WLAN_P2P_ASYNC_RESULT) == WICED_TRUE)
            {
                /* Signals success to break a busy loop and keep going to bring up the connection */
                headset_worker_signal_for_event_semaphore(WICED_WLAN_P2P_ASYNC_RESULT, WICED_SUCCESS);
                wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Headset WLAN connected! : %02X\n", result);
            }
            break;
        case CONNECTION_P2P_DISCONNECTED:
            if (app_get_service_state(SERVICE_WLAN) != SERVICE_ENABLED)
            {
                break;
            }
            /* Make a stopping state immediately here.
             * When the P2P GO disconnect session, we P2P GC doesn't have a way to know that disconnection event.
             * So we're catching this event from FW's link event; takes several secs.
             * This FW link event is delivered 2~3 times in a short time so make the state to stopping immediately to block duplicate event */
            app_set_service_state(SERVICE_WLAN, SERVICE_STOPPING);
            /* To disconnect wlan connection */
            headset_worker_push_work(WICED_WLAN_DISCONNECT, headset_wlan_disconnect_service, (void *)service, WICED_FALSE);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "Headset WLAN disconnected! : %02X\n", result);

            break;
        case CONNECTION_P2P_FAILED:
            if (headset_worker_check_pending_event_semaphore(WICED_WLAN_P2P_ASYNC_RESULT) == WICED_TRUE)
            {
                /* Signals failure to break a busy loop */
                headset_worker_signal_for_event_semaphore(WICED_WLAN_P2P_ASYNC_RESULT, WICED_ERROR);
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Headset WLAN failed to connect: %02X\n", result);
            }
            /* TODO: Need a retry concept? */
            break;
        default:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unknown P2P result ! : %02X\n", result);
            break;
    }
}

/*************************************************************
 * BUTTON events handler
 *
 * @return WICED_SUCCESS for success
 */
static wiced_result_t headset_wlan_button_handler(app_service_action_t action)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_app_service_t* service = NULL;
    headset_upnpavrender_context_t* upnpavrender_context = NULL;
    headset_wlan_context_t* headset_wlan_context = NULL;
    upnpavrender_transport_state_t state = UPNPAVRENDER_TRANSPORT_STOPPED;

    service = wiced_get_entry(SERVICE_WLAN);

    if (service == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to get service entry\n");
        return result;
    }

    /* Get WLAN context */
    headset_wlan_context = (headset_wlan_context_t *) service->arg1;

    /* Get UPnP context */
    upnpavrender_context = headset_wlan_context->upnpavrender_context;

    /* TODO: map buttons to UPnP functions */
    switch( action )
    {
        case ACTION_PAUSE_PLAY:
        {
            upnpavrender_service_get_transport_state( upnpavrender_context->service_handle, &state );

            if ( state == UPNPAVRENDER_TRANSPORT_PLAYING )
            {
                upnpavrender_service_pause( upnpavrender_context->service_handle );
            }
            else if ( state == UPNPAVRENDER_TRANSPORT_PAUSED_PLAYBACK )
            {
                upnpavrender_service_resume( upnpavrender_context->service_handle );
            }
        }
            break;
        case ACTION_FORWARD:
            /* map to stop playing */
            upnpavrender_service_stop_playing( upnpavrender_context->service_handle );
            break;
        case ACTION_BACKWARD:
            break;
        case ACTION_VOLUME_UP:
        {
            int volume;
            upnpavrender_service_get_volume( upnpavrender_context->service_handle, &volume );
            volume += HEADSET_AUDIO_VOLUME_INTERVAL;
            upnpavrender_service_set_volume( upnpavrender_context->service_handle, volume );
        }
            break;
        case ACTION_VOLUME_DOWN:
        {
            int volume;
            upnpavrender_service_get_volume( upnpavrender_context->service_handle, &volume );
            volume -= HEADSET_AUDIO_VOLUME_INTERVAL;
            upnpavrender_service_set_volume( upnpavrender_context->service_handle, volume );
        }
            break;
        case ACTION_WLAN_INIT:
            /* Init WLAN (Turn on the WLAN) */
            headset_worker_push_work(WICED_WLAN_INIT, headset_wlan_init_service, (void *)service, WICED_FALSE);
            break;
        case ACTION_WLAN_DEINIT:
            /* Deinit WLAN (Turn off the WLAN) */
            headset_worker_push_work(WICED_WLAN_DEINIT, headset_wlan_deinit_service, (void *)service, WICED_FALSE);
            break;
        case ACTION_WLAN_CONNECT:
            /* To connect WLAN and start UPnP AV renderer if the WLAN link is up
             * Need to process asynchronous connection events */
            headset_worker_push_work(WICED_WLAN_CONNECT, headset_wlan_connect_service, (void *)service, WICED_FALSE);
            break;
        case ACTION_WLAN_DISCONNECT:
            /* To disconnect WLAN and stop UPnP AV renderer */
            app_set_service_state(SERVICE_WLAN, SERVICE_STOPPING);
            headset_worker_push_work(WICED_WLAN_DISCONNECT, headset_wlan_disconnect_service, (void *)service, WICED_FALSE);
            break;
        case ACTION_WLAN_MODE_SWITCH:
            /* Inform that the switching is reserved */
            HEADSET_LED_WLAN_SWITCHING

            /* Stop the playing first */
            upnpavrender_service_get_transport_state( upnpavrender_context->service_handle, &state );
            if ( state == UPNPAVRENDER_TRANSPORT_PLAYING )
            {
                upnpavrender_service_stop_playing( upnpavrender_context->service_handle );
            }

            /* Check if the P2P is waiting for the connection result */
            if (headset_worker_check_pending_event_semaphore(WICED_WLAN_P2P_ASYNC_RESULT) == WICED_TRUE)
            {
                /* Signals failure to break a busy loop */
                headset_worker_signal_for_event_semaphore( WICED_WLAN_P2P_ASYNC_RESULT, WICED_ERROR );
            }

            headset_worker_push_work( WICED_WLAN_CONNECT, headset_wlan_mode_switch_service, (void *)service, WICED_FALSE );
            break;
        default:
            break;
    }

    return result;
}

/*************************************************************
 * Preinit P2P
 *
 */
static void headset_wlan_preinit_p2p(void)
{
    connection_manager_context_t cm_context;
    uint8_t p2p_devname_len;

    /* Get default settings */
    connection_get_settings(&cm_context);

    /* Override P2P Device Name */
    p2p_devname_len = strlen(cm_context.p2p_details.wps_device_details.device_name);
    memset(cm_context.p2p_details.wps_device_details.device_name, 0x0, p2p_devname_len);
    strncpy(cm_context.p2p_details.wps_device_details.device_name, HEADSET_P2P_DEVICE_NAME, p2p_devname_len);

    /* Do override other P2P settings here */

    /* Set default settings */
    connection_set_settings(&cm_context);
}

/*************************************************************
 * Start WLAN function
 *
 * @return WICED_SUCCESS for success
 */
wiced_result_t headset_wlan_application_start(headset_wlan_context_t* wlan_context)
{
    wiced_result_t result = WICED_SUCCESS;
    headset_wlan_context_t* headset_wlan_context = NULL;

    /* Init WLAN context */
    headset_wlan_context = calloc_named("headset_wlan_context", 1, sizeof(headset_wlan_context_t));

    if (headset_wlan_context == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocation headset WLAN context\n");
        return WICED_ERROR;
    }

    memcpy(headset_wlan_context, wlan_context, sizeof(headset_wlan_context_t));

    /* Add WLAN service entry */
    HEADSET_CHECK_RESULT(headset_wlan_add_service(headset_wlan_context));

    /* Preinit P2P */
    if (headset_wlan_context->wlan_interface == WICED_P2P_INTERFACE)
    {
        headset_wlan_preinit_p2p();
    }

    /* To skip IV boundary checking while reordering the AMPDU */
    wwd_wifi_set_iovar_value(IOVAR_STR_APIVTW_OVERRIDE, WICED_TRUE, WWD_STA_INTERFACE);

_exit:
    return result;
}

/*************************************************************
 * Stop WLAN function
 *
 */
wiced_result_t headset_wlan_application_stop(wiced_interface_t wlan_iface)
{
    wiced_result_t result = WICED_SUCCESS;

    HEADSET_CHECK_RESULT(headset_wlan_deinit_service(NULL));

_exit:
    return result;
}

