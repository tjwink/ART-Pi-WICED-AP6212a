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
 * Wi-Fi Connection Manager
 *
 * This library includes some APIs for
 *  - Wi-Fi Direct Group Owner: Persistent group
 *  - Wi-Fi Direct Client mode
 *  - Wi-Fi WPS Registrar mode
 *  - Wi-Fi WPS Enrollee mode
 *
 * TROUBLESHOOTING
 *
 */

#include "wiced.h"
#include "platform_dct.h"
#include "wps_host_interface.h"
#include "internal/wiced_internal_api.h"

#include "connection_manager.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define MAC_ADDRESS_LOCALLY_ADMINISTERED_BIT  0x02
#define MAX_SSID_LEN                          32
#define WIFI_DCT_P2P_PGROUP_OFFSET            CONFIG_AP_LIST_SIZE - 1
#define WLC_EVENT_MSG_LINK                    0x01/* link is up */

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
 *                    Function Declarations
 ******************************************************/
extern uint8_t* p2p_write_invitation_request( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer );

static void p2p_connection_request_callback                        ( p2p_discovered_device_t* device );
static void p2p_legacy_device_connection_request_callback          ( p2p_legacy_device_t* device );
static void p2p_wpa2_client_association_callback                   ( besl_mac_t* );
static void p2p_wps_enrollee_association_callback                  ( besl_mac_t* );
static void p2p_group_formation_result_callback                    ( void* not_used );
static void p2p_wps_result_callback                                ( wps_result_t* wps_result );
static void p2p_device_disassociation_callback                     ( besl_mac_t* );
static void p2p_legacy_device_disassociation_callback              ( besl_mac_t* );
static void* connection_link_events_handler                        (const wwd_event_header_t* event_header, const uint8_t* event_data, void* handler_user_data);

static wiced_result_t connection_p2p_discovery_enable                         (wiced_bool_t listen_discovery);
static wiced_result_t connection_p2p_discovery_disable                        ( void );
static wiced_result_t connection_create_p2p_worker_thread                     ( void );
static wiced_result_t connection_p2p_connection_request_handler               ( void* connecting_device );
static wiced_result_t connection_p2p_legacy_device_connection_request_handler ( void* device_mac );
static wiced_result_t connection_p2p_wpa2_client_association_handler          ( void* mac );
static wiced_result_t connection_p2p_wps_enrollee_association_handler         ( void* mac );
static wiced_result_t connection_p2p_wps_result_handler                       ( void* result );
static wiced_result_t connection_p2p_group_formation_result_handler           ( void* not_used );
static wiced_result_t connection_p2p_device_disassociation_handler            ( void* mac );
static wiced_result_t connection_p2p_legacy_device_disassociation_handler     ( void* mac );
static wiced_result_t connection_check_mac_locally_administered_bit           ( void );

static wiced_result_t connection_p2p_go_start                                 ( void );
static wiced_result_t connection_p2p_go_stop                                  ( void );
static wiced_result_t connection_p2p_gc_start                                 ( wiced_bool_t reinvoke_group );
static wiced_result_t connection_p2p_gc_stop                                  ( void );
static wiced_result_t connection_wps_registrar_start                          ( void );
static wiced_result_t connection_wps_registrar_stop                           ( void );
static wiced_result_t connection_wps_enrollee_start                           ( wiced_bool_t check_stored_ap );
static wiced_result_t connection_wps_enrollee_stop                            ( void );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static p2p_workspace_t          p2p_workspace;
static besl_p2p_device_detail_t p2p_details =
{
    .wps_device_details =
    {
        .device_name     = "WICED_WIFI_CM_P2P",
        .manufacturer    = "Cypress",
        .model_name      = PLATFORM,
        .model_number    = "Wiced",
        .serial_number   = "12345670",
        .device_category = WICED_WPS_DEVICE_COMPUTER,
        .sub_category    = 7,
        .config_methods  = WPS_CONFIG_PUSH_BUTTON | WPS_CONFIG_VIRTUAL_PUSH_BUTTON | WPS_CONFIG_DISPLAY | WPS_CONFIG_VIRTUAL_DISPLAY_PIN | WPS_CONFIG_KEYPAD,
        .authentication_type_flags = WPS_OPEN_AUTHENTICATION | WPS_WPA_PSK_AUTHENTICATION | WPS_WPA2_PSK_AUTHENTICATION,
        .encryption_type_flags     = WPS_NO_ENCRYPTION | WPS_MIXED_ENCRYPTION,
    },
    .listen_channel =
    {
        .country_string  = "XX\x04",
        .operating_class = 81,
        .channel         = 1,
    },
    .operating_channel =
    {
        .country_string  = "XX\x04",
        .operating_class = 81,
        .channel         = 6,
    },
    .channel_list =
    {
        .country_string  = "XX\x04",
        .p2p_channel_list_table =
        {
            { 81, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, },
            { 115, 36, 40, 44, 48, },
            { 124, 149, 153, 157, 161, },
        },
    },
    .group_owner_intent = 5,
    .go_configuration_timeout = 100,    // 1000 milliseconds (Units of 10 milliseconds)
    .client_configuration_timeout = 50, // 500 milliseconds
    .device_password_id = WPS_DEFAULT_DEVICEPWDID,
    .peer_device_timeout = 60000,       // For timing devices out of the p2p peer list (in milliseconds)
    .group_formation_timeout = 30000,      // For timing out a group formation attempt (in milliseconds)
    .p2p_capability = 0x0820, // Intra BSS Distribution, Invitation Procedure
};

static wiced_wps_device_detail_t registrar_details =
{
    .device_name                      = PLATFORM,
    .manufacturer                     = "Cypress",
    .model_name                       = PLATFORM,
    .model_number                     = "1.0",
    .serial_number                    = "1408248",
    .device_category                  = WICED_WPS_DEVICE_NETWORK_INFRASTRUCTURE,
    .sub_category                     = 1,
    .config_methods                   = WPS_CONFIG_PUSH_BUTTON | WPS_CONFIG_VIRTUAL_PUSH_BUTTON,
    .authentication_type_flags        = WPS_OPEN_AUTHENTICATION | WPS_WPA_PSK_AUTHENTICATION | WPS_WPA2_PSK_AUTHENTICATION,
    .encryption_type_flags            = WPS_NO_ENCRYPTION | WPS_MIXED_ENCRYPTION,
    .add_config_methods_to_probe_resp = 1,
};

static wiced_ip_setting_t softap_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255, 255, 255, 0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
};

static wiced_wps_device_detail_t enrollee_details =
{
    .device_name               = PLATFORM,
    .manufacturer              = "Cypress",
    .model_name                = PLATFORM,
    .model_number              = "1.0",
    .serial_number             = "1408248",
    .device_category           = WICED_WPS_DEVICE_COMPUTER,
    .sub_category              = 7,
    .config_methods            = WPS_CONFIG_LABEL | WPS_CONFIG_VIRTUAL_PUSH_BUTTON,
    .authentication_type_flags = WPS_OPEN_AUTHENTICATION | WPS_WPA_PSK_AUTHENTICATION | WPS_WPA2_PSK_AUTHENTICATION,
    .encryption_type_flags     = WPS_NO_ENCRYPTION | WPS_MIXED_ENCRYPTION,
};

static wiced_worker_thread_t p2p_worker_thread;
static uint8_t               p2p_worker_thread_running = 0;
static char                  default_pin_string[9] = "12345670";
static char                  default_pbc_string[9] = "00000000";

connection_status_t connection_status                           = CONNECTION_IDLE;
void (*connection_p2p_result_callback)(connection_p2p_result_t) = NULL;
static const wwd_event_num_t link_events[]                      = {WLC_E_LINK, WLC_E_NONE};
/* Let's move this into DCT area later; if needed */
static besl_mac_t persistent_go_address;

/******************************************************
 *               Function Definitions
 ******************************************************/

/*!
 ******************************************************************************
 * Return current settings
 *
 */
void connection_get_settings( connection_manager_context_t* cm_context )
{
    if (cm_context != NULL)
    {
        memcpy(&cm_context->p2p_details, &p2p_details, sizeof(besl_p2p_device_detail_t));
        memcpy(&cm_context->registrar_details, &registrar_details, sizeof(wiced_wps_device_detail_t));
        memcpy(&cm_context->enrollee_details, &enrollee_details, sizeof(wiced_wps_device_detail_t));
        memcpy(&cm_context->softap_ip_settings, &softap_ip_settings, sizeof(wiced_ip_setting_t));
    }
}

/*!
 ******************************************************************************
 * Override current settings
 *
 */
void connection_set_settings( connection_manager_context_t* cm_context )
{
    if (cm_context != NULL)
    {
        memcpy(&p2p_details, &cm_context->p2p_details, sizeof(besl_p2p_device_detail_t));
        memcpy(&registrar_details, &cm_context->registrar_details, sizeof(wiced_wps_device_detail_t));
        memcpy(&enrollee_details, &cm_context->enrollee_details, sizeof(wiced_wps_device_detail_t));
        memcpy(&softap_ip_settings, &cm_context->softap_ip_settings, sizeof(wiced_ip_setting_t));
    }
}

/*!
 ******************************************************************************
 * Return a current connection status
 *
 * @return  connection status
 */
connection_status_t connection_get_status( void )
{
    return connection_status;
}

/*!
 ******************************************************************************
 * Register a callback for P2P connection results
 */
void connection_register_p2p_result_callback( void (*p2p_result_callback)(connection_p2p_result_t) )
{
    connection_p2p_result_callback = p2p_result_callback;
    wwd_management_set_event_handler(link_events, connection_link_events_handler, NULL, WWD_STA_INTERFACE);
}

/*!
 ******************************************************************************
 * Launch a connection
 *
 * @return  WICED_SUCCESS for success, otherwise error
 */
wiced_result_t connection_launch( connection_status_t connections )
{
    wiced_result_t              result = WICED_SUCCESS;

    if (connections & CONNECTION_P2P_GO)
    {
        result = connection_p2p_go_start();
        WPRINT_LIB_INFO(( " Launched P2P GO: %08X \n", result));
    }
    if (connections & CONNECTION_P2P_GC)
    {
        /* Override go intent as 0 */
        p2p_details.group_owner_intent = 0;

        result = connection_p2p_gc_start(WICED_FALSE);
        WPRINT_LIB_INFO(( " Launched P2P GC: %08X \n", result));
    }
    if (connections & CONNECTION_P2P_GO_NEGOTIATION)
    {
        result = connection_p2p_gc_start(WICED_FALSE);
        WPRINT_LIB_INFO(( " Launched P2P GO Negotiation: %08X \n", result));
    }
    if (connections & CONNECTION_P2P_GC_REINVOKE)
    {
        result = connection_p2p_gc_start(WICED_TRUE);
        WPRINT_LIB_INFO(( " Launched P2P GC with connecting to stored persistent GO: %08X \n", result));
    }
    if (connections & CONNECTION_WPS_REGISTRAR)
    {
        result = connection_wps_registrar_start();
        WPRINT_LIB_INFO(( " Launched WPS Registar: %08X \n", result));
    }
    if (connections & CONNECTION_WPS_ENROLLEE)
    {
        result = connection_wps_enrollee_start(WICED_FALSE);
        WPRINT_LIB_INFO(( " Launched WPS Enrollee: %08X \n", result));
    }
    if (connections & CONNECTION_WPS_ENROLLEE_REINVOKE)
    {
        result = connection_wps_enrollee_start(WICED_TRUE);
        WPRINT_LIB_INFO(( " Launched WPS Enrollee with connecting to stored AP: %08X \n", result));
    }

    return result;
}

/*!
 ******************************************************************************
 * Kill an existing connection
 *
 * @return  WICED_SUCCESS for success, otherwise error
 */
wiced_result_t connection_kill( connection_status_t connections )
{
    wiced_result_t              result = WICED_SUCCESS;

    if (connections & CONNECTION_P2P_GO)
    {
        result = connection_p2p_go_stop();
        WPRINT_LIB_INFO(( " Killed P2P GO: %08X \n", result));
    }
    if (connections & CONNECTION_P2P_GC ||
        connections & CONNECTION_P2P_GC_REINVOKE ||
        connections & CONNECTION_P2P_GO_NEGOTIATION)
    {
        result = connection_p2p_gc_stop();
        WPRINT_LIB_INFO(( " Killed P2P GC: %08X \n", result));
    }
    if (connections & CONNECTION_WPS_REGISTRAR)
    {
        result = connection_wps_registrar_stop();
        WPRINT_LIB_INFO(( " Killed WPS Registar: %08X \n", result));
    }
    if (connections & CONNECTION_WPS_ENROLLEE ||
        connections & CONNECTION_WPS_ENROLLEE_REINVOKE)
    {
        result = connection_wps_enrollee_stop();
        WPRINT_LIB_INFO(( " Killed WPS Enrollee: %08X \n", result));
    }

    host_rtos_delay_milliseconds(100);

    return result;
}

/*!
 ******************************************************************************
 * Kill all existing connections
 *
 * @return  WICED_SUCCESS for success, otherwise error
 */
wiced_result_t connection_killall( void )
{
    connection_status_t connections = CONNECTION_P2P_GO|CONNECTION_P2P_GC|CONNECTION_WPS_REGISTRAR|CONNECTION_WPS_ENROLLEE;
    return connection_kill(connections);
}

/*!
 ******************************************************************************
 * Start autonomous P2P Group Owner
 *
 * @return  WICED_SUCCESS for success, otherwise error
 */
wiced_result_t connection_p2p_go_start( void )
{
    platform_dct_p2p_config_t*  dct_p2p_group_owner_config;
    besl_result_t               besl_result = BESL_SUCCESS;
    wiced_result_t              result = WICED_SUCCESS;

    if ( connection_check_mac_locally_administered_bit( ) != WICED_SUCCESS )
    {
        return WICED_BADARG;
    }

    if ( (wwd_wifi_is_ready_to_transceive( WWD_P2P_INTERFACE ) == WWD_SUCCESS) ||
         (connection_status & CONNECTION_P2P_GO) )
    {
        WPRINT_LIB_INFO(( " P2P interface is already in use "));
        return WICED_ERROR;
    }
    else
    {
        connection_p2p_discovery_disable();
    }

    memset(&p2p_workspace, 0, sizeof(p2p_workspace_t));
    p2p_workspace.i_am_group_owner = 1;

    besl_p2p_init_common( &p2p_workspace, &p2p_details );
    besl_p2p_register_p2p_device_connection_callback( &p2p_workspace, p2p_connection_request_callback);
    besl_p2p_register_legacy_device_connection_callback( &p2p_workspace, p2p_legacy_device_connection_request_callback );
    besl_p2p_register_wpa2_client_association_callback( &p2p_workspace, p2p_wpa2_client_association_callback );
    besl_p2p_register_wps_enrollee_association_callback( &p2p_workspace, p2p_wps_enrollee_association_callback );
    besl_p2p_register_p2p_device_disassociation_callback( &p2p_workspace, p2p_device_disassociation_callback );
    besl_p2p_register_legacy_device_disassociation_callback( &p2p_workspace, p2p_legacy_device_disassociation_callback );

    p2p_workspace.form_persistent_group = 1;
    p2p_workspace.initiate_negotiation = 1;

    p2p_workspace.wps_device_details = &p2p_details.wps_device_details;

    p2p_workspace.reinvoking_group = 1;
    wiced_dct_read_lock( (void**) &dct_p2p_group_owner_config, WICED_TRUE, DCT_P2P_CONFIG_SECTION, 0, sizeof(platform_dct_p2p_config_t) );
    memcpy( &p2p_workspace.persistent_group, &dct_p2p_group_owner_config->p2p_group_owner_settings, sizeof( wiced_config_soft_ap_t ) );
    wiced_dct_read_unlock( (void*) dct_p2p_group_owner_config, WICED_TRUE );
    p2p_workspace.group_candidate.ssid_length = p2p_workspace.persistent_group.SSID.length;
    memcpy( &p2p_workspace.group_candidate.ssid, p2p_workspace.persistent_group.SSID.value, p2p_workspace.group_candidate.ssid_length );
    p2p_workspace.p2p_passphrase_length = p2p_workspace.persistent_group.security_key_length;
    memcpy( p2p_workspace.p2p_passphrase, p2p_workspace.persistent_group.security_key, p2p_workspace.p2p_passphrase_length );
    p2p_workspace.operating_channel.channel = p2p_workspace.persistent_group.channel;
    p2p_workspace.group_candidate.operating_channel.channel = p2p_workspace.operating_channel.channel;

    p2p_workspace.device_name_length = strlen(p2p_workspace.wps_device_details->device_name);
    memcpy((char*)&p2p_workspace.device_name, (char*)p2p_details.wps_device_details.device_name, p2p_workspace.device_name_length);

    if (connection_create_p2p_worker_thread() != WICED_SUCCESS)
    {
        besl_p2p_deinit( &p2p_workspace );
        WPRINT_LIB_INFO( ("P2P GO Start was NOT successful\r\n") );
        return WICED_ERROR;
    }

    besl_p2p_register_wps_result_callback( &p2p_workspace, p2p_wps_result_callback );
    besl_p2p_register_group_formation_result_callback( &p2p_workspace, p2p_group_formation_result_callback );

    if ( (besl_result = besl_p2p_group_owner_start(&p2p_workspace)) != BESL_SUCCESS )
    {
        WPRINT_APP_INFO( ("Error starting group owner: %08X\n", (unsigned int )besl_result ) );

        besl_p2p_deinit( &p2p_workspace );

        if ( p2p_worker_thread_running == 1 )
        {
            wiced_rtos_delete_worker_thread( &p2p_worker_thread );
            p2p_worker_thread_running = 0;
        }

        WPRINT_LIB_INFO( ("P2P GO Start was NOT successful\r\n") );
        return WICED_ERROR;
    }

    WPRINT_LIB_INFO( ("P2P GO Start was successful\r\n") );
    connection_status |= CONNECTION_P2P_GO;

    return result;
}

/*!
 ******************************************************************************
 * Stop P2P group owner
 *
 * @return  WICED_SUCCESS for success, otherwise error
 */

wiced_result_t connection_p2p_go_stop( void )
{
    wiced_result_t result = WICED_SUCCESS;

    if ( !(connection_status & CONNECTION_P2P_GO) )
    {
        WPRINT_LIB_INFO(( "P2P group owner not up\n"));
        return WICED_SUCCESS;
    }

    if ( p2p_workspace.p2p_wps_agent == NULL )
    {
        WPRINT_LIB_INFO(("P2P registrar not yet initialized\n"));
    }
    else if ( p2p_workspace.p2p_wps_agent->wps_result != WPS_NOT_STARTED )
    {
        if ( besl_wps_abort( p2p_workspace.p2p_wps_agent ) == BESL_SUCCESS )
        {
            WPRINT_LIB_INFO(("P2P registrar stopped\n"));
        }
    }
    else
    {
        WPRINT_LIB_INFO(("P2P registrar not running\n"));
    }

    wwd_wifi_deauth_all_associated_client_stas( WWD_DOT11_RC_UNSPECIFIED, WWD_P2P_INTERFACE );
    host_rtos_delay_milliseconds( 100 ); /* Delay to allow the deauthentication frames to be sent */

    if ( p2p_worker_thread_running == 1 )
    {
        wiced_rtos_delete_worker_thread( &p2p_worker_thread );
        p2p_worker_thread_running = 0;
    }
    if ( besl_p2p_deinit( &p2p_workspace ) != BESL_SUCCESS )
    {
        WPRINT_LIB_INFO(("Error when stopping P2P group owner\n"));
        result = WICED_ERROR;
    }

    if (result == WICED_SUCCESS)
    {
        WPRINT_LIB_INFO( ("P2P GO Stop was successful\r\n") );
        connection_status &= ~CONNECTION_P2P_GO;
    }
    else
    {
        WPRINT_LIB_INFO( ("P2P GO Stop was NOT successful\r\n") );
    }

    return result;
}

/*!
 ******************************************************************************
 * Start P2P client
 *
 * @return  WICED_SUCCESS for success, otherwise error
 */
wiced_result_t connection_p2p_gc_start( wiced_bool_t reinvoke_group )
{
    wiced_result_t result = WICED_SUCCESS;

    if ( connection_check_mac_locally_administered_bit( ) != WICED_SUCCESS )
    {
        return WICED_BADARG;
    }

    if (connection_status & CONNECTION_P2P_GC)
    {
        WPRINT_LIB_INFO(( " P2P interface is already in use "));
        return WICED_ERROR;
    }

    /* connection_p2p_connection_request_handler will take a role for accepting invitation, group formation requests */
    if (connection_p2p_discovery_enable(WICED_FALSE) == WICED_SUCCESS)
    {
        WPRINT_LIB_INFO( ("P2P GC Start was successful\r\n") );
        connection_status |= CONNECTION_P2P_GC;
    }
    else
    {
        WPRINT_LIB_INFO( ("P2P GC Start was NOT successful\r\n") );
        result = WICED_ERROR;
    }

    /* Trying to find a persistent group in WIFI DCT[4] and reinvoking this group if exist */
    if (result == WICED_SUCCESS && reinvoke_group == WICED_TRUE && (!(NULL_MAC(&persistent_go_address))))
    {
        platform_dct_wifi_config_t* dct_wifi_config;
        p2p_discovered_device_t* devices;
        uint8_t device_count;

        /* Delay to allow scan-results to be stored */
        wiced_rtos_delay_milliseconds( 1000 );

        besl_p2p_get_discovered_peers(&p2p_workspace, &devices, &device_count);

        for (device_count = 0; device_count < P2P_MAX_DISCOVERED_DEVICES; device_count++)
        {
            if (memcmp(&persistent_go_address, &devices[device_count].p2p_device_address, sizeof(besl_mac_t)) == 0)
            {
                p2p_workspace.candidate_device = &devices[device_count];
                WPRINT_LIB_INFO(("Found persistent GO in the Air > %02X:%02X:%02X:%02X:%02X:%02X\n",
                                 devices[device_count].p2p_device_address.octet[0],
                                 devices[device_count].p2p_device_address.octet[1],
                                 devices[device_count].p2p_device_address.octet[2],
                                 devices[device_count].p2p_device_address.octet[3],
                                 devices[device_count].p2p_device_address.octet[4],
                                 devices[device_count].p2p_device_address.octet[5]));
                break;
            }
        }

        if (device_count == P2P_MAX_DISCOVERED_DEVICES)
        {
            WPRINT_LIB_INFO( ("Failed to find persistent GO in the Air. Check if the GO device is discoverable now\r\n") );
            return WICED_SUCCESS;
        }

        WPRINT_LIB_INFO( ("Sending invitation request !\r\n") );

        wiced_dct_read_lock( (void**) &dct_wifi_config, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) );
        p2p_workspace.group_candidate.ssid_length = dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].details.SSID.length;
        memcpy( p2p_workspace.group_candidate.ssid, dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].details.SSID.value, p2p_workspace.group_candidate.ssid_length );
        memcpy( &p2p_workspace.group_candidate.bssid, &dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].details.BSSID, sizeof( besl_mac_t ) );
        p2p_workspace.p2p_passphrase_length = dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].security_key_length;
        memcpy( p2p_workspace.p2p_passphrase, dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].security_key, p2p_workspace.p2p_passphrase_length);
        memcpy( &p2p_workspace.group_candidate.operating_channel, &p2p_workspace.candidate_device->operating_channel, sizeof( p2p_channel_info_t ));
        wiced_dct_read_unlock( (void*) dct_wifi_config, WICED_TRUE );

        if (besl_p2p_send_action_frame( &p2p_workspace, p2p_workspace.candidate_device, p2p_write_invitation_request, p2p_workspace.current_channel, 2 ) != BESL_SUCCESS)
        {
            WPRINT_LIB_INFO( ("Failed to Send invitation request. Wait for the invitation\r\n") );
        }
    }
    else
    {
        WPRINT_LIB_INFO( ("No persistent GO information! Wait for the invitation\r\n") );
    }

    return result;
}

/*!
 ******************************************************************************
 * Stop P2P client
 *
 * @return  WICED_SUCCESS for success, otherwise error
 */
wiced_result_t connection_p2p_gc_stop( void )
{
    wiced_result_t result = WICED_SUCCESS;

    if ( !(connection_status & CONNECTION_P2P_GC) )
    {
        WPRINT_LIB_INFO( ("P2P group client not up\r\n") );
        return WICED_SUCCESS;
    }

    /* connection_p2p_connection_request_handler will take a role for accepting invitation, group formation requests */
    if (connection_p2p_discovery_disable() == WICED_SUCCESS)
    {
        WPRINT_LIB_INFO( ("P2P GC Stop was successful\r\n") );
        connection_status &= ~CONNECTION_P2P_GC;
    }
    else
    {
        WPRINT_LIB_INFO( ("P2P GC Stop was NOT successful\r\n") );
        result = WICED_ERROR;
    }

    return result;
}

/*!
 ******************************************************************************
 * Start WPS Registrar
 *
 * @return  WICED_SUCCESS for success, otherwise error
 */
wiced_result_t connection_wps_registrar_start( void )
{
    wiced_result_t           result = WICED_SUCCESS;
    wiced_wps_credential_t   ap_info;
    wiced_config_soft_ap_t*  softap_info;

    if ( (wwd_wifi_is_ready_to_transceive( WWD_AP_INTERFACE ) == WWD_SUCCESS) ||
         (connection_status & CONNECTION_WPS_REGISTRAR) )
    {
        WPRINT_LIB_INFO(( " WICED AP interface is already in use\n"));
        return WICED_ERROR;
    }

    WPRINT_LIB_INFO( ("Starting access point\r\n") );
    wiced_network_up( WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &softap_ip_settings );

    /* Extract the settings from the WICED softAP to pass to the WPS registrar */
    wiced_dct_read_lock( (void**) &softap_info, WICED_FALSE, DCT_WIFI_CONFIG_SECTION, OFFSETOF(platform_dct_wifi_config_t, soft_ap_settings), sizeof(wiced_config_soft_ap_t) );
    memcpy( ap_info.passphrase, softap_info->security_key, sizeof(ap_info.passphrase) );
    memcpy( &ap_info.ssid,      &softap_info->SSID,        sizeof(wiced_ssid_t) );
    ap_info.passphrase_length = softap_info->security_key_length;
    ap_info.security          = softap_info->security;
    wiced_dct_read_unlock( softap_info, WICED_FALSE );

    WPRINT_LIB_INFO( ("Starting WPS Registrar in PBC mode\r\n") );

    if ( wiced_wps_registrar( WICED_WPS_PBC_MODE, &registrar_details, "00000000", &ap_info, 1 ) == WICED_SUCCESS )
    {
        WPRINT_LIB_INFO( ("WPS enrollment was successful\r\n") );
        connection_status |= CONNECTION_WPS_REGISTRAR;
    }
    else
    {
        WPRINT_LIB_INFO( ("WPS enrollment was not successful\r\n") );
        result = WICED_ERROR;
    }

    return result;
}

/*!
 ******************************************************************************
 * Stop WPS Registrar
 *
 * @return  WICED_SUCCESS for success, otherwise error
 */
wiced_result_t connection_wps_registrar_stop( void )
{
    wiced_result_t           result = WICED_SUCCESS;
    /*
     * WAR: Re-Init the WLAN to make this working
     * STEPs: 1. Start AP; 2. Stop AP; 3. Start P2P GO; 4 IOVAR p2p_ifadd -> wlog says the firmware is corrupted.
     * TODO: Need to debug the firmware error
     */
    if ( (result = wiced_wlan_connectivity_deinit()) != WICED_SUCCESS )
    {
        WPRINT_LIB_INFO( ("Failed to deinit wlan connectivity\r\n") );
    }
    if ( (result = wiced_wlan_connectivity_init() != WICED_SUCCESS) )
    {
        WPRINT_LIB_INFO( ("Failed to init wlan connectivity\r\n") );
    }

    if (result == WICED_SUCCESS)
    {
        connection_status &= ~CONNECTION_WPS_REGISTRAR;
    }
    return result;
}

/*!
 ******************************************************************************
 * Start WPS Enrollee
 *
 * @return  WICED_SUCCESS for success, otherwise error
 */
wiced_result_t connection_wps_enrollee_start( wiced_bool_t check_stored_ap )
{
    wiced_wps_credential_t      credential[WIFI_DCT_P2P_PGROUP_OFFSET];
    platform_dct_wifi_config_t* wifi_config;
    wiced_result_t              result = WICED_SUCCESS;
    wiced_band_list_t           band_list;
    int32_t                     band = 0;
    uint32_t                    channel;

    if (check_stored_ap == WICED_TRUE)
    {
        result = wiced_network_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);
        if (result == WICED_SUCCESS)
        {
            WPRINT_LIB_INFO(( " Connected to known AP in WIFI DCT\n"));
            connection_status |= CONNECTION_WPS_ENROLLEE;
            return result;
        }
        else
        {
            wiced_network_down(WICED_STA_INTERFACE);
        }
    }

    if ( (wwd_wifi_is_ready_to_transceive( WWD_STA_INTERFACE ) == WWD_SUCCESS ) ||
         (connection_status & CONNECTION_WPS_ENROLLEE) )
    {
        WPRINT_LIB_INFO(( " WICED STA interface is already in use\n"));
        return WICED_ERROR;
    }

    memset( &credential, 0, sizeof( credential ) );

    /* Run the WPS Enrolle */
    if ( wwd_wifi_get_supported_band_list( &band_list ) == WWD_SUCCESS )
    {
        if ( band_list.number_of_bands == 2 )
        {
            wwd_wifi_set_preferred_association_band( WLC_BAND_2G );
            if ( wwd_wifi_get_preferred_association_band( &band ) == WWD_SUCCESS )
            {
                WPRINT_LIB_INFO( ("Preferred band for association is %sGHz\n", band == WLC_BAND_2G ? "2.4" : "5" ) );
            }
        }
    }

    WPRINT_LIB_INFO( ("Starting WPS Enrollee in PBC mode. Press the WPS button on your AP now.\n") );

    result = wiced_wps_enrollee( WICED_WPS_PBC_MODE, &enrollee_details, "00000000", credential, WIFI_DCT_P2P_PGROUP_OFFSET );
    if (result == WICED_SUCCESS)
    {
        uint8_t i;
        uint8_t dct_index;

        WPRINT_LIB_INFO( ("WPS enrollment was successful\n") );

        /* Copy Wi-Fi credentials obtained from WPS to the Wi-Fi config section in the DCT */
        wiced_dct_read_lock( (void**) &wifi_config, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof( platform_dct_wifi_config_t ) );

        /* To find an empty slot in DCT WIFI */
        for ( i = 0; i < WIFI_DCT_P2P_PGROUP_OFFSET; i++ )
        {
            if (wifi_config->stored_ap_list[i].details.SSID.length == 0)
            {
                dct_index = i;
                break;
            }
        }
        /* No empty slot; do overwrite the slots from head */
        if ( i == WIFI_DCT_P2P_PGROUP_OFFSET )
        {
            dct_index = 0;
        }

        /* Because we don't know a last updated slot after (re)boot, need to prepare a next slot to be overwritten */
        if (dct_index + 1 == WIFI_DCT_P2P_PGROUP_OFFSET)
        {
            memset (&wifi_config->stored_ap_list[0], 0x0, sizeof(wiced_config_ap_entry_t));
        }
        else
        {
            memset (&wifi_config->stored_ap_list[dct_index+1], 0x0, sizeof(wiced_config_ap_entry_t));
        }

        for ( i = 0; i < WIFI_DCT_P2P_PGROUP_OFFSET; i++ )
        {
            if (credential[i].ssid.length != 0)
            {
                memcpy( (void *)&wifi_config->stored_ap_list[dct_index].details.SSID, &credential[i].ssid, sizeof(wiced_ssid_t) );
                memcpy( wifi_config->stored_ap_list[dct_index].security_key, &credential[i].passphrase, credential[i].passphrase_length );

                wifi_config->stored_ap_list[dct_index].details.security    = credential[i].security;
                wifi_config->stored_ap_list[dct_index].security_key_length = credential[i].passphrase_length;

                WPRINT_LIB_INFO( ("Storing AP credential for '%s' into WIFI_DCT[%u]\n",
                                  wifi_config->stored_ap_list[dct_index].details.SSID.value, (unsigned int)dct_index) );

                /* To support multiple credentials if exist */
                dct_index ++;
                if (dct_index == WIFI_DCT_P2P_PGROUP_OFFSET)
                {
                    dct_index = 0;
                }
            }
        }

        wiced_dct_write ( (const void*)wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof (platform_dct_wifi_config_t) );

        wiced_dct_read_unlock( (void*)wifi_config, WICED_TRUE );

        /* AP credentials have been stored in the DCT, now join the AP */
        result = wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

        if (result != WICED_SUCCESS)
        {
            WPRINT_LIB_INFO( ("Network up is failed\r\n") );
            return result;
        }

        /* Enable PM1 */
        if (wwd_wifi_enable_powersave() != WWD_SUCCESS)
        {
            WPRINT_LIB_INFO( ("Failed to set PM1 on STA interface\n") );
        }

        /* Get the channel that the radio is on. For 40MHz capable radios this may display the center frequency of the channel rather than the primary channel of the WLAN */
        if ( wwd_wifi_get_channel( WWD_STA_INTERFACE, &channel ) == WWD_SUCCESS )
        {
            WPRINT_LIB_INFO( ("Associated on channel: %u\n", (unsigned int) channel ) );
        }

        WPRINT_LIB_INFO( ("Joining AP was successful\r\n") );
        connection_status |= CONNECTION_WPS_ENROLLEE;
    }
    else
    {
        WPRINT_LIB_INFO( ("WPS enrollment was not successful\r\n") );
        result = WICED_ERROR;
    }

    return result;
}

/*!
 ******************************************************************************
 * Stop WPS Enrollee
 *
 * @return  WICED_SUCCESS for success, otherwise error
 */
wiced_result_t connection_wps_enrollee_stop( void )
{
    wiced_result_t           result = WICED_SUCCESS;

    if ( ( wwd_wifi_is_ready_to_transceive( WWD_STA_INTERFACE ) != WWD_SUCCESS ))
    {
        WPRINT_LIB_INFO(( "WPS Enrollee(STA) is not up\n"));
        return WICED_SUCCESS;
    }

    if (wiced_network_down( WICED_STA_INTERFACE ) == WICED_SUCCESS)
    {
        WPRINT_LIB_INFO( ("WPS Enrollee(STA) stop was successful\r\n") );
        connection_status &= ~CONNECTION_WPS_ENROLLEE;
    }
    else if (wiced_leave_ap( WICED_STA_INTERFACE ) == WICED_SUCCESS)
    {
        WPRINT_LIB_INFO( ("WPS Enrollee(STA) stop was successful\r\n") );
        connection_status &= ~CONNECTION_WPS_ENROLLEE;
    }
    else
    {
        WPRINT_LIB_INFO( ("WPS Enrollee(STA) stop was NOT successful\r\n") );
        result = WICED_ERROR;
    }
    return result;
}

/*!
 ******************************************************************************
 * Enable P2P in discovery mode
 *
 * @return  WICED_SUCCESS for success, otherwise error
 */

static wiced_result_t connection_p2p_discovery_enable(wiced_bool_t listen_discovery)
{
    besl_result_t result;

    if ( p2p_workspace.p2p_initialised == 1 )
    {
        if ( wwd_wifi_is_ready_to_transceive( WWD_P2P_INTERFACE ) == WWD_SUCCESS )
        {
            if ( besl_p2p_group_owner_is_up() == WICED_TRUE )
            {
                WPRINT_LIB_INFO(( "P2P group owner is already up\n" ));
            }
            else
            {
                WPRINT_LIB_INFO(( "P2P client is already up\n" ));
            }
            return WICED_SUCCESS;
        }
        WPRINT_LIB_INFO(( "Calling p2p discovery disable again\n" ));
        connection_p2p_discovery_disable();
    }

    if ( p2p_worker_thread_running != 1 )
    {
        if ( connection_create_p2p_worker_thread() == WICED_SUCCESS )
        {
            if ( ( result = besl_p2p_init( &p2p_workspace, &p2p_details ) ) != BESL_SUCCESS )
            {
                WPRINT_LIB_INFO(( "besl_p2p_init failed %u\n", (unsigned int)result ));
                if ( p2p_worker_thread_running == 1 )
                {
                    wiced_rtos_delete_worker_thread( &p2p_worker_thread );
                    p2p_worker_thread_running = 0;
                }
                return WICED_ERROR;
            }
            besl_p2p_register_p2p_device_connection_callback( &p2p_workspace, p2p_connection_request_callback);
            besl_p2p_register_legacy_device_connection_callback( &p2p_workspace, p2p_legacy_device_connection_request_callback );
            besl_p2p_register_wpa2_client_association_callback( &p2p_workspace, p2p_wpa2_client_association_callback );
            besl_p2p_register_wps_enrollee_association_callback( &p2p_workspace, p2p_wps_enrollee_association_callback );
            besl_p2p_register_group_formation_result_callback( &p2p_workspace, p2p_group_formation_result_callback );
            besl_p2p_register_wps_result_callback( &p2p_workspace, p2p_wps_result_callback );
            if ( listen_discovery == WICED_TRUE )
                besl_p2p_listen_start( &p2p_workspace );
            else
                besl_p2p_start( &p2p_workspace );
        }
    }

    return WICED_SUCCESS;
}


/*!
 ******************************************************************************
 * Disable P2P discovery mode
 *
 * @return  WICED_SUCCESS for success, otherwise error
 */
static wiced_result_t connection_p2p_discovery_disable( void )
{
    if ( p2p_workspace.p2p_initialised )
    {
        if ( besl_p2p_deinit( &p2p_workspace ) != 0 )
        {
            WPRINT_LIB_INFO(("Error when disabling P2P discovery\n"));
            return WICED_ERROR;
        }
        else
        {
            WPRINT_LIB_INFO(("P2P discovery is now disabled\n"));
        }
        if ( p2p_worker_thread_running == 1 )
        {
            wiced_rtos_delete_worker_thread( &p2p_worker_thread );
            p2p_worker_thread_running = 0;
        }
    }
    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * P2P connection request handler
 * This is an example only. The console app does not easily allow keyboard entry from more than one thread so the handler advises which P2P
 * related command should be entered by the user.
 *
 * @return  WICED_SUCCESS for success, otherwise error
 */
static wiced_result_t connection_p2p_connection_request_handler( void* connecting_device )
{
    p2p_discovered_device_t* device = connecting_device;
    platform_dct_wifi_config_t* dct_wifi_config;
    platform_dct_p2p_config_t* dct_p2p_config;

    if ( device == NULL )
    {
        WPRINT_LIB_INFO( ( "Connection Request from NULL device\n" ) );
        return WICED_SUCCESS;
    }

    WPRINT_LIB_INFO( ( "Connection Request from:" ) );
    WPRINT_LIB_INFO( ( "  %02X:%02X:%02X:%02X:%02X:%02X    ",
                device->p2p_device_address.octet[0],
                device->p2p_device_address.octet[1],
                device->p2p_device_address.octet[2],
                device->p2p_device_address.octet[3],
                device->p2p_device_address.octet[4],
                device->p2p_device_address.octet[5] ) );
    WPRINT_LIB_INFO( ( " %s", device->device_name ) );
    WPRINT_LIB_INFO( ( " status=%02X", device->status ) );
    WPRINT_LIB_INFO( ( "\n") );

    /* Handle Negotiation Requests and Provision Requests */
    if ( device->status == P2P_DEVICE_REQUESTED_TO_FORM_GROUP)
    {
        /* Ignore request if the configuration method is not allowed */
        if ( ! ( p2p_workspace.allowed_configuration_methods & device->preferred_config_method ) )
        {
            WPRINT_LIB_INFO( ("WPS configuration method is not allowed\n") );
            //return WICED_SUCCESS;
        }

        if ( p2p_workspace.p2p_wps_agent != NULL )
        {
            if ( p2p_workspace.p2p_wps_agent->wps_result == WPS_IN_PROGRESS )
            {
                WPRINT_LIB_INFO( ("WPS already running.\n") );
                return WICED_SUCCESS;
            }
        }

        /* The group owner gets the WPS configuration method in a Provision Discovery Request but the STA may get it from a Negotiation Request or Provision Request. */
        if ( p2p_workspace.group_owner_is_up == 1 )
        {
            if ( device->preferred_config_method == PUSH_BUTTON )
            {
                WPRINT_LIB_INFO(("Starting registrar in PBC mode\n"));
                p2p_workspace.p2p_wps_mode = WPS_PBC_MODE;
                p2p_workspace.p2p_wps_device_password_id = WPS_PUSH_BTN_DEVICEPWDID;
                strncpy( p2p_workspace.p2p_wps_pin, default_pbc_string, 8 );
                p2p_workspace.p2p_wps_pin[8] = 0;
                besl_p2p_start_registrar();
            }
            else if ( device->preferred_config_method == KEYPAD )
            {
                WPRINT_LIB_INFO(("Starting registrar in PIN mode\n"));
                p2p_workspace.p2p_wps_mode = WPS_PIN_MODE;
                strncpy( p2p_workspace.p2p_wps_pin, default_pin_string, 8 );
                p2p_workspace.p2p_wps_pin[8] = 0;
                besl_p2p_start_registrar();
            }
            else
            {
                WPRINT_LIB_INFO( ( "Unknown WPS configuration method %u\n", (unsigned int)device->preferred_config_method));
                return WICED_ERROR;
            }
        }
        else if ( device->group_owner_capability & 0x01 )
        {
            WPRINT_LIB_INFO(("Starting to find group owner\n"));

            p2p_workspace.group_candidate.ssid_length = strlen( device->ssid );
            memcpy( p2p_workspace.group_candidate.ssid, device->ssid, MIN( MAX_SSID_LEN, p2p_workspace.group_candidate.ssid_length ) );
            memcpy( &p2p_workspace.group_candidate.p2p_device_address, &device->p2p_device_address, sizeof( wiced_mac_t ) );

            if ( p2p_workspace.p2p_wps_device_password_id == WPS_PUSH_BTN_DEVICEPWDID )
            {
                p2p_workspace.provisioning_config_method = PUSH_BUTTON;
            }
            else if ( p2p_workspace.p2p_wps_device_password_id == WPS_USER_SPEC_DEVICEPWDID )
            {
                p2p_workspace.provisioning_config_method = DISPLAY;
            }
            else
            {
                p2p_workspace.provisioning_config_method = KEYPAD;
            }
            besl_p2p_find_group_owner( &p2p_workspace );
        }
        else
        {
            WPRINT_LIB_INFO(("Starting group formation in PBC mode\n"));

            memcpy( &p2p_workspace.group_candidate.p2p_device_address, &device->p2p_device_address, sizeof( wiced_mac_t ) );
            memcpy( p2p_workspace.p2p_wps_pin, default_pbc_string, 9 );
            p2p_workspace.p2p_wps_mode = WPS_PBC_MODE;
            p2p_workspace.p2p_wps_device_password_id = WPS_PUSH_BTN_DEVICEPWDID;
            p2p_workspace.provisioning_config_method = PUSH_BUTTON;

            p2p_workspace.initiate_negotiation = 0;
            p2p_workspace.ok_to_accept_negotiation = 1;

            WPRINT_LIB_INFO(("P2P group owner intent = %d\n", p2p_workspace.group_owner_intent));

            if ( besl_p2p_start_negotiation( &p2p_workspace ) != 0 )
            {
                WPRINT_LIB_INFO(("besl_p2p_start_negotiation failed\n"));
            }
        }
    }
    else if ( ( p2p_workspace.p2p_current_state == P2P_STATE_DISCOVERY ) && ( device->status == P2P_DEVICE_INVITATION_REQUEST ) )
    {
        if ( device->invitation_flags == 0 )
        {
            WPRINT_LIB_INFO(("Invitation flag is 0\n"));
            p2p_workspace.group_candidate.ssid_length = p2p_workspace.invitation_candidate.ssid_length;
            memcpy( p2p_workspace.group_candidate.ssid, p2p_workspace.invitation_candidate.ssid, p2p_workspace.group_candidate.ssid_length );
            memcpy( &p2p_workspace.group_candidate.p2p_device_address, &device->p2p_device_address, sizeof( besl_mac_t ) );
            besl_p2p_find_group_owner( &p2p_workspace );
        }
        else
        {
            if ( memcmp( &device->group_owner_device_address, &p2p_workspace.p2p_device_address, sizeof( besl_mac_t ) ) == 0 )
            {
                /* Reinvoke our group */
                WPRINT_LIB_INFO(("Reinvoke our group  \n"));
                besl_p2p_register_p2p_device_disassociation_callback( &p2p_workspace, p2p_device_disassociation_callback );
                besl_p2p_register_legacy_device_disassociation_callback( &p2p_workspace, p2p_legacy_device_disassociation_callback );
                p2p_workspace.reinvoking_group = 1;

                wiced_dct_read_lock( (void**) &dct_p2p_config, WICED_TRUE, DCT_P2P_CONFIG_SECTION, 0, sizeof(platform_dct_p2p_config_t) );
                p2p_workspace.i_am_group_owner = 1;
                p2p_workspace.form_persistent_group = 1;
                memcpy( &p2p_workspace.persistent_group, &dct_p2p_config->p2p_group_owner_settings, sizeof( wiced_config_soft_ap_t ) );
                p2p_workspace.group_candidate.ssid_length = p2p_workspace.persistent_group.SSID.length;
                memcpy( p2p_workspace.group_candidate.ssid, p2p_workspace.persistent_group.SSID.value, p2p_workspace.group_candidate.ssid_length );
                p2p_workspace.p2p_passphrase_length = p2p_workspace.persistent_group.security_key_length;
                memcpy( p2p_workspace.p2p_passphrase, p2p_workspace.persistent_group.security_key, p2p_workspace.p2p_passphrase_length );
                p2p_workspace.operating_channel.channel = p2p_workspace.persistent_group.channel;
                p2p_workspace.group_candidate.operating_channel.channel = p2p_workspace.operating_channel.channel;
                wiced_dct_read_unlock( (void*) dct_p2p_config, WICED_TRUE );
            }
            else
            {
                /* Reinvoke their group XXX should check against workspace invitation candidate */
                wiced_dct_read_lock( (void**) &dct_wifi_config, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) );

                if (memcmp(dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].details.SSID.value, p2p_workspace.invitation_candidate.ssid, SSID_NAME_SIZE) != 0)
                {
                    WPRINT_LIB_INFO(("Invitation request from NOT stored persistent group\n"));
                    /* TODO: Need to do something ??? */
                    return WICED_ERROR;
                }
                else
                {
                    WPRINT_LIB_INFO(("Invitation request from Stored persistent group\n"));
                    p2p_workspace.reinvoking_group = 1;

                    p2p_workspace.group_candidate.ssid_length = dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].details.SSID.length;
                    memcpy( p2p_workspace.group_candidate.ssid, dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].details.SSID.value, p2p_workspace.group_candidate.ssid_length );
                    memcpy( &p2p_workspace.group_candidate.bssid, (char *)&dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].details.BSSID, sizeof( besl_mac_t ) );
                    p2p_workspace.p2p_passphrase_length = dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].security_key_length;
                    memcpy( p2p_workspace.p2p_passphrase, dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].security_key, p2p_workspace.p2p_passphrase_length);
                    memcpy( &p2p_workspace.group_candidate.operating_channel, &p2p_workspace.invitation_candidate.operating_channel, sizeof( p2p_channel_info_t ) );
                    wiced_dct_read_unlock( (void*) dct_wifi_config, WICED_TRUE );
                }
            }
            besl_p2p_host_negotiation_complete(&p2p_workspace);
        }
    }

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * Legacy WPS device connection request handler
 * This is an example only. The console app does not easily allow keyboard entry from more than one thread so the handler advises which P2P
 * related command should be entered by the user.
 *
 * @return  0 for success
 */

static wiced_result_t connection_p2p_legacy_device_connection_request_handler( void* device )
{
    p2p_legacy_device_t* legacy_device = device;

    WPRINT_LIB_INFO( ( "\nConnection Request from WPS legacy device using push button mode:" ) );
    WPRINT_LIB_INFO( ( "  %02X:%02X:%02X:%02X:%02X:%02X    ",
        legacy_device->mac_address.octet[0],
        legacy_device->mac_address.octet[1],
        legacy_device->mac_address.octet[2],
        legacy_device->mac_address.octet[3],
        legacy_device->mac_address.octet[4],
        legacy_device->mac_address.octet[5] ) );
    WPRINT_LIB_INFO( ( "%s\n", legacy_device->device_name ) );

    if ( p2p_workspace.p2p_wps_agent != NULL )
    {
        if ( p2p_workspace.p2p_wps_agent->wps_result == WPS_IN_PROGRESS )
        {
            WPRINT_LIB_INFO( ("WPS running.\n") );
            return WICED_SUCCESS;
        }
        else
        {
            WPRINT_LIB_INFO(("Starting registrar in PBC mode\n"));
            p2p_workspace.p2p_wps_mode = WPS_PBC_MODE;
            p2p_workspace.p2p_wps_device_password_id = WPS_PUSH_BTN_DEVICEPWDID;
            strncpy( p2p_workspace.p2p_wps_pin, default_pbc_string, 8 );
            p2p_workspace.p2p_wps_pin[8] = 0;
            besl_p2p_start_registrar();
        }
    }

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * WPA2 client association handler
 *
 * This is only an indication of association. The EAPOL key handshake might still fail.
 *
 * @return  0 for success
 */

static wiced_result_t connection_p2p_wpa2_client_association_handler( void* mac )
{
    wiced_mac_t mac_address;

    if (connection_p2p_result_callback != NULL)
    {
        connection_p2p_result_callback(CONNECTION_P2P_CONNECTED);
    }

    if ( mac != NULL )
    {
        memcpy(&mac_address, mac, sizeof(wiced_mac_t));

        WPRINT_LIB_INFO( ( "\nWPA2 client associated:" ) );
        WPRINT_LIB_INFO( ( "  %02X:%02X:%02X:%02X:%02X:%02X\n",
            mac_address.octet[0],
            mac_address.octet[1],
            mac_address.octet[2],
            mac_address.octet[3],
            mac_address.octet[4],
            mac_address.octet[5] ) );
    }
    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * WPS enrollee association handler
 *
 * This is assumed to be a WPS enrollee because there was no RSN IE in the association request.
 *
 * @return  0 for success
 */

static wiced_result_t connection_p2p_wps_enrollee_association_handler( void* mac )
{
    wiced_mac_t mac_address;

    if ( mac != NULL )
    {
        memcpy(&mac_address, mac, sizeof(wiced_mac_t));

        WPRINT_LIB_INFO( ( "\nWPS enrollee associated:" ) );
        WPRINT_LIB_INFO( ( "  %02X:%02X:%02X:%02X:%02X:%02X\n",
            mac_address.octet[0],
            mac_address.octet[1],
            mac_address.octet[2],
            mac_address.octet[3],
            mac_address.octet[4],
            mac_address.octet[5] ) );
    }
    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * WPA2 client association handler
 *
 * This is only an indication of association. The EAPOL key handshake might still fail.
 *
 * @return  0 for success
 */

static wiced_result_t connection_p2p_wps_result_handler( void* result )
{
    wps_result_t* wps_result = (wps_result_t*)result;

    /* Print result (if enabled) */
    if ( *wps_result == WPS_COMPLETE )
    {
        WPRINT_LIB_INFO(( "connection_p2p_wps_result_handler: WPS completed successfully\n" ));
    }
    else if ( *wps_result == WPS_PBC_OVERLAP )
    {
        WPRINT_LIB_INFO(( "connection_p2p_wps_result_handler: PBC overlap detected - wait and try again\n" ));
    }
    else if ( *wps_result == WPS_ABORTED )
    {
        WPRINT_LIB_INFO(( "connection_p2p_wps_result_handler: WPS aborted\n" ));
    }
    else
    {
        WPRINT_LIB_INFO(( "connection_p2p_wps_result_handler: WPS timed out\n" ));
    }

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * P2P group formation result handler
 *
 */

static wiced_result_t connection_p2p_group_formation_result_handler( void* not_used )
{
    p2p_workspace_t* workspace = &p2p_workspace;
    platform_dct_wifi_config_t* dct_wifi_config;
    platform_dct_p2p_config_t* dct_p2p_config;
    uint32_t channel;

    if ( wiced_network_is_ip_up( WICED_P2P_INTERFACE ) == WICED_TRUE )
    {
        if ( workspace->i_am_group_owner != 1 )
        {
            if ( workspace->group_candidate.p2p_capability & P2P_GROUP_CAPABILITY_P2P_PERSISTENT_GROUP )
            {
                WPRINT_LIB_INFO(( "Storing credentials for %s into WIFI DCT[%d] section.\n", workspace->group_candidate.ssid, WIFI_DCT_P2P_PGROUP_OFFSET ));
                wiced_dct_read_lock( (void**) &dct_wifi_config, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof(platform_dct_wifi_config_t) );
                memcpy(dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].details.SSID.value, workspace->group_candidate.ssid, MIN( workspace->group_candidate.ssid_length, MAX_SSID_LEN ) );
                dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].details.SSID.length = workspace->group_candidate.ssid_length;
                memcpy(&dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].details.BSSID, &workspace->group_candidate.bssid, sizeof( besl_mac_t ) );
                dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].details.security = WICED_SECURITY_WPA2_AES_PSK;
                memcpy(dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].security_key, (char*)workspace->p2p_passphrase, 64);
                dct_wifi_config->stored_ap_list[WIFI_DCT_P2P_PGROUP_OFFSET].security_key_length = workspace->p2p_passphrase_length;
                /* Write Their Persistent P2P Group in WiFi DCT[4] */
                wiced_dct_write( (const void*)dct_wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof( platform_dct_wifi_config_t));
                wiced_dct_read_unlock( (void*) dct_wifi_config, WICED_TRUE );

                /* Store the p2p device address */
                memcpy(&persistent_go_address, &workspace->group_candidate.p2p_device_address, sizeof( besl_mac_t) );
            }

            /* Enable PM1 */
            if (besl_p2p_client_enable_powersave(workspace, PM1_POWERSAVE_MODE) != BESL_SUCCESS)
            {
                WPRINT_LIB_INFO( ("Failed to set PM1 on P2P interface\n") );
            }

            if (connection_p2p_result_callback != NULL)
            {
                connection_p2p_result_callback(CONNECTION_P2P_CONNECTED);
            }

            /* Print the device interface, not BSSID, and SSID */
            WPRINT_LIB_INFO(( "status,COMPLETE,result,CLIENT,groupid,%02X:%02X:%02X:%02X:%02X:%02X %s\n", workspace->group_candidate.p2p_device_address.octet[0],
                        workspace->group_candidate.p2p_device_address.octet[1],
                        workspace->group_candidate.p2p_device_address.octet[2],
                        workspace->group_candidate.p2p_device_address.octet[3],
                        workspace->group_candidate.p2p_device_address.octet[4],
                        workspace->group_candidate.p2p_device_address.octet[5],
                        workspace->group_candidate.ssid));
        }
        else if ( p2p_workspace.initiate_negotiation == 1 )
        {
            /* Print the device interface and SSID */
            WPRINT_LIB_INFO(( "status,COMPLETE,result,GO,groupid,%02X:%02X:%02X:%02X:%02X:%02X %s\n", workspace->p2p_device_address.octet[0],
                        workspace->p2p_device_address.octet[1],
                        workspace->p2p_device_address.octet[2],
                        workspace->p2p_device_address.octet[3],
                        workspace->p2p_device_address.octet[4],
                        workspace->p2p_device_address.octet[5],
                        workspace->group_candidate.ssid));
        }
        else
        {
            WPRINT_LIB_INFO(( "status,COMPLETE\n"));
        }

        if ( ( workspace->i_am_group_owner == 1 ) && ( workspace->form_persistent_group == 1 ) )
        {
            wiced_dct_read_lock( (void**) &dct_p2p_config, WICED_TRUE, DCT_P2P_CONFIG_SECTION, 0, sizeof(platform_dct_p2p_config_t) );
            workspace->persistent_group.details_valid = CONFIG_VALIDITY_VALUE;
            memcpy( &dct_p2p_config->p2p_group_owner_settings, &workspace->persistent_group, sizeof( wiced_config_soft_ap_t ) );
            /* Write Our Persistent P2P Group in P2P DCT */
            wiced_dct_write( (const void*)dct_p2p_config, DCT_P2P_CONFIG_SECTION, 0, sizeof( platform_dct_p2p_config_t));
            wiced_dct_read_unlock( (void*) dct_p2p_config, WICED_TRUE );
        }

        /* Get the channel that the radio is on.
           For 40/80MHz capable radios this may display the center frequency of the channel rather than the primary channel of the WLAN */
        if ( wwd_wifi_get_channel( WWD_P2P_INTERFACE, &channel ) == WWD_SUCCESS )
        {
            WPRINT_LIB_INFO( ("\n*** Associated on channel: %u ***\n", (unsigned int) channel ) );
        }
    }
    else
    {
        if (connection_p2p_result_callback != NULL)
        {
            connection_p2p_result_callback(CONNECTION_P2P_FAILED);
        }
        WPRINT_LIB_INFO(( "status,COMPLETE,result,FAIL,group_formation, interface %u\n", (unsigned int)workspace->p2p_interface ));
    }

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * P2P device disassociation handler
 *
 * @return  0 for success
 */

static wiced_result_t connection_p2p_device_disassociation_handler( void* mac )
{
    wiced_mac_t mac_address;

    if (connection_p2p_result_callback != NULL)
    {
        connection_p2p_result_callback(CONNECTION_P2P_DISCONNECTED);
    }

    if ( mac != NULL )
    {
        memcpy(&mac_address, mac, sizeof(wiced_mac_t));

        WPRINT_LIB_INFO( ( "\nP2P device disassociated:" ) );
        WPRINT_LIB_INFO( ( "  %02X:%02X:%02X:%02X:%02X:%02X\n",
            mac_address.octet[0],
            mac_address.octet[1],
            mac_address.octet[2],
            mac_address.octet[3],
            mac_address.octet[4],
            mac_address.octet[5] ) );
    }
    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * P2P legacy device disassociation handler for non-P2P devices and WPS enrollees (P2P and non-P2P)
 *
 * @return  0 for success
 */

static wiced_result_t connection_p2p_legacy_device_disassociation_handler( void* mac )
{
    wiced_mac_t mac_address;

    if (connection_p2p_result_callback != NULL)
    {
        connection_p2p_result_callback(CONNECTION_P2P_DISCONNECTED);
    }

    if ( mac != NULL )
    {
        memcpy(&mac_address, mac, sizeof(wiced_mac_t));

        WPRINT_LIB_INFO( ( "\nLegacy device disassociated:" ) );
        WPRINT_LIB_INFO( ( "  %02X:%02X:%02X:%02X:%02X:%02X\n",
            mac_address.octet[0],
            mac_address.octet[1],
            mac_address.octet[2],
            mac_address.octet[3],
            mac_address.octet[4],
            mac_address.octet[5] ) );
    }
    return WICED_SUCCESS;
}


static wiced_result_t connection_check_mac_locally_administered_bit( void )
{
    wiced_mac_t   mac;

    wiced_wifi_get_mac_address( &mac );
    if ( mac.octet[0] & MAC_ADDRESS_LOCALLY_ADMINISTERED_BIT )
    {
        WPRINT_LIB_INFO(( "Error: MAC address is locally administered. Modify MAC address in generated_mac_address.txt file to be globally\n" ));
        WPRINT_LIB_INFO(( "administered, e.g. if first byte of MAC address is 0x02 change it to 0x00. If testing multiple Wiced p2p devices\n" ));
        WPRINT_LIB_INFO(( "ensure that they have unique MAC addresses.\n" ));
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * WLAN link events handler
 *
 * To notify link down event when we are in GC role
 */

static void* connection_link_events_handler(const wwd_event_header_t* event_header, const uint8_t* event_data, void* handler_user_data)
{
    UNUSED_PARAMETER(event_data);

    if (event_header->event_type == WLC_E_LINK)
    {
        if ((event_header->flags & WLC_EVENT_MSG_LINK) == 0)
        {
            if (connection_p2p_result_callback != NULL)
            {
                connection_p2p_result_callback(CONNECTION_P2P_DISCONNECTED);
            }
        }
    }

    return handler_user_data;
}

/*!
 ******************************************************************************
 * P2P device connection request callback
 *
 * This callback is called as the result of an event so a message is sent to the worker thread which has enough memory to print.
 *
 */

static void p2p_connection_request_callback( p2p_discovered_device_t* device )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, connection_p2p_connection_request_handler, device );
}

/*!
 ******************************************************************************
 * P2P legacy device connection request callback
 *
 * This indicates if a non-P2P client device is sending probe requests with push button mode asserted. The callback is called as the result of an event
 * so a message is sent to the worker thread which has enough memory to print.
 *
 */

static void p2p_legacy_device_connection_request_callback( p2p_legacy_device_t* device )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, connection_p2p_legacy_device_connection_request_handler, device );
}

/*!
 ******************************************************************************
 * WPA2 client association callback
 *
 * This callback is called as the result of an event so a message is sent to the worker thread which has enough memory to print.
 * This call back occurs when a device associates using WPA2, but before the device has completed the key handshake and requested an IP address.
 *
 */

static void p2p_wpa2_client_association_callback( besl_mac_t* mac )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, connection_p2p_wpa2_client_association_handler, mac );
}

/*!
 ******************************************************************************
 * WPS enrollee association callback
 *
 * This callback is called as the result of an event so a message is sent to the worker thread which has enough memory to print.
 * This callback occurs when a device associates with the WPS IE and without the RSN IE in its association request, which implies that it is going to attempt the WPS handshake.
 *
 */

static void p2p_wps_enrollee_association_callback( besl_mac_t* mac )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, connection_p2p_wps_enrollee_association_handler, mac );
}

/*!
 ******************************************************************************
 * WPS result callback
 *
 * This callback is called as the result of an event so a message is sent to the worker thread which has enough memory to print.
 * This callback occurs when a device associates with the WPS IE and without the RSN IE in its association request, which implies that it is going to attempt the WPS handshake.
 *
 */

static void p2p_wps_result_callback( wps_result_t* wps_result )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, connection_p2p_wps_result_handler, wps_result );
}

/*!
 ******************************************************************************
 * P2P group formation result callback
 *
 */

static void p2p_group_formation_result_callback( void* not_used )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, connection_p2p_group_formation_result_handler, NULL );
}

/*!
 ******************************************************************************
 * P2P device disassociation callback
 *
 * This callback is called as the result of an event so a message is sent to the worker thread which has enough memory to print.
 *
 */

static void p2p_device_disassociation_callback( besl_mac_t* device )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, connection_p2p_device_disassociation_handler, device );
}

/*!
 ******************************************************************************
 * P2P legacy device disassociation callback
 *
 * This callback is called as the result of an event so a message is sent to the worker thread which has enough memory to print.
 *
 */

static void p2p_legacy_device_disassociation_callback( besl_mac_t* mac )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, connection_p2p_legacy_device_disassociation_handler, mac );
}

/*!
 ******************************************************************************
 * Create P2P worker thread to handle connection request callbacks
 *
 * @return  0 for success, otherwise error
 */

static wiced_result_t connection_create_p2p_worker_thread( void )
{
    WPRINT_LIB_INFO( ( "Creating p2p app worker thread\n" ) );
    memset( &p2p_worker_thread, 0, sizeof( wiced_worker_thread_t) );
    if ( wiced_rtos_create_worker_thread( &p2p_worker_thread, WICED_DEFAULT_WORKER_PRIORITY, 4096, 5 ) != WICED_SUCCESS )  // XXX reduce mem size later
    {
        WPRINT_LIB_INFO(("Error when creating P2P worker thread\n"));
        return WICED_ERROR;
    }

    p2p_worker_thread_running = 1;
    return WICED_SUCCESS;
}
