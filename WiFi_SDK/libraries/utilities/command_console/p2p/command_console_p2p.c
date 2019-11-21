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
#include "command_console_p2p.h"

#include "command_console.h"
#include "wiced_p2p.h"
#include "wps_host.h"
#include "wiced_wifi.h"
#include "wiced_management.h"
#include "wiced_wps.h"
#include "besl_host.h"
#include "platform_dct.h"
#include "wiced_framework.h"
#include "wwd_wlioctl.h"

#define MAX_PASSPHRASE_LENGTH                 64
#define MAX_SSID_LENGTH                       32
#define DOT11_PMK_LENGTH                      32
#define MAC_ADDRESS_LOCALLY_ADMINISTERED_BIT  0x02

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
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
static besl_p2p_device_detail_t device_details =
{
    .wps_device_details =
    {
        .device_name     = "WICED_P2P",
        .manufacturer    = "Cypress",
        .model_name      = "BCM943362",
        .model_number    = "Wiced",
        .serial_number   = "12345670",
        .device_category = (besl_wps_device_category_t)WICED_WPS_DEVICE_COMPUTER,
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
    .group_owner_intent = 1,
    .go_configuration_timeout = 100,    // 1000 milliseconds (Units of 10 milliseconds)
    .client_configuration_timeout = 50, // 500 milliseconds
    .device_password_id = WPS_DEFAULT_DEVICEPWDID,
    .peer_device_timeout = 60000,       // For timing devices out of the p2p peer list (in milliseconds)
    .group_formation_timeout = 30000,      // For timing out a group formation attempt (in milliseconds)
    .p2p_capability = 0x0820, // Intra BSS Distribution, Invitation Procedure
};

p2p_workspace_t       p2p_workspace;
static uint8_t        test_client_join_mode = 0;
wiced_worker_thread_t p2p_worker_thread;
static char           test_pin_string[9];

extern char           last_started_ssid[33];

void                  p2p_connection_request_callback( p2p_discovered_device_t* device );
void                  p2p_legacy_device_connection_request_callback( p2p_legacy_device_t* device );
void                  p2p_wpa2_client_association_callback( besl_mac_t* );
void                  p2p_wps_enrollee_association_callback( besl_mac_t* );
void                  p2p_group_formation_result_callback( void* );
void                  p2p_wps_result_callback( wps_result_t* wps_result );
void                  p2p_device_disassociation_callback( besl_mac_t* );
void                  p2p_legacy_device_disassociation_callback( besl_mac_t* );
static wiced_result_t create_p2p_worker_thread( void );
static wiced_result_t p2p_connection_request_handler( void* connecting_device );
static wiced_result_t p2p_legacy_device_connection_request_handler( void* device_mac );
static wiced_result_t p2p_wpa2_client_association_handler( void* mac );
static wiced_result_t p2p_wps_enrollee_association_handler( void* mac );
static wiced_result_t p2p_group_formation_result_handler( void* not_used );
static wiced_result_t p2p_wps_result_handler( void* result );
static wiced_result_t p2p_device_disassociation_handler( void* mac );
static wiced_result_t p2p_legacy_device_disassociation_handler( void* mac );
static int            check_mac_locally_administered_bit( void );
extern void           dehyphenate_pin(char* str );

static uint8_t        connecting_device_index = 255;
static uint8_t        p2p_worker_thread_running = 0;



/******************************************************
 *               Function Definitions
 ******************************************************/

/*!
 ******************************************************************************
 * Connect to another P2P device
 *
 * @return  0 for success, otherwise error
 */

int p2p_connect( int argc, char* argv[] )
{
    uint8_t selected_device;

    if ( !( argc > 1 ) )
    {
        WPRINT_APP_INFO(( "Insufficient arguments\n" ));
        return ( -1 );
    }

    if ( p2p_workspace.p2p_initialised != 1 )
    {
        if ( p2p_discovery_enable( 0, NULL ) != 0 )
        {
            return -1;
        }
    }
    else if ( wwd_wifi_is_ready_to_transceive(WWD_P2P_INTERFACE) == WWD_SUCCESS )
    {
        if ( besl_p2p_group_owner_is_up() == WICED_TRUE )
        {
            WPRINT_APP_INFO(( "P2P group owner is already up\n" ));
        }
        else
        {
            WPRINT_APP_INFO(( "P2P client is already up\n" ));
        }
        return -1;
    }
    else
    {
        p2p_discovery_disable( 0, NULL );
        p2p_discovery_enable( 0, NULL );
    }

    if ( strcmp( argv[1], "pin" ) == 0 )
    {
        /* PIN mode*/
        p2p_workspace.p2p_wps_mode = WPS_PIN_MODE;
        if ( argc == 3 ) // PIN is supplied by other device and entered by the user
        {
            if ( is_digit_str(argv[2]) && ( ( strlen( argv[2] ) == 4 ) || ( strlen( argv[2] ) == 8 ) ) )
            {
                if ( strlen( argv[2] ) == 8 )
                {
                    if ( !besl_wps_validate_pin_checksum( argv[3] ) )
                    {
                        WPRINT_APP_INFO(("Invalid PIN checksum\n"));
                        return ( ERR_CMD_OK );
                    }
                }
                WPRINT_APP_INFO(("Starting group formation in PIN mode\n"));
                p2p_workspace.p2p_wps_device_password_id = WPS_USER_SPEC_DEVICEPWDID;
                strncpy( p2p_workspace.p2p_wps_pin, argv[2], 8 );
                p2p_workspace.p2p_wps_pin[8] = 0;
            }
            else
            {
                WPRINT_APP_INFO(("PIN must be 4 or 8 digits\n"));
                return ( ERR_CMD_OK );
            }
        }
        else if ( argc == 2 ) // PIN is auto-generated by this device and entered in the other device by its user
        {
            p2p_workspace.p2p_wps_device_password_id = WPS_DEVICEPWDID_REG_SPEC;
            besl_wps_generate_pin( p2p_workspace.p2p_wps_pin );
            WPRINT_APP_INFO(("Starting group formation in PIN mode. Enter this PIN in the other device: %s\n", p2p_workspace.p2p_wps_pin));
        }
        else
        {
            WPRINT_APP_INFO(( "Wrong number of arguments\n"));
            return -1;
        }
    }
    else if ( strcmp( argv[1], "pbc" ) == 0 )
    {
        /* Push Button mode */
        if (argc == 2)
        {
            p2p_workspace.p2p_wps_mode = WPS_PBC_MODE;
            p2p_workspace.p2p_wps_device_password_id = WPS_PUSH_BTN_DEVICEPWDID;
            WPRINT_APP_INFO(("Starting group formation in PBC mode\n"));
            char pbc_password[9] = "00000000";
            memcpy( p2p_workspace.p2p_wps_pin, pbc_password, 9 );
        }
        else
        {
            WPRINT_APP_INFO(( "Wrong number of arguments\n" ));
            return ERR_UNKNOWN;
        }
    }
    else
    {
        return ERR_UNKNOWN;
    }

    WPRINT_APP_INFO(( "Discovering peers... please wait\n" ));
    host_rtos_delay_milliseconds( 3000 ); // Delay so that the peer list has a chance to fill

    /* Display the list of discovered devices */
    p2p_peer_list( 0, NULL );
    if ( p2p_workspace.discovered_device_count < 1 )
    {
        WPRINT_APP_INFO(( "No peer devices found\n" ));
        return 0;
    }

    /* Allow the user to select from the list unless they have accepted a connection request, in which case the index of the connecting device will not be 255 */
    wiced_bool_t found = WICED_FALSE;
    if ( connecting_device_index == 255 )
    {
        WPRINT_APP_INFO(( "Enter the number (#) of the peer to connect to\n" ));
        do
        {
            selected_device = (1 + getchar( ) - '1')%10;
            if ( selected_device < P2P_MAX_DISCOVERED_DEVICES )
            {
                if( p2p_workspace.discovered_devices[selected_device].status != P2P_DEVICE_INVALID )
                {
                    found = WICED_TRUE;
                }
            }
        } while ( found != WICED_TRUE );
    }
    else
    {
        selected_device = connecting_device_index;
        connecting_device_index = 255;
    }

    /* Attempt to form a new group or join an existing one */
    memcpy( &p2p_workspace.group_candidate.p2p_device_address, &p2p_workspace.discovered_devices[selected_device].p2p_device_address, sizeof( wiced_mac_t ) );

    /* Check if it's a GO or not first */
    if ( p2p_workspace.discovered_devices[selected_device].group_owner_capability & 0x01 )
    {
        p2p_workspace.group_candidate.ssid_length = strlen( p2p_workspace.discovered_devices[selected_device].ssid );
        memcpy( p2p_workspace.group_candidate.ssid, p2p_workspace.discovered_devices[selected_device].ssid, MIN( 32, p2p_workspace.group_candidate.ssid_length ) );

        // This will need to be adjusted to accommodate NFC
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
        p2p_workspace.initiate_negotiation = 1;
        p2p_workspace.ok_to_accept_negotiation = 1;

        if ( besl_p2p_start_negotiation( &p2p_workspace ) != 0 )
        {
            WPRINT_APP_INFO(("besl_p2p_start_negotiation failed\n"));
        }
        else
        {
            WPRINT_APP_INFO(("Candidate Group ID: %02X:%02X:%02X:%02X:%02X:%02X %s\n",
            p2p_workspace.p2p_device_address.octet[0],
            p2p_workspace.p2p_device_address.octet[1],
            p2p_workspace.p2p_device_address.octet[2],
            p2p_workspace.p2p_device_address.octet[3],
            p2p_workspace.p2p_device_address.octet[4],
            p2p_workspace.p2p_device_address.octet[5],
            p2p_workspace.group_candidate.ssid));
        }
    }

    return 0;
}


/*!
 ******************************************************************************
 * Test P2P client connecting to an existing group owner multiple times. Use
 * the p2p_go_client_test_mode command on the group owner to enter the WPS PIN
 * and put the group owner into a test mode where it accepts connection
 * requests.
 *
 * @return  0 for success, otherwise error
 */
int p2p_client_test( int argc, char* argv[] )
{
    int i;
    int iterations = 5;
    int ssid_len = 0;
    p2p_discovered_device_t* devices;
    uint8_t device_count;
    wiced_bool_t go_found;

    if ( ! ( ( argc == 3 ) || ( argc == 4 ) ) )
    {
        WPRINT_APP_INFO(("Wrong number of arguments\n"));
        return -1;
    }

    if ( argc == 4 )
    {
        iterations = atoi(argv[argc - 1]);
    }

    if ( !besl_wps_validate_pin_checksum( argv[2] ) )
    {
        WPRINT_APP_INFO(("Invalid PIN checksum\n"));
        return ( ERR_CMD_OK );
    }

    for ( i = 0; i < iterations; i++ )
    {
        WPRINT_APP_INFO(("-------------------- Iteration %d --------------------\n", i));

        p2p_discovery_disable( 0, NULL );
        p2p_discovery_enable( 0, NULL );

        while ( p2p_workspace.p2p_current_state == P2P_STATE_UNINITIALIZED )
        {
            host_rtos_delay_milliseconds( 10 );
        }

        p2p_workspace.p2p_wps_mode = WPS_PIN_MODE;
        p2p_workspace.p2p_wps_device_password_id = WPS_DEVICEPWDID_REG_SPEC;
        p2p_workspace.provisioning_config_method = KEYPAD;
        strncpy( p2p_workspace.p2p_wps_pin, argv[2], 8 );
        p2p_workspace.p2p_wps_pin[8] = 0;

        go_found = WICED_FALSE;
        host_rtos_delay_milliseconds( 2000 ); /* Delay so that the peer list has a chance to fill */
        besl_p2p_get_discovered_peers(&p2p_workspace, &devices, &device_count);

        int a = P2P_MAX_DISCOVERED_DEVICES;
        while( ( a > 0 ) && ( go_found == WICED_FALSE ) )
        {
            --a;
            if ( ( devices[a].group_owner_capability & 0x01 ) == 1 )
            {
                ssid_len = strlen( devices[a].ssid );
                if ( memcmp( argv[1], devices[a].ssid, ssid_len ) == 0 )
                {
                    go_found = WICED_TRUE;
                }
            }
        }

        if ( go_found )
        {
            WPRINT_APP_INFO(("Found GO index %u\n", (unsigned int)a));
            p2p_workspace.group_candidate.ssid_length = MIN( ssid_len, 32 );
            memcpy( p2p_workspace.group_candidate.ssid, devices[a].ssid, p2p_workspace.group_candidate.ssid_length );
            memcpy( &p2p_workspace.group_candidate.p2p_device_address, &p2p_workspace.discovered_devices[a].p2p_device_address, sizeof( wiced_mac_t ) );
            besl_p2p_find_group_owner( &p2p_workspace );

            while ( ( wiced_network_is_ip_up( WICED_P2P_INTERFACE ) != WICED_TRUE ) && ( p2p_workspace.p2p_result != BESL_P2P_ERROR_FAIL ) )
            {
                host_rtos_delay_milliseconds( 1000 );
            }
            if ( p2p_workspace.p2p_result == BESL_P2P_ERROR_FAIL )
            {
                WPRINT_APP_INFO(("P2P connection failed. P2P current state: %u\n", (unsigned int) p2p_workspace.p2p_current_state));
            }
        }
        else
        {
            WPRINT_APP_INFO(("Group owner not found\n"));
        }

        wiced_network_down( WICED_P2P_INTERFACE );
        host_rtos_delay_milliseconds( 10 );

        /* The workspace will be deinitialised at the start of the loop */

        WPRINT_APP_INFO(("Client stopped\n"));
    }

    if ( p2p_worker_thread_running == 1 )
    {
        WPRINT_APP_INFO(("Deleting p2p worker thread\n"));
        if ( wiced_rtos_delete_worker_thread( &p2p_worker_thread ) != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("Problem deleting p2p worker thread\n"));
        }
        p2p_worker_thread_running = 0;
    }
    if ( p2p_workspace.p2p_initialised )
    {
        WPRINT_APP_INFO(("Deinit p2p workspace\n"));
        besl_p2p_deinit( &p2p_workspace );
    }

    return 0;
}

/*!
 ******************************************************************************
 * Test P2P group formation using the negotiation handshake. Run this command
 * on the device which initiates the connection request.
 *
 * @return  0 for success, otherwise error
 */
int p2p_negotiation_test( int argc, char* argv[] )
{
    int i;
    int iterations = 5;
    uint8_t selected_device;
    besl_mac_t target_device_address;
    char wps_pin[9];

    /* PIN mode*/
    if ( is_digit_str(argv[1]) && ( strlen( argv[1] ) == 8 ) )
    {
        if ( !besl_wps_validate_pin_checksum( argv[1] ) )
        {
            WPRINT_APP_INFO(("Invalid PIN checksum\n"));
            return ( ERR_CMD_OK );
        }
        WPRINT_APP_INFO(("Starting group formation in PIN mode\n"));
        strncpy( wps_pin, argv[1], 8 );
        wps_pin[8] = 0;
        if ( argc == 3 )
        {
            iterations = atoi(argv[argc - 1]);
        }
    }
    else
    {
        WPRINT_APP_INFO(("PIN must be 8 digits\n"));
        return ( ERR_CMD_OK );
    }

    if ( p2p_workspace.p2p_initialised != 1 )
    {
        p2p_discovery_enable( 0, NULL );
    }
    else if ( wwd_wifi_is_ready_to_transceive(WWD_P2P_INTERFACE) == WWD_SUCCESS )
    {
        if ( besl_p2p_group_owner_is_up() == WICED_TRUE )
        {
            WPRINT_APP_INFO(( "P2P group owner is already up\n" ));
        }
        else
        {
            WPRINT_APP_INFO(( "P2P client is already up\n" ));
        }
        return -1;
    }
    else
    {
        p2p_discovery_disable( 0, NULL );
        p2p_discovery_enable( 0, NULL );
    }

    WPRINT_APP_INFO(( "Discovering peers... please wait\n" ));
    host_rtos_delay_milliseconds( 2000 ); // Delay so that the peer list has a chance to fill

    /* Display the list of discovered devices */
    p2p_peer_list( 0, NULL );
    if ( p2p_workspace.discovered_device_count < 1 )
    {
        WPRINT_APP_INFO(( "No peer devices found\n" ));
        return 0;
    }

    /* Allow the user to select from the list */
    wiced_bool_t found = WICED_FALSE;
    if ( connecting_device_index == 255 )
    {
        WPRINT_APP_INFO(( "Enter the number (#) of the peer to connect to\n" ));
        do
        {
            selected_device = (1 + getchar( ) - '1')%10;
            if ( selected_device < P2P_MAX_DISCOVERED_DEVICES )
            {
                if( p2p_workspace.discovered_devices[selected_device].status != P2P_DEVICE_INVALID )
                {
                    found = WICED_TRUE;
                    /* Attempt to form a new group or join an existing one */
                    memcpy( &target_device_address, &p2p_workspace.discovered_devices[selected_device].p2p_device_address, sizeof( wiced_mac_t ) );
                }
            }
        } while ( found != WICED_TRUE );
    }

    for ( i = 0; i < iterations; i++ )
    {
        WPRINT_APP_INFO(("-------------------- Iteration %d --------------------\n", i));

        p2p_discovery_disable( 0, NULL );
        p2p_discovery_enable( 0, NULL );

        while ( p2p_workspace.p2p_current_state == P2P_STATE_UNINITIALIZED )
        {
            host_rtos_delay_milliseconds( 10 );
        }

        memcpy( &p2p_workspace.group_candidate.p2p_device_address, &target_device_address, sizeof( wiced_mac_t ) );

        p2p_workspace.p2p_wps_mode = WPS_PIN_MODE;
        p2p_workspace.p2p_wps_device_password_id = WPS_DEVICEPWDID_REG_SPEC;
        p2p_workspace.provisioning_config_method = KEYPAD;
        strncpy( p2p_workspace.p2p_wps_pin, argv[2], 8 );
        p2p_workspace.p2p_wps_pin[8] = 0;
        p2p_workspace.initiate_negotiation = 1;
        p2p_workspace.ok_to_accept_negotiation = 1;

        if ( besl_p2p_start_negotiation( &p2p_workspace ) != 0 )
        {
            WPRINT_APP_INFO(("besl_p2p_start_negotiation failed\n"));
        }
        else
        {
            WPRINT_APP_INFO(("Candidate Group ID: %02X:%02X:%02X:%02X:%02X:%02X %s\n",
            p2p_workspace.p2p_device_address.octet[0],
            p2p_workspace.p2p_device_address.octet[1],
            p2p_workspace.p2p_device_address.octet[2],
            p2p_workspace.p2p_device_address.octet[3],
            p2p_workspace.p2p_device_address.octet[4],
            p2p_workspace.p2p_device_address.octet[5],
            p2p_workspace.group_candidate.ssid));
        }
        while ( ( wiced_network_is_ip_up( WICED_P2P_INTERFACE ) != WICED_TRUE ) && ( p2p_workspace.p2p_result != BESL_P2P_ERROR_FAIL ) )
        {
            host_rtos_delay_milliseconds( 1000 );
        }
        if ( p2p_workspace.p2p_result == BESL_P2P_ERROR_FAIL )
        {
            WPRINT_APP_INFO(("P2P connection failed. P2P current state: %u\n", (unsigned int) p2p_workspace.p2p_current_state));
        }

        wiced_network_down( WICED_P2P_INTERFACE );
        host_rtos_delay_milliseconds( 10 );

        /* The workspace will be deinitialised at the start of the loop */

        WPRINT_APP_INFO(("p2p stopped\n"));
    }

    if ( p2p_worker_thread_running == 1 )
    {
        WPRINT_APP_INFO(("Deleting p2p worker thread\n"));
        if ( wiced_rtos_delete_worker_thread( &p2p_worker_thread ) != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("Problem deleting p2p worker thread\n"));
        }
        p2p_worker_thread_running = 0;
    }
    if ( p2p_workspace.p2p_initialised )
    {
        WPRINT_APP_INFO(("Deinit p2p workspace\n"));
        besl_p2p_deinit( &p2p_workspace );
    }

    return 0;
}

/*!
 ******************************************************************************
 * Put P2P group owner into test mode where it accepts p2p client joins
 *
 * @return  0 for success, otherwise error
 */
int p2p_go_client_test_mode( int argc, char* argv[] )
{
    if ( ( p2p_workspace.i_am_group_owner != 1 ) || ( wwd_wifi_is_ready_to_transceive( WWD_P2P_INTERFACE ) != WWD_SUCCESS ) )
    {
        WPRINT_APP_INFO(( "P2P group owner not up\n"));
        return 0;
    }

    if ( ! ( ( argc == 2 ) || ( argc == 3 ) ) )
    {
        WPRINT_APP_INFO(("Wrong number of arguments\n"));
        return -1;
    }

    test_client_join_mode = atoi(argv[1]);

    if ( test_client_join_mode == 1 )
    {
        if ( argc != 3 )
        {
            WPRINT_APP_INFO(("Wrong number of arguments\n"));
            return -1;
        }
    }
    else
    {
        return 0;
    }

    if ( is_digit_str(argv[2]) && ( strlen( argv[2] ) == 8 ) )
    {
        if ( !besl_wps_validate_pin_checksum( argv[2] ) )
        {
            WPRINT_APP_INFO(("Invalid PIN checksum\n"));
            return ( ERR_CMD_OK );
        }
        strncpy( test_pin_string, argv[2], 8 );
        test_pin_string[8] = 0;
    }
    else
    {
        WPRINT_APP_INFO(("Invalid PIN\n"));
        return ( ERR_CMD_OK );
    }

    return 0;
}


/*!
 ******************************************************************************
 * Enable P2P in discovery mode
 *
 * @return  0 for success, otherwise error
 */

int p2p_discovery_enable( int argc, char* argv[] )
{
    besl_result_t result;
    wiced_bool_t listen_discovery = WICED_FALSE;

    if ( check_mac_locally_administered_bit( ) != 0 )
    {
        return -1;
    }

    if ( argc > 1 )
    {
        if ( strncmp( argv[1], "listen", 6 ) == 0 ) {
            listen_discovery = WICED_TRUE;
        }
        else
        {
            listen_discovery = WICED_FALSE;
        }
    }

    if ( p2p_workspace.p2p_initialised == 1 )
    {
        if ( wwd_wifi_is_ready_to_transceive( WWD_P2P_INTERFACE ) == WWD_SUCCESS )
        {
            if ( besl_p2p_group_owner_is_up() == WICED_TRUE )
            {
                WPRINT_APP_INFO(( "P2P group owner is already up\n" ));
            }
            else
            {
                WPRINT_APP_INFO(( "P2P client is already up\n" ));
            }
            return 0;
        }
        WPRINT_APP_INFO(( "Calling p2p discovery disable again\n" ));
        p2p_discovery_disable( 0, NULL );
    }

    if ( p2p_worker_thread_running != 1 )
    {
        if ( create_p2p_worker_thread() == WICED_SUCCESS )
        {
            if ( ( result = besl_p2p_init( &p2p_workspace, &device_details ) ) != BESL_SUCCESS )
            {
                WPRINT_APP_INFO(( "besl_p2p_init failed %u\n", (unsigned int)result ));
                if ( p2p_worker_thread_running == 1 )
                {
                    wiced_rtos_delete_worker_thread( &p2p_worker_thread );
                    p2p_worker_thread_running = 0;
                }
                return -1;
            }
            besl_p2p_register_p2p_device_connection_callback( &p2p_workspace, p2p_connection_request_callback);
            besl_p2p_register_legacy_device_connection_callback( &p2p_workspace, p2p_legacy_device_connection_request_callback );
            besl_p2p_register_wpa2_client_association_callback( &p2p_workspace, p2p_wpa2_client_association_callback );
            besl_p2p_register_wps_enrollee_association_callback( &p2p_workspace, p2p_wps_enrollee_association_callback );
            besl_p2p_register_group_formation_result_callback( &p2p_workspace, p2p_group_formation_result_callback );
            besl_p2p_register_wps_result_callback( &p2p_workspace, p2p_wps_result_callback );
            if (listen_discovery == WICED_TRUE) {
                besl_p2p_listen_start( &p2p_workspace );
            }
            else
            {
                besl_p2p_start( &p2p_workspace );
            }
        }
    }

    return 0;
}


/*!
 ******************************************************************************
 * Disable P2P discovery mode
 *
 * @return  0 for success, otherwise error
 */
int p2p_discovery_disable( int argc, char* argv[] )
{
    if ( p2p_workspace.p2p_initialised )
    {
        if ( besl_p2p_deinit( &p2p_workspace ) != 0 )
        {
            WPRINT_APP_INFO(("Error when disabling P2P discovery\n"));
        }
        else
        {
            WPRINT_APP_INFO(("P2P discovery is now disabled\n"));
        }
        if ( p2p_worker_thread_running == 1 )
        {
            wiced_rtos_delete_worker_thread( &p2p_worker_thread );
            p2p_worker_thread_running = 0;
        }
    }
    return 0;
}


/*!
 ******************************************************************************
 * Test P2P discovery enable/disable
 *
 * @return  0 for success, otherwise error
 */

int p2p_discovery_test( int argc, char* argv[] )
{
    int i;
    int iterations = 10;

    if (  argc > 1 )
    {
        iterations = atoi(argv[argc - 1]);
    }

    for ( i = 0; i < iterations; i++ )
    {
        WPRINT_APP_INFO(("Iteration %d ", i));
        p2p_discovery_enable( 0, NULL );
        WPRINT_APP_INFO(("enabled, "));
        p2p_discovery_disable( 0, NULL );
        WPRINT_APP_INFO(("disabled\n"));
    }

    return 0;
}



/*!
 ******************************************************************************
 * Start autonomous P2P Group Owner
 *
 * @return  0 for success, otherwise error
 */
int p2p_go_start( int argc, char* argv[] )
{
    int                         ssid_suffix_length = 0;
    int                         offset = 0;
#ifdef WICED_DCT_INCLUDE_P2P_CONFIG
    platform_dct_p2p_config_t*  dct_p2p_group_owner_config;
#endif
    besl_result_t               result;

    if ( check_mac_locally_administered_bit( ) != 0 )
    {
        return -1;
    }

    if ( wwd_wifi_is_ready_to_transceive( WWD_P2P_INTERFACE ) == WWD_SUCCESS )
    {
        WPRINT_APP_INFO(( " P2P interface is already in use "));
        return -1;
    }
    else
    {
        p2p_discovery_disable( 0, NULL );
    }

    memset(&p2p_workspace, 0, sizeof(p2p_workspace_t));
    p2p_workspace.i_am_group_owner = 1;

    if ( argc == 2 || argc == 5 )
    {
        besl_p2p_init_common( &p2p_workspace, &device_details );
        besl_p2p_register_p2p_device_connection_callback( &p2p_workspace, p2p_connection_request_callback);
        besl_p2p_register_legacy_device_connection_callback( &p2p_workspace, p2p_legacy_device_connection_request_callback );
        besl_p2p_register_wpa2_client_association_callback( &p2p_workspace, p2p_wpa2_client_association_callback );
        besl_p2p_register_wps_enrollee_association_callback( &p2p_workspace, p2p_wps_enrollee_association_callback );
        besl_p2p_register_p2p_device_disassociation_callback( &p2p_workspace, p2p_device_disassociation_callback );
        besl_p2p_register_legacy_device_disassociation_callback( &p2p_workspace, p2p_legacy_device_disassociation_callback );
    }
    else
    {
        return -1;
    }

    /* Check if this is to be a persistent group or non-persistent group */
    if ( strcmp( argv[1], "p" ) == 0 )
    {
        p2p_workspace.form_persistent_group = 1;
        offset = 2;
    }
    else if ( ( strcmp( argv[1], "n" ) != 0 ) )
    {
        return -1;
    }

    p2p_workspace.wps_device_details = &device_details.wps_device_details;

    /* Create new group with specified parameters */
    if ( argc == 5 )
    {
        /* Append the SSID suffix to the Wi-Fi Direct SSID */
        ssid_suffix_length = strlen( argv[2] );
        if ( ( p2p_workspace.group_candidate.ssid_length + ssid_suffix_length - offset ) <= MAX_SSID_LENGTH )
        {
            memcpy( &p2p_workspace.group_candidate.ssid[p2p_workspace.group_candidate.ssid_length - offset], argv[2], ssid_suffix_length );
            p2p_workspace.group_candidate.ssid_length += ( ssid_suffix_length - offset );
        }

        p2p_workspace.p2p_passphrase_length = strlen( argv[3] );
        memcpy( &p2p_workspace.p2p_passphrase, argv[3], p2p_workspace.p2p_passphrase_length );

        /* Store the operating channel */
        p2p_workspace.operating_channel.channel = (uint8_t)atoi(argv[4]);
    }
    /* If there's a valid group in flash then use it, otherwise create new parameters which will be stored after group formation. */
    else if ( p2p_workspace.form_persistent_group == 1 )
    {
        p2p_workspace.reinvoking_group = 1;

#ifdef WICED_DCT_INCLUDE_P2P_CONFIG
        wiced_dct_read_lock( (void**) &dct_p2p_group_owner_config, WICED_TRUE, DCT_P2P_CONFIG_SECTION, 0, sizeof(platform_dct_p2p_config_t) );
        memcpy( &p2p_workspace.persistent_group, &dct_p2p_group_owner_config->p2p_group_owner_settings, sizeof( wiced_config_soft_ap_t ) );
        wiced_dct_read_unlock( (void*) dct_p2p_group_owner_config, WICED_TRUE );
#endif
        p2p_workspace.group_candidate.ssid_length = p2p_workspace.persistent_group.SSID.length;
        memcpy( &p2p_workspace.group_candidate.ssid, p2p_workspace.persistent_group.SSID.value, p2p_workspace.group_candidate.ssid_length );
        p2p_workspace.p2p_passphrase_length = p2p_workspace.persistent_group.security_key_length;
        memcpy( p2p_workspace.p2p_passphrase, p2p_workspace.persistent_group.security_key, p2p_workspace.p2p_passphrase_length );
        p2p_workspace.operating_channel.channel = p2p_workspace.persistent_group.channel;
        p2p_workspace.group_candidate.operating_channel.channel = p2p_workspace.operating_channel.channel;
    }

    p2p_workspace.device_name_length = strlen(p2p_workspace.wps_device_details->device_name);
    memcpy((char*)&p2p_workspace.device_name, (char*)device_details.wps_device_details.device_name, p2p_workspace.device_name_length);

    create_p2p_worker_thread();
    besl_p2p_register_wps_result_callback( &p2p_workspace, p2p_wps_result_callback );
    besl_p2p_register_group_formation_result_callback( &p2p_workspace, p2p_group_formation_result_callback );

    if ( ( result = besl_p2p_group_owner_start(&p2p_workspace) ) != BESL_SUCCESS )
    {
        WPRINT_APP_INFO( ("Error starting group owner: %u\n", (unsigned int )result ) );

        besl_p2p_deinit( &p2p_workspace );

        if ( p2p_worker_thread_running == 1 )
        {
            wiced_rtos_delete_worker_thread( &p2p_worker_thread );
            p2p_worker_thread_running = 0;
        }
    }
    else
    {
        memset( last_started_ssid, 0, sizeof(last_started_ssid) );
        memcpy( last_started_ssid, p2p_workspace.group_candidate.ssid, p2p_workspace.group_candidate.ssid_length );
    }

    return 0;
}


/*!
 ******************************************************************************
 * Stop P2P group owner
 *
 * @return  0 for success, otherwise error
 */

int p2p_go_stop( int argc, char* argv[] )
{
    if ( ( p2p_workspace.i_am_group_owner != 1 ) || ( wwd_wifi_is_ready_to_transceive( WWD_P2P_INTERFACE ) != WWD_SUCCESS ) )
    {
        WPRINT_APP_INFO(( "P2P group owner not up\n"));
        return 0;
    }

    p2p_registrar_stop( 0, NULL );
    wwd_wifi_deauth_all_associated_client_stas( WWD_DOT11_RC_UNSPECIFIED, WWD_P2P_INTERFACE );
    host_rtos_delay_milliseconds( 100 ); /* Delay to allow the deauthentication frames to be sent */

    if ( p2p_worker_thread_running == 1 )
    {
        wiced_rtos_delete_worker_thread( &p2p_worker_thread );
        p2p_worker_thread_running = 0;
    }
    if ( besl_p2p_deinit( &p2p_workspace ) != 0 )
    {
        WPRINT_APP_INFO(("Error when stopping P2P group owner\n"));
    }

    return 0;
}


/*!
 ******************************************************************************
 * Test P2P group owner repetitive start/stop
 *
 * @return  0 for success, otherwise error
 */

int p2p_go_test( int argc, char* argv[] )
{
    int i;
    int iterations = 10;

    if ( argc < 2 )
    {
        return -1;
    }

    if (  ( argc == 3 ) || ( argc == 6 ) )
    {
        iterations = atoi(argv[argc - 1]);
        argc--;
    }

    for ( i = 0; i < iterations; i++ )
    {
        WPRINT_APP_INFO(("Iteration %d\n", i));
        p2p_go_start( argc, argv );

        p2p_workspace.p2p_wps_device_password_id = WPS_DEVICEPWDID_REG_SPEC;
        besl_wps_generate_pin( p2p_workspace.p2p_wps_pin );
        WPRINT_APP_INFO(("Starting registrar in PIN mode. Enter this PIN in the other device: %s\n", p2p_workspace.p2p_wps_pin));
        besl_p2p_start_registrar();
        host_rtos_delay_milliseconds( 5000 );

        p2p_go_stop( 0, NULL );
        WPRINT_APP_INFO(("Group owner stopped\n"));
    }

    return 0;
}


/*!
 ******************************************************************************
 * Prints the P2P peer list
 *
 * @return  0 for success, otherwise error
 */
int p2p_peer_list( int argc, char* argv[] )
{
    p2p_discovered_device_t* devices;
    uint8_t device_count;
    uint8_t is_group_owner = 0;
    char group_owner[]     = "GO    ";
    char group_client[]    = "CLIENT";
    char sta[]             = "STA   ";
    char* role             = NULL;
    uint16_t channel       = 0;

    if ( p2p_workspace.p2p_initialised != 1 )
    {
        WPRINT_APP_INFO( ("Enabling P2P discovery and scanning for peers, please wait\n") );
        if ( p2p_discovery_enable( 0, NULL ) != 0 )
        {
            return -1;
        }
        host_rtos_delay_milliseconds( 3000 ); // Delay so that the peer list has a chance to fill
    }

    besl_p2p_get_discovered_peers(&p2p_workspace, &devices, &device_count);
    device_count = 0;
    WPRINT_APP_INFO( (" # Type    P2P Device Address  Chan  Device Name                       SSID (for GO)\n") );
    WPRINT_APP_INFO( ("-----------------------------------------------------------------------------------------------------\n") );
    while ( device_count < P2P_MAX_DISCOVERED_DEVICES )
    {
        if ( devices[device_count].status != P2P_DEVICE_INVALID )
        {
            is_group_owner = ( devices[device_count].group_owner_capability & 0x01 );
            if ( is_group_owner == 1 )
            {
                role = group_owner;
            }
            else if ( devices[device_count].status == P2P_DEVICE_ASSOCIATED_CLIENT )
            {
                role = group_client;
            }
            else
            {
                role = sta;
            }
            channel        = is_group_owner ? devices[device_count].operating_channel.channel : devices[device_count].listen_channel;

            WPRINT_APP_INFO( ( " %u ", device_count ) );
            WPRINT_APP_INFO( ( "%s  ", role) );
            WPRINT_APP_INFO( ( "%02X:%02X:%02X:%02X:%02X:%02X    ",
                devices[device_count].p2p_device_address.octet[0],
                devices[device_count].p2p_device_address.octet[1],
                devices[device_count].p2p_device_address.octet[2],
                devices[device_count].p2p_device_address.octet[3],
                devices[device_count].p2p_device_address.octet[4],
                devices[device_count].p2p_device_address.octet[5] ) );
            WPRINT_APP_INFO( ( "%2u ", channel ) );
            WPRINT_APP_INFO( ( "  %-32s", devices[device_count].device_name ) );
            if ( is_group_owner )
            {
                WPRINT_APP_INFO( ("  %s", devices[device_count].ssid ) );
            }
            WPRINT_APP_INFO( ( "\n") );
        }
        device_count++;
    }
    return 0;
}


/*!
 ******************************************************************************
 * Start P2P WPS Registrar
 *
 * @return  0 for success, otherwise error
 */
int p2p_registrar_start( int argc, char* argv[] )
{
    static char pin[10];
    static char pbc_password[10] = "00000000";

    if ( ( p2p_workspace.i_am_group_owner != 1 ) || ( wwd_wifi_is_ready_to_transceive( WWD_P2P_INTERFACE ) != WWD_SUCCESS ) )
    {
        WPRINT_APP_INFO(( "P2P group owner not up\n"));
        return 0;
    }

    // PIN mode
    if ( strcmp( argv[1], "pin" ) == 0 )
    {
        p2p_workspace.p2p_wps_mode = WPS_PIN_MODE;
        if ( argc >= 3 )
        {
            memset(pin, 0, sizeof(pin));
            strncpy( pin, argv[2], ( sizeof(pin) - 1 ));
            if ( argc == 4 ) // Then PIN may be in the form nnnn nnnn
            {
                if ( ( strlen( argv[2] ) == 4 ) && ( strlen( argv[3] ) == 4 ) )
                {
                    strncat( pin, argv[3], 4 );
                }
                else
                {
                    WPRINT_APP_INFO(("Invalid PIN format\n"));
                    return ( ERR_CMD_OK );
                }
                argc--;
            }
            if ( argc == 3 )
            {
                if ( strlen( pin ) == 9 ) // Then PIN may be in the form nnnn-nnnn
                {
                    dehyphenate_pin( pin );
                }
                //WPRINT_APP_INFO(("pin %s\n", pin));
                if ( is_digit_str(pin) && ( ( strlen( pin ) == 4 ) || ( strlen( pin ) == 8 ) ) )
                {
                    if ( strlen( pin ) == 8 )
                    {
                        if ( !besl_wps_validate_pin_checksum( pin ) )
                        {
                            WPRINT_APP_INFO(("Invalid PIN checksum\n"));
                            return ( ERR_CMD_OK );
                        }
                    }
                    WPRINT_APP_INFO(("Starting Registrar in PIN mode\n"));
                    strncpy( p2p_workspace.p2p_wps_pin, pin, 8 );
                    p2p_workspace.p2p_wps_pin[8] = 0;
                    besl_p2p_start_registrar();
                }
                else
                {
                    WPRINT_APP_INFO(("PIN must be 4 or 8 digits\n"));
                    return ( ERR_CMD_OK );
                }
            }
            else
            {
                return ERR_UNKNOWN;
            }
        }
        else if ( argc == 2 )
        {
            p2p_workspace.p2p_wps_device_password_id = WPS_DEVICEPWDID_REG_SPEC;
            besl_wps_generate_pin( p2p_workspace.p2p_wps_pin );
            WPRINT_APP_INFO(("Starting registrar in PIN mode. Enter this PIN in the other device: %s\n", p2p_workspace.p2p_wps_pin));
            besl_p2p_start_registrar();
        }
    }
    else if ( strcmp( argv[1], "pbc" ) == 0 )
    {
        /* Push Button mode */
        if (argc == 2)
        {
            WPRINT_APP_INFO(("Starting registrar in PBC mode\n"));
            p2p_workspace.p2p_wps_mode = WPS_PBC_MODE;
            p2p_workspace.p2p_wps_device_password_id = WPS_PUSH_BTN_DEVICEPWDID;
            memcpy( p2p_workspace.p2p_wps_pin, pbc_password, 9 );
            besl_p2p_start_registrar();
        }
        else
        {
            return ERR_UNKNOWN;
        }
    }
    else
    {
        return ERR_UNKNOWN;
    }

    return 0;
}


/*!
 ******************************************************************************
 * Stop P2P WPS Registrar
 *
 * @return  0 for success, otherwise error
 */
int p2p_registrar_stop( int argc, char* argv[] )
{
    if ( wwd_wifi_is_ready_to_transceive( WWD_P2P_INTERFACE ) != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("P2P registrar stop: group owner is not up\n"));
    }
    else if ( p2p_workspace.p2p_wps_agent == NULL )
    {
        WPRINT_APP_INFO(("P2P registrar not yet initialized\n"));
    }
    else if ( p2p_workspace.p2p_wps_agent->wps_result != WPS_NOT_STARTED )
    {
        if ( besl_wps_abort( p2p_workspace.p2p_wps_agent ) == BESL_SUCCESS )
        {
            WPRINT_APP_INFO(("P2P registrar stopped\n"));
        }
    }
    else
    {
        WPRINT_APP_INFO(("P2P registrar not running\n"));
    }
    return ERR_CMD_OK;
}


/*!
 ******************************************************************************
 * Disassociate from a P2P group owner
 *
 * @return  0 for success, otherwise error
 */

int p2p_leave( int argc, char* argv[] )
{
    if ( ( wwd_wifi_is_ready_to_transceive( WWD_P2P_INTERFACE ) != WWD_SUCCESS ) || ( besl_p2p_group_owner_is_up( ) == WICED_TRUE ) )
    {
        WPRINT_APP_INFO(( "P2P client interface is not up\n" ));
        return WICED_ERROR;
    }

    if ( wwd_wifi_leave( WWD_P2P_INTERFACE ) != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(( "Error when disassociating from Group Owner\n"));
    }

    if ( p2p_worker_thread_running == 1 )
    {
        wiced_rtos_delete_worker_thread( &p2p_worker_thread );
        p2p_worker_thread_running = 0;
    }

    if ( besl_p2p_deinit( &p2p_workspace ) != 0 )
    {
        WPRINT_APP_INFO(("Error when deinitializing P2P workspace\n"));
    }

    return 0;
}


/*!
 ******************************************************************************
 * P2P device connection request callback
 *
 * This callback is called as the result of an event so a message is sent to the worker thread which has enough memory to print.
 *
 */

void p2p_connection_request_callback( p2p_discovered_device_t* device )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, p2p_connection_request_handler, device );
}

/*!
 ******************************************************************************
 * P2P legacy device connection request callback
 *
 * This indicates if a non-P2P client device is sending probe requests with push button mode asserted. The callback is called as the result of an event
 * so a message is sent to the worker thread which has enough memory to print.
 *
 */

void p2p_legacy_device_connection_request_callback( p2p_legacy_device_t* device )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, p2p_legacy_device_connection_request_handler, device );
}

/*!
 ******************************************************************************
 * WPA2 client association callback
 *
 * This callback is called as the result of an event so a message is sent to the worker thread which has enough memory to print.
 * This call back occurs when a device associates using WPA2, but before the device has completed the key handshake and requested an IP address.
 *
 */

void p2p_wpa2_client_association_callback( besl_mac_t* mac )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, p2p_wpa2_client_association_handler, mac );
}

/*!
 ******************************************************************************
 * WPS enrollee association callback
 *
 * This callback is called as the result of an event so a message is sent to the worker thread which has enough memory to print.
 * This callback occurs when a device associates with the WPS IE and without the RSN IE in its association request, which implies that it is going to attempt the WPS handshake.
 *
 */

void p2p_wps_enrollee_association_callback( besl_mac_t* mac )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, p2p_wps_enrollee_association_handler, mac );
}

/*!
 ******************************************************************************
 * WPS result callback
 *
 * This callback is called as the result of an event so a message is sent to the worker thread which has enough memory to print.
 * This callback occurs when a device associates with the WPS IE and without the RSN IE in its association request, which implies that it is going to attempt the WPS handshake.
 *
 */

void p2p_wps_result_callback( wps_result_t* wps_result )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, p2p_wps_result_handler, wps_result );
}

/*!
 ******************************************************************************
 * P2P group formation result callback
 *
 */

void p2p_group_formation_result_callback( void* not_used )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, p2p_group_formation_result_handler, NULL );
}

/*!
 ******************************************************************************
 * P2P device disassociation callback
 *
 * This callback is called as the result of an event so a message is sent to the worker thread which has enough memory to print.
 *
 */

void p2p_device_disassociation_callback( besl_mac_t* device )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, p2p_device_disassociation_handler, device );
}

/*!
 ******************************************************************************
 * P2P legacy device disassociation callback
 *
 * This callback is called as the result of an event so a message is sent to the worker thread which has enough memory to print.
 *
 */

void p2p_legacy_device_disassociation_callback( besl_mac_t* mac )
{
    wiced_rtos_send_asynchronous_event( &p2p_worker_thread, p2p_legacy_device_disassociation_handler, mac );
}

/*!
 ******************************************************************************
 * Create P2P worker thread to handle connection request callbacks
 *
 * @return  0 for success, otherwise error
 */

static wiced_result_t create_p2p_worker_thread( void )
{
    WPRINT_APP_INFO( ( "Creating p2p app worker thread\n" ) );
    memset( &p2p_worker_thread, 0, sizeof( wiced_worker_thread_t) );
    if ( wiced_rtos_create_worker_thread( &p2p_worker_thread, WICED_DEFAULT_WORKER_PRIORITY, 4096, 5 ) != WICED_SUCCESS )  // XXX reduce mem size later
    {
        WPRINT_APP_INFO(("Error when creating P2P worker thread\n"));
        return WICED_ERROR;
    }
    p2p_worker_thread_running = 1;
    return WICED_SUCCESS;
}


/*!
 ******************************************************************************
 * P2P connection request handler
 * This is an example only. The console app does not easily allow keyboard entry from more than one thread so the handler advises which P2P
 * related command should be entered by the user.
 *
 * @return  0 for success, otherwise error
 */

static wiced_result_t p2p_connection_request_handler( void* connecting_device )
{
    p2p_discovered_device_t* device = connecting_device;

    if ( device == NULL )
    {
        WPRINT_APP_INFO( ( "Connection Request from NULL device\n" ) );
        return WICED_SUCCESS;
    }

    /* Handle Negotiation Requests and Provision Requests */
    if ( device->status == P2P_DEVICE_REQUESTED_TO_FORM_GROUP )
    {
        /* Ignore request if the configuration method is not allowed */
        if ( ! ( p2p_workspace.allowed_configuration_methods & device->preferred_config_method ) )
        {
            return WICED_SUCCESS;
        }

        WPRINT_APP_INFO( ( "Connection Request from:" ) );
        WPRINT_APP_INFO( ( "  %02X:%02X:%02X:%02X:%02X:%02X    ",
            device->p2p_device_address.octet[0],
            device->p2p_device_address.octet[1],
            device->p2p_device_address.octet[2],
            device->p2p_device_address.octet[3],
            device->p2p_device_address.octet[4],
            device->p2p_device_address.octet[5] ) );
        WPRINT_APP_INFO( ( " %s", device->device_name ) );
        WPRINT_APP_INFO( ( "\n") );

        if ( p2p_workspace.p2p_wps_agent != NULL )
        {
            if ( p2p_workspace.p2p_wps_agent->wps_result == WPS_IN_PROGRESS )
            {
                WPRINT_APP_INFO( ("WPS already running. To cancel use the following command: 'p2p_registrar_stop'\n") );
                return WICED_SUCCESS;
            }
        }

        /* The group owner gets the WPS configuration method in a Provision Discovery Request but the STA may get it from a Negotiation Request or Provision Request. */
        if ( p2p_workspace.group_owner_is_up == 1 )
        {
            if ( device->preferred_config_method == PUSH_BUTTON )
            {
                WPRINT_APP_INFO( ( "Start the WPS registrar using the following command: 'p2p_registrar_start pbc'\n" ) );
            }
            else if ( device->preferred_config_method == DISPLAY )
            {
                WPRINT_APP_INFO( ( "Start the WPS registrar using the following command: 'p2p_registrar_start pin'\n" ) );
            }
            else if ( device->preferred_config_method == KEYPAD )
            {
                WPRINT_APP_INFO( ( "Read the PIN from the other device and enter it using the following command: 'p2p_registrar_start pin <8 digit PIN>'\n" ) );
                if ( test_client_join_mode == 1 )
                {
                    WPRINT_APP_INFO( ( "Test mode client\n" ) );
                    p2p_workspace.p2p_wps_mode = WPS_PIN_MODE;
                    strncpy( p2p_workspace.p2p_wps_pin, test_pin_string, 8 );
                    p2p_workspace.p2p_wps_pin[8] = 0;
                    besl_p2p_start_registrar();
                }
            }
            else
            {
                WPRINT_APP_INFO( ( "Unknown WPS configuration method %u\n", (unsigned int)device->preferred_config_method));
            }
        }
        else
        {
            int i = 0;
            wiced_bool_t found = WICED_FALSE;

            while ( ( i < P2P_MAX_DISCOVERED_DEVICES ) && ( found == WICED_FALSE ))
            {
                if ( memcmp( &p2p_workspace.discovered_devices[i].p2p_device_address, &device->p2p_device_address, sizeof(besl_mac_t) ) == 0 )
                {
                    found = WICED_TRUE;
                }
                else
                {
                    i++;
                }
            }

            if ( found == WICED_TRUE )
            {
                connecting_device_index = i;
                if ( device->p2p_wps_device_password_id == WPS_PUSH_BTN_DEVICEPWDID )
                {
                    WPRINT_APP_INFO( ( "Use the following command to accept the connection: 'p2p_connect pbc'\n" ) );
                }
                else if ( device->p2p_wps_device_password_id == WPS_USER_SPEC_DEVICEPWDID )
                {
                    WPRINT_APP_INFO( ( "Use the following command to accept the connection: 'p2p_connect pin'\n" ) );
                }
                else if ( device->p2p_wps_device_password_id == WPS_DEVICEPWDID_REG_SPEC )
                {
                    WPRINT_APP_INFO( ( "Read the PIN from the other device and use the following command to accept the connection: 'p2p_connect pin <8 digit PIN>'\n" ) );
                }
                else
                {
                    WPRINT_APP_INFO(("Unknown WPS device password id %u\n", (unsigned int)device->p2p_wps_device_password_id ) );
                }
            }
        }
    }
    /* Handle Invitation Requests */
    else if ( device->status == P2P_DEVICE_INVITATION_REQUEST )
    {
        WPRINT_APP_INFO( ( "Invitation Request from:" ) );
        WPRINT_APP_INFO( ( "  %02X:%02X:%02X:%02X:%02X:%02X    ",
            device->p2p_device_address.octet[0],
            device->p2p_device_address.octet[1],
            device->p2p_device_address.octet[2],
            device->p2p_device_address.octet[3],
            device->p2p_device_address.octet[4],
            device->p2p_device_address.octet[5] ) );
        WPRINT_APP_INFO( ( " %s", device->device_name ) );
        WPRINT_APP_INFO( ( "\n") );
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

static wiced_result_t p2p_legacy_device_connection_request_handler( void* device )
{
    p2p_legacy_device_t* legacy_device = device;

    WPRINT_APP_INFO( ( "\nConnection Request from WPS legacy device using push button mode:" ) );
    WPRINT_APP_INFO( ( "  %02X:%02X:%02X:%02X:%02X:%02X    ",
        legacy_device->mac_address.octet[0],
        legacy_device->mac_address.octet[1],
        legacy_device->mac_address.octet[2],
        legacy_device->mac_address.octet[3],
        legacy_device->mac_address.octet[4],
        legacy_device->mac_address.octet[5] ) );
    WPRINT_APP_INFO( ( "%s\n", legacy_device->device_name ) );

    if ( p2p_workspace.p2p_wps_agent != NULL )
    {
        if ( p2p_workspace.p2p_wps_agent->wps_result == WPS_IN_PROGRESS )
        {
            WPRINT_APP_INFO( ("WPS running. To cancel use the following command: 'p2p_registrar_stop'\n") );
            return WICED_SUCCESS;
        }
        else
        {
            WPRINT_APP_INFO( ( "Start the WPS registrar using the following command: 'p2p_registrar_start pbc'\n" ) );
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

static wiced_result_t p2p_wpa2_client_association_handler( void* mac )
{
    wiced_mac_t mac_address;

    if ( mac != NULL )
    {
        memcpy(&mac_address, mac, sizeof(wiced_mac_t));

        WPRINT_APP_INFO( ( "\nWPA2 client associated:" ) );
        WPRINT_APP_INFO( ( "  %02X:%02X:%02X:%02X:%02X:%02X\n",
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

static wiced_result_t p2p_wps_enrollee_association_handler( void* mac )
{
    wiced_mac_t mac_address;

    if ( mac != NULL )
    {
        memcpy(&mac_address, mac, sizeof(wiced_mac_t));

        WPRINT_APP_INFO( ( "\nWPS enrollee associated:" ) );
        WPRINT_APP_INFO( ( "  %02X:%02X:%02X:%02X:%02X:%02X\n",
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

static wiced_result_t p2p_wps_result_handler( void* result )
{
    wps_result_t* wps_result = (wps_result_t*)result;

    /* Print result (if enabled) */
    if ( *wps_result == WPS_COMPLETE )
    {
        WPRINT_APP_INFO(( "p2p_wps_result_handler: WPS completed successfully\n" ));
    }
    else if ( *wps_result == WPS_PBC_OVERLAP )
    {
        WPRINT_APP_INFO(( "p2p_wps_result_handler: PBC overlap detected - wait and try again\n" ));
    }
    else if ( *wps_result == WPS_ABORTED )
    {
        WPRINT_APP_INFO(( "p2p_wps_result_handler: WPS aborted\n" ));
    }
    else
    {
        WPRINT_APP_INFO(( "p2p_wps_result_handler: WPS timed out\n" ));
    }

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * P2P device disassociation handler
 *
 * @return  0 for success
 */

static wiced_result_t p2p_device_disassociation_handler( void* mac )
{
    wiced_mac_t mac_address;

    if ( mac != NULL )
    {
        memcpy(&mac_address, mac, sizeof(wiced_mac_t));

        WPRINT_APP_INFO( ( "\nP2P device disassociated:" ) );
        WPRINT_APP_INFO( ( "  %02X:%02X:%02X:%02X:%02X:%02X\n",
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

static wiced_result_t p2p_legacy_device_disassociation_handler( void* mac )
{
    wiced_mac_t mac_address;

    if ( mac != NULL )
    {
        memcpy(&mac_address, mac, sizeof(wiced_mac_t));

        WPRINT_APP_INFO( ( "\nLegacy device disassociated:" ) );
        WPRINT_APP_INFO( ( "  %02X:%02X:%02X:%02X:%02X:%02X\n",
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
 * Set P2P group owner intent
 *
 * @return  0 for success, otherwise error
 */

int p2p_set_go_intent( int argc, char* argv[] )
{
    uint8_t intent;

    if ( argc == 2 )
    {
        intent = (uint8_t)( atoi( argv[1] ) );

        if ( intent <= 15 )
        {
            device_details.group_owner_intent = intent;
        }
        else
        {
            WPRINT_APP_INFO(( "Intent value must be between 0 and 15 (15 = must be group owner)\n"));
        }
    }
    else
    {
        WPRINT_APP_INFO(( "Wrong number of arguments\n"));
        return -1;
    }

    return 0;
}

/*!
 ******************************************************************************
 * Set P2P listen channel to one from the set of social channels { 1, 6, or 11 }
 *
 * @return  0 for success, otherwise error
 */

int p2p_set_listen_channel( int argc, char* argv[] )
{
    uint8_t channel;

    if ( argc == 2 )
    {
        channel = (uint8_t)( atoi( argv[1] ) );

        if ( ( channel == 1 ) || ( channel == 6 ) || ( channel == 11 ) )
        {
            device_details.listen_channel.channel = channel;
        }
        else
        {
            WPRINT_APP_INFO(( "Listen channel must be 1, 6 or 11\n"));
        }
    }
    else
    {
        WPRINT_APP_INFO(( "Wrong number of arguments\n"));
        return -1;
    }

    return 0;
}

/*!
 ******************************************************************************
 * Set P2P operating channel
 *
 * @return  0 for success, otherwise error
 */

int p2p_set_operating_channel( int argc, char* argv[] )
{
    uint8_t channel;

    if ( argc == 2 )
    {
        channel = (uint8_t)( atoi( argv[1] ) );

        if ( ( channel > 0 ) && ( channel <= 11 ) )
        {
            device_details.operating_channel.channel = channel;
        }
        else
        {
            WPRINT_APP_INFO(( "Operating channel must be in the range 1..11\n"));
        }
    }
    else
    {
        WPRINT_APP_INFO(( "Wrong number of arguments\n"));
        return -1;
    }

    return 0;
}

/*!
 ******************************************************************************
 * Set P2P Group Owner allow or disallow support for PBC mode
 *
 * @return  0 for success, otherwise error
 */

int p2p_set_go_pbc_mode_support( int argc, char* argv[] )
{
    uint8_t allow_pbc_mode;

    if ( argc == 2 )
    {
        allow_pbc_mode = (uint8_t)( atoi( argv[1] ) );

        if ( allow_pbc_mode == 1 )
        {
            device_details.wps_device_details.config_methods |= PUSH_BUTTON;
        }
        else
        {
            device_details.wps_device_details.config_methods &= (~PUSH_BUTTON);
        }
        if ( device_details.wps_device_details.config_methods & PUSH_BUTTON )
        {
            WPRINT_APP_INFO(( "Push button allowed\n" ));
        }
        else
        {
            WPRINT_APP_INFO(( "Push button disallowed\n" ));
        }
    }
    else
    {
        WPRINT_APP_INFO(( "Wrong number of arguments\n"));
        return -1;
    }

    return 0;
}


/*!
 ******************************************************************************
 * P2P group formation result handler
 */

static wiced_result_t p2p_group_formation_result_handler( void* not_used )
{
#ifdef WICED_DCT_INCLUDE_P2P_CONFIG
    platform_dct_p2p_config_t*  dct_p2p_group_owner_config;
#endif
    if ( wiced_network_is_ip_up( WICED_P2P_INTERFACE ) == WICED_TRUE )
    {
        if ( p2p_workspace.i_am_group_owner != WICED_TRUE )
        {
            WPRINT_APP_INFO( ( "P2P result: client\n" ) );
        }
        else
        {
            if ( p2p_workspace.form_persistent_group == 1 )
            {
                p2p_workspace.persistent_group.details_valid = CONFIG_VALIDITY_VALUE;

#ifdef WICED_DCT_INCLUDE_P2P_CONFIG
                wiced_dct_read_lock( (void**) &dct_p2p_group_owner_config, WICED_TRUE, DCT_P2P_CONFIG_SECTION, 0, sizeof(platform_dct_p2p_config_t) );
                memcpy( &dct_p2p_group_owner_config->p2p_group_owner_settings, &p2p_workspace.persistent_group, sizeof(wiced_config_soft_ap_t) );
                wiced_dct_write( (const void*) dct_p2p_group_owner_config, DCT_P2P_CONFIG_SECTION, 0, sizeof(platform_dct_p2p_config_t) );
                wiced_dct_read_unlock( (void*) dct_p2p_group_owner_config, WICED_TRUE );
#endif
            }
            //WPRINT_APP_INFO( ( "P2P result: group owner\n") );
        }
    }
    else
    {
        WPRINT_APP_INFO( ( "P2P result: failed to form group\n" ) );
    }

    return WICED_SUCCESS;
}


static int check_mac_locally_administered_bit( void )
{
    wiced_mac_t   mac;

    wiced_wifi_get_mac_address( &mac );
    if ( mac.octet[0] & MAC_ADDRESS_LOCALLY_ADMINISTERED_BIT )
    {
        WPRINT_APP_INFO(( "Error: MAC address is locally administered. Modify MAC address in generated_mac_address.txt file to be globally\n" ));
        WPRINT_APP_INFO(( "administered, e.g. if first byte of MAC address is 0x02 change it to 0x00. If testing multiple Wiced p2p devices\n" ));
        WPRINT_APP_INFO(( "ensure that they have unique MAC addresses.\n" ));
        return -1;
    }
    return 0;
}

int p2p_enable_ops( int argc, char* argv[] )
{
    wl_p2p_ops_t ops;

    if ( (wwd_wifi_is_ready_to_transceive(WWD_P2P_INTERFACE) != WWD_SUCCESS) ||
        (besl_p2p_group_owner_is_up() != WICED_TRUE) )
    {
        WPRINT_APP_INFO(( "Unable to enable p2p_ops as it's not a P2P GO\n" ));
        return -1;
    }

    if ( argc > 2 )
    {
        /* Invalid number of arguments passed by the User */
        WPRINT_APP_INFO(( "Please review the arguments for this command.\n" ));
        return ( -1 );
    }

    if ( argc == 1 )
    {
        /* User has not provided any arguments, So move ahead with default values */
        ops.ctw = 10;
    }
    /* Arg 2 is used to set CTWindow for OPS */
    else
    {
        ops.ctw = atoi(argv[1]);
    }

    ops.ops = 1;

    WPRINT_APP_INFO(( "Enabling P2P OPS with default CTWindow = %d\n", ops.ctw ));

    return p2p_set_ops (&ops);
}

int p2p_disable_ops( int argc, char* argv[] )
{
    wl_p2p_ops_t ops;

    if ( (wwd_wifi_is_ready_to_transceive(WWD_P2P_INTERFACE) != WWD_SUCCESS) ||
        (besl_p2p_group_owner_is_up() != WICED_TRUE) )
    {
        WPRINT_APP_INFO(( "Unable to disable p2p_ops as it's not a P2P GO\n" ));
        return -1;
    }

    if ( argc > 1 )
    {
        /* Invalind number of arguments passed by the User */
        WPRINT_APP_INFO(( "Please review the arguments for this command.\n" ));
        return ( -1 );
    }

    ops.ops = 0;

    return p2p_set_ops (&ops);
}

int p2p_enable_noa( int argc, char* argv[] )
{
    wl_p2p_sched_t noa;

    if ( argc > 4 || argc == 2 || argc == 3 )
    {
        WPRINT_APP_INFO(( "Please review the arguments to this command\n" ));
        return ( -1 );
    }

    if ( (wwd_wifi_is_ready_to_transceive(WWD_P2P_INTERFACE) != WWD_SUCCESS) ||
        (besl_p2p_group_owner_is_up() != WICED_TRUE) )
    {
        WPRINT_APP_INFO(( "Unable to set p2p_noa as it's not a P2P GO\n" ));
        return -1;
    }

    noa.type = WL_P2P_SCHED_TYPE_ABS;
    noa.action = WL_P2P_SCHED_ACTION_DOZE;
    noa.option = WL_P2P_SCHED_OPTION_NORMAL;
    noa.desc[0].start = 0;

    if ( argc == 1 )
    {
        /* User has not provided any arguments, So move ahead with default values */
        noa.desc[0].interval = 102400;
        noa.desc[0].duration = 80000;
        noa.desc[0].count = 255;
    }
    /* argc == 4. Other 3 arguments are used to set interval, Duration and Count */
    else
    {
        noa.desc[0].interval = atoi(argv[1]);
        noa.desc[0].duration = atoi(argv[2]);
        noa.desc[0].count = atoi(argv[3]);
    }

    WPRINT_APP_INFO(( "Enabling P2P NoA with interval = %d, Duration = %d and Count = %d\n",
        (int)noa.desc[0].interval, (int)noa.desc[0].duration, (int)noa.desc[0].count ));

    return p2p_set_noa (&noa);
}

int p2p_disable_noa( int argc, char* argv[] )
{
    wl_p2p_sched_t noa;

    if ( argc > 1 )
    {
        WPRINT_APP_INFO(( "This command does not take any arguments\n" ));
        return ( -1 );
    }

    if ( (wwd_wifi_is_ready_to_transceive(WWD_P2P_INTERFACE) != WWD_SUCCESS) ||
        (besl_p2p_group_owner_is_up() != WICED_TRUE) )
    {
        WPRINT_APP_INFO(( "Unable to set p2p_noa as it's not a P2P GO\n" ));
        return -1;
    }

    /* It is important to nullify the wl_p2p_sched_t struct */
    memset( &noa, 0, sizeof(wl_p2p_sched_t) );

    /* Set P2P action to -1 to reset the scheduled NoA */
    noa.action = WL_P2P_SCHED_ACTION_RESET;

    return p2p_set_noa (&noa);
}

