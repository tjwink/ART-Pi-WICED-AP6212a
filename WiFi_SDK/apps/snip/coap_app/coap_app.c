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
 * COAP server application
 *
 * This application snippet demonstrates how to react to COAP packet
 * from  network client and activate the services exported by Coap.
 *
 * Features demonstrated
 *  - Wi-Fi softAP mode
 *  - DHCP server
 *  - COAP receive / transmit
 *
 * The snippet application demonstrates CoAP functionality with secure connection ( with DTLS ) and non-secure connection ( without DTLS )
 * over UDP.
 *
 * Application Instructions to run CoAP in secure mode ( with DTLS enabled )
 * 1. Make sure flag DTLS_ENABLE is enabled
 * 2. By default snippet application uses ECDHE-ECDSA cipher suite with only client authentication enabled. To run DTLS with PSK security, Please enable SECURITY_TYPE_PSK flag below.
 *    Make sure that only MBEDTLS_TLS_PSK_WITH_AES_128_CCM_8 cipher suite in WICED/security/BESL/mbedtls_open/include/mbedtls/config.h under MBEDTLS_SSL_CIPHERSUITES is enabled, otherwise server will always try to pick the more secure cipher suite.
 * 3. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 * 4. Connect your computer using Wi-Fi to "WICED COAP SoftAP"
 *        - SoftAP credentials are defined in wifi_config_dct.h
 * Follow these instructions to set up the remote DTLS client
 * 1. Download MBEDTLS on linux machine https://tls.mbed.org/
 * 2. Please refer DTLS client snippet application present in programs/ssl/dtls_client.c in MBEDTLS.
 *    To test PSK security, Call mbedtls_ssl_conf_psk API in dtls_client.c snippet with proper
 *    identity name & key in MBEDTLS
 * 3. Make sure SERVER_PORT is set to 5684 ( DTLS port ) & SERVER_ADDR is assigned to WICED IP.
 * 4. Launch the DTLS client on the linux machine.
 *
 *   TODO : Secure CoAP [ with DTLS security ] server allows multiple clients to connect to
 *   server. Whenever client disconnects, usually an alert message is received; however due
 *   to unreliability of UDP connection, on some occasions, this alert message may not reach
 *   the server. In such scenarios, the CoAP server does not cleanup the data structure for
 *   this client (unless the client reconnects with the same port). Logic needs to be added
 *   to periodically cleanup such stale entries.
 *
 *
 * Application Instructions to run CoAP in non-secure mode ( without DTLS enabled )
 * 1. Make sure flag DTLS_ENABLE is disabled
 * 2. Connect a PC terminal to the serial port of the WICED Eval board,
 *     then build and download the application as described in the WICED
 *     Quick Start Guide
 * 3. Connect your computer using Wi-Fi to "WICED COAP SoftAP"
 *      - SoftAP credentials are defined in wifi_config_dct.h
 * 4. Install Coper plug-in on top of Firefox from below location
 *      - https://addons.mozilla.org/en-US/firefox/addon/copper-270430/
 * 5. The above plug-in helps to validate Coap server functions
 *
 *   When the Wi-Fi client (computer/Firfox cliet) joins the WICED softAP,
 *   it receives an IP address such as 192.168.0.2. To force
 *   the app to send COAP packets directly to the computer (rather than
 *   to a broadcast address), comment out the #define UDP_TARGET_IS_BROADCAST
 *   and change the target IP address to the IP address of your computer.
 *
 *   Open Firefox and try coap://192.168.0.1:5683/light and this will execute light on /off
 *   when we press PUT button along with data "1" (ON) or "0" (OFF), data must be available
 *   in the outgoing tab button on Firefox COAP plug-in
 *
 */

#include "server/coap_server.h"

/******************************************************
 *                      Macros
 ******************************************************/

/* Comment this flag out to switch to UDP */
#define DTLS_ENABLE

/* Uncomment below flag for PSK security */
//#define SECURITY_TYPE_PSK

#ifdef SECURITY_TYPE_PSK
#ifndef DTLS_ENABLE
#error PSK only makes sense with DTLS
#endif
#endif

#ifdef UDP_TARGET_IS_BROADCAST
#define UDP_TARGET_IP MAKE_IPV4_ADDRESS( 192,168,0,255 )
#else
#define UDP_TARGET_IP MAKE_IPV4_ADDRESS( 192,168,0,2 )
#endif

#ifdef DTLS_ENABLE
#define WICED_COAP_TARGET_PORT ( 5684 )
#else
#define WICED_COAP_TARGET_PORT ( 5683 )
#endif


#define CERTIFICATE_ECDSA_STRING  \
"-----BEGIN CERTIFICATE-----\r\n" \
"MIICHzCCAaWgAwIBAgIBCTAKBggqhkjOPQQDAjA+MQswCQYDVQQGEwJOTDERMA8G\r\n" \
"A1UEChMIUG9sYXJTU0wxHDAaBgNVBAMTE1BvbGFyc3NsIFRlc3QgRUMgQ0EwHhcN\r\n" \
"MTMwOTI0MTU1MjA0WhcNMjMwOTIyMTU1MjA0WjA0MQswCQYDVQQGEwJOTDERMA8G\r\n" \
"A1UEChMIUG9sYXJTU0wxEjAQBgNVBAMTCWxvY2FsaG9zdDBZMBMGByqGSM49AgEG\r\n" \
"CCqGSM49AwEHA0IABDfMVtl2CR5acj7HWS3/IG7ufPkGkXTQrRS192giWWKSTuUA\r\n" \
"2CMR/+ov0jRdXRa9iojCa3cNVc2KKg76Aci07f+jgZ0wgZowCQYDVR0TBAIwADAd\r\n" \
"BgNVHQ4EFgQUUGGlj9QH2deCAQzlZX+MY0anE74wbgYDVR0jBGcwZYAUnW0gJEkB\r\n" \
"PyvLeLUZvH4kydv7NnyhQqRAMD4xCzAJBgNVBAYTAk5MMREwDwYDVQQKEwhQb2xh\r\n" \
"clNTTDEcMBoGA1UEAxMTUG9sYXJzc2wgVGVzdCBFQyBDQYIJAMFD4n5iQ8zoMAoG\r\n" \
"CCqGSM49BAMCA2gAMGUCMQCaLFzXptui5WQN8LlO3ddh1hMxx6tzgLvT03MTVK2S\r\n" \
"C12r0Lz3ri/moSEpNZWqPjkCMCE2f53GXcYLqyfyJR078c/xNSUU5+Xxl7VZ414V\r\n" \
"fGa5kHvHARBPc8YAIVIqDvHH1Q==\r\n" \
"-----END CERTIFICATE-----\r\n"

#define PRIVATE_KEY_ECDSA_STRING  \
"-----BEGIN EC PRIVATE KEY-----\r\n" \
"MHcCAQEEIPEqEyB2AnCoPL/9U/YDHvdqXYbIogTywwyp6/UfDw6noAoGCCqGSM49\r\n" \
"AwEHoUQDQgAEN8xW2XYJHlpyPsdZLf8gbu58+QaRdNCtFLX3aCJZYpJO5QDYIxH/\r\n" \
"6i/SNF1dFr2KiMJrdw1VzYoqDvoByLTt/w==\r\n" \
"-----END EC PRIVATE KEY-----\r\n"



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
wiced_result_t handle_light( void *context, wiced_coap_server_service_t* service, wiced_coap_server_request_t *request );

/******************************************************
 *               Variables Definitions
 ******************************************************/

static const wiced_ip_setting_t device_init_ip_settings =
{ INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192,168, 0, 1) ), INITIALISER_IPV4_ADDRESS( .netmask, MAKE_IPV4_ADDRESS(255,255,255, 0) ), INITIALISER_IPV4_ADDRESS( .gateway, MAKE_IPV4_ADDRESS(192,168, 0, 1) ), };

/******************************************************
 *               Function Definitions
 ******************************************************/
static wiced_coap_server_t server;
static wiced_coap_server_service_t service[ 7 ];
#ifdef DTLS_ENABLE
static wiced_coap_security_t security;
#ifdef SECURITY_TYPE_PSK
static wiced_dtls_psk_info_t psk_info[3];
#endif
#define SECURITY &security
#else
#define SECURITY NULL
#endif

static uint8_t laststatus;

void application_start( void )
{
    wiced_coap_security_t *sec_ptr = SECURITY;

    wiced_init( );

    wiced_network_up( WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &device_init_ip_settings );

    if ( wiced_coap_server_init( &server ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO (("Failed to initialise COAP server\n"));
        return;
    }

#ifdef DTLS_ENABLE
#ifdef SECURITY_TYPE_PSK
    /* configure multiple client identity-key pair for PSK cipher suite */
    /* set security type */
    security.type = WICED_COAP_SECURITY_TYPE_PSK;

    /* set multiple PSK identity & key */
    psk_info[0].identity = "Client_identity\0";
    psk_info[0].identity_length = strlen ("Client_identity\0");
    psk_info[0].key = "secretPSK\0";
    psk_info[0].key_length = strlen ("secretPSK\0");

    psk_info[1].identity = "Client_identity1\0";
    psk_info[1].identity_length = strlen ("Client_identity1\0");
    psk_info[1].key = "secretPSK1\0";
    psk_info[1].key_length = strlen ("secretPSK1\0");

    psk_info[2].identity = "Client_identity2\0";
    psk_info[2].identity_length = strlen ("Client_identity2\0");
    psk_info[2].key = "secretPSK2\0";
    psk_info[2].key_length = strlen ("secretPSK2\0");

    security.args.psk_info = &psk_info[0];

#else
    /* configure certificate and key for ECC type */
    security.type = WICED_COAP_SECURITY_TYPE_NONPSK;

    security.args.cert_info.certificate_data = (uint8_t*) CERTIFICATE_ECDSA_STRING;
    security.args.cert_info.certificate_length = strlen ( CERTIFICATE_ECDSA_STRING );

    security.args.cert_info.private_key = PRIVATE_KEY_ECDSA_STRING;
    security.args.cert_info.key_length  = strlen( PRIVATE_KEY_ECDSA_STRING );

#endif
#endif

    /* Start COAP server */
    if ( wiced_coap_server_start( &server, WICED_AP_INTERFACE, WICED_COAP_TARGET_PORT, sec_ptr ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO (("Failed to start COAP server\n"));
        return;
    }

#ifdef SECURITY_TYPE_PSK
    wiced_dtls_add_psk_identity( &server.identity, &psk_info[1] );
    wiced_dtls_add_psk_identity( &server.identity, &psk_info[2] );
    wiced_dtls_remove_psk_identity(&server.identity, &psk_info[1]);
#endif

    if ( wiced_coap_server_add_service( &server, &service[ 0 ], "LIGHT", handle_light, WICED_COAP_CONTENTTYPE_TEXT_PLAIN ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO (("Error in adding service\n"));
        return;
    }

    if ( wiced_coap_server_add_service( &server, &service[ 1 ], "HOME/FAN", handle_light, WICED_COAP_CONTENTTYPE_TEXT_PLAIN ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO (("Error in adding service\n"));
        return;
    }

    if ( wiced_coap_server_add_service( &server, &service[ 2 ], "HOME/BULB", handle_light, WICED_COAP_CONTENTTYPE_TEXT_PLAIN ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO (("Error in adding service\n"));
        return;
    }

    if ( wiced_coap_server_add_service( &server, &service[ 3 ], "HOME/AC", handle_light, WICED_COAP_CONTENTTYPE_TEXT_PLAIN ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO (("Error in adding service\n"));
        return;
    }

    if ( wiced_coap_server_add_service( &server, &service[ 4 ], "HOME/FRIDGE", handle_light, WICED_COAP_CONTENTTYPE_TEXT_PLAIN ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO (("Error in adding service\n"));
        return;
    }

    if ( wiced_coap_server_add_service( &server, &service[ 5 ], "HOME/DOOR", handle_light, WICED_COAP_CONTENTTYPE_TEXT_PLAIN ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO (("Error in adding service\n"));
        return;
    }
}

wiced_result_t handle_light( void *context, wiced_coap_server_service_t* service, wiced_coap_server_request_t* request )
{
    static uint8_t light;
    wiced_coap_server_response_t response;
    wiced_coap_notification_type type = WICED_COAP_NOTIFICATION_TYPE_NONE;

    memset( &response, 0, sizeof( response ) );

    switch ( request->method )
    {
        case WICED_COAP_METHOD_GET:
            response.payload.data = &laststatus;
            response.payload.len = 1;
            break;

        case WICED_COAP_METHOD_POST:
            if ( request->payload.data[ 0 ] == '1' )
            {
                wiced_gpio_output_high( WICED_LED1 );
                light = 1;
            }
            else
            {
                wiced_gpio_output_low( WICED_LED1 );
                light = 0;
            }

            if ( laststatus != light )
            {
                type = WICED_COAP_NOTIFICATION_TYPE_CONFIRMABLE;
                laststatus = light;
                response.payload.data = &light;
                response.payload.len = 1;
            }
            break;

        case WICED_COAP_METHOD_DELETE:
            /* Free service only after sending response back to client */
            wiced_coap_server_delete_service( context, service );
            WPRINT_APP_INFO(( "service deleted successfully\n" ));
            break;

        default:
            WPRINT_APP_INFO(( "unknown method\n" ));
            break;
    }

    /* Send COAP response back to client */
    wiced_coap_server_send_response( context, service, request->req_handle, &response, type );

    return WICED_SUCCESS;
}
