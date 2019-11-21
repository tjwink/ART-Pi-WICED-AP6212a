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
 * Websocket Client Application
 *
 * This application snippet demonstrates how to use a simple Websocket Client protocol
 *
 * Features demonstrated
 *  - Wi-Fi client mode
 *  - DNS lookup
 *  - Secure/unsecure websocket( wiced_websocket_secure_connect/wiced_websocket_connect) client connection
 *
 * Application Instructions
 * 1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *    in the wifi_config_dct.h header file to match your Wi-Fi access point
 * 2. Connect a PC terminal to the serial port of the WICED Eval board,
 *    then build and download the application as described in the WICED
 *    Quick Start Guide
 *
 * After the download completes, the application :
 *  - Connects to the Wi-Fi network specified
 *  - Resolves the websocket server ip address
 *  - Connects to the websocket server and sends two test frames
 *  --1. character set 0-9
 *  --2. websocket ping request/response (not icmp)
 *
 *  The snippet also shows how the websocket API can be used.
 *  i.e. wiced_websocket_t API has the following callbacks:
 *   on open callback       : called once websocket handshake is complete
 *   on close callback      : called on socket close
 *   on error callback      : called on websocket error defined by wiced_websocket_error_t
 *   on message callback    : called whenever a new frame arrives on the websocket
 *
 *   Also note when sending frames through the wiced_websocket_frame_t structure,
 *   the application must define if this is last frame in message or not
 *   Frames are currently limited to 1024 bytes
 *
 *Limitations:
 *   Does not yet handle fragmentation across multiple packets
 *
 *   Note: The default websocket server used with this snippet is from echo.websocket.org
 *   This is a publicly available server, and is not managed by Cypress
 */

#include <stdlib.h>
#include "wiced.h"
#include "websocket.h"
#include "wiced_tls.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define WEBSOCKET_SNIPPET_VERIFY(x)  {wiced_result_t res = (x); if (res != WICED_SUCCESS){ goto exit;}}
#define FINAL_FRAME WICED_TRUE
#define USE_WEBSOCKET_SECURE_CLIENT

/******************************************************
 *                    Constants
 ******************************************************/
#define BUFFER_LENGTH     (2048)
#define PING_MESSAGE      "A tiny, cute Websocket Client!"

#define WEBSOCKET_SECURE_SERVER_STD_PORT  (443)
#define WEBSOCKET_SERVER_STD_PORT         (80)

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

wiced_result_t on_open_websocket_callback   ( void* websocket );
wiced_result_t on_close_websocket_callback  ( void* websocket );
wiced_result_t on_error_websocket_callback  ( void* websocket );
static wiced_result_t on_message_websocket_callback( void* websocket, uint8_t* data, uint32_t length, wiced_websocket_frame_type_t type, wiced_websocket_frame_flags_t flags );

/******************************************************
 *               Variables Definitions
 ******************************************************/

static uint8_t rx_buffer[ BUFFER_LENGTH ];
static uint8_t tx_buffer[ BUFFER_LENGTH ];

static char greetings_client[] = { 'H','e','l','l','o',' ','S','e','r','v','e','r','!','\0' };

static wiced_semaphore_t received_message_event;

#ifdef USE_WEBSOCKET_SECURE_CLIENT
/* Godaddy Root CA certificate(for connecting to echo.websocket.org) + securedemo.wiced.cypress.com certificate */
static const char httpbin_root_ca_certificate[] =
    "-----BEGIN CERTIFICATE-----\n"                                      \
    "MIIEfTCCA2WgAwIBAgIDG+cVMA0GCSqGSIb3DQEBCwUAMGMxCzAJBgNVBAYTAlVT\n" \
    "MSEwHwYDVQQKExhUaGUgR28gRGFkZHkgR3JvdXAsIEluYy4xMTAvBgNVBAsTKEdv\n" \
    "IERhZGR5IENsYXNzIDIgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMTQwMTAx\n" \
    "MDcwMDAwWhcNMzEwNTMwMDcwMDAwWjCBgzELMAkGA1UEBhMCVVMxEDAOBgNVBAgT\n" \
    "B0FyaXpvbmExEzARBgNVBAcTClNjb3R0c2RhbGUxGjAYBgNVBAoTEUdvRGFkZHku\n" \
    "Y29tLCBJbmMuMTEwLwYDVQQDEyhHbyBEYWRkeSBSb290IENlcnRpZmljYXRlIEF1\n" \
    "dGhvcml0eSAtIEcyMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAv3Fi\n" \
    "CPH6WTT3G8kYo/eASVjpIoMTpsUgQwE7hPHmhUmfJ+r2hBtOoLTbcJjHMgGxBT4H\n" \
    "Tu70+k8vWTAi56sZVmvigAf88xZ1gDlRe+X5NbZ0TqmNghPktj+pA4P6or6KFWp/\n" \
    "3gvDthkUBcrqw6gElDtGfDIN8wBmIsiNaW02jBEYt9OyHGC0OPoCjM7T3UYH3go+\n" \
    "6118yHz7sCtTpJJiaVElBWEaRIGMLKlDliPfrDqBmg4pxRyp6V0etp6eMAo5zvGI\n" \
    "gPtLXcwy7IViQyU0AlYnAZG0O3AqP26x6JyIAX2f1PnbU21gnb8s51iruF9G/M7E\n" \
    "GwM8CetJMVxpRrPgRwIDAQABo4IBFzCCARMwDwYDVR0TAQH/BAUwAwEB/zAOBgNV\n" \
    "HQ8BAf8EBAMCAQYwHQYDVR0OBBYEFDqahQcQZyi27/a9BUFuIMGU2g/eMB8GA1Ud\n" \
    "IwQYMBaAFNLEsNKR1EwRcbNhyz2h/t2oatTjMDQGCCsGAQUFBwEBBCgwJjAkBggr\n" \
    "BgEFBQcwAYYYaHR0cDovL29jc3AuZ29kYWRkeS5jb20vMDIGA1UdHwQrMCkwJ6Al\n" \
    "oCOGIWh0dHA6Ly9jcmwuZ29kYWRkeS5jb20vZ2Ryb290LmNybDBGBgNVHSAEPzA9\n" \
    "MDsGBFUdIAAwMzAxBggrBgEFBQcCARYlaHR0cHM6Ly9jZXJ0cy5nb2RhZGR5LmNv\n" \
    "bS9yZXBvc2l0b3J5LzANBgkqhkiG9w0BAQsFAAOCAQEAWQtTvZKGEacke+1bMc8d\n" \
    "H2xwxbhuvk679r6XUOEwf7ooXGKUwuN+M/f7QnaF25UcjCJYdQkMiGVnOQoWCcWg\n" \
    "OJekxSOTP7QYpgEGRJHjp2kntFolfzq3Ms3dhP8qOCkzpN1nsoX+oYggHFCJyNwq\n" \
    "9kIDN0zmiN/VryTyscPfzLXs4Jlet0lUIDyUGAzHHFIYSaRt4bNYC8nY7NmuHDKO\n" \
    "KHAN4v6mF56ED71XcLNa6R+ghlO773z/aQvgSMO3kwvIClTErF0UZzdsyqUvMQg3\n" \
    "qm5vjLyb4lddJIGvl5echK1srDdMZvNhkREg5L4wn3qkKQmw4TRfZHcYQFHfjDCm\n" \
    "rw==\n"                                                             \
    "-----END CERTIFICATE-----\n"                                        \
    "-----BEGIN CERTIFICATE-----\n" \
    "MIICTzCCAbgCCQDpIVfd7XLcTTANBgkqhkiG9w0BAQsFADBsMQswCQYDVQQGEwJJ\n" \
    "TjETMBEGA1UECAwKU29tZS1TdGF0ZTEhMB8GA1UECgwYSW50ZXJuZXQgV2lkZ2l0\n" \
    "cyBQdHkgTHRkMSUwIwYDVQQDDBxzZWN1cmVkZW1vLndpY2VkLmN5cHJlc3MuY29t\n" \
    "MB4XDTE3MDcxMjEwMDQzMVoXDTIzMDEwMjEwMDQzMVowbDELMAkGA1UEBhMCSU4x\n" \
    "EzARBgNVBAgMClNvbWUtU3RhdGUxITAfBgNVBAoMGEludGVybmV0IFdpZGdpdHMg\n" \
    "UHR5IEx0ZDElMCMGA1UEAwwcc2VjdXJlZGVtby53aWNlZC5jeXByZXNzLmNvbTCB\n" \
    "nzANBgkqhkiG9w0BAQEFAAOBjQAwgYkCgYEAwFec9pyXjZ1IIQpl4H3hL+L994iC\n" \
    "ADE31oZHS+jERINxmw12AO3CRZR5yNUmjRUP4HMFjBJWI4qNBHGhkQ1dWG45VOhl\n" \
    "GlYBT8Ch1TU+hwzqYCsjjbRTuNjU+Mrj+ykz65ZjqLvtsJATNJX2e8evQSq8saCK\n" \
    "8qwNXAgLZc6uiG0CAwEAATANBgkqhkiG9w0BAQsFAAOBgQArp7Cu/LzSHXLR9azO\n" \
    "CRCWo8NkZTJ/QbOZJpu6eGU5x10Y+SMJAUBQ5P7OQLnK5yumnCIdmA+OEFKfstym\n" \
    "a77DNL/jd/RqhJ/vvkEsBkwDmrVMX9JEzcE+MKuvLdMjb767SL9jiee5hFo1HGoI\n" \
    "xZfeZ3Y1ENfAb/gWeuENecyCAg==\n" \
    "-----END CERTIFICATE-----\n";
#endif

/* Following are the standard header fields in use
 * by the websoket protocol
 * Additional header fields may be added
 *  GET /chat HTTP/1.1
    Host: server.example.com
    Upgrade: websocket
    Connection: Upgrade
    Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==
    Origin: http://example.com
    Sec-WebSocket-Protocol: chat, superchat
    Sec-WebSocket-Version: 13
 */

static const wiced_websocket_client_url_protocol_t websocket_header =
{
   .request_uri            = "?encoding=text",
   .host                   = "echo.websocket.org", // connection to hosts with an ip-address(like internal servers) will also work.
   //.host                   = "192.168.1.49",
   .origin                 = "172.16.61.228",
   .sec_websocket_protocol =  NULL,
};

static wiced_websocket_t    websocket;
#ifdef USE_WEBSOCKET_SECURE_CLIENT
static wiced_tls_identity_t  tls_identity;
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/

/* To test if the client can send larger payloads(more than MTU size).
 * If tested with an echo websocket server, it will receive the same payload
 * testing both rx/tx paths
 */
static wiced_result_t send_large_buffer_data( wiced_websocket_t* websocket )
{
    memset(tx_buffer, 'A', sizeof(tx_buffer) );
    WICED_VERIFY( wiced_websocket_send(websocket, tx_buffer, sizeof(tx_buffer), WEBSOCKET_TEXT_FRAME, WEBSOCKET_FRAME_FLAG_UNFRAGMENTED) );
    return WICED_SUCCESS;
}

static void print_binary_data(uint8_t* data, uint32_t length)
{
    int i = 0;
    int offset = 0;

    printf("\n===== Start of Data Blob =====\n");
    for ( ; i < length/4; i++, offset = i*4 )
    {
        printf("\r\n");
        printf("%x\t%x\t%x\t%x\t", data[offset], data[offset + 1], data[offset + 2], data[offset + 3] );
    }

    printf("\r\n");
    if( length % 4 )
    {
        for( i = 0; i < length%4; i++ )
        {
            printf("%x\t", data[offset + i] );
        }
    }
    printf("\r\n");

    printf("\n===== End of Data Blob =====\n");
}

void application_start( )
{

    wiced_init( );

    wiced_rtos_init_semaphore( &received_message_event );

    if ( wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "\r\nFailed to bring up network\r\n" ) );
        return;
    }

#ifdef USE_WEBSOCKET_SECURE_CLIENT
    wiced_result_t result = WICED_ERROR;
    /* Initialize the root CA certificate */
    result = wiced_tls_init_root_ca_certificates( httpbin_root_ca_certificate, strlen(httpbin_root_ca_certificate) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "Error: Root CA certificate failed to initialize: %u\n", result) );
        return;
    }
#endif

    /* Initialise the websocket */
    wiced_websocket_initialise( &websocket, rx_buffer, BUFFER_LENGTH );

    /*register websocket callbacks */
    wiced_websocket_register_callbacks( &websocket, on_open_websocket_callback, on_close_websocket_callback, on_message_websocket_callback, on_error_websocket_callback  );

    WPRINT_APP_INFO( ( "\r\nConnecting to server...\r\n\r\n" ) );

    /* Establish a websocket connection with the server */
    WPRINT_APP_INFO(("Trying to connect\n"));
#ifdef USE_WEBSOCKET_SECURE_CLIENT
    WEBSOCKET_SNIPPET_VERIFY( wiced_websocket_secure_connect( &websocket, &websocket_header, &tls_identity, (uint16_t)WEBSOCKET_SECURE_SERVER_STD_PORT, WICED_STA_INTERFACE ) );
#else
    WEBSOCKET_SNIPPET_VERIFY( wiced_websocket_connect( &websocket, &websocket_header, (uint16_t)WEBSOCKET_SERVER_STD_PORT, WICED_STA_INTERFACE ) );
#endif

    WPRINT_APP_INFO( ( "\r\nConnected to server \r\n\r\n" ) );

    WPRINT_APP_INFO( ( "Sending Greetings from Client to Server...\r\n\r\n" ) );

    /* Send the text data to the server */
    WEBSOCKET_SNIPPET_VERIFY( wiced_websocket_send( &websocket, (uint8_t*)greetings_client, strlen(greetings_client), WEBSOCKET_TEXT_FRAME, WEBSOCKET_FRAME_FLAG_UNFRAGMENTED ) );
    wiced_rtos_get_semaphore( &received_message_event, WICED_WAIT_FOREVER );

    /* demonstrates different fragmented/continued 'frame' types with Text payload */
    wiced_websocket_send( &websocket, (uint8_t*)"Hi, ", strlen("Hi, "),  WEBSOCKET_TEXT_FRAME, WEBSOCKET_FRAME_FLAG_FRAGMENTED_FIRST );
    wiced_rtos_get_semaphore( &received_message_event, WICED_WAIT_FOREVER );

    wiced_websocket_send( &websocket, (uint8_t*)"I'm a Wiced ", strlen("I'm a Wiced "), WEBSOCKET_TEXT_FRAME, WEBSOCKET_FRAME_FLAG_CONTINUED );
    wiced_rtos_get_semaphore( &received_message_event, WICED_WAIT_FOREVER );

    wiced_websocket_send( &websocket, (uint8_t*)"Websocket Client!!" , strlen("Websocket Client!!"), WEBSOCKET_TEXT_FRAME, WEBSOCKET_FRAME_FLAG_FRAGMENTED_FINAL );
    wiced_rtos_get_semaphore( &received_message_event, WICED_WAIT_FOREVER );

    WPRINT_APP_INFO( ( "Sending ping frame. Expecting PONG response...\r\n\r\n" ) );

    /* send PING frame */
    char* ping_message = PING_MESSAGE;

    WEBSOCKET_SNIPPET_VERIFY( wiced_websocket_send( &websocket, (uint8_t*)ping_message, strlen(ping_message), WEBSOCKET_PING, WEBSOCKET_FRAME_FLAG_UNFRAGMENTED ) );

    wiced_rtos_get_semaphore( &received_message_event, WICED_WAIT_FOREVER );

    WPRINT_APP_INFO(("Sending Payload larger than MTU...\n"));
    send_large_buffer_data(&websocket);
    wiced_rtos_get_semaphore( &received_message_event, WICED_WAIT_FOREVER );

    WPRINT_APP_INFO( ( "\r\nWebsocket client demo complete. Closing connection.\r\n") );

exit:

    wiced_websocket_close( &websocket, WEBSOCKET_CLOSE_STATUS_CODE_NORMAL, "Closing session\r\n" );

    wiced_websocket_uninitialise( &websocket );

}

wiced_result_t on_open_websocket_callback( void* websocket )
{
    UNUSED_PARAMETER( websocket );

    WPRINT_APP_INFO( ( "\r\nConnection open received\r\n " ) );

    return WICED_SUCCESS;
}

wiced_result_t on_close_websocket_callback( void* websocket )
{
    UNUSED_PARAMETER( websocket );

    WPRINT_APP_INFO( ( "\r\nConnection close received\r\n " ) );

    return WICED_SUCCESS;
}

static wiced_result_t on_message_websocket_callback( void* socket, uint8_t* data, uint32_t length, wiced_websocket_frame_type_t type, wiced_websocket_frame_flags_t flags )
{
    wiced_websocket_t* websocket = ( wiced_websocket_t* )socket;

    WPRINT_APP_INFO( ( "[App] Message received @websocket: %p\n", websocket) );

    switch( type )
    {
        case WEBSOCKET_TEXT_FRAME :
        {
            WPRINT_APP_INFO( ( "\t{ TEXT" ));
            WPRINT_APP_DEBUG( (" [final? %s continuation? %s]",( (int)flags == WEBSOCKET_FRAME_FLAG_UNFRAGMENTED || (int)flags == WEBSOCKET_FRAME_FLAG_FRAGMENTED_FINAL ) ? "yes" :"no",
                ( (int) flags == WEBSOCKET_FRAME_FLAG_FRAGMENTED_FIRST || (int) flags == WEBSOCKET_FRAME_FLAG_CONTINUED || (int)flags == WEBSOCKET_FRAME_FLAG_FRAGMENTED_FINAL ) ? "yes":"no" ) );
            WPRINT_APP_INFO((" }\r\n"));

            WPRINT_APP_INFO( ( "\t%.*s\r\n", (int)length, (char*)data ) );
            break;
        }

        case WEBSOCKET_BINARY_FRAME:
        {
            WPRINT_APP_INFO( ( "\t{ BLOB" ));
            WPRINT_APP_DEBUG( (" [final? %s continuation? %s]",( (int)flags == WEBSOCKET_FRAME_FLAG_UNFRAGMENTED || (int)flags == WEBSOCKET_FRAME_FLAG_FRAGMENTED_FINAL ) ? "yes" :"no",
                ( (int) flags == WEBSOCKET_FRAME_FLAG_FRAGMENTED_FIRST || (int) flags == WEBSOCKET_FRAME_FLAG_CONTINUED || (int)flags == WEBSOCKET_FRAME_FLAG_FRAGMENTED_FINAL ) ? "yes":"no" ) );
            WPRINT_APP_INFO((" }\r\n"));

            if( data && (length != 0) )
            {
                print_binary_data(data, length);
            }

            break;
        }

        /* Handle response to Ping/Pong etc. inside the library but print the payload here, if any */
        case WEBSOCKET_PONG:
        {
            WPRINT_APP_INFO( ( "\t{ PONG } ::\r\n" ) );
            if( data )
            {
                WPRINT_APP_INFO( ( "\t%.*s\r\n", (int)length, (char*)data ) );
            }
            break;
        }

        case WEBSOCKET_PING:
        {
            WPRINT_APP_INFO( ( "\t{ PING } ::\r\n" ) );
            if( data )
            {
                WPRINT_APP_INFO( ( "\t%.*s\r\n", (int)length, (char*)data ) );
            }
            break;
        }

        case WEBSOCKET_CONNECTION_CLOSE:
        {
            WPRINT_APP_INFO( ( "\t{ CLOSE } ::\r\n" ) );
            /* TODO: print if any close message is received too */
            wiced_websocket_unregister_callbacks( (wiced_websocket_t*)websocket );
            break;
        }

        /* CONTINUATION_FRAME will be presented as either TEXT/BINARY depending on the type
         * of first fragment */
        case WEBSOCKET_CONTINUATION_FRAME:
        default:
            WPRINT_APP_INFO( ( "\t{ RESERVED/CONTINUATION } - Not handled\r\n" ) );
            break;
    }

    wiced_rtos_set_semaphore( &received_message_event );

    return WICED_SUCCESS;
}

wiced_result_t on_error_websocket_callback( void* websocket )
{
    WPRINT_APP_INFO( ( "\r\nError number received = %d\r\n", ((wiced_websocket_t*)websocket)->error_type ) );

    return WICED_SUCCESS;
}
