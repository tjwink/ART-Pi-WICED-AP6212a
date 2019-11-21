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
 * www.httpbin.org Client Application
 *
 * This application snippet demonstrates how to use the WICED HTTP Client API
 * to connect to https://www.httpbin.org
 *
 * Features demonstrated
 *  - Wi-Fi client mode
 *  - DNS lookup
 *  - Secure HTTPS client connection
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
 *  - Resolves the www.httpbin.org IP address using a DNS lookup
 *  - Sends multiple GET requests to https://www.httpbin.org
 *  - Prints the results to the UART
 *
 */

#include <stdlib.h>
#include "wiced.h"
#include "wiced_tls.h"
#include "http_client.h"
#include "JSON.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define SERVER_HOST        "www.httpbin.org"

#define SERVER_PORT        ( 443 )
#define DNS_TIMEOUT_MS     ( 10000 )
#define CONNECT_TIMEOUT_MS ( 3000 )
#define TOTAL_REQUESTS     ( 2 )

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

static void  event_handler( http_client_t* client, http_event_t event, http_response_t* response );
static void  print_data   ( char* data, uint32_t length );
static void  print_content( char* data, uint32_t length );
static void  print_header ( http_header_field_t* header, uint32_t number_of_fields );
static wiced_result_t parse_http_response_info(wiced_json_object_t * json_object );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const char httpbin_root_ca_certificate[] =
        "-----BEGIN CERTIFICATE-----\n"
        "MIIFCzCCA/OgAwIBAgISAxPNnIyDId0ADM/B6tI0D21XMA0GCSqGSIb3DQEBCwUA\n"
        "MEoxCzAJBgNVBAYTAlVTMRYwFAYDVQQKEw1MZXQncyBFbmNyeXB0MSMwIQYDVQQD\n"
        "ExpMZXQncyBFbmNyeXB0IEF1dGhvcml0eSBYMzAeFw0xNzA1MTYwMDEzMDBaFw0x\n"
        "NzA4MTQwMDEzMDBaMBYxFDASBgNVBAMTC2h0dHBiaW4ub3JnMIIBIjANBgkqhkiG\n"
        "9w0BAQEFAAOCAQ8AMIIBCgKCAQEA2/PNpMVE+Sv/GYdYE11d3xLZCdME6+eBNqpJ\n"
        "TR1Lbm+ynJig6I6kVY3SSNWlDwLn2qGgattSLCdSk5k3z+vkNLtj6/esNruBFQLk\n"
        "BIRc610SiiIQptPJQPaVnhIRHXAdwRpjA7Bdhkt9yKfpY5cXOJOUQp0dBrIxVPc0\n"
        "lo3gedfNwYDgNwujjn2OsSqFBEf39oFWAyP5sDorckrukb0p562HU9bSg6Es6Box\n"
        "pa8LZCRHpbW0TzSsCauMiqKdYcE6WwBtJ19P0DAFsUHIfhod7ykO+GAnKa5fllgc\n"
        "Du/s5QXEVHG0U6Joai/SNNn4I4pj74y8gnat4eazqvNGRr6PtQIDAQABo4ICHTCC\n"
        "AhkwDgYDVR0PAQH/BAQDAgWgMB0GA1UdJQQWMBQGCCsGAQUFBwMBBggrBgEFBQcD\n"
        "AjAMBgNVHRMBAf8EAjAAMB0GA1UdDgQWBBT/ZrDwFEaz9KxXCFGkrNtMFbbFXzAf\n"
        "BgNVHSMEGDAWgBSoSmpjBH3duubRObemRWXv86jsoTBwBggrBgEFBQcBAQRkMGIw\n"
        "LwYIKwYBBQUHMAGGI2h0dHA6Ly9vY3NwLmludC14My5sZXRzZW5jcnlwdC5vcmcv\n"
        "MC8GCCsGAQUFBzAChiNodHRwOi8vY2VydC5pbnQteDMubGV0c2VuY3J5cHQub3Jn\n"
        "LzAnBgNVHREEIDAeggtodHRwYmluLm9yZ4IPd3d3Lmh0dHBiaW4ub3JnMIH+BgNV\n"
        "HSAEgfYwgfMwCAYGZ4EMAQIBMIHmBgsrBgEEAYLfEwEBATCB1jAmBggrBgEFBQcC\n"
        "ARYaaHR0cDovL2Nwcy5sZXRzZW5jcnlwdC5vcmcwgasGCCsGAQUFBwICMIGeDIGb\n"
        "VGhpcyBDZXJ0aWZpY2F0ZSBtYXkgb25seSBiZSByZWxpZWQgdXBvbiBieSBSZWx5\n"
        "aW5nIFBhcnRpZXMgYW5kIG9ubHkgaW4gYWNjb3JkYW5jZSB3aXRoIHRoZSBDZXJ0\n"
        "aWZpY2F0ZSBQb2xpY3kgZm91bmQgYXQgaHR0cHM6Ly9sZXRzZW5jcnlwdC5vcmcv\n"
        "cmVwb3NpdG9yeS8wDQYJKoZIhvcNAQELBQADggEBAEfy43VHVIo27A9aTxkebtRK\n"
        "vx/+nRbCVreVMkwCfqgbpr2T+oB8Cd8qZ4bTPtB+c0tMo8WhMO1m+gPBUrJeXtSW\n"
        "Iq5H6dUtelPAP6w9CsbFeaCM2v++Rz1UHCvTxqF0avyQHc4MKJv52rYPDPlwS4JB\n"
        "XN4UFRVjQZWaSSvFYPsea/rI1nlSZRwTlLBO/ijJeA8nJDmrVbC3eWH7wffrCJoM\n"
        "WOfnEWZz5r5IaJCm0eIx2jVVzFDVj0dnUjCjvCnDl8bZOcfzyoL3+Nq9rfsQORLU\n"
        "auYPbGmt+Av5/PYSWkpAiyxubfUV9gsABuQ+K5hUiLJtovufTPp6EcTN8hztPFA=\n"
        "-----END CERTIFICATE-----\n"
        "-----BEGIN CERTIFICATE-----\n"
        "MIIEkjCCA3qgAwIBAgIQCgFBQgAAAVOFc2oLheynCDANBgkqhkiG9w0BAQsFADA/\n"
        "MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n"
        "DkRTVCBSb290IENBIFgzMB4XDTE2MDMxNzE2NDA0NloXDTIxMDMxNzE2NDA0Nlow\n"
        "SjELMAkGA1UEBhMCVVMxFjAUBgNVBAoTDUxldCdzIEVuY3J5cHQxIzAhBgNVBAMT\n"
        "GkxldCdzIEVuY3J5cHQgQXV0aG9yaXR5IFgzMIIBIjANBgkqhkiG9w0BAQEFAAOC\n"
        "AQ8AMIIBCgKCAQEAnNMM8FrlLke3cl03g7NoYzDq1zUmGSXhvb418XCSL7e4S0EF\n"
        "q6meNQhY7LEqxGiHC6PjdeTm86dicbp5gWAf15Gan/PQeGdxyGkOlZHP/uaZ6WA8\n"
        "SMx+yk13EiSdRxta67nsHjcAHJyse6cF6s5K671B5TaYucv9bTyWaN8jKkKQDIZ0\n"
        "Z8h/pZq4UmEUEz9l6YKHy9v6Dlb2honzhT+Xhq+w3Brvaw2VFn3EK6BlspkENnWA\n"
        "a6xK8xuQSXgvopZPKiAlKQTGdMDQMc2PMTiVFrqoM7hD8bEfwzB/onkxEz0tNvjj\n"
        "/PIzark5McWvxI0NHWQWM6r6hCm21AvA2H3DkwIDAQABo4IBfTCCAXkwEgYDVR0T\n"
        "AQH/BAgwBgEB/wIBADAOBgNVHQ8BAf8EBAMCAYYwfwYIKwYBBQUHAQEEczBxMDIG\n"
        "CCsGAQUFBzABhiZodHRwOi8vaXNyZy50cnVzdGlkLm9jc3AuaWRlbnRydXN0LmNv\n"
        "bTA7BggrBgEFBQcwAoYvaHR0cDovL2FwcHMuaWRlbnRydXN0LmNvbS9yb290cy9k\n"
        "c3Ryb290Y2F4My5wN2MwHwYDVR0jBBgwFoAUxKexpHsscfrb4UuQdf/EFWCFiRAw\n"
        "VAYDVR0gBE0wSzAIBgZngQwBAgEwPwYLKwYBBAGC3xMBAQEwMDAuBggrBgEFBQcC\n"
        "ARYiaHR0cDovL2Nwcy5yb290LXgxLmxldHNlbmNyeXB0Lm9yZzA8BgNVHR8ENTAz\n"
        "MDGgL6AthitodHRwOi8vY3JsLmlkZW50cnVzdC5jb20vRFNUUk9PVENBWDNDUkwu\n"
        "Y3JsMB0GA1UdDgQWBBSoSmpjBH3duubRObemRWXv86jsoTANBgkqhkiG9w0BAQsF\n"
        "AAOCAQEA3TPXEfNjWDjdGBX7CVW+dla5cEilaUcne8IkCJLxWh9KEik3JHRRHGJo\n"
        "uM2VcGfl96S8TihRzZvoroed6ti6WqEBmtzw3Wodatg+VyOeph4EYpr/1wXKtx8/\n"
        "wApIvJSwtmVi4MFU5aMqrSDE6ea73Mj2tcMyo5jMd6jmeWUHK8so/joWUoHOUgwu\n"
        "X4Po1QYz+3dszkDqMp4fklxBwXRsW10KXzPMTZ+sOPAveyxindmjkW8lGy+QsRlG\n"
        "PfZ+G6Z6h7mjem0Y+iWlkYcV4PIWL1iwBi8saCbGS5jN2p8M+X+Q7UNKEkROb3N6\n"
        "KOqkqm57TH2H3eDJAkSnh6/DNFu0Qg==\n"
        "-----END CERTIFICATE-----\n"
        "-----BEGIN CERTIFICATE-----\n"
        "MIIDSjCCAjKgAwIBAgIQRK+wgNajJ7qJMDmGLvhAazANBgkqhkiG9w0BAQUFADA/\n"
        "MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\n"
        "DkRTVCBSb290IENBIFgzMB4XDTAwMDkzMDIxMTIxOVoXDTIxMDkzMDE0MDExNVow\n"
        "PzEkMCIGA1UEChMbRGlnaXRhbCBTaWduYXR1cmUgVHJ1c3QgQ28uMRcwFQYDVQQD\n"
        "Ew5EU1QgUm9vdCBDQSBYMzCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEB\n"
        "AN+v6ZdQCINXtMxiZfaQguzH0yxrMMpb7NnDfcdAwRgUi+DoM3ZJKuM/IUmTrE4O\n"
        "rz5Iy2Xu/NMhD2XSKtkyj4zl93ewEnu1lcCJo6m67XMuegwGMoOifooUMM0RoOEq\n"
        "OLl5CjH9UL2AZd+3UWODyOKIYepLYYHsUmu5ouJLGiifSKOeDNoJjj4XLh7dIN9b\n"
        "xiqKqy69cK3FCxolkHRyxXtqqzTWMIn/5WgTe1QLyNau7Fqckh49ZLOMxt+/yUFw\n"
        "7BZy1SbsOFU5Q9D8/RhcQPGX69Wam40dutolucbY38EVAjqr2m7xPi71XAicPNaD\n"
        "aeQQmxkqtilX4+U9m5/wAl0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNV\n"
        "HQ8BAf8EBAMCAQYwHQYDVR0OBBYEFMSnsaR7LHH62+FLkHX/xBVghYkQMA0GCSqG\n"
        "SIb3DQEBBQUAA4IBAQCjGiybFwBcqR7uKGY3Or+Dxz9LwwmglSBd49lZRNI+DT69\n"
        "ikugdB/OEIKcdBodfpga3csTS7MgROSR6cz8faXbauX+5v3gTt23ADq1cEmv8uXr\n"
        "AvHRAosZy5Q6XkjEGB5YGV8eAlrwDPGxrancWYaLbumR9YbK+rlmM6pZW87ipxZz\n"
        "R8srzJmwN0jP41ZL9c8PDHIyh8bwRLtTcm1D9SZImlJnt1ir/md2cXjbDaJWFBM5\n"
        "JDGFoqgCWjBH4d1QB7wCCZAA62RjYJsWvIjJEubSfZGL+T0yjWW06XyxV3bqxbYo\n"
        "Ob8VZRzI9neWagqNdwvYkQsEjgfbKbYK7p2CNTUQ\n"
        "-----END CERTIFICATE-----\n";

/******************************************************
 *               Function Definitions
 ******************************************************/

static http_client_t  client;
static http_request_t requests[TOTAL_REQUESTS];
static http_client_configuration_info_t client_configuration;

static const char* request_uris[] =
{
    [0] = "/get",
    [1] = "/html",
};

void application_start( void )
{
    wiced_ip_address_t  ip_address;
    wiced_result_t      result;
    http_header_field_t header[2];

    wiced_init( );
    result = wiced_network_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO( ( "STA unable to join AP \n" ) );
        return;
    }

    WPRINT_APP_INFO( ( "Resolving IP address of %s\n", SERVER_HOST ) );
    wiced_hostname_lookup( SERVER_HOST, &ip_address, DNS_TIMEOUT_MS, WICED_STA_INTERFACE );
    WPRINT_APP_INFO( ( "%s is at %u.%u.%u.%u\n", SERVER_HOST,
                                                 (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 24),
                                                 (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 16),
                                                 (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 8),
                                                 (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 0) ) );

    /* Initialize the root CA certificate */
    result = wiced_tls_init_root_ca_certificates( httpbin_root_ca_certificate, strlen(httpbin_root_ca_certificate) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "Error: Root CA certificate failed to initialize: %u\n", result) );
        return;
    }

    http_client_init( &client, WICED_STA_INTERFACE, event_handler, NULL );
    WPRINT_APP_INFO( ( "Connecting to %s\n", SERVER_HOST ) );

    /* configure HTTP client parameters */
    client_configuration.flag = (http_client_configuration_flags_t)(HTTP_CLIENT_CONFIG_FLAG_SERVER_NAME | HTTP_CLIENT_CONFIG_FLAG_MAX_FRAGMENT_LEN);
    client_configuration.server_name = (uint8_t*) SERVER_HOST;
    client_configuration.max_fragment_length = TLS_FRAGMENT_LENGTH_1024;
    http_client_configure(&client, &client_configuration);

    /* if you set hostname, library will make sure subject name in the server certificate is matching with host name you are trying to connect. pass NULL if you don't want to enable this check */
    client.peer_cn = (uint8_t*) SERVER_HOST;

    if ( ( result = http_client_connect( &client, (const wiced_ip_address_t*)&ip_address, SERVER_PORT, HTTP_USE_TLS, CONNECT_TIMEOUT_MS ) ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "Error: failed to connect to server: %u\n", result) );
        return;
    }

    WPRINT_APP_INFO( ( "Connected\n" ) );

    wiced_JSON_parser_register_callback(parse_http_response_info);

    header[0].field        = HTTP_HEADER_HOST;
    header[0].field_length = sizeof( HTTP_HEADER_HOST ) - 1;
    header[0].value        = SERVER_HOST;
    header[0].value_length = sizeof( SERVER_HOST ) - 1;

    http_request_init( &requests[0], &client, HTTP_GET, request_uris[0], HTTP_1_1 );
    http_request_write_header( &requests[0], &header[0], 1 );
    http_request_write_end_header( &requests[0] );
    http_request_flush( &requests[0] );

    header[1].field        = HTTP_HEADER_HOST;
    header[1].field_length = sizeof( HTTP_HEADER_HOST ) - 1;
    header[1].value        = SERVER_HOST;
    header[1].value_length = sizeof( SERVER_HOST ) - 1;

    http_request_init( &requests[1], &client, HTTP_GET, request_uris[1], HTTP_1_1 );
    http_request_write_header( &requests[1], &header[1], 1 );
    http_request_write_end_header( &requests[1] );
    http_request_flush( &requests[1] );
}

static void event_handler( http_client_t* client, http_event_t event, http_response_t* response )
{
    switch( event )
    {
        case HTTP_CONNECTED:
            WPRINT_APP_INFO(( "Connected to %s\n", SERVER_HOST ));
            break;

        case HTTP_DISCONNECTED:
        {
            WPRINT_APP_INFO(( "Disconnected from %s\n", SERVER_HOST ));
            http_request_deinit( &requests[0] );
            http_request_deinit( &requests[1] );
            break;
        }

        case HTTP_DATA_RECEIVED:
        {
            if ( response->request == &requests[0] )
            {
                /* Response to first request. Simply print the result */
                WPRINT_APP_INFO( ( "\nRecieved response for request #1. Content received:\n" ) );

                /* print only HTTP header */
                if(response->response_hdr != NULL)
                {
                    WPRINT_APP_INFO( ( "\n HTTP Header Information for response1 : \n" ) );
                    print_content( (char*) response->response_hdr, response->response_hdr_length );
                }

                /* print payload information comes as HTTP response body */
                WPRINT_APP_INFO( ( "\n Payload Information for response1 : \n" ) );
                print_content( (char*) response->payload, response->payload_data_length );
                if(wiced_JSON_parser( (const char*)response->payload , response->payload_data_length ) != WICED_SUCCESS)
                {
                    WPRINT_APP_INFO( ( "\n JSON parsing Error: \n" ) );
                }

                if(response->remaining_length == 0)
                {
                   WPRINT_APP_INFO( ( "Received total payload data for response1 \n" ) );
                }
            }
            else if ( response->request == &requests[1] )
            {
                /* Response to 2nd request. Simply print the result */
                WPRINT_APP_INFO( ( "\nRecieved response for request #2. Content received:\n" ) );

                /* Response to second request. Parse header for "Date" and "Content-Length" */
                http_header_field_t header_fields[2];
                uint32_t size = sizeof( header_fields ) / sizeof(http_header_field_t);

                /* only process HTTP header when response contains it */
                if(response->response_hdr != NULL)
                {
                    WPRINT_APP_INFO( ( "\n HTTP Header Information for response2 : \n" ) );
                    print_content( (char*) response->response_hdr, response->response_hdr_length );

                    header_fields[ 0 ].field        = HTTP_HEADER_DATE;
                    header_fields[ 0 ].field_length = sizeof( HTTP_HEADER_DATE ) - 1;
                    header_fields[ 0 ].value        = NULL;
                    header_fields[ 0 ].value_length = 0;
                    header_fields[ 1 ].field        = HTTP_HEADER_CONTENT_LENGTH;
                    header_fields[ 1 ].field_length = sizeof( HTTP_HEADER_CONTENT_LENGTH ) - 1;
                    header_fields[ 1 ].value        = NULL;
                    header_fields[ 1 ].value_length = 0;

                    if ( http_parse_header( response->response_hdr, response->response_hdr_length, header_fields, size ) == WICED_SUCCESS )
                    {
                        WPRINT_APP_INFO( ( "\nParsing response of request #2 for \"Date\" and \"Content-Length\". Fields found:\n" ) );
                        print_header( header_fields, size );
                    }
                }

                /* Print payload information that comes as response body */
                WPRINT_APP_INFO( ( "Payload Information for response2 : \n" ) );
                print_content( (char*) response->payload, response->payload_data_length );

                if(response->remaining_length == 0)
                {
                    WPRINT_APP_INFO( ( "Received total payload data for response2 \n" ) );
                }
            }
        break;
        }
        default:
        break;
    }
}

static void print_data( char* data, uint32_t length )
{
    uint32_t a;

    for ( a = 0; a < length; a++ )
    {
        WPRINT_APP_INFO( ( "%c", data[a] ) );
    }
}

static void print_content( char* data, uint32_t length )
{
    WPRINT_APP_INFO(( "==============================================\n" ));
    print_data( (char*)data, length );
    WPRINT_APP_INFO(( "\n==============================================\n" ));
}

static void print_header( http_header_field_t* header_fields, uint32_t number_of_fields )
{
    uint32_t a;

    WPRINT_APP_INFO(( "==============================================\n" ));
    for ( a = 0; a < 2; a++ )
    {
        print_data( header_fields[a].field, header_fields[a].field_length );
        WPRINT_APP_INFO(( " : " ));
        print_data( header_fields[a].value, header_fields[a].value_length );
        WPRINT_APP_INFO(( "\n" ));
    }
    WPRINT_APP_INFO(( "==============================================\n" ));
}

static wiced_result_t parse_http_response_info(wiced_json_object_t * json_object )
{
    if(strncmp(json_object->object_string, "url", sizeof("url")-1) == 0)
    {
        WPRINT_APP_INFO (("Requested URL : %.*s\n",json_object->value_length, json_object->value));
    }

    return WICED_SUCCESS;
}
