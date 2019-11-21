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
 * HTTP2 test application.
 * This application should test the following operations
 * 1- Connect/upgrade the server to HTTP2
 * 2- Provide requests to the server
 * 3- Receive header and/or data from the server and print the output on console
 *
 * Steps:
 * 1. Update CLIENT AP credentials in default_wifi_config_dct.h
 * 2. Check if the IP address given in NGHTTP2_SERVER_IP_ADDRESS is correct for host "https://www.nghttp2.org" (Note: Online DNS lookup tools can be used to identify the IP address)
 * 3. Build this application and run
 *
 */
#include "wiced.h"
#include "http2.h"
#include "http2_pvt.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( command, ok_message, error_message )   \
{                                                                                           \
    ret = (command);                                                                        \
    http_test_print_status( ret, (const char *)ok_message, (const char *)error_message );   \
    if ( ret != WICED_SUCCESS ) break;                                                      \
}

#define NUM_REQUESTS_PER_CONNECT            (3)
#define NUM_CONNECT_DISCONNECT_TRIES        (2)
/******************************************************
 *                    Constants
 ******************************************************/
#define HTTP_CONNECTION_ENABLE_TLS

#define NGHTTP2_SERVER_IP_ADDRESS   MAKE_IPV4_ADDRESS(139,162,123,134)  /* https://www.nghttp2.org */

#define HTTP2_WORKER_THREAD_STACK_SIZE          (6 * 1024)

#define NGHTTP2_ROOT_CA_CERTIFICATE_CHAIN  \
"-----BEGIN CERTIFICATE-----\r\n"\
"MIIFCzCCA/OgAwIBAgISA6bZ1evq3eG+tFJ8X9NQUi30MA0GCSqGSIb3DQEBCwUA\r\n"\
"MEoxCzAJBgNVBAYTAlVTMRYwFAYDVQQKEw1MZXQncyBFbmNyeXB0MSMwIQYDVQQD\r\n"\
"ExpMZXQncyBFbmNyeXB0IEF1dGhvcml0eSBYMzAeFw0xNzAxMjEyMjU4MDBaFw0x\r\n"\
"NzA0MjEyMjU4MDBaMBYxFDASBgNVBAMTC25naHR0cDIub3JnMIIBIjANBgkqhkiG\r\n"\
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEAvBDTIRFqCB/YEms23jIQpWmUHMH+YUjMLPcI\r\n"\
"0Xt2QRHIxbNGGEglpnvx/6eTZJLWtIKY5DRB62SnDP+zpNxrlP4f86PaC+WW3V5f\r\n"\
"5YW2PoVsdXl42W28uMFXaTjU56HIXtHqO0yle7oRCPLxaixZe9Zuf7PT5qw07TK4\r\n"\
"rqpobQJKngtV7o7uGcQqOWegnnstYce74uoXHSIjdnUstZDliECRPt73A/o1iec7\r\n"\
"PQsg7K1piGaz5wZTqjmNqEhw+sbndTceMaZFhPJoyLaa1ADxyQy/50GGHRST48LC\r\n"\
"3UwBkVK3WryjR9rFl4TPr0NGNnU94LOYbt0Z4inPI6bhezhVtQIDAQABo4ICHTCC\r\n"\
"AhkwDgYDVR0PAQH/BAQDAgWgMB0GA1UdJQQWMBQGCCsGAQUFBwMBBggrBgEFBQcD\r\n"\
"AjAMBgNVHRMBAf8EAjAAMB0GA1UdDgQWBBSv+053eJRrPBYYvnT0cwO3kZRbEzAf\r\n"\
"BgNVHSMEGDAWgBSoSmpjBH3duubRObemRWXv86jsoTBwBggrBgEFBQcBAQRkMGIw\r\n"\
"LwYIKwYBBQUHMAGGI2h0dHA6Ly9vY3NwLmludC14My5sZXRzZW5jcnlwdC5vcmcv\r\n"\
"MC8GCCsGAQUFBzAChiNodHRwOi8vY2VydC5pbnQteDMubGV0c2VuY3J5cHQub3Jn\r\n"\
"LzAnBgNVHREEIDAeggtuZ2h0dHAyLm9yZ4IPd3d3Lm5naHR0cDIub3JnMIH+BgNV\r\n"\
"HSAEgfYwgfMwCAYGZ4EMAQIBMIHmBgsrBgEEAYLfEwEBATCB1jAmBggrBgEFBQcC\r\n"\
"ARYaaHR0cDovL2Nwcy5sZXRzZW5jcnlwdC5vcmcwgasGCCsGAQUFBwICMIGeDIGb\r\n"\
"VGhpcyBDZXJ0aWZpY2F0ZSBtYXkgb25seSBiZSByZWxpZWQgdXBvbiBieSBSZWx5\r\n"\
"aW5nIFBhcnRpZXMgYW5kIG9ubHkgaW4gYWNjb3JkYW5jZSB3aXRoIHRoZSBDZXJ0\r\n"\
"aWZpY2F0ZSBQb2xpY3kgZm91bmQgYXQgaHR0cHM6Ly9sZXRzZW5jcnlwdC5vcmcv\r\n"\
"cmVwb3NpdG9yeS8wDQYJKoZIhvcNAQELBQADggEBAA8caivsaSVSxYNd3xmQLfEf\r\n"\
"Lk9IVCI+h8bFvi3CgLGZuFQtfUAXUSdmfNfgbj9AEgNLehvEMYwh0HVsyjLxJnI6\r\n"\
"aKlZgPdEDu8azUYBOGhZ8Jq6DWLF2qbDLaB+KbfiHyT6B8U9o0UB6u9A4y4snbbz\r\n"\
"+U8T4PqcAcdZ445QWoTaVa+f5e+0yfqMx+yA4WzPL6IRR8JykhlaeOzIJ4Xzgbi/\r\n"\
"aqqxg0GPqj8SyFodd50MygKYTHyRhL1MpYiZsh3LVZEONaktdZT50F5iU/TtT0WU\r\n"\
"Ou+yEcnNaxS2ePIWLFiBMECPj0ijHeZjDM2cxWoB27mivtiLmp+o9C3tAVvx+vg=\r\n"\
"-----END CERTIFICATE-----\r\n"\
"-----BEGIN CERTIFICATE-----\r\n"\
"MIIEkjCCA3qgAwIBAgIQCgFBQgAAAVOFc2oLheynCDANBgkqhkiG9w0BAQsFADA/\r\n"\
"MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\r\n"\
"DkRTVCBSb290IENBIFgzMB4XDTE2MDMxNzE2NDA0NloXDTIxMDMxNzE2NDA0Nlow\r\n"\
"SjELMAkGA1UEBhMCVVMxFjAUBgNVBAoTDUxldCdzIEVuY3J5cHQxIzAhBgNVBAMT\r\n"\
"GkxldCdzIEVuY3J5cHQgQXV0aG9yaXR5IFgzMIIBIjANBgkqhkiG9w0BAQEFAAOC\r\n"\
"AQ8AMIIBCgKCAQEAnNMM8FrlLke3cl03g7NoYzDq1zUmGSXhvb418XCSL7e4S0EF\r\n"\
"q6meNQhY7LEqxGiHC6PjdeTm86dicbp5gWAf15Gan/PQeGdxyGkOlZHP/uaZ6WA8\r\n"\
"SMx+yk13EiSdRxta67nsHjcAHJyse6cF6s5K671B5TaYucv9bTyWaN8jKkKQDIZ0\r\n"\
"Z8h/pZq4UmEUEz9l6YKHy9v6Dlb2honzhT+Xhq+w3Brvaw2VFn3EK6BlspkENnWA\r\n"\
"a6xK8xuQSXgvopZPKiAlKQTGdMDQMc2PMTiVFrqoM7hD8bEfwzB/onkxEz0tNvjj\r\n"\
"/PIzark5McWvxI0NHWQWM6r6hCm21AvA2H3DkwIDAQABo4IBfTCCAXkwEgYDVR0T\r\n"\
"AQH/BAgwBgEB/wIBADAOBgNVHQ8BAf8EBAMCAYYwfwYIKwYBBQUHAQEEczBxMDIG\r\n"\
"CCsGAQUFBzABhiZodHRwOi8vaXNyZy50cnVzdGlkLm9jc3AuaWRlbnRydXN0LmNv\r\n"\
"bTA7BggrBgEFBQcwAoYvaHR0cDovL2FwcHMuaWRlbnRydXN0LmNvbS9yb290cy9k\r\n"\
"c3Ryb290Y2F4My5wN2MwHwYDVR0jBBgwFoAUxKexpHsscfrb4UuQdf/EFWCFiRAw\r\n"\
"VAYDVR0gBE0wSzAIBgZngQwBAgEwPwYLKwYBBAGC3xMBAQEwMDAuBggrBgEFBQcC\r\n"\
"ARYiaHR0cDovL2Nwcy5yb290LXgxLmxldHNlbmNyeXB0Lm9yZzA8BgNVHR8ENTAz\r\n"\
"MDGgL6AthitodHRwOi8vY3JsLmlkZW50cnVzdC5jb20vRFNUUk9PVENBWDNDUkwu\r\n"\
"Y3JsMB0GA1UdDgQWBBSoSmpjBH3duubRObemRWXv86jsoTANBgkqhkiG9w0BAQsF\r\n"\
"AAOCAQEA3TPXEfNjWDjdGBX7CVW+dla5cEilaUcne8IkCJLxWh9KEik3JHRRHGJo\r\n"\
"uM2VcGfl96S8TihRzZvoroed6ti6WqEBmtzw3Wodatg+VyOeph4EYpr/1wXKtx8/\r\n"\
"wApIvJSwtmVi4MFU5aMqrSDE6ea73Mj2tcMyo5jMd6jmeWUHK8so/joWUoHOUgwu\r\n"\
"X4Po1QYz+3dszkDqMp4fklxBwXRsW10KXzPMTZ+sOPAveyxindmjkW8lGy+QsRlG\r\n"\
"PfZ+G6Z6h7mjem0Y+iWlkYcV4PIWL1iwBi8saCbGS5jN2p8M+X+Q7UNKEkROb3N6\r\n"\
"KOqkqm57TH2H3eDJAkSnh6/DNFu0Qg==\r\n"\
"-----END CERTIFICATE-----\r\n"\
"-----BEGIN CERTIFICATE-----\r\n"\
"MIIDSjCCAjKgAwIBAgIQRK+wgNajJ7qJMDmGLvhAazANBgkqhkiG9w0BAQUFADA/\r\n"\
"MSQwIgYDVQQKExtEaWdpdGFsIFNpZ25hdHVyZSBUcnVzdCBDby4xFzAVBgNVBAMT\r\n"\
"DkRTVCBSb290IENBIFgzMB4XDTAwMDkzMDIxMTIxOVoXDTIxMDkzMDE0MDExNVow\r\n"\
"PzEkMCIGA1UEChMbRGlnaXRhbCBTaWduYXR1cmUgVHJ1c3QgQ28uMRcwFQYDVQQD\r\n"\
"Ew5EU1QgUm9vdCBDQSBYMzCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEB\r\n"\
"AN+v6ZdQCINXtMxiZfaQguzH0yxrMMpb7NnDfcdAwRgUi+DoM3ZJKuM/IUmTrE4O\r\n"\
"rz5Iy2Xu/NMhD2XSKtkyj4zl93ewEnu1lcCJo6m67XMuegwGMoOifooUMM0RoOEq\r\n"\
"OLl5CjH9UL2AZd+3UWODyOKIYepLYYHsUmu5ouJLGiifSKOeDNoJjj4XLh7dIN9b\r\n"\
"xiqKqy69cK3FCxolkHRyxXtqqzTWMIn/5WgTe1QLyNau7Fqckh49ZLOMxt+/yUFw\r\n"\
"7BZy1SbsOFU5Q9D8/RhcQPGX69Wam40dutolucbY38EVAjqr2m7xPi71XAicPNaD\r\n"\
"aeQQmxkqtilX4+U9m5/wAl0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNV\r\n"\
"HQ8BAf8EBAMCAQYwHQYDVR0OBBYEFMSnsaR7LHH62+FLkHX/xBVghYkQMA0GCSqG\r\n"\
"SIb3DQEBBQUAA4IBAQCjGiybFwBcqR7uKGY3Or+Dxz9LwwmglSBd49lZRNI+DT69\r\n"\
"ikugdB/OEIKcdBodfpga3csTS7MgROSR6cz8faXbauX+5v3gTt23ADq1cEmv8uXr\r\n"\
"AvHRAosZy5Q6XkjEGB5YGV8eAlrwDPGxrancWYaLbumR9YbK+rlmM6pZW87ipxZz\r\n"\
"R8srzJmwN0jP41ZL9c8PDHIyh8bwRLtTcm1D9SZImlJnt1ir/md2cXjbDaJWFBM5\r\n"\
"JDGFoqgCWjBH4d1QB7wCCZAA62RjYJsWvIjJEubSfZGL+T0yjWW06XyxV3bqxbYo\r\n"\
"Ob8VZRzI9neWagqNdwvYkQsEjgfbKbYK7p2CNTUQ\r\n"\
"-----END CERTIFICATE-----\r\n"\
"\0"\
"\0"

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
static void http_test_print_status      ( wiced_result_t restult, const char * ok_message, const char * error_message );
static void http_test_hpack_integer     ( uint32_t value, uint8_t prefix  );
static void http_test_hpack_huffman     ( );
static void http_test_indexed_header    ( http_connection_t* connect );
static wiced_result_t http_recv_header_callback         ( http_connection_ptr_t connect, http_request_id_t request, http_header_info_t* headers, http_frame_type_t type, http_frame_flags_t flags, void* user_data );
static wiced_result_t http_recv_data_callback           ( http_connection_ptr_t connect, http_request_id_t request, uint8_t* data, uint32_t lenght, http_frame_flags_t flags, void* user_data );
static wiced_result_t http_disconnect_event_callback    ( http_connection_ptr_t connect, http_request_id_t last_processed_request, uint32_t error, void* user_data );

/******************************************************
 *               Variable Definitions
 ******************************************************/
static wiced_ip_address_t  INITIALISER_IPV4_ADDRESS( http_server_address, NGHTTP2_SERVER_IP_ADDRESS );
static http_connection_t connect;
static http_connection_callbacks_t callbacks;
static uint8_t scratch [ HTTP2_FRAME_SCRATCH_BUFF_SIZE ];
static wiced_semaphore_t app_semaphore;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( void )
{
    wiced_result_t        ret = WICED_SUCCESS;
    http_security_info_t  security;
#ifdef HTTP_CONNECTION_ENABLE_TLS
    uint16_t port = 443;
    http_security_info_t* security_p = &security;
#else
    uint16_t port = 80;
    http_security_info_t* security_p = NULL;
#endif
    int i;

    (void)http_test_hpack_huffman;

    /* Initialise the device and WICED framework */
    wiced_init( );

    /* Bring up the network interface */
    WPRINT_APP_INFO(("Joining the network...\n"));
    ret = wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
    if (ret != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Failed to join the network. Error = [%d]\n", ret));
        return;
    }

    wiced_rtos_init_semaphore( &app_semaphore );
    wiced_rtos_set_semaphore( &app_semaphore );

    WPRINT_APP_INFO(("HTTP2 test application\n"));
    WPRINT_APP_INFO(("======================\n"));

    callbacks.http_disconnect_callback          = http_disconnect_event_callback;
    callbacks.http_request_recv_header_callback = http_recv_header_callback;
    callbacks.http_request_recv_data_callback   = http_recv_data_callback;

    (void)http_test_hpack_integer;

    security.cert = NULL;
    security.key = NULL;
    security.ca_cert = NGHTTP2_ROOT_CA_CERTIFICATE_CHAIN;
    security.ca_cert_len = strlen(NGHTTP2_ROOT_CA_CERTIFICATE_CHAIN);
    (void)security;

    ret = http_connection_init(&connect, security_p, &callbacks, scratch, sizeof(scratch), HTTP2_WORKER_THREAD_STACK_SIZE, &app_semaphore);
    http_test_print_status(ret, "HTTP Init.", "HTTP Init. Make sure your certificates are valid.");
    if (ret != WICED_SUCCESS)
    {
        return;
    }

    i = 0;
    WPRINT_APP_INFO(("\r\n TOTAL CONNECT-DISCONNECT ITERATIONS = [%d]\r\n", NUM_CONNECT_DISCONNECT_TRIES));
    while (i < NUM_CONNECT_DISCONNECT_TRIES)
    {
        int request_counts = 0;

        WPRINT_APP_INFO(("\r\n CONNECT COUNT = [%d]\r\n", i + 1));

        RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( http_connect( &connect, &http_server_address, port, WICED_STA_INTERFACE, NULL ), "HTTP Connect.", "HTTP connect. Did you check your HTTP server IP address?" );
        wiced_rtos_delay_milliseconds(1000);
        WPRINT_APP_INFO(("\r\n TOTAL NUMBER OF REQUEST = [%d]\r\n", NUM_REQUESTS_PER_CONNECT));
        while ( request_counts < NUM_REQUESTS_PER_CONNECT)
        {
            WPRINT_APP_INFO(("\r\n SENDING REQUEST - [%d]\r\n", request_counts + 1));
            WPRINT_APP_INFO(("\r\n -------------------- \r\n"));
            http_test_indexed_header( &connect );
            wiced_rtos_delay_milliseconds(10000);
            WPRINT_APP_INFO(("\r\n =================== \r\n"));
            request_counts++;
        }
        RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( http_disconnect( &connect ), "HTTP Disconnect.", "HTTP Disconnect." );
        wiced_rtos_delay_milliseconds(1000);
        i++;
    }
    ret = http_connection_deinit( &connect );
    http_test_print_status(ret, "HTTP DeInit.", "HTTP DeInit.");
    WPRINT_APP_INFO(("done\r\n"));
}

/******************************************************
 *               Static Function Definitions
 ******************************************************/
static void http_test_hpack_integer( uint32_t value, uint8_t prefix )
{
    uint32_t ret_value = 0;
    uint8_t  int_array [8];
    uint8_t* buffer = int_array;
    uint32_t size = sizeof(int_array);

    http_hpack_put_integer  ( value, prefix, &buffer, &size );

    buffer = int_array;
    size   = sizeof(int_array);
    http_hpack_get_integer ( &ret_value, prefix, &buffer, &size );

    if ( value != ret_value )
    {
        WPRINT_APP_INFO(( "HPACK error values don't match\n"));
    }
    else
    {
        WPRINT_APP_INFO(( "HPACK ok values match\n"));
    }
}

static void http_test_hpack_huffman(  )
{
    uint8_t  array [] = {0x82, 0x86, 0x84, 0x41, 0x8c, 0xf1, 0xe3, 0xc2, 0xe5, 0xf2, 0x3a, 0x6b, 0xa0, 0xab, 0x90, 0xf4, 0xff};
    uint8_t* buffer = array;
    uint8_t  string [1024];
    uint8_t* output = string;
    uint32_t output_size = sizeof(string);
    uint32_t size = sizeof(array);
    http_header_info_t head;


    http_hpack_decode_headers( &head, &output, &output_size, &buffer, &size);
    WPRINT_APP_INFO(( "HPACK ok values match\n"));

}
#if 0
static void http_test_indexed_header ( void )
{
    //uint8_t  stream[]= { 0x04, 0x0C, 0x2f, 0x73, 0x61, 0x6d, 0x70, 0x6c, 0x65, 0x2f, 0x70, 0x61, 0x74, 0x68 };
    uint8_t  stream[100];
    uint8_t* stream_buffer = stream;
    uint32_t stream_size = sizeof(stream);
    uint8_t  headers[100];
    uint32_t headers_size = sizeof(headers);
    uint8_t* headers_buffer = headers;
    http_header_info_t source_header;
    http_header_info_t header;

    source_header.name = (uint8_t*)"password";
    source_header.name_length = strlen("password");
    source_header.value = (uint8_t*)"secret";
    source_header.value_length = strlen("secret");

    http_hpack_encode_header( &source_header, &stream_buffer, &stream_size );

    stream_buffer = stream;
    stream_size = sizeof(stream);

    http_hpack_decode_header( &header, &headers_buffer, &headers_size, &stream_buffer, &stream_size );
    WPRINT_APP_INFO(" Test resut!\r\n");
}
#endif

static void http_test_indexed_header ( http_connection_t* connect )
{
    //uint8_t  stream[]= { 0x82,0x86, 0x84,0x04, 0x0C, 0x2f, 0x73, 0x61, 0x6d, 0x70, 0x6c, 0x65, 0x2f, 0x70, 0x61, 0x74, 0x68 };
    http_request_id_t  request;
    http_header_info_t source_headers[8];

    source_headers[0].name = (uint8_t*)":method";
    source_headers[0].name_length = strlen(":method");
    source_headers[0].value = (uint8_t*)"GET";
    source_headers[0].value_length = strlen("GET");
    source_headers[0].next = &source_headers[1];

    source_headers[1].name = (uint8_t*)":path";
    source_headers[1].name_length = strlen(":path");
    source_headers[1].value =(uint8_t*) "/";
    source_headers[1].value_length = strlen("/");
    source_headers[1].next = &source_headers[2];

    source_headers[2].name = (uint8_t*)":scheme";
    source_headers[2].name_length = strlen(":scheme");
    source_headers[2].value = (uint8_t*)"https";
    source_headers[2].value_length = strlen("https");
    source_headers[2].next = &source_headers[3];

    source_headers[3].name = (uint8_t*)":authority";
    source_headers[3].name_length = strlen(":authority");
    source_headers[3].value = (uint8_t*)"127.0.0.1";
    source_headers[3].value_length = strlen("127.0.0.1");
    source_headers[3].next = &source_headers[4];

    source_headers[4].name = (uint8_t*)"accept";
    source_headers[4].name_length = strlen("accept");
    source_headers[4].value = (uint8_t*)"*/*";
    source_headers[4].value_length = strlen("*/*");
    source_headers[4].next = &source_headers[5];

    source_headers[5].name = (uint8_t*)"accept-encoding";
    source_headers[5].name_length = strlen("accept-encoding");
    source_headers[5].value = (uint8_t*)"deflate";
    source_headers[5].value_length = strlen("deflate");
    source_headers[5].next = &source_headers[6];

    source_headers[6].name = (uint8_t*)"user-agent";
    source_headers[6].name_length = strlen("user-agent");
    source_headers[6].value = (uint8_t*)"wiced";
    source_headers[6].value_length = strlen("wiced");
    source_headers[6].next = NULL;

    WPRINT_APP_INFO((" ==== Sending Complete Frame (Header) Request\n"));

    http_request_put_headers( connect, &request, source_headers, HTTP_FRAME_FLAGS_END_STREAM );
    WPRINT_APP_INFO((" ==== Created Request ID = [%lu]\n", request));
}

static wiced_result_t http_disconnect_event_callback ( http_connection_ptr_t connect, http_request_id_t last_processed_request, uint32_t error, void* user_data )
{
    wiced_semaphore_t* semaphore = (wiced_semaphore_t*)user_data;

    WPRINT_APP_INFO(("\n ==== Got Disconnection ==== \n"));
    wiced_rtos_set_semaphore( semaphore );
    return WICED_SUCCESS;
}
/*
 * A simple callback for loging received headers.
 */
static wiced_result_t http_recv_header_callback ( http_connection_ptr_t connect, http_request_id_t request, http_header_info_t* headers, http_frame_type_t type, http_frame_flags_t flags, void* user_data )
{
    char buffer[200];
    if ( type == HTTP_FRAME_TYPE_PUSH_PROMISE )
    {
        WPRINT_APP_INFO(("\nReceived a Push Promise request %u.\r\n", (unsigned int) request ));
    }
    else
    {
        WPRINT_APP_INFO(("\nReceived a header (%u) of type (%d).\r\n", (unsigned int) request, type ));
    }
    while ( headers )
    {
        if ( headers->name_length > 0 )
        {
            memcpy( buffer, headers->name, headers->name_length );
            buffer[ headers->name_length ] = '\0';
            WPRINT_APP_INFO(("%s :", buffer ));
            memcpy( buffer, headers->value, headers->value_length );
            buffer[ headers->value_length ] = '\0';
            WPRINT_APP_INFO(("%s\n", buffer ));
        }
        headers = headers->next;
    }
    return WICED_SUCCESS;
}
/*
 * A simple callback for logging received data.
 */
static wiced_result_t http_recv_data_callback   ( http_connection_ptr_t connect, http_request_id_t request, uint8_t* data, uint32_t length, http_frame_flags_t flags, void* user_data )
{
    uint32_t i;
    wiced_semaphore_t* semaphore = (wiced_semaphore_t*)user_data;

    WPRINT_APP_INFO(("\nRECEIVED DATA %d\r\n", (int)length));
    WPRINT_APP_INFO(("=================\r\n"));
    if ( flags & HTTP_FRAME_FLAGS_END_STREAM )
    {
        wiced_rtos_set_semaphore( semaphore );
    }
    for ( i =0; i < length; i++ )
    {
        printf("%c",data[i]);
    }
    return WICED_SUCCESS;
}

/*
 * A simple result log function
 */
static void  http_test_print_status ( wiced_result_t result, const char * ok_message, const char * error_message )
{
    if ( result == WICED_SUCCESS )
    {
        if ( ok_message != NULL )
        {
            WPRINT_APP_INFO(( "OK (%s)\n\n", (ok_message)));
        }
        else
        {
            WPRINT_APP_INFO(( "OK.\n\n" ));
        }
    }
    else
    {
        if ( error_message != NULL )
        {
            WPRINT_APP_INFO(( "ERROR [%d] (%s)\n\n", result, (error_message)));
        }
        else
        {
            WPRINT_APP_INFO(( "ERROR.\n\n" ));
        }
    }
}
