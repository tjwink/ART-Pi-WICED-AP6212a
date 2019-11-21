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
 * Handles http proxy data and sends to cloud server using HTTP and HTTPS protocol stack
 *
 */

#include "wiced.h"
#include "wiced_network.h"
#include "wiced_tcpip.h"
#ifdef CLOUD_PROTO_HTTPS
#include "wiced_tls.h"
#endif
#include "gateway.h"
#include "cloudconfig.h"
#include "httpproxy.h"
#include "http_client.h"
#include "JSON.h"

extern wiced_ip_address_t ip_address;
wiced_semaphore_t http_post_semaphore;
wiced_semaphore_t http_observe_semaphore;

static uint8_t resp_buffer_post[ BUFFER_LENGTH ];
static uint8_t resp_buffer_observe[ BUFFER_LENGTH ];
static uint8_t hostname[ HTTP_HEADER_LEN ];
static wiced_thread_t httpobserve_handler;
static iot_gateway_device_t obsrv_msg;

static http_client_t  client_post, client_observe;

http_request_t requests[TOTAL_REQUESTS];

static wiced_result_t sensors_http_observe( uint16_t connid, char *urioption, char *header, char *payload );
static wiced_result_t sensors_http_post( uint16_t connid, char *urioption, char *header, char *payload );
static wiced_result_t create_http_observe_thread( iot_gateway_device_t *device );
static wiced_result_t http_observe( iot_gateway_device_t *device );
static void event_handler( http_client_t* client, http_event_t event, http_response_t* response );

wiced_result_t http_process_request( iot_gateway_device_t *device );

wiced_result_t init_http_client( )
{

#ifdef CLOUD_PROTO_HTTPS
    wiced_result_t result;
    static const char digicert_global_root_ca_certificate[] =
    "-----BEGIN CERTIFICATE-----\n"
    "MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n"
    "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
    "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n"
    "QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n"
    "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
    "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n"
    "9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n"
    "CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n"
    "nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n"
    "43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n"
    "T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n"
    "gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n"
    "BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n"
    "TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n"
    "DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n"
    "hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n"
    "06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n"
    "PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n"
    "YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n"
    "CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n"
    "-----END CERTIFICATE-----\n";


    result = wiced_tls_init_root_ca_certificates( digicert_global_root_ca_certificate,(uint32_t)strlen(digicert_global_root_ca_certificate) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO (("Error: Root CA certificate failed to initialize: %u\n", result));
        return -1;
    }
    else
    {
        WPRINT_APP_INFO (("#### ROOT CA INSTALLED: %u\n", result));
    }

#endif

    wiced_rtos_init_semaphore( &http_post_semaphore );

    return http_client_init( &client_post, WICED_STA_INTERFACE, event_handler, NULL );
}


static void print_data( char* data, uint32_t length )
{
    uint32_t a;

    for ( a = 0; a < length; a++ )
    {
        WPRINT_APP_DEBUG( ( "%c", data[a] ) );
    }
}

static void print_content( char* data, uint32_t length )
{
    WPRINT_APP_DEBUG(( "==============================================\n" ));
    print_data( (char*)data, length );
    WPRINT_APP_DEBUG(( "\n==============================================\n" ));
}


static void event_handler( http_client_t* client, http_event_t event, http_response_t* response )
{
    switch( event )
    {
        case HTTP_CONNECTED:
            WPRINT_APP_DEBUG(( "Connected to %s\n", client->config->server_name ));

            break;

        case HTTP_DISCONNECTED:
        {
            WPRINT_APP_DEBUG(( "Disconnected from %s\n", client->config->server_name ));
            break;
        }

        case HTTP_DATA_RECEIVED:
        {
            print_content( (char*) response->response_hdr, response->response_hdr_length );
            print_content( (char*) response->payload, response->payload_data_length );

            if ( response->request == &requests[REQUEST_POST] )
            {
                memcpy(resp_buffer_post, response->response_hdr, response->response_hdr_length >= BUFFER_LENGTH ? BUFFER_LENGTH : response->response_hdr_length);
                wiced_rtos_set_semaphore(&http_post_semaphore);
            }
            else
            {
                memcpy(resp_buffer_observe, response->payload, response->payload_data_length >= BUFFER_LENGTH ? BUFFER_LENGTH : response->payload_data_length);
                wiced_rtos_set_semaphore(&http_observe_semaphore);
            }

            break;
        }
        default:
        break;
    }
}


void http_observe_thread( uint32_t arg )
{

    WPRINT_APP_DEBUG (("http_observe_thread created\n"));

    http_client_init( &client_observe, WICED_STA_INTERFACE, event_handler, NULL );

    wiced_rtos_init_semaphore( &http_observe_semaphore );

    while ( 1 )
    {
        /* check for network link */
        if ( wiced_network_is_ip_up( WICED_STA_INTERFACE ) != WICED_TRUE )
        {
            wiced_rtos_delay_milliseconds( 100 );
            continue;
        }

        for ( int i = 0; i < IOT_GATEWAY_GATTS_MAX_CONN; i++ )
        {
            if ( iot_gateway_devices[ i ].conn_id == obsrv_msg.conn_id )
            {
                break;
            }
        }

        sensors_http_observe( obsrv_msg.conn_id, (char*) obsrv_msg.http_proxy_info.http_proxy_uri, (char*) obsrv_msg.http_proxy_info.http_proxy_header, (char*) obsrv_msg.http_proxy_info.http_proxy_body );

    }
}

wiced_result_t create_http_observe_thread( iot_gateway_device_t *device )
{
    uint32_t stack_size = 2 * 1024;
    wiced_result_t result;

    result = wiced_rtos_create_thread( &httpobserve_handler, 1, "Http Obsrv Thrd", http_observe_thread, stack_size, device );
    if ( result )
    {
        WPRINT_APP_INFO( ( "thread init failed !!!!!!!!!\n" ) );
    }

    return result;
}

wiced_result_t http_observe( iot_gateway_device_t *device )
{
    wiced_result_t ret;

    if ( httpobserve_handler.thread != NULL )
    {
        WPRINT_APP_INFO( ("Already one observe is running !!!\n") );
        return WICED_ERROR;
    }

    /* update the new observe value. but dont create any extra thread*/
    memcpy( &obsrv_msg, device, sizeof(iot_gateway_device_t) );

    /* check for duplicate call*/
    /* for each client there will be a one observe thread*/
    ret = create_http_observe_thread( device );

    return ret;
}

wiced_result_t sensors_http_observe( uint16_t connid, char *urioption, char *header, char *payload )
{
    wiced_result_t result;
    char *observe, *space;
    char *statusline = "\"status\":\"ok\"";
    char *pattern_start = ",\"";
    char *pattern_end = "\"]}";
    char *lineend;
    http_client_configuration_info_t client_configuration;

    /* configure HTTP client parameters */
    client_configuration.flag = (http_client_configuration_flags_t) ( HTTP_CLIENT_CONFIG_FLAG_SERVER_NAME | HTTP_CLIENT_CONFIG_FLAG_MAX_FRAGMENT_LEN );
    client_configuration.server_name = (uint8_t*) hostname;
    client_configuration.max_fragment_length = TLS_FRAGMENT_LENGTH_1024;
    http_client_configure( &client_observe, &client_configuration );

    /* if you set hostname, library will make sure subject name in the server certificate is matching with host name you are trying to connect. pass NULL if you don't want to enable this check */
    client_observe.peer_cn = NULL;

    if ( ( result = http_client_connect( &client_observe, (const wiced_ip_address_t*) &ip_address, SERVER_PORT_HTTP, HTTP_NO_SECURITY, CONNECT_TIMEOUT_MS ) ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "OBSERVE Error: failed to connect to server: %u\n", result) );
        return result;
    }

    //Ignore spaces in URI option. Just to avoid changing Home Device applications using old format
    space = strnstrn( urioption, strlen( urioption ), " ", strlen( " " ) );
    if(space != NULL)
        *space = '\0';

    http_request_init( &requests[ REQUEST_OBSERVE ], &client_observe, HTTP_POST, urioption, HTTP_1_1 );

    http_request_write( &requests[ REQUEST_OBSERVE ], (uint8_t*) header, strlen( header ) );

    // Inserting two CRLF's to indicated end of header
    http_request_write_end_header( &requests[ REQUEST_OBSERVE ] );
    http_request_write_end_header( &requests[ REQUEST_OBSERVE ] );

    http_request_write( &requests[ REQUEST_OBSERVE ], (uint8_t*) payload, strlen( payload ) );

    memset( resp_buffer_observe, 0, BUFFER_LENGTH );

    result = http_request_flush( &requests[ REQUEST_OBSERVE ] );

    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( (" Observe request flush failed\n") );
    }
    else
    {
        result = wiced_rtos_get_semaphore( &http_observe_semaphore, HTTP_REQUEST_TIMEOUT_VALUE );

        if ( result != WICED_SUCCESS )
        {
            WPRINT_APP_INFO( (" Observe request timed out\n") );
        }
        else
        {
            /*parse response*/
            observe = strnstrn( (char*) resp_buffer_observe, strlen( (char*) resp_buffer_observe ), statusline, strlen( statusline ) );
            if ( observe != NULL )
            {
                observe = strnstrn( (char*) resp_buffer_observe, strlen( (char*) resp_buffer_observe ), "result", strlen( "result" ) );
                if ( observe == NULL )
                {
                    WPRINT_APP_INFO( ("Invalid payload: Not found [%s]\n","result") );
                    return WICED_ERROR;
                }
                observe = strnstrn( observe, strlen( observe ), pattern_start, strlen( pattern_start ) );
                if ( observe == NULL )
                {
                    WPRINT_APP_INFO( ("Invalid payload: Not found [%s]\n",pattern_start) );
                    return WICED_ERROR;
                }
                observe = observe + strlen( pattern_start );
                lineend = strnstrn( observe, strlen( observe ), pattern_end, strlen( pattern_end ) );
                if ( lineend == NULL )
                {
                    WPRINT_APP_INFO( ("Invalid payload: Not found [%s]\n",pattern_end) );
                    return WICED_ERROR;
                }
                *lineend = '\0';
                WPRINT_APP_INFO( ("\nobserve = %s\n",observe) );
                gateway_gatt_send_notification( connid, HANDLE_HTTP_PROXY_SERVICE_OBSERVE_VAL, strlen( observe ) + 1, (uint8_t*) observe );
            }
        }
    }

    http_request_deinit( &requests[ REQUEST_OBSERVE ] );
    http_client_disconnect( &client_observe );
    return result;
}

wiced_result_t sensors_http_post( uint16_t connid, char *urioption, char *header, char *payload )
{
    wiced_result_t result;
    char *status_code, *space;
    char *lineend;
    http_client_configuration_info_t client_configuration;

    /* configure HTTP client parameters */
    client_configuration.flag = (http_client_configuration_flags_t) ( HTTP_CLIENT_CONFIG_FLAG_SERVER_NAME | HTTP_CLIENT_CONFIG_FLAG_MAX_FRAGMENT_LEN );
    client_configuration.server_name = (uint8_t*) hostname;
    client_configuration.max_fragment_length = TLS_FRAGMENT_LENGTH_1024;
    http_client_configure( &client_post, &client_configuration );

    /* if you set hostname, library will make sure subject name in the server certificate is matching with host name you are trying to connect. pass NULL if you don't want to enable this check */
    client_post.peer_cn = NULL;
#ifdef CLOUD_PROTO_HTTPS
    if ( ( result = http_client_connect( &client_post, (const wiced_ip_address_t*)&ip_address, SERVER_PORT_HTTPS, HTTP_USE_TLS, CONNECT_TIMEOUT_MS ) ) != WICED_SUCCESS )
#else
    if ( ( result = http_client_connect( &client_post, (const wiced_ip_address_t*) &ip_address, SERVER_PORT_HTTP, HTTP_NO_SECURITY, CONNECT_TIMEOUT_MS ) ) != WICED_SUCCESS )
#endif
    {
        WPRINT_APP_INFO( ( "Error: failed to connect to server: %u\n", result) );
        return result;
    }

    //Ignore spaces in URI option. Just to avoid changing Home Device applications using old format
    space = strnstrn( urioption, strlen( urioption ), " ", strlen( " " ) );
    if(space != NULL)
        *space = '\0';

    http_request_init( &requests[ REQUEST_POST ], &client_post, HTTP_POST, urioption, HTTP_1_1 );

    http_request_write( &requests[ REQUEST_POST ], (uint8_t*) header, strlen( header ) );

    // Inserting two CRLF's to indicated end of header
    http_request_write_end_header( &requests[ REQUEST_POST ] );
    http_request_write_end_header( &requests[ REQUEST_POST ] );

    http_request_write( &requests[ 0 ], (uint8_t*) payload, strlen( payload ) );

    memset(resp_buffer_post, 0, BUFFER_LENGTH);

    result = http_request_flush( &requests[ REQUEST_POST ] );

    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( (" POST request flush failed\n") );
    }
    else
    {
        result = wiced_rtos_get_semaphore( &http_post_semaphore, HTTP_REQUEST_TIMEOUT_VALUE );

        if ( result != WICED_SUCCESS )
            WPRINT_APP_INFO( (" POST request timed out \n") );
        else
        {
            status_code = strnstrn( (char*) resp_buffer_post, strlen( (char*) resp_buffer_post ), "HTTP/1.1 ", strlen( "HTTP/1.1 " ) );
            status_code = status_code + strlen( "HTTP/1.1 " );
            lineend = strnstrn( status_code, strlen( status_code ), " ", strlen( " " ) );
            *lineend = '\0';

            WPRINT_APP_INFO( ("\nhttp_post : status code = %s\n", status_code) );

            gateway_gatt_send_notification( connid, HANDLE_HTTP_PROXY_SERVICE_HTTP_STATUS_CODE_VAL, strlen( status_code ) + 1, (uint8_t*) status_code );
        }
    }
    http_request_deinit( &requests[ REQUEST_POST ] );
    http_client_disconnect( &client_post );
    return result;
}

wiced_result_t http_process_request( iot_gateway_device_t *device )
{
    uint8_t *host;
    uint8_t *lineend = NULL;
    uint8_t name[ HTTP_HEADER_LEN ];
    http_proxy_info_t *msg = &( device->http_proxy_info );
    uint8_t retry_count = 0;

    if ( msg->http_proxy_control != HTTP_REQ_NULL )
    {
        /*extract host name from header*/
        host = (uint8_t*) strnstrn( (char*) msg->http_proxy_header, strlen( (char*) msg->http_proxy_header ), "Host:", strlen( "Host:" ) );
        host = host + strlen( "Host:" );
        if ( host != NULL )
        {
            memcpy( name, host, strlen( (char*) host ) );
            lineend = (uint8_t*) strnstrn( (char*) name, strlen( (char*) name ), "\r\n", strlen( "\r\n" ) );
        }
        if ( lineend != NULL )
        {
            *lineend = '\0';
        }

        WPRINT_APP_DEBUG (("Host name : %s\n",name));

        if ( strcmp( (char*) hostname, (char*) name ) )
        {
            memcpy( hostname, name, strlen( (char*) name ) );

            while ( (retry_count < MAX_RETRY_COUNT_FOR_DNS_LOOKUP) && wiced_hostname_lookup( (char*) hostname, &ip_address, 5000, WICED_STA_INTERFACE ) != WICED_SUCCESS ) //NX_WAIT_FOREVER
            {
                retry_count++;
            }

            if(retry_count == MAX_RETRY_COUNT_FOR_DNS_LOOKUP)
            {
                WICED_BT_TRACE( "DNS Lookup failed..\n" );
                return WICED_TIMEOUT;
            }

            WPRINT_APP_INFO( ( "1-->Server is at %u.%u.%u.%u\n", (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 24), (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 16), (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 8), (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 0) ) );

        }
    }

    WPRINT_APP_DEBUG (("msg->http_proxy_control = %d\n",msg->http_proxy_control));

    switch ( msg->http_proxy_control )
    {
        case HTTP_REQ_POST:
            sensors_http_post( device->conn_id, (char*) msg->http_proxy_uri, (char*) msg->http_proxy_header, (char*) msg->http_proxy_body );
            break;
        case HTTP_REQ_OBSERVE:
            http_observe( device );
            break;
        default:
            WPRINT_APP_DEBUG (("http_proxy_control event %d not handled\n", msg->http_proxy_control));
            break;
    }

    return WICED_SUCCESS;

}
