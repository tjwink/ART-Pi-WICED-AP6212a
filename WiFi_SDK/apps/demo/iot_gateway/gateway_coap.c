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
 * * Handles http proxy data and sends to cloud server using CoAP protocol stack
 *
 */

#include "wiced.h"
#include "client/coap_client.h"
#include "gateway.h"
#include "cloudconfig.h"
#include "httpproxy.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define COAP_TARGET_PORT          5683
#define WICED_COAP_TIMEOUT       (10000)

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_result_t coap_receive_callback( wiced_coap_client_event_info_t event_info );
static wiced_result_t coap_wait_for( wiced_coap_client_event_type_t event, uint32_t timeout );
static wiced_result_t coap_write( uint16_t connid, uint8_t *coap_cik, uint8_t *alias, uint8_t *writedata );
wiced_result_t coap_observe( uint16_t connid, uint8_t *coap_cik, uint8_t *alias );
static void coap_setup( wiced_coap_client_request_t* request, char* service_name, char* cid );
wiced_result_t init_coap_client( void );
wiced_result_t coap_process_request( iot_gateway_device_t *device );

/******************************************************
 *               Variables Definitions
 ******************************************************/
static wiced_coap_client_t* client;
static wiced_coap_client_event_type_t expected_event;
static wiced_semaphore_t send_semaphore;
static wiced_coap_token_info_t token;
static uint16_t observe_connid;

extern wiced_ip_address_t ip_address;

/******************************************************
 *               Function Definitions
 ******************************************************/

static void coap_setup( wiced_coap_client_request_t* request, char* service_name, char* cid )
{
    memset( &request->options, 0, sizeof(wiced_coap_option_info_t) );

    wiced_coap_set_uri_path( &request->options, (char*) "1a" );
    wiced_coap_set_uri_path( &request->options, service_name );
    wiced_coap_set_uri_query( &request->options, cid );
}

static wiced_result_t coap_write( uint16_t connid, uint8_t *coap_cik, uint8_t *alias, uint8_t *writedata )
{
    wiced_coap_client_request_t request;
    char *status_timeout = "408"; //request timeout
    char *status_ok = "200"; //ok
    char status_code[ 5 ];

    memset( &request, 0, sizeof(wiced_coap_client_request_t) );

    coap_setup( &request, (char*) alias, (char*) coap_cik );

    memset( &request.payload, 0, sizeof(wiced_coap_buffer_t) );

    request.payload_type = WICED_COAP_CONTENTTYPE_TEXT_PLAIN;

    request.payload.data = writedata;
    request.payload.len = strlen( (char*) writedata );

    WPRINT_APP_DEBUG (("write data > %s\n",request.payload.data));

    wiced_coap_client_post( client, &request, WICED_COAP_MSGTYPE_CON, ip_address, COAP_TARGET_PORT );

    if ( coap_wait_for( WICED_COAP_CLIENT_EVENT_TYPE_POSTED, WICED_COAP_TIMEOUT ) != WICED_SUCCESS )
    {
        memcpy( status_code, status_timeout, ( strlen( status_timeout ) + 1 ) );
    }
    else
    {
        memcpy( status_code, status_ok, ( strlen( status_ok ) + 1 ) );
    }

    gateway_gatt_send_notification( connid, HANDLE_HTTP_PROXY_SERVICE_HTTP_STATUS_CODE_VAL, strlen( status_code ) + 1, (uint8_t*) status_code );

    return WICED_SUCCESS;
}

wiced_result_t coap_observe( uint16_t conn_id, uint8_t *coap_cik, uint8_t *alias )
{
    wiced_coap_client_request_t request;
    uint16_t value;

    //update the global value
    observe_connid = conn_id;

    value = rand( );

    memset( &request, 0, sizeof(wiced_coap_client_request_t) );

    coap_setup( &request, (char*) alias, (char*) coap_cik );

    memcpy( token.data, &value, sizeof( value ) );
    token.token_len = 2;

    wiced_coap_client_observe( client, &request, WICED_COAP_MSGTYPE_CON, &token, ip_address, COAP_TARGET_PORT );

    WPRINT_APP_DEBUG (("coap_wait_for event %d\n",WICED_COAP_CLIENT_EVENT_TYPE_OBSERVED));

    return WICED_SUCCESS;
}

/*
 * A blocking call to an expected event.
 */
static wiced_result_t coap_wait_for( wiced_coap_client_event_type_t event, uint32_t timeout )
{
    WPRINT_APP_DEBUG (("wait for event %d timeout %d\n",event, timeout));

    if ( wiced_rtos_get_semaphore( &send_semaphore, timeout ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("wiced_rtos_get_semaphore timeout\n") );
        return WICED_ERROR;
    }
    else
    {
        WPRINT_APP_INFO( ("wiced_rtos_get_semaphore break %d event %d expevent %d\r\n",(int)timeout,(int)event,(int)expected_event) );
        if ( event != expected_event )
        {
            return WICED_ERROR;
        }
    }

    return WICED_SUCCESS;
}

static wiced_result_t coap_receive_callback( wiced_coap_client_event_info_t event_info )
{
    switch ( event_info.type )
    {
        case WICED_COAP_CLIENT_EVENT_TYPE_POSTED:
            WPRINT_APP_DEBUG (("Data written successfully to Exosite Cloud\n"));
            expected_event = event_info.type;
            wiced_rtos_set_semaphore( &send_semaphore );
            break;
        case WICED_COAP_CLIENT_EVENT_TYPE_GET_RECEIVED:
            WPRINT_APP_DEBUG (("Data Received from Exosite cloud\n"));
            WPRINT_APP_DEBUG (("Payload value : %s\n", event_info.payload.data ));
            expected_event = event_info.type;
            wiced_rtos_set_semaphore( &send_semaphore );
            break;
        case WICED_COAP_CLIENT_EVENT_TYPE_OBSERVED:
            WPRINT_APP_INFO( ("Observing to service is successfully\n") );
            expected_event = event_info.type;
            wiced_rtos_set_semaphore( &send_semaphore );
            break;
        case WICED_COAP_CLIENT_EVENT_TYPE_NOTIFICATION:
            WPRINT_APP_INFO( ("Notification from Exosite cloud\n") );
            WPRINT_APP_INFO( ( "Payload value : %s\n", event_info.payload.data ) );
            expected_event = event_info.type;
            wiced_rtos_set_semaphore( &send_semaphore );
            /* send notification to sensor device*/

            gateway_gatt_send_notification( observe_connid, HANDLE_HTTP_PROXY_SERVICE_OBSERVE_VAL, strlen( (char*) event_info.payload.data ) + 1, event_info.payload.data );

            break;
        default:
            WPRINT_APP_INFO( ("Wrong event %d\n",event_info.type) );
            break;
    }

    return WICED_SUCCESS;
}

wiced_result_t init_coap_client( )
{
    /* Memory allocated for CoAP client object*/
    client = (wiced_coap_client_t*) malloc( WICED_COAP_OBJECT_MEMORY_SIZE_REQUIREMENT );
    if ( client == NULL )
    {
        WPRINT_APP_INFO( ("Dont have memory to allocate for CoAP client object...\n") );
        return WICED_ERROR;
    }

    //initialize token
    memset( &token, 0, sizeof(wiced_coap_token_info_t) );

    if ( wiced_coap_client_init( client, WICED_STA_INTERFACE, coap_receive_callback ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("coap client init failed\n") );
        return WICED_ERROR;
    }

    if ( wiced_rtos_init_semaphore( &send_semaphore ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("Semaphore Create Failed\n") );
        return WICED_ERROR;
    }
    /* for Exosite CoAP there is no header element for host name. So URI is hard coded */
    ip_address.ip.v4 = MAKE_IPV4_ADDRESS( 104, 237, 152, 179 ); //coap.exosite.com
    ip_address.version = WICED_IPV4;

    WPRINT_APP_INFO( ( "CoAP Server is at %u.%u.%u.%u\n", (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 24), (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 16), (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 8), (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 0) ) );

    return WICED_SUCCESS;

}

wiced_result_t coap_process_request( iot_gateway_device_t *device )
{
    uint8_t *lineend;
    uint8_t *payload;

    http_proxy_info_t *msg = &( device->http_proxy_info );

    /* parse payload */
    char *pattern0 = "cik\":\"";
    char *pattern1 = "alias\":\"";
    char *pattern3 = "\"]";
    uint8_t coap_cik[ 50 ];
    uint8_t coap_alias[ 20 ];

    payload = (uint8_t*) strnstrn( (char*) msg->http_proxy_body, strlen( (char*) msg->http_proxy_body ), pattern0, strlen( pattern0 ) );
    payload = payload + strlen( pattern0 );
    strncpy( (char*) coap_cik, (char*) payload, 50 );
    lineend = (uint8_t*) strnstrn( (char*) coap_cik, 50, "\"}", strlen( "\"}" ) );
    *lineend = '\0';
    WPRINT_APP_DEBUG (("coap_cik = %s\n",coap_cik));

    payload = (uint8_t*) strnstrn( (char*) msg->http_proxy_body, strlen( (char*) msg->http_proxy_body ), pattern1, strlen( pattern1 ) );
    payload = payload + strlen( pattern1 );
    strncpy( (char*) coap_alias, (char*) payload, 20 );
    lineend = (uint8_t*) strnstrn( (char*) coap_alias, 20, "\"}", strlen( "\"}" ) );
    *lineend = '\0';
    WPRINT_APP_DEBUG (("alias = %s\n",coap_alias));

    payload = (uint8_t*) strnstrn( (char*) payload, strlen( (char*) payload ), "},\"", strlen( "},\"" ) );
    payload = payload + strlen( "},\"" );
    lineend = (uint8_t*) strnstrn( (char*) payload, strlen( (char*) payload ), pattern3, strlen( pattern3 ) );
    *lineend = '\0';

    WPRINT_APP_DEBUG (("payload = %s\n",payload));

    WPRINT_APP_DEBUG (("msg->http_proxy_control = %d\n",msg->http_proxy_control));

    switch ( msg->http_proxy_control )
    {
        case HTTP_REQ_POST:
            if ( coap_write( device->conn_id, coap_cik, coap_alias, payload ) != WICED_SUCCESS )
            {
                WPRINT_APP_INFO( ( "write failed\n" ) );
            }
            break;
        case HTTP_REQ_OBSERVE:
            coap_observe( device->conn_id, coap_cik, coap_alias );
            break;
        default:
            WPRINT_APP_INFO( ("http_proxy_control event %d not handled\n", msg->http_proxy_control) );
            break;
    }

    return WICED_SUCCESS;

}
