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
 * Demo App : WICED CoAP client with Exosite cloud.
 *
 * This snippet application demonstrates that how you can communicate with Exosite cloud with WICED device
 * by using CoAP functionality. You can read, write and observe for particular service with Exosite cloud.
 *
 * Please refer below link to create account on Exosite cloud and setup your account.
 * https://portals.exosite.com/login
 *
 * connect WICED device to any access point. and configure below parameters.
 *
 * char* hostname = "coap.exosite.com"                     // hostname of Exosite cloud
 * char* cid = "f5034cee0cdcc5d35c162cb36d83eeddcad0728a"  // CID number you will get at time of configuration of exosite.
 * char* alias = "COAP";                                   // Alias name you created on Exosite cloud.
 *
 * Demo application in which WICED device will write "10" value to exosite cloud on COAP alias. then it will read back same value and will
 * show in putty. Then WICED device will observe for alias COAP so you will receive latest state or value of COAP alias from cloud. Now change
 * value for COAP alias as "1" in exosite cloud interface, you will receive notification in WICED and LED will be turned on. you can turn OFF LED
 * changing the value of alias COAP to any value [ other than "1" ] in exosite cloud.
 *
 * Known limitation : after few notifications, no new notification received in client.
 *
 */

#include "client/coap_client.h"
#include "wiced.h"
#include "wiced_crypto.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define COAP_TARGET_PORT          5683
#define WICED_COAP_TIMEOUT       (10000)

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
static wiced_result_t coap_receive_callback( wiced_coap_client_event_info_t event_info );
static wiced_result_t coap_wait_for( wiced_coap_client_event_type_t event, uint32_t timeout );
static wiced_result_t write( );
static wiced_result_t read( );
static wiced_result_t observe( );
static void exosite_setup( wiced_coap_client_request_t* request, char* service_name, char* cid );

/******************************************************
 *               Variables Definitions
 ******************************************************/
char* hostname = "coap.exosite.com";
/* Replace cid generated by exosite cloud and alias name created by you */
char* cid = "9874f62028b9d8595ff6f759d5d311dde2e7272b";
char* alias = "COAP";
/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_coap_client_t* client;
wiced_ip_address_t ip;
static wiced_coap_client_event_type_t expected_event;
static wiced_semaphore_t semaphore;
wiced_coap_token_info_t token;

void application_start( void )
{
    /* Initialise the device and WICED framework */
    wiced_init( );

    /* Memory allocated for CoAP client object*/
    client = (wiced_coap_client_t*) malloc( WICED_COAP_OBJECT_MEMORY_SIZE_REQUIREMENT );
    if ( client == NULL )
    {
        WPRINT_APP_ERROR(("Dont have memory to allocate for CoAP client object...\n"));
        return;
    }

    /* Bring up the softAP and network interface */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    wiced_hostname_lookup( hostname, &ip, 5000, WICED_STA_INTERFACE );

    WPRINT_APP_INFO( ( "Server is at %u.%u.%u.%u\n", (uint8_t)(GET_IPV4_ADDRESS(ip) >> 24),
                    (uint8_t)(GET_IPV4_ADDRESS(ip) >> 16),
                    (uint8_t)(GET_IPV4_ADDRESS(ip) >> 8),
                    (uint8_t)(GET_IPV4_ADDRESS(ip) >> 0) ) );

    wiced_coap_client_init( client, WICED_STA_INTERFACE, coap_receive_callback );

    wiced_rtos_init_semaphore( &semaphore );

    if ( write( ) != WICED_SUCCESS )
    {
        printf( "write failed\n" );
    }

    if ( read( ) != WICED_SUCCESS )
    {
        printf( "read failed\n" );
    }

    if ( observe( ) != WICED_SUCCESS )
    {
        printf( "Observe failed\n" );
    }

    wiced_rtos_delay_microseconds( 20000 );

}

static void exosite_setup( wiced_coap_client_request_t* request, char* service_name, char* cid )
{
    wiced_coap_set_uri_path( &request->options, (char*) "1a" );
    wiced_coap_set_uri_path( &request->options, alias );
    wiced_coap_set_uri_query( &request->options, cid );
}

static wiced_result_t write( )
{
    wiced_coap_client_request_t request;

    memset( &request, 0, sizeof( request ) );

    wiced_crypto_get_random( &token, sizeof( token ) );

    exosite_setup( &request, alias, cid );

    request.payload_type = WICED_COAP_CONTENTTYPE_TEXT_PLAIN;
    request.payload.data = (uint8_t*) "10";
    request.payload.len = 2;

    wiced_coap_client_post( client, &request, WICED_COAP_MSGTYPE_CON, ip, COAP_TARGET_PORT );

    if ( coap_wait_for( WICED_COAP_CLIENT_EVENT_TYPE_POSTED, WICED_COAP_TIMEOUT ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

static wiced_result_t read( )
{
    wiced_coap_client_request_t request;

    memset( &request, 0, sizeof( request ) );

    exosite_setup( &request, alias, cid );

    wiced_coap_client_get( client, &request, WICED_COAP_MSGTYPE_CON, ip, COAP_TARGET_PORT );

    if ( coap_wait_for( WICED_COAP_CLIENT_EVENT_TYPE_GET_RECEIVED, WICED_COAP_TIMEOUT ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

static wiced_result_t observe( )
{
    wiced_coap_client_request_t request;
    uint16_t value;

    wiced_crypto_get_random( &value, sizeof( value ) );
    memset( &request, 0, sizeof( request ) );

    exosite_setup( &request, alias, cid );

    memcpy( token.data, &value, sizeof( value ) );
    token.token_len = 2;

    wiced_coap_client_observe( client, &request, WICED_COAP_MSGTYPE_CON, &token, ip, COAP_TARGET_PORT );

    if ( coap_wait_for( WICED_COAP_CLIENT_EVENT_TYPE_OBSERVED, WICED_COAP_TIMEOUT ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/*
 * A blocking call to an expected event.
 */
static wiced_result_t coap_wait_for( wiced_coap_client_event_type_t event, uint32_t timeout )
{
    if ( wiced_rtos_get_semaphore( &semaphore, timeout ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    else
    {
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
            WPRINT_APP_INFO(("Data written successfully to Exosite Cloud\n"));
            expected_event = event_info.type;
            wiced_rtos_set_semaphore( &semaphore );
            break;
        case WICED_COAP_CLIENT_EVENT_TYPE_GET_RECEIVED:
            WPRINT_APP_INFO(("Data Received from Exosite cloud\n"));
            printf( "Payload value : %s\n", event_info.payload.data );
            expected_event = event_info.type;
            wiced_rtos_set_semaphore( &semaphore );
            break;
        case WICED_COAP_CLIENT_EVENT_TYPE_OBSERVED:
            WPRINT_APP_INFO(("Observing to service is successfully\n"));
            expected_event = event_info.type;
            wiced_rtos_set_semaphore( &semaphore );
            break;
        case WICED_COAP_CLIENT_EVENT_TYPE_NOTIFICATION:
            WPRINT_APP_INFO(("Notification from Exosite cloud\n"));
            printf( "Payload value : %s\n", event_info.payload.data );
            if ( ( strcmp ( (char*) event_info.payload.data, "1") ) == 0 )
            {
                wiced_gpio_output_high( WICED_LED1 );
            }
            else
            {
                wiced_gpio_output_low( WICED_LED1 );
            }
            break;
        default:
            WPRINT_APP_INFO(("Wrong event\n"));
            break;
    }

    return WICED_SUCCESS;
}
