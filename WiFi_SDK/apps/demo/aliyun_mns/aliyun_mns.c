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
 * Aliyun MNS API Application
 *
 * This application demonstrates how to use the Aliyun MNS
 * to perform queue functions and transmit messages
 *
 * Usage:
 * 1. Fill in account information in aliyun_common.h
 * 2. Create buffers and http_client related variables
 * 3. Create structures to hold account information and buffers
 * 4. Initialize wiced_aliyun_t variable
 * 5. Connect to a network and start SNTP time sync
 * 6. Initialize, configure, and connect the http_client
 * 7. Create an XML structure
 * 8. Fill in XML as needed
 * 9. Call wiced_aliyun_execute_function() to execute a function (send message, create queue, etc.)
 *
 */

#include <stdlib.h>
#include "aliyun_common.h"
#include "aliyun_protocol.h"
#include "xml.h"
#include "wiced.h"
#include "sntp.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define SERVER_PORT        ( 443 )
#define DNS_TIMEOUT_MS     ( 10000 )
#define CONNECT_TIMEOUT_MS ( 3000 )

/* Increase the HTTP_TOTAL_REQUEST, if the number of commands in the application increases. */
#define HTTP_TOTAL_REQUEST (18)

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

static void event_handler( http_client_t* client, http_event_t event, http_response_t* response );
static void print_data   ( char* data, uint32_t length );
static void print_content( char* data, uint32_t length );

/******************************************************
 *               Variable Definitions
 ******************************************************/
char date_buffer[DATE_BUFFER_SIZE];
char plaintext_signature_buffer[SIGNATURE_PLAINTEXT_BUFFER_SIZE];
char signature_buffer[SIGNATURE_BUFFER_SIZE];
char resource_buffer[RESOURCE_BUFFER_SIZE];
char xmns_buffer[XMNS_BUFFER_SIZE];
char authorization_buffer[AUTHORIZATION_BUFFER_SIZE];
char content_length_buffer[CONTENT_LENGTH_BUFFER_SIZE];

wiced_aliyun_queue_t myQueue =
{
        .account_id = ALIYUN_ACCOUNT_ID,
        .region = ALIYUN_REGION,
        .access_key = ALIYUN_ACCESS_KEY,
        .secret_key = ALIYUN_SECRET_KEY,
        .queue_name = ALIYUN_QUEUE_NAME,
};

wiced_aliyun_buffers_t buffers =
{
        .xmns = xmns_buffer,
        .date_string = date_buffer,
        .sig_plain = plaintext_signature_buffer,
        .signature = signature_buffer,
        .resource = resource_buffer,
        .authorization = authorization_buffer,
        .content_length = content_length_buffer
};

static http_client_t http_client;
static http_request_t http_request[HTTP_TOTAL_REQUEST];

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    char *testmsg1 = "TEST MESSAGE 1";
    char *testmsg2 = "TEST MESSAGE 2";
    char *testmsg3 = "TEST MESSAGE 3";
    char *testmsg4 = "TEST MESSAGE 4";
    int request_index = 0;

    wiced_ip_address_t ip_address;
    wiced_result_t result;

    wiced_init( );
    // Bring up network and resolve IP of host
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
    WPRINT_APP_INFO( ( "Resolving IP address of %s\n", ALIYUN_HOST ) );
    wiced_hostname_lookup( ALIYUN_HOST, &ip_address, DNS_TIMEOUT_MS, WICED_STA_INTERFACE );
    WPRINT_APP_INFO( ( "%s is at %u.%u.%u.%u\n", ALIYUN_HOST,
            (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 24),
            (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 16),
            (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 8),
            (uint8_t)(GET_IPV4_ADDRESS(ip_address) >> 0) ) );

    // Connect to SNTP server
    sntp_start_auto_time_sync( 1 * HOURS_MS );

    // Create XML buffer and initialize
    wiced_xml_t xml;
    char xml_buffer[XML_BUFFER_SIZE];
    wiced_xml_init( &xml, (char*)&xml_buffer, XML_BUFFER_SIZE );

    // Create HTTP headers, client configuration
    http_header_field_t http_headers[HTTP_HEADERS_NUM_MAX];
    http_client_init( &http_client, WICED_STA_INTERFACE, event_handler, NULL );
    http_client_configuration_info_t client_configuration;
    client_configuration.flag = HTTP_CLIENT_CONFIG_FLAG_SERVER_NAME | HTTP_CLIENT_CONFIG_FLAG_MAX_FRAGMENT_LEN;
    client_configuration.server_name = (uint8_t*)ALIYUN_HOST;
    client_configuration.max_fragment_length = TLS_FRAGMENT_LENGTH_4096;
    http_client_configure(&http_client, &client_configuration);

    // Connect client
    result = http_client_connect( &http_client, (const wiced_ip_address_t*)&ip_address, SERVER_PORT, HTTP_USE_TLS, CONNECT_TIMEOUT_MS );
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("http_connect failed\r\n"));
        return;
    }
    // Create main wiced_aliyun_t variable and initialize
    wiced_aliyun_t aliyun;
    wiced_aliyun_init( &aliyun, &myQueue, &buffers, &xml, (http_header_field_t *)&http_headers, &http_client );

    WPRINT_APP_INFO( ( "\n\n\n\n////====STARTING ALIYUN TEST====\\\\\\\\\n\n\n" ) );
    WPRINT_APP_INFO( ( "===Aliyun Config===\n" ) );
    WPRINT_APP_INFO( ( "Account ID: %s\n", aliyun.wiced_aliyun_queue->account_id ) );
    WPRINT_APP_INFO( ( "Region: %s\n", aliyun.wiced_aliyun_queue->region ) );
    WPRINT_APP_INFO( ( "Access Key: %s\n", aliyun.wiced_aliyun_queue->access_key ) );
    WPRINT_APP_INFO( ( "Secret Key: %s\n", aliyun.wiced_aliyun_queue->secret_key ) );
    WPRINT_APP_INFO( ( "Queue Name: %s\n", aliyun.wiced_aliyun_queue->queue_name ) );
    WPRINT_APP_INFO( ( "=========\n\n" ) );

    // List queues
    WPRINT_APP_INFO(("****LIST QUEUES****\n"));
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], LISTQUEUE, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(200);

    //Create Queue
    WPRINT_APP_INFO(("****CREATE QUEUE****\n"));
    wiced_xml_start(aliyun.wiced_xml,CREATEQUEUE);
    wiced_xml_add_parameter(aliyun.wiced_xml, XML_DELAY_SECONDS, "0");
    wiced_xml_end(aliyun.wiced_xml,CREATEQUEUE);
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], CREATEQUEUE, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    //List queues (verify queue creation)
    WPRINT_APP_INFO(("****LIST QUEUES****\n"));
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], LISTQUEUE, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    //Get attributes of Queue
    WPRINT_APP_INFO(("****GET QUEUE ATTRIBUTES****\n"));
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], GETQUEUEATTRIBUTES, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    //Set attributes of Queue
    WPRINT_APP_INFO(("****SET QUEUE ATTRIBUTES****\n"));
    wiced_xml_start(aliyun.wiced_xml,SETQUEUEATTRIBUTES);
    wiced_xml_add_parameter(aliyun.wiced_xml, XML_VISIBILITY_TIMEOUT, "5");
    wiced_xml_end(aliyun.wiced_xml,SETQUEUEATTRIBUTES);
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], SETQUEUEATTRIBUTES, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    //Get attributes of queue again (verify change)
    WPRINT_APP_INFO(("****GET QUEUE ATTRIBUTES****\n"));
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], GETQUEUEATTRIBUTES, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    // SEND MESSAGE 1
    WPRINT_APP_INFO(("****SEND TEST MESSAGE 1****\n"));
    wiced_xml_start(aliyun.wiced_xml, SENDMESSAGE);
    wiced_xml_add_parameter(aliyun.wiced_xml, XML_MESSAGE_BODY, testmsg1 );
    wiced_xml_end(aliyun.wiced_xml, SENDMESSAGE);
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], SENDMESSAGE, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    // SEND A MESSAGE 2
    WPRINT_APP_INFO(("****SEND TEST MESSAGE 2****\n"));
    wiced_xml_start(aliyun.wiced_xml, SENDMESSAGE);
    wiced_xml_add_parameter(aliyun.wiced_xml, XML_MESSAGE_BODY, testmsg2 );
    wiced_xml_end(aliyun.wiced_xml, SENDMESSAGE);
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], SENDMESSAGE, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    // RECEIVE MESSAGE 1
    WPRINT_APP_INFO(("****RECEIVE TEST MESSAGE 1****\n"));
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], RECEIVEMESSAGE, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    // RECEIVE MESSAGE 2
    WPRINT_APP_INFO(("****RECEIVE TEST MESSAGE 2****\n"));
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], RECEIVEMESSAGE, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    //DELAY
    WPRINT_APP_INFO( ("****WAIT FOR VISIBILITY TIMEOUT TO EXPIRE (5s)****\n\n") );
    wiced_rtos_delay_milliseconds(5000);

    // PEEK MESSAGE 1
    WPRINT_APP_INFO(("****PEEK TEST MESSAGE 1****\n"));
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], PEEKMESSAGE, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    // RECEIVE MESSAGE 1
    WPRINT_APP_INFO(("****RECEIVE TEST MESSAGE 1****\n"));
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], RECEIVEMESSAGE, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    // RECEIVE MESSAGE 2
    WPRINT_APP_INFO(("****RECEIVE TEST MESSAGE 2****\n"));
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], RECEIVEMESSAGE, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    // BATCH SEND MESSAGE
    WPRINT_APP_INFO(("****BATCH SEND MESSAGE****\n"));
    wiced_xml_start( aliyun.wiced_xml, BATCHSENDMESSAGE );
    wiced_xml_start_message( aliyun.wiced_xml );
    wiced_xml_add_parameter( aliyun.wiced_xml, XML_MESSAGE_BODY, testmsg3 );
    wiced_xml_end_message( aliyun.wiced_xml );
    wiced_xml_start_message(aliyun.wiced_xml);
    wiced_xml_add_parameter(aliyun.wiced_xml, XML_MESSAGE_BODY, testmsg4 );
    wiced_xml_end_message(aliyun.wiced_xml);
    wiced_xml_end(aliyun.wiced_xml, BATCHSENDMESSAGE);
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], BATCHSENDMESSAGE, NULL, NULL, NULL);
    wiced_rtos_delay_milliseconds(300);

    //DELAY
    WPRINT_APP_INFO( ("****WAIT FOR VISIBILITY TIMEOUT TO EXPIRE (5s)****\n\n") );
    wiced_rtos_delay_milliseconds(5000);

    // BATCH PEEK MESSAGE
    WPRINT_APP_INFO(("****BATCH PEEK MESSAGE****\n"));
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], BATCHPEEKMESSAGE, "10", NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    // BATCH RECEIVE MESSAGE
    WPRINT_APP_INFO(("****BATCH RECEIVE MESSAGE****\n"));
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], BATCHRECEIVEMESSAGE, "10", NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    // Delete Queue
    WPRINT_APP_INFO(("****DELETE QUEUE****\n"));
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], DELETEQUEUE, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    //List queues (verify queue creation)
    WPRINT_APP_INFO(("****LIST QUEUES****\n"));
    wiced_aliyun_execute_function( &aliyun, &http_request[request_index++], LISTQUEUE, NULL, NULL, NULL );
    wiced_rtos_delay_milliseconds(300);

    WPRINT_APP_INFO( ( "\n\n\n\n////====ALIYUN TEST END====\\\\\\\\\n\n\n" ) );
}

static void event_handler( http_client_t* client, http_event_t event, http_response_t* response )
{
    WPRINT_APP_INFO (("event %d\r\n", event));
    switch( event )
    {
    case HTTP_CONNECTED:
        WPRINT_APP_INFO(( "Connected to %s\n", ALIYUN_HOST ));
        break;

    case HTTP_DISCONNECTED:
    {
        int request_index = 0;

        WPRINT_APP_INFO(( "Disconnected from %s\n", ALIYUN_HOST ));
        for ( request_index = 0; request_index < HTTP_TOTAL_REQUEST; request_index++ )
        {
            http_request_deinit( &http_request[request_index] );
        }

        break;
    }

    case HTTP_DATA_RECEIVED:
    {
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
        if(response->remaining_length == 0)
        {
            WPRINT_APP_INFO( ( "Received total payload data for response1 \n" ) );
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
