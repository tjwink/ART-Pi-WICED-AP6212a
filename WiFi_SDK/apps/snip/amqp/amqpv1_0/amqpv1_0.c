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
 */

#include "wiced.h"
#include "amqp.h"
#include "resources.h"


/** @file
 *
 * Sample AMQP Application
 *
 * This application snippet demonstrates how to use
 *  -- the WICED AMQP (Advanced Message Queuing Protocol)1.0 client library.
 *  -- Enable AMQP_TLS_SECURITY_ENABLE macro to test in secure environment.
 *  -- Disable by commenting the macro AMQP_TLS_SECURITY_ENABLE to test in non secure environment.
 *
 * SECURITY:
 *  Security in AMQP is done through SSL and/or SASL. This application  has support to test AMQP over SSL.
 *  To setup a secure RabbitMQ SSL server, see https://www.rabbitmq.com/ssl.html.
 *  For testing with the already provided client certificate/key, user can the following certificate/key for the server
 *  (Already signed by the  CA certificate).
 *
 *  Use root ca certificate from the resource folder
 *
 *  #########server certificate##########
 *  -----BEGIN CERTIFICATE-----
 *  MIID+DCCA2GgAwIBAgIJALVGBV3GrJvvMA0GCSqGSIb3DQEBCwUAMF8xEzARBgNV
 *  BAMMCldJQ0VTIFRFU1QxEjAQBgNVBAoMCXdpY2VkLmNvbTEUMBIGA1UECwwLZ2Vu
 *  ZXJhdGUtQ0ExHjAcBgkqhkiG9w0BCQEWD3dpY2VkQHdpY2VkLmNvbTAeFw0xNzA5
 *  MjYxMjQ1NDBaFw0zMjA5MjIxMjQ1NDBaMFsxDzANBgNVBAMMBnNlcnZlcjESMBAG
 *  A1UECgwJd2ljZWQuY29tMRQwEgYDVQQLDAtnZW5lcmF0ZS1DQTEeMBwGCSqGSIb3
 *  DQEJARYPd2ljZWRAd2ljZWQuY29tMIGfMA0GCSqGSIb3DQEBAQUAA4GNADCBiQKB
 *  gQDTKJhtm3o6JMXutu6xCPOXxsWhYhoz7VpyXTlsL2LSeKwP0df2HgNsbyAfaoOT
 *  ndIw77atvDBNjjwqZb4oLqxhq1ZZK6WtSTeFr5smG/1Wlv+3//bacB9JhC1FcY09
 *  m5o4mJA8/8F78XVNYCOwE8gPmHTSui2sPG5PO9A4FrTsewIDAQABo4IBvjCCAbow
 *  DAYDVR0TAQH/BAIwADARBglghkgBhvhCAQEEBAMCBkAwCwYDVR0PBAQDAgXgMCEG
 *  CWCGSAGG+EIBDQQUFhJCcm9rZXIgQ2VydGlmaWNhdGUwHQYDVR0OBBYEFDlB99HP
 *  ee86SBCd1jUc6iKJkvOmMIGRBgNVHSMEgYkwgYaAFLP9T+mRPngmbmm5zLWua5XK
 *  EnAKoWOkYTBfMRMwEQYDVQQDDApXSUNFUyBURVNUMRIwEAYDVQQKDAl3aWNlZC5j
 *  b20xFDASBgNVBAsMC2dlbmVyYXRlLUNBMR4wHAYJKoZIhvcNAQkBFg93aWNlZEB3
 *  aWNlZC5jb22CCQDXSVxrY/fHfjBEBgNVHREEPTA7hwQKKAJphxD+gAAAAAAAADLj
 *  daK6xziWhwR/AAABhxAAAAAAAAAAAAAAAAAAAAABgglsb2NhbGhvc3QwbgYDVR0g
 *  BGcwZTBjBgMrBQgwXDAcBggrBgEFBQcCARYQaHR0cDovL2xvY2FsaG9zdDA8Bggr
 *  BgEFBQcCAjAwMAwWBVdJQ0VEMAMCAQEaIFRoaXMgQ0EgaXMgZm9yIGEgdGVzdGlu
 *  ZyBwdXJwb3NlMA0GCSqGSIb3DQEBCwUAA4GBAAm8/HNYmmWl4PpBn1Kj2EaWwb6A
 *  t1fWhAArWOeDfcpKuU2KEjNMyDLBwaEoFnqalWDTxEPiVhGhdM2ocx0GBKqMxUI5
 *  CRJ2ghRvLfijL44zftEhg6S5GTC2Ktn7CRhrwuTSnIwxYaa+aWGRJ/lV0z9sP7JI
 *  t5qAlaAmxeUdZ/9W
 *  -----END CERTIFICATE-----
 *
 *  #########server private key##########
 *  -----BEGIN RSA PRIVATE KEY-----
 *  MIICXQIBAAKBgQDTKJhtm3o6JMXutu6xCPOXxsWhYhoz7VpyXTlsL2LSeKwP0df2
 *  HgNsbyAfaoOTndIw77atvDBNjjwqZb4oLqxhq1ZZK6WtSTeFr5smG/1Wlv+3//ba
 *  cB9JhC1FcY09m5o4mJA8/8F78XVNYCOwE8gPmHTSui2sPG5PO9A4FrTsewIDAQAB
 *  AoGBALN4b4XJfetpUeoBBYLmztOTmGoATbEQ7a0CW0n+RIoLEoMnodyHyfUhxjWt
 *  fEO9Aeeh9qxXpN5mI/ENJMBWWx/k0vvtdMhQ+b5ObHU+ozxJH3p2JY7o6a6z6OFq
 *  vlNz8zCAITh+BCiOT1ot1ze+vh8ccpirsdXXQDHki3aUTbyxAkEA8Y9aYZ0n71AP
 *  R2N8IA2illPgsRiktv0/ghd8XNeDq4lDCuhVhj9nvXBo7LRG8CcKFKnzauWzFoDy
 *  op03LiDuLQJBAN/IAL65YXxrQecm+4KVmeV29nOWy0x3Eu0QwGDF4BdugHQMg7OU
 *  KBCApexKlyKN3dcsIBLVb8y65x/MGw9nFkcCQHfPCxvpL5pxkfJtdG4NdTusRIBx
 *  4ZhlCS/D6EnAFq1ouhjZ3Tllj1WVQGVOkPSh2E0hcfruDKI3uKBQ68J0UM0CQArp
 *  uf+TEsn6gRBAums32HV0Q7iHVgq4k9ezxW3yuGbsIJ+ILLQJOZr0ayMG0DADMxpX
 *  MIk6l2UoiDTowzkREekCQQC3nj5INpsD6dmMnRRC6fn7CEcDa7c5tQhzQKLDNMkK
 *  POqJ1vqE4eUUssSBMPKDkVJrulR2eXZDtLzgIBxZwNGL
 *  -----END RSA PRIVATE KEY-----
 *
 */
/******************************************************
 *                      Macros
 ******************************************************/
static void  amqp_print_status ( wiced_result_t result, const char * ok_message, const char * error_message );


#define RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( command, ok_message, error_message )   \
        {                                                                                   \
            ret = (command);                                                                \
            amqp_print_status( ret, (const char *)ok_message, (const char *)error_message );\
            if ( ret != WICED_SUCCESS ) break;                                              \
        }


/******************************************************
 *                    Constants
 ******************************************************/
/* Change the IP address to match the server address */
#define AMQP_BROKER_IP_ADDRESS MAKE_IPV4_ADDRESS(192,168,1,9)
/* AMQP message content */
#define AMQP_MESSAGE_STR      "HELLO WICED"

/* uncomment below line to enable AMQP over TLS */
//#define AMQP_TLS_SECURITY_ENABLE

#define AMQP_MAX_RESOURCE_SIZE      (0x7fffffff)

/* uncomment below line to enable AMQP receive link */
#define AMQP_WITH_RECEIVE_LINK
/* uncomment below line to enable AMQP send link */
#define AMQP_WITH_SEND_LINK


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
static wiced_result_t wiced_amqp_connection_event_cb( wiced_amqp_event_t event, void *arg, void *conn );
static wiced_result_t amqp_wait_for                 ( wiced_amqp_event_t event, uint32_t timeout );
static wiced_result_t amqp_connect( wiced_amqp_connection_instance *connection_instance );
static wiced_result_t amqp_begin_session( wiced_amqp_session_instance_t* session_instance, wiced_amqp_connection_instance *connection_instance);

#if defined(AMQP_WITH_RECEIVE_LINK)|| defined(AMQP_WITH_SEND_LINK)
static wiced_result_t amqp_attach_link( wiced_amqp_link_instance* link_instance, wiced_amqp_session_instance_t* session_instance, char *source, char *target, wiced_amqp_role_types_t role );
#endif

static wiced_result_t amqp_conn_close( wiced_amqp_connection_instance *connection_instance );

#ifdef AMQP_WITH_SEND_LINK
static wiced_result_t amqp_send( wiced_amqp_link_instance* link_instance, wiced_amqp_message_t* message );
#endif

#if defined(AMQP_WITH_RECEIVE_LINK)|| defined(AMQP_WITH_SEND_LINK)
static wiced_result_t amqp_detach( wiced_amqp_link_instance *link_instance );
#endif

static wiced_result_t amqp_end( wiced_amqp_session_instance_t *session_instance );

/******************************************************
 *               Variable Definitions
 ******************************************************/
static const wiced_ip_address_t  INITIALISER_IPV4_ADDRESS( broker_address, AMQP_BROKER_IP_ADDRESS );
static wiced_amqp_callbacks_t    callbacks =
{
        .connection_event = wiced_amqp_connection_event_cb,
};
static wiced_amqp_event_t        expected_event;
static wiced_semaphore_t         semaphore;

#if defined(AMQP_WITH_RECEIVE_LINK)|| defined(AMQP_WITH_SEND_LINK)
static uint32_t                  handle_count;
uint32_t count = 1;
#endif

#ifdef AMQP_TLS_SECURITY_ENABLE
    static wiced_amqp_security_t     security;
#endif

#ifdef AMQP_WITH_RECEIVE_LINK
    wiced_amqp_link_instance link_instance_receiver;
    uint32_t message_count = 0;
#endif

#ifdef AMQP_WITH_SEND_LINK
    wiced_amqp_link_instance link_instance_sender;
    wiced_amqp_message_t message;
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Main AMQP thread.
 *
 * The thread will create a connection and then loop( single iteration) over the following:
 * - Open a connection
 * - Begin a session
 * - Attach a send link based on macro.
 * - Attach a receive link based on macro.
 * - Transfer sample data.
 * - Displays received data.
 * - Detach a receive link based on macro.
 * - Detach a send link based on macro.
 * - Ends a Session.
 * - Close the connection
 */
void application_start( void )
{
    wiced_result_t ret = WICED_SUCCESS;
    wiced_amqp_connection_instance connection_instance;
    wiced_amqp_session_instance_t session_instance;


#ifdef AMQP_TLS_SECURITY_ENABLE
    uint32_t        size_out;
#endif

    wiced_init( );

    /* Bringup the network interface */
    ret = wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
    if ( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "Unable to do network up\r\n"));
        return;
    }

    WPRINT_APP_INFO( ("[AMQP] Connecting to broker %u.%u.%u.%u ...", (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 24), (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 16), (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 8), (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 0)) );

    connection_instance.conn = malloc( sizeof(wiced_amqp_connection_t) );
    if ( connection_instance.conn != NULL )
    {
        memset( connection_instance.conn, 0, sizeof(wiced_amqp_connection_t) );
#ifdef AMQP_TLS_SECURITY_ENABLE
        /* set peer_cn, library will make sure subject name in the server certificate is matching with host name you are trying to connect. pass NULL if you don't want to enable this check */
        connection_instance.conn->peer_cn = NULL;
        /* Read root CA certificate (self certified) from resources*/
        resource_get_readonly_buffer ( &resources_apps_DIR_amqps_DIR_amqps_root_cacert_cer, 0, AMQP_MAX_RESOURCE_SIZE, &size_out, (const void **)&security.ca_cert );
        security.ca_cert_len = size_out;

        /* Read client's certificate from resources */
        resource_get_readonly_buffer ( &resources_apps_DIR_amqps_DIR_amqps_client_cert_cer, 0, AMQP_MAX_RESOURCE_SIZE, &size_out, (const void **)&security.cert );
        security.cert_len = size_out;

        /* Read client's private key from resources */
        resource_get_readonly_buffer ( &resources_apps_DIR_amqps_DIR_amqps_client_key_key, 0, AMQP_MAX_RESOURCE_SIZE, &size_out, (const void **)&security.key );
        security.key_len = size_out;
#endif

#ifdef AMQP_TLS_SECURITY_ENABLE
        ret = wiced_amqp_init( &broker_address, WICED_STA_INTERFACE, &callbacks, &connection_instance, &security );
#else
        ret = wiced_amqp_init( &broker_address, WICED_STA_INTERFACE, &callbacks, &connection_instance, NULL );
#endif
        if ( ret != WICED_SUCCESS )
        {
            WPRINT_APP_INFO( ("[AMQP] Failed to do amqp init...\r\n") );
            goto ERROR_RESOURCE_INIT;
            return;
        }
        wiced_rtos_init_semaphore( &semaphore );

        do
        {
            WPRINT_APP_INFO( ("[AMQP] sending connection header ...\r\n") );
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_connect( &connection_instance ), NULL, "Did you configure you broker IP address?\n" );

            wiced_rtos_delay_milliseconds( 2000 );

            WPRINT_APP_INFO( ("[AMQP] sending BEGIN header ...\r\n") );
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_begin_session ( &session_instance, &connection_instance ), NULL, "Sending AMQP BEGIN performative failed\n" );

#ifdef AMQP_WITH_SEND_LINK
            wiced_rtos_delay_milliseconds( 2000 );
            WPRINT_APP_INFO( ("[AMQP] Create Sender link by sending LINK packet ... ...\r\n") );
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_attach_link( &link_instance_sender, &session_instance, "ingress", "examples",WICED_AMQP_ROLE_SENDER ), NULL, "Sending AMQP Attach performative failed\n" );
#endif

            wiced_rtos_delay_milliseconds( 2000 );

#ifdef AMQP_WITH_RECEIVE_LINK
            WPRINT_APP_INFO( ("[AMQP] Create receiver link by sending LINK packet ...\r\n") );
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_attach_link( &link_instance_receiver, &session_instance, "examples","ingress", WICED_AMQP_ROLE_RECEIVER ), NULL, "Sending AMQP Attach performative failed\n" );
#endif

#ifdef AMQP_WITH_SEND_LINK
            while ( count <= 10 )
            {
                WPRINT_APP_INFO( ("[AMQP] Transfer Application Data no. %ld ...\r\n",count) );
                RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_send( &link_instance_sender, &message ), NULL, "Sending AMQP Transfer performative failed\n" );
                wiced_rtos_delay_milliseconds( 2000 );
                count++ ;
            }
            WPRINT_APP_INFO( ("[AMQP] sending Amqp detach packet for detaching sender link ...\r\n") );
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_detach( &link_instance_sender ), NULL, "Sending AMQP Detach performative failed\n" );
            wiced_rtos_delay_milliseconds( 2000 );
#endif

#ifdef AMQP_WITH_RECEIVE_LINK
            count = 0;
            while ( count <= 10 )
            {
                wiced_rtos_delay_milliseconds( 5000 );
                count++;
            }
            WPRINT_APP_INFO( ("[AMQP] sending Amqp detach packet for detaching receiver link  ...\r\n") );
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_detach( &link_instance_receiver ), NULL, "Sending AMQP Detach performative failed\n" );
            wiced_rtos_delay_milliseconds( 2000 );
#endif

            WPRINT_APP_INFO( ("[AMQP] sending Amqp end packet ...\r\n") );
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_end( &session_instance ), NULL, "Sending AMQP End performative failed\n" );
            wiced_rtos_delay_milliseconds( 2000 );

            WPRINT_APP_INFO( ("[AMQP] sending Amqp close packet ...\r\n") );
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_conn_close( &connection_instance ), NULL, "Sending AMQP CLOSE performative failed\n" );

            wiced_rtos_delay_milliseconds( 2000 );

        } while ( 0 );

        WPRINT_APP_INFO( ("[AMQP] Deinit connection...") );
        wiced_rtos_deinit_semaphore( &semaphore );
        ret = wiced_amqp_deinit( connection_instance.conn );
ERROR_RESOURCE_INIT:
        #ifdef AMQP_TLS_SECURITY_ENABLE
                /* Free security resources, only needed at initialization */
                resource_free_readonly_buffer( &resources_apps_DIR_amqps_DIR_amqps_root_cacert_cer, security.ca_cert );
                resource_free_readonly_buffer( &resources_apps_DIR_amqps_DIR_amqps_client_cert_cer, security.cert );
                resource_free_readonly_buffer( &resources_apps_DIR_amqps_DIR_amqps_client_key_key, security.key );
        #endif
        amqp_print_status( ret, NULL, NULL );
        free( connection_instance.conn );
        connection_instance.conn = NULL;

    }

}

/******************************************************
 *               Static Function Definitions
 ******************************************************/

/*
 * A simple result log function
 */
static void  amqp_print_status ( wiced_result_t result, const char * ok_message, const char * error_message )
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
            WPRINT_APP_INFO(( "ERROR (%s)\n\n", (error_message)));
        }
        else
        {
            WPRINT_APP_INFO(( "ERROR.\n\n" ));
        }
    }
}

/*
 * detach a link and wait for 5 seconds to receive a detach event
 */
#if defined(AMQP_WITH_RECEIVE_LINK)|| defined(AMQP_WITH_SEND_LINK)
static wiced_result_t amqp_detach( wiced_amqp_link_instance *link_instance )
{
    if ( wiced_amqp_detach( link_instance ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( amqp_wait_for( WICED_AMQP_EVENT_DETACH_RCVD, 5000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}
#endif
/*
 * end a session and wait for 5 seconds to receive a detach event
 */
static wiced_result_t amqp_end( wiced_amqp_session_instance_t *session_instance )
{
    if ( wiced_amqp_end( session_instance ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( amqp_wait_for( WICED_AMQP_EVENT_END_RCVD, 5000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Call back function to handle connection events.
 */
static wiced_result_t wiced_amqp_connection_event_cb( wiced_amqp_event_t event, void *arg, void *conn )
{
    switch ( event )
    {
        case WICED_AMQP_EVENT_HDR_RCVD:
        {
            wiced_amqp_protocol_header_arg_t *conn_args = arg;
            WPRINT_APP_INFO( ( "*****Protocol Header Packet received***** \r\n" ) );
            WPRINT_APP_INFO( ( " protocol id = %d\r\n major version = %d \r\n minor version = %d\r\n revision id = %d\r\n", conn_args->protocol_id, conn_args->major, conn_args->minor, conn_args->revision ) );
            expected_event = event;
            wiced_rtos_set_semaphore( &semaphore );
            break;
        }
        case WICED_AMQP_EVENT_OPEN_RCVD:
        {
            wiced_amqp_packet_content *args = arg;
            WPRINT_APP_INFO( ( "*****Open Packet received***** \r\n" ) );
            WPRINT_APP_INFO( ( " channel max = %d\r\n idle_timeout = %d\r\n max frame size = 0x%x\r\n", (int) args->args.open_args.channel_max, (int) args->args.open_args.idle_timeout, (unsigned int) args->args.open_args.max_frame_size ) );
            WPRINT_APP_INFO((" container_id = %.*s\r\n host name = %.*s\r\n",args->args.open_args.container_id_size, args->args.open_args.container_id, args->args.open_args.host_name_size, args->args.open_args.host_name ) );
            expected_event = event;
            wiced_rtos_set_semaphore( &semaphore );

            break;
        }
        case WICED_AMQP_EVENT_BEGIN_RCVD:
        {
            expected_event = event;
            wiced_rtos_set_semaphore( &semaphore );
            break;
        }
        case WICED_AMQP_EVENT_ATTACH_RCVD:
        {
            expected_event = event;
            wiced_rtos_set_semaphore( &semaphore );
            break;
        }
        case WICED_AMQP_EVENT_FLOW_RCVD:
        {
            wiced_amqp_packet_content *args = arg;
            WPRINT_APP_INFO((" *****Flow Packet received*****\r\n"));
            WPRINT_APP_INFO((" next incoming id %x \r\n",(int)args->args.flow_args.next_incoming_id));
            WPRINT_APP_INFO((" incoming window %x \r\n",(int)args->args.flow_args.incoming_window));
            WPRINT_APP_INFO((" next outgoing id %x \r\n",(int)args->args.flow_args.next_outgoing_id));
            WPRINT_APP_INFO((" outgoing window %x \r\n",(int)args->args.flow_args.outgoing_window));
            WPRINT_APP_INFO((" handle %x \r\n",(int)args->args.flow_args.handle));
            WPRINT_APP_INFO((" link credit  %x \r\n",(int)args->args.flow_args.link_credit));
            break;
        }
        case WICED_AMQP_EVENT_TRANSFER_RCVD:
        {
            wiced_amqp_packet_content* args = arg;
            WPRINT_APP_INFO( (" Received Transfer perofmrmative in application \r\n"));
            WPRINT_APP_INFO(( " data  : %.*s \r\n", (int) args->args.transfer_args.message.data_lenth, args->args.transfer_args.message.data ));
            WPRINT_APP_INFO( (" datalength  : %d \r\n", (int)args->args.transfer_args.message.data_lenth));
            WPRINT_APP_INFO( (" delivery tag  : %d \r\n", (int)args->args.transfer_args.message.delivery_tag));
            WPRINT_APP_INFO( (" settle  : %d \r\n", (int)args->args.transfer_args.message.settle));
            WPRINT_APP_INFO( (" delivery ID  : %d \r\n", (int)args->args.transfer_args.delivery_id));

#ifdef AMQP_WITH_RECEIVE_LINK
            ++message_count;

            /* send more link credit to sender, if you wish to receive more packets from sender */
            if ( message_count == WICED_AMQP_LINK_CREDIT )
            {
                message_count = 0;
                if ( wiced_amqp_update_link_credit( &link_instance_receiver, WICED_AMQP_LINK_CREDIT ) != WICED_SUCCESS )
                {
                    WPRINT_APP_INFO( (" Failed in updating link credit, Please make sure receiver link is passed in API \r\n"));
                    break;
                }
            }
#endif
            break;
        }
        case WICED_AMQP_EVENT_DISPOSITION_RCVD:
        {
            wiced_amqp_packet_content *args = arg;
            /* parameters received in disposition [ QOS 1] */
            if ( args != NULL )
            {
                WPRINT_APP_INFO((" *****Disposition Packet received*****\r\n"));
                WPRINT_APP_INFO( ( " Transfer ID : %d \r\n ", (int)args->args.disposition_args.delivery_id ));
                WPRINT_APP_INFO( ( " Transfer tag : %d \r\n ", (int)args->args.disposition_args.delivery_tag ));
                WPRINT_APP_INFO( ( " Delivery State : %d \r\n ", args->args.disposition_args.delivery_state ));
            }

            expected_event = event;
            wiced_rtos_set_semaphore( &semaphore );

            break;
        }
        case WICED_AMQP_EVENT_DETACH_RCVD:
        case WICED_AMQP_EVENT_END_RCVD:
        {
            expected_event = event;
            wiced_rtos_set_semaphore( &semaphore );
            break;
        }
        case WICED_AMQP_EVENT_UNKNOWN_EVENT:
        case WICED_AMQP_EVENT_CONNECTION_ERROR:
        {
            WPRINT_APP_INFO( ( "[AMQP] ERROR LOST CONNECTION\r\n" ) );
            break;
        }
        case WICED_AMQP_EVENT_CLOSE_RCVD:
        {
            expected_event = event;
            wiced_rtos_set_semaphore( &semaphore );
            break;
        }
        default:
            break;
    }
    return WICED_SUCCESS;
}

/*
 * Call back function to handle channel events.
 *
 * For each event:
 *  - The call back will set the expected_event global to the received event.
 *  - The call back will set the event semaphore to run any blocking thread functions waiting on this event
 *  - Some events will also log other global variables required for extra processing.
 *
 * A thread function will probably be waiting for the received event. Once the event is received and the
 * semaphore is set, the thread function will check for the received event and make sure it matches what
 * it is expecting.
 *
 * Note:  This mechanism is not thread safe as we are using a non protected global variable for read/write.
 * However as this snip is a single controlled thread, there is no risc of racing conditions. It is
 * however not recommended for multi-threaded applications.
 */

/*
 * A blocking call to an expected event.
 */
static wiced_result_t amqp_wait_for( wiced_amqp_event_t event, uint32_t timeout )
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
/*
 * open a connection and wait for 5 seconds to receive a connection open event
 */
static wiced_result_t amqp_connect( wiced_amqp_connection_instance *connection_instance)
{
    connection_instance->open_args.channel_max = 0xFFFF;
    connection_instance->open_args.max_frame_size = 0xFFFFFFFF;
    connection_instance->open_args.container_id = (uint8_t*) "WICED";
    connection_instance->open_args.host_name = NULL;
    connection_instance->open_args.idle_timeout = 0;

    connection_instance->header_args.protocol_id = WICED_AMQP_PROTOCOL_ID_OPEN;
    connection_instance->header_args.major = WICED_AMQP_PROTOCOL_VERSION_MAJOR;
    connection_instance->header_args.minor = WICED_AMQP_PROTOCOL_VERSION_MINOR;
    connection_instance->header_args.revision = WICED_AMQP_PROTOCOL_VERSION_REVISION;

    if ( wiced_amqp_open( connection_instance ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( amqp_wait_for( WICED_AMQP_EVENT_OPEN_RCVD, 10000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

static wiced_result_t amqp_begin_session( wiced_amqp_session_instance_t* session_instance, wiced_amqp_connection_instance *connection_instance)
{
    if ( wiced_amqp_begin( session_instance, connection_instance ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( amqp_wait_for( WICED_AMQP_EVENT_BEGIN_RCVD, 10000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

#if defined(AMQP_WITH_RECEIVE_LINK)|| defined(AMQP_WITH_SEND_LINK)
static wiced_result_t amqp_attach_link( wiced_amqp_link_instance* link_instance, wiced_amqp_session_instance_t* session_instance, char *source, char *target, wiced_amqp_role_types_t role )
{
    link_instance->session = session_instance;
    /* Requested user not to change the below value */
    /* TODO : Need to add mechanism auto allocate internally */
    link_instance->link_args.handle = handle_count++;
    link_instance->link_args.role = role;

    if ( link_instance->link_args.role == WICED_AMQP_ROLE_SENDER )
    {
        link_instance->link_args.name = (uint8_t*) "sender-link";
    }
    else
    {
        link_instance->link_args.name = (uint8_t*) "receiver-link";
    }

    link_instance->link_args.source = (uint8_t*) source;
    link_instance->link_args.target = (uint8_t*) target;

    link_instance->link_args.snd_settle_mode = WICED_FALSE;
    link_instance->link_args.rcv_settle_mode = WICED_FALSE;
    link_instance->link_credit = WICED_AMQP_LINK_CREDIT;

    if ( wiced_amqp_attach( link_instance, session_instance ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( amqp_wait_for( WICED_AMQP_EVENT_ATTACH_RCVD, 10000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}
#endif

#ifdef AMQP_WITH_SEND_LINK
static int i = 1;
static wiced_result_t amqp_send( wiced_amqp_link_instance* link_instance, wiced_amqp_message_t* message )
{
    message->delivery_tag = i++ ;
    message->data = (uint8_t*) AMQP_MESSAGE_STR;
    message->data_lenth = strlen( AMQP_MESSAGE_STR );
    message->message_format = 0;
    message->settle = WICED_FALSE;
    message->more = WICED_FALSE;
    if ( wiced_amqp_send( link_instance, message ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    if ( amqp_wait_for( WICED_AMQP_EVENT_DISPOSITION_RCVD, 5000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}
#endif
/*
 * close a connection and wait for 5 seconds to receive a connection close event
 */
static wiced_result_t amqp_conn_close( wiced_amqp_connection_instance *connection_instance )
{

    if ( wiced_amqp_close(connection_instance) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    if ( amqp_wait_for( WICED_AMQP_EVENT_CLOSE_RCVD, 5000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}
