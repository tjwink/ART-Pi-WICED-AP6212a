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
 * Send/Receive AMQP Application
 *
 * This application snippet demonstrates how to use
 * the WICED AMQP (Advanced Message Queuing Protocol) client library.
 *
 * Features demonstrated
 *  - AMQP Client initiation
 *  - AMQP Client data publish
 *  - AMQP Client queue declare and binging
 *  - AMQP Client data reception
 *
 * To demonstrate the app, work through the following steps.
 *  1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *     in the wifi_config_dct.h header file to match your Wi-Fi access point
 *  2. Modify the AMQP_BROKER_IP_ADDRESS to have the ip address of your AMQP broker.
 *  3. Plug the WICED eval board into your computer
 *  4. Open a terminal application and connect to the WICED eval board
 *  5. Build and download the application (to the WICED board)
 *
 * Before running the application on the WICED, Make sure an AMQP daemon that supports
 * version 0.9-1 (ex: rabbitmq-server) is running on your borker machine.
 *
 * After the download completes, the terminal displays WICED startup
 * information and then :
 *  - Joins a Wi-Fi network
 *  - Starts an AMQP connection with the server
 *  - Declare a WICED queue
 *  - Bind the WICED queue to the amq.direct exchange (with routing key "WICED_KEY" )
 *  - Publish "Hello WICED" to the amq.direct exchange
 *  - Received "Hello WICED" from the WICED queue.
 *  - Close the AMQP connection
 *
 * TROUBLESHOOTING
 *   If you are having difficulty connecting the AMQP broker
 *   Make sure your server IP address is configured correctly
 *   Make sure the AMQP version proposed by WICED (0.9-1) is supported by the broker.
 *
 * NOTES:
 *  - The snip application covers only a small set of the methods supported by AMQP.
 *    A more through application should handle events like AMQP_EVENT_CONTENT_RECV_HEADER.
 *    The application should use this event to detect the size of the received messages
 *    and hence send an ACK when the full size is received.
 *
 *  - For higher performance, user can opt to use no_ack consumes or set the QOS parameters
 *    For more details refer back the AMQP standard.
 *
 *  - The AMQP library doesn't protect against user errors, nor does it hold a connection
 *    state i.e if a user decided to send a wrong command in a wrong time, the AMQP library
 *    won't stop him. The server would signal an error though.
 *
 *  - Calling AMQP APIs from the event handler call back functions is possible. The only exception
 *    is the amqp_connection_deinit which terminates the thread where the call back is being
 *    issued from.
 *
 * SECURITY:
 *  Security in AMQP is done through SSL and/or SASL. This application is testing AMQP over SSL.
 *  To setup a secure RabbitMQ SSL server, see https://www.rabbitmq.com/ssl.html.
 *  For testing with the already provided client certificate/key, user can the following certificate/key for the server
 *  (Already signed by the  CA certificate).
 *  Use root ca certificate from the resource folder
 *
 *  #########server certificate##########
 *  -----BEGIN CERTIFICATE-----
 *  MIIC7DCCAdSgAwIBAgIBATANBgkqhkiG9w0BAQsFADATMREwDwYDVQQDDAhNeVRl
 *  c3RDQTAeFw0xNjA5MzAwNTM5MTlaFw0xNzA5MzAwNTM5MTlaMC8xHDAaBgNVBAMM
 *  E3RlamEtTGF0aXR1ZGUtRTc0NDAxDzANBgNVBAoMBnNlcnZlcjCCASIwDQYJKoZI
 *  hvcNAQEBBQADggEPADCCAQoCggEBAJXbJERQs2UtKo6TLCJ0fjoV8iQab3Sfwhhx
 *  7+T+sEl5TCV4gce5AnMoMuBfQR4uoNShnMpao4II19doUUnMRNbqQB0jg25olb0X
 *  13EAf0Ft8xYQt3XYGwJdEQIjGOxWjd2xA1nDl2lTZJkm4l2uypzzIttyQMHIRHfA
 *  VTr5aQV9/z75aHczTkaTn18Y2WDcQBcFuNazAFbledPJGl3T/xcSKJnJ7J2WBHlK
 *  ZDw4oLySEBXZgY1++b65azlNF4+LlT4ESUR8suHtJrET/0Itd9217nofvDWsc4SX
 *  zQWjQ+9UiP1N0urCwXVzf3Z2LzbyIZh+NAwDRD0foPkHIOMjWrMCAwEAAaMvMC0w
 *  CQYDVR0TBAIwADALBgNVHQ8EBAMCBSAwEwYDVR0lBAwwCgYIKwYBBQUHAwEwDQYJ
 *  KoZIhvcNAQELBQADggEBAJhmw9CM5l4ypdZ6U2h1sfBV1AdtNmZ81aBj1D+FhaJr
 *  OD2ICHPsEgrobzdobS34JIcW/Jf8h25z9384vj1hwE3IofZTGjUgXXPrAqsSRZ6v
 *  F441fhOiBYTMsg/VbPOXUr5iQKT6quNC8Or6oFEzHzlvecRknykmGL6iKgwHoC0c
 *  PfYyBk0G+K4jk0UkCXIKxiU2VwBUVIgpwMDHBTpEV3LT3phxou3utqnfCRdCoK/4
 *  1T4u9BCS9Zw0iDve1JKUD3wtXleQaCFdk4MdX4/uV1kd/N4bHbcXObV/sdmQrrvw
 *  YXS0fJKwwD3q4PigK10cZFPtkpEZuM/H05DFCEkiB0o=
 *  -----END CERTIFICATE-----
 *
 *  #########server private key##########
 *  -----BEGIN RSA PRIVATE KEY-----
 *  MIIEpQIBAAKCAQEAldskRFCzZS0qjpMsInR+OhXyJBpvdJ/CGHHv5P6wSXlMJXiB
 *  x7kCcygy4F9BHi6g1KGcylqjggjX12hRScxE1upAHSODbmiVvRfXcQB/QW3zFhC3
 *  ddgbAl0RAiMY7FaN3bEDWcOXaVNkmSbiXa7KnPMi23JAwchEd8BVOvlpBX3/Pvlo
 *  dzNORpOfXxjZYNxAFwW41rMAVuV508kaXdP/FxIomcnsnZYEeUpkPDigvJIQFdmB
 *  jX75vrlrOU0Xj4uVPgRJRHyy4e0msRP/Qi133bXueh+8NaxzhJfNBaND71SI/U3S
 *  6sLBdXN/dnYvNvIhmH40DANEPR+g+Qcg4yNaswIDAQABAoIBAFHcsZYkI6vcnYff
 *  O9finWXrwSgZzNL/xs8Fxs/olbK9cWxyxSqulXkE9638Ox1ayX7+fiFInFOXsxk4
 *  IlfXH6/rrXR9hvnUZiAzGvzfEaVTg9yE60OT90U2Q+lP9Ph+W+dEIJlzsI3YAXds
 *  d3JH3uUYKO2KH3f8h/KG/chGoSlfEgaQZP2R7NbTQRlnYyrd050lnkjmU2ONMnOt
 *  k1GCvOPqaQG6YlbO3GuqGfbnZ3VUqoNyvY1NOP7jZWir+MVXQ2bt+Rswni3rVmK9
 *  v4pepTeV0395jNpiL6dv6dX8fe16L80ZFGEsj+ZBctgrEGvNMqeLYzW/gl3HvGNf
 *  /FPuWXkCgYEAxJUa3FjEq3iinHU3rlXrlHbjMH9uxxVtQCqZu2XFhPgnbGUsVO1b
 *  R03KckkBTL/cSnzTDZIu/XW5Qzqa9HEIna+9YZlrUoEKtjUNDjlBkBW3I1TWXvph
 *  trp64XNvYhuGbPJP4rCUXS8gxZP1pWutjswujA6rCvcoq+qPmRCkYjUCgYEAwyaA
 *  f4i0n5wi914lPPXeAs5pg42r3OnMcge+sPUjgRLb2vH0XXPE9YqzHE92wGvb0k+O
 *  x6wmOxTOKCbYDcleYt0wUNpbZbHXIc4xs0nKb6uZjAvYIaj9VrPZVmqqGNvgmzNh
 *  Lb4Nv55GBMpa5KEpFUcSsQ5AMIpRICcR0JofZkcCgYEAqT3NQELUHuPe62AnLdoe
 *  1VR9R7WOQ6t4wTNr4uA9arrSBgocXolyejLIxheAHPlYyHAqq5ZdCi0d8Hk2Cph9
 *  2HQCUVJqCT+7Xx7RPJuijE+Fuc9CN8bL9Ssau5pMHGSiGI2MpRdsw8TdK/y0EVRg
 *  uX2j3USLkQc0Zr2sHxUq0LkCgYEAuSRuBJDNzB91jPnkmlEor8DkmcpuosetDwIK
 *  CBiJ9orNyoqCK6cJ1WohQ1qgby3k/0I7U3QfUS4L6Evx7iJ9SY5PqGWBEJoIYCof
 *  PNllFUzX/+W8xzJsJzW5nCLV/X+dN6EnMR5+LK0cAK96TC4Zq/Yh5Zh/jY2sKbTn
 *  UeDmqUECgYEAsPQZFXF5X/AJTYe02LH2PfWjVTpD3D05jMcPKnOdh7AlknVjaHXr
 *  CW6To/Vbwjlku7WLdMnjHkWH+4KiyZ2dKLWbHNMRwJKKlyK4Awg2mu6gCIzAQYx9
 *  z2LAsYeSj2bVxQf3fN30emhRwVEKmepxoo/HXjtscuEUyuJAZAfEh+8=
 *  -----END RSA PRIVATE KEY-----
 *
 */

/******************************************************
 *                      Macros
 ******************************************************/


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
#define AMQP_BROKER_IP_ADDRESS MAKE_IPV4_ADDRESS(192,168,1,3)
/* Name of receive Queue */
#define AMQP_RECEIVE_QUEUE_NAME "WICED_QUEUE"
/* Routing Key used for Binding between exchange and queue */
#define AMQP_ROUTING_KEY_NAME "WICED_KEY"
/* AMQP message content */
#define AMQP_MESSAGE_STR      "HELLO WICED"

/* MAX RESOUCE FILE SIZE */
#define AMQP_MAX_RESOURCE_SIZE  (0x7fffffff)
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
static wiced_result_t wiced_amqp_connection_event_cb( wiced_amqp_event_t event, void *args                 , wiced_amqp_connection_t *conn );
static wiced_result_t wiced_amqp_channel_event_cb   ( wiced_amqp_event_t event, void *args, uint16_t channel, wiced_amqp_connection_t *conn );
static wiced_result_t amqp_wait_for                 ( wiced_amqp_event_t event, uint32_t timeout );
static wiced_result_t amqp_conn_open                ( wiced_amqp_connection_t *conn );
static wiced_result_t amqp_conn_close               ( wiced_amqp_connection_t *conn );
static wiced_result_t amqp_ch_open                  ( wiced_amqp_connection_t *conn, uint16_t channel );
static wiced_result_t amqp_ch_close                 ( wiced_amqp_connection_t *conn, uint16_t channel );
static wiced_result_t amqp_ch_publish               ( wiced_amqp_connection_t *conn, uint16_t channel, const char *key );
static wiced_result_t amqp_ch_send_string           ( wiced_amqp_connection_t *conn, uint16_t channel, const char * str );
static wiced_result_t amqp_ch_queue_declare         ( wiced_amqp_connection_t *conn, uint16_t channel, const char *queue );
static wiced_result_t amqp_ch_queue_bind            ( wiced_amqp_connection_t *conn, uint16_t channel, const char *queue, const char *key );
static wiced_result_t amqp_ch_consume               ( wiced_amqp_connection_t *conn, uint16_t channel, const char *queue );
static wiced_result_t amqp_ch_recv_string           ( wiced_amqp_connection_t *conn, uint16_t channel );
static void           amqp_print_status             ( wiced_result_t restult, const char * ok_message, const char * error_message );

/******************************************************
 *               Variable Definitions
 ******************************************************/
static const wiced_ip_address_t  INITIALISER_IPV4_ADDRESS( broker_address, AMQP_BROKER_IP_ADDRESS );
static wiced_amqp_connection_t   connection;
static wiced_amqp_callbacks_t    callbacks =
{
    .connection_event = wiced_amqp_connection_event_cb,
    .channel_event    = wiced_amqp_channel_event_cb,
};
static wiced_amqp_event_t        expected_event;
static wiced_semaphore_t         semaphore;
static uint8_t                   str[128];
static uint64_t                  delivery_tag;
static wiced_amqp_security_t     security;

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Main AMQP thread.
 *
 * The thread will create a connection and then loop over the following:
 *
 * - Open a connection
 * - Open a channel
 * - Create (declare) a "WICED_QUEUE" queue
 * - Bind the queue to "amq.direct" exchange
 * - Send data through to the exchange ( using publish and send string)
 * - Receive data from the queue ( using consume and receive string )
 * - Close the channel
 * - Close the connection
 */
void application_start( void )
{
    uint32_t        size_out;
    wiced_result_t  ret = WICED_SUCCESS;

    wiced_init( );

    wiced_rtos_init_semaphore( &semaphore );

    /* Read root CA certificate (self certified) from resources*/
    resource_get_readonly_buffer ( &resources_apps_DIR_amqps_DIR_amqps_root_cacert_cer, 0, AMQP_MAX_RESOURCE_SIZE, &size_out, (const void **)&security.ca_cert );
    security.ca_cert_len = size_out;

    /* Read client's certificate from resources */
    resource_get_readonly_buffer ( &resources_apps_DIR_amqps_DIR_amqps_client_cert_cer, 0, AMQP_MAX_RESOURCE_SIZE, &size_out, (const void **)&security.cert );
    security.cert_len = size_out;

    /* Read client's private key from resources */
    resource_get_readonly_buffer ( &resources_apps_DIR_amqps_DIR_amqps_client_key_key, 0, AMQP_MAX_RESOURCE_SIZE, &size_out, (const void **)&security.key );
    security.key_len = size_out;

    /* Bringup the network interface */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    WPRINT_APP_INFO(("[AMQP] Connecting to broker %u.%u.%u.%u ...", (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 24),
                                                          (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 16),
                                                          (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 8),
                                                          (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 0)));

    ret = amqp_connection_init( &broker_address, WICED_STA_INTERFACE, &callbacks, &connection, &security );
    amqp_print_status( ret, NULL, NULL );

    /* Free security resources, only needed at initialization */
    resource_free_readonly_buffer( &resources_apps_DIR_amqps_DIR_amqps_root_cacert_cer, security.ca_cert );
    resource_free_readonly_buffer( &resources_apps_DIR_amqps_DIR_amqps_client_cert_cer, security.cert );
    resource_free_readonly_buffer( &resources_apps_DIR_amqps_DIR_amqps_client_key_key, security.key );

    while( ret == WICED_SUCCESS )
    {

        WPRINT_APP_INFO(("[AMQP] Opening connection..."));
        RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_conn_open( &connection ), NULL, "Did you configure you broker IP address?\n" );

        WPRINT_APP_INFO(("[AMQP] Opening channel..."));
        RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_ch_open( &connection, 1 ), NULL, NULL );

        WPRINT_APP_INFO(("[AMQP] Declaring queue..."));
        RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_ch_queue_declare( &connection, 1, AMQP_RECEIVE_QUEUE_NAME ), NULL, NULL );

        WPRINT_APP_INFO(("[AMQP] Binding queue..."));
        RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_ch_queue_bind( &connection, 1, AMQP_RECEIVE_QUEUE_NAME, AMQP_ROUTING_KEY_NAME ), NULL, NULL );

        WPRINT_APP_INFO(("[AMQP] Consuming..."));
        RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_ch_consume( &connection, 1, AMQP_RECEIVE_QUEUE_NAME ), NULL, NULL );

        WPRINT_APP_INFO(("[AMQP] Publishing..."));
        RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_ch_publish( &connection, 1, AMQP_ROUTING_KEY_NAME ), NULL, NULL );

        WPRINT_APP_INFO(("[AMQP] Sending message..."));
        RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_ch_send_string( &connection, 1, AMQP_MESSAGE_STR ), AMQP_MESSAGE_STR, NULL );

        WPRINT_APP_INFO(("[AMQP] Receiving message..."));
        RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_ch_recv_string( &connection, 1 ), str, NULL );

        WPRINT_APP_INFO(("[AMQP] Closing channel..."));
        RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_ch_close( &connection, 1 ), NULL, NULL);

        WPRINT_APP_INFO(("[AMQP] Closing connection..."));
        RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_conn_close( &connection ), NULL, NULL );

        wiced_rtos_delay_milliseconds( 2000 );

    }
    WPRINT_APP_INFO(("[AMQP] Deinit connection..."));
    ret = amqp_connection_deinit( &connection );
    amqp_print_status( ret, NULL, NULL );

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
 * Call back function to handle connection events.
 */
static wiced_result_t wiced_amqp_connection_event_cb( wiced_amqp_event_t event, void *args, wiced_amqp_connection_t *conn )
{
    switch( event )
    {
        case AMQP_EVENT_CONNECTION_RECV_OPEN_OK:
        case AMQP_EVENT_CONNECTION_RECV_CLOSE_OK:
        {
            expected_event = event;
            wiced_rtos_set_semaphore( &semaphore );
        }
        break;
        case AMQP_EVENT_CONNECTION_ERROR:
        {
            WPRINT_APP_INFO(("[AMQP] ERROR LOST CONNECTION\n\n"));
        }
        break;
        case AMQP_EVENT_PROTOCOL_RECV_HEADER:
        {
            WPRINT_APP_INFO(("[AMQP] ERROR RECEIVED PROTOCOL HEADER.\n"));
            WPRINT_APP_INFO(("[AMQP] Broker should support AMQP0.9-1.\n\n"));
        }
        break;
        case AMQP_EVENT_CONNECTION_RECV_CLOSE:
        {
            wiced_amqp_connection_close_arg_t   *close_args = ( wiced_amqp_connection_close_arg_t* )args;
            /* Data is sent as short string */
            memcpy( str, close_args->reply_text.str, close_args->reply_text.len );
            str[ close_args->reply_text.len ] = '\0';
            WPRINT_APP_INFO(("[AMQP] ERROR RECEIVED BROKER CLOSE REQUEST.\n"));
            WPRINT_APP_INFO(("[AMQP] REPLY TEXT: %s.\n\n",str));
        }
        break;
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
static wiced_result_t wiced_amqp_channel_event_cb( wiced_amqp_event_t event, void *args, uint16_t channel, wiced_amqp_connection_t *conn )
{
    switch( event )
    {
        case AMQP_EVENT_CHANNEL_RECV_OPEN_OK:
        case AMQP_EVENT_CHANNEL_RECV_CLOSE_OK:
        case AMQP_EVENT_QUEUE_RECV_DECLARE_OK:
        case AMQP_EVENT_QUEUE_RECV_BIND_OK:
        case AMQP_EVENT_BASIC_RECV_CONSUME_OK:
        {
            expected_event = event;
            wiced_rtos_set_semaphore( &semaphore );
        }
        break;
        case AMQP_EVENT_BASIC_RECV_DELIVER:
        {
            wiced_amqp_basic_deliver_arg_t *deliver_args = (wiced_amqp_basic_deliver_arg_t *)args;
            delivery_tag = deliver_args->delivery_tag;
        }
        break;
        case AMQP_EVENT_CONTENT_RECV_CONTENT:
        {
            wiced_amqp_frame_t      *frame = ( wiced_amqp_frame_t *) args;

            /* Data is sent as short string */
            memcpy( str, frame->buffer.data, frame->size );
            str[ frame->size ] = '\0';
            expected_event = event;
            wiced_rtos_set_semaphore( &semaphore );
        }
        break;
        default:
        break;
    }
    return WICED_SUCCESS;
}

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
 * Open a connection and wait for 5 seconds to receive a connection open OK event
 */
static wiced_result_t amqp_conn_open( wiced_amqp_connection_t *conn )
{
    if ( amqp_connection_open( conn ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( amqp_wait_for( AMQP_EVENT_CONNECTION_RECV_OPEN_OK, 5000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Close a connection and wait for 5 seconds to receive a connection close OK event
 */
static wiced_result_t amqp_conn_close( wiced_amqp_connection_t *conn )
{
    if ( amqp_connection_close( conn ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( amqp_wait_for( AMQP_EVENT_CONNECTION_RECV_CLOSE_OK, 5000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Open a channel and wait for 5 seconds to receive a channel open OK event
 */
static wiced_result_t amqp_ch_open( wiced_amqp_connection_t *conn, uint16_t channel )
{
    if ( amqp_channel_open( channel, conn ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( amqp_wait_for( AMQP_EVENT_CHANNEL_RECV_OPEN_OK, 5000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Close a channel and wait for 5 seconds to receive a channel close OK event
 */
static wiced_result_t amqp_ch_close( wiced_amqp_connection_t *conn, uint16_t channel )
{
    if ( amqp_channel_close( channel, conn ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( amqp_wait_for( AMQP_EVENT_CHANNEL_RECV_CLOSE_OK, 5000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Start channel publish (i.e indicate sending data ) to amq.direct exchange.
 *
 * Publish is an asynchronous method ( i.e no response is expected )
 */
static wiced_result_t amqp_ch_publish( wiced_amqp_connection_t *conn, uint16_t channel, const char *key )
{
    wiced_amqp_basic_publish_arg_t args;
    args.mandatory  = 0;
    args.immediate  = 0;
    AMQP_SHORT_STRING( &args.exchange, "amq.direct" );
    AMQP_SHORT_STRING( &args.routing_key, key );
    if ( amqp_channel_basic_publish( channel, &args, conn ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Send channel data to amq.direct exchange.
 *
 * Sending is an asynchronous method ( i.e no response is expected )
 */
static wiced_result_t amqp_ch_send_string( wiced_amqp_connection_t *conn, uint16_t channel, const char * str )
{
    /* Publishing started, put header */
    wiced_amqp_content_header_arg_t  args;
    wiced_amqp_frame_t               frame;

    memset( &args, 0, sizeof( wiced_amqp_content_header_arg_t ));
    args.size               = strlen(str);
    args.content_type_flag  = WICED_TRUE;
    args.delivery_mode_flag = WICED_TRUE;
    args.delivery_mode      = 2;
    AMQP_SHORT_STRING( &args.content_type, "text/plain" );
    if ( amqp_channel_content_put_header( channel, &args, conn ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    amqp_channel_content_create_frame( channel, &frame, conn );

    /* put the data, don't change the data buffer pointer of the frame */
    memcpy( frame.buffer.data, str, strlen(str));

    /* Set the frame size to the size of the data put */
    frame.size = strlen(str);

    /* Send contents. */
    if ( amqp_channel_content_put_content( channel, &frame, conn ) != WICED_SUCCESS )
    {
        /* if sending is success, no need to delete the frame */
        amqp_channel_content_delete_frame( &frame );
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Create (declare) a queue and wait for 5 seconds to receive a queue declare OK event
 */
static wiced_result_t amqp_ch_queue_declare( wiced_amqp_connection_t *conn, uint16_t channel, const char *queue )
{
    wiced_amqp_queue_declare_arg_t  args;

    args.passive     = 0;
    args.durable     = 0;
    args.exclusive   = 0;
    args.auto_delete = 0;
    args.no_wait     = 0;
    args.arguments   = NULL;
    AMQP_SHORT_STRING( &args.queue , queue );
    if ( amqp_channel_queue_declare( channel, &args, conn ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( amqp_wait_for( AMQP_EVENT_QUEUE_RECV_DECLARE_OK, 5000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Bind (link) the queue to an amq.direct exchange (using WICED_KEY as a key ) and wait for 5 seconds to receive a queue bind OK event
 */
static wiced_result_t amqp_ch_queue_bind( wiced_amqp_connection_t *conn, uint16_t channel, const char *queue, const char *key )
{
    wiced_amqp_queue_bind_arg_t        args;

    args.no_wait   = 0;
    args.arguments = NULL;
    AMQP_SHORT_STRING( &args.queue      , queue );
    AMQP_SHORT_STRING( &args.exchange   , "amq.direct" );
    AMQP_SHORT_STRING( &args.routing_key, key );
    if ( amqp_channel_queue_bind( channel, &args, conn ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( amqp_wait_for( AMQP_EVENT_QUEUE_RECV_BIND_OK, 5000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Request receiving from WICED_QUEUE queue and wait for 5 seconds to receive a queue consume OK event
 */
static wiced_result_t amqp_ch_consume( wiced_amqp_connection_t *conn, uint16_t channel, const char *queue )
{
    wiced_amqp_basic_consume_arg_t  args;

    args.no_ack     = 0;
    args.no_local   = 0;
    args.no_wait    = 0;
    args.exclusive  = 0;
    args.arguments  = NULL;
    AMQP_SHORT_STRING( &args.queue , queue );
    if ( amqp_channel_basic_consume( channel, &args, conn ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( amqp_wait_for( AMQP_EVENT_BASIC_RECV_CONSUME_OK, 5000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

/*
 * Receive from WICED_QUEUE queue (wait for 5 seconds to receive from queue).
 *
 * After receiving send an ACK to indicate successful reception.
 *
 */
static wiced_result_t amqp_ch_recv_string( wiced_amqp_connection_t *conn, uint16_t channel )
{
    wiced_amqp_basic_ack_arg_t  args;

    if ( amqp_wait_for( AMQP_EVENT_CONTENT_RECV_CONTENT, 5000 ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    args.delivery_tag   = delivery_tag;
    args.multiple       = 0;
    if ( amqp_channel_basic_ack( channel, &args, conn ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}
