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
 * Microsoft Azure AMQP demo application
 *
 * This application demonstrates how data can be sent from WICED to Microsoft Azure cloud using AMQP v1.0 protocol.
 *
 * To demonstrate the app, work through the following steps:
 *  1. Plug the WICED eval board into your computer
 *  2. Open a terminal application and connect to the WICED eval board
 *  3. Build and download the application (to the WICED board)
 *  4. The application is configured to use the Wi-Fi configuration from the default wifi_config_dct.h
 *  5. In order for WICED to authenticate and connect to Azure cloud, the following needs to be done
 *           a. Setup an Azure cloud service account. Refer to this URL for details: https://azure.microsoft.com/en-in/develop/iot/
 *           b. Ensure that a valid client certificate and private key are provided as part of security resources via
 *              the MACROs CERTIFICATE and PRIVATE_KEY. For instance, the user must place the PRIVATE_KEY in the resources
 *              folder of the SDK and set the PRIVATE_KEY to that path in the application makefile. Refer to wiced_tls.h documentation
 *              for more information
 *           c. Configure the URL of the AMQP_BROKER_IP_ADDRESS to correspond to the IotHub account settings.
 *           d. Modify the AMQP_HOST_NAME, AMQP_SASL_PLAIN_USER_NAME and AMQP_PLAIN_USER_PASSWORD to match the IotHub account settings.
 *           e. Update the send target address AMQP_SENDER_LINK_TARGET_ADDRESS based on the IoTHub settings.
 *  6. The WICED device shall now connect to Azure cloud and send data AMQP_MESSAGE_STR.
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
#define AMQP_BROKER_IP_ADDRESS          "<iothubname>.azure-devices.net"


/* AMQP message content */
#define AMQP_MESSAGE_STR                "HELLO WICED"

#define AMQP_MAX_RESOURCE_SIZE          (0x7fffffff)
#define AMQP_HOST_NAME                  "{iothubName}.azure-devices.net"
#define AMQP_SASL_PLAIN_USER_NAME       "{policyName}@sas.root.{iothubName}"
/* Generate SAS token eg: "SharedAccessSignature sr=<iothubName>.azure-devices.net&sig=<signature-string>&se=<expiry>&skn=<policyName>" */
/* Use Device explorer twin tool on windows to generate sas token for particular device */
#define AMQP_SASL_PLAIN_USER_PASSWORD   "SharedAccessSignature sig={signature-string}&se={expiry}&skn={policyName}&sr={URL-encoded-resourceURI}"
#define AMQP_SENDER_LINK_TARGET_ADDRESS "/devices/<DEVICE_ID>/messages/events"

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
static void  amqp_print_status ( wiced_result_t result, const char * ok_message, const char * error_message );

static wiced_result_t wiced_amqp_connection_event_cb( wiced_amqp_event_t event, void *arg, void *conn );

static wiced_result_t amqp_wait_for( wiced_amqp_event_t event, uint32_t timeout );

static wiced_result_t amqp_connect( wiced_amqp_connection_instance *connection_instance );

static wiced_result_t amqp_begin_session( wiced_amqp_session_instance_t* session_instance, wiced_amqp_connection_instance *connection_instance );

static wiced_result_t amqp_attach_link( wiced_amqp_link_instance* link_instance, wiced_amqp_session_instance_t* session_instance, char *source, char *target, wiced_amqp_role_types_t role );

static wiced_result_t amqp_conn_close( wiced_amqp_connection_instance *connection_instance );

static wiced_result_t amqp_send( wiced_amqp_link_instance* link_instance, wiced_amqp_message_t* message );

static wiced_result_t amqp_detach( wiced_amqp_link_instance *link_instance );

static wiced_result_t amqp_end( wiced_amqp_session_instance_t *session_instance );

/******************************************************
 *               Variable Definitions
 ******************************************************/
static  wiced_ip_address_t   broker_address;

static wiced_amqp_callbacks_t    callbacks =
{
        .connection_event = wiced_amqp_connection_event_cb,
};

static wiced_amqp_event_t        expected_event;

static wiced_semaphore_t         semaphore;

static uint32_t                  handle_count;

static wiced_amqp_security_t     security;

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 * Main AMQP thread.
 *
 * The thread will create a connection and then loop( single iteration) over the following:
 * - Open a connection
 * - Begin a session
 * - Attach a send link.
 * - Transfer sample data.
 * - Detach a send link.
 * - Ends a Session.
 * - Close the connection
 */
void application_start( void )
{
    wiced_result_t ret = WICED_SUCCESS;
    wiced_amqp_connection_instance connection_instance;
    wiced_amqp_session_instance_t session_instance;

    wiced_amqp_link_instance link_instance_sender;
    wiced_amqp_message_t message;

    uint32_t size_out;

    wiced_init( );

    /* Bringup the network interface */
    ret = wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
    if ( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "Unable to do network up\r\n"));
        return;
    }

    WPRINT_APP_INFO( ( "Resolving IP address of amqp broker...\n" ) );
    ret = wiced_hostname_lookup( AMQP_BROKER_IP_ADDRESS, &broker_address, 10000, WICED_STA_INTERFACE );
    WPRINT_APP_INFO(("Resolved Broker IP: %u.%u.%u.%u\n\n", (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 24),
                    (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 16),
                    (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 8),
                    (uint8_t)(GET_IPV4_ADDRESS(broker_address) >> 0)));
    if ( ret == WICED_ERROR || broker_address.ip.v4 == 0 )
    {
        WPRINT_APP_INFO(("Error in resolving DNS\n"));
        return;
    }

    connection_instance.conn = calloc( sizeof(wiced_amqp_connection_t), sizeof(char) );
    if ( connection_instance.conn != NULL )
    {
        /* Read root CA certificate (self certified) from resources*/
        resource_get_readonly_buffer( &resources_apps_DIR_azure_iot_hub_DIR_rootca_cer, 0, AMQP_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.ca_cert );
        security.ca_cert_len = size_out;

        /* Read client's certificate from resources */
        resource_get_readonly_buffer( &resources_apps_DIR_azure_iot_hub_DIR_client_cer, 0, AMQP_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.cert );
        security.cert_len = size_out;

        /* Read client's private key from resources */
        resource_get_readonly_buffer( &resources_apps_DIR_azure_iot_hub_DIR_privkey_cer, 0, AMQP_MAX_RESOURCE_SIZE, &size_out, (const void **) &security.key );
        security.key_len = size_out;

        if ( security.ca_cert == NULL || security.cert == NULL || security.key == NULL )
        {
            WPRINT_APP_INFO( ("[AMQP] Check the certificates...\r\n") );
            return;
        }
        ret = wiced_amqp_init( &broker_address, WICED_STA_INTERFACE, &callbacks, &connection_instance, &security );
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

            wiced_rtos_delay_milliseconds( 2000 );
            WPRINT_APP_INFO( ("[AMQP] Create Sender link by sending LINK packet ... ...\r\n") );
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_attach_link( &link_instance_sender, &session_instance, "ingress",AMQP_SENDER_LINK_TARGET_ADDRESS ,WICED_AMQP_ROLE_SENDER ), NULL, "Sending AMQP Attach performative failed\n" );
            wiced_rtos_delay_milliseconds( 2000 );

            WPRINT_APP_INFO( ("[AMQP] Transfer Application Data...\r\n") );
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_send( &link_instance_sender, &message ), NULL, "Sending AMQP Transfer performative failed\n" );
            wiced_rtos_delay_milliseconds( 2000 );

            WPRINT_APP_INFO( ("[AMQP] sending Amqp detach packet for detaching sender link ...\r\n") );
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_detach( &link_instance_sender ), NULL, "Sending AMQP Detach performative failed\n" );
            wiced_rtos_delay_milliseconds( 2000 );

            WPRINT_APP_INFO( ("[AMQP] sending Amqp end packet ...\r\n") );
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_end( &session_instance ), NULL, "Sending AMQP End performative failed\n" );
            wiced_rtos_delay_milliseconds( 2000 );

            WPRINT_APP_INFO( ("[AMQP] sending Amqp close packet ...\r\n") );
            RUN_COMMAND_PRINT_STATUS_AND_BREAK_ON_ERROR( amqp_conn_close( &connection_instance ), NULL, "Sending AMQP CLOSE performative failed\n" );

            wiced_rtos_delay_milliseconds( 2000 );

        } while ( ret == WICED_SUCCESS );

        WPRINT_APP_INFO( ("[AMQP] Deinit connection...") );
        wiced_rtos_deinit_semaphore( &semaphore );
        ret = wiced_amqp_deinit( connection_instance.conn );

        ERROR_RESOURCE_INIT:
                /* Free security resources, only needed at initialization */
                resource_free_readonly_buffer( &resources_apps_DIR_azure_iot_hub_DIR_rootca_cer, security.ca_cert );
                resource_free_readonly_buffer( &resources_apps_DIR_azure_iot_hub_DIR_client_cer, security.cert );
                resource_free_readonly_buffer( &resources_apps_DIR_azure_iot_hub_DIR_privkey_cer, security.key );
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
    connection_instance->open_args.host_name = (uint8_t*) AMQP_HOST_NAME;
    connection_instance->open_args.host_name_size = strlen( (char*) connection_instance->open_args.host_name );
    connection_instance->open_args.idle_timeout = 0;

    connection_instance->header_args.protocol_id = WICED_AMQP_PROTOCOL_ID_SASL;
    connection_instance->header_args.major = WICED_AMQP_PROTOCOL_VERSION_MAJOR;
    connection_instance->header_args.minor = WICED_AMQP_PROTOCOL_VERSION_MINOR;
    connection_instance->header_args.revision = WICED_AMQP_PROTOCOL_VERSION_REVISION;

    connection_instance->plain_config.authcid = AMQP_SASL_PLAIN_USER_NAME;
    connection_instance->plain_config.authzid = NULL;
    connection_instance->plain_config.passwd = AMQP_SASL_PLAIN_USER_PASSWORD;
    connection_instance->conn->peer_cn = (uint8_t*) "*.azure-devices.net";

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
