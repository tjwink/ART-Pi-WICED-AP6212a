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
 * A rudimentary AWS IOT Greengrass application which demonstrates:
 * i.   How to discover this device's cores & groups as per AWS IoT configuration.
 * ii.  Select one of the cores from the discovery response and connect to it using MQTT protocol
 * iii. Publish a message on a particular topic.
 *
 * This application publishes "LIGHT ON" or "LIGHT OFF" message for each button press
 * to topic "WICED_BULB" with QOS1. Button used is WICED_BUTTON1 on the WICED board.
 * If we press the button first time it publishes "LIGHT OFF" and if we press next the same button
 * it will publish "LIGHT ON".
 *
 *  The application should provide typical AWS IoT configuration like:
 *  i.   Publisher's 'Thing' name( Your device's logical name as per AWS IoT console)
 *  ii.  Publisher's AWS Security credentials - Private Key & Certificate.
 *  iii. AWS IoT Root CA certificate
 *  iv.  Publisher's AWS endpoint( typically unique for a single account )
 *         For example - "a38tdxxxxxxxx.iot.us-east-1.amazonaws.com"
 *         Note: This app can also be used to connect & publish directly to AWS MQTT broker.
 *         For example - "data.iot.us-east-1.amazonaws.com" is valid MQTT Broker URI.
 *         Use this if you don't know your AWS endpoint URI.
 *
 * To run the app, work through the following steps:
 *
 *  1. Modify Wi-Fi configuration settings CLIENT_AP_SSID and CLIENT_AP_PASSPHRASE in wifi_config_dct.h
 *     to match your router settings.
 *
 *  2. Fill the AWS-IoT Thing information(Name, Private-key, Certificate) and
 *     AWS-IoT Endpoint information(AWS Endpoint URI, IP-address, port etc. ).
 *     See wiced_aws.h for more details.
 *
 *     One can provide Security credentials by copying relevant Client Certificates/Private-Keys/Root-CA
 *     @ resources/apps/aws/greengrass/publisher folder. Ensure that latest & valid certificates & private-keys are provided
 *     for this device. This is the demonstrated method in this application.
 *
 *  3. Make sure resource entries are added in publisher.mk file.
 *
 *     $(NAME)_RESOURCES  := apps/aws/greengrass/rootca.cer \
 *                           apps/aws/greengrass/publisher/privkey.cer \
 *                           apps/aws/greengrass/publisher/client.cer
 *
 *
 *  4. Build and run this application.
 *
 *  5. Run another greengrass device application subscribing to the same topic
 *  (either connected to same Greengrass core device or AWS IoT Cloud)
 */

#include "wiced.h"
#include "wiced_aws.h"
#include "aws_common.h"

#include "resources.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define APPLICATION_DELAY_IN_MILLISECONDS           (1000)

#define WICED_TOPIC                                 "WICED_BULB"
#define MSG_ON                                      "LIGHT ON"
#define MSG_OFF                                     "LIGHT OFF"
#define APP_PUBLISH_RETRY_COUNT                     (5)
#define HOST_IPv4_LEN_MAX                           (16)
#define PUBLISHER_CERTIFICATES_MAX_SIZE             (0x7fffffff)

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_semaphore_t  wake_semaphore;
static uint8_t            do_publish = 0;
static wiced_bool_t       is_connected = WICED_FALSE;

static wiced_aws_thing_security_info_t my_publisher_security_creds =
{
    .private_key         = NULL,
    .key_length          = 0,
    .certificate         = NULL,
    .certificate_length  = 0,
};

static wiced_aws_endpoint_info_t my_publisher_aws_iot_endpoint = {
    .transport           = WICED_AWS_TRANSPORT_MQTT_NATIVE,
    .uri                 = "a38td4ke8seeky.iot.us-east-1.amazonaws.com",
    .peer_common_name    = NULL,
    .ip_addr             = {0},
    .port                = WICED_AWS_IOT_DEFAULT_MQTT_PORT,
    .root_ca_certificate = NULL,
    .root_ca_length      = 0,
};

/* Fill this endpoint information in Greengrass discovery callback */
static wiced_aws_endpoint_info_t my_publisher_greengrass_core_endpoint = {
    .transport           = WICED_AWS_TRANSPORT_MQTT_NATIVE,
    .ip_addr = { 0 },
};

static wiced_aws_thing_info_t my_publisher_aws_config = {
    .name            = "wiced_ggd_1",
    .credentials     = &my_publisher_security_creds,
};

static wiced_aws_handle_t my_app_gg_handle;

/******************************************************
 *               Static Function Definitions
 ******************************************************/

static void button_press_callback( void* arg )
{
    if(do_publish == 0)
    {
        do_publish = 1;
        wiced_rtos_set_semaphore( &wake_semaphore );
    }
}

static wiced_result_t get_aws_credentials_from_resources( void )
{
    uint32_t size_out = 0;
    wiced_result_t result = WICED_ERROR;

    wiced_aws_thing_security_info_t* security = &my_publisher_security_creds;
    uint8_t** root_ca_certificate = &my_publisher_aws_iot_endpoint.root_ca_certificate;

    if( security->certificate && security->private_key && (*root_ca_certificate) )
    {
        WPRINT_APP_INFO(("\n[Application/AWS] Security Credentials already set(not NULL). Abort Reading from Resources...\n"));
        return WICED_SUCCESS;
    }

    /* Get AWS Root CA certificate filename: 'rootca.cer' located @ resources/apps/aws/iot folder */
    result = resource_get_readonly_buffer( &resources_apps_DIR_aws_DIR_iot_DIR_rootca_cer, 0, PUBLISHER_CERTIFICATES_MAX_SIZE, &size_out, (const void **) root_ca_certificate);
    if( result != WICED_SUCCESS )
    {
        goto _fail_aws_certificate;
    }
    if( size_out < 64 )
    {
        WPRINT_APP_INFO( ( "\n[Application/AWS] Invalid Root CA Certificate! Replace the dummy certificate with AWS one[<YOUR_WICED_SDK>/resources/app/aws/iot/'rootca.cer']\n\n" ) );
        resource_free_readonly_buffer( &resources_apps_DIR_aws_DIR_iot_DIR_rootca_cer, (const void *)*root_ca_certificate );
        goto _fail_aws_certificate;
    }

    my_publisher_aws_iot_endpoint.root_ca_length = size_out;

    /* Get Publisher's Certificate filename: 'client.cer' located @ resources/apps/aws/greengrass/pubisher folder */
    result = resource_get_readonly_buffer( &resources_apps_DIR_aws_DIR_greengrass_DIR_publisher_DIR_client_cer, 0, PUBLISHER_CERTIFICATES_MAX_SIZE, &size_out, (const void **) &security->certificate );
    if( result != WICED_SUCCESS )
    {
        goto _fail_client_certificate;
    }
    if(size_out < 64)
    {
        WPRINT_APP_INFO( ( "\n[Application/AWS] Invalid Device Certificate! Replace the dummy certificate with AWS one[<YOUR_WICED_SDK>/resources/app/aws/greengrass/publisher/'client.cer']\n\n" ) );
        resource_free_readonly_buffer( &resources_apps_DIR_aws_DIR_greengrass_DIR_publisher_DIR_client_cer, (const void *)security->certificate );
        goto _fail_client_certificate;
    }

    security->certificate_length = size_out;

    /* Get Publisher's Private Key filename: 'privkey.cer' located @ resources/apps/aws/greengrass/publisher folder */
    result = resource_get_readonly_buffer( &resources_apps_DIR_aws_DIR_greengrass_DIR_publisher_DIR_privkey_cer, 0, PUBLISHER_CERTIFICATES_MAX_SIZE, &size_out, (const void **) &security->private_key );
    if( result != WICED_SUCCESS )
    {
        goto _fail_private_key;
    }
    if(size_out < 64)
    {
        WPRINT_APP_INFO( ( "\n[Application/AWS] Invalid Device Private-Key! Replace the dummy Private-key with AWS one[<YOUR_WICED_SDK>/resources/app/aws/greengrass/publisher/'privkey.cer'\n\n" ) );
        resource_free_readonly_buffer( &resources_apps_DIR_aws_DIR_greengrass_DIR_publisher_DIR_privkey_cer, (const void *)security->private_key );
        goto _fail_private_key;
    }
    security->key_length = size_out;

    return WICED_SUCCESS;

_fail_private_key:
    resource_free_readonly_buffer( &resources_apps_DIR_aws_DIR_greengrass_DIR_publisher_DIR_client_cer, (const void *)security->certificate );
_fail_client_certificate:
    resource_free_readonly_buffer( &resources_apps_DIR_aws_DIR_iot_DIR_rootca_cer, (const void *)*root_ca_certificate );
_fail_aws_certificate:
    return WICED_ERROR;
}

static void convert_ip_string_to_ip_addr( const char* ip_str, wiced_ip_address_t* ip_addr )
{
    int         i;
    char        *ptr;
    uint16_t    len;
    uint8_t     octet;

    /* TODO: Handle IPv6 address as well */

    WPRINT_APP_DEBUG( ("Input IP String = [%s]\n", ip_str));

    ip_addr->version = WICED_IPV4;
    ip_addr->ip.v4 = 0x0;
    ptr = (char *)ip_str;
    len = (uint16_t)(HOST_IPv4_LEN_MAX - (ip_str - ptr));
    WPRINT_APP_DEBUG(("Output IP Addr = [\n"));
    for (i = 3; i >= 0; i--)
    {
        /* Extract the octets from IP string and assign */
        octet = (uint8_t)strtol(ptr, NULL, 10);
        WPRINT_APP_DEBUG(("%d.", octet));

        ip_addr->ip.v4 |= ((uint32_t)octet << (i << 3));
        len = (uint16_t)(HOST_IPv4_LEN_MAX - (ip_str - ptr));
        ptr = strnstrn( (const char *)ptr, len, ".", 1);
        if (ptr != NULL)
        {
            ptr++;
        }
    }
    WPRINT_APP_DEBUG(("]\n"));

    return;
}
/*
 * Call back function to handle connection events.
 */
static void my_publisher_aws_callback( wiced_aws_handle_t aws, wiced_aws_event_type_t event, wiced_aws_callback_data_t* data )
{
    if( !aws || !data || (aws != my_app_gg_handle) )
        return;

    switch ( event )
    {
        case WICED_AWS_EVENT_CONNECTED:
        {
            if( data->connection.status == WICED_SUCCESS )
            {
                is_connected = WICED_TRUE;
            }
            break;
        }

        case WICED_AWS_EVENT_DISCONNECTED:
        {
            if( data->disconnection.status == WICED_SUCCESS )
            {
                is_connected = WICED_FALSE;
            }
            break;
        }

        case WICED_AWS_EVENT_PUBLISHED:
        case WICED_AWS_EVENT_SUBSCRIBED:
        case WICED_AWS_EVENT_UNSUBSCRIBED:
        case WICED_AWS_EVENT_PAYLOAD_RECEIVED:
        default:
            break;
    }
}

static void my_publisher_greengrass_discovery_callback( wiced_aws_greengrass_discovery_callback_data_t* data )
{
    linked_list_t* group_list   = NULL;
    linked_list_node_t* node    = NULL;
    wiced_aws_greengrass_core_info_t* info = NULL;
    wiced_aws_greengrass_core_connection_info_t* connection = NULL;

    wiced_aws_endpoint_info_t* endpoint = &my_publisher_greengrass_core_endpoint;

    /* Fill 'my_publisher_greengrass_core_endpoint' with a Group information fetched from group-list.
     * Application may use some filters( metadata/core-name etc.) to select which core it wants
     * to connect to. Right now, we are just using the first core(and its first 'Connection' field)
     * from the list.
     */
    group_list = data->groups;

    if( !group_list || !group_list->count )
    {
        WPRINT_APP_INFO(("[Application/AWS] Greengrass discovery Payload is empty\n") );
        return;
    }

    linked_list_get_front_node( group_list, &node );
    if( !node )
    {
        WPRINT_APP_INFO(("[Application/AWS] Greengrass discovery - Node not found\n" ) );
        return;
    }

    info = &(( wiced_aws_greengrass_core_t *) node->data)->info;

    // WPRINT_APP_INFO((" ==== Core/Group Information ====\n"));
    // WPRINT_APP_INFO(("%s: %s\n", GG_GROUP_ID,           info->group_id));
    // WPRINT_APP_INFO(("%s: %s\n", GG_CORE_THING_ARN,     info->thing_arn));
    // WPRINT_APP_INFO(("%s: %s\n", GG_ROOT_CAS,         info->root_ca_certificate));
    // WPRINT_APP_INFO((" ==== End of Core/Group Information ====\n"));

    /* Set-up the Root CA for this Core endpoint */
    if( !endpoint->root_ca_certificate )
    {
        int length = strlen(info->root_ca_certificate);

        endpoint->root_ca_certificate = malloc( length + 1 );
        endpoint->root_ca_length      = length;
        memcpy( endpoint->root_ca_certificate, info->root_ca_certificate, length);
    }

    linked_list_get_front_node( &info->connections, &node );
    if( !node )
    {
        WPRINT_APP_INFO(("[Application/AWS] Greengrass discovery - Connections not found\n" ) );
        free(endpoint->root_ca_certificate);
        return;
    }

    /* Set-up the Connection parameters */
    connection = &(( wiced_aws_greengrass_core_connection_t *) node->data)->info;

    endpoint->port = atoi(connection->port);
    convert_ip_string_to_ip_addr( connection->ip_address, &endpoint->ip_addr );

    WPRINT_APP_INFO( ("[Application/AWS] Core[ port: %d, IP address:" IPV4_PRINT_FORMAT " ]\r\n",
                    (int)endpoint->port, IPV4_SPLIT_TO_PRINT(endpoint->ip_addr) ) );

    endpoint->peer_common_name = NULL;
    return;
}

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( void )
{
    uint32_t count = 0;
    wiced_aws_handle_t aws_connection = 0;
    wiced_result_t ret = WICED_SUCCESS;
    char*                 msg = MSG_OFF;
    int pub_retries = 0;

    wiced_init( );

    /* Bring up the network interface */
    ret = wiced_network_up( WICED_AWS_DEFAULT_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
    if ( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "[Application/AWS] Not able to join the requested AP\n\n" ) );
        return;
    }

    ret = get_aws_credentials_from_resources();
    if( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("[Application/AWS] Error fetching credentials from resources\n" ) );
        return;
    }

    ret = wiced_aws_init( &my_publisher_aws_config , my_publisher_aws_callback );
    if( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "[Application/AWS] Failed to Initialize AWS library\n\n" ) );
        return;
    }

    ret = wiced_aws_discover( &my_publisher_aws_iot_endpoint, my_publisher_greengrass_discovery_callback );
    if( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "[Application/AWS] Failed to Discover nearby Greengrass cores\n\n" ) );
        return;
    }

    wiced_rtos_delay_milliseconds(1000);

    aws_connection = (wiced_aws_handle_t)wiced_aws_create_endpoint(&my_publisher_greengrass_core_endpoint);
    if( !aws_connection )
    {
        WPRINT_APP_INFO( ( "[Application/AWS] Failed to create AWS connection handle\n\n" ) );
        return;
    }

    my_app_gg_handle = aws_connection;

    wiced_rtos_init_semaphore( &wake_semaphore );

    WPRINT_APP_INFO(("[Application/AWS] Opening connection...\n"));
    ret = wiced_aws_connect(aws_connection);
    if ( ret != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("[Application/AWS] Connection Failed\r\n"));
        return;
    }

    do_publish = 0;
    wiced_rtos_delay_milliseconds(100);

    /* configure push button to publish a message */
    wiced_gpio_input_irq_enable( WICED_BUTTON1, IRQ_TRIGGER_RISING_EDGE, button_press_callback, NULL );

    while ( 1 )
    {

        pub_retries = 0;

        wiced_rtos_get_semaphore( &wake_semaphore, APPLICATION_DELAY_IN_MILLISECONDS * 5 );
        if ( is_connected == WICED_FALSE )
        {
            break;
        }
        if ( do_publish == 1 )
        {
            WPRINT_APP_INFO(("[Application/AWS] Publishing..."));
            if ( count % 2 )
            {
                msg = MSG_ON;
            }
            else
            {
                msg = MSG_OFF;
            }
            do
            {
                ret = wiced_aws_publish( aws_connection, WICED_TOPIC, (uint8_t *)msg, strlen( msg ), WICED_AWS_QOS_ATMOST_ONCE );
                pub_retries++ ;
            } while ( ( ret != WICED_SUCCESS ) && ( pub_retries < APP_PUBLISH_RETRY_COUNT ) );
            if ( ret != WICED_SUCCESS )
            {
                WPRINT_APP_INFO((" Failed\r\n"));
                break;
            }
            else
            {
                WPRINT_APP_INFO((" Success\r\n"));
            }

            do_publish = 0;
            count++ ;
        }

        wiced_rtos_delay_milliseconds( 100 );
    }


    WPRINT_APP_INFO(("[Application/AWS] Closing connection...\r\n"));
    wiced_aws_disconnect( aws_connection );

    wiced_rtos_deinit_semaphore( &wake_semaphore );

    WPRINT_APP_INFO(("[Application/AWS] Deinitializing AWS library...\r\n"));
    ret = wiced_aws_deinit( );

    return;
}
