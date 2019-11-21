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
 * Implementation for Wiced AWS Greengrass discovery
 *
 */

#include "stdint.h"
#include "aws_common.h"
#include "aws_internal.h"
#include "http_client.h"
#include "wiced_tls.h"
#include "wwd_debug.h"
#include "linked_list.h"
#include "JSON.h"
#include "http.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define WICED_AWS_GG_DISCOVERY_PAYLOAD_LENGTH  (6000)

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
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

static uint8_t aws_discovery_payload[WICED_AWS_GG_DISCOVERY_PAYLOAD_LENGTH];
static wiced_semaphore_t discovery_done;
static uint16_t received_data_length;
static uint8_t json_object_counter = 0;
static uint8_t gg_group_found;
static linked_list_t* group_list = NULL;
static wiced_aws_greengrass_discovery_callback_data_t discovery_data;
static wiced_aws_greengrass_callback_t discovery_callback = NULL;

/******************************************************
 *               Function Definitions
 ******************************************************/

#ifdef AWS_GG_DEBUG

static void dump_connection( wiced_aws_greengrass_core_connection_t* connection )
{
    wiced_aws_greengrass_core_connection_info_t* info = NULL;

    if( !connection )
        return;

    info = &connection->info;

    WPRINT_APP_INFO(("\t ==== Core's Connection Information ====\n"));
    WPRINT_APP_INFO(("\t %s: %s\n", GG_HOST_ADDRESS, info->ip_address));
    WPRINT_APP_INFO(("\t %s: %s\n", GG_PORT, info->port));
    WPRINT_APP_INFO(("\t %s: %s\n", GG_METADATA, info->metadata));
    WPRINT_APP_INFO(("\t ==== End of Connection Information ====\n"));
}

static void dump_core( wiced_aws_greengrass_core_t* core )
{
    uint32_t i = 0;
    uint32_t count;
    wiced_aws_greengrass_core_info_t* info = NULL;
    linked_list_t* connections = NULL;
    linked_list_node_t* node = NULL;

    if( !core )
    {
        return;
    }

    info = &core->info;
    WPRINT_APP_INFO((" ==== Core/Group Information ====\n"));
    WPRINT_APP_INFO(("%s: %s\n", GG_GROUP_ID, info->group_id));
    WPRINT_APP_INFO(("%s: %s\n", GG_CORE_THING_ARN, info->thing_arn));
    WPRINT_APP_INFO(("%s: %s\n", GG_ROOT_CAS, info->root_ca_certificate));
    WPRINT_APP_INFO((" ==== End of Core/Group Information ====\n"));
    connections = &info->connections;
    count = connections->count;
    for( i = 0; i < count; i++ )
    {
        linked_list_get_front_node( connections, &node);
        if( node )
        {
            dump_connection( node->data );
        }
    }
}

static void dump_group_list(void)
{
    uint32_t i = 0;
    linked_list_node_t* node = NULL;
    uint32_t count = group_list->count;

    for( i = 0; i < count; i++ )
    {
        linked_list_get_front_node(group_list, &node);
        if( node )
        {
            dump_core( node->data );
        }
    }
}

#endif

static int greengrass_initialize_group_list(void)
{
    group_list = malloc( sizeof(linked_list_t) );
    if( !group_list )
        return 0;
    linked_list_init(group_list);
    return 1;
}

/* Each Group has only one Core; But One core may have many connection endpoints */
static void greengrass_initialize_core_node( char* group_id, uint16_t length )
{
    wiced_aws_greengrass_core_info_t* info = NULL;
    wiced_aws_greengrass_core_t* core = NULL;

    if( !group_id || !length )
        return;

    core = malloc( sizeof(wiced_aws_greengrass_core_t) );
    if( !core )
    {
        return;
    }

    info = &core->info;

    linked_list_init(&info->connections);

    info->group_id = malloc((size_t)length + 1);
    strncpy( info->group_id, group_id, length );
    info->group_id[length] = '\0';

    linked_list_set_node_data( &core->node, (void *)core );

    linked_list_insert_node_at_rear(group_list, &core->node);

    return;
}

static void greengrass_add_core_root_ca( char* root_ca, uint16_t length )
{
    char* src;
    char* dst;
    int i = 0;
    int count = 0;

    linked_list_node_t* node = NULL;
    wiced_aws_greengrass_core_t* core = NULL;
    wiced_aws_greengrass_core_info_t* info = NULL;

    linked_list_get_rear_node(group_list, &node);
    if( !node )
    {
        return;
    }

    core = (wiced_aws_greengrass_core_t*)node->data;
    info = &core->info;

    info->root_ca_certificate = malloc((size_t)length + 1);

    /* Remove '\' 'n' characters which is added by AWS in the root CA cert. */
    src = root_ca;
    dst = info->root_ca_certificate;
    for (i = 0; i < length; i++)
    {
        if ( ('\\' == *src) && ('n' == *(src + 1)) )
        {
            src += 2;
            *dst++ = '\n';
            i++;
            count++;
            continue;
        }
        *dst++ = *src++;
    }

    info->root_ca_certificate[length-count] = '\0';
    return;
}

static void greengrass_add_core_thing_arn( char* thing_arn, uint16_t length )
{
    linked_list_node_t* node = NULL;
    wiced_aws_greengrass_core_t* core = NULL;
    wiced_aws_greengrass_core_info_t* info = NULL;

    linked_list_get_rear_node(group_list, &node);
    if( !node )
    {
        return;
    }

    core = node->data;
    info = &core->info;

    info->thing_arn = malloc((size_t)length + 1);
    strncpy( info->thing_arn, thing_arn, length );
    info->thing_arn[length] = '\0';

    return;
}

static void greengrass_add_connection_node_metadata( char* metadata, uint16_t length )
{
    linked_list_node_t* node = NULL;
    wiced_aws_greengrass_core_info_t* core_info = NULL;
    linked_list_t* connection_list = NULL;
    wiced_aws_greengrass_core_connection_info_t* connection = NULL;

    linked_list_get_rear_node(group_list, &node);
    if( !node || !node->data )
    {
        return;
    }

    core_info = &( (wiced_aws_greengrass_core_t *)(node->data) )->info;

    connection_list = &core_info->connections;

    linked_list_get_rear_node(connection_list, &node);
    if( !node || !node->data )
    {
        return;
    }

    connection = &( (wiced_aws_greengrass_core_connection_t *)node->data)->info;

    connection->metadata = malloc( (size_t)length + 1 );
    if( !connection->metadata )
        return;
    strncpy(connection->metadata, metadata, length);
    connection->metadata[length] = '\0';

    return;
}

static void greengrass_add_connection_node_port( char* port, uint16_t length )
{
    linked_list_node_t* node = NULL;
    wiced_aws_greengrass_core_info_t* core_info = NULL;
    linked_list_t* connection_list = NULL;
    wiced_aws_greengrass_core_connection_info_t* connection = NULL;

    linked_list_get_rear_node(group_list, &node);
    if( !node || !node->data )
    {
        return;
    }

    core_info = &( (wiced_aws_greengrass_core_t *)(node->data) )->info;

    connection_list = &core_info->connections;

    linked_list_get_rear_node(connection_list, &node);
    if( !node || !node->data )
    {
        return;
    }

    connection = &( (wiced_aws_greengrass_core_connection_t *)node->data)->info;

    connection->port = malloc( (size_t)length + 1 );
    if( !connection->port )
        return;
    strncpy(connection->port, port, length);
    connection->port[length] = '\0';

    return;
}

static void greengrass_initialize_connection_node( char* host_address, uint16_t length )
{
    linked_list_node_t* node = NULL;
    wiced_aws_greengrass_core_info_t* core_info = NULL;

    linked_list_t* connection_list = NULL;
    wiced_aws_greengrass_core_connection_info_t* info = NULL;
    wiced_aws_greengrass_core_connection_t* connection = NULL;

    linked_list_get_rear_node(group_list, &node);
    if( !node || !node->data )
    {
        return;
    }

    core_info = &( (wiced_aws_greengrass_core_t *)(node->data) )->info;

    connection_list = &core_info->connections;

    /* hostAddress field indicates start of a new connection entry for this core */
    connection = malloc(sizeof(wiced_aws_greengrass_core_connection_t) );
    if( !connection )
    {
        return;
    }

    info = &connection->info;
    node = &connection->node;

    linked_list_set_node_data( node, connection );

    /* Copy host-address */
    info->ip_address = malloc( (size_t)length + 1 );
    strncpy(info->ip_address, host_address, length);
    info->ip_address[length] = '\0';

    linked_list_insert_node_at_rear( connection_list, node );

    return;
}

static void free_conn_info( wiced_aws_greengrass_core_connection_t* node )
{
    wiced_aws_greengrass_core_connection_info_t* info = NULL;
    if( !node )
    {
        return;
    }

    info = &node->info;

    free(info->port);
    free(info->ip_address);
    free(info->metadata);

    info->port          = NULL;
    info->ip_address  = NULL;
    info->metadata      = NULL;
}

static void free_core( wiced_aws_greengrass_core_t* core )
{
    uint32_t i = 0;
    uint32_t count = 0;
    linked_list_node_t* node = NULL;
    wiced_aws_greengrass_core_info_t* info = NULL;
    if( !core )
    {
        return;
    }

    info = &core->info;

    free(info->group_id);
    free(info->thing_arn);
    free(info->root_ca_certificate);

    info->root_ca_length        = 0;
    info->group_id              = NULL;
    info->thing_arn             = NULL;
    info->root_ca_certificate   = NULL;

    count = info->connections.count;

    for( i = 0; i < count; i++ )
    {
        node = NULL;
        linked_list_remove_node_from_front(&info->connections, &node);
        if( node )
        {
            free_conn_info( (wiced_aws_greengrass_core_connection_t*)node->data );
            free(node->data);
        }
    }

    linked_list_deinit(&info->connections);
}

static void free_group_list( void )
{
    uint32_t i = 0;
    linked_list_node_t* node = NULL;
    uint32_t count = group_list->count;

    for( i = 0; i < count; i++ )
    {
        node = NULL;
        linked_list_remove_node_from_front(group_list, &node);
        if( node )
        {
            free_core( node->data );
            free(node->data);
        }
    }

    linked_list_deinit(group_list);
    free(group_list);
}

static void greengrass_discovery_http_event_handler( http_client_t* client, http_event_t event, http_response_t* response )
{
    wiced_result_t result = WICED_ERROR;
    (void)client;
    switch( event )
    {
        case HTTP_CONNECTED:
            WPRINT_APP_INFO(( "[AWS-Greengrass] Connected to AWS...\n" ));
            break;

        case HTTP_DISCONNECTED:
        {
            WPRINT_APP_INFO(( "[AWS-Greengrass] Disconnected from AWS..\n" ));
            break;
        }

        case HTTP_DATA_RECEIVED:
        {
                /* Print payload information that comes as response body */
                WPRINT_APP_INFO( ( "[AWS-Greengrass] Greengrass discovery Payload(length: %d)\n", response->payload_data_length ) );
                //print_content( (char*) response->payload, response->payload_data_length );

                memcpy( aws_discovery_payload + received_data_length, response->payload, response->payload_data_length );
                received_data_length = (uint16_t) (received_data_length + response->payload_data_length );

                if(response->remaining_length == 0)
                {
#ifdef AWS_GG_DEBUG
                    WPRINT_APP_INFO( ( "[AWS-Greengrass] Received total payload data for response:\n" ) );
                    WPRINT_APP_INFO( ("----------------------------------------------------------\n"));
                    WPRINT_APP_INFO( ( "%.*s \n", received_data_length + 1, aws_discovery_payload ) );
                    WPRINT_APP_INFO( ("----------------------------------------------------------\n"));
#endif
                    result = wiced_JSON_parser( (const char*)aws_discovery_payload, (uint32_t)received_data_length );
                    if( result != WICED_SUCCESS )
                    {
                        WPRINT_APP_INFO(("[AWS-Greengrass] JSON parser error\n"));
                        break;
                    }

                    received_data_length = 0;
                    //dump_group_list();
                    discovery_data.groups = group_list;

                    if( discovery_callback )
                    {
                        discovery_callback( &discovery_data );
                    }
                    WPRINT_APP_INFO(("Setting semaphore\n"));
                    wiced_rtos_set_semaphore( &discovery_done );
                }
            break;
        }
        case HTTP_NO_EVENT:
        default:
        {
            break;
        }
    }
}

static wiced_result_t json_callback_for_discovery_payload (wiced_json_object_t * json_object )
{
    /* Make sure that first JSON object is "GGGroups"; if we find it, all good; else it is probably not a valid json payload */
    if( json_object_counter == 0 )
    {
        if( strncmp( GG_GROUP_KEY, json_object->object_string, strlen(GG_GROUP_KEY) ) == 0 )
        {
            if( greengrass_initialize_group_list() )
            {
                /* if we only can create a linked list for collecting all groups */
                gg_group_found = 1;
            }
        }
        json_object_counter++;
        return WICED_SUCCESS;
    }

    /* Ignore JSON objects if 'GGGroups' object was not found earlier */
    if( json_object_counter && !gg_group_found )
    {
        json_object_counter++;
        return WICED_BADARG;
    }

    /* Here is a valid JSON Discovery Payload */

    /* First lookout for 'GGGroupID' and if found create a 'core' node corresponding to it */
    if( strncmp(GG_GROUP_ID, json_object->object_string, strlen(GG_GROUP_ID) ) == 0 )
    {
        greengrass_initialize_core_node(json_object->value, json_object->value_length );
        return WICED_SUCCESS;
    }

    /* store the 'ThingARN' for this group */
    if( strncmp(GG_CORE_THING_ARN, json_object->object_string, strlen(GG_CORE_THING_ARN) ) == 0 )
    {
        greengrass_add_core_thing_arn(json_object->value, json_object->value_length);
        return WICED_SUCCESS;
    }
    /* If 'HostAddress' is available, create a connection node */
    if( strncmp( GG_HOST_ADDRESS, json_object->object_string, strlen(GG_HOST_ADDRESS)  ) == 0 )
    {
        greengrass_initialize_connection_node(json_object->value, json_object->value_length);
        return WICED_SUCCESS;
    }
    /* fill 'PortNumber' to the connection node created earlier */
    if( strncmp(GG_PORT, json_object->object_string, strlen(GG_PORT) ) == 0 )
    {
        greengrass_add_connection_node_port(json_object->value, json_object->value_length);
        return WICED_SUCCESS;
    }
    /* And 'Metadata' to the connection node */
    if( strncmp(GG_METADATA, json_object->object_string, strlen(GG_METADATA) ) == 0 )
    {
        greengrass_add_connection_node_metadata(json_object->value, json_object->value_length);
        return WICED_SUCCESS;
    }

    if( strncmp(GG_ROOT_CAS, json_object->object_string, strlen(GG_ROOT_CAS) ) == 0 )
    {
        return WICED_SUCCESS;
    }

    if( strncmp( GG_BEGIN_CERTIFICATE, json_object->value, strlen(GG_BEGIN_CERTIFICATE) ) == 0 )
    {
        greengrass_add_core_root_ca( json_object->value, json_object->value_length );
        return WICED_SUCCESS;
    }
    return WICED_SUCCESS;
}

wiced_result_t aws_internal_discover_greengrass_cores( wiced_aws_endpoint_info_t* aws_iot_endpoint, wiced_aws_greengrass_callback_t cb )
{
    wiced_result_t result               = WICED_ERROR;
    char* thing_name                    = NULL;

    http_client_t                       client;
    http_client_configuration_info_t    config;
    http_header_field_t                 header;
    http_request_t                      request;

    char* discovery_uri = NULL;

    /*  During discovery, we will set-up a local HTTP client, wait for gg-json discovery payload and once done, deinit it */

    thing_name = (char *)g_aws_thing.config->name;

    /* If we can't resolve the IP address; bail out */
    result = aws_resolve_endpoint_address( aws_iot_endpoint );
    if( result != WICED_SUCCESS )
    {
        return result;
    }

    /* Before we Set up HTTPS client, set up TLS stack with AWS IoT Endpoint's Root Certificates */
    result = wiced_tls_init_root_ca_certificates( (const char *)aws_iot_endpoint->root_ca_certificate, aws_iot_endpoint->root_ca_length );
    if( result != WICED_SUCCESS )
    {
        return result;
    }

    result = http_client_init( &client, WICED_AWS_DEFAULT_INTERFACE, greengrass_discovery_http_event_handler, g_aws_thing.tls_identity );
    if( result != WICED_SUCCESS )
    {
        return result;
    }

    wiced_rtos_init_semaphore( &discovery_done );

    /* configure HTTP client configuration parameters */
    config.flag                = (http_client_configuration_flags_t)(HTTP_CLIENT_CONFIG_FLAG_SERVER_NAME | HTTP_CLIENT_CONFIG_FLAG_MAX_FRAGMENT_LEN);
    config.server_name         = (uint8_t*)aws_iot_endpoint->uri;
    config.max_fragment_length = TLS_FRAGMENT_LENGTH_1024;

    /* Register callback for AWS discovery payload */
    wiced_JSON_parser_register_callback( json_callback_for_discovery_payload );

    result = http_client_configure( &client, &config );
    if( result != WICED_SUCCESS )
    {
        goto deinit_on_error;
    }

    client.peer_cn = (uint8_t*)aws_iot_endpoint->uri;

    if ( ( result = http_client_connect( &client, (const wiced_ip_address_t*)&aws_iot_endpoint->ip_addr, WICED_AWS_GG_HTTPS_SERVER_PORT, HTTP_USE_TLS, WICED_AWS_GG_HTTPS_CONNECT_TIMEOUT ) ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "[AWS-GreenGrass] failed to connect to Server@ %s [result: %u]\n", aws_iot_endpoint->uri, result ) );
        goto deinit_on_error;
    }

    header.field        = (char *)HTTP_HEADER_HOST;
    header.field_length = sizeof( HTTP_HEADER_HOST ) - 1;
    header.value        = (char*)(aws_iot_endpoint->uri ) ;
    header.value_length = (uint16_t)strlen( aws_iot_endpoint->uri );

    discovery_uri = (char *)malloc( strlen(thing_name) +  (sizeof( GREENGRASS_DISCOVERY_HTTP_REQUEST_URI_PREFIX ) - 1) + 1 );
    memset( discovery_uri, 0, strlen(thing_name) + (sizeof(GREENGRASS_DISCOVERY_HTTP_REQUEST_URI_PREFIX ) -1 ) + 1);

    strncpy(discovery_uri, GREENGRASS_DISCOVERY_HTTP_REQUEST_URI_PREFIX, sizeof( GREENGRASS_DISCOVERY_HTTP_REQUEST_URI_PREFIX ) - 1 );
    strncpy(discovery_uri + strlen(GREENGRASS_DISCOVERY_HTTP_REQUEST_URI_PREFIX), thing_name, strlen(thing_name) );
    WPRINT_APP_INFO( ("[AWS-Greengrass] Discovery URI is: %s (len:%d)\n", discovery_uri, (int)strlen(discovery_uri) ) );

    http_request_init( &request, &client, HTTP_GET, discovery_uri, HTTP_1_1 );
    http_request_write_header( &request, &header, 1 );
    http_request_write_end_header( &request );
    http_request_flush( &request );

    if( cb )
    {
        discovery_callback = cb;
    }

    result = wiced_rtos_get_semaphore( &discovery_done, WICED_NEVER_TIMEOUT );
    if( result != WICED_SUCCESS )
    {
        goto disconnect_on_error;
    }

    free_group_list();

disconnect_on_error:
    http_request_deinit( &request );
    http_client_disconnect(&client);
    free( discovery_uri );
    wiced_rtos_delay_milliseconds(1000);
deinit_on_error:
    http_client_deinit(&client);
    wiced_rtos_deinit_semaphore( &discovery_done );
    return  result;
}
