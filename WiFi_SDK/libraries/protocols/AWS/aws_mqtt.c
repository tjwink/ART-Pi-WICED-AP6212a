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
 * Implementation for Wiced AWS MQTT APIs
 *
 */

#include <stdint.h>
#include "aws_common.h"
#include "wiced_aws.h"
#include "aws_internal.h"
#include "mqtt_api.h"
#include "wwd_debug.h"

/******************************************************
 *                      Macros
 ******************************************************/

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

/******************************************************
 *               Function Definitions
 ******************************************************/

/* FIXME:  Remove this static definitions and unify in a local structure */
static wiced_aws_callback_message_t aws_cb_data;
static uint32_t offset = 0;

/*
 * Call back function to handle connection events.
 */
static wiced_result_t mqtt_connection_event_cb( wiced_mqtt_object_t mqtt_object, wiced_mqtt_event_info_t *event )
{
    wiced_aws_internal_handle_t* handle = g_aws_thing.mqtt_handle;
    wiced_aws_mqtt_obj_t* mqtt = &handle->mqtt;
    wiced_aws_handle_t aws = (wiced_aws_handle_t) handle;

    if( !handle || (handle->transport != WICED_AWS_TRANSPORT_MQTT_NATIVE) )
    {
        WPRINT_LIB_INFO(("[AWS/MQTT] AWS connection is NULL or Invalid\n"));
        return WICED_ERROR;
    }

    if( mqtt->base != mqtt_object )
    {
        WPRINT_LIB_INFO(("[AWS/MQTT] MQTT object mismatch\n"));
        return WICED_ERROR;
    }

    WPRINT_LIB_INFO(("[AWS/MQTT] Event received %d\n", event->type ) );

    switch ( event->type )
    {
        case WICED_MQTT_EVENT_TYPE_DISCONNECTED:
        {
            wiced_aws_callback_data_t data;
            data.disconnection.status = WICED_SUCCESS;
            g_aws_thing.aws_callback( aws, WICED_AWS_EVENT_DISCONNECTED, &data);
            break;
        }

        case WICED_MQTT_EVENT_TYPE_CONNECT_REQ_STATUS:
        {
            wiced_aws_callback_data_t data;
            data.connection.status = WICED_SUCCESS;
            g_aws_thing.aws_callback( aws, WICED_AWS_EVENT_CONNECTED, &data);
            break;
        }

        case WICED_MQTT_EVENT_TYPE_PUBLISHED:
        case WICED_MQTT_EVENT_TYPE_SUBCRIBED:
        case WICED_MQTT_EVENT_TYPE_UNSUBSCRIBED:
            break;

        case WICED_MQTT_EVENT_TYPE_PUBLISH_MSG_RECEIVED:
        {
            wiced_mqtt_topic_msg_t* msg_fragment = &(event->data.pub_recvd);

            /* 'total-data' will be available in multiple frames */
            if( msg_fragment->data_len < msg_fragment->total_length )
            {
                if( offset == 0 )
                {
                    aws_cb_data.data         = malloc(msg_fragment->total_length);
                    aws_cb_data.data_length  = msg_fragment->total_length;
                    aws_cb_data.topic        = msg_fragment->topic;
                    aws_cb_data.topic_length = msg_fragment->topic_len;
                }

                memcpy( aws_cb_data.data + offset, msg_fragment->data, msg_fragment->data_len );

                offset = offset + msg_fragment->data_len;

                /* We copied all the frames for this message */
                if( offset == msg_fragment->total_length )
                {
                    wiced_aws_callback_data_t data;
                    data.message = aws_cb_data;
                    g_aws_thing.aws_callback( aws, WICED_AWS_EVENT_PAYLOAD_RECEIVED, &data );
                    offset = 0;
                    free(aws_cb_data.data);
                }
            }
            else
            {
                wiced_aws_callback_data_t data;

                aws_cb_data.data         = msg_fragment->data;
                aws_cb_data.data_length  = msg_fragment->data_len;
                aws_cb_data.topic        = msg_fragment->topic;
                aws_cb_data.topic_length = msg_fragment->topic_len;

                data.message = aws_cb_data;

                g_aws_thing.aws_callback( aws, WICED_AWS_EVENT_PAYLOAD_RECEIVED, &data );
            }
            break;
        }
        case WICED_MQTT_EVENT_TYPE_UNKNOWN:
        default:
            break;
    }
    return WICED_SUCCESS;
}

wiced_result_t aws_internal_mqtt_init( wiced_aws_internal_handle_t* aws, wiced_aws_endpoint_info_t* ep )
{
    wiced_aws_mqtt_obj_t* mqtt = NULL;

    wiced_mqtt_object_t base = (wiced_mqtt_object_t) malloc( WICED_MQTT_OBJECT_MEMORY_SIZE_REQUIREMENT );
    if( base == NULL )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    mqtt                   = &aws->mqtt;
    mqtt->base             = base;
    mqtt->endpoint         = ep;

    mqtt->security.ca_cert      = (char *)ep->root_ca_certificate;
    mqtt->security.ca_cert_len  = ep->root_ca_length;
    mqtt->security.cert         = (char *)g_aws_thing.config->credentials->certificate;
    mqtt->security.cert_len     = g_aws_thing.config->credentials->certificate_length;
    mqtt->security.key          = (char *)g_aws_thing.config->credentials->private_key;
    mqtt->security.key_len      = g_aws_thing.config->credentials->key_length;

    wiced_mqtt_init( base );

    return WICED_SUCCESS;
}

wiced_result_t aws_internal_mqtt_connect( wiced_aws_internal_handle_t* aws )
{
    wiced_aws_endpoint_info_t* endpoint = NULL;
    wiced_mqtt_object_t base;
    wiced_aws_mqtt_obj_t* mqtt = NULL;
    wiced_mqtt_pkt_connect_t conninfo;
    wiced_result_t ret = WICED_SUCCESS;

    if( !aws || !( verify_aws_type( aws, WICED_AWS_TRANSPORT_MQTT_NATIVE ) ) )
    {
        return WICED_BADARG;
    }

    mqtt     = &aws->mqtt;
    base     = mqtt->base;
    endpoint = mqtt->endpoint;

    memset( &conninfo, 0, sizeof( conninfo ) );

    /* initialize the common part */
    conninfo.mqtt_version   = WICED_MQTT_PROTOCOL_VER4;
    conninfo.password       = NULL;
    conninfo.username       = NULL;
    conninfo.clean_session  = 1;
    conninfo.keep_alive     = WICED_AWS_MQTT_KEEP_ALIVE_TIMEOUT;
    conninfo.port_number    = (uint16_t)( (endpoint->port == 0) ? WICED_AWS_IOT_DEFAULT_MQTT_PORT : endpoint->port );
    conninfo.client_id      = (uint8_t *)g_aws_thing.config->name;
    conninfo.peer_cn        = (uint8_t *) ( endpoint->peer_common_name ? endpoint->peer_common_name : endpoint->uri );

    ret = aws_resolve_endpoint_address( endpoint );
    if( ret != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    ret = wiced_mqtt_connect( base, &endpoint->ip_addr, WICED_AWS_DEFAULT_INTERFACE, mqtt_connection_event_cb, &mqtt->security, &conninfo );
    if ( ret != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

wiced_result_t aws_internal_mqtt_disconnect( wiced_aws_internal_handle_t* aws )
{
    wiced_result_t ret = WICED_ERROR;
    wiced_aws_mqtt_obj_t* aws_mqtt_obj = NULL;
    wiced_mqtt_object_t mqtt;

    if( !aws || !verify_aws_type( aws, WICED_AWS_TRANSPORT_MQTT_NATIVE ) )
        return WICED_BADARG;

    aws_mqtt_obj = &aws->mqtt;
    mqtt         = aws_mqtt_obj->base;

    ret = wiced_mqtt_disconnect( mqtt );
    return ret;
}

wiced_result_t aws_internal_mqtt_publish( wiced_aws_internal_handle_t* aws, char* topic, uint8_t* data, uint32_t length, int qos )
{
    wiced_mqtt_msgid_t pktid;
    wiced_aws_mqtt_obj_t* aws_mqtt_obj = NULL;
    wiced_mqtt_object_t mqtt;

    if( !aws || !verify_aws_type( aws, WICED_AWS_TRANSPORT_MQTT_NATIVE ) )
        return WICED_BADARG;

    aws_mqtt_obj = &aws->mqtt;

    mqtt = aws_mqtt_obj->base;

    pktid = wiced_mqtt_publish(mqtt, topic, data, length, (uint8_t)qos);
    if( pktid == 0 )
        return WICED_ERROR;

    return WICED_SUCCESS;
}

wiced_result_t aws_internal_mqtt_subscribe( wiced_aws_internal_handle_t* aws, char* topic, int qos )
{
    wiced_mqtt_msgid_t pktid;
    wiced_aws_mqtt_obj_t* aws_mqtt_obj = NULL;
    wiced_mqtt_object_t mqtt;

    if( !aws || !verify_aws_type( aws, WICED_AWS_TRANSPORT_MQTT_NATIVE ) )
        return WICED_BADARG;

    aws_mqtt_obj = &aws->mqtt;

    mqtt = aws_mqtt_obj->base;

    pktid = wiced_mqtt_subscribe(mqtt, topic, (uint8_t)qos);
    if( pktid == 0 )
        return WICED_ERROR;

    return WICED_SUCCESS;
}

wiced_result_t aws_internal_mqtt_unsubscribe( wiced_aws_internal_handle_t* aws, char* topic )
{
    wiced_mqtt_msgid_t pktid;
    wiced_aws_mqtt_obj_t* aws_mqtt_obj = NULL;
    wiced_mqtt_object_t mqtt;

    if( !aws || !verify_aws_type( aws, WICED_AWS_TRANSPORT_MQTT_NATIVE ) )
        return WICED_BADARG;

    aws_mqtt_obj = &aws->mqtt;

    mqtt = aws_mqtt_obj->base;

    pktid = wiced_mqtt_unsubscribe(mqtt, topic);
    if( pktid == 0 )
        return WICED_ERROR;

    return WICED_SUCCESS;
}

wiced_result_t aws_internal_mqtt_deinit( wiced_aws_internal_handle_t* aws )
{
    wiced_result_t ret = WICED_ERROR;
    wiced_aws_mqtt_obj_t* aws_mqtt_obj = NULL;
    wiced_mqtt_object_t mqtt;

    if( !aws || !verify_aws_type( aws, WICED_AWS_TRANSPORT_MQTT_NATIVE ) )
        return WICED_BADARG;

    aws_mqtt_obj = &aws->mqtt;

    mqtt = aws_mqtt_obj->base;

    ret = wiced_mqtt_deinit( mqtt );
    if( ret != WICED_SUCCESS )
    {
        /* Make sure that we note this error condition and do something about it */
    }

    free( mqtt );
    return ret;
}
