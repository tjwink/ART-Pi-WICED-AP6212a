/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *  Connection functions
 *
 *  Provides connection methods for use by applications
 */

#include "wiced.h"
#include "mqtt_internal.h"
#include "mqtt_connection.h"
#include "mqtt_frame.h"
#include "mqtt_manager.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define MQTT_FIXED_HEADER_SIZE      (2)

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

static uint8_t get_variable_length_bytes( uint32_t size )
{
    uint32_t x = size;
    uint8_t len = 0;
    do
    {
        len++;
        x = x / 128;
    }  while ( x > 0 );

    return len;
}

wiced_result_t mqtt_connection_init( const wiced_ip_address_t *address, uint16_t port_number, wiced_interface_t interface, const wiced_mqtt_callback_t callbacks, mqtt_connection_t *conn, const wiced_mqtt_security_t *security )
{
    wiced_result_t ret = WICED_SUCCESS;
    if ( port_number == 0 )
    {
        port_number = ( security == NULL ) ? WICED_MQTT_CONNECTION_DEFAULT_PORT : WICED_MQTT_CONNECTION_SECURE_PORT;
    }

    if ( conn->network_status == MQTT_NETWORK_DISCONNECTED )
    {
        conn->callbacks = callbacks;
        ret = mqtt_network_init( address, port_number, interface, conn, &conn->socket, security );
        if ( ret == WICED_SUCCESS )
        {
            conn->network_status = MQTT_NETWORK_CONNECTED;
        }
    }
    return ret;
}

wiced_result_t mqtt_connect( mqtt_connection_t *conn, mqtt_connect_arg_t *args, mqtt_session_t *session )
{
    mqtt_socket_t *mqtt_socket = &conn->socket;
    mqtt_event_message_t current_event;

    if ( args->clean_session == 1 )
    {
        mqtt_session_init( session );
    }
    else
    {
        if ( conn->session_init == WICED_TRUE )
        {
            mqtt_session_init( session );
        }
        conn->session_init = WICED_FALSE;
    }
    conn->session = session;

    current_event.send_context.event_t = MQTT_EVENT_SEND_CONNECT;
    current_event.send_context.conn = conn;
    current_event.send_context.args.conn_args = *args;
    current_event.event_type = MQTT_SEND_EVENT;

    /*push into the main queue*/
    return wiced_rtos_push_to_queue( &mqtt_socket->queue, &current_event, WICED_NO_WAIT );
}

wiced_result_t mqtt_disconnect( mqtt_connection_t *conn )
{
    mqtt_socket_t *mqtt_socket = &conn->socket;
    mqtt_event_message_t current_event;

    current_event.send_context.event_t = MQTT_EVENT_SEND_DISCONNECT;
    current_event.send_context.conn = conn;
    current_event.event_type = MQTT_SEND_EVENT;

    return wiced_rtos_push_to_queue( &mqtt_socket->queue, &current_event, WICED_NO_WAIT );
}

wiced_result_t mqtt_subscribe( mqtt_connection_t *conn, mqtt_subscribe_arg_t *args )
{
    mqtt_socket_t *mqtt_socket = &conn->socket;
    mqtt_event_message_t current_event;

    current_event.send_context.event_t = MQTT_EVENT_SEND_SUBSCRIBE;
    current_event.send_context.conn = conn;
    current_event.send_context.args.sub_args = *args;
    current_event.event_type = MQTT_SEND_EVENT;

    return wiced_rtos_push_to_queue( &mqtt_socket->queue, &current_event, WICED_NO_WAIT );
}

wiced_result_t mqtt_unsubscribe( mqtt_connection_t *conn, mqtt_unsubscribe_arg_t *args )
{
    mqtt_socket_t *mqtt_socket = &conn->socket;
    mqtt_event_message_t current_event;
    wiced_result_t result = WICED_SUCCESS;

    current_event.send_context.event_t = MQTT_EVENT_SEND_UNSUBSCRIBE;
    current_event.send_context.conn = conn;
    current_event.send_context.args.unsub_args = *args;
    current_event.event_type = MQTT_SEND_EVENT;

    result = wiced_rtos_push_to_queue( &mqtt_socket->queue, &current_event, WICED_NO_WAIT );
    return result;
}

wiced_result_t mqtt_publish( mqtt_connection_t *conn, mqtt_publish_arg_t *args )
{
    mqtt_socket_t *mqtt_socket = &conn->socket;
    mqtt_event_message_t current_event;

    current_event.send_context.event_t = MQTT_EVENT_SEND_PUBLISH;
    current_event.send_context.conn = conn;
    current_event.send_context.args.pub_args = *args;
    current_event.event_type = MQTT_SEND_EVENT;

    return wiced_rtos_push_to_queue( &mqtt_socket->queue, &current_event, WICED_NO_WAIT );
}

wiced_result_t mqtt_connection_deinit( mqtt_connection_t *conn )
{
    if ( conn->network_status == MQTT_NETWORK_CONNECTED )
    {
        mqtt_network_deinit( &conn->socket );
        conn->network_status = MQTT_NETWORK_DISCONNECTED;
    }
    return WICED_SUCCESS;
}

/******************************************************
 *      Backend functions called from mqtt_queue
 ******************************************************/

wiced_result_t mqtt_backend_put_connect( mqtt_connect_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret;
    mqtt_frame_t frame;
    mqtt_connect_arg_t *final_args = args;

    /* Fixing connect args based on specification */
    if ( final_args->username_flag == WICED_FALSE )
    {
        /* make sure password flag is 0 as well */
        final_args->password_flag = WICED_FALSE;
    }

    if ( final_args->will_flag == WICED_FALSE )
    {
        /* make sure will_retain and will_qos are 0 */
        final_args->will_retain = WICED_FALSE;
        final_args->will_qos = MQTT_QOS_DELIVER_AT_MOST_ONCE;
    }

    /* Send Protocol Header */
    ret = mqtt_frame_create( MQTT_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    mqtt_frame_put_connect( &frame, final_args );
    return mqtt_frame_send( &frame, &conn->socket );
}

wiced_result_t mqtt_backend_get_connack( mqtt_connack_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret = WICED_SUCCESS;
    UNUSED_PARAMETER(args);
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */

        if ( conn->callbacks != NULL )
        {

            wiced_mqtt_event_info_t event;
            event.type = WICED_MQTT_EVENT_TYPE_CONNECT_REQ_STATUS;
            event.data.err_code = args->return_code;
            conn->callbacks( (void*) conn, &event );
        }
    }
    return ret;
}

/* | Header + data-1 | data-2 | data-3 | data-n | */
static wiced_result_t mqtt_frame_send_fragments_with_header( wiced_tcp_socket_t* socket, uint8_t* header, uint16_t header_length, uint8_t* data, uint32_t data_length )
{
    wiced_result_t  ret = WICED_ERROR;
    int iterations = 0;

    uint8_t*        packet_data = NULL;
    wiced_packet_t* packet = NULL;
    uint16_t        content_length = 0;

    uint16_t available = 0;
    uint32_t consumed  = 0;
    uint32_t old_consumed = 0;
    uint32_t total_bytes_to_consume = header_length + data_length;

    WPRINT_LIB_DEBUG( ("\t\tTotal TCP payload: %d bytes\n", (int)total_bytes_to_consume ) );

    while( consumed < total_bytes_to_consume )
    {
        iterations++;
        old_consumed = consumed;
        (void)old_consumed;
        /* create a tcp packet and fetch how much we can fit in single packet(available) */
        ret = wiced_packet_create_tcp( socket, content_length, &packet, &packet_data, &available );

        if( ret != WICED_SUCCESS )
        {
            WPRINT_LIB_INFO( ("[MQTT-LIB] Error creating TCP packet\r\n" ) );
            return ret;
        }
        /* if still NULL packet, return error */
        if( !packet )
        {
            WPRINT_LIB_INFO( ("[MQTT-LIB] Error TCP packet is NULL\r\n" ) );
            return WICED_ERROR;
        }

        /* TODO: Handle the case where 'available' is not zero but still less than 'header-length' */

        /* First copy the header if it has not been copied */
        if( consumed < header_length )
        {
            /* copy the header */
            memcpy( packet_data, header, header_length );
            /* Update data pointer */
            packet_data += header_length;
            /* Space left in TCP packet */
            available = (uint16_t) (available - header_length );
            /* udpate 'consumed' flag */
            consumed += header_length;
        }

        /* if current TCP packet still has space and we have more data to copy; copy as much as possible */
        if( available && (consumed < total_bytes_to_consume) )
        {
            uint16_t min_available = (uint16_t) MIN( (total_bytes_to_consume - consumed ), available );
            /* copy the data */
            memcpy( packet_data, data, min_available );
            /* udpate data pointer */
            packet_data += min_available;
            /* update 'consumed' */
            consumed += min_available;

            /* We are done with this packet by now..So mark the end of the 'data' in TCP packet */
            wiced_packet_set_data_end(packet, packet_data);

            /* And send it on the wire */
            wiced_tcp_send_packet(socket, packet);

            WPRINT_LIB_DEBUG( ("\t\t ---> Sent %u out of %u bytes (in TCP packet:%d, size:%u)\n",
                    (unsigned int)consumed, (unsigned int)total_bytes_to_consume, iterations, (unsigned int) (consumed - old_consumed ) ) );
        }
    }
    return WICED_SUCCESS;
}

static void mqtt_prepare_header( mqtt_event_t event, mqtt_raw_buffer_t* buffer, uint32_t payload_len, void* mqtt_args)
{
    uint8_t     len = 0;

    switch( event )
    {
        case MQTT_EVENT_SEND_PUBLISH:
        {
            mqtt_publish_arg_t* args = (mqtt_publish_arg_t*) mqtt_args;
            /* 1st byte of Fixed header */
            MQTT_BUFFER_PUT_BIT  ( buffer, args->retain, 0, 0 );
            MQTT_BUFFER_PUT_2BIT ( buffer, args->qos, 1, 0 );
            MQTT_BUFFER_PUT_BIT  ( buffer, args->dup, 3, 0 );
            MQTT_BUFFER_PUT_4BIT ( buffer, MQTT_PACKET_TYPE_PUBLISH, 4, 1 );

            /* 'Remaining Length' field of MQTT - atleast 1 byte */
            MQTT_BUFFER_PUT_VARIABLE_LENGTH( buffer, payload_len, len );

            MQTT_BUFFER_PUT_STRING( buffer, args->topic.str, args->topic.len );
            if ( args->qos != MQTT_QOS_DELIVER_AT_MOST_ONCE )
            {
                MQTT_BUFFER_PUT_SHORT( buffer, args->packet_id );
            }
            break;
        }

        case MQTT_EVENT_SEND_SUBSCRIBE:
        case MQTT_EVENT_SEND_UNSUBSCRIBE:
        case MQTT_EVENT_SEND_PUBREL:
        case MQTT_EVENT_SEND_PUBACK:
        case MQTT_EVENT_SEND_PUBCOMP:
        case MQTT_EVENT_SEND_PUBREC:
        case MQTT_EVENT_SEND_DISCONNECT:
        case MQTT_EVENT_SEND_CONNECT:
        case MQTT_EVENT_RECV_CONNACK:
        case MQTT_EVENT_RECV_PUBLISH:
        case MQTT_EVENT_RECV_PUBACK:
        case MQTT_EVENT_RECV_PUBREC:
        case MQTT_EVENT_RECV_PUBREL:
        case MQTT_EVENT_RECV_PUBCOMP:
        case MQTT_EVENT_RECV_SUBACK:
        case MQTT_EVENT_RECV_UNSUBACK:
        case MQTT_EVENT_RECV_PINGRES:
        case MQTT_EVENT_CONNECTION_CLOSE:
        case MQTT_EVENT_SEND_PINGREQ:
        case MQTT_EVENT_TICK:
        default:
            break;
    }

}

wiced_result_t mqtt_backend_put_publish(  mqtt_publish_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret = WICED_SUCCESS;
    mqtt_frame_t        frame;
    mqtt_publish_arg_t  *final_args = args;
    mqtt_raw_buffer_t   buffer;
    uint8_t  mqtt_header[MQTT_MAX_HEADER_SIZE]  = { 0 };
    uint16_t mqtt_header_size = 0;

    uint32_t payload_len = 0;
    wiced_tcp_socket_t* socket = &conn->socket.socket;

    /* Deduce Maximum TCP segment size */
    uint16_t max_tcp_segment_size = (uint16_t) WICED_MAXIMUM_SEGMENT_SIZE( socket );

    /* Point the buffer's data-pointer to start of mqtt-header */
    buffer.data = (uint8_t*)&mqtt_header;

    /* Fixed MQTT header size */
    mqtt_header_size = (uint16_t)( mqtt_header_size + MQTT_FIXED_HEADER_SIZE );

    /* Calculate payload length in advance */
    payload_len =  ( uint32_t )(args->qos == MQTT_QOS_DELIVER_AT_MOST_ONCE ? 0 : 2)     /* 2 bytes of Packet-identifier  */
                        + ( uint32_t ) ( args->topic.len + sizeof(args->topic.len))     /* Topic string length + Topic string */
                        + ( uint32_t ) args->data_len;                                  /* size of message    */

    /* Increment Header-size as per payload-length */
    mqtt_header_size = (uint16_t)(mqtt_header_size + get_variable_length_bytes(payload_len) - 1 );

    /* Increment header as per other members of Variable header */
    mqtt_header_size = (uint16_t)(mqtt_header_size + args->topic.len + (uint16_t)sizeof(args->topic.len) +
                        (args->qos == MQTT_QOS_DELIVER_AT_MOST_ONCE ? 0 : 2) );

    if( mqtt_header_size > MQTT_MAX_HEADER_SIZE )
    {
        WPRINT_LIB_INFO( ("[MQTT-LIB] MQTT-frame header(%d) exceeds MAX Header length(%d)...\n", mqtt_header_size, MQTT_MAX_HEADER_SIZE ) );
        return WICED_BADARG;
    }

    /* do the fragmentation if required */
    WPRINT_LIB_DEBUG( ("\n\tMQTT frame Message-Payload:%d (TCP-segment-size:%d MQTT Header: %d)\n",
                (int)args->data_len, (int)max_tcp_segment_size, (int)(mqtt_header_size) ) );

    /* If data is larger than a single TCP packet */
    if( args->data_len > (uint32_t)(max_tcp_segment_size - mqtt_header_size) )
    {
        WPRINT_LIB_DEBUG( ("\tStart Fragmentation...\n") );

        /* Prepare the MQTT header */
        mqtt_prepare_header( MQTT_EVENT_SEND_PUBLISH, &buffer, payload_len, (void *)args );

        /* send mqtt-frame in TCP segments */
        return mqtt_frame_send_fragments_with_header( socket, mqtt_header, mqtt_header_size, args->data, args->data_len);
    }
    else
    {
        /* Single TCP packet is enough for application-data */
        ret = mqtt_frame_create( MQTT_CONNECTION_FRAME_MAX, &frame, &conn->socket );
        if ( ret != WICED_SUCCESS )
        {
            return ret;
        }
        mqtt_frame_put_publish( &frame, final_args );
        return mqtt_frame_send( &frame, &conn->socket );
    }
    return ret;
}

wiced_result_t mqtt_backend_get_publish( mqtt_publish_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret = WICED_SUCCESS;
    wiced_mqtt_event_info_t event;

    /* Do MQTT-manager book-keeping only if there is no more data to come;else
     * wait for full-frame to come */
    if( !conn->cached_frame.data_pending )
    {
        ret = mqtt_manager( MQTT_EVENT_RECV_PUBLISH, args, conn );
    }

    if ( ( ret == WICED_SUCCESS ) && ( args->data != NULL ) )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks != NULL )
        {
            event.type                          = WICED_MQTT_EVENT_TYPE_PUBLISH_MSG_RECEIVED;
            event.data.pub_recvd.topic          = args->topic.str;
            event.data.pub_recvd.topic_len      = args->topic.len;
            event.data.pub_recvd.data           = args->data;
            event.data.pub_recvd.data_len       = args->data_len;
            event.data.pub_recvd.total_length   = args->total_length;

            if( conn->cached_frame.data_pending )
            {
                /* Notify the application that this is chunked message payload */
                event.data.pub_recvd.chunked = 1;
            }
            else
            {
                event.data.pub_recvd.chunked = 0;
            }

            conn->callbacks( (void*) conn, &event );
        }
    }
    return ret;
}

wiced_result_t mqtt_backend_put_puback( const mqtt_puback_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret;
    mqtt_frame_t frame;

    /* Send Protocol Header */
    ret = mqtt_frame_create( MQTT_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    mqtt_frame_put_puback( &frame, args );
    return mqtt_frame_send( &frame, &conn->socket );
}

wiced_result_t mqtt_backend_get_puback( mqtt_puback_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret;
    ret = mqtt_manager( MQTT_EVENT_RECV_PUBACK, args, conn );
    if ( ret == WICED_SUCCESS )
    {
        if ( conn->callbacks != NULL )
        {
            wiced_mqtt_event_info_t event;
            event.type = WICED_MQTT_EVENT_TYPE_PUBLISHED;
            event.data.msgid = args->packet_id;
            conn->callbacks( (void*) conn, &event );
        }
    }
    return ret;
}

wiced_result_t mqtt_backend_put_pubrec( const mqtt_pubrec_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret;
    mqtt_frame_t frame;

    /* Send Protocol Header */
    ret = mqtt_frame_create( MQTT_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    mqtt_frame_put_pubrec( &frame, args );
    return mqtt_frame_send( &frame, &conn->socket );
}

wiced_result_t mqtt_backend_get_pubrec( mqtt_pubrec_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret;
    ret = mqtt_manager( MQTT_EVENT_RECV_PUBREC, args, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks != NULL )
        {
            wiced_mqtt_event_info_t event;
            event.type = WICED_MQTT_EVENT_TYPE_UNKNOWN;
            event.data.msgid = args->packet_id;
            conn->callbacks( (void*) conn, &event );
        }

    }
    return ret;
}

wiced_result_t mqtt_backend_put_pubrel( const mqtt_pubrel_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret;
    mqtt_frame_t frame;

    /* Send Protocol Header */
    ret = mqtt_frame_create( MQTT_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    mqtt_frame_put_pubrel( &frame, args );
    return mqtt_frame_send( &frame, &conn->socket );
}

wiced_result_t mqtt_backend_get_pubrel( mqtt_pubrel_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret;
    ret = mqtt_manager( MQTT_EVENT_RECV_PUBREL, args, conn );
    if ( ret == WICED_SUCCESS )
    {
        if ( conn->callbacks != NULL )
        {
            wiced_mqtt_event_info_t event;
            event.type = WICED_MQTT_EVENT_TYPE_UNKNOWN;
            event.data.msgid = args->packet_id;
            conn->callbacks( (void*) conn, &event );
        }

    }
    return ret;
}

wiced_result_t mqtt_backend_put_pubcomp( const mqtt_pubcomp_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret;
    mqtt_frame_t frame;

    /* Send Protocol Header */
    ret = mqtt_frame_create( MQTT_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    mqtt_frame_put_pubcomp( &frame, args );
    return mqtt_frame_send( &frame, &conn->socket );
}

wiced_result_t mqtt_backend_get_pubcomp( mqtt_pubcomp_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret;
    ret = mqtt_manager( MQTT_EVENT_RECV_PUBCOMP, args, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks != NULL )
        {
            wiced_mqtt_event_info_t event;
            event.type = WICED_MQTT_EVENT_TYPE_PUBLISHED;
            event.data.msgid = args->packet_id;
            conn->callbacks( (void*) conn, &event );
        }

    }
    return ret;
}

wiced_result_t mqtt_backend_put_subscribe( mqtt_subscribe_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret;
    mqtt_frame_t frame;
    mqtt_subscribe_arg_t *final_args = args;

    /* Send Protocol Header */
    ret = mqtt_frame_create( MQTT_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    mqtt_frame_put_subscribe( &frame, final_args );
    return mqtt_frame_send( &frame, &conn->socket );
}

wiced_result_t mqtt_backend_get_suback( wiced_mqtt_suback_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret;
    ret = mqtt_manager( MQTT_EVENT_RECV_SUBACK, args, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */

        if ( conn->callbacks != NULL )
        {

            wiced_mqtt_event_info_t event;
            event.type = WICED_MQTT_EVENT_TYPE_SUBCRIBED;
            event.data.msgid = args->packet_id;
            conn->callbacks( (void*) conn, &event );
        }

    }
    return ret;
}

wiced_result_t mqtt_backend_put_unsubscribe( mqtt_unsubscribe_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret;
    mqtt_frame_t frame;
    mqtt_unsubscribe_arg_t *final_args = args;

    /* Generate packet ID */
    // final_args.packet_id = conn->packet_id++;

    /* Send Protocol Header */
    ret = mqtt_frame_create( MQTT_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    mqtt_frame_put_unsubscribe( &frame, final_args );
    return mqtt_frame_send( &frame, &conn->socket );
}

wiced_result_t mqtt_backend_get_unsuback( mqtt_unsuback_arg_t *args, mqtt_connection_t *conn )
{
    wiced_result_t ret;
    ret = mqtt_manager( MQTT_EVENT_RECV_UNSUBACK, args, conn );
    if ( ret == WICED_SUCCESS )
    {

        if ( conn->callbacks != NULL )
        {

            wiced_mqtt_event_info_t event;
            event.type = WICED_MQTT_EVENT_TYPE_UNSUBSCRIBED;
            event.data.msgid = args->packet_id;

            conn->callbacks( (void*) conn, &event );
        }
    }
    return ret;
}

wiced_result_t mqtt_backend_put_disconnect( mqtt_connection_t *conn )
{
    wiced_result_t ret;
    mqtt_frame_t frame;

    ret = mqtt_frame_create( MQTT_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    mqtt_frame_put_disconnect( &frame );
    return mqtt_frame_send( &frame, &conn->socket );
}

wiced_result_t mqtt_backend_connection_close( mqtt_connection_t *conn )
{
    wiced_result_t ret;
    ret = mqtt_manager( MQTT_EVENT_CONNECTION_CLOSE, NULL, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */

        if ( conn->callbacks != NULL )
        {

            wiced_mqtt_event_info_t event;
            event.type = WICED_MQTT_EVENT_TYPE_DISCONNECTED;
            conn->callbacks( (void*) conn, &event );
        }
        conn->callbacks = NULL;
    }
    return ret;
}

wiced_result_t mqtt_backend_put_pingreq( mqtt_connection_t *conn )
{
    wiced_result_t ret;
    mqtt_frame_t frame;

    ret = mqtt_frame_create( MQTT_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    mqtt_frame_put_pingreq( &frame );
    return mqtt_frame_send( &frame, &conn->socket );
}

wiced_result_t mqtt_backend_get_pingres( mqtt_connection_t *conn )
{
    return mqtt_manager( MQTT_EVENT_RECV_PINGRES, NULL, conn );
}
