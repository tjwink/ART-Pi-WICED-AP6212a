/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *  I/O functions
 *  Provides functions for sending and receiving to the network for use by
 *  framing layer
 */

#include "mqtt_internal.h"
#include "mqtt_frame.h"
#include "mqtt_network.h"
#include "mqtt_connection.h"
#include "mqtt_manager.h"
#include "wiced_tls.h"

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
typedef struct wiced_mqtt_network_message_s
{
    wiced_packet_t                   *packet;
    void                             *data;
} wiced_mqtt_network_message_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static wiced_result_t mqtt_network_recv_thread( wiced_tcp_socket_t *socket, void *args );
static wiced_result_t mqtt_disconnect_callback( wiced_tcp_socket_t *socket, void *args );
static void mqtt_thread_main( uint32_t arg );
//static void wiced_process_mqtt_request( void *arg );
static wiced_result_t  mqtt_frame_receive_fragmented( mqtt_connection_t* conn, uint8_t* data, uint32_t length, uint8_t chunked, uint8_t first_fragment, mqtt_frame_tcp_wrapper_t* packet );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t mqtt_core_init( mqtt_connection_t *conn )
{
    wiced_result_t result = WICED_SUCCESS;
    mqtt_socket_t *mqtt_socket = &conn->socket;

    if ( ( result = wiced_rtos_init_queue( &mqtt_socket->queue, "Mqtt library queue", sizeof(mqtt_event_message_t), WICED_MQTT_QUEUE_SIZE ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_INFO(("[MQTT LIB] Initializing queue failed\n"));
        return result;
    }

    conn->core_init = WICED_TRUE;

    if ( ( result = wiced_rtos_create_thread( &mqtt_socket->net_thread, WICED_DEFAULT_LIBRARY_PRIORITY, "Mqtt libary theard", mqtt_thread_main, MQTT_THEARD_STACK_SIZE, conn ) ) != WICED_SUCCESS )
    {
        goto ERROR_QUEUE_THREAD;
    }
    conn->session_init = WICED_TRUE;
    conn->network_status = MQTT_NETWORK_DISCONNECTED;
    wiced_rtos_init_mutex(&conn->lock);
    return result;

ERROR_QUEUE_THREAD:
    wiced_rtos_deinit_queue( &mqtt_socket->queue );

    conn->core_init = WICED_FALSE;

    return result;
}

wiced_result_t mqtt_core_deinit( mqtt_connection_t *conn )
{
    mqtt_socket_t *mqtt_socket = &conn->socket;

    if ( conn->core_init == WICED_TRUE )
    {
        conn->core_init = WICED_FALSE;

        if ( conn->net_init_ok == WICED_TRUE )
        {
            conn->net_init_ok = WICED_FALSE;
            mqtt_connection_deinit( conn );
        }
        wiced_rtos_deinit_queue( &mqtt_socket->queue );

        if ( wiced_rtos_is_current_thread( &mqtt_socket->net_thread ) != WICED_SUCCESS )
        {
            wiced_rtos_thread_force_awake( &mqtt_socket->net_thread );
            wiced_rtos_thread_join( &mqtt_socket->net_thread );
            wiced_rtos_delete_thread( &mqtt_socket->net_thread );
        }
    }
    wiced_rtos_deinit_mutex(&conn->lock);
    return WICED_SUCCESS;
}

wiced_result_t mqtt_network_init( const wiced_ip_address_t *server_ip_address, uint16_t portnumber, wiced_interface_t interface, void *p_user, mqtt_socket_t *socket, const wiced_mqtt_security_t *security )
{
    wiced_result_t result = WICED_SUCCESS;
    mqtt_connection_t *conn = (mqtt_connection_t *) p_user;
    /* Create a TCP socket */
    if ( ( result = wiced_tcp_create_socket( &socket->socket, interface ) ) != WICED_SUCCESS )
    {
        goto ERROR_CREATE_SOCKET;
    }
    socket->is_root_ca_used = WICED_FALSE;
    socket->is_client_auth_enabled = WICED_FALSE;

    if ( security != NULL )
    {
        if ( ( result = wiced_tls_init_root_ca_certificates( security->ca_cert, security->ca_cert_len ) ) != WICED_SUCCESS )
        {
            goto ERROR_CA_CERT_INIT;
        }
        socket->is_root_ca_used = WICED_TRUE;
        if ( ( security->cert != NULL ) && ( security->key != NULL ) )
        {
            result = wiced_tls_init_identity( &socket->tls_identity, (char*) security->key, security->key_len, (const uint8_t*) security->cert, security->cert_len );
            if ( result != WICED_SUCCESS )
            {
                goto ERROR_TLS_INIT;
            }
            wiced_tls_init_context( &socket->tls_context, &socket->tls_identity, (const char*) conn->peer_cn );
            socket->is_client_auth_enabled = WICED_TRUE;
        }
        else
        {
            wiced_tls_init_context( &socket->tls_context, NULL,  (const char*) conn->peer_cn );
        }

        wiced_tcp_enable_tls( &socket->socket, &socket->tls_context );
    }

    socket->p_user = p_user;
    socket->server_ip_address = *server_ip_address;
    socket->portnumber = portnumber;
    conn->net_init_ok = WICED_TRUE;

    result = mqtt_network_connect( socket );
    if ( result != WICED_SUCCESS )
    {
        goto ERROR_QUEUE_INIT;;
    }

    /*for receive*/

    result = wiced_tcp_register_callbacks( &socket->socket, NULL, mqtt_network_recv_thread, mqtt_disconnect_callback, socket );
    if ( result != WICED_SUCCESS )
    {
        goto ERROR_NETWORK_INIT;
    }

    return result;

ERROR_NETWORK_INIT:
    mqtt_network_disconnect( socket );

ERROR_QUEUE_INIT:
    if ( security == NULL )
    {
        goto ERROR_CA_CERT_INIT;
    }

    wiced_tls_reset_context( &socket->tls_context );
    if ( socket->is_client_auth_enabled == WICED_TRUE )
    {
        wiced_tls_deinit_identity( &socket->tls_identity );
        socket->is_client_auth_enabled = WICED_FALSE;
    }

ERROR_TLS_INIT:
 if ( socket->is_root_ca_used == WICED_TRUE )
    {
        wiced_tls_deinit_root_ca_certificates( );
        socket->is_root_ca_used = WICED_FALSE;
    }

ERROR_CA_CERT_INIT:
    wiced_tcp_delete_socket( &socket->socket );

ERROR_CREATE_SOCKET:

    return result;
}

wiced_result_t mqtt_network_deinit( mqtt_socket_t *socket )
{
    wiced_tcp_unregister_callbacks( &socket->socket );
    mqtt_network_disconnect( socket );

    if(socket->is_client_auth_enabled == WICED_TRUE)
    {
        wiced_tls_deinit_identity( &socket->tls_identity );
        socket->is_client_auth_enabled = WICED_FALSE;
    }

    if ( socket->is_root_ca_used == WICED_TRUE )
    {
        wiced_tls_deinit_root_ca_certificates( );
        socket->is_root_ca_used = WICED_FALSE;
    }
    wiced_tcp_delete_socket( &socket->socket );
    return WICED_SUCCESS;
}

static wiced_result_t mqtt_frame_receive_fragmented( mqtt_connection_t* conn, uint8_t* data, uint32_t length, uint8_t chunked, uint8_t first_fragment, mqtt_frame_tcp_wrapper_t* packet )
{
    wiced_mqtt_buffer_t     buffer;
    uint32_t size;
    wiced_result_t result = WICED_ERROR;

    if( chunked == FRAME_NO_OVERLAP_NETWORK_FRAGMENT )
    {
        buffer.data     = data;
        buffer.packet   = packet->packet;
        result = mqtt_frame_recv( &buffer, (uint16_t)length, (void*)conn, &size);
        return result;
    }
    else if( chunked == FRAME_OVERLAPS_NETWORK_FRAGMENTS )
    {
        if( first_fragment )
        {
            buffer.data = data;
            buffer.packet = packet->packet;
            result = mqtt_frame_recv( &buffer, (uint16_t)length, (void*)conn, &size);
            return result;
        }
        else
        {
            mqtt_frame_recv_data_fragments( data, (uint16_t)length, (void* )conn );
        }
    }
    else if( chunked == FRAME_OVERLAPS_NETWORK_PACKETS )
    {
        if( first_fragment )
        {
            buffer.data = data;
            buffer.packet = packet->packet;
            result = mqtt_frame_recv(&buffer, (uint16_t)length, (void*)conn, &size);
        }
        else
        {
            mqtt_frame_recv_data_fragments( data, (uint16_t)length, (void* )conn );
        }
    }

    return WICED_SUCCESS;
}

/* This function takes care of any stack-level fragmentation happening for TCP packets. For example -
 * Network stack may choose to queue multiple fragments for a single TCP packet
 *  |===================================================================|
    |    ___________________________________________________________    |
    |   |           |           |           |       |               |   |
    |   |           |           |           |       |               |   |
    |   | NX-FRAG 1 | NX-FRAG 2 | NX-FRAG 3 |   ... | NX-FRAG (n)   |   |
    |   |           |           |           |       |               |   |
    |   |           |           |           |       |               |   |
    |   |___________|___________|___________|_______|_______________|   |
    |                  TCP PACKET 1                                     |
    |===================================================================|
 */

static wiced_result_t mqtt_consume_tcp_packet( mqtt_connection_t* conn, mqtt_frame_tcp_wrapper_t* pkt, uint32_t bytes_to_consume, uint8_t chunked, uint8_t is_pending )
{
    wiced_result_t ret = WICED_ERROR;
    uint8_t first_fragment = 1;
    uint16_t available_packet = 0;
    /* Higher layer frame is encapsulated in one or less TCP network packet */
    if( chunked == 0 )
    {
        /* Send the bytes to MQTT/Higher layer protocol here; note that this will move the data-pointer as well */
        ret = mqtt_frame_receive_fragmented( conn, pkt->data_start, bytes_to_consume, chunked, first_fragment, pkt );

        /* Increase the 'consumed' counter of the packet */
        pkt->consumed_bytes = pkt->consumed_bytes + bytes_to_consume;

        /* Decrease the 'remaining' counter of the packet */
        pkt->remaining_bytes = pkt->remaining_bytes - bytes_to_consume;

        /* If Packet is not exhausted yet and we've consumed the fragment of the packet completely;
           Move the data-pointer counter to next-fragment of the packet */
        if( bytes_to_consume == pkt->frag_length && pkt->remaining_bytes > 0 )
        {
            wiced_packet_get_data(pkt->packet, (uint16_t)bytes_to_consume, &pkt->data_start, (uint16_t*)&pkt->frag_length, &available_packet );
        }
        return ret;
    }

    /*
     * Data is spread over multiple fragments of a single TCP packet
    |===================================================================|
    |    ___________________________________________________________    |
    |   |           |           |           |       |               |   |
    |   |           |           |           |       |               |   |
    |   | NX-FRAG 1 | NX-FRAG 2 | NX-FRAG 3 |   ... | NX-FRAG (n)   |   |
    |   |           |           |           |       |               |   |
    |   |           |           |           |       |               |   |
    |   |___________|___________|___________|_______|_______________|   |
    |                  TCP PACKET 1                                     |
    |===================================================================|
    */
    else if( chunked == 1 )
    {
        while( bytes_to_consume )
        {
            uint16_t nr_bytes = (uint16_t)MIN(pkt->frag_length, bytes_to_consume);
            /* if data received is for 'pending-bytes' of previously receive frame, then mark it appropriately as not-first */
            if( is_pending == 1 )
            {
                first_fragment = 0;
            }
            ret = mqtt_frame_receive_fragmented( conn, pkt->data_start, nr_bytes, chunked, first_fragment, pkt );
            /* 'consumed' counter */
            pkt->consumed_bytes = pkt->consumed_bytes + nr_bytes;

            /* 'available' counter */
            pkt->remaining_bytes -= nr_bytes;

            /* update if data is still there */
            bytes_to_consume     -=  nr_bytes;

            /* First-fragment got consumed */
            first_fragment = 0;

            if( pkt->remaining_bytes > 0 )
            {
                wiced_packet_get_data( pkt->packet, pkt->frag_length, &pkt->data_start, (uint16_t*)&pkt->frag_length, &available_packet );
            }
        }
        return ret;
    }

    /* If Data is spread over multiple TCP packets;consume all the fragments of the packet */
    else if( chunked == 2 )
    {
        while( bytes_to_consume )
        {
            /* if data received is for 'pending-bytes' of previously receive frame, then mark it appropriately as not-first */
            if( is_pending == 1 )
            {
                first_fragment = 0;
            }
            /* consume one fragment at a time as MQTT frame size is larger than a single packet */
            ret = mqtt_frame_receive_fragmented( conn, pkt->data_start, pkt->frag_length, chunked, first_fragment, pkt );

            pkt->consumed_bytes = pkt->consumed_bytes + pkt->frag_length;

            pkt->remaining_bytes -= pkt->frag_length;

            bytes_to_consume -= pkt->frag_length;

            first_fragment = 0;

            wiced_packet_get_data( pkt->packet, pkt->frag_length, &pkt->data_start, (uint16_t*)&pkt->frag_length, &available_packet );
        }
        return ret;
    }
    else
    {
        /* do nothing */
    }
    return ret;
}

static wiced_result_t parse_for_expected_length( mqtt_frame_tcp_wrapper_t* pkt, uint32_t* size, mqtt_frame_type_t* type )
{
    uint8_t min_mqtt_header_size = 2;
    uint32_t new_size = 0;
    mqtt_raw_buffer_t buffer;
    *size = 0;

    if( pkt->total_bytes < min_mqtt_header_size )
    {
        /* A corner case where starting few bytes of MQTT frame is at end of first TCP segment and
         * rest of frame follows in next TCP segment is possible;
         * We are not handling this case right now */
        return WICED_ERROR;
    }

    buffer.data = pkt->data_start;

    if( !buffer.data )
    {
        return WICED_BADARG;
    }

    /* fetch MQTT frame type */
    *type = ( ( *(buffer.data) & 0xF0 ) >> 4 );

    /* Account for fixed-header 1st byte */
    new_size += 1;

    buffer.data++;

    switch( *type )
    {
        case MQTT_PACKET_TYPE_PUBLISH:
        {
            uint32_t value = 0;
            uint32_t multiplier = 1;
            uint8_t* data_gb;
            uint8_t multi_bytes = 0;
            do
            {
                if ( multi_bytes < 4 )
                {
                    data_gb = buffer.data++ ;
                    multi_bytes++ ;
                    value += ( ( *data_gb ) & 0x7F ) * multiplier;
                }
                else
                {
                    return WICED_ERROR;
                }
                multiplier *= 128;
            } while ( ( ( *data_gb ) & 0x80 ) != 0 );

            new_size += value;
            new_size += multi_bytes;
            *size = new_size;

            break;
        }
        case MQTT_PACKET_TYPE_UNSUBACK:
            *size = 4;
            break;
        case MQTT_PACKET_TYPE_SUBACK:
            *size = 5;
            break;
        case MQTT_PACKET_TYPE_CONNACK:
            *size = 4;
            break;
        case MQTT_PACKET_TYPE_PUBACK:
            *size = 4;
            break;
        case MQTT_PACKET_TYPE_PUBREC:
            *size = 4;
            break;
        case MQTT_PACKET_TYPE_PUBREL:
            *size = 4;
            break;
        case MQTT_PACKET_TYPE_PUBCOMP:
            *size = 4;
            break;
        case MQTT_PACKET_TYPE_PINGRESP:
            *size = 2;
            break;
        case MQTT_PACKET_TYPE_PINGREQ:
        case MQTT_PACKET_TYPE_SUBSCRIBE:
        case MQTT_PACKET_TYPE_UNSUBSCRIBE:
        case MQTT_PACKET_TYPE_CONNECT:
        case MQTT_PACKET_TYPE_DISCONNECT:
        default:
            *size = 0;
            return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

static wiced_result_t do_receive_pending( mqtt_connection_t* conn, mqtt_frame_tcp_wrapper_t* pkt, uint32_t* still_pending, uint8_t* packet_exhausted )
{
    uint32_t pending = 0;
    uint32_t temp = 0;
    (void)temp;
    /* if there is no pending data or no cached frame, we should not be reaching here */
    if( !conn || conn->cached_frame.data_pending == 0 )
    {
        WPRINT_LIB_INFO(("[MQTT-LIB] Data pending is zero or Connection is NULL\r\n"));
        return WICED_ERROR;
    }

    pending = conn->cached_frame.data_pending;

    /* if 'pending-bytes' are less than or equal to total-bytes available in current packet */
    if( pending <= pkt->total_bytes )
    {
        /* We are done with previous frame's pending-bytes...no more pending data */

        /* if total-bytes available in current-packet is equal to 'pending-bytes' */
        if( pending == pkt->total_bytes )
        {
            /* We are done with the packet too */
            *packet_exhausted = 1;
        }
        else
        {
            /* There might be another MQTT frame tucked in this packet; flag it accordingly..packet is not done yet */
            *packet_exhausted = 0;
        }
        /* And consume it/pass it to higher layer; note that data could be spread over multiple fragments of same TCP packet */
        mqtt_consume_tcp_packet(conn, pkt, pending, FRAME_OVERLAPS_NETWORK_FRAGMENTS, 1 );
        temp = pending;
        pending = 0;
    }

    /* total-bytes available in current-packet is still less than what is pending */
    else if( pending > pkt->total_bytes )
    {
        /* update 'pending-bytes' */
        pending = pending - pkt->total_bytes;

        temp = pkt->total_bytes;

        /* consume whatever has arrived in this packet */
        mqtt_consume_tcp_packet( conn, pkt, pkt->total_bytes, FRAME_OVERLAPS_NETWORK_PACKETS, 1 );

        /* this packet is going to be consumed completely */
        *packet_exhausted = 1;
    }
    else
    {
        /* Do nothing */
    }

    WPRINT_LIB_DEBUG( ("\t\t<--- Received %d bytes more(total: %d pending: %d) in TCP packet(size:%d)\n",
               (int)temp, (int)conn->cached_frame.total_bytes, (int)pending, (int)pkt->total_bytes) );

    /* update the flags */
    *still_pending                      = pending;
    conn->cached_frame.data_pending     = pending;

    return WICED_SUCCESS;
}

static wiced_result_t create_wrapper_packet( wiced_packet_t* tcp_packet, mqtt_frame_tcp_wrapper_t* pkt )
{
    wiced_result_t result = WICED_ERROR;
    uint16_t offset = 0;
    uint16_t total_bytes = 0;

    if( !tcp_packet || !pkt )
    {
        return result;
    }

    /* Get first fragment of the packet */
    result = wiced_packet_get_data( tcp_packet, offset, &pkt->data_start, &pkt->frag_length, &total_bytes );
    if( result != WICED_SUCCESS || total_bytes == 0 )
    {
        WPRINT_LIB_INFO( ("[MQTT-LIB] Error getting TCP Packet Data\n" ) );
        return WICED_ERROR;
    }

    WPRINT_LIB_DEBUG(("[MQTT-LIB] Available TCP Wrapper Packet( available:%x frag:%d )\n", (unsigned int)total_bytes, (int)pkt->frag_length ) );

    /* Wrapper packet just created; consumed bytes hence zero */
    pkt->consumed_bytes         = 0;
    pkt->total_bytes            = total_bytes;
    pkt->remaining_bytes        = total_bytes;
    pkt->packet                 = tcp_packet;

    return WICED_SUCCESS;
}

/* come here as result of receive_callback */
static void mqtt_process_tcp_receive( void *args )
{
    wiced_packet_t* packet = NULL;
    uint8_t packet_exhausted = 0;
    mqtt_frame_tcp_wrapper_t pkt;
    mqtt_connection_t* conn = (mqtt_connection_t *)args;

    mqtt_socket_t* socket = &conn->socket;
    wiced_result_t result = WICED_ERROR;
    uint32_t bytes_expected = 0;
    uint8_t chunked = 0;

    /* take the lock */
    wiced_rtos_lock_mutex( &conn->lock );

    /* get the TCP packet */
    result = wiced_tcp_receive( &socket->socket, &packet, WICED_NO_WAIT );
    if( result != WICED_SUCCESS )
    {
        WPRINT_LIB_INFO(("[MQTT-LIB] TCP receive error\r\n"));
        goto exit;
    }

    /* Put it an assembly-packet wrapper; Also fetches data from TCP packet */
    result = create_wrapper_packet( packet, &pkt );
    if( result != WICED_SUCCESS )
    {
        WPRINT_LIB_INFO(("[MQTT-LIB] TCP get packet data error\r\n"));
        goto exit;
    }

    conn->cached_frame.current_packet = &pkt;

    /* check if this packet is continuation of previously received frames */
    if( conn->cached_frame.data_pending )
    {
        uint32_t still_pending = 0;

        /* Receive and consume whatever we could */
        result = do_receive_pending(conn, &pkt, &still_pending, &packet_exhausted);

        if( result != WICED_SUCCESS )
        {
            WPRINT_LIB_INFO(("[MQTT-LIB] Error while consuming pending bytes\r\n"));
            goto exit;
        }

        /* if current packet is exhausted and still the frame awaits more data, it will come in another receive_callback */
        if( still_pending && packet_exhausted )
        {
            /* delete this packet and let another receive_callback come */
            goto exit;
        }
    }

    /* Packet got exhausted while fulfilling last pending packet; nothing else left, release it and wait for another call to come */
    if( packet_exhausted )
    {
        /* delete this one and let another receive_callback come */
        wiced_rtos_unlock_mutex( &conn->lock );
        wiced_packet_delete(packet);
        return;
    }

    /* Start of new frame - give it to higher layer and deduce what is the total-size of the frame */
    while( !packet_exhausted )
    {
        mqtt_frame_type_t type;
        result = parse_for_expected_length( &pkt, &bytes_expected, &type );

        conn->cached_frame.total_bytes = bytes_expected;

        if( result != WICED_SUCCESS || bytes_expected == 0)
        {
            WPRINT_LIB_INFO(("[MQTT-LIB] Error fetching expected length\r\n"));
            goto exit;
        }

        WPRINT_LIB_DEBUG(("[MQTT-LIB] Received New MQTT frame (size: %d type:%d)\n", (int)bytes_expected, type) );

        /* TCP data expected by higher layer is sitting in a single TCP Packet fragment */
        if( bytes_expected <= pkt.frag_length )
        {
            chunked = 0;
            mqtt_consume_tcp_packet( conn, &pkt, bytes_expected, chunked, 0 );
            packet_exhausted = ( (pkt.consumed_bytes < pkt.total_bytes) ? 0 : 1 );
            continue;
        }
        else if( bytes_expected <= pkt.total_bytes )
        {
            chunked = 1;
            mqtt_consume_tcp_packet( conn, &pkt, bytes_expected, chunked, 0 );
            packet_exhausted = ((pkt.consumed_bytes < pkt.total_bytes) ? 0 : 1 );
            continue;
        }
        else if( bytes_expected > pkt.total_bytes )
        {
            chunked = 2;
            conn->cached_frame.data_pending = bytes_expected - pkt.total_bytes;
            WPRINT_LIB_DEBUG( ("\tStart Assembling...\n" ) );
            WPRINT_LIB_DEBUG( ("\t\t<--- Received %d bytes more(total: %d pending: %d) in TCP packet(size:%d)\n",
                (int)pkt.total_bytes, (int)conn->cached_frame.total_bytes, (int)conn->cached_frame.data_pending, (int)pkt.total_bytes) );
            mqtt_consume_tcp_packet( conn, &pkt, pkt.total_bytes, chunked, 0 );

            packet_exhausted = 1;

            continue;
        }
        else
        {
            /* Do nothing */
        }
    }

exit:
    if( packet_exhausted == 1)
    {
        WPRINT_LIB_DEBUG(("[MQTT-LIB] Received packet consumed SUCCESS\r\n"));
    }
    else
    {
        WPRINT_LIB_DEBUG(("[MQTT-LIB] Received Packet ERROR\r\n"));
    }
    /* Packet exhausted now or some other error came... Delete it. And allow another packet to be processed */
    if( packet )
    {
        wiced_packet_delete( packet );
    }
    wiced_rtos_unlock_mutex( &conn->lock );
}

static void mqtt_thread_main( uint32_t arg )
{
    mqtt_connection_t *conn = (mqtt_connection_t*)arg;
    mqtt_socket_t *socket = (mqtt_socket_t*) &conn->socket;
    mqtt_event_message_t current_event;
    wiced_result_t result = WICED_SUCCESS;

    while ( conn->core_init == WICED_TRUE )
    {
        if ( ( result = wiced_rtos_pop_from_queue( &socket->queue, &current_event, WICED_NEVER_TIMEOUT ) ) != WICED_SUCCESS )
        {
            current_event.event_type = MQTT_ERROR_EVENT;
        }

        switch ( current_event.event_type )
        {
            case MQTT_RECEIVE_EVENT:
                mqtt_process_tcp_receive( (void *)arg );
                break;

            case MQTT_SEND_EVENT:
                mqtt_manager( current_event.send_context.event_t, &current_event.send_context.args, current_event.send_context.conn );
                break;

            case MQTT_ERROR_EVENT:
                break;

            case MQTT_DISCONNECT_EVENT:
                mqtt_frame_recv( NULL, 0, socket->p_user, NULL );
                break;

            default:
                break;
        }
    }
    WICED_END_OF_CURRENT_THREAD( );
}

wiced_result_t mqtt_network_connect( mqtt_socket_t *socket )
{
    return wiced_tcp_connect( &socket->socket, &socket->server_ip_address, socket->portnumber, WICED_MQTT_CONNECTION_TIMEOUT );
}

wiced_result_t mqtt_network_disconnect( mqtt_socket_t *socket )
{
    return wiced_tcp_disconnect( &socket->socket );
}

wiced_result_t mqtt_network_create_buffer( wiced_mqtt_buffer_t *buffer, uint16_t size, mqtt_socket_t *socket )
{
    uint16_t available_data_length;

    /* Create the TCP packet. Memory for the tx_data is automatically allocated */
    return wiced_packet_create_tcp( &socket->socket, size, &buffer->packet, &buffer->data, &available_data_length );
}

wiced_result_t mqtt_network_delete_buffer( wiced_mqtt_buffer_t *buffer )
{
    /* Create the TCP packet. Memory for the tx_data is automatically allocated */
    return wiced_packet_delete( buffer->packet );
}

wiced_result_t mqtt_network_send_buffer( const wiced_mqtt_buffer_t *buffer, mqtt_socket_t *socket )
{
    wiced_mqtt_network_message_t message;
    wiced_result_t result = WICED_SUCCESS;
    message.packet = buffer->packet;
    message.data = buffer->data;

    /*Set the end of the data portion*/
    wiced_packet_set_data_end( message.packet, message.data );
    result = wiced_tcp_send_packet( &socket->socket, message.packet );
    if ( result != WICED_SUCCESS )
    {
        /*Delete packet, since the send failed*/
        wiced_packet_delete( message.packet );
    }
    return result;
}

static wiced_result_t mqtt_disconnect_callback( wiced_tcp_socket_t *socket, void *args )
{
    mqtt_socket_t *mqtt_socket = (mqtt_socket_t *) args;
    mqtt_event_message_t current_event;
    UNUSED_PARAMETER(socket);
    current_event.event_type = MQTT_DISCONNECT_EVENT;
    return wiced_rtos_push_to_queue( &mqtt_socket->queue, &current_event, WICED_NO_WAIT );
}

static wiced_result_t mqtt_network_recv_thread( wiced_tcp_socket_t *socket, void *args )
{
    mqtt_event_message_t current_event;
    mqtt_socket_t *mqtt_socket = (mqtt_socket_t *) args;
    UNUSED_PARAMETER(socket);
    current_event.event_type = MQTT_RECEIVE_EVENT;
    return wiced_rtos_push_to_queue( &mqtt_socket->queue, &current_event, WICED_NO_WAIT );
}
