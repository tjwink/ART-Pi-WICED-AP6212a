/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *  Connection manager
 *
 *  Handles connection state and server exceptions.
 */

#include "wiced.h"
#include "mqtt_api.h"
#include "mqtt_internal.h"
#include "mqtt_connection.h"
#include "mqtt_frame.h"
#include "mqtt_manager.h"

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
static wiced_result_t mqtt_manager_resend_packet( mqtt_frame_type_t type, void *arg, void *p_user );
static wiced_result_t mqtt_manager_tick( void* arg );
static wiced_result_t mqtt_manager_heartbeat_init( uint16_t keep_alive, void *p_user, mqtt_heartbeat_t *heartbeat );
inline static void mqtt_manager_heartbeat_send_reset( mqtt_heartbeat_t *heartbeat );
inline static void mqtt_manager_heartbeat_recv_reset( mqtt_heartbeat_t *heartbeat );
inline static wiced_result_t mqtt_manager_heartbeat_send_step( mqtt_heartbeat_t *heartbeat );
inline static wiced_result_t mqtt_manager_heartbeat_recv_step( mqtt_heartbeat_t *heartbeat );
static void mqtt_manager_heartbeat_deinit( mqtt_heartbeat_t *heartbeat );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

static wiced_result_t mqtt_manager_tick( void* arg )
{
    mqtt_connection_t *conn = (mqtt_connection_t *) arg;
    if ( mqtt_manager( MQTT_EVENT_TICK, NULL, conn ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

static wiced_result_t mqtt_manager_heartbeat_init( uint16_t keep_alive, void *p_user, mqtt_heartbeat_t *heartbeat )
{
    heartbeat->step_value = 1;
    heartbeat->reset_value = keep_alive;
    heartbeat->send_counter = keep_alive;
    heartbeat->recv_counter = (uint32_t) 2 * keep_alive;
    return wiced_rtos_register_timed_event( &heartbeat->timer, WICED_NETWORKING_WORKER_THREAD, mqtt_manager_tick, heartbeat->step_value * 1000, p_user );
}

inline static void mqtt_manager_heartbeat_send_reset( mqtt_heartbeat_t *heartbeat )
{
    heartbeat->send_counter = heartbeat->reset_value;
}

inline static void mqtt_manager_heartbeat_recv_reset( mqtt_heartbeat_t *heartbeat )
{
    heartbeat->recv_counter = 2 * heartbeat->reset_value;
}

inline static wiced_result_t mqtt_manager_heartbeat_send_step( mqtt_heartbeat_t *heartbeat )
{
    if ( heartbeat->send_counter >= heartbeat->step_value )
    {
        heartbeat->send_counter -= heartbeat->step_value;
    }
    else
    {
        heartbeat->send_counter = 0;
    }
    return ( heartbeat->send_counter == 0 ) ? WICED_ERROR : WICED_SUCCESS;
}

inline static wiced_result_t mqtt_manager_heartbeat_recv_step( mqtt_heartbeat_t *heartbeat )
{
    if ( heartbeat->recv_counter >= heartbeat->step_value )
    {
        heartbeat->recv_counter -= heartbeat->step_value;
    }
    else
    {
        heartbeat->recv_counter = 0;
    }
    return ( heartbeat->recv_counter == 0 ) ? WICED_ERROR : WICED_SUCCESS;
}

static void mqtt_manager_heartbeat_deinit( mqtt_heartbeat_t *heartbeat )
{
    wiced_rtos_deregister_timed_event( &heartbeat->timer );
}

wiced_result_t mqtt_manager( mqtt_event_t event, void *args, mqtt_connection_t *conn )
{
    wiced_result_t result = WICED_SUCCESS;
    if ( conn->network_status == MQTT_NETWORK_CONNECTED )
     {
        switch ( event )
        {
            case MQTT_EVENT_SEND_CONNECT:
            {
                mqtt_connect_arg_t *connect_arg = (mqtt_connect_arg_t *) args;
                if ( connect_arg->keep_alive )
                {
                    mqtt_manager_heartbeat_init( connect_arg->keep_alive, conn, &conn->heartbeat );
                }
                mqtt_backend_put_connect( args, conn );
            }
                break;

            case MQTT_EVENT_SEND_DISCONNECT:
            {
                mqtt_backend_put_disconnect( conn );
                mqtt_manager_heartbeat_deinit( &conn->heartbeat );
                mqtt_backend_connection_close( conn );
            }
                break;

            case MQTT_EVENT_SEND_SUBSCRIBE:
            {
                mqtt_backend_put_subscribe( args, conn );
                mqtt_manager_heartbeat_send_reset( &conn->heartbeat );
                if ( mqtt_session_add_item( MQTT_PACKET_TYPE_SUBSCRIBE, args, conn->session ) != WICED_SUCCESS )
                {
                    /* TODO : error handling */
                    WPRINT_LIB_ERROR( ("[MQTT] Adding subscribe to session queue fail.\n ") );
                }
            }
                break;

            case MQTT_EVENT_SEND_UNSUBSCRIBE:
            {
                mqtt_backend_put_unsubscribe( args, conn );
                mqtt_manager_heartbeat_send_reset( &conn->heartbeat );
                if ( mqtt_session_add_item( MQTT_PACKET_TYPE_UNSUBSCRIBE, args, conn->session ) != WICED_SUCCESS )
                {
                    /* TODO : error handling */
                    WPRINT_LIB_ERROR( ("[MQTT] Adding unsubscribe to session queue fail.\n ") );
                }
            }
                break;

            case MQTT_EVENT_SEND_PUBLISH:
            {
                mqtt_publish_arg_t *publish_args = (mqtt_publish_arg_t *) args;
                if ( mqtt_session_add_item( MQTT_PACKET_TYPE_PUBLISH, args, conn->session ) != WICED_SUCCESS )
                {
                    /* TODO : error handling */
                    WPRINT_LIB_ERROR( ("[MQTT] Adding publish to session queue fail.\n ") );
                }
                result = mqtt_backend_put_publish( args, conn );
                if ( ( publish_args->qos != MQTT_QOS_DELIVER_AT_MOST_ONCE ) )
                {
                    mqtt_manager_heartbeat_send_reset( &conn->heartbeat );
                }

                if ( publish_args->qos == MQTT_QOS_DELIVER_AT_MOST_ONCE )
                {
                    if ( mqtt_session_remove_item( MQTT_PACKET_TYPE_PUBLISH, publish_args->packet_id, conn->session ) != WICED_SUCCESS )
                    {
                        /* TODO : error handling */
                        WPRINT_LIB_ERROR(( "[MQTT] publish with Qos level 0 %d not in session queue.\n ", publish_args->packet_id ));
                    }
                    if ( ( result == WICED_SUCCESS ) & ( conn->callbacks != NULL ) )
                    {
                        wiced_mqtt_event_info_t callback_event;
                        callback_event.type = WICED_MQTT_EVENT_TYPE_PUBLISHED;
                        conn->callbacks( (void*) conn, &callback_event );
                    }
                }
            }
                break;

            case MQTT_EVENT_RECV_PUBLISH:
            {
                mqtt_publish_arg_t *publish_args = (mqtt_publish_arg_t *) args;
                mqtt_manager_heartbeat_recv_reset( &conn->heartbeat );
                if ( publish_args->qos == MQTT_QOS_DELIVER_AT_LEAST_ONCE )
                {
                    mqtt_puback_arg_t puback_args;
                    puback_args.packet_id = publish_args->packet_id;
                    mqtt_backend_put_puback( &puback_args, conn );
                    mqtt_manager_heartbeat_send_reset( &conn->heartbeat );
                }
                else if ( publish_args->qos == MQTT_QOS_DELIVER_EXACTLY_ONCE )
                {
                    mqtt_pubrec_arg_t pubrec_args;
                    pubrec_args.packet_id = publish_args->packet_id;
                    mqtt_backend_put_pubrec( &pubrec_args, conn );
                    mqtt_manager_heartbeat_send_reset( &conn->heartbeat );
                    if ( mqtt_session_item_exist( MQTT_PACKET_TYPE_PUBREC, publish_args->packet_id, conn->session ) != WICED_SUCCESS )
                    {
                        /* new publish packet */
                        if ( mqtt_session_add_item( MQTT_PACKET_TYPE_PUBREC, &pubrec_args, conn->session ) != WICED_SUCCESS )
                        {
                            /* TODO : error handling */
                            WPRINT_LIB_ERROR( ("[MQTT] Adding pubrec to session queue fail.\n ") );
                        }
                    }
                    else
                    {
                        /* This item is already received before shouldn't be passed to user */
                        publish_args->data = NULL;
                    }
                }
            }
                break;

            case MQTT_EVENT_RECV_SUBACK:
            {
                wiced_mqtt_suback_arg_t *suback_args = (wiced_mqtt_suback_arg_t *) args;
                mqtt_manager_heartbeat_recv_reset( &conn->heartbeat );
                if ( mqtt_session_remove_item( MQTT_PACKET_TYPE_SUBSCRIBE, suback_args->packet_id, conn->session ) != WICED_SUCCESS )
                {
                    /* TODO : error handling */
                    WPRINT_LIB_ERROR( ("[MQTT] suback %d not in session queue.\n ", suback_args->packet_id ) );
                }
            }
                break;

            case MQTT_EVENT_RECV_UNSUBACK:
            {
                mqtt_unsuback_arg_t *unsuback_args = (mqtt_unsuback_arg_t *) args;
                mqtt_manager_heartbeat_recv_reset( &conn->heartbeat );
                if ( mqtt_session_remove_item( MQTT_PACKET_TYPE_UNSUBSCRIBE, unsuback_args->packet_id, conn->session ) != WICED_SUCCESS )
                {
                    /* TODO : error handling */
                    WPRINT_LIB_ERROR( ("[MQTT] unsuback %d not in session queue.\n ", unsuback_args->packet_id) );
                }
            }
                break;

            case MQTT_EVENT_RECV_PUBACK:
            {
                mqtt_puback_arg_t *puback_args = (mqtt_puback_arg_t *) args;
                mqtt_manager_heartbeat_recv_reset( &conn->heartbeat );
                if ( mqtt_session_remove_item( MQTT_PACKET_TYPE_PUBLISH, puback_args->packet_id, conn->session ) != WICED_SUCCESS )
                {
                    /* TODO : error handling */
                    WPRINT_LIB_ERROR( ("[MQTT] puback %d not in session queue.\n ", puback_args->packet_id) );
                }
            }
                break;

            case MQTT_EVENT_RECV_PUBREC:
            {
                mqtt_pubrec_arg_t *pubrec_args = (mqtt_pubrec_arg_t *) args;
                mqtt_pubrel_arg_t pubrel_args;
                pubrel_args.packet_id = pubrec_args->packet_id;

                mqtt_manager_heartbeat_recv_reset( &conn->heartbeat );
                if ( mqtt_session_remove_item( MQTT_PACKET_TYPE_PUBLISH, pubrec_args->packet_id, conn->session ) != WICED_SUCCESS )
                {
                    WPRINT_LIB_ERROR( ("[MQTT] publish %d not in session queue.\n ", pubrec_args->packet_id) );
                }

                mqtt_backend_put_pubrel( &pubrel_args, conn );
                mqtt_manager_heartbeat_send_reset( &conn->heartbeat );
                if ( mqtt_session_item_exist( MQTT_PACKET_TYPE_PUBREL, pubrel_args.packet_id, conn->session ) != WICED_SUCCESS )
                {
                    if ( mqtt_session_add_item( MQTT_PACKET_TYPE_PUBREL, &pubrel_args, conn->session ) != WICED_SUCCESS )
                    {
                        /* TODO : error handling */
                        WPRINT_LIB_ERROR( ("[MQTT] Adding pubrel to session queue fail.\n ") );
                    }
                }
            }
                break;

            case MQTT_EVENT_RECV_PUBREL:
            {
                mqtt_pubrel_arg_t *pubrel_args = (mqtt_pubrel_arg_t *) args;
                mqtt_pubcomp_arg_t pubcomp_args;
                pubcomp_args.packet_id = pubrel_args->packet_id;

                mqtt_manager_heartbeat_recv_reset( &conn->heartbeat );
                if ( mqtt_session_remove_item( MQTT_PACKET_TYPE_PUBREC, pubrel_args->packet_id, conn->session ) != WICED_SUCCESS )
                {
                    /* TODO : error handling */
                    WPRINT_LIB_ERROR( ("[MQTT] pubrec %d not in session queue.\n ", pubrel_args->packet_id) );
                }
                mqtt_backend_put_pubcomp( &pubcomp_args, conn );
                mqtt_manager_heartbeat_send_reset( &conn->heartbeat );
            }
                break;

            case MQTT_EVENT_RECV_PUBCOMP:
            {
                mqtt_pubcomp_arg_t *pubcomp_args = (mqtt_pubcomp_arg_t *) args;

                mqtt_manager_heartbeat_recv_reset( &conn->heartbeat );
                if ( mqtt_session_remove_item( MQTT_PACKET_TYPE_PUBREL, pubcomp_args->packet_id, conn->session ) != WICED_SUCCESS )
                {
                    /* TODO : error handling */
                    WPRINT_LIB_ERROR( ("[MQTT] pubrel %d not in session queue.\n ", pubcomp_args->packet_id) );
                }
            }
                break;

            case MQTT_EVENT_RECV_PINGRES:
            case MQTT_EVENT_RECV_CONNACK:
            {
                mqtt_manager_heartbeat_recv_reset( &conn->heartbeat );
                /* Resend any thing in the session */
                if ( mqtt_session_iterate_through_items( mqtt_manager_resend_packet, conn, conn->session ) != WICED_SUCCESS )
                {
                    /* TODO : error handling */
                    WPRINT_LIB_ERROR( ("[MQTT] Error resending session messages.\n " ) );
                }
            }
                break;

            case MQTT_EVENT_TICK:
            {
                if ( mqtt_manager_heartbeat_recv_step( &conn->heartbeat ) != WICED_SUCCESS )
                {
                    /* Reset counter timed out, since we didn't receive any mqtt packet from broker */
                    mqtt_socket_t *mqtt_socket = (mqtt_socket_t *) &conn->socket;
                    mqtt_event_message_t current_event;

                    current_event.event_type = MQTT_DISCONNECT_EVENT;
                    return wiced_rtos_push_to_queue( &mqtt_socket->queue, &current_event, WICED_NO_WAIT );
                }
                else
                {
                    if ( mqtt_manager_heartbeat_send_step( &conn->heartbeat ) != WICED_SUCCESS )
                    {
                        mqtt_backend_put_pingreq( conn );
                        mqtt_manager_heartbeat_send_reset( &conn->heartbeat );
                    }
                }
            }
                break;

            case MQTT_EVENT_CONNECTION_CLOSE:
            {
                mqtt_connection_deinit( conn );
                mqtt_manager_heartbeat_deinit( &conn->heartbeat );

            }
                break;

            case MQTT_EVENT_SEND_PINGREQ:
            case MQTT_EVENT_SEND_PUBREL:
            case MQTT_EVENT_SEND_PUBREC:
            case MQTT_EVENT_SEND_PUBCOMP:
            case MQTT_EVENT_SEND_PUBACK:
            default:
                break;
        }
    }
    else
    {
        WPRINT_LIB_ERROR( ("[MQTT LIB] Not connected\r\n ") );
        return WICED_ERROR;
    }
    return result;
}

static wiced_result_t mqtt_manager_resend_packet( mqtt_frame_type_t type, void *arg, void *p_user )
{
    mqtt_connection_t *conn = (mqtt_connection_t*) p_user;
    wiced_result_t result;
    switch ( type )
    {
        case MQTT_PACKET_TYPE_SUBSCRIBE:
        {
            result = mqtt_backend_put_subscribe( arg, conn );
        }
            break;
        case MQTT_PACKET_TYPE_UNSUBSCRIBE:
        {
            result = mqtt_backend_put_unsubscribe( arg, conn );
        }
            break;
        case MQTT_PACKET_TYPE_PUBLISH:
        {
            result = mqtt_backend_put_publish( arg, conn );
        }
            break;
        case MQTT_PACKET_TYPE_PUBREC:
        {
            result = mqtt_backend_put_pubrec( arg, conn );
        }
            break;
        case MQTT_PACKET_TYPE_PUBREL:
        {
            result = mqtt_backend_put_pubrel( arg, conn );
        }
            break;
        case MQTT_PACKET_TYPE_PUBCOMP:
        case MQTT_PACKET_TYPE_PUBACK:
        case MQTT_PACKET_TYPE_UNSUBACK:
        case MQTT_PACKET_TYPE_SUBACK:
        case MQTT_PACKET_TYPE_CONNACK:
        case MQTT_PACKET_TYPE_PINGREQ:
        case MQTT_PACKET_TYPE_PINGRESP:
        case MQTT_PACKET_TYPE_CONNECT:
        case MQTT_PACKET_TYPE_DISCONNECT:
        default:
            return WICED_ERROR;
    }
    return result;
}
