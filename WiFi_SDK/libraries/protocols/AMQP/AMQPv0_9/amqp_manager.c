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
 *  Connection manager
 *
 *  Handles connection state and server exceptions.
 */

#include "wiced.h"
#include "amqp.h"
#include "amqp_connection.h"
#include "amqp_frame.h"
#include "amqp_manager.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define IS_RECV_EVENT(X)    ( (X) >= AMQP_EVENT_CONNECTION_RECV_START )

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
static wiced_result_t amqp_manager_tick( void* arg )
{
    wiced_amqp_connection_t  *conn = (wiced_amqp_connection_t *)arg;
    if ( amqp_manager( AMQP_EVENT_MANAGER_TICK, NULL, 0, conn ) != WICED_SUCCESS )
    {
        if ( conn->callbacks.connection_event != NULL )
        {
            conn->callbacks.connection_event( AMQP_EVENT_CONNECTION_ERROR, NULL, conn );
        }
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

static
wiced_result_t amqp_manager_heartbeat_init( wiced_amqp_heartbeat_t *heartbeat, wiced_amqp_connection_t *conn )
{
    return wiced_rtos_register_timed_event( &heartbeat->timer, WICED_NETWORKING_WORKER_THREAD, amqp_manager_tick, heartbeat->step_value * 1000, conn );
}

inline static
void amqp_manager_heartbeat_send_reset( wiced_amqp_heartbeat_t *heartbeat )
{
    heartbeat->send_counter = heartbeat->reset_value;
}

inline static
void amqp_manager_heartbeat_recv_reset( wiced_amqp_heartbeat_t *heartbeat )
{
    heartbeat->recv_counter = 2 * heartbeat->reset_value;
}

inline static
wiced_result_t amqp_manager_heartbeat_send_step( wiced_amqp_heartbeat_t *heartbeat )
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

inline static
wiced_result_t amqp_manager_heartbeat_recv_step( wiced_amqp_heartbeat_t *heartbeat )
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

static
void amqp_manager_heartbeat_handle_event( wiced_amqp_event_t event, wiced_amqp_heartbeat_t *heartbeat)
{
    if ( IS_RECV_EVENT( event ) )
    {
        amqp_manager_heartbeat_recv_reset( heartbeat );
    }
    else
    {
        amqp_manager_heartbeat_send_reset( heartbeat );
    }
}

static
void amqp_manager_heartbeat_deinit( wiced_amqp_heartbeat_t *heartbeat )
{
    wiced_rtos_deregister_timed_event( &heartbeat->timer );
}

static
wiced_result_t amqp_channel_manager( wiced_amqp_event_t event, void *args, uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t result = WICED_SUCCESS ;

    if ( 0 != channel )
    {
        amqp_manager_heartbeat_handle_event( event, &conn->heartbeat );
        switch ( event )
        {
            case AMQP_EVENT_CHANNEL_SEND_OPEN:
            {
                amqp_channel_backend_put_open( channel, conn );
            }
            break;
            case AMQP_EVENT_CHANNEL_SEND_CLOSE:
            {
                amqp_channel_backend_put_close( channel, (wiced_amqp_channel_close_arg_t*)args, conn );
            }
            break;
            case AMQP_EVENT_CHANNEL_SEND_FLOW:
            {
                amqp_channel_backend_put_flow( channel, args, conn );
            }
            break;
            case AMQP_EVENT_EXCHANGE_SEND_DECLARE:
            {
                amqp_channel_exchange_backend_put_declare( channel, args, conn );
            }
            break;
            case AMQP_EVENT_EXCHANGE_SEND_DELETE:
            {
                amqp_channel_exchange_backend_put_delete( channel, args, conn );
            }
            break;
            case AMQP_EVENT_QUEUE_SEND_DECLARE:
            {
                amqp_channel_queue_backend_put_declare( channel, args, conn );
            }
            break;
            case AMQP_EVENT_QUEUE_SEND_BIND:
            {
                amqp_channel_queue_backend_put_bind( channel, args, conn );
            }
            break;
            case AMQP_EVENT_QUEUE_SEND_UNBIND:
            {
                amqp_channel_queue_backend_put_unbind( channel, args, conn );
            }
            break;
            case AMQP_EVENT_QUEUE_SEND_PURGE:
            {
                amqp_channel_queue_backend_put_purge( channel, args, conn );
            }
            break;
            case AMQP_EVENT_QUEUE_SEND_DELETE:
            {
                amqp_channel_queue_backend_put_delete( channel, args, conn );
            }
            break;
            case AMQP_EVENT_BASIC_SEND_QOS:
            {
                amqp_channel_basic_backend_put_qos( channel, args, conn );
            }
            break;
            case AMQP_EVENT_BASIC_SEND_CONSUME:
            {
                amqp_channel_basic_backend_put_consume( channel, args, conn );
            }
            break;
            case AMQP_EVENT_BASIC_SEND_CANCEL:
            {
                amqp_channel_basic_backend_put_cancel( channel, args, conn );
            }
            break;
            case AMQP_EVENT_BASIC_SEND_PUBLISH:
            {
                amqp_channel_basic_backend_put_publish( channel, args, conn );
            }
            break;
            case AMQP_EVENT_BASIC_SEND_GET:
            {
                amqp_channel_basic_backend_put_get( channel, args, conn );
            }
            break;
            case AMQP_EVENT_BASIC_SEND_ACK:
            {
                amqp_channel_basic_backend_put_ack( channel, args, conn );
            }
            break;
            case AMQP_EVENT_BASIC_SEND_REJECT:
            {
                amqp_channel_basic_backend_put_reject( channel, args, conn );
            }
            break;
            case AMQP_EVENT_BASIC_SEND_RECOVER:
            {
                amqp_channel_basic_backend_put_recover( channel, args, conn );
            }
            break;
            case AMQP_EVENT_BASIC_SEND_RECOVER_ASYNC:
            {
                amqp_channel_basic_backend_put_recover_async( channel, args, conn );
            }
            break;
            case AMQP_EVENT_CONTENT_SEND_HEADER:
            {
                amqp_channel_content_backend_put_header( channel, args, conn );
            }
            break;
            case AMQP_EVENT_CONTENT_SEND_CONTENT:
            {
                amqp_channel_content_backend_put_content( args, conn );
            }
            break;
            case AMQP_EVENT_TX_SEND_SELECT:
            {
                amqp_channel_tx_backend_put_select( channel, conn );
            }
            break;
            case AMQP_EVENT_TX_SEND_COMMIT:
            {
                amqp_channel_tx_backend_put_commit( channel, conn );
            }
            break;
            case AMQP_EVENT_TX_SEND_ROLLBACK:
            {
                amqp_channel_tx_backend_put_rollback( channel, conn );
            }
            break;
            case AMQP_EVENT_CHANNEL_RECV_CLOSE       :
            {
                amqp_channel_backend_put_close_ok( channel, conn );
            }
            break;
            case AMQP_EVENT_CHANNEL_RECV_FLOW        :
            {
                amqp_channel_backend_put_flow_ok( channel, args, conn );
            }
            break;
            case AMQP_EVENT_CONTENT_RECV_CONTENT        :
            case AMQP_EVENT_PROTOCOL_SEND_HEADER        :
            case AMQP_EVENT_CONNECTION_SEND_CLOSE       :
            case AMQP_EVENT_BASIC_SEND_PUBLISH_OK       :
            case AMQP_EVENT_CONTENT_SEND_HEADER_OK      :
            case AMQP_EVENT_CONTENT_SEND_CONTENT_OK     :
            case AMQP_EVENT_CONNECTION_SEND_HEARTBEAT   :
            case AMQP_EVENT_CONNECTION_ERROR            :
            case AMQP_EVENT_MANAGER_TICK                :
            case AMQP_EVENT_CONNECTION_RECV_START       :
            case AMQP_EVENT_CONNECTION_RECV_TUNE        :
            case AMQP_EVENT_CONNECTION_RECV_OPEN_OK     :
            case AMQP_EVENT_CONNECTION_RECV_CLOSE       :
            case AMQP_EVENT_CONNECTION_RECV_CLOSE_OK    :
            case AMQP_EVENT_CHANNEL_RECV_OPEN_OK        :
            case AMQP_EVENT_CHANNEL_RECV_FLOW_OK        :
            case AMQP_EVENT_CHANNEL_RECV_CLOSE_OK       :
            case AMQP_EVENT_EXCHANGE_RECV_DECLARE_OK    :
            case AMQP_EVENT_EXCHANGE_RECV_DELETE_OK     :
            case AMQP_EVENT_QUEUE_RECV_DECLARE_OK       :
            case AMQP_EVENT_QUEUE_RECV_BIND_OK          :
            case AMQP_EVENT_QUEUE_RECV_UNBIND_OK        :
            case AMQP_EVENT_QUEUE_RECV_PURGE_OK         :
            case AMQP_EVENT_QUEUE_RECV_DELETE_OK        :
            case AMQP_EVENT_BASIC_RECV_QOS_OK           :
            case AMQP_EVENT_BASIC_RECV_CONSUME_OK       :
            case AMQP_EVENT_BASIC_RECV_CANCEL_OK        :
            case AMQP_EVENT_BASIC_RECV_RETURN           :
            case AMQP_EVENT_BASIC_RECV_DELIVER          :
            case AMQP_EVENT_BASIC_RECV_GET_OK           :
            case AMQP_EVENT_BASIC_RECV_GET_EMPTY        :
            case AMQP_EVENT_BASIC_RECV_RECOVER_OK       :
            case AMQP_EVENT_TX_RECV_SELECT_OK           :
            case AMQP_EVENT_TX_RECV_COMMIT_OK           :
            case AMQP_EVENT_TX_RECV_ROLLBACK_OK         :
            case AMQP_EVENT_CONTENT_RECV_HEADER         :
            case AMQP_EVENT_CONNECTION_RECV_HEARTBEAT   :
            case AMQP_EVENT_PROTOCOL_RECV_HEADER        :
                break;
            default:
                result = WICED_ERROR;
                break;
        }
    }
    else
    {
        result = WICED_ERROR;
    }

    return result;

}


wiced_result_t amqp_manager( wiced_amqp_event_t event, void *args, uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t result = WICED_SUCCESS ;

    wiced_rtos_get_semaphore( &conn->semaphore, NEVER_TIMEOUT );
    switch ( event )
    {
        case AMQP_EVENT_PROTOCOL_SEND_HEADER:
        case AMQP_EVENT_PROTOCOL_RECV_HEADER:
        case AMQP_EVENT_CONNECTION_SEND_CLOSE:
        case AMQP_EVENT_CONNECTION_RECV_START:
        case AMQP_EVENT_CONNECTION_RECV_TUNE:
        case AMQP_EVENT_CONNECTION_RECV_OPEN_OK:
        case AMQP_EVENT_CONNECTION_RECV_CLOSE_OK:
        case AMQP_EVENT_CONNECTION_RECV_CLOSE:
        case AMQP_EVENT_CONNECTION_RECV_HEARTBEAT:
        {
            if ( 0 == channel )
            {
                amqp_manager_heartbeat_handle_event( event, &conn->heartbeat );
                switch( event )
                {
                    case AMQP_EVENT_PROTOCOL_SEND_HEADER:
                    {
                        amqp_connection_backend_put_protocol_header( args, conn );
                    }
                    break;
                    case AMQP_EVENT_CONNECTION_SEND_CLOSE:
                    {
                        amqp_connection_backend_put_close( args, conn );
                        amqp_manager_heartbeat_deinit( &conn->heartbeat );
                    }
                    break;
                    case AMQP_EVENT_CONNECTION_RECV_START:
                    {
                        amqp_connection_backend_put_start_ok( conn );
                    }
                    break;
                    case AMQP_EVENT_CONNECTION_RECV_TUNE:
                    {
                        wiced_amqp_connection_tune_ok_arg_t  args2;

                        args2.channel_max = AMQP_CONNECTION_CHANNEL_MAX;
                        args2.frame_max   = AMQP_CONNECTION_FRAME_MAX;
                        args2.heartbeat   = AMQP_CONNECTION_HEARTBEAT;

                        if ( args2.heartbeat != 0 )
                        {
                            conn->heartbeat.reset_value = args2.heartbeat;
                            conn->heartbeat.step_value  = 1;
                            amqp_manager_heartbeat_init( &conn->heartbeat, conn );
                            amqp_manager_heartbeat_send_reset( &conn->heartbeat );
                            amqp_manager_heartbeat_recv_reset( &conn->heartbeat );
                        }
                        amqp_connection_backend_put_tune_ok( &args2, conn );
                        amqp_connection_backend_put_open( conn );
                    }
                    break;
                    case AMQP_EVENT_CONNECTION_RECV_OPEN_OK:
                    {
                        /* We can now send/receive a close request*/
                        conn->is_open   = WICED_TRUE;
                    }
                    break;
                    case AMQP_EVENT_CONNECTION_RECV_CLOSE_OK:
                    {
                        conn->is_open   = WICED_FALSE;
                        amqp_manager_heartbeat_deinit( &conn->heartbeat );
                        amqp_network_disconnect( &conn->socket );
                    }
                    break;
                    case AMQP_EVENT_CONNECTION_RECV_CLOSE:
                    {
                        amqp_connection_backend_put_close_ok( conn );
                        conn->is_open   = WICED_FALSE;
                        amqp_manager_heartbeat_deinit( &conn->heartbeat );
                        amqp_network_disconnect( &conn->socket );
                    }
                    break;
                    case AMQP_EVENT_CHANNEL_SEND_OPEN         :
                    case AMQP_EVENT_CHANNEL_SEND_FLOW         :
                    case AMQP_EVENT_CHANNEL_SEND_CLOSE        :
                    case AMQP_EVENT_EXCHANGE_SEND_DECLARE     :
                    case AMQP_EVENT_EXCHANGE_SEND_DELETE      :
                    case AMQP_EVENT_QUEUE_SEND_DECLARE        :
                    case AMQP_EVENT_QUEUE_SEND_BIND           :
                    case AMQP_EVENT_QUEUE_SEND_UNBIND         :
                    case AMQP_EVENT_QUEUE_SEND_PURGE          :
                    case AMQP_EVENT_QUEUE_SEND_DELETE         :
                    case AMQP_EVENT_BASIC_SEND_QOS            :
                    case AMQP_EVENT_BASIC_SEND_CONSUME        :
                    case AMQP_EVENT_BASIC_SEND_CANCEL         :
                    case AMQP_EVENT_BASIC_SEND_PUBLISH        :
                    case AMQP_EVENT_BASIC_SEND_PUBLISH_OK     :
                    case AMQP_EVENT_BASIC_SEND_GET            :
                    case AMQP_EVENT_BASIC_SEND_ACK            :
                    case AMQP_EVENT_BASIC_SEND_REJECT         :
                    case AMQP_EVENT_BASIC_SEND_RECOVER_ASYNC  :
                    case AMQP_EVENT_BASIC_SEND_RECOVER        :
                    case AMQP_EVENT_TX_SEND_SELECT            :
                    case AMQP_EVENT_TX_SEND_COMMIT            :
                    case AMQP_EVENT_TX_SEND_ROLLBACK          :
                    case AMQP_EVENT_CONTENT_SEND_HEADER       :
                    case AMQP_EVENT_CONTENT_SEND_HEADER_OK    :
                    case AMQP_EVENT_CONTENT_SEND_CONTENT      :
                    case AMQP_EVENT_CONTENT_SEND_CONTENT_OK   :
                    case AMQP_EVENT_CONNECTION_SEND_HEARTBEAT :
                    case AMQP_EVENT_CONNECTION_ERROR          :
                    case AMQP_EVENT_MANAGER_TICK              :
                    case AMQP_EVENT_CHANNEL_RECV_OPEN_OK      :
                    case AMQP_EVENT_CHANNEL_RECV_FLOW         :
                    case AMQP_EVENT_CHANNEL_RECV_FLOW_OK      :
                    case AMQP_EVENT_CHANNEL_RECV_CLOSE        :
                    case AMQP_EVENT_CHANNEL_RECV_CLOSE_OK     :
                    case AMQP_EVENT_EXCHANGE_RECV_DECLARE_OK  :
                    case AMQP_EVENT_EXCHANGE_RECV_DELETE_OK   :
                    case AMQP_EVENT_QUEUE_RECV_DECLARE_OK     :
                    case AMQP_EVENT_QUEUE_RECV_BIND_OK        :
                    case AMQP_EVENT_QUEUE_RECV_UNBIND_OK      :
                    case AMQP_EVENT_QUEUE_RECV_PURGE_OK       :
                    case AMQP_EVENT_QUEUE_RECV_DELETE_OK      :
                    case AMQP_EVENT_BASIC_RECV_QOS_OK         :
                    case AMQP_EVENT_BASIC_RECV_CONSUME_OK     :
                    case AMQP_EVENT_BASIC_RECV_CANCEL_OK      :
                    case AMQP_EVENT_BASIC_RECV_RETURN         :
                    case AMQP_EVENT_BASIC_RECV_DELIVER        :
                    case AMQP_EVENT_BASIC_RECV_GET_OK         :
                    case AMQP_EVENT_BASIC_RECV_GET_EMPTY      :
                    case AMQP_EVENT_BASIC_RECV_RECOVER_OK     :
                    case AMQP_EVENT_TX_RECV_SELECT_OK         :
                    case AMQP_EVENT_TX_RECV_COMMIT_OK         :
                    case AMQP_EVENT_TX_RECV_ROLLBACK_OK       :
                    case AMQP_EVENT_CONTENT_RECV_HEADER       :
                    case AMQP_EVENT_CONTENT_RECV_CONTENT      :
                    case AMQP_EVENT_CONNECTION_RECV_HEARTBEAT :
                    case AMQP_EVENT_PROTOCOL_RECV_HEADER      :
                    {
                        /* TODO : add implementations here, or add fall-through to default */
                    }
                    break;

                    default:
                    break;
                }
            }
            else
            {
                if ( WICED_TRUE == conn->is_open )
                {
                    wiced_amqp_connection_close_arg_t   args2;

                    /* Connection start, send an exception */
                    args2.class  = AMQP_FRAME_CLASS_CONNECTION;
                    args2.method = AMQP_FRAME_METHOD_CONNECTION_CLOSE;
                    args2.reply_code = AMQP_FRAME_ERROR_COMMAND_INVALID;
                    AMQP_SHORT_STRING( &args2.reply_text, "command invalid");
                    amqp_connection_backend_put_close( &args2, conn );
                }
                result = WICED_ERROR;
            }
        }
        break;
        case AMQP_EVENT_MANAGER_TICK:
        {
            if ( amqp_manager_heartbeat_recv_step( &conn->heartbeat ) != WICED_SUCCESS )
            {
                /* Reset counter timed out and we didn't receive any thing from broker */
                conn->is_open = WICED_FALSE;
                amqp_network_disconnect( &conn->socket );
                result = WICED_ERROR;
            }
            else
            {
                if ( amqp_manager_heartbeat_send_step( &conn->heartbeat ) != WICED_SUCCESS )
                {
                    amqp_connection_backend_put_heartbeat( conn );
                    amqp_manager_heartbeat_send_reset( &conn->heartbeat );
                }
            }
        }
        break;
        case AMQP_EVENT_CONNECTION_ERROR:
        {
            if ( conn->is_open == WICED_TRUE )
            {
                conn->is_open = WICED_FALSE;
                result = WICED_ERROR;
            }
        }
        break;

        case AMQP_EVENT_CHANNEL_SEND_OPEN         :
        case AMQP_EVENT_CHANNEL_SEND_FLOW         :
        case AMQP_EVENT_CHANNEL_SEND_CLOSE        :
        case AMQP_EVENT_EXCHANGE_SEND_DECLARE     :
        case AMQP_EVENT_EXCHANGE_SEND_DELETE      :
        case AMQP_EVENT_QUEUE_SEND_DECLARE        :
        case AMQP_EVENT_QUEUE_SEND_BIND           :
        case AMQP_EVENT_QUEUE_SEND_UNBIND         :
        case AMQP_EVENT_QUEUE_SEND_PURGE          :
        case AMQP_EVENT_QUEUE_SEND_DELETE         :
        case AMQP_EVENT_BASIC_SEND_QOS            :
        case AMQP_EVENT_BASIC_SEND_CONSUME        :
        case AMQP_EVENT_BASIC_SEND_CANCEL         :
        case AMQP_EVENT_BASIC_SEND_PUBLISH        :
        case AMQP_EVENT_BASIC_SEND_PUBLISH_OK     :
        case AMQP_EVENT_BASIC_SEND_GET            :
        case AMQP_EVENT_BASIC_SEND_ACK            :
        case AMQP_EVENT_BASIC_SEND_REJECT         :
        case AMQP_EVENT_BASIC_SEND_RECOVER_ASYNC  :
        case AMQP_EVENT_BASIC_SEND_RECOVER        :
        case AMQP_EVENT_TX_SEND_SELECT            :
        case AMQP_EVENT_TX_SEND_COMMIT            :
        case AMQP_EVENT_TX_SEND_ROLLBACK          :
        case AMQP_EVENT_CONTENT_SEND_HEADER       :
        case AMQP_EVENT_CONTENT_SEND_HEADER_OK    :
        case AMQP_EVENT_CONTENT_SEND_CONTENT      :
        case AMQP_EVENT_CONTENT_SEND_CONTENT_OK   :
        case AMQP_EVENT_CONNECTION_SEND_HEARTBEAT :
        case AMQP_EVENT_CHANNEL_RECV_OPEN_OK      :
        case AMQP_EVENT_CHANNEL_RECV_FLOW         :
        case AMQP_EVENT_CHANNEL_RECV_FLOW_OK      :
        case AMQP_EVENT_CHANNEL_RECV_CLOSE        :
        case AMQP_EVENT_CHANNEL_RECV_CLOSE_OK     :
        case AMQP_EVENT_EXCHANGE_RECV_DECLARE_OK  :
        case AMQP_EVENT_EXCHANGE_RECV_DELETE_OK   :
        case AMQP_EVENT_QUEUE_RECV_DECLARE_OK     :
        case AMQP_EVENT_QUEUE_RECV_BIND_OK        :
        case AMQP_EVENT_QUEUE_RECV_UNBIND_OK      :
        case AMQP_EVENT_QUEUE_RECV_PURGE_OK       :
        case AMQP_EVENT_QUEUE_RECV_DELETE_OK      :
        case AMQP_EVENT_BASIC_RECV_QOS_OK         :
        case AMQP_EVENT_BASIC_RECV_CONSUME_OK     :
        case AMQP_EVENT_BASIC_RECV_CANCEL_OK      :
        case AMQP_EVENT_BASIC_RECV_RETURN         :
        case AMQP_EVENT_BASIC_RECV_DELIVER        :
        case AMQP_EVENT_BASIC_RECV_GET_OK         :
        case AMQP_EVENT_BASIC_RECV_GET_EMPTY      :
        case AMQP_EVENT_BASIC_RECV_RECOVER_OK     :
        case AMQP_EVENT_TX_RECV_SELECT_OK         :
        case AMQP_EVENT_TX_RECV_COMMIT_OK         :
        case AMQP_EVENT_TX_RECV_ROLLBACK_OK       :
        case AMQP_EVENT_CONTENT_RECV_HEADER       :
        case AMQP_EVENT_CONTENT_RECV_CONTENT      :
        default:
        if ( ( WICED_TRUE != conn->is_open ) ||
             ( amqp_channel_manager( event, args, channel, conn ) != WICED_SUCCESS ) )
        {
            wiced_amqp_connection_close_arg_t   args2;

            /* Connection start, send an exception */
            args2.class  = AMQP_FRAME_CLASS_CONNECTION;
            args2.method = AMQP_FRAME_METHOD_CONNECTION_CLOSE;
            args2.reply_code = AMQP_FRAME_ERROR_COMMAND_INVALID;
            AMQP_SHORT_STRING( &args2.reply_text, "command invalid" );
            amqp_connection_backend_put_close( &args2, conn );

            result = WICED_ERROR;
        }
    }
    wiced_rtos_set_semaphore( &conn->semaphore );
    return result;
}
