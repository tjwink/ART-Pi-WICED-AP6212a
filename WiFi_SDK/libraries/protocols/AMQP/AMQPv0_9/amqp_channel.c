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
 *  Channel functions
 *
 *  Provides channel methods for use in applications
 */

#include "wiced.h"
#include "amqp.h"
#include "amqp_connection.h"
#include "amqp_manager.h"
#include "amqp_frame.h"

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
wiced_result_t amqp_channel_open(  uint16_t channel, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_CHANNEL_SEND_OPEN, NULL, channel, conn );
}

wiced_result_t amqp_channel_close(  uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_amqp_channel_close_arg_t args;

    args.reply_code = AMQP_PROTOCOL_REPLY_SUCCESS;
    args.class      = 0;
    args.method     = 0;
    AMQP_SHORT_STRING( &args.reply_text, STR( AMQP_PROTOCOL_REPLY_SUCCESS ) );

    return amqp_manager( AMQP_EVENT_CHANNEL_SEND_CLOSE, &args, channel, conn );
}

wiced_result_t amqp_channel_flow( uint16_t channel, wiced_amqp_channel_flow_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_CHANNEL_SEND_FLOW, args, channel, conn );
}

wiced_result_t amqp_channel_exchange_declare( uint16_t channel, wiced_amqp_exchange_declare_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_EXCHANGE_SEND_DECLARE, args, channel, conn );
}

wiced_result_t amqp_channel_exchange_delete( uint16_t channel, wiced_amqp_exchange_delete_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_EXCHANGE_SEND_DELETE, args, channel, conn );
}

wiced_result_t amqp_channel_queue_declare( uint16_t channel, wiced_amqp_queue_declare_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_QUEUE_SEND_DECLARE, args, channel, conn );
}

wiced_result_t amqp_channel_queue_bind( uint16_t channel, wiced_amqp_queue_bind_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_QUEUE_SEND_BIND, args, channel, conn );
}

wiced_result_t amqp_channel_queue_unbind( uint16_t channel, wiced_amqp_queue_unbind_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_QUEUE_SEND_UNBIND, args, channel, conn );
}

wiced_result_t amqp_channel_queue_purge( uint16_t channel, wiced_amqp_queue_purge_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_QUEUE_SEND_PURGE, args, channel, conn );
}

wiced_result_t amqp_channel_queue_delete( uint16_t channel, wiced_amqp_queue_delete_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_QUEUE_SEND_DELETE, args, channel, conn );
}

wiced_result_t amqp_channel_basic_qos( uint16_t channel, wiced_amqp_basic_qos_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_BASIC_SEND_QOS, args, channel, conn );
}

wiced_result_t amqp_channel_basic_consume( uint16_t channel, wiced_amqp_basic_consume_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_BASIC_SEND_CONSUME, args, channel, conn );
}

wiced_result_t amqp_channel_basic_cancel( uint16_t channel, wiced_amqp_basic_cancel_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_BASIC_SEND_CANCEL, args, channel, conn );
}

wiced_result_t amqp_channel_basic_get( uint16_t channel, wiced_amqp_basic_get_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_BASIC_SEND_GET, args, channel, conn );
}

wiced_result_t amqp_channel_basic_reject( uint16_t channel, wiced_amqp_basic_reject_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_BASIC_SEND_REJECT, args, channel, conn );
}

wiced_result_t amqp_channel_basic_recover( uint16_t channel, wiced_amqp_basic_recover_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_BASIC_SEND_RECOVER, args, channel, conn );
}

wiced_result_t amqp_channel_basic_recover_async( uint16_t channel, wiced_amqp_basic_recover_async_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_BASIC_SEND_RECOVER_ASYNC, args, channel, conn );
}

wiced_result_t amqp_channel_basic_publish(  uint16_t channel, wiced_amqp_basic_publish_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_BASIC_SEND_PUBLISH, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_BASIC_SEND_PUBLISH_OK, NULL, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_basic_ack(  uint16_t channel, wiced_amqp_basic_ack_arg_t *args, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_BASIC_SEND_ACK, args, channel, conn );
}

wiced_result_t amqp_channel_tx_select( uint16_t channel, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_TX_SEND_SELECT, NULL, channel, conn );
}

wiced_result_t amqp_channel_tx_commit( uint16_t channel, wiced_amqp_connection_t *conn )
{
    return amqp_manager( AMQP_EVENT_TX_SEND_COMMIT, NULL, channel, conn );
}

wiced_result_t amqp_channel_tx_rollback( uint16_t channel, wiced_amqp_connection_t *conn )
{
    /* Make sure the static is valid to send */
    return amqp_manager( AMQP_EVENT_TX_SEND_ROLLBACK, NULL, channel, conn );
}

wiced_result_t amqp_channel_content_put_header(  uint16_t channel, wiced_amqp_content_header_arg_t *args,wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_CONTENT_SEND_HEADER, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_CONTENT_SEND_HEADER_OK, NULL, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_content_put_content(  uint16_t channel, wiced_amqp_content_content_arg_t *args,wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;

    args->buffer.data += args->size;
    ret = amqp_manager( AMQP_EVENT_CONTENT_SEND_CONTENT, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_CONTENT_SEND_CONTENT_OK, NULL, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_content_create_frame(  uint16_t channel, wiced_amqp_content_content_arg_t *args,wiced_amqp_connection_t *conn )
{
    return amqp_frame_create( WICED_AMQP_FRAME_TYPE_CONTENT, channel, AMQP_CONNECTION_FRAME_MAX, args, &conn->socket );
}

wiced_result_t amqp_channel_content_delete_frame(  wiced_amqp_content_content_arg_t *args )
{
    return amqp_frame_delete( args );
}


/******************************************************
 *      Backend functions called from amqp_queue
 ******************************************************/
wiced_result_t amqp_channel_backend_put_open(  uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_channel_open( &frame );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_backend_get_open_ok(  uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_CHANNEL_RECV_OPEN_OK, NULL, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_CHANNEL_RECV_OPEN_OK, NULL, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_backend_put_close(  uint16_t channel, const wiced_amqp_channel_close_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_channel_close( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_backend_get_close(  uint16_t channel, wiced_amqp_channel_close_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_CHANNEL_RECV_CLOSE, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_CHANNEL_RECV_CLOSE, args, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_backend_put_close_ok(  uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_channel_close_ok( &frame );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_backend_get_close_ok(  uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_CHANNEL_RECV_CLOSE_OK, NULL, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_CHANNEL_RECV_CLOSE_OK, NULL, channel, conn );
        }
    }
    return ret;
}



wiced_result_t amqp_channel_backend_put_flow( uint16_t channel, const wiced_amqp_channel_flow_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_channel_flow( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_backend_get_flow( uint16_t channel, wiced_amqp_channel_flow_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_CHANNEL_RECV_FLOW, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_CHANNEL_RECV_FLOW, args, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_backend_put_flow_ok( uint16_t channel, const wiced_amqp_channel_flow_ok_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_channel_flow_ok( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_backend_get_flow_ok( uint16_t channel, wiced_amqp_channel_flow_ok_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_CHANNEL_RECV_FLOW_OK, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_CHANNEL_RECV_FLOW_OK, args, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_exchange_backend_put_declare( uint16_t channel, const wiced_amqp_exchange_declare_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_exchange_declare( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_exchange_backend_get_declare_ok( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_EXCHANGE_RECV_DECLARE_OK, NULL, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_EXCHANGE_RECV_DECLARE_OK, NULL, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_exchange_backend_put_delete( uint16_t channel, const wiced_amqp_exchange_delete_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_exchange_delete( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_exchange_backend_get_delete_ok( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_EXCHANGE_RECV_DELETE_OK, NULL, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_EXCHANGE_RECV_DELETE_OK, NULL, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_queue_backend_put_declare( uint16_t channel, const wiced_amqp_queue_declare_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    /* Send Close request */
    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_queue_declare( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_queue_backend_get_declare_ok( uint16_t channel, wiced_amqp_queue_declare_ok_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_QUEUE_RECV_DECLARE_OK, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_QUEUE_RECV_DECLARE_OK, args, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_queue_backend_put_bind( uint16_t channel, const wiced_amqp_queue_bind_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_queue_bind( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_queue_backend_get_bind_ok( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_QUEUE_RECV_BIND_OK, NULL, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_QUEUE_RECV_BIND_OK, NULL, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_queue_backend_put_unbind( uint16_t channel, const wiced_amqp_queue_unbind_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_queue_unbind( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_queue_backend_get_unbind_ok( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_QUEUE_RECV_UNBIND_OK, NULL, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_QUEUE_RECV_UNBIND_OK, NULL, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_queue_backend_put_purge( uint16_t channel, const wiced_amqp_queue_purge_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_queue_purge( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_queue_backend_get_purge_ok( uint16_t channel, wiced_amqp_queue_purge_ok_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_QUEUE_RECV_PURGE_OK, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_QUEUE_RECV_PURGE_OK, args, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_queue_backend_put_delete( uint16_t channel, const wiced_amqp_queue_delete_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_queue_delete( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_queue_backend_get_delete_ok( uint16_t channel, wiced_amqp_queue_delete_ok_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_QUEUE_RECV_DELETE_OK, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_QUEUE_RECV_DELETE_OK, args, channel, conn );
        }
    }
    return ret;
}


wiced_result_t amqp_channel_basic_backend_put_publish(  uint16_t channel, const wiced_amqp_basic_publish_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_basic_publish( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_basic_backend_get_return( uint16_t channel, wiced_amqp_basic_return_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_BASIC_RECV_RETURN, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_BASIC_RECV_RETURN, args, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_basic_backend_get_deliver( uint16_t channel, wiced_amqp_basic_deliver_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_BASIC_RECV_DELIVER, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_BASIC_RECV_DELIVER, args, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_basic_backend_put_qos( uint16_t channel, const wiced_amqp_basic_qos_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_basic_qos( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_basic_backend_get_qos_ok( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_BASIC_RECV_QOS_OK, NULL, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_BASIC_RECV_QOS_OK, NULL, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_basic_backend_put_consume( uint16_t channel, const wiced_amqp_basic_consume_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_basic_consume( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_basic_backend_get_consume_ok( uint16_t channel, wiced_amqp_basic_consume_ok_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_BASIC_RECV_CONSUME_OK, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_BASIC_RECV_CONSUME_OK, args, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_basic_backend_put_cancel( uint16_t channel, const wiced_amqp_basic_cancel_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_basic_cancel( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_basic_backend_get_cancel_ok( uint16_t channel, wiced_amqp_basic_cancel_ok_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_BASIC_RECV_CANCEL_OK, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_BASIC_RECV_CANCEL_OK, args, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_basic_backend_put_get( uint16_t channel, const wiced_amqp_basic_get_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_basic_get( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_basic_backend_get_get_ok( uint16_t channel, wiced_amqp_basic_get_ok_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_BASIC_RECV_GET_OK, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_BASIC_RECV_GET_OK, args, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_basic_backend_get_get_empty( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_BASIC_RECV_GET_EMPTY, NULL, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_BASIC_RECV_GET_EMPTY, NULL, channel, conn );
        }
    }
    return ret;
}


wiced_result_t amqp_channel_basic_backend_put_ack( uint16_t channel, const wiced_amqp_basic_ack_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_basic_ack( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_basic_backend_put_reject( uint16_t channel, const wiced_amqp_basic_reject_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_basic_reject( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_basic_backend_put_recover( uint16_t channel, const wiced_amqp_basic_recover_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    /* Send Close request */
    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_basic_recover( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_basic_backend_put_recover_async( uint16_t channel, const wiced_amqp_basic_recover_async_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_basic_recover_async( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_basic_backend_get_recover_ok( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_BASIC_RECV_RECOVER_OK, NULL, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_BASIC_RECV_RECOVER_OK, NULL, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_tx_backend_put_select( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    /* Send Close request */
    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_tx_select( &frame );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_tx_backend_get_select_ok( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_TX_RECV_SELECT_OK, NULL, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_TX_RECV_SELECT_OK, NULL, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_tx_backend_put_commit( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_tx_commit( &frame );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_tx_backend_get_commit_ok( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_TX_RECV_COMMIT_OK, NULL, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_TX_RECV_COMMIT_OK, NULL, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_tx_backend_put_rollback( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_tx_rollback( &frame );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_tx_backend_get_rollback_ok( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_TX_RECV_ROLLBACK_OK, NULL, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_TX_RECV_ROLLBACK_OK, NULL, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_content_backend_put_header(  uint16_t channel, const wiced_amqp_content_header_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t                  ret;
    wiced_amqp_frame_t              frame;
    wiced_amqp_content_header_arg_t args2;
    uint16_t size = (uint16_t) args->size;

    memset( &args2, 0, sizeof( wiced_amqp_content_header_arg_t ));
    args2.size               = size;
    args2.content_type_flag  = 1;
    args2.delivery_mode_flag = 1;
    args2.delivery_mode      = 2;
    AMQP_SHORT_STRING( &args2.content_type, "text/plain");

    /* Send Channel Open */
    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_HEADER, channel, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_content_header( &frame, &args2 );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_channel_content_backend_get_header( uint16_t channel, wiced_amqp_content_header_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_CONTENT_RECV_HEADER, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_CONTENT_RECV_HEADER, args, channel, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_channel_content_backend_put_content( wiced_amqp_frame_t *frame, wiced_amqp_connection_t *conn )
{
    return amqp_frame_send( frame, &conn->socket );
}

wiced_result_t amqp_channel_content_backend_get_content( uint16_t channel, wiced_amqp_frame_t *frame, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    ret = amqp_manager( AMQP_EVENT_CONTENT_RECV_CONTENT, frame, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        /* Publish is an async method (we don't get an OK), so we simulate the OK after sending it */
        if ( conn->callbacks.channel_event != NULL )
        {
            conn->callbacks.channel_event( AMQP_EVENT_CONTENT_RECV_CONTENT, frame, channel, conn );
        }
    }
    return ret;
}
