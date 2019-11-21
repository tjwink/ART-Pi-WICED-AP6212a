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
 *  Connection functions
 *
 *  Provides connection methods for use by applications
 */

#include "wiced.h"
#include "amqp.h"
#include "amqp_connection.h"
#include "amqp_frame.h"
#include "amqp_manager.h"

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
wiced_result_t amqp_connection_init( const wiced_ip_address_t *address, wiced_interface_t interface, const wiced_amqp_callbacks_t *callbacks, wiced_amqp_connection_t *conn, const wiced_amqp_security_t *security )
{
    uint16_t port_number = security == NULL ? AMQP_CONNECTION_DEFAULT_PORT : AMQP_CONNECTION_SECURITY_PORT;

    wiced_rtos_init_semaphore( &conn->semaphore );
    wiced_rtos_set_semaphore( &conn->semaphore );
    conn->callbacks = *callbacks;

    return  amqp_network_init( address, port_number, interface, conn, &conn->socket, security );
}

wiced_result_t amqp_connection_open( wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    wiced_amqp_protocol_header_arg_t     args;

    args.major       = AMQP_PROTOCOL_VERSION_MAJOR;
    args.minor       = AMQP_PROTOCOL_VERSION_MINOR;
    args.revision    = AMQP_PROTOCOL_VERSION_REVISION;

    ret = amqp_network_connect( &conn->socket );
    if (  ret != WICED_SUCCESS )
    {
        return ret;
    }

    /* Make sure the static is valid to send */
    return  amqp_manager( AMQP_EVENT_PROTOCOL_SEND_HEADER, &args, 0, conn );
}

wiced_result_t amqp_connection_close( wiced_amqp_connection_t *conn )
{
    wiced_amqp_connection_close_arg_t args;

    args.reply_code = AMQP_PROTOCOL_REPLY_SUCCESS;
    args.class      = 0;
    args.method     = 0;
    AMQP_SHORT_STRING( &args.reply_text, STR( AMQP_PROTOCOL_REPLY_SUCCESS ) );

    return amqp_manager( AMQP_EVENT_CONNECTION_SEND_CLOSE, &args, 0, conn );
}

wiced_result_t amqp_connection_deinit( wiced_amqp_connection_t *conn )
{
    wiced_rtos_deinit_semaphore( &conn->semaphore );
    return amqp_network_deinit( &conn->socket );
}

/******************************************************
 *      Backend functions called from amqp_queue
 ******************************************************/
wiced_result_t amqp_connection_backend_put_protocol_header( const wiced_amqp_protocol_header_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_PROTOCOL_HEADER, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_protocol_header( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_connection_backend_get_start( uint16_t channel, wiced_amqp_connection_start_arg_t *args, wiced_amqp_connection_t *conn )
{

    /* Make sure the static is valid to send */
    return amqp_manager( AMQP_EVENT_CONNECTION_RECV_START, args, channel, conn );
}

wiced_result_t amqp_connection_backend_put_start_ok( wiced_amqp_connection_t *conn )
{
    uint8_t                              response[12] = {'\0','g','u','e','s','t','\0','g','u','e','s','t'};
    wiced_amqp_field_t                   client_properties[6];
    wiced_amqp_connection_start_ok_arg_t args;
    wiced_amqp_frame_t                   frame;
    wiced_result_t                       ret;

    client_properties[0].type = 'S';
    AMQP_SHORT_STRING( &client_properties[0].name, "product" );
    AMQP_LONG_STRING(  &client_properties[0].value.S, "WICED" );

    client_properties[1].type = 'S';
    AMQP_SHORT_STRING( &client_properties[1].name, "version" );
    AMQP_LONG_STRING(  &client_properties[1].value.S, "0.1" );

    client_properties[2].type = 'S';
    AMQP_SHORT_STRING( &client_properties[2].name, "plaform" );
    AMQP_LONG_STRING(  &client_properties[2].value.S, "WICED" );
    client_properties[3].type = 'S';
    AMQP_SHORT_STRING( &client_properties[3].name, "copyright" );
    AMQP_LONG_STRING(  &client_properties[3].value.S, "Copyright (c) 2007-2015 Broadcom Inc." ); /* license_checker: this is not a license */

    client_properties[4].name.str = NULL;

    AMQP_SHORT_STRING( &args.mechnism, "PLAIN" );
    AMQP_SHORT_STRING( &args.locale, "en_US" );

    args.response.str = response;
    args.response.len = sizeof( response );

    args.client_properties = client_properties;

    /* Send Start-OK */
    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_connection_start_ok( &frame, &args );
    amqp_frame_send( &frame, &conn->socket );
    return WICED_SUCCESS;
}

wiced_result_t amqp_connection_backend_get_tune( uint16_t channel, wiced_amqp_connection_tune_arg_t *args, wiced_amqp_connection_t *conn )
{
    /* Make sure the static is valid to send */
    return amqp_manager( AMQP_EVENT_CONNECTION_RECV_TUNE, args, channel, conn );
}

wiced_result_t amqp_connection_backend_put_tune_ok( const wiced_amqp_connection_tune_ok_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    /* Send Start-OK */
    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_connection_tune_ok( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_connection_backend_put_open( wiced_amqp_connection_t *conn )
{
    wiced_result_t                   ret;
    wiced_amqp_frame_t               frame;
    wiced_amqp_connection_open_arg_t args;

    AMQP_SHORT_STRING( &args.path, "/" );


    /* Send Start-OK */
    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_connection_open( &frame, &args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_connection_backend_get_open_ok( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;

    /* Make sure the state is valid to send */
    ret = amqp_manager( AMQP_EVENT_CONNECTION_RECV_OPEN_OK, NULL, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        if ( conn->callbacks.connection_event != NULL )
        {
            conn->callbacks.connection_event( AMQP_EVENT_CONNECTION_RECV_OPEN_OK, NULL, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_connection_backend_put_close( const wiced_amqp_connection_close_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    /* Send Close request */
    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_connection_close( &frame, args );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_connection_backend_get_close(  uint16_t channel, wiced_amqp_connection_close_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;

    ret = amqp_manager( AMQP_EVENT_CONNECTION_RECV_CLOSE, args, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        if ( conn->callbacks.connection_event != NULL )
        {
            conn->callbacks.connection_event( AMQP_EVENT_CONNECTION_RECV_CLOSE, args, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_connection_backend_put_close_ok( wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    /* Send Close request */
    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_METHOD, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_connection_close_ok( &frame );
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_connection_backend_get_close_ok( uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;

    ret = amqp_manager( AMQP_EVENT_CONNECTION_RECV_CLOSE_OK, NULL, channel, conn );
    if ( ret == WICED_SUCCESS )
    {
        if ( conn->callbacks.connection_event != NULL )
        {
            conn->callbacks.connection_event( AMQP_EVENT_CONNECTION_RECV_CLOSE_OK, NULL, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_connection_backend_close( wiced_amqp_connection_t *conn )
{
    wiced_rtos_deinit_semaphore( &conn->semaphore );
    return amqp_network_deinit( &conn->socket );
}

wiced_result_t amqp_connection_backend_get_heartbeat( uint16_t channel, wiced_amqp_connection_t *conn )
{
    /* May need to be modified to pass error to user */
    return amqp_manager( AMQP_EVENT_CONNECTION_RECV_HEARTBEAT, NULL, channel, conn );
}

wiced_result_t amqp_connection_backend_put_heartbeat( wiced_amqp_connection_t *conn )
{
    wiced_result_t      ret;
    wiced_amqp_frame_t  frame;

    /* Send Close request */
    ret = amqp_frame_create( WICED_AMQP_FRAME_TYPE_HEARTBEAT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    return amqp_frame_send( &frame, &conn->socket );
}

wiced_result_t amqp_connection_backend_get_protocol_header( wiced_amqp_protocol_header_arg_t *args, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;

    ret = amqp_manager( AMQP_EVENT_PROTOCOL_RECV_HEADER, args, 0, conn );
    if ( ret == WICED_SUCCESS )
    {
        if ( conn->callbacks.connection_event != NULL )
        {
            conn->callbacks.connection_event( AMQP_EVENT_PROTOCOL_RECV_HEADER, args, conn );
        }
    }
    return ret;
}

wiced_result_t amqp_connection_backend_closed( wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;

    ret = amqp_manager( AMQP_EVENT_CONNECTION_ERROR, NULL, 0, conn );
    if ( ret != WICED_SUCCESS )
    {
        if ( conn->callbacks.connection_event != NULL )
        {
            conn->callbacks.connection_event( AMQP_EVENT_CONNECTION_ERROR, NULL, conn );
            return ret;
        }
    }
    return WICED_SUCCESS;
}
