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
#include "amqp_internal.h"
#include "amqp_frame.h"
#include "amqp_session.h"
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

/******************************************************
 *               Protocol Header Frames
 ******************************************************/

wiced_result_t wiced_amqp_begin( wiced_amqp_session_instance_t* session_instance, wiced_amqp_connection_instance *connection_instance )
{
    amqp_channel_endpoint_t* channel_endpoint;
    amqp_channel_endpoint_t* current_node;
    wiced_result_t result;
    int i = 0;

    /* return error if connection instance has not been created */
    if ( connection_instance == NULL )
    {
        WPRINT_LIB_DEBUG (("Connection instance is not created"));
        return WICED_ERROR;
    }

    session_instance->connection_instance = connection_instance;
    session_instance->begin_args.remote_incoming_window = 0;
    session_instance->begin_args.remote_outgoing_window = 0;
    session_instance->begin_args.incoming_window = DEFAULT_INCOMING_WINDOW;
    session_instance->begin_args.outgoing_window = DEFAULT_OUTGOING_WINDOW;
    session_instance->begin_args.handle_max = 4294967295u;
    session_instance->begin_args.desired_incoming_window = 1;
    session_instance->begin_args.next_outgoing_id = 0;
    session_instance->begin_args.next_incoming_id = 0;

    /* Find the lowest channel number available in endpoint list */
    linked_list_get_front_node( &connection_instance->channel_endpoint, (linked_list_node_t**) &current_node );
    while ( current_node != NULL )
    {
        amqp_channel_endpoint_t* endpoint = (amqp_channel_endpoint_t*) current_node;

        if ( endpoint->outgoing_channel > i )
        {
            break;
        }

        i++ ;
        current_node = (amqp_channel_endpoint_t*) endpoint->this_node.next;
    }

    /* create new endpoint */
    result = amqp_get_buffer( (void**) &channel_endpoint, sizeof(amqp_channel_endpoint_t) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_DEBUG(("memory not available to create channel endpoint"));
        return WICED_ERROR;
    }

    channel_endpoint->outgoing_channel = (uint16_t) i;
    connection_instance->endpoint_count++ ;
    result = linked_list_insert_node_at_rear( &connection_instance->channel_endpoint, &channel_endpoint->this_node );

    channel_endpoint->context = session_instance;
    session_instance->endpoints = channel_endpoint;

    linked_list_init( &session_instance->link_endpoint);

    amqp_manager( WICED_AMQP_EVENT_BEGIN_SENT, session_instance, 0, connection_instance );

    return WICED_SUCCESS;

}


wiced_result_t wiced_amqp_end( wiced_amqp_session_instance_t* session_instance )
{
    return amqp_manager( WICED_AMQP_EVENT_END_SENT, session_instance, 0, session_instance->connection_instance );
}

/******************************************************
 *      Backend functions called from amqp_queue
 ******************************************************/

wiced_result_t amqp_connection_backend_get_begin_performative( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret;

    ret = amqp_manager( WICED_AMQP_EVENT_BEGIN_RCVD, args, 0, conn_instance );
    if ( ret == WICED_SUCCESS )
    {
        if ( conn_instance->conn->callbacks.connection_event != NULL )
        {
            conn_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_BEGIN_RCVD, args, conn_instance );
        }
    }
    return ret;
}

wiced_result_t amqp_connection_backend_put_begin_performative( wiced_amqp_session_instance_t* session_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_EVENT_BEGIN_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &session_instance->connection_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_begin( &frame, session_instance );
    return amqp_frame_send( &frame, &session_instance->connection_instance->conn->socket );
}


wiced_result_t amqp_frame_get_begin_performative( wiced_amqp_frame_t *frame, void *arg )
{
    wiced_amqp_packet_content *args = (wiced_amqp_packet_content *) arg;
     uint16_t descriptor = 0 ;
     uint32_t size = 0 ;
     uint32_t temp = 0 ;
     uint32_t count = 0 ;

    AMQP_BUFFER_GET_LONG( &frame->buffer, args->fixed_frame.size );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->fixed_frame.doff );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->fixed_frame.type );
    AMQP_BUFFER_GET_SHORT( &frame->buffer, args->fixed_frame.channel );

    AMQP_BUFFER_GET_SHORT( &frame->buffer, descriptor );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->fixed_frame.performative_type );

    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

    if ( temp == 0xc0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, size );

        AMQP_BUFFER_GET_OCTET( &frame->buffer, count );
    }

    size =  size - 1;

    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x43 )
        {
            args->args.begin_args.remote_channel = 0;
            count-- ;
            size-- ;
        }
        else
        {
            ( ( &frame->buffer )->data )-- ;
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size-- ;
            AMQP_BUFFER_GET_SHORT( &frame->buffer, args->args.begin_args.remote_channel );
            count-- ;
            size = size - 2;
        }

    }

    if ( count != 0 && size != 0 )
    {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x43 )
        {
            args->args.begin_args.next_outgoing_id = 0;
            count-- ;
            size-- ;
        }
        else
        {
//            ( ( &frame->buffer )->data )-- ;
//            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
//            size-- ;
//            AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->args.open_args.host_name, args->args.open_args.host_name_size );
//            count-- ;
//            size-- ;
//            size = size - args->args.open_args.host_name_size;
        }

    }

    if ( count != 0 && size != 0 )
    {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x43 )
        {
            args->args.begin_args.remote_incoming_window = 0;
            count-- ;
            size-- ;
        }
        else
        {
            ( ( &frame->buffer )->data )-- ;
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size-- ;
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.begin_args.remote_incoming_window );
            count-- ;
            size = size - 4;
        }

    }

    if ( count != 0 && size != 0 )
    {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x43 )
        {
            args->args.begin_args.remote_outgoing_window = 0;
            count-- ;
            size-- ;
        }
        else
        {
            ( ( &frame->buffer )->data )-- ;
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size-- ;
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.begin_args.remote_outgoing_window );
            count-- ;
            size = size - 4;
        }

    }

    if ( count != 0 && size != 0 )
    {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x43 )
        {
            args->args.begin_args.handle_max = 0;
            count-- ;
            size-- ;
        }
        else
        {
            ( ( &frame->buffer )->data )-- ;
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size-- ;
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.begin_args.handle_max );
            count-- ;
            size = size - 4;
        }

    }

    if ( count != 0 && size != 0 )
     {
         AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
         if ( temp == 0x40 )
         {
             count-- ;
             size-- ;
         }
     }

    if ( count != 0 && size != 0 )
     {
         AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
         if ( temp == 0x40 )
         {
             count-- ;
             size-- ;
         }
     }

    if ( count != 0 && size != 0 )
     {
         AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
         if ( temp == 0x40 )
         {
             count-- ;
             size-- ;
         }
     }

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_begin( wiced_amqp_frame_t *frame,  void* arg )
{
    uint32_t total_size = 0;
    uint32_t count = 0;
    uint32_t size = 0;
    wiced_amqp_session_instance_t *params = arg;

    /* Add remote channel null */
    size++;
    count++;

    /* As it is container id string-8 type  code : 0xal*/
    if ( params->begin_args.next_outgoing_id == 0 )
    {
        count++;
        size++;
    }
    else
    {
        count++;
        size += 5;
    }

    /* Encode incoming window */
    count++;
    size += 5;

    /* Encode outgoing window */
    count++;
    size += 5;

    /* Encode outgoing window */
    count++;
    size += 5;

    size = size + 1;
    total_size = size + 8 + 3 + 2;
    /* fill fixed AMQP header */
    AMQP_BUFFER_PUT_LONG( &frame->buffer, total_size );                                 /*  4  total size   */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x02 );                                      /*  1  doff         */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x00 );                                      /*  1  type         */
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, params->endpoints->outgoing_channel );       /*  2  channel      */
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );                                    /*  2  descriptor   */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x11 );                                      /*  1  performative */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xc0 );                                      /*  1  constructor  */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, size );                                      /*  1  size         */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, count);                                      /*  1  count        */

    /* Add remote channel null */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );

    /* As it is container id string-8 type  code : 0xal*/
    if ( params->begin_args.next_outgoing_id == 0 )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x43 );
    }
    else
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
        AMQP_BUFFER_PUT_LONG( &frame->buffer, params->begin_args.next_outgoing_id );
    }

    /* Encode incoming window */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
    AMQP_BUFFER_PUT_LONG( &frame->buffer, params->begin_args.incoming_window );

    /* Encode outgoing window */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
    AMQP_BUFFER_PUT_LONG( &frame->buffer, params->begin_args.outgoing_window );

    /* Encode outgoing window */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
    AMQP_BUFFER_PUT_LONG( &frame->buffer, params->begin_args.handle_max );

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_end_performative( wiced_amqp_frame_t *frame, void *arg )
{
    wiced_amqp_packet_content *args = (wiced_amqp_packet_content *) arg;
    uint16_t descriptor = 0;
    uint32_t temp = 0;

    AMQP_BUFFER_GET_LONG( &frame->buffer, args->fixed_frame.size );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->fixed_frame.doff );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->fixed_frame.type );
    AMQP_BUFFER_GET_SHORT( &frame->buffer, args->fixed_frame.channel );

    AMQP_BUFFER_GET_SHORT( &frame->buffer, descriptor );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->fixed_frame.performative_type );

    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

    (void) temp;

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_end_performative( wiced_amqp_frame_t *frame,  void* arg )
{
    wiced_amqp_session_instance_t* session_instance = arg;
    amqp_fixed_frame fixed_frame;
    fixed_frame.channel = session_instance->endpoints->outgoing_channel;
    fixed_frame.doff = 0x02;
    fixed_frame.type = WICED_AMQP_FRAME_TYPE;
    fixed_frame.performative_type = AMQP_END;
    fixed_frame.size = 8 + 3 + 1 + 1 + 1 + 1;

    AMQP_BUFFER_PUT_LONG( &frame->buffer, fixed_frame.size ); /*  4 */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, fixed_frame.doff );/*  1 */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, fixed_frame.type );/*  1 */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, fixed_frame.channel );/*  2 */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );/*  2 */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, fixed_frame.performative_type );/*  1 */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xC0 );

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x02 );

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x01 );

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );

    return WICED_SUCCESS;
}


wiced_result_t amqp_connection_backend_get_end_performative( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret;

    ret = amqp_manager( WICED_AMQP_EVENT_END_RCVD, args, 0, conn_instance );
    if ( ret == WICED_SUCCESS )
    {
        if ( conn_instance->conn->callbacks.connection_event != NULL )
        {
            conn_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_END_RCVD, args, conn_instance );
        }
    }
    return ret;
}

wiced_result_t amqp_connection_backend_put_end_performative(  wiced_amqp_session_instance_t* session_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_EVENT_END_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &session_instance->connection_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_end_performative( &frame, session_instance );
    return amqp_frame_send( &frame, &session_instance->connection_instance->conn->socket );
}
