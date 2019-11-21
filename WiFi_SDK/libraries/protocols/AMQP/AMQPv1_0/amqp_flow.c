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
 *  flow functions
 *
 *  Provides flow methods for use by library
 */

#include "wiced.h"
#include "amqp.h"
#include "amqp_internal.h"
#include "amqp_frame.h"
#include "amqp_flow.h"
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

wiced_result_t wiced_amqp_update_link_credit( wiced_amqp_link_instance *link_instance, uint32_t link_credit )
{
     wiced_amqp_flow_instance args;

     /* link credit can only be updated for receiver link, if its sender link then just return */
     if ( link_instance->link_args.role == WICED_AMQP_ROLE_SENDER )
     {
         return WICED_ERROR;
     }

     args.flow_args.next_incoming_id = link_instance->session->begin_args.next_incoming_id;
     args.flow_args.next_outgoing_id = link_instance->session->begin_args.next_outgoing_id;
     args.flow_args.link_credit = ( link_instance->link_credit + link_credit );
     args.flow_args.incoming_window = link_instance->session->begin_args.incoming_window;
     args.flow_args.handle = link_instance->link_args.handle;
     args.flow_args.outgoing_window = link_instance->session->begin_args.outgoing_window;
     args.flow_args.delivery_count = link_instance->delivery_count;
     args.link_instance = link_instance;

     return amqp_manager( WICED_AMQP_EVENT_FLOW_SENT, &args, 0, link_instance->session->connection_instance );
}

wiced_result_t amqp_frame_put_flow( wiced_amqp_frame_t *frame,  void* arg )
{
    uint32_t total_size = 0;
    uint32_t count = 0;
    uint32_t size = 0;
    wiced_amqp_flow_instance *args = arg;

    /* Encode next incoming id as it is uint code : 0x70 */
    if ( args->flow_args.next_incoming_id == 0xFFFFFFFF )
    {
        size++ ;
        count++ ;
    }
    else
    {
        size++ ;
        size = size + 4;
        count++ ;
    }


    /* Encode incoming_window as it is uint code : 0x70 */
    if ( args->flow_args.incoming_window == 0xFFFFFFFF )
    {
        size++ ;
        count++ ;
    }
    else
    {
        size++ ;
        size = size + 4;
        count++ ;
    }

    /* Encode next outgoing id as it is uint code : 0x70 */
    if ( args->flow_args.next_outgoing_id == 0xFFFFFFFF )
    {
        size++ ;
        count++ ;
    }
    else
    {
        size++ ;
        size = size + 4;
        count++ ;
    }


    /* Encode outgoing_window as it is uint code : 0x70 */
    if ( args->flow_args.outgoing_window == 0xFFFFFFFF )
    {
        size++ ;
        count++ ;
    }
    else
    {
        size++ ;
        size = size + 4;
        count++ ;
    }


    /* Encode handle as it is uint code : 0x70 */
    if ( args->flow_args.handle == 0xFFFFFFFF )
    {
        size++ ;
        count++ ;
    }
    else
    {
        size++ ;
        size = size + 4;
        count++ ;
    }

    /* Encode delivery_count as it is uint code : 0x70 */
    if ( args->flow_args.delivery_count == 0xFFFFFFFF )
    {
        size++ ;
        count++ ;
    }
    else
    {
        size++ ;
        size = size + 4;
        count++ ;
    }


    /* Encode link_credit as it is uint code : 0x70 */
    if ( args->flow_args.link_credit == 0xFFFFFFFF )
    {
        size++ ;
        count++ ;
    }
    else
    {
        size++ ;
        size = size + 4;
        count++ ;
    }

    /* skipping available, drain, echo and properties */


    size = size + 1; /* for count */
    total_size = size + 8 + 3 + 1 + 1 ; /* (8)fixed frame + (3)performative + (1)descriptor + (1)size byte */

    AMQP_BUFFER_PUT_LONG( &frame->buffer, total_size ); /*  4  total size    */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x02 );      /*  1  doff          */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x00 );      /*  1  type          */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x00 );      /*  2  channel       */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );    /*  2  descriptor   */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, AMQP_FLOW );      /*  1  performative */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xc0 );      /*  1  constructor  */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, size );      /*  1  size         */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, count);      /*  1  count        */


    /* Encode next incoming id as it is uint code : 0x70 */
    if ( args->flow_args.next_incoming_id == 0xFFFFFFFF )
    {
        size-- ;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );
        count-- ;
    }
    else
    {
        size++ ;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
        size = size - 4;
        AMQP_BUFFER_PUT_LONG( &frame->buffer, args->flow_args.next_incoming_id);
        count-- ;
    }

    /* Encode  incoming window as it is uint code : 0x70 */
    if ( args->flow_args.incoming_window == 0xFFFFFFFF )
    {
        size-- ;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );
        count-- ;
    }
    else
    {
        size++ ;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
        size = size - 4;
        AMQP_BUFFER_PUT_LONG( &frame->buffer, args->flow_args.incoming_window);
        count-- ;
    }


    /* Encode next outgoing id as it is uint code : 0x70 */
    if ( args->flow_args.next_outgoing_id == 0xFFFFFFFF )
    {
        size-- ;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );
        count-- ;
    }
    else
    {
        size++ ;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
        size = size - 4;
        AMQP_BUFFER_PUT_LONG( &frame->buffer, args->flow_args.next_outgoing_id);
        count-- ;
    }

    /* Encode  outgoing window as it is uint code : 0x70 */
    if ( args->flow_args.outgoing_window == 0xFFFFFFFF )
    {
        size-- ;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );
        count-- ;
    }
    else
    {
        size++ ;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
        size = size - 4;
        AMQP_BUFFER_PUT_LONG( &frame->buffer, args->flow_args.outgoing_window);
        count-- ;
    }

    /* Encode  handle as it is uint code : 0x70 */
    if ( args->flow_args.handle == 0xFFFFFFFF )
    {
        size-- ;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );
        count-- ;
    }
    else
    {
        size++ ;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
        size = size - 4;
        AMQP_BUFFER_PUT_LONG( &frame->buffer, args->flow_args.handle );
        count-- ;
    }

    /* Encode  delivery_count as it is uint code : 0x70 */
    if ( args->flow_args.delivery_count == 0xFFFFFFFF )
    {
        size-- ;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );
        count-- ;
    }
    else
    {
        size++ ;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
        size = size - 4;
        AMQP_BUFFER_PUT_LONG( &frame->buffer, args->flow_args.delivery_count );
        count-- ;
    }

    /* Encode  link_credit as it is uint code : 0x70 */
    if ( args->flow_args.link_credit == 0xFFFFFFFF )
    {
        size-- ;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );
        count-- ;
    }
    else
    {
        size++ ;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
        size = size - 4;
        AMQP_BUFFER_PUT_LONG( &frame->buffer, args->flow_args.link_credit );
        count-- ;
    }

    return WICED_SUCCESS;
}


wiced_result_t amqp_frame_get_flow_performative( wiced_amqp_frame_t *frame, void *arg )
{
    wiced_amqp_packet_content *args = (wiced_amqp_packet_content *) arg;
    uint16_t descriptor = 0;
    uint32_t size = 0;
    uint32_t temp = 0;
    uint32_t count = 0;

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
    else if ( temp == 0xd0 )
    {
        AMQP_BUFFER_GET_LONG( &frame->buffer, size );

        AMQP_BUFFER_GET_LONG( &frame->buffer, count );
    }
    else
    {

    }
    size--; /* for count */

    /* parsing next incoming id */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        size--;

        if ( temp == 0x43 )
        {
            args->args.flow_args.next_incoming_id = 0;
            count-- ;
            size-- ;
        }
        else if ( temp == 0x52 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.flow_args.next_incoming_id );
            count-- ;
            size-- ;
        }
        else if ( temp == 0x70 )
        {
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.flow_args.next_incoming_id );
            count-- ;
            size = size - 4;
        }
        else if ( temp == 0x40 )
        {

        }
    }

    /* parsing  incoming window */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        size--;

        if ( temp == 0x43 )
        {
            args->args.flow_args.incoming_window = 0;
            count-- ;
            size-- ;
        }
        else if ( temp == 0x52 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.flow_args.incoming_window );
            count-- ;
            size-- ;
        }
        else if ( temp == 0x70 )
        {
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.flow_args.incoming_window );
            count-- ;
            size = size - 4;
        }
        else if ( temp == 0x40 )
        {

        }
    }


    /* parsing  next outgoing id */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        size--;

        if ( temp == 0x43 )
        {
            args->args.flow_args.next_outgoing_id = 0;
            count-- ;
            size-- ;
        }
        else if ( temp == 0x52 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.flow_args.next_outgoing_id );
            count-- ;
            size-- ;
        }
        else if ( temp == 0x70 )
        {
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.flow_args.next_outgoing_id );
            count-- ;
            size = size - 4;
        }
        else if ( temp == 0x40 )
        {

        }
    }

    /* parsing outgoing window */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        size--;

        if ( temp == 0x43 )
        {
            args->args.flow_args.outgoing_window = 0;
            count-- ;
            size-- ;
        }
        else if ( temp == 0x52 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.flow_args.outgoing_window );
            count-- ;
            size-- ;
        }
        else if ( temp == 0x70 )
        {
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.flow_args.outgoing_window );
            count-- ;
            size = size - 4;
        }
        else if ( temp == 0x40 )
        {

        }
    }

    /* parsing handle */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        size--;

        if ( temp == 0x43 )
        {
            args->args.flow_args.handle = 0;
            count-- ;
            size-- ;
        }
        else if ( temp == 0x52 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.flow_args.handle );
            count-- ;
            size-- ;
        }
        else if ( temp == 0x70 )
        {
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.flow_args.handle );
            count-- ;
            size = size - 4;
        }
        else if ( temp == 0x40 )
        {

        }
    }

    /* parsing delivery count */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        size--;
        if ( temp == 0x43 )
        {
            args->args.flow_args.delivery_count = 0;
            count-- ;
            size-- ;
        }
        else if ( temp == 0x52 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.flow_args.delivery_count );
            count-- ;
            size-- ;
        }
        else if ( temp == 0x70 )
        {
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.flow_args.delivery_count );
            count-- ;
            size = size - 4;
        }
        else if ( temp == 0x40 )
        {

        }
    }

    /* parsing link credit */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        size-- ;
        if ( temp == 0x43 )
        {
            args->args.flow_args.link_credit = 0;
            count-- ;
            size-- ;
        }
        else if ( temp == 0x52 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.flow_args.link_credit );
            count-- ;
            size-- ;
        }
        else if ( temp == 0x70 )
        {
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.flow_args.link_credit );
            count-- ;
            size = size - 4;
        }
        else if ( temp == 0x40 )
        {

        }
    }
    /* rest all parameters skipping */
    frame->buffer.data = frame->start + args->fixed_frame.size;
    return WICED_SUCCESS;
}

wiced_result_t amqp_connection_backend_put_flow_performative( wiced_amqp_flow_instance* flow_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_EVENT_FLOW_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &flow_instance->link_instance->session->connection_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_flow( &frame, flow_instance );
    return amqp_frame_send( &frame, &flow_instance->link_instance->session->connection_instance->conn->socket );
}

wiced_result_t amqp_connection_backend_get_flow_performative( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret = WICED_SUCCESS;

     ret = amqp_manager( WICED_AMQP_EVENT_FLOW_RCVD, args, 0, conn_instance );

    //TODO : Need to use all the args what we have received
    if ( ret == WICED_SUCCESS )
    {
        if ( conn_instance->conn->callbacks.connection_event != NULL )
        {
            conn_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_FLOW_RCVD, args, conn_instance );
        }
    }
    return ret;

}
