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
#include "amqp_open.h"
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

wiced_result_t wiced_amqp_open( wiced_amqp_connection_instance *connection_instance)
{
    wiced_result_t ret;
    connection_instance->current_connection_state = AMQP_CONNECTION_START;
    if ( connection_instance->header_args.protocol_id == WICED_AMQP_PROTOCOL_ID_SASL )
    {
        connection_instance->is_sasl = 1;
    }
    ret = amqp_network_connect( &connection_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }

    connection_instance->current_connection_state = AMQP_CONNECTION_START;
    linked_list_init( &connection_instance->channel_endpoint );

    /* Make sure the static is valid to send */
    if ( connection_instance->is_sasl == WICED_TRUE )
    {
        return amqp_manager( WICED_AMQP_EVENT_SASL_HDR_SENT, connection_instance, 0, connection_instance );
    }
    else
    {
        return amqp_manager( WICED_AMQP_EVENT_HDR_SENT, connection_instance, 0, connection_instance );
    }

}

wiced_result_t wiced_amqp_close( wiced_amqp_connection_instance *connection_instance)
{
    /* Make sure the static is valid to send */
    return amqp_manager( WICED_AMQP_EVENT_CLOSE_SENT, connection_instance, 0, connection_instance );
}

/******************************************************
 *               Protocol Header Frames
 ******************************************************/

wiced_result_t amqp_frame_get_protocol_header( wiced_amqp_frame_t *frame, wiced_amqp_protocol_header_arg_t *args )
{
    WPRINT_LIB_DEBUG( ( "[ AMQP LIB ] : Parsing Amqp protocol header\r\n" ) );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->protocol_id );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->major );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->minor );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->revision );

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_protocol_header( wiced_amqp_frame_t *frame, const wiced_amqp_protocol_header_arg_t *args )
{
    /* A long value carrying string AMQP */
    uint32_t amqp_str = AMQP_FRAME_HEADER_PROTOCOL_STR;
    AMQP_BUFFER_PUT_LONG( &frame->buffer, amqp_str );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->protocol_id );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->major );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->minor );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->revision );

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_open_performative( wiced_amqp_frame_t *frame, void *arg )
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
    else if ( temp == 0xd0 )
    {
        AMQP_BUFFER_GET_LONG( &frame->buffer, size );

        AMQP_BUFFER_GET_LONG( &frame->buffer, count );
    }
    else
    {

    }

    if ( count != 0 && size != 0 )
    {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x40 )
        {
            args->args.open_args.container_id = NULL;
            count-- ;
            size-- ;
        }
        else
        {
            uint8_t temp_size =0 ;
            ( ( &frame->buffer )->data )-- ;
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size-- ;
            AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->args.open_args.container_id, args->args.open_args.container_id_size );
            count-- ;
            size-- ;
            size = size - temp_size;
        }

    }

    if ( count != 0 && size != 0 )
    {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x40 )
        {
            args->args.open_args.host_name = NULL;
            count-- ;
            size-- ;
        }
        else
        {
            uint8_t temp_size = 0;
            ( ( &frame->buffer )->data )-- ;
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size-- ;
            AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->args.open_args.host_name, args->args.open_args.host_name_size );
            count-- ;
            size-- ;
            size = size - temp_size;
        }

    }

    if ( count != 0 && size != 0 )
    {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x40 )
        {
            args->args.open_args.max_frame_size = 0;
            count-- ;
            size-- ;
        }
        else
        {
            ( ( &frame->buffer )->data )-- ;
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size-- ;
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.open_args.max_frame_size );
            count-- ;
            size = size - 4;
        }

    }

    if ( count != 0 && size != 0 )
    {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x40 )
        {
            args->args.open_args.channel_max = 0;
            count-- ;
            size-- ;
        }
        else
        {
            ( ( &frame->buffer )->data )-- ;
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size-- ;
            AMQP_BUFFER_GET_SHORT( &frame->buffer, args->args.open_args.channel_max );
            count-- ;
            size = size - 2;
        }

    }

    if ( count != 0 && size != 0 )
    {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x40 )
        {
            args->args.open_args.idle_timeout = 0;
            count-- ;
            size-- ;
        }
        else
        {
            ( ( &frame->buffer )->data )-- ;
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size-- ;
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.open_args.idle_timeout );
            count-- ;
            size = size - 4;
        }

    }

    return WICED_SUCCESS;
}
wiced_result_t amqp_frame_put_open( wiced_amqp_frame_t *frame,  void* arg )
{
    uint32_t total_size = 0;
    uint32_t count = 0;
    uint32_t size = 0;
    wiced_amqp_connection_instance *args = arg;

    /* Encode container id */
    if ( args->open_args.container_id == NULL )
    {
        size++ ;
        count++ ;
    }
    else
    {
        /* As it is container id string-8 type  code : 0xal*/
        size++ ;
        size++ ;
        size = size + args->open_args.container_id_size;
        count++ ;
    }

    /* Encode host name */
    if ( args->open_args.host_name == NULL )
    {
        size++ ;
        count++ ;
    }
    else
    {
        /* As it is host_name  string-8 type  code : 0xal*/
        size++ ;
        size++ ;
        size = size + args->open_args.host_name_size;
        count++ ;
    }

    /* Encode max frame size as it is uint code : 0x70 */
    size++ ;
    size = size + 4;
    count++ ;

    /* Encode channel max as it is ushort code : 0x60 */
    size++ ;
    size = size + 2;
    count++ ;

    //    /* Encode idle time out as it is miliseconds (uint) code : 0x60 */
    //    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
    //    AMQP_BUFFER_PUT_LONG( &frame->buffer, args->open_args.idle_timeout );
    //    size = size + 1 + 4;
    //    count++;
    //    size++;

    size = size + 1; /* for count */
    total_size = size + 8 + 3 + 1 + 1 ; /* (8)fixed frame + (3)performative + (1)descriptor + (1)size byte */

    AMQP_BUFFER_PUT_LONG( &frame->buffer, total_size ); /*  4  total size    */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x02 );      /*  1  doff          */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x00 );      /*  1  type          */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x00 );      /*  2  channel       */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );    /*  2  descriptor   */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x10 );      /*  1  performative */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xc0 );      /*  1  constructor  */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, size );      /*  1  size         */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, count);      /*  1  count        */

    /* Encode container id */
    if ( args->open_args.container_id == NULL )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );
        size++ ;
        count++ ;
    }
    else
    {
        /* As it is container id string-8 type  code : 0xal*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xa1 );
        size++ ;
        AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->open_args.container_id, args->open_args.container_id_size );
        size++ ;
        size = size + strlen( (char*) args->open_args.container_id );
        count++ ;

    }

    /* Encode host name */
    if ( args->open_args.host_name == NULL )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );
        size++ ;
        count++ ;
    }
    else
    {
        /* As it is host_name  string-8 type  code : 0xal*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xa1 );
        size++ ;
        AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->open_args.host_name,  args->open_args.host_name_size);
        size++ ;
        size = size + args->open_args.host_name_size;
        count++ ;

    }

    /* Encode max frame size as it is uint code : 0x70 */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
    size++ ;
    AMQP_BUFFER_PUT_LONG( &frame->buffer, args->open_args.max_frame_size );
    size = size + 4;
    count++ ;

    /* Encode channel max as it is ushort code : 0x60 */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x60 );
    size++ ;
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, args->open_args.channel_max );
    size = size + 2;
    count++ ;

    //    /* Encode idle time out as it is miliseconds (uint) code : 0x60 */
    //    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
    //    AMQP_BUFFER_PUT_LONG( &frame->buffer, args->open_args.idle_timeout );
    //    size = size + 1 + 4;
    //    count++;
    //    size++;
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_close_performative( wiced_amqp_frame_t *frame, void *arg )
{
    (void)arg;
    (void)frame;
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_close_performative( wiced_amqp_frame_t *frame,  void* arg )
{
    wiced_amqp_packet_content close_packet;
    (void) arg;
    close_packet.fixed_frame.channel = 0x00;
    close_packet.fixed_frame.doff = 0x02;
    close_packet.fixed_frame.type = WICED_AMQP_FRAME_TYPE;
    close_packet.fixed_frame.performative_type = 0x18;
    close_packet.fixed_frame.size = 8 + 3 + 1 + 1 + 1 + 1;

    AMQP_BUFFER_PUT_LONG( &frame->buffer, close_packet.fixed_frame.size ); /*  4 */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, close_packet.fixed_frame.doff );/*  1 */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, close_packet.fixed_frame.type );/*  1 */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, close_packet.fixed_frame.channel );/*  2 */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );/*  2 */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, close_packet.fixed_frame.performative_type );/*  1 */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xC0 );

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x02 );

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x01 );

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );

    return WICED_SUCCESS;
}
