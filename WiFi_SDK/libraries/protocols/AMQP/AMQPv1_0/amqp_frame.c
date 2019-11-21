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
 *  Frame packing and unpacking functions
 */

#include "wiced.h"
#include "amqp.h"
#include "amqp_connection.h"
#include "amqp_frame.h"
#include "amqp_session.h"
#include "amqp_link.h"
#include "amqp_open.h"
#include "amqp_transfer.h"
#include "amqp_flow.h"
#include "amqp_sasl.h"

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
#define LENGTH_OF_STRING(X) ((X).len + sizeof((X).len))
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
wiced_result_t  amqp_frame_recv( wiced_amqp_buffer_t *buffer, void *p_user, uint32_t *size )
{
    wiced_amqp_connection_instance *connection_instance = p_user;
    uint32_t amqp_str;
    wiced_amqp_frame_t frame = { 0 };
    wiced_result_t result = WICED_SUCCESS;

    if ( NULL == buffer )
    {
        WPRINT_LIB_DEBUG( ( "[ AMQP LIB ERROR ] : Nothing received empty buffer\r\n" ) );
    }
    /* Check valid type type */
    frame.start = buffer->data;
    frame.buffer = *buffer;
    frame.buffer.data = frame.start;

    if ( connection_instance->current_connection_state == AMQP_CONNECTION_HDR_SENT || connection_instance->current_connection_state == AMQP_CONNECTION_SASL_HDR_SENT   )
    {
        wiced_amqp_protocol_header_arg_t args;
        if ( frame.size != 7 )
        {
            WPRINT_LIB_ERROR( ( "[ AMQP LIB ERROR ] : Received protocol header size is not matching and size = %d\r\n",frame.size ) );
        }
        memset( &args, 0, sizeof(wiced_amqp_protocol_header_arg_t) );
        AMQP_BUFFER_GET_LONG( &frame.buffer, amqp_str );
        amqp_frame_get_protocol_header( &frame, &args );
        result = amqp_connection_backend_get_protocol_header( &args, connection_instance );
        *size = 8;
    }
    else
    {
        amqp_fixed_frame fixed_frame;
        uint8_t performative_type = AMQP_UNKNOWN_TYPE;
        performative_type  = get_fixed_frame( frame , &fixed_frame);

        switch ( performative_type )
        {
            case AMQP_SASL_SRV_MECHS:
            {
                wiced_amqp_packet_content args;
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_sasl_mechansims( &frame, &args );
                result = amqp_connection_backend_get_sasl_mechanisms( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_SASL_INIT:
            {
                break;
            }
            case AMQP_SASL_OUTCOME:
            {
                wiced_amqp_packet_content args;
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_sasl_outcome( &frame, &args );
                result = amqp_connection_backend_get_sasl_outcome( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_OPEN:
            {
                wiced_amqp_packet_content args;
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_open_performative( &frame, &args );
                result = amqp_connection_backend_get_open_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_BEGIN:
            {
                wiced_amqp_packet_content args;
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_begin_performative( &frame, &args );
                result = amqp_connection_backend_get_begin_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_ATTACH:
            {
                wiced_amqp_packet_content args;
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_attach_performative( &frame, &args );
                result = amqp_connection_backend_get_attach_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_FLOW:
            {
                wiced_amqp_packet_content args;
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_flow_performative( &frame, &args );
                result = amqp_connection_backend_get_flow_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_TRANSFER:
            {
                wiced_amqp_packet_content args;
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_transfer_performative( &frame, &args );
                result = amqp_connection_backend_get_transfer_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_DISPOSITON:
            {
                wiced_amqp_packet_content args;
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_disposition_performative( &frame, &args );
                result = amqp_connection_backend_get_disposition_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_DETACH:
            {
                wiced_amqp_packet_content args;
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_detach_performative( &frame, &args );
                result = amqp_connection_backend_get_detach_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_END:
            {
                wiced_amqp_packet_content args;
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                amqp_frame_get_end_performative( &frame, &args );
                result = amqp_connection_backend_get_end_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_CLOSE:
            {
                wiced_amqp_packet_content args;
                memset( &args, 0, sizeof(wiced_amqp_packet_content) );
                memcpy( &args.fixed_frame, &fixed_frame, sizeof(amqp_fixed_frame) );
                amqp_frame_get_close_performative( &frame, &args );
                result = amqp_connection_backend_get_close_performative( &args, connection_instance );
                *size = args.fixed_frame.size;
                break;
            }
            case AMQP_UNKNOWN_TYPE:
            {
                break;
            }
            default:
            {
                break;
            }

        }

   }
    return result;
}

uint8_t get_fixed_frame( wiced_amqp_frame_t frame, amqp_fixed_frame *fixed_frame )
{
    uint8_t performative_type = AMQP_UNKNOWN_TYPE;
     uint16_t descriptor;

    AMQP_BUFFER_GET_LONG( &frame.buffer,fixed_frame->size );
    AMQP_BUFFER_GET_OCTET( &frame.buffer, fixed_frame->doff );
    AMQP_BUFFER_GET_OCTET( &frame.buffer, fixed_frame->type );
    AMQP_BUFFER_GET_SHORT( &frame.buffer, fixed_frame->channel );

    AMQP_BUFFER_GET_SHORT( &frame.buffer, descriptor );
    AMQP_BUFFER_GET_OCTET( &frame.buffer, fixed_frame->performative_type );
    performative_type = fixed_frame->performative_type;
    return performative_type;
}

wiced_result_t  amqp_frame_create( wiced_amqp_frame_type_t type, uint16_t channel, uint16_t max_size, wiced_amqp_frame_t *frame, wiced_amqp_socket_t *socket )
{
    wiced_result_t ret;
    (void) type;
    (void) channel;
    ret = amqp_network_create_buffer( &frame->buffer, max_size, socket );
    if ( ret == WICED_SUCCESS )
    {
        frame->size = 0;
        frame->start = frame->buffer.data;
    }
    return ret;
}

wiced_result_t  amqp_frame_delete( wiced_amqp_frame_t *frame )
{
    return amqp_network_delete_buffer( &frame->buffer );
}


wiced_result_t  amqp_frame_send( wiced_amqp_frame_t *frame, wiced_amqp_socket_t *socket )
{
    return amqp_network_send_buffer( &frame->buffer, socket );
}


wiced_result_t amqp_get_buffer( void** buffer, uint32_t size )
{
    /* Allocate buffer object */
    *buffer = malloc_named( "amqp_buffer", size );
    wiced_assert( "failed to malloc", ( buffer != NULL ) );
    if ( *buffer == NULL )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    return WICED_SUCCESS;
}
