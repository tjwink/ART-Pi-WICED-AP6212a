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
#include "amqp_frame.h"
#include "amqp_transfer.h"
#include "amqp_manager.h"
#include "amqp.h"
#include "wiced_tls.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define TCP_SERVER_THREAD_PRIORITY          (WICED_DEFAULT_LIBRARY_PRIORITY)
#define TCP_SERVER_STACK_SIZE               (6200)
#define TCP_CLIENT_PORT                     (50007)
#define TCP_CLIENT_INTERVAL                 (2)
#define TCP_CLIENT_CONNECT_TIMEOUT          (500)
#define TCP_CONNECTION_NUMBER_OF_RETRIES    (3)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    wiced_packet_t* packet;
    void*           data;
} wiced_amqp_network_message_t;

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

wiced_result_t wiced_amqp_send( wiced_amqp_link_instance* link_instance, wiced_amqp_message_t* message )
{
    wiced_result_t result;
    amqp_delivery_t* delivery;
    amqp_transfer_t transfer_instance;

    if ( link_instance == NULL || link_instance->link_state != AMQP_LINK_STATE_ATTACHED )
    {
        return WICED_ERROR;
    }

    /* make sure we have enough link credit from peer before sending data */
    if ( link_instance->link_credit == 0 )
    {
        return WICED_ERROR;
    }

    /* create delivery instance and keep it in link as pending deliveries till we dont get Disposition [ acknowledgement ] */
    result = amqp_get_buffer( (void**) &delivery, sizeof(amqp_delivery_t) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_DEBUG(("memory not available to create link endpoint"));
        return WICED_ERROR;
    }

    /* delivery ID should be unique for particular session */
    link_instance->session->begin_args.next_outgoing_id++;
    /* use delivery count as delivery tag so which can be unique */
    link_instance->delivery_count += 1;
    /* decrement remote session window */
    link_instance->session->begin_args.remote_incoming_window--;

    if ( message->settle == WICED_FALSE )
    {
        delivery->delivery_id = link_instance->session->begin_args.next_outgoing_id;
        delivery->link = link_instance;
        delivery->delivery_tag = message->delivery_tag;
        /* add delivery information to delivery list which will be used when we get disposition */
        result = linked_list_insert_node_at_rear( &link_instance->message_delivery_list, &delivery->this_node );
    }

    /* Fill parameters needed to send transfer packet */
    transfer_instance.delivery_id  = link_instance->session->begin_args.next_outgoing_id;
    transfer_instance.delivery_tag = message->delivery_tag;
    transfer_instance.handle = link_instance->link_endpoint->output_handle;
    transfer_instance.message = message;
    transfer_instance.more = message->more;
    transfer_instance.settle = message->settle;
    transfer_instance.link = link_instance;

    /* decrement the link credit */
    link_instance->link_credit--;

    return amqp_manager( WICED_AMQP_EVENT_TRANSFER_SENT, &transfer_instance, 0, link_instance->session->connection_instance );
}

wiced_result_t amqp_connection_backend_put_transfer_performative( amqp_transfer_t* transfer_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_EVENT_TRANSFER_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &transfer_instance->link->session->connection_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_transfer( &frame, transfer_instance );
    return amqp_frame_send( &frame, &transfer_instance->link->session->connection_instance->conn->socket );
}

wiced_result_t amqp_connection_backend_put_disposition_performative( amqp_disposition_t* disposition_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_EVENT_DISPOSITION_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &disposition_instance->link->session->connection_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_disposition( &frame, disposition_instance );
    return amqp_frame_send( &frame, &disposition_instance->link->session->connection_instance->conn->socket );
}

wiced_result_t amqp_frame_put_transfer( wiced_amqp_frame_t *frame,  void* arg )
{
    uint64_t total_size = 0;
    uint32_t count = 0;
    uint32_t size = 0;
    amqp_transfer_t *params = arg;

    if ( params->handle == 0 )
    {
        count++;
        size++;
    }
    else
    {
        count++;
        size += 2;
    }

    if ( params->delivery_id == 0 )
    {
        count++;
        size++;
    }
    else if ( params->delivery_id <= 255 )
    {
        count++;
        size += 2;
    }
    else
    {
        count++;
        size += 5;
    }

    /* size for delivery id */
    count++;
    size += 6;

    /* size for message format, settle and more flag */
    count += 3;
    size += 3;

    size = size + 1;
    total_size = size + 8 + 3 + 2 + params->message->data_lenth + 5;
    /* fill fixed AMQP header */
    AMQP_BUFFER_PUT_LONG( &frame->buffer, total_size );                                 /*  4  total size   */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x02 );                                      /*  1  doff         */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x00 );                                      /*  1  type         */
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, params->link->session->endpoints->outgoing_channel );       /*  2  channel      */
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );                                    /*  2  descriptor   */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x14 );                                      /*  1  performative */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xc0 );                                      /*  1  constructor  */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, size );                                      /*  1  size         */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, count);                                      /*  1  count        */

    /* encoding handle */
    if ( params->handle == 0 )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x43 );
    }
    else
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x52 );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, params->handle );
    }

    /* encoding delivery-id */
    if ( params->delivery_id == 0 )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x43 );
    }
    else if ( params->delivery_id <= 255 )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x52 );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, params->delivery_id );
    }
    else
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
        AMQP_BUFFER_PUT_LONG( &frame->buffer, params->delivery_id );
    }

    /* encoding delivery-tag */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xa0 );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x04 );
    AMQP_BUFFER_PUT_LONG( &frame->buffer, params->delivery_tag );

    /* encoding message-format */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x43 );

    /* Encoding settle flag */
    if ( params->settle == WICED_TRUE )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x41 );
    }
    else
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x42 );
    }

    /* Encoding more flag */
    if ( params->more == WICED_TRUE )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x41 );
    }
    else
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x42 );
    }

    /* Encoding payload data */
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x77 );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xa1 );
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, params->message->data, params->message->data_lenth );

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_disposition( wiced_amqp_frame_t *frame,  void* arg )
{
    uint64_t total_size = 0;
    uint32_t count = 0;
    uint32_t size = 0;
    amqp_disposition_t *params = arg;

    /* Field Role */
    count++;
    size++;

    /* Field First */
    count++;
    if ( params->first == 0 )
    {
        size ++;
    }
    else
    {
        size = size + 2;
    }

    /* Field last */
    count++;
    if ( params->last == 0 )
    {
        size ++;
    }
    else
    {
        size = size + 2;
    }

    /* Field settled */
    count++;
    size ++;

    count++;
    size += 4;

    size = size + 1;
    total_size = size + 8 + 3 + 2;
    /* fill fixed AMQP header */
    AMQP_BUFFER_PUT_LONG( &frame->buffer, total_size );                                 /*  4  total size   */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x02 );                                      /*  1  doff         */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x00 );                                      /*  1  type         */
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, params->link->session->endpoints->outgoing_channel );       /*  2  channel      */
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );                                    /*  2  descriptor   */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x15 );                                      /*  1  performative */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xc0 );                                      /*  1  constructor  */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, size );                                      /*  1  size         */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, count);                                      /*  1  count        */

    /* encoding Role */
    if ( params->link->link_args.role == WICED_AMQP_ROLE_RECEIVER )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x41 );
    }
    else
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x42 );
    }

    /* encoding first */
    if ( params->first == 0)
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x43 );
    }
    else
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x52 );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, params->first );
    }

    /* encoding last */
    if ( params->last == 0 )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x43 );
    }
    else
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x52 );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, params->last);
    }

    /* encoding delivery-tag */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x41 );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x00 );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x53 );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x24 );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x45 );

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_disposition_performative( wiced_amqp_frame_t *frame, void *arg )
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

    size--;
    if ( count != 0 && size != 0 )
    {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        count-- ;
        size-- ;
        if ( temp == 0x41 )
        {
            args->args.disposition_args.is_receiver = WICED_TRUE;
        }
        if ( temp == 0x42 )
        {
            args->args.disposition_args.is_receiver = WICED_FALSE;
        }
    }

    /* parsing first */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

        if ( temp == 0x43 )
        {
            args->args.disposition_args.first = 0;
            count-- ;
            size-- ;
        }
        if( temp == 0x52 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.disposition_args.first );
            count--;
            size -= 2;
        }
        if ( temp == 0x70 )
        {
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.disposition_args.first  );
            count--;
            size= size - 4;
        }
    }

    /* parsing last */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

        if ( temp == 0x40 )
        {
            /* If not set, this is taken to be the same as first. */
            args->args.disposition_args.last = args->args.disposition_args.first;
            count-- ;
            size-- ;
        }
        if ( temp == 0x43 )
        {
            args->args.disposition_args.last = 0;
            count-- ;
            size-- ;
        }
        if( temp == 0x52 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.disposition_args.last );
            count--;
            size -= 2;
        }
        if ( temp == 0x70 )
        {
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.disposition_args.last );
            count--;
            size= size - 4;
        }
    }

    /* parse settled flag */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        count-- ;
        size-- ;
        if ( temp == 0x41 )
        {
            args->args.disposition_args.settled = WICED_TRUE;
        }
        if ( temp == 0x42 )
        {
            args->args.disposition_args.settled = WICED_FALSE;
        }
    }

    /* parsing delivery status  */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_SHORT( &frame->buffer, temp );
        count-- ;
        size -= 2 ;

        AMQP_BUFFER_GET_OCTET ( &frame->buffer, temp );
        size--;

        if ( temp == 0x23 )
        {
            args->args.disposition_args.delivery_state = WICED_AMQP_DELIVERY_RECEIVED;
        }

        /* delivery accepted if it is 0x24 */
        if ( temp == 0x24 )
        {
            args->args.disposition_args.delivery_state = WICED_AMQP_DELIVERY_ACCEPTED;
        }

        if ( temp == 0x25 )
        {
            args->args.disposition_args.delivery_state = WICED_AMQP_DELIVERY_REJECTED;
        }

        if ( temp == 0x26 )
        {
            args->args.disposition_args.delivery_state = WICED_AMQP_DELIVERY_RELEASED;
        }

        if ( temp == 0x27 )
        {
            args->args.disposition_args.delivery_state = WICED_AMQP_DELIVERY_MODIFIED;
        }

    }

    /* parse list of elements */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_SHORT( &frame->buffer, temp );
        count-- ;
        size--;

        AMQP_BUFFER_GET_OCTET ( &frame->buffer, temp );
        size--;
    }

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_transfer_performative( wiced_amqp_frame_t *frame, void *arg )
{
    wiced_amqp_packet_content *args = (wiced_amqp_packet_content *) arg;
    uint16_t descriptor = 0;
    uint64_t size = 0;
    uint64_t temp = 0;
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

    size--;

    /* parsing handle */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        count-- ;
        size-- ;
        if ( temp == 0x43 )
        {
            args->args.transfer_args.handle = 0;
        }
        if ( temp == 0x52 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size--;
            args->args.transfer_args.handle = (uint32_t)temp;
         }
    }

    /* parse delivery-id */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        count-- ;
        size-- ;
        if ( temp == 0x43 )
        {
            args->args.transfer_args.delivery_id = 0;
        }
        if ( temp == 0x52 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size--;
            args->args.transfer_args.delivery_id = (uint32_t)temp;
        }
        if ( temp == 0x70 )
        {
            AMQP_BUFFER_GET_LONG( &frame->buffer, temp );
            size= size - 4;
            args->args.transfer_args.delivery_id = (uint32_t)temp;
        }
    }

    /* parse delivery-tag */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        size--;
        if ( temp == 0xa0 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            AMQP_BUFFER_GET_LONG_LONG( &frame->buffer, temp );
            args->args.transfer_args.message.delivery_tag = temp;
        }
        if ( temp == 0xb0 )
        {

        }
        count--;
        size -= 9;
    }

    /* parse message format */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        count-- ;
        size-- ;
        if ( temp == 0x43 )
        {
            args->args.transfer_args.message.message_format = 0;
        }
        if ( temp == 0x52 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size-- ;
            args->args.transfer_args.message.message_format = (uint8_t) temp;
        }
    }

    /* parse settled flag */
    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
    if ( temp == 0x41 )
    {
        args->args.transfer_args.message.settle = WICED_TRUE;
    }
    else
    {
        args->args.transfer_args.message.settle = WICED_FALSE;
    }
    count--;
    size--;

    /* parse more flag */
    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
    if ( temp == 0x41 )
    {
        args->args.transfer_args.message.more = WICED_TRUE;
    }
    else
    {
        args->args.transfer_args.message.more = WICED_FALSE;
    }
    count--;
    size--;

    /* skip transfer arguments which we dont need for now [rcv-settle-mode, state, resume, aborted, batchable ] */
    AMQP_BUFFER_GET_LONG( &frame->buffer, temp );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
    size = size - 5;

    AMQP_BUFFER_GET_SHORT( &frame->buffer, temp );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

    /* skip message header */
    if ( temp == 0x70 )
    {
        /* if message header present then skip */
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        size = temp;
        frame->buffer.data += temp;
        size -= temp;
    }

    AMQP_BUFFER_GET_SHORT( &frame->buffer, temp );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

    /* skip message-properties */
    if ( temp == 0x73 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

        /* get the list of elements */
        if ( temp == 0x45 )
        {
            temp = 0;
        }
        if ( temp == 0xc0 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        }
        if ( temp == 0xd0 )
        {
            AMQP_BUFFER_GET_LONG( &frame->buffer, temp );
        }
        size = temp;
        frame->buffer.data += temp;
        size -= temp;
    }

    AMQP_BUFFER_GET_SHORT( &frame->buffer, temp );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

    /* Read actual message payload */
    if ( temp == 0x77 || temp == 0x75 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

        if ( temp == 0xa1 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            args->args.transfer_args.message.data_lenth = (uint32_t) temp;
            args->args.transfer_args.message.data = frame->buffer.data;
        }
        else if ( temp == 0xb1 )
        {
            AMQP_BUFFER_GET_LONG( &frame->buffer, temp );
            args->args.transfer_args.message.data_lenth = (uint32_t) temp;
            args->args.transfer_args.message.data = frame->buffer.data;
        }
    }

    return WICED_SUCCESS;
}

wiced_result_t amqp_connection_backend_get_disposition_performative( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret;

    ret = amqp_manager( WICED_AMQP_EVENT_DISPOSITION_RCVD, args, 0, conn_instance );
    if ( ret == WICED_SUCCESS )
    {
        if ( conn_instance->conn->callbacks.connection_event != NULL )
        {
            conn_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_DISPOSITION_RCVD, args, conn_instance );
        }
    }
    return ret;
}


wiced_result_t amqp_connection_backend_get_transfer_performative( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret = WICED_SUCCESS;

    ret = amqp_manager( WICED_AMQP_EVENT_TRANSFER_RCVD, args, 0, conn_instance );
    if ( ret == WICED_SUCCESS )
    {
        if ( conn_instance->conn->callbacks.connection_event != NULL )
        {
            conn_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_TRANSFER_RCVD, args, conn_instance );
        }
    }
    return ret;
}

