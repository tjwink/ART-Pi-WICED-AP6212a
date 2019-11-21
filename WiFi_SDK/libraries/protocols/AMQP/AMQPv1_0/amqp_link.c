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
#include "amqp_link.h"
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

wiced_result_t wiced_amqp_detach( wiced_amqp_link_instance *link_instance)
{
    wiced_amqp_delink_instance args;

    memset( &args, 0, sizeof(wiced_amqp_delink_instance) );

    args.detach_args.closed = WICED_TRUE;
    args.detach_args.handle = link_instance->link_endpoint->output_handle;
    args.link = link_instance;
    return amqp_manager( WICED_AMQP_EVENT_DETACH_SENT, &args, 0, link_instance->session->connection_instance );
}

wiced_result_t wiced_amqp_attach( wiced_amqp_link_instance* link_instance, wiced_amqp_session_instance_t* session_instance )
{
    wiced_result_t result = WICED_ERROR;
    amqp_link_endpoints *link_endpoint;


    link_instance->link_args.max_message_size = 1024;
    link_instance->link_args.initial_delivery_count = 0;

    /* create new link endpoint */
    result = amqp_get_buffer( (void**) &link_endpoint, sizeof(amqp_link_endpoints) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_DEBUG(("memory not available to create link endpoint"));
        return WICED_ERROR;
    }

    memset( link_endpoint, 0, sizeof(amqp_link_endpoints) );

    link_endpoint->output_handle = link_instance->link_args.handle;
    session_instance->link_endpoint_count++ ;

    link_endpoint->name = (uint8_t*) link_instance->link_args.name;
    result = linked_list_insert_node_at_rear( &session_instance->link_endpoint, &link_endpoint->this_node );

    /* initialize message delivery list */
    linked_list_init( &link_instance->message_delivery_list );

    link_instance->link_endpoint = link_endpoint;
    link_endpoint->context = (wiced_amqp_link_instance*) link_instance;

    return amqp_manager( WICED_AMQP_EVENT_ATTACH_SENT, link_instance, 0, session_instance->connection_instance );
}

/******************************************************
 *               Protocol Header Frames
 ******************************************************/

wiced_result_t amqp_frame_put_link( wiced_amqp_frame_t *frame,  void* params )
{
    uint32_t total_size = 0;
    uint32_t count = 0;
    uint32_t size = 0;
    wiced_amqp_link_instance *args = params;

    /* Encode name */
    if ( args->link_args.name == NULL )
    {
        size++ ;
        count++ ;
    }
    else
    {
        /* As it is name string-8 type  code : 0xal*/
        size++ ;
        size++ ;
        size = size + strlen( (char*) args->link_args.name );
        count++ ;
    }

    //    /* Encode handle as it is uint code : 0x70 */
    //    size++ ;
    //    size = size + 4;
    //    count++ ;
    size++ ;
    size = size + 4;
    count++ ;

    /* Encode role as it is based on boolean sender/recevier (FALSE/TRUE) */
    size++ ;
    size = size + 1;
    count++ ;

    /* Encode snd_settle_mode as it is ubyte code : 0x50 */
    size++ ;
    size = size + 1;
    count++ ;

    /* Encode recv_settle_mode as it is ubyte code : 0x50 */
    size++ ;
    size = size + 1;
    count++ ;

    /* Encode source as it is list */
    //TODO: currently handling only first element; need to handle all elements in the list
    if ( args->link_args.source == NULL )
    {
        size++;
        size++;

        /* To represent list code : 0xC0 */
        size++ ;
        /* to represent size of list */
        size++ ;
        /* to represent count */
        size++ ;
        /* to represent NULL */
        size++ ;
        count++ ;
    }
    else
    {
        size++;
        size++;
        /* To represent list code : 0xC0 */
        size++ ;
        /* to represent size of list */
        size++ ;
        /* to represent count */
        size++ ;
        /* As it is name string-8 type  code : 0xal*/
        size++ ;
        size++ ;
        size = size + strlen( (char*) args->link_args.source );
        count++ ;
    }

    /* Encode target as it is list */
    //TODO: currently handling only first element; need to handle all elements in the list
    if ( args->link_args.target == NULL )
    {
        size++;
        size++;
        /* To represent list code : 0xC0 */
        size++ ;
        /* to represent size of list */
        size++ ;
        /* to represent count */
        size++ ;
        /* to represent NULL */
        size++ ;
        count++ ;
    }
    else
    {
        size++;
        size++;
        /* To represent list code : 0xC0 */
        size++ ;
        /* to represent size of list */
        size++ ;
        /* to represent count */
        size++ ;
        /* As it is name string-8 type  code : 0xal*/
        size++ ;
        size++ ;
        size = size + strlen( (char*) args->link_args.target );
        count++ ;
    }

    /* Encode unsettled currently not handled so always NULL */
    //TODO : As we are ignoring so alway NULL
    size++ ;
    count++ ;

    /* Encode incoming-unsettled currently not handled so always NULL */
    //TODO : As we are ignoring so alway NULL
    size++ ;
    count++ ;

    /* Encode intial-delivery-count as it is uint code : 0x70 */
    //TODO : As we are ignoring for receiver so alway NULL

    if ( args->link_args.role == WICED_AMQP_ROLE_SENDER )
    {
        size++ ;
        size = size + 4;
        count++ ;
    }
    else
    {
        size++ ;
        count++ ;
    }
    /* Encode max-message-size as it is ulong code : 0x80 */
    size++ ;
    size = size + 8;
    count++ ;

    size = size + 1; /* for count */
    total_size = size + 8 + 3 + 1 + 1 ; /* (8)fixed frame + (3)performative + (1)descriptor + (1)size byte */

    /* TODO : Added extra byte, this will be addressed later */
    AMQP_BUFFER_PUT_LONG( &frame->buffer, total_size+1 ); /*  4  total size    */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x02 );      /*  1  doff          */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x00 );      /*  1  type          */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, args->session->endpoints->outgoing_channel );      /*  2  channel       */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );    /*  2  descriptor   */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, AMQP_ATTACH );      /*  1  performative */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xc0 );      /*  1  constructor  */

    /* TODO : Added extra byte, this will be addressed later */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, size + 1 );      /*  1  size         */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, count);      /*  1  count        */

    /* Encode container id */
    if ( args->link_args.name == NULL )
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
        AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->link_args.name, strlen( (char* ) args->link_args.name ) );
        size++ ;
        size = size + strlen( (char*) args->link_args.name );
        count++ ;

    }
        /* Encode handle handle as it is uint code : 0x70 */
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
        size++ ;
        AMQP_BUFFER_PUT_LONG( &frame->buffer, args->link_args.handle );
        size = size + 4;
        count++ ;

//    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x43 );
//    size++ ;
//    count++ ;

    /* Encode role as it is based on boolean sender/recevier (FALSE/TRUE) */
    if ( args->link_args.role == WICED_AMQP_ROLE_RECEIVER )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x41 );
        size++ ;
        count++ ;
    }
    else if ( args->link_args.role == WICED_AMQP_ROLE_SENDER )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x42 );
        size++ ;
        count++ ;
    }
    else
    {
        WPRINT_LIB_DEBUG ((" Invalid value for role of link \r\n"));
    }

    /* Encode snd_settle_mode as it is ubyte code : 0x50 */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x50 );
    size++ ;
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->link_args.snd_settle_mode );
    size = size + 1;
    count++ ;

    /* Encode recv_settle_mode as it is ubyte code : 0x50 */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x50 );
    size++ ;
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->link_args.rcv_settle_mode );
    size = size + 1;
    count++ ;

    /* Encode source as it is list */
    //TODO: currently handling only first element; need to handle all elements in the list
    if ( args->link_args.source == NULL )
    {
        uint32_t temp_size = 0;

        AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );    /*  2  descriptor   */
        size++;

        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x28 );      /*  1  performative */
        size++;

        /* To represent list code : 0xC0 */
        temp_size++;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xC0 );
        size++ ;
        /* to represent size of list */
        temp_size++;
        temp_size = temp_size + strlen( (char*) args->link_args.source );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, temp_size+1 );
        size++ ;
        /* to represent count */
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x01 );
        size++ ;
        /* to represent NULL */
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );
        size++ ;
        count++;
    }
    else
    {
        uint32_t temp_size = 0;

        AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );    /*  2  descriptor   */
        size++;

        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x28 );      /*  1  performative */
        size++;

        /* To represent list code : 0xC0 */
        temp_size++;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xC0 );
        size++ ;
        /* to represent size of list */
        temp_size++;
        temp_size = temp_size + strlen( (char*) args->link_args.source );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, temp_size+1 );
        size++ ;
        /* to represent count */
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x01 );
        size++ ;
        /* As it is name string-8 type  code : 0xal*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xa1 );
        size++ ;
        AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->link_args.source, strlen( (char* )args->link_args.source ) );
        size++ ;
        size = size + strlen( (char*) args->link_args.source );
        count++ ;
    }

    /* Encode target as it is list */
    //TODO: currently handling only first element; need to handle all elements in the list
    if ( args->link_args.target == NULL )
    {
        uint32_t temp_size = 0;

        AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );    /*  2  descriptor   */
        size++;

        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x28 );      /*  1  performative */
        size++;

        /* To represent list code : 0xC0 */
        temp_size++;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xC0 );
        size++ ;
        /* to represent size of list */
        temp_size++;
        temp_size = temp_size + strlen( (char*) args->link_args.target );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, temp_size+1 );
        size++ ;
        /* to represent count */AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x01 );
        size++ ;
        /* to represent NULL */AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );
        size++ ;
        count++ ;
    }
    else
    {
        uint32_t temp_size = 0;

        AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );    /*  2  descriptor   */
        size++;

        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x29 );      /*  1  performative */
        size++;

        /* To represent list code : 0xC0 */
        temp_size++;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xC0 );
        size++ ;
        /* to represent size of list */
        temp_size++;
        temp_size = temp_size + strlen( (char*) args->link_args.target );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, temp_size+1 );
        size++ ;
        /* to represent count */AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x01 );
        size++ ;
        /* As it is name string-8 type  code : 0xal*/
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xa1 );
        size++ ;
        AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->link_args.target, strlen( (char* )args->link_args.target ) );
        size++ ;
        size = size + strlen( (char*) args->link_args.target );
        count++ ;
    }

    /* Encode unsettled currently not handled so always NULL */
    //TODO : As we are ignoring so always NULL
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );
    size++ ;
    count++ ;

    /* Encode incoming-unsettled currently not handled so always NULL */
    //TODO : As we are ignoring so always NULL
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );
    size++ ;
    count++ ;

   /* Encode initial-delivery-count as it is uint code : 0x70 */
    //TODO : As we are ignoring so always NULL

    if ( args->link_args.role == WICED_AMQP_ROLE_SENDER )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
        size++ ;
        AMQP_BUFFER_PUT_LONG( &frame->buffer, args->link_args.initial_delivery_count);
        size = size + 4;
        count++ ;
    }
    else
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x43 );
        size++ ;
        count++ ;
    }



    /* Encode max-message-size as it is ulong code : 0x80 */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x80 );
    size++ ;
    AMQP_BUFFER_PUT_LONG_LONG( &frame->buffer, args->link_args.max_message_size);
    size = size + 8;
    count++ ;

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_attach_performative( wiced_amqp_frame_t *frame, void *arg )
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

    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x40 )
        {
            args->args.link_args.name = NULL;
            count-- ;
            size-- ;
        }
        else
        {
            uint8_t temp_size =0 ;
            ( ( &frame->buffer )->data )-- ;
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size-- ;
            AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->args.link_args.name, temp_size );
            count-- ;
            size-- ;
            size = size - temp_size;
        }

    }

    /* parsing handle */
    if ( count != 0 && size != 0 )
    {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x43 )
        {
            args->args.link_args.handle = 0;
            count-- ;
            size-- ;
        }
    }

    /* Parsing  role as it is based on boolean sender/receiver (FALSE/TRUE)  */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        count-- ;
        size-- ;
        if ( temp == 0x40 )
        {
            WPRINT_LIB_DEBUG ((" NO value for role of link is received \r\n"));
        }
        else
        {
            if ( temp == 0x41 )
            {
                args->args.link_args.role = WICED_TRUE;
            }
            else if ( temp == 0x42 )
            {
                args->args.link_args.role = WICED_FALSE;
            }
            else
            {
                args->args.link_args.role = 0xFF ;
                WPRINT_LIB_DEBUG ((" Invalid value for role of link received \r\n"));
            }
        }
    }
        /* parsing snd_settle_mode as it is ubyte code : 0x50 */
    if ( count != 0 && size != 0 )
    {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x40 )
        {
            WPRINT_LIB_DEBUG ((" No value for role of snd_settle_mode received \r\n"));
            count-- ;
            size-- ;
        }
        else if ( temp == 0x50 )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.link_args.snd_settle_mode );
            size-- ;
            count-- ;
        }
    }
    /* parsing recv_settle_mode as it is ubyte code : 0x50 */
    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x40 )
        {
            WPRINT_LIB_DEBUG ((" No value for role of recv_settle_mode received \r\n"));
            count-- ;
            size-- ;
        }
        else if(temp == 0x50)
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.link_args.rcv_settle_mode );
            size-- ;
            count-- ;
        }
    }

    /* Parsing source */
    if ( count != 0 && size != 0 )
    {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x40 )
        {
            WPRINT_LIB_DEBUG ((" No value for source received \r\n"));
            count-- ;
            size-- ;
        }
        else
        {
            uint32_t temp_size = 0;
            ( ( &frame->buffer )->data )-- ;
            AMQP_BUFFER_GET_SHORT( &frame->buffer, temp );
            size = size - 2;

            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size--;

            /*parse list = 0xc0*/
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size--;

            /*PARSE list size */
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp_size );
            size--;

            /*parse the count */
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size--;
            temp_size--;

            /* parse the source */
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size--;
            temp_size--;
            if(temp == 0x40 )
            {
                args->args.link_args.source = NULL;
                args->args.link_args.source_size = 0;
            }
            else if(temp == 0xa1)
            {
                ( ( &frame->buffer )->data )-- ;
                size--;
                temp_size--;
                AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.link_args.source_size );
                AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->args.link_args.source,  args->args.link_args.source_size );
                size = size -  args->args.link_args.source_size;
                temp_size = temp_size -  args->args.link_args.source_size;

            }
            else
            {

            }

            /* skip all other elements */
            ( ( &frame->buffer )->data ) = ( ( &frame->buffer )->data ) + temp_size;
        }

    }

    /* Parsing target */
       if ( count != 0 && size != 0 )
       {
           AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
           if ( temp == 0x40 )
           {
               WPRINT_LIB_DEBUG ((" No value for target received \r\n"));
               count-- ;
               size-- ;
           }
           else
           {
               uint32_t temp_size = 0;
               ( ( &frame->buffer )->data )-- ;
               AMQP_BUFFER_GET_SHORT( &frame->buffer, temp );
               size = size - 2;

               AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
               size--;

               /*parse list = 0xc0*/
               AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
               size--;

               /*PARSE list size */
               AMQP_BUFFER_GET_OCTET( &frame->buffer, temp_size );
               size--;

               /*parse the count */
               AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
               size--;
               temp_size--;

               /* parse the source */
               AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
               size--;
               temp_size--;
               if(temp == 0x40 )
               {
                   args->args.link_args.target = NULL;
                   args->args.link_args.target_size = 0;
               }
               else if(temp == 0xa1)
               {
                   ( ( &frame->buffer )->data )-- ;

                   size--;
                   temp_size--;
                   AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.link_args.target_size );
                   AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->args.link_args.target,  args->args.link_args.target_size );
                   size = size -  args->args.link_args.target_size;
                   temp_size = temp_size -  args->args.link_args.target_size;

               }
               else
               {

               }

               /* skip all other elements */
               ( ( &frame->buffer )->data ) = ( ( &frame->buffer )->data ) + temp_size;
           }

       }

       /* parsing unsettled */

       if ( count != 0 && size != 0 )
       {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x40 )
        {
            WPRINT_LIB_DEBUG ((" No value for unsettled received \r\n"));
            count-- ;
            size-- ;
        }
        else
        {

        }

     }

       /* parsing incoming unsettled */
    if ( count != 0 && size != 0 )
    {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x40 )
        {
            WPRINT_LIB_DEBUG ((" No value for initial delivery count received \r\n"));
            count-- ;
            size-- ;
        }
        else
        {

        }
    }

       /* parsing initial delivery count */
    if ( count != 0 && size != 0 )
    {

        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x40 )
        {
            args->args.link_args.initial_delivery_count = 0xFFFFFFFF;
            WPRINT_LIB_DEBUG ((" No value for initial delivery count received \r\n"));
            count-- ;
            size-- ;
        }
       //TODO : Implement if we receive any value.

    }

       /* parsing max message size */

    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        count-- ;
        size-- ;
        if ( temp == 0x40 )
        {
            WPRINT_LIB_DEBUG ((" No value for target received \r\n"));

        }
        else if ( temp == 0x80 )
        {
            AMQP_BUFFER_GET_LONG_LONG( &frame->buffer, args->args.link_args.max_message_size);
            count-- ;
            size = size - 8;
        }
        else
        {

        }
    }
    return WICED_SUCCESS;
}

wiced_result_t amqp_connection_backend_get_attach_performative( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret = WICED_SUCCESS;

     ret = amqp_manager( WICED_AMQP_EVENT_ATTACH_RCVD, args, 0, conn_instance );

    //TODO : Need to use all the args what we have received
    if ( ret == WICED_SUCCESS )
    {
        if ( conn_instance->conn->callbacks.connection_event != NULL )
        {
            conn_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_ATTACH_RCVD, args, conn_instance );
        }
    }
    return ret;

}

wiced_result_t amqp_connection_backend_put_attach_performative( wiced_amqp_link_instance* link_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send attach packet */
    ret = amqp_frame_create( WICED_AMQP_EVENT_ATTACH_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &link_instance->session->connection_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_link( &frame, link_instance );
    return amqp_frame_send( &frame, &link_instance->session->connection_instance->conn->socket );
    return WICED_SUCCESS;
}

wiced_result_t amqp_connection_backend_get_detach_performative( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance )
{
    wiced_result_t ret = WICED_SUCCESS;

    ret = amqp_manager( WICED_AMQP_EVENT_DETACH_RCVD, args, 0, conn_instance );

    //TODO : Need to use all the args what we have received
    if ( ret == WICED_SUCCESS )
    {
        if ( conn_instance->conn->callbacks.connection_event != NULL )
        {
            conn_instance->conn->callbacks.connection_event( WICED_AMQP_EVENT_DETACH_RCVD, args, conn_instance );
        }
    }
    return ret;

}

wiced_result_t amqp_connection_backend_put_detach_performative( wiced_amqp_delink_instance* delink_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_EVENT_DETACH_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &delink_instance->link->session->connection_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_detach_performative( &frame, delink_instance );
    return amqp_frame_send( &frame, &delink_instance->link->session->connection_instance->conn->socket );
}


wiced_result_t amqp_frame_get_detach_performative( wiced_amqp_frame_t *frame, void *arg )
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

    size = size - 1;

    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x43 )
        {
            args->args.detach_args.handle = 0;
            count-- ;
            size-- ;
        }
        else if ( temp == 0x40 )
        {
            args->args.detach_args.handle_is_not_specified = 1;
        }
        else if ( temp == 0x70 )
        {
            ( ( &frame->buffer )->data )-- ;
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size-- ;
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.detach_args.handle);
            count-- ;
            size = size - 4;
        }
        else
        {
            ( ( &frame->buffer )->data )-- ;
            AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
            size-- ;
            AMQP_BUFFER_GET_SHORT( &frame->buffer, args->args.detach_args.handle );
            count-- ;
            size = size - 2;
        }

    }

    if ( count != 0 && size != 0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
        if ( temp == 0x41 )
        {
            args->args.detach_args.closed = WICED_TRUE;
        }
        else if ( temp == 0x42 )
        {
            args->args.detach_args.closed = WICED_FALSE;
        }
        else
        {

        }
    }



    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_detach_performative( wiced_amqp_frame_t *frame,  void* arg )
{
    wiced_amqp_packet_content detach_packet;
    wiced_amqp_delink_instance *delink_instance = arg;
    uint32_t size = 0;
    uint32_t count = 0;
    uint32_t total_size = 0;

    detach_packet.fixed_frame.channel = delink_instance->link->session->endpoints->outgoing_channel;
    detach_packet.fixed_frame.doff = 0x02;
    detach_packet.fixed_frame.type = WICED_AMQP_FRAME_TYPE;
    detach_packet.fixed_frame.performative_type = AMQP_DETACH;

    detach_packet.fixed_frame.size = 8 + 3 + 1 + 1 + 1 + 1;

    /* link handle */
    size++ ;
    size = size + 4;
    count++ ;

    /* closed field */
    size++ ;
    count++ ;

    /* error field */
    size++ ;
    count++ ;

    size = size + 1; /* for count */

    total_size = size + 8 + 3 + 1 + 1;

    AMQP_BUFFER_PUT_LONG( &frame->buffer, total_size ); /*  4 */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, detach_packet.fixed_frame.doff );/*  1 */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, detach_packet.fixed_frame.type );/*  1 */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, detach_packet.fixed_frame.channel );/*  2 */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 );/*  2 */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, detach_packet.fixed_frame.performative_type );/*  1 */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xC0 );

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, size );

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, count );

//    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );

    /* encode handle */
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x70 );
    size++ ;
    AMQP_BUFFER_PUT_LONG( &frame->buffer, delink_instance->detach_args.handle );
    size = size + 4;
    count++ ;

  //  AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x43 );


    /* Encode closed filed */
    if ( delink_instance->detach_args.closed == WICED_TRUE )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x41 );

    }
    else if ( delink_instance->detach_args.closed == WICED_FALSE )
    {
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x42 );

    }
    else
    {

    }

    /*error should be NULL*/
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x40 );

    return WICED_SUCCESS;
}

