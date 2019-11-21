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
 *  I/O functions
 *
 */
#include "wiced.h"
#include "amqp_manager.h"
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

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                   Function prototypes
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

wiced_result_t amqp_frame_get_sasl_mechansims( wiced_amqp_frame_t *frame, void *arg )
{
    wiced_amqp_packet_content *args = (wiced_amqp_packet_content *) arg;
    uint16_t descriptor = 0;
    uint32_t size = 0;
    uint32_t temp = 0;
    uint32_t count = 0;
    uint32_t i = 0;
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

    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

    if ( temp == 0xe0 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, size );

        AMQP_BUFFER_GET_OCTET( &frame->buffer, count );
        args->args.mech_args.count = count;
    }
    else
    {

    }

    /* for symbol : 0xA3 */
    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

    /*Decoding list*/

    if ( temp == 0xA3 )
    {
        for ( i = 0; i < count; i++ )
        {
            AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.mech_args.list[i].size );
            args->args.mech_args.list[ i ].item = ( &frame->buffer )->data;
            ( ( &frame->buffer )->data ) = ( ( &frame->buffer )->data ) + args->args.mech_args.list[ i ].size;
        }
    }
    else if ( temp == 0xB3 )
    {
        for ( i = 0; i < count; i++ )
        {
            AMQP_BUFFER_GET_LONG( &frame->buffer, args->args.mech_args.list[i].size );
            args->args.mech_args.list[ i ].item = ( &frame->buffer )->data;
            ( ( &frame->buffer )->data ) = ( ( &frame->buffer )->data ) + args->args.mech_args.list[ i ].size;
        }
    }
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_sasl_outcome( wiced_amqp_frame_t *frame, void *arg )
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

    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );

    if ( temp == 0x50 )
    {
        AMQP_BUFFER_GET_OCTET( &frame->buffer, args->args.outcome_args.code );
    }
    else
    {

    }

    /* TODO : Parse additional data */
    return WICED_SUCCESS;
}
wiced_result_t sasl_init_get_initial_response(wiced_amqp_connection_instance *args,char **data,uint32_t *initial_response_size)
{
    wiced_result_t result = WICED_SUCCESS;
    uint32_t authzid_length = args->plain_config.authzid == NULL ? 0 : strlen( args->plain_config.authzid );
    uint32_t authcid_length = strlen( args->plain_config.authcid );
    uint32_t passwd_length = strlen( args->plain_config.passwd );

    if ( ( args->plain_config.authcid == NULL ) || ( args->plain_config.passwd == NULL ) )
    {
        result = WICED_ERROR;
    }
    else
    {
        /*  authcid   = 1*SAFE ; MUST accept up to 255 octets] ;  authzid   = 1*SAFE ; MUST accept up to 255 octets]  ; passwd    = 1*SAFE ; MUST accept up to 255 octets] */
        if ( ( authcid_length > 255 ) || ( authcid_length == 0 ) || ( authzid_length > 255 ) || ( passwd_length > 255 ) || ( passwd_length == 0 ) )
        {
            result = WICED_ERROR;
        }
        else
        {
            /* Ignore UTF8 for now */
            *data = (char*) calloc( authzid_length + authcid_length + passwd_length + 2 ,sizeof(char));
            if ( *data == NULL )
            {
                return WICED_ERROR;
            }
            else
            {
                /*  [The mechanism consists of a single message, a string of [UTF-8] encoded [Unicode] characters, from the client to the server.] */
                /*  [The client presents the authorization identity (identity to act as), followed by a NUL (U+0000) character, followed by the authentication identity (identity whose password will be used), followed by a NUL (U+0000) character, followed by the clear-text password.] */
                /* [   message   = [authzid] UTF8NUL authcid UTF8NUL passwd] */
                /*  [The authorization identity (authzid), authentication identity (authcid), password (passwd), and NUL character deliminators SHALL be transferred as [UTF-8] encoded strings of [Unicode] characters.] */
                /*  [As the NUL (U+0000) character is used as a deliminator, the NUL (U+0000) character MUST NOT appear in authzid, authcid, or passwd productions.] */

                /*  [As with other SASL mechanisms, the client does not provide an authorization identity when it wishes the server to derive an identity from the credentials and use that as the authorization identity.] */
                if ( authzid_length > 0 )
                {
                    (void) memcpy( *data, args->plain_config.authzid, authzid_length );
                }

                *( *data + authzid_length ) = '\0';
                (void) memcpy( *data + authzid_length + 1, args->plain_config.authcid, authcid_length );
                *( *data + authzid_length + authcid_length + 1 ) = '\0';
                (void) memcpy( *data + authzid_length + authcid_length + 2, args->plain_config.passwd, passwd_length );
                *initial_response_size = (uint32_t) ( authzid_length + authcid_length + passwd_length + 2 );
            }
        }
    }
    return result;

}
wiced_result_t amqp_frame_put_sasl_init( wiced_amqp_frame_t *frame,  void* arg )

{
    uint32_t total_size = 0;
    uint32_t count = 0;
    uint32_t size = 0;
    uint32_t initial_response_size = 0;
    wiced_amqp_connection_instance *args = arg;
    char* data = NULL;
    wiced_result_t ret = WICED_SUCCESS;

    ret = sasl_init_get_initial_response( args, &data, &initial_response_size );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }

    size = strlen( SASL_METHOD_PLAIN ) + 1 + 1 + initial_response_size + 1 + 1;

    count = 2;

    size = size + 1; /* for count */
    total_size = size + 8 + 3 + 1 + 1; /* (8)fixed frame + (3)performative + (1)descriptor + (1)size byte */

    AMQP_BUFFER_PUT_LONG( &frame->buffer, total_size ); /*  4  total size    */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x02 ); /*  1  doff          */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x01 ); /*  1  type          */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x00 ); /*  2  channel       */

    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0053 ); /*  2  descriptor   */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0x41 ); /*  1  performative */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xc0 ); /*  1  constructor  */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, size ); /*  1  size         */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, count); /*  1  count        */

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xA3); /*  1  symbol        */
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, SASL_METHOD_PLAIN, strlen(SASL_METHOD_PLAIN) );

    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0xa0); /*  1  count        */
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, data, initial_response_size );

    free( data );
    data = NULL;

    return WICED_SUCCESS;
}

wiced_result_t amqp_connection_backend_get_sasl_mechanisms( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance )
{
    return amqp_manager( WICED_AMQP_EVENT_SASL_SRV_MECH_RCVD, args, 0, conn_instance );
}

wiced_result_t amqp_connection_backend_get_sasl_outcome( wiced_amqp_packet_content  *args, wiced_amqp_connection_instance *conn_instance )
{
    return amqp_manager( WICED_AMQP_EVENT_SASL_OUTCOME_RCVD, args, 0, conn_instance );
}

wiced_result_t amqp_connection_backend_put_sasl_init( wiced_amqp_connection_instance* conn_instance )
{
    wiced_result_t ret;
    wiced_amqp_frame_t frame;

    /* Send Protocol Header */
    ret = amqp_frame_create( WICED_AMQP_EVENT_SASL_INIT_SENT, 0, AMQP_CONNECTION_FRAME_MAX, &frame, &conn_instance->conn->socket );
    if ( ret != WICED_SUCCESS )
    {
        return ret;
    }
    amqp_frame_put_sasl_init( &frame, conn_instance );
    return amqp_frame_send( &frame, &conn_instance->conn->socket );
}
