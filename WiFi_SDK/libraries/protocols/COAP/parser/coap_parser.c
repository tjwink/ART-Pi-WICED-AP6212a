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
 * Common functions to used for both CoAP client and server
 */

#include "parser/coap_parser.h"
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
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Declarations
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t coap_parse_header( coap_header_t *hdr, const uint8_t *buf, size_t buflen )
{
    if ( buflen < COAP_HEADER_LENGTH )
    {
        return WICED_ERROR;
    }

    hdr->ver = ( buf[ 0 ] & COAP_HEADER_VERSION_MASK ) >> COAP_HEADER_VERSION_POSITION;

    if ( hdr->ver != COAP_PROTOCOL_VER )
    {
        return WICED_ERROR;
    }

    hdr->t = (uint8_t) ( ( buf[ 0 ] & COAP_HEADER_TYPE_MASK ) >> COAP_HEADER_TYPE_POSITION );

    hdr->tkl = buf[ 0 ] & COAP_HEADER_TOKEN_LEN_MASK;
    buf++ ;

    hdr->code = buf[ 0 ];
    buf++ ;

    hdr->id[ 0 ] = buf[ 0 ];
    buf++ ;

    hdr->id[ 1 ] = buf[ 0 ];
    buf++ ;

    return WICED_SUCCESS;
}

wiced_result_t coap_parse_token( wiced_coap_token_info_t* token_info, size_t token_len, uint8_t *buf, size_t buflen )
{
    if ( token_len == 0 )
    {
        token_info->token_len = 0;
        return WICED_SUCCESS;
    }
    else if ( token_len <= COAP_TOKEN_LENGTH )
    {
        if ( COAP_HEADER_LENGTH + token_len > (uint8_t) buflen )
        {
            return WICED_ERROR; // tok bigger than packet
        }

        memcpy( &token_info->data, buf + COAP_HEADER_LENGTH, token_len ); // past header
        token_info->token_len = token_len;
        return WICED_SUCCESS;
    }
    else
    {
        // invalid size
        return WICED_ERROR;
    }
}

wiced_result_t coap_parse_option( wiced_coap_option_t * options, uint16_t * running_delta, uint8_t * *buf, size_t buflen )
{
    uint8_t *data = *buf;
    uint8_t headlen = 1;
    uint16_t option_len, delta, option_number;

    if ( buflen < headlen ) // too small
    {
        return WICED_ERROR;
    }

    delta = ( data[ 0 ] & 0xF0 ) >> COAP_DELTA_LENGTH;
    option_len = data[ 0 ] & 0x0F;
    ++data;

    if ( delta == 13 )
    {
        headlen++ ;
        if ( buflen < headlen )
        {
            WPRINT_LIB_ERROR( ( "too short header\n" ) );
            return WICED_ERROR;
        }
        delta = (uint16_t) ( data[ 0 ] + 13 );
        data++ ;
    }
    else if ( delta == 14 )
    {
        headlen += 2;
        if ( buflen < headlen )
        {
            WPRINT_LIB_ERROR( ( "too short header\n" ) );
            return WICED_ERROR;
        }
        delta = ( ( data[ 0 ] << 8 ) | data[ 1 ] ) + 269;
        data += 2;
    }
    else if ( delta == 15 )
    {
        WPRINT_LIB_ERROR( ( "too short header\n" ) );
        return WICED_ERROR;
    }

    if ( option_len == 13 )
    {
        headlen++ ;
        if ( buflen < headlen )
        {
            WPRINT_LIB_ERROR( ( "too short header\n" ) );
            return WICED_ERROR;
        }
        option_len = data[ 0 ] + 13;
        data++ ;
    }
    else if ( option_len == 14 )
    {
        headlen += 2;
        if ( buflen < headlen )
        {
            WPRINT_LIB_ERROR( ( "option too short\n" ) );
            return WICED_ERROR;
        }
        option_len = ( ( data[ 0 ] << 8 ) | data[ 1 ] ) + 269;
        data += 2;
    }
    else if ( option_len == 15 )
    {
        WPRINT_LIB_ERROR( ( "option len invalid\n" ) );
        return WICED_ERROR;
    }

    option_number = delta + *running_delta;
    options->num = option_number;
    options->buf.len = option_len;
    options->buf.data = data;

    *buf = data + option_len;
    *running_delta += delta;

    return WICED_SUCCESS;
}

wiced_result_t coap_parse_options_and_payload( coap_packet_t** packet, uint8_t * buf, size_t buflen )
{
    size_t option_index = 0;
    uint16_t delta = 0;
    coap_packet_t* pkt = *packet;
    uint8_t *start = buf + COAP_HEADER_LENGTH + pkt->hdr.tkl;
    uint8_t *end = buf + buflen;
    wiced_result_t result;

    UNUSED_PARAMETER( result );

    if ( start > end )
    {
        return WICED_ERROR; // out of bounds

    }
    while ( ( option_index < WICED_COAP_MAX_OPTIONS ) && ( start < end ) && ( *start != 0xFF ) )
    {
        if ( ( result = coap_parse_option( &pkt->options.option[ option_index ], &delta, &start, end - start ) != WICED_SUCCESS ) )
        {
            WPRINT_LIB_ERROR( ( "Error in parsing COAP option\n" ) );
            return WICED_ERROR;
        }
        option_index++ ;
    }

    pkt->options.num_opts = option_index;

    if ( ( start + 1 < end ) && ( *start == PAYLOAD_MARKER ) )
    {
        pkt->payload.data = (uint8_t*) ( start + 1 );
        pkt->payload.len = end - ( start + 1 );
    }
    else
    {
        pkt->payload.data = NULL;
        pkt->payload.len = 0;
    }

    return WICED_SUCCESS;
}

void coap_option_nibble( uint8_t value, uint8_t *nibble )
{
    if ( value < 13 )
    {
        *nibble = ( 0xFF & value );
    }
    else if ( value <= 0xFF + 13 )
    {
        *nibble = 13;
    }
    else if ( value <= 0xFFFF + 269 )
    {
        *nibble = 14;
    }
}

int coap_frame_create( coap_packet_t* pkt, uint8_t* udp_data )
{
    int buflen = 0;
    size_t opts_len = 0;
    size_t i;
    uint8_t *p = NULL;
    uint16_t running_delta = 0;

    udp_data[ 0 ] = ( pkt->hdr.ver & COAP_HEADER_POSITION ) << COAP_HEADER_VERSION_POSITION;
    udp_data[ 0 ] |= ( pkt->hdr.t & COAP_HEADER_POSITION ) << COAP_HEADER_TYPE_POSITION;
    udp_data[ 0 ] |= ( pkt->hdr.tkl & COAP_HEADER_TOKEN_LEN_MASK );
    udp_data[ 1 ] = pkt->hdr.code;
    udp_data[ 2 ] = pkt->hdr.id[ 0 ];
    udp_data[ 3 ] = pkt->hdr.id[ 1 ];

    // Add Token
    p = udp_data + COAP_HEADER_LENGTH;

    if ( pkt->hdr.tkl > 0 )
    {
        memcpy( p, pkt->token.data, pkt->hdr.tkl );
    }

    // Add Options
    p += pkt->hdr.tkl;

    if ( pkt->options.num_opts > WICED_COAP_MAX_OPTIONS )
    {
        WPRINT_LIB_ERROR( ( "Too many options\n" ) );
        return buflen;
    }

    for ( i = 0; i < pkt->options.num_opts; i++ )
    {
        uint8_t options_delta;
        uint8_t len;
        uint8_t delta = 0;

        options_delta = pkt->options.option[ i ].num - running_delta;

        coap_option_nibble( options_delta, &delta );
        coap_option_nibble( pkt->options.option[ i ].buf.len, &len );

        *p++ = ( 0xFF & ( delta << 4 | len ) );

        if ( delta == 13 )
        {
            *p++ = ( options_delta - 13 );
        }
        else if ( delta == 14 )
        {
            *p++ = ( ( options_delta - 269 ) >> 8 );
            *p++ = ( 0xFF & ( options_delta - 269 ) );
        }
        if ( len == 13 )
        {
            *p++ = ( pkt->options.option[ i ].buf.len - 13 );
        }
        else if ( len == 14 )
        {
            *p++ = ( pkt->options.option[ i ].buf.len >> 8 );
            *p++ = ( 0xFF & ( pkt->options.option[ i ].buf.len - 269 ) );
        }

        memcpy( p, pkt->options.option[ i ].buf.data, pkt->options.option[ i ].buf.len );
        p += pkt->options.option[ i ].buf.len;
        running_delta = pkt->options.option[ i ].num;
    }

    opts_len = ( p - udp_data ) - COAP_HEADER_LENGTH;

    if ( pkt->payload.len > 0 )
    {
        udp_data[ COAP_HEADER_LENGTH + opts_len ] = PAYLOAD_MARKER;
        memcpy( udp_data + COAP_HEADER_LENGTH + opts_len + PAYLOAD_MARKER_LENGTH, pkt->payload.data, pkt->payload.len );
        buflen = opts_len + COAP_HEADER_LENGTH + PAYLOAD_MARKER_LENGTH + pkt->payload.len;
    }
    else
    {
        buflen = opts_len + COAP_HEADER_LENGTH;
    }

    return buflen;
}

wiced_coap_option_t *coap_find_options( coap_packet_t* request, uint8_t num )
{
    size_t i;
    wiced_coap_option_t *first = NULL;

    for ( i = 0; i < request->options.num_opts; i++ )
    {
        if ( request->options.option[ i ].num == num )
        {
            first = &request->options.option[ i ];
            return first;
        }
    }

    return first;
}

wiced_result_t coap_release_buffer( void* buffer )
{
    wiced_assert( "buffer is NULL", ( buffer != NULL ) );
    free( buffer );
    return WICED_SUCCESS;
}

wiced_result_t coap_get_buffer( void** buffer, uint32_t size )
{
    /* Allocate buffer object */
    *buffer = malloc_named( "coap_buffer", size );
    wiced_assert( "failed to malloc", ( buffer != NULL ) );
    if ( *buffer == NULL )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    return WICED_SUCCESS;
}

void coap_set_content_type( wiced_coap_option_info_t* coap_option, wiced_coap_content_type_t content_type )
{
    coap_option->option[ coap_option->num_opts ].num = COAP_OPTION_CONTENT_FORMAT;
    coap_serialize_int_option( coap_option, content_type );
    coap_option->option[ coap_option->num_opts ].buf.data = (uint8_t*) coap_option->option[ coap_option->num_opts ].type;
    coap_option->num_opts++ ;
}

void coap_set_observer( wiced_coap_option_info_t* coap_option, uint32_t value )
{
    coap_option->option[ coap_option->num_opts ].num = COAP_OPTION_OBSERVE;
    coap_serialize_int_option( coap_option, value );
    coap_option->option[ coap_option->num_opts ].buf.data = (uint8_t*) coap_option->option[ coap_option->num_opts ].type;
    coap_option->num_opts++ ;
}

void coap_serialize_int_option( wiced_coap_option_info_t* coap_option, uint32_t value )
{
    int i = 0;
    if ( 0xFF000000 & value )
    {
        ++i;
    }
    if ( 0xFFFF0000 & value )
    {
        ++i;
    }
    if ( 0xFFFFFF00 & value )
    {
        ++i;
    }
    if ( 0xFFFFFFFF & value )
    {
        ++i;
    }

    coap_option->option[ coap_option->num_opts ].buf.len = i;
    i = 0;

    if ( 0xFF000000 & value )
    {
        coap_option->option[ coap_option->num_opts ].type[ i++ ] = (uint8_t) ( value >> 24 );
    }
    if ( 0xFFFF0000 & value )
    {
        coap_option->option[ coap_option->num_opts ].type[ i++ ] = (uint8_t) ( value >> 16 );
    }
    if ( 0xFFFFFF00 & value )
    {
        coap_option->option[ coap_option->num_opts ].type[ i++ ] = (uint8_t) ( value >> 8 );
    }
    if ( 0xFFFFFFFF & value )
    {
        coap_option->option[ coap_option->num_opts ].type[ i++ ] = (uint8_t) ( value );
    }

}

void wiced_coap_set_uri_path( wiced_coap_option_info_t* coap_option, char *value )
{
    coap_option->option[ coap_option->num_opts ].num = COAP_OPTION_URI_PATH;
    coap_option->option[ coap_option->num_opts ].buf.data = (uint8_t*) value;
    coap_option->option[ coap_option->num_opts ].buf.len = strlen( value );

    coap_option->num_opts++ ;
}

void wiced_coap_set_uri_query( wiced_coap_option_info_t* coap_options, char *value )
{
    coap_options->option[ coap_options->num_opts ].num = COAP_OPTION_URI_QUERY;
    coap_options->option[ coap_options->num_opts ].buf.data = (uint8_t*) value;
    coap_options->option[ coap_options->num_opts ].buf.len = strlen( value );

    coap_options->num_opts++ ;
}
