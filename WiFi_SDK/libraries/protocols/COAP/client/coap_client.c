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
 *
 *  COAP Client :
 *
 *  This library is used to support CoAP client on WICED platform.
 *
 *  Functionality :
 *  -  Client can perform POST,GET,OBSERVE operation with server.
 *  -  Get notification from server.
 *
 */

#include "coap_client.h"
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
static wiced_result_t client_receive_callback( wiced_udp_socket_t *socket, void *args );
static wiced_result_t parse_response( coap_packet_t* packet, uint8_t* data, uint16_t data_length );
static wiced_result_t send_ack( wiced_udp_socket_t* socket, coap_packet_t* response, wiced_ip_address_t ip, uint16_t port );
static wiced_result_t send_request( wiced_udp_socket_t* socket, coap_packet_t* client_request, wiced_ip_address_t ip, uint16_t port );

/******************************************************
 *               Variables Declarations
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_coap_client_init( wiced_coap_client_t* client, wiced_interface_t interface, wiced_service_callback callback )
{
    wiced_result_t result;

    /* Create UDP socket */
    if ( ( result = wiced_udp_create_socket( &client->socket, 0, interface ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_INFO( ("UDP socket creation failed\n") );
        return result;
    }

    client->callback = callback;

    if ( ( result = wiced_udp_register_callbacks( &client->socket, client_receive_callback, client ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_INFO(("Error in registering udp callback\n"));
        return result;
    }

    return WICED_SUCCESS;

}

wiced_result_t wiced_coap_client_get( wiced_coap_client_t* client, wiced_coap_client_request_t* client_request, wiced_coap_msgtype_t msg_type, wiced_ip_address_t ip, uint16_t port )
{
    coap_packet_t packet;
    wiced_result_t result;
    uint16_t id;

    wiced_crypto_get_random( &id, sizeof( id ) );

    packet.hdr.ver = COAP_PROTOCOL_VER;
    packet.hdr.t = msg_type;
    packet.hdr.code = WICED_COAP_METHOD_GET;
    packet.hdr.id[ 0 ] = (uint8_t) ( id >> 8 );
    packet.hdr.id[ 1 ] = (uint8_t) id;
    packet.hdr.tkl = 0;

    memcpy( &packet.options, &client_request->options, sizeof( client_request->options ) );

    packet.payload.data = NULL;
    packet.payload.len = 0;

    if ( ( result = send_request( &client->socket, &packet, ip, port ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(("CoAP Request send failed\n"));
        return result;
    }

    return WICED_SUCCESS;
}

static wiced_result_t send_request( wiced_udp_socket_t* socket, coap_packet_t* client_request, wiced_ip_address_t ip, uint16_t port )
{
    wiced_packet_t *tx_packet;
    uint16_t available_data_length;
    wiced_result_t result;
    uint8_t *tx_udp_data;
    int length;

    if ( ( result = wiced_packet_create_udp( socket, 1024, &tx_packet, (uint8_t**) &tx_udp_data, &available_data_length ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(( "UDP TX packet creation failed\n" ));
        return result;
    }

    length = coap_frame_create( client_request, tx_udp_data );

    wiced_packet_set_data_end( tx_packet, (uint8_t*) tx_udp_data + length );

    if ( ( result = wiced_udp_send( socket, &ip, port, tx_packet ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR (( "udp packet send failed\n" ));
        wiced_packet_delete( tx_packet );
        return result;
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_coap_client_post( wiced_coap_client_t* client, wiced_coap_client_request_t* client_request, wiced_coap_msgtype_t msg_type, wiced_ip_address_t ip, uint16_t port )
{
    coap_packet_t packet;
    wiced_result_t result;
    uint16_t id;

    wiced_crypto_get_random( &id, sizeof( id ) );

    packet.hdr.ver = COAP_PROTOCOL_VER;
    packet.hdr.t = WICED_COAP_MSGTYPE_CON;
    packet.hdr.code = WICED_COAP_METHOD_POST;
    packet.hdr.id[ 0 ] = (uint8_t) ( id >> 8 );
    packet.hdr.id[ 1 ] = (uint8_t) id;
    packet.hdr.tkl = 0;

    if ( client_request->payload_type != WICED_COAP_CONTENTTYPE_NONE )
    {
        coap_set_content_type( &client_request->options, client_request->payload_type );
    }

    memcpy( &packet.options, &client_request->options, sizeof( client_request->options ) );

    packet.payload.data = client_request->payload.data;
    packet.payload.len = client_request->payload.len;

    if ( ( result = send_request( &client->socket, &packet, ip, port ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(("CoAP Request send failed\n"));
        return result;
    }

    return WICED_SUCCESS;
}

static wiced_result_t client_receive_callback( wiced_udp_socket_t *socket, void *args )
{
    wiced_ip_address_t udp_src_ip_addr;
    uint16_t udp_src_port;
    wiced_packet_t* packet;
    coap_packet_t response;
    char* udp_data;
    uint16_t data_length;
    uint16_t available_data_length;
    wiced_coap_client_event_info_t event_info;
    wiced_coap_option_t* option;
    wiced_result_t result;

    wiced_coap_client_t* client = (wiced_coap_client_t*) args;

    memset( &event_info, 0, sizeof( event_info ) );
    wiced_udp_receive( socket, &packet, WICED_TIMEOUT );
    wiced_udp_packet_get_info( packet, &udp_src_ip_addr, &udp_src_port );
    wiced_packet_get_data( packet, 0, (uint8_t**) &udp_data, &data_length, &available_data_length );
    if ( data_length < available_data_length )
    {
        WPRINT_LIB_INFO( ( "Fragmented packets not supported\n" ) );
        wiced_packet_delete( packet );
        return WICED_ERROR;
    }
    udp_data[ data_length ] = '\x0'; /* Null terminate the received string */

    if ( ( result = parse_response( &response, (uint8_t *) udp_data, data_length ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_INFO( ( "Error in parsing CoAP response packet\n" ) );
        wiced_packet_delete( packet );
        return result;;
    }

    /* Fill Token Information */
    event_info.token.token_len = response.token.token_len;
    memcpy( event_info.token.data, response.token.data, event_info.token.token_len );

    /* Fill Options Information which will be used by Application */
    event_info.opt.payload_type = WICED_COAP_CONTENTTYPE_NONE;
    event_info.opt.uri_path = NULL;

    option = coap_find_options( &response, COAP_OPTION_CONTENT_FORMAT );
    if ( option != NULL )
    {
        event_info.opt.payload_type = (wiced_coap_content_type_t) *(option->buf.data);
    }

    option = coap_find_options( &response, COAP_OPTION_URI_PATH );
    if ( option != NULL )
    {
        event_info.opt.uri_path = (char*) option->buf.data;
    }

    /* Fill Payload Information to be used by Application */
    event_info.payload.data = response.payload.data;
    event_info.payload.len = response.payload.len;

    switch ( response.hdr.code )
    {
        case COAP_RSPCODE_CHANGED:
            event_info.type = WICED_COAP_CLIENT_EVENT_TYPE_POSTED;
            break;

        case COAP_RSPCODE_CONTENT:
            option = coap_find_options( &response, COAP_OPTION_OBSERVE );

            if ( option != NULL )
            {
                /* If observe option is present & type is ACK then observe is successful */
                if ( response.hdr.t == WICED_COAP_MSGTYPE_ACK )
                {
                    event_info.type = WICED_COAP_CLIENT_EVENT_TYPE_OBSERVED;
                }
                else
                {
                    /* Send acknowledgment to server if type is CON */
                    if ( response.hdr.t == WICED_COAP_MSGTYPE_CON )
                    {
                        send_ack( socket, &response, udp_src_ip_addr, udp_src_port );
                    }
                    event_info.type = WICED_COAP_CLIENT_EVENT_TYPE_NOTIFICATION;
                }
            }
            else
            {
                event_info.type = WICED_COAP_CLIENT_EVENT_TYPE_GET_RECEIVED;
            }
            break;
    }

    client->callback( event_info );
    wiced_packet_delete( packet );

    return WICED_SUCCESS;
}

static wiced_result_t send_ack( wiced_udp_socket_t* socket, coap_packet_t* response, wiced_ip_address_t ip, uint16_t port )
{
    coap_packet_t packet;
    wiced_result_t result;

    memset( &packet, 0, sizeof( packet ) );

    packet.hdr.ver = COAP_PROTOCOL_VER;
    packet.hdr.t = WICED_COAP_MSGTYPE_ACK;
    packet.hdr.code = COAP_RSPCODE_EMPTY;
    memcpy( packet.hdr.id, response->hdr.id, sizeof( response->hdr.id ) );

    if ( ( result = send_request( socket, &packet, ip, port ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(("CoAP Request send failed\n"));
        return result;
    }

    return WICED_SUCCESS;

}

static wiced_result_t parse_response( coap_packet_t* packet, uint8_t* data, uint16_t data_length )
{
    wiced_result_t result;

    if ( ( result = coap_parse_header( &packet->hdr, data, data_length ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(("Error in parsing CoAP header\n"));
        return result;
    }

    if ( ( result = coap_parse_token( &packet->token, packet->hdr.tkl, data, data_length ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(("Error in parsing CoAP Token\n"));
        return result;
    }

    if ( ( result = coap_parse_options_and_payload( &packet, data, data_length ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(("Error in parsing CoAP Options and payload information\n"));
        return result;

    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_coap_client_observe( wiced_coap_client_t* client, wiced_coap_client_request_t* client_request, wiced_coap_msgtype_t msg_type, wiced_coap_token_info_t* token, wiced_ip_address_t ip, uint16_t port )
{
    coap_packet_t packet;
    wiced_result_t result;
    uint16_t id;
    int i = 0;

    memset( &packet, 0, sizeof( packet ) );
    wiced_crypto_get_random( &id, sizeof( id ) );

    packet.hdr.ver = COAP_PROTOCOL_VER;
    packet.hdr.t = msg_type;
    packet.hdr.code = WICED_COAP_METHOD_GET;
    packet.hdr.tkl = token->token_len;
    packet.hdr.id[ 0 ] = (uint8_t) ( id >> 8 );
    packet.hdr.id[ 1 ] = (uint8_t) id;

    packet.token.token_len = token->token_len;
    memcpy( packet.token.data, token->data, token->token_len );

    coap_set_observer( &packet.options, 0 );

    packet.options.num_opts = client_request->options.num_opts + 1;

    for ( i = 1; i < WICED_COAP_MAX_OPTIONS; i++ )
    {
        packet.options.option[ i ].num = client_request->options.option[ i - 1 ].num;
        packet.options.option[ i ].buf = client_request->options.option[ i - 1 ].buf;
    }

    packet.payload.data = NULL;
    packet.payload.len = 0;

    if ( ( result = send_request( &client->socket, &packet, ip, port ) ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(("CoAP Request send failed\n"));
        return result;
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_coap_client_deinit( wiced_coap_client_t* client )
{
    wiced_udp_delete_socket( &client->socket );
    wiced_udp_unregister_callbacks( &client->socket );
    return WICED_SUCCESS;

}
