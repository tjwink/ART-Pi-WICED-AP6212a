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

#include "wiced.h"
#include "tftp.h"

/******************************************************
 *                      Macros
 ******************************************************/
/*TODO:
 * Add retry count to exit after certain number of tries
 * Reading is calling the same data again and again.
 */
/******************************************************
 *                    Constants
 ******************************************************/
#define STR(s) str(s)
#define str(s) #s

#define TFTP_THREAD_STACK_SIZE          4096
#define TFTP_DATA_PACKET_LENGTH         560

#define TFTP_OPCODE_RRQ                 (1)
#define TFTP_OPCODE_WRQ                 (2)
#define TFTP_OPCODE_DATA                (3)
#define TFTP_OPCODE_ACK                 (4)
#define TFTP_OPCODE_ERROR               (5)
#define TFTP_OPCODE_OACK                (6)

#define TFTP_OPTION_TIMEOUT_STR         "timeout"
#define TFTP_OPTION_BLK_SIZE_STR        "blksize"
#define TFTP_OPTION_TRANSFER_SIZE_STR   "tsize"
#define TFTP_OPTION_TIMEOUT_VAL         2
#define TFTP_OPTION_BLK_SIZE_VAL        512
#define TFTP_OPTION_TRANSFER_SIZE_VAL   0

#define TFTP_ERROR_MESSAGE              "FILE_ERROR"
#define TFTP_SERVER_PORT                (69)
#define TFTP_MAX_TIMEOUT_COUNT          (10)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct tftp_client_connection_s
{
    uint16_t            tid;
    uint16_t            blk_size;
    uint16_t            timeout;
    uint16_t            curr_blk;
    uint32_t            t_size;
    wiced_ip_address_t  ip;
    wiced_udp_socket_t  socket;
    wiced_interface_t   interface;
}tftp_client_connection_t;

typedef enum
{
    TFTP_SERVER_STATE_LISTEN,    /* Send a read request */
    TFTP_SERVER_STATE_RRQ_DATA,  /* Receiving RRQ data */
    TFTP_SERVER_STATE_WRQ_DATA   /* Receiving RRQ data */
} tftp_server_state_t;

typedef enum
{
    TFTP_STATE_LISTEN,  /* Wait for requests */
    TFTP_STATE_OACK,    /* Wait for OACK */
    TFTP_STATE_REQUEST, /* Send a request */
    TFTP_STATE_GET,     /* Wait for data */
    TFTP_STATE_PUT      /* Send data */
} tftp_state_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
static const char *tftp_mode_str[ ] =
{
    "netascii",
    "octet"   ,
    "mail"
};

/******************************************************
 *               Function Definitions
 ******************************************************/
static uint16_t tftp_get_tid( void )
{
    /* TODO: replace with a random generator */
    return 50816;
}

static void tftp_buffer_add_string( char **buffer, const char * str )
{
    char *dest = *buffer;
    strcpy( dest, str );
    dest   += strlen( str );
    *dest   = '\0';
    dest   += 1 ;
    *buffer = dest;
}

static wiced_result_t tftp_error_create_packet( uint8_t** p_buffer, uint16_t error, const char* error_message )
{
    uint8_t *buffer = *p_buffer;

    /* Set Opcode */
    buffer[ 0 ] = 0;
    buffer[ 1 ] = TFTP_OPCODE_ERROR;

    /* Copy filename */
    buffer[ 2 ] = ( error & 0xFF00 ) >> 8;
    buffer[ 3 ] = ( error & 0xFF );
    buffer     += 4;

    /* Copy message string     */
    if ( error_message != NULL )
    {
        memcpy( buffer, error_message, strlen( error_message ) );
        buffer += strlen( error_message );
    }
    buffer  += 1;
    *buffer  = 0;
    buffer  += 1;
    *buffer  = 0;

    return WICED_SUCCESS;
}

static wiced_result_t tftp_error_send_packet( tftp_connection_t *conn, uint16_t error, const char * error_message )
{
    uint8_t*        tx_data;
    uint8_t*        buffer;
    uint16_t        available_data_length;
    wiced_packet_t* packet;

    if ( wiced_packet_create_udp( &conn->socket, TFTP_DATA_PACKET_LENGTH, &packet, (uint8_t**) &tx_data, &available_data_length ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    buffer = tx_data;
    tftp_error_create_packet( &buffer, error, error_message );
    wiced_packet_set_data_end( packet, buffer );

    if ( wiced_udp_send( &conn->socket, &conn->ip, conn->tid, packet ) != WICED_SUCCESS )
    {
        if(NULL!=packet)
            wiced_packet_delete( packet );
        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

static wiced_result_t tftp_data_parse_packet( uint8_t* buffer, uint16_t size, uint16_t *blk_num, uint8_t **data, uint16_t *recv_size )
{
    if ( ( buffer[ 0 ] != 0 ) || ( buffer[ 1 ] != TFTP_OPCODE_DATA ) )
    {
        return WICED_ERROR;
    }

    ( *blk_num ) = ( buffer[ 2 ] << 8 ) + buffer[ 3 ];
    *data = &buffer[ 4 ];
    *recv_size = size - 4;

    return WICED_SUCCESS;
}

static wiced_result_t tftp_data_create_packet( uint8_t** p_buffer, uint16_t blk_num, uint8_t* data, uint16_t size )
{
    uint8_t *buffer = *p_buffer;

    buffer[ 0 ] = 0;
    buffer[ 1 ] = TFTP_OPCODE_DATA;
    if ( ( buffer[ 0 ] != 0 ) || ( buffer[ 1 ] != TFTP_OPCODE_DATA ) )
    {
        return WICED_ERROR;
    }

    buffer[ 2 ] = ( blk_num & 0xFF00 ) >> 8;
    buffer[ 3 ] = blk_num & 0xFF;

    buffer += 4;

    memcpy( buffer, data, size );
    buffer += size;
    *p_buffer = buffer;

    return WICED_SUCCESS;
}

static wiced_result_t tftp_data_send_packet( tftp_connection_t *conn, uint8_t* data, uint16_t size )
{
    uint8_t*        tx_data;
    uint8_t*        buffer;
    uint16_t        available_data_length;
    wiced_packet_t* packet;

    if ( wiced_packet_create_udp( &conn->socket, TFTP_DATA_PACKET_LENGTH, &packet, (uint8_t**) &tx_data, &available_data_length ) != WICED_SUCCESS )
    {
        goto ERROR;
    }

    buffer = tx_data;
    tftp_data_create_packet( &buffer, conn->blk_num, data, size );

    wiced_packet_set_data_end( packet, buffer );

    if ( wiced_udp_send( &conn->socket, &conn->ip, conn->tid, packet ) != WICED_SUCCESS )
    {
        goto PACKET_ERROR;
    }
    return WICED_SUCCESS;

PACKET_ERROR:
    if(NULL!=packet)
        wiced_packet_delete( packet );
ERROR:
    return WICED_ERROR;
}

static wiced_result_t tftp_ack_parse_packet( uint8_t* buffer, uint16_t *blk_num )
{
    if ( ( buffer[ 0 ] != 0 ) || ( buffer[ 1 ] != TFTP_OPCODE_ACK ) )
    {
        return WICED_ERROR;
    }

    ( *blk_num ) = ( buffer[ 2 ] << 8 ) + buffer[ 3 ];

    return WICED_SUCCESS;
}

static wiced_result_t tftp_ack_create_packet( uint8_t **p_buffer, uint16_t blk_num )
{
    uint8_t *buffer = *p_buffer;

    /* Set Opcode */
    buffer[ 0 ] = 0;
    buffer[ 1 ] = TFTP_OPCODE_ACK;

    /* Copy filename */
    buffer[ 2 ] = ( blk_num & 0xFF00 ) >> 8;
    buffer[ 3 ] = ( blk_num & 0xFF );

    *p_buffer = buffer + 4;
    return WICED_SUCCESS;
}

static wiced_result_t tftp_ack_send_packet( tftp_connection_t *conn )
{
    uint8_t*        tx_data;
    uint8_t*        buffer;
    uint16_t        available_data_length;
    wiced_packet_t* packet;

    if ( wiced_packet_create_udp( &conn->socket, TFTP_DATA_PACKET_LENGTH, &packet, (uint8_t**) &tx_data, &available_data_length ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    buffer = tx_data;
    tftp_ack_create_packet( &buffer, conn->blk_num );
    wiced_packet_set_data_end( packet, buffer );

    if ( wiced_udp_send( &conn->socket, &conn->ip, conn->tid, packet ) != WICED_SUCCESS )
    {
        if(NULL!=packet)
            wiced_packet_delete( packet );

        return WICED_ERROR;
    }
    return WICED_SUCCESS;
}

static wiced_result_t tftp_oack_parse_packet( uint8_t* buffer, uint16_t size, char** timeout, char** blk_size, char** t_size )
{
    char*   end = (char*)buffer + size;
    char*   str;
    *timeout    = NULL;
    *blk_size   = NULL;
    *t_size     = NULL;

    if ( ( buffer[ 0 ] != 0 ) || ( buffer[ 1 ] != TFTP_OPCODE_OACK ) )
    {
        return WICED_ERROR;
    }

    buffer += 2;
    str = (char*)buffer;
    while ( str < end )
    {
        if ( strcmp( str, TFTP_OPTION_TRANSFER_SIZE_STR ) == 0 )
        {
            str += strlen( str ) + 1;
            *t_size = str;
        }
        if ( strcmp( str, TFTP_OPTION_BLK_SIZE_STR ) == 0 )
        {
            str += strlen( str ) + 1;
            *blk_size = str;
        }
        if ( strcmp( str, TFTP_OPTION_TIMEOUT_STR ) == 0 )
        {
            str += strlen( str ) + 1;
            *timeout = str;
        }
        str += strlen( str ) + 1;
    }
    return WICED_SUCCESS;
}

static wiced_result_t tftp_oack_create_packet( uint8_t **p_buffer, const char* timeout, const char* blk_size, const char* t_size )
{
    uint8_t *buffer = *p_buffer;

    /* Set Opcode */
    buffer[ 0 ] = 0;
    buffer[ 1 ] = TFTP_OPCODE_OACK;
    buffer     += 2;

    if ( t_size != NULL )
    {
        /* add timeout option */
        tftp_buffer_add_string( (char**) &buffer, TFTP_OPTION_TRANSFER_SIZE_STR );
        tftp_buffer_add_string( (char**) &buffer, t_size );
    }

    if ( timeout != NULL )
    {
        /* add timeout option */
        tftp_buffer_add_string( (char**) &buffer, TFTP_OPTION_TIMEOUT_STR );
        tftp_buffer_add_string( (char**) &buffer, timeout );
    }

    if ( blk_size != NULL )
    {
        /* add timeout option */
        tftp_buffer_add_string( (char**) &buffer, TFTP_OPTION_BLK_SIZE_STR );
        tftp_buffer_add_string( (char**) &buffer, blk_size );
    }

    *p_buffer = buffer;

    return WICED_SUCCESS;
}

static wiced_result_t tftp_oack_send_packet( tftp_connection_t* conn, const char* timeout, const char* blk_size, const char* t_size )
{
    uint8_t*        tx_data;
    uint8_t*        buffer;
    uint16_t        available_data_length;
    wiced_packet_t* packet;

    if ( wiced_packet_create_udp( &conn->socket, TFTP_DATA_PACKET_LENGTH, &packet, &tx_data, &available_data_length ) != WICED_SUCCESS )
    {
        goto ERROR;
    }

    buffer = tx_data;
    tftp_oack_create_packet( &buffer, timeout, blk_size, t_size );
    wiced_packet_set_data_end( packet, buffer );

    if ( wiced_udp_send( &conn->socket, &conn->ip, conn->tid, packet ) != WICED_SUCCESS )
    {
        goto PACKET_ERROR;
    }
    return WICED_SUCCESS;

PACKET_ERROR:
    if(NULL!=packet)
        wiced_packet_delete( packet );
ERROR:
    return WICED_ERROR;
}

static wiced_result_t tftp_oack( tftp_connection_t* conn, tftp_state_t* state )
{
    char*           blk_size;
    char*           timeout;
    char*           t_size;
    uint8_t*        buffer;
    uint16_t        rx_data_length;
    uint16_t        available_data_length;
    wiced_packet_t* packet;
    wiced_result_t  result;

    result = wiced_udp_receive( &conn->socket, &packet, conn->tftp.timeout * 1000 );

    switch ( result )
    {
        case WICED_SUCCESS:
            /* Get info about the received UDP packet */
            wiced_udp_packet_get_info( packet, &conn->ip, &conn->tid );

            /* Extract the received data from the UDP packet */
            wiced_packet_get_data( packet, 0, (uint8_t**) &buffer, &rx_data_length, &available_data_length );
            if ( rx_data_length < available_data_length )
            {
                /* Fragmented packets aren't supported */
                goto PACKET_ERROR;
            }

            /* Verify RRQ options */
            if ( tftp_oack_parse_packet( buffer, rx_data_length, &timeout, &blk_size, &t_size ) != WICED_SUCCESS )
            {
                /* For the case where no OACK packet is responded
                 * - To simpify the handling, the data packet will be dropped
                 *   Server will re-send it as wrong ack number is received.
                 */
                if ( ( buffer[ 0 ] == 0 ) && ( buffer[ 1 ] == TFTP_OPCODE_DATA ) )
                {
                   goto PACKET_DATA;
                }
                goto PACKET_ERROR;
            }

            if ( ( blk_size != NULL ) && ( conn->tftp.block_size != atoi( blk_size ) ) )
            {
                goto PACKET_ERROR;
            }

            if ( ( timeout != NULL ) && ( conn->tftp.timeout != atoi( timeout ) ) )
            {
                goto PACKET_ERROR;
            }

            if ( ( t_size == NULL ) && ( conn->tftp.request == TFTP_GET ) )
            {
                goto PACKET_ERROR;
            }

            conn->tftp.transfer_size = atoi( t_size );

PACKET_DATA:
            /* Inform user */
            if ( conn->callback.tftp_establish != NULL )
            {
                conn->callback.tftp_establish( &conn->tftp, conn->p_user );
            }

            if(NULL!=packet)
                wiced_packet_delete( packet );

            conn->blk_num = 0;
            if ( conn->tftp.request == TFTP_GET )
            {
                if ( tftp_ack_send_packet( conn ) != WICED_SUCCESS )
                {
                    goto SOCKET_ERROR;
                }
                *state = TFTP_STATE_GET;
                conn->blk_num = 1;
            }
            else
            {
                *state = TFTP_STATE_PUT;
            }
            conn->retry = 0;
            break;
        case WICED_TIMEOUT:
            /* Resend request */
            *state = TFTP_STATE_REQUEST;
            conn->retry++ ;
            if ( conn->retry >= TFTP_MAX_TIMEOUT_COUNT )
            {
                goto SOCKET_ERROR;
            }
            break;
        case WICED_ERROR:
        default:
            goto SOCKET_ERROR;
    }
    return WICED_SUCCESS;

PACKET_ERROR:
    if(NULL!=packet)
        wiced_packet_delete( packet );
SOCKET_ERROR:
    wiced_udp_delete_socket( &conn->socket );
    return WICED_ERROR;
}

static wiced_result_t tftp_request_parse_packet(uint8_t* buffer, uint16_t size, tftp_request_t* request, const char** filename, const char** timeout, const char** blk_size, const char** t_size )
{
    const char* str;
    const char* end = (const char*)buffer + size;

    *timeout    = NULL;
    *blk_size   = NULL;
    *t_size     = NULL;

    if ( ( buffer[ 0 ] != 0 ) || ( buffer[ 1 ] == TFTP_OPCODE_RRQ ) )
    {
        *request = TFTP_GET;
    }
    else if ( ( buffer[ 0 ] != 0 ) || ( buffer[ 1 ] == TFTP_OPCODE_WRQ ) )
    {
        *request = TFTP_PUT;
    }
    else
    {
        return WICED_ERROR;
    }

    str = (const char *)buffer + 2;
    *filename = str;
    /* Skip octet for now */
    str += strlen(str) + 1;
    str += strlen(str) + 1;


    while ( str < end )
    {
        if ( strcmp( str, TFTP_OPTION_TIMEOUT_STR ) == 0)
        {
            str += strlen( str ) + 1;
            *timeout = str;
            str += strlen( str ) + 1;
            continue;
        }
        if ( strcmp( str, TFTP_OPTION_TRANSFER_SIZE_STR ) == 0)
        {
            str += strlen( str ) + 1;
            *t_size = str;
            str += strlen( str ) + 1;
            continue;
        }
        if ( strcmp( str, TFTP_OPTION_BLK_SIZE_STR ) == 0)
        {
            str += strlen( str ) + 1;
            *blk_size = str;
            str += strlen( str ) + 1;
            continue;
        }
        str += strlen(str) + 1;
    }


    /* tftp_parse_options */
    return WICED_SUCCESS;
}

static wiced_result_t tftp_request_create_packet( uint8_t** p_buffer, tftp_request_t request, const char* filename, const char* mode, const char* timeout, const char* blk_size, const char* t_size )
{
    uint8_t *buffer = *p_buffer;

    /* Set Opcode */
    buffer[ 0 ] = 0;

    if ( request == TFTP_GET )
    {
        buffer[ 1 ] = TFTP_OPCODE_RRQ;
    }
    else
    {
        buffer[ 1 ] = TFTP_OPCODE_WRQ;
    }
    buffer += 2;

    /* Copy filename */
    tftp_buffer_add_string( (char**) &buffer, filename );

    /* Copy mode */
    tftp_buffer_add_string( (char**) &buffer, mode );

    /* add timeout option */
    if ( timeout != NULL )
    {
        tftp_buffer_add_string( (char**) &buffer, TFTP_OPTION_TIMEOUT_STR );
        tftp_buffer_add_string( (char**) &buffer, timeout );
    }

    /* add transfer size option */
    if ( t_size != NULL )
    {
        tftp_buffer_add_string( (char**) &buffer, TFTP_OPTION_TRANSFER_SIZE_STR );
        tftp_buffer_add_string( (char**) &buffer, t_size );
    }

    /* add transfer size option */
    if ( blk_size != NULL )
    {
        tftp_buffer_add_string( (char**) &buffer, TFTP_OPTION_BLK_SIZE_STR );
        tftp_buffer_add_string( (char**) &buffer, blk_size );
    }

    *p_buffer = buffer;
    return WICED_SUCCESS;
}

static wiced_result_t tftp_request( tftp_connection_t* conn, tftp_state_t* state )
{
    uint8_t*        tx_data;
    uint8_t*        buffer;
    uint16_t        available_data_length;
    wiced_packet_t* packet;

    /* Get a random TID and use it as a port */
    conn->tid = tftp_get_tid( );

    /* Create the channel socket on the new port */
    if ( wiced_udp_create_socket( &conn->socket, conn->tid, conn->interface ) != WICED_SUCCESS )
    {
        goto ERROR;
    }

    if ( wiced_packet_create_udp( &conn->socket, TFTP_DATA_PACKET_LENGTH, &packet, &tx_data, &available_data_length ) != WICED_SUCCESS )
    {
        goto SOCKET_ERROR;
    }
    conn->tftp.timeout      = TFTP_OPTION_TIMEOUT_VAL;
    conn->tftp.block_size   = TFTP_OPTION_BLK_SIZE_VAL;
    buffer = tx_data;
    tftp_request_create_packet( &buffer, conn->tftp.request, conn->tftp.filename, tftp_mode_str[conn->tftp.mode], STR(TFTP_OPTION_TIMEOUT_VAL), STR(TFTP_OPTION_BLK_SIZE_VAL), STR(TFTP_OPTION_TRANSFER_SIZE_VAL) );
    wiced_packet_set_data_end( packet, buffer );

    if ( wiced_udp_send( &conn->socket, &conn->ip, TFTP_SERVER_PORT, packet ) != WICED_SUCCESS )
    {
        goto PACKET_ERROR;
    }

    *state = TFTP_STATE_OACK;

    return WICED_SUCCESS;

PACKET_ERROR:
    if(NULL!=packet)
        wiced_packet_delete( packet );
SOCKET_ERROR:
    wiced_udp_delete_socket( &conn->socket );
ERROR:
    return WICED_ERROR;
}


wiced_result_t tftp_listen( tftp_connection_t* conn, tftp_state_t* state )
{
    uint8_t         status   = TFTP_NO_ERROR;
    uint16_t        rx_data_length;
    uint16_t        available_data_length;
    uint8_t*        buffer;
    const char*     timeout  = NULL;
    const char*     t_size   = NULL;
    const char*     blk_size = NULL;
    wiced_packet_t* packet;

    if ( wiced_udp_create_socket( &conn->socket, TFTP_SERVER_PORT, conn->interface ) != WICED_SUCCESS )
    {
        goto ERROR;
    }

    if ( wiced_udp_receive( &conn->socket, &packet, WICED_NEVER_TIMEOUT ) != WICED_SUCCESS)
    {
        goto SOCKET_ERROR;
    }

    /* Get info about the received UDP packet */
    wiced_udp_packet_get_info( packet, &conn->ip, &conn->tid );

    /* Extract the received data from the UDP packet */
    wiced_packet_get_data( packet, 0, (uint8_t**) &buffer, &rx_data_length, &available_data_length );
    if ( rx_data_length < available_data_length )
    {
        /* Fragmented packets aren't supported */
        goto PACKET_ERROR;
    }

    /* Verify RRQ options */
    if ( tftp_request_parse_packet( buffer, rx_data_length, &conn->tftp.request, &conn->tftp.filename, &timeout, &blk_size, &t_size ) != WICED_SUCCESS )
    {
        goto PACKET_ERROR;
    }

    conn->tftp.timeout = TFTP_OPTION_TIMEOUT_VAL;
    if ( timeout != NULL)
    {
        /* Timeout option was request by user, respond with default */
        timeout = STR(TFTP_OPTION_TIMEOUT_VAL);
    }

    conn->tftp.block_size = TFTP_OPTION_BLK_SIZE_VAL;
    if ( blk_size != NULL)
    {
        /* Block size option was request by user, respond with default */
        blk_size = STR(TFTP_OPTION_BLK_SIZE_VAL);
    }

    if ( ( t_size == NULL ) && ( conn->tftp.request == TFTP_OPCODE_WRQ ))
    {
        /* WICED requires the file size ahead */
        goto PACKET_ERROR;
    }
    conn->tftp.transfer_size = atoi(t_size);

    /* As we are the server, we flip requests, a GET from the user is a PUT for us. */
    if (conn->tftp.request == TFTP_GET)
    {
        conn->tftp.request = TFTP_PUT;
    }
    else
    {
        conn->tftp.request = TFTP_GET;
    }


    /* Inform user */
    if ( conn->callback.tftp_establish( &conn->tftp, conn->p_user ) != WICED_SUCCESS )
    {
        status = TFTP_ERROR_ACCESS_VIOLATION;
    }

    /* Close this socket and open a new one for the new port (host TID) */
    if(NULL!=packet)
        wiced_packet_delete( packet );

    wiced_udp_delete_socket( &conn->socket );

    /* Create the channel socket on the new port */
    if ( wiced_udp_create_socket( &conn->socket, conn->tid, conn->interface ) != WICED_SUCCESS )
    {
        goto ERROR;
    }

    if ( status == TFTP_NO_ERROR )
    {
        conn->retry = 0;
        if (conn->tftp.request == TFTP_GET)
        {
            conn->blk_num = 1;
            *state = TFTP_STATE_GET;
        }
        else
        {
            conn->blk_num = 0;
            *state = TFTP_STATE_PUT;
        }

        if ( tftp_oack_send_packet( conn, timeout, blk_size, t_size ) != WICED_SUCCESS )
        {
            goto SOCKET_ERROR;
        }
    }
    else
    {
        /* Return to previous listen state */
        *state = TFTP_STATE_LISTEN;

        /* User not happy, drop request */
        if ( tftp_error_send_packet( conn, status, TFTP_ERROR_MESSAGE ) != WICED_SUCCESS )
        {
            goto SOCKET_ERROR;
        }
        wiced_udp_delete_socket( &conn->socket );
    }

    return WICED_SUCCESS;

PACKET_ERROR:
    if(NULL!=packet)
        wiced_packet_delete( packet );
SOCKET_ERROR:
    wiced_udp_delete_socket( &conn->socket );
ERROR:
    if ( *state != TFTP_STATE_LISTEN )
    {
        /* connection already informed to user, need to inform for closing */
        conn->callback.tftp_close( &conn->tftp, status, conn->p_user );
    }
    return WICED_ERROR;
}

wiced_result_t tftp_write( tftp_connection_t* conn, tftp_state_t* state )
{
    uint8_t             status          = TFTP_ERROR_NOT_DEFINED;
    uint16_t            rx_data_length;
    uint16_t            available_data_length;
    uint16_t            port;
    uint16_t            blk_num;
    uint8_t*            data;
    uint8_t*            buffer;
    wiced_ip_address_t  ip;
    wiced_result_t      result;
    wiced_packet_t*     packet;

    result = wiced_udp_receive( &conn->socket, &packet, conn->tftp.timeout * 1000 );

    switch ( result )
    {
        case WICED_SUCCESS:
            /* Get info about the received UDP packet */
            wiced_udp_packet_get_info( packet, &ip, &port );

            if ( port != conn->tid )
            {
                goto PACKET_ERROR;
            }

            /* Extract the received data from the UDP packet */
            wiced_packet_get_data( packet, 0, (uint8_t**) &buffer, &rx_data_length, &available_data_length );
            if ( rx_data_length < available_data_length )
            {
                /* Fragmented packets aren't supported */
                goto PACKET_ERROR;
            }

            /* Read data packet */
            if ( tftp_data_parse_packet( buffer, rx_data_length, &blk_num, &data, &conn->tftp.block_size ) != WICED_SUCCESS )
            {
                goto PACKET_ERROR;
            }

            if ( blk_num == conn->blk_num )
            {
                conn->retry = 0;
                /* Inform user */
                if ( conn->callback.tftp_write != NULL )
                {
                    if ( conn->callback.tftp_write( &conn->tftp, data, conn->p_user ) != WICED_SUCCESS )
                    {
                        /* User not happy, drop request */
                        goto PACKET_ERROR;
                    }
                }

            }
            else if ( blk_num == conn->blk_num - 1 )
            {
                conn->blk_num--;
                conn->retry++;
                if ( conn->retry >= TFTP_MAX_TIMEOUT_COUNT )
                {
                    goto SOCKET_ERROR;
                }
            }
            else
            {
                goto PACKET_ERROR;
            }

            if(NULL!=packet)
                wiced_packet_delete( packet );

            break;
        case WICED_TIMEOUT:
            /* resend last block */
            conn->blk_num--;
            conn->retry++;
            if ( conn->retry >= TFTP_MAX_TIMEOUT_COUNT )
            {
                goto SOCKET_ERROR;
            }
            break;
        case WICED_ERROR:
        default:
            goto SOCKET_ERROR;
    }

    /* Close this socket and open a new one for the new port (host TID) */
    if(NULL!=packet)
        wiced_packet_delete( packet );

    if ( tftp_ack_send_packet( conn ) != WICED_SUCCESS )
    {
        goto SOCKET_ERROR;
    }

    if ( conn->tftp.block_size != TFTP_OPTION_BLK_SIZE_VAL )
    {
        status = TFTP_NO_ERROR;
        conn->callback.tftp_close( &conn->tftp, status, conn->p_user );
        wiced_udp_delete_socket( &conn->socket );
        *state = TFTP_STATE_LISTEN;
    }
    else
    {
        conn->blk_num++;
    }
    return WICED_SUCCESS;

PACKET_ERROR:
    if(NULL!=packet)
        wiced_packet_delete( packet );
SOCKET_ERROR:
    wiced_udp_delete_socket( &conn->socket );
    conn->callback.tftp_close( &conn->tftp, status, conn->p_user );
    return WICED_ERROR;
}


wiced_result_t tftp_read( tftp_connection_t* conn, tftp_state_t* state )
{
    uint8_t             status          = TFTP_ERROR_NOT_DEFINED;
    uint8_t             read_buff_fill  = 1;
    uint8_t             read_buff[ TFTP_OPTION_BLK_SIZE_VAL ];
    uint16_t            rx_data_length;
    uint16_t            available_data_length;
    uint16_t            port;
    uint16_t            blk_num;
    uint8_t*            buffer;
    wiced_ip_address_t  ip;
    wiced_result_t      result;
    wiced_packet_t*     packet;
    if ( conn->blk_num != 0)
    {
        result = wiced_udp_receive( &conn->socket, &packet, conn->tftp.timeout * 1000 );

        switch ( result )
        {
            case WICED_SUCCESS:
                /* Get info about the received UDP packet */
                wiced_udp_packet_get_info( packet, &ip, &port );

                if ( port != conn->tid )
                {
                    goto PACKET_ERROR;
                }

                /* Extract the received data from the UDP packet */
                wiced_packet_get_data( packet, 0, (uint8_t**) &buffer, &rx_data_length, &available_data_length );
                if ( rx_data_length < available_data_length )
                {
                    /* Fragmented packets aren't supported */
                    goto PACKET_ERROR;
                }

                /* Read data packet */
                if ( tftp_ack_parse_packet( buffer, &blk_num ) != WICED_SUCCESS )
                {
                    goto PACKET_ERROR;
                }
                if ( blk_num == conn->blk_num )
                {
                    conn->retry = 0;
                    /* Request new data from user */
                    read_buff_fill = 1;
                }
                else
                {
                    read_buff_fill = 0;
                    if ( blk_num == conn->blk_num - 1 )
                    {
                        conn->blk_num--;
                        conn->retry++;
                        if ( conn->retry >= TFTP_MAX_TIMEOUT_COUNT )
                        {
                            goto PACKET_ERROR;
                        }
                    }
                    else
                    {
                        goto PACKET_ERROR;
                    }
                }
                break;
            case WICED_TIMEOUT:
                /* make sure we do not request a new fill */
                /* but instead we use the current payload */
                read_buff_fill=0;

                /* resend last block */
                conn->blk_num--;
                conn->retry++;
                if ( conn->retry >= TFTP_MAX_TIMEOUT_COUNT )
                {
                    goto SOCKET_ERROR;
                }
                break;
            case WICED_ERROR:
            default:
                goto SOCKET_ERROR;
        }

        /* Close this socket and open a new one for the new port (host TID) */
        if(NULL!=packet)
            wiced_packet_delete( packet );
    }
    conn->blk_num++;
    if ( read_buff_fill != 0 )
    {
        if ( conn->callback.tftp_read(&conn->tftp, read_buff, conn->p_user) != WICED_SUCCESS )
        {
                goto SOCKET_ERROR;
        }
    }
    if ( tftp_data_send_packet( conn, read_buff, conn->tftp.block_size ) != WICED_SUCCESS )
    {
        goto SOCKET_ERROR;
    }

    if ( conn->tftp.block_size != TFTP_OPTION_BLK_SIZE_VAL )
    {
        status = TFTP_NO_ERROR;
        conn->callback.tftp_close( &conn->tftp, status, conn->p_user );
        wiced_udp_delete_socket( &conn->socket );
        *state = TFTP_STATE_LISTEN;
    }

    return WICED_SUCCESS;

PACKET_ERROR:
    if(NULL!=packet)
        wiced_packet_delete( packet );
SOCKET_ERROR:
    wiced_udp_delete_socket( &conn->socket );
    conn->callback.tftp_close( &conn->tftp, status, conn->p_user );
    return WICED_ERROR;
}

/*
 * This thread is responsible for creating new sessions for clients
 */
void tftp_server_thread( uint32_t arg )
{
    uint8_t             tftp_complete   = WICED_FALSE;
    tftp_connection_t*  server          = (tftp_connection_t*) arg;
    tftp_state_t        state           = TFTP_STATE_LISTEN;

    server->tftp_complete = WICED_FALSE;

    while ( tftp_complete != WICED_TRUE )
    {
        switch ( state )
        {
            case TFTP_STATE_LISTEN:
                if ( tftp_listen( server, &state ) != WICED_SUCCESS )
                {
                    tftp_complete = 1;
                }
            break;
            case TFTP_STATE_GET:
                if ( tftp_write( server, &state ) != WICED_SUCCESS )
                {
                    tftp_complete = 1;
                }
                break;
            case TFTP_STATE_PUT:
                if ( tftp_read( server, &state ) != WICED_SUCCESS )
                {
                    tftp_complete = 1;
                }
                break;
            default:
            break;
        }
    }

    server->tftp_complete = WICED_TRUE;

    WICED_END_OF_CURRENT_THREAD( );
}

/*
 * This thread is responsible for receiving/transmiting data from tftp sever
 */
void tftp_client_thread( uint32_t arg )
{
    uint8_t             tftp_complete   = WICED_FALSE;
    tftp_state_t        state           = TFTP_STATE_REQUEST;
    tftp_connection_t*  client          = (tftp_connection_t*) arg;

    client->tftp_complete = WICED_FALSE;

    while ( tftp_complete != WICED_TRUE )
    {
        switch ( state )
        {
            case TFTP_STATE_REQUEST:
                /* send initial read request with options */
                if ( tftp_request( client, &state ) != WICED_SUCCESS )
                {
                    tftp_complete = WICED_TRUE;
                }
                break;
            case TFTP_STATE_OACK:
                if ( tftp_oack( client, &state ) != WICED_SUCCESS )
                {
                    tftp_complete = 1;
                }
                break;
            case TFTP_STATE_GET:
                if ( tftp_write( client, &state ) != WICED_SUCCESS )
                {
                    tftp_complete = 1;
                }
                break;
            case TFTP_STATE_PUT:
                if ( tftp_read( client, &state ) != WICED_SUCCESS )
                {
                    tftp_complete = 1;
                }
                break;
            case TFTP_STATE_LISTEN:
                tftp_complete = 1;
                break;
            default:
                break;
        }
    }

    client->tftp_complete = WICED_TRUE;

    WICED_END_OF_CURRENT_THREAD( );
}


wiced_result_t tftp_server_start( tftp_connection_t* server, wiced_interface_t interface, tftp_callback_t* callback, void* p_user )
{
    server->callback    = *callback;
    server->p_user      = p_user;
    server->interface   = interface;

    /* Create server thread and return */
    return wiced_rtos_create_thread( &server->thread, WICED_DEFAULT_LIBRARY_PRIORITY, "TFTP_SERVER", tftp_server_thread, TFTP_THREAD_STACK_SIZE, server );
}

wiced_result_t tftp_server_stop( tftp_connection_t* server )
{
    /* Kill Gedday thread */
    wiced_rtos_thread_force_awake( &server->thread );
    wiced_rtos_thread_join       ( &server->thread );
    wiced_rtos_delete_thread     ( &server->thread );
    return WICED_SUCCESS;
}



wiced_result_t tftp_client_get( tftp_connection_t* client, wiced_ip_address_t host, wiced_interface_t interface, const char * filename, tftp_mode_t mode, tftp_callback_t *callback, void *p_user )
{
    wiced_result_t result;

    /* Initialize client */
    client->callback        = *callback;
    client->ip              = host;
    client->interface       = interface;
    client->p_user          = p_user;
    client->tftp.filename       = filename;
    client->tftp.transfer_size  = 0;
    client->tftp.mode           = mode;
    client->tftp.request        = TFTP_GET;
    client->retry               = 0;


    /* Create server thread and wait for complete */
    result = wiced_rtos_create_thread( &client->thread, WICED_DEFAULT_LIBRARY_PRIORITY, "TFTP_CLIENT", tftp_client_thread, TFTP_THREAD_STACK_SIZE, client );
    if (result == WICED_SUCCESS)
    {
        result = wiced_rtos_thread_join(&client->thread);
        wiced_rtos_delete_thread(&client->thread);
    }
    return result;
}

wiced_result_t tftp_client_put( tftp_connection_t* client, wiced_ip_address_t host, wiced_interface_t interface, const char * filename, tftp_mode_t mode, tftp_callback_t *callback, void *p_user )
{
    wiced_result_t result;

    /* Initialize client */
    client->callback        = *callback;
    client->ip              = host;
    client->interface       = interface;
    client->p_user          = p_user;
    client->tftp.filename       = filename;
    client->tftp.transfer_size  = 400*1024;
    client->tftp.mode           = mode;
    client->tftp.request        = TFTP_PUT;
    client->retry               = 0;


    /* Create server thread and wait for complete */
    result = wiced_rtos_create_thread( &client->thread, WICED_DEFAULT_LIBRARY_PRIORITY, "TFTP_CLIENT", tftp_client_thread, TFTP_THREAD_STACK_SIZE, client );
    if (result == WICED_SUCCESS)
    {
        result = wiced_rtos_thread_join(&client->thread);
        wiced_rtos_delete_thread(&client->thread);
    }
    return result;
}
