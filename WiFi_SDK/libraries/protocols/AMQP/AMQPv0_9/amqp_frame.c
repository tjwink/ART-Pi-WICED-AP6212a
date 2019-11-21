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

/******************************************************
 *                      Macros
 ******************************************************/

/* Note: pointers are not guaranteed to be aligned, so we can't just assign
 * them to htonx values.
 */

/*                      *
 *  BUFFER PUT MACROS   *
 *                      */
#define AMQP_BUFFER_PUT_OCTET( BUFFER, VALUE )   \
do                                               \
{                                                \
    uint8_t *data_po = (BUFFER)->data;           \
    *(data_po)   = (VALUE);                      \
    ((BUFFER)->data)++;                          \
}while(0)

#define AMQP_BUFFER_PUT_SHORT( BUFFER, VALUE )                 \
do                                                             \
{                                                              \
    uint8_t *data_ps = (BUFFER)->data;                         \
    *(data_ps)     = (uint8_t)(((VALUE) & 0x0000FF00) >> 8);   \
    *(data_ps + 1) = (uint8_t) ((VALUE) & 0x000000FF);         \
    ((BUFFER)->data) += 2;                                     \
}while(0)

#define AMQP_BUFFER_PUT_LONG( BUFFER, VALUE )                  \
do                                                             \
{                                                              \
    uint8_t *data_pl = (BUFFER)->data;                         \
    *(data_pl)     = (uint8_t)(((VALUE) & 0xFF000000) >> 24);  \
    *(data_pl + 1) = (uint8_t)(((VALUE) & 0x00FF0000) >> 16);  \
    *(data_pl + 2) = (uint8_t)(((VALUE) & 0x0000FF00) >> 8 );  \
    *(data_pl + 3) = (uint8_t) ((VALUE) & 0x000000FF);         \
    ((BUFFER)->data) += 4;                                     \
}while(0)

#define AMQP_BUFFER_PUT_LONG_LONG( BUFFER, VALUE )                       \
do                                                                       \
{                                                                        \
    uint8_t *data_pll = (BUFFER)->data;                                  \
    *(data_pll)     = (uint8_t)(((VALUE) & 0xFF00000000000000) >> 56);   \
    *(data_pll + 1) = (uint8_t)(((VALUE) & 0x00FF000000000000) >> 48);   \
    *(data_pll + 2) = (uint8_t)(((VALUE) & 0x0000FF0000000000) >> 40);   \
    *(data_pll + 3) = (uint8_t)(((VALUE) & 0x000000FF00000000) >> 32);   \
    *(data_pll + 4) = (uint8_t)(((VALUE) & 0x00000000FF000000) >> 24);   \
    *(data_pll + 5) = (uint8_t)(((VALUE) & 0x0000000000FF0000) >> 16);   \
    *(data_pll + 6) = (uint8_t)(((VALUE) & 0x000000000000FF00) >> 8 );   \
    *(data_pll + 7) = (uint8_t) ((VALUE) & 0x00000000000000FF);          \
    ((BUFFER)->data) += 8;                                               \
}while(0)

#define AMQP_BUFFER_PUT_SHORT_STRING( BUFFER, STRING, LEN) \
do                                                         \
{                                                          \
    uint8_t *data_pss = (BUFFER)->data;                    \
    *(data_pss) = (LEN);                                   \
    memcpy( data_pss + 1, (STRING), (LEN));                \
    ((BUFFER)->data) += (LEN) + 1;                         \
}while(0)

#define AMQP_BUFFER_PUT_LONG_STRING( BUFFER, STRING, LEN)  \
do                                                         \
{                                                          \
    uint8_t *data_pls = (BUFFER)->data;                    \
    AMQP_BUFFER_PUT_LONG( (BUFFER), (LEN) );               \
    memcpy( (data_pls) + 4, (STRING), (LEN));              \
    ((BUFFER)->data) += (LEN);                             \
}while(0)

#define AMQP_BUFFER_PUT_BIT( BUFFER, VALUE, BIT, INC)                                   \
do                                                                                      \
{                                                                                       \
    uint8_t  mask_pb = ( 1 << (BIT));                                                   \
    uint8_t *data_pb = (BUFFER)->data;                                                  \
    (*data_pb) = (uint8_t)( (VALUE) ? (*data_pb) | mask_pb : (*data_pb) & (~mask_pb) ); \
    if (INC) { ((BUFFER)->data) += 1; }                                                 \
}while(0)

/*                      *
 *  BUFFER GET MACROS   *
 *                      */
#define AMQP_BUFFER_GET_OCTET( BUFFER, VALUE )   \
do                                               \
{                                                \
    uint8_t *data_go = (BUFFER)->data;           \
    (VALUE) = *(data_go);                        \
    ((BUFFER)->data)++;                          \
}while(0)

#define AMQP_BUFFER_GET_SHORT( BUFFER, VALUE )        \
do                                                    \
{                                                     \
    uint8_t *data_gs = (BUFFER)->data;                \
    VALUE = (uint16_t)(         (*(data_gs    ) << 8)); \
    VALUE = (uint16_t)( VALUE + (*(data_gs + 1)     )); \
    ((BUFFER)->data) += 2;                            \
}while(0)

#define AMQP_BUFFER_GET_LONG( BUFFER, VALUE )               \
do                                                          \
{                                                           \
    uint8_t *data_gl = (BUFFER)->data;                      \
    (VALUE) = (uint32_t) (                    (((uint32_t)*(data_gl    )) << 24));    \
    (VALUE) = (uint32_t) ((uint32_t)(VALUE) + (((uint32_t)*(data_gl + 1)) << 16));  \
    (VALUE) = (uint32_t) ((uint32_t)(VALUE) + (((uint32_t)*(data_gl + 2)) << 8 ));  \
    (VALUE) = (uint32_t) ((uint32_t)(VALUE) + (((uint32_t)*(data_gl + 3))      ));         \
    ((BUFFER)->data) += 4;                                  \
}while(0)

#define AMQP_BUFFER_GET_LONG_LONG( BUFFER, VALUE )                     \
do                                                                     \
{                                                                      \
    uint8_t *data_gll = (BUFFER)->data;                                \
    (VALUE) = (uint64_t) (                    (((uint64_t)*(data_gll    )) << 56)); \
    (VALUE) = (uint64_t) ((uint64_t)(VALUE) + (((uint64_t)*(data_gll + 1)) << 48)); \
    (VALUE) = (uint64_t) ((uint64_t)(VALUE) + (((uint64_t)*(data_gll + 2)) << 40)); \
    (VALUE) = (uint64_t) ((uint64_t)(VALUE) + (((uint64_t)*(data_gll + 3)) << 32)); \
    (VALUE) = (uint64_t) ((uint64_t)(VALUE) + (((uint64_t)*(data_gll + 4)) << 24)); \
    (VALUE) = (uint64_t) ((uint64_t)(VALUE) + (((uint64_t)*(data_gll + 5)) << 16)); \
    (VALUE) = (uint64_t) ((uint64_t)(VALUE) + (((uint64_t)*(data_gll + 6)) << 8 )); \
    (VALUE) = (uint64_t) ((uint64_t)(VALUE) + (((uint64_t)*(data_gll + 7))      ));       \
    ((BUFFER)->data) += 8;                                             \
}while(0)

#define AMQP_BUFFER_GET_SHORT_STRING( BUFFER, STRING, LEN) \
do                                                         \
{                                                          \
    uint8_t *data_gss = (BUFFER)->data;                    \
    (LEN) = *(data_gss);                                   \
    (STRING) = (data_gss) + 1;                             \
    ((BUFFER)->data) += (LEN) + 1;                         \
}while(0)

#define AMQP_BUFFER_GET_LONG_STRING( BUFFER, STRING, LEN)  \
do                                                         \
{                                                          \
    AMQP_BUFFER_GET_LONG( (BUFFER), (LEN) );               \
    (STRING) = (BUFFER)->data;                             \
    (BUFFER)->data += (LEN);                               \
}while(0)

#define AMQP_BUFFER_GET_BIT( BUFFER, VALUE, BIT, INC)      \
do                                                         \
{                                                          \
    uint8_t  mask_gb = ( 1 << (BIT));                      \
    uint8_t *data_gb = (BUFFER)->data;                     \
    (VALUE) = *data_gb & mask_gb;                          \
    (VALUE) = (VALUE) ? 1 : 0;                             \
    if (INC) { ((BUFFER)->data) += 1; }                    \
}while(0)

/******************************************************
 *                    Constants
 ******************************************************/
#define AMQP_FRAME_DELIMITER            (0xCE)

#define AMQP_FRAME_HEADER_PROTOCOL_STR  (0x414D5150)


#define AMQP_FRAME_CONTENT_HEADER_FLAGS_CONTENT_TYPE_MASK       (1 << 15)
#define AMQP_FRAME_CONTENT_HEADER_FLAGS_CONTENT_ENCODING_MASK   (1 << 14)
#define AMQP_FRAME_CONTENT_HEADER_FLAGS_HEADERS_MASK            (1 << 13)
#define AMQP_FRAME_CONTENT_HEADER_FLAGS_DELIVERY_MODE_MASK      (1 << 12)
#define AMQP_FRAME_CONTENT_HEADER_FLAGS_PRIORITY_MASK           (1 << 11)
#define AMQP_FRAME_CONTENT_HEADER_FLAGS_CORRELATION_ID_MASK     (1 << 10)
#define AMQP_FRAME_CONTENT_HEADER_FLAGS_REPLY_TO_MASK           (1 << 9)
#define AMQP_FRAME_CONTENT_HEADER_FLAGS_EXPIRATION_MASK         (1 << 8)
#define AMQP_FRAME_CONTENT_HEADER_FLAGS_MESSAGE_ID_MASK         (1 << 7)
#define AMQP_FRAME_CONTENT_HEADER_FLAGS_TIMESTAMP_MASK          (1 << 6)
#define AMQP_FRAME_CONTENT_HEADER_FLAGS_TYPE_MASK               (1 << 5)
#define AMQP_FRAME_CONTENT_HEADER_FLAGS_USER_ID_MASK            (1 << 4)
#define AMQP_FRAME_CONTENT_HEADER_FLAGS_APP_ID_MASK             (1 << 3)


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
static
wiced_result_t  amqp_frame_put_field( wiced_amqp_field_t *field, wiced_amqp_frame_t *frame )
{
    if ( field->name.str != NULL )
    {
        AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, field->name.str, field->name.len );
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, field->type );
        frame->size = (uint16_t)( frame->size + field->name.len + (uint16_t) sizeof(field->name.len) + 1 );
        switch (field->type)
        {
            case 'B':
                AMQP_BUFFER_PUT_OCTET( &frame->buffer, field->value.B );
                frame->size++;
                break;
            case 'u':
                AMQP_BUFFER_PUT_SHORT( &frame->buffer, field->value.u );
                frame->size = (uint16_t)( frame->size + 2 );
                break;
            case 'i':
                AMQP_BUFFER_PUT_LONG( &frame->buffer, field->value.i );
                frame->size = (uint16_t)( frame->size + 4 );
                break;
            case 'l':
                AMQP_BUFFER_PUT_LONG_LONG( &frame->buffer, field->value.l );
                frame->size = (uint16_t)( frame->size + 8 );
                break;
            case 's':
                AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, field->value.s.str, field->value.s.len );
                frame->size = (uint16_t) ( frame->size + field->value.s.len + (uint16_t) sizeof(field->value.s.len) );
                break;
            case 'S':
                AMQP_BUFFER_PUT_LONG_STRING( &frame->buffer, field->value.S.str, field->value.S.len );
                frame->size = (uint16_t) ( frame->size + field->value.S.len + sizeof(field->value.S.len) );
                break;
            default:
                printf( "WICED FRAME TYPE NOT SUPPORTED\n" );
                wiced_assert( "WICED FRAME TYPE NOT SUPPORTED\n", 0 != 1 );
                break;
        }
    }
    return WICED_SUCCESS;
}

static
wiced_result_t  amqp_frame_get_field( wiced_amqp_field_t *field, wiced_amqp_frame_t *frame )
{
    AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, field->name.str, field->name.len );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, field->type );
    switch ( field->type )
    {
        case 'B':
            AMQP_BUFFER_GET_OCTET( &frame->buffer, field->value.B );
            break;
        case 'u':
            AMQP_BUFFER_GET_SHORT( &frame->buffer, field->value.u );
            break;
        case 'i':
            AMQP_BUFFER_GET_LONG( &frame->buffer, field->value.i );
            break;
        case 'l':
            AMQP_BUFFER_GET_LONG_LONG( &frame->buffer, field->value.l );
            break;
        case 's':
            AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, field->value.s.str, field->value.s.len );
            break;
        case 'S':
            AMQP_BUFFER_GET_LONG_STRING( &frame->buffer, field->value.S.str, field->value.S.len );
            break;
        case 'F':
            /* Another feild table, read this and drop it for now */
            {
            uint32_t size;
            printf("WARNING: AMQP Recursive field tables, dropping internal one.\n");
            AMQP_BUFFER_GET_LONG( &frame->buffer, size );
            frame->buffer.data += size;
            }
            break;
        default:
            printf( "WICED FRAME TYPE NOT SUPPORTED\n" );
            wiced_assert( "WICED FRAME TYPE NOT SUPPORTED\n", 0 != 1 );
            break;
    }
    return WICED_SUCCESS;
}

static
wiced_result_t  amqp_frame_get_table( wiced_amqp_field_t *field, wiced_amqp_frame_t *frame )
{
    uint8_t      field_count = 0;
    uint8_t     *end;
    uint32_t     size;

    /* Set size to 0 for now */
    AMQP_BUFFER_GET_LONG( &frame->buffer, size );
    end = frame->buffer.data + size;

    while ( (frame->buffer.data < end ) &&  ( field_count < AMQP_FRAME_MAX_TABLE_SIZE ))
    {
        amqp_frame_get_field( field, frame );
        field++;
        field_count++;
    }

    if (frame->buffer.data != end)
    {
        printf("AMQP: Some fields were dropped due to size limiations.\n");
    }
    /* Update final table size */
    frame->buffer.data = end;
    return WICED_SUCCESS;
}

static
wiced_result_t  amqp_frame_put_table( wiced_amqp_field_t *field, wiced_amqp_frame_t *frame )
{
    uint8_t     *end = frame->buffer.data;
    uint16_t     size;

    /* Set size to 0 for now */
    AMQP_BUFFER_PUT_LONG( &frame->buffer, 0 );
    frame->size = (uint16_t) ( frame->size + 4 );
    size = frame->size;

    if ( NULL != field )
    {
        while ( field->name.str != NULL )
        {
            amqp_frame_put_field( field, frame );
            field++;
        }
    }

    /* Update final table size */
    size = (uint16_t) ( frame->size - size );
    frame->buffer.data = end;
    AMQP_BUFFER_PUT_LONG( &frame->buffer, size );
    frame->buffer.data += size;

    return WICED_SUCCESS;
}

/******************************************************
 *               Interface functions
 ******************************************************/
static
wiced_result_t amqp_frame_recv_connection_handle ( uint16_t method, uint16_t channel, wiced_amqp_frame_t *frame, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    switch (method)
    {
        case AMQP_FRAME_METHOD_CONNECTION_START :
        {
            wiced_amqp_field_t                server_properties[ AMQP_FRAME_MAX_TABLE_SIZE ];
            wiced_amqp_connection_start_arg_t args;
            args.server_properties = server_properties;
            amqp_frame_get_connection_start( frame, &args );
            ret = amqp_connection_backend_get_start( channel, &args, conn );
        }
        break;
        case AMQP_FRAME_METHOD_CONNECTION_TUNE  :
        {
            wiced_amqp_connection_tune_arg_t  args;
            amqp_frame_get_connection_tune( frame, &args );
            ret = amqp_connection_backend_get_tune( channel, &args, conn );
        }
        break;
        case AMQP_FRAME_METHOD_CONNECTION_OPEN_OK  :
        {
            ret = amqp_connection_backend_get_open_ok( channel, conn );
        }
        break;
        case AMQP_FRAME_METHOD_CONNECTION_CLOSE    :
        {
            wiced_amqp_connection_close_arg_t  args;
            amqp_frame_get_connection_close( frame, &args );
            ret = amqp_connection_backend_get_close( channel, &args, conn );
        }
        break;
        case AMQP_FRAME_METHOD_CONNECTION_CLOSE_OK :
        {
            ret = amqp_connection_backend_get_close_ok( channel, conn );
        }
        break;
        case AMQP_FRAME_METHOD_CONNECTION_SECURE_OK :
        default:
            return WICED_ERROR;
    }
    return ret;
}

static
wiced_result_t amqp_frame_recv_channel_handle ( uint16_t method, uint16_t channel, wiced_amqp_frame_t *frame, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret = WICED_ERROR;
    switch ( method)
    {
        case AMQP_FRAME_METHOD_CHANNEL_OPEN_OK  :
        {
            ret = amqp_channel_backend_get_open_ok( channel, conn );
            break;
        }
        case AMQP_FRAME_METHOD_CHANNEL_CLOSE:
        {
            wiced_amqp_channel_close_arg_t   args;
            amqp_frame_get_channel_close( frame, &args );
            ret = amqp_channel_backend_get_close( channel, &args, conn );
            break;
        }
        case AMQP_FRAME_METHOD_CHANNEL_CLOSE_OK :
        {
            amqp_channel_backend_get_close_ok( channel, conn );
            break;
        }
        case AMQP_FRAME_METHOD_CHANNEL_FLOW     :
        {
            wiced_amqp_channel_flow_arg_t   args;
            amqp_frame_get_channel_flow( frame, &args );
            ret = amqp_channel_backend_get_flow( channel, &args, conn );
            break;
        }
        case AMQP_FRAME_METHOD_CHANNEL_FLOW_OK  :
        {
            wiced_amqp_channel_flow_ok_arg_t   args;
            amqp_frame_get_channel_flow_ok( frame, &args );
            ret = amqp_channel_backend_get_flow_ok( channel, &args, conn );
            break;
        }
        default:
            break;
    }
    return ret;
}


static
wiced_result_t amqp_frame_recv_exchange_handle ( uint16_t method, uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    switch ( method )
    {
        case AMQP_FRAME_METHOD_EXCHANGE_DECLARE_OK  :
            ret = amqp_channel_exchange_backend_get_declare_ok( channel, conn );
            break;
        case AMQP_FRAME_METHOD_EXCHANGE_DELETE_OK  :
            ret = amqp_channel_exchange_backend_get_delete_ok( channel, conn );
            break;
        default:
            ret = WICED_ERROR;
            break;
    }
    return ret;
}

static
wiced_result_t amqp_frame_recv_queue_handle ( uint16_t method, uint16_t channel, wiced_amqp_frame_t *frame, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    switch ( method )
    {
        case AMQP_FRAME_METHOD_QUEUE_DECLARE_OK  :
        {
            wiced_amqp_queue_declare_ok_arg_t   args;
            amqp_frame_get_queue_declare_ok( frame, &args );
            ret = amqp_channel_queue_backend_get_declare_ok( channel, &args, conn );
            break;
        }
        case AMQP_FRAME_METHOD_QUEUE_BIND_OK  :
            ret = amqp_channel_queue_backend_get_bind_ok( channel, conn );
            break;

        case AMQP_FRAME_METHOD_QUEUE_UNBIND_OK  :
            ret = amqp_channel_queue_backend_get_unbind_ok( channel, conn );
            break;

        case AMQP_FRAME_METHOD_QUEUE_PURGE_OK    :
        {
            wiced_amqp_queue_purge_ok_arg_t   args;
            amqp_frame_get_queue_purge_ok( frame, &args );
            ret = amqp_channel_queue_backend_get_purge_ok( channel, &args, conn );
            break;
        }
        case AMQP_FRAME_METHOD_QUEUE_DELETE_OK  :
        {
            wiced_amqp_queue_delete_ok_arg_t   args;
            amqp_frame_get_queue_delete_ok( frame, &args );
            ret = amqp_channel_queue_backend_get_delete_ok( channel, &args, conn );
            break;
        }

        default:
            ret = WICED_ERROR;
            break;
    }
    return ret;
}

static
wiced_result_t amqp_frame_recv_basic_handle ( uint16_t method, uint16_t channel, wiced_amqp_frame_t *frame, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    switch ( method)
    {
        case AMQP_FRAME_METHOD_BASIC_QOS_OK  :
        {
            ret = amqp_channel_basic_backend_get_qos_ok( channel, conn );
            break;
        }
        case AMQP_FRAME_METHOD_BASIC_CONSUME_OK  :
        {
            wiced_amqp_basic_consume_ok_arg_t   args;
            amqp_frame_get_basic_consume_ok( frame, &args );
            ret = amqp_channel_basic_backend_get_consume_ok( channel, &args, conn );
            break;
        }
        case AMQP_FRAME_METHOD_BASIC_CANCEL_OK  :
        {
            wiced_amqp_basic_cancel_ok_arg_t   args;
            amqp_frame_get_basic_cancel_ok( frame, &args );
            ret = amqp_channel_basic_backend_get_cancel_ok( channel, &args, conn );
            break;
        }
        case AMQP_FRAME_METHOD_BASIC_RETURN  :
        {
            wiced_amqp_basic_return_arg_t   args;
            amqp_frame_get_basic_return( frame, &args );
            ret = amqp_channel_basic_backend_get_return( channel, &args, conn );
            break;
        }
        case AMQP_FRAME_METHOD_BASIC_DELIVER  :
        {
            wiced_amqp_basic_deliver_arg_t   args;
            amqp_frame_get_basic_deliver( frame, &args );
            ret = amqp_channel_basic_backend_get_deliver( channel, &args, conn );
            break;
        }
        case AMQP_FRAME_METHOD_BASIC_GET_OK  :
        {
            wiced_amqp_basic_get_ok_arg_t   args;
            amqp_frame_get_basic_get_ok( frame, &args );
            ret = amqp_channel_basic_backend_get_get_ok( channel, &args, conn );
            break;
        }
        case AMQP_FRAME_METHOD_BASIC_GET_EMPTY  :
        {
            ret = amqp_channel_basic_backend_get_get_empty( channel, conn );
            break;
        }
        case AMQP_FRAME_METHOD_BASIC_RECOVER_OK  :
        {
            ret = amqp_channel_basic_backend_get_recover_ok( channel, conn );
            break;
        }
        default:
            ret = WICED_ERROR;
            break;
    }
    return ret;
}

static
wiced_result_t amqp_frame_recv_tx_handle ( uint16_t method, uint16_t channel, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    switch ( method )
    {
        case AMQP_FRAME_METHOD_TX_SELECT_OK  :
            ret = amqp_channel_tx_backend_get_select_ok( channel, conn );
            break;

        case AMQP_FRAME_METHOD_TX_COMMIT_OK  :
            ret = amqp_channel_tx_backend_get_commit_ok( channel, conn );
            break;

        case AMQP_FRAME_METHOD_TX_ROLLBACK_OK  :
            ret = amqp_channel_tx_backend_get_rollback_ok( channel, conn );
            break;

        default:
            ret = WICED_ERROR;
            break;
    }
    return ret;
}

static
wiced_result_t amqp_frame_recv_method_handle( uint16_t channel, wiced_amqp_frame_t *frame, wiced_amqp_connection_t *conn )
{
    uint16_t        class;
    uint16_t        method;
    wiced_result_t  ret;

    AMQP_BUFFER_GET_SHORT( &frame->buffer, class );
    AMQP_BUFFER_GET_SHORT( &frame->buffer, method );
    switch (class)
    {
    case AMQP_FRAME_CLASS_CONNECTION:
        ret = amqp_frame_recv_connection_handle( method, channel, frame, conn );
        break;
    case AMQP_FRAME_CLASS_CHANNEL   :
        ret = amqp_frame_recv_channel_handle( method, channel, frame, conn );
        break;
    case AMQP_FRAME_CLASS_EXCHANGE  :
        ret = amqp_frame_recv_exchange_handle( method, channel, conn );
        break;
    case AMQP_FRAME_CLASS_QUEUE     :
        ret = amqp_frame_recv_queue_handle( method, channel, frame, conn );
        break;
    case AMQP_FRAME_CLASS_BASIC     :
        ret = amqp_frame_recv_basic_handle( method, channel, frame, conn );
        break;
    case AMQP_FRAME_CLASS_TX        :
        ret = amqp_frame_recv_tx_handle( method, channel, conn );
        break;
    default:
        return WICED_ERROR;
    }
    return ret;
}

static
wiced_result_t amqp_frame_recv_content_handle ( uint16_t type, uint16_t channel, wiced_amqp_frame_t *frame, wiced_amqp_connection_t *conn )
{
    wiced_result_t ret;
    switch ( type )
    {
        case WICED_AMQP_FRAME_TYPE_HEADER:
        {
            wiced_amqp_content_header_arg_t args;
            amqp_frame_get_content_header( frame, &args );
            ret = amqp_channel_content_backend_get_header( channel, &args, conn );
            break;
        }
        case WICED_AMQP_FRAME_TYPE_CONTENT:
        {
            ret = amqp_channel_content_backend_get_content( channel, frame, conn );
            break;
        }
        default:
            return WICED_ERROR;
    }
    return ret;
}

wiced_result_t  amqp_frame_recv( wiced_amqp_buffer_t *buffer, void *p_user, uint32_t *size )
{
    uint8_t                  delimiter;
    uint16_t                 channel;
    wiced_amqp_frame_type_t  type;
    wiced_amqp_frame_t       frame;
    wiced_result_t           result;
    wiced_amqp_connection_t *conn = (wiced_amqp_connection_t *)p_user;

    if ( NULL == buffer )
    {
        /* Receive error, probably connection closed */
        return amqp_connection_backend_closed( p_user );
    }
    /* Check valid type type */
    frame.start  = buffer->data;
    frame.buffer = *buffer;
    AMQP_BUFFER_GET_OCTET( buffer, type );
    if ( (type == WICED_AMQP_FRAME_TYPE_METHOD ) || (type == WICED_AMQP_FRAME_TYPE_HEADER ) || (type == WICED_AMQP_FRAME_TYPE_CONTENT ) || (type == WICED_AMQP_FRAME_TYPE_HEARTBEAT ))
    {
        uint32_t frame_size;
        AMQP_BUFFER_GET_SHORT( buffer, channel );
        /* An AMQP frame can support up to 2^32 bytes in a frame size. WICED packets on the other hand
         * Can only support 2^16 bytes. However at connection setup we set the maximum size to be within
         * the WICED packet size (i.e < 2^16). So we know for sure that the size fits in frame.size.
         */
        AMQP_BUFFER_GET_LONG( buffer, frame_size );
        frame.size = (uint16_t) frame_size;
        buffer->data += frame.size;
        AMQP_BUFFER_GET_OCTET( buffer, delimiter );
        if ( delimiter != AMQP_FRAME_DELIMITER )
        {
            return WICED_ERROR;
        }
        *size = (uint32_t)( frame.size + 8 );    /* set total frame size used */
        frame.buffer.data = frame.start + 7;

        switch ( type )
        {
            case WICED_AMQP_FRAME_TYPE_METHOD:
                result = amqp_frame_recv_method_handle( channel, &frame, conn );
                break;
            case WICED_AMQP_FRAME_TYPE_HEADER:
            case WICED_AMQP_FRAME_TYPE_CONTENT:
                result = amqp_frame_recv_content_handle( type, channel, &frame, p_user );
                break;
            case WICED_AMQP_FRAME_TYPE_HEARTBEAT:
                result = amqp_connection_backend_get_heartbeat( channel, p_user );
                break;
            case WICED_AMQP_FRAME_TYPE_PROTOCOL_HEADER:
            default:
                result = WICED_ERROR;
                break;
        }
    }
    else
    {
        uint32_t    amqp_str;
        frame.buffer.data = frame.start;
        AMQP_BUFFER_GET_LONG( &frame.buffer, amqp_str );

        if ( amqp_str == AMQP_FRAME_HEADER_PROTOCOL_STR )
        {
            wiced_amqp_protocol_header_arg_t args;

            *size = 8;
            amqp_frame_get_protocol_header( &frame, &args );
            result = amqp_connection_backend_get_protocol_header( &args, p_user );
        }
        else
        {
            return WICED_ERROR;
        }
    }
    return result;
}


wiced_result_t  amqp_frame_create( wiced_amqp_frame_type_t type, uint16_t channel, uint16_t max_size, wiced_amqp_frame_t *frame, wiced_amqp_socket_t *socket )
{
    wiced_result_t ret;

    ret = amqp_network_create_buffer( &frame->buffer, max_size, socket );
    if ( ret == WICED_SUCCESS )
    {
        frame->size     = 0;
        frame->start    = frame->buffer.data;
        if ( type != WICED_AMQP_FRAME_TYPE_PROTOCOL_HEADER )
        {
            AMQP_BUFFER_PUT_OCTET( &frame->buffer, type );
            AMQP_BUFFER_PUT_SHORT( &frame->buffer, channel );
            AMQP_BUFFER_PUT_LONG( &frame->buffer, 0 );
        }
    }
    return ret;
}

wiced_result_t  amqp_frame_delete( wiced_amqp_frame_t *frame )
{
    return amqp_network_delete_buffer( &frame->buffer );
}


wiced_result_t  amqp_frame_send( wiced_amqp_frame_t *frame, wiced_amqp_socket_t *socket )
{
    wiced_amqp_frame_type_t type;
    uint8_t     *end = frame->buffer.data;

    /* Setting Size (in a hacky way) */
    frame->buffer.data = frame->start;
    AMQP_BUFFER_GET_OCTET( &frame->buffer, type );
    if ( (type == WICED_AMQP_FRAME_TYPE_METHOD ) || (type == WICED_AMQP_FRAME_TYPE_HEADER ) || (type == WICED_AMQP_FRAME_TYPE_CONTENT ) || (type == WICED_AMQP_FRAME_TYPE_HEARTBEAT ))
    {
        uint16_t  channel;
        AMQP_BUFFER_GET_SHORT( &frame->buffer, channel );
        AMQP_BUFFER_PUT_LONG( &frame->buffer, frame->size );
        /* Adding frame dilimeter */
        frame->buffer.data = end;
        AMQP_BUFFER_PUT_OCTET( &frame->buffer, AMQP_FRAME_DELIMITER );
    }
    else
    {
        /* Protocol header frame,  don't add size or delimeter */
        frame->buffer.data = end;
    }

    return amqp_network_send_buffer( &frame->buffer, socket );
}

/******************************************************
 *               Protocol Header Frames
 ******************************************************/
wiced_result_t amqp_frame_put_protocol_header( wiced_amqp_frame_t *frame, const wiced_amqp_protocol_header_arg_t *args )
{
    /* A long value carrying string AMQP */
    uint32_t    amqp_str = AMQP_FRAME_HEADER_PROTOCOL_STR;
    AMQP_BUFFER_PUT_LONG( &frame->buffer, amqp_str );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0 );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->major );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->minor );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->revision );

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_protocol_header( wiced_amqp_frame_t *frame, wiced_amqp_protocol_header_arg_t *args )
{
    uint8_t temp;

    AMQP_BUFFER_GET_OCTET( &frame->buffer, temp );
    UNUSED_VARIABLE( temp );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->major );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->minor );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->revision );

    return WICED_SUCCESS;
}

/******************************************************
 *               Connection Class Frames
 ******************************************************/

wiced_result_t amqp_frame_get_connection_start( wiced_amqp_frame_t *frame, wiced_amqp_connection_start_arg_t *args )
{
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->major );
    AMQP_BUFFER_GET_OCTET( &frame->buffer, args->minor );
    amqp_frame_get_table( args->server_properties, frame );
    AMQP_BUFFER_GET_LONG_STRING( &frame->buffer, args->mechnism.str, args->mechnism.len );
    AMQP_BUFFER_GET_LONG_STRING( &frame->buffer, args->locale.str, args->locale.len );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_connection_start_ok( wiced_amqp_frame_t *frame, const wiced_amqp_connection_start_ok_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_CONNECTION );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_CONNECTION_START_OK );
    frame->size = (uint16_t) ( frame->size + 4 );
    amqp_frame_put_table( args->client_properties, frame );
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->mechnism.str, args->mechnism.len );
    AMQP_BUFFER_PUT_LONG_STRING( &frame->buffer, args->response.str, args->response.len );
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->locale.str, args->locale.len );
    frame->size = (uint16_t) ( frame->size + args->mechnism.len + args->locale.len + (uint16_t) args->response.len + (uint16_t) sizeof( args->mechnism.len ) +  (uint16_t) sizeof( args->locale.len ) + (uint16_t) sizeof( args->response.len ) );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_connection_tune( wiced_amqp_frame_t *frame, wiced_amqp_connection_tune_arg_t *args )
{
    AMQP_BUFFER_GET_SHORT( &frame->buffer, args->channel_max );
    AMQP_BUFFER_GET_LONG( &frame->buffer, args->frame_max );
    AMQP_BUFFER_GET_SHORT( &frame->buffer, args->heartbeat );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_connection_tune_ok( wiced_amqp_frame_t *frame, const wiced_amqp_connection_tune_ok_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_CONNECTION );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_CONNECTION_TUNE_OK );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, args->channel_max );
    AMQP_BUFFER_PUT_LONG( &frame->buffer, args->frame_max );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, args->heartbeat );
    frame->size = (uint16_t) ( frame->size + 12 );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_connection_open( wiced_amqp_frame_t *frame, const wiced_amqp_connection_open_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_CONNECTION );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_CONNECTION_OPEN );
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->path.str, args->path.len );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0x0000 ); /* reserved capabilities | insist */
    frame->size = (uint16_t) ( frame->size + args->path.len + (uint16_t) sizeof( args->path.len ) + 6 );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_connection_close( wiced_amqp_frame_t *frame, const wiced_amqp_connection_close_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_CONNECTION );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_CONNECTION_CLOSE );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, args->reply_code );
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->reply_text.str, args->reply_text.len );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, args->class );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, args->method ); /* reserved */
    frame->size = (uint16_t) ( frame->size + args->reply_text.len + (uint16_t) sizeof( args->reply_text.len ) + 10 );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_connection_close( wiced_amqp_frame_t *frame, wiced_amqp_connection_close_arg_t *args )
{
    AMQP_BUFFER_GET_SHORT( &frame->buffer, args->reply_code );
    AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->reply_text.str, args->reply_text.len );
    AMQP_BUFFER_GET_SHORT( &frame->buffer, args->class );
    AMQP_BUFFER_GET_SHORT( &frame->buffer, args->method ); /* reserved */
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_connection_close_ok( wiced_amqp_frame_t *frame )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_CONNECTION );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_CONNECTION_CLOSE_OK );
    frame->size = (uint16_t) ( frame->size + 4 );
    return WICED_SUCCESS;
}

/******************************************************
 *               Channel Class Frames
 ******************************************************/

wiced_result_t amqp_frame_put_channel_open( wiced_amqp_frame_t *frame )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_CHANNEL );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_CHANNEL_OPEN );
    AMQP_BUFFER_PUT_OCTET( &frame->buffer, 0 ); /* reserved out-of-band */
    frame->size = (uint16_t) ( frame->size + 5 );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_channel_close( wiced_amqp_frame_t *frame, const wiced_amqp_channel_close_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_CHANNEL );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_CHANNEL_CLOSE );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, args->reply_code );
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->reply_text.str, args->reply_text.len );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, args->class );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, args->method ); /* reserved */
    frame->size = (uint16_t) ( frame->size + args->reply_text.len + (uint16_t) sizeof( args->reply_text.len ) + 10 );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_channel_close( wiced_amqp_frame_t *frame, wiced_amqp_channel_close_arg_t *args )
{
    AMQP_BUFFER_GET_SHORT( &frame->buffer, args->reply_code );
    AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->reply_text.str, args->reply_text.len );
    AMQP_BUFFER_GET_SHORT( &frame->buffer, args->class );
    AMQP_BUFFER_GET_SHORT( &frame->buffer, args->method ); /* reserved */
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_channel_flow( wiced_amqp_frame_t *frame, const wiced_amqp_channel_flow_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_CHANNEL );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_CHANNEL_FLOW );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->active, 0, 1 );
    frame->size = (uint16_t) ( frame->size + 5 );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_channel_close_ok( wiced_amqp_frame_t *frame )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_CHANNEL );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_CHANNEL_CLOSE_OK );
    frame->size = (uint16_t) ( frame->size + 4 );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_channel_flow( wiced_amqp_frame_t *frame, wiced_amqp_channel_flow_arg_t *args )
{
    AMQP_BUFFER_GET_BIT( &frame->buffer, args->active, 0, 0 );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_channel_flow_ok( wiced_amqp_frame_t *frame, const wiced_amqp_channel_flow_ok_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_CHANNEL );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_CHANNEL_FLOW_OK );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->active, 0, 1 );
    frame->size = (uint16_t) ( frame->size + 5 );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_channel_flow_ok( wiced_amqp_frame_t *frame, wiced_amqp_channel_flow_ok_arg_t *args )
{
    return amqp_frame_get_channel_flow( frame, args );
}

/******************************************************
 *               Exchange Class Frames
 ******************************************************/

wiced_result_t amqp_frame_put_exchange_declare( wiced_amqp_frame_t *frame, const wiced_amqp_exchange_declare_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_EXCHANGE );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_EXCHANGE_DECLARE );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0);  /* reserved ticket */
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->exchange.str, args->exchange.len );
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->type.str, args->type.len );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->passive, 0, 0 );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->durable, 1, 0 );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, 0, 2, 0 );     /* reserved auto-delete */
    AMQP_BUFFER_PUT_BIT( &frame->buffer, 0, 3, 0 );     /* reserved internal */
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->no_wait, 4, 1 );
    frame->size = (uint16_t) ( frame->size + 7 );
    frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->exchange ) + LENGTH_OF_STRING( args->type ) );
    amqp_frame_put_table( args->arguments, frame );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_exchange_delete( wiced_amqp_frame_t *frame, const wiced_amqp_exchange_delete_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_EXCHANGE );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_EXCHANGE_DELETE );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0);  /* reserved "ticket" */
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->exchange.str, args->exchange.len );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->if_unused, 0, 0 );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->no_wait, 1, 1 );
    frame->size = (uint16_t) ( frame->size + 7 );
    frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->exchange ) );
    return WICED_SUCCESS;
}

/******************************************************
 *               Queue Class Frames
 ******************************************************/

wiced_result_t amqp_frame_put_queue_declare( wiced_amqp_frame_t *frame, const wiced_amqp_queue_declare_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_QUEUE );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_QUEUE_DECLARE );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0);  /* reserved "ticket" */
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->queue.str, args->queue.len );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->passive     , 0, 0 );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->durable     , 1, 0 );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->exclusive   , 2, 0 );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->auto_delete , 3, 0 );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->no_wait     , 4, 1 );
    frame->size = (uint16_t) ( frame->size + 7 );
    frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->queue ) );
    amqp_frame_put_table( args->arguments, frame );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_queue_declare_ok( wiced_amqp_frame_t *frame, wiced_amqp_queue_declare_ok_arg_t *args )
{
    AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->queue.str, args->queue.len );
    AMQP_BUFFER_GET_LONG( &frame->buffer, args->message_count );
    AMQP_BUFFER_GET_LONG( &frame->buffer, args->consumer_count );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_queue_bind( wiced_amqp_frame_t *frame, const wiced_amqp_queue_bind_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_QUEUE );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_QUEUE_BIND );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0);  /* reserved ticket */
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->queue.str, args->queue.len );
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->exchange.str, args->exchange.len );
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->routing_key.str, args->routing_key.len );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->no_wait     , 0, 1 );
    frame->size = (uint16_t) ( frame->size +  7 );   /* sum of shorts, longs and bytes */
    frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->queue ) + LENGTH_OF_STRING( args->exchange ) + LENGTH_OF_STRING( args->routing_key ) );
    amqp_frame_put_table( args->arguments, frame );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_queue_unbind( wiced_amqp_frame_t *frame, const wiced_amqp_queue_unbind_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_QUEUE );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_QUEUE_UNBIND );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0);  /* reserved ticket */
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->queue.str, args->queue.len );
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->exchange.str, args->exchange.len );
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->routing_key.str, args->routing_key.len );
    frame->size = (uint16_t) ( frame->size + 6 );   /* sum of shorts, longs and bytes */
    frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->queue ) + LENGTH_OF_STRING( args->exchange ) + LENGTH_OF_STRING( args->routing_key ) );
    amqp_frame_put_table( args->arguments, frame );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_queue_purge( wiced_amqp_frame_t *frame, const wiced_amqp_queue_purge_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_QUEUE );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_QUEUE_PURGE );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0);  /* reserved ticket */
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->queue.str, args->queue.len );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->no_wait     , 0, 1 );
    frame->size = (uint16_t) ( frame->size + 7 );   /* sum of shorts, longs and bytes */
    frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->queue ) );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_queue_purge_ok( wiced_amqp_frame_t *frame, wiced_amqp_queue_purge_ok_arg_t *args )
{
    AMQP_BUFFER_GET_LONG( &frame->buffer, args->message_count );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_queue_delete( wiced_amqp_frame_t *frame, const wiced_amqp_queue_delete_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_QUEUE );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_QUEUE_DELETE );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0);  /* reserved ticket */
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->queue.str, args->queue.len );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->if_unused , 0, 0 );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->if_empty  , 1, 0 );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->no_wait   , 2, 1 );
    frame->size = (uint16_t) ( frame->size + 7 );   /* sum of shorts, longs and bytes */
    frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->queue ) );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_queue_delete_ok( wiced_amqp_frame_t *frame, wiced_amqp_queue_delete_ok_arg_t *args )
{
    return amqp_frame_get_queue_purge_ok( frame, args );
}

/******************************************************
 *               Queue Class Frames
 ******************************************************/
wiced_result_t amqp_frame_put_basic_qos( wiced_amqp_frame_t *frame, const wiced_amqp_basic_qos_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_BASIC );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_BASIC_QOS );
    AMQP_BUFFER_PUT_LONG( &frame->buffer, args->prefetch_size);
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, args->prefetch_count );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->global     , 0, 1 );
    frame->size = (uint16_t) ( frame->size + 11 );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_basic_consume( wiced_amqp_frame_t *frame, const wiced_amqp_basic_consume_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_BASIC );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_BASIC_CONSUME );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0);  /* reserved ticket */
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->queue.str, args->queue.len );
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->consumer_tag.str, args->consumer_tag.len );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->no_local  , 0, 0 );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->no_ack    , 1, 0 );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->exclusive , 2, 0 );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->no_wait   , 3, 1 );
    frame->size = (uint16_t) ( frame->size + 7 );   /* sum of shorts, longs and bytes */
    frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->queue ) + LENGTH_OF_STRING( args->consumer_tag ) );
    amqp_frame_put_table( args->arguments, frame );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_basic_consume_ok( wiced_amqp_frame_t *frame, wiced_amqp_basic_cancel_ok_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->consumer_tag.str, args->consumer_tag.len );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_basic_cancel( wiced_amqp_frame_t *frame, const wiced_amqp_basic_cancel_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_BASIC );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_BASIC_CANCEL );
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->consumer_tag.str, args->consumer_tag.len );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->no_wait   , 0, 1 );
    frame->size = (uint16_t) ( frame->size + 5 );   /* sum of shorts, longs and bytes */
    frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->consumer_tag ) );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_basic_cancel_ok( wiced_amqp_frame_t *frame, wiced_amqp_basic_cancel_ok_arg_t *args )
{
    return amqp_frame_get_basic_consume_ok( frame, args );
}

wiced_result_t amqp_frame_put_basic_publish( wiced_amqp_frame_t *frame, const wiced_amqp_basic_publish_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_BASIC );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_BASIC_PUBLISH );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0);  /* reserved ticket */
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->exchange.str, args->exchange.len );
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->routing_key.str, args->routing_key.len );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->mandatory  , 0, 0 );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->immediate  , 1, 1 );
    frame->size = (uint16_t) ( frame->size + 7 );   /* sum of shorts, longs and bytes */
    frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->exchange ) + LENGTH_OF_STRING( args->routing_key ) );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_basic_return( wiced_amqp_frame_t *frame, wiced_amqp_basic_return_arg_t *args )
{
    AMQP_BUFFER_GET_SHORT( &frame->buffer, args->reply_code );
    AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->reply_text.str, args->reply_text.len );
    AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->exchange.str, args->exchange.len );
    AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->routing_key.str, args->routing_key.len );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_basic_deliver( wiced_amqp_frame_t *frame, wiced_amqp_basic_deliver_arg_t *args )
{
    AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->consumer_tag.str, args->consumer_tag.len );
    AMQP_BUFFER_GET_LONG_LONG( &frame->buffer, args->delivery_tag );
    AMQP_BUFFER_GET_BIT( &frame->buffer, args->redeliverd , 0, 1 );
    AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->exchange.str, args->exchange.len );
    AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->routing_key.str, args->routing_key.len );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_basic_get( wiced_amqp_frame_t *frame, const wiced_amqp_basic_get_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_BASIC );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_BASIC_GET );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0);  /* reserved ticket */
    AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->queue.str, args->queue.len );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->no_ack  , 0, 1 );
    frame->size = (uint16_t) ( frame->size + 7 );   /* sum of shorts, longs and bytes */
    frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->queue ) );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_basic_get_ok( wiced_amqp_frame_t *frame, wiced_amqp_basic_get_ok_arg_t *args )
{
    AMQP_BUFFER_GET_LONG_LONG( &frame->buffer, args->delivery_tag );
    AMQP_BUFFER_GET_BIT( &frame->buffer, args->redeliverd , 0, 1 );
    AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->exchange.str, args->exchange.len );
    AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->routing_key.str, args->routing_key.len );
    AMQP_BUFFER_GET_LONG( &frame->buffer, args->message_count );
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_basic_ack( wiced_amqp_frame_t *frame, const wiced_amqp_basic_ack_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_BASIC );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_BASIC_ACK );
    AMQP_BUFFER_PUT_LONG_LONG( &frame->buffer, args->delivery_tag );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->multiple , 0, 1 );
    frame->size = (uint16_t) ( frame->size + 13 );   /* sum of shorts, longs and bytes */
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_basic_reject( wiced_amqp_frame_t *frame, const wiced_amqp_basic_reject_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_BASIC );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_BASIC_REJECT );
    AMQP_BUFFER_PUT_LONG_LONG( &frame->buffer, args->delivery_tag );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->requeue , 0, 1 );
    frame->size = (uint16_t) ( frame->size + 13 );   /* sum of shorts, longs and bytes */
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_basic_recover_async( wiced_amqp_frame_t *frame, const wiced_amqp_basic_recover_async_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_BASIC );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_BASIC_RECOVER_ASYNC );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->requeue , 0, 1 );
    frame->size = (uint16_t) ( frame->size + 5 );   /* sum of shorts, longs and bytes */
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_basic_recover( wiced_amqp_frame_t *frame, const wiced_amqp_basic_recover_arg_t *args )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_BASIC );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_BASIC_RECOVER );
    AMQP_BUFFER_PUT_BIT( &frame->buffer, args->requeue , 0, 1 );
    frame->size = (uint16_t) ( frame->size + 5 );   /* sum of shorts, longs and bytes */
    return WICED_SUCCESS;
}

/******************************************************
 *               TX Class Frames
 ******************************************************/

wiced_result_t amqp_frame_put_tx_select( wiced_amqp_frame_t *frame )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_TX );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_TX_SELECT );
    frame->size = (uint16_t) ( frame->size + 4 );   /* sum of shorts, longs and bytes */
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_tx_commit( wiced_amqp_frame_t *frame )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_TX );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_TX_COMMIT );
    frame->size = (uint16_t) ( frame->size + 4 );   /* sum of shorts, longs and bytes */
    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_put_tx_rollback( wiced_amqp_frame_t *frame )
{
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_TX );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_METHOD_TX_ROLLBACK );
    frame->size = (uint16_t) ( frame->size + 4 );   /* sum of shorts, longs and bytes */
    return WICED_SUCCESS;
}

/******************************************************
 *               Content/Header Frames
 ******************************************************/
wiced_result_t amqp_frame_put_content_header( wiced_amqp_frame_t *frame,  const wiced_amqp_content_header_arg_t *args )
{
    uint16_t    property_flags = 0;
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, AMQP_FRAME_CLASS_BASIC );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, 0 );     /* weight */
    AMQP_BUFFER_PUT_LONG_LONG( &frame->buffer, args->size );
    property_flags = (uint16_t)( property_flags | ( args->content_type_flag       ?   AMQP_FRAME_CONTENT_HEADER_FLAGS_CONTENT_TYPE_MASK       : 0 ) );
    property_flags = (uint16_t)( property_flags | ( args->content_encoding_flag   ?   AMQP_FRAME_CONTENT_HEADER_FLAGS_CONTENT_ENCODING_MASK   : 0 ) );
    property_flags = (uint16_t)( property_flags | ( args->headers_flag            ?   AMQP_FRAME_CONTENT_HEADER_FLAGS_HEADERS_MASK            : 0 ) );
    property_flags = (uint16_t)( property_flags | ( args->delivery_mode_flag      ?   AMQP_FRAME_CONTENT_HEADER_FLAGS_DELIVERY_MODE_MASK      : 0 ) );
    property_flags = (uint16_t)( property_flags | ( args->priority_flag           ?   AMQP_FRAME_CONTENT_HEADER_FLAGS_PRIORITY_MASK           : 0 ) );
    property_flags = (uint16_t)( property_flags | ( args->correlation_id_flag     ?   AMQP_FRAME_CONTENT_HEADER_FLAGS_CORRELATION_ID_MASK     : 0 ) );
    property_flags = (uint16_t)( property_flags | ( args->reply_to_flag           ?   AMQP_FRAME_CONTENT_HEADER_FLAGS_REPLY_TO_MASK           : 0 ) );
    property_flags = (uint16_t)( property_flags | ( args->expiration_flag         ?   AMQP_FRAME_CONTENT_HEADER_FLAGS_EXPIRATION_MASK         : 0 ) );
    property_flags = (uint16_t)( property_flags | ( args->message_id_flag         ?   AMQP_FRAME_CONTENT_HEADER_FLAGS_MESSAGE_ID_MASK         : 0 ) );
    property_flags = (uint16_t)( property_flags | ( args->timestamp_flag          ?   AMQP_FRAME_CONTENT_HEADER_FLAGS_TIMESTAMP_MASK          : 0 ) );
    property_flags = (uint16_t)( property_flags | ( args->type_flag               ?   AMQP_FRAME_CONTENT_HEADER_FLAGS_TYPE_MASK               : 0 ) );
    property_flags = (uint16_t)( property_flags | ( args->user_id_flag            ?   AMQP_FRAME_CONTENT_HEADER_FLAGS_USER_ID_MASK            : 0 ) );
    property_flags = (uint16_t)( property_flags | ( args->app_id_flag             ?   AMQP_FRAME_CONTENT_HEADER_FLAGS_APP_ID_MASK             : 0 ) );
    AMQP_BUFFER_PUT_SHORT( &frame->buffer, property_flags );     /* weight */
    frame->size = (uint16_t) ( frame->size + 14 );   /* sum of shorts, longs and bytes */

    if ( args->content_type_flag       ) { AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->content_type.str, args->content_type.len );          frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->content_type     ) );}
    if ( args->content_encoding_flag   ) { AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->content_encoding.str, args->content_encoding .len ); frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->content_encoding ) );}
    if ( args->headers_flag            ) { amqp_frame_put_table( args->headers, frame );                                                                                                                                                  }
    if ( args->delivery_mode_flag      ) { AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->delivery_mode );                                            frame->size ++;                                                                       }
    if ( args->priority_flag           ) { AMQP_BUFFER_PUT_OCTET( &frame->buffer, args->priority );                                                 frame->size ++;                                                                       }
    if ( args->correlation_id_flag     ) { AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->correlation_id.str, args->correlation_id.len );      frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->correlation_id   ) );}
    if ( args->reply_to_flag           ) { AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->reply_to.str, args->reply_to.len );                  frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->reply_to         ) );}
    if ( args->expiration_flag         ) { AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->expiration.str, args->expiration.len );              frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->expiration       ) );}
    if ( args->message_id_flag         ) { AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->message_id.str, args->message_id.len );              frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->message_id       ) );}
    if ( args->timestamp_flag          ) { AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->timestamp.str, args->timestamp.len );                frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->timestamp        ) );}
    if ( args->type_flag               ) { AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->type.str, args->type.len );                          frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->type             ) );}
    if ( args->user_id_flag            ) { AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->user_id.str, args->user_id.len );                    frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->user_id          ) );}
    if ( args->app_id_flag             ) { AMQP_BUFFER_PUT_SHORT_STRING( &frame->buffer, args->app_id.str, args->app_id.len );                      frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->app_id           ) );}

    return WICED_SUCCESS;
}

wiced_result_t amqp_frame_get_content_header( wiced_amqp_frame_t *frame,  wiced_amqp_content_header_arg_t *args )
{
    uint16_t    property_flags = 0;
    uint16_t    class, weight;
    AMQP_BUFFER_GET_SHORT( &frame->buffer, class );
    AMQP_BUFFER_GET_SHORT( &frame->buffer, weight );     /* weight */
    AMQP_BUFFER_GET_LONG_LONG( &frame->buffer, args->size );
    AMQP_BUFFER_GET_SHORT( &frame->buffer, property_flags );
    args->content_type_flag       = property_flags &   AMQP_FRAME_CONTENT_HEADER_FLAGS_CONTENT_TYPE_MASK      ? 1 : 0;
    args->content_encoding_flag   = property_flags &   AMQP_FRAME_CONTENT_HEADER_FLAGS_CONTENT_ENCODING_MASK  ? 1 : 0;
    args->headers_flag            = property_flags &   AMQP_FRAME_CONTENT_HEADER_FLAGS_HEADERS_MASK           ? 1 : 0;
    args->delivery_mode_flag      = property_flags &   AMQP_FRAME_CONTENT_HEADER_FLAGS_DELIVERY_MODE_MASK     ? 1 : 0;
    args->priority_flag           = property_flags &   AMQP_FRAME_CONTENT_HEADER_FLAGS_PRIORITY_MASK          ? 1 : 0;
    args->correlation_id_flag     = property_flags &   AMQP_FRAME_CONTENT_HEADER_FLAGS_CORRELATION_ID_MASK    ? 1 : 0;
    args->reply_to_flag           = property_flags &   AMQP_FRAME_CONTENT_HEADER_FLAGS_REPLY_TO_MASK          ? 1 : 0;
    args->expiration_flag         = property_flags &   AMQP_FRAME_CONTENT_HEADER_FLAGS_EXPIRATION_MASK        ? 1 : 0;
    args->message_id_flag         = property_flags &   AMQP_FRAME_CONTENT_HEADER_FLAGS_MESSAGE_ID_MASK        ? 1 : 0;
    args->timestamp_flag          = property_flags &   AMQP_FRAME_CONTENT_HEADER_FLAGS_TIMESTAMP_MASK         ? 1 : 0;
    args->type_flag               = property_flags &   AMQP_FRAME_CONTENT_HEADER_FLAGS_TYPE_MASK              ? 1 : 0;
    args->user_id_flag            = property_flags &   AMQP_FRAME_CONTENT_HEADER_FLAGS_USER_ID_MASK           ? 1 : 0;
    args->app_id_flag             = property_flags &   AMQP_FRAME_CONTENT_HEADER_FLAGS_APP_ID_MASK            ? 1 : 0;

    if ( args->content_type_flag       ) { AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->content_type.str, args->content_type.len );          frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->content_type     ) );}
    if ( args->content_encoding_flag   ) { AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->content_encoding.str, args->content_encoding .len ); frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->content_encoding ) );}
    if ( args->headers_flag            ) { amqp_frame_get_table( args->headers, frame );                                                                                                                                                  }
    if ( args->delivery_mode_flag      ) { AMQP_BUFFER_GET_OCTET( &frame->buffer, args->delivery_mode );                                            frame->size ++;                                                                       }
    if ( args->priority_flag           ) { AMQP_BUFFER_GET_OCTET( &frame->buffer, args->priority );                                                 frame->size ++;                                                                       }
    if ( args->correlation_id_flag     ) { AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->correlation_id.str, args->correlation_id.len );      frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->correlation_id   ) );}
    if ( args->reply_to_flag           ) { AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->reply_to.str, args->reply_to.len );                  frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->reply_to         ) );}
    if ( args->expiration_flag         ) { AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->expiration.str, args->expiration.len );              frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->expiration       ) );}
    if ( args->message_id_flag         ) { AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->message_id.str, args->message_id.len );              frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->message_id       ) );}
    if ( args->timestamp_flag          ) { AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->timestamp.str, args->timestamp.len );                frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->timestamp        ) );}
    if ( args->type_flag               ) { AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->type.str, args->type.len );                          frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->type             ) );}
    if ( args->user_id_flag            ) { AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->user_id.str, args->user_id.len );                    frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->user_id          ) );}
    if ( args->app_id_flag             ) { AMQP_BUFFER_GET_SHORT_STRING( &frame->buffer, args->app_id.str, args->app_id.len );                      frame->size = (uint16_t) ( frame->size + LENGTH_OF_STRING( args->app_id           ) );}

    return WICED_SUCCESS;
}
