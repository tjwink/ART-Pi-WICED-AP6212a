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
 *  AMQP frame APIs and types.
 *
 *  Internal types not to be included directly by applications.
 */
#pragma once

#include "wiced.h"
#include "amqp_network.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
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
    *(data_po)   = (uint8_t)(VALUE);                      \
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
    *(data_pss) = (uint8_t)(LEN);                                   \
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
 *                   Enumerations
 ******************************************************/

/******************************************************
 *              AMQP Type Definitions
 ******************************************************/


typedef enum wiced_amqp_frame_type_e
{
    WICED_AMQP_FRAME_TYPE_PROTOCOL_HEADER   = 0,
    WICED_AMQP_FRAME_TYPE_METHOD            = 1,
    WICED_AMQP_FRAME_TYPE_HEADER            = 2,
    WICED_AMQP_FRAME_TYPE_CONTENT           = 3,
    WICED_AMQP_FRAME_TYPE_HEARTBEAT         = 8,
}wiced_amqp_frame_type_t;

typedef struct wiced_amqp_frame_s
{
    uint16_t                size;
    uint8_t                *start;
    wiced_amqp_buffer_t     buffer;
} wiced_amqp_frame_t;

/******************************************************
 *            Header Frame Type Definitions
 ******************************************************/


typedef struct wiced_amqp_protocol_header_arg_s
{
    uint8_t                     protocol_id;
    uint8_t                     major;
    uint8_t                     minor;
    uint8_t                     revision;
} wiced_amqp_protocol_header_arg_t;

typedef struct amqp_fixed_frame_t
{
        uint32_t size;
        uint8_t  doff;
        uint8_t  type;
        uint16_t channel;
        uint8_t  performative_type;
}amqp_fixed_frame;

/******************************************************
 *         Connection Frame Type Definitions
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

wiced_result_t  amqp_frame_create( wiced_amqp_frame_type_t type, uint16_t channel, uint16_t max_size, wiced_amqp_frame_t *frame, wiced_amqp_socket_t *socket );
wiced_result_t  amqp_frame_send  ( wiced_amqp_frame_t *frame, wiced_amqp_socket_t *socket );
wiced_result_t  amqp_frame_recv  ( wiced_amqp_buffer_t *buffer, void *p_user, uint32_t *size );
wiced_result_t  amqp_frame_delete( wiced_amqp_frame_t *frame );
uint8_t get_fixed_frame( wiced_amqp_frame_t frame, amqp_fixed_frame *fixed_frame );

/* helper functions */
wiced_result_t amqp_get_buffer( void** buffer, uint32_t size );

#ifdef __cplusplus
} /* extern "C" */
#endif
