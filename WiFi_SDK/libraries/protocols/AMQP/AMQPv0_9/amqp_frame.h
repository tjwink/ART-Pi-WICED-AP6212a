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
#define AMQP_FRAME_CLASS_CONNECTION     (10)
#define AMQP_FRAME_CLASS_CHANNEL        (20)
#define AMQP_FRAME_CLASS_EXCHANGE       (40)
#define AMQP_FRAME_CLASS_QUEUE          (50)
#define AMQP_FRAME_CLASS_BASIC          (60)
#define AMQP_FRAME_CLASS_TX             (90)

#define AMQP_FRAME_METHOD_CONNECTION_START      (10)
#define AMQP_FRAME_METHOD_CONNECTION_START_OK   (11)
#define AMQP_FRAME_METHOD_CONNECTION_SECURE     (20)
#define AMQP_FRAME_METHOD_CONNECTION_SECURE_OK  (21)
#define AMQP_FRAME_METHOD_CONNECTION_TUNE       (30)
#define AMQP_FRAME_METHOD_CONNECTION_TUNE_OK    (31)
#define AMQP_FRAME_METHOD_CONNECTION_OPEN       (40)
#define AMQP_FRAME_METHOD_CONNECTION_OPEN_OK    (41)
#define AMQP_FRAME_METHOD_CONNECTION_CLOSE      (50)
#define AMQP_FRAME_METHOD_CONNECTION_CLOSE_OK   (51)

#define AMQP_FRAME_METHOD_CHANNEL_OPEN          (10)
#define AMQP_FRAME_METHOD_CHANNEL_OPEN_OK       (11)
#define AMQP_FRAME_METHOD_CHANNEL_FLOW          (20)
#define AMQP_FRAME_METHOD_CHANNEL_FLOW_OK       (21)
#define AMQP_FRAME_METHOD_CHANNEL_CLOSE         (40)
#define AMQP_FRAME_METHOD_CHANNEL_CLOSE_OK      (41)

#define AMQP_FRAME_METHOD_EXCHANGE_DECLARE      (10)
#define AMQP_FRAME_METHOD_EXCHANGE_DECLARE_OK   (11)
#define AMQP_FRAME_METHOD_EXCHANGE_DELETE       (20)
#define AMQP_FRAME_METHOD_EXCHANGE_DELETE_OK    (21)

#define AMQP_FRAME_METHOD_QUEUE_DECLARE         (10)
#define AMQP_FRAME_METHOD_QUEUE_DECLARE_OK      (11)
#define AMQP_FRAME_METHOD_QUEUE_BIND            (20)
#define AMQP_FRAME_METHOD_QUEUE_BIND_OK         (21)
#define AMQP_FRAME_METHOD_QUEUE_UNBIND          (50)
#define AMQP_FRAME_METHOD_QUEUE_UNBIND_OK       (51)
#define AMQP_FRAME_METHOD_QUEUE_PURGE           (30)
#define AMQP_FRAME_METHOD_QUEUE_PURGE_OK        (31)
#define AMQP_FRAME_METHOD_QUEUE_DELETE          (40)
#define AMQP_FRAME_METHOD_QUEUE_DELETE_OK       (41)

#define AMQP_FRAME_METHOD_BASIC_QOS             (10)
#define AMQP_FRAME_METHOD_BASIC_QOS_OK          (11)
#define AMQP_FRAME_METHOD_BASIC_CONSUME         (20)
#define AMQP_FRAME_METHOD_BASIC_CONSUME_OK      (21)
#define AMQP_FRAME_METHOD_BASIC_CANCEL          (30)
#define AMQP_FRAME_METHOD_BASIC_CANCEL_OK       (31)
#define AMQP_FRAME_METHOD_BASIC_PUBLISH         (40)
#define AMQP_FRAME_METHOD_BASIC_RETURN          (50)
#define AMQP_FRAME_METHOD_BASIC_DELIVER         (60)
#define AMQP_FRAME_METHOD_BASIC_GET             (70)
#define AMQP_FRAME_METHOD_BASIC_GET_OK          (71)
#define AMQP_FRAME_METHOD_BASIC_GET_EMPTY       (72)
#define AMQP_FRAME_METHOD_BASIC_ACK             (80)
#define AMQP_FRAME_METHOD_BASIC_REJECT          (90)
#define AMQP_FRAME_METHOD_BASIC_RECOVER_ASYNC   (100)
#define AMQP_FRAME_METHOD_BASIC_RECOVER         (110)
#define AMQP_FRAME_METHOD_BASIC_RECOVER_OK      (111)

#define AMQP_FRAME_METHOD_TX_SELECT          (10)
#define AMQP_FRAME_METHOD_TX_SELECT_OK       (11)
#define AMQP_FRAME_METHOD_TX_COMMIT          (20)
#define AMQP_FRAME_METHOD_TX_COMMIT_OK       (21)
#define AMQP_FRAME_METHOD_TX_ROLLBACK        (30)
#define AMQP_FRAME_METHOD_TX_ROLLBACK_OK     (31)

#define AMQP_FRAME_ERROR_COMMAND_INVALID     (503)
/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *              AMQP Type Definitions
 ******************************************************/

typedef uint8_t wiced_amqp_bit_t;

typedef struct wiced_amqp_short_string_s
{
    uint8_t     len;
    uint8_t    *str;
} wiced_amqp_short_string_t;

typedef struct wiced_amqp_long_string_s
{
    uint32_t    len;
    uint8_t    *str;
} wiced_amqp_long_string_t;

typedef long long wiced_amqp_timestamp_t;

typedef union wiced_amqp_filed_value_u
{
    uint8_t     t;
    int8_t      b;
    uint8_t     B;
    int16_t     U;
    uint16_t    u;
    int32_t     I;
    uint32_t    i;
    int64_t     L;
    uint64_t    l;
    float       f;
    double      d;
    uint32_t    D;
    wiced_amqp_short_string_t s;
    wiced_amqp_long_string_t  S;
    wiced_amqp_timestamp_t    T;
} wiced_amqp_field_value_t;

typedef struct wiced_ampq_field_s
{
    wiced_amqp_short_string_t name;
    uint8_t                   type;
    wiced_amqp_field_value_t  value;
} wiced_amqp_field_t;

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
    uint8_t                     major;
    uint8_t                     minor;
    uint8_t                     revision;
} wiced_amqp_protocol_header_arg_t;

/******************************************************
 *         Connection Frame Type Definitions
 ******************************************************/

typedef struct wiced_amqp_connection_start_arg_s
{
    uint8_t                      major;
    uint8_t                      minor;
    wiced_amqp_field_t          *server_properties;
    wiced_amqp_long_string_t     mechnism;
    wiced_amqp_long_string_t     locale;
} wiced_amqp_connection_start_arg_t;

typedef struct wiced_amqp_connection_start_ok_arg_s
{
    wiced_amqp_field_t          *client_properties;
    wiced_amqp_short_string_t    mechnism;
    wiced_amqp_long_string_t     response;
    wiced_amqp_short_string_t    locale;
} wiced_amqp_connection_start_ok_arg_t;


typedef struct wiced_amqp_connection_tune_arg_s
{
    uint16_t                      channel_max;
    uint16_t                      heartbeat;
    uint32_t                      frame_max;
} wiced_amqp_connection_tune_arg_t;

typedef wiced_amqp_connection_tune_arg_t wiced_amqp_connection_tune_ok_arg_t;

typedef struct wiced_amqp_connection_open_arg_s
{
    wiced_amqp_short_string_t    path;
} wiced_amqp_connection_open_arg_t;

typedef struct wiced_amqp_connection_close_arg_s
{
    uint16_t                      reply_code;
    uint16_t                      class;
    uint16_t                      method;
    wiced_amqp_short_string_t     reply_text;
} wiced_amqp_connection_close_arg_t;

/******************************************************
 *            Channel Frame Type Definitions
 ******************************************************/

typedef uint8_t                           wiced_amqp_channel_open_arg_t;

typedef wiced_amqp_connection_close_arg_t wiced_amqp_channel_close_arg_t;

typedef struct wiced_amqp_channel_flow_arg_s
{
    wiced_amqp_bit_t            active;
} wiced_amqp_channel_flow_arg_t;
typedef wiced_amqp_channel_flow_arg_t        wiced_amqp_channel_flow_ok_arg_t;

/******************************************************
 *            Exchange Frame Type Definitions
 ******************************************************/
typedef struct wiced_amqp_exchange_declare_arg_s
{
    wiced_amqp_short_string_t   exchange;
    wiced_amqp_short_string_t   type;
    wiced_amqp_bit_t            passive;
    wiced_amqp_bit_t            durable;
    wiced_amqp_bit_t            no_wait;
    wiced_amqp_field_t         *arguments;
} wiced_amqp_exchange_declare_arg_t;

typedef struct wiced_amqp_exchange_delete_arg_s
{
    wiced_amqp_short_string_t   exchange;
    wiced_amqp_bit_t            if_unused;
    wiced_amqp_bit_t            no_wait;
} wiced_amqp_exchange_delete_arg_t;

/******************************************************
 *            Queue Frame Type Definitions
 ******************************************************/
typedef struct wiced_amqp_queue_declare_arg_s
{
    wiced_amqp_short_string_t   queue;
    wiced_amqp_bit_t            passive;
    wiced_amqp_bit_t            durable;
    wiced_amqp_bit_t            exclusive;
    wiced_amqp_bit_t            auto_delete;
    wiced_amqp_bit_t            no_wait;
    wiced_amqp_field_t         *arguments;
} wiced_amqp_queue_declare_arg_t;

typedef struct wiced_amqp_queue_declare_ok_arg_s
{
    wiced_amqp_short_string_t   queue;
    uint32_t                    message_count;
    uint32_t                    consumer_count;
} wiced_amqp_queue_declare_ok_arg_t;

typedef struct wiced_amqp_queue_bind_arg_s
{
    wiced_amqp_short_string_t   queue;
    wiced_amqp_short_string_t   exchange;
    wiced_amqp_short_string_t   routing_key;
    wiced_amqp_bit_t            no_wait;
    wiced_amqp_field_t         *arguments;
} wiced_amqp_queue_bind_arg_t;

typedef struct wiced_amqp_queue_unbind_arg_s
{
    wiced_amqp_short_string_t   queue;
    wiced_amqp_short_string_t   exchange;
    wiced_amqp_short_string_t   routing_key;
    wiced_amqp_field_t         *arguments;
} wiced_amqp_queue_unbind_arg_t;

typedef struct wiced_amqp_queue_purge_arg_s
{
    wiced_amqp_short_string_t   queue;
    wiced_amqp_bit_t            no_wait;
} wiced_amqp_queue_purge_arg_t;

typedef struct wiced_amqp_queue_purge_ok_arg_s
{
    uint32_t                    message_count;
} wiced_amqp_queue_purge_ok_arg_t;

typedef struct wiced_amqp_queue_delete_arg_s
{
    wiced_amqp_short_string_t   queue;
    wiced_amqp_bit_t            if_unused;
    wiced_amqp_bit_t            if_empty;
    wiced_amqp_bit_t            no_wait;
} wiced_amqp_queue_delete_arg_t;

typedef wiced_amqp_queue_purge_ok_arg_t wiced_amqp_queue_delete_ok_arg_t;


/******************************************************
 *             Basic Frame Type Definitions
 ******************************************************/
typedef struct wiced_amqp_basic_qos_arg_s
{
    uint32_t                    prefetch_size;
    uint16_t                    prefetch_count;
    wiced_amqp_bit_t            global;
} wiced_amqp_basic_qos_arg_t;

typedef struct wiced_amqp_basic_consume_arg_s
{
    wiced_amqp_short_string_t   queue;
    wiced_amqp_short_string_t   consumer_tag;
    wiced_amqp_bit_t            no_local;
    wiced_amqp_bit_t            no_ack;
    wiced_amqp_bit_t            exclusive;
    wiced_amqp_bit_t            no_wait;
    wiced_amqp_field_t         *arguments;
} wiced_amqp_basic_consume_arg_t;

typedef struct wiced_amqp_basic_consume_ok_arg_s
{
    wiced_amqp_short_string_t   consumer_tag;
} wiced_amqp_basic_consume_ok_arg_t;

typedef struct wiced_amqp_basic_cancel_arg_s
{
    wiced_amqp_short_string_t   consumer_tag;
    wiced_amqp_bit_t            no_wait;
} wiced_amqp_basic_cancel_arg_t;

typedef wiced_amqp_basic_consume_ok_arg_t wiced_amqp_basic_cancel_ok_arg_t;

typedef struct wiced_amqp_basic_publish_arg_s
{
    wiced_amqp_short_string_t   exchange;
    wiced_amqp_short_string_t   routing_key;
    wiced_amqp_bit_t            mandatory;
    wiced_amqp_bit_t            immediate;
} wiced_amqp_basic_publish_arg_t;

typedef struct wiced_amqp_basic_return_arg_s
{
    uint16_t                    reply_code;
    wiced_amqp_short_string_t   reply_text;
    wiced_amqp_short_string_t   exchange;
    wiced_amqp_short_string_t   routing_key;
} wiced_amqp_basic_return_arg_t;

typedef struct wiced_amqp_basic_deliver_arg_s
{
    wiced_amqp_short_string_t   consumer_tag;
    wiced_amqp_short_string_t   exchange;
    wiced_amqp_short_string_t   routing_key;
    uint64_t                    delivery_tag;
    wiced_amqp_bit_t            redeliverd;
} wiced_amqp_basic_deliver_arg_t;


typedef struct wiced_amqp_basic_get_arg_s
{
    wiced_amqp_short_string_t   queue;
    wiced_amqp_bit_t            no_ack;
} wiced_amqp_basic_get_arg_t;

typedef struct wiced_amqp_basic_get_ok_arg_s
{
    wiced_amqp_short_string_t   exchange;
    wiced_amqp_short_string_t   routing_key;
    uint64_t                    delivery_tag;
    wiced_amqp_bit_t            redeliverd;
    uint32_t                    message_count;
} wiced_amqp_basic_get_ok_arg_t;

typedef struct wiced_amqp_basic_ack_arg_s
{
    uint64_t                    delivery_tag;
    wiced_amqp_bit_t            multiple;
} wiced_amqp_basic_ack_arg_t;

typedef struct wiced_amqp_basic_reject_arg_s
{
    uint64_t                    delivery_tag;
    wiced_amqp_bit_t            requeue;
} wiced_amqp_basic_reject_arg_t;

typedef struct wiced_amqp_basic_recover_async_arg_s
{
    wiced_amqp_bit_t            requeue;
} wiced_amqp_basic_recover_async_arg_t;

typedef wiced_amqp_basic_recover_async_arg_t wiced_amqp_basic_recover_arg_t;

/******************************************************
 *             Content Frame Type Definitions
 ******************************************************/

typedef struct wiced_amqp_content_header_arg_s
{
    uint16_t                    class_id;
    uint64_t                    size;
    wiced_amqp_bit_t            content_type_flag;
    wiced_amqp_bit_t            content_encoding_flag;
    wiced_amqp_bit_t            headers_flag;
    wiced_amqp_bit_t            delivery_mode_flag;
    wiced_amqp_bit_t            priority_flag;
    wiced_amqp_bit_t            correlation_id_flag;
    wiced_amqp_bit_t            reply_to_flag;
    wiced_amqp_bit_t            expiration_flag;
    wiced_amqp_bit_t            message_id_flag;
    wiced_amqp_bit_t            timestamp_flag;
    wiced_amqp_bit_t            type_flag;
    wiced_amqp_bit_t            user_id_flag;
    wiced_amqp_bit_t            app_id_flag;
    wiced_amqp_short_string_t   content_type;
    wiced_amqp_short_string_t   content_encoding;
    wiced_amqp_field_t         *headers;
    uint8_t                     delivery_mode;
    uint8_t                     priority;
    wiced_amqp_short_string_t   correlation_id;
    wiced_amqp_short_string_t   reply_to;
    wiced_amqp_short_string_t   expiration;
    wiced_amqp_short_string_t   message_id;
    wiced_amqp_short_string_t   timestamp;
    wiced_amqp_short_string_t   type;
    wiced_amqp_short_string_t   user_id;
    wiced_amqp_short_string_t   app_id;
} wiced_amqp_content_header_arg_t;

typedef wiced_amqp_frame_t wiced_amqp_content_content_arg_t;
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

wiced_result_t amqp_frame_put_protocol_header    ( wiced_amqp_frame_t *frame, const wiced_amqp_protocol_header_arg_t      *args );
wiced_result_t amqp_frame_get_protocol_header    ( wiced_amqp_frame_t *frame,       wiced_amqp_protocol_header_arg_t      *args );
wiced_result_t amqp_frame_get_connection_start   ( wiced_amqp_frame_t *frame,       wiced_amqp_connection_start_arg_t     *args );
wiced_result_t amqp_frame_put_connection_start_ok( wiced_amqp_frame_t *frame, const wiced_amqp_connection_start_ok_arg_t  *args );
wiced_result_t amqp_frame_get_connection_tune    ( wiced_amqp_frame_t *frame,       wiced_amqp_connection_tune_arg_t      *args );
wiced_result_t amqp_frame_put_connection_tune_ok ( wiced_amqp_frame_t *frame, const wiced_amqp_connection_tune_ok_arg_t   *args );
wiced_result_t amqp_frame_put_connection_open    ( wiced_amqp_frame_t *frame, const wiced_amqp_connection_open_arg_t      *args );
wiced_result_t amqp_frame_put_connection_close   ( wiced_amqp_frame_t *frame, const wiced_amqp_connection_close_arg_t     *args );
wiced_result_t amqp_frame_get_connection_close   ( wiced_amqp_frame_t *frame,       wiced_amqp_connection_close_arg_t     *args );
wiced_result_t amqp_frame_put_connection_close_ok( wiced_amqp_frame_t *frame                                                    );

wiced_result_t amqp_frame_put_channel_open       ( wiced_amqp_frame_t *frame                                                    );
wiced_result_t amqp_frame_put_channel_close      ( wiced_amqp_frame_t *frame, const wiced_amqp_channel_close_arg_t        *args );
wiced_result_t amqp_frame_get_channel_close      ( wiced_amqp_frame_t *frame,       wiced_amqp_channel_close_arg_t        *args );
wiced_result_t amqp_frame_put_channel_close_ok   ( wiced_amqp_frame_t *frame                                                    );
wiced_result_t amqp_frame_put_channel_flow       ( wiced_amqp_frame_t *frame, const wiced_amqp_channel_flow_arg_t         *args );
wiced_result_t amqp_frame_get_channel_flow       ( wiced_amqp_frame_t *frame,       wiced_amqp_channel_flow_arg_t         *args );
wiced_result_t amqp_frame_put_channel_flow_ok    ( wiced_amqp_frame_t *frame, const wiced_amqp_channel_flow_ok_arg_t      *args );
wiced_result_t amqp_frame_get_channel_flow_ok    ( wiced_amqp_frame_t *frame,       wiced_amqp_channel_flow_ok_arg_t      *args );

wiced_result_t amqp_frame_put_exchange_declare   ( wiced_amqp_frame_t *frame, const wiced_amqp_exchange_declare_arg_t     *args );
wiced_result_t amqp_frame_put_exchange_delete    ( wiced_amqp_frame_t *frame, const wiced_amqp_exchange_delete_arg_t      *args );

wiced_result_t amqp_frame_put_queue_declare      ( wiced_amqp_frame_t *frame, const wiced_amqp_queue_declare_arg_t        *args );
wiced_result_t amqp_frame_get_queue_declare_ok   ( wiced_amqp_frame_t *frame,       wiced_amqp_queue_declare_ok_arg_t     *args );
wiced_result_t amqp_frame_put_queue_bind         ( wiced_amqp_frame_t *frame, const wiced_amqp_queue_bind_arg_t           *args );
wiced_result_t amqp_frame_put_queue_unbind       ( wiced_amqp_frame_t *frame, const wiced_amqp_queue_unbind_arg_t         *args );
wiced_result_t amqp_frame_put_queue_purge        ( wiced_amqp_frame_t *frame, const wiced_amqp_queue_purge_arg_t          *args );
wiced_result_t amqp_frame_get_queue_purge_ok     ( wiced_amqp_frame_t *frame,       wiced_amqp_queue_purge_ok_arg_t       *args );
wiced_result_t amqp_frame_put_queue_delete       ( wiced_amqp_frame_t *frame, const wiced_amqp_queue_delete_arg_t         *args );
wiced_result_t amqp_frame_get_queue_delete_ok    ( wiced_amqp_frame_t *frame,       wiced_amqp_queue_delete_ok_arg_t      *args );

wiced_result_t amqp_frame_put_basic_qos          ( wiced_amqp_frame_t *frame, const wiced_amqp_basic_qos_arg_t            *args );
wiced_result_t amqp_frame_put_basic_consume      ( wiced_amqp_frame_t *frame, const wiced_amqp_basic_consume_arg_t        *args );
wiced_result_t amqp_frame_get_basic_consume_ok   ( wiced_amqp_frame_t *frame,       wiced_amqp_basic_cancel_ok_arg_t      *args );
wiced_result_t amqp_frame_put_basic_cancel       ( wiced_amqp_frame_t *frame, const wiced_amqp_basic_cancel_arg_t         *args );
wiced_result_t amqp_frame_get_basic_cancel_ok    ( wiced_amqp_frame_t *frame,       wiced_amqp_basic_cancel_ok_arg_t      *args );
wiced_result_t amqp_frame_put_basic_publish      ( wiced_amqp_frame_t *frame, const wiced_amqp_basic_publish_arg_t        *args );
wiced_result_t amqp_frame_get_basic_return       ( wiced_amqp_frame_t *frame,       wiced_amqp_basic_return_arg_t         *args );
wiced_result_t amqp_frame_get_basic_deliver      ( wiced_amqp_frame_t *frame,       wiced_amqp_basic_deliver_arg_t        *args );
wiced_result_t amqp_frame_put_basic_get          ( wiced_amqp_frame_t *frame, const wiced_amqp_basic_get_arg_t            *args );
wiced_result_t amqp_frame_get_basic_get_ok       ( wiced_amqp_frame_t *frame,       wiced_amqp_basic_get_ok_arg_t         *args );
wiced_result_t amqp_frame_put_basic_ack          ( wiced_amqp_frame_t *frame, const wiced_amqp_basic_ack_arg_t            *args );
wiced_result_t amqp_frame_put_basic_reject       ( wiced_amqp_frame_t *frame, const wiced_amqp_basic_reject_arg_t         *args );
wiced_result_t amqp_frame_put_basic_recover_async( wiced_amqp_frame_t *frame, const wiced_amqp_basic_recover_async_arg_t  *args );
wiced_result_t amqp_frame_put_basic_recover      ( wiced_amqp_frame_t *frame, const wiced_amqp_basic_recover_arg_t        *args );

wiced_result_t amqp_frame_put_tx_select          ( wiced_amqp_frame_t *frame                                                    );
wiced_result_t amqp_frame_put_tx_commit          ( wiced_amqp_frame_t *frame                                                    );
wiced_result_t amqp_frame_put_tx_rollback        ( wiced_amqp_frame_t *frame                                                    );

wiced_result_t amqp_frame_put_content_header     ( wiced_amqp_frame_t *frame, const wiced_amqp_content_header_arg_t       *args );
wiced_result_t amqp_frame_get_content_header     ( wiced_amqp_frame_t *frame,       wiced_amqp_content_header_arg_t       *args );

#ifdef __cplusplus
} /* extern "C" */
#endif
