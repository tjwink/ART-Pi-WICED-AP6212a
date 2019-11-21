/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *  MQTT frame APIs and types.
 *
 *  Internal types not to be included directly by applications.
 */
#pragma once

#include "wiced.h"
#include "mqtt_network.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define MQTT_FRAME_CLASS_CONNECTION     (10)
#define MQTT_FRAME_CLASS_CHANNEL        (20)
#define MQTT_FRAME_CLASS_EXCHANGE       (40)
#define MQTT_FRAME_CLASS_QUEUE          (50)
#define MQTT_FRAME_CLASS_BASIC          (60)
#define MQTT_FRAME_CLASS_TX             (90)

#define MQTT_FRAME_METHOD_CONNECTION_START      (10)
#define MQTT_FRAME_METHOD_CONNECTION_START_OK   (11)
#define MQTT_FRAME_METHOD_CONNECTION_SECURE     (20)
#define MQTT_FRAME_METHOD_CONNECTION_SECURE_OK  (21)
#define MQTT_FRAME_METHOD_CONNECTION_TUNE       (30)
#define MQTT_FRAME_METHOD_CONNECTION_TUNE_OK    (31)
#define MQTT_FRAME_METHOD_CONNECTION_OPEN       (40)
#define MQTT_FRAME_METHOD_CONNECTION_OPEN_OK    (41)
#define MQTT_FRAME_METHOD_CONNECTION_CLOSE      (50)
#define MQTT_FRAME_METHOD_CONNECTION_CLOSE_OK   (51)

#define MQTT_FRAME_METHOD_CHANNEL_OPEN          (10)
#define MQTT_FRAME_METHOD_CHANNEL_OPEN_OK       (11)
#define MQTT_FRAME_METHOD_CHANNEL_FLOW          (20)
#define MQTT_FRAME_METHOD_CHANNEL_FLOW_OK       (21)
#define MQTT_FRAME_METHOD_CHANNEL_CLOSE         (40)
#define MQTT_FRAME_METHOD_CHANNEL_CLOSE_OK      (41)

#define MQTT_FRAME_METHOD_EXCHANGE_DECLARE      (10)
#define MQTT_FRAME_METHOD_EXCHANGE_DECLARE_OK   (11)
#define MQTT_FRAME_METHOD_EXCHANGE_DELETE       (20)
#define MQTT_FRAME_METHOD_EXCHANGE_DELETE_OK    (21)

#define MQTT_FRAME_METHOD_QUEUE_DECLARE         (10)
#define MQTT_FRAME_METHOD_QUEUE_DECLARE_OK      (11)
#define MQTT_FRAME_METHOD_QUEUE_BIND            (20)
#define MQTT_FRAME_METHOD_QUEUE_BIND_OK         (21)
#define MQTT_FRAME_METHOD_QUEUE_UNBIND          (50)
#define MQTT_FRAME_METHOD_QUEUE_UNBIND_OK       (51)
#define MQTT_FRAME_METHOD_QUEUE_PURGE           (30)
#define MQTT_FRAME_METHOD_QUEUE_PURGE_OK        (31)
#define MQTT_FRAME_METHOD_QUEUE_DELETE          (40)
#define MQTT_FRAME_METHOD_QUEUE_DELETE_OK       (41)

#define MQTT_FRAME_METHOD_BASIC_QOS             (10)
#define MQTT_FRAME_METHOD_BASIC_QOS_OK          (11)
#define MQTT_FRAME_METHOD_BASIC_CONSUME         (20)
#define MQTT_FRAME_METHOD_BASIC_CONSUME_OK      (21)
#define MQTT_FRAME_METHOD_BASIC_CANCEL          (30)
#define MQTT_FRAME_METHOD_BASIC_CANCEL_OK       (31)
#define MQTT_FRAME_METHOD_BASIC_PUBLISH         (40)
#define MQTT_FRAME_METHOD_BASIC_RETURN          (50)
#define MQTT_FRAME_METHOD_BASIC_DELIVER         (60)
#define MQTT_FRAME_METHOD_BASIC_GET             (70)
#define MQTT_FRAME_METHOD_BASIC_GET_OK          (71)
#define MQTT_FRAME_METHOD_BASIC_GET_EMPTY       (72)
#define MQTT_FRAME_METHOD_BASIC_ACK             (80)
#define MQTT_FRAME_METHOD_BASIC_REJECT          (90)
#define MQTT_FRAME_METHOD_BASIC_RECOVER_ASYNC   (100)
#define MQTT_FRAME_METHOD_BASIC_RECOVER         (110)
#define MQTT_FRAME_METHOD_BASIC_RECOVER_OK      (111)

#define MQTT_FRAME_METHOD_TX_SELECT          (10)
#define MQTT_FRAME_METHOD_TX_SELECT_OK       (11)
#define MQTT_FRAME_METHOD_TX_COMMIT          (20)
#define MQTT_FRAME_METHOD_TX_COMMIT_OK       (21)
#define MQTT_FRAME_METHOD_TX_ROLLBACK        (30)
#define MQTT_FRAME_METHOD_TX_ROLLBACK_OK     (31)

#define MQTT_FRAME_ERROR_COMMAND_INVALID     (503)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *              MQTT Type Definitions
 ******************************************************/

typedef uint8_t mqtt_bit_t;

typedef enum mqtt_qos_s
{
    MQTT_QOS_DELIVER_AT_MOST_ONCE   =  0x00,
    MQTT_QOS_DELIVER_AT_LEAST_ONCE  =  0x01,
    MQTT_QOS_DELIVER_EXACTLY_ONCE   =  0x02,
    MQTT_QOS_DELIVER_FAILURE        =  0x80
} mqtt_qos_t;

typedef enum mqtt_return_code_e
{
    WICED_MQTT_RETURN_CODE_ACCEPTED   = 0,
    WICED_MQTT_RETURN_CODE_UNACCEPTED_CONTROL_VERSION   = 1,
    WICED_MQTT_RETURN_CODE_IDENTIFIER_REJECTED   = 2,
    WICED_MQTT_RETURN_CODE_SERVER_BUSY   = 3,
    WICED_MQTT_RETURN_CODE_BAD_USERNAME_PASSWORD   = 4,
    WICED_MQTT_RETURN_CODE_UNAUTHORIZED_ACCESS   = 5,
}mqtt_return_code_t;

typedef struct mqtt_string_s
{
    uint16_t    len;
    uint8_t    *str;
} mqtt_string_t;

typedef long long mqtt_timestamp_t;

typedef enum mqtt_frame_type_e
{
    MQTT_PACKET_TYPE_CONNECT                = 1,
    MQTT_PACKET_TYPE_CONNACK                = 2,
    MQTT_PACKET_TYPE_PUBLISH                = 3,
    MQTT_PACKET_TYPE_PUBACK                 = 4,
    MQTT_PACKET_TYPE_PUBREC                 = 5,
    MQTT_PACKET_TYPE_PUBREL                 = 6,
    MQTT_PACKET_TYPE_PUBCOMP                = 7,
    MQTT_PACKET_TYPE_SUBSCRIBE              = 8,
    MQTT_PACKET_TYPE_SUBACK                 = 9,
    MQTT_PACKET_TYPE_UNSUBSCRIBE            = 10,
    MQTT_PACKET_TYPE_UNSUBACK               = 11,
    MQTT_PACKET_TYPE_PINGREQ                = 12,
    MQTT_PACKET_TYPE_PINGRESP               = 13,
    MQTT_PACKET_TYPE_DISCONNECT             = 14,
}mqtt_frame_type_t;

typedef struct mqtt_frame_s
{
    uint16_t                size;
    uint8_t                *start;
    wiced_mqtt_buffer_t     buffer;
} mqtt_frame_t;

/******************************************************
 *            Header Frame Type Definitions
 ******************************************************/
typedef struct mqtt_protocol_header_arg_s
{
    uint8_t                     major;
    uint8_t                     minor;
    uint8_t                     revision;
} mqtt_protocol_header_arg_t;

/******************************************************
 *         Connection Frame Type Definitions
 ******************************************************/
typedef struct mqtt_connect_arg_s
{
    uint8_t               mqtt_version;
    uint16_t              keep_alive;
    mqtt_bit_t            username_flag;
    mqtt_bit_t            password_flag;
    mqtt_bit_t            will_flag;
    mqtt_qos_t            will_qos;
    mqtt_bit_t            will_retain;
    mqtt_bit_t            clean_session;
    mqtt_string_t         client_id;
    mqtt_string_t         will_topic;
    mqtt_string_t         will_message;
    mqtt_string_t         password;
    mqtt_string_t         username;
} mqtt_connect_arg_t;

typedef struct mqtt_connack_arg_s
{
    mqtt_bit_t            session_present;
    mqtt_return_code_t    return_code;
} mqtt_connack_arg_t;

typedef struct mqtt_publish_arg_s
{
    uint16_t                    packet_id;
    mqtt_bit_t                  dup;
    mqtt_bit_t                  retain;
    mqtt_qos_t                  qos;
    mqtt_string_t               topic;
    uint8_t                    *data;
    uint32_t                    data_len;
    uint32_t                    total_length;
} mqtt_publish_arg_t;

typedef struct mqtt_subscribe_arg_s
{
    uint16_t                    packet_id;
    mqtt_string_t               topic_filter;
    mqtt_qos_t                  qos;
} mqtt_subscribe_arg_t;

typedef struct mqtt_unsubscribe_arg_s
{
    uint16_t                    packet_id;
    mqtt_string_t               topic_filter;
} mqtt_unsubscribe_arg_t;

typedef struct wiced_mqtt_suback_arg_s
{
    uint16_t                    packet_id;
    mqtt_return_code_t          return_code;
} wiced_mqtt_suback_arg_t;

typedef struct mqtt_unsuback_arg_s
{
    uint16_t                    packet_id;
} mqtt_unsuback_arg_t;

typedef mqtt_unsuback_arg_t mqtt_puback_arg_t;

typedef mqtt_unsuback_arg_t mqtt_pubrec_arg_t;

typedef mqtt_unsuback_arg_t mqtt_pubrel_arg_t;

typedef mqtt_unsuback_arg_t mqtt_pubcomp_arg_t;


/******************************************************
 *             Content Frame Type Definitions
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

wiced_result_t  mqtt_frame_create               ( uint16_t max_size, mqtt_frame_t *frame, mqtt_socket_t *socket );
wiced_result_t  mqtt_frame_send                 ( mqtt_frame_t *frame, mqtt_socket_t *socket );
wiced_result_t  mqtt_frame_recv                 ( wiced_mqtt_buffer_t *buffer, uint16_t buffer_data_len, void *p_user, uint32_t *size );
wiced_result_t  mqtt_frame_recv_data_fragments  ( uint8_t *buffer, uint16_t buffer_data_len, void *p_user );
wiced_result_t  mqtt_frame_delete               ( mqtt_frame_t *frame );

wiced_result_t mqtt_frame_put_connect            ( mqtt_frame_t *frame, const mqtt_connect_arg_t     *args );
wiced_result_t mqtt_frame_get_connack            ( mqtt_frame_t *frame,       mqtt_connack_arg_t     *args );
wiced_result_t mqtt_frame_put_publish            ( mqtt_frame_t *frame, const mqtt_publish_arg_t     *args );
wiced_result_t mqtt_frame_get_publish            ( mqtt_frame_t *frame,       mqtt_publish_arg_t     *args );
wiced_result_t mqtt_frame_put_puback             ( mqtt_frame_t *frame, const mqtt_puback_arg_t      *args );
wiced_result_t mqtt_frame_get_puback             ( mqtt_frame_t *frame,       mqtt_puback_arg_t      *args );
wiced_result_t mqtt_frame_put_pubrec             ( mqtt_frame_t *frame, const mqtt_pubrec_arg_t      *args );
wiced_result_t mqtt_frame_get_pubrec             ( mqtt_frame_t *frame,       mqtt_pubrec_arg_t      *args );
wiced_result_t mqtt_frame_put_pubrel             ( mqtt_frame_t *frame, const mqtt_pubrel_arg_t      *args );
wiced_result_t mqtt_frame_get_pubrel             ( mqtt_frame_t *frame,       mqtt_pubrel_arg_t      *args );
wiced_result_t mqtt_frame_put_pubcomp            ( mqtt_frame_t *frame, const mqtt_pubcomp_arg_t     *args );
wiced_result_t mqtt_frame_get_pubcomp            ( mqtt_frame_t *frame,       mqtt_pubcomp_arg_t     *args );

wiced_result_t mqtt_frame_put_subscribe          ( mqtt_frame_t *frame, const mqtt_subscribe_arg_t   *args );
wiced_result_t mqtt_frame_get_suback             ( mqtt_frame_t *frame,       wiced_mqtt_suback_arg_t      *args );
wiced_result_t mqtt_frame_put_unsubscribe        ( mqtt_frame_t *frame, const mqtt_unsubscribe_arg_t *args );
wiced_result_t mqtt_frame_get_unsuback           ( mqtt_frame_t *frame,       mqtt_unsuback_arg_t    *args );
wiced_result_t mqtt_frame_put_disconnect         ( mqtt_frame_t *frame );
wiced_result_t mqtt_frame_put_pingreq            ( mqtt_frame_t *frame );

#ifdef __cplusplus
} /* extern "C" */
#endif
