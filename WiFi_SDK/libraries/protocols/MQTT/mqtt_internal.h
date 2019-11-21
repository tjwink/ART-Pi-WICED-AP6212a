/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *  WICED MQTT constants, data types and APIs.
 */
#pragma once

#include "wiced.h"
#include "mqtt_api.h"
#include "mqtt_frame.h"
#include "mqtt_network.h"
#include "mqtt_session.h"


#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                    Macros
 ******************************************************/

#define MQTT_CONNECTION_FRAME_MAX                     ( 4 * 1024 )        /* Maximum frame size for a connection          */
#define MQTT_CONNECTION_DATA_SIZE_MAX                 ( MQTT_CONNECTION_FRAME_MAX - 8 ) /* Maximum size to put in a frame */
#define MQTT_THEARD_STACK_SIZE                        ( WICED_DEFAULT_APPLICATION_STACK_SIZE )
#define MQTT_MAX_HEADER_SIZE                          ( 256 )
#define CACHED_TOPIC_LENGTH_MAX                       ( 64 )

#define FRAME_OVERLAPS_NETWORK_PACKETS      2
#define FRAME_OVERLAPS_NETWORK_FRAGMENTS    1
#define FRAME_NO_OVERLAP_NETWORK_FRAGMENT   0

/*                      *
 *  BUFFER PUT MACROS   *
 *                      */
#define MQTT_BUFFER_PUT_OCTET( BUFFER, VALUE )   \
    do                                               \
    {                                                \
        uint8_t *data_po = (BUFFER)->data;           \
        *(data_po)   = (VALUE);                      \
        ((BUFFER)->data)++;                          \
    }while(0)

#define MQTT_BUFFER_PUT_SHORT( BUFFER, VALUE )                 \
    do                                                             \
    {                                                              \
        uint8_t *data_ps = (BUFFER)->data;                         \
        *(data_ps)     = (uint8_t)(((VALUE) & 0x0000FF00) >> 8);   \
        *(data_ps + 1) = (uint8_t) ((VALUE) & 0x000000FF);         \
        ((BUFFER)->data) += 2;                                     \
    }while(0)

#define MQTT_BUFFER_PUT_LONG( BUFFER, VALUE )                  \
    do                                                             \
    {                                                              \
        uint8_t *data_pl = (BUFFER)->data;                         \
        *(data_pl)     = (uint8_t)(((VALUE) & 0xFF000000) >> 24);  \
        *(data_pl + 1) = (uint8_t)(((VALUE) & 0x00FF0000) >> 16);  \
        *(data_pl + 2) = (uint8_t)(((VALUE) & 0x0000FF00) >> 8 );  \
        *(data_pl + 3) = (uint8_t) ((VALUE) & 0x000000FF);         \
        ((BUFFER)->data) += 4;                                     \
    }while(0)


#define MQTT_BUFFER_PUT_STRING( BUFFER, STRING, LEN)       \
    do                                                         \
    {                                                          \
        uint8_t *data = (BUFFER)->data;                        \
        MQTT_BUFFER_PUT_SHORT( BUFFER, LEN );                  \
        memcpy( (data) + 2, STRING, LEN);                      \
        ((BUFFER)->data) += LEN;                               \
    }while(0)

#define MQTT_BUFFER_PUT_VARIABLE_LENGTH( BUFFER, VAL, LEN) \
    do                                                         \
    {                                                          \
        uint32_t X = VAL;                                      \
        do                                                     \
        {                                                      \
            uint8_t *data = (BUFFER)->data;                    \
            (LEN)++;                                           \
            *data = (uint8_t)( X % 128 );                      \
            X     = X / 128;                                   \
            if ( X > 0 ){ *data |= 0x80;}                        \
            ((BUFFER)->data) += 1;                             \
        }  while ( X > 0 );                                    \
    }while( 0 )


#define MQTT_BUFFER_PUT_BIT( BUFFER, VALUE, BIT, INC)                                   \
    do                                                                                      \
    {                                                                                       \
        uint8_t  mask_pb = ( 1 << (BIT));                                                   \
        uint8_t *data_pb = (BUFFER)->data;                                                  \
        (*data_pb) = (uint8_t)( (VALUE) ? (*data_pb) | mask_pb : (*data_pb) & (~mask_pb) ); \
        if (INC) { ((BUFFER)->data) += 1; }                                                 \
    }while(0)



#define MQTT_BUFFER_PUT_2BIT( BUFFER, VALUE, BIT, INC)                                  \
    do                                                                                      \
    {                                                                                       \
        uint8_t  mask_pb = ( 3 << (BIT));                                                   \
        uint8_t *data_pb = (BUFFER)->data;                                                  \
        (*data_pb) = (uint8_t)( (*data_pb) & (~mask_pb) );                                  \
        (*data_pb) = (uint8_t)( (*data_pb) | ( (VALUE & 3) << (BIT) ) );                    \
        if (INC) { ((BUFFER)->data) += 1; }                                                 \
    }while(0)

#define MQTT_BUFFER_PUT_4BIT( BUFFER, VALUE, BIT, INC)                                  \
    do                                                                                      \
    {                                                                                       \
        uint8_t  mask_pb = ( 0xF << (BIT));                                                 \
        uint8_t *data_pb = (BUFFER)->data;                                                  \
        (*data_pb) = (uint8_t)( (*data_pb) & (~mask_pb) );                                  \
        (*data_pb) = (uint8_t)( (*data_pb) | ( (VALUE & 0xF) << (BIT) ) );                  \
        if (INC) { ((BUFFER)->data) += 1; }                                                 \
    }while(0)

/*                      *
 *  BUFFER GET MACROS   *
 *                      */
#define MQTT_BUFFER_GET_OCTET( BUFFER, VALUE )   \
    do                                               \
    {                                                \
        uint8_t *data_go = (BUFFER)->data;           \
        (VALUE) = *(data_go);                        \
        ((BUFFER)->data)++;                          \
    }while(0)

#define MQTT_BUFFER_GET_SHORT( BUFFER, VALUE )        \
    do                                                    \
    {                                                     \
        uint8_t *data_gs = (BUFFER)->data;                \
        VALUE = (uint16_t)(         (*(data_gs    ) << 8)); \
        VALUE = (uint16_t)( VALUE + (*(data_gs + 1)     )); \
        ((BUFFER)->data) += 2;                            \
    }while(0)

#define MQTT_BUFFER_GET_LONG( BUFFER, VALUE )               \
    do                                                          \
    {                                                           \
        uint8_t *data_gl = (BUFFER)->data;                      \
        (VALUE) = (uint32_t) (                    (((uint32_t)*(data_gl    )) << 24));    \
        (VALUE) = (uint32_t) ((uint32_t)(VALUE) + (((uint32_t)*(data_gl + 1)) << 16));  \
        (VALUE) = (uint32_t) ((uint32_t)(VALUE) + (((uint32_t)*(data_gl + 2)) << 8 ));  \
        (VALUE) = (uint32_t) ((uint32_t)(VALUE) + (((uint32_t)*(data_gl + 3))      ));         \
        ((BUFFER)->data) += 4;                                  \
    }while(0)

#define MQTT_BUFFER_GET_STRING( BUFFER, STRING, LEN)       \
    do                                                         \
    {                                                          \
        MQTT_BUFFER_GET_SHORT( (BUFFER), (LEN) );              \
        (STRING) = (BUFFER)->data;                             \
        (BUFFER)->data += (LEN);                               \
    }while(0)


#define MQTT_BUFFER_GET_VARIABLE_LENGTH( BUFFER, VALUE )   \
    do                                                         \
    {                                                          \
        uint8_t *data_gb;                                      \
        uint32_t multiplier = 1;                               \
        VALUE = 0;                                             \
        do                                                     \
        {                                                      \
            data_gb = (BUFFER)->data++;                        \
            VALUE += ((*data_gb) & 127 ) * multiplier;         \
            multiplier *= 128;                                 \
        } while (((*data_gb) & 128) != 0);                     \
    }while(0)


#define MQTT_BUFFER_GET_BIT( BUFFER, VALUE, BIT, INC)      \
    do                                                         \
    {                                                          \
        uint8_t  mask_gb = ( 1 << (BIT));                      \
        uint8_t *data_gb = (BUFFER)->data;                     \
        (VALUE) = *data_gb & mask_gb;                          \
        (VALUE) = (VALUE) ? 1 : 0;                             \
        if (INC) { ((BUFFER)->data) += 1; }                    \
    }while(0)

#define MQTT_BUFFER_GET_2BIT( BUFFER, VALUE, BIT, INC)     \
    do                                                         \
    {                                                          \
        uint8_t  mask_gb = ( 3 << (BIT));                      \
        uint8_t *data_gb = (BUFFER)->data;                     \
        (VALUE) = *data_gb & mask_gb;                          \
        (VALUE) = (VALUE) >> (BIT);                            \
        if (INC) { ((BUFFER)->data) += 1; }                    \
    }while(0)

#define MQTT_BUFFER_GET_4BIT( BUFFER, VALUE, BIT, INC)     \
    do                                                         \
    {                                                          \
        uint8_t  mask_gb = ( 0xF << (BIT));                    \
        uint8_t *data_gb = (BUFFER)->data;                     \
        (VALUE) = *data_gb & mask_gb;                          \
        (VALUE) = (VALUE) >> (BIT);                            \
        if (INC) { ((BUFFER)->data) += 1; }                    \
    }while(0)

/******************************************************
 *                   Enumerations
 ******************************************************/
/**
 * Defines the network connection status
 */
typedef enum mqtt_network_status_s
{
    MQTT_NETWORK_DISCONNECTED       = 0x00,
    MQTT_NETWORK_CONNECTED          = 0x01,
}mqtt_network_status_t;

typedef enum mqtt_event_e
{
    MQTT_EVENT_SEND_CONNECT              = 1,
    MQTT_EVENT_RECV_CONNACK              = 2,
    MQTT_EVENT_SEND_DISCONNECT           = 3,
    MQTT_EVENT_CONNECTION_CLOSE          = 4,
    MQTT_EVENT_SEND_SUBSCRIBE            = 5,
    MQTT_EVENT_RECV_SUBACK               = 6,
    MQTT_EVENT_SEND_UNSUBSCRIBE          = 7,
    MQTT_EVENT_RECV_UNSUBACK             = 8,
    MQTT_EVENT_SEND_PUBLISH              = 9,
    MQTT_EVENT_RECV_PUBACK               = 10,
    MQTT_EVENT_RECV_PUBLISH              = 11,
    MQTT_EVENT_SEND_PUBACK               = 12,
    MQTT_EVENT_RECV_PUBREC               = 13,
    MQTT_EVENT_SEND_PUBREC               = 14,
    MQTT_EVENT_RECV_PUBREL               = 15,
    MQTT_EVENT_SEND_PUBREL               = 16,
    MQTT_EVENT_RECV_PUBCOMP              = 17,
    MQTT_EVENT_SEND_PUBCOMP              = 18,
    MQTT_EVENT_RECV_PINGRES              = 20,
    MQTT_EVENT_SEND_PINGREQ              = 21,
    MQTT_EVENT_TICK                      = 22,
} mqtt_event_t;

typedef enum
{
    MQTT_RECEIVE_EVENT,
    MQTT_SEND_EVENT,
    MQTT_ERROR_EVENT,
    MQTT_DISCONNECT_EVENT
} mqtt_main_event_t;

/******************************************************
 *                   Typedefs
 ******************************************************/

/* Segmented parts of MQTT frame will not come with a valid header/topic-string etc.
 * Hence, need to cache header details. Library will use these to provide application
 * enough context(which topic/header/packet-id )for the segmented data.
 */
typedef struct mqtt_cached_frame_header
{
    uint8_t     topic_str[CACHED_TOPIC_LENGTH_MAX];
    uint8_t     topic_len;
    uint32_t    total_length;
    uint8_t     qos;
    uint16_t    packet_id;
} mqtt_cached_frame_header_t;

typedef struct mqtt_heartbeat_s
{
    wiced_timed_event_t               timer;
    uint32_t                          reset_value;
    uint32_t                          step_value;
    uint32_t                          send_counter;
    uint32_t                          recv_counter;
} mqtt_heartbeat_t;

/**
 * This structure is a TCP packet wrapper for a MQTT cached frame(which is spread over multiple TCP packets)
 */
typedef struct mqtt_frame_tcp_wrapper_s
{
    uint32_t                total_bytes;
    uint32_t                consumed_bytes;
    uint32_t                remaining_bytes;
    wiced_packet_t*         packet;
    uint8_t*                data_start;
    uint16_t                frag_length;
} mqtt_frame_tcp_wrapper_t;

/**
 * Represents a cached frame tied to an MQTT connection. If a MQTT frame is segmented
 * across multiple TCP packets, all the details can be found here. This is NOT an MQTT
 * frame itself but a structure which captures all the necessary details related to an MQTT
 * frame - later used by the library as and when rest of the TCP packets arrive.
 */
typedef struct mqtt_cached_frame_s
{
    mqtt_cached_frame_header_t      header;             /* Cache the header details arrived in first few bytes */
    mqtt_frame_tcp_wrapper_t*       current_packet;     /* Wrapper for current TCP packet */
    uint32_t                        data_pending;       /* number of bytes 'yet' to be received */
    uint32_t                        total_bytes;        /* Total number of bytes to be received */
} mqtt_cached_frame_t;

/**
 *  IMPORTANT:
 *       Any change made to the size of this structure should be reflected
 *       in the macro WICED_MQTT_OBJECT_MEMORY_SIZE_REQUIREMENT defined in MQTT API header file
 */
typedef struct mqtt_connection_s
{
    uint8_t                         session_init;
    uint8_t                         net_init_ok;
    uint16_t                        packet_id;
    mqtt_socket_t                   socket;
    wiced_mqtt_callback_t           callbacks;
    mqtt_heartbeat_t                heartbeat;
    mqtt_session_t*                 session;
    uint8_t*                        peer_cn;
    mqtt_network_status_t           network_status;
    wiced_bool_t                    core_init;
    wiced_mutex_t                   lock;
    mqtt_cached_frame_t             cached_frame;
} mqtt_connection_t;

typedef struct mqtt_send_context_t
{
        mqtt_event_t event_t;
        mqtt_connection_t *conn;
        union
        {
                mqtt_connect_arg_t conn_args;
                mqtt_publish_arg_t pub_args;
                mqtt_subscribe_arg_t sub_args;
                mqtt_unsubscribe_arg_t unsub_args;
        } args;

}mqtt_send_context;

typedef struct
{
    mqtt_main_event_t  event_type;
    mqtt_send_context  send_context;
} mqtt_event_message_t;

typedef struct
{
    uint8_t* data;
} mqtt_raw_buffer_t;

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
 *                API Definitions
 ******************************************************/
wiced_result_t mqtt_connection_init( const wiced_ip_address_t *address, uint16_t port_number, wiced_interface_t interface, const wiced_mqtt_callback_t callbacks, mqtt_connection_t *conn, const wiced_mqtt_security_t *security );
wiced_result_t mqtt_connection_deinit( mqtt_connection_t *conn );
wiced_result_t mqtt_connect( mqtt_connection_t *conn, mqtt_connect_arg_t *args, mqtt_session_t *session );
wiced_result_t mqtt_disconnect( mqtt_connection_t *conn );
wiced_result_t mqtt_publish( mqtt_connection_t *conn, mqtt_publish_arg_t *args );
wiced_result_t mqtt_subscribe( mqtt_connection_t *conn, mqtt_subscribe_arg_t *args );
wiced_result_t mqtt_unsubscribe( mqtt_connection_t *conn, mqtt_unsubscribe_arg_t *args );
wiced_result_t mqtt_core_init( mqtt_connection_t *conn );
wiced_result_t mqtt_core_deinit( mqtt_connection_t *conn );



#ifdef __cplusplus
} /* extern "C" */
#endif
