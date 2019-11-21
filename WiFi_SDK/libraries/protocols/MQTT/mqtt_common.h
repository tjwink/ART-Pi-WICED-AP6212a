/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *  WICED MQTT constants, data types which are common to API and library
 */
#pragma once

#include "wiced.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define WICED_MQTT_QUEUE_SIZE                           (10)
#define WICED_MQTT_CONNECTION_DEFAULT_PORT              (1883)          /* Unsecure port based on broker */
#define WICED_MQTT_CONNECTION_SECURE_PORT               (8883)          /* secure port based on broker */
#define WICED_MQTT_PROTOCOL_VER4                        (4)             /* Mqtt protocol version 4 */
#define WICED_MQTT_PROTOCOL_VER3                        (3)             /* Mqtt protocol version 3 */
#define WICED_MQTT_CONNECTION_TIMEOUT                   (5000)          /* Tcp connection timeout */
#define WICED_MQTT_CONNECTION_NUMBER_OF_RETRIES         (3)             /* Tcp connection retries */
/******************************************************
 *                  typedef Enumerations
 ******************************************************/
/**
 * Defines the QoS levels supported by MQTT library
 */
typedef enum wiced_mqtt_qos_level_s
{
    WICED_MQTT_QOS_DELIVER_AT_MOST_ONCE   = 0x00,               /* QoS level 0 */
    WICED_MQTT_QOS_DELIVER_AT_LEAST_ONCE  = 0x01,               /* QoS level 1 */
    WICED_MQTT_QOS_DELIVER_EXACTLY_ONCE   = 0x02,               /* QoS level 2 */
    WICED_MQTT_QOS_DELIVER_FAILURE        = 0x80,               /* Invalid QoS level */
} wiced_mqtt_qos_level_t;

/**
 * Defines MQTT event types received in call-back function
 */
typedef enum wiced_mqtt_event_s
{
    WICED_MQTT_EVENT_TYPE_CONNECT_REQ_STATUS = 1,               /* Event sent when broker accepts CONNECT request */
    WICED_MQTT_EVENT_TYPE_DISCONNECTED,                         /* Event sent when broker accepts DISCONNECT request or when there is any network issue */
    WICED_MQTT_EVENT_TYPE_PUBLISHED,                            /* Event sent for QOS-1 and QOS-2 for the published when successfully delivered. No event will be sent for QOS-0. */
    WICED_MQTT_EVENT_TYPE_SUBCRIBED,                            /* Event sent when broker accepts SUBSCRIBED request */
    WICED_MQTT_EVENT_TYPE_UNSUBSCRIBED,                         /* Event sent when broker accepts UNSUBSCRIBED request */
    WICED_MQTT_EVENT_TYPE_PUBLISH_MSG_RECEIVED,                 /* Event sent when PUBLISH message is received from the broker for a subscribed topic */
    WICED_MQTT_EVENT_TYPE_UNKNOWN                               /* Event type not known */
} wiced_mqtt_event_type_t;

/**
 * List of error codes returned by MQTT library
 */
typedef enum wiced_mqtt_err_code_s
{
    WICED_MQTT_CONN_ERR_CODE_NONE = 0,                          /* Success */
    WICED_MQTT_CONN_ERR_CODE_UNACCEPTABLE_PROTOCOL_VERSION,     /* Protocol version is not acceptable by MQTT Broker */
    WICED_MQTT_CONN_ERR_CODE_IDENTIFIER_REJECTED,               /* Client ID rejected by MQTT Broker */
    WICED_MQTT_CONN_ERR_CODE_SERVER_UNAVAILABLE,                /* MQTT Broker unavailable */
    WICED_MQTT_CONN_ERR_CODE_BAD_USER_NAME_OR_PASSWORD,         /* Bad user name or password */
    WICED_MQTT_CONN_ERR_CODE_NOT_AUTHORIZED,                    /* Client not authorized to connect */
    WICED_MQTT_CONN_ERR_CODE_INVALID                            /* Invalid error code */
} wiced_mqtt_conn_err_code_t;

/******************************************************
 *              MQTT Type Definitions
 ******************************************************/
/**
 * Pointer to MQTT object
 */
typedef void* wiced_mqtt_object_t;

/**
 * Message ID for the given MQTT message
 */
typedef uint16_t wiced_mqtt_msgid_t;

/**
 * Contains the message received for a topic from the Broker
 * For MQTT message fragmented across multiple TCP packets, MQTT library will send
 * the chunks of 'MQTT message' as soon as they arrive. It will be notified to the application
 * using 'chunked' field and user can keep track of message received using 'data-len'
 * 'total-length' fields described in below structure.
 *
 * For MQTT message/frames receivd in one or less TCP packet, 'data_len' will be same as 'total_length'
 * and 'chunked' field will be set to 0.
 */
typedef struct wiced_mqtt_topic_msg_s
{
    uint8_t*    topic;                                          /* Name of the topic associated with the message. It's not 'null' terminated */
    uint32_t    topic_len;                                      /* Length of the topic */
    uint8_t     chunked;                                        /* Signal the application that message payload will arrive in chunks */
    uint8_t*    data;                                           /* Payload of the message */
    uint32_t    data_len;                                       /* Length of the message payload; if chunked = 1, it refers to the size of that chunk */
    uint32_t    total_length;                                   /* Total length of message payload; Relevant only when chunked = 1 */
} wiced_mqtt_topic_msg_t;

/* MQTT Event info */
typedef struct wiced_mqtt_event_info_s
{
    wiced_mqtt_event_type_t             type;                   /* Message event type */
    union
    {
        wiced_mqtt_conn_err_code_t      err_code;               /* Valid only for WICED_MQTT_EVENT_TYPE_CONNECT_REQ_STATUS event. Indicates the error identified while connecting to Broker */
        wiced_mqtt_msgid_t              msgid;                  /* Valid only for WICED_MQTT_EVENT_TYPE_PUBLISHED, WICED_MQTT_EVENT_TYPE_SUBCRIBED, WICED_MQTT_EVENT_TYPE_UNSUBSCRIBED events. Indicates message ID */
        wiced_mqtt_topic_msg_t          pub_recvd;              /* Valid only for WICED_MQTT_EVENT_TYPE_PUBLISH_MSG_RECEIVED event. Indicates the message received from Broker */
    }                                   data;                   /* Event data */
} wiced_mqtt_event_info_t;

/**
 * Contains information related to establishing connection with Broker
 */
typedef struct wiced_mqtt_pkt_connect_s
{
    uint16_t    port_number;                                    /* Indicates mqtt broker port number to which publisher/subscriber want to communicate, '0' indicates,library to take default values( 1883 as open port, 8883 as secure port) */
    uint8_t     mqtt_version;                                   /* Indicates mqtt version number. Supported versions are 3 and 4. Any value other than 4 will be treated as 3 (default)*/
    uint16_t    keep_alive;                                     /* Indicates keep alive interval to Broker */
    uint8_t     clean_session;                                  /* Indicates if the session to be cleanly started */
    uint8_t*    client_id;                                      /* Client ID */
    uint8_t*    username;                                       /* User name to connect to Broker */
    uint8_t*    password;                                       /* Password to connect to Broker */
    uint8_t*    peer_cn;
} wiced_mqtt_pkt_connect_t;

typedef struct wiced_mqtt_security_s
{
     char*      ca_cert;                                        /* CA certificate, common between client and MQTT Broker */
     uint32_t   ca_cert_len;                                    /* CA certificate length */
     char*      cert;                                           /* Client certificate in PEM format */
     uint32_t   cert_len;                                       /* Client certificate length */
     char*      key;                                            /* Client private key */
     uint32_t   key_len;                                        /* Client private key length */
} wiced_mqtt_security_t;

/** Call-back function for MQTT events
 *
 * @param[in] mqtt_obj          : Contains address of a memory location which is passed during MQTT init
 * @param[in] event             : Pointer to event structure which contains the details about the event
 *
 * @return @ref wiced_result_t
 */
typedef wiced_result_t (*wiced_mqtt_callback_t)( wiced_mqtt_object_t mqtt_object, wiced_mqtt_event_info_t *event );

#ifdef __cplusplus
} /* extern "C" */
#endif
