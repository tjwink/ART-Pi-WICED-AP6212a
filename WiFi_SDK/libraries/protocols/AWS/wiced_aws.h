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
 *  WICED Amazon Web Service APIs & data-structures
 */
#pragma once

#include "wiced_tcpip.h"
#include "wiced_result.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/*****************************************************************************/
/**
 *
 * @defgroup wiced_aws          Wiced Amazon Web Service Library
 * @ingroup  ipcoms
 *
 * Communication functions for AWS IoT & AWS Greengrass services
 *
 * AWS IoT provides secure, bi-directional communication between Internet-connected devices such as sensors,
 * actuators, embedded micro-controllers, or smart appliances and the AWS Cloud.
 *
 * AWS Greengrass is software that extends AWS cloud capabilities to local devices(typically Edge or Gateway devices),
 * making it possible for them to collect and analyze data closer to the source of information (Nodes, Wiced Devices, Amazon FreeRTOS devices).
 * With AWS Greengrass, devices securely communicate on a local network and exchange messages with each other
 * without having to connect to the cloud. AWS Greengrass provides a local pub/sub message manager that can
 * intelligently buffer messages if connectivity is lost so that inbound and outbound messages to the cloud are preserved.
 *
 * Wiced AWS library provides application developers an easy-to-use, unified interface for quickly enabling
 * AWS communication in their applications. The library provides a single interface to communicate with AWS
 * using different protocols. Currently, only MQTT & HTTP ( using Client Certificates )
 * are supported. See <https://docs.aws.amazon.com/iot/latest/developerguide/protocols.html> for more details.
 *
 *  @{
 */
/*****************************************************************************/

/**
 * List of AWS IoT protocols
 */

typedef enum
{
    WICED_AWS_TRANSPORT_MQTT_NATIVE = 0,                /**< MQTT-native i.e. MQTT over TCP sockets */
    WICED_AWS_TRANSPORT_MQTT_WEBSOCKET,                 /**< MQTT over Websockets */
    WICED_AWS_TRANSPORT_RESTFUL_HTTPS,                  /**< AWS RESTful HTTPS APIs */
    WICED_AWS_TRANSPORT_INVALID,                        /**< Invalid transport type */
} wiced_aws_transport_type_t;

/**
 * QoS types for publish/subscribe messages over MQTT
 */
typedef enum
{
    WICED_AWS_QOS_ATMOST_ONCE    = 0x00,                /**< QoS level 0 */
    WICED_AWS_QOS_ATLEAST_ONCE   = 0x01,                /**< QoS level 1 */
    WICED_AWS_QOS_EXACTLY_ONCE   = 0x02,                /**< QoS level 2 */
    WICED_AWS_QOS_INVALID        = 0x80,                /**< Invalid QoS level */
} wiced_aws_qos_level_t;

/**
 * List of AWS events
 */
typedef enum
{
    WICED_AWS_EVENT_CONNECTED,                          /**< The connection has been accepted by remote */
    WICED_AWS_EVENT_DISCONNECTED,                       /**< Disconnection received from peer or some other network failure */
    WICED_AWS_EVENT_PUBLISHED ,                         /**< Publication event; Either accepted by remote( received an acknowledgment ) or timed-out */
    WICED_AWS_EVENT_SUBSCRIBED,                         /**< Subscription event; Either acknowledged by remote or timed-out */
    WICED_AWS_EVENT_UNSUBSCRIBED,                       /**< Unsubscription event; acknowledged by remote or timed-out */
    WICED_AWS_EVENT_PAYLOAD_RECEIVED,                   /**< Data received event */
}  wiced_aws_event_type_t;

/*******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    uint8_t*                                    private_key;            /**< AWS Thing's private key */
    uint16_t                                    key_length;             /**< Private key length */
    uint8_t*                                    certificate;            /**< AWS Thing's certificate to be used by by brokers to verify your credentials */
    uint16_t                                    certificate_length;     /**< Certificate length */
} wiced_aws_thing_security_info_t;

typedef struct
{
    wiced_aws_transport_type_t                  transport;              /**< Transport type */
    char*                                       uri;                    /**< Connection-ID string for the remote peer */
    char*                                       peer_common_name;       /**< Remote Peer's common name to be used during handshake */
    wiced_ip_address_t                          ip_addr;                /**< IP-address of the remote */
    int                                         port;                   /**< Port-number on which it is looking for the connection */
    uint8_t*                                    root_ca_certificate;    /**< Root CA certificate for this endpoint */
    uint16_t                                    root_ca_length;         /**< Root CA certificate length */
} wiced_aws_endpoint_info_t;

typedef struct
{
    const char*                                 name;                   /**< Name of 'Thing' */
    wiced_aws_thing_security_info_t*            credentials;            /**< Thing's Security credentials */
} wiced_aws_thing_info_t;


/**
 *  Connection endpoint information of a Greengrass core device. A greengrass core can have one or more endpoints.
 */
typedef struct
{
    char*                                       metadata;               /**< Any metadata associated with this Connection - Example - Interface name etc. */
    char*                                       ip_address;             /**< IP-Address of the Core */
    char*                                       port;                   /**< Port number of the Core */
} wiced_aws_greengrass_core_connection_info_t;

/**
 * Node Wrapper structure for Connection Information of a greengrass core.
 */
typedef struct
{
    wiced_aws_greengrass_core_connection_info_t info;
    linked_list_node_t                          node;
} wiced_aws_greengrass_core_connection_t;

/**
 * Greengrass Core Information retrieved from AWS. A device can be part of multiple groups and can select
 * to which Group's core it wants to connect to.
 * Note: Each group have only one core.
 */
typedef struct
{
    char*                                       group_id;               /**< Group-ID */
    char*                                       thing_arn;              /**< Amazon resource name of the'Core' device of this Group */
    char*                                       root_ca_certificate;    /**< Root CA certificate for this 'Core' */
    uint16_t                                    root_ca_length;         /**< Length of the certificate */
    linked_list_t                               connections;            /**< A linked-list to store all Connection endpoints( @ref wiced_aws_greengrass_core_connection_t ) available for this core.
                                                                             For example: A core can have multiple network Interfaces. */
} wiced_aws_greengrass_core_info_t;

/**
 * Node Wrapper structure for a greengrass core Information
 */

typedef struct
{
    wiced_aws_greengrass_core_info_t            info;
    linked_list_node_t                          node;
} wiced_aws_greengrass_core_t;

/** @} */

/**
 * @addtogroup wiced_aws Greengrass
 *
 * Wiced Amazon Web services library - A one-stop library for communicating with AWS services( AWS IoT or AWS Greengrass ).
 *
 * Discovery callback returns a list of all Groups the thing is part of along with Connection Informations of the cores of these groups.
 * Application upon receiving this information can select which group/core it wants to connect to.
 *
 * Note that each Greengrass group can have only one Greengrass core.
 * However, each greengrass core can have one or more Connections endpoints.
 *                             Greengrass Discovery Payload architecture
 *                             =========================================
 *
 *
 *
 *             groups
 *               +
 *               |
 *               |     +--------+---------+-------+-----------+----+      +--------+---------+-------+-----------+----+
 *               |     |        |         |       |           |    |      |        |         |       |           |    |
 *               +---->|Group-ID|Thing ARN|Root CA|connections|next+----->|Group-ID|Thing ARN|Root CA|connections|next+----->NULL
 *                     |        |         |       |           |    |      |        |         |       |           |    |
 *                     +--------+---------+-------+----+------+----+      +--------+---------+-------+----+------+----+
 *                                                     |
 *                                                     |
 *                                                     |
 *                                                     v
 *                                                   +----------+-----------+--------+----+      +----------+-----------+--------+----+
 *                                                   |          |           |        |    |      |          |           |        |    |
 *                                                   |IP-address|Port-Number|Metadata|next+----->|IP-address|Port-Number|Metadata|next+----->NULL
 *                                                   |          |           |        |    |      |          |           |        |    |
 *                                                   +----------+-----------+--------+----+      +----------+-----------+--------+----+
 *
 *  @{
 */

typedef struct
{
    linked_list_t* groups;
} wiced_aws_greengrass_discovery_callback_data_t;

typedef struct
{
    wiced_result_t status;
} wiced_aws_callback_connection_data_t;

typedef struct
{
    wiced_result_t status;
}wiced_aws_callback_disconnection_data_t;

typedef struct
{
    wiced_result_t status;
} wiced_aws_callback_publish_data_t;

typedef struct
{
    wiced_result_t status;
} wiced_aws_callback_subscribe_data_t;

typedef struct
{
    wiced_result_t status;
} wiced_aws_callback_unsubscribe_data_t;

typedef struct
{
    wiced_result_t                              status;
    uint8_t*                                    data;
    uint32_t                                    data_length;
    uint8_t*                                    topic;
    uint32_t                                    topic_length;
} wiced_aws_callback_message_t;

typedef union
{
    wiced_aws_callback_connection_data_t        connection;
    wiced_aws_callback_disconnection_data_t     disconnection;
    wiced_aws_callback_publish_data_t           publish;
    wiced_aws_callback_subscribe_data_t         subscribe;
    wiced_aws_callback_unsubscribe_data_t       unsubscribe;
    wiced_aws_callback_message_t                message;
} wiced_aws_callback_data_t;

typedef uint32_t wiced_aws_handle_t;

typedef void (*wiced_aws_greengrass_callback_t)( wiced_aws_greengrass_discovery_callback_data_t* groups );

typedef void( *wiced_aws_callback_t)( wiced_aws_handle_t aws, wiced_aws_event_type_t event, wiced_aws_callback_data_t* data );

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/** Initializes Wiced AWS library
 *
 * @param[in] thing          : Device's Information like 'Thing' name and its AWS IoT security credentials(namely private key and certificate)
 *                             Each Device/Thing has a unique set of private-key & Certificate.
 *
 * @param[in] cb             : Callback to notify application of AWS events See @ref wiced_aws_event_type_t
 *
 * @return @ref wiced_result_t

 * NOTE :  Systemwide Initialization. Resources allocated as part of this API will be shared by multiple AWS connections.
 *
 */
wiced_result_t wiced_aws_init( wiced_aws_thing_info_t* thing, wiced_aws_callback_t cb );

/** Discovers Greengrass cores(groups) of which this 'Thing' is part of.
 *
 * @param[in] endpoint       : AWS IoT endpoint information - Used for establishing HTTPS connection to AWS IoT Cloud.
 *
 * @param[in] gg_cb          : Greengrass discovery payload callback - Notify application of Greengrass Cores information
 *
 * @return @ref wiced_result_t
 *
 */
wiced_result_t wiced_aws_discover( wiced_aws_endpoint_info_t* endpoint, wiced_aws_greengrass_callback_t gg_cb );

/** Creates a local connection-handle for a given endpoint information
 *
 * @param[in] endpoint        : AWS IoT or Greengrass core endpoint information.
 *
 * @return wiced_aws_handle_t : Connection handle to be used for connect/[sub|unsub]scribe/publish/disconnect operations to a particular endpoint.
 *
 */
wiced_aws_handle_t wiced_aws_create_endpoint( wiced_aws_endpoint_info_t* endpoint );

/** Establish Connection to an AWS IoT or Greengrass core
 *
 * @param[in] aws             : Connection handle
 *
 * @return @ref wiced_result_t
 *
 */
wiced_result_t wiced_aws_connect( wiced_aws_handle_t aws );

/** Publish message to remote endpoint on the given topic.
 *
 *
 * @param[in] aws             : Connection handle
 * @param[in] topic           : Contains the topic on which the message to be published
 * @param[in] message         : Pointer to the message to be published
 * @param[in] msg_len         : Length of the message pointed by 'message' pointer
 * @param[in] qos             : QoS level to be used for publishing the given message
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_aws_publish( wiced_aws_handle_t aws, char* topic, uint8_t* data, uint32_t length, wiced_aws_qos_level_t qos );

/** Subscribes for a topic.
 *
 *
 * @param[in] aws             : Connection handle
 * @param[in] topic           : Contains the topic to be subscribed to
 * @param[in] qos             : QoS level to be used for receiving the message on the given topic
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_aws_subscribe( wiced_aws_handle_t aws, char* topic, wiced_aws_qos_level_t qos );

/** Unsubscribe the topic
 *
 *
 * @param[in] aws             : Connection handle
 * @param[in] topic           : Contains the topic to be unsubscribed
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_aws_unsubscribe( wiced_aws_handle_t aws, char* topic );

/** Disconnects AWS connection and free up the resources.
 *
 * @param[in] aws             : Connection handle
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_aws_disconnect( wiced_aws_handle_t aws );

/** Deinitializes Wiced AWS library. Free up all the allocated resources.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_aws_deinit( void );

/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif
