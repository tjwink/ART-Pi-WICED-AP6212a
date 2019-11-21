/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *  WICED MQTT APIs.
 */
#pragma once

#include "wiced.h"
#include "mqtt_common.h"
#include "mqtt_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/
/* NOTE : don't try to modify this value */
/* Indicate that application has to allocate memory and give the reference and this should not be in stack and application has to free it after use */
#define WICED_MQTT_OBJECT_MEMORY_SIZE_REQUIREMENT       sizeof(mqtt_connection_t)

/******************************************************
 *               Function Declarations
 * @endcond
 ******************************************************/

/*****************************************************************************/
/**
 * @defgroup mqtt       MQTT
 * @ingroup ipcoms
 *
 * Communication functions for MQTT (Message Queue Telemetry Transport)
 *
 * MQTT is a publish-subscribe-based "lightweight" messaging protocol for use on top of the TCP/IP protocol.
 * It is designed for connections with remote locations where a "small code footprint" is required or the
 * network bandwidth is limited. The publish-subscribe messaging pattern requires a message broker.
 *
 * WICED implements MQTT version 3.1.1 client library i.e. the publisher and subscriber roles need the
 * support of a public/local broker to exchange data. The MQTT library supports CONNECT, DISCONNECT,
 * SUBSCRIBE, UNSUBSCRIBE, and PUBLISH methods. The current implementation has provision for QOS 0, 1 & 2.
 * The SUBSCRIBE method is capable of subscribing to one filter at a time. The MQTT client library on WICED is
 * capable of both secure [with TLS security] and non-secure mode of connection. MQTT core library internally
 * implements keep-alive mechanism.
 *
 *  @{
 */
/*****************************************************************************/

/** Initializes MQTT object
 *
 * @param[in] mqtt_obj          : Contains address of a memory location, having size of WICED_MQTT_OBJECT_MEMORY_SIZE_REQUIREMENT bytes
 *                                Application has to allocate it non stack memory area. And application has to free it after use
 * @return @ref wiced_result_t
 * NOTE :  The mqtt_obj memory here can be freed or reused by application after calling wiced_mqtt_deinit()
 *
 */
wiced_result_t wiced_mqtt_init( wiced_mqtt_object_t mqtt_obj );


/** De-initializes MQTT object
 *
 * @param[in] mqtt_obj          : Contains address of a memory location which is passed during MQTT init
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_mqtt_deinit( wiced_mqtt_object_t mqtt_obj );


/** Establishes connection with MQTT broker.
 *
 * NOTE:
 *      This is an asynchronous API. Connection status will be notified using callback function.
 *      WICED_MQTT_EVENT_TYPE_CONNECTED event will be sent using callback function
 *
 * @param[in] mqtt_obj          : Contains address of a memory location which is passed during MQTT init
 * @param[in] address           : IP address of the Broker
 * @param[in] interface         : Network interface to be used for establishing connection with Broker
 * @param[in] callback          : Event callback function which is used for notifying the events from library
 * @param[in] security          : Security related information for establishing secure connection with Broker. If NULL, connection with Broker will be unsecured.
 * @param[in] conninfo          : MQTT connect message related information
 *
 * @return @ref wiced_result_t
 * NOTE: Allocate memory for conninfo->client_id, conninfo->username, conninfo->password in non-stack area.
 *       And free/resuse them after getting event WICED_MQTT_EVENT_TYPE_CONNECT_REQ_STATUS or WICED_MQTT_EVENT_TYPE_DISCONNECTED
 */
wiced_result_t wiced_mqtt_connect( wiced_mqtt_object_t mqtt_obj, wiced_ip_address_t *address, wiced_interface_t interface, wiced_mqtt_callback_t callback, wiced_mqtt_security_t *security, wiced_mqtt_pkt_connect_t *conninfo );


/** Disconnect from MQTT broker.
 *
 * NOTE:
 *      This is an asynchronous API. Disconnect status will be notified using using callback function.
 *      WICED_MQTT_EVENT_TYPE_DISCONNECTED event will be sent using callback function
 *
 * @param[in] mqtt_obj          : Contains address of a memory location which is passed during MQTT init
 *
 * @return @ref wiced_result_t
 *
 * NOTE: Allocate memory for conninfo->client_id, conninfo->username, conninfo->password in non-stack area.
 *       And free/resuse them after getting event WICED_MQTT_EVENT_TYPE_CONNECT_REQ_STATUS or WICED_MQTT_EVENT_TYPE_DISCONNECTED
 *
 */
wiced_result_t wiced_mqtt_disconnect( wiced_mqtt_object_t mqtt_obj );


/** Publish message to MQTT Broker on the given Topic
 *
 * NOTE:
 *      This is an asynchronous API. Publish status will be notified using using callback function.
 *      WICED_MQTT_EVENT_TYPE_PUBLISHED event will be sent using callback function
 *
 * @param[in] mqtt_obj          : Contains address of a memory location which is passed during MQTT init
 * @param[in] topic             : Contains the topic on which the message to be published
 * @param[in] message           : Pointer to the message to be published
 * @param[in] msg_len           : Length of the message pointed by 'message' pointer
 * @param[in] qos               : QoS level to be used for publishing the given message
 *
 * @return wiced_mqtt_msgid_t   : ID for the message being published
 * NOTE: Allocate memory for topic, data in non-stack area.
 *       And free/resuse them after getting event WICED_MQTT_EVENT_TYPE_PUBLISHED
 *       or WICED_MQTT_EVENT_TYPE_DISCONNECTED for given message ID (wiced_mqtt_msgid_t)
 */
wiced_mqtt_msgid_t wiced_mqtt_publish( wiced_mqtt_object_t mqtt_obj, char *topic, uint8_t *data, uint32_t data_len, uint8_t qos );


/** Subscribe for a topic with MQTT Broker
 *
 * NOTE:
 *      This is an asynchronous API. Subscribe status will be notified using using callback function.
 *      WICED_MQTT_EVENT_TYPE_SUBCRIBED event will be sent using callback function
 *
 * @param[in] mqtt_obj          : Contains address of a memory location which is passed during MQTT init
 * @param[in] topic             : Contains the topic to be subscribed to
 * @param[in] qos               : QoS level to be used for receiving the message on the given topic
 *
 * @return wiced_mqtt_msgid_t   : ID for the message being subscribed
 * NOTE: Allocate memory for topic in non-stack area.
 *       And free/resuse them after getting event WICED_MQTT_EVENT_TYPE_SUBCRIBED
 *       or WICED_MQTT_EVENT_TYPE_DISCONNECTED for given message ID (wiced_mqtt_msgid_t)
 */
wiced_mqtt_msgid_t wiced_mqtt_subscribe( wiced_mqtt_object_t mqtt_obj, char *topic, uint8_t qos );


/** Unsubscribe the topic from MQTT Broker
 *
 * NOTE:
 *      This is an asynchronous API. Unsubscribe status will be notified using using callback function.
 *      WICED_MQTT_EVENT_TYPE_UNSUBCRIBED event will be sent using callback function
 *
 * @param[in] mqtt_obj          : Contains address of a memory location which is passed during MQTT init
 * @param[in] topic             : Contains the topic to be unsubscribed
 *
 * @return wiced_mqtt_msgid_t   : ID for the message being subscribed
 * NOTE: Allocate memory for topic in non-stack area.
 *       And free/resuse them after getting event WICED_MQTT_EVENT_TYPE_UNSUBSCRIBED
 *       or WICED_MQTT_EVENT_TYPE_DISCONNECTED for given message ID (wiced_mqtt_msgid_t)
 */
wiced_mqtt_msgid_t wiced_mqtt_unsubscribe( wiced_mqtt_object_t mqtt_obj, char *topic );

#ifdef __cplusplus
} /* extern "C" */
#endif
