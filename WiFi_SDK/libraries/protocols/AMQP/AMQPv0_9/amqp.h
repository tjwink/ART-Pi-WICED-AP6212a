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
 *  WICED AMQP constants, data types and APIs.
 */
#pragma once

#include "wiced.h"
#include "amqp_frame.h"
#include "amqp_network.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *             Connection Configuration
 ******************************************************/
#define AMQP_CONNECTION_FRAME_MAX       (4*1024)        /* Maximum frame size for a connection          */
#define AMQP_CONNECTION_CHANNEL_MAX     (1)             /* Maximum number of channels per connection    */
#define AMQP_CONNECTION_HEARTBEAT       (5)             /* Time interval between hearbeats in seconds   */

#define AMQP_FRAME_MAX_TABLE_SIZE       (10)            /* Maximum number of Items in a table           */
/******************************************************
 *                    Constants
 ******************************************************/
#define AMQP_PROTOCOL_VERSION_MAJOR     (0)
#define AMQP_PROTOCOL_VERSION_MINOR     (9)
#define AMQP_PROTOCOL_VERSION_REVISION  (1)

#define AMQP_CONNECTION_DEFAULT_PORT    (5672)
#define AMQP_CONNECTION_SECURITY_PORT   (5671)
#define AMQP_CONNECTION_DATA_SIZE_MAX   (AMQP_CONNECTION_FRAME_MAX - 8) /* Maximum size to put in a frame */

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef enum wiced_amqp_event_e
{
    /* Send events */
    AMQP_EVENT_PROTOCOL_SEND_HEADER      =  0,
    AMQP_EVENT_CONNECTION_SEND_CLOSE     =  1,
    AMQP_EVENT_CHANNEL_SEND_OPEN         =  2,
    AMQP_EVENT_CHANNEL_SEND_FLOW         =  3,
    AMQP_EVENT_CHANNEL_SEND_CLOSE        =  4,
    AMQP_EVENT_EXCHANGE_SEND_DECLARE     =  5,
    AMQP_EVENT_EXCHANGE_SEND_DELETE      =  6,
    AMQP_EVENT_QUEUE_SEND_DECLARE        =  7,
    AMQP_EVENT_QUEUE_SEND_BIND           =  8,
    AMQP_EVENT_QUEUE_SEND_UNBIND         =  9,
    AMQP_EVENT_QUEUE_SEND_PURGE          = 10,
    AMQP_EVENT_QUEUE_SEND_DELETE         = 11,
    AMQP_EVENT_BASIC_SEND_QOS            = 12,
    AMQP_EVENT_BASIC_SEND_CONSUME        = 13,
    AMQP_EVENT_BASIC_SEND_CANCEL         = 14,
    AMQP_EVENT_BASIC_SEND_PUBLISH        = 15,
    AMQP_EVENT_BASIC_SEND_PUBLISH_OK     = 16,
    AMQP_EVENT_BASIC_SEND_GET            = 17,
    AMQP_EVENT_BASIC_SEND_ACK            = 18,
    AMQP_EVENT_BASIC_SEND_REJECT         = 19,
    AMQP_EVENT_BASIC_SEND_RECOVER_ASYNC  = 20,
    AMQP_EVENT_BASIC_SEND_RECOVER        = 21,
    AMQP_EVENT_TX_SEND_SELECT            = 22,
    AMQP_EVENT_TX_SEND_COMMIT            = 23,
    AMQP_EVENT_TX_SEND_ROLLBACK          = 24,
    AMQP_EVENT_CONTENT_SEND_HEADER       = 25,
    AMQP_EVENT_CONTENT_SEND_HEADER_OK    = 26,
    AMQP_EVENT_CONTENT_SEND_CONTENT      = 27,
    AMQP_EVENT_CONTENT_SEND_CONTENT_OK   = 28,
    AMQP_EVENT_CONNECTION_SEND_HEARTBEAT = 29,

    AMQP_EVENT_CONNECTION_ERROR          = 30,
    AMQP_EVENT_MANAGER_TICK              = 31,

    /* Receive events */
    AMQP_EVENT_CONNECTION_RECV_START     = 32,
    AMQP_EVENT_CONNECTION_RECV_TUNE      = 33,
    AMQP_EVENT_CONNECTION_RECV_OPEN_OK   = 34,
    AMQP_EVENT_CONNECTION_RECV_CLOSE     = 35,
    AMQP_EVENT_CONNECTION_RECV_CLOSE_OK  = 36,
    AMQP_EVENT_CHANNEL_RECV_OPEN_OK      = 37,
    AMQP_EVENT_CHANNEL_RECV_FLOW         = 38,
    AMQP_EVENT_CHANNEL_RECV_FLOW_OK      = 39,
    AMQP_EVENT_CHANNEL_RECV_CLOSE        = 40,
    AMQP_EVENT_CHANNEL_RECV_CLOSE_OK     = 41,
    AMQP_EVENT_EXCHANGE_RECV_DECLARE_OK  = 42,
    AMQP_EVENT_EXCHANGE_RECV_DELETE_OK   = 43,
    AMQP_EVENT_QUEUE_RECV_DECLARE_OK     = 44,
    AMQP_EVENT_QUEUE_RECV_BIND_OK        = 45,
    AMQP_EVENT_QUEUE_RECV_UNBIND_OK      = 46,
    AMQP_EVENT_QUEUE_RECV_PURGE_OK       = 47,
    AMQP_EVENT_QUEUE_RECV_DELETE_OK      = 48,
    AMQP_EVENT_BASIC_RECV_QOS_OK         = 49,
    AMQP_EVENT_BASIC_RECV_CONSUME_OK     = 50,
    AMQP_EVENT_BASIC_RECV_CANCEL_OK      = 51,
    AMQP_EVENT_BASIC_RECV_RETURN         = 52,
    AMQP_EVENT_BASIC_RECV_DELIVER        = 53,
    AMQP_EVENT_BASIC_RECV_GET_OK         = 54,
    AMQP_EVENT_BASIC_RECV_GET_EMPTY      = 55,
    AMQP_EVENT_BASIC_RECV_RECOVER_OK     = 56,
    AMQP_EVENT_TX_RECV_SELECT_OK         = 57,
    AMQP_EVENT_TX_RECV_COMMIT_OK         = 58,
    AMQP_EVENT_TX_RECV_ROLLBACK_OK       = 59,
    AMQP_EVENT_CONTENT_RECV_HEADER       = 60,
    AMQP_EVENT_CONTENT_RECV_CONTENT      = 61,
    AMQP_EVENT_CONNECTION_RECV_HEARTBEAT = 62,
    AMQP_EVENT_PROTOCOL_RECV_HEADER      = 63,

} wiced_amqp_event_t;

typedef struct wiced_amqp_heartbeat_s
{
    wiced_timed_event_t               timer;
    uint32_t                          reset_value;
    uint32_t                          step_value;
    uint32_t                          send_counter;
    uint32_t                          recv_counter;
}wiced_amqp_heartbeat_t;

typedef struct wiced_amqp_connection_s wiced_amqp_connection_t;

typedef struct wiced_amqp_callbacks_s
{
    wiced_result_t (*connection_event)( wiced_amqp_event_t event, void *arg                  , wiced_amqp_connection_t *conn );
    wiced_result_t (*channel_event)   ( wiced_amqp_event_t event, void *args, uint16_t channel, wiced_amqp_connection_t *conn );
} wiced_amqp_callbacks_t;

struct wiced_amqp_connection_s
{
    uint8_t                         is_open;
    wiced_semaphore_t               semaphore;
    wiced_amqp_socket_t             socket;
    wiced_amqp_callbacks_t          callbacks;
    wiced_amqp_heartbeat_t          heartbeat;
};

typedef wiced_amqp_socket_security_t   wiced_amqp_security_t;
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
 *               Connection API Definitions
 ******************************************************/

/** Creates and initializes a new AMQP connection
 *
 * @note : this function must be called before any call to other connection functions.
 *
 * @param address [in]   : a pointer to the broker's IP address to connect to
 * @param interface [in] : the interface to the network
 * @param callbacks [in] : a pair of callback functions ( connection call back and channel
 *                         call back). Will be called every connection or channel event respectively
 * @param conn[in]       : a connection state type. Should be passed to all later calls of AMQP
 * @param security [in]  : security parameters to be used. Includes RSA public certificate, private
 *                         key and root certificate (usually self certified).
 *                         If set to NULL, no security is going to be used.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_connection_init             ( const wiced_ip_address_t *address, wiced_interface_t interface, const wiced_amqp_callbacks_t *callbacks, wiced_amqp_connection_t *conn, const wiced_amqp_security_t *security );

/** frees and deinitializes an AMQP connection
 *
 * @note : this function must be called last, after all other calls to AMQP functions.
 * The function shoudln't be called from a call back context as it tries to destroy the network
 * thread from which callback are running.
 *
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_connection_deinit           ( wiced_amqp_connection_t *conn );

/** Starts a connection with the broker
 *
 *
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_connection_open             ( wiced_amqp_connection_t *conn );

/** Closes a connection with the broker
 *
 * @note : Can only be followed by amqp_connection_open to ropen the connectoin or
 * amqp_connection_deinit to free all connection resources.
 *
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_connection_close            ( wiced_amqp_connection_t *conn );

/** Opens an AMQP channel for data communication
 *
 * @note : should be called before any other channel method calls.
 *
 * @param channel[in]    : channel number. Numbers goes sequentially starting from 1 to 255.
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_open                ( uint16_t channel                                            , wiced_amqp_connection_t *conn );

/** Closes an AMQP channel already opened for data communication
 *
 * @note : should be called last after all other channel method calls.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_close               ( uint16_t channel                                            , wiced_amqp_connection_t *conn );

/** Enables/Disables communication on a channel opened for data communication
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP channel flow method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_flow                ( uint16_t channel, wiced_amqp_channel_flow_arg_t        *args, wiced_amqp_connection_t *conn );

/** Creates an exchange if it does not already exist
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP exchange declare method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_exchange_declare    ( uint16_t channel, wiced_amqp_exchange_declare_arg_t    *args, wiced_amqp_connection_t *conn );

/** Deletes an exchange
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP exchange delete method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_exchange_delete     ( uint16_t channel, wiced_amqp_exchange_delete_arg_t     *args, wiced_amqp_connection_t *conn );

/** Creates a queue if it does not already exist
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP queue declare method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_queue_declare       ( uint16_t channel, wiced_amqp_queue_declare_arg_t       *args, wiced_amqp_connection_t *conn );

/** Binds a queue to an exchange
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP queue bind method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_queue_bind          ( uint16_t channel, wiced_amqp_queue_bind_arg_t          *args, wiced_amqp_connection_t *conn );

/** Unbinds a queue from an exchange
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP queue unbind method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_queue_unbind        ( uint16_t channel, wiced_amqp_queue_unbind_arg_t        *args, wiced_amqp_connection_t *conn );

/** Removes all messages from a queue which are not awaiting acknowledgment.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP queue purge method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_queue_purge         ( uint16_t channel, wiced_amqp_queue_purge_arg_t         *args, wiced_amqp_connection_t *conn );

/** Deletes a queue.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP queue delete method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_queue_delete        ( uint16_t channel, wiced_amqp_queue_delete_arg_t        *args, wiced_amqp_connection_t *conn );

/** Requests a specific quality of service.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP basic qos method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_basic_qos           ( uint16_t channel, wiced_amqp_basic_qos_arg_t           *args, wiced_amqp_connection_t *conn );

/** Asks the server to start a "consumer".
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP basic consume method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_basic_consume       ( uint16_t channel, wiced_amqp_basic_consume_arg_t       *args, wiced_amqp_connection_t *conn );

/** Cancels a consumer.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP basic cacnel method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_basic_cancel        ( uint16_t channel, wiced_amqp_basic_cancel_arg_t        *args, wiced_amqp_connection_t *conn );

/** Requests direct access to a message in a queue.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP basic get method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_basic_get           ( uint16_t channel, wiced_amqp_basic_get_arg_t           *args, wiced_amqp_connection_t *conn );

/** Rejects a message.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP basic reject method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_basic_reject         ( uint16_t channel, wiced_amqp_basic_reject_arg_t       *args, wiced_amqp_connection_t *conn );

/** Asks the server to redeliver all unacknowledged messages on the channel.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP basic recover method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_basic_recover       ( uint16_t channel, wiced_amqp_basic_recover_arg_t       *args, wiced_amqp_connection_t *conn );

/** Asks the server to redeliver all unacknowledged messages on the channel.
 *
 * @note: This method in deprecated in favour of synchronous  recover
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP basic recover async method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_basic_recover_async ( uint16_t channel, wiced_amqp_basic_recover_async_arg_t *args, wiced_amqp_connection_t *conn );

/** Publishes a messge to a specific exchange.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP basic publish method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_basic_publish       ( uint16_t channel, wiced_amqp_basic_publish_arg_t       *args, wiced_amqp_connection_t *conn );

/** Acknowledges one or more messages delivered via the Deliver or Get methods.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP basic ack method arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_basic_ack           ( uint16_t channel, wiced_amqp_basic_ack_arg_t           *args, wiced_amqp_connection_t *conn );

/** Sets the channel to use standard transactions.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_tx_select           ( uint16_t channel                                            , wiced_amqp_connection_t *conn );

/** Commits all message publications and acknowledgments performed in the current transaction.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_tx_commit           ( uint16_t channel                                            , wiced_amqp_connection_t *conn );

/** Abandons all message publications and acknowledgments performed in the current transaction.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_tx_rollback         ( uint16_t channel                                            , wiced_amqp_connection_t *conn );

/** Publishes content header on a channel.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP content header arguments
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_content_put_header  ( uint16_t channel, wiced_amqp_content_header_arg_t      *args, wiced_amqp_connection_t *conn );

/** Publishes contents on a channel.
 *
 * @note: when publishing contents, a user should create a new contents frame by calling
 *        amqp_channel_content_create_frame. If the content publish is successful there is
 *        no need to free the contents frame. Otherwise a user should call
 *        amqp_channel_content_delete_frame to free a created contents frame.
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP content frame
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_content_put_content ( uint16_t channel, wiced_amqp_content_content_arg_t     *args, wiced_amqp_connection_t *conn );

/** Creates a contents frame to pass data on.
 *
 * @param channel[in]    : channel number of an already opened channel.
 * @param args[in]       : a pointer to a AMQP content frame
 * @param conn[in]       : a pointer to a connection state type.
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_content_create_frame( uint16_t channel, wiced_amqp_content_content_arg_t     *args, wiced_amqp_connection_t *conn );

/** Deletes a contents frame.
 *
 * @param args[in]       : a pointer to a AMQP content frame
 *
 * @return    Wiced Result
 */
wiced_result_t amqp_channel_content_delete_frame(                  wiced_amqp_content_content_arg_t     *args );


static inline void AMQP_SHORT_STRING( wiced_amqp_short_string_t* string_struct, const char* string )
{
    string_struct->str = (uint8_t *)string;
    string_struct->len = (uint8_t) strlen( string );
}

static inline void AMQP_LONG_STRING( wiced_amqp_long_string_t* string_struct, const char* string )
{
    string_struct->str = (uint8_t *)string;
    string_struct->len = (uint32_t) strlen( string );
}


#ifdef __cplusplus
} /* extern "C" */
#endif
