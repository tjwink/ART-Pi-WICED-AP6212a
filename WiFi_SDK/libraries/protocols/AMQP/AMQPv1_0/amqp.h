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
 *  WICED AMQPv1_0 constants, data types and APIs.
 */
#pragma once

#include "wiced.h"
//TODO : need to move amqp internal header inside the library.
#include "amqp_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define WICED_AMQP_PROTOCOL_VERSION_MAJOR          (1)
#define WICED_AMQP_PROTOCOL_VERSION_MINOR          (0)
#define WICED_AMQP_PROTOCOL_VERSION_REVISION       (0)

#define WICED_AMQP_PROTOCOL_ID_OPEN                (0)
#define WICED_AMQP_PROTOCOL_ID_TLS                 (2)
#define WICED_AMQP_PROTOCOL_ID_SASL                (3)
#define WICED_AMQP_MAJOR_NUMBER                    (1)
#define WICED_AMQP_MINOR_NUMBER                    (0)
#define WICED_AMQP_REVISION_NUMBER                 (0)

#define WICED_AMQP_LINK_CREDIT                     (10)
/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef enum wiced_amqp_event_e
{
        WICED_AMQP_EVENT_HDR_SENT            =  0,
        WICED_AMQP_EVENT_HDR_RCVD            =  1,
        WICED_AMQP_EVENT_OPEN_SENT           =  2,
        WICED_AMQP_EVENT_OPEN_RCVD           =  3,
        WICED_AMQP_EVENT_BEGIN_SENT          =  4,
        WICED_AMQP_EVENT_BEGIN_RCVD          =  5,
        WICED_AMQP_EVENT_ATTACH_SENT         =  6,
        WICED_AMQP_EVENT_ATTACH_RCVD         =  7,
        WICED_AMQP_EVENT_FLOW_SENT           =  8,
        WICED_AMQP_EVENT_FLOW_RCVD           =  9,
        WICED_AMQP_EVENT_TRANSFER_SENT       = 10,
        WICED_AMQP_EVENT_TRANSFER_RCVD       = 11,
        WICED_AMQP_EVENT_DISPOSITION_SENT    = 12,
        WICED_AMQP_EVENT_DISPOSITION_RCVD    = 13,
        WICED_AMQP_EVENT_DETACH_SENT         = 14,
        WICED_AMQP_EVENT_DETACH_RCVD         = 15,
        WICED_AMQP_EVENT_END_SENT            = 16,
        WICED_AMQP_EVENT_END_RCVD            = 17,
        WICED_AMQP_EVENT_CLOSE_SENT          = 18,
        WICED_AMQP_EVENT_CLOSE_RCVD          = 19,
        WICED_AMQP_EVENT_UNKNOWN_EVENT       = 20,
        WICED_AMQP_EVENT_CONNECTION_ERROR    = 21,
        WICED_AMQP_EVENT_SASL_HDR_SENT       = 22,
        WICED_AMQP_EVENT_SASL_HDR_RCVD       = 23,
        WICED_AMQP_EVENT_SASL_SRV_MECH_RCVD  = 24,
        WICED_AMQP_EVENT_SASL_INIT_SENT      = 25,
        WICED_AMQP_EVENT_SASL_OUTCOME_RCVD   = 26
} wiced_amqp_event_t;

/* Frame type */
enum wiced_amqp_frame_types_t
{
        WICED_AMQP_FRAME_TYPE                = 0,
        WICED_AMQP_SASL_FRAME_TYPE           = 1
};

typedef enum wiced_amqp_role_types_e
{
        WICED_AMQP_ROLE_SENDER               = 0,
        WICED_AMQP_ROLE_RECEIVER             = 1
}wiced_amqp_role_types_t;

/* Frame type */
typedef enum wiced_amqp_delivery_status_types_e
{
        WICED_AMQP_DELIVERY_ACCEPTED            = 0,
        WICED_AMQP_DELIVERY_REJECTED            = 1,
        WICED_AMQP_DELIVERY_RELEASED            = 2,
        WICED_AMQP_DELIVERY_MODIFIED            = 3,
        WICED_AMQP_DELIVERY_RECEIVED            = 4
} wiced_amqp_delivery_status_types_t;

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct SASL_PLAIN_CONFIG_TAG
{
    const char* authcid;
    const char* passwd;
    const char* authzid;
} wiced_sasl_plain_config;

typedef struct wiced_amqp_connection_s wiced_amqp_connection_t;

typedef struct wiced_amqp_callbacks_s
{
        wiced_result_t (*connection_event)( wiced_amqp_event_t event, void *arg, void *conn );
} wiced_amqp_callbacks_t;

struct wiced_amqp_connection_s
{
        uint8_t is_open;
        wiced_semaphore_t semaphore;
        wiced_amqp_socket_t socket;
        wiced_amqp_callbacks_t callbacks;
        wiced_amqp_heartbeat_t heartbeat;
        sasl_client_negotiation_state sasl_state;
        uint8_t*                        peer_cn;
};

typedef wiced_amqp_socket_security_t wiced_amqp_security_t;

typedef struct wiced_sasl_mechanism_s
{
        uint8_t* item;
        uint32_t size;
} wiced_sasl_mechanism_t;
typedef struct wiced_amqp_sasl_server_mechnasims_args_s
{
        wiced_sasl_mechanism_t list[4];
        uint32_t count;
} wiced_amqp_sasl_server_mechnasims_args;

typedef struct wiced_amqp_sasl_outcome_args_s
{
        uint8_t code;
        char* additional_data;
} wiced_amqp_sasl_outcome_args;

/* OPEN related data structure information */

typedef struct wiced_amqp_open_performative_args_s
{
        uint8_t* host_name;
        uint8_t* container_id;
        uint8_t host_name_size;
        uint8_t container_id_size;
        uint32_t max_frame_size;
        uint32_t idle_timeout;
        uint16_t channel_max;
        uint16_t port_number; /* Indicates amqp broker port number to which peer want to communicate, '0' indicates,library to take default values( 5762 as open port, 5761 as secure port) */
        uint8_t* peer_cn;
} wiced_amqp_open_performative_args;

typedef struct WICED_AMQP_OPEN_INSTANCE_TAG
{
        wiced_amqp_connection_t *conn;
        wiced_amqp_protocol_header_arg_t header_args;
        uint8_t current_connection_state;
        linked_list_t channel_endpoint;
        uint32_t endpoint_count;
        uint32_t remote_max_frame_size;
        /* options */
        wiced_amqp_open_performative_args open_args;
        uint32_t remote_idle_timeout;
        uint32_t remote_channel_max;
        uint8_t  is_sasl;
        wiced_sasl_plain_config plain_config;
} wiced_amqp_connection_instance;

/* CLOSE related data structure information */

typedef struct wiced_amqp_close_performative_args_s
{
        amqp_error error;
} wiced_amqp_close_performative_args;

/* SESSION related data structure information */

typedef struct wiced_amqp_begin_performative_args_s
{
        uint16_t remote_channel;
        uint32_t next_outgoing_id;
        uint32_t next_incoming_id;
        uint32_t desired_incoming_window;
        uint32_t incoming_window;
        uint32_t outgoing_window;
        uint32_t handle_max;
        uint32_t remote_incoming_window;
        uint32_t remote_outgoing_window;
} wiced_amqp_begin_performative_args;

typedef struct wiced_amqp_disposition_performative_args_s
{
        uint32_t delivery_id;
        uint64_t delivery_tag;
        wiced_bool_t is_receiver;
        uint32_t first;
        uint32_t last;
        wiced_bool_t settled;
        wiced_amqp_delivery_status_types_t delivery_state;
} wiced_amqp_disposition_performative_args;

typedef struct wiced_amqp_session_instance_s
{
        wiced_amqp_connection_instance *connection_instance;
        amqp_channel_endpoint_t *endpoints;
        wiced_amqp_begin_performative_args begin_args;
        linked_list_t link_endpoint;
        uint8_t link_endpoint_count;
} wiced_amqp_session_instance_t;

/* LINK instance data structures */
typedef struct wiced_amqp_link_performative_args_s
{
        uint8_t* name;
        uint32_t handle;
        wiced_amqp_role_types_t role; /* WICED_TRUE = receiver(internally use 0x41) WICED_FALSE = sender(internally use 0x42) */
        uint8_t snd_settle_mode;
        uint8_t rcv_settle_mode;
        uint8_t source_size;
        uint8_t* source;
        uint8_t target_size;
        uint8_t* target;
        uint32_t initial_delivery_count;
        uint64_t max_message_size;
        uint8_t name_size;
} wiced_amqp_link_performative_args;

typedef struct WICED_AMQP_LINK_INSTANCE_TAG
{
        uint8_t link_state;
        uint8_t previous_link_state;
        uint32_t link_credit;
        uint32_t available;
        amqp_link_endpoints *link_endpoint;
        /* Maintains list of packets which are sent but not yet acknowledged [disposition] */
        linked_list_t message_delivery_list;
        uint32_t delivery_count;
        wiced_amqp_link_performative_args link_args;
        wiced_amqp_session_instance_t *session;
} wiced_amqp_link_instance;

/* Flow related data structure information */
typedef struct wiced_amqp_flow_performative_args_s
{
        uint32_t next_incoming_id;
        uint32_t incoming_window;
        uint32_t next_outgoing_id;
        uint32_t outgoing_window;
        uint32_t handle;
        uint32_t delivery_count;
        uint32_t link_credit;
        uint32_t available;
        uint8_t drain;
        uint8_t echo;
} wiced_amqp_flow_performative_args;

/* Flow instance data structure */
typedef struct WICED_AMQP_FLOW_INSTANCE_TAG
{
        wiced_amqp_flow_performative_args flow_args;
        wiced_amqp_link_instance *link_instance;
} wiced_amqp_flow_instance;

/* DETACH related data structure information */

typedef struct wiced_amqp_detach_performative_args_s
{
        uint32_t handle;
        uint8_t closed;
        amqp_error error;
        uint8_t handle_is_not_specified;
} wiced_amqp_detach_performative_args;

typedef struct WICED_AMQP_DELINK_INSTANCE_TAG
{
        wiced_amqp_detach_performative_args detach_args;
        wiced_amqp_link_instance *link;
} wiced_amqp_delink_instance;

typedef struct wiced_amqp_message_s
{
        wiced_bool_t settle;
        wiced_bool_t more;
        uint64_t delivery_tag;
        uint8_t* data;
        uint32_t data_lenth;
        uint8_t  message_format;
} wiced_amqp_message_t;

typedef struct wiced_amqp_transfer_performative_args_s
{
        wiced_amqp_message_t message;
        uint32_t handle;
        uint32_t delivery_id;
} wiced_amqp_transfer_performative_args;

/* MISC data structures */
typedef union wiced_amqp_performative_args_t
{
        wiced_amqp_open_performative_args open_args;
        wiced_amqp_begin_performative_args begin_args;
        wiced_amqp_link_performative_args link_args;
        wiced_amqp_transfer_performative_args transfer_args;
        wiced_amqp_flow_performative_args flow_args;
        wiced_amqp_disposition_performative_args disposition_args;
        wiced_amqp_detach_performative_args detach_args;
        wiced_amqp_close_performative_args close_args;
        wiced_amqp_sasl_server_mechnasims_args mech_args;
        wiced_amqp_sasl_outcome_args outcome_args;
} wiced_amqp_performative_args;

typedef struct amqp_packet_content_t
{
        amqp_fixed_frame fixed_frame;
        wiced_amqp_performative_args args;
}wiced_amqp_packet_content;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/*****************************************************************************/
/**
 *
 *  @defgroup amqp          AMQP
 *  @ingroup  ipcoms
 *
 *  @addtogroup amqp_v1_0      AMQP v1.0
 *  @ingroup amqp
 *
 * Communication functions for AMQP v1.0 (Advanced Message Queuing Protocol v1.0) Client
 *
 *  @{
 */
/*****************************************************************************/


/** Creates and initializes a new AMQP connection
 *
 * @note This function must be called before any call to other connection functions.
 *
 * @param address [in]   : A pointer to the broker's IP address to connect to.
 * @param interface [in] : WICED interface
 * @param callbacks [in] : Event callback function
 * @param conn [in]       : AMQP connection state type. Should be passed to all subsequent AMQP calls as part of the data structure
 * @param security [in]  : Security parameters to be used - Includes RSA public certificate, private
 *                         key and root certificate (usually self certified).
 *                         If set to NULL, non-secure connection shall be established.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_amqp_init( const wiced_ip_address_t *address, wiced_interface_t interface, const wiced_amqp_callbacks_t *callbacks, wiced_amqp_connection_instance *conn, const wiced_amqp_security_t *security );

/** Frees and deinitializes an AMQP connection
 *
 * @note This function must be called last (after all other calls to AMQP APIs).
 * The function must not be called from a callback context as it could terminate the network
 * thread.
 *
 * @param conn [in]       : A pointer to a connection state type.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_amqp_deinit( wiced_amqp_connection_t *conn );

/** Opens a connection with the amqp server
 *
 * @param connection_instance [in]       : A pointer to AMQP connection instance.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_amqp_open( wiced_amqp_connection_instance *connection_instance );

/** Create and begin AMQP session with peer over the AMQP connection.
 *
 * NOTE : Session flow control is currently not supported. Each session can receive/send maximum of 1024 packets.
 *
 * @param connection_instance [in]       : A pointer to AMQP connection instance.
 * @param connection_instance [in]       : A pointer to AMQP session instance.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_amqp_begin( wiced_amqp_session_instance_t* session_instance, wiced_amqp_connection_instance *connection_instance );

/** Create and attach link with peer within the session.
 *
 * @param link_instance [in]       : A pointer to AMQP link instance.
 * @param session_instance [in]    : A pointer to AMQP session instance.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_amqp_attach( wiced_amqp_link_instance* link_instance, wiced_amqp_session_instance_t* session_instance  );

/** Update link credit, to send updated flow to sender. only receiver will be able to update the link credit
 *
 * @param link_instance [in]  : A pointer to AMQP link instance.
 * @param link_credit [in]    : number of link credit.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_amqp_update_link_credit( wiced_amqp_link_instance *link_instance, uint32_t link_credit );

/** Send data over the established AMQP link.
 *
 * @param link_instance [in]  : A pointer to AMQP link instance.
 * @param message [in]        : A pointer to AMQP message to be sent
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_amqp_send( wiced_amqp_link_instance* link_instance, wiced_amqp_message_t* message );

/** Detach the link within the session
 *
 * @param link_instance [in]  : A pointer to AMQP link instance.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_amqp_detach( wiced_amqp_link_instance *link_instance);

/** End the session within the connection
 *
 * @param session_instance [in]  : A pointer to AMQP Session instance.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_amqp_end( wiced_amqp_session_instance_t* session_instance );

/** Closes the connection with the AMQP peer
 *
 * @param connection_instance [in] : A pointer to AMQP connection instance.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_amqp_close( wiced_amqp_connection_instance *connection_instance );

/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif
