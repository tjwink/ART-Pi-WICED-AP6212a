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

#pragma once

#include "wiced.h"
#include "amqp_frame.h"
#include "amqp_network.h"

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#define AMQP_CONNECTION_FRAME_MAX       (4*1024)        /* Maximum frame size for a connection          */
#define AMQP_CONNECTION_CHANNEL_MAX     (1)             /* Maximum number of channels per connection    */
#define AMQP_CONNECTION_HEARTBEAT       (5)             /* Time interval between hearbeats in seconds   */

/******************************************************
 *                    Constants
 ******************************************************/
#define AMQP_CONNECTION_DEFAULT_PORT    (5672)
#define AMQP_CONNECTION_SECURITY_PORT   (5671)
#define AMQP_CONNECTION_DATA_SIZE_MAX   (AMQP_CONNECTION_FRAME_MAX - 8) /* Maximum size to put in a frame */

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum SASL_CLIENT_NEGOTIATION_STATE_TAG
{
    SASL_CLIENT_NEGOTIATION_NOT_STARTED      =  0,
    SASL_CLIENT_NEGOTIATION_MECH_RCVD        =  1,
    SASL_CLIENT_NEGOTIATION_INIT_SENT        =  2,
    SASL_CLIENT_NEGOTIATION_CHALLENGE_RCVD   =  3,
    SASL_CLIENT_NEGOTIATION_RESPONSE_SENT    =  4,
    SASL_CLIENT_NEGOTIATION_OUTCOME_RCVD     =  5,
    SASL_CLIENT_NEGOTIATION_ERROR            =  6
} sasl_client_negotiation_state;

typedef enum connection_states_t
{
        AMQP_CONNECTION_START               =   0,
        AMQP_CONNECTION_HDR_RCVD            =   1,
        AMQP_CONNECTION_HDR_SENT            =   2,
        AMQP_CONNECTION_HDR_EXCH            =   3,
        AMQP_CONNECTION_OPEN_PIPE           =   4,
        AMQP_CONNECTION_OC_PIPE             =   5,
        AMQP_CONNECTION_OPEN_RCVD           =   6,
        AMQP_CONNECTION_OPEN_SENT           =   7,
        AMQP_CONNECTION_CLOSE_PIPE          =   8,
        AMQP_CONNECTION_OPENED              =   9,
        AMQP_CONNECTION_CLOSE_RCVD          =  10,
        AMQP_CONNECTION_CLOSE_SENT          =  11,
        AMQP_CONNECTION_DISCARDING          =  12,
        AMQP_CONNECTION_END                 =  14,
        AMQP_CONNECTION_ERROR               =  15,
        AMQP_CONNECTION_INVALID_STATE       =  16,
        AMQP_CONNECTION_NONE_STATE          =  17,
        AMQP_CONNECTION_SASL_HDR_RCVD       =  18,
        AMQP_CONNECTION_SASL_HDR_SENT       =  19,
        AMQP_CONNECTION_SASL_HDR_EXCH       =  20,
        AMQP_CONNECTION_SASL_NEGOTIATION_MECH_RCVD = 21,
        AMQP_CONNECTION_SASL_INIT_SENT      = 22,
        AMQP_CONNECTION_SASL_OUTCOME_RCVD   = 23

}wiced_amqp_connection_states;

typedef enum session_states_t
{
        AMQP_SESSION_UNMAPPED         =   0,
        AMQP_SESSION_BEGIN_SENT       =   1,
        AMQP_SESSION_BEGIN_RCVD       =   2,
        AMQP_SESSION_MAPPED           =   3,
        AMQP_SESSION_END_SENT         =   4,
        AMQP_SESSION_END_RCVD         =   5,
        AMQP_SESSION_DISCARDING       =   6,
        AMQP_SESSION_INVALID_STATE
}wiced_amqp_session_states;

typedef enum link_states_t
{
        AMQP_LINK_STATE_DETACHED      =   0,
        AMQP_LINK_STATE_HALF_ATTACHED =   1,
        AMQP_LINK_STATE_ATTACHED      =   2,
        AMQP_LINK_STATE_ERROR         =   3,
        AMQP_LINK_STATE_INITIAL
}wiced_amqp_link_states;

typedef enum wiced_amqp_performative_type_t
{
        AMQP_OPEN                = 16,
        AMQP_BEGIN               = 17,
        AMQP_ATTACH              = 18,
        AMQP_FLOW                = 19,
        AMQP_TRANSFER            = 20,
        AMQP_DISPOSITON          = 21,
        AMQP_DETACH              = 22,
        AMQP_END                 = 23,
        AMQP_CLOSE               = 24,
        AMQP_SASL_SRV_MECHS      = 64,
        AMQP_SASL_INIT           = 65,
        AMQP_SASL_CHALLENGE      = 66,
        AMQP_SASL_RESPONSE       = 67,
        AMQP_SASL_OUTCOME        = 68,
        AMQP_UNKNOWN_TYPE
} wiced_amqp_performative_type;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/* TODO : this will be used in future */
typedef struct key_value_pair_s
{
        void *key;
        void *value;
} key_value_pair;

typedef struct amqp_map_s
{
        int count;
        key_value_pair *pairs;
} amqp_map;

typedef struct amqp_error_s
{
        char* condition;
        char* descrption;
        amqp_map info;
} amqp_error;

typedef struct wiced_amqp_heartbeat_s
{
        wiced_timed_event_t timer;
        uint32_t reset_value;
        uint32_t step_value;
        uint32_t send_counter;
        uint32_t recv_counter;
} wiced_amqp_heartbeat_t;
/* connection endpoint information */

typedef struct amqp_channel_endpoints_s
{
        linked_list_node_t this_node;
        uint16_t incoming_channel;
        uint16_t outgoing_channel;
        void* context;
} amqp_channel_endpoint_t;

/* LINK related data structures */

typedef struct amqp_link_endpoints_s
{
        linked_list_node_t this_node;
        uint8_t* name;
        uint32_t input_handle;
        uint32_t output_handle;
        void* context;
} amqp_link_endpoints;


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

#ifdef __cplusplus
} /* extern "C" */
#endif
