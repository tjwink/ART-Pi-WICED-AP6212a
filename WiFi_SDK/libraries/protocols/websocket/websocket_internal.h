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

/**
 * @websocket_internal.h
 *
 * Internal Header file for Websocket APIs
 */

#pragma once

#include "wiced_network.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
 ******************************************************/

#define GET_TCP_SOCKET_FROM_WEBSOCKET(s, w) s = (w)->core.socket;

#define GET_WEBSOCKET_FROM_TCP_SOCKET(w, s) w = (s)->callback_arg;

#define GET_SERVER_FROM_WEBSOCKET(s, w) s = (w)->core.args;

#define GET_TCP_SERVER_FROM_WEBSOCKET_SERVER(t, w) t = &(w)->tcp_server;


/******************************************************
 *                    Constants
 ******************************************************/

#define WICED_MAXIMUM_NUMBER_OF_SERVER_WEBSOCKETS   (4)
#define SERVER_READ_HANDSHAKE_TIMEOUT               (2000) // what is the right value for handshake timeout for a server?

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum
{
    WEBSOCKET_ROLE_CLIENT,
    WEBSOCKET_ROLE_SERVER,
    WEBSOCKET_ROLE_INVALID,
} wiced_websocket_role_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

struct wiced_websocket_server_config;
struct wiced_websokcet;

typedef struct
{
    char* host;
    char* port;
    char* origin;                     //!< origin
    char* protocols;                  //!< protocol-list supported by websocket
    char* resource_name;
    char* sec_websocket_key;
    char* sec_websocket_version;
} websocket_client_handshake_fields_t;

typedef struct
{
    char* sec_websocket_version;
    char* sec_websocket_accept;
    char* sec_websocket_protocol;
} websocket_server_handshake_fields_t;

typedef struct
{
    union {
        websocket_client_handshake_fields_t client;
        websocket_server_handshake_fields_t server;
    };
} wiced_websocket_handshake_fields_t;

typedef struct
{
    uint8_t                         continuation;
    uint8_t                         first_fragment_opcode;
    uint8_t*                        frame_buffer;
    uint32_t                        length;
} wiced_websocket_receive_params_t;

typedef struct
{
    wiced_websocket_role_t              role;
    wiced_tcp_socket_t*                 socket;
    void*                               args;
    wiced_websocket_receive_params_t    receive;
} wiced_websocket_core_t;

struct wiced_websocket_server
{
    struct wiced_websocket*                 sockets;
    wiced_tcp_server_t                      tcp_server;
    struct wiced_websocket_server_config*   config;
    void*                                   user_data;
    wiced_bool_t                            quit;
};

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t websocket_receive_frame( struct wiced_websocket* websocket );

 #ifdef __cplusplus
} /* extern "C" */
#endif
