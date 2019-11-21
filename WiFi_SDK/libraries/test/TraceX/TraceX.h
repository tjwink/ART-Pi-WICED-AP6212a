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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#if defined(TX_ENABLE_EVENT_TRACE)

#include "wiced_rtos.h"
#include "wiced_tcpip.h"

/******************************************************
 *                     Macros
 ******************************************************/

/* Initialize the TraceX configuration */
#define WICED_TRACEX_EMPTY_CONFIG_INITIALIZER                                 \
    {                                                                         \
        .loop_rec = WICED_FALSE,                                              \
        .filter = 0,                                                          \
        .tcp_server.enable = WICED_FALSE,                                     \
        INITIALISER_IPV4_ADDRESS(.tcp_server.ip, MAKE_IPV4_ADDRESS(0,0,0,0)), \
        .tcp_server.port = WICED_ANY_PORT,                                    \
        .tcp_server.max_data_len = 0,                                         \
        .tcp_server.timeout = 0,                                              \
        .tcp_server.num_retries = 0,                                          \
        .buf.addr = NULL,                                                     \
        .buf.size = 0,                                                        \
        .buf.obj_cnt = 0                                                      \
    }

/* Initialize the TraceX configuration with default values */
#define WICED_TRACEX_DEFAULT_CONFIG_INITIALIZER                               \
    {                                                                         \
        .loop_rec = WICED_FALSE,                                              \
        .filter = WICED_TRACEX_DEFAULT_FILTER,                                \
        .tcp_server.enable = WICED_FALSE,                                     \
        INITIALISER_IPV4_ADDRESS(.tcp_server.ip, WICED_TRACEX_TCP_SERVER_IP), \
        .tcp_server.port = WICED_TRACEX_TCP_SERVER_PORT,                      \
        .tcp_server.max_data_len = WICED_TRACEX_TCP_MAX_PACKET_LENGTH,        \
        .tcp_server.timeout = WICED_TRACEX_TCP_CONNECT_TIMEOUT,               \
        .tcp_server.num_retries = WICED_TRACEX_TCP_CONNECTION_NUM_RETRIES,    \
        .buf.addr = WICED_TRACEX_BUFFER_ADDRESS,                              \
        .buf.size = WICED_TRACEX_BUFFER_SIZE,                                 \
        .buf.obj_cnt = WICED_TRACEX_OBJECT_COUNT                              \
    }

/******************************************************
 *                    Constants
 ******************************************************/


#ifndef WICED_TRACEX_TCP_SERVER_IP
#define WICED_TRACEX_TCP_SERVER_IP                  MAKE_IPV4_ADDRESS(192,168,1,1)
#endif

#ifndef WICED_TRACEX_TCP_SERVER_PORT
#define WICED_TRACEX_TCP_SERVER_PORT                (19702)
#endif

#ifndef WICED_TRACEX_TCP_MAX_PACKET_LENGTH
#define WICED_TRACEX_TCP_MAX_PACKET_LENGTH          (1450)
#endif

#ifndef WICED_TRACEX_TCP_CONNECT_TIMEOUT
#define WICED_TRACEX_TCP_CONNECT_TIMEOUT            (500)
#endif

#ifndef WICED_TRACEX_TCP_CONNECTION_NUM_RETRIES
#define WICED_TRACEX_TCP_CONNECTION_NUM_RETRIES     (3)
#endif

/* MUST be a multiple of 4 bytes */
#ifndef WICED_TRACEX_BUFFER_SIZE
#define WICED_TRACEX_BUFFER_SIZE                    (0x10000)
#endif

#ifndef WICED_TRACEX_OBJECT_COUNT
#define WICED_TRACEX_OBJECT_COUNT                   (100)
#endif

#ifndef WICED_TRACEX_DEFAULT_FILTER
#define WICED_TRACEX_DEFAULT_FILTER                 (~TX_TRACE_USER_EVENTS)
#endif

/* Maximum length of filter string */
#define WICED_TRACEX_FILTER_MAX_STRING_LEN          (256)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    WICED_TRACEX_STATE_DISABLED = 0,
    WICED_TRACEX_STATE_DISABLED_BUF_FULL,   /* Disabled after buffer full */
    WICED_TRACEX_STATE_DISABLED_TCP_ERR,    /* Disabled due to TCP error */
    WICED_TRACEX_STATE_DISABLED_TRACEX_ERR, /* Disabled due to TraceX error */
    WICED_TRACEX_STATE_ENABLED              /* Enabled */
} wiced_tracex_state_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    wiced_bool_t       enable;
    wiced_ip_address_t ip;              /* Only IPv4 addresses supported */
    uint16_t           port;
    uint32_t           max_data_len;    /* Max packet data length */
    uint32_t           timeout;         /* Connection timeout in ms */
    uint32_t           num_retries;
} wiced_tracex_tcp_server_t;

typedef struct
{
    wiced_bool_t              loop_rec;
    uint32_t                  filter;
    wiced_tracex_tcp_server_t tcp_server;
    struct
    {
        void*                 addr;
        size_t                size;
        uint32_t              obj_cnt;
    } buf;
} wiced_tracex_config_t;

typedef struct
{
    wiced_tracex_state_t state;
    wiced_bool_t         tcp_connected; /* Connected to TCP server */
} wiced_tracex_status_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

#ifndef WICED_TRACEX_BUFFER_ADDRESS
extern uint32_t wiced_tracex_buf[WICED_TRACEX_BUFFER_SIZE/4];
#define WICED_TRACEX_BUFFER_ADDRESS  (wiced_tracex_buf)
#define WICED_TRACEX_BUFFER_DECLARE
#endif

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t wiced_tracex_enable( wiced_tracex_config_t* config );
wiced_result_t wiced_tracex_disable( void );
wiced_result_t wiced_tracex_config_get( wiced_tracex_config_t* config );
wiced_result_t wiced_tracex_status_get( wiced_tracex_status_t* status );
wiced_result_t wiced_tracex_filter_list_get( char *str, size_t len );
wiced_result_t wiced_tracex_filter_set( uint32_t mask );
wiced_result_t wiced_tracex_filter_mask_to_str( uint32_t mask, char* str, size_t len );
wiced_result_t wiced_tracex_filter_str_to_mask( char* str, uint32_t* mask );
wiced_result_t wiced_tracex_buf_send( wiced_tracex_tcp_server_t* server );

#endif /* defined(TX_ENABLE_EVENT_TRACE) */

#ifdef __cplusplus
} /*extern "C" */
#endif


