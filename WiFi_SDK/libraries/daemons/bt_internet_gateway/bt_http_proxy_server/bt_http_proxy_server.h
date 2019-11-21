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

#include "wiced.h"
#include "dns.h"
#include "wiced_bt_dev.h"
#include "big_stack_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define HPS_HEADERS_CHARACTERISTIC_VALUE_LENGTH (512)
#define HPS_BODY_CHARACTERISTIC_VALUE_LENGTH    (512)
#define HPS_URI_CHARACTERISTIC_VALUE_LENGTH     (512)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    HPS_IDLE,
    HPS_BLE_CONNECTION_ESTABLISHED,
    HPS_TCP_CONNECTION_INITIATED,
    HPS_TCP_CONNECTION_ESTABLISHED,
    HPS_HTTP_REQUEST_SENT,
    HPS_HTTP_RESPONSE_RECEIVED,
} hps_connection_state_t;

typedef enum
{
    HOSTNAME_UNRESOLVED,
    HOSTNAME_LOOKUP_PENDING,
    HOSTNAME_LOOKUP_TIMEOUT,
    HOSTNAME_RESOLVED
} hps_connection_dns_lookup_state_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef void (*hps_server_ble_connect_callback_t)      ( uint8_t* bd_address_ptr, wiced_bool_t* is_connection_allowed );
typedef void (*hps_sever_https_certificate_callback_t) ( const uint8_t** ca_certificate_ptr, uint16_t* ca_certificate_length, const uint8_t** client_certificate_ptr, uint16_t* client_certificate_length );

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    hps_connection_state_t            state;
    hps_connection_dns_lookup_state_t dns_state;
    uint8_t                           bd_address[ BD_ADDR_LEN ];
    uint16_t                          connection_handle;
    wiced_ip_address_t                server_ip;
    wiced_tcp_socket_t                tcp_socket;
    wiced_packet_t*                   tcp_tx_packet;
    wiced_packet_t*                   tcp_rx_packet;
    uint16_t                          write_handle_in_progress;
    const char*                       http_method;
    wiced_timed_event_t               receive_timer;
    wiced_bool_t                      receive_timer_expiry;

    /* characteristic values storage */
    uint8_t                           headers_char_value[ HPS_HEADERS_CHARACTERISTIC_VALUE_LENGTH ];
    uint16_t                          headers_length;
    uint8_t                           body_char_value[ HPS_BODY_CHARACTERISTIC_VALUE_LENGTH ];
    uint16_t                          body_length;
    uint8_t                           uri_char_value[ HPS_URI_CHARACTERISTIC_VALUE_LENGTH ];
    uint16_t                          uri_length;
    uint8_t*                          hostname_end;
    uint8_t*                          hostname_start;
    wiced_bool_t                      hostname_found_in_uri;
    wiced_bool_t                      https_enabled;
    uint8_t                           https_security_char_value;
    uint8_t                           status_code_char_value[ 3 ];
    uint8_t                           security_char_value;
    uint8_t                           control_point_char_value;
    uint16_t                          client_char_config;
} hps_connection_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t hps_server_start                               ( hps_connection_t* connections_array, uint8_t connection_count, wiced_interface_t interface, big_peer_device_link_keys_callback_t keys_callback );
wiced_result_t hps_server_stop                                ( void );
wiced_result_t hps_server_register_ble_connection_callback    ( hps_server_ble_connect_callback_t callback );
wiced_result_t hps_server_register_https_certificate_callback ( hps_sever_https_certificate_callback_t callback );

#ifdef __cplusplus
} /* extern "C" */
#endif
