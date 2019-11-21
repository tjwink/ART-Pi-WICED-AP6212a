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

#include "tls_types.h"
#include "wiced_network.h"

#ifndef WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY
#include "wiced_supplicant.h"
#endif /* WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY */

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

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t wiced_tls_encrypt_packet                     ( wiced_tls_workspace_t* context, wiced_packet_t* packet );
wiced_result_t wiced_tls_decrypt_packet                     ( wiced_tls_workspace_t* context, wiced_packet_t* packet );
wiced_result_t wiced_tls_receive_packet                     ( wiced_tcp_socket_t* socket, wiced_packet_t** packet, uint32_t timeout );
wiced_bool_t   wiced_tls_is_encryption_enabled              ( wiced_tcp_socket_t* socket );
wiced_result_t wiced_tls_close_notify                       ( wiced_tcp_socket_t* socket );
wiced_result_t wiced_tls_calculate_overhead                 ( wiced_tls_workspace_t* context, uint16_t available_space, uint16_t* header, uint16_t* footer );
wiced_result_t wiced_tls_calculate_encrypt_buffer_length    ( wiced_tls_workspace_t* context, uint16_t payload_size, uint16_t* required_size);
#ifndef WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY
wiced_result_t wiced_tls_receive_eap_packet                 ( supplicant_workspace_t* supplicant, besl_packet_t* packet, uint32_t timeout );
#endif /* WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY */
#ifdef __cplusplus
} /*extern "C" */
#endif
