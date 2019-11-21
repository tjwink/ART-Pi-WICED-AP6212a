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
 *          TLS -> Host Function Declarations
 ******************************************************/

/******************************************************
 *           Host -> TLS Function Declarations
 ******************************************************/
extern tls_result_t tls_host_receive_packet( mbedtls_ssl_context* ssl, tls_packet_t** packet, uint32_t timeout );

extern tls_result_t tls_host_free_packet     ( tls_packet_t* packet );
extern tls_result_t tls_host_create_buffer   ( wiced_tls_workspace_t* ssl, uint8_t** buffer, uint16_t buffer_size );
extern tls_result_t tls_host_get_packet_data ( mbedtls_ssl_context* ssl, tls_packet_t* packet, uint32_t offset, uint8_t** data, uint16_t* data_length, uint16_t* available_data_length );
extern tls_result_t tls_host_set_packet_start( tls_packet_t* packet, uint8_t* start );
extern tls_result_t tls_host_set_packet_end( tls_packet_t* packet, uint8_t* end );
extern tls_result_t tls_host_send_tcp_packet ( void* context, tls_packet_t* packet );
extern tls_result_t tls_calculate_encrypt_buffer_length( mbedtls_ssl_context* context, uint16_t* required_buff_size, uint16_t payload_size);

extern void* tls_host_get_defragmentation_buffer ( uint16_t size );
extern void  tls_host_free_defragmentation_buffer( void* buffer );

extern int ssl_flush_output( void *ssl, const uint8_t* buffer, size_t length );
#ifndef DISABLE_EAP_TLS
int eap_ssl_flush_output( void* context, const uint8_t* buffer, size_t length );
#endif /* DISABLE_EAP_TLS */

extern void* tls_host_malloc( const char* name, uint32_t size );
extern void  tls_host_free  ( void* p );
extern uint64_t tls_host_get_time_ms( void );

extern tls_result_t tls_complete_record_defragmentation(mbedtls_ssl_context* ssl, uint32_t timeout);
extern void tls_consume_received_packet_bytes( mbedtls_ssl_context *ssl, uint16_t number_of_bytes );
extern void  tls_host_free  ( void* p );

#ifdef __cplusplus
} /*extern "C" */
#endif
