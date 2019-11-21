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

#include "wiced_result.h"
#include "supplicant_structures.h"
#include "wwd_constants.h"
#include "besl_host.h"

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

besl_result_t besl_supplicant_init                   ( supplicant_workspace_t* workspace, supplicant_connection_info_t *conn_info );
besl_result_t besl_supplicant_deinit                 ( supplicant_workspace_t* workspace );
besl_result_t besl_supplicant_start                  ( supplicant_workspace_t* workspace );
besl_result_t besl_supplicant_wait_till_complete     ( supplicant_workspace_t* workspace );
besl_result_t besl_supplicant_stop                   ( supplicant_workspace_t* workspace );
besl_result_t supplicant_host_send_eap_tls_fragments ( supplicant_workspace_t* workspace, uint8_t* buffer, uint16_t length );
besl_result_t supplicant_receive_eap_tls_packet      ( void* workspace_in, tls_packet_t** packet, uint32_t timeout );
void          besl_supplicant_set_identity           ( supplicant_workspace_t* workspace, const char* eap_identity, uint32_t eap_identity_length );
void          besl_supplicant_set_inner_identity     ( supplicant_workspace_t* workspace, eap_type_t eap_type, void* inner_identity );
void          supplicant_host_consume_tls_bytes      ( tls_packet_t* packet, int32_t number_of_bytes );
//void          wiced_supplicant_thread_main           ( uint32_t arg );
besl_result_t supplicant_tls_agent_init              ( tls_agent_workspace_t* workspace );
besl_result_t supplicant_tls_agent_deinit            ( tls_agent_workspace_t* workspace );
besl_result_t supplicant_send_eap_tls_packet         ( supplicant_workspace_t* workspace, tls_agent_event_message_t* tls_agent_message, uint32_t timeout );
besl_result_t supplicant_outgoing_pop                ( void* workspace, besl_event_message_t*   outgoing_packet );
besl_result_t supplicant_outgoing_push               ( void* workspace, besl_event_message_t* message );
besl_result_t supplicant_host_get_tls_data           ( besl_packet_t eapol_packet, uint16_t offset, uint8_t** data, uint16_t* fragment_available_data_length, uint16_t *total_available_data_length );
besl_result_t supplicant_tls_agent_finish_connect    ( supplicant_workspace_t* workspace );
besl_result_t supplicant_process_event               ( supplicant_workspace_t* workspace, besl_event_message_t* message);
besl_result_t supplicant_tls_agent_start             ( supplicant_workspace_t* workspace );
besl_result_t supplicant_tls_calculate_overhead      ( supplicant_workspace_t* workspace, uint16_t available_space, uint16_t* header, uint16_t* footer );

wiced_result_t wiced_supplicant_enable_tls            ( supplicant_workspace_t* supplicant, void* context );
wiced_result_t wiced_supplicant_start_tls             ( supplicant_workspace_t* supplicant, wiced_tls_endpoint_type_t type, wiced_tls_certificate_verification_t verification );
wiced_result_t wiced_supplicant_start_tls_with_ciphers( supplicant_workspace_t* supplicant, wiced_tls_endpoint_type_t type, wiced_tls_certificate_verification_t verification, const cipher_suite_t* cipher_list[] );

besl_result_t supplicant_phase2_init                   ( supplicant_workspace_t* workspace, eap_type_t eap_type );
besl_result_t supplicant_phase2_start                  ( supplicant_workspace_t* workspace );
besl_result_t supplicant_process_ttls_phase2_event     ( supplicant_workspace_t* workspace, besl_packet_t packet);

besl_result_t supplicant_phase2_deinit                 ( supplicant_workspace_t* workspace );
besl_result_t supplicant_phase2_stop                   ( supplicant_workspace_t* workspace );
besl_result_t supplicant_inner_packet_set_data        ( besl_packet_t* packet, int32_t size );
besl_result_t supplicant_process_peap_event          ( supplicant_workspace_t* workspace, besl_packet_t packet);
uint32_t      supplicant_eap_get_timer               ( supplicant_workspace_t* workspace );
void          supplicant_eap_handshake_cleanup       ( supplicant_workspace_t* workspace );


besl_result_t supplicant_peap_init_state( supplicant_workspace_t* workspace, eap_type_t type );
besl_result_t supplicant_init_state(supplicant_workspace_t* workspace, eap_type_t eap_type );

#ifdef __cplusplus
} /*extern "C" */
#endif
