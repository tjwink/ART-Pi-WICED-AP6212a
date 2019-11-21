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

#include "p2p_structures.h"
#include "wwd_events.h"
#include "wwd_wlioctl.h"

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

extern const wwd_event_num_t p2p_discovery_events[];
extern const wwd_event_num_t p2p_group_owner_events[];

/******************************************************
 *              Function Declarations
 ******************************************************/

extern besl_result_t                 p2p_stop                                 ( p2p_workspace_t* workspace );
extern p2p_client_info_descriptor_t* p2p_host_find_associated_p2p_device      ( p2p_workspace_t* workspace, const besl_mac_t* p2p_device_address );
extern besl_mac_t*                   p2p_host_find_associated_legacy_device   ( p2p_workspace_t* workspace, const besl_mac_t* mac_address );
extern besl_result_t                 p2p_host_notify_connection_request       ( p2p_workspace_t* workspace, p2p_discovered_device_t* device );
extern besl_result_t                 p2p_host_send_message                    ( p2p_message_t* message, uint32_t timeout_ms );
extern void                          p2p_thread_start                         ( p2p_workspace_t* workspace );
extern void                          p2p_host_add_vendor_ie                   ( uint32_t interface, void* data, uint16_t data_length, uint32_t packet_mask );
extern void                          p2p_host_remove_vendor_ie                ( uint32_t interface, void* data, uint16_t data_length, uint32_t packet_mask );
extern void*                         p2p_event_handler                        ( const wwd_event_header_t* event_header, const uint8_t* event_data, /*@returned@*/ void* handler_user_data );
extern void                          p2p_rewrite_group_owner_management_ies   ( void* wps_agent_owner );
extern besl_result_t                 p2p_process_probe_request                ( p2p_workspace_t* workspace, const uint8_t* data, uint32_t data_length );
extern void                          p2p_process_action_frame                 ( p2p_workspace_t* workspace, const uint8_t* data, const wwd_event_header_t* event_header );
extern p2p_discovered_device_t*      p2p_find_device                          ( p2p_workspace_t* workspace, besl_mac_t* mac);
extern besl_result_t                 p2p_process_new_device_data              ( p2p_workspace_t* workspace, p2p_discovered_device_t* device );
extern besl_result_t                 p2p_process_association_request          ( p2p_workspace_t* workspace, const uint8_t* data, const wwd_event_header_t* event_header );
extern besl_result_t                 p2p_update_devices                       ( p2p_workspace_t* workspace );
extern besl_result_t                 p2p_deinit                               ( p2p_workspace_t* workspace );
extern void                          p2p_stop_timer                           ( void* workspace );
extern void                          p2p_clear_event_queue                    ( void );
extern besl_result_t                 p2p_set_ops                              ( wl_p2p_ops_t* ops );
extern besl_result_t                 p2p_set_noa                              ( wl_p2p_sched_t* noa );

#ifdef __cplusplus
} /*extern "C" */
#endif
