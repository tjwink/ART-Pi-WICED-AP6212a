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

//#include "p2p_structures.h"
//#include "wwd_events.h"
//#include "wwd_rtos.h"

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

extern uint8_t*      p2p_write_invitation_request( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer );
extern uint8_t*      p2p_write_provision_discovery_response( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer );
extern uint8_t*      p2p_write_negotiation_response( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer );
extern uint8_t*      p2p_write_invitation_response( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer );
extern uint8_t*      p2p_write_presence_response( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer );
extern uint8_t*      p2p_write_negotiation_confirmation( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer );
extern uint8_t*      p2p_write_device_discoverability_response( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer );
extern uint8_t*      p2p_write_go_discoverability_request( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer );
extern uint8_t*      p2p_write_provision_discovery_request( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer );
extern uint8_t*      p2p_write_negotiation_request( p2p_workspace_t* workspace, const p2p_discovered_device_t* device, void* buffer );
extern besl_result_t p2p_write_association_request_ie( p2p_workspace_t* workspace );
extern besl_result_t p2p_write_wps_probe_response_ie( p2p_workspace_t* workspace );
extern besl_result_t p2p_write_wps_probe_request_ie( p2p_workspace_t* workspace );
extern besl_result_t p2p_write_probe_request_ie( p2p_workspace_t* workspace );
extern besl_result_t p2p_write_probe_response_ie( p2p_workspace_t* workspace );
extern besl_result_t p2p_write_beacon_ie( p2p_workspace_t* workspace );
extern void          p2p_remove_ies( p2p_workspace_t* workspace );

#ifdef __cplusplus
} /*extern "C" */
#endif
