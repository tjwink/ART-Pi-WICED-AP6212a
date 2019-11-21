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

#include "wps_host.h"
#include "wps_structures.h"

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

extern besl_result_t besl_wps_init                        ( wps_agent_t* workspace, const besl_wps_device_detail_t* details, wps_agent_type_t type, wwd_interface_t interface );
extern besl_result_t besl_wps_get_result                  ( wps_agent_t* workspace );
extern besl_result_t besl_wps_deinit                      ( wps_agent_t* workspace );
extern besl_result_t besl_wps_start                       ( wps_agent_t* workspace, besl_wps_mode_t mode, const char* password, besl_wps_credential_t* credentials, uint16_t credential_length );
extern besl_result_t besl_p2p_wps_start                   ( wps_agent_t* workspace );
extern besl_result_t besl_wps_restart                     ( wps_agent_t* workspace );
extern besl_result_t besl_wps_reset_registrar             ( wps_agent_t* workspace, besl_mac_t* mac );
extern besl_result_t besl_wps_wait_till_complete          ( wps_agent_t* workspace );
extern besl_result_t besl_wps_abort                       ( wps_agent_t* workspace );
extern besl_result_t besl_wps_management_set_event_handler( wps_agent_t* workspace, wiced_bool_t enable );
extern besl_result_t besl_wps_scan                        ( wps_agent_t* workspace, wps_ap_t** ap_array, uint16_t* ap_array_size, wwd_interface_t interface );
extern besl_result_t besl_wps_set_directed_wps_target     ( wps_agent_t* workspace, wps_ap_t* ap, uint32_t maximum_join_attempts );
extern void           besl_wps_generate_pin                ( char*        wps_pin_string );
extern int            besl_wps_validate_pin_checksum       ( const char* str );
#ifdef __cplusplus
} /*extern "C" */
#endif
