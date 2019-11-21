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

#include "besl_constants.h"
#include "wps_structures.h"
#include "wps_host.h"

/******************************************************
 *                      Macros
 ******************************************************/

#ifdef DEBUG
#define BESL_LIBRARY_INFO(x)           BESL_INFO(x)
#define BESL_LIBRARY_DEBUG(x)          BESL_DEBUG(x)
#define BESL_LIBRARY_ERROR(x)          BESL_ERROR(x)
#define BESL_LIBRARY_ASSERT(string, x) BESL_ASSERT(string, x)
#else
#define BESL_LIBRARY_INFO(x)
#define BESL_LIBRARY_DEBUG(x)
#define BESL_LIBRARY_ERROR(x)
#define BESL_LIBRARY_ASSERT(string, x)
#endif

/******************************************************
 *                    Constants
 ******************************************************/

#define WPS_TEMPLATE_UUID        "\x77\x5b\x66\x80\xbf\xde\x11\xd3\x8d\x2f"

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef uint8_t* (*wps_packet_generator_t)(wps_agent_t* workspace, uint8_t* iter);

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    uint8_t                valid_message_type;
    uint8_t                outgoing_message_type;
    uint32_t               tlv_mask;
    uint32_t               encrypted_tlv_mask;
    wps_packet_generator_t packet_generator;
} wps_state_machine_state_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

extern void         wps_send_eapol_packet       ( besl_packet_t packet, wps_agent_t* workspace, eapol_packet_type_t type, besl_mac_t* their_mac_address, uint16_t content_size );
extern void         wps_abort                   ( wps_agent_t* workspace );
extern wps_result_t wps_send_basic_packet       ( wps_agent_t* workspace, uint8_t type, uint16_t optional_config_error );
extern void         wps_enrollee_init           ( wps_agent_t* workspace );
extern void         wps_enrollee_start          ( wps_agent_t* workspace, wwd_interface_t interface );
extern void         wps_enrollee_reset          ( wps_agent_t* workspace, wwd_interface_t interface );
extern void         wps_registrar_init          ( wps_agent_t* workspace );
extern void         wps_registrar_start         ( wps_agent_t* workspace );
extern void         wps_registrar_reset         ( wps_agent_t* workspace );
extern wps_result_t wps_pbc_overlap_check       ( const besl_mac_t* data );
extern void         wps_clear_pbc_overlap_array ( void );
extern void         wps_record_last_pbc_enrollee( const besl_mac_t* mac );
extern void         wps_update_pbc_overlap_array( wps_agent_t* workspace, const besl_mac_t* mac );
extern void         wps_pbc_overlap_array_notify( const besl_mac_t* mac );
#ifdef __cplusplus
} /*extern "C" */
#endif
