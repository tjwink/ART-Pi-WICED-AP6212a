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

#include "besl_structures.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define EAP_TLS_FLAG_LENGTH_INCLUDED 0x80
#define EAP_TLS_FLAG_MORE_FRAGMENTS  0x40

#define LEAP_VERSION        1
#define LEAP_CHALLENGE_LEN  8
#define LEAP_RESPONSE_LEN   24
#define LEAP_KEY_LEN        16

#define AVP_CODE_EAP_MESSAGE    79
#define AVP_FLAG_MANDATORY_MASK 0x40
#define AVP_FLAG_VENDOR_MASK    0x80
#define AVP_LENGTH_SIZE         3


/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    TLS_AGENT_EVENT_EAPOL_PACKET,
    TLS_AGENT_EVENT_ABORT_REQUESTED,
} tls_agent_event_t;

/* High level states
 * INITIALISING ( scan, join )
 * EAP_HANDSHAKE ( go through EAP state machine )
 * WPS_HANDSHAKE ( go through WPS state machine )
 */
typedef enum
{
    SUPPLICANT_INITIALISING,
    SUPPLICANT_INITIALISED,
    SUPPLICANT_IN_EAP_METHOD_HANDSHAKE,
    SUPPLICANT_CLOSING_EAP,
} supplicant_main_stage_t;

typedef enum
{
    SUPPLICANT_EAP_START         = 0,    /* (EAP start ) */
    SUPPLICANT_EAP_IDENTITY      = 1,    /* (EAP identity request, EAP identity response) */
    SUPPLICANT_EAP_NAK           = 2,
    SUPPLICANT_EAP_METHOD        = 3,
} supplicant_state_machine_stage_t;


typedef enum {
    TUNNEL_TYPE_NONE        = 0,
    TUNNEL_TYPE_EAP         = 1,
    TUNNEL_TYPE_CHAP        = 2,
    TUNNEL_TYPE_MSCHAP      = 3,
    TUNNEL_TYPE_MSCHAPV2    = 4,
    TUNNEL_TYPE_PAP         = 5,
} supplicant_tunnel_auth_type_t;

typedef enum
{
    SUPPLICANT_LEAP_IDENTITY              = SUPPLICANT_EAP_IDENTITY,
    SUPPLICANT_LEAP_RESPOND_CHALLENGE     = 2,
    SUPPLICANT_LEAP_REQUEST_CHALLENGE     = 3,
    SUPPLICANT_LEAP_DONE                  = 4,
} supplicant_leap_state_machine_t;

#ifdef __cplusplus
} /*extern "C" */
#endif
