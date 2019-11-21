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
#include <stdint.h>
#include "tls_types.h"
#include "besl_constants.h"
#include "supplicant_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/
#define SUPPLICANT_MAX_IDENTITY_LENGTH 32
#define SUPPLICANT_MAX_PASSWORD_LENGTH 64

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef void* tls_agent_packet_t;

/******************************************************
 *                Packed Structures
 ******************************************************/

/******************************************************
 *                Unpacked Structures
 ******************************************************/

typedef struct
{
    tls_agent_event_t           event_type;
    union
    {
        tls_agent_packet_t      packet;
        uint32_t                value;
    } data;
} tls_agent_event_message_t;

typedef struct
{
    void*                       tls_agent_host_workspace;
} tls_agent_workspace_t;

typedef struct supplicant_phase2_state_s
{
    eap_type_t              eap_type;
    besl_result_t           result;
    supplicant_main_stage_t main_stage;
    uint8_t                 sub_stage;

    uint8_t                 identity[SUPPLICANT_MAX_IDENTITY_LENGTH];
    uint8_t                 identity_length;

    uint8_t                 password[SUPPLICANT_MAX_PASSWORD_LENGTH];
    uint8_t                 password_length;

    uint8_t                 request_id;
    uint8_t                 leap_ap_challenge[LEAP_CHALLENGE_LEN];/* Used only in eap-ttls with Cisco-LEAP inner authentication*/
} supplicant_phase2_state_t;

typedef struct supplicant_inner_identity_s
{
    uint8_t                 identity[SUPPLICANT_MAX_IDENTITY_LENGTH];
    uint8_t                 identity_length;

    /* in Windows password is UNICODE */
    uint8_t                 password[SUPPLICANT_MAX_PASSWORD_LENGTH];
    uint8_t                 password_length;
}supplicant_inner_identity_t;

//typedef struct supplicant_peap_workspace_s* supplicant_peap_workspace_ptr_t;
typedef struct supplicant_phase2_workspace_s* supplicant_phase2_workspace_ptr_t;

typedef struct supplicant_connection_info_
{
    wwd_interface_t interface;              /* Network interface type*/

    wiced_tls_identity_t *tls_identity;     /* Identity for the secure connection */
    wiced_tls_session_t *tls_session;       /* Used to store TLS session information */
    wiced_tls_context_t *context;           /* TLS context required for secure connection */

    const char* trusted_ca_certificates;    /* Root-CA certificate */
    uint32_t root_ca_cert_length;           /* Root-CA certificate length */

    eap_type_t eap_type;                    /* EAP type */
    wiced_security_t auth_type;             /* Security Auth type */
    const char *eap_identity;               /* EAP identity string, 'null' terminated */

    const char *user_name;                  /* [Used incase of PEAP and EAP_TTLS] User name string, 'null' terminated */
    const char *password;                   /* [Used incase of PEAP and EAP_TTLS] Password string, 'null' terminated */

    const uint8_t* user_cert;               /* [Used incase of EAP-TLS and optionally in EAP_TTLS] Client certificate */
    uint32_t user_cert_length;              /* [Used incase of EAP-TLS and optionally in EAP_TTLS] Client certificate length */

    supplicant_tunnel_auth_type_t tunnel_auth_type;/* tunnel authentication type EAP, CHAP etc..*/
    eap_type_t inner_eap_type;              /* [Used incase of EAP_TTLS] inner eap type*/
    uint8_t is_client_cert_required;        /* [Used incase of EAP_TTLS] Specifies if client certificate is required*/

    const char* private_key;                /* [Used incase of EAP-TLS] Client private key */
    uint32_t key_length;                    /* [Used incase of EAP-TLS] Client private key length */
} supplicant_connection_info_t;

typedef struct
{
    eap_type_t                  eap_type;
    void*                       supplicant_host_workspace;
    uint32_t                    interface;
    besl_result_t               supplicant_result;

    /* State machine stages */
    supplicant_main_stage_t     current_main_stage;
    uint8_t                     current_sub_stage; /* Either a value from wps_eap_state_machine_stage_t or wps_state_machine_stage_t */
    wiced_security_t            auth_type; /* Security auth type */

    /* The ID of the last received packet we should use when replying */
    uint8_t                     last_received_id;

    uint8_t                     have_packet;      /* Flags that a packet is already created for TLS records */
    uint32_t                    start_time;
    uint32_t                    eap_handshake_start_time;
    uint32_t                    eap_handshake_current_time;

    besl_mac_t                  supplicant_mac_address;
    besl_mac_t                  authenticator_mac_address;
    uint8_t                     outer_eap_identity[32];
    uint8_t                     outer_eap_identity_length;

    wiced_tls_context_t*        tls_context;
    tls_agent_workspace_t       tls_agent;
    uint8_t                     tls_length_overhead;    /* This is a workaround flag for eap_tls_packet lenght parameter */
    uint8_t*                    buffer;    // XXX temporary until we review how the TLS engine is working with EAP transport
    uint32_t                    buffer_size;
    uint8_t*                    data_start;
    uint8_t*                    data_end;

    supplicant_tunnel_auth_type_t tunnel_auth_type;
    eap_type_t                  inner_eap_type;
    supplicant_inner_identity_t inner_identity;
    supplicant_phase2_workspace_ptr_t ptr_phase2;
} supplicant_workspace_t;

#ifdef __cplusplus
} /*extern "C" */
#endif
