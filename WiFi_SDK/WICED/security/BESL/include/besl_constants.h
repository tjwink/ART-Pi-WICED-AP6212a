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

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define EAP_MTU_SIZE  ( 1020 )

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    EAP_CODE_REQUEST  = 1,
    EAP_CODE_RESPONSE = 2,
    EAP_CODE_SUCCESS  = 3,
    EAP_CODE_FAILURE  = 4
} eap_code_t;

/*
 * EAP Request and Response data begins with one octet Type. Success and
 * Failure do not have additional data.
 */
typedef enum {
    EAP_TYPE_NONE         = 0,
    EAP_TYPE_IDENTITY     = 1   /* RFC 3748 */,
    EAP_TYPE_NOTIFICATION = 2   /* RFC 3748 */,
    EAP_TYPE_NAK          = 3   /* Response only, RFC 3748 */,
    EAP_TYPE_MD5          = 4,  /* RFC 3748 */
    EAP_TYPE_OTP          = 5   /* RFC 3748 */,
    EAP_TYPE_GTC          = 6,  /* RFC 3748 */
    EAP_TYPE_TLS          = 13  /* RFC 2716 */,
    EAP_TYPE_LEAP         = 17  /* Cisco proprietary */,
    EAP_TYPE_SIM          = 18  /* draft-haverinen-pppext-eap-sim-12.txt */,
    EAP_TYPE_TTLS         = 21  /* draft-ietf-pppext-eap-ttls-02.txt */,
    EAP_TYPE_AKA          = 23  /* draft-arkko-pppext-eap-aka-12.txt */,
    EAP_TYPE_PEAP         = 25  /* draft-josefsson-pppext-eap-tls-eap-06.txt */,
    EAP_TYPE_MSCHAPV2     = 26  /* draft-kamath-pppext-eap-mschapv2-00.txt */,
    EAP_TYPE_TLV          = 33  /* draft-josefsson-pppext-eap-tls-eap-07.txt */,
    EAP_TYPE_FAST         = 43  /* draft-cam-winget-eap-fast-00.txt */,
    EAP_TYPE_PAX          = 46, /* draft-clancy-eap-pax-04.txt */
    EAP_TYPE_EXPANDED_NAK = 253 /* RFC 3748 */,
    EAP_TYPE_WPS          = 254 /* Wireless Simple Config */,
    EAP_TYPE_PSK          = 255 /* EXPERIMENTAL - type not yet allocated draft-bersani-eap-psk-09 */
} eap_type_t;

/**
 * EAPOL types
 */

typedef enum
{
    EAP_PACKET                   = 0,
    EAPOL_START                  = 1,
    EAPOL_LOGOFF                 = 2,
    EAPOL_KEY                    = 3,
    EAPOL_ENCAPSULATED_ASF_ALERT = 4
} eapol_packet_type_t;


/*
 * MSCHAPV2 codes
 */
typedef enum {
    MSCHAPV2_OPCODE_CHALLENGE       = 1,
    MSCHAPV2_OPCODE_RESPONSE        = 2,
    MSCHAPV2_OPCODE_SUCCESS         = 3,
    MSCHAPV2_OPCODE_FAILURE         = 4,
    MSCHAPV2_OPCODE_CHANGE_PASSWORD = 7,
} mschapv2_opcode_t;
#ifdef __cplusplus
} /*extern "C" */
#endif
