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

/** @file
 *  WICED COAP constants, data types which are common to API and library
 */

#pragma once

#include "wiced.h"
#include "wiced_crypto.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define WICED_COAP_MAX_OPTIONS         (16)
#define WICED_COAP_MAX_SERVICE_LENGTH  (255)
#define WICED_COAP_MAX_TOKEN           (8)

typedef enum
{
    WICED_COAP_CONTENTTYPE_NONE         = -1,
    WICED_COAP_CONTENTTYPE_TEXT_PLAIN   = 0,
} wiced_coap_content_type_t;

typedef enum
{
    WICED_COAP_METHOD_GET    = 1,
    WICED_COAP_METHOD_POST   = 2,
    WICED_COAP_METHOD_PUT    = 3,
    WICED_COAP_METHOD_DELETE = 4
} wiced_coap_method_t;

typedef enum
{
    WICED_COAP_MSGTYPE_CON    = 0,
    WICED_COAP_MSGTYPE_NONCON = 1,
    WICED_COAP_MSGTYPE_ACK    = 2,
    WICED_COAP_MSGTYPE_RESET  = 3
} wiced_coap_msgtype_t;

/**
 *  Defines observer notification type supported by CoAP server.
 */
typedef enum
{
    WICED_COAP_NOTIFICATION_TYPE_NONE           = -1, /* Don't send notification to observer */
    WICED_COAP_NOTIFICATION_TYPE_CONFIRMABLE    = 0, /* Notify observer with confirmation */
    WICED_COAP_NOTIFICATION_TYPE_NONCONFIRMABLE = 1
/* Notify observer without confirmation */
} wiced_coap_notification_type;

typedef enum
{
    WICED_COAP_SECURITY_TYPE_PSK       = 0,  /* security type PSK */
    WICED_COAP_SECURITY_TYPE_NONPSK    = 1,  /* security type other than PSK */
} wiced_coap_security_type;

typedef struct
{
        uint8_t *data;
        size_t  len;
} wiced_coap_buffer_t;

/**
 *  Defines information related to security.
 */
typedef struct wiced_coap_security_s
{
    wiced_coap_security_type type;
    union
    {
        wiced_dtls_psk_info_t*    psk_info;
        wiced_dtls_nonpsk_info_t  cert_info;
    } args;
} wiced_coap_security_t;

/**
 *  Defines information related to Options.
 */
typedef struct wiced_coap_option_s
{
        uint8_t             num; /* Option number as per CoAP specification */
        wiced_coap_buffer_t buf; /* Option information */
        char                type[ 4 ];
} wiced_coap_option_t;

/**
 *  Defines information related to Token.
 */
typedef struct wiced_coap_token_info_s
{
        size_t  token_len; /* Token length */
        uint8_t data[ WICED_COAP_MAX_TOKEN ]; /* Token value */
} wiced_coap_token_info_t;

/**
 *  Defines information related to total no. of options and option details.
 */
typedef struct wiced_coap_option_info
{
        uint8_t             num_opts; /* Number of options */
        wiced_coap_option_t option[ WICED_COAP_MAX_OPTIONS ]; /* Option information */
} wiced_coap_option_info_t;

/** API is used to set URI path as option in request.
 *
 * @param Option [in]   : Pass option structure.
 * @param value [in]    : value needs to be set for URI path.
 *
 */
void wiced_coap_set_uri_path( wiced_coap_option_info_t* Option, char* value );

/** API is used to set URI query as option in request.
 *
 * @param Option [in]   : Pass option structure.
 * @param value [in]    : value needs to be set for URI Query.
 *
 */
void wiced_coap_set_uri_query( wiced_coap_option_info_t* Option, char* value );

#ifdef __cplusplus
} /* extern "C" */
#endif
