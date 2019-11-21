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
#include "coap_common.h"

#ifdef __cplusplus
extern "C"
{
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
/**
 * Defines COAP Client event types received in call-back function
 */
typedef enum wiced_coap_client_event_type_s
{
    WICED_COAP_CLIENT_EVENT_TYPE_POSTED = 1,
    WICED_COAP_CLIENT_EVENT_TYPE_GET_RECEIVED,
    WICED_COAP_CLIENT_EVENT_TYPE_OBSERVED,
    WICED_COAP_CLIENT_EVENT_TYPE_NOTIFICATION,
    WICED_COAP_CLIENT_EVENT_TYPE_DELETED
} wiced_coap_client_event_type_t;


/**
 * Defines COAP Client event types received in call-back function
 */
typedef struct wiced_coap_client_event_info_s
{
    wiced_coap_client_event_type_t  type;
    wiced_coap_token_info_t         token;
    struct
    {
        char*                       uri_path;
        wiced_coap_content_type_t   payload_type;
    } opt;
    wiced_coap_buffer_t             payload;
} wiced_coap_client_event_info_t;

/******************************************************
 *                    Structures
 ******************************************************/

/**
 *  Defines response information needs to be send to client.
 */
typedef struct wiced_coap_client_request_s
{
        wiced_coap_option_info_t  options;       /* Options information received in response */
        wiced_coap_content_type_t payload_type;  /* pay-load type needs to be sent */
        wiced_coap_buffer_t       payload;       /* pay-load */
} wiced_coap_client_request_t;

/** Service call-back
 *
 * @param event_info [in] : Information received as response related to
 * payload and options and payload type.
 *
 * @return @ref wiced_result_t
 */
typedef wiced_result_t (*wiced_service_callback)( wiced_coap_client_event_info_t event_info );

#ifdef __cplusplus
} /* extern "C" */
#endif
