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

/******************************************************
 *                    Structures
 ******************************************************/

/**
 *  Defines request information needs to be used.
 */
typedef struct wiced_coap_server_request_s
{
        wiced_coap_method_t method;     /* method type requested by client */
        wiced_coap_buffer_t payload;    /* pay-load information given by client */
        void*               req_handle; /* request handle used to send response */
} wiced_coap_server_request_t;

/**
 *  Defines response information needs to be send to client.
 */
typedef struct wiced_coap_server_response_s
{
        wiced_coap_option_info_t    options;        /* Options information */
        wiced_coap_content_type_t   payload_type;   /* pay-load type needs to be sent */
        wiced_coap_buffer_t         payload;        /* pay-load buffer */
} wiced_coap_server_response_t;

typedef struct wiced_coap_server_service_s wiced_coap_server_service_t;

/** Service call-back
 *
 * @param server [in]    : Server instance.
 * @param service [in]   : Service object used for the client request.
 * @param request [in]   : Request information form client
 *
 * @return @ref wiced_result_t
 */
typedef wiced_result_t (*wiced_coap_server_callback)( void* server, wiced_coap_server_service_t* service, wiced_coap_server_request_t* request );

#ifdef __cplusplus
} /* extern "C" */
#endif
