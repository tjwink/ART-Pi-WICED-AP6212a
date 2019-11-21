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

#include "linked_list.h"
#include "coap_client_common.h"
#include "parser/coap_parser.h"

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

/* Conservative size limit, as not all options have to be set at the same time. Check when Proxy-Uri option is used
 * Hdr       options (cof + observer + strings
 * Note : Please update MAX_HEADER_SIZE if you are adding any new option.
 */

#define COAP_MAX_HEADER_SIZE             ( 4 + COAP_TOKEN_LENGTH + 3 + 4 + 255)  /* 274 */
#define COAP_MAX_RETRANSMIT               4
#define COAP_RESPONSE_TIME                2
/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct wiced_coap_client_s
{
        wiced_thread_t          event_thread;
        wiced_queue_t           event_queue;
        wiced_udp_socket_t      socket;
        wiced_service_callback  callback;
} wiced_coap_client_t;

typedef struct
{
        coap_packet_t             request;
        wiced_coap_content_type_t payload_type;
} coap_client_request_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

#ifdef __cplusplus
} /* extern "C" */
#endif
