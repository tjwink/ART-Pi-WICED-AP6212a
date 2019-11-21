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

#include "wiced.h"
#include "coap_common.h"
#include "coap_client_common.h"
#include "coap_client_internal.h"

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

/******************************************************
 *                 Type Definitions
 ******************************************************/
/* NOTE : don't try to modify this value */
/* application has to allocate memory and give the reference and this should not be in stack and application has to free it after use */
#define WICED_COAP_OBJECT_MEMORY_SIZE_REQUIREMENT       sizeof(wiced_coap_client_t)


/******************************************************
 *               Function Declarations
 *
 ******************************************************/

/*****************************************************************************/
/**
 *
 *  @defgroup coap          CoAP
 *  @ingroup  ipcoms
 *
 *  @addtogroup coap_client      CoAP Client
 *  @ingroup coap
 *
 * Constrained application protocol (CoAP) is a specialized web transfer protocol for use with
 * constrained nodes and constrained (e.g., low power, lossy) networks which runs over UDP.
 * CoAP provides a request/response interaction model between application endpoints, supports
 * built in discovery of services & resources, and includes key concepts of the web such as URIs
 * and media types.
 *
 * CoAP client implementation on WICED provides APIs to connect to CoAP cloud services
 * such as Exosite. The library supports various CoAP methods such as GET, POST & DELETE resources
 * from the server. The client Implementation also allows users to observe resources and receive
 * notification whenever the resource is updated.
 *
 *  @{
 */
/*****************************************************************************/

/** Initialize instance for CoAP client.
 *
 * @param client [in]   : Client instance to be created.
 * @param interface [in]: WLAN interface (STA,AP).
 * @param callback [in] : Event callback function needs to be registered to library.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_coap_client_init( wiced_coap_client_t* client, wiced_interface_t interface, wiced_service_callback callback );

/** This is an asynchronous API. The response of GET will be given in the event callback with event type WICED_COAP_CLIENT_EVENT_TYPE_GET_RECEIVED.
 *
 * @param client [in]   : Client instance that is already created.
 * @param request [in]  : request with required option and payload information.
 * @param msg_type [in] : message type that which client wants to send.
 * @param ip [in]       : ip address to where you want to send request.
 * @param port [in]     : Port no to which you want to send request.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_coap_client_get( wiced_coap_client_t* client, wiced_coap_client_request_t* request, wiced_coap_msgtype_t msg_type,  wiced_ip_address_t ip, uint16_t port );

/** This is an asynchronous API. The response of POST will be given in the event callback with event type WICED_COAP_CLIENT_EVENT_TYPE_POSTED.
 *
 * @param client [in]   : Client instance that is already created.
 * @param request [in]  : request with required option and payload information.
 * @param msg_type [in] : message type that which client wants to send.
 * @param ip [in]       : ip address to where you want to send request.
 * @param port [in]     : Port no to which you want to send request.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_coap_client_post( wiced_coap_client_t* client, wiced_coap_client_request_t* request, wiced_coap_msgtype_t msg_type,  wiced_ip_address_t ip, uint16_t port );

/** This is an asynchronous API. The response of OBSERVE will be given in the event callback with event type WICED_COAP_CLIENT_EVENT_TYPE_OBSERVED.
 *
 * @param client [in]   : Client instance that is already created.
 * @param request [in]  : request with required option and payload information.
 * @param msg_type [in] : message type that which client wants to send.
 * @param token_id [in] : token information with token and length.
 * @param ip [in]       : ip address to where you want to send request.
 * @param port [in]     : Port no to which you want to send request.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_coap_client_observe( wiced_coap_client_t* client, wiced_coap_client_request_t* client_request, wiced_coap_msgtype_t msg_type, wiced_coap_token_info_t* token_id, wiced_ip_address_t ip, uint16_t port );

/** DeInitialize instance for CoAP client.
 *
 * @param client [in]   : Client instance to be deInitialized.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_coap_client_deinit( wiced_coap_client_t* client );

/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif
