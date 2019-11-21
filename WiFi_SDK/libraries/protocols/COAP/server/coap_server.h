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
#include "wiced_crypto.h"
#include "coap_server_common.h"
#include "coap_server_internal.h"
#include "wiced_dtls.h"

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
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                 Function Definitions
 ******************************************************/

/*****************************************************************************/
/**
 *
 * @defgroup coap        CoAP
 * @ingroup  ipcoms
 *
 * @addtogroup coap_server CoAP Server
 * @ingroup coap
 *
 * Constrained application protocol (CoAP) is a specialized web transfer protocol for use with
 * constrained nodes and constrained (e.g., low power, lossy) networks which runs over UDP.
 * CoAP provides a request/response interaction model between application endpoints, supports
 * built in discovery of services & resources, and includes key concepts of the web such as URIs
 * and media types.
 *
 * CoAP server implementation on WICED provides APIs to register different services with server.
 * The library supports ping/pong, discovery of well-known services and various CoAP methods (GET,
 * POST, and DELETE). The CoAP server is capable of adding & removing observers for particular
 * resource & sending notification to observer [with retransmission for confirmable message types]
 * whenever resources are updated.
 *
 *  @{
 */
/*****************************************************************************/


/** Create and Initialize COAP server
 *
 * @param server [in]   : Server instance to be created
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_coap_server_init( wiced_coap_server_t* server );

/** Destroy and de-initialize COAP server
 *
 * @param server [in]   : Server instance to be destroyed
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_coap_server_deinit( wiced_coap_server_t* server );

/** Start COAP Server
 *
 * @param server [in]    : Server instance
 * @param interface [in] : Network interface
 * @param port [in]      : Port no to which you want server to listen for requests.
 * @param security [in]  : security parameters to be used. Includes RSA public certificate, private
 *                         key and root certificate (usually self certified)
 *                         If set to NULL, no security is going to be used
 *
 * @return @ref wiced_result_t
 */
wiced_result_t wiced_coap_server_start( wiced_coap_server_t* server, wiced_interface_t interface, uint16_t port, wiced_coap_security_t *security );

/** Stop COAP Server
 *
 * @param server [in]   : Server instance that needs to be stopped
 * @param security [in] : security parameters to be used for de-initializing identity. Make sure server and security variable exist till @ref wiced_coap_server_deinit API returns.
 *
 *  @return @ref wiced_result_t
 */
wiced_result_t wiced_coap_server_stop( wiced_coap_server_t* server, wiced_coap_security_t *security );

/** Register or Add new service with COAP server
 *
 * @param server             : Server instance
 * @param service            : Service object to be added to server
 * @param service_name[in]   : Name of the service it should be 'null' terminated
 * @param callback [in]      : Service call-back
 * @param content_type [in]  : Content type
 *
 *  @return @ref wiced_result_t
 */
wiced_result_t wiced_coap_server_add_service( wiced_coap_server_t* server, wiced_coap_server_service_t* service, char* service_name, wiced_coap_server_callback callback, wiced_coap_content_type_t type );

/** De-register or delete particular service with COAP server
 *
 * @param server[in]     : Server instance
 * @param service[in]    : Service object that needs to be delete
 *
 *  @return @ref wiced_result_t
 */
wiced_result_t wiced_coap_server_delete_service( wiced_coap_server_t* server, wiced_coap_server_service_t* service );

/** Send response back to COAP client
 *
 * @param server[in]            : server instance to be used to send response
 * @param service [in]          : service object to be used to send response
 * @param req_handle [in]       : request handle
 * @param response [in]         : Response that will be sent from callback
 * @param notification_type[in] : Notification type
 *
 *  @return @ref wiced_result_t
 */
wiced_result_t wiced_coap_server_send_response( void* server, wiced_coap_server_service_t* service, void* req_handle, wiced_coap_server_response_t* response, wiced_coap_notification_type notification_type );

#ifdef __cplusplus
} /* extern "C" */
#endif
