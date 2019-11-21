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
 *
 */
#pragma once

#include "http.h"
#include "http_client.h"
#include "callback_loop.h"
#include "dpm.h"
#include "wiced_duktape.h"
#include "linked_list.h"

#ifdef __cplusplus
extern "C" {
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
typedef enum
{
    LWS_XHR_CALLBACK_EVENT_ONLOAD,
    LWS_XHR_CALLBACK_EVENT_ONERROR,
    LWS_XHR_CALLBACK_EVENT_ONREADYSTATECHANGE,

    LWS_XHR_CALLBACK_EVENT_HTTPSTATUS,
    LWS_XHR_CALLBACK_EVENT_DISCONNECT,
} dpm_xhr_callback_event_id;

typedef enum
{
    LWS_XHR_ERROR_NONE = 0,
    LWS_XHR_ERROR_NOMEM = 1,
    LWS_XHR_ERROR_INVALID_STATE,
    LWS_XHR_ERROR_INVALID_METHOD,
    LWS_XHR_ERROR_INVALID_URL,
    LWS_XHR_ERROR_SYNC_MODE,
    LWS_XHR_ERROR_INTERNAL,
    LWS_XHR_ERROR_HOSTLOOKUP,
    LWS_XHR_ERROR_CONNECT,
    LWS_XHR_ERROR_INVALID_PAYLOAD,

} dpm_xhr_errorno_t;

typedef enum
{
    LWS_XHR_READYSTATE_UNINIT,
    LWS_XHR_READYSTATE_OPEN,
    LWS_XHR_READYSTATE_SENT,
    LWS_XHR_READYSTATE_RECEIVING,
    LWS_XHR_READYSTATE_LOADED
} dpm_xhr_readystate_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
  linked_list_node_t node;
  duk_context *ctx;
  void *data;
  http_client_t client;
  http_client_configuration_info_t config;
  http_request_t request;
  http_method_t method;
  http_security_t security;
  char * url;
  wiced_bool_t async;
  uint16_t readyState;
  uint16_t http_status;
  uint8_t *body;
  uint32_t body_length;
  dpm_xhr_errorno_t error;
  int relative_path;
}dpm_xhr_handle_t;

/******************************************************
 *               Function Declarations
 ******************************************************/
duk_ret_t dpm_new_xhr(duk_context *ctx);
duk_ret_t dpm_xhr_open(duk_context *ctx);
duk_ret_t dpm_xhr_send(duk_context *ctx);
duk_ret_t dpm_xhr_set_request_header(duk_context *ctx);
duk_ret_t dpm_xhr_on_load(duk_context *ctx);
duk_ret_t dpm_xhr_on_error(duk_context *ctx);
duk_ret_t dpm_xhr_shutdown(duk_context *ctx);


void xmlhttprequest_callback_handler(wiced_duktape_callback_queue_element_t *message);

#ifdef __cplusplus
} /* extern "C" */
#endif
