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

#include "duktape.h"
#include <assert.h>
#include <unistd.h>

#include "wiced_duktape.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
// There are two slots for holding callbacks.  One is for the CLOSED event.
// The other slot is for all others since they never conflict in practice.
#define LWS_CLOSED 0
#define LWS_TIMEOUT 1
#define LWS_PREPARE 1
#define LWS_IDLE 1
#define LWS_CHECK 1
#define LWS_ASYNC 1
#define LWS_POLL 1
#define LWS_SIGNAL 1
#define LWS_EXIT 1
#define LWS_CONNECTION 1
#define LWS_READ 1
#define LWS_ONMESSAGE 1

// For XMLHttpRequest, slot0 is used for error callback
#define LWS_XHR_LOAD 1
#define LWS_XHR_ERROR 0

// Audio
#define LWS_AUDIO_EVENT 1

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
typedef int duv_callback_id;

 // Ref for userdata and event callbacks
typedef struct {
  int ref;
  int context;
  int callbacks[2];
} duv_handle_t;

typedef struct {
  int req_ref; // ref for uv_req_t's userdata
  int context;
  int callback_ref; // ref for callback
  int data_ref;
  void* data; // extra data
} duv_req_t;

/******************************************************
 *               Function Declarations
 ******************************************************/
duk_ret_t dukopen_dpm(duk_context *ctx);

#include "refs.h"
#include "utils.h"
#include "schema.h"

#ifdef __cplusplus
} /* extern "C" */
#endif
