/*
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *  Connection manager APIs.
 *
 *  Internal, not to be used directly by applications.
 */
#pragma once

#include "wiced.h"
#include "mqtt_internal.h"

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

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions

 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
/*
 * Connection manager API.
 *
 * Internal, not to be used by user applications.
 */
wiced_result_t mqtt_manager     ( mqtt_event_t event, void *args, mqtt_connection_t *conn );

#ifdef __cplusplus
} /* extern "C" */
#endif
