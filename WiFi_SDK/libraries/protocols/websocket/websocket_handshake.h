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
#include "websocket.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define SHA_LENGTH                              ( 20 )
#define WEBSOCKET_KEY_LENGTH                    ( 16 )

/* Base 64 encoding adds 1 byte for every 3 bytes being encoded. Also add 2 bytes for padding and 1 byte for null character. Must be a multiple of 4 */
#define CLIENT_WEBSOCKET_BASE64_KEY_LENGTH      ( 25 )

/*The server websocket key involves concatenation of GUID, hence the longer length*/
#define SERVER_WEBSOCKET_BASE64_SHA1_KEY_LENGTH   ( 29 )

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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/* @brief                               generate opening handshake header, send handshake and verify server response
 *
 * @param websocket                     websocket to send handshake on
 * @param websocket_header_fields       application populated header fields
 *
 * @return                              WICED_SUCCESS if successful, or WICED_ERROR.
 */
wiced_result_t wiced_establish_websocket_handshake( wiced_websocket_t* websocket, wiced_websocket_handshake_fields_t* handshake );

/* @brief                               get the subprotocol negotiated with server (assuming it was requested)
 *
 * @param subprotocol                   subprotocol used
 *
 * @return                              WICED_SUCCESS if successful, or WICED_ERROR.
 */
wiced_result_t wiced_get_websocket_subprotocol( char* subprotocol );

#ifdef __cplusplus
} /* extern "C" */
#endif
