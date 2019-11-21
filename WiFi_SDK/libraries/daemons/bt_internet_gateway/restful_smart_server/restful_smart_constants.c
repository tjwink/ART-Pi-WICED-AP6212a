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
#include "restful_smart_constants.h"
#include "restful_smart_server.h"
#include "restful_smart_response.h"
#include "restful_smart_ble.h"
#include "http_server.h"

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
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

//const char* const restful_smart_http_status_table[] =
//{
//    [BT_REST_GATEWAY_STATUS_200] = "HTTP/1.1 - 200 - OK - application/json",
//    [BT_REST_GATEWAY_STATUS_400] = "HTTP/1.1 - 400 - Bad Request",
//    [BT_REST_GATEWAY_STATUS_403] = "HTTP/1.1 - 403 - Forbidden",
//    [BT_REST_GATEWAY_STATUS_404] = "HTTP/1.1 - 404 - Not Found",
//    [BT_REST_GATEWAY_STATUS_405] = "HTTP/1.1 - 405 - Method not allowed",
//    [BT_REST_GATEWAY_STATUS_406] = "HTTP/1.1 - 406 - Not acceptable",
//    [BT_REST_GATEWAY_STATUS_412] = "HTTP/1.1 - 412 - Precondition failed",
//    [BT_REST_GATEWAY_STATUS_415] = "HTTP/1.1 - 415 - Unsupported media type",
//    [BT_REST_GATEWAY_STATUS_504] = "HTTP/1.1 - 504 - Not able to connect",
//};

const http_status_codes_t restful_smart_http_status_table[] =
{
    [REST_SMART_STATUS_200] = HTTP_200_TYPE,
    [REST_SMART_STATUS_400] = HTTP_400_TYPE,
    [REST_SMART_STATUS_403] = HTTP_403_TYPE,
    [REST_SMART_STATUS_404] = HTTP_404_TYPE,
    [REST_SMART_STATUS_405] = HTTP_405_TYPE,
    [REST_SMART_STATUS_406] = HTTP_406_TYPE,
    [REST_SMART_STATUS_412] = HTTP_412_TYPE,
    [REST_SMART_STATUS_415] = HTTP_415_TYPE,
    [REST_SMART_STATUS_504] = HTTP_504_TYPE,
};

const pairing_status_t pairing_status_table[] =
{
    [PAIRING_FAILED]              = { "0", "Pairing Failed"                             },
    [PAIRING_SUCCESSFUL]          = { "1", "Pairing Successful"                         },
    [PAIRING_ABORTED]             = { "2", "Pairing Aborted"                            },
    [LE_LEGACY_OOB_EXPECTED]      = { "3", "LE Legacy Pairing OOB Expected"             },
    [LE_SECURE_OOB_EXPECTED]      = { "4", "LE Secure Connections Pairing OOB Expected" },
    [PASSKEY_INPUT_EXPECTED]      = { "5", "Passkey Input Expected"                     },
    [PASSKEY_DISPLAY_EXPECTED]    = { "6", "Passkey Display Expected"                   },
    [NUMERIC_COMPARISON_EXPECTED] = { "7", "Numeric Comparison Expected"                },
};

