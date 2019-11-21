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
 * Pre-built SDP server database
 *
 * (Available soon: tool for generating SDP database)
 *
 */
#include "wiced_bt_cfg.h"
#include "bt_rfcomm_server.h"
#include "wiced_bt_sdp.h"

/******************************************************
 *                     Macros
*******************************************************/

/******************************************************
 *               Variables Definitions
 *****************************************************/

const uint8_t wiced_bt_sdp_db[] =
{
    SDP_ATTR_SEQUENCE_1(166),                                           // length is the sum of all records

    // first SDP record Serial Port
    SDP_ATTR_SEQUENCE_1(93),                                            // 2 bytes, length of the record
    SDP_ATTR_RECORD_HANDLE(0x10001),                                    // 8 byte
    SDP_ATTR_CLASS_ID(UUID_SERVCLASS_SERIAL_PORT),                      // 8
    SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(BT_RFCOMM_SERVER_APP_SCN),       // 17
    SDP_ATTR_BROWSE_LIST,                                               // 8
    SDP_ATTR_LANGUAGE_BASE_ATTR_ID_LIST,                                // 14
    SDP_ATTR_PROFILE_DESC_LIST(0x1101, 0x0100),                         // 13
    SDP_ATTR_SERVICE_NAME(4),                                           // 9
        'P', 'o', 'r', 't',
    SDP_ATTR_SERVICE_DESCRIPTION(11),                                   // 16
        'S', 'e', 'r', 'i', 'a', 'l', ' ', 'P', 'o', 'r', 't',

    // second SDP record Device ID
    SDP_ATTR_SEQUENCE_1(69),                                            // 2 bytes, length of the record
    SDP_ATTR_RECORD_HANDLE(0x10002),                                    // 8 byte
    SDP_ATTR_CLASS_ID(UUID_SERVCLASS_PNP_INFORMATION),                  // 8
    SDP_ATTR_PROTOCOL_DESC_LIST(1),                                     // 18
    SDP_ATTR_UINT2(ATTR_ID_SPECIFICATION_ID, 0x103),                    // 6
    SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID, 0x0f),                            // 6
    SDP_ATTR_UINT2(ATTR_ID_PRODUCT_ID, 0x0144),                         // 6
    SDP_ATTR_UINT2(ATTR_ID_PRODUCT_VERSION, 0x0001),                    // 6
    SDP_ATTR_BOOLEAN(ATTR_ID_PRIMARY_RECORD, 0x01),                     // 5
    SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID_SOURCE, DI_VENDOR_ID_SOURCE_BTSIG) // 6
};

const uint16_t wiced_bt_sdp_db_size = (sizeof(wiced_bt_sdp_db));

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
