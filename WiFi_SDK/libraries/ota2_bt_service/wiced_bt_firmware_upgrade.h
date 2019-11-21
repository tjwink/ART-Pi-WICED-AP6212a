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
 * WICED Firmware Upgrade Service
 *
 * This file provides definitions that can be used by a peer Firmware Upgrade
 * applications to perform Firmware Upgrade
 *
 */

/** @defgroup wiced_Firmware_upgrade   WICED Firmware Upgrade
 *
 *  WICED Firmware Upgrade functionality is used by the peer applications to
 *  upgrade firmware on WICED Bluetooth devices
 *
 */

#ifndef WICED_BT_FIRMWARE_UPGRADE__H
#define WICED_BT_FIRMWARE_UPGRADE__H
#ifdef __cplusplus
extern "C" {
#endif

// Please note that all UUIDs need to be reversed when publishing in the database

// {aE5D1E47-5C13-43A0-8635-82AD38A1381F}
// static const GUID WSRU_OTA_SERVICE =
// { 0xae5d1e47, 0x5c13, 0x43a0, { 0x86, 0x35, 0x82, 0xad, 0x38, 0xa1, 0x38, 0x1f } };
#define UUID_OTA_FW_UPGRADE_SERVICE                             0x1f, 0x38, 0xa1, 0x38, 0xad, 0x82, 0x35, 0x86, 0xa0, 0x43, 0x13, 0x5c, 0x47, 0x1e, 0x5d, 0xae

// {C7261110-F425-447A-A1BD-9D7246768BD8}
//static const GUID GUID_OTA_SEC_FW_UPGRADE_SERVICE =
//{ 0xc7261110, 0xf425, 0x447a,{ 0xa1, 0xbd, 0x9d, 0x72, 0x46, 0x76, 0x8b, 0xd8 } };
#define UUID_OTA_SEC_FW_UPGRADE_SERVICE                         0xd8, 0x8b, 0x76, 0x46, 0x72, 0x9d, 0xbd, 0xa1, 0x7a, 0x44, 0x25, 0xf4, 0x10, 0x11, 0x26, 0xc7

// {a3DD50BF-F7A7-4E99-838E-570A086C661B}
// static const GUID WSRU_OTA_CHARACTERISTIC_CONTROL_POINT =
// { 0xa3dd50bf, 0xf7a7, 0x4e99, { 0x83, 0x8e, 0x57, 0xa, 0x8, 0x6c, 0x66, 0x1b } };
#define UUID_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT        0x1b, 0x66, 0x6c, 0x08, 0x0a, 0x57, 0x8e, 0x83, 0x99, 0x4e, 0xa7, 0xf7, 0xbf, 0x50, 0xdd, 0xa3

// {a2E86C7A-D961-4091-B74F-2409E72EFE26}
// static const GUID WSRU_OTA_CHARACTERISTIC_DATA =
// { 0xa2e86c7a, 0xd961, 0x4091, { 0xb7, 0x4f, 0x24, 0x9, 0xe7, 0x2e, 0xfe, 0x26 } };
#define UUID_OTA_FW_UPGRADE_CHARACTERISTIC_DATA                 0x26, 0xfe, 0x2e, 0xe7, 0x09, 0x24, 0x4f, 0xb7, 0x91, 0x40, 0x61, 0xd9, 0x7a, 0x6c, 0xe8, 0xa2

// {a47F7608-2E2D-47EB-91E9-75D4EDC4DE4B}
// static const GUID WSRU_OTA_CHARACTERISTIC_APP_INFO =
// { 0xa47f7608, 0x2e2d, 0x47eb, { 0x91, 0xe9, 0x75, 0xd4, 0xed, 0xc4, 0xde, 0x4b } };

#define UUID_OTA_FW_UPGRADE_SERVICE_CHARACTERISTIC_APP_INFO     0x4b, 0xde, 0xc4, 0xed, 0xd4, 0x75, 0x3b, 0x91, 0xeb, 0x47, 0x2d, 0x2e, 0x08, 0x76, 0x7f, 0xa4

// Maximum data packet length used during FW download
#define WICED_OTA_FW_UPGRADE_MAX_DATA_LEN                        128

// command definitions for the OTA FW upgrade
#define WICED_OTA_UPGRADE_COMMAND_PREPARE_DOWNLOAD               1
#define WICED_OTA_UPGRADE_COMMAND_DOWNLOAD                       2
#define WICED_OTA_UPGRADE_COMMAND_VERIFY                         3
#define WICED_OTA_UPGRADE_COMMAND_FINISH                         4
#define WICED_OTA_UPGRADE_COMMAND_GET_STATUS                     5 // not currently used
#define WICED_OTA_UPGRADE_COMMAND_CLEAR_STATUS                   6 // not currently used
#define WICED_OTA_UPGRADE_COMMAND_ABORT                          7

// event definitions for the OTA FW upgrade
#define WICED_OTA_UPGRADE_STATUS_OK                              0
#define WICED_OTA_UPGRADE_STATUS_UNSUPPORTED_COMMAND             1
#define WICED_OTA_UPGRADE_STATUS_ILLEGAL_STATE                   2
#define WICED_OTA_UPGRADE_STATUS_VERIFICATION_FAILED             3
#define WICED_OTA_UPGRADE_STATUS_INVALID_IMAGE                   4
#define WICED_OTA_UPGRADE_STATUS_INVALID_IMAGE_SIZE              5
#define WICED_OTA_UPGRADE_STATUS_MORE_DATA                       6
#define WICED_OTA_UPGRADE_STATUS_INVALID_APPID                   7
#define WICED_OTA_UPGRADE_STATUS_INVALID_VERSION                 8
#define WICED_OTA_UPGRADE_STATUS_CONTINUE                        9

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif
