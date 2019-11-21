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
* WICED Bluetooth OTA Upgrade
*
* This file provides definitions and function prototypes for the
* WICED Bluetooth Over-the-Air Upgrade
*
*/
#ifndef WSR_UPGRADE_OTA_H
#define WSR_UPGRADE_OTA_H

#ifdef __cplusplus
extern "C" {
#endif

// Please note that all UUIDs need to be reversed when publishing in the database

// {9E5D1E47-5C13-43A0-8635-82AD38A1386F}
// static const GUID WSRU_OTA_SERVICE =
// { 0x9e5d1e47, 0x5c13, 0x43a0, { 0x86, 0x35, 0x82, 0xad, 0x38, 0xa1, 0x38, 0x6f } };
#define UUID_WSRU_OTA_SERVICE                      0x6f, 0x38, 0xa1, 0x38, 0xad, 0x82, 0x35, 0x86, 0xa0, 0x43, 0x13, 0x5c, 0x47, 0x1e, 0x5d, 0x9e

// {E3DD50BF-F7A7-4E99-838E-570A086C666B}
// static const GUID WSRU_OTA_CHARACTERISTIC_CONTROL_POINT =
// { 0xe3dd50bf, 0xf7a7, 0x4e99, { 0x83, 0x8e, 0x57, 0xa, 0x8, 0x6c, 0x66, 0x6b } };
#define UUID_WSRU_OTA_CHARACTERISTIC_CONTROL_POINT 0x6b, 0x66, 0x6c, 0x08, 0x0a, 0x57, 0x8e, 0x83, 0x99, 0x4e, 0xa7, 0xf7, 0xbf, 0x50, 0xdd, 0xe3

// {92E86C7A-D961-4091-B74F-2409E72EFE36}
// static const GUID WSRU_OTA_CHARACTERISTIC_DATA =
// { 0x92e86c7a, 0xd961, 0x4091, { 0xb7, 0x4f, 0x24, 0x9, 0xe7, 0x2e, 0xfe, 0x36 } };
#define UUID_WSRU_OTA_CHARACTERISTIC_DATA          0x36, 0xfe, 0x2e, 0xe7, 0x09, 0x24, 0x4f, 0xb7, 0x91, 0x40, 0x61, 0xd9, 0x7a, 0x6c, 0xe8, 0x92

// {347F7608-2E2D-47EB-91E9-75D4EDC4DE3B}
// static const GUID WSRU_OTA_CHARACTERISTIC_APP_INFO =
// { 0x347f7608, 0x2e2d, 0x47eb, { 0x91, 0xe9, 0x75, 0xd4, 0xed, 0xc4, 0xde, 0x3b } };

#define UUID_WSRU_OTA_CHARACTERISTIC_APP_INFO      0x3b, 0xde, 0xc4, 0xed, 0xd4, 0x75, 0x3b, 0x91, 0xeb, 0x47, 0x2d, 0x2e, 0x08, 0x76, 0x7f, 0x34

// command definitions for the OTA FW upgrade
#define WICED_OTA_UPGRADE_COMMAND_PREPARE_DOWNLOAD                 1
#define WICED_OTA_UPGRADE_COMMAND_DOWNLOAD                         2
#define WICED_OTA_UPGRADE_COMMAND_VERIFY                           3
#define WICED_OTA_UPGRADE_COMMAND_FINISH                           4
#define WICED_OTA_UPGRADE_COMMAND_GET_STATUS                       5 // not currently used
#define WICED_OTA_UPGRADE_COMMAND_CLEAR_STATUS                     6 // not currently used
#define WICED_OTA_UPGRADE_COMMAND_ABORT                            7

// event definitions for the OTA FW upgrade
#define WICED_OTA_UPGRADE_STATUS_OK                                0
#define WICED_OTA_UPGRADE_STATUS_UNSUPPORTED_COMMAND               1
#define WICED_OTA_UPGRADE_STATUS_ILLEGAL_STATE                     2
#define WICED_OTA_UPGRADE_STATUS_VERIFICATION_FAILED               3
#define WICED_OTA_UPGRADE_STATUS_INVALID_IMAGE                     4
#define WICED_OTA_UPGRADE_STATUS_INVALID_IMAGE_SIZE                5
#define WICED_OTA_UPGRADE_STATUS_MORE_DATA                         6
#define WICED_OTA_UPGRADE_STATUS_INVALID_APPID                     7
#define WICED_OTA_UPGRADE_STATUS_INVALID_VERSION                   8
#define WICED_OTA_UPGRADE_STATUS_CONTINUE                          9

/******************************************************
 *                      Constants
 ******************************************************/

// device states during OTA FW upgrade
#define WSRU_OTA_STATE_IDLE                   0
#define WSRU_OTA_STATE_READY_FOR_DOWNLOAD     1
#define WSRU_OTA_STATE_DATA_TRANSFER          2
#define WSRU_OTA_STATE_VERIFICATION           3
#define WSRU_OTA_STATE_VERIFIED               4
#define WSRU_OTA_STATE_ABORTED                5


#define WSRU_OTA_READ_CHUNK                     128
#define WSRU_OTA_CHUNK_SIZE_TO_COMMIT           128

/******************************************************
 *                     Structures
 ******************************************************/

// structure to pass application information to upgrade application
#pragma pack(1)
typedef struct
{
    uint16_t id;
    uint8_t  version_major;
    uint8_t  version_minor;
} wsru_ota_app_info_t;
#pragma pack()

typedef struct
{
    int32_t   wsru_state;
    uint8_t   wsru_bdaddr[6];               // BDADDR of connected device
    uint16_t  wsru_client_configuration;    // characteristic client configuration descriptor
    uint8_t   wsru_status;                  // Current status
    uint16_t  wsru_current_offset;          // Offset in the image to store the data
    int32_t   wsru_total_len;               // Total length expected from the host
    int32_t   wsru_current_block_offset;
    int32_t   wsru_total_offset;
    uint32_t  wsru_crc32;
    uint8_t   wsru_indication_sent;
    uint8_t   wsru_read_buffer[WSRU_OTA_CHUNK_SIZE_TO_COMMIT];
} wsru_ota_t;

wiced_bool_t    wsr_upgrade_ota_init(  );
void            wsr_upgrade_ota_connected( uint8_t *p_bda);
int32_t         wsr_upgrade_ota_handle_command( uint16_t conn_id, uint8_t *data, int32_t len );
int32_t         wsr_upgrade_ota_handle_configuration( uint16_t conn_id, uint8_t *data, int32_t len );
int32_t         wsr_upgrade_ota_handle_data( uint16_t conn_id, uint8_t *data, int32_t len );
void            wsr_upgrade_indication_confirm();
void            wsr_upgrade_ota_set_client_configuration( uint16_t client_config );

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif
