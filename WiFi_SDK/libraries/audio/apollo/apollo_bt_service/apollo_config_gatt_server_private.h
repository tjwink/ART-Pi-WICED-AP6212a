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
 * This file provides definitions and function prototypes for Apollo config
 * device
 *
 */
#include "wiced_bt_dev.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 *                                Constants
 ******************************************************************************/

/* UUID value of Apollo config network info */
#define UUID_APOLLO_CONFIG_SERVICE                             0x46, 0xc2, 0x01, 0x0a, 0x04, 0x55, 0x78, 0xbc, 0x22, 0x4b, 0xba, 0x9c, 0x43, 0x45, 0x57, 0x04
#define UUID_APOLLO_CONFIG_CHARACTERISTIC_NW_MODE              0x61, 0xb8, 0x16, 0xde, 0xcf, 0xcf, 0x48, 0xb8, 0xc4, 0x4b, 0xd9, 0x9d, 0xef, 0x19, 0x1c, 0x5b
#define UUID_APOLLO_CONFIG_CHARACTERISTIC_NW_SECURITY          0xa1, 0x93, 0xcd, 0xa5, 0x84, 0x0a, 0xaf, 0xbb, 0x4a, 0x4c, 0xbb, 0xed, 0xa4, 0xab, 0xc2, 0xca
#define UUID_APOLLO_CONFIG_CHARACTERISTIC_NW_SSID              0x56, 0xb3, 0xf6, 0xce, 0xa6, 0x19, 0x08, 0x95, 0xad, 0x48, 0xaa, 0xee, 0x7c, 0xef, 0xa0, 0xac
#define UUID_APOLLO_CONFIG_CHARACTERISTIC_NW_PASSPHRASE        0xce, 0xa6, 0x15, 0xb4, 0x33, 0xd8, 0x76, 0xa8, 0x8b, 0x4c, 0xe4, 0x93, 0x33, 0xde, 0xb7, 0x40
#define UUID_APOLLO_CONFIG_CHARACTERISTIC_NW_CHANNEL           0xeb, 0xf0, 0x90, 0x54, 0xf9, 0xdc, 0x85, 0xba, 0x53, 0x44, 0xa9, 0x94, 0x2e, 0xe7, 0xf3, 0x87
#define UUID_APOLLO_CONFIG_CHARACTERISTIC_NW_BAND              0x31, 0xcb, 0x14, 0xad, 0xa0, 0x34, 0x39, 0x8e, 0xe4, 0x42, 0x07, 0x5b, 0x79, 0xbf, 0xb3, 0x31


// speaker configuration
/* UUID value of the Apollo config Characteristic, Configuration */
#define UUID_APOLLO_CONFIG_SPEAKER_SERVICE                     0x11, 0x25, 0xbf, 0xc7, 0x8a, 0x50, 0x59, 0x9e, 0x2f, 0x49, 0x2a, 0x5d, 0xdd, 0xda, 0x77, 0xc9
#define UUID_APOLLO_CONFIG_CHARACTERISTIC_SPEAKER_NAME         0x1d, 0xd1, 0x05, 0x07, 0x3f, 0x4b, 0x7e, 0xb9, 0x4a, 0x4e, 0x6d, 0x53, 0x5b, 0x2c, 0x35, 0xfc
#define UUID_APOLLO_CONFIG_CHARACTERISTIC_SPEAKER_CHANNEL      0x59, 0x97, 0x77, 0x48, 0x89, 0x00, 0x25, 0x93, 0xf7, 0x41, 0x8f, 0xf0, 0xbd, 0x08, 0x68, 0xda
#define UUID_APOLLO_CONFIG_CHARACTERISTIC_SPEAKER_OUTPUT_PORT  0xcd, 0xa4, 0x55, 0xc0, 0xb9, 0x37, 0x9c, 0xa6, 0x18, 0x48, 0xce, 0x76, 0xdc, 0xbd, 0xf0, 0xb3
#define UUID_APOLLO_CONFIG_CHARACTERISTIC_SPEAKER_OUTPUT_VOL   0x2c, 0x6b, 0xa0, 0xf7, 0x16, 0x50, 0x33, 0xb5, 0xa4, 0x4d, 0xcc, 0x2d, 0xab, 0x9e, 0x74, 0x98


// source/sender configuration
#define UUID_APOLLO_CONFIG_SOURCE_SERVICE                      0x13, 0x58, 0x4d, 0xda, 0x4b, 0x69, 0xfe, 0x8e, 0x28, 0x4e, 0x38, 0xc2, 0x71, 0x13, 0x1d, 0x52
#define UUID_APOLLO_CONFIG_CHARACTERISTIC_SOURCE_INPUT         0xb0, 0xf8, 0xb2, 0x73, 0x64, 0x82, 0xab, 0xb0, 0xa3, 0x4f, 0x92, 0x55, 0xd3, 0x34, 0xd7, 0x1c
#define UUID_APOLLO_CONFIG_CHARACTERISTIC_SOURCE_LOOPBACK      0xc0, 0x71, 0x34, 0xef, 0x54, 0x47, 0x93, 0x9b, 0xb7, 0x49, 0xd7, 0x2b, 0x10, 0x32, 0xf1, 0xfe
#define UUID_APOLLO_CONFIG_CHARACTERISTIC_SOURCE_INPUT_PORT    0xbc, 0x73, 0xda, 0x7f, 0x7a, 0x61, 0xa7, 0x85, 0x74, 0x4b, 0x9f, 0x56, 0xb5, 0xf9, 0xc5, 0xac
#define UUID_APOLLO_CONFIG_CHARACTERISTIC_SOURCE_INPUT_VOL     0xf6, 0x59, 0xe0, 0x25, 0x36, 0xd3, 0x10, 0xbc, 0x86, 0x4b, 0xab, 0xd0, 0xe6, 0x7c, 0xc7, 0x04


/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/
typedef enum
{
    HANDLE_APOLLO_CONFIG_GATT_SERVICE = 0x1,

    HANDLE_APOLLO_CONFIG_GAP_SERVICE = 0x14,
        HANDLE_APOLLO_CONFIG_GAP_SERVICE_CHAR_DEV_NAME,
        HANDLE_APOLLO_CONFIG_GAP_SERVICE_CHAR_DEV_NAME_VAL,

        HANDLE_APOLLO_CONFIG_GAP_SERVICE_CHAR_DEV_APPEARANCE,
        HANDLE_APOLLO_CONFIG_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,

    HANDLE_APOLLO_CONFIG_NW_SERVICE = 0x28,
        HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_MODE,
        HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_MODE_VAL,

        HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SECURITY,
        HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SECURITY_VAL,

        HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SSID,
        HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_SSID_VAL,

        HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_PASSPHRASE,
        HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_PASSPHRASE_VAL,

        HANDLE_APOLLO_CONFIG_NW_SERVICE_NW_CHAR_CHANNEL,
        HANDLE_APOLLO_CONFIG_NW_SERVICE_NW_CHAR_CHANNEL_VAL,

        HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_BAND,
        HANDLE_APOLLO_CONFIG_NW_SERVICE_CHAR_BAND_VAL, //0x34

    HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE = 0x40,
        HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_NAME,
        HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_NAME_VAL,

        HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_CHANNEL,
        HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_CHANNEL_VAL,

        HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_PORT,
        HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_PORT_VAL,

        HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_VOL,
        HANDLE_APOLLO_CONFIG_SPEAKER_SERVICE_CHAR_OUTPUT_VOL_VAL,//0x48


    HANDLE_APOLLO_CONFIG_SOURCE_SERVICE = 0x50,
        HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT,
        HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_VAL,

        HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_LOOPBACK,
        HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_LOOPBACK_VAL,

        HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_PORT,
        HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_PORT_VAL,

        HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_VOL,
        HANDLE_APOLLO_CONFIG_SOURCE_SERVICE_CHAR_INPUT_VOL_VAL, //0x59


    HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE = 0x60,
        HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_MFR_NAME,
        HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,

        HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_MODEL_NUM,
        HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,

        HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_SYSTEM_ID,
        HANDLE_APOLLO_CONFIG_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL, //0x66

    HANDLE_APOLLO_CONFIG_BATTERY_SERVICE = 0x70,
        HANDLE_APOLLO_CONFIG_BATTERY_SERVICE_CHAR_LEVEL,
        HANDLE_APOLLO_CONFIG_BATTERY_SERVICE_CHAR_LEVEL_VAL,

}apollo_config_db;

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t apollo_config_advertisement_stopped( void );

#ifdef __cplusplus
} /* extern "C" */
#endif

