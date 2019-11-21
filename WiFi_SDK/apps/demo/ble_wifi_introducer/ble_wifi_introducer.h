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
 * This file provides definitions and function prototypes for WiFi Introducer Sensor
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
#define WIFI_INTRODUCER_MAX_NUM_CLIENTS 1

/* WiFi Introducer Sensor App Timer Timeout in seconds  */
#define WIFI_INTRODUCER_APP_TIMEOUT_IN_SECONDS                 1

/* WiFi Introducer Sensor App Fine Timer Timeout in milli seconds  */
#define WIFI_INTRODUCER_APP_FINE_TIMEOUT_IN_MS                 1000

/* WiFi Introducer Sensor Connection Idle  Timeout in milli seconds  */
#define WIFI_INTRODUCER_CONN_IDLE_TIMEOUT_IN_SECONDS           3

/* Maximum number of bond information stored in the DCT */
#define MAX_BOND_INFO (5)

#define WIFI_INTRODUCER_GATT_ATTRIBUTE_SIZE (22)


/******************************************************************************
 *                          Constants
 ******************************************************************************/

/* UUID value of Apollo config network info */
#define UUID_WIFI_INTRODUCER_SERVICE                             0x23, 0x20, 0x56, 0x7c, 0x05, 0xcf, 0x6e, 0xb4, 0xc3, 0x41, 0x77, 0x28, 0x51, 0x82, 0x7e, 0x1b
#define UUID_WIFI_INTRODUCER_CHARACTERISTIC_NW_SECURITY          0xa1, 0x93, 0xcd, 0xa5, 0x84, 0x0a, 0xaf, 0xbb, 0x4a, 0x4c, 0xbb, 0xed, 0xa4, 0xab, 0xc2, 0xca
#define UUID_WIFI_INTRODUCER_CHARACTERISTIC_NW_SSID              0x56, 0xb3, 0xf6, 0xce, 0xa6, 0x19, 0x08, 0x95, 0xad, 0x48, 0xaa, 0xee, 0x7c, 0xef, 0xa0, 0xac
#define UUID_WIFI_INTRODUCER_CHARACTERISTIC_NW_PASSPHRASE        0xce, 0xa6, 0x15, 0xb4, 0x33, 0xd8, 0x76, 0xa8, 0x8b, 0x4c, 0xe4, 0x93, 0x33, 0xde, 0xb7, 0x40

/* UUID value to  the Hello Sensor Characteristic, Value Notification */
#define UUID_WIFI_INTRODUCER_CHARACTERISTIC_NOTIFY              0x26, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8a


/******************************************************
 *                   Enumerations
 ******************************************************/

/**
 *  Bluetooth Smart address type
 */
typedef enum
{
    BT_SMART_ADDR_TYPE_PUBLIC  = 0x00, /**< Public address */
    BT_SMART_ADDR_TYPE_RANDOM  = 0x01  /**< Random address */
} wiced_bt_wifi_introducer_address_type_t;

/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/
typedef enum
{
    HANDLE_WIFI_INTRO_SENS_GATT_SERVICE = 0x1,

    HANDLE_WIFI_INTRO_SENS_GAP_SERVICE = 0x14,
        HANDLE_WIFI_INTRO_SENS_GAP_SERVICE_CHAR_DEV_NAME,
        HANDLE_WIFI_INTRO_SENS_GAP_SERVICE_CHAR_DEV_NAME_VAL,

        HANDLE_WIFI_INTRO_SENS_GAP_SERVICE_CHAR_DEV_APPEARANCE,
        HANDLE_WIFI_INTRO_SENS_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,

    HANDLE_WIFI_INTRO_SENS_NW_SERVICE = 0x28,


        HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_NOTIFY,
        HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_NOTIFY_VAL,
        HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_CFG_DESC,

        HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_SECURITY,
        HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_SECURITY_VAL,

        HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_SSID,
        HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_SSID_VAL,

        HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_PASSPHRASE,
        HANDLE_WIFI_INTRO_SENS_NW_SERVICE_CHAR_PASSPHRASE_VAL,


    HANDLE_WIFI_INTRO_SENS_DEV_INFO_SERVICE = 0x60,
        HANDLE_WIFI_INTRO_SENS_DEV_INFO_SERVICE_CHAR_MFR_NAME,
        HANDLE_WIFI_INTRO_SENS_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,

        HANDLE_WIFI_INTRO_SENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM,
        HANDLE_WIFI_INTRO_SENS_DEV_INFO_SERVICE_CHAR_MODEL_NUM_VAL,

        HANDLE_WIFI_INTRO_SENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID,
        HANDLE_WIFI_INTRO_SENS_DEV_INFO_SERVICE_CHAR_SYSTEM_ID_VAL, //0x66

    HANDLE_WIFI_INTRO_SENS_BATTERY_SERVICE = 0x70,
        HANDLE_WIFI_INTRO_SENS_BATTERY_SERVICE_CHAR_LEVEL,
        HANDLE_WIFI_INTRO_SENS_BATTERY_SERVICE_CHAR_LEVEL_VAL,
}wifi_introducer_db_tags;

typedef struct
{
    wiced_security_t    security;
    char                nw_ssid_name[WIFI_INTRODUCER_GATT_ATTRIBUTE_SIZE];
    char                nw_pass_phrase[WIFI_INTRODUCER_GATT_ATTRIBUTE_SIZE];
} wifi_introducer_gatt_server_dct_t;

#ifdef __cplusplus
} /*extern "C" */
#endif
