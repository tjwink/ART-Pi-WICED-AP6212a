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
 * This file provides definitions and function prototypes for IoT Gateway
 * device
 *
 */
#ifndef _IOT_GATEWAY_H_
#define _IOT_GATEWAY_H_

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
 *                                Constants
 ******************************************************************************/

/* IoT Gateway App Timer Timeout in seconds  */
#define IOT_GATEWAY_APP_TIMEOUT_IN_SECONDS                 1

/* IoT Gateway App Fine Timer Timeout in milli seconds  */
#define IOT_GATEWAY_APP_FINE_TIMEOUT_IN_MS                 500

/* IoT Gateway Connection Idle  Timeout in milli seconds  */
#define IOT_GATEWAY_CONN_IDLE_TIMEOUT_IN_SECONDS           3

/* Debug - Timer tick print interval */
#define IOT_GATEWAY_APP_TIMER_DEBUG_PRINT_INTERVAL          1000

/*GPIO pins */ //Todo update as per the tag board
/*GPIO Settings */

#define IOT_GATEWAY_VS_ID                      WICED_NVRAM_VSID_START
#define IOT_GATEWAY_LOCAL_KEYS_VS_ID          ( WICED_NVRAM_VSID_START + 1 )
#define IOT_GATEWAY_PAIRED_KEYS_VS_ID          ( WICED_NVRAM_VSID_START + 2 )

#ifdef IOT_GATEWAY_POWER_SAVE_MODE
#define IOT_GATEWAY_SWITCH_TO_POWER_SAVE_TIMEOUT   10
#endif

#define WIFI_SSID_LEN 32
#define WIFI_PCODE_LEN 32
#define WIFI_SECURITY_LEN 32
/******************************************************************************
 *                          Constants
 ******************************************************************************/

/* UUID value of the IoT Gateway wifi config Service */
#define UUID_WIFI_CONFIG_SERVICE                    0x23, 0x20, 0x56, 0x7c, 0x05, 0xcf, 0x6e, 0xb4, 0xc3, 0x41, 0x77, 0x28, 0x51, 0x82, 0x7e, 0x1c
/* UUID value of the IoT Gateway Characteristic, ssid */
#define UUID_WIFI_CONFIG_CHARACTERISTIC_SSID      0x26, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8b
/* UUID value of the IoT Gateway Characteristic, security type */
#define UUID_WIFI_CONFIG_CHARACTERISTIC_SEC      0x26, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8c

/* UUID value of the IoT Gateway Characteristic, passcode */
#define UUID_WIFI_CONFIG_CHARACTERISTIC_PCODE      0x26, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8d

/* UUID value of the IoT Gateway Characteristic, connection */
#define UUID_WIFI_CONFIG_CHARACTERISTIC_CON      0x26, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8e

#define UUID_HTTP_PROXY_SERVICE                    0x24, 0x20, 0x56, 0x7c, 0x05, 0xcf, 0x6e, 0xb4, 0xc3, 0x41, 0x77, 0x28, 0x51, 0x82, 0x7e, 0x1c
#define UUID_HTTP_PROXY_SERVICE_URI      0x27, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8a
#define UUID_HTTP_PROXY_SERVICE_HTTP_HEADER      0x27, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8b
#define UUID_HTTP_PROXY_SERVICE_HTTP_ENTITY_BODY      0x27, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8c
#define UUID_HTTP_PROXY_SERVICE_HTTP_CONTROL_POINT      0x27, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8d
#define UUID_HTTP_PROXY_SERVICE_HTTP_STATUS_CODE      0x27, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8e
#define UUID_HTTP_PROXY_SERVICE_HTTPS_SCEURITY       0x27, 0xf6, 0x69, 0x91, 0x68, 0xee, 0xc2, 0xbe, 0x44, 0x4d, 0xb9, 0x5c, 0x3f, 0x2d, 0xc3, 0x8f
/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/
typedef enum
{
    HANDLE_WIFI_CONFIG_SERVICE = 0x70,

    HANDLE_WIFI_CONFIG_SERVICE_CHAR_SSID, // characteristic handle
    HANDLE_WIFI_CONFIG_SERVICE_CHAR_SSID_VAL, // char value handle

    HANDLE_WIFI_CONFIG_SERVICE_SEC, // characteristic handle
    HANDLE_WIFI_CONFIG_SERVICE_SEC_VAL, // char value handle

    HANDLE_WIFI_CONFIG_SERVICE_PCODE, // characteristic handle
    HANDLE_WIFI_CONFIG_SERVICE_PCODE_VAL, // char value handle
    HANDLE_WIFI_CONFIG_SERVICE_CON, // characteristic handle
    HANDLE_WIFI_CONFIG_SERVICE_CON_VAL, // char value handle

    HANDLE_HTTP_PROXY_SERVICE = 0x80,
    HANDLE_HTTP_PROXY_SERVICE_URI,
    HANDLE_HTTP_PROXY_SERVICE_URI_VAL,

    HANDLE_HTTP_PROXY_SERVICE_HTTP_HEADERS,
    HANDLE_HTTP_PROXY_SERVICE_HTTP_HEADERS_VAL,

    HANDLE_HTTP_PROXY_SERVICE_HTTP_ENTITY_BODY,
    HANDLE_HTTP_PROXY_SERVICE_HTTP_ENTITY_BODY_VAL,

    HANDLE_HTTP_PROXY_SERVICE_HTTP_CONTROL_POINT,
    HANDLE_HTTP_PROXY_SERVICE_HTTP_CONTROL_POINT_VAL,

    HANDLE_HTTP_PROXY_SERVICE_HTTP_STATUS_CODE,
    HANDLE_HTTP_PROXY_SERVICE_HTTP_STATUS_CODE_VAL,

    HANDLE_HTTP_PROXY_SERVICE_HTTPS_SCEURITY,
    HANDLE_HTTP_PROXY_SERVICE_HTTPS_SCEURITY_VAL,

    HANDLE_HTTP_OBSERVE_SERVICE = 0x90,
    HANDLE_HTTP_PROXY_SERVICE_OBSERVE_VAL,

    // Client Configuration
    HDLD_CURRENT_TIME_SERVICE_CURRENT_TIME_CLIENT_CONFIGURATION,
} iot_gateway_db_tags;

typedef PACKED struct
{
    uint8_t wifi_config_ssid[WIFI_SSID_LEN];
    uint8_t wifi_config_sec[WIFI_SECURITY_LEN];
    uint8_t wifi_config_pcode[WIFI_PCODE_LEN];
    uint32_t wifi_config_connect;
}wifi_info_t;

void gateway_gatt_send_notification( uint16_t conn_id, uint16_t code, uint16_t val_len, uint8_t *p_val );

#ifdef __cplusplus
}
#endif

#endif // _IOT_GATEWAY_H_

