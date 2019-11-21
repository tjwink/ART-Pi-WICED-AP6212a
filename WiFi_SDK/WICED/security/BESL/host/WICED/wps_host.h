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

#include "wwd_structures.h"
#include "wwd_wlioctl.h"
#include "wwd_debug.h"
#include "wiced_utilities.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *            Constants
 ******************************************************/
#define WPS_ASSERT(x)

/******************************************************
 *            Enumerations
 ******************************************************/

typedef enum
{
    BESL_WPS_PBC_MODE = 1,
    BESL_WPS_PIN_MODE = 2
} besl_wps_mode_t;

typedef enum
{
    BESL_WPS_DEVICE_COMPUTER               = 1,
    BESL_WPS_DEVICE_INPUT                  = 2,
    BESL_WPS_DEVICE_PRINT_SCAN_FAX_COPY    = 3,
    BESL_WPS_DEVICE_CAMERA                 = 4,
    BESL_WPS_DEVICE_STORAGE                = 5,
    BESL_WPS_DEVICE_NETWORK_INFRASTRUCTURE = 6,
    BESL_WPS_DEVICE_DISPLAY                = 7,
    BESL_WPS_DEVICE_MULTIMEDIA             = 8,
    BESL_WPS_DEVICE_GAMING                 = 9,
    BESL_WPS_DEVICE_TELEPHONE              = 10,
    BESL_WPS_DEVICE_AUDIO                  = 11,
    BESL_WPS_DEVICE_OTHER                  = 0xFF,
} besl_wps_device_category_t;

/******************************************************
 *             Structures
 ******************************************************/

typedef void (*wps_scan_handler_t)(wl_escan_result_t* result, void* user_data);

typedef struct
{
    besl_wps_device_category_t device_category;
    uint16_t sub_category;
    char*    device_name;
    char*    manufacturer;
    char*    model_name;
    char*    model_number;
    char*    serial_number;
    uint32_t config_methods;
    uint32_t os_version;
    uint16_t authentication_type_flags;
    uint16_t encryption_type_flags;
    uint8_t  add_config_methods_to_probe_resp;
} besl_wps_device_detail_t;

typedef struct
{
    wiced_ssid_t       ssid;
    wiced_security_t security;
    uint8_t          passphrase[64];
    uint16_t         passphrase_length;
} besl_wps_credential_t;

typedef enum
{
    BESL_WPS_CONFIG_USBA                  = 0x0001,
    BESL_WPS_CONFIG_ETHERNET              = 0x0002,
    BESL_WPS_CONFIG_LABEL                 = 0x0004,
    BESL_WPS_CONFIG_DISPLAY               = 0x0008,
    BESL_WPS_CONFIG_EXTERNAL_NFC_TOKEN    = 0x0010,
    BESL_WPS_CONFIG_INTEGRATED_NFC_TOKEN  = 0x0020,
    BESL_WPS_CONFIG_NFC_INTERFACE         = 0x0040,
    BESL_WPS_CONFIG_PUSH_BUTTON           = 0x0080,
    BESL_WPS_CONFIG_KEYPAD                = 0x0100,
    BESL_WPS_CONFIG_VIRTUAL_PUSH_BUTTON   = 0x0280,
    BESL_WPS_CONFIG_PHYSICAL_PUSH_BUTTON  = 0x0480,
    BESL_WPS_CONFIG_VIRTUAL_DISPLAY_PIN   = 0x2008,
    BESL_WPS_CONFIG_PHYSICAL_DISPLAY_PIN  = 0x4008
} besl_wps_configuration_method_t;



/******************************************************
 *             Function declarations
 ******************************************************/

#ifdef __cplusplus
} /* extern "C" */
#endif
