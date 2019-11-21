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
 * WPS Registrar Application
 *
 * This application snippet demonstrates how to use the
 * WPS Registrar on an existing softAP interface.
 *
 * Features demonstrated
 *  - Wi-Fi softAP mode
 *  - WPS Registrar
 *
 * Application Instructions
 *   1. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *   2. Using a WPS capable device (Windows PC, Android device),
 *      search for the WICED softAP "WPS_REGISTRAR_EXAMPLE" using
 *      the Wi-Fi setup method for the device
 *   3. Attempt to connect to the WICED softAP using WPS
 *   4. Connection progress is printed to the console
 *
 *   The application starts a softAP, and then immediately enables
 *   the WPS registrar which runs for 2 minutes before timing out.
 *   If your WPS capable device does not connect within the 2 minute
 *   window, reset the WICED eval board and try again.
 *
 *   Results are printed to the terminal.
 *
 * Notes
 *   1. A Windows client detects that WPS is enabled on the softAP
 *      and automatically attempts to connect using PBC mode
 *      Windows does not support PIN mode.
 *   2. Apple iOS devices such as iPhones DO NOT support WPS.
 *
 */

#include "wiced.h"
#include "wiced_wps.h"
#include "wps_host_interface.h"

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

static const wiced_wps_device_detail_t registrar_details =
{
    .device_name                      = PLATFORM,
    .manufacturer                     = "Cypress",
    .model_name                       = PLATFORM,
    .model_number                     = "1.0",
    .serial_number                    = "1408248",
    .device_category                  = WICED_WPS_DEVICE_NETWORK_INFRASTRUCTURE,
    .sub_category                     = 1,
    .config_methods                   = WPS_CONFIG_PUSH_BUTTON | WPS_CONFIG_VIRTUAL_PUSH_BUTTON | WPS_CONFIG_VIRTUAL_DISPLAY_PIN,
    .authentication_type_flags        = WPS_OPEN_AUTHENTICATION | WPS_WPA_PSK_AUTHENTICATION | WPS_WPA2_PSK_AUTHENTICATION,
    .encryption_type_flags            = WPS_NO_ENCRYPTION | WPS_MIXED_ENCRYPTION,
    .add_config_methods_to_probe_resp = 1,
};

static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255, 255, 255, 0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
};

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    wiced_wps_credential_t   ap_info;
    wiced_config_soft_ap_t*  softap_info;

    wiced_init();

    WPRINT_APP_INFO( ("Starting access point\r\n") );
    wiced_network_up( WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &device_init_ip_settings );

    /* Extract the settings from the WICED softAP to pass to the WPS registrar */
    wiced_dct_read_lock( (void**) &softap_info, WICED_FALSE, DCT_WIFI_CONFIG_SECTION, OFFSETOF(platform_dct_wifi_config_t, soft_ap_settings), sizeof(wiced_config_soft_ap_t) );
    memcpy( ap_info.passphrase, softap_info->security_key, sizeof(ap_info.passphrase) );
    memcpy( &ap_info.ssid,      &softap_info->SSID,        sizeof(wiced_ssid_t) );
    ap_info.passphrase_length = softap_info->security_key_length;
    ap_info.security          = softap_info->security;
    wiced_dct_read_unlock( softap_info, WICED_FALSE );

    WPRINT_APP_INFO( ("Starting WPS Registrar in PBC mode\r\n") );

    if ( wiced_wps_registrar( WICED_WPS_PBC_MODE, &registrar_details, "00000000", &ap_info, 1 ) == WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("WPS enrollment was successful\r\n") );
    }
    else
    {
        WPRINT_APP_INFO( ("WPS enrollment was not successful\r\n") );
    }
}
