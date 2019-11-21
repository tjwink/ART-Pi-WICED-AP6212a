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
 * WPS Enrollee Application
 *
 * This application snippet demonstrates how to use the WPS Enrollee
 *
 * Features demonstrated
 *  - WPS Enrollee
 *
 * Application Instructions
 *   1. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *   2. Using a WPS capable Wi-Fi Access Point, press the WPS button on
 *      the AP to start a WPS setup session
 *   3. Connection progress is printed to the console
 *
 * The WPS Enrollee runs for up to 2 minutes before either successfully
 * connecting to the AP or timing out.
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

static const wiced_wps_device_detail_t enrollee_details =
{
    .device_name               = PLATFORM,
    .manufacturer              = "Cypress",
    .model_name                = PLATFORM,
    .model_number              = "1.0",
    .serial_number             = "1408248",
    .device_category           = WICED_WPS_DEVICE_COMPUTER,
    .sub_category              = 7,
    .config_methods            = WPS_CONFIG_LABEL | WPS_CONFIG_VIRTUAL_PUSH_BUTTON | WPS_CONFIG_VIRTUAL_DISPLAY_PIN,
    .authentication_type_flags = WPS_OPEN_AUTHENTICATION | WPS_WPA_PSK_AUTHENTICATION | WPS_WPA2_PSK_AUTHENTICATION,
    .encryption_type_flags     = WPS_NO_ENCRYPTION | WPS_MIXED_ENCRYPTION,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    wiced_wps_credential_t      credential[CONFIG_AP_LIST_SIZE];
    platform_dct_wifi_config_t* wifi_config;
    wiced_result_t              result = WICED_SUCCESS;
    wiced_band_list_t           band_list;
    int32_t                     band = 0;
    uint32_t                    channel;

    memset( &credential, 0, sizeof( credential ) );

    wiced_init();

    if ( wwd_wifi_get_supported_band_list( &band_list ) == WWD_SUCCESS )
    {
        if ( band_list.number_of_bands == 2 )
        {
            wwd_wifi_set_preferred_association_band( WLC_BAND_5G );
            if ( wwd_wifi_get_preferred_association_band( &band ) == WWD_SUCCESS )
            {
                WPRINT_APP_INFO( ("Preferred band for association is 5GHz\n") );
            }
        }
    }

    WPRINT_APP_INFO( ("Starting WPS Enrollee in PBC mode. Press the WPS button on your AP now.\n") );

    result = wiced_wps_enrollee( WICED_WPS_PBC_MODE, &enrollee_details, "00000000", credential, CONFIG_AP_LIST_SIZE );
    if (WICED_SUCCESS == result)
    {
        uint32_t i;

        WPRINT_APP_INFO( ("WPS enrollment was successful\n") );

        /* Copy Wi-Fi credentials obtained from WPS to the Wi-Fi config section in the DCT */
        wiced_dct_read_lock( (void**) &wifi_config, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, 0, sizeof( platform_dct_wifi_config_t ) );

        for ( i = 0; i < CONFIG_AP_LIST_SIZE; i++ )
        {
            memcpy( (void *)&wifi_config->stored_ap_list[ i ].details.SSID, &credential[ i ].ssid, sizeof(wiced_ssid_t) );
            memcpy( wifi_config->stored_ap_list[ i ].security_key, &credential[ i ].passphrase, credential[ i ].passphrase_length );

            wifi_config->stored_ap_list[i].details.security    = credential[i].security;
            wifi_config->stored_ap_list[i].security_key_length = credential[i].passphrase_length;
        }

        wiced_dct_write ( (const void*)wifi_config, DCT_WIFI_CONFIG_SECTION, 0, sizeof (platform_dct_wifi_config_t) );

        wiced_dct_read_unlock( (void*)wifi_config, WICED_TRUE );

        /* AP credentials have been stored in the DCT, now join the AP */
        wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

        /* Get the channel that the radio is on. For 40MHz capable radios this may display the center frequency of the channel rather than the primary channel of the WLAN */
        if ( wwd_wifi_get_channel( WWD_STA_INTERFACE, &channel ) == WWD_SUCCESS )
        {
            WPRINT_APP_INFO( ("Associated on channel: %u\n", (unsigned int) channel ) );
        }
    }
    else
    {
        WPRINT_APP_INFO( ("WPS enrollment was not successful\r\n") );
    }
}
