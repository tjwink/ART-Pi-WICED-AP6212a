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

/**
 * @file
 * This file performs the WPS negotiation for the Appliance App
 *
 */


#include "wiced_debug.h"
#include "wps_host.h"
#include "string.h"
#include "wiced_wps.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define MAX_CREDENTIALS     5

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

static const wiced_wps_device_detail_t details =
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

void do_wps( wiced_wps_mode_t wps_mode, char* pin )
{
    WINFO_APP(("Starting WPS enrollee. Press WPS button on access point\n"));
    wiced_wps_credential_t credential[MAX_CREDENTIALS];
    memset(credential, 0, MAX_CREDENTIALS*sizeof(wiced_wps_credential_t));
    besl_wps_init();
    besl_wps_enrollee( wps_mode, &details, pin, credential, MAX_CREDENTIALS, NULL);
    besl_wps_deinit();

    /* Check if we got valid credentials */
    if (credential[0].SSID.length == 0)
    {
        /* WPS failed. Abort */
        WINFO_APP(("No access point found. Halting\n"));
        while(1);
    }

    /* Attempt to join the Wi-Fi network. Try all the credentials in a round robin fashion */
    int a = 0;
    wiced_result_t ret;
    wiced_wps_credential_t* cred;
    do
    {
        cred = &credential[a];
        WINFO_APP(("Joining : %s\n", cred->SSID.value));
        ret = wiced_wifi_join( (char*)cred->SSID.value, cred->security, (uint8_t*) cred->passphrase, cred->passphrase_length, NULL );
        if (ret != WICED_SUCCESS)
        {
            WINFO_APP(("Failed to join  : %s   .. retrying\n", cred->SSID.value));
            ++a;
            if (credential[a].SSID.length == 0)
            {
                a = 0;
            }
        }
    }
    while (ret != WICED_SUCCESS);
    WINFO_APP(("Successfully joined : %s\n", cred->SSID.value));

}
