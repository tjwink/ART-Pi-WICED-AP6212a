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
 * Sample application to show how Wifi Device onboarding works.
 * This onboarding method is applicable for Wifi-only devices
 * and does not require any additional h/w like BLE or MFI.
 * This method hosts a HTTPS server first and then puts the device in AP + STA mode.
 *
 */

#include "device_onboarding.h"

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

static const wiced_ip_setting_t ap_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS( 255,255,255,  0 ) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
};

static wiced_semaphore_t app_semaphore;
static uint32_t onboarding_status;

/******************************************************
 *               Function Definitions
 ******************************************************/

/* When WiFi onboarding service is started, application will wait for
 * this callback(onboarding either failed or succeed), application then may
 * decide the next step. For example - if onboarding is success, application
 * may call the wifi_onboarding_stop and trigger a reboot.
 * On onboarding failure, application may retry certain number of times by starting the
 * onboarding service again(after stopping it first)
 */
static void app_wifi_onboarding_callback( uint32_t result )
{
    WPRINT_APP_INFO( ( "[App] WiFi Onboarding callback..." ) );
    if( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("Onboarding Failed\r\n" ) );
    }
    else
    {
        WPRINT_APP_INFO( ("Onboarding successfull\r\n" ) );
    }

    onboarding_status = result;

    wiced_rtos_set_semaphore(&app_semaphore);
}

void application_start(void)
{
    wiced_bool_t* device_configured;

    wiced_result_t result = WICED_ERROR;

    wiced_init();

    result = wiced_dct_read_lock( (void**) &device_configured, WICED_FALSE, DCT_WIFI_CONFIG_SECTION, OFFSETOF(platform_dct_wifi_config_t, device_configured), sizeof(wiced_bool_t) );

    if( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("[App] Error fetching platform DCT section\n") );
        goto exit;
    }

    if( *device_configured != WICED_TRUE )
    {
        wiced_rtos_init_semaphore(&app_semaphore);

        WPRINT_APP_INFO( ("[App] Starting Wifi Onboarding service...\n") );

        result = wiced_wifi_device_onboarding_start(&ap_ip_settings, app_wifi_onboarding_callback);

        if( result != WICED_SUCCESS )
        {
            WPRINT_APP_INFO( ("[App] Error Starting the on-boarding of device\n") );
            goto exit_onboarding;
        }

        WPRINT_APP_INFO( ("[App] Waiting for Onboarding callback...\n") );

        result = wiced_rtos_get_semaphore(&app_semaphore, WICED_NEVER_TIMEOUT);
        if( result != WICED_SUCCESS )
        {
            WPRINT_APP_INFO( ( "[App] Sempahore get error or timeout\r\n" ) );
            goto exit_onboarding;
        }

        WPRINT_APP_INFO( ("[App] Stopping Wifi Onboarding service...\n") );
        result = wiced_wifi_device_onboarding_stop();

        if( result != WICED_SUCCESS )
        {
            WPRINT_APP_INFO( ("[App] Error Stopping the on-boarding service\n") );
            goto exit_onboarding;
        }

        wiced_framework_reboot();
    }

    WPRINT_APP_INFO( ( "[App] Normal Application start\n" ) );

    result = wiced_network_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "[App] Failed to join network \n") );
        goto exit;
    }

    WPRINT_APP_INFO( ( "[App] Started STA successfully with saved configuration\n" ) );
    goto exit;

exit_onboarding:
    wiced_rtos_deinit_semaphore(&app_semaphore);
exit:
    wiced_dct_read_unlock(device_configured, WICED_FALSE);
    return;
}
