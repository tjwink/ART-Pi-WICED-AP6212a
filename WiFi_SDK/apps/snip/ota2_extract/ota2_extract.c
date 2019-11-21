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
 * Over-The-Air2 (OTA2) Upgrade & Factory Reset Application
 *
 * ------------------------------------------------------
 * PLEASE read the following documentation before trying
 * this application!
 * ------------------------------------------------------
 *
 * This application demonstrates how to use the WICED build system with a WICED development
 * board to demonstrate the OTA upgrade and factory reset capability.
 *
 * Features demonstrated
 *  - WICED OTA2 Upgrade
 *  - Factory Reset process
 *
 * WICED Multi-Application Support:
 * =================================
 * As of WICED-SDK 3.1.1, WICED Application Framework (WAF) supports loading and storing of multiple
 * application binaries in the external serial flash. Up to 8 binaries are supported. The first
 * five binaries are reserved for internal usage and the last three binaries are free for users to use.
 * The binaries are organised as follows:
 *  - Factory reset application (FR_APP)
 *  - DCT backup image (DCT_IMAGE)
 *  - OTA upgrade application (OTA_APP)
 *  - Resources file system (FILESYSTEM_IMAGE)
 *  - WIFI firmware (WIFI_FIRMWARE)
 *  - Application 0 (APP0)
 *  - Application 1 (APP1)
 *  - Application 2 (APP2)
 *
 * OTA Snippet Application:
 * =========================
 * This snippet application demonstrates how to use WICED multi-application support to perform
 * factory reset and OTA upgrade. The following steps assume you have a BCM943362WCD4 WICED evaluation board
 * (a BCM943362WCD4 WICED module on a WICED evaluation board). If your board is different, substitute
 * BCM943362WCD4 for your platform name.
 *
 * Prepare the WICED evaluation board for OTA upgrade
 *     1. Build the snip.ota_fr application to function as your factory reset and OTA
 *        application
 *     2. Notice that the factory reset application (FR_APP) is set in <WICED-SDK>/apps/snip/ota_fr/ota_fr.mk
 *        to point to the snip.ota_fr application elf file.
 *     3. Run the following make target to download your production and factory reset applications
 *        to the board:
 *            make snip.ota_fr-BCM943362WCD4 download download_apps run
 *     4. Build an application that will upgrade the current application.
 *        For this example we will build the snip.scan application:
 *            make snip.scan-BCM943362WCD4
 *
 * Upgrade the application running on the WICED evaluation board
 *   After carefully completing the above steps, the WICED evaluation board is ready to for an OTA upgrade.
 *   'Loading OTA upgrade app' log message is displayed in the terminal when the OTA upgrade application is ready.
 *   Work through the following steps:
 *   - Using the Wi-Fi connection manager on your computer, search for, and connect to,
 *     the Wi-Fi AP called : Wiced_Device
 *   - Open a web browser and enter the IP address of the eval board: 192.168.10.1 (default)
 *   - After a short period, the WICED Webserver OTA Upgrade webpage appears
 *   - Click 'Choose File' and navigate to the file
 *     <WICED-SDK>/build/snip_scan-BCM943362WCD4/Binary/snip_scan-BCM943362WCD4.stripped.elf
 *     (this is the snip.scan application binary file that was created in step 2 above)
 *   - Click 'Open' (the dialogue box disappears)
 *   - Click 'Start upgrade' to begin the upgrade process
 *      - The progress bar within the webpage indicates the upgrade progress
 *      - The webpage displays 'Transfer completed, WICED device is rebooting now' when the
 *        process completes
 *   - With the upgrade complete, the snip.scan application runs and Wi-Fi scan results are regularly
 *     printed to the terminal
 *
 * Perform factory reset on the WICED evaluation board
 *   To perform factory reset on the WICED evaluation board, work through the following steps:
 *   - Push and hold the SW1 button THEN momentarily press and release the Reset button.
 *     The D1 LED flashes quickly to indicate factory reset will occur *IF* SW1 is held
 *     for a further 5 seconds. Continue to hold SW1.
 *   - After the copy process is complete, the WICED evaluation board reboots and runs the factory
 *     reset (OTA_FR) application. Observe the log messages at the terminal to confirm the factory reset
 *     is completed successfully.
 *
 */

#include "wiced.h"
#include "platform_init.h"
#include "wwd_debug.h"
#include "waf_platform.h"
#include "wiced_framework.h"
#include "wiced_ota2_server.h"
#include "wiced_ota2_image.h"

#if 1   /* Set this to 0 for no printing in this file */
#include "mini_printf.h"
#define OTA2_EXTRACT_PRINTF(arg) {mini_printf arg;}
#else
#define OTA2_EXTRACT_PRINTF(arg)
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum
{
    EXTRACT_RESULT_OK = 0,
    EXTRACT_RESULT_ERROR_BATTERY_LOW,
    EXTRACT_RESULT_ERROR_DCT_READ_FAIL,
    EXTRACT_RESULT_ERROR_DCT_COPY_FAIL,
    EXTRACT_RESULT_ERROR_SOFTAP_FAIL,
    EXTRACT_RESULT_ERROR_UPGRADE_FAIL,
    EXTRACT_RESULT_ERROR_FACTORY_RESET_FAIL,

} extractor_extract_result_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t ota2_extract_check_battery_level( void );

/******************************************************
 *               Variable Definitions
 ******************************************************/

#ifndef PLATFORM_OTA2_SOFTAP_NOT_SUPPORTED
static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255, 255, 255, 0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
};
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    wiced_result_t              result;
    extractor_extract_result_t  extract_result;
    platform_dct_ota2_config_t  dct_ota2_config;

    /*
     * Initially:
     * We don't use the normal wiced_init(), as we don't want wiced_wlan_connectivity_init( ); unless we are running SoftAP
     * We don't use wiced_core_init(), as we don't want to use wiced_platform_init( ), as the filesystem may be invalid, and we don't want to assert
     */
    platform_init_external_devices();

    /* initialize DCT mutex */
    wiced_dct_init();

    OTA2_EXTRACT_PRINTF(("\r\nHi, I'm the OTA2 extraction app (ota2_extract).\r\n"));

    extract_result = EXTRACT_RESULT_OK;

    if ( ota2_extract_check_battery_level() != WICED_SUCCESS)
    {
        extract_result = EXTRACT_RESULT_ERROR_BATTERY_LOW;
    }

    if (wiced_dct_read_with_copy( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) ) != WICED_SUCCESS)
    {
        extract_result = EXTRACT_RESULT_ERROR_DCT_READ_FAIL;
    }

    if (extract_result == EXTRACT_RESULT_OK)
    {

        switch (dct_ota2_config.boot_type)
        {
            default:
                /* we should never get here */
                OTA2_EXTRACT_PRINTF(("Extract Unexpected boot_type : %d!\r\n", dct_ota2_config.boot_type ));
                break;

            case OTA2_BOOT_EXTRACT_FACTORY_RESET:
                OTA2_EXTRACT_PRINTF(("Extract the Factory Reset!\r\n"));
                result = wiced_dct_ota2_save_copy(OTA2_BOOT_FACTORY_RESET);
                if (result != WICED_SUCCESS)
                {
                    extract_result = EXTRACT_RESULT_ERROR_DCT_COPY_FAIL;
                    break;
                }
                result = wiced_ota2_image_extract ( WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP );
                if (result != WICED_SUCCESS)
                {
                    /* NOTE: boot_type is still OTA2_BOOT_FAILSAFE_FACTORY_RESET, so bootloader will run ota2_failsafe */
                    extract_result = EXTRACT_RESULT_ERROR_FACTORY_RESET_FAIL;
                }
                else
                {
                    extract_result = EXTRACT_RESULT_OK;
                    /* Instruct bootloader that we completed the extraction successfully */
                    dct_ota2_config.boot_type = OTA2_BOOT_FACTORY_RESET;
                    wiced_dct_write( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) );
                }
                break;

            case OTA2_BOOT_EXTRACT_UPDATE:
                OTA2_EXTRACT_PRINTF(("Extract from Staged Area!\r\n"));
                result = wiced_dct_ota2_save_copy(OTA2_BOOT_UPDATE);
                if (result != WICED_SUCCESS)
                {
                    extract_result = EXTRACT_RESULT_ERROR_DCT_COPY_FAIL;
                    break;
                }
                result = wiced_ota2_image_extract ( WICED_OTA2_IMAGE_TYPE_STAGED );
                if (result != WICED_SUCCESS)
                {
                    /* NOTE: boot_type is still OTA2_BOOT_FAILSAFE_UPDATE, so bootloader will run ota2_failsafe */
                    extract_result = EXTRACT_RESULT_ERROR_UPGRADE_FAIL;
                    wiced_ota2_image_fakery(WICED_OTA2_IMAGE_INVALID);
                    dct_ota2_config.boot_type = OTA2_BOOT_EXTRACT_FACTORY_RESET;
                    wiced_dct_write( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) );
                }
                else
                {
                    extract_result = EXTRACT_RESULT_OK;
                    /* Instruct bootloader that we completed the extraction successfully */
                    dct_ota2_config.boot_type = OTA2_BOOT_UPDATE;
                    wiced_dct_write( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) );
                }
                break;
#ifndef PLATFORM_OTA2_SOFTAP_NOT_SUPPORTED
            case OTA2_BOOT_SOFTAP_UPDATE:
            {
                wiced_config_soft_ap_t* soft_ap;
                /* we need Wifi and other services for SoftAP */
                wiced_init( );

                /* set back to normal so we don't run SoftAP if there is a reset during download */
                dct_ota2_config.boot_type = OTA2_BOOT_NORMAL;
                wiced_dct_write( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) );
                wiced_waf_app_set_boot( DCT_APP0_INDEX, PLATFORM_DEFAULT_LOAD );

                /* Get the SoftAP name and output to console */
                result = wiced_dct_read_lock( (void**) &soft_ap, WICED_TRUE, DCT_WIFI_CONFIG_SECTION, OFFSETOF(platform_dct_wifi_config_t, soft_ap_settings), sizeof(wiced_config_soft_ap_t) );
                if ( result == WICED_SUCCESS )
                {
                    char ssid_name[33] = { 0 };
                    memcpy(ssid_name, soft_ap->SSID.value, soft_ap->SSID.length);
                    OTA2_EXTRACT_PRINTF(("SoftAP start, AP name: %s\r\n", ssid_name));
                    wiced_dct_read_unlock( soft_ap, WICED_FALSE );
                }
                else
                {
                    OTA2_EXTRACT_PRINTF(("SoftAP start\r\n"));
                }

                if (wiced_network_up( WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &device_init_ip_settings ) != WICED_SUCCESS)
                {
                    extract_result = EXTRACT_RESULT_ERROR_SOFTAP_FAIL;
                    break;
                }

                if (wiced_ota_server_start( WICED_AP_INTERFACE ) != WICED_SUCCESS)
                {
                    extract_result = EXTRACT_RESULT_ERROR_SOFTAP_FAIL;
                    break;
                }

                /* wait for new SoftAP to finish, server will reset when done. (see libraries/daemons/ota2_server/ota2_server.c) */
                while(1)
                {
                    wiced_rtos_delay_milliseconds( 100 );
                }
            }
            break;
#endif
        }
    } /* check battery level */


    switch ( extract_result )
    {
        case EXTRACT_RESULT_OK:
            /* set APP0 as initial app to run - we set it to OTA_APP to get to this application */
            wiced_waf_app_set_boot( DCT_APP0_INDEX, PLATFORM_DEFAULT_LOAD );

            /* clear force_factory_reset Flag */
            dct_ota2_config.force_factory_reset = 0;
            wiced_dct_write( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) );
            break;
        case EXTRACT_RESULT_ERROR_BATTERY_LOW:
            OTA2_EXTRACT_PRINTF(("Extract BATTERY LOW - DID  NOT EXTRACT!\r\n"));
            break;
        case EXTRACT_RESULT_ERROR_DCT_READ_FAIL:
            OTA2_EXTRACT_PRINTF(("Extract DCT Read failed - DID  NOT EXTRACT!\r\n"));
            break;
        case EXTRACT_RESULT_ERROR_DCT_COPY_FAIL:
            OTA2_EXTRACT_PRINTF(("Extract DCT copy failed - DID  NOT EXTRACT!\r\n"));
            break;
        case EXTRACT_RESULT_ERROR_UPGRADE_FAIL:
            OTA2_EXTRACT_PRINTF(("Extract from Staged Area FAILED!\r\n"));
            break;
        case EXTRACT_RESULT_ERROR_FACTORY_RESET_FAIL:
            OTA2_EXTRACT_PRINTF(("Extract From Factory Reset FAILED!\r\n"));
            break;
        case EXTRACT_RESULT_ERROR_SOFTAP_FAIL:
            OTA2_EXTRACT_PRINTF(("Extract Start SoftAP FAILED!\r\n"));
            break;
    }

#if defined(OTA2_REBOOT_AFTER_EXTRACTION)
    OTA2_EXTRACT_PRINTF(("Extract done - REBOOTING\r\n"));
    wiced_framework_reboot();
#else
    OTA2_EXTRACT_PRINTF(("Manually Reboot NOW !!!\r\n"));
    while ( 1 )
    {
        wiced_rtos_delay_milliseconds( 100 );
    }
#endif
}

/* Returns WICED_ERROR if battery level considered too low */
wiced_result_t ota2_extract_check_battery_level( void )
{
#ifdef CHECK_BATTERY_LEVEL_BEFORE_OTA2_UPGRADE
    /* check for battery level before doing large amount of writing */
    if (platform_check_battery_level(CHECK_BATTERY_LEVEL_OTA2_UPGRADE_MINIMUM) != WICED_SUCCESS)
    {
        /* check_battery_level() failed */
        return WICED_ERROR
    }
#endif /* CHECK_BATTERY_LEVEL_BEFORE_OTA2_UPGRADE */

    return WICED_SUCCESS;
}


