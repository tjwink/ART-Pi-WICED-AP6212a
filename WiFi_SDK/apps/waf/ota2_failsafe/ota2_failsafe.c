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
 * This application is used by the OTA2 Update Capability as a fallback
 * minimum extractor to fix a failed OTA2 extraction.
 *
 * Use case:
 *  OTA2 Extraction is in process
 *  Power outage or board reset happens and leaves the FLASH in an potentially unusable state
 *  - This is a potentially bad situation
 *     If the Apps LUT, Filesystem, OTA2 Extractor or Application Extraction was interrupted,
 *       the system may not work.
 *  - This extractor will be run by the bootloader to extract the ota2_extract application
 *     from either the Staged Area (if valid) or the Factory Reset and instructs the
 *     ota2_extract application to extract the Staged Area or Factory Reset Area (as appropriate)
 *
 * This application is meant to be programmed into the flash at Manufacture and not be updated.
 * As such, the ota2_extract application can never be compressed in the OTA2 image files, as this
 *   application does not know about decompression.
 *
 *   We can't count on anything of these being valid.
 *
 *   Apps LUT
 *   Resource Filesystem
 *   OTA2 extractor
 *   Application(s)
 *
 */

#include "wiced.h"
#include "wwd_debug.h"
#include "waf_platform.h"
#include "platform_init.h"
#include "wiced_framework.h"
#include "wiced_ota2_image.h"
#include "elf.h"
#include "spi_flash.h"

#if 1   /* Set this to 0 for no printing in this file */
#include "mini_printf.h"
#define OTA2_FAILSAFE_PRINTF(arg) {mini_printf arg;}
#else
#define OTA2_FAILSAFE_PRINTF(arg)
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

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    wiced_result_t              result;
    platform_dct_ota2_config_t  dct_ota2_config;
    uint32_t                    ota_extractor_flash_destination;
    uint32_t                    ota2_extractor_destination_size;
    uint32_t                    apps_lut_flash_destination;
    uint32_t                    apps_lut_destination_size;

    OTA2_FAILSAFE_PRINTF(("\r\nOTA2_Failsafe: Started ...\r\n"));

    /*
     * We don't use wiced_init(), as we don't want wiced_wlan_connectivity_init()
     * We don't use wiced_core_init(), as we don't want to use wiced_platform_init()
     * We don't use wiced_platform_init(), as the filesystem may be invalid, and we don't want to assert
     */
    platform_init_external_devices();

    wiced_dct_read_with_copy( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) );

    if (dct_ota2_config.boot_type == OTA2_BOOT_FAILSAFE_UPDATE)
    {
        if (wiced_ota2_image_validate(WICED_OTA2_IMAGE_TYPE_STAGED) == WICED_SUCCESS)
        {
            OTA2_FAILSAFE_PRINTF(("\r\nOTA2_Failsafe: Extract Staged\r\n"));
            result = wiced_ota2_image_extract_uncompressed_component( WICED_OTA2_IMAGE_TYPE_STAGED, WICED_OTA2_IMAGE_COMPONENT_OTA_APP, &ota_extractor_flash_destination, &ota2_extractor_destination_size);
            if (result == WICED_SUCCESS)
            {
                /* extract the APPs LUT so that the bootloader can load in the OTA2 Extractor on a re-boot */
                result = wiced_ota2_image_extract_uncompressed_component( WICED_OTA2_IMAGE_TYPE_STAGED, WICED_OTA2_IMAGE_COMPONENT_LUT, &apps_lut_flash_destination, &apps_lut_destination_size);
            }
            if (result == WICED_SUCCESS)
            {
                /* set the Staged Area to be "Extract on Boot" */
                wiced_ota2_image_update_staged_status(WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT);
                dct_ota2_config.boot_type = OTA2_BOOT_EXTRACT_UPDATE; /* Instruct bootloader to execute a Staged Area Update */
                wiced_dct_write( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) );
                wiced_framework_reboot( );
            }
        }
    }

    /* if we extracted the OTA2 Staged Area Image, we rebooted before we get here */

    /* Factory Reset Failsafe */
    OTA2_FAILSAFE_PRINTF(("\r\nOTA2_Failsafe: Extract Factory Reset\r\n"));
    result = wiced_ota2_image_extract_uncompressed_component( WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP, WICED_OTA2_IMAGE_COMPONENT_OTA_APP, &ota_extractor_flash_destination, &ota2_extractor_destination_size);
    if (result == WICED_SUCCESS)
    {
        /* extract the APPs LUT so that the bootloader can load in the OTA2 Extractor on a re-boot */
        result = wiced_ota2_image_extract_uncompressed_component( WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP, WICED_OTA2_IMAGE_COMPONENT_LUT, &apps_lut_flash_destination, &apps_lut_destination_size);
    }

    if (result == WICED_SUCCESS)
    {
        dct_ota2_config.boot_type = OTA2_BOOT_EXTRACT_FACTORY_RESET;    /* Instruct bootloader to execute a Factory Reset Update */
        wiced_dct_write( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) );
        wiced_framework_reboot( );
    }

    OTA2_FAILSAFE_PRINTF(("OTA2_Failsafe Extraction Failed !!!\r\n"));
    while ( 1 )
    {
        wiced_rtos_delay_milliseconds( 100 );
    }
}
