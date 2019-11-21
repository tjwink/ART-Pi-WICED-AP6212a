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
* WICED Secondary bootloader for Iwa Platform
*
* This file provides function required to support dynamic loading of applications.
*
* This is the first application loaded by the ROM bootup code.
* Then this application loads Curr App/ Failsafe / Extract / FR App
* as per the program's logic.
*
*/

#include "wiced.h"
#include "wiced_hal_puart.h"
#include "20719mapb0.h"
#include "platform_dct.h"
#include "waf_platform.h"
#include "wiced_ota2_image.h"

#ifndef WICED_DISABLE_STDIO
/* If you use mini_printf() in the bootloader. the console probably won't work with your Application */
#define BOOTLOADER_PRINTF(arg) {printf arg;}
#else
#define BOOTLOADER_PRINTF(arg)
#endif

/******************************************************
 *                      Macros
 ******************************************************/
/* factory reset */
#define FACTORY_RESET_BUTTON_MILLISECONDS       PLATFORM_FACTORY_RESET_TIMEOUT

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum
{
    START_SEQUENCE_RUN_APP          = 0,
    START_SEQUENCE_FACTORY_RESET,
    START_SEQUENCE_OTA_UPDATE,
    START_SEQUENCE_SOFT_AP,

} bootloader_start_sequence_t;


typedef enum
{
    START_RESULT_OK = 0,
    START_RESULT_ERROR_BATTERY_LOW,
    START_RESULT_ERROR_DCT_COPY_FAIL,
    START_RESULT_ERROR_UPGRADE_FAIL,
    START_RESULT_ERROR_FACTORY_RESET_FAIL,

} bootloader_start_result_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t load_program( const load_details_t * load_details, uint32_t* new_entry_point );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
extern wiced_result_t platform_load_app (uint32_t source, uint32_t* startaddr);

void application_start ( void )
{
    bootloader_start_sequence_t start_sequence;
    load_details_t*       load_details;
    uint32_t                    entry_point;
    boot_detail_t               boot;
    wiced_result_t              result;

    wiced_ota2_image_status_t   staging_image_status;
    platform_dct_ota2_config_t  dct_ota2_config;
    uint32_t                    button_pressed_milliseconds;

#ifndef WICED_DISABLE_STDIO
    platform_init_external_devices();
#endif

    /* watch dog reset not supported */

    BOOTLOADER_PRINTF (("Bootloader Starts ... \r\n"));

    /* check for factory reset button being pressed */
    button_pressed_milliseconds = wiced_waf_get_factory_reset_button_time();

   /* get ota2 config status */
   if (wiced_dct_read_with_copy( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) ) != WICED_SUCCESS)
   {
        BOOTLOADER_PRINTF (("Bootloader: OTA2 config DCT read failed !!!\r\n"));
   }

   /* determine if button held for ~10 seconds, run factory reset */
    if ( (button_pressed_milliseconds >= FACTORY_RESET_BUTTON_MILLISECONDS) ||
         (dct_ota2_config.force_factory_reset != 0) ||
         /* Note: OTA2_BOOT_FAILSAFE_FACTORY_RESET is set in bt-ota service before starting any download */
         (dct_ota2_config.boot_type == OTA2_BOOT_FAILSAFE_FACTORY_RESET))
    {
        uint32_t entry_point;
        if ( platform_load_app(OTA2_IMAGE_FR_APP_AREA_BASE, &entry_point) == WICED_SUCCESS )
        {
            wiced_waf_start_app( entry_point );
            return;
        }
    }

    /* Check boot_type FAILSAFE - if so, use ota2_failsafe to recover */
    if (dct_ota2_config.boot_type == OTA2_BOOT_FAILSAFE_UPDATE)
    {
        uint32_t entry_point;
        BOOTLOADER_PRINTF (("Bootloader: Loading Failsafe\n"));
        if ( platform_load_app(OTA2_IMAGE_FAILSAFE_APP_AREA_BASE, &entry_point) == WICED_SUCCESS )
        {
            wiced_waf_start_app( entry_point );
            return;
        }
    }

    BOOTLOADER_PRINTF (("Bootloader: boot type : %d\r\n",dct_ota2_config.boot_type));

    /* Assume a normal boot */
    start_sequence = START_SEQUENCE_RUN_APP;

    /* look at the boot type to determine what to do
     *  We checked for a failsafe situation above.
     */
    switch (dct_ota2_config.boot_type)
    {
        case OTA2_BOOT_EXTRACT_UPDATE:
            start_sequence = START_SEQUENCE_OTA_UPDATE;
            break;

        default:
            /* Check if OTA2 Staged Area Image is marked as extract on boot */
            result = wiced_ota2_image_get_status ( WICED_OTA2_IMAGE_TYPE_STAGED, &staging_image_status );
            if ((result == WICED_SUCCESS) && (staging_image_status == WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT))
            {
                start_sequence = START_SEQUENCE_OTA_UPDATE;
                dct_ota2_config.boot_type = OTA2_BOOT_EXTRACT_UPDATE;
            }
            break;
    }

    /* save - boot_type possibly changed */
    if (wiced_dct_write( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) ) != WICED_SUCCESS)
    {
        BOOTLOADER_PRINTF (("OTA2 Config DCT Write failed, continue on. We will reset if there is a problem and try again.\r\n"));
    }

    BOOTLOADER_PRINTF (("Bootloader: start_sequence = %d\n",start_sequence));

    if ( start_sequence == START_SEQUENCE_OTA_UPDATE )
    {
            /* Set the OTA2 extractor in the boot_detail */
            wiced_waf_app_set_boot( DCT_OTA_APP_INDEX, PLATFORM_DEFAULT_LOAD );
    }

    /* boot the device with the app set in the boot_detail */
    boot.load_details.valid = 1;
    boot.entry_point        = 0;
    wiced_dct_read_with_copy( &boot, DCT_INTERNAL_SECTION, OFFSET( platform_dct_header_t, boot_detail ), sizeof(boot_detail_t) );

    load_details = &boot.load_details;
    entry_point  = boot.entry_point;

    BOOTLOADER_PRINTF (("Bootloader: boot entrypoint [0x%x], valid=%d, id=%d, location=0x%x\n",
            (unsigned int)boot.entry_point,
            boot.load_details.valid,
            boot.load_details.source.id,
            (unsigned int)boot.load_details.source.detail.external_fixed.location));

    if ( load_details->valid != 0 )
    {
        if (load_program( load_details, &entry_point ) == WICED_SUCCESS)
        {
            BOOTLOADER_PRINTF (("wiced_waf_start_app() 0x%x\r\n", entry_point  ));
        }
    }

    wiced_waf_start_app( entry_point );

}

static wiced_result_t load_program( const load_details_t * load_details, uint32_t* new_entry_point )
{
    wiced_result_t result = WICED_ERROR;

    if ( load_details->source.id == EXTERNAL_FIXED_LOCATION )
    {
        /* Fixed location in serial flash source - i.e. no filesystem */
        result = wiced_waf_app_load( &load_details->source, new_entry_point );
    }
    else if ( load_details->source.id == EXTERNAL_FILESYSTEM_FILE )
    {
        /* Not supported */
        result = WICED_ERROR;
    }

    if ( load_details->load_once != 0 )
    {
        boot_detail_t boot;

        boot.entry_point                 = 0;
        boot.load_details.load_once      = 1;
        boot.load_details.valid          = 0;
        boot.load_details.destination.id = INTERNAL;
        wiced_dct_write_boot_details( &boot );
    }

    return result;
}
