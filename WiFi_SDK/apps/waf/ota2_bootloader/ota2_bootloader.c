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

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "wicedfs.h"
#include "platform_dct.h"
#include "elf.h"
#include "wiced_framework.h"
#include "wiced_utilities.h"
#include "platform_config.h"
#include "platform_resource.h"
#include "waf_platform.h"
#include "wwd_rtos.h"
#include "wiced_rtos.h"

#include "spi_flash.h"
#include "platform.h"
#include "platform_init.h"
#include "platform_stdio.h"
#include "platform_peripheral.h"
#include "platform_dct.h"

#include "wiced_ota2_image.h"

#ifndef WICED_DISABLE_STDIO
/* If you use mini_printf() in the bootloader. the console probably won't work with your Application */
#include "mini_printf.h"
#define BOOTLOADER_PRINTF(arg) {mini_printf arg;}
#else
#define BOOTLOADER_PRINTF(arg)
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#ifndef OTA2_BOOTLOADER_NO_SOFTAP_SUPPORT
#define SOFTAP_START_BUTTON_MILLISECONDS_MIN     4000           // allow 4 - 6 seconds
#define SOFTAP_START_BUTTON_MILLISECONDS_MAX     6000
#define FACTORY_RESET_BUTTON_MILLISECONDS       PLATFORM_FACTORY_RESET_TIMEOUT
#else
/* no softap on startup here */
#define FACTORY_RESET_BUTTON_MILLISECONDS
#define FACTORY_RESET_BUTTON_MILLISECONDS       PLATFORM_FACTORY_RESET_TIMEOUT
#endif

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

static wiced_result_t ota2_failsafe_app_load( uint32_t flash_location, uint32_t* entry_point );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

int main( void )
{
    bootloader_start_sequence_t start_sequence;
    const load_details_t*       load_details;
    uint32_t                    entry_point;
    boot_detail_t               boot;
    wiced_result_t              result;
    bootloader_start_result_t   boot_result;

    wiced_ota2_image_status_t   staging_image_status;

    uint32_t                    button_pressed_milliseconds;
    platform_dct_ota2_config_t  dct_ota2_config;

    boot_result = START_RESULT_OK;

    /*
     * We don't use wiced_init(), as we don't want wiced_wlan_connectivity_init()
     * We don't use wiced_core_init(), as we don't want to use wiced_platform_init()
     * We don't use wiced_platform_init(), as the filesystem may be invalid, and we don't want to assert
     */
#ifndef WICED_DISABLE_STDIO
    platform_init_external_devices();
#endif

    BOOTLOADER_PRINTF(("OTA2 Bootloader!\r\n"));

    /* set up timing for no Operating System, we haven't started an OS yet */
    NoOS_setup_timing( );

    /* check for factory reset button being pressed */
    button_pressed_milliseconds = wiced_waf_get_factory_reset_button_time();

    NoOS_stop_timing( );

    /* get ota2 status */
    wiced_dct_read_with_copy( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) );

    /*
     * If the SoftAP just ran, dct_ota2_config.boot_type is set to
     *      OTA2_BOOT_EXTRACT_FACTORY_RESET  or  OTA2_BOOT_EXTRACT_UPDATE
     * If we get here with these boot types, extract the OTA2 image
     *
     * When extraction is started, dct_ota2_config.boot_type is set to
     *      OTA2_BOOT_FAILSAFE_FACTORY_RESET  or  OTA2_BOOT_FAILSAFE_UPDATE
     * If we get here with these boot types, extraction failed, use ota2_failsafe to recover.
     *
     * When the extraction is finished, the boot_type is set to
     *      OTA2_BOOT_FACTORY_RESET          or  OTA2_BOOT_UPDATE
     * If we get here with these boot types, run the Application
     *
     */

    /* Check for Watchdog reset
     * This overrides everything. Use ota2_failsafe to recover
     */
    if (platform_watchdog_check_last_reset() == WICED_TRUE)
    {
        /* watchdog reset happened - which image do we try to use? */
        wiced_ota2_image_status_t   staged_update_status;

        dct_ota2_config.boot_type = OTA2_BOOT_FAILSAFE_FACTORY_RESET;
        /* if the downloaded image is good, use that over the factory reset */
        result = wiced_ota2_image_get_status( WICED_OTA2_IMAGE_TYPE_STAGED, &staged_update_status );
        if ((result == WICED_SUCCESS) &&
            ((staged_update_status == WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE) ||
             (staged_update_status == WICED_OTA2_IMAGE_VALID) ||
             (staged_update_status == WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT) ||
             (staged_update_status == WICED_OTA2_IMAGE_DOWNLOAD_EXTRACTED)) )
        {
            dct_ota2_config.boot_type = OTA2_BOOT_FAILSAFE_UPDATE;
        }

        if (wiced_dct_write( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) ) != WICED_SUCCESS)
        {
            BOOTLOADER_PRINTF(("OTA2 Config DCT Write failed, continue on. We will reset if there is a problem and try again.\r\n"));
        }
    }

    /* Check boot_type FAILSAFE - if so, use ota2_failsafe to recover */
    BOOTLOADER_PRINTF(("Bootloader: boot_type = %d!\r\n", dct_ota2_config.boot_type));
    if ((dct_ota2_config.boot_type == OTA2_BOOT_FAILSAFE_FACTORY_RESET) ||
        (dct_ota2_config.boot_type == OTA2_BOOT_FAILSAFE_UPDATE))
    {
        uint32_t entry_point;
        if (ota2_failsafe_app_load( OTA2_IMAGE_FAILSAFE_APP_AREA_BASE, &entry_point ) == WICED_SUCCESS)
        {
            wiced_waf_start_app( entry_point );
        }
    }

    /* Assume a normal boot */
    start_sequence = START_SEQUENCE_RUN_APP;

    /* look at the boot type to determine what to do
     *  We checked for a failsafe situation above.
     */
    switch (dct_ota2_config.boot_type)
    {
        case OTA2_BOOT_EXTRACT_FACTORY_RESET:
            start_sequence = START_SEQUENCE_FACTORY_RESET;
            break;
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

    /* determine if button held for ~5 seconds, start SoftAP for manual OTA upload */
    if ((button_pressed_milliseconds >= SOFTAP_START_BUTTON_MILLISECONDS_MIN) &&
        (button_pressed_milliseconds <= SOFTAP_START_BUTTON_MILLISECONDS_MAX))
    {
        /* Start SoftAP, DHCP & minimal WebServer */
        start_sequence = START_SEQUENCE_SOFT_AP;
        dct_ota2_config.boot_type = OTA2_BOOT_SOFTAP_UPDATE;
    }

    /* determine if button held for ~10 seconds, run factory reset */
    if ( (button_pressed_milliseconds >= FACTORY_RESET_BUTTON_MILLISECONDS) ||
         (dct_ota2_config.force_factory_reset != 0) )
    {
        /* OTA2 User pushed button factory reset here !!! */
        wiced_ota2_image_status_t   factory_reset_app_status;
        start_sequence = START_SEQUENCE_FACTORY_RESET;
        result = wiced_ota2_image_get_status( WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP, &factory_reset_app_status );
        if ((result != WICED_SUCCESS) && (factory_reset_app_status != WICED_OTA2_IMAGE_VALID))
        {
            BOOTLOADER_PRINTF(("Factory Reset Image invalid! Just try to run the Application.\r\n"));
            start_sequence = START_SEQUENCE_RUN_APP;
        }
        else
        {
            BOOTLOADER_PRINTF(("Force Factory Reset.\r\n"));
            dct_ota2_config.force_factory_reset = 1;
            dct_ota2_config.boot_type = OTA2_BOOT_EXTRACT_FACTORY_RESET;
        }
    }

    /* save - boot_type possibly changed */
    if (wiced_dct_write( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) ) != WICED_SUCCESS)
    {
        BOOTLOADER_PRINTF(("OTA2 Config DCT Write failed, continue on. We will reset if there is a problem and try again.\r\n"));
    }

    switch (start_sequence)
    {
        case START_SEQUENCE_FACTORY_RESET:
        case START_SEQUENCE_OTA_UPDATE:
        case START_SEQUENCE_SOFT_AP:
            /* Set the OTA2 extractor in the boot_detail */
            wiced_waf_app_set_boot( DCT_OTA_APP_INDEX, PLATFORM_DEFAULT_LOAD );
            break;
        default:
            break;

    }
    BOOTLOADER_PRINTF(("OTA2 Bootloader -- start_sequence : %d ", start_sequence));

    /* boot the device with the app set in the boot_detail */
    boot.load_details.valid = 1;
    boot.entry_point        = 0;
    wiced_dct_read_with_copy( &boot, DCT_INTERNAL_SECTION, OFFSET( platform_dct_header_t, boot_detail ), sizeof(boot_detail_t) );

    load_details = &boot.load_details;
    entry_point  = boot.entry_point;

    if ( load_details->valid != 0 )
    {
        if (load_program( load_details, &entry_point ) == WICED_SUCCESS)
        {
            BOOTLOADER_PRINTF(("wiced_waf_start_app() 0x%lx\r\n", entry_point  ));
        }
    }

    wiced_waf_start_app( entry_point );

    while(1)
    {
        (void)boot_result;
    }

    /* Should never get here */
    return 0;
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
        /* Filesystem location in serial flash source */
        result = wiced_waf_app_load( &load_details->source, new_entry_point );
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

/********************************************************************************************
 *          Failsafe Support
 ********************************************************************************************/

static wiced_bool_t ota2_failsafe_is_elf_segment_load( elf_program_header_t* prog_header )
{
    if ( ( prog_header->data_size_in_file == 0 ) || /* size is zero */
         ( ( prog_header->type & 0x1 ) == 0 ) ) /* non- loadable segment */
    {
        return WICED_FALSE;
    }

    return WICED_TRUE;
}

static wiced_result_t ota2_failsafe_app_read( uint32_t location, uint8_t* data, uint32_t offset, uint32_t size )
{
    wiced_result_t result = WICED_ERROR;
    sflash_handle_t sflash_handle;
    sflash_read_t  sflash_read_func;

    if ( PLATFORM_SECURESFLASH_ENABLED )
    {
        sflash_read_func = sflash_read_secure;
        offset = SECURE_SECTOR_ADDRESS( offset ) + OFFSET_WITHIN_SECURE_SECTOR( offset );
    }
    else
    {
        sflash_read_func = sflash_read;
    }

    if (init_sflash( &sflash_handle, PLATFORM_SFLASH_PERIPHERAL_ID, SFLASH_WRITE_NOT_ALLOWED ) == 0)
    {
        if (sflash_read_func( &sflash_handle, ( location + offset ), data, size ) == 0)
        {
            result = WICED_SUCCESS;
        }
        deinit_sflash( &sflash_handle );
    }
    return result;
}

static wiced_result_t ota2_failsafe_app_area_erase(uint32_t flash_location, elf_header_t* header)
{

    uint32_t     i;
    uint32_t     start_address = 0xFFFFFFFF;
    uint32_t     end_address   = 0x00000000;

    for ( i = 0; i < header->program_header_entry_count; i++ )
    {
        elf_program_header_t prog_header;
        uint32_t offset;

        offset = header->program_header_offset + (header->program_header_entry_size * i);
        ota2_failsafe_app_read( flash_location, (uint8_t*) &prog_header, offset, sizeof( prog_header ) );

        if ( ota2_failsafe_is_elf_segment_load( &prog_header ) == WICED_FALSE )
        {
            continue;
        }

        if ( prog_header.physical_address < start_address )
        {
            start_address = prog_header.physical_address;
        }

        if ( prog_header.physical_address + prog_header.data_size_in_file > end_address )
        {
            end_address = prog_header.physical_address + prog_header.data_size_in_file;
        }
    }
    platform_erase_app_area( start_address, end_address - start_address );
    return WICED_SUCCESS;
}

static wiced_result_t ota2_failsafe_app_load( uint32_t flash_location, uint32_t* entry_point )
{
    wiced_result_t       result = WICED_SUCCESS;
    uint32_t             i;
    elf_header_t         header;
    elf_program_header_t prog_header;

    /* assume pointer is good (only called from within this file) and assume error */
    *entry_point = 0;

    /* Read the image header */
    result = ota2_failsafe_app_read( flash_location, (uint8_t*) &header, 0, sizeof( header ) );
    if (result != WICED_SUCCESS)
    {
        return result;
    }

    /* Quick ELF header validity check */
    if (header.ident.magic != ELF_MAGIC_NUMBER)
    {
        return result;
    }

    /* Erase the application area */
    result = ota2_failsafe_app_area_erase( flash_location, &header );
    if (result != WICED_SUCCESS)
    {
        return result;
    }

    for ( i = 0; i < header.program_header_entry_count; i++ )
    {
        wiced_bool_t is_elf;
        unsigned long offset = header.program_header_offset + header.program_header_entry_size * (unsigned long) i;

        result = ota2_failsafe_app_read( flash_location, (uint8_t*) &prog_header, offset, sizeof( prog_header ) );
        is_elf = ota2_failsafe_is_elf_segment_load( &prog_header );
        if ( is_elf == WICED_FALSE )
        {
            continue;
        }
        result = ota2_failsafe_app_read( flash_location, (uint8_t*)prog_header.physical_address, prog_header.data_offset, prog_header.data_size_in_file );
        if (result != WICED_SUCCESS)
        {
            break;
        }
    }

    if (result == WICED_SUCCESS)
    {
        /* return the program entry point */
        *entry_point = header.entry;
    }

    return result;
}
