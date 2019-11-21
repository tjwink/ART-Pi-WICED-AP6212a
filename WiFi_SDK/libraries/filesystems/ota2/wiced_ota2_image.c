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
 *  OTA Image status check and extraction routines
 *
 *  OTA Image File consists of:
 *
 *  wiced_ota2_image_header_t            - info about the OTA Image
 *  wiced_ota2_image_component_t         - array of individual component info structs
 *  ota_data                            - all the data
 *
 */

#include <stdlib.h>
#include <string.h>

#include "wwd_assert.h"
#include "wiced_result.h"
#include "wiced_utilities.h"
#include "wiced_time.h"
#include "platform_dct.h"
#include "wiced_rtos.h"
#include "wiced_apps_common.h"
#include "wiced_waf_common.h"
#include "wiced_dct_common.h"
#include "waf_platform.h"
#include "platform.h"
#include "wiced_platform.h"
#include "platform_dct.h"
#include "platform_peripheral.h"
#include "spi_flash.h"

#include "wiced_ota2_image.h"
#include "wiced_dct_common.h"
#ifdef WICED_PLATFORM_INCLUDES_OCF_FS
#include "platform_ocf.h"
#endif
/* define to verify writes  */
//#define VERIFY_SFLASH_WRITES    1

// if we are running bootloader, do not print

#if defined(BOOTLOADER)
#define OTA2_WPRINT_MSG(a,b,format,...)
#define OTA2_WPRINTF(format,...)
#else

#include "wiced_log.h"
#include "mini_printf.h"    /* to show progress */

#define OTA2_WPRINT_MSG     wiced_log_msg
#define OTA2_WPRINTF        wiced_log_printf
#endif

/* only time how long it takes when in an app - bootloader doesn't have the time stuff */
#if !defined(BOOTLOADER)
#define  DEBUG_GET_DECOMP_TIME      1
#endif

uint32_t wiced_ota2_image_get_offset( wiced_ota2_image_type_t ota_type );
static wiced_result_t wiced_ota2_read_ota2_header(wiced_ota2_image_type_t ota_type, wiced_ota2_image_header_t* ota2_header);

#ifdef WICED_PLATFORM_INCLUDES_OCF_FS
extern wiced_result_t wiced_hal_eflash_erase( uint32_t offset, uint32_t length );
#endif

/******************************************************
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

char *ota2_image_type_name_string[] =
{
    "OTA2_IMAGE_TYPE_NONE",
    "OTA2_IMAGE_TYPE_FACTORY",
    "OTA2_IMAGE_TYPE_CURRENT_APP",
    "OTA2_IMAGE_TYPE_LAST_KNOWN_GOOD",
    "OTA2_IMAGE_TYPE_STAGED"
};

#if  !defined(BOOTLOADER)

#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
static wiced_ota2_image_type_t  g_curr_image_type = WICED_OTA2_IMAGE_TYPE_STAGED;
#endif

/* used by wiced_ota2_safe_write_data_to_sflash() so we don't have to allocate a buffer every time! */
uint8_t sector_in_ram[SECTOR_SIZE];
uint8_t component_decomp_ram[SECTOR_SIZE];

/******************************************************
 *               Function Declarations
 ******************************************************/
static void wiced_ota2_print_header_info(wiced_ota2_image_header_t* ota2_header, const char* mssg);

#ifdef WICED_PLATFORM_INCLUDES_OCF_FS
static wiced_result_t wiced_ota2_read_ocf(uint32_t offset, uint8_t* data, uint32_t size);
#else
static wiced_result_t wiced_ota2_read_sflash(uint32_t offset, uint8_t* data, uint32_t size);
#endif

/******************************************************
 *               Internal Functions
 ******************************************************/

static void wiced_ota2_print_component_info(wiced_ota2_image_component_t *component)
{
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "          name: %s\r\n", component->name);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "          type: %d\r\n", component->type);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "          comp: %d\r\n", component->compression);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "           crc: 0x%08lx\r\n", component->crc);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "      src_size: %8ld 0x%lx\r\n", component->source_size, component->source_size);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "       src_off: %8ld 0x%lx\r\n", component->source_offset, component->source_offset);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "      dst_size: %8ld 0x%lx\r\n", component->destination_size, component->destination_size);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "          dest: %8ld 0x%lx\r\n", component->destination, component->destination);
}

static void wiced_ota2_print_header_info(wiced_ota2_image_header_t* ota2_header, const char* mssg)
{
    if( ota2_header == NULL )
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_INFO, "wiced_ota2_print_header_info() ota2_header == NULL\r\n" );
        return;
    }

    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "OTA header Information: %s\r\n", mssg);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "       ota_version: %d\r\n", ota2_header->ota2_version);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "    software_major: %d\r\n", ota2_header->major_version);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "    software_minor: %d\r\n", ota2_header->minor_version);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "          platform: %s\r\n", ota2_header->platform_name);

    switch( (wiced_ota2_image_status_t)ota2_header->download_status )
    {
    default:
    case WICED_OTA2_IMAGE_INVALID:
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "            status: %d (invalid)\r\n", ota2_header->download_status);
        break;
    case WICED_OTA2_IMAGE_VALID:
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "            status: %d (valid)\r\n", ota2_header->download_status);
        break;
    case WICED_OTA2_IMAGE_DOWNLOAD_IN_PROGRESS:
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "            status: %d (download in progress)\r\n", ota2_header->download_status);
        break;
    case WICED_OTA2_IMAGE_DOWNLOAD_FAILED:
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "            status: %d (download FAILED)\r\n", ota2_header->download_status);
        break;
    case WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE:
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "            status: %d (download complete)\r\n", ota2_header->download_status);
        break;
    case WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT:
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "            status: %d (extract on boot)\r\n", ota2_header->download_status);
        break;
    case WICED_OTA2_IMAGE_DOWNLOAD_EXTRACTED:
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "            status: %d (extracted!)\r\n", ota2_header->download_status);
        break;
    }
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "    bytes_received: %ld\r\n", ota2_header->bytes_received);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "        header_crc: 0x%lx\r\n", ota2_header->header_crc);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "      magic_string: %c%c%c%c%c%c%c%c\r\n", ota2_header->magic_string[0], ota2_header->magic_string[1],
                                                 ota2_header->magic_string[2], ota2_header->magic_string[3],
                                                 ota2_header->magic_string[4], ota2_header->magic_string[5],
                                                 ota2_header->magic_string[6], ota2_header->magic_string[7] );
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "         sign_type: %d\r\n", ota2_header->secure_sign_type);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "  secure_signature: TODO \r\n"); /* TODO */

    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "        image_size: %ld\r\n", ota2_header->image_size);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "   component_count: %d\r\n", ota2_header->component_count);
    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, " data start offset: %ld\r\n", ota2_header->data_start);
}

/**
 * Write any size data to FLASH
 * Handle erasing then writing the data
 * Erasing / writing is done on sector boundaries, work around this limitation
 *
 * @param[in]  offset   - offset to the sector
 * @param[in]  data     - the data
 * @param[in]  size     - size of the data
 *
 * @return - WICED_SUCCESS
 *           WICED_BADARG
 *           WICED_OUT_OF_HEAP_SPACE
 */
#ifdef WICED_PLATFORM_INCLUDES_OCF_FS
static wiced_result_t wiced_ota2_safe_write_data_to_ocf( uint32_t offset, uint8_t *data, uint32_t size )
{
    uint32_t        current_offset, sector_base_offset, in_sector_offset;
    uint32_t        bytes_to_write;
    uint32_t        chunk_size;
    uint8_t*        in_ptr;
    wiced_result_t  result = WICED_SUCCESS;

    /* read a whole sector into RAM
     * modify the parts of the data in the RAM copy
     * erase the whole sector
     * write the new data to the sector
     */

    current_offset = offset;
    bytes_to_write = size;
    in_ptr = data;

    platform_watchdog_kick( );

    while ((result == WICED_SUCCESS) && (bytes_to_write > 0))
    {
        /* figure out the base of the sector we want to modify
         * NOTE: This is an offset into the FLASH
         */
        sector_base_offset = current_offset - (current_offset % SECTOR_SIZE);

        /* read the whole sector into our RAM buffer before we modify it */
        if ( ocf_buffer_read (sector_base_offset, sector_in_ram, SECTOR_SIZE) != 0 )
        {
            /* read sector fail */
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_safe_write_data_to_ocf() read sector error!\r\n");
            result = WICED_ERROR;
            break;
        }

        /* how far into the sector will we start copying ? */
        in_sector_offset = current_offset - sector_base_offset;

        /* How many bytes in this sector after in_sector_offset will we copy?
         * We are going to copy from in_sector_offset up to the end of the sector
         */
        chunk_size = SECTOR_SIZE - in_sector_offset;
        if (chunk_size > bytes_to_write)
        {
            chunk_size = bytes_to_write;
        }

        /* copy new data into in-RAM buffer */
        memcpy(&sector_in_ram[in_sector_offset], in_ptr, chunk_size);

        if (wiced_hal_eflash_erase (sector_base_offset,SECTOR_SIZE) != 0)
        {
            /* erase sector fail */
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_safe_write_data_to_ocf() erase sector error!\r\n");
            result = WICED_ERROR;
            break;
        }

        if ( 0 != ocf_buffer_write( sector_base_offset, sector_in_ram, SECTOR_SIZE ) )
        {
            /* Verify Error - Chip not erased properly */
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_safe_write_data_to_ocf() WRITE ERROR @ %d!\r\n", __LINE__);
            result = WICED_ERROR;
        }

        bytes_to_write -= chunk_size;
        in_ptr         += chunk_size;
        current_offset += chunk_size;
    }//end while loop

    return result;
}
#else
static wiced_result_t wiced_ota2_safe_write_data_to_sflash( uint32_t offset, uint8_t *data, uint32_t size )
{
    uint32_t        current_offset, sector_base_offset, in_sector_offset;
    uint32_t        bytes_to_write;
    uint32_t        chunk_size;
    uint8_t*        in_ptr;
    sflash_handle_t sflash_handle;
    wiced_result_t  result = WICED_SUCCESS;

    if (init_sflash( &sflash_handle, PLATFORM_SFLASH_PERIPHERAL_ID, SFLASH_WRITE_ALLOWED )!= 0)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_safe_write_data_to_sflash() Failed to init SFLASH for writing.\r\n");
        return WICED_ERROR;
    }

    /* read a whole sector into RAM
     * modify the parts of the data in the RAM copy
     * erase the whole sector
     * write the new data to the sector
     */

    current_offset = offset;
    bytes_to_write = size;
    in_ptr = data;

    platform_watchdog_kick( );

    while ((result == WICED_SUCCESS) && (bytes_to_write > 0))
    {
        /* figure out the base of the sector we want to modify
         * NOTE: This is an offset into the FLASH
         */
        sector_base_offset = current_offset - (current_offset % SECTOR_SIZE);

        /* read the whole sector into our RAM buffer before we modify it */
        if (sflash_read( &sflash_handle, (unsigned long)sector_base_offset, sector_in_ram, SECTOR_SIZE) != 0 )
        {
            /* read sector fail */
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_safe_write_data_to_sflash() read sector error!\r\n");
            result = WICED_ERROR;
            break;
        }

        /* how far into the sector will we start copying ? */
        in_sector_offset = current_offset - sector_base_offset;

        /* How many bytes in this sector after in_sector_offset will we copy?
         * We are going to copy from in_sector_offset up to the end of the sector
         */
        chunk_size = SECTOR_SIZE - in_sector_offset;
        if (chunk_size > bytes_to_write)
        {
            chunk_size = bytes_to_write;
        }

        /* copy new data into in-RAM buffer */
        memcpy(&sector_in_ram[in_sector_offset], in_ptr, chunk_size);

        if (sflash_sector_erase( &sflash_handle, (unsigned long)sector_base_offset) != 0)
        {
            /* erase sector fail */
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_safe_write_data_to_sflash() erase sector error!\r\n");
            result = WICED_ERROR;
            break;
        }

        if ( 0 != sflash_write( &sflash_handle, (unsigned long)sector_base_offset, sector_in_ram, SECTOR_SIZE ) )
        {
            /* Verify Error - Chip not erased properly */
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_safe_write_data_to_sflash() WRITE ERROR @ %d!\r\n", __LINE__);
            result = WICED_ERROR;
        }

#ifdef VERIFY_SFLASH_WRITES
        if (sflash_read( &sflash_handle, (unsigned long)sector_base_offset, sector_in_ram, SECTOR_SIZE) != 0 )
        {
            /* read sector fail */
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_safe_write_data_to_sflash() read sector error!\r\n");
            result = WICED_ERROR;
            break;
        }

        if ( (memcmp(&sector_in_ram[in_sector_offset], in_ptr, chunk_size) != 0) )
        {
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_safe_write_data_to_sflash() Verify ERROR!\r\n");
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "     base:%lx offset:%lx to_write:%ld chunk:%lx!\r\n", sector_base_offset, current_offset, bytes_to_write, chunk_size);
            result = WICED_ERROR;
        }
#endif

        bytes_to_write -= chunk_size;
        in_ptr         += chunk_size;
        current_offset += chunk_size;
    }

    if (deinit_sflash( &sflash_handle ) != WICED_SUCCESS)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_safe_write_data_to_sflash() Failed to deinit SFLASH.\r\n");
        result = WICED_ERROR;
    }

    return result;
}
#endif //WICED_PLATFORM_INCLUDES_OCF_FS

#ifdef CURRENT_APPLICATION_USES_INTERNAL_FLASH
/**
 * Write any size data to FLASH
 * Handle erasing then writing the data
 * Erasing / writing is done on sector boundaries, work around this limitation
 *
 * @param[in]  offset   - offset to the sector
 * @param[in]  data     - the data
 * @param[in]  size     - size of the data
 *
 * @return - WICED_SUCCESS
 *           WICED_BADARG
 *           WICED_OUT_OF_HEAP_SPACE
 */
static wiced_result_t wiced_ota2_safe_write_data_to_internal_flash( uint32_t offset, uint8_t *data, uint32_t size )
{
    uint32_t        current_offset, sector_base_offset, in_sector_offset;
    uint32_t        bytes_to_write;
    uint32_t        chunk_size;
    uint8_t*        in_ptr;
    wiced_result_t  result = WICED_SUCCESS;

    /* read a whole sector into RAM
     * modify the parts of the data in the RAM copy
     * erase the whole sector
     * write the new data to the sector
     */
    current_offset = offset;
    bytes_to_write = size;
    in_ptr = data;
    while ((result == WICED_SUCCESS) && (bytes_to_write > 0))
    {
        /* figure out the base of the sector we want to modify
         * NOTE: This is an offset into the FLASH
         */
        sector_base_offset = current_offset - (current_offset % SECTOR_SIZE);

        /* how far into the sector will we start copying ? */
        in_sector_offset = current_offset - sector_base_offset;

        /* How many bytes in this sector after in_sector_offset will we copy?
         * We are going to copy from in_sector_offset up to the end of the sector
         */
        chunk_size = SECTOR_SIZE - in_sector_offset;
        if (chunk_size > bytes_to_write)
        {
            chunk_size = bytes_to_write;
        }

        if ( memcpy(&sector_in_ram[in_sector_offset], in_ptr, chunk_size) != &sector_in_ram[in_sector_offset] )
        {
            /* read sector fail */
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "memcpy failed to write to local buffer!\r\n");
            result = WICED_ERROR;
            break;
        }

        if (platform_write_flash_chunk( sector_base_offset, sector_in_ram, SECTOR_SIZE) != PLATFORM_SUCCESS)
        {
            /* Verify Error - Chip not erased properly */
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_safe_write_data_to_internal_flash() WRITE ERROR!\r\n");
            result = WICED_ERROR;
        }

        bytes_to_write -= chunk_size;
        in_ptr         += chunk_size;
        current_offset += chunk_size;
    }

    return result;
}
#endif

/* *****************************************************
 *
 *  swap the data for network order before saving
 */
static wiced_result_t wiced_ota2_write_ota2_header(wiced_ota2_image_type_t ota_type, wiced_ota2_image_header_t* ota2_header)
{
    wiced_ota2_image_header_t   swapped_ota2_header;
    uint32_t                    base_ota_offset;
    wiced_result_t              result = WICED_SUCCESS;

    base_ota_offset = wiced_ota2_image_get_offset(ota_type);
    if ((base_ota_offset == 0) || (ota2_header == NULL))
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "wiced_ota2_write_ota2_header() Bad args!\r\n");
        return WICED_BADARG;
    }

    /* make our copy of the data so we can swap it without affecting the caller's copy */
    memcpy( &swapped_ota2_header, ota2_header, sizeof(wiced_ota2_image_header_t));
    wiced_ota2_image_header_swap_network_order(&swapped_ota2_header, WICED_OTA2_IMAGE_SWAP_HOST_TO_NETWORK);
#ifdef WICED_PLATFORM_INCLUDES_OCF_FS
    if (wiced_ota2_safe_write_data_to_ocf( base_ota_offset, (uint8_t*)&swapped_ota2_header, sizeof(wiced_ota2_image_header_t)) != WICED_SUCCESS)
#else
    if (wiced_ota2_safe_write_data_to_sflash( base_ota_offset, (uint8_t*)&swapped_ota2_header, sizeof(wiced_ota2_image_header_t)) != WICED_SUCCESS)
#endif
    {
        result = WICED_ERROR;
    }

    return result;
}

/* *****************************************************
 *
 */
static wiced_result_t wiced_ota_read_component_header( wiced_ota2_image_type_t ota_type, uint16_t component_index, wiced_ota2_image_component_t* component_header)
{
    uint32_t    base_ota_offset;
    uint32_t    component_header_offset;

    base_ota_offset = wiced_ota2_image_get_offset(ota_type);
    if ((base_ota_offset == 0) || (component_header == NULL))
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota_read_component_header() Bad args!\r\n");
        return WICED_BADARG;
    }

    component_header_offset = base_ota_offset + sizeof(wiced_ota2_image_header_t) + (component_index * sizeof(wiced_ota2_image_component_t));
#ifdef WICED_PLATFORM_INCLUDES_OCF_FS
    if (wiced_ota2_read_ocf( component_header_offset, (uint8_t*)component_header, sizeof(wiced_ota2_image_header_t)) != WICED_SUCCESS)
#else
    if (wiced_ota2_read_sflash( component_header_offset, (uint8_t*)component_header, sizeof(wiced_ota2_image_header_t)) != WICED_SUCCESS)
#endif
    {
        /* read header fail */
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota_read_component_header() read Failed!\r\n");
        return WICED_ERROR;
    }
    wiced_ota2_image_component_header_swap_network_order(component_header, WICED_OTA2_IMAGE_SWAP_NETWORK_TO_HOST);
    return WICED_SUCCESS;
}

static wiced_result_t wiced_ota2_image_copy_uncompressed_component(wiced_ota2_image_type_t ota_type,
                                                                  wiced_ota2_image_header_t* ota2_header,
                                                                  wiced_ota2_image_component_t* component_header,
                                                                  ota2_boot_type_t new_boot_type)
{
    wiced_result_t  result = WICED_SUCCESS;
#ifdef CURRENT_APPLICATION_USES_INTERNAL_FLASH
    wiced_bool_t    erase_sectors = WICED_TRUE;
#endif
    uint32_t    pos, chunk_size, base_ota_offset;
    uint32_t    src_offset;
    uint32_t    src_size;
    uint32_t    dst_offset;
    uint16_t    progress = 0;
    OTA2_CRC_VAR crc_check = OTA2_CRC_INIT_VALUE;

    base_ota_offset = wiced_ota2_image_get_offset( ota_type );
    if (base_ota_offset == 0)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_copy_uncompressed_component(%s) get_address() failed\r\n", ota2_image_type_name_string[ota_type]);
        return WICED_BADARG;
    }

    if ((ota2_header == NULL) || (component_header == NULL))
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_copy_uncompressed_component() bad args\r\n");
        return WICED_BADARG;
    }

    /* point to the component in the OTA image file and the destination */
    src_offset = (base_ota_offset + ota2_header->data_start + component_header->source_offset);
    src_size   = component_header->source_size;
    dst_offset = (OTA2_IMAGE_FLASH_BASE + component_header->destination);

    /* for the DCT, we need to choose the non-current DCT to write to */
    if ( (component_header->type == WICED_OTA2_IMAGE_COMPONENT_DCT) && (new_boot_type != OTA2_BOOT_NEVER_RUN_BEFORE) )
    {
        uint32_t curr_dct;
        curr_dct = (uint32_t)wiced_dct_get_current_address( DCT_INTERNAL_SECTION );
        if (curr_dct == (uint32_t)GET_CURRENT_ADDRESS_FAILED)
        {
            return WICED_ERROR;
        }
        dst_offset = OTA2_IMAGE_FLASH_BASE + (curr_dct == PLATFORM_DCT_COPY1_START_ADDRESS ) ?
                                                          PLATFORM_DCT_COPY2_START_ADDRESS : PLATFORM_DCT_COPY1_START_ADDRESS;
    }

    pos = 0;
    crc_check = OTA2_CRC_INIT_VALUE;
    while ( pos < src_size )
    {
        chunk_size = ((src_size - pos) > SECTOR_SIZE ) ? SECTOR_SIZE : ( src_size - pos );
        memset(component_decomp_ram, 0x00, chunk_size);
#ifdef WICED_PLATFORM_INCLUDES_OCF_FS
        if (wiced_ota2_read_ocf( (src_offset + pos), component_decomp_ram, chunk_size) != WICED_SUCCESS)
#else
        if (wiced_ota2_read_sflash( (unsigned long)(src_offset + pos), component_decomp_ram, chunk_size) != WICED_SUCCESS)
#endif
        {
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_copy_uncompressed_component() read component chunk FAILED!\r\n");
            result = WICED_ERROR;
            goto _no_comp_end;
        }

        crc_check = OTA2_CRC_FUNCTION(component_decomp_ram, chunk_size, crc_check);

        /* We need to write the DCT component into the non-current DCT area
         *    AND set the boot_type, crc32 and write_incomplete while writing the sectors
         *    AFTER writing all the sectors, clear crc32 and write_incomplete
         *
         * Do these changes *after* the crc_check update - this crc_check is for validating the extraction,
         *    and is not the DCT crc32 value.
         */
        if ( (component_header->type == WICED_OTA2_IMAGE_COMPONENT_DCT) && (new_boot_type != OTA2_BOOT_NEVER_RUN_BEFORE) )
        {
#if  defined(DCT_BOOTLOADER_CRC_IS_IN_HEADER)
            /* Change the DCT crc32 - set to 0x00 when writing DCT is finished */
            if  ( ((uint32_t)pos <= OFFSETOF(platform_dct_data_t, dct_header)) &&
                  (((uint32_t)pos + chunk_size) >= (OFFSETOF(platform_dct_data_t, dct_header) + OFFSETOF(platform_dct_header_t, crc32))) )
            {
                uint32_t offset;
                /*
                 * Find the offset to the dct_header.crc32 within the DCT we are about to write and set to 0x0001
                 * into the structure before we save it.
                 */
                offset = (OFFSETOF(platform_dct_data_t, dct_header) + OFFSETOF(platform_dct_header_t, crc32)) - (uint32_t)pos;
                component_decomp_ram[offset] = 0x01;
            }
#else
            /* Change the DCT crc32 - set to 0x00 when writing DCT is finished */
            if  ( ((uint32_t)pos <= OFFSETOF(platform_dct_data_t, dct_version)) &&
                  (((uint32_t)pos + chunk_size) >= (OFFSETOF(platform_dct_data_t, dct_version) + OFFSETOF(platform_dct_version_t, crc32))) )
            {
                uint32_t offset;
                /*
                 * Find the offset to the dct_header.crc32 within the DCT we are about to write and set to 0x0001
                 * into the structure before we save it.
                 */
                offset = (OFFSETOF(platform_dct_data_t, dct_version) + OFFSETOF(platform_dct_version_t, crc32)) - (uint32_t)pos;
                component_decomp_ram[offset] = 0x01;
            }

#endif
            /* Change the DCT write_incomplete - clear it at the end of DCT writing */
            if  ( ((uint32_t)pos <= OFFSETOF(platform_dct_data_t, dct_header)) &&
                  (((uint32_t)pos + chunk_size) >= (OFFSETOF(platform_dct_data_t, dct_header) + OFFSETOF(platform_dct_header_t, write_incomplete))) )
            {
                uint32_t offset;
                /*
                 * Find the offset to the dct_header.write_incomplete within the DCT we are about to write and set to non-zero
                 * into the structure before we save it.
                 */
                offset = (OFFSETOF(platform_dct_data_t, dct_header) + OFFSETOF(platform_dct_header_t, write_incomplete)) - (uint32_t)pos;
                component_decomp_ram[offset] = 0x01;
            }

            /* Change the DCT boot_type */
            if  ( ((uint32_t)pos <= OFFSETOF(platform_dct_data_t, ota2_config)) &&
                  (((uint32_t)pos + chunk_size) >= (OFFSETOF(platform_dct_data_t, ota2_config) + OFFSETOF(platform_dct_ota2_config_t, boot_type))) )
            {
                uint32_t offset;
                /*
                 * Find the offset to the ota2_config.boot_type within the DCT we are about to write and write the new boot type
                 * into the structure before we save it.
                 */
                offset = (OFFSETOF(platform_dct_data_t, ota2_config) + OFFSETOF(platform_dct_ota2_config_t, boot_type)) - (uint32_t)pos;
                component_decomp_ram[offset] = new_boot_type;
            }
        }
#ifdef CURRENT_APPLICATION_USES_INTERNAL_FLASH
        if (component_header->type == WICED_OTA2_IMAGE_COMPONENT_APP0)
        {
            //First erase app section
            if (erase_sectors == WICED_TRUE)
            {
                OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "\r\nErasing App sectors of internal flash!\r\n");
                if (platform_erase_flash( PLATFORM_APP_INT_START_SECTOR, PLATFORM_APP_INT_END_SECTOR ) != 0)
                {
                    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "ERROR erasing internal flash!\r\n");
                }
                else
                {
                    OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "Internal flash App area successfully erased!\r\n");
                }
                erase_sectors = WICED_FALSE;
            }

            if (wiced_ota2_safe_write_data_to_internal_flash( (dst_offset + pos), component_decomp_ram, chunk_size) != WICED_SUCCESS)
            {
                OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "ERROR writing component to internal flash!\r\n");
                result = WICED_ERROR;
                goto _no_comp_end;
            }
        }
        else
        {
#endif
#ifdef WICED_PLATFORM_INCLUDES_OCF_FS
            if (wiced_ota2_safe_write_data_to_ocf( (dst_offset + pos), component_decomp_ram, chunk_size) != WICED_SUCCESS)
#else
            if (wiced_ota2_safe_write_data_to_sflash( (dst_offset + pos), component_decomp_ram, chunk_size) != WICED_SUCCESS)
#endif
            {
                OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_copy_uncompressed_component() wiced_ota2_safe_write_data_to_sflash() failed!\r\n");
                result = WICED_ERROR;
                goto _no_comp_end;
            }
#ifdef CURRENT_APPLICATION_USES_INTERNAL_FLASH
        }
#endif

        if ((progress++ & 0x03) == 0)
        {
            mini_printf(".");
        }
        /* Blink LED while we are Extracting pieces of the component from the OTA2 Image */
        wiced_led_set_state(WICED_LED_INDEX_1, ((progress & 0x01) == 0) ? WICED_LED_OFF : WICED_LED_ON);

        pos += chunk_size;
    } /* while pos < src_size */

    /* Now that the DCT is completely written, set it so that it is the valid DCT
     *    set crc32 and write_incomplete to 0x00
     */
    if ( (component_header->type == WICED_OTA2_IMAGE_COMPONENT_DCT) && (new_boot_type != OTA2_BOOT_NEVER_RUN_BEFORE) )
    {
        uint32_t        offset;
        char            write_incomplete = 0;
        OTA2_CRC_VAR    crc32 = 0x00000000;
#ifdef WICED_PLATFORM_INCLUDES_OCF_FS
#if  defined(DCT_BOOTLOADER_CRC_IS_IN_HEADER)
        /* Change the DCT crc32 - set to 0x00 when writing DCT is finished */
        offset = (OFFSETOF(platform_dct_data_t, dct_header) + OFFSETOF(platform_dct_header_t, crc32));

        if ( 0 != wiced_ota2_safe_write_data_to_ocf ( (dst_offset + offset) , &crc32, sizeof(crc32)) )
        {
            result = WICED_ERROR;
            goto _no_comp_end;
        }
#else
        /* Change the DCT crc32 - set to 0x00 when writing DCT is finished */
        offset = (OFFSETOF(platform_dct_data_t, dct_version) + OFFSETOF(platform_dct_version_t, crc32));

        if ( 0 != wiced_ota2_safe_write_data_to_ocf ( (dst_offset + offset) , (uint8_t*)&crc32, sizeof(crc32)) )
        {
            result = WICED_ERROR;
            goto _no_comp_end;
        }
#endif
        /* Change the DCT write_incomplete - clear it at the end of DCT writing */
        offset = (OFFSETOF(platform_dct_data_t, dct_header) + OFFSETOF(platform_dct_header_t, write_incomplete));

        if ( 0 != wiced_ota2_safe_write_data_to_ocf ( (dst_offset + offset) , (uint8_t*)&write_incomplete, sizeof(write_incomplete)) )
        {
            result = WICED_ERROR;
            goto _no_comp_end;
        }
#else
        sflash_handle_t sflash_handle;
        sflash_write_t  sflash_write_func = ( PLATFORM_SECUREDCT_ENABLED ) ? sflash_dct_write_secure : sflash_write;

        if (init_sflash( &sflash_handle, PLATFORM_SFLASH_PERIPHERAL_ID, SFLASH_WRITE_ALLOWED )!= 0)
        {
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "wiced_ota2_image_copy_uncompressed_component() Failed to init SFLASH for writing.\r\n");
            result = WICED_ERROR;
            goto _no_comp_end;
        }

#if  defined(DCT_BOOTLOADER_CRC_IS_IN_HEADER)
        /* Change the DCT crc32 - set to 0x00 when writing DCT is finished */
        offset = (OFFSETOF(platform_dct_data_t, dct_header) + OFFSETOF(platform_dct_header_t, crc32));

        if ( 0 != sflash_write_func( &sflash_handle, (unsigned long)(dst_offset + offset) , &crc32, sizeof(crc32)) )
        {
            result = WICED_ERROR;
            goto _deint_sflash;
        }
#else
        /* Change the DCT crc32 - set to 0x00 when writing DCT is finished */
        offset = (OFFSETOF(platform_dct_data_t, dct_version) + OFFSETOF(platform_dct_version_t, crc32));

        if ( 0 != sflash_write_func( &sflash_handle, (unsigned long)(dst_offset + offset) , &crc32, sizeof(crc32)) )
        {
            result = WICED_ERROR;
            goto _deint_sflash;
        }
#endif
        /* Change the DCT write_incomplete - clear it at the end of DCT writing */
        offset = (OFFSETOF(platform_dct_data_t, dct_header) + OFFSETOF(platform_dct_header_t, write_incomplete));

        if ( 0 != sflash_write_func( &sflash_handle, (unsigned long)(dst_offset + offset) , &write_incomplete, sizeof(write_incomplete)) )
        {
            result = WICED_ERROR;
            goto _deint_sflash;
        }

_deint_sflash:
        if (deinit_sflash( &sflash_handle ) != WICED_SUCCESS)   /* TODO: leave sflash in a known state */
        {
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "wiced_ota2_read_sflash() Failed to deinit SFLASH.\r\n");
            result = WICED_ERROR;
        }
#endif
    }

_no_comp_end:

mini_printf("\r\n");

    if (component_header->crc != crc_check)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_copy_uncompressed_component() CRC Failed (expected -> 0x%lx != 0x%lx <- actual) size:%ld %ld %s.\r\n",
                         (uint32_t)component_header->crc, (uint32_t)crc_check,
                         component_header->source_size, component_header->destination_size,
                         component_header->name);
        result = WICED_ERROR;
    }

    /* Turn LED ON when done Extracting */
    wiced_led_set_state(WICED_LED_INDEX_1, WICED_LED_ON);
    return result;
}

/******************************************************
 *               External Functions
 ******************************************************/

/**
 * Validate OTA Image
 *
 * @param[in]  ota_type     - OTA Image type
 *
 * @return WICED_SUCCESS
 *         WICED_ERROR      - Bad OTA Image
 *         WICED_BADARG     - NULL pointer passed in or bad size
 */
wiced_result_t wiced_ota2_image_validate ( wiced_ota2_image_type_t ota_type )
{
    wiced_ota2_image_header_t    ota2_header;
    wiced_ota2_image_component_t component_header;
    uint16_t                     index;
    uint32_t                     offset, size;
    OTA2_CRC_VAR                 header_crc;     /* for calculating the CRC */
    OTA2_CRC_VAR                 ota2_header_crc; /* The OTA header CRC before we clear it to re-compute */
    wiced_bool_t                 print_header;

    if (wiced_ota2_read_ota2_header(ota_type, &ota2_header) != WICED_SUCCESS)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_validate() wiced_ota2_read_ota2_header() fail.\r\n");
        return WICED_ERROR;
    }

    /* check magic string */
    if (strncmp((char*)ota2_header.magic_string, WICED_OTA2_IMAGE_MAGIC_STRING, WICED_OTA2_IMAGE_MAGIC_STR_LEN) != 0)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_validate() OTA Image Magic String fail.\r\n");
        return WICED_ERROR;
    }

    /* OTA version */
    if (ota2_header.ota2_version != WICED_OTA2_IMAGE_VERSION)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_validate() OTA Image version fail.\r\n");
        return WICED_ERROR;
    }

    /* check hardware platform - may or may not have APPS_CHIP_REVISION in OTA2 Image header */
    if (strncmp((char *)ota2_header.platform_name, PLATFORM, strlen(PLATFORM)) != 0)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_validate() OTA Image Platform type fail. %s != %s \r\n", ota2_header.platform_name, PLATFORM);
        return WICED_UNSUPPORTED;
    }

    /* check component count */
    if ((ota2_header.component_count == 0) || (ota2_header.component_count > DCT_MAX_APP_COUNT))
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_validate() OTA Image component count fail. %d\r\n", ota2_header.component_count);
        return WICED_ERROR;
    }

    /* check downloaded bytes vs. expected image size  */
    if (ota2_header.bytes_received < ota2_header.image_size)
    {
        /* this is normal while we are downloading  */
        if (ota2_header.download_status != WICED_OTA2_IMAGE_DOWNLOAD_IN_PROGRESS)
        {
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "wiced_ota2_image_validate() OTA Image size error: actual -> %ld != %ld <- expected\r\n", ota2_header.bytes_received, ota2_header.image_size);
            return WICED_ERROR;
        }
    }

    print_header = WICED_TRUE;

    switch( (wiced_ota2_image_status_t)ota2_header.download_status )
    {
        case WICED_OTA2_IMAGE_DOWNLOAD_IN_PROGRESS:
            if (ota2_header.bytes_received < ota2_header.data_start)
            {
                /* not enough bytes to check the header CRC, assume all is going well */
                return WICED_SUCCESS;
            }
            print_header = WICED_FALSE;
            break;
        default:
        case WICED_OTA2_IMAGE_INVALID:
        case WICED_OTA2_IMAGE_DOWNLOAD_FAILED:
        case WICED_OTA2_IMAGE_DOWNLOAD_UNSUPPORTED:
            OTA2_WPRINTF("wiced_ota2_image_validate() fail download_status:%d.\r\n", ota2_header.download_status);
            return WICED_ERROR;
        case WICED_OTA2_IMAGE_DOWNLOAD_EXTRACTED:   /* extracted is possibly valid - needed for getting version after extracted */
        case WICED_OTA2_IMAGE_VALID:
        case WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE:
        case WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT:
            break;
    }

    if (print_header == WICED_TRUE)
    {
        wiced_ota2_print_header_info(&ota2_header, "OTA Image Validate:");
    }

     /* prepare for Header CRC check
     * We do not need to save these values here, as they are local
     */
    /* save this so we can check it ! */
    ota2_header_crc = ota2_header.header_crc;
    ota2_header.header_crc = 0;

    /* only clear this out for non-FACTORY_RESET images */
    if (ota_type != WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP)
    {
        ota2_header.download_status = 0;
        ota2_header.bytes_received = 0;
    }

    header_crc = OTA2_CRC_INIT_VALUE;
    header_crc = OTA2_CRC_FUNCTION((uint8_t*)&ota2_header, sizeof(wiced_ota2_image_header_t), header_crc);

    /* check that the component sizes are reasonable (the source size fits in the image) */
    for (index = 0; index < ota2_header.component_count; index++)
    {
        /* read the component header */
        if (wiced_ota_read_component_header( ota_type, index, &component_header) != WICED_SUCCESS)
        {
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_validate() wiced_ota_read_component_header() failed. \r\n");
            return WICED_ERROR;
        }

        if (print_header == WICED_TRUE)
        {
            wiced_ota2_print_component_info(&component_header);
        }

        /* if source size is greater than destination, we have a problem! */
        if (component_header.source_size > component_header.destination_size)
        {
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_validate() OTA Image component %ld source size > dest %ld \r\n",
                             component_header.source_size, component_header.destination_size);
            return WICED_ERROR;
        }

        offset = component_header.source_offset;   /* offset from start of data (after components in OTA Image) */
        size   =  component_header.source_size;    /* size in the in OTA Image */
        /* if component offset + size is greater than the total image size, this is bad */
        if ((ota2_header.data_start + offset + size) > ota2_header.image_size)
        {
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_validate() OTA Image component source + size > image size %ld > %ld\r\n",
                           (ota2_header.data_start + offset + size), ota2_header.image_size);
            return WICED_ERROR;
        }

        header_crc = OTA2_CRC_FUNCTION((uint8_t*)&component_header, sizeof(wiced_ota2_image_component_t), header_crc);
    }

    /* check crc value */
    if (ota2_header_crc != header_crc)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "\r\nwiced_ota2_image_validate() computed CRC 0x%lx does not match OTA Header CRC 0x%lx \r\n\r\n",
                        header_crc, ota2_header_crc);
        return WICED_ERROR;
    }

    // TODO: other checks?
//    dst_addr = component_header.destination;
//    dst_size = component_header.destination_size;

    return WICED_SUCCESS;

}

/**
 * Extract OTA2 extractor (or Apps LUT) from the OTA2 Image to the current area
 *
 * NOTE: This is used by the OTA2 Failsafe code
 *
 * @param[in]  ota_type     - OTA Image type
 * @param[in]  component    - OTA Component to extract (ONLY WICED_OTA2_IMAGE_COMPONENT_LUT or WICED_OTA2_IMAGE_COMPONENT_OTA_APP)
 * @param[out] destination  - address extracted to in FLASH
 * @param[out] destination  - extracted size
 *
 * @return WICED_SUCCESS
 *         WICED_ERROR      - Bad OTA Image, not fully downloaded
 *         WICED_BADARG     - NULL pointer passed in or bad size
 */
wiced_result_t wiced_ota2_image_extract_uncompressed_component( wiced_ota2_image_type_t ota_type, wiced_ota2_image_component_type_t component,
                                                                uint32_t* destination, uint32_t* destination_size )
{
    int                             index;
    wiced_ota2_image_header_t       ota2_header;
    wiced_ota2_image_component_t    component_header;
    wiced_ota2_image_status_t       status;
    wiced_result_t                  result = WICED_ERROR;

    /* we only support LUT and OTA_APP */
    if ((component != WICED_OTA2_IMAGE_COMPONENT_LUT) && (component != WICED_OTA2_IMAGE_COMPONENT_OTA_APP))
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_extract_uncompressed_component(%s) bad arg!\r\n", ota2_image_type_name_string[ota_type]);
        return WICED_BADARG;
    }

    if ((destination == NULL) || (destination_size == NULL))
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_extract_uncompressed_component(%s) bad arg!\r\n", ota2_image_type_name_string[ota_type]);
        return WICED_BADARG;
    }

    *destination = 0;
    *destination_size = 0;

    /* validate the image */
    if (wiced_ota2_image_validate(ota_type) != WICED_SUCCESS)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_extract_uncompressed_component(%s) validate() failed\r\n", ota2_image_type_name_string[ota_type]);
        return WICED_BADARG;
    }

    /* check download status */
    wiced_ota2_image_get_status ( ota_type, &status );
    if ( (status != WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE) && (status != WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT) &&
         (status != WICED_OTA2_IMAGE_VALID))
    {
        return WICED_ERROR;
    }

     if (wiced_ota2_read_ota2_header(ota_type, &ota2_header) != WICED_SUCCESS)
     {
         return WICED_ERROR;
     }

    /* Extract the component and check CRC */

    /* read the component header(s) and find the right component */
     for (index = 0; index < ota2_header.component_count; index++)
     {

        if (wiced_ota_read_component_header( ota_type, index, &component_header) != WICED_SUCCESS)
        {
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_extract_uncompressed_component() wiced_ota_read_component_header() failed. \r\n");
            return WICED_ERROR;
        }
        if (component_header.type == component)
        {
            if (component_header.compression != WICED_OTA2_IMAGE_COMPONENT_COMPRESSION_NONE)
            {
                OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_extract_uncompressed_component() component is compressed - fail. \r\n");
                return WICED_ERROR;
            }

            mini_printf("extracting: %s\r\n", component_header.name);

            result = wiced_ota2_image_copy_uncompressed_component( ota_type, &ota2_header, &component_header, OTA2_BOOT_NEVER_RUN_BEFORE);
            if (result == WICED_SUCCESS)
            {
                *destination      = component_header.destination;
                *destination_size =  component_header.destination_size;
            }
            break;
        }
     }

    return result;
}

/**
 * Extract OTA Image
 * NOTE: All information regarding location of data in the system is part of the OTA Image.
 *
 * @param[in]  ota_type     - OTA Image type
 *
 * @return WICED_SUCCESS
 *         WICED_ERROR      - Bad OTA Image, not fully downloaded
 *         WICED_BADARG     - NULL pointer passed in or bad size
 */
wiced_result_t wiced_ota2_image_extract ( wiced_ota2_image_type_t ota_type )
{
    uint16_t                    index;
    wiced_ota2_image_header_t   ota2_header;
    wiced_ota2_image_status_t   status;
    wiced_result_t              result;
    platform_dct_ota2_config_t  ota2_config;
    ota2_boot_type_t            starting_boot_type;

    /* validate the image */
    if (wiced_ota2_image_validate(ota_type) != WICED_SUCCESS)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_extract(%s) validate() failed\r\n", ota2_image_type_name_string[ota_type]);
        return WICED_BADARG;
    }

    /* check download status */
    wiced_ota2_image_get_status ( ota_type, &status );
    if ( (status != WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE) && (status != WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT) &&
         (status != WICED_OTA2_IMAGE_VALID))
    {
        return WICED_ERROR;
    }

#ifdef CHECK_BATTERY_LEVEL_BEFORE_OTA2_UPGRADE
    /* check for battery level before doing any writing! - todo: make this a platform-specific function*/
     if (platform_check_battery_level(CHECK_BATTERY_LEVEL_OTA2_UPGRADE_MINIMUM) != WICED_SUCCESS)
     {
         OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "check_battery_level() failed\r\n");
         return WICED_ERROR;
     }
#endif /* CHECK_BATTERY_LEVEL_BEFORE_OTA2_UPGRADE */

     if (wiced_ota2_read_ota2_header(ota_type, &ota2_header) != WICED_SUCCESS)
     {
         return WICED_ERROR;
     }

     OTA2_WPRINTF("\r\n");

     /* set the boot_type so that if there is a reset or power cycle, we will recover using OTA2 Failsafe */
     wiced_dct_read_with_copy( &ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) );
     switch(ota_type)
     {
         case WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP:
             ota2_config.boot_type = OTA2_BOOT_FAILSAFE_FACTORY_RESET;
             break;
         case WICED_OTA2_IMAGE_TYPE_STAGED:
             ota2_config.boot_type = OTA2_BOOT_FAILSAFE_UPDATE;
             break;
         default:
             /* we don't extract anything else - return badarg */
             return WICED_BADARG;
     }
     /* keep track of the boot type so we can re-write it after the DCT has been extracted! */
     starting_boot_type = ota2_config.boot_type;
     wiced_dct_write( &ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) );

    /* Extract the components and check CRCs while doing so - bail out if any one component fails */
     result = WICED_SUCCESS;
    for (index = 0; index < ota2_header.component_count; index++)
    {
#ifdef  DEBUG_GET_DECOMP_TIME
        wiced_time_t    start_time, end_time;
#endif
        wiced_ota2_image_component_t  component_header;

        /* read the component header */
        if (wiced_ota_read_component_header( ota_type, index, &component_header) != WICED_SUCCESS)
        {
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_extract() wiced_ota_read_component_header() failed. \r\n");
            return WICED_ERROR;
        }

        wiced_ota2_print_component_info(&component_header);

        mini_printf("extracting: %s\r\n", component_header.name);

#ifdef  DEBUG_GET_DECOMP_TIME
        wiced_time_get_time( &start_time );
#endif
        /* decompress component and write to FLASH */
        /* need to do this in chunks  !!! The file system is > 800k !!! */
        switch (component_header.compression)  // TODO
        {
            case WICED_OTA2_IMAGE_COMPONENT_COMPRESSION_LZW:   // TODO
                OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_extract() LZW COMPRESSION UNSUPPORTED .\r\n");
                result = WICED_ERROR;
                break;
            case WICED_OTA2_IMAGE_COMPONENT_COMPRESSION_GZIP:  // TODO
                OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_extract() GZIP COMPRESSION UNSUPPORTED .\r\n");
                result = WICED_ERROR;
                break;
            case WICED_OTA2_IMAGE_COMPONENT_COMPRESSION_BZ2:   // TODO
                OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_extract() BZ2 COMPRESSION UNSUPPORTED .\r\n");
                result = WICED_ERROR;
                break;
            case WICED_OTA2_IMAGE_COMPONENT_COMPRESSION_NONE:
                result = wiced_ota2_image_copy_uncompressed_component( ota_type, &ota2_header, &component_header, starting_boot_type);
                break;
            default:
                OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_extract() UNDEFINED COMPRESSION UNSUPPORTED .\r\n");
                result = WICED_ERROR;
                break;
        } /* switch */

#ifdef  DEBUG_GET_DECOMP_TIME
        wiced_time_get_time( &end_time );
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "\r\ntime: %ld\r\n", (end_time - start_time));
#endif
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "\r\n");

    }   /* for component_count */

    /* we always want to leave the Factory Reset Application status alone - no touchy-touchy! */
    if (ota_type != WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP)
    {
        wiced_ota2_image_update_staged_status(WICED_OTA2_IMAGE_DOWNLOAD_EXTRACTED);
    }

    return result;
}

#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS

/**
 * Set current image type (staging or FS) before calling wiced_ota2_image_write_data
 * If this function is not called then Staging image will be the default image type.
 *
 * @param[in]  image_type - current image type WICED_OTA2_IMAGE_TYPE_STAGED or
 *                                      WICED_OTA2_IMAGE_TYPE_FS
 *
 * @return - WICED_SUCCESS
 */
wiced_result_t  wiced_ota2_set_current_update (wiced_ota2_image_type_t image_type)
{
    g_curr_image_type = image_type;

    return WICED_SUCCESS;
}
#endif

/**
 * Write OTA Image to the Staging area
 * NOTE: The total size of the OTA image is included in a valid OTA image header.
 *       This function will update the status as appropriate
 *
 * @param[in]  data      - pointer to part or all of an OTA image to be stored in the staging area
 * @param[in]  offset    - offset from start of OTA Image to store this data
 * @param[in]  size      - size of the data to store
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG
 */
wiced_result_t  wiced_ota2_image_write_data(uint8_t* data, uint32_t offset, uint32_t size)
{
    wiced_result_t              result;
    wiced_ota2_image_header_t   ota2_header;

    if ((data == NULL) || (size == 0))
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_write() Bad Args data:%p size:%ld!\r\n", data, size);
        return WICED_BADARG;
    }

#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
    /* We always write to the staging area, make sure we don't go past the area size */
    if (g_curr_image_type == WICED_OTA2_IMAGE_TYPE_STAGED)
    {
        if ( (offset + size) > OTA2_IMAGE_STAGING_AREA_SIZE)
        {
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_write() offset:0x%lx + size:0x%lx larger than staging area: 0x%x!\r\n", offset, size, OTA2_IMAGE_STAGING_AREA_SIZE);
            return WICED_BADARG;
        }
    }
    else if (g_curr_image_type == WICED_OTA2_IMAGE_TYPE_FS)
    {
        if ( (offset + size) > OTA2_IMAGE_FS_AREA_SIZE)
        {
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_write() offset:0x%lx + size:0x%lx larger than fs area: 0x%x!\r\n", offset, size, OTA2_IMAGE_FS_AREA_SIZE);
            return WICED_BADARG;
        }
    }
    else
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_write() Image type %d is not Valid\r\n", (int)g_curr_image_type);
        return WICED_BADARG;
    }
#else

    /* We always write to the staging area, make sure we don't go past the area size */
    if ( (offset + size) > OTA2_IMAGE_STAGING_AREA_SIZE)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_write() offset:0x%lx + size:0x%lx larger than staging area: 0x%x!\r\n", offset, size, OTA2_IMAGE_STAGING_AREA_SIZE);
        return WICED_BADARG;
    }
#endif

    /* Blink LED OFF while writing to FLASH (downloading an OTA2 Image) */
    wiced_led_set_state(WICED_LED_INDEX_1, WICED_LED_OFF);
#ifdef WICED_PLATFORM_INCLUDES_OCF_FS
#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
   if (wiced_ota2_safe_write_data_to_ocf( (wiced_ota2_image_get_offset(g_curr_image_type) + offset), data, size) != WICED_SUCCESS)
#else
    if (wiced_ota2_safe_write_data_to_ocf( (wiced_ota2_image_get_offset(WICED_OTA2_IMAGE_TYPE_STAGED) + offset), data, size) != WICED_SUCCESS)
#endif //WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
#else
    if (wiced_ota2_safe_write_data_to_sflash( (wiced_ota2_image_get_offset(WICED_OTA2_IMAGE_TYPE_STAGED) + offset), data, size) != WICED_SUCCESS)
#endif //WICED_PLATFORM_INCLUDES_OCF_FS
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_write() wiced_ota2_safe_write_data_to_sflash() failed!\r\n");
        return WICED_ERROR;
    }

    /* Turn LED ON when done writing to FLASH */
    wiced_led_set_state(WICED_LED_INDEX_1, WICED_LED_ON);
#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
    if (wiced_ota2_read_ota2_header(g_curr_image_type, &ota2_header) != WICED_SUCCESS)
#else
    if (wiced_ota2_read_ota2_header(WICED_OTA2_IMAGE_TYPE_STAGED, &ota2_header) != WICED_SUCCESS)
#endif
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_write() wiced_ota2_read_ota2_header() failed!\r\n");
        return WICED_ERROR;
    }

    /* update the header info */
    result = wiced_ota2_image_update_staged_header(ota2_header.bytes_received + size);
    if (result == WICED_UNSUPPORTED)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_write() wiced_ota2_update_staged_header() OTA image not supported!\r\n");
        return WICED_UNSUPPORTED;
    }

    if (result != WICED_SUCCESS)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_write() wiced_ota2_update_staged_header() failed!\r\n");
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/** Get the OTA2 application software version from the header
 *
 * NOTE: Only updates the major & minor values on WICED_SUCCESS
 *
 * @param image_type [in]   : OTA2 Image type to get software app version
 * @param major [out]   : ptr to store software major version
 * @param minor [out]   : ptr to store software minor version
 *
 * @return  WICED_SUCCESS
 *          WICED_BADARG
 *          WICED_ERROR
 */
wiced_result_t wiced_ota2_image_get_version(wiced_ota2_image_type_t image_type, uint16_t* major, uint16_t* minor)
{
    wiced_ota2_image_header_t   ota2_header;

    if ((major == NULL) || (minor == NULL))
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_get_version() Bad Args!\r\n");
        return WICED_BADARG;
    }

    if (wiced_ota2_image_validate ( image_type ) != WICED_SUCCESS)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_get_version() wiced_ota2_image_validate() failed!\r\n");
        return WICED_ERROR;
    }

    if (wiced_ota2_read_ota2_header(image_type, &ota2_header) != WICED_SUCCESS)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_image_get_version() wiced_ota2_read_ota2_header() failed!\r\n");
        return WICED_ERROR;
    }

    *major = ota2_header.major_version;
    *minor = ota2_header.minor_version;

    return WICED_SUCCESS;
}

/** Update the OTA image header after writing (parts of) the downloaded OTA image to FLASH
 *
 * @param total_bytes_received - number of bytes written to the image
 *
 * @return  WICED_SUCCESS
 *          WICED_BADARG
 *          WICED_ERROR
 */
wiced_result_t wiced_ota2_image_update_staged_header(uint32_t total_bytes_received)
{
    wiced_result_t              result;
    wiced_ota2_image_header_t   ota2_header;

#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
    if (wiced_ota2_read_ota2_header(g_curr_image_type, &ota2_header) != WICED_SUCCESS)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_update_staged_header() wiced_ota2_read_ota2_header() failed!\r\n");
        return WICED_ERROR;
    }
#else
    if (wiced_ota2_read_ota2_header(WICED_OTA2_IMAGE_TYPE_STAGED, &ota2_header) != WICED_SUCCESS)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_update_staged_header() wiced_ota2_read_ota2_header() failed!\r\n");
        return WICED_ERROR;
    }
#endif

    if (total_bytes_received > ota2_header.image_size)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_update_staged_header() total_bytes_received %ld > %ld ota2_header.image_size!\r\n", total_bytes_received, ota2_header.image_size);
        return WICED_ERROR;
    }

    ota2_header.bytes_received = total_bytes_received;

    /* assume in progress */
    ota2_header.download_status = WICED_OTA2_IMAGE_DOWNLOAD_IN_PROGRESS;
    if (ota2_header.bytes_received == 0)
    {
        ota2_header.download_status = WICED_OTA2_IMAGE_INVALID;
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_update_staged_header() ota2_header.bytes_received == 0!\r\n");
    }
    else if (ota2_header.bytes_received >= ota2_header.image_size)
    {
        ota2_header.download_status = WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE;
    }
#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
     if (wiced_ota2_write_ota2_header(g_curr_image_type, &ota2_header) != WICED_SUCCESS)
#else
    if (wiced_ota2_write_ota2_header(WICED_OTA2_IMAGE_TYPE_STAGED, &ota2_header) != WICED_SUCCESS)
#endif
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_update_staged_header() wiced_ota2_write_ota2_header() FAILED!\r\n");
        return WICED_ERROR;
    }

    /* verify that the image is valid, if not, mark as failed */
#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
    result = wiced_ota2_image_validate(g_curr_image_type);
#else
    result = wiced_ota2_image_validate(WICED_OTA2_IMAGE_TYPE_STAGED);
#endif
    if (result == WICED_UNSUPPORTED)    /* result from validate */
    {
        ota2_header.download_status = WICED_OTA2_IMAGE_DOWNLOAD_UNSUPPORTED;
    }
    else if (result != WICED_SUCCESS)
    {
        ota2_header.download_status = WICED_OTA2_IMAGE_DOWNLOAD_FAILED;
    }

    return result;
}

/** Update the OTA image header status
 *
 * @param total_bytes_received - number of bytes written to the image
 *
 * @return  WICED_SUCCESS
 *          WICED_BADARG
 *          WICED_ERROR
 */
wiced_result_t wiced_ota2_image_update_staged_status(wiced_ota2_image_status_t new_status)
{
    wiced_ota2_image_header_t    ota2_header;

#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
    if (wiced_ota2_read_ota2_header(g_curr_image_type, &ota2_header) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }
#else
    if (wiced_ota2_read_ota2_header(WICED_OTA2_IMAGE_TYPE_STAGED, &ota2_header) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }
#endif

    ota2_header.download_status = new_status;

#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
    if (wiced_ota2_write_ota2_header(g_curr_image_type, &ota2_header) != WICED_SUCCESS)
#else
    if (wiced_ota2_write_ota2_header(WICED_OTA2_IMAGE_TYPE_STAGED, &ota2_header) != WICED_SUCCESS)
#endif
    {
        return WICED_ERROR;
    }

    /* If we are complete, verify that the image is valid, if not, mark as failed */
    if ((ota2_header.download_status == WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE) ||
        (ota2_header.download_status == WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT))
    {
        /* verify that the image is valid, if not, mark as failed */
#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
        if (wiced_ota2_image_validate(g_curr_image_type) != WICED_SUCCESS)
#else
        if (wiced_ota2_image_validate(WICED_OTA2_IMAGE_TYPE_STAGED) != WICED_SUCCESS)
#endif
        {
            ota2_header.download_status = WICED_OTA2_IMAGE_DOWNLOAD_FAILED;
#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
            if (wiced_ota2_write_ota2_header(g_curr_image_type, &ota2_header) != WICED_SUCCESS)
#else
            if (wiced_ota2_write_ota2_header(WICED_OTA2_IMAGE_TYPE_STAGED, &ota2_header) != WICED_SUCCESS)
#endif
            {
                return WICED_ERROR;
            }
        }
    }

    /* read and print out the header info */
#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
    if (wiced_ota2_read_ota2_header(g_curr_image_type, &ota2_header) != WICED_SUCCESS)
#else
    if (wiced_ota2_read_ota2_header(WICED_OTA2_IMAGE_TYPE_STAGED, &ota2_header) != WICED_SUCCESS)
#endif
    {
        return WICED_ERROR;
    }

    wiced_ota2_print_header_info(&ota2_header, "OTA Image Header after Updating OTAImage header:");
    return WICED_SUCCESS;

}

/** Call this to set the flag to force a facory reset on reboot
 *  NOTE: This is equal to holding the "factory reset button" for 5 seconds.
 *          Use this if there is no button on the system.
 *
 * @param   N/A
 *
 * @return  WICED_SUCCESS
 *          WICED_ERROR
 */
wiced_result_t wiced_ota2_force_factory_reset_on_reboot( void )
{
    platform_dct_ota2_config_t  dct_ota2_config;
#ifndef WICED_PLATFORM_INCLUDES_OCF_FS
/* Note: In case of on chip flash (OCF) there is no separate factory reset image. Due to less memory.
  * So we are skipping the image validation part and set the DCT ota2 config section for force factory reset.
  * So in the next boot when bootloader found factory reset is needed, it will start factory reset app.
  */
    wiced_ota2_image_header_t   ota2_header;

    /* check if the factory image is valid */
    if (wiced_ota2_read_ota2_header(WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP, &ota2_header) != WICED_SUCCESS)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_force_factory_reset_on_reboot() wiced_ota2_read_ota2_header() FAILED!\r\n");
        return WICED_ERROR;
    }

    if (ota2_header.download_status != WICED_OTA2_IMAGE_VALID)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_force_factory_reset_on_reboot() ota2_header.download_status != WICED_OTA2_IMAGE_VALID!\r\n");
        return WICED_ERROR;
    }
#endif
    /* set the dct flag to force the Factory reset at reboot */
    if( wiced_dct_read_with_copy( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) ) == WICED_SUCCESS)
    {
        dct_ota2_config.force_factory_reset = 1;
        if( wiced_dct_write( &dct_ota2_config, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t) ) != WICED_SUCCESS)
        {
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_force_factory_reset_on_reboot() wiced_dct_write() failed!\r\n");
            return WICED_ERROR;
        }
    }

    return WICED_SUCCESS;
}

/** Get the last boot type
 *
 * @param   N/A
 *
 * @return  ota2_boot_type_t
 */
ota2_boot_type_t wiced_ota2_get_boot_type( void )
{
    wiced_result_t      result;
    ota2_boot_type_t boot_type = OTA2_BOOT_NORMAL;

    /* Did User cause a Factory Reset ? */
    platform_dct_ota2_config_t  ota2_header;

    /* is saved DCT valid ? */
    result = wiced_dct_ota2_read_saved_copy( &ota2_header, DCT_OTA2_CONFIG_SECTION, 0, sizeof(platform_dct_ota2_config_t));
    if (result == WICED_SUCCESS)
    {
        boot_type = ota2_header.boot_type;
    }
    else
    {
        boot_type = OTA2_BOOT_NEVER_RUN_BEFORE;
    }

    return boot_type;
}

/* DEBUGGING - fake that we downloaded the image
 * download the image manually before calling this...
 */
wiced_result_t wiced_ota2_image_fakery(wiced_ota2_image_status_t new_status)
{
    wiced_ota2_image_header_t    ota2_header;

#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
    if (wiced_ota2_read_ota2_header(g_curr_image_type, &ota2_header) != WICED_SUCCESS)
#else
    if (wiced_ota2_read_ota2_header(WICED_OTA2_IMAGE_TYPE_STAGED, &ota2_header) != WICED_SUCCESS)
#endif
    {
        return WICED_ERROR;
    }

    ota2_header.bytes_received = ota2_header.image_size;
    ota2_header.download_status = new_status;

#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
    if (wiced_ota2_write_ota2_header(g_curr_image_type, &ota2_header) != WICED_SUCCESS)
#else
    if (wiced_ota2_write_ota2_header(WICED_OTA2_IMAGE_TYPE_STAGED, &ota2_header) != WICED_SUCCESS)
#endif
    {
        return WICED_ERROR;
    }

    wiced_ota2_print_header_info(&ota2_header, "OTA Image Header after FAKING IT OTAImage header:");
    return WICED_SUCCESS;
}
#endif  /* !defined(BOOTLOADER) */

/**
 * read data from FLASH
 *
 * @param[in]  offset   - offset to the data
 * @param[in]  data     - the data area to copy into
 * @param[in]  size     - size of the data
 *
 * @return - WICED_SUCCESS
 *           WICED_BADARG
 */
#ifdef WICED_PLATFORM_INCLUDES_OCF_FS
static wiced_result_t wiced_ota2_read_ocf(uint32_t offset, uint8_t* data, uint32_t size)
{
    wiced_result_t  result = WICED_SUCCESS;

    if (ocf_buffer_read( offset, data, size) != 0 )
    {
        /* read sector fail */
        //OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_read_sflash() read error!\r\n");
        printf ("wiced_ota2_read_sflash() read error!\r\n");
        result = WICED_ERROR;
    }

    return result;
}
#else
static wiced_result_t wiced_ota2_read_sflash(uint32_t offset, uint8_t* data, uint32_t size)
{
    wiced_result_t  result = WICED_SUCCESS;
    sflash_handle_t sflash_handle;

    if (init_sflash( &sflash_handle, PLATFORM_SFLASH_PERIPHERAL_ID, SFLASH_WRITE_NOT_ALLOWED ) != WICED_SUCCESS)
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_read_sflash() Failed to init SFLASH for reading.\r\n");
        return WICED_ERROR;
    }

    if (sflash_read( &sflash_handle, (unsigned long)offset, data, size) != 0 )
    {
        /* read sector fail */
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_read_sflash() read error!\r\n");
        result = WICED_ERROR;
    }

    if (deinit_sflash( &sflash_handle ) != WICED_SUCCESS)   /* TODO: leave sflash in a known state */
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_read_sflash() Failed to deinit SFLASH.\r\n");
        result = WICED_ERROR;
    }

    return result;
}

#endif

/* *****************************************************
 *
 *read the ota2_header and swap values from network order to host order
 */
static wiced_result_t wiced_ota2_read_ota2_header(wiced_ota2_image_type_t ota_type, wiced_ota2_image_header_t* ota2_header)
{
    uint32_t    base_ota2_offset;

    base_ota2_offset = wiced_ota2_image_get_offset(ota_type);
    if ((base_ota2_offset == 0) || (ota2_header == NULL))
    {
        OTA2_WPRINTF("wiced_ota2_read_ota2_header() Bad args.\r\n");
        return WICED_BADARG;
    }
#ifdef WICED_PLATFORM_INCLUDES_OCF_FS
    if (wiced_ota2_read_ocf( base_ota2_offset, (uint8_t*)ota2_header, sizeof(wiced_ota2_image_header_t)) != WICED_SUCCESS)
#else
    if (wiced_ota2_read_sflash( base_ota2_offset, (uint8_t*)ota2_header, sizeof(wiced_ota2_image_header_t)) != WICED_SUCCESS)
#endif
    {
        OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "wiced_ota2_read_ota2_header(%s) Failed to read SFLASH.\r\n", ota2_image_type_name_string[ota_type]);
        return WICED_ERROR;
    }

    wiced_ota2_image_header_swap_network_order(ota2_header, WICED_OTA2_IMAGE_SWAP_NETWORK_TO_HOST);
    return WICED_SUCCESS;
}

/**
 * Get status of OTA Image at download location
 *
 * @param[in]  ota_type     - OTA Image type
 * @param[out] status       - Receives the OTA Image status.
 *
 * @return WICED_SUCCESS
 *         WICED_ERROR      - Bad OTA Image
 *         WICED_BADARG     - NULL pointer passed in
 */
wiced_result_t wiced_ota2_image_get_status ( wiced_ota2_image_type_t ota_type, wiced_ota2_image_status_t *status )
{
    wiced_ota2_image_header_t    ota2_header;

    if (status == NULL)
    {
        OTA2_WPRINTF("wiced_ota2_get_status() bad arg.\r\n");
        return WICED_BADARG;
    }

    if (ota_type == WICED_OTA2_IMAGE_TYPE_CURRENT_APP)
    {
        /* Current App - OK */
        *status = WICED_OTA2_IMAGE_VALID;
        return WICED_SUCCESS;
    }

    if (wiced_ota2_read_ota2_header(ota_type, &ota2_header) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* return the status */
    *status = ota2_header.download_status;

    switch (*status)
    {
        case WICED_OTA2_IMAGE_DOWNLOAD_IN_PROGRESS:
            OTA2_WPRINTF("wiced_ota2_get_status(%d) Download in progress.\r\n", ota_type);
            break;
        case WICED_OTA2_IMAGE_DOWNLOAD_FAILED:
            OTA2_WPRINTF("wiced_ota2_get_status(%d) Download FAILED!\r\n", ota_type);
            break;
        case WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE:
            OTA2_WPRINTF("wiced_ota2_get_status(%d) Download complete. Ready for extract.\r\n", ota_type);
            break;
        case WICED_OTA2_IMAGE_VALID:
            OTA2_WPRINTF("wiced_ota2_get_status(%d) Download valid.\r\n", ota_type);
            break;
        case WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT:
            OTA2_WPRINTF("wiced_ota2_get_status(%d) Download complete. Extract on Reboot.\r\n", ota_type);
            break;
        case WICED_OTA2_IMAGE_DOWNLOAD_EXTRACTED:
            OTA2_WPRINTF("wiced_ota2_get_status(%d) Download complete. Download extracted (Used).\r\n", ota_type);
            break;
        case WICED_OTA2_IMAGE_INVALID:
        default:
            OTA2_WPRINTF("wiced_ota2_get_status(%d) Download invalid.\r\n", ota_type);
            break;
    }
    return WICED_SUCCESS;
}

/**
 * Get the OTA Image offset into the SFLASH
 *
 * @param[in]  ota_type     - OTA Image type
 *
 * @return - offset to the ota image header
 *           0 if bad ota_type
 */
uint32_t wiced_ota2_image_get_offset( wiced_ota2_image_type_t ota_type )
{
    uint32_t   ota_offset = 0;
    switch (ota_type)
    {
        case WICED_OTA2_IMAGE_TYPE_FACTORY_RESET_APP:
            ota_offset = OTA2_IMAGE_FACTORY_RESET_AREA_BASE;
            break;
        case WICED_OTA2_IMAGE_TYPE_STAGED:
            ota_offset = OTA2_IMAGE_STAGING_AREA_BASE;
            break;
        case WICED_OTA2_IMAGE_TYPE_LAST_KNOWN_GOOD:
#if defined(OTA2_IMAGE_LAST_KNOWN_GOOD_AREA_BASE)
            ota_offset= OTA2_IMAGE_LAST_KNOWN_GOOD_AREA_BASE;
            break;
#endif
#ifdef WICED_PLATFORM_SUPPORTS_OTA2_UPDATE_FS
        case WICED_OTA2_IMAGE_TYPE_FS:
            ota_offset = OTA2_IMAGE_CURR_FS_AREA_BASE;
            break;
#endif
        case WICED_OTA2_IMAGE_TYPE_NONE:
        default:
            OTA2_WPRINT_MSG( WLF_OTA2, WICED_LOG_DEBUG0, "wiced_ota2_image_get_offset() UNKNOWN OTA_TYPE. \r\n");
            break;
    }
    return ota_offset;
}
