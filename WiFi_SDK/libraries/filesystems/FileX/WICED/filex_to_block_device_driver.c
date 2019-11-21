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
 *  Block device driver for FileX
 *  Provides the functions which are called by the bottom
 *  of FileX
 */

#include <stddef.h>
#include "tx_api.h"
#include "fx_api.h"
#include "wiced_block_device.h"
#include "wiced_filex.h"

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

/******************************************************
 *               Function Declarations
 ******************************************************/
/* from filex_utilities/file fx_partition_offset_calculate.c, no header available */
UINT    _fx_partition_offset_calculate(void  *partition_sector, UINT partition,
                                     ULONG *partition_start, ULONG *partition_size);

/******************************************************
 *               Function Definitions
 ******************************************************/

VOID  wiced_filex_driver( FX_MEDIA* media_ptr )
{
    /* Get a handle to the block device */
    wiced_block_device_t* device = (wiced_block_device_t*) media_ptr -> fx_media_driver_info;

    media_ptr -> fx_media_driver_status =  FX_ACCESS_ERROR;

    /* Process the driver request specified in the media control block.  */
    switch(media_ptr -> fx_media_driver_request)
    {

        case FX_DRIVER_INIT:
        {
            /* FileX is requesting to init the underlying block device */
            wiced_result_t result;

            result = device->driver->init( device, FILEX_WRITE_STRATEGY );
            if ( result != WICED_SUCCESS )
            {
                break;
            }

            /* Inform FileX whether the block device needs to know when sectors become free */
            if ( device->erase_block_size != BLOCK_DEVICE_ERASE_NOT_REQUIRED )
            {
                media_ptr ->fx_media_driver_free_sector_update = FX_TRUE;
            }

            /* Inform FileX whether the block device is write protected */
            if ( device->write_block_size == BLOCK_DEVICE_WRITE_NOT_ALLOWED )
            {
                media_ptr ->fx_media_driver_write_protect = FX_TRUE;
            }

            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            break;
        }

        case FX_DRIVER_UNINIT:
        {
            /* FileX is requesting to de-init the underlying block device */
            wiced_result_t result;

            result = device->driver->deinit( device );
            if ( result == WICED_SUCCESS )
            {
                media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            }

            break;
        }

        case FX_DRIVER_READ:
        {
            /* FileX is requesting to read from the underlying block device */
            wiced_result_t result;
            uint64_t requested_address = (uint64_t)(media_ptr -> fx_media_driver_logical_sector + media_ptr -> fx_media_hidden_sectors) * media_ptr -> fx_media_bytes_per_sector;
            uint64_t requested_size    = (uint64_t)(media_ptr -> fx_media_driver_sectors) * media_ptr -> fx_media_bytes_per_sector;

            result = device->driver->read( device, requested_address, media_ptr -> fx_media_driver_buffer, requested_size );

            if ( result == WICED_SUCCESS )
            {
                /* Successful driver request.  */
                media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            }
            break;
        }

        case FX_DRIVER_WRITE:
        {
            /* FileX is requesting to write to the underlying block device */
            wiced_result_t result;
            uint64_t requested_address = (uint64_t)(media_ptr -> fx_media_driver_logical_sector + media_ptr -> fx_media_hidden_sectors) * media_ptr -> fx_media_bytes_per_sector;
            uint64_t requested_size    = (uint64_t)(media_ptr -> fx_media_driver_sectors) * media_ptr -> fx_media_bytes_per_sector;

            result = device->driver->write( device, requested_address, media_ptr -> fx_media_driver_buffer, requested_size );

            if ( result == WICED_SUCCESS )
            {
                /* Successful driver request.  */
                media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            }
            break;
        }

        case FX_DRIVER_FLUSH:
        {
            /* FileX is requesting to flush the underlying block device */
            wiced_result_t result;

            /* Check whether flush is implemented on the device */
            if ( device->driver->flush == NULL )
            {
                /* Flush not implemented on this block device */
                media_ptr -> fx_media_driver_status =  FX_SUCCESS;
                break;
            }

            /* Perform the flush */
            result = device->driver->flush( device );

            if ( result == WICED_SUCCESS )
            {
                /* Successful driver request.  */
                media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            }
            break;
        }

        case FX_DRIVER_ABORT:
        {
            /* Abort is not implemented.  */
            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            break;
        }

        case FX_DRIVER_BOOT_READ:
        {
            /* FileX is requesting to read the boot sector of the underlying block device */
            wiced_result_t result;

            /* Read the first 512 bytes (at least)  */
            result = device->driver->read( device, media_ptr -> fx_media_hidden_sectors * FX_BOOT_SECTOR_SIZE, media_ptr -> fx_media_driver_buffer, MAX(FX_BOOT_SECTOR_SIZE, device->read_block_size) );
            if ( result != WICED_SUCCESS )
            {
                break;
            }

            if ( FX_SUCCESS != _fx_partition_offset_calculate( media_ptr -> fx_media_driver_buffer, 0, &media_ptr -> fx_media_hidden_sectors, NULL ) )
            {
                media_ptr -> fx_media_driver_status =  FX_MEDIA_INVALID;
                return;
            }

            /* Ensure this is less than the destination.  */
            /* Read the boot sector into the destination.  */
            result = device->driver->read( device, media_ptr -> fx_media_hidden_sectors * FX_BOOT_SECTOR_SIZE, media_ptr -> fx_media_driver_buffer, MIN( FX_BOOT_SECTOR_SIZE, media_ptr -> fx_media_memory_size ) );
            if ( result != WICED_SUCCESS )
            {
                break;
            }

            /* Successful driver request.  */
            media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            break;
        }

        case FX_DRIVER_BOOT_WRITE:
        {
            /* FileX is requesting to write to the underlying block device */
            wiced_result_t result;

            /* Write the boot record and return to the caller.  */
            result = device->driver->write( device, media_ptr -> fx_media_hidden_sectors * FX_BOOT_SECTOR_SIZE, media_ptr -> fx_media_driver_buffer, media_ptr -> fx_media_bytes_per_sector );
            if ( result == WICED_SUCCESS )
            {
                /* Successful driver request.  */
                media_ptr -> fx_media_driver_status =  FX_SUCCESS;
            }

            break ;
        }

        default:
        {

            /* Invalid driver request.  */
            media_ptr -> fx_media_driver_status =  FX_IO_ERROR;
            break;
        }
    }
}
