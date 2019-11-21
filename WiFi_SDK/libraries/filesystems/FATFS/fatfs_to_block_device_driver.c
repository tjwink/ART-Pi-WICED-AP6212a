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
 *  Block device driver for FatFs
 *  Provides the functions which are called by the bottom
 *  of FatFS
 */

/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"        /* FatFs lower layer API */
#include "wiced_block_device.h"
#include "internal/wiced_filesystem_internal.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/* Only enable ONE of the following options */

/* Enable to make File-X read-only */
//#define FATFS_WRITE_STRATEGY  BLOCK_DEVICE_READ_ONLY

#define FATFS_WRITE_STRATEGY  BLOCK_DEVICE_WRITE_IMMEDIATELY

/* Enable to allow write-behind in File-X */
//#define FATFS_WRITE_STRATEGY  BLOCK_DEVICE_WRITE_BEHIND_ALLOWED

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
 *               Function Definitions
 ******************************************************/

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/
#ifndef FATFS_VER011
static wiced_block_device_t* fatfs_block_dev = NULL;
static wiced_block_device_t* get_block_device(void)
{
    if (!fatfs_block_dev)
    {
        const filesystem_list_t* curr_item = all_filesystem_devices;

        while ( curr_item->device != NULL )
        {
            if ( curr_item->type == WICED_FILESYSTEM_HANDLE_FATFS)
            {
                fatfs_block_dev = curr_item->device;
                break;
            }
        }
    }
    return fatfs_block_dev;
}
#endif

DSTATUS disk_status (
#ifdef FATFS_VER011
    void* user_data        /* Physical drive nmuber to identify the drive */
#else
    BYTE pdrv              /* Physical drive nmuber to identify the drive */
#endif
)
{
    wiced_result_t result;
    wiced_block_device_status_t status;
#ifdef FATFS_VER011
    wiced_block_device_t* device = (wiced_block_device_t*) user_data;
#else
    wiced_block_device_t* device = get_block_device();
#endif

    /* Get the block device status */
    result = device->driver->status( device, &status );
    if ( result != WICED_SUCCESS )
    {
        return STA_NOINIT;
    }

    /* Translate the status into a FatFS status */
    if ( status == BLOCK_DEVICE_UNINITIALIZED )
    {
        return STA_NOINIT;
    }

    if ( status == BLOCK_DEVICE_DOWN )
    {
        return STA_NODISK;
    }

    if ( status == BLOCK_DEVICE_UP_READ_ONLY )
    {
        return STA_PROTECT;
    }

    return 0;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize (
#ifdef FATFS_VER011
        void* user_data                /* Physical drive nmuber to identify the drive */
#else
        BYTE pdrv
#endif
)
{
    wiced_result_t result;
#ifdef FATFS_VER011
    wiced_block_device_t* device = (wiced_block_device_t*) user_data;
#else
    wiced_block_device_t* device = get_block_device();
#endif

    /* Init the block device */
    result = device->driver->init( device, FATFS_WRITE_STRATEGY );
    if ( result != WICED_SUCCESS )
    {
        return STA_NOINIT;
    }

#ifdef FATFS_VER011
    return disk_status ( user_data );
#else
    return disk_status ( pdrv);
#endif
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
#ifdef FATFS_VER011
    void* user_data,         /* Physical drive nmuber to identify the drive */
    FATFS_BYTE *buff,        /* Data buffer to store read data */
    FATFS_DWORD sector,      /* Sector address in LBA */
    FATFS_UINT count         /* Number of sectors to read */
#else
    BYTE pdrv,
    BYTE *buff,        /* Data buffer to store read data */
    DWORD sector,      /* Sector address in LBA */
    UINT count         /* Number of sectors to read */
#endif
)
{
    /* Read from the block device */
    wiced_result_t result;
#ifdef FATFS_VER011
    wiced_block_device_t* device = (wiced_block_device_t*) user_data;
#else
    wiced_block_device_t* device = get_block_device();
#endif

    result = device->driver->read( device, sector * DEFAULT_SECTOR_SIZE, buff, count * DEFAULT_SECTOR_SIZE );

    if ( result != WICED_SUCCESS )
    {
        return RES_ERROR;
    }

    return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _FS_READONLY == 0
DRESULT disk_write (
#ifdef FATFS_VER011
    void* user_data,            /* Physical drive nmuber to identify the drive */
    const FATFS_BYTE *buff,     /* Data to be written */
    FATFS_DWORD sector,         /* Sector address in LBA */
    FATFS_UINT count            /* Number of sectors to write */
#else
    BYTE pdrv,           /* Physical drive nmuber to identify the drive */
    const BYTE *buff,    /* Data to be written */
    DWORD sector,        /* Sector address in LBA */
    UINT count           /* Number of sectors to write */
#endif
)
{
    /* Write to the block device */
    wiced_result_t result;
#ifdef FATFS_VER011
    wiced_block_device_t* device = (wiced_block_device_t*) user_data;
#else
    wiced_block_device_t* device = get_block_device();
#endif

    result = device->driver->write( device, sector * DEFAULT_SECTOR_SIZE, buff, count * DEFAULT_SECTOR_SIZE );

    if ( result != WICED_SUCCESS )
    {
        return RES_ERROR;
    }

    return RES_OK;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if !_FS_READONLY
DRESULT disk_ioctl (
#ifdef FATFS_VER011
    void* user_data,
    FATFS_BYTE cmd,        /* Control code */
#else
    BYTE pdrv,
    BYTE cmd,         /* Control code */
#endif
    void *buff        /* Buffer to send/receive control data */
)
{
    /* Handle the various IOCTL's */
#ifdef FATFS_VER011
    wiced_block_device_t* device = (wiced_block_device_t*) user_data;
#else
    wiced_block_device_t* device = get_block_device();
#endif

    switch ( cmd )
    {
        case CTRL_SYNC        :   /* Complete pending write process (needed at _FS_READONLY == 0) */
            if ( device->driver->flush( device ) != WICED_SUCCESS )
            {
                return RES_ERROR;
            }
            break;
        case GET_SECTOR_COUNT :   /* Get media size (needed at _USE_MKFS == 1) */
            {
#ifdef FATFS_VER011
                FATFS_DWORD* number_of_sectors = (FATFS_DWORD*) buff;
#else
                DWORD* number_of_sectors = (DWORD*) buff;
#endif
                *number_of_sectors = device->device_size / DEFAULT_SECTOR_SIZE;

            }
            break;
        case GET_SECTOR_SIZE  :   /* Get sector size (needed at _MAX_SS != _MIN_SS) */
            return RES_ERROR; /* TODO: Not implemented yet */
            break;
        case GET_BLOCK_SIZE   :   /* Get erase block size (needed at _USE_MKFS == 1) */
            {
#ifdef FATFS_VER011
                FATFS_DWORD* block_size = (FATFS_DWORD*) buff;
#else
                DWORD* block_size = (DWORD*) buff;
#endif
                *block_size = ( device->erase_block_size == BLOCK_DEVICE_ERASE_NOT_REQUIRED )? 1 : device->erase_block_size;
            }
            break;
        case CTRL_TRIM        :   /* Inform device that the data on the block of sectors is no longer used (needed at _USE_TRIM == 1) */
            {
#ifdef FATFS_VER011
                FATFS_DWORD* start_end_sector = (FATFS_DWORD*) buff;
#else
                DWORD* start_end_sector = (DWORD*) buff;
#endif
                uint64_t start = start_end_sector[0] * start_end_sector[2];
                uint64_t size  = (start_end_sector[1] - start_end_sector[0]) * start_end_sector[2];
                if ( ( device->driver->erase != NULL ) &&
                     ( device->driver->erase( device, start, size ) != WICED_SUCCESS ) )
                {
                    return RES_ERROR;
                }
            }
            break;

    }

    return RES_OK;
}
#endif



#if !_FS_READONLY && !_FS_NORTC
#ifdef FATFS_VER011
FATFS_DWORD get_fattime (void)
#else
DWORD get_fattime (void)
#endif
{
    wiced_time_t time = 0;
    wiced_time_get_time( &time );
    return time;
}
#endif /* if !_FS_READONLY && !_FS_NORTC */
