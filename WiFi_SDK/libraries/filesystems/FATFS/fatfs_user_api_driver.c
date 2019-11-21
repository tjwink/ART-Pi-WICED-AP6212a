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
 *  User API driver for FatFs
 *  Adapts the top level FatFS API to match the Wiced API
 */

#include "wiced_result.h"
#include "internal/wiced_filesystem_internal.h"

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
static wiced_result_t fatfs_shim_init             ( void );
static wiced_result_t fatfs_shim_mount            ( wiced_block_device_t* device, wiced_filesystem_t* fs_handle_out );
static wiced_result_t fatfs_shim_unmount          ( wiced_filesystem_t* fs_handle );
static wiced_result_t fatfs_shim_file_get_details ( wiced_filesystem_t* fs_handle, const char* filename, wiced_dir_entry_details_t* details_out );
static wiced_result_t fatfs_shim_file_open        ( wiced_filesystem_t* fs_handle, wiced_file_t* file_handle_out, const char* filename, wiced_filesystem_open_mode_t mode );
static wiced_result_t fatfs_shim_file_seek        ( wiced_file_t* file_handle, int64_t offset, wiced_filesystem_seek_type_t whence );
static wiced_result_t fatfs_shim_file_tell        ( wiced_file_t* file_handle, uint64_t* location );
static wiced_result_t fatfs_shim_file_read        ( wiced_file_t* file_handle, void* data, uint64_t bytes_to_read, uint64_t* returned_bytes_count );
static wiced_result_t fatfs_shim_file_write       ( wiced_file_t* file_handle, const void* data, uint64_t bytes_to_write, uint64_t* written_bytes_count );
static wiced_result_t fatfs_shim_file_flush       ( wiced_file_t* file_handle );
static int            fatfs_shim_file_end_reached ( wiced_file_t* file_handle );
static wiced_result_t fatfs_shim_file_close       ( wiced_file_t* file_handle );
static wiced_result_t fatfs_shim_file_delete      ( wiced_filesystem_t* fs_handle, const char* filename );
static wiced_result_t fatfs_shim_dir_open         ( wiced_filesystem_t* fs_handle, wiced_dir_t* dir_handle, const char* dir_name );
static wiced_result_t fatfs_shim_dir_read         ( wiced_dir_t* dir_handle, char* name_buffer, unsigned int name_buffer_length, wiced_dir_entry_type_t* type, wiced_dir_entry_details_t* details );
static int            fatfs_shim_dir_end_reached  ( wiced_dir_t* dir_handle );
static wiced_result_t fatfs_shim_dir_rewind       ( wiced_dir_t* dir_handle );
static wiced_result_t fatfs_shim_dir_close        ( wiced_dir_t* dir_handle );
static wiced_result_t fatfs_shim_dir_create       ( wiced_filesystem_t* fs_handle, const char* directory_name );
static wiced_result_t fatfs_shim_format           ( wiced_block_device_t* device );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/* This is the User API driver structure for FatFS */
wiced_filesystem_driver_t wiced_filesystem_driver_fatfs =
{
    .init             = fatfs_shim_init            ,
    .mount            = fatfs_shim_mount           ,
    .unmount          = fatfs_shim_unmount         ,
    .file_get_details = fatfs_shim_file_get_details,
    .file_open        = fatfs_shim_file_open       ,
    .file_seek        = fatfs_shim_file_seek       ,
    .file_tell        = fatfs_shim_file_tell       ,
    .file_read        = fatfs_shim_file_read       ,
    .file_write       = fatfs_shim_file_write      ,
    .file_flush       = fatfs_shim_file_flush      ,
    .file_end_reached = fatfs_shim_file_end_reached,
    .file_close       = fatfs_shim_file_close      ,
    .file_delete      = fatfs_shim_file_delete     ,
    .dir_open         = fatfs_shim_dir_open        ,
    .dir_read         = fatfs_shim_dir_read        ,
    .dir_end_reached  = fatfs_shim_dir_end_reached ,
    .dir_rewind       = fatfs_shim_dir_rewind      ,
    .dir_close        = fatfs_shim_dir_close       ,
    .dir_create       = fatfs_shim_dir_create      ,
    .dir_delete       = fatfs_shim_file_delete     ,
    .format           = fatfs_shim_format          ,
};

#ifndef FATFS_VER011
BYTE Buff[4096];        /* Working buffer */
#endif
/******************************************************
 *               Function Definitions
 ******************************************************/

/* Initialises FatFS shim - nothing to be done */
static wiced_result_t fatfs_shim_init             ( void )
{
    return WICED_SUCCESS;
}

/* Internal function for mounting a FatFS filesystem from a block device (with a "mount_now" parameter) */
static wiced_result_t fatfs_internal_mount( wiced_block_device_t* device, wiced_filesystem_t* fs_handle_out, wiced_bool_t mount_now )
{
    static uint8_t next_drive_id = 0;
    FRESULT fatfs_result;

    /* Create a logical drive name  "0:", "1:" etc */
    unsigned_to_decimal_string( next_drive_id, (char*) &fs_handle_out->data.fatfs.drive_id, 1, 3 );
    strcat( (char*) &fs_handle_out->data.fatfs.drive_id, ":" );

    /* Mount the drive */
#ifdef FATFS_VER011
    fatfs_result = f_mount ( &fs_handle_out->data.fatfs.handle, (char*) &fs_handle_out->data.fatfs.drive_id, (mount_now == WICED_TRUE)? 1 : 0, device );
#else
    fatfs_result = f_mount ( &fs_handle_out->data.fatfs.handle, (char*) &fs_handle_out->data.fatfs.drive_id, (mount_now == WICED_TRUE)? 1 : 0 );
#endif
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    if (mount_now == WICED_TRUE)
    {
      next_drive_id++;
    }

    return WICED_SUCCESS;
}

/* Unmounts a FatFS filesystem from a block device */
static wiced_result_t fatfs_shim_unmount          ( wiced_filesystem_t* fs_handle )
{
    FRESULT fatfs_result;

    /* Unmount the drive */
#ifdef FATFS_VER011
    fatfs_result = f_mount ( NULL, (TCHAR*) &fs_handle->data.fatfs.drive_id, 1, NULL );
#else
    fatfs_result = f_mount ( NULL, (TCHAR*) &fs_handle->data.fatfs.drive_id, 1 );
#endif
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }
    return WICED_SUCCESS;
}

/* Formats a block device with a FatFS filesystem */
static wiced_result_t fatfs_shim_format( wiced_block_device_t* device )
{
    FRESULT fatfs_result;
    wiced_result_t result;
    wiced_filesystem_t fs_handle;

    /* Check that the block sizes are OK */

    /* Init the block device to populate the device sizes */
    result = device->driver->init( device, BLOCK_DEVICE_READ_ONLY );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    if ( ( device->erase_block_size != BLOCK_DEVICE_ERASE_NOT_REQUIRED ) &&
         ( ( ( device->erase_block_size < DEFAULT_SECTOR_SIZE ) &&
             ( ( DEFAULT_SECTOR_SIZE % device->erase_block_size ) != 0 ) ) ||
           ( ( device->erase_block_size >= DEFAULT_SECTOR_SIZE ) &&
             ( ( device->erase_block_size % DEFAULT_SECTOR_SIZE ) != 0 ) ) ) )
    {
        /* Erase block size is invalid - not a multiple or sub-multiple of DEFAULT_SECTOR_SIZE */
        return WICED_FILESYSTEM_BLOCK_SIZE_BAD;
    }

    if ( ( device->write_block_size != BLOCK_DEVICE_WRITE_NOT_ALLOWED ) &&
         ( ( ( device->write_block_size < DEFAULT_SECTOR_SIZE ) &&
             ( ( DEFAULT_SECTOR_SIZE % device->write_block_size ) != 0 ) ) ||
           ( ( device->write_block_size >= DEFAULT_SECTOR_SIZE ) &&
             ( ( device->write_block_size % DEFAULT_SECTOR_SIZE ) != 0 ) ) ) )
    {
        /* Write block size is invalid - not a multiple or sub-multiple of DEFAULT_SECTOR_SIZE */
        return WICED_FILESYSTEM_BLOCK_SIZE_BAD;
    }

    if ( ( ( device->read_block_size < DEFAULT_SECTOR_SIZE ) &&
           ( ( DEFAULT_SECTOR_SIZE % device->read_block_size ) != 0 ) ) ||
         ( ( device->read_block_size >= DEFAULT_SECTOR_SIZE ) &&
           ( ( device->read_block_size % DEFAULT_SECTOR_SIZE ) != 0 ) ) )
    {
        /* Read block size is invalid - not a multiple or sub-multiple of DEFAULT_SECTOR_SIZE */
        return WICED_FILESYSTEM_BLOCK_SIZE_BAD;
    }

    /* Temporarily mount the drive (with  mount-later flag) */
    result = fatfs_internal_mount( device, &fs_handle, WICED_FALSE );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    /* Format device */
#ifdef FATFS_VER011
    fatfs_result = f_mkfs ( (char*) &fs_handle.data.fatfs.drive_id, 1, DEFAULT_SECTOR_SIZE );
#else
    fatfs_result = f_mkfs ( (char*) &fs_handle.data.fatfs.drive_id, FM_SFD | FM_FAT32, DEFAULT_SECTOR_SIZE, Buff, sizeof(Buff) );
#endif
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    /* Unmount again */
    return fatfs_shim_unmount( &fs_handle );
}

/* Mounts a FatFS filesystem from a block device */
static wiced_result_t fatfs_shim_mount            ( wiced_block_device_t* device, wiced_filesystem_t* fs_handle_out )
{
    wiced_result_t result;

    /* Format if required */
    if ( device->init_data->volatile_and_requires_format_when_mounting == WICED_TRUE )
    {
        /* Format device */
        result = fatfs_shim_format( device );
        if ( result != WICED_SUCCESS )
        {
            return result;
        }
    }

    /* Do the mount */
    return fatfs_internal_mount( device, fs_handle_out, WICED_TRUE );
}

/* Opens a file within a FatFS filesystem */
static wiced_result_t fatfs_shim_file_open        ( wiced_filesystem_t* fs_handle, wiced_file_t* file_handle_out, const char* filename, wiced_filesystem_open_mode_t mode )
{
    FRESULT fatfs_result;
#ifdef FATFS_VER011
    FATFS_BYTE    fatfs_mode;
#else
    BYTE    fatfs_mode;
#endif
    FIL*    file_handle =  &file_handle_out->data.fatfs;

    /* Match the Wiced mode to a FatFS mode */
    switch ( mode )
    {
        case WICED_FILESYSTEM_OPEN_ZERO_LENGTH:
            fatfs_mode = FA_CREATE_ALWAYS | FA_READ | FA_WRITE;
            break;

        case WICED_FILESYSTEM_OPEN_WRITE_CREATE:
        case WICED_FILESYSTEM_OPEN_APPEND_CREATE:
            fatfs_mode = FA_OPEN_ALWAYS | FA_READ | FA_WRITE;
            break;

        case WICED_FILESYSTEM_OPEN_FOR_READ:
            fatfs_mode = FA_OPEN_EXISTING | FA_READ;
            break;

        case WICED_FILESYSTEM_OPEN_FOR_WRITE:
        case WICED_FILESYSTEM_OPEN_APPEND:
            fatfs_mode = FA_OPEN_EXISTING | FA_READ | FA_WRITE;
            break;

        default:
            /* Unknown mode */
            return WICED_BADARG;
    }

    /* Change default drive to the requested drive */
    fatfs_result = f_chdrive ( (char*) &fs_handle->data.fatfs.drive_id );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    /* Open the file */
    fatfs_result = f_open ( file_handle, filename, fatfs_mode );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    /* If appending was requested , move to the end of the file */
    if ( ( mode == WICED_FILESYSTEM_OPEN_APPEND ) ||
         ( mode == WICED_FILESYSTEM_OPEN_APPEND_CREATE ) )
    {
        /* Seek to end of the file */
        fatfs_result = f_lseek( file_handle, f_size( file_handle ) );
        if ( fatfs_result != FR_OK )
        {
            f_close( file_handle );
            return WICED_FILESYSTEM_ERROR;
        }
    }
    return WICED_SUCCESS;
}

/* Get details of a file within a FatFS filesystem */
static wiced_result_t fatfs_shim_file_get_details ( wiced_filesystem_t* fs_handle, const char* filename, wiced_dir_entry_details_t* details_out )
{
    FILINFO file_stat;
    FRESULT fatfs_result;

    /* Change default drive to the requested drive */
    fatfs_result = f_chdrive ( (TCHAR*) &fs_handle->data.fatfs.drive_id );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    fatfs_result = f_stat ( filename, &file_stat );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    /* Fill in the details structure */
    details_out->size                  = file_stat.fsize;
    details_out->attributes_available  = file_stat.fattrib;
    details_out->date_time             = ( file_stat.fdate << 16 ) + file_stat.ftime;  /* TODO: This is wrong - need to do conversion */
    details_out->attributes_available  = WICED_TRUE;
    details_out->date_time_available   = WICED_TRUE;
    details_out->permissions_available = WICED_FALSE;

    return WICED_SUCCESS;
}

/* Close a file within a FatFS filesystem */
static wiced_result_t fatfs_shim_file_close       ( wiced_file_t* file_handle )
{
    FRESULT fatfs_result;

    fatfs_result = f_close( &file_handle->data.fatfs );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    return WICED_SUCCESS;
}

/* Delete a file within a FatFS filesystem */
static wiced_result_t fatfs_shim_file_delete      ( wiced_filesystem_t* fs_handle, const char* filename )
{
    FRESULT fatfs_result;

    /* Change default drive to the requested drive */
    fatfs_result = f_chdrive ( (TCHAR*) &fs_handle->data.fatfs.drive_id );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    /* Delete the file */
    fatfs_result = f_unlink( filename );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    return WICED_SUCCESS;
}

/* Seek to a location in an open file within a FatFS filesystem */
static wiced_result_t fatfs_shim_file_seek        ( wiced_file_t* file_handle, int64_t offset, wiced_filesystem_seek_type_t whence )
{
    FRESULT fatfs_result;
#ifdef FATFS_VER011
    FATFS_DWORD   new_location;
    FATFS_DWORD   file_size = f_size( &file_handle->data.fatfs );
#else
    DWORD   new_location;
    DWORD   file_size = f_size( &file_handle->data.fatfs );
#endif
    /* Translate Wiced "whence" to an absolute location, since FatFS does not do relative seeks */
    switch ( whence )
    {
        case WICED_FILESYSTEM_SEEK_SET:
            new_location = offset;
            break;

        case WICED_FILESYSTEM_SEEK_CUR:
            new_location = f_tell( &file_handle->data.fatfs ) + offset;
            break;

        case WICED_FILESYSTEM_SEEK_END:
            new_location = file_size + offset;
            break;

        default:
            return WICED_BADARG;
    }

    if ( new_location < 0 )
    {
        return WICED_BADARG;
    }

    /* Perform the seek */
    fatfs_result = f_lseek ( &file_handle->data.fatfs, new_location );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    return WICED_SUCCESS;
}

/* Get the current location in an open file within a FatFS filesystem */
static wiced_result_t fatfs_shim_file_tell        ( wiced_file_t* file_handle, uint64_t* location )
{
    *location = f_tell( &file_handle->data.fatfs );
    return WICED_SUCCESS;
}

/* Read data from an open file within a FatFS filesystem */
static wiced_result_t fatfs_shim_file_read        ( wiced_file_t* file_handle, void* data, uint64_t bytes_to_read, uint64_t* returned_bytes_count )
{
    FRESULT fatfs_result;
#ifdef FATFS_VER011
    FATFS_UINT    bytes_read;
#else
    UINT    bytes_read;
#endif
    fatfs_result = f_read ( &file_handle->data.fatfs, data, bytes_to_read, &bytes_read );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    *returned_bytes_count = bytes_read;

    return WICED_SUCCESS;
}

/* Write data to an open file within a FatFS filesystem */
static wiced_result_t fatfs_shim_file_write       ( wiced_file_t* file_handle, const void* data, uint64_t bytes_to_write, uint64_t* written_bytes_count )
{
    FRESULT fatfs_result;
#ifdef FATFS_VER011
    FATFS_UINT    bytes_written;
#else
    UINT    bytes_written;
#endif
    fatfs_result = f_write ( &file_handle->data.fatfs, data, bytes_to_write, &bytes_written );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    *written_bytes_count = bytes_written;

    return WICED_SUCCESS;
}

/* Flush unwritten data in an open file within a FatFS filesystem */
static wiced_result_t fatfs_shim_file_flush       ( wiced_file_t* file_handle )
{
    FRESULT fatfs_result;

    fatfs_result = f_sync( &file_handle->data.fatfs );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    return WICED_SUCCESS;
}

/* Get end-of-file (EOF) flag for an open file within a FatFS filesystem */
static int            fatfs_shim_file_end_reached ( wiced_file_t* file_handle )
{
    return f_eof( &file_handle->data.fatfs );
}

/* Opens a directory within a FatFS filesystem */
static wiced_result_t fatfs_shim_dir_open         ( wiced_filesystem_t* fs_handle, wiced_dir_t* dir_handle, const char* dir_name )
{
    FRESULT fatfs_result;

    /* Change default drive to the requested drive */
    fatfs_result = f_chdrive ( (TCHAR*) &fs_handle->data.fatfs.drive_id );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    fatfs_result = f_opendir ( &dir_handle->data.fatfs.handle, (TCHAR*) dir_name );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    dir_handle->data.fatfs.eodir = WICED_FALSE;

    return WICED_SUCCESS;
}

/* Reads directory entry from an open within a FatFS filesystem */
static wiced_result_t fatfs_shim_dir_read         ( wiced_dir_t* dir_handle, char* name_buffer, unsigned int name_buffer_length, wiced_dir_entry_type_t* type, wiced_dir_entry_details_t* details )
{
    FRESULT fatfs_result;
    FILINFO info;
#ifdef FATFS_VER011
#if _USE_LFN
    info.lfname = name_buffer;
    info.lfsize = name_buffer_length;
#endif /* if _USE_LFN */
#endif
    fatfs_result = f_readdir ( &dir_handle->data.fatfs.handle, &info );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    /* Zero length filename indicates end of directory entries */
    if ( info.fname[0] == '\x00' )
    {
        dir_handle->data.fatfs.eodir = WICED_TRUE;
        return WICED_FILESYSTEM_END_OF_RESOURCE;
    }

    /* Copy the directory entry name to the output buffer */
#ifdef FATFS_VER011
#if _USE_LFN
    if ( name_buffer[0] == '\x00' ) /* Indicates that the filename is only in the short filename part */
    {
        strlcpy( name_buffer, info.fname, MIN( name_buffer_length, 13 ) );
    }
#else
    strlcpy( name_buffer, info.fname, MIN( name_buffer_length, 13 ) );
#endif /* _USE_LFN */
#else
    strlcpy( name_buffer, info.fname, MIN( name_buffer_length, sizeof(info.fname) ) );
#endif
    /* Copy the directory entry details to the detail structure */
    details->size                  = info.fsize;
    details->attributes            = info.fattrib;
    details->date_time             = ( info.fdate << 16 ) + info.ftime;  /* TODO: This is wrong - need to do conversion */
    details->attributes_available  = WICED_TRUE;
    details->date_time_available   = WICED_TRUE;
    details->permissions_available = WICED_FALSE;


    *type = ( ( details->attributes & AM_DIR ) != 0 ) ? WICED_FILESYSTEM_DIR : WICED_FILESYSTEM_FILE;

    return WICED_SUCCESS;
}

/* Get end-of-directory flag for an open directory within a FatFS filesystem */
static int            fatfs_shim_dir_end_reached  ( wiced_dir_t* dir_handle )
{
    return dir_handle->data.fatfs.eodir;
}

/* Moves the current location within a directory back to the first entry within a FatFS filesystem */
static wiced_result_t fatfs_shim_dir_rewind       ( wiced_dir_t* dir_handle )
{
    FRESULT fatfs_result;

    /* Rewind is achieved by passing NULL to f_readdir */
    fatfs_result = f_readdir ( &dir_handle->data.fatfs.handle, NULL );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    dir_handle->data.fatfs.eodir = WICED_FALSE;

    return WICED_SUCCESS;
}

/* Closes an open directory within a FatFS filesystem */
static wiced_result_t fatfs_shim_dir_close        ( wiced_dir_t* dir_handle )
{
    FRESULT fatfs_result;

    fatfs_result = f_closedir( &dir_handle->data.fatfs.handle );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    return WICED_SUCCESS;
}

/* Creates a new directory within a FatFS filesystem */
static wiced_result_t fatfs_shim_dir_create       ( wiced_filesystem_t* fs_handle, const char* directory_name )
{
    FRESULT fatfs_result;

    /* Change default drive to the requested drive */
    fatfs_result = f_chdrive ( (TCHAR*) &fs_handle->data.fatfs.drive_id );
    if ( fatfs_result != FR_OK )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    /* Create the directory */
    fatfs_result = f_mkdir( directory_name );
    if ( ( fatfs_result != FR_OK ) && ( fatfs_result != FR_EXIST ) )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    return WICED_SUCCESS;
}
