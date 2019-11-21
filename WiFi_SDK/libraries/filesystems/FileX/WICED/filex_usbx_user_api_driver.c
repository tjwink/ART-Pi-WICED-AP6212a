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
 *  User API driver for FileX
 *  Adapts the top level FileX API to match the Wiced API
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
static wiced_result_t filex_usbx_shim_init             ( void );
static wiced_result_t filex_usbx_shim_mount            ( wiced_block_device_t* device, wiced_filesystem_t* fs_handle_out );
static wiced_result_t filex_usbx_shim_unmount          ( wiced_filesystem_t* fs_handle );
static wiced_result_t filex_usbx_shim_file_get_details ( wiced_filesystem_t* fs_handle, const char* filename, wiced_dir_entry_details_t* details_out );
static wiced_result_t filex_usbx_shim_file_open        ( wiced_filesystem_t* fs_handle, wiced_file_t* file_handle_out, const char* filename, wiced_filesystem_open_mode_t mode );
static wiced_result_t filex_usbx_shim_file_seek        ( wiced_file_t* file_handle, int64_t offset, wiced_filesystem_seek_type_t whence );
static wiced_result_t filex_usbx_shim_file_tell        ( wiced_file_t* file_handle, uint64_t* location );
static wiced_result_t filex_usbx_shim_file_read        ( wiced_file_t* file_handle, void* data, uint64_t bytes_to_read, uint64_t* returned_bytes_count );
static wiced_result_t filex_usbx_shim_file_write       ( wiced_file_t* file_handle, const void* data, uint64_t bytes_to_write, uint64_t* written_bytes_count );
static wiced_result_t filex_usbx_shim_file_flush       ( wiced_file_t* file_handle );
static int            filex_usbx_shim_file_end_reached ( wiced_file_t* file_handle );
static wiced_result_t filex_usbx_shim_file_close       ( wiced_file_t* file_handle );
static wiced_result_t filex_usbx_shim_file_delete      ( wiced_filesystem_t* fs_handle, const char* filename );
static wiced_result_t filex_usbx_shim_dir_open         ( wiced_filesystem_t* fs_handle, wiced_dir_t* dir_handle, const char* dir_name );
static wiced_result_t filex_usbx_shim_dir_read         ( wiced_dir_t* dir_handle, char* name_buffer, unsigned int name_buffer_length, wiced_dir_entry_type_t* type, wiced_dir_entry_details_t* details );
static int            filex_usbx_shim_dir_end_reached  ( wiced_dir_t* dir_handle );
static wiced_result_t filex_usbx_shim_dir_rewind       ( wiced_dir_t* dir_handle );
static wiced_result_t filex_usbx_shim_dir_close        ( wiced_dir_t* dir_handle );
static wiced_result_t filex_usbx_shim_dir_create       ( wiced_filesystem_t* fs_handle, const char* directory_name );
static wiced_result_t filex_usbx_shim_dir_delete       ( wiced_filesystem_t* fs_handle, const char* directory_name );
static wiced_result_t filex_usbx_shim_format           ( wiced_block_device_t* device );

extern UINT  fx_media_format_oem_name_set(const UCHAR new_oem_name[8]);  /* Missing from fx_api.h */

/******************************************************
 *               Variable Definitions
 ******************************************************/

/* This is the User API driver structure for FileX */
wiced_filesystem_driver_t wiced_filesystem_driver_filex_usbx =
{
    .init             = filex_usbx_shim_init            ,
    .mount            = filex_usbx_shim_mount           ,
    .unmount          = filex_usbx_shim_unmount         ,
    .file_get_details = filex_usbx_shim_file_get_details,
    .file_open        = filex_usbx_shim_file_open       ,
    .file_seek        = filex_usbx_shim_file_seek       ,
    .file_tell        = filex_usbx_shim_file_tell       ,
    .file_read        = filex_usbx_shim_file_read       ,
    .file_write       = filex_usbx_shim_file_write      ,
    .file_flush       = filex_usbx_shim_file_flush      ,
    .file_end_reached = filex_usbx_shim_file_end_reached,
    .file_close       = filex_usbx_shim_file_close      ,
    .file_delete      = filex_usbx_shim_file_delete     ,
    .dir_open         = filex_usbx_shim_dir_open        ,
    .dir_read         = filex_usbx_shim_dir_read        ,
    .dir_end_reached  = filex_usbx_shim_dir_end_reached ,
    .dir_rewind       = filex_usbx_shim_dir_rewind      ,
    .dir_close        = filex_usbx_shim_dir_close       ,
    .dir_create       = filex_usbx_shim_dir_create      ,
    .dir_delete       = filex_usbx_shim_dir_delete      ,
    .format           = filex_usbx_shim_format          ,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

/* Initialises FileX shim - nothing to be done */
static wiced_result_t filex_usbx_shim_init             ( void )
{
    fx_system_initialize();
    return WICED_SUCCESS;
}

/* Formats a block device with a FileX filesystem */
static wiced_result_t filex_usbx_shim_format           ( wiced_block_device_t* device )
{
    /* TODOFS: To be implemented */
    UNUSED_PARAMETER(device);
    return WICED_FILESYSTEM_ERROR;
}

/* Mounts a FileX filesystem from a block device */
static wiced_result_t filex_usbx_shim_mount            ( wiced_block_device_t* device, wiced_filesystem_t* fs_handle_out )
{
    fs_handle_out->data.filex_usbx.handle = (FX_MEDIA*)device->device_specific_data;

    return WICED_SUCCESS;
}

/* Unmounts a FileX filesystem from a block device */
static wiced_result_t filex_usbx_shim_unmount          ( wiced_filesystem_t* fs_handle )
{
    fs_handle->data.filex_usbx.handle       = NULL;

    return WICED_SUCCESS;
}

/* Opens a file within a FileX filesystem */
static wiced_result_t filex_usbx_shim_file_open        ( wiced_filesystem_t* fs_handle, wiced_file_t* file_handle_out, const char* filename, wiced_filesystem_open_mode_t mode )
{
    UINT      filex_result;
    FX_MEDIA* media         = fs_handle->data.filex_usbx.handle;
    FX_FILE*  filex_handle  = &file_handle_out->data.filex;
    UINT      filex_mode    = FX_OPEN_FOR_WRITE;

    /* Match the Wiced mode to a FileX mode */
    switch ( mode )
    {
        case WICED_FILESYSTEM_OPEN_ZERO_LENGTH:
            /* Delete file if it exists to zero the length */
            filex_result = fx_file_delete( media, (CHAR *)filename );
            if ( ( filex_result != FX_SUCCESS   ) &&
                 ( filex_result != FX_NOT_FOUND ) )
            {
                return WICED_FILESYSTEM_ERROR;
            }
            /* Intentionally fall through to re-create the file with zero length */
            /* The following line disables Eclipse static analysis warning */
            /* no break */
            /* Fall-Through */

        case WICED_FILESYSTEM_OPEN_WRITE_CREATE:
        case WICED_FILESYSTEM_OPEN_APPEND_CREATE:
            /* Create file if it doesn't exist */
            filex_result = fx_file_create( media, (CHAR *)filename );
            if ( ( filex_result != FX_SUCCESS         ) &&
                 ( filex_result != FX_ALREADY_CREATED ) )
            {
                return WICED_FILESYSTEM_ERROR;
            }
            break;

        case WICED_FILESYSTEM_OPEN_FOR_READ:
            filex_mode = FX_OPEN_FOR_READ;
            break;

        case WICED_FILESYSTEM_OPEN_FOR_WRITE:
        case WICED_FILESYSTEM_OPEN_APPEND:
            /* Do Nothing */
            break;

        default:
            /* Unknown mode */
            return WICED_BADARG;
    }

    /* Open the file */
    filex_result = fx_file_open( media, filex_handle, (CHAR *)filename, filex_mode);
    if ( filex_result != FX_SUCCESS )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    /* If appending was requested , move to the end of the file */
    if ( ( mode == WICED_FILESYSTEM_OPEN_APPEND ) ||
         ( mode == WICED_FILESYSTEM_OPEN_APPEND_CREATE ) )
    {
        /* Seek to end of the file */
        filex_result = fx_file_relative_seek( filex_handle, 0, FX_SEEK_END );
        if ( filex_result != FX_SUCCESS )
        {
            fx_file_close( filex_handle );
            return WICED_FILESYSTEM_ERROR;
        }
    }
    return WICED_SUCCESS;
}

/* Get details of a file within a FileX filesystem */
static wiced_result_t filex_usbx_shim_file_get_details ( wiced_filesystem_t* fs_handle, const char* filename, wiced_dir_entry_details_t* details_out )
{
    UINT      filex_result;
    FX_MEDIA* media         = fs_handle->data.filex_usbx.handle;
    UINT      attributes, year, month, day, hour, minute, second;
    ULONG     size;

    filex_result = fx_directory_information_get( media, (CHAR *)filename, &attributes, &size, &year, &month, &day, &hour, &minute, &second );
    if ( filex_result != FX_SUCCESS )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    /* Fill in the details structure */
    details_out->size                  = size;
    details_out->attributes_available  = attributes;
    /* TODO */
    // details_out->date_time             = ??? conversion needed;
    details_out->date_time_available   = WICED_FALSE;
    details_out->attributes_available  = WICED_TRUE;
    details_out->permissions_available = WICED_FALSE;

    return WICED_SUCCESS;
}

/* Close a file within a FileX filesystem */
static wiced_result_t filex_usbx_shim_file_close       ( wiced_file_t* file_handle )
{
    FX_FILE*  filex_handle  = &file_handle->data.filex;
    FX_MEDIA* media         = file_handle->filesystem->data.filex_usbx.handle;
    UINT      filex_result;

    filex_result = fx_file_close( filex_handle );
    if ( filex_result != FX_SUCCESS )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    /* At fx_file_close it update the directory entry but changed sector(s)
     * could be cached by FileX and not updated to the physical media.
     * As FileX does not provide API to flush a file, flush media is called here
     * to avoid file corruption.
     */
    filex_result = fx_media_flush( media );
    if ( filex_result != FX_SUCCESS )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    return WICED_SUCCESS;
}

/* Delete a file within a FileX filesystem */
static wiced_result_t filex_usbx_shim_file_delete      ( wiced_filesystem_t* fs_handle, const char* filename )
{
    FX_MEDIA* media  = fs_handle->data.filex_usbx.handle;
    UINT      filex_result;

    filex_result = fx_file_delete( media, (CHAR *)filename );
    if ( filex_result != FX_SUCCESS )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    return WICED_SUCCESS;
}

/* Seek to a location in an open file within a FileX filesystem */
static wiced_result_t filex_usbx_shim_file_seek        ( wiced_file_t* file_handle, int64_t offset, wiced_filesystem_seek_type_t whence )
{
    FX_FILE*  filex_handle  = &file_handle->data.filex;
    UINT filex_result;
    UINT filex_whence;

    /* Match Wiced whence to a FileX equivalent */
    switch ( whence )
    {
        case WICED_FILESYSTEM_SEEK_SET:
            filex_whence = FX_SEEK_BEGIN;
            break;

        case WICED_FILESYSTEM_SEEK_CUR:
            if ( offset < 0 )
            {
                filex_whence = FX_SEEK_BACK;
            }
            else
            {
                filex_whence = FX_SEEK_FORWARD;
            }
            break;

        case WICED_FILESYSTEM_SEEK_END:
            filex_whence = FX_SEEK_END;
            break;

        default:
            return WICED_BADARG;
    }

    /* Perform the seek */
    filex_result = fx_file_relative_seek( filex_handle, (ULONG) offset, filex_whence );
    if ( filex_result != FX_SUCCESS )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    return WICED_SUCCESS;
}

/* Get the current location in an open file within a FileX filesystem */
static wiced_result_t filex_usbx_shim_file_tell        ( wiced_file_t* file_handle, uint64_t* location )
{
    FX_FILE*  filex_handle  = &file_handle->data.filex;

    *location = filex_handle->fx_file_current_file_offset;

    return WICED_SUCCESS;
}

/* Read data from an open file within a FileX filesystem */
static wiced_result_t filex_usbx_shim_file_read        ( wiced_file_t* file_handle, void* data, uint64_t bytes_to_read, uint64_t* returned_bytes_count )
{
    FX_FILE*  filex_handle  = &file_handle->data.filex;
    UINT filex_result;
    ULONG bytes_read;

    *returned_bytes_count = 0;

    filex_result = fx_file_read( filex_handle, data, (ULONG) bytes_to_read, &bytes_read );
    if ( filex_result != FX_SUCCESS )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    *returned_bytes_count = bytes_read;

    return WICED_SUCCESS;
}

/* Write data to an open file within a FileX filesystem */
static wiced_result_t filex_usbx_shim_file_write       ( wiced_file_t* file_handle, const void* data, uint64_t bytes_to_write, uint64_t* written_bytes_count )
{
    FX_FILE*  filex_handle  = &file_handle->data.filex;
    UINT filex_result;

    *written_bytes_count = 0;
    filex_result = fx_file_write( filex_handle, (VOID *)data, (ULONG) bytes_to_write );
    if ( filex_result != FX_SUCCESS )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    *written_bytes_count = bytes_to_write;

    return WICED_SUCCESS;
}

/* Flush unwritten data in an open file within a FileX filesystem */
static wiced_result_t filex_usbx_shim_file_flush       ( wiced_file_t* file_handle )
{
    UINT      filex_result;
    FX_MEDIA* media        = file_handle->filesystem->data.filex_usbx.handle;

    filex_result = fx_media_flush( media );
    if ( filex_result != FX_SUCCESS )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    return WICED_SUCCESS;

}

/* Get end-of-file (EOF) flag for an open file within a FileX filesystem */
static int            filex_usbx_shim_file_end_reached ( wiced_file_t* file_handle )
{
    FX_FILE*  filex_handle  = &file_handle->data.filex;

    return ( filex_handle->fx_file_current_file_offset == filex_handle->fx_file_current_file_size ) ? 1 : 0;
}

/* Opens a directory within a FileX filesystem */
static wiced_result_t filex_usbx_shim_dir_open         ( wiced_filesystem_t* fs_handle, wiced_dir_t* dir_handle, const char* dir_name )
{
    FX_MEDIA* media = fs_handle->data.filex_usbx.handle;
    UINT filex_result;

    /* TODO : this will affact the path when opening files !!! */
    filex_result = fx_directory_default_set( media, (CHAR *)dir_name );
    if ( filex_result != FX_SUCCESS )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    media->fx_media_default_path.fx_path_current_entry = 0;

    /* Save the path details in the directory handle */
    memcpy( &dir_handle->data.filex.path, &media->fx_media_default_path, sizeof(FX_PATH) );

    dir_handle->data.filex.eodir    = WICED_FALSE;

    return WICED_SUCCESS;
}

/* Reads directory entry from an open within a FileX filesystem */
static wiced_result_t filex_usbx_shim_dir_read         ( wiced_dir_t* dir_handle, char* name_buffer, unsigned int name_buffer_length, wiced_dir_entry_type_t* type, wiced_dir_entry_details_t* details )
{
    FX_MEDIA* media = dir_handle->filesystem->data.filex_usbx.handle;
    UINT      filex_result;
    CHAR      current_name[FX_MAX_LONG_NAME_LEN];  /* Temporary buffer needed as FX_MAX_LONG_NAME_LEN bytes of space are mandatory */
    UINT      attributes, year, month, day, hour, minute, second;
    ULONG     size;
    UINT      filename_len;

    /* FileX has a different function for the first entry and subsequent entries */

    /* Copy curent location from directory handle */
    memcpy( &media->fx_media_default_path, &dir_handle->data.filex.path, sizeof(FX_PATH) );

    if ( dir_handle->data.filex.path.fx_path_current_entry == 0 )
    {
        filex_result = fx_directory_first_full_entry_find( media, current_name, &attributes, &size, &year, &month, &day, &hour, &minute, &second );
    }
    else
    {
        filex_result = fx_directory_next_full_entry_find( media, current_name, &attributes, &size, &year, &month, &day, &hour, &minute, &second );
    }

    /* Update curent location in directory handle */
    memcpy( &dir_handle->data.filex.path, &media->fx_media_default_path, sizeof(FX_PATH) );


    /* Check errors */
    if ( filex_result == FX_NO_MORE_ENTRIES )
    {
        dir_handle->data.filex.eodir = WICED_TRUE;
        return WICED_FILESYSTEM_END_OF_RESOURCE;
    }

    if ( filex_result != FX_SUCCESS )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    /* Copy the filename from the temporary buffer */
    current_name[ FX_MAX_LONG_NAME_LEN-1 ] = '\x00';

    filename_len = strnlen(current_name, sizeof(current_name) ) + 1;

    strlcpy( name_buffer, current_name, MIN( name_buffer_length, filename_len));

    if ( filename_len >= name_buffer_length )
    {
        return WICED_FILESYSTEM_FILENAME_BUFFER_TOO_SMALL;
    }

    /* Fill in the details of the directory entry */

    *type                          =  ( attributes & FX_DIRECTORY ) ? WICED_FILESYSTEM_DIR : WICED_FILESYSTEM_FILE;

    details->attributes            = (wiced_filesystem_attribute_type_t) attributes;
//    TODO: need a conversion here
//    details->date_time             = year , month,  day, hour, minute, second
    details->size                  = size;
    details->attributes_available  = WICED_TRUE;
    details->date_time_available   = WICED_FALSE;
    details->permissions_available = WICED_FALSE;

    return WICED_SUCCESS;
}

/* Get end-of-directory flag for an open directory within a FileX filesystem */
static int            filex_usbx_shim_dir_end_reached  ( wiced_dir_t* dir_handle )
{
    return dir_handle->data.filex.eodir;
}

/* Moves the current location within a directory back to the first entry within a FileX filesystem */
static wiced_result_t filex_usbx_shim_dir_rewind       ( wiced_dir_t* dir_handle )
{
    dir_handle->data.filex.path.fx_path_current_entry = 0;

    return WICED_SUCCESS;
}

/* Closes an open directory within a FileX filesystem */
static wiced_result_t filex_usbx_shim_dir_close        ( wiced_dir_t* dir_handle )
{
    FX_MEDIA* media         = dir_handle->filesystem->data.filex_usbx.handle;
    UINT      filex_result;

    filex_result = fx_directory_default_set( media, FX_NULL );
    if ( filex_result != FX_SUCCESS )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    return WICED_SUCCESS;
}

/* Creates a new directory within a FileX filesystem */
static wiced_result_t filex_usbx_shim_dir_create       ( wiced_filesystem_t* fs_handle, const char* directory_name )
{
    FX_MEDIA* media         = fs_handle->data.filex_usbx.handle;
    UINT      filex_result;

    /* Test if write directory exists */
    filex_result = fx_directory_name_test( media, (CHAR *)directory_name );
    if ( filex_result == FX_SUCCESS )
    {
        /* Directory already exists - exit */
        return WICED_SUCCESS;
    }
    if ( filex_result != FX_NOT_FOUND )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    if ( ( strcmp( directory_name, "/" ) == 0 ) ||
         ( strcmp( directory_name, "\\" ) == 0 ) ||
         ( directory_name[0] ==  '\x00' ) )
    {
        /* root directory does not need creation */
        return WICED_SUCCESS;
    }


    /* Directory is not found, create it */
    filex_result = fx_directory_create( media, (CHAR *)directory_name );
    if ( filex_result != FX_SUCCESS )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    fx_media_flush( media );

    return WICED_SUCCESS;
}

static wiced_result_t filex_usbx_shim_dir_delete       ( wiced_filesystem_t* fs_handle, const char* directory_name )
{
    FX_MEDIA* media  = fs_handle->data.filex_usbx.handle;
    UINT      filex_result;

    filex_result = fx_directory_delete( media, (CHAR *)directory_name );
    if ( filex_result != FX_SUCCESS )
    {
        return WICED_FILESYSTEM_ERROR;
    }

    return WICED_SUCCESS;
}
