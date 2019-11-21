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
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "typedefs.h"
#include "command_console.h"
#include "command_console_fs.h"
#include "wiced_filesystem.h"
#include "wiced_time.h"
#include "platform_cache.h"
#include "wiced_osl.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define LOCAL_BUFFER_SIZE   (64*1024)

#define MAX_PATH_LEN   (1024)

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
static void memdump(uint8_t* bptr, uint32_t len);
static int create_test_file ( const char* file_name, uint64_t file_size );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_bool_t       current_filesystem_is_mounted = WICED_FALSE;
static wiced_filesystem_t current_filesystem_handle;
static char               current_working_directory[MAX_PATH_LEN] = "/";

/******************************************************
 *               Function Definitions
 ******************************************************/

int mk_file (int argc, char* argv[])
{
    char* file_name;
    uint32_t file_size;
    if ( argc == 3 )
    {
        file_name = argv[1];
        file_size = atoi( argv[2] );
        if ( current_filesystem_is_mounted != WICED_TRUE )
        {
            printf( "Filesystem not currently mounted\n" );
            return ERR_UNKNOWN;
        }
        if ( create_test_file( file_name, file_size ) != 0 )
            return ERR_UNKNOWN;
    }
    else
        printf( "mkfile <file name> <file length>\n" );

    return ERR_CMD_OK;
}

static int create_test_file ( const char* file_name, uint64_t file_size )
{
    wiced_result_t     result;
    uint32_t           i;
    uint64_t           file_size_remaining;
    wiced_file_t       file_handle = {0};
    uint32_t           requested_length;
    char*              local_buffer;

    local_buffer = (char*) osl_malloc_align( (uint)LOCAL_BUFFER_SIZE, (uint)PLATFORM_L1_CACHE_SHIFT );
    if ( local_buffer == NULL )
    {
        printf( "Failed allocate temporary buffer\n" );
        return -1;
    }
    printf( "Creating test file '%s' with size %lld\n", file_name, file_size );

    /* Open file for writing.  */
    result = wiced_filesystem_file_open( &current_filesystem_handle, &file_handle, file_name, WICED_FILESYSTEM_OPEN_ZERO_LENGTH );
    if ( result != WICED_SUCCESS )
    {
        printf( "Failed to open file %s\n", file_name );
        return -2;
    }

    /* Total bytes to write */
    file_size_remaining = file_size;
    while ( file_size_remaining > 0 )
    {
        uint64_t bytes_written;
        requested_length = MIN( file_size_remaining, LOCAL_BUFFER_SIZE );

        /* Fill the memory with random data */
        for (i = 0; i < requested_length; i++)
        {
            local_buffer[i] = rand();
        }

        /* Copy the file in blocks */
        result = wiced_filesystem_file_write( &file_handle, local_buffer, requested_length, &bytes_written );
        if ( ( result != WICED_SUCCESS ) || ( requested_length != bytes_written ) )
        {
            printf( "Failed to write to file" );
            wiced_filesystem_file_close( &file_handle );
            return -3;
        }

        file_size_remaining -= requested_length;
    };

    free( local_buffer );

    /* Done */
    result = wiced_filesystem_file_close( &file_handle );
    if (result != WICED_SUCCESS )
    {
        printf( "Failed to close file" );
        return -4;
    }

    return 0;
}


int mount (int argc, char* argv[])
{

    const filesystem_list_t* curr_item = all_filesystem_devices;

    if ( argc == 2 )
    {
        if ( current_filesystem_is_mounted == WICED_TRUE )
        {
            /* Unmount existing filesystem */
            wiced_filesystem_unmount( &current_filesystem_handle );
            current_filesystem_is_mounted = WICED_FALSE;
        }

        while ( curr_item->device != NULL )
        {
            if ( strcmp ( argv[1], curr_item->name ) == 0 )
            {
                wiced_result_t result;

                /* Found requested device */
                result = wiced_filesystem_mount( curr_item->device, curr_item->type, &current_filesystem_handle, curr_item->name );
                if ( result != WICED_SUCCESS )
                {
                    printf( "Error mounting filesystem\n" );
                    return ERR_UNKNOWN;
                }
                current_filesystem_is_mounted = WICED_TRUE;
                return ERR_CMD_OK;
            }
            curr_item++;
        }

        /* Not found - print options */
        printf( "Filesystem %s not found\n", argv[1] );
    }
    printf( "mount <filesystem name>\n" );
    printf( "Valid filesystem names:\n" );
    curr_item = all_filesystem_devices;
    while ( curr_item->device != NULL )
    {
        printf( "%s\n", curr_item->name );
        curr_item++;
    }

    return ERR_CMD_OK; /* Return ok since a custom message has been printed already */
}

int unmount (int argc, char* argv[])
{
    if ( current_filesystem_is_mounted != WICED_TRUE )
    {
        printf( "Not currently mounted\n" );
        return ERR_UNKNOWN;
    }

    /* Unmount existing filesystem */
    wiced_filesystem_unmount( &current_filesystem_handle );
    current_filesystem_is_mounted = WICED_FALSE;

    return ERR_CMD_OK;
}

/* function getcwd is conflicted with POSIX getcwd. name changed to get_cwd */
int get_cwd (int argc, char* argv[])
{
    printf("%s\n", current_working_directory);

    return ERR_CMD_OK;
}

int mk_dir (int argc, char* argv[])
{
    wiced_result_t result;
    char* directory_name = argv[1];

    if ( current_filesystem_is_mounted != WICED_TRUE )
    {
        printf( "Filesystem not currently mounted\n" );
        return ERR_UNKNOWN;
    }

    result = wiced_filesystem_dir_create( &current_filesystem_handle, directory_name );

    if ( result != WICED_SUCCESS )
    {
       printf( "Unable to create %s.\n", directory_name );
       return ERR_UNKNOWN;
    }

    printf( "%s successfully created.\n", directory_name );

    return ERR_CMD_OK;
}

int rm_dir (int argc, char* argv[])
{
    wiced_result_t result;
    char* directory_name = argv[1];

    if ( current_filesystem_is_mounted != WICED_TRUE )
    {
        printf( "Filesystem not currently mounted\n" );
        return ERR_UNKNOWN;
    }

    result = wiced_filesystem_dir_delete( &current_filesystem_handle, directory_name );

    if ( result != WICED_SUCCESS )
    {
       printf( "Unable to delete %s.\n", directory_name );
       return ERR_UNKNOWN;
    }

    printf( "%s successfully deleted.\n", directory_name );

    return ERR_CMD_OK;
}

int change_dir (int argc, char* argv[])
{
    wiced_result_t result;
    wiced_dir_t    dir_handle;
    char*          new_path = argv[1];

    if ( current_filesystem_is_mounted != WICED_TRUE )
    {
        printf( "Filesystem not currently mounted\n" );
        return ERR_UNKNOWN;
    }

    /* Check path exists */
    result = wiced_filesystem_dir_open( &current_filesystem_handle, &dir_handle, new_path );
    if ( result != WICED_SUCCESS )
    {
        printf( "Cannot change to %s \n", new_path );
        return ERR_UNKNOWN;
    }

    (void) wiced_filesystem_dir_close( &dir_handle );

    if ( strlen(new_path) >= sizeof(current_working_directory) )
    {
        printf( "Path too long" );
        return ERR_UNKNOWN;
    }

    strlcpy( current_working_directory, new_path, sizeof(current_working_directory) );

    return ERR_CMD_OK;
}

int cat_file (int argc, char* argv[])
{
    wiced_result_t   result;
    uint64_t         data_length_read;
    wiced_file_t     file_handle;
    char*            file_name = argv[1];
    char*            local_buffer;

    if ( current_filesystem_is_mounted != WICED_TRUE )
    {
        printf( "Filesystem not currently mounted\n" );
        return ERR_UNKNOWN;
    }

    /* Open the source file.  */
    result = wiced_filesystem_file_open( &current_filesystem_handle, &file_handle, file_name, WICED_FILESYSTEM_OPEN_FOR_READ );
    if ( result != WICED_SUCCESS )
    {
        printf( "Failed to open file %s\n", file_name );
        return ERR_UNKNOWN;
    }

    local_buffer = (char*) malloc( LOCAL_BUFFER_SIZE );
    do
    {
        /* Read the file in blocks.  */
        result = wiced_filesystem_file_read( &file_handle, local_buffer, LOCAL_BUFFER_SIZE, &data_length_read );

        /* Dump content of the source file */
        memdump((uint8_t *) local_buffer, data_length_read);
    } while ( ( result == WICED_SUCCESS ) && ( data_length_read == LOCAL_BUFFER_SIZE ) ); /* Check if end of file.  */

    free( local_buffer );

    wiced_filesystem_file_close( &file_handle );

    return ERR_CMD_OK;
}

int cp_file (int argc, char* argv[])
{
    int              status = ERR_UNKNOWN;
    wiced_result_t   result;
    uint64_t         data_length_read = 1;
    uint64_t         data_length_write = 1;
    wiced_file_t     source_file_handle;
    wiced_file_t     target_file_handle;
    char*            source_file_name = argv[1];
    char*            target_file_name = argv[2];
    char*            local_buffer = NULL;

    if ( current_filesystem_is_mounted != WICED_TRUE )
    {
        printf( "Filesystem not currently mounted\n" );
        return ERR_UNKNOWN;
    }

    memset( &source_file_handle, 0, sizeof( wiced_file_t) );
    memset( &target_file_handle, 0, sizeof( wiced_file_t) );

    /* Open the source file.  */
    result = wiced_filesystem_file_open( &current_filesystem_handle, &source_file_handle, source_file_name, WICED_FILESYSTEM_OPEN_FOR_READ );
    if ( result != WICED_SUCCESS )
    {
        printf( "Failed to open source file %s\n", source_file_name );
        status = ERR_UNKNOWN;
        goto finish;
    }
    /* Open the target file.  */
    result = wiced_filesystem_file_open( &current_filesystem_handle, &target_file_handle, target_file_name, WICED_FILESYSTEM_OPEN_ZERO_LENGTH );
    if ( result != WICED_SUCCESS )
    {
        printf( "Failed to open target file %s\n", target_file_name );
        status = ERR_UNKNOWN;
        goto finish;
    }

    local_buffer = (char*) malloc( LOCAL_BUFFER_SIZE );

    if ( NULL == local_buffer )
    {
        printf( "Failed to allocate copy buffer.\n" );
        status = ERR_UNKNOWN;
        goto finish;
    }

    while ( data_length_read && data_length_write )
    {
        wiced_filesystem_file_read( &source_file_handle, local_buffer, LOCAL_BUFFER_SIZE, &data_length_read );

        if ( data_length_read )
        {
            wiced_filesystem_file_write( &target_file_handle, (const char *) local_buffer, data_length_read, &data_length_write );

            if ( data_length_read != data_length_write )
            {
                printf( "cp_file: try = %lu, write = %lu\n", ( uint32_t ) data_length_read, ( uint32_t ) data_length_write );
                status = ERR_UNKNOWN;
                break;
            }
            else
            {
                status = ERR_CMD_OK;
            }
        }
    }

finish:

    if ( local_buffer != NULL )
    {
        free( local_buffer );
    }

    if ( source_file_handle.driver != NULL )
    {
        wiced_filesystem_file_close( &source_file_handle );
    }

    if ( target_file_handle.driver != NULL )
    {
        wiced_filesystem_file_close( &target_file_handle );
    }

    return status;
}

int rm_file (int argc, char* argv[])
{
    wiced_result_t result;
    char* file_name = argv[1];

    if ( current_filesystem_is_mounted != WICED_TRUE )
    {
        printf( "Filesystem not currently mounted\n" );
        return ERR_UNKNOWN;
    }

    result = wiced_filesystem_file_delete( &current_filesystem_handle, file_name );

    if ( result != WICED_SUCCESS )
    {
       printf( "Unable to delete %s.\n", file_name );
       return ERR_UNKNOWN;
    }

    printf( "%s successfully deleted.\n", file_name );

    return ERR_CMD_OK;
}

int ls_dir (int argc, char* argv[])
{
    char                      current_name[MAX_PATH_LEN];
    wiced_dir_t               dir_handle;
    wiced_dir_entry_details_t entry_info;
    wiced_dir_entry_type_t    entry_type;
    wiced_result_t            result;
    uint64_t                  total_size = 0;
    uint32_t                  dir_count = 0;
    uint32_t                  file_count = 0;

    if ( current_filesystem_is_mounted != WICED_TRUE )
    {
        printf( "Filesystem not currently mounted\n" );
        return ERR_UNKNOWN;
    }

    result = wiced_filesystem_dir_open( &current_filesystem_handle, &dir_handle, current_working_directory );
    if ( result != WICED_SUCCESS )
    {
        printf( "Unable to open directory %s\n", current_working_directory );
        return ERR_UNKNOWN;
    }

    while ( result == WICED_SUCCESS )
    {
        result = wiced_filesystem_dir_read( &dir_handle, current_name, MAX_PATH_LEN, &entry_type, &entry_info );

        if ( result == WICED_FILESYSTEM_FILENAME_BUFFER_TOO_SMALL )
        {
            /* Truncation is ok */
            result = WICED_SUCCESS;
        }

        if ( result == WICED_SUCCESS )
        {
            total_size += entry_info.size;
            if ( entry_type == WICED_FILESYSTEM_DIR )
            {
                dir_count++;
            }
            else if ( entry_type == WICED_FILESYSTEM_FILE )
            {
                file_count++;
            }

            if ( entry_info.attributes_available == WICED_TRUE )
            {
                printf("%c%c%c%c%c%c\t",
                       (entry_info.attributes & WICED_FILESYSTEM_ATTRIBUTE_ARCHIVE)   ? 'A' : '-',
                       (entry_info.attributes & WICED_FILESYSTEM_ATTRIBUTE_DIRECTORY) ? 'D' : '-',
                       (entry_info.attributes & WICED_FILESYSTEM_ATTRIBUTE_VOLUME)    ? 'V' : '-',
                       (entry_info.attributes & WICED_FILESYSTEM_ATTRIBUTE_SYSTEM)    ? 'S' : '-',
                       (entry_info.attributes & WICED_FILESYSTEM_ATTRIBUTE_HIDDEN)    ? 'H' : '-',
                       (entry_info.attributes & WICED_FILESYSTEM_ATTRIBUTE_READ_ONLY) ? 'R' : '-');
            }

            printf("%10llu\t", entry_info.size);

            if ( entry_info.date_time_available == WICED_TRUE )
            {
                wiced_iso8601_time_t iso8601_time;
                wiced_time_convert_utc_ms_to_iso8601( entry_info.date_time * 1000, &iso8601_time );
                printf("%27s\t", (char*)&iso8601_time );
            }
            printf("%s\n", current_name);
        }
    }

    result = wiced_filesystem_dir_close( &dir_handle );

    printf( "\n%ld Dir(s), %ld File(s)\n", dir_count, file_count );
    printf( "\n" );

    return ERR_CMD_OK;
}

static void memdump(uint8_t* bptr, uint32_t len)
{
    uint32_t i, j;

    for ( i = 0 ; i < len; i+=16 )
    {
        uint8_t* bptr_char = bptr;

        /* Print first byte address in line */
        printf( "0x%08X: ", (unsigned int)bptr );

        /* Print line data in hex */
        for ( j = 0; j < 16; j++ )
        {
            printf( "%02X ", *bptr++ );
        }

        /* print line data as chars */
        for ( j = 0; j < 16; j++ )
        {
            char chChar = *bptr_char++;

            /* Print only text characters */
            if ( (chChar < ' ') || (chChar > 'z') )
            {
                chChar = '.';
            }

            printf( "%c", chChar );
        }

        /* Next line data */
        printf( "\n" );
    }
}
