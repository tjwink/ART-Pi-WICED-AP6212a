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
 *  Implementation for WicedFS filesystem creation code
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <inttypes.h>
#include <sys/stat.h>
#include "wiced_filesystem.h"
#include "tester.h"

/******************************************************
 *                      Macros
 ******************************************************/
#ifndef _WIN32
/*define_style_exception_start*/
#define off64_t __off64_t
#define _stati64 stat64
/*define_style_exception_end*/
#endif /* ifndef _WIN32 */

/******************************************************
 *                    Constants
 ******************************************************/
#define WICEDFS_CREATE_BUFFER_SIZE (1024*1024)

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
static int            get_sorted_filelist ( const char* dir_name, char** list, unsigned long* item_len, unsigned long* item_count );
static int64_t        fsize               ( const char* filename );
static char           isdir               ( const char* filename );
static wiced_result_t write_dir           ( wiced_filesystem_t* out_filesystem, const char* pc_dir_name, const char* wiced_dir_name );

/******************************************************
 *               Variable Definitions
 ******************************************************/
static char buffer[WICEDFS_CREATE_BUFFER_SIZE];

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t create_wiced_filesystem( wiced_block_device_t* device, wiced_filesystem_handle_type_t fs_type, const char* dir_name )
{
    wiced_filesystem_t fs_handle;
    wiced_result_t     result;

    /* Format the filesystem */
    result = wiced_filesystem_format( device, fs_type );
    if ( result != WICED_SUCCESS )
    {
        printf("Error: Failed to format filesystem\n");
        return result;
    }

    result = wiced_filesystem_mount( device, fs_type, &fs_handle, dir_name );
    if ( result != WICED_SUCCESS )
    {
        printf("Error: Failed to mount filesystem\n");
        return result;
    }

    /* Recursively write the supplied directory path into the WicedFS image */
    result = write_dir( &fs_handle, dir_name, "" );
    if ( result != WICED_SUCCESS )
    {
        printf("Error: Failed to write directory to filesystem\n");
        wiced_filesystem_unmount( &fs_handle );
        return result;
    }

    result = wiced_filesystem_unmount( &fs_handle );
    if ( result != WICED_SUCCESS )
    {
        printf("Error: Failed to unmount filesystem\n");
        return result;
    }

    return result;
}




/******************************************************
 *               Static Function Definitions
 ******************************************************/


/**
 * Write the specified PC directory into the supplied WicedFS image file
 *
 * Recursively reads the specified host PC directory and writes the files
 * and subdirectories into the supplied WicedFS file handle.
 *
 * @param[in] output_handle : File handle of the WicedFS image
 * @param[in] dir_name      : Directory path on Host PC
 *
 * @return 0 = success
 */
static wiced_result_t write_dir( wiced_filesystem_t* out_filesystem, const char* pc_dir_name, const char* wiced_dir_name )
{
    unsigned long  item_len;
    unsigned long  item_count;
    char*          list;
    unsigned long  i;
    wiced_result_t result;
    char*          slashed_wiced_dir_name;

    slashed_wiced_dir_name    = (char*) malloc( strlen(wiced_dir_name) + 2 );
    strcpy( slashed_wiced_dir_name, wiced_dir_name );
    strcat( slashed_wiced_dir_name, "/" );
//    if ( strcmp( wiced_dir_name, "" ) == 0 )
//    {
//        wiced_dir_name = "/";
//    }


    /* Get a sorted list of the contents of the given directory pathname */
    get_sorted_filelist( pc_dir_name, &list, &item_len, &item_count );

    /* Create the directory */
    result = wiced_filesystem_dir_create( out_filesystem, slashed_wiced_dir_name );
    if ( result != WICED_SUCCESS )
    {
        printf("Error: Failed to create directory %s in filesystem\n", wiced_dir_name );
        free( slashed_wiced_dir_name );
        free( list );
        return result;
    }

    /* Loop for each item in the supplied directory path */
    for ( i = 0; i < item_count; i++ )
    {
        char                   path_is_dir;
        char *                 full_pc_path;
        char *                 full_wiced_path;

        /* Allocate space for strings containing the full path on the host PC / Wiced to this item */
        full_pc_path    = (char*) malloc( strlen(pc_dir_name)    + strlen( &list[item_len*i] ) + 2 );
        full_wiced_path = (char*) malloc( strlen(wiced_dir_name) + strlen( &list[item_len*i] ) + 2 );

        /* Make a string of the full path on the host PC to this item */
        strcpy( full_pc_path, pc_dir_name );
        strcat( full_pc_path, "/" );
        strcat( full_pc_path, &list[item_len*i] );

        /* Make a string of the full path on Wiced to this item */
        strcpy( full_wiced_path, wiced_dir_name );
        strcat( full_wiced_path, "/" );
        strcat( full_wiced_path, &list[item_len*i] );

        /* Check whether the item is a directory */
        path_is_dir = isdir( full_pc_path );

        if ( path_is_dir )
        {
            /* Item is a directory - the content data is another directory header
             * Recurse into the subdirectory and write it to the image file.
             */
            result = write_dir( out_filesystem, full_pc_path, full_wiced_path );
            if ( result != WICED_SUCCESS )
            {
                printf("Error: Failed to write directory %s to filesystem\n", full_wiced_path);
                free( full_pc_path );
                free( full_wiced_path );
                free( slashed_wiced_dir_name );
                free( list );
                return result;
            }
        }
        else
        {
            /* Item is a file */
            uint32_t bytes_read;
            FILE*    input_handle;
            wiced_file_t file_handle;

            /* Open the file */
            input_handle = fopen( full_pc_path, "rb" );
            if( input_handle == NULL )
            {
                printf( "Couldn't open input file %s\n", full_pc_path );
                free( full_pc_path );
                free( full_wiced_path );
                free( slashed_wiced_dir_name );
                free( list );
                return WICED_ERROR;
            }


            result = wiced_filesystem_file_open( out_filesystem, &file_handle, full_wiced_path, WICED_FILESYSTEM_OPEN_WRITE_CREATE );
            if ( result != WICED_SUCCESS )
            {
                printf( "Error opening filesystem file %s for writing\n", full_wiced_path );
                free( full_pc_path );
                free( full_wiced_path );
                free( slashed_wiced_dir_name );
                free( list );
                return result;
            }

            /* Read the file content in chunks until end-of-file is reached */
            while ( ( bytes_read = (uint32_t) fread( buffer, 1, sizeof(buffer), input_handle ) ) != 0 )
            {
                /* Write the chunk to the image file */
                uint64_t written_bytes_count;
                result = wiced_filesystem_file_write( &file_handle, buffer, bytes_read, &written_bytes_count );
                if ( result != WICED_SUCCESS )
                {
                    printf( "Error writing to filesystem file %s\n", full_wiced_path );
                    free( full_pc_path );
                    free( full_wiced_path );
                    fclose( input_handle );
                    free( slashed_wiced_dir_name );
                    free( list );
                    return result;
                }
            }

            result = wiced_filesystem_file_close( &file_handle );
            if ( result != WICED_SUCCESS )
            {
                printf( "Error closing filesystem file %s\n", full_wiced_path );
                free( full_pc_path );
                free( full_wiced_path );
                free( slashed_wiced_dir_name );
                free( list );
                return result;
            }

            fclose( input_handle );
        }

        free( full_pc_path );
        free( full_wiced_path );

    }

    free( slashed_wiced_dir_name );
    free( list );

    return WICED_SUCCESS;
}


/**
 * Gets a sorted list of files contained in the specified host PC directory
 *
 * @note: the list returned is allocated with malloc() and must be freed by the caller
 *
 * @param[in]  dir_name   : The path to the directory to read
 * @param[out] list       : Receives the sorted file list - Malloced - must be freed by caller
 * @param[out] item_len   : Receives the maximum string length in the list (including terminating null)
 * @param[out] item_count : Receives the number of items in the list
 *
 * @return 0 = success
 */
static int get_sorted_filelist( const char* dir_name, char** list, unsigned long* item_len, unsigned long* item_count )
{
    DIR*           dir_handle;
    struct dirent* file_desc;
    const char *   last_item = "\x00";
    char *         curr_find;
    unsigned long  file_count;
    unsigned long  max_filename_len;
    char*          unsorted;
    char*          sorted;
    unsigned long  file_num;
    unsigned long  sorted_file_num;

    /* Open the specified directory */
    dir_handle = opendir( dir_name );

    if( dir_handle == NULL )
    {
        printf( "Couldn't open directory %s\n", dir_name );
        return -1;
    }



    /* Read directory to find out how much space is required to store filenames */

    file_count = 0;
    max_filename_len = 0;
    while( ( file_desc = readdir( dir_handle ) ) != NULL )
    {
        /* Ignore "." and ".." */
        if ( ( 0 == strcmp( ".",  file_desc->d_name ) ) ||
             ( 0 == strcmp( "..", file_desc->d_name ) ) )
        {
            continue;
        }

        /* Count filenames */
        file_count++;

        /* Find longest filename */
        if ( max_filename_len < strlen( file_desc->d_name ) )
        {
            max_filename_len = strlen( file_desc->d_name );
        }
    }

    *item_len = max_filename_len+1;

    /* Allocate space for sorting filenames */

    unsorted = (char*) malloc( *item_len * file_count );
    sorted   = (char*) malloc( *item_len * file_count );


    /* Rewind to the start of the directory ready to re-read the entries */
    rewinddir( dir_handle );

    /* Read and store filenames for sorting */
    file_num = 0;
    while( ( file_desc = readdir( dir_handle ) ) != NULL )
    {
        /* Ignore "." and ".." */
        if ( ( 0 == strcmp( ".",  file_desc->d_name ) ) ||
             ( 0 == strcmp( "..", file_desc->d_name ) ) )
        {
            continue;
        }

        /* Copy the filename into the unsorted list */
        strcpy( &unsorted[*item_len * file_num], file_desc->d_name );
        file_num++;
    }


    /* Sort filenames */

    /* Cycle through the sorted list slots */
    for( sorted_file_num = 0; sorted_file_num < file_count; sorted_file_num++ )
    {
        unsigned long unsorted_file_num;

        /* Cycle through the unsorted list */
        curr_find = NULL;
        for( unsorted_file_num = 0; unsorted_file_num < file_count; unsorted_file_num++ )
        {
            /* Check whether the item is greater than the previous sorted item AND
             * either it is the first unsorted item, or is smaller than the smallest
             * unsorted item found so far...
             */
            if( ( 0 > strcmp( last_item, &unsorted[*item_len * unsorted_file_num] ) ) &&
                ( ( curr_find == NULL ) || ( 0 < strcmp( curr_find, &unsorted[*item_len * unsorted_file_num] ) ) ) )
            {
                /* Save item as it is the smallest found that is still larger than the sorted list top */
                curr_find = &unsorted[*item_len * unsorted_file_num];
            }
        }

        /* Copy the smallest unsorted item found into the next sorted slot */
        strcpy( &sorted[*item_len * sorted_file_num], curr_find );
        last_item = curr_find;
    }

    /* The unsorted list is no longer needed */
    free( unsorted );

    /* Nor is the directory handle */
    closedir( dir_handle );

    /* Return the sorted results */
    *list = sorted;
    *item_count = file_count;
    return 0;
}



/** Get the size of a file on the host PC
 *
 * @param[in] filename : the pathname of the file
 *
 * @return the file size, or negative on error
 */
static int64_t fsize( const char *filename )
{
    struct _stati64 st;

    if (_stati64(filename, &st) == 0)
    {
        return st.st_size;
    }

    return -1;
}


/** Determines if the given path is a directory
 *
 * @param[in] filename : the pathname to examine
 *
 * @return 1 = directory, 0 = file, negative = error
 */
static char isdir( const char *filename )
{
    struct stat st;

    if ( stat( filename, &st ) == 0 )
    {
        return ( S_ISDIR( st.st_mode ) ) ? 1 : 0;
    }

    return -1;
}
