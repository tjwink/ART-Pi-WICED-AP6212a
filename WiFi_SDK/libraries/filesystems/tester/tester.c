
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
 *  Tester tool for Wiced filesystem
 *
 */




#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <errno.h>

#include "tester.h"
#include "platform_assert.h"
#include "wiced_time.h"
#include "wiced_utilities.h"
#include "wiced_filesystem.h"

#include "wicedfs_create.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define COMPARE_BUFFER_SIZE     (1024*1024)
#define IMAGE_FILENAME          "image.bin"
#define PATH_BUFFER_SIZE        (256)
#define DIRECTORY_SEPARATOR_STR "/"
#define ROOT_DIRECTORY          "/"
#define DIRECTORY_SEPARATOR_CHR '/'

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
static int             cmp_fn                ( const void * a, const void * b );
static int             get_sorted            ( const char* dir_name, char *** ptr_list, unsigned long* count );
static void            free_list             ( char*** ptr_list, unsigned long count );
static int             test_compare_dir      ( wiced_filesystem_t* fs_handle, const char* wdir_name, const char* dir_name );

/******************************************************
 *               Variable Definitions
 ******************************************************/
static char buffer1[COMPARE_BUFFER_SIZE];
static char buffer2[COMPARE_BUFFER_SIZE];

const wiced_block_device_driver_t tester_block_device_driver;

tester_block_device_specific_data_t tester_block_device_specific_data =
{
    .filename = IMAGE_FILENAME,
};

const wiced_block_device_init_data_t tester_block_device_init_data =
{
    .base_address_offset = 0,
    .maximum_size = 0,
    .volatile_and_requires_format_when_mounting = 0,
};

wiced_block_device_t tester_block_device =
{
        .init_data = &tester_block_device_init_data,
        .driver = &tester_block_device_driver,
        .device_specific_data = &tester_block_device_specific_data,
};

/******************************************************
 *               Function Definitions
 ******************************************************/

int main (int argc, const char * argv[])
{
    wiced_filesystem_t fs_handle;
    wiced_result_t result;

    /* Check command-line argument */
    if ( argc != 2 )
    {
        printf( "Usage: tester <source_dir>\n\n" );
        return -1;
    }

    /* TEST WicedFS */
    printf( "Testing WicedFS\n\n");

    /* Create the WicedFS image file from the specified directory */
    if ( 0 != create_wicedfs( IMAGE_FILENAME, argv[1] ) )
    {
        printf( "Error creating file system image file %s\n", IMAGE_FILENAME );
        return -2;
    }

    result = wiced_filesystem_init();
    if ( result != WICED_SUCCESS  )
    {
        printf( "Error initing filesystems\n" );
        return -3;
    }

    result = wiced_filesystem_mount ( &tester_block_device, WICED_FILESYSTEM_HANDLE_WICEDFS, &fs_handle, "TestWicedFS" );
    if ( result != WICED_SUCCESS )
    {
        printf( "Error mounting filesystem\n" );
        return -4;
    }


    /* Compare the Wiced filesystem to the Host PC directory */
    if ( 0 != test_compare_dir( &fs_handle, ROOT_DIRECTORY, argv[1] ) )
    {
        return -10;
    }

    printf( "Success!\n");

    /* Clean up */
    result = wiced_filesystem_unmount( &fs_handle );
    if ( result != WICED_SUCCESS )
    {
        printf( "Error unmounting filesystem\n" );
        return -5;
    }


    /* TEST FATFS */
    printf( "\n\nTesting FATFS\n\n");

    /* Create the Wiced filesystem from the specified directory */
    result = create_wiced_filesystem( &tester_block_device, WICED_FILESYSTEM_HANDLE_FATFS, argv[1] );
    if ( result != WICED_SUCCESS )
    {
        printf( "Error creating file system image file for FATFS\n" );
        return -2;
    }

    if ( WICED_SUCCESS != wiced_filesystem_init() )
    {
        printf( "Error initing filesystems\n" );
        return -3;
    }

    if ( WICED_SUCCESS != wiced_filesystem_mount ( &tester_block_device, WICED_FILESYSTEM_HANDLE_FATFS, &fs_handle, "TestFATFS" ) )
    {
        printf( "Error mounting filesystem\n" );
        return -4;
    }


    /* Compare the Wiced filesystem to the Host PC directory */
    if ( 0 != test_compare_dir( &fs_handle, ROOT_DIRECTORY, argv[1] ) )
    {
        return -11;
    }

    printf( "Success!\n");

    /* Clean up */
    if ( WICED_SUCCESS != wiced_filesystem_unmount( &fs_handle ) )
    {
        printf( "Error unmounting filesystem\n" );
        return -5;
    }


    /* TEST FileX */
    printf( "\n\nTesting FileX\n\n");

    /* Create the Wiced filesystem from the specified directory */
    result = create_wiced_filesystem( &tester_block_device, WICED_FILESYSTEM_HANDLE_FILEX, argv[1] );
    if ( result != WICED_SUCCESS )
    {
        printf( "Error creating file system image file for FileX\n" );
        return -2;
    }

    if ( WICED_SUCCESS != wiced_filesystem_init() )
    {
        printf( "Error initing filesystems\n" );
        return -3;
    }

    if ( WICED_SUCCESS != wiced_filesystem_mount ( &tester_block_device, WICED_FILESYSTEM_HANDLE_FILEX, &fs_handle, "TestFileX" ) )
    {
        printf( "Error mounting filesystem\n" );
        return -4;
    }


    /* Compare the Wiced filesystem to the Host PC directory */
    if ( 0 != test_compare_dir( &fs_handle, ROOT_DIRECTORY, argv[1] ) )
    {
        return -12;
    }

    printf( "Success!\n");

    /* Clean up */
    if ( WICED_SUCCESS != wiced_filesystem_unmount( &fs_handle ) )
    {
        printf( "Error unmounting filesystem\n" );
        return -5;
    }

    return 0;

}


/******************************************************
 *               Static Function Definitions
 ******************************************************/

/** String comparison function to be used with qsort
 *
 * @param[in] a : first string
 * @param[in] b : second string
 *
 * @return
 */
static int cmp_fn( const void * a, const void * b )
{
    return strcmp( *((char **)a), *((char**) b) );
}


/** Get a sorted list of the content of a directory
 *
 * @note : Both the items in the list, and the list
 *         itself are allocated using malloc( ) and
 *         must be freed by the caller to avoid memory leakage
 *
 * @param[in]  dir_name : Path of the directory
 * @param[out] ptr_list : Receives the list of item names
 * @param[out] count    : Receives the number of items in the list
 */
static int get_sorted( const char* dir_name, char *** ptr_list, unsigned long* count )
{
    DIR* dir;
    struct dirent * entry;
    unsigned long entry_num;

    /* Open the specified directory for reading */
    if ( NULL == ( dir = opendir( dir_name ) ) )
    {
        printf( "Error opening disk dir %s\n", dir_name );
        return -2;
    }


    /* Count the number of items in the directory */
    *count = 0;
    while ( ( entry = readdir( dir ) ) != NULL )
    {
        (*count)++;
    }

    /* Rewind the directory ready for re-reading */
    rewinddir( dir );

    /* Allocate space for the list pointers */
    *ptr_list = (char**) malloc( (*count) * sizeof(char*) );

    /* Read the directory items into the list*/
    entry_num = 0;
    while ( ( entry = readdir( dir ) ) != NULL )
    {
        /* Allocate space in the list for this item name */
        (*ptr_list)[entry_num] = (char*) malloc( strlen( entry->d_name ) + 1 );

        /* Copy the item name into the list */
        strcpy( (*ptr_list)[entry_num], entry->d_name );
        entry_num++;
    }

    /* Sort the list */
    qsort( *ptr_list, (*count), sizeof(char*), cmp_fn );

    /* Finished */
    closedir( dir );

    return 0;
}


/** Free a list created with get_sorted( )
 *
 * @param[in] ptr_list : a pointer to the list
 * @param[in] count    : the number of items in the list
 */
static void free_list( char*** ptr_list, unsigned long count )
{
    unsigned long i;
    /* Free each item in the list */
    for( i = 0; i < count; i++ )
    {
        free( (*ptr_list)[i] );
    }
    /* Free the list pointers */
    free( *ptr_list );
}

/** Compare a Wiced filesystem against a Host PC directory
 *
 * Recursively compares the contents of the Wiced to a Host PC directory
 *
 * @param[in] fs_handle : Filesystem handle obtained from wiced_filesystem_mount
 * @param[in] wdir_name : Directory path in the Wiced filesystem
 * @param[in] dir_name  : Directory path on the Host PC
 *
 * @return 0 = Directories match exactly
 */
static int test_compare_dir( wiced_filesystem_t* fs_handle, const char* wdir_name, const char* dir_name )
{
    wiced_dir_t   wdir;
    char          namebuf[PATH_BUFFER_SIZE];
    char **       sorted_ptr_list;
    unsigned long sorted_count;
    unsigned long entry_num;

    /* Open the Wiced directory */
    if ( WICED_SUCCESS != wiced_filesystem_dir_open( fs_handle, &wdir, wdir_name ) )
    {
        printf( "Error opening wiced dir %s\n", wdir_name );
        return -1;
    }

    /* Open the Wiced directory */
    if ( WICED_SUCCESS != wiced_filesystem_dir_rewind( &wdir ) )
    {
        printf( "Error rewinding wiced dir %s\n", wdir_name );
        return -1;
    }

    printf("opened dir %s\n", wdir_name );

    /* Open the local Host PC directory and get a sorted list of items */
    if ( 0 != get_sorted( dir_name, &sorted_ptr_list, &sorted_count ))
    {
        printf( "Error sorting disk dir %s\n", dir_name );
        wiced_filesystem_dir_close( &wdir );
        return -2;
    }

    /* Loop for each local Host PC item */
    entry_num = 0;
    while ( entry_num < sorted_count )
    {
        FILE*                     file;
        char                      path[PATH_BUFFER_SIZE];
        char                      wpath[PATH_BUFFER_SIZE];
        struct                    stat st;
        wiced_dir_entry_type_t    entry_type;
        wiced_dir_entry_details_t details;
        wiced_result_t result;

        /* Ignore "." and ".." */
        if ( ( 0 == strcmp( sorted_ptr_list[entry_num], ".." ) ) ||
             ( 0 == strcmp( sorted_ptr_list[entry_num], "." ) ) )
        {
            entry_num++;
            continue;
        }

        /* Start constructing the full host path of this item - Copy the base directory */
        strcpy( path, dir_name );

        /* If the base directory does not have a separator, add one */
        if ( dir_name[ strlen(dir_name) - 1 ] != DIRECTORY_SEPARATOR_CHR )
        {
            strcat( path, DIRECTORY_SEPARATOR_STR );
        }

        /* Add the item name to complete the path */
        strcat( path, sorted_ptr_list[entry_num] );

        /* Start constructing the full Wiced path of this item - Copy the base directory */
        strcpy( wpath, wdir_name );

        /* If the base directory does not have a separator, add one */
        if ( wdir_name[ strlen(wdir_name) - 1 ] != DIRECTORY_SEPARATOR_CHR )
        {
            strcat( wpath, DIRECTORY_SEPARATOR_STR );
        }

        /* Add the item name to complete the path */
        strcat( wpath, sorted_ptr_list[entry_num] );

        /* Check if the host PC item is a directory */
        if ( ( stat( path, &st ) == 0) &&
             ( S_ISDIR(st.st_mode ) ) )
        {
            /* Item is a sub-directory - Recurse into it. */
            int retval;
            printf( "recursing to %s , %s\n", wpath, path );
            retval = test_compare_dir( fs_handle, wpath, path );
            if ( retval != 0 )
            {
                printf("failure in directory %s\n", wpath );
                free_list( &sorted_ptr_list, sorted_count );
                wiced_filesystem_dir_close( &wdir );
                return retval;
            }

            entry_num++;
            continue;
        }

        /* Read the next item name from the Wiced directory */
        do
        {
            result = wiced_filesystem_dir_read( &wdir, namebuf, sizeof(namebuf), &entry_type, &details );
        } while ( ( result == WICED_SUCCESS ) &&
                  ( entry_type == WICED_FILESYSTEM_DIR ) );  /* Skip directories since they may be out of order and are handled above */

        if ( result != WICED_SUCCESS )
        {
            printf( "Error reading wiced directory\n" );
            free_list( &sorted_ptr_list, sorted_count );
            wiced_filesystem_dir_close( &wdir );
            return -3;
        }


        /* Compare the item name against the one from the Host PC directory */
        if ( 0 != strcmp( namebuf, sorted_ptr_list[entry_num] ) )
        {
            printf( "Error names do not match: wiced \"%s\", disk \"%s\"\n", namebuf, sorted_ptr_list[entry_num] );
            free_list( &sorted_ptr_list, sorted_count );
            wiced_filesystem_dir_close( &wdir );
            return -4;
        }

        /* Check if the item is a file */
        if ( (stat(path, &st) == 0) &&
             (! S_ISDIR(st.st_mode) ) )
        {
            /* Item is a file - compare the content */
            size_t         num_read;
            wiced_file_t bfile;

            /* Open the Host PC file for reading */
            if ( NULL == ( file = fopen ( path, "rb" ) ) )
            {
                printf( "Error opening disk file %s , %s\n", path, strerror( errno ) );
                free_list( &sorted_ptr_list, sorted_count );
                wiced_filesystem_dir_close( &wdir );
                return -5;
            }

            /* Open the Wiced for reading */
            if ( WICED_SUCCESS != wiced_filesystem_file_open( fs_handle, &bfile, wpath, WICED_FILESYSTEM_OPEN_FOR_READ ) )
            {
                printf( "Error opening wiced file %s\n", wpath );
                free_list( &sorted_ptr_list, sorted_count );
                wiced_filesystem_dir_close( &wdir );
                fclose(file);
                return -5;
            }

            /* Read chunks from Host PC file until end-of-file reached */
            while ( ( num_read = fread( buffer1, 1, sizeof(buffer1), file ) ) != 0 )
            {
                uint64_t bnum_read;

                /* Read a chunk from the Wiced file */
                if ( WICED_SUCCESS != wiced_filesystem_file_read ( &bfile, buffer2, sizeof(buffer2), &bnum_read ) )
                {
                    printf( "Error: error reading\n" );
                    free_list( &sorted_ptr_list, sorted_count );
                    wiced_filesystem_dir_close( &wdir );
                    return -23;
                }

                /* Check that the number of bytes read matches */
                if ( bnum_read != num_read )
                {
                    printf( "Error: num read mismatch %lu %lu\n", (unsigned long)num_read, (unsigned long)bnum_read );
                    free_list( &sorted_ptr_list, sorted_count );
                    wiced_filesystem_dir_close( &wdir );
                    return -6;
                }

                /* Check that the data matches */
                if ( 0 != memcmp( buffer1, buffer2, num_read ) )
                {
                    printf( "Error data mismatch in file %s\n", wpath );
                    free_list( &sorted_ptr_list, sorted_count );
                    wiced_filesystem_dir_close( &wdir );
                    return -7;
                }

            }

            /* Close both files */
            wiced_filesystem_file_close( &bfile );
            fclose( file );
        }
        printf( "verified %s\n", namebuf );
        entry_num++;
    }

    /* Cleanup */
    free_list( &sorted_ptr_list, sorted_count );

    wiced_filesystem_dir_close( &wdir );

    return 0;
}





















/**
 * Converts a unsigned long int to a decimal string
 *
 * @param value[in]      : The unsigned long to be converted
 * @param output[out]    : The buffer which will receive the decimal string
 * @param min_length[in] : the minimum number of characters to output (zero padding will apply if required).
 * @param max_length[in] : the maximum number of characters to output (up to 10 ). There must be space for terminating NULL.
 *
 * @note: A terminating NULL is added. Wnsure that there is space in the buffer for this.
 *
 * @return the number of characters returned (excluding terminating null)
 *
 */
uint8_t unsigned_to_decimal_string( uint32_t value, char* output, uint8_t min_length, uint8_t max_length )
{
    uint8_t digits_left;
    char buffer[ 10 ] = "0000000000";
    max_length = MIN( max_length, sizeof( buffer ) );
    digits_left = max_length;
    while ( ( value != 0 ) && ( digits_left != 0 ) )
    {
        --digits_left;
        buffer[ digits_left ] = (char) (( value % 10 ) + '0');
        value = value / 10;
    }

    digits_left = (uint8_t) MIN( ( max_length - min_length ), digits_left );
    memcpy( output, &buffer[ digits_left ], (size_t)( max_length - digits_left ) );

    /* Add terminating null */
    output[( max_length - digits_left )] = '\x00';

    return (uint8_t) ( max_length - digits_left );
}



wiced_result_t wiced_time_get_time( wiced_time_t* time_ms )
{
    struct timeval tp;
    gettimeofday( &tp, NULL );
    *time_ms = (uint64_t)tp.tv_sec * (uint64_t)1000 + (uint64_t)tp.tv_usec / (uint64_t)1000;
    return WICED_SUCCESS;
}


/* Returns the amount of characters in a string without terminating zero.  */
size_t strnlen (const char *s, size_t n)
{
  const char *p = s;
  /* We don't check here for NULL pointers.  */
  for (;*p != 0 && n > 0; p++, n--)
    ;
  return (size_t) (p - s);
}

