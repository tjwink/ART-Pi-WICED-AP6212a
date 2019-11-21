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

#include "wiced.h"
#include "wiced_filesystem.h"
#include "sdiovar.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define PATH_BUFFER_SIZE        FX_MAXIMUM_PATH
#define DIRECTORY_SEPARATOR_STR "/"
#define ROOT_DIRECTORY          "/"
#define DIRECTORY_SEPARATOR_CHR '/'
/* reduce FILE_BUFFER_SIZE or increse APPLICATION_STACK_SIZE in mk file,
   if you meet stack overflow */
#define FILE_BUFFER_SIZE        (65536)
#define SRC_DIR                 "/src"
#define TGT_DIR                 "/tgt"
#define PROGRESS_UNIT           (50)

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
static int list_files( wiced_filesystem_t* fs_handle, const char* dir_name );
static int mirror_files( wiced_filesystem_t* fs_handle, const char* src_dir_name, const char* tgt_dir_name );
static int delete_files( wiced_filesystem_t* fs_handle, const char* dir_name );
static int copy_file( wiced_filesystem_t* fs_handle, const char* src_path, const char* tgt_path );

/******************************************************
 *               Variable Definitions
 ******************************************************/
extern uint sd_msglevel;
extern wiced_block_device_t block_device_sdmmc;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    wiced_filesystem_t fs_handle;
    wiced_result_t result;
    wiced_dir_t dir;

    sd_msglevel = SDH_ERROR_VAL;

    printf( "Testing SD/MMC fs\n\n");

    if ( !PLATFORM_FEATURE_ENAB(SDIO) )
    {
        printf( "\nSD card NOT supported\n" );
        return;
    }

    result = wiced_filesystem_init();
    if ( result != WICED_SUCCESS  )
    {
        printf( "Error initing filesystems\n");
        return;
    }

    /* TEST FileX */
    printf( "\n\nTesting SD/MMC on FileX\n\n");

    result = wiced_filesystem_mount( &block_device_sdmmc, WICED_FILESYSTEM_HANDLE_FILEX, &fs_handle, "TestWicedFS" );

    if ( result != WICED_SUCCESS )
    {
        printf( "Error mounting filesystem\n" );
    }

    if ( WICED_SUCCESS != wiced_filesystem_dir_open( &fs_handle, &dir, SRC_DIR ) )
    {
        wiced_filesystem_dir_close( &dir );
        printf( "source dir %s is not existing.\n", SRC_DIR );
        goto finish;
    }

    if ( WICED_SUCCESS == wiced_filesystem_dir_open( &fs_handle, &dir, TGT_DIR ) )
    {
        printf( "target dir %s is already existing.\n", TGT_DIR );
        wiced_filesystem_dir_close( &dir );

        printf( "deleting %s first...\n", TGT_DIR );

        if ( 0 != delete_files( &fs_handle, TGT_DIR ) )
        {
            printf( "fail to deleting %s.\n", TGT_DIR );
            goto finish;
        }
    }

    printf( "creating target directory %s.\n", TGT_DIR );

    if ( WICED_SUCCESS != wiced_filesystem_dir_create( &fs_handle, TGT_DIR ) )
    {
        printf( "Error creating target directory %s\n", TGT_DIR );
        goto finish;
    }

    if ( 0 != mirror_files( &fs_handle, SRC_DIR, TGT_DIR ) )
    {
        printf( "fail to mirroring.\n" );
        goto finish;
    }

    list_files( &fs_handle, ROOT_DIRECTORY );

finish:
    /* Clean up */
    if ( WICED_SUCCESS != wiced_filesystem_unmount( &fs_handle ) )
    {
        printf( "Error unmounting filesystem\n" );
        return;
    }

    printf( "\nSD Card driver test done successfully.\n\n" );
}

static int list_files( wiced_filesystem_t* fs_handle, const char* dir_name )
{
    wiced_dir_t   dir;
    int len;
    char namebuf[PATH_BUFFER_SIZE];
    char pathbuf[PATH_BUFFER_SIZE];
    wiced_dir_entry_type_t    entry_type;
    wiced_dir_entry_details_t details;

    printf( "list_files: %s\n", dir_name );

    strcpy(namebuf, dir_name);
    len = strlen(namebuf);
    if ( len > 1 && namebuf[ len - 1 ] == DIRECTORY_SEPARATOR_CHR )
    {
        namebuf[ len - 1 ] = '\0';
    }

    /* Open the Wiced directory */
    if ( WICED_SUCCESS != wiced_filesystem_dir_open( fs_handle, &dir, namebuf ) )
    {
        printf( "%d: Error opening dir %s\n", __LINE__, namebuf );
        return -1;
    }

    while ( 1 != wiced_filesystem_dir_end_reached( &dir ) )
    {
        if ( WICED_SUCCESS == wiced_filesystem_dir_read( &dir, namebuf, sizeof(namebuf), &entry_type, &details ) )
        {
            // printf( "%d: %s%s\n", entry_type, dir_name, namebuf );
            printf( "%s%s\n", dir_name, namebuf );
            if ( entry_type == WICED_FILESYSTEM_DIR && 0 != strcmp( ".", namebuf ) && 0 != strcmp( "..", namebuf ) )
            {
                sprintf( pathbuf, "%s%s%s", dir_name, namebuf, DIRECTORY_SEPARATOR_STR );
                list_files( fs_handle, pathbuf );
            }
        }
        else
        {
            break;
        }
    }

    wiced_filesystem_dir_close( &dir );

    return 0;
}

static int mirror_files( wiced_filesystem_t* fs_handle, const char* src_dir_name, const char* tgt_dir_name )
{
    wiced_dir_t   src_dir, tgt_dir;
    char namebuf[PATH_BUFFER_SIZE];
    char src_pathbuf[PATH_BUFFER_SIZE];
    char tgt_pathbuf[PATH_BUFFER_SIZE];
    wiced_dir_entry_type_t    entry_type;
    wiced_dir_entry_details_t details;

    printf( "mirror_files: %s to %s\n", src_dir_name, tgt_dir_name );

    /* Open the Wiced directory */
    if ( WICED_SUCCESS != wiced_filesystem_dir_open( fs_handle, &src_dir, src_dir_name ) )
    {
        printf( "%d: Error opening dir %s\n", __LINE__, src_dir_name );
        return -1;
    }

    printf( "mirror_files: source dir %s opened\n", src_dir_name );

    if ( WICED_SUCCESS != wiced_filesystem_dir_open( fs_handle, &tgt_dir, tgt_dir_name ) )
    {
        if ( WICED_SUCCESS != wiced_filesystem_dir_create( fs_handle, tgt_dir_name ) )
        {
            wiced_filesystem_dir_close( &src_dir );
            printf( "Error creating dir %s\n", tgt_dir_name );
            return -1;
        }
    }

    printf( "mirror_files: target dir %s opened\n", tgt_dir_name );

    while ( 1 != wiced_filesystem_dir_end_reached( &src_dir ) )
    {
        if ( WICED_SUCCESS == wiced_filesystem_dir_read( &src_dir, namebuf, sizeof(namebuf), &entry_type, &details ) )
        {
            if ( entry_type == WICED_FILESYSTEM_DIR )
            {
                if ( 0 == strcmp( ".", namebuf ) || 0 == strcmp( "..", namebuf ) )
                {
                    continue;
                }
                sprintf( src_pathbuf, "%s%s%s", src_dir_name, DIRECTORY_SEPARATOR_STR, namebuf );
                sprintf( tgt_pathbuf, "%s%s%s", tgt_dir_name, DIRECTORY_SEPARATOR_STR, namebuf );
                mirror_files( fs_handle, src_pathbuf, tgt_pathbuf );
            }
            else if ( entry_type == WICED_FILESYSTEM_FILE )
            {
                sprintf( src_pathbuf, "%s%s%s", src_dir_name, DIRECTORY_SEPARATOR_STR, namebuf );
                sprintf( tgt_pathbuf, "%s%s%s", tgt_dir_name, DIRECTORY_SEPARATOR_STR, namebuf );
                copy_file( fs_handle, src_pathbuf, tgt_pathbuf );

            }
            else if ( entry_type == WICED_FILESYSTEM_LINK )
            {
                printf( "mirroring file type 'WICED_FILESYSTEM_LINK' not supported.\n");
            }
            else
            {
                printf( "fail to mirroring unknown file type.\n");
            }
        }
        else
        {
            break;
        }
    }

    wiced_filesystem_dir_close( &src_dir );
    wiced_filesystem_dir_close( &tgt_dir );

    return 0;
}

static int delete_files( wiced_filesystem_t* fs_handle, const char* dir_name )
{
    wiced_dir_t   dir;
    char namebuf[PATH_BUFFER_SIZE];
    char pathbuf[PATH_BUFFER_SIZE];
    wiced_dir_entry_type_t    entry_type;
    wiced_dir_entry_details_t details;

    printf( "delete_files: %s\n", dir_name );

    /* Open the Wiced directory */
    if ( WICED_SUCCESS != wiced_filesystem_dir_open( fs_handle, &dir, dir_name ) )
    {
        printf( "%d: Error opening dir %s\n", __LINE__, dir_name );
        return -1;
    }

    while ( 1 != wiced_filesystem_dir_end_reached( &dir ) )
    {
        if ( WICED_SUCCESS == wiced_filesystem_dir_read( &dir, namebuf, sizeof(namebuf), &entry_type, &details ) )
        {
            if ( entry_type == WICED_FILESYSTEM_DIR )
            {
                if ( 0 == strcmp( ".", namebuf ) || 0 == strcmp( "..", namebuf ) )
                {
                    continue;
                }
                sprintf( pathbuf, "%s%s%s", dir_name, DIRECTORY_SEPARATOR_STR, namebuf );
                if ( 0 == delete_files( fs_handle, pathbuf ) )
                {
                    printf( "deleting %s...\n", pathbuf );
                    if ( WICED_SUCCESS != wiced_filesystem_dir_delete( fs_handle, pathbuf ) )
                    {
                        printf( "Error deleting %s\n", pathbuf );
                        wiced_filesystem_dir_close( &dir );
                        return -1;
                    }
                }
            }
            else
            {
                sprintf( pathbuf, "%s%s%s", dir_name, DIRECTORY_SEPARATOR_STR, namebuf );
                printf( "deleting %s...\n", pathbuf );
                if ( WICED_SUCCESS != wiced_filesystem_file_delete( fs_handle, pathbuf ) )
                {
                    printf( "Error deleting %s\n", pathbuf );
                    wiced_filesystem_dir_close( &dir );
                    return -1;
                }
            }
        }
        else
        {
            break;
        }
    }

    wiced_filesystem_dir_close( &dir );

    return 0;
}

static int copy_file( wiced_filesystem_t* fs_handle, const char* src_path, const char* tgt_path )
{
    wiced_file_t f_src;
    wiced_file_t f_tgt;
    char filebuf[FILE_BUFFER_SIZE];
    int result = 0;
    uint64_t read_count = 1;
    uint64_t write_count = 1;
    int progress = 0;

    printf( "copy_file: %s to %s\n", src_path, tgt_path );

    if ( WICED_SUCCESS != wiced_filesystem_file_open( fs_handle, &f_src, src_path, WICED_FILESYSTEM_OPEN_FOR_READ ))
    {
        printf( "Error opening src file %s\n", src_path );
        result = -1;
        goto finish;
    }

    if ( WICED_SUCCESS != wiced_filesystem_file_open( fs_handle, &f_tgt, tgt_path, WICED_FILESYSTEM_OPEN_ZERO_LENGTH ))
    {
        printf( "Error opening target file %s\n", tgt_path );
        result = -1;
        goto finish;
    }

    while ( read_count && write_count )
    {
        wiced_filesystem_file_read( &f_src, filebuf, ( uint64_t ) FILE_BUFFER_SIZE, &read_count );

        if ( 0 == ( progress % PROGRESS_UNIT ) )
        {
           // printf("R");
        }

        if ( read_count )
        {
            wiced_filesystem_file_write( &f_tgt, (const char *) filebuf, read_count, &write_count );

            if ( 0 == ( progress % PROGRESS_UNIT ) )
            {
                // printf("W");
            }

            if ( read_count != write_count )
            {
                printf( "wiced_filesystem_file_write: try = %lu, write = %lu\n", ( uint32_t ) read_count, ( uint32_t ) write_count );
                result = -1;
                break;
            }
        }
        progress++;
    }

    printf("\n");


finish:
    /*
       Note:
       Because there is bug in original FileX 5.5 exFAT 'fx_media_flush' and 'fx_media_close',
       close f_tgt file which is opened for WRITE mode first before closing f_src.
       fx_media_flush and fx_media_close don't call _fx_directory_exFAT_entry_write for exFAT filesystem,
       which corrupts directory entries.
    */
    wiced_filesystem_file_close( &f_tgt );
    wiced_filesystem_file_close( &f_src );

    return result;
}
