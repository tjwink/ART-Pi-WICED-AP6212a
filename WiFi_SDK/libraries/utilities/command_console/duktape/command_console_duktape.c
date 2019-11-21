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

#include "wiced.h"
#include "wiced_duktape.h"
#include "wiced_filesystem.h"
#include "command_console.h"
#include "command_console_duktape.h"
#include "tftp.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define LOG_LABEL           "duk:cmd"
#define LOG_DEBUG_ENABLE    0

/******************************************************
 *                    Constants
 ******************************************************/

#define TFTP_BUFFER_SIZE    (8 * 1024)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct tftp_buffer_s
{
    char*       ptr;
    uint32_t    size;
    uint32_t    pos;
} tftp_buffer_t;

typedef struct tftp_transfer_s
{
    wiced_bool_t        stop;
    tftp_buffer_t       buffer;
    wiced_filesystem_t* fs_ptr;
    wiced_file_t        file;
} tftp_transfer_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_filesystem_t   fs_handle;
static wiced_bool_t         fs_mounted;

/******************************************************
 *               Function Definitions
 ******************************************************/

static wiced_filesystem_t* filesystem_mount( const char* fs_device )
{
    wiced_filesystem_t*         fs_ptr;
    const filesystem_list_t*    fs = all_filesystem_devices;

    if ( fs_device == NULL )
    {
        return NULL;
    }

    /* Check if filesystem is valid */
    while ( fs->device != NULL )
    {
        if ( strcmp( fs_device, fs->name ) == 0 )
        {
            LOGD( "Found filesystem device '%s'", fs->name );
            break;
        }

        fs++;
    }

    if ( fs->device == NULL )
    {
        LOGE( "Could not find filesystem device '%s'", fs_device );
        return NULL;
    }

    /* Check if device is already mounted; if not, then mount it */
    LOGD( "Attemping to get filesystem handle" );
    fs_ptr = wiced_filesystem_retrieve_mounted_fs_handle( fs->name );
    if ( fs_ptr == NULL )
    {
        wiced_result_t result;

        LOGD( "Mounting filesystem device '%s'", fs->name );

        result = wiced_filesystem_mount( fs->device, fs->type, &fs_handle,
                                         fs->name );
        if ( result != WICED_SUCCESS )
        {
            LOGE( "Failed to mount filesystem device '%s' (result=%d)",
                  fs->name, result );
            return NULL;
        }

        LOGI( "Mounted filesystem device '%s'", fs->name );

        fs_ptr = &fs_handle;
        fs_mounted = WICED_TRUE;
    }

    return fs_ptr;
}

static wiced_result_t filesystem_unmount( void )
{
    wiced_result_t result = WICED_SUCCESS;

    if ( fs_mounted == WICED_TRUE )
    {
        result = wiced_filesystem_unmount( &fs_handle );
        if ( result != WICED_SUCCESS )
        {
            LOGE( "Failed to unmount filesystem device (result=%d)", result );
            result = WICED_ERROR;
        }
    }

    fs_mounted = WICED_FALSE;

    return result;
}

static wiced_result_t tftp_establish( tftp_t* tftp, void* p_user )
{
    tftp_transfer_t *tftp_transfer = (tftp_transfer_t*) p_user;

    LOGI( "TFTP connection established (%s '%s')",
          tftp->request == TFTP_GET ? "PUT" : "GET", tftp->filename );

    if ( tftp->request == TFTP_GET )
    {
        /* We need at least 1 byte at the very end for '\0' so Duktape can
         * use it to detect end of buffer. Just to make things nice, make the
         * buffer size a multiple of 4-bytes, i.e., in the worst case scenario,
         * we allocate 4 more bytes than needed.
         */
        tftp_transfer->buffer.size = tftp->transfer_size != 0 ?
                                     ( tftp->transfer_size + 4 ) & ~0x3 :
                                     TFTP_BUFFER_SIZE;

        LOGD( "Allocating %lu bytes for TFTP transfer buffer",
              tftp_transfer->buffer.size );
        tftp_transfer->buffer.ptr = (char*)calloc( 1,
                                                   tftp_transfer->buffer.size );
        if ( tftp_transfer->buffer.ptr == NULL )
        {
            LOGE( "Failed to allocate memory for TFTP transfer buffer" );
            return WICED_ERROR;
        }

        if ( tftp_transfer->fs_ptr != NULL )
        {
            wiced_result_t result;

            /* Check for directories and create them */
            if ( strchr( tftp->filename, '/' ))
            {
                char*   path = strdup( tftp->filename );
                int     pos = 0;

                while ( strchr( &path[pos], '/' ))
                {
                    char*   dir;
                    char*   rest;

                    rest = strstr( &path[pos], "/" );
                    dir = strndup( path, strlen( path ) - strlen( rest ));
                    LOGD( "dir=%s, rest=%s", dir, rest );

                    result = wiced_filesystem_dir_create( tftp_transfer->fs_ptr, dir );
                    free(dir);

                    if ( result != WICED_SUCCESS )
                    {
                        LOGE( "Failed to create directory '%s'", dir );
                        break;
                    }

                    pos += strlen(&path[pos]) - strlen(rest) + 1;
                }

                free( path );

                if ( result != WICED_SUCCESS )
                {
                    return WICED_ERROR;
                }
            }

            /* Open file for writing */
            result = wiced_filesystem_file_open( tftp_transfer->fs_ptr,
                                                 &tftp_transfer->file, tftp->filename,
                                                 WICED_FILESYSTEM_OPEN_ZERO_LENGTH );
            if ( result != WICED_SUCCESS )
            {
                LOGE( "Failed to create file '%s'", tftp->filename );
            }
        }
    }
    else
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

static wiced_result_t tftp_read( tftp_t* tftp, uint8_t* data, void* p_user )
{
    return WICED_ERROR;
}

static wiced_result_t tftp_write( tftp_t* tftp, uint8_t* data, void* p_user )
{
    tftp_transfer_t *tftp_transfer = (tftp_transfer_t*) p_user;

    /* Make sure we leave at least 1 byte at the end for '\0' */
    if ( tftp_transfer->buffer.pos + tftp->block_size >
         tftp_transfer->buffer.size - 1 )
    {
        LOGE( "TFTP file too large for buffer" );

        /* Clean up the buffer here */
        free( tftp_transfer->buffer.ptr );
        tftp_transfer->buffer.ptr = NULL;

        return WICED_ERROR;
    }

    LOGD( "TFTP transfer %d bytes to pos %lu", tftp->block_size,
          tftp_transfer->buffer.pos );

    memcpy( &tftp_transfer->buffer.ptr[tftp_transfer->buffer.pos], data,
            tftp->block_size );

    tftp_transfer->buffer.pos += tftp->block_size;

    if ( tftp_transfer->fs_ptr != NULL )
    {
        wiced_result_t  result;
        uint64_t        bytes_w;

        result = wiced_filesystem_file_write( &tftp_transfer->file, data,
                                              tftp->block_size, &bytes_w );
        if ( result != WICED_SUCCESS )
        {
                LOGE( "Failed to write to file" );
        }

        LOGD( "Wrote %lu bytes to file", (uint32_t)( bytes_w & 0xFFFFFFFF ));
    }

    return WICED_SUCCESS;
}

static wiced_result_t tftp_close( tftp_t* tftp, int status, void* p_user )
{
    tftp_transfer_t *tftp_transfer = (tftp_transfer_t*) p_user;

    LOGI( "TFTP connection closed" );

    if ( tftp_transfer->fs_ptr != NULL )
    {
        LOGI( "Closing file" );
        wiced_filesystem_file_close( &tftp_transfer->file );
    }

    tftp_transfer->stop = WICED_TRUE;

    return WICED_SUCCESS;
}

/** Evaluate a string buffer with Duktape
 *
 * @param[in] argc  : Number of arguments
 * @param[in] argv  : Array of arguments
 *
 * @return  Console error code indicating if the command ran correctly
 */

int command_console_duktape_eval( int argc, char* argv[] )
{
    if ( argc != 2 )
    {
        LOGE( "Invalid number of arguments\n" );
        return (argc < 2 ? ERR_INSUFFICENT_ARGS : ERR_TOO_MANY_ARGS);
    }

    LOGD( "Evaluating buffer '%s'", argv[1] );

    if ( wiced_duktape_eval_buffer( argv[1] ) != WICED_SUCCESS )
    {
        LOGE( "Failed to evaluate script" );
        return ERR_UNKNOWN;
    }

    return ERR_CMD_OK;
}

/** Evaluate a file with Duktape
 *
 * @param[in] argc  : Number of arguments
 * @param[in] argv  : Array of arguments
 *
 * @return  Console error code indicating if the command ran correctly
 */

int command_console_duktape_file( int argc, char* argv[] )
{
    if ( argc != 2 )
    {
        LOGE( "Invalid number of arguments\n" );
        return (argc < 2 ? ERR_INSUFFICENT_ARGS : ERR_TOO_MANY_ARGS);
    }

    LOGD( "Evaluating file '%s'", argv[1] );

    if ( wiced_duktape_eval_file( argv[1] ) != WICED_SUCCESS )
    {
        LOGE( "Failed to evaluate script" );
        return ERR_UNKNOWN;
    }

    return ERR_CMD_OK;
}

/** Start a TFTP server and evaluate uploaded file with Duktape
 *
 * @param[in] argc  : Number of arguments
 * @param[in] argv  : Array of arguments
 *
 * @return  Console error code indicating if the command ran correctly
 */

int command_console_duktape_tftp( int argc, char* argv[] )
{
    int                 ret = ERR_CMD_OK;
    wiced_result_t      result;
    tftp_connection_t   tftp_conn;
    tftp_callback_t     tftp_cb;
    tftp_transfer_t     tftp_transfer;

    /* Make sure network is up */
    if ( wiced_network_is_ip_up( WICED_STA_INTERFACE ) != WICED_TRUE )
    {
        LOGI( "Bringing up network interface" );

        result = wiced_network_up( WICED_STA_INTERFACE,
                                   WICED_USE_EXTERNAL_DHCP_SERVER, NULL );
        if ( result != WICED_SUCCESS )
        {
            LOGE( "Failed to bring up network interface" );
            return ERR_UNKNOWN;
        }

        LOGI( "Network interface brought up" );
    }

    memset( &tftp_transfer, 0, sizeof( tftp_transfer ));

    /* Get a filesystem handle */
    if (( argc == 2 ) && ( argv[1] != NULL ))
    {
        tftp_transfer.fs_ptr = filesystem_mount( argv[1] );
        if ( tftp_transfer.fs_ptr == NULL )
        {
            LOGE( "Failed to mount filesystem device '%s'- not saving to filesystem",
                  argv[1] );
        }
    }

    /* Set TFTP callbacks */
    tftp_cb.tftp_establish = tftp_establish;
    tftp_cb.tftp_read = tftp_read;
    tftp_cb.tftp_write = tftp_write;
    tftp_cb.tftp_close = tftp_close;

    /* Start TFTP server */
    LOGI( "Starting TFTP server" );
    result = tftp_server_start( &tftp_conn, WICED_STA_INTERFACE, &tftp_cb,
                                &tftp_transfer );
    if ( result != WICED_SUCCESS )
    {
        LOGE( "Failed to start TFTP server" );
        return ERR_UNKNOWN;
    }

    LOGI( "TFTP server started; press CTRL-C to quit" );
    while ( 1 )
    {
        uint8_t     character;
        uint32_t    expected_transfer_size = 1;

        if ( tftp_transfer.stop == WICED_TRUE )
        {
            break;
        }

        /* Monitor for CTRL-C, which becomes the EOT character */
        result = wiced_uart_receive_bytes( STDIO_UART , &character,
                                           &expected_transfer_size, 1000 );
        if ( result == WICED_SUCCESS && character == 3 )
        {
            break;
        }
    }

    LOGI( "Stopping TFTP server" );
    tftp_server_stop( &tftp_conn );

    if ( tftp_transfer.buffer.ptr != NULL )
    {
        LOGI( "Running uploaded script" );

        if ( wiced_duktape_eval_buffer( tftp_transfer.buffer.ptr ) !=
             WICED_SUCCESS )
        {
            LOGE( "Failed to evaluate script" );
        }

        LOGD( "Freeing TFTP transfer buffer" );
        free( tftp_transfer.buffer.ptr );
    }

    if ( tftp_transfer.fs_ptr != NULL )
    {
        filesystem_unmount();
        tftp_transfer.fs_ptr = NULL;
    }

    return ret;
}

/** Stop the running instance of Duktape and destroy the heap
 *
 * @param[in] argc  : Number of arguments (unused)
 * @param[in] argv  : Array of arguments (unused)
 *
 * @return  Console error code indicating if the command ran correctly
 */

int command_console_duktape_stop( int argc, char* argv[] )
{
    if ( wiced_duktape_stop_eval() != WICED_SUCCESS )
    {
        LOGE( "Failed to stop Duktape" );
        return ERR_UNKNOWN;
    }

    return ERR_CMD_OK;
}

/** Run one or all of the Duktape API tests
 *
 * @param[in] argc  : Number of arguments
 * @param[in] argv  : Array of arguments
 *
 * @return  Console error code indicating if the command ran correctly
 */

int command_console_duktape_api_test( int argc, char* argv[] )
{
    const char* test = NULL;

    if ( argc == 2 )
    {
        test = argv[1];
    }

    if ( wiced_duktape_run_api_tests( test ) != WICED_SUCCESS )
    {
        LOGE( "Failed to run Duktape API tests" );
        return ERR_UNKNOWN;
    }

    return ERR_CMD_OK;
}

