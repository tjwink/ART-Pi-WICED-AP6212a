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
 *  This application demonstrates how to add files to your WICED application, and then open those files for read (read-only).
 *  In order to add your own resources to the application, the following assets must be installed in your WICED SDK tree:
 *
 *  resources/apps/resource_read/test_file01.txt
 *  resources/apps/resource_read/test_file02.txt
 *
 *  In general, you can add files to the filesystem by installing the file as:
 *
 *  resources/apps/<app-name>/<filename>
 */

#include "wiced.h"
#include "resources.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define RES_01 "test_file01.txt"
#define RES_02 "test_file02.txt"

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

void application_start( )
{
    uint32_t size_out;
    const void * buffer;
    resource_result_t result;

    wiced_init();

    /* Remove buffering from all std streams */
    setvbuf( stdin, NULL, _IONBF, 0 );
    setvbuf( stdout, NULL, _IONBF, 0 );
    setvbuf( stderr, NULL, _IONBF, 0 );

    printf( "\nReading Resources from WicedFS demo\n\n");

    // RES_01
    printf( "Loading resource %s...\n\n", RES_01 );
    // for automatic resource name generation convention, refer to resources/README.txt
    result = resource_get_readonly_buffer ( &resources_apps_DIR_resource_read_DIR_test_file01_txt, 0, 0x7fffffff, &size_out, &buffer );
    if ( result != RESOURCE_SUCCESS )
    {
        printf( "Failed to read %s\n", RES_01 );
        return;
    }
    printf( "resource %s size = %d\n\n", RES_01, (int) size_out );
    /* because buffer is not end with NULL terminator, print buffer to stdout with size */
    fwrite( buffer, 1, size_out, stdout );
    printf("\n\n");
    resource_free_readonly_buffer( &resources_apps_DIR_resource_read_DIR_test_file01_txt, buffer );

    // RES_02
    printf( "Loading resource %s...\n\n", RES_02 );
    // for automatic resource name generation convention, refer to resources/README.txt
    result = resource_get_readonly_buffer ( &resources_apps_DIR_resource_read_DIR_test_file02_txt, 0, 0x7fffffff, &size_out, &buffer );
    if ( result != RESOURCE_SUCCESS )
    {
        printf( "Failed to read %s\n", RES_02 );
        return;
    }
    printf( "resource %s size = %d\n\n", RES_02, (int) size_out );
    /* because buffer is not end with NULL terminator, print buffer to stdout with size */
    fwrite( buffer, 1, size_out, stdout );
    printf("\n\n");
    resource_free_readonly_buffer( &resources_apps_DIR_resource_read_DIR_test_file02_txt, buffer );
}
