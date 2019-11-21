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
 * TFTP protocol and Upgrade
 *
 * ------------------------------------------------------
 * PLEASE read the following documentation before trying
 * this application!
 * ------------------------------------------------------
 *
 * This application demonstrates how to use the WICED development board to demonstrate the trivial
 * file transfer protocol (TFTP) and use it for OTA upgrade.
 *
 * Features demonstrated
 *  - WICED OTA Upgrade
 *  - TFTP protocol
 *
 *  TFTP application:
 *  ==================
 *  The application requires an external TFTP application to be running on your PC. if the snip
 *  application is running as a server, then a tftp client application is required to be running
 *  on your machine. On the other hand, if your snip application is configured as a client, then
 *  a TFTP server is required to be running on your machine.
 *  The WICED TFTP implementation uses TFTP options and requires the opposing side to support those
 *  options. When picking a TFTP application to run on you machine, make sure it support TFTP options.
 *  Some TFTP applications include:
 *  TFTPGUIUtil (http://sourceforge.net/projects/tftputil/)
 *  tftpd32 (http://tftpd32.jounin.net/tftpd32_download.html)
 *  TFTP_Client Command line (https://tftpclient.codeplex.com/releases/view/104363)
 *
 *  Windows Security:
 *  ==================
 *  The below tests were conducted with windows fireware off. TFTP is a trivial non secure protocol. Windows firewall
 *  will probably block all applications communicating through it.
 *  It is important to make sure your firewall doens't block (or interfere) with your TFTP applications.
 *
 * WICED Multi-Application Support:
 * =================================
 * As of WICED-SDK 3.1.1, WICED Application Framework (WAF) supports loading and storing of multiple
 * application binaries in the external serial flash. Up to 8 binaries are supported. The first
 * five binaries are reserved for internal usage and the last three binaries are free for users to use.
 * The binaries are organised as follows:
 *  - Factory reset application (FR_APP)
 *  - DCT backup image (DCT_IMAGE)
 *  - OTA upgrade application (OTA_APP)
 *  - Resources file system (FILESYSTEM_IMAGE)
 *  - WIFI firmware (WIFI_FIRMWARE)
 *  - Application 0 (APP0)
 *  - Application 1 (APP1)
 *  - Application 2 (APP2)
 *
 * TFTP Snippet Application:
 * =========================
 * This snippet application demonstrates how to use TFTP protocol support to perform
 * file transfer and implement OTA upgrade. The following steps assume you have a BCM943362WCD4 WICED
 * evaluation board (a BCM943362WCD4 WICED module on a WICED evaluation board). If your board is different,
 * substitute BCM943362WCD4 for your platform name.
 *
 * Prepare the WICED evaluation board for OTA upgrade
 *     1. Build the snip.tftp application to function as your factory reset and OTA
 *        application. For more details on OTA and factory reset have a look at the OTA_FR snip application.
 *     2. Run the following make target to download your production and factory reset applications
 *        to the board:
 *            make snip.ota_fr-BCM943362WCD4 download download_apps run
 *     3. Build an application that will upgrade the current application.
 *        For this example we will build the snip.scan application:
 *            make snip.scan-BCM943362WCD4
 *     4. On your machine, Install a TFTP client that supports TFTP options (see the list above).
 * To read the DCT image
 *     5. Configure your TFTP client for 2 seconds time out and block size of 512 bytes.
 *     6. In the filename type dct.elf and run TFTP get.
 *     7. The terminal will show a print messages indicating tranfering files to your host machine.
 *     8. After the transfer your tftp folder on the client should include a dct.elf file
 * To write APP0 image
 *     5. Configure your TFTP client for 2 seconds time out and block size of 512 bytes.
 *     6. In the filename type app0.elf and run TFTP put.
 *     7. The terminal will show print messages indicating transferring files from your host machine.
 *     8. After the transfer the WICED will restart and run the new application.
 *
 */

#include "wiced.h"
#include "wwd_debug.h"
#include "wiced_framework.h"
#include "waf_platform.h"
#include "tftp.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
/* Undef TFTP_SNIP_CLIENT to run the test as a server */
//#define TFTP_SNIP_CLIENT
/* Undef TFTP_GET to test TFTP PUT (i.e sending files) */
#define TFTP_SNIP_GET

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
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
static tftp_connection_t conn;
static tftp_callback_t callbacks;

static uint32_t current_size = 0;
static uint32_t offset = 0;
static wiced_app_t app;

#ifdef TFTP_SNIP_CLIENT
static wiced_ip_address_t INITIALISER_IPV4_ADDRESS( host_ip, MAKE_IPV4_ADDRESS(192, 168, 1, 141) );
#endif
/******************************************************
 *               Function Definitions
 ******************************************************/
static uint8_t get_file_index( const char * filename )
{
    if ( strcmp( filename, "fr.elf" ) == 0 )
    {
        return DCT_FR_APP_INDEX;
    }

    if ( strcmp( filename, "dct.elf" ) == 0 )
    {
        return DCT_DCT_IMAGE_INDEX;
    }
    if ( strcmp( filename, "ota.elf" ) == 0 )
    {
        return DCT_OTA_APP_INDEX;
    }
    if ( strcmp( filename, "app0.elf" ) == 0 )
    {
        return DCT_APP0_INDEX;
    }

    if ( strcmp( filename, "app1.elf" ) == 0 )
    {
        return DCT_APP1_INDEX;
    }
    if ( strcmp( filename, "app2.elf" ) == 0 )
    {
        return DCT_APP2_INDEX;
    }
    if ( strcmp( filename, "wifi.elf" ) == 0 )
    {
        return DCT_WIFI_FIRMWARE_INDEX;
    }
    if ( strcmp( filename, "fs.elf" ) == 0 )
    {
        return DCT_FILESYSTEM_IMAGE_INDEX;
    }
    return -1;
}

wiced_result_t snip_tftp_establish( tftp_t* tftp, void* p_user )
{

    uint32_t file_size = tftp->transfer_size;
    uint8_t file_index = get_file_index( tftp->filename );
    if ( file_index == (uint8_t) -1 )
    {
        return WICED_ERROR;
    }
    if ( wiced_framework_app_open( file_index, &app ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    if ( wiced_framework_app_get_size( &app, &current_size ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }
    printf( "current size %lu\n", current_size );
    if ( tftp->request == TFTP_GET )
    {
        if ( wiced_framework_app_set_size( &app, file_size ) != WICED_SUCCESS )
        {
            return WICED_ERROR;
        }
        if ( wiced_framework_app_get_size( &app, &current_size ) != WICED_SUCCESS )
        {
            return WICED_ERROR;
        }
    }
    else
    {
        if ( current_size == 0 )
        {
            return WICED_ERROR;
        }
    }
    offset = 0;
    printf( "new size %lu\n", current_size );
    return WICED_SUCCESS;
}

wiced_result_t snip_tftp_write( tftp_t* tftp, uint8_t* data, void* p_user )
{
    printf( "Writing size %d from offset %lu [0x%x] \r\n", tftp->block_size, offset, data[ 0 ] );

    if ( wiced_framework_app_write_chunk( &app, data, tftp->block_size ) != WICED_SUCCESS )
    {
        printf( "Error Writing\n" );
    }

    offset += tftp->block_size;
    return WICED_SUCCESS;
}

wiced_result_t snip_tftp_read( tftp_t* tftp, uint8_t* data, void* p_user )
{
    uint16_t size = MIN(tftp->block_size, current_size - offset);

    if ( wiced_framework_app_read_chunk( &app, offset, data, size ) != WICED_SUCCESS )
    {
        printf( "Error Reading\n" );
    }
    printf( "Reading size %d from offset %lu [0x%x] \r\n", size, offset, data[ 1 ] );
    offset += tftp->block_size;
    tftp->block_size = size;
    return WICED_SUCCESS;
}

wiced_result_t snip_tftp_close( tftp_t* tftp, int status, void* p_user )
{
    uint8_t file_index = app.app_id;
    if ( status == TFTP_NO_ERROR )
    {
        printf( "File transfer complete.\r\n" );
        if ( ( tftp->request == TFTP_GET ) &&
             (  ( file_index == DCT_APP0_INDEX )
             || ( file_index == DCT_APP1_INDEX )
             || ( file_index == DCT_APP2_INDEX )
             || ( file_index == DCT_FR_APP_INDEX )
             || ( file_index == DCT_OTA_APP_INDEX ) )
           )
        {
            wiced_framework_set_boot( file_index, PLATFORM_DEFAULT_LOAD );
            printf( "Restarting...\n" );
            wiced_framework_reboot( );
        }
    }
    else
    {
        printf( "Connect closed.\n" );
    }
    return WICED_SUCCESS;
}

void application_start( )
{
    wiced_init( );

    WPRINT_APP_INFO( ( "Time for an upgrade. TFTP upgrade starting ...\r\n" ) );

    /* Bringup the network interface */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    callbacks.tftp_establish = snip_tftp_establish;
    callbacks.tftp_close = snip_tftp_close;
    callbacks.tftp_write = snip_tftp_write;
    callbacks.tftp_read = snip_tftp_read;

#ifdef TFTP_SNIP_CLIENT
#ifdef TFTP_SNIP_GET
    if ( tftp_client_get( &conn, host_ip, WICED_STA_INTERFACE, "app0.elf", TFTP_MODE_OCTET, &callbacks, NULL ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to init tftp\n"));
    }
#else
    if ( tftp_client_put( &conn
                    ,host_ip
                    ,WICED_STA_INTERFACE
                    ,"dct.elf"
                    ,TFTP_MODE_OCTET
                    ,&callbacks
                    ,NULL) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to init tftp\n"));
    }
#endif
#else
    if ( tftp_server_start( &conn
                    ,WICED_STA_INTERFACE
                    ,&callbacks
                    ,NULL
            ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to init tftp\n"));
    }
#endif

    while ( 1 )
        wiced_rtos_delay_milliseconds( 1000 );
}

