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

#include "typedefs.h"
#include "bcmdevs.h"
#include "wiced_osl.h"
#include "wiced.h"
#include "command_console.h"
#include "command_console_fs.h"
#include "wiced_management.h"
#include "wiced_resource.h"
#include "wiced_usb.h"
#include "wiced_filesystem.h"
#include "ux_api.h"
#include "ux_hcd_ehci.h"
#include "ux_hcd_ohci.h"
#include "ux_host_class_hub.h"
#include "ux_host_class_storage.h"
#include "usb_host_storage_read_write.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
/* Parameters to init Console Command */
#define MAX_LINE_LENGTH                         (128)
#define MAX_HISTORY_LENGTH                      (20)

/* Parameters to init USB Host. Usually keep using default values, adjustable for user scenario */
#define MAX_HOST_CLASS_DRIVER                   (8)
#define MAX_HOST_HCD                            (2)
#define MAX_HOST_DEVICES                        (8)
#define MAX_HOST_ED                             (80)
#define MAX_HOST_TD                             (128)
#define MAX_HOST_ISO_TD                         (2)
#define USB_HOST_THREAD_STACK_SIZE              (8 * 1024)

/* Application timeout for waiting usb device insertion */
#define USB_HOST_APP_DETECTION_TIMEOUT          (30) //sec

/* Application delay time parameters */
#define USB_HOST_APP_DEVICE_SCAN_DELAY_TIME     (1000) //ms


/* USB Host console command sets */
#define USB_HOST_APP_COMMANDS \
    { (char*) "disk_rw",        usb_host_app_storage_rw,            0, NULL, NULL, (char*) "",              (char*) "single file read/write test sha1sum" }, \
    { (char*) "disk_rwthp",     usb_host_app_storage_rw_throughput, 0, NULL, NULL, (char*) "",              (char*) "read/write throughput measurement" },


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
 *               Variable Definitions
 ******************************************************/
/* USB Host console command sets */
static char line_buffer[MAX_LINE_LENGTH];
static char history_buffer_storage[MAX_LINE_LENGTH * MAX_HISTORY_LENGTH];

static const command_t commands[] =
{
    USB_HOST_APP_COMMANDS
    FS_COMMANDS
    CMD_TABLE_END
};

/* Host STORAGE class (Noted that: Now support only ONE storage partition!) */
static UX_HOST_CLASS_STORAGE        *storage = NULL;

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_result_t usb_host_app_event_handler( uint32_t event, void *param1, void *param2 );

/******************************************************
 *               Function Definitions
 ******************************************************/
void application_start( void )
{
    wiced_usb_user_config_t user_cfg;
    UINT status;

    WPRINT_APP_INFO( ("\n") );
    WPRINT_APP_INFO( ("+-------------------------------------------------+\n") );
    WPRINT_APP_INFO( ("+ USB20 Host Storage Class Read/Write Application +\n") );
    WPRINT_APP_INFO( ("+-------------------------------------------------+\n") );

    /* Init Filesystem  */
    status = wiced_filesystem_init();
    if( status != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("Filesystem init failed. status=%d\n", status) );
        return;
    }

    /* Run console application function  */
    command_console_init( STDIO_UART, MAX_LINE_LENGTH, line_buffer, MAX_HISTORY_LENGTH, history_buffer_storage, " " );
    console_add_cmd_table( commands );

    /* Init USB20 Host SW & HW  */
    memset( (void*) &user_cfg, 0, sizeof(user_cfg) );
    user_cfg.host_max_class         = MAX_HOST_CLASS_DRIVER;
    user_cfg.host_max_hcd           = MAX_HOST_HCD;
    user_cfg.host_max_devices       = MAX_HOST_DEVICES;
    user_cfg.host_max_ed            = MAX_HOST_ED;
    user_cfg.host_max_td            = MAX_HOST_TD;
    user_cfg.host_max_iso_td        = MAX_HOST_ISO_TD;
    user_cfg.host_event_callback    = usb_host_app_event_handler;
    user_cfg.host_thread_stack_size = USB_HOST_THREAD_STACK_SIZE;

    status = wiced_usb_host_init( &user_cfg );
    if( status != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("USB20 Host init failed. status=%d\n", status) );
        return;
    }
}

static wiced_result_t usb_host_app_event_handler( uint32_t event, void *param1, void *param2 )
{
    //UX_HOST_CLASS *class = (UX_HOST_CLASS *) param1;
    void *instance = (void *) param2;

    UNUSED_PARAMETER( param1 );

    WPRINT_APP_INFO( ("USB20 Host: event (%ld)\n", event) );

    switch ( event )
    {
        case USB_HOST_EVENT_STORAGE_DEVICE_INSERTION:
            WPRINT_APP_INFO( ("USB_HOST_EVENT_STORAGE_DEVICE_INSERTION\n") );

            /* Retrieve the class instance.  */
            storage = (UX_HOST_CLASS_STORAGE *) instance;
            WPRINT_APP_INFO( ("Found Storage Class device\n") );
            break;

        case USB_HOST_EVENT_STORAGE_DEVICE_REMOVAL:
            WPRINT_APP_INFO( ("USB_HOST_EVENT_STORAGE_DEVICE_REMOVAL\n") );

            /* Check if the storage device is removed. Due to single USB port, now we assume all devices are cleaned at one removal! */
            if ( instance == storage )
            {
                /* Reset pointers to null.  */
                storage = NULL;
            }
            break;

        default :
            WPRINT_APP_INFO( ("USB20 Host: unknown event (%ld)\n", event) );
            break;
    }

    return WICED_SUCCESS;
}

/**
 * USB Host app: Single file read/write test sha1sum
 *
 * Prerequisite:
 *     USB Host Controller: OHCI/EHCI
 *     USB Host Class: Storage
 *     USB device: USB flash disk with one file named ¡§IMAGE.TXT¡¨ under USB disk root directory
 *
 * Notes:
 *     This test will adopt filesystem command for single file read/write test with sha1sum check.
 *     Get a prepared usb flash disk, under USB disk root directory, create one file named ¡§IMAGE.TXT¡¨.
 *     Then plug it into Wiced USB Host, test will start immediately after correct bus enumeraton.
 *
 *     During test, Wiced USB Host reads ¡§IMAGE.TXT¡¨ file from flash disk, and then writes a ¡§COPYFILE.TXT¡¨ file into flash disk.
 *     After file read/write process, SHA-1 checksum will be proceeded between two files to verify if the read/write files are identical.
 *
 *     Filesystem command reports the test result with success or error.
 *     Test stops after above process.
 *
 * @@ USB Bulk transfer!!
 */
int usb_host_app_storage_rw( int argc, char* argv[] )
{
    UINT cnt;

    WPRINT_APP_INFO( ("+++START: single file read/write test sha1sum\n") );

    /* Wait USB disk plugged in, or timeout to error  */
    cnt = USB_HOST_APP_DETECTION_TIMEOUT;
    while ( (storage == NULL) && (cnt > 0) )
    {
        WPRINT_APP_INFO( ("Please plug in USB disk...\n") );
        wiced_rtos_delay_milliseconds( USB_HOST_APP_DEVICE_SCAN_DELAY_TIME );
        cnt --;
        if ( cnt == 0 )
        {
            WPRINT_APP_INFO( ("No disk plugged!\n") );
            goto _error_exit;
        }
    }

    WPRINT_APP_INFO( ("***USB disk ready. Start test...\n") );

    /* USB Host mass storage single file read/write test  */
    if ( file_rw_sha1sum_test(0, NULL) != ERR_CMD_OK )
    {
        WPRINT_APP_INFO( ("File read/write with SHA1sum check failed!\n") );
    }

    WPRINT_APP_INFO( ("---END: single file read/write test sha1sum\n") );
    return ERR_CMD_OK;

_error_exit:
    return ERR_UNKNOWN;
}

/**
 * USB Host app: File read/write throughput measurement
 *
 * Prerequisite:
 *     USB Host Controller: OHCI/EHCI
 *     USB Host Class: Storage
 *     USB device: USB flash disk with prepared single file
 *
 * Notes:
 *     This test will adopt filesystem command for file read/write throughput measurement.
 *     Get a prepared usb flash disk and plug it into Wiced USB Host, test will start immediately after correct bus enumeraton.
 *
 *     During test, Wiced USB Host uses different local buffer sizes to read/write file from/into flash disk.
 *     And the time spent for read/write file is calculated to compute throughput.
 *
 *     Filesystem command reports the throughput result (Mbit/sec) with different burst lengths (8k/16k/32k/64k), or any error.
 *     Test stops after above process, or when encounter any error.
 *
 * @@ USB Bulk transfer!!
 */
int usb_host_app_storage_rw_throughput( int argc, char* argv[] )
{
    UINT cnt;

    WPRINT_APP_INFO( ("+++START: read/write throughput measurement\n") );

    /* Wait USB disk plugged in, or timeout to error  */
    cnt = USB_HOST_APP_DETECTION_TIMEOUT;
    while ( (storage == NULL) && (cnt > 0) )
    {
        WPRINT_APP_INFO( ("Please plug in USB disk...\n") );
        wiced_rtos_delay_milliseconds( USB_HOST_APP_DEVICE_SCAN_DELAY_TIME );
        cnt --;
        if ( cnt == 0 )
        {
            WPRINT_APP_INFO( ("No disk plugged!\n") );
            goto _error_exit;
        }
    }

    WPRINT_APP_INFO( ("***USB disk ready. Start test...\n") );

    file_rw_tput_test( 0, NULL );

    WPRINT_APP_INFO( ("---END: read/write throughput measurement\n") );
    return ERR_CMD_OK;

_error_exit:
    return ERR_UNKNOWN;
}

