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
#include "wiced_management.h"
#include "wiced_resource.h"
#include "wiced_usb.h"
#include "ux_api.h"
#include "ux_hcd_ehci.h"
#include "ux_hcd_ohci.h"
#include "ux_host_class_hub.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_mouse.h"
#include "usb_host_hid_mouse.h"

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
#define USB_HOST_APP_MOUSE_KEY_SCAN_DELAY_TIME  (300) //ms


/* USB Host console command sets */
#define USB_HOST_APP_COMMANDS \
    { (char*) "hid_mouse",      usb_host_app_hid_mouse,             0, NULL, NULL, (char*) "",              (char*) "hid mouse test" },


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
    CMD_TABLE_END
};

/* Host HID class  */
static UX_HOST_CLASS_HID            *hid = NULL;
static UX_HOST_CLASS_HID_CLIENT     *hid_client = NULL;
static UX_HOST_CLASS_HID_MOUSE      *mouse = NULL;

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
    WPRINT_APP_INFO( ("+ USB20 Host HID Mouse Class Application          +\n") );
    WPRINT_APP_INFO( ("+-------------------------------------------------+\n") );

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
        case USB_HOST_EVENT_HID_DEVICE_INSERTION:
            WPRINT_APP_INFO( ("USB_HOST_EVENT_HID_DEVICE_INSERTION\n") );

            /* Retrieve the class instance.  */
            hid = (UX_HOST_CLASS_HID *) instance;

            /* Get the HID client */
            hid_client = hid -> ux_host_class_hid_client;

            /* Ensure the client is valid.  */
            if ( hid_client != UX_NULL )
            {
                /* Ensure the client instance is live.  */
                if ( hid_client -> ux_host_class_hid_client_local_instance != UX_NULL )
                {
                    /* Check the class if this is a Keyboard HID device.  */
                    /* Check the class if this is a Mouse HID device.  */
                    if ( ux_utility_memory_compare(hid_client -> ux_host_class_hid_client_name, _ux_system_host_class_hid_client_mouse_name,
                        ux_utility_string_length_get( _ux_system_host_class_hid_client_mouse_name ) ) == UX_SUCCESS )
                    {
                        /* We have found the mouse.  */
                        mouse = (UX_HOST_CLASS_HID_MOUSE *) hid_client -> ux_host_class_hid_client_local_instance;
                        WPRINT_APP_INFO( ("Found Mouse HID Class device\n") );
                    }
                    else
                    {
                        WPRINT_APP_INFO( ("Unknown HID Class device\n") );
                    }
                }
            }
            break;

        case USB_HOST_EVENT_HID_DEVICE_REMOVAL:
            WPRINT_APP_INFO( ("USB_HOST_EVENT_HID_DEVICE_REMOVAL\n") );

            /* Check if the hid device is removed. Due to single USB port, now we assume all devices are cleaned at one removal! */
            if ( instance == hid )
            {
                /* Reset pointers to null.  */
                hid = NULL;
                hid_client = NULL;
                mouse = NULL;
            }
            break;

        default :
            WPRINT_APP_INFO( ("USB20 Host: unknown event (%ld)\n", event) );
            break;
    }

    return WICED_SUCCESS;
}

/**
 * USB Host app: HID mouse test
 *
 * Prerequisite:
 *     USB Host Controller: OHCI/EHCI
 *     USB Host Class: HID (with mouse client)
 *     USB device: USB HID mouse
 *
 * Notes:
 *     This test will adopt usb host class driver to get usb mouse position and key input by user.
 *     Get a usb mouse and plug it into Wiced USB Host, test will start immediately after correct bus enumeraton.
 *
 *     During test, Wiced console will scan and dump the XY-axis position and user pressed button from mouse.
 *     User may check if the output position and button are identical to those you had pressed to verify the result.
 *
 *     Test stops after mouse unplugged.
 *
 * @@ USB Interrupt transfer!!
 */
int usb_host_app_hid_mouse( int argc, char* argv[] )
{
    ULONG mouse_buttons = 0;
    SLONG mouse_x_position = 0;
    SLONG mouse_y_position = 0;
    UINT status;
    UINT cnt;

    WPRINT_APP_INFO( ("+++START: hid mouse test\n") );

    /* Wait USB mouse plugged in, or timeout to error  */
    cnt = USB_HOST_APP_DETECTION_TIMEOUT;
    while ( (mouse == NULL) && (cnt > 0) )
    {
        WPRINT_APP_INFO( ("Please plug in USB mouse...\n") );
        wiced_rtos_delay_milliseconds( USB_HOST_APP_DEVICE_SCAN_DELAY_TIME );
        cnt --;
        if ( cnt == 0 )
        {
            WPRINT_APP_INFO( ("No mouse plugged!\n") );
            goto _error_exit;
        }
    }

    WPRINT_APP_INFO( ("***USB mouse ready. Move your mouse, or unplug mouse to stop test...\n") );

    /* Main loop  */
    while ( 1 )
    {
        /* Get the mouse position.  */
        status = ux_host_class_hid_mouse_position_get( mouse, &mouse_x_position, &mouse_y_position );
        if ( status == UX_SUCCESS )
        {
            WPRINT_APP_INFO( ("Mouse: X-(%ld) Y-(%ld)\n", mouse_x_position, mouse_y_position) );
        }

        /* Get the mouse buttons.  */
        status = ux_host_class_hid_mouse_buttons_get( mouse, &mouse_buttons );
        if ( status == UX_SUCCESS )
        {
            WPRINT_APP_INFO( ("Mouse: Button-(0x%08lx)\n", mouse_buttons) );
        }

        wiced_rtos_delay_milliseconds( USB_HOST_APP_MOUSE_KEY_SCAN_DELAY_TIME );

        /* Check for test stop.  */
        if ( mouse == NULL )
        {
            break;
        }
    }

    WPRINT_APP_INFO( ("---END: hid mouse test\n") );
    return ERR_CMD_OK;

_error_exit:
    return ERR_UNKNOWN;
}

