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
#include "ux_host_class_cdc_acm.h"
#include "usb_host_cdc_acm_read_write.h"

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

/* Application read/write buffer size definition */
#define MAX_CDC_ACM_RECEPTION_BUFFER_SIZE       (2048)
#define MAX_CDC_ACM_XMIT_BUFFER_SIZE            (2048)
#define MAX_CDC_ACM_RECEPTION_BLOCK_SIZE        (1024)
#define MAX_CDC_ACM_DATA_DUMP_BUFFER_SIZE       (MAX_CDC_ACM_RECEPTION_BUFFER_SIZE+1)

/* CDC-ACM console command string */
#define MAX_CDC_ACM_CONSOLE_CMD_STRING_LEN      (32)

/* CDC-ACM Line Coding config */
#define USB_HOST_CDC_ACM_LINE_CODING_RATE       (115200)
#define USB_HOST_CDC_ACM_LINE_CODING_STOP_BIT   (UX_HOST_CLASS_CDC_ACM_LINE_CODING_STOP_BIT_0)
#define USB_HOST_CDC_ACM_LINE_CODING_PARITY     (UX_HOST_CLASS_CDC_ACM_LINE_CODING_PARITY_NONE)
#define USB_HOST_CDC_ACM_LINE_CODING_DATA_BIT   (8)

/* CDC-ACM usb data read/write transfer timeout config */
#define USB_HOST_CDC_ACM_TRANSFER_TIMEOUT       (300000)

/* ASCII code definition */
#define HEX_ASCII_CHAR_CR                       (0x0D)
#define HEX_ASCII_CHAR_LF                       (0x0A)

/* SHA1 checksum for data verification usage */
#define SHA1_LENGTH                             (20) /* Length of the SHA1 hash */

/* Application timeout for waiting usb device insertion */
#define USB_HOST_APP_DETECTION_TIMEOUT          (30) //sec

/* Application delay time parameters */
#define USB_HOST_APP_DEVICE_SCAN_DELAY_TIME     (1000) //ms
#define USB_HOST_APP_RX_DATA_CHECK_DELAY_TIME   (1) //ms

/* DDR buffer for debug usage (if external DDR available) */
#if !PLATFORM_NO_DDR
#define USB_HOST_SAVE_DEBUG_DATA_IN_DDR         (1)
#else
#define USB_HOST_SAVE_DEBUG_DATA_IN_DDR         (0)
#endif

#define USB_HOST_BUFFER_DDR_OFFSET              (0x01000000) //DDR address at 16M
#define USB_HOST_BUFFER_DDR_CLEAN_SIZE          (0x00800000) //Assume clean DDR for 8M size
#define USB_HOST_LOG_BUFFER_ADDRESS             ((uint8_t *)PLATFORM_DDR_BASE( USB_HOST_BUFFER_DDR_OFFSET ))


/* USB Host console command sets */
#define USB_HOST_APP_COMMANDS \
    { (char*) "console_rw",     usb_host_app_cdc_acm_console_rw,    1, NULL, NULL, (char*) "<cmd string>",  (char*) "cdc-acm console read/write" }, \
    { (char*) "get_file",       usb_host_app_cdc_acm_get_file,      1, NULL, NULL, (char*) "<file size>",   (char*) "cdc-acm get file with sha1sum check" },


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

/* Host CDC-ACM class  */
static UX_HOST_CLASS_CDC_ACM            *cdc_acm[USB_CDC_ACM_DEVICE_HANDLE_MAX];
static uint32_t                         cdc_acm_device_index = 0;
static ULONG                            cdc_acm_command_received_count = 0;
static UX_HOST_CLASS_CDC_ACM_RECEPTION  cdc_acm_reception;
static UCHAR                            *cdc_acm_rx_buffer_ptr = NULL;  /* Must use cache-aligned buffer */
static UCHAR                            *cdc_acm_tx_buffer_ptr = NULL;  /* Must use cache-aligned buffer */
static UCHAR                            *cdc_acm_global_reception_buffer = NULL;
static ULONG                            cdc_acm_global_reception_size = 0;
static ULONG                            cdc_acm_total_rx_length = 0;
static UCHAR                            cdc_acm_data_dump_buffer[MAX_CDC_ACM_DATA_DUMP_BUFFER_SIZE] = {0};
static UCHAR                            cdc_acm_console_cmd_string[MAX_CDC_ACM_CONSOLE_CMD_STRING_LEN+1] = {0};

/* SHA1 checksum for data verification usage */
static sha1_context                     cdc_acm_sha1_ctx;
static uint8_t                          cdc_acm_hash_value[SHA1_LENGTH];

/* DDR buffer for debug usage (if external DDR available) */
#if (USB_HOST_SAVE_DEBUG_DATA_IN_DDR)
static uint8_t                          *usb_host_log_data = USB_HOST_LOG_BUFFER_ADDRESS;
static uint32_t                         usb_host_log_idx = 0;
#endif

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_result_t usb_host_app_event_handler( uint32_t event, void *param1, void *param2 );
static UCHAR* get_cdc_acm_rx_buffer( ULONG buffer_size );
static UCHAR* get_cdc_acm_tx_buffer( ULONG buffer_size );
static void cdc_acm_console_rw_rx_callback( UX_HOST_CLASS_CDC_ACM *cdc_acm, UINT status, UCHAR *reception_buffer, ULONG reception_size );
static void cdc_acm_get_file_rx_callback( UX_HOST_CLASS_CDC_ACM *cdc_acm, UINT status, UCHAR *reception_buffer, ULONG reception_size );

/******************************************************
 *               Function Definitions
 ******************************************************/
void application_start( void )
{
    wiced_usb_user_config_t user_cfg;
    UINT status;

    WPRINT_APP_INFO( ("\n") );
    WPRINT_APP_INFO( ("+-------------------------------------------------+\n") );
    WPRINT_APP_INFO( ("+ USB20 Host CDC-ACM Class Application            +\n") );
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
    UINT i;

    UNUSED_PARAMETER( param1 );

    WPRINT_APP_INFO( ("USB20 Host: event (%ld)\n", event) );

    switch ( event )
    {
        case USB_HOST_EVENT_CDC_ACM_DEVICE_INSERTION:
            WPRINT_APP_INFO( ("USB_HOST_EVENT_CDC_ACM_DEVICE_INSERTION\n") );

            /* Retrieve the class instance.  */
            if ( cdc_acm_device_index < USB_CDC_ACM_DEVICE_HANDLE_MAX )
            {
                cdc_acm[cdc_acm_device_index] = (UX_HOST_CLASS_CDC_ACM *) instance;
                WPRINT_APP_INFO( ("Found CDC-ACM Class device (with index %lu)\n", cdc_acm_device_index) );
                cdc_acm_device_index ++;
            }
            else
            {
                WPRINT_APP_INFO( ("CDC-ACM Class device number exceeds max (%d)!\n", USB_CDC_ACM_DEVICE_HANDLE_MAX) );
            }
            break;

        case USB_HOST_EVENT_CDC_ACM_DEVICE_REMOVAL:
            WPRINT_APP_INFO( ("USB_HOST_EVENT_CDC_ACM_DEVICE_REMOVAL\n") );

            /* Check if the cdc-acm device is removed. Due to single USB port, now we assume all devices are cleaned at one removal! */
            for ( i = 0; i < USB_CDC_ACM_DEVICE_HANDLE_MAX; i++ )
            {
                /* Reset pointers to null.  */
                cdc_acm[i] = NULL;
            }
            cdc_acm_device_index = 0;
            break;

        default :
            WPRINT_APP_INFO( ("USB20 Host: unknown event (%ld)\n", event) );
            break;
    }

    return WICED_SUCCESS;
}

static UCHAR* get_cdc_acm_rx_buffer( ULONG buffer_size )
{
    /* Allocate Align-32 buffer
     * Note: the buffer will not be freed once allocated
     */
    if ( cdc_acm_rx_buffer_ptr == NULL )
    {
        cdc_acm_rx_buffer_ptr = osl_malloc_align( (UINT) buffer_size, (UINT) PLATFORM_L1_CACHE_SHIFT );
    }
    return cdc_acm_rx_buffer_ptr;
}

static UCHAR* get_cdc_acm_tx_buffer( ULONG buffer_size )
{
    /* Allocate Align-32 buffer
     * Note: the buffer will not be freed once allocated
     */
    if ( cdc_acm_tx_buffer_ptr == NULL )
    {
        cdc_acm_tx_buffer_ptr = osl_malloc_align( (UINT) buffer_size, (UINT) PLATFORM_L1_CACHE_SHIFT );
    }
    return cdc_acm_tx_buffer_ptr;
}

static void cdc_acm_console_rw_rx_callback( UX_HOST_CLASS_CDC_ACM *cdc_acm, UINT status, UCHAR *reception_buffer, ULONG reception_size )
{
    if ( status != UX_SUCCESS )
    {
        WPRINT_APP_INFO( ("%s: status=%d\n", __func__, status) );
    }

    /* And move to the next reception buffer.  Check if we are at the end of the application buffer.  */
    if ( cdc_acm_reception.ux_host_class_cdc_acm_reception_data_tail + cdc_acm_reception. ux_host_class_cdc_acm_reception_block_size >=
         cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer + cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer_size )
    {
        /* We are at the end of the buffer. Move back to the beginning.  */
        cdc_acm_reception.ux_host_class_cdc_acm_reception_data_tail = cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer;
    }
    else
    {
        /* Program the tail to be after the current buffer.  */
        cdc_acm_reception.ux_host_class_cdc_acm_reception_data_tail += cdc_acm_reception.ux_host_class_cdc_acm_reception_block_size;
    }

    /* Keep the buffer pointer and length received.  */
    cdc_acm_global_reception_buffer = reception_buffer;
    cdc_acm_global_reception_size = reception_size;

    /* We have received a response.  */
    cdc_acm_command_received_count ++;

    /* Save reception buffer data into data dump buffer */
    memcpy( (void *) cdc_acm_data_dump_buffer, (void *) cdc_acm_global_reception_buffer, cdc_acm_global_reception_size );
    cdc_acm_data_dump_buffer[cdc_acm_global_reception_size] = '\0';

    WPRINT_APP_DEBUG( ("cdc-acm cb: count=%lu, size=%lu, data=%s\n", cdc_acm_command_received_count, cdc_acm_global_reception_size, cdc_acm_data_dump_buffer) );
    WPRINT_APP_INFO( ("%s", cdc_acm_data_dump_buffer) );
}

static void cdc_acm_get_file_rx_callback( UX_HOST_CLASS_CDC_ACM *cdc_acm, UINT status, UCHAR *reception_buffer, ULONG reception_size )
{
    if ( status != UX_SUCCESS )
    {
        WPRINT_APP_INFO( ("%s: status=%d\n", __func__, status) );
    }

    /* And move to the next reception buffer.  Check if we are at the end of the application buffer.  */
    if ( cdc_acm_reception.ux_host_class_cdc_acm_reception_data_tail + cdc_acm_reception. ux_host_class_cdc_acm_reception_block_size >=
         cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer + cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer_size )
    {
        /* We are at the end of the buffer. Move back to the beginning.  */
        cdc_acm_reception.ux_host_class_cdc_acm_reception_data_tail = cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer;
    }
    else
    {
        /* Program the tail to be after the current buffer.  */
        cdc_acm_reception.ux_host_class_cdc_acm_reception_data_tail += cdc_acm_reception.ux_host_class_cdc_acm_reception_block_size;
    }

    /* Keep the buffer pointer and length received.  */
    cdc_acm_global_reception_buffer = reception_buffer;
    cdc_acm_global_reception_size = reception_size;

    /* We have received a response.  */
    cdc_acm_command_received_count ++;
    cdc_acm_total_rx_length += cdc_acm_global_reception_size;

    #if (USB_HOST_SAVE_DEBUG_DATA_IN_DDR)
    /* Save reception buffer data into DDR for debug only!!! */
    memcpy( (void *) (usb_host_log_data + usb_host_log_idx), (void *) cdc_acm_global_reception_buffer, cdc_acm_global_reception_size );
    usb_host_log_idx += cdc_acm_global_reception_size;
    #endif

    /* Update SHA1 checksum by incoming data */
    mbedtls_sha1_update( &cdc_acm_sha1_ctx, (unsigned char *)cdc_acm_global_reception_buffer, cdc_acm_global_reception_size );
}

/**
 * USB Host app: CDC-ACM Console Read/Write (RX/TX)
 *
 * Prerequisite:
 *     USB Host Controller: OHCI/EHCI
 *     USB Host Class: CDC-ACM
 *     USB device: CDC-ACM USB device (with System Log output capability on its USB/CDC-ACM Serial COM Port)
 *
 * Notes:
 *     This test will adopt usb host class driver to verify the console command string (like: ls, ll, pwd...etc) TX to CDC-ACM USB device,
 *     and RX the command response contents those send back from CDC-ACM USB device.
 *     Get a linux-based CDC-ACM USB device. Then plug it into Wiced USB Host, test will start immediately after correct bus enumeraton.
 *     (Before start, please make sure that your CDC-ACM USB device is capable of dumping System Log on its USB/CDC-ACM Serial COM Port)
 *
 *     During test, check USB Host CDC-ACM TX/RX respectively by the following ways:
 *     TX: Test program send user input console command string to CDC-ACM device. Open CDC-ACM device's console program,
 *         then verify the output on CDC-ACM device's console is identical or not.
 *     RX: When CDC-ACM device got console command, it will process command response, dumping any data (like: current file list) to Wiced USB Host.
 *         Test program will dump the received contents on Wiced console.
 *         Then verify the output on Wiced console to see if it is the desired result in CDC-ACM device system.
 *
 *     Test stops after the command response successfully received by 4390x.
 *
 * @@ USB Bulk transfer!!
 */
int usb_host_app_cdc_acm_console_rw( int argc, char* argv[] )
{
    UX_HOST_CLASS_CDC_ACM *cdc_acm_data_instance = NULL;
    UX_HOST_CLASS_CDC_ACM *cdc_acm_control_instance = NULL;
    UCHAR *cdc_acm_reception_buffer = get_cdc_acm_rx_buffer( MAX_CDC_ACM_RECEPTION_BUFFER_SIZE );
    UCHAR *cdc_acm_xmit_buffer = get_cdc_acm_tx_buffer( MAX_CDC_ACM_XMIT_BUFFER_SIZE );
    UX_HOST_CLASS_CDC_ACM_LINE_CODING line_coding = {0};
    UINT cmd_string_len = 0;
    ULONG ioctl_set_value = 0;
    ULONG actual_length = 0;
    UINT status;
    UINT cnt;
    UINT i;

    WPRINT_APP_INFO( ("+++START: cdc-acm console read/write\n") );

    /* Check valid cmd string and store it */
    if ( argc != 2 )
    {
        WPRINT_APP_INFO( ("Invalid cmd string! Only support single cmd string without space!\n") );
        goto _error_exit;
    }

    cmd_string_len = strlen( argv[1] );
    if ( cmd_string_len > MAX_CDC_ACM_CONSOLE_CMD_STRING_LEN )
    {
        WPRINT_APP_INFO( ("Cmd string too long! Only support string length under %d char\n", MAX_CDC_ACM_CONSOLE_CMD_STRING_LEN) );
        goto _error_exit;
    }
    memset( (void *) cdc_acm_console_cmd_string, 0, sizeof(cdc_acm_console_cmd_string) );
    memcpy( (UCHAR *) cdc_acm_console_cmd_string, (UCHAR *) (argv[1]), cmd_string_len );
    cdc_acm_console_cmd_string[cmd_string_len] = '\0';

    /* Wait USB cdc-acm plugged in, or timeout to error  */
    cnt = USB_HOST_APP_DETECTION_TIMEOUT;
    while ( (cdc_acm[0] == NULL) && (cnt > 0) )
    {
        WPRINT_APP_INFO( ("Please plug in USB cdc-acm device...\n") );
        wiced_rtos_delay_milliseconds( USB_HOST_APP_DEVICE_SCAN_DELAY_TIME );
        cnt --;
        if (cnt == 0)
        {
            WPRINT_APP_INFO( ("No cdc-acm device plugged!\n") );
            goto _error_exit;
        }
    }

    WPRINT_APP_INFO( ("***USB cdc-acm ready. Sending cdc-acm console command string to device...\n") );

    /*
     * Search for cdc-acm device instance array for:
     * 1. Instance with Data Class Interface: ---<Mandatory>--- : Data path, with 1 x Bulk IN Endpoint & 1 x Bulk OUT Endpoint
     * 2. Instance with Control Class Interface: ---<Optional>--- : Control path, with 1 x Interrupt IN Endpoint
     *
     * PS: Ioctl with "PRIVATE" type is in spite of instance's interface class!
     */
    for ( i = 0; i < USB_CDC_ACM_DEVICE_HANDLE_MAX; i++ )
    {
        if ( (cdc_acm[i] != NULL) && (cdc_acm[i] -> ux_host_class_cdc_acm_interface -> ux_interface_descriptor.bInterfaceClass == UX_HOST_CLASS_CDC_DATA_CLASS) )
        {
            cdc_acm_data_instance = cdc_acm[i];
            WPRINT_APP_INFO( ("Found USB cdc-acm DATA instance (with index %u)\n", i) );
            break;
        }
    }
    if ( cdc_acm_data_instance == NULL )
    {
        /* Mandatory, return error */
        WPRINT_APP_INFO( ("No USB cdc-acm DATA instance !!!\n") );
        goto _error_exit;
    }

    for ( i = 0; i < USB_CDC_ACM_DEVICE_HANDLE_MAX; i++ )
    {
        if ( (cdc_acm[i] != NULL) && (cdc_acm[i] -> ux_host_class_cdc_acm_interface -> ux_interface_descriptor.bInterfaceClass == UX_HOST_CLASS_CDC_CONTROL_CLASS) )
        {
            cdc_acm_control_instance = cdc_acm[i];
            WPRINT_APP_INFO( ("Found USB cdc-acm CONTROL instance (with index %u)\n", i) );
            break;
        }
    }
    if ( cdc_acm_control_instance == NULL )
    {
        /* Optional, just need a notice */
        WPRINT_APP_INFO( ("No USB cdc-acm CONTROL instance !!!\n") );
    }

    /*
     * Config cdc-cam transfer timeout
     * - Currently used only in Data Class Interface read/write data transfer
     *
     * This is optional to set user cdc-acm transfer timeout
     * Without config, the default cdc-cam transfer timeout UX_HOST_CLASS_CDC_ACM_CLASS_TRANSFER_TIMEOUT_DEFAULT will be used!
     */
    ioctl_set_value = USB_HOST_CDC_ACM_TRANSFER_TIMEOUT;
    status = ux_host_class_cdc_acm_ioctl( cdc_acm_data_instance, UX_HOST_CLASS_CDC_ACM_PRIVATE_IOCTL_SET_TRANSFER_TIMEOUT,(void *) &ioctl_set_value );
    if ( status != UX_SUCCESS )
    {
        WPRINT_APP_INFO( ("cdc-acm set transfer timeout failed. status=%d\n", status) );
        goto _error_exit;
    }

    /*
     * Config cdc-cam line coding values to device
     * - Specific to Control Class Interface
     *
     * This is optional to set user cdc-acm line coding values to device
     * Without config, the default cdc-cam line coding values will be used!
     */
    if ( cdc_acm_control_instance != NULL )
    {
        line_coding.ux_host_class_cdc_acm_line_coding_dter      = USB_HOST_CDC_ACM_LINE_CODING_RATE;
        line_coding.ux_host_class_cdc_acm_line_coding_stop_bit  = USB_HOST_CDC_ACM_LINE_CODING_STOP_BIT;
        line_coding.ux_host_class_cdc_acm_line_coding_parity    = USB_HOST_CDC_ACM_LINE_CODING_PARITY;
        line_coding.ux_host_class_cdc_acm_line_coding_data_bits = USB_HOST_CDC_ACM_LINE_CODING_DATA_BIT;

        status = ux_host_class_cdc_acm_ioctl( cdc_acm_control_instance, UX_HOST_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING, (void *) &line_coding );
        if ( status != UX_SUCCESS )
        {
            WPRINT_APP_INFO( ("cdc-acm set line coding failed. status=%d\n", status) );
            goto _error_exit;
        }
    }

    /* Check cdc-acm buffer */
    if ( ( cdc_acm_reception_buffer == NULL ) || ( cdc_acm_xmit_buffer == NULL ) )
    {
        WPRINT_APP_INFO( ("cdc-acm buffer alloc error!\n") );
        goto _error_exit;
    }

    /* Clean cdc-acm buffer */
    memset( (void *) cdc_acm_reception_buffer, 0, MAX_CDC_ACM_RECEPTION_BUFFER_SIZE );
    memset( (void *) cdc_acm_xmit_buffer, 0, MAX_CDC_ACM_XMIT_BUFFER_SIZE );

    /* Reset counter */
    cdc_acm_command_received_count = 0;

    /* Start the reception for cdc-acm.  */
    if ( cdc_acm_reception.ux_host_class_cdc_acm_reception_state == UX_HOST_CLASS_CDC_ACM_RECEPTION_STATE_STOPPED )
    {
        /*
         * Be noted that:
         * 1. The data_buffer address assigned for reception must be at cache-aligned address!!!
         * 2. The block_size is the request_length for one USB transfer request
         * 3. The callback is called when a full or partial USB transfer has been done for a Bulk IN transfer
         */
        cdc_acm_reception.ux_host_class_cdc_acm_reception_block_size = MAX_CDC_ACM_RECEPTION_BLOCK_SIZE;
        cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer = cdc_acm_reception_buffer;
        cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer_size = MAX_CDC_ACM_RECEPTION_BUFFER_SIZE;
        cdc_acm_reception.ux_host_class_cdc_acm_reception_callback = cdc_acm_console_rw_rx_callback;

        WPRINT_APP_INFO( ("cdc-acm reception_start\n") );
        status = _ux_host_class_cdc_acm_reception_start( cdc_acm_data_instance, &cdc_acm_reception );
        if ( status != UX_SUCCESS )
        {
            WPRINT_APP_INFO( ("cdc-acm reception start failed. status=%d\n", status) );
            goto _error_exit;
        }
    }

    /* Perform a write to the modem. And wait for the answer. */
    WPRINT_APP_INFO( ("Send console cmd: %s\n", cdc_acm_console_cmd_string) );
    memcpy( (UCHAR *) cdc_acm_xmit_buffer, (UCHAR *) cdc_acm_console_cmd_string, cmd_string_len );
    cdc_acm_xmit_buffer[cmd_string_len] = HEX_ASCII_CHAR_CR;

    /* Send the AT command.  */
    status = ux_host_class_cdc_acm_write( cdc_acm_data_instance, cdc_acm_xmit_buffer, (cmd_string_len+1), &actual_length );
    if ( status != UX_SUCCESS )
    {
        WPRINT_APP_INFO( ("cdc-acm write failed. status=%d\n", status) );
        goto _error_exit;
    }

    /* Wait for the answer.  */
    while( cdc_acm_command_received_count == 0 )
    {
        wiced_rtos_delay_milliseconds( USB_HOST_APP_RX_DATA_CHECK_DELAY_TIME );
    }

    /* Reset the status count.  */
    cdc_acm_command_received_count = 0;

    WPRINT_APP_INFO( ("---END: cdc-acm console read/write\n") );
    return ERR_CMD_OK;

_error_exit:
    return ERR_UNKNOWN;
}

/**
 * USB Host app: CDC-ACM Get File with SHA1SUM Check
 *
 * Prerequisite:
 *     USB Host Controller: OHCI/EHCI
 *     USB Host Class: CDC-ACM
 *     USB device: CDC-ACM USB device (with File Send capability on its USB/CDC-ACM Serial COM Port)
 *     SHA1 Checksum of Input File: http://onlinemd5.com/
 *
 * Notes:
 *     This test will adopt usb host class driver to verify the binary data (in file format) RX from CDC-ACM USB device.
 *     Get a linux-based CDC-ACM USB device. Before start, please make sure that your CDC-ACM USB device
 *     is capable of File Send on its USB/CDC-ACM Serial COM Port. Then plug it into Wiced USB Host.
 *     Usually, CDC-ACM device will have another Serial COM Port (or in form of USB-RS232) for system debug purpose.
 *
 *     You may achieve this test by using only one x86 Win7 PC with two COM ports and two TeraTerm window. Ex:
 *     1. Plug CDC-ACM device debug UART into another x86 PC RS232 serial COM port, assumed COM1
 *     2. Plug 4390x Wiced debug UART into x86 PC RS232 serial COM port, assumed COM2
 *     3. After correct bus enumeraton, 4390x execute this command function (on COM2 TeraTerm) to wait for the data sent from CDC-ACM device.
 *     4. On COM1 TeraTerm function bar, select [File] -> [Send file...] to input your file with "Binary" checkbox checked.
 *     5. If you're sending printable ascii codes in file, please be caution to the new line (CR & LF) handle of your TeraTerm.
 *     6. Test will start immediately and the data will be sent from CDC-ACM usb device and received by 4390x usb host.
 *
 *     During test, check USB Host CDC-ACM RX by the following ways:
 *     RX: The program is designed to get the file sent from CDC-ACM device.
 *         Test program will calculate the SHA1 Checksum by collecting all incoming data.
 *         When total received data length equals to your input size when executing this command, program will print out SHA1 Checksum result.
 *         Then verify the SHA1 Checksum result showed on Wiced debug console to see if it is correct.
 *
 *     Test stops after the file successfully received by 4390x and dumped SHA1 Checksum result.
 *
 * @@ USB Bulk transfer!!
 */
int usb_host_app_cdc_acm_get_file( int argc, char* argv[] )
{
    UX_HOST_CLASS_CDC_ACM *cdc_acm_data_instance = NULL;
    UX_HOST_CLASS_CDC_ACM *cdc_acm_control_instance = NULL;
    UCHAR *cdc_acm_reception_buffer = get_cdc_acm_rx_buffer( MAX_CDC_ACM_RECEPTION_BUFFER_SIZE );
    UCHAR *cdc_acm_xmit_buffer = get_cdc_acm_tx_buffer( MAX_CDC_ACM_XMIT_BUFFER_SIZE );
    UX_HOST_CLASS_CDC_ACM_LINE_CODING line_coding = {0};
    ULONG in_file_size = atol( argv[1] );
    ULONG ioctl_set_value = 0;
    UINT status;
    UINT cnt;
    UINT i;

    WPRINT_APP_INFO( ("+++START: cdc-acm get file with sha1sum check (input file: %lu bytes)\n", in_file_size) );

    /* Check valid file size */
    if ( in_file_size == 0 )
    {
        WPRINT_APP_INFO( ("Invalid file size!\n") );
        goto _error_exit;
    }

    /* Wait USB cdc-acm plugged in, or timeout to error  */
    cnt = USB_HOST_APP_DETECTION_TIMEOUT;
    while ( (cdc_acm[0] == NULL) && (cnt > 0) )
    {
        WPRINT_APP_INFO( ("Please plug in USB cdc-acm device...\n") );
        wiced_rtos_delay_milliseconds( USB_HOST_APP_DEVICE_SCAN_DELAY_TIME );
        cnt --;
        if ( cnt == 0 )
        {
            WPRINT_APP_INFO( ("No cdc-acm device plugged!\n") );
            goto _error_exit;
        }
    }

    WPRINT_APP_INFO( ("***USB cdc-acm ready. Send file from your cdc-acm device terminal to start...\n") );

    /*
     * Search for cdc-acm device instance array for:
     * 1. Instance with Data Class Interface: ---<Mandatory>--- : Data path, with 1 x Bulk IN Endpoint & 1 x Bulk OUT Endpoint
     * 2. Instance with Control Class Interface: ---<Optional>--- : Control path, with 1 x Interrupt IN Endpoint
     *
     * PS: Ioctl with "PRIVATE" type is in spite of instance's interface class!
     */
    for ( i = 0; i < USB_CDC_ACM_DEVICE_HANDLE_MAX; i++ )
    {
        if ( (cdc_acm[i] != NULL) && (cdc_acm[i] -> ux_host_class_cdc_acm_interface -> ux_interface_descriptor.bInterfaceClass == UX_HOST_CLASS_CDC_DATA_CLASS) )
        {
            cdc_acm_data_instance = cdc_acm[i];
            WPRINT_APP_INFO( ("Found USB cdc-acm DATA instance (with index %u)\n", i) );
            break;
        }
    }
    if ( cdc_acm_data_instance == NULL )
    {
        /* Mandatory, return error */
        WPRINT_APP_INFO( ("No USB cdc-acm DATA instance !!!\n") );
        goto _error_exit;
    }

    for ( i = 0; i < USB_CDC_ACM_DEVICE_HANDLE_MAX; i++ )
    {
        if ( (cdc_acm[i] != NULL) && (cdc_acm[i] -> ux_host_class_cdc_acm_interface -> ux_interface_descriptor.bInterfaceClass == UX_HOST_CLASS_CDC_CONTROL_CLASS) )
        {
            cdc_acm_control_instance = cdc_acm[i];
            WPRINT_APP_INFO( ("Found USB cdc-acm CONTROL instance (with index %u)\n", i) );
            break;
        }
    }
    if ( cdc_acm_control_instance == NULL )
    {
        /* Optional, just need a notice */
        WPRINT_APP_INFO( ("No USB cdc-acm CONTROL instance !!!\n") );
    }

    /*
     * Config cdc-cam transfer timeout
     * - Currently used only in Data Class Interface read/write data transfer
     *
     * This is optional to set user cdc-acm transfer timeout
     * Without config, the default cdc-cam transfer timeout UX_HOST_CLASS_CDC_ACM_CLASS_TRANSFER_TIMEOUT_DEFAULT will be used!
     */
    ioctl_set_value = USB_HOST_CDC_ACM_TRANSFER_TIMEOUT;
    status = ux_host_class_cdc_acm_ioctl( cdc_acm_data_instance, UX_HOST_CLASS_CDC_ACM_PRIVATE_IOCTL_SET_TRANSFER_TIMEOUT,(void *) &ioctl_set_value );
    if ( status != UX_SUCCESS )
    {
        WPRINT_APP_INFO( ("cdc-acm set transfer timeout failed. status=%d\n", status) );
        goto _error_exit;
    }

    /*
     * Config cdc-cam line coding values to device
     * - Specific to Control Class Interface
     *
     * This is optional to set user cdc-acm line coding values to device
     * Without config, the default cdc-cam line coding values will be used!
     */
    if ( cdc_acm_control_instance != NULL )
    {
        line_coding.ux_host_class_cdc_acm_line_coding_dter      = USB_HOST_CDC_ACM_LINE_CODING_RATE;
        line_coding.ux_host_class_cdc_acm_line_coding_stop_bit  = USB_HOST_CDC_ACM_LINE_CODING_STOP_BIT;
        line_coding.ux_host_class_cdc_acm_line_coding_parity    = USB_HOST_CDC_ACM_LINE_CODING_PARITY;
        line_coding.ux_host_class_cdc_acm_line_coding_data_bits = USB_HOST_CDC_ACM_LINE_CODING_DATA_BIT;

        status = ux_host_class_cdc_acm_ioctl( cdc_acm_control_instance, UX_HOST_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING, (void *) &line_coding );
        if ( status != UX_SUCCESS )
        {
            WPRINT_APP_INFO( ("cdc-acm set line coding failed. status=%d\n", status) );
            goto _error_exit;
        }
    }

    /* Check cdc-acm buffer */
    if ( ( cdc_acm_reception_buffer == NULL ) || ( cdc_acm_xmit_buffer == NULL ) )
    {
        WPRINT_APP_INFO( ("cdc-acm buffer alloc error!\n") );
        goto _error_exit;
    }

    /* Clean cdc-acm buffer */
    memset( (void *) cdc_acm_reception_buffer, 0, MAX_CDC_ACM_RECEPTION_BUFFER_SIZE );
    memset( (void *) cdc_acm_xmit_buffer, 0, MAX_CDC_ACM_XMIT_BUFFER_SIZE );

    #if (USB_HOST_SAVE_DEBUG_DATA_IN_DDR)
    /* Reset DDR buffer address from start */
    usb_host_log_idx = 0;

    /* Clean DDR buffer from start */
    memset( (void *) usb_host_log_data, 0, USB_HOST_BUFFER_DDR_CLEAN_SIZE );
    #endif

    /* Init and start SHA1 checksum */
    mbedtls_sha1_starts( &cdc_acm_sha1_ctx );

    /* Reset counter */
    cdc_acm_total_rx_length = 0;
    cdc_acm_command_received_count = 0;

    /* Start the reception for cdc-acm.  */
    if ( cdc_acm_reception.ux_host_class_cdc_acm_reception_state == UX_HOST_CLASS_CDC_ACM_RECEPTION_STATE_STOPPED )
    {
        /*
         * Be noted that:
         * 1. The data_buffer address assigned for reception must be at cache-aligned address!!!
         * 2. The block_size is the request_length for one USB transfer request
         * 3. The callback is called when a full or partial USB transfer has been done for a Bulk IN transfer
         */
        cdc_acm_reception.ux_host_class_cdc_acm_reception_block_size = MAX_CDC_ACM_RECEPTION_BLOCK_SIZE;
        cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer = cdc_acm_reception_buffer;
        cdc_acm_reception.ux_host_class_cdc_acm_reception_data_buffer_size = MAX_CDC_ACM_RECEPTION_BUFFER_SIZE;
        cdc_acm_reception.ux_host_class_cdc_acm_reception_callback = cdc_acm_get_file_rx_callback;

        WPRINT_APP_INFO( ("cdc-acm reception_start\n") );
        status = _ux_host_class_cdc_acm_reception_start( cdc_acm_data_instance, &cdc_acm_reception );
        if ( status != UX_SUCCESS )
        {
            WPRINT_APP_INFO( ("cdc-acm reception start failed. status=%d\n", status) );
            goto _error_exit;
        }
    }

    /* Main loop  */
    while ( 1 )
    {
        /* Wait data callback from reception */
        while( cdc_acm_command_received_count == 0 )
        {
            wiced_rtos_delay_milliseconds( USB_HOST_APP_RX_DATA_CHECK_DELAY_TIME );
        }

        /* Compute the final SHA1 checksum value after all data received */
        if ( cdc_acm_total_rx_length == in_file_size )
        {
            mbedtls_sha1_finish( &cdc_acm_sha1_ctx, (unsigned char *)cdc_acm_hash_value );

            WPRINT_APP_INFO( ("\n\nGet file with SHA1 Checksum [") );
            for ( i=0; i<SHA1_LENGTH; i++ )
            {
                WPRINT_APP_INFO( ("%02X", cdc_acm_hash_value[i]) );
            }
            WPRINT_APP_INFO( ("]\n") );

            break;
        }

        /* Reset the status count.  */
        cdc_acm_command_received_count = 0;
    }

    /* Stop the reception for cdc-acm when out of main loop */
    if ( cdc_acm_reception.ux_host_class_cdc_acm_reception_state != UX_HOST_CLASS_CDC_ACM_RECEPTION_STATE_STOPPED )
    {
        WPRINT_APP_INFO( ("cdc-acm reception_stop\n") );
        status = _ux_host_class_cdc_acm_reception_stop( cdc_acm_data_instance, &cdc_acm_reception );
        if ( status != UX_SUCCESS )
        {
            WPRINT_APP_INFO( ("cdc-acm reception stop failed. status=%d\n", status) );
            goto _error_exit;
        }
    }

    WPRINT_APP_INFO( ("---END: cdc-acm get file with sha1sum check\n") );
    return ERR_CMD_OK;

_error_exit:
    return ERR_UNKNOWN;
}

