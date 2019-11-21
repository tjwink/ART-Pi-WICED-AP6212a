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
#include "ux_host_class_prolific.h"
#include "usb_host_pl2303_read_write.h"

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
#define MAX_PROLIFIC_RECEPTION_BUFFER_SIZE      (512)
#define MAX_PROLIFIC_XMIT_BUFFER_SIZE           (512)
#define MAX_PROLIFIC_RECEPTION_BLOCK_SIZE       (64)
#define MAX_PROLIFIC_DATA_DUMP_BUFFER_SIZE      (MAX_PROLIFIC_RECEPTION_BUFFER_SIZE+1)

/* Prolific Line Coding config */
#define USB_HOST_PROLIFIC_LINE_CODING_RATE      (115200)
#define USB_HOST_PROLIFIC_LINE_CODING_STOP_BIT  (UX_HOST_CLASS_PROLIFIC_LINE_CODING_STOP_BIT_0)
#define USB_HOST_PROLIFIC_LINE_CODING_PARITY    (UX_HOST_CLASS_PROLIFIC_LINE_CODING_PARITY_NONE)
#define USB_HOST_PROLIFIC_LINE_CODING_DATA_BIT  (8)

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
    { (char*) "console_rw",     usb_host_app_pl2303_console_rw,     0, NULL, NULL, (char*) "",              (char*) "prolific console read/write loopback" }, \
    { (char*) "get_file",       usb_host_app_pl2303_get_file,       1, NULL, NULL, (char*) "<file size>",   (char*) "prolific get file with sha1sum check" },


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

/* Host Prolific PL2303 (CDC-ACM Like) class  */
static UX_HOST_CLASS_PROLIFIC           *prolific;
static ULONG                            prolific_command_received_count = 0;
static UX_HOST_CLASS_PROLIFIC_RECEPTION prolific_reception;
static UCHAR                            *prolific_rx_buffer_ptr = NULL; /* Must use cache-aligned buffer */
static UCHAR                            *prolific_tx_buffer_ptr = NULL; /* Must use cache-aligned buffer */
static UCHAR                            *prolific_global_reception_buffer = NULL;
static ULONG                            prolific_global_reception_size = 0;
static ULONG                            prolific_total_rx_length = 0;
static UCHAR                            prolific_data_dump_buffer[MAX_PROLIFIC_DATA_DUMP_BUFFER_SIZE] = {0};

/* SHA1 checksum for data verification usage */
static sha1_context                     prolific_sha1_ctx;
static uint8_t                          prolific_hash_value[SHA1_LENGTH];

/* DDR buffer for debug usage (if external DDR available) */
#if (USB_HOST_SAVE_DEBUG_DATA_IN_DDR)
static uint8_t                          *usb_host_log_data = USB_HOST_LOG_BUFFER_ADDRESS;
static uint32_t                         usb_host_log_idx = 0;
#endif

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_result_t usb_host_app_event_handler( uint32_t event, void *param1, void *param2 );
static UCHAR* get_prolific_rx_buffer( ULONG buffer_size );
static UCHAR* get_prolific_tx_buffer( ULONG buffer_size );
static void pl2303_console_rw_rx_callback( UX_HOST_CLASS_PROLIFIC *prolific, UINT status, UCHAR *reception_buffer, ULONG reception_size );
static void pl2303_get_file_rx_callback( UX_HOST_CLASS_PROLIFIC *prolific, UINT status, UCHAR *reception_buffer, ULONG reception_size );

/******************************************************
 *               Function Definitions
 ******************************************************/
void application_start( void )
{
    wiced_usb_user_config_t user_cfg;
    UINT status;

    WPRINT_APP_INFO( ("\n") );
    WPRINT_APP_INFO( ("+-------------------------------------------------+\n") );
    WPRINT_APP_INFO( ("+ USB20 Host Prolific (PL2303) Class Application  +\n") );
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
        case USB_HOST_EVENT_PROLIFIC_DEVICE_INSERTION:
            WPRINT_APP_INFO( ("USB_HOST_EVENT_PROLIFIC_DEVICE_INSERTION\n") );

            /* Retrieve the class instance.  */
            prolific = (UX_HOST_CLASS_PROLIFIC *) instance;
            WPRINT_APP_INFO( ("Found Prolific PL2303 Class device\n") );
            break;

        case USB_HOST_EVENT_PROLIFIC_DEVICE_REMOVAL:
            WPRINT_APP_INFO( ("USB_HOST_EVENT_PROLIFIC_DEVICE_REMOVAL\n") );

            /* Check if the storage device is removed. Due to single USB port, now we assume all devices are cleaned at one removal! */
            if ( instance == prolific )
            {
                /* Reset pointers to null.  */
                prolific = NULL;
            }
            break;

        default :
            WPRINT_APP_INFO( ("USB20 Host: unknown event (%ld)\n", event) );
            break;
    }

    return WICED_SUCCESS;
}

static UCHAR* get_prolific_rx_buffer( ULONG buffer_size )
{
    /* Allocate Align-32 buffer
     * Note: the buffer will not be freed once allocated
     */
    if ( prolific_rx_buffer_ptr == NULL )
    {
        prolific_rx_buffer_ptr = osl_malloc_align( (UINT) buffer_size, (UINT) PLATFORM_L1_CACHE_SHIFT );
    }
    return prolific_rx_buffer_ptr;
}

static UCHAR* get_prolific_tx_buffer( ULONG buffer_size )
{
    /* Allocate Align-32 buffer
     * Note: the buffer will not be freed once allocated
     */
    if ( prolific_tx_buffer_ptr == NULL )
    {
        prolific_tx_buffer_ptr = osl_malloc_align( (UINT) buffer_size, (UINT) PLATFORM_L1_CACHE_SHIFT );
    }
    return prolific_tx_buffer_ptr;
}

static void pl2303_console_rw_rx_callback( UX_HOST_CLASS_PROLIFIC *prolific, UINT status, UCHAR *reception_buffer, ULONG reception_size )
{
    if ( status != UX_SUCCESS )
    {
        WPRINT_APP_INFO( ("%s: status=%d\n", __func__, status) );
    }

    /* And move to the next reception buffer.  Check if we are at the end of the application buffer.  */
    if ( prolific_reception.ux_host_class_prolific_reception_data_tail + prolific_reception. ux_host_class_prolific_reception_block_size >=
         prolific_reception.ux_host_class_prolific_reception_data_buffer + prolific_reception.ux_host_class_prolific_reception_data_buffer_size )
    {
        /* We are at the end of the buffer. Move back to the beginning.  */
        prolific_reception.ux_host_class_prolific_reception_data_tail = prolific_reception.ux_host_class_prolific_reception_data_buffer;
    }
    else
    {
        /* Program the tail to be after the current buffer.  */
        prolific_reception.ux_host_class_prolific_reception_data_tail += prolific_reception.ux_host_class_prolific_reception_block_size;
    }

    /* Keep the buffer pointer and length received.  */
    prolific_global_reception_buffer = reception_buffer;
    prolific_global_reception_size = reception_size;

    /* We have received a response.  */
    prolific_command_received_count ++;

    /* Save reception buffer data into data dump buffer */
    memcpy( (void *) prolific_data_dump_buffer, (void *) prolific_global_reception_buffer, prolific_global_reception_size );
    prolific_data_dump_buffer[prolific_global_reception_size] = '\0';

    WPRINT_APP_INFO( ("prolific cb: count=%lu, size=%lu, data=%s\n", prolific_command_received_count, prolific_global_reception_size, prolific_data_dump_buffer) );
}

static void pl2303_get_file_rx_callback( UX_HOST_CLASS_PROLIFIC *prolific, UINT status, UCHAR *reception_buffer, ULONG reception_size )
{
    if ( status != UX_SUCCESS )
    {
        WPRINT_APP_INFO( ("%s: status=%d\n", __func__, status) );
    }

    /* And move to the next reception buffer.  Check if we are at the end of the application buffer.  */
    if ( prolific_reception.ux_host_class_prolific_reception_data_tail + prolific_reception. ux_host_class_prolific_reception_block_size >=
         prolific_reception.ux_host_class_prolific_reception_data_buffer + prolific_reception.ux_host_class_prolific_reception_data_buffer_size )
    {
        /* We are at the end of the buffer. Move back to the beginning.  */
        prolific_reception.ux_host_class_prolific_reception_data_tail = prolific_reception.ux_host_class_prolific_reception_data_buffer;
    }
    else
    {
        /* Program the tail to be after the current buffer.  */
        prolific_reception.ux_host_class_prolific_reception_data_tail += prolific_reception.ux_host_class_prolific_reception_block_size;
    }

    /* Keep the buffer pointer and length received.  */
    prolific_global_reception_buffer = reception_buffer;
    prolific_global_reception_size = reception_size;

    /* We have received a response.  */
    prolific_command_received_count ++;
    prolific_total_rx_length += prolific_global_reception_size;

    #if (USB_HOST_SAVE_DEBUG_DATA_IN_DDR)
    /* Save reception buffer data into DDR for debug only!!! */
    memcpy( (void *) (usb_host_log_data + usb_host_log_idx), (void *) prolific_global_reception_buffer, prolific_global_reception_size );
    usb_host_log_idx += prolific_global_reception_size;
    #endif

    /* Update SHA1 checksum by incoming data */
    mbedtls_sha1_update( &prolific_sha1_ctx, (unsigned char *)prolific_global_reception_buffer, prolific_global_reception_size );
}

/**
 * USB Host app: Prolific PL2303 Console Read/Write (RX/TX) Loopback
 *
 * Prerequisite:
 *     USB Host Controller: OHCI/EHCI
 *     USB Host Class: Prolific
 *     USB device: Prolific PL2303 USB to RS232 convertor cable
 *
 * Notes:
 *     This test will adopt usb host class driver to verify the key TX to PL2303, and the RX key that input by user.
 *     Get a PL2303 USB to RS232 convertor cable. Then plug the USB connector side into Wiced USB Host,
 *     and plug the RS232 side into one x86 PC RS232 serial COM port. Open console program (like TeraTerm) in x86 PC.
 *     Test will start immediately after correct bus enumeraton.
 *
 *     During test, check USB Host Prolific PL2303 TX/RX respectively by the following ways:
 *     TX: Once pressing any keyboard key from x86 PC console program for PL2303, test program will send (echo) the same key to PL2303 device.
 *         Verify the output on x86 PC console for PL2303 is identical or not.
 *     RX: Press any keyboard key from x86 PC console program for PL2303. This will make PL2303 device sending the key data to Wiced USB Host.
 *         Test program will dump the key on Wiced console when 4390x received data.
 *         Then verify the output on Wiced console to see if it is identical.
 *
 *     Test loopback infinitely.
 *
 * @@ USB Bulk transfer!!
 */
int usb_host_app_pl2303_console_rw( int argc, char* argv[] )
{
    UCHAR *prolific_reception_buffer = get_prolific_rx_buffer( MAX_PROLIFIC_RECEPTION_BUFFER_SIZE );
    UCHAR *prolific_xmit_buffer = get_prolific_tx_buffer( MAX_PROLIFIC_XMIT_BUFFER_SIZE );
    UX_HOST_CLASS_PROLIFIC_LINE_CODING line_coding = {0};
    ULONG actual_length = 0;
    UINT status;
    UINT cnt;

    WPRINT_APP_INFO( ("+++START: prolific console read/write loopback\n") );

    /* Wait USB prolific plugged in, or timeout to error  */
    cnt = USB_HOST_APP_DETECTION_TIMEOUT;
    while ( (prolific == NULL) && (cnt > 0) )
    {
        WPRINT_APP_INFO( ("Please plug in USB prolific device...\n") );
        wiced_rtos_delay_milliseconds( USB_HOST_APP_DEVICE_SCAN_DELAY_TIME );
        cnt --;
        if ( cnt == 0 )
        {
            WPRINT_APP_INFO( ("No prolific device plugged!\n") );
            goto _error_exit;
        }
    }

    WPRINT_APP_INFO( ("***USB prolific ready. Input any COM keys from your prolific device terminal...\n") );

    /*
     * Config prolific line coding values to device
     *
     * This is optional to set user prolific line coding values to device
     * Without config, the default prolific line coding values will be used!
     */
    line_coding.ux_host_class_prolific_line_coding_dter      = USB_HOST_PROLIFIC_LINE_CODING_RATE;
    line_coding.ux_host_class_prolific_line_coding_stop_bit  = USB_HOST_PROLIFIC_LINE_CODING_STOP_BIT;
    line_coding.ux_host_class_prolific_line_coding_parity    = USB_HOST_PROLIFIC_LINE_CODING_PARITY;
    line_coding.ux_host_class_prolific_line_coding_data_bits = USB_HOST_PROLIFIC_LINE_CODING_DATA_BIT;

    status = ux_host_class_prolific_ioctl( prolific, UX_HOST_CLASS_PROLIFIC_IOCTL_SET_LINE_CODING, (void *) &line_coding );
    if ( status != UX_SUCCESS )
    {
        WPRINT_APP_INFO( ("prolific set line coding failed. status=%d\n", status) );
        goto _error_exit;
    }

    /* Check prolific buffer */
    if ( ( prolific_reception_buffer == NULL ) || ( prolific_xmit_buffer == NULL ) )
    {
        WPRINT_APP_INFO( ("prolific buffer alloc error!\n") );
        goto _error_exit;
    }

    /* Clean prolific buffer */
    memset( (void *) prolific_reception_buffer, 0, MAX_PROLIFIC_RECEPTION_BUFFER_SIZE );
    memset( (void *) prolific_xmit_buffer, 0, MAX_PROLIFIC_XMIT_BUFFER_SIZE );

    /* Reset counter */
    prolific_command_received_count = 0;

    /* Start the reception for prolific */
    if ( prolific_reception.ux_host_class_prolific_reception_state == UX_HOST_CLASS_PROLIFIC_RECEPTION_STATE_STOPPED )
    {
        /*
         * Be noted that:
         * 1. The data_buffer address assigned for reception must be at cache-aligned address!!!
         * 2. The block_size is the request_length for one USB transfer request
         * 3. The callback is called when a full or partial USB transfer has been done for a Bulk IN transfer
         */
        prolific_reception.ux_host_class_prolific_reception_block_size = MAX_PROLIFIC_RECEPTION_BLOCK_SIZE;
        prolific_reception.ux_host_class_prolific_reception_data_buffer = prolific_reception_buffer;
        prolific_reception.ux_host_class_prolific_reception_data_buffer_size = MAX_PROLIFIC_RECEPTION_BUFFER_SIZE;
        prolific_reception.ux_host_class_prolific_reception_callback = pl2303_console_rw_rx_callback;

        status = ux_host_class_prolific_reception_start( prolific, &prolific_reception );
        if ( status != UX_SUCCESS )
        {
            WPRINT_APP_INFO( ("prolific reception start failed. status=%d\n", status) );
            goto _error_exit;
        }
    }

    /* Main loop  */
    while ( 1 )
    {
        /* We may receive one char at a time. */
        while( prolific_command_received_count == 0 )
        {
            wiced_rtos_delay_milliseconds( USB_HOST_APP_RX_DATA_CHECK_DELAY_TIME );
        }

        /* Copy the buffer received in the transmit buffer.  */
        memcpy( (UCHAR *) prolific_xmit_buffer, (UCHAR *) prolific_global_reception_buffer, prolific_global_reception_size );
        WPRINT_APP_INFO( ("prolific rx %s, size=%lu\n", prolific_global_reception_buffer, prolific_global_reception_size) );

        /* Echo the response. The data_buffer address assigned for transmit must be at cache-aligned address!!! */
        status = ux_host_class_prolific_write( prolific, prolific_xmit_buffer, prolific_global_reception_size, &actual_length );
        if ( status != UX_SUCCESS )
        {
            WPRINT_APP_INFO( ("prolific write echo response failed. status=%d\n", status) );
            goto _error_exit;
        }

        /* Reset the status count.  */
        prolific_command_received_count = 0;

        /* Check if that was a CR.  */
        if ( *( prolific_global_reception_buffer + prolific_global_reception_size - 1 ) == HEX_ASCII_CHAR_CR )
        {
            /* Mark the LF.  */
            memset( (void *) prolific_xmit_buffer, 0, sizeof(prolific_xmit_buffer) );
            prolific_xmit_buffer[0] = HEX_ASCII_CHAR_LF;

            /* Send the LF.  */
            WPRINT_APP_INFO( ("is CR, write LF...\n") );
            status = ux_host_class_prolific_write( prolific, prolific_xmit_buffer, 1, &actual_length );
            if ( status != UX_SUCCESS )
            {
                WPRINT_APP_INFO( ("prolific write LF failed. status=%d\n", status) );
                goto _error_exit;
            }
        }
    }

    WPRINT_APP_INFO( ("---END: prolific console read/write loopback\n") );
    return ERR_CMD_OK;

_error_exit:
    return ERR_UNKNOWN;
}

/**
 * USB Host app: Prolific PL2303 Get File with SHA1SUM Check
 *
 * Prerequisite:
 *     USB Host Controller: OHCI/EHCI
 *     USB Host Class: Prolific
 *     USB device: Prolific PL2303 USB to RS232 convertor cable
 *     SHA1 Checksum of Input File: http://onlinemd5.com/
 *
 * Notes:
 *     This test will adopt usb host class driver to verify the binary data (in file format) RX from PL2303.
 *     Get a PL2303 USB to RS232 convertor cable. Then plug the USB connector side into Wiced USB Host,
 *     and plug the RS232 side into one x86 PC RS232 serial COM port. Open console program (like TeraTerm) in x86 PC.
 *
 *     You may achieve this test by using only one x86 Win7 PC with two COM ports and two TeraTerm window. Ex:
 *     1. Plug the PL2303 RS232 side into x86 PC RS232 serial COM port, assumed COM1
 *     2. Plug Wiced board debug UART into another x86 PC RS232 serial COM port, assumed COM2
 *     3. After correct bus enumeraton, 4390x execute this command function (on COM2 TeraTerm) to wait for the data sent from PL2303.
 *     4. On COM1 TeraTerm function bar, select [File] -> [Send file...] to input your file with "Binary" checkbox checked.
 *     5. If you're sending printable ascii codes in file, please be caution to the new line (CR & LF) handle of your TeraTerm.
 *     6. Test will start immediately and the data will be sent from PL2303 usb device and received by 4390x usb host.
 *
 *     During test, check USB Host Prolific PL2303 RX by the following ways:
 *     RX: The program is designed to get the file sent from PL2303 device.
 *         Test program will calculate the SHA1 Checksum by collecting all incoming data.
 *         When total received data length equals to your input size when executing this command, program will print out SHA1 Checksum result.
 *         Then verify the SHA1 Checksum result showed on Wiced debug console to see if it is correct.
 *
 *     Test stops after the file successfully received by 4390x and dumped SHA1 Checksum result.
 *
 * @@ USB Bulk transfer!!
 */
int usb_host_app_pl2303_get_file( int argc, char* argv[] )
{
    UCHAR *prolific_reception_buffer = get_prolific_rx_buffer( MAX_PROLIFIC_RECEPTION_BUFFER_SIZE );
    UCHAR *prolific_xmit_buffer = get_prolific_tx_buffer( MAX_PROLIFIC_XMIT_BUFFER_SIZE );
    UX_HOST_CLASS_PROLIFIC_LINE_CODING line_coding = {0};
    ULONG in_file_size = atol( argv[1] );
    UINT status;
    UINT cnt;
    int i;

    WPRINT_APP_INFO( ("+++START: prolific get file with sha1sum check (input file: %lu bytes)\n", in_file_size) );

    /* Check valid file size */
    if ( in_file_size == 0 )
    {
        WPRINT_APP_INFO( ("Invalid file size!\n") );
        goto _error_exit;
    }

    /* Wait USB prolific plugged in, or timeout to error  */
    cnt = USB_HOST_APP_DETECTION_TIMEOUT;
    while ( (prolific == NULL) && (cnt > 0) )
    {
        WPRINT_APP_INFO( ("Please plug in USB prolific device...\n") );
        wiced_rtos_delay_milliseconds( USB_HOST_APP_DEVICE_SCAN_DELAY_TIME );
        cnt --;
        if ( cnt == 0 )
        {
            WPRINT_APP_INFO( ("No prolific device plugged!\n") );
            goto _error_exit;
        }
    }

    WPRINT_APP_INFO( ("***USB prolific ready. Send file from your prolific device terminal to start...\n") );

    /*
     * Config prolific line coding values to device
     *
     * This is optional to set user prolific line coding values to device
     * Without config, the default prolific line coding values will be used!
     */
    line_coding.ux_host_class_prolific_line_coding_dter      = USB_HOST_PROLIFIC_LINE_CODING_RATE;
    line_coding.ux_host_class_prolific_line_coding_stop_bit  = USB_HOST_PROLIFIC_LINE_CODING_STOP_BIT;
    line_coding.ux_host_class_prolific_line_coding_parity    = USB_HOST_PROLIFIC_LINE_CODING_PARITY;
    line_coding.ux_host_class_prolific_line_coding_data_bits = USB_HOST_PROLIFIC_LINE_CODING_DATA_BIT;

    status = ux_host_class_prolific_ioctl( prolific, UX_HOST_CLASS_PROLIFIC_IOCTL_SET_LINE_CODING, (void *) &line_coding );
    if ( status != UX_SUCCESS )
    {
        WPRINT_APP_INFO( ("prolific set line coding failed. status=%d\n", status) );
        goto _error_exit;
    }

    /* Check prolific buffer */
    if ( ( prolific_reception_buffer == NULL ) || ( prolific_xmit_buffer == NULL ) )
    {
        WPRINT_APP_INFO( ("prolific buffer alloc error!\n") );
        goto _error_exit;
    }

    /* Clean prolific buffer */
    memset( (void *) prolific_reception_buffer, 0, MAX_PROLIFIC_RECEPTION_BUFFER_SIZE );
    memset( (void *) prolific_xmit_buffer, 0, MAX_PROLIFIC_XMIT_BUFFER_SIZE );

    #if (USB_HOST_SAVE_DEBUG_DATA_IN_DDR)
    /* Reset DDR buffer address from start */
    usb_host_log_idx = 0;

    /* Clean DDR buffer from start */
    memset( (void *) usb_host_log_data, 0, USB_HOST_BUFFER_DDR_CLEAN_SIZE );
    #endif

    /* Init and start SHA1 checksum */
    mbedtls_sha1_starts( &prolific_sha1_ctx );

    /* Reset counter */
    prolific_total_rx_length = 0;
    prolific_command_received_count = 0;

    /* Start the reception for prolific */
    if ( prolific_reception.ux_host_class_prolific_reception_state == UX_HOST_CLASS_PROLIFIC_RECEPTION_STATE_STOPPED )
    {
        /*
         * Be noted that:
         * 1. The data_buffer address assigned for reception must be at cache-aligned address!!!
         * 2. The block_size is the request_length for one USB transfer request
         * 3. The callback is called when a full or partial USB transfer has been done for a Bulk IN transfer
         */
        prolific_reception.ux_host_class_prolific_reception_block_size = MAX_PROLIFIC_RECEPTION_BLOCK_SIZE;
        prolific_reception.ux_host_class_prolific_reception_data_buffer = prolific_reception_buffer;
        prolific_reception.ux_host_class_prolific_reception_data_buffer_size = MAX_PROLIFIC_RECEPTION_BUFFER_SIZE;
        prolific_reception.ux_host_class_prolific_reception_callback = pl2303_get_file_rx_callback;

        status = ux_host_class_prolific_reception_start( prolific, &prolific_reception );
        if ( status != UX_SUCCESS )
        {
            WPRINT_APP_INFO( ("prolific reception start failed. status=%d\n", status) );
            goto _error_exit;
        }
    }

    /* Main loop  */
    while ( 1 )
    {
        /* Wait data callback from reception */
        while( prolific_command_received_count == 0 )
        {
            wiced_rtos_delay_milliseconds( USB_HOST_APP_RX_DATA_CHECK_DELAY_TIME );
        }

        /* Compute the final SHA1 checksum value after all data received */
        if ( prolific_total_rx_length == in_file_size )
        {
            mbedtls_sha1_finish( &prolific_sha1_ctx, (unsigned char *)prolific_hash_value );

            WPRINT_APP_INFO( ("\n\nGet file with SHA1 Checksum [") );
            for ( i=0; i<SHA1_LENGTH; i++ )
            {
                WPRINT_APP_INFO( ("%02X", prolific_hash_value[i]) );
            }
            WPRINT_APP_INFO( ("]\n") );

            break;
        }

        /* Reset the status count.  */
        prolific_command_received_count = 0;
    }

    /* Stop the reception for prolific when out of main loop */
    if ( prolific_reception.ux_host_class_prolific_reception_state != UX_HOST_CLASS_PROLIFIC_RECEPTION_STATE_STOPPED )
    {
        status = ux_host_class_prolific_reception_stop( prolific, &prolific_reception );
        if ( status != UX_SUCCESS )
        {
            WPRINT_APP_INFO( ("prolific reception stop failed. status=%d\n", status) );
            goto _error_exit;
        }
    }

    WPRINT_APP_INFO( ("---END: prolific get file with sha1sum check\n") );
    return ERR_CMD_OK;

_error_exit:
    return ERR_UNKNOWN;
}

