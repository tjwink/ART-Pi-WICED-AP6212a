/**
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *
 */
#include "wiced.h"
#include "wiced_rtos.h"
#include "wiced_utilities.h"
#include "wiced_platform.h"
//#include "wiced_bt_platform.h"
#include "bt_hci_interface.h"
#include "bt_bus.h"
#include "bt_mfg_test.h"
#include "bt_transport_driver.h"
#include "bt_transport_thread.h"
#include "bt_firmware.h"

/******************************************************
 *                      Macros
 ******************************************************/

/* Verify if Bluetooth function returns success.
 * Otherwise, returns the error code immediately.
 * Assert in DEBUG build.
 */

/******************************************************
 *                    Constants
 ******************************************************/

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

extern wiced_result_t bt_mfgtest_transport_driver_bus_read_handler        ( bt_packet_t** packet );
static wiced_result_t bt_mfgtest_transport_driver_event_handler           ( bt_transport_driver_event_t event );
static wiced_result_t bt_mfgtest_transport_thread_received_packet_handler ( bt_packet_t* packet );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_ring_buffer_t pc_uart_ring_buffer;
static uint8_t             pc_uart_ring_buffer_data[BT_BUS_RX_FIFO_SIZE];
extern const char          brcm_patch_version[];
extern const uint8_t       brcm_patchram_buf[];
extern const int           brcm_patch_ram_length;

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t bt_mfgtest_start( const wiced_uart_config_t* config )
{
    wiced_result_t result;
    uint32_t content_length = 0;

    result = bt_bus_init();
    if ( result != WICED_BT_SUCCESS )
    {
        WPRINT_LIB_ERROR( ( "Error initialising BT bus\n" ) );
        return result;
    }

    /* if the app supports bridge functionality BT FW will be provided by the
     * host(PC) app and hence we are not required to download here. */
#ifndef WICED_BT_BRIDGE_MODE
    result = bt_firmware_download( brcm_patchram_buf, brcm_patch_ram_length, brcm_patch_version );
    if ( result != WICED_BT_SUCCESS )
    {
        WPRINT_LIB_ERROR( ( "Error downloading HCI firmware\n" ) );
        return result;
    }
#endif /* WICED_BT_BRIDGE_SUPPORT */

    result = bt_transport_driver_init( bt_mfgtest_transport_driver_event_handler, bt_mfgtest_transport_driver_bus_read_handler );
    if ( result != WICED_BT_SUCCESS )
    {
        WPRINT_LIB_ERROR( ( "Error initialising BT transport driver\n" ) );
        return result;
    }

    /* Initialise BT transport thread */
    result = bt_transport_thread_init( bt_mfgtest_transport_thread_received_packet_handler );
    if ( result != WICED_BT_SUCCESS )
    {
        WPRINT_LIB_ERROR( ( "Error initialising BT transport thread\n" ) );
        return result;
    }

    ring_buffer_init( &pc_uart_ring_buffer, pc_uart_ring_buffer_data, BT_BUS_RX_FIFO_SIZE );

    result = wiced_uart_init( STDIO_UART, config, &pc_uart_ring_buffer );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ( "Error initialising UART connection to PC\n" ) );
        return result;
    }

    /* Grab message from PC and pass it over to the controller */
    while ( 1 )
    {
        hci_command_header_t *header;
        bt_packet_t*         packet;
        uint32_t             expected_bytes = sizeof(hci_command_header_t);
        uint8_t              data[sizeof(wiced_hci_command_header_t)], header_size = sizeof(hci_command_header_t);

        header = (hci_command_header_t *)&data[0];

        /* Read HCI header */
        wiced_uart_receive_bytes( STDIO_UART, (void*)header, &expected_bytes, WICED_NEVER_TIMEOUT );

        if(HCI_WICED_PACKET == header->packet_type)
        {
            /* Wiced Header is of 5 bytes, so read extra header bytes */
            if (expected_bytes != sizeof(wiced_hci_command_header_t))
            {
                expected_bytes = 1;
                wiced_uart_receive_bytes( STDIO_UART, (void*)&data[4], &expected_bytes, WICED_NEVER_TIMEOUT );
            }
            /* Last two bytes content payload length */
            content_length = (data[3] + (data[4] << 8));

            /* Update header size */
            header_size = sizeof(wiced_hci_command_header_t);
        }
        else
        {
            content_length = header->content_length;
        }

        /* Allocate dynamic packet */
        bt_packet_pool_dynamic_allocate_packet( &packet, header_size, content_length );

        /* Copy header to packet */
        memcpy( packet->packet_start, header, header_size );

        /* Read the remaining packet */
        if ( content_length > 0 )
        {
            expected_bytes = content_length;

            wiced_uart_receive_bytes( STDIO_UART, packet->data_start, &expected_bytes, WICED_NEVER_TIMEOUT );

            /* Set the end of the packet */
            packet->data_end += content_length;
        }

        /* Send packet to the controller */
        bt_transport_driver_send_packet( packet );
    }

    return WICED_BT_SUCCESS;
}


wiced_result_t bt_mfgtest_console_start( const wiced_uart_config_t* config )
{
    wiced_result_t result;

    WPRINT_LIB_INFO(( "bt_mfgtest_console_start!\n"));

    result = bt_bus_init();
    if ( result != WICED_BT_SUCCESS )
    {
        WPRINT_LIB_ERROR( ( "Error initialising BT bus\n" ) );
        return result;
    }

    return WICED_BT_SUCCESS;
}

wiced_result_t bt_mfgtest_console_download_fw()
{
    wiced_result_t result;

    WPRINT_LIB_INFO(( "bt_mfgtest_console_download_fw!\n"));

    result = bt_firmware_download_lowrate( brcm_patchram_buf, brcm_patch_ram_length, brcm_patch_version );
    if ( result != WICED_BT_SUCCESS )
    {
        WPRINT_LIB_ERROR( ( "Error downloading HCI firmware\n" ) );
        return result;
    }

    return WICED_BT_SUCCESS;
}

wiced_result_t bt_mfgtest_console_send_hci( uint8_t *cmd, uint8_t len, \
                                            uint8_t *res, uint8_t res_len )
{
    hci_event_extended_header_t hci_event;
    char hci_command[20];
    char hci_response[256];
    uint8_t index = 0;
    wiced_result_t status = WICED_BT_ERROR;

    for (index = 0 ; index < len ; index++)
    {
        hci_command[index] = *(cmd+index);
    }

    bt_bus_transmit( (const uint8_t* ) hci_command, len );
    /* First reset command requires extra delay between write and read */
    bt_bus_receive( (uint8_t* ) &hci_event, sizeof( hci_event ), 1000 );

    if (hci_event.header.content_length > 4)
    {
        bt_bus_receive( (uint8_t* ) &hci_response, hci_event.header.content_length - 4, 1000 );
    }

    if (memcmp(&hci_event, res, 7) == 0)
    {
        status = WICED_BT_SUCCESS;
    }
    else
    {
        status = WICED_BT_ERROR;
    }

    return status;
}

wiced_result_t bt_mfgtest_console_receive_hci( uint8_t *read_data, uint8_t len)
{
    wiced_result_t result;

    result = bt_bus_receive( read_data, len, 1000 );
    return result;
}

static wiced_result_t bt_mfgtest_transport_driver_event_handler( bt_transport_driver_event_t event )
{
    if ( event == TRANSPORT_DRIVER_INCOMING_PACKET_READY )
    {
        return bt_transport_thread_notify_packet_received();
    }

    return WICED_BT_ERROR;
}

static wiced_result_t bt_mfgtest_transport_thread_received_packet_handler( bt_packet_t* packet )
{
    /* Pass HCI event packet to STDIO UART */
    wiced_uart_transmit_bytes( STDIO_UART, (const void*)packet->packet_start, packet->data_end - packet->packet_start );

    /* Release packet */
    return bt_packet_pool_free_packet( packet );
}
