/**
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

#include <string.h>
#include "wiced_rtos.h"
#include "wiced_utilities.h"
#include "bt_hci_interface.h"
#include "platform_bluetooth.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "wiced_uart.h"
/******************************************************
 *                      Macros
 ******************************************************/
/* Verify if bt_bus function returns success.
 * Otherwise, returns the error code immediately.
 * Assert in DEBUG build.
 */
#define VERIFY_RETVAL( function_call ) \
do \
{ \
    wiced_result_t verify_result = (function_call); \
    if ( verify_result != WICED_SUCCESS ) \
    { \
        WPRINT_APP_INFO(("[%s %d] VERIFY FAILED\n",__func__,__LINE__)); \
        wiced_assert( "Bus function failed", 0!=0 ); \
        return verify_result; \
    } \
} while ( 0 )

/* Verify if HCI response is expected.
 * Otherwise, returns HCI unexpected response immediately.
 * Assert in DEBUG build.
 */
#define VERIFY_RESPONSE( a, b, size ) \
{ \
    if ( memcmp( (a), (b), (size) ) != 0 ) \
    { \
        wiced_assert( "HCI unexpected response", 0!=0 ); \
        return WICED_BT_UNKNOWN_PACKET; \
    } \
}

/******************************************************
 *                    Constants
 ******************************************************/
#define DEFAULT_READ_TIMEOUT (100)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
/*BT chip specific configuration information*/
extern const platform_bluetooth_config_t wiced_bt_config;

/******************************************************
 *               extern Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/
wiced_result_t bt_issue_reset ( void );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t bt_issue_reset ( void )
{
    uint32_t length = 4;
    uint8_t hci_data[ 8 ] = { 0x00 };
    uint8_t hci_reset_cmd[] = { 0x1, 0x03, 0x0c, 0x00}; /* reset code as per spec */
    uint8_t hardware_error[ 4 ] = { 0x04, 0x10, 0x01, 0x00 };
    uint8_t hci_reset_expected_event[] = {0x04, 0x0E, 0x04, 0x01, 0x03, 0x0C, 0x00};

    wiced_hci_uart_write(hci_reset_cmd, length);

    length = 7; /* length of expected response */
    /* if any hardware parsing error event just ignore it and read next bytes */
    wiced_hci_uart_read(hci_data, &length, 110);

    if ( !memcmp( hardware_error, hci_data, 4 ) )
    {
        WPRINT_APP_INFO(( "hardware parsing error received \n" ));
        length = sizeof(hardware_error);
        VERIFY_RETVAL( wiced_hci_uart_read(hci_data, &length,100) );
    }

    if ( memcmp( hci_data, hci_reset_expected_event, 7 )!=0 )
    {
        WPRINT_APP_INFO(( "HCI_CMD_RESET command reponse is wrong \n" ));
        return WICED_BT_ERROR;
    }

    return WICED_BT_SUCCESS;
}


static wiced_result_t bt_host_update_baudrate( uint32_t newBaudRate )
{
    char update_baudrate_cmd[ 10 ];
    wiced_result_t result;
    uint8_t hci_event[8] = { 0x00 };
    uint32_t length = 7;
    platform_uart_config_t bt_uart_config;
    uint8_t update_baudrate_expected_event[] = {0x04, 0x0E, 0x04, 0x01, 0x18, 0xFC, 0x00};

    /* format of transmit bytes for update baud rate command
     * 0x01, 0x18, 0xFC, 0x06, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff
     */
    update_baudrate_cmd[ 0 ] = 0x01;
    update_baudrate_cmd[ 1 ] = 0x18;
    update_baudrate_cmd[ 2 ] = 0xfc;
    update_baudrate_cmd[ 3 ] = 0x06;
    update_baudrate_cmd[ 4 ] = 0x00; /* Encoded baud rate ; disable : 0 , enable : 1 */
    update_baudrate_cmd[ 5 ] = 0x00; /* use Encoded form  ; disable : 0 , enable : 1 */
    /* issue BT hci update baudrate */
    update_baudrate_cmd[ 6 ] = ( newBaudRate ) & 0xff;
    update_baudrate_cmd[ 7 ] = ( newBaudRate >> 8 ) & 0xff;
    update_baudrate_cmd[ 8 ] = ( newBaudRate >> 16 ) & 0xff;
    update_baudrate_cmd[ 9 ] = ( newBaudRate >> 24 ) & 0xff;

    wiced_hci_uart_write((uint8_t* ) update_baudrate_cmd, 10 );
    wiced_hci_uart_read(hci_event, &length,1000);

    wiced_rtos_delay_milliseconds( 40 );

    if ( memcmp( hci_event, update_baudrate_expected_event, 7 ) != 0 )
    {
        WPRINT_APP_INFO(( "update baudrate failed\n" ));
        return WICED_BT_ERROR;
    }

    memset( &bt_uart_config, 0, sizeof( bt_uart_config ) );

    /* Update host uart baudrate*/
    bt_uart_config.baud_rate = newBaudRate;
    result = wiced_hci_uart_reconfig( &bt_uart_config );
    if (result != WICED_BT_SUCCESS)
    {
        WPRINT_LIB_ERROR(( "bt_host_update_baudrate Fail!%d\n", result));
    }
    return result;
}

wiced_result_t bt_firmware_download( const uint8_t* firmware_image, uint32_t size, const char* version )
{
    uint8_t* data = (uint8_t*) firmware_image;
    uint32_t remaining_length = size;
    uint8_t hci_event[100];

    if(bt_issue_reset()!= WICED_BT_SUCCESS)
        return WICED_BT_ERROR;

    if( wiced_bt_config.featured_baud_rate == BAUDRATE_3MBPS )
    {
        if ( bt_host_update_baudrate( BAUDRATE_3MBPS ) != WICED_BT_SUCCESS )
        return WICED_BT_ERROR;
    }

    /* Send hci_download_minidriver command */
    if ( wiced_bt_config.patchram_download_mode == PATCHRAM_DOWNLOAD_MODE_MINIDRV_CMD )
    {
        uint8_t minidrv[] = {0x1, 0x2e, 0xfc, 00};
        uint8_t hci_data[100];
        uint32_t length = 7;
        wiced_hci_uart_write(minidrv, 4);
        wiced_hci_uart_read(hci_data, &length,100);

    }

    /* The firmware image (.hcd format) contains a collection of hci_write_ram command + a block of the image,
     * followed by a hci_write_ram image at the end. Parse and send each individual command and wait for the response.
     * This is to ensure the integrity of the firmware image sent to the bluetooth chip.
     */
    while ( remaining_length )
    {
        uint32_t data_length = data[ 2 ] + 3; /* content of data length + 2 bytes of opcode and 1 byte of data length */
        hci_command_opcode_t command_opcode = *(hci_command_opcode_t*) data;
        uint8_t temp_data[ 512 ];
        uint32_t bytes_read;
        memset( &hci_event, 0, sizeof( hci_event ) );
        memset( temp_data, 0, sizeof( temp_data ) );

        temp_data[ 0 ] = HCI_COMMAND_PACKET;
        memcpy( &temp_data[ 1 ], data, data_length );

        /* Send hci_write_ram command. The length of the data immediately follows the command opcode */
        wiced_hci_uart_write(temp_data, data_length+1);

        bytes_read = 7;
        wiced_hci_uart_read(hci_event, &bytes_read,220);

        switch ( command_opcode )
        {
            case HCI_CMD_OPCODE_WRITE_RAM:
                /* Update remaining length and data pointer */
                data += data_length;
                remaining_length -= data_length;
                break;

            case HCI_CMD_OPCODE_LAUNCH_RAM:
                /* All responses have been read. Now let's flush residual data if any and reset remaining length */
                bytes_read = 1;
                remaining_length = 0;
                break;

            default:
                return WICED_BT_UNKNOWN_PACKET;
        }
    }

    /* Wait for bluetooth chip to pull its RTS (host's CTS) low. From observation using CRO, it takes the bluetooth chip > 170ms to pull its RTS low after CTS low */

    return 0;

}
