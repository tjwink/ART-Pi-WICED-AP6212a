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
 *  Gspi Slave Application
 *
 *  This application demonstrates how a SPI master communicate with a GSPI slave:
 *  A callback function is registered with slave driver and it's called every time
 *  when the slave driver receives command from master.
 *
 */

#include "wiced.h"
#include "gspi_slave.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define GSPI_DEVICE_SEND_BUFFER_SIZE ( 1024*3 )
#define GSPI_DEVICE_RECEIVE_BUFFER_SIZE ( 1024*3 )

/******************************************************
 *             Static Function Declarations
 ******************************************************/
static wiced_result_t process_command_callback( gspi_sw_tx_header_t* gspi_sw_master_cmd );
static uint8_t receive_buffer[GSPI_DEVICE_RECEIVE_BUFFER_SIZE];

/******************************************************
 *               Variable Definitions
 ******************************************************/
static gspi_slave_device_t gspi_slave_device;


static const gspi_slave_application_t gspi_slave_app =
{
        .send_data_buffer_size = GSPI_DEVICE_SEND_BUFFER_SIZE,
        .receive_data_buffer_size = GSPI_DEVICE_RECEIVE_BUFFER_SIZE,
        .command_callback = process_command_callback,
};

void application_start( void )
{
    /* Initialize the WICED device */
    wiced_init();

    /* Initialize GSPI slave device */
    gspi_slave_init( &gspi_slave_device, &gspi_slave_app );
}

static wiced_result_t process_command_callback( gspi_sw_tx_header_t* gspi_sw_tx_hdr )
{
    uint8_t *data_ptr;
    uint32_t data_length;

    wiced_result_t result = WICED_SUCCESS;

    data_ptr = gspi_sw_tx_hdr->buffer;
    data_length = gspi_sw_tx_hdr->gspi_sw_hdr.data_length;

    if (gspi_sw_tx_hdr->gspi_sw_hdr.direction == GSPI_MASTER_TRANSFER_WRITE )
    {
        WPRINT_APP_DEBUG( ("%s: WRITE, datat_address : 0x%lx, data length: %ld\n", __FUNCTION__,
                            gspi_sw_tx_hdr->gspi_sw_hdr.address,
                            data_length) );
        memcpy(receive_buffer, data_ptr, data_length);
    }
    else
    {
        WPRINT_APP_DEBUG( ("%s: READ, datat_address : 0x%lx, data length: %ld\n", __FUNCTION__,
                            gspi_sw_tx_hdr->gspi_sw_hdr.address,
                            data_length) );
        gspi_slave_send_response_data( &gspi_slave_device, (uint8_t *)receive_buffer, data_length );
    }

    return result;
}
