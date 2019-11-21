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
 */

#include "wiced.h"
#include "spi_slave.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define SPI_SLAVE_THREAD_STACK_SIZE  ( 4096 )
#define SPI_SLAVE_THREAD_PRIORITY    ( WICED_DEFAULT_LIBRARY_PRIORITY )

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

static void spi_slave_thread_main( uint32_t arg );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t spi_slave_init( spi_slave_t* device, const spi_slave_device_config_t* config )
{
    uint32_t       buffer_size = 0;
    uint32_t       a;
    wiced_result_t result;

    if ( device == NULL || config == NULL )
    {
        return WICED_BADARG;
    }

    memset( device, 0, sizeof( *device ) );

    device->config = config;

    /* Get the largest register data size. Allocate buffer with the largest size */
    for ( a = 0; a < config->register_count; a++ )
    {
        if ( buffer_size < config->register_list[a].data_length )
        {
            buffer_size = config->register_list[a].data_length;
        }
    }
    device->buffer_size = sizeof(wiced_spi_slave_data_buffer_t) - 1 + buffer_size;
    device->buffer      = (wiced_spi_slave_data_buffer_t*) malloc_named( "SPI slave buffer", device->buffer_size );
    if ( device->buffer == NULL )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    /* Init SPI slave */
    result = wiced_spi_slave_init( config->spi, &config->config );
    if ( result != WICED_SUCCESS )
    {
        goto cleanup;
    }

    /* Create SPI device thread */
    result = wiced_rtos_create_thread( &device->thread, SPI_SLAVE_THREAD_PRIORITY, "SPI Slave", spi_slave_thread_main, SPI_SLAVE_THREAD_STACK_SIZE, (void*) device );
    if ( result != WICED_SUCCESS )
    {
        wiced_spi_slave_deinit( config->spi );
        goto cleanup;
    }

    return WICED_SUCCESS;

    cleanup:
    free( device->buffer );
    return result;
}

wiced_result_t spi_slave_deinit( spi_slave_t* device )
{
    UNUSED_PARAMETER( device );
    return WICED_SUCCESS;
}

static void spi_slave_thread_main( uint32_t arg )
{
    spi_slave_t* device = (spi_slave_t*)arg;

    while ( device->quit != WICED_TRUE )
    {
        const spi_slave_register_t* current_register = NULL;
        wiced_spi_slave_command_t   command;
        wiced_result_t              result;
        uint32_t                    a;

        /* Wait for command indefinitely */
        result = wiced_spi_slave_receive_command( device->config->spi, &command, WICED_NEVER_TIMEOUT );
        if ( result != WICED_SUCCESS )
        {
            continue;
        }

        /* Command has been received. Search for register */
        for ( a = 0; a < device->config->register_count; a++ )
        {
            if ( command.address == device->config->register_list[a].address )
            {
                current_register = &device->config->register_list[a];
            }
        }
        if ( current_register == NULL )
        {
            wiced_spi_slave_send_error_status( device->config->spi, SPI_SLAVE_TRANSFER_ADDRESS_UNAVAILABLE );
            continue;
        }

        /* Check for register access */
        if ( command.direction == SPI_SLAVE_TRANSFER_READ && current_register->access == SPI_SLAVE_ACCESS_WRITE_ONLY )
        {
            wiced_spi_slave_send_error_status( device->config->spi, SPI_SLAVE_TRANSFER_READ_NOT_ALLOWED );
            continue;
        }
        if ( command.direction == SPI_SLAVE_TRANSFER_WRITE && current_register->access == SPI_SLAVE_ACCESS_READ_ONLY )
        {
            wiced_spi_slave_send_error_status( device->config->spi, SPI_SLAVE_TRANSFER_WRITE_NOT_ALLOWED );
            continue;
        }

        /* Check for correct data length */
        if ( command.data_length != current_register->data_length )
        {
            wiced_spi_slave_send_error_status( device->config->spi, SPI_SLAVE_TRANSFER_LENGTH_MISMATCH );
            continue;
        }

        /* All conditions have been satisfied. Call read callback before transfer */
        if ( current_register->read_callback != NULL && command.direction == SPI_SLAVE_TRANSFER_READ )
        {
            current_register->read_callback( device, device->buffer->data );
        }

        /* If data type is static, copy data from static data pointer to buffer.
         * If data type is dynamic, user is expected to fill in data in the access start callback.
         */
        if ( current_register->data_type == SPI_SLAVE_REGISTER_DATA_STATIC )
        {
            memset( device->buffer, 0, device->buffer_size );
            memcpy( device->buffer->data, current_register->static_data, current_register->data_length );
        }
        device->buffer->data_length = current_register->data_length;

        /* Read/write data from/to the SPI master */
        result = wiced_spi_slave_transfer_data( device->config->spi, command.direction, device->buffer, WICED_NEVER_TIMEOUT );

        /* Call read callback after transfer */
        if ( result == WICED_SUCCESS && current_register->write_callback != NULL && command.direction == SPI_SLAVE_TRANSFER_WRITE )
        {
            current_register->write_callback( device, device->buffer->data );
        }
    }

    WICED_END_OF_CURRENT_THREAD( );
}
