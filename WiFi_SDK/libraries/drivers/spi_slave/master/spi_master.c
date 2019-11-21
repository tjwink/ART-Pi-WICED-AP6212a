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
#include "spi_master.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define DATA_READY_TIMEOUT_MS  ( 1000 )

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

static void           spi_slave_ready_pin_irq_handler     ( void* arg );
static wiced_result_t spi_slave_data_ready_worker_callback( void* arg );
static wiced_result_t spi_slave_convert_status_to_result  ( wiced_spi_slave_transfer_status_t status );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t spi_master_init( spi_master_t* master, const wiced_spi_device_t* device, wiced_gpio_t data_ready_pin, spi_master_data_ready_callback_t data_ready_callback )
{
    wiced_result_t result;

    if ( master == NULL || device == NULL )
    {
        return WICED_BADARG;
    }

    master->device              = device;
    master->in_transaction      = WICED_FALSE;
    master->data_ready_pin      = data_ready_pin;
    master->data_ready_callback = data_ready_callback;

    /* Init data ready semaphore */
    result = wiced_rtos_init_semaphore( &master->data_ready_semaphore );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    /* Init ready pin interrupt */
    result = wiced_gpio_init( data_ready_pin, INPUT_PULL_UP );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }
    result = wiced_gpio_input_irq_enable( data_ready_pin, IRQ_TRIGGER_BOTH_EDGES, spi_slave_ready_pin_irq_handler, (void*)master );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    /* Init SPI master on the master CPU */
    result = wiced_spi_init( master->device );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    return WICED_SUCCESS;
}

wiced_result_t spi_master_write_register( spi_master_t* master, uint16_t address, uint16_t size, uint8_t* data_out )
{
    wiced_result_t                    result;
    wiced_spi_slave_command_t         command;
    wiced_spi_message_segment_t       message;
    wiced_spi_slave_transfer_status_t status;

    memset( &command, 0x00, sizeof( command ) );

    /* Indicate that master is currently in transaction */
    master->in_transaction = WICED_TRUE;

    /* Send command */
    command.direction   = SPI_SLAVE_TRANSFER_WRITE;
    command.address     = address;
    command.data_length = size;
    message.length      = sizeof(command);
    message.tx_buffer   = &command;
    message.rx_buffer   = NULL;

    /* Wait until data ready pin is set to low logic level( falling edge is detected ) */
    /* Slave is ready to receive a command only when it sets its ready pin to low logic level */
    if( wiced_gpio_input_get( master->data_ready_pin ) == WICED_TRUE )
    {
        while(1)
        {
            result = wiced_rtos_get_semaphore( &master->data_ready_semaphore, DATA_READY_TIMEOUT_MS );
            if( result != WICED_SUCCESS )
            {
                master->in_transaction = WICED_FALSE;
                return result;
            }

            /* Check whether the ready ping is set to high logic level, if not wait on a semaphore again */
            if( wiced_gpio_input_get( master->data_ready_pin ) == WICED_FALSE )
            {
                break;
            }
            else
            {
                //wiced_assert("Unexpected condition", 0!=0);
            }
        }
    }

    /* Send the command */
    result = wiced_spi_transfer( master->device, &message, 1 );
    if( result != WICED_SUCCESS )
    {
        master->in_transaction = WICED_FALSE;
        return result;
    }

    /* Wait until data ready pin is set to high logic level( rising edge is detected ) */
    if( wiced_gpio_input_get( master->data_ready_pin ) == WICED_FALSE )
    {
        while(1)
        {
            result = wiced_rtos_get_semaphore( &master->data_ready_semaphore, DATA_READY_TIMEOUT_MS );
            if( result != WICED_SUCCESS )
            {
                master->in_transaction = WICED_FALSE;
                return result;
            }
            /* Check whether the ready ping is set to high logic level, if not wait on a semaphore again */
            if( wiced_gpio_input_get( master->data_ready_pin ) == WICED_TRUE )
            {
                break;
            }
            else
            {
                //wiced_assert("Unexpected condition", 0!=0);
            }
        }
    }

    /* Read status byte */
    message.length    = 1;
    message.tx_buffer = NULL;
    message.rx_buffer = &status;
    result = wiced_spi_transfer( master->device, &message, 1 );
    if ( result != WICED_SUCCESS )
    {
        master->in_transaction = WICED_FALSE;
        return result;
    }
    if ( status != SPI_SLAVE_TRANSFER_SUCCESS )
    {
        master->in_transaction = WICED_FALSE;
        return spi_slave_convert_status_to_result( status );
    }

    /* Write data value */
    message.length      = size;
    message.tx_buffer   = data_out;
    message.rx_buffer   = NULL;
    result = wiced_spi_transfer( master->device, &message, 1 );
    if ( result != WICED_SUCCESS )
    {
        master->in_transaction = WICED_FALSE;
        return result;
    }

    return WICED_SUCCESS;
}

wiced_result_t spi_master_read_register( spi_master_t* master, uint16_t address, uint16_t size, uint8_t* data_in )
{
    wiced_result_t                    result;
    wiced_spi_slave_command_t         command;
    wiced_spi_message_segment_t       message;
    wiced_spi_slave_transfer_status_t status;

    memset(&command, 0x00, sizeof( command ) );

    /* Indicate that master is currently in transaction */
    master->in_transaction = WICED_TRUE;

    /* Send command */
    command.direction   = SPI_SLAVE_TRANSFER_READ;
    command.address     = address;
    command.data_length = size;
    message.length      = sizeof(command);
    message.tx_buffer   = &command;
    message.rx_buffer   = NULL;

    /* Wait until data ready pin is set to low logic level( falling edge is detected ) */
    /* Slave is ready to receive a command only when it sets its ready pin to low logic level */
    if( wiced_gpio_input_get( master->data_ready_pin ) == WICED_TRUE )
    {
        while(1)
        {
            result = wiced_rtos_get_semaphore( &master->data_ready_semaphore, DATA_READY_TIMEOUT_MS );
            if( result != WICED_SUCCESS )
            {
                master->in_transaction = WICED_FALSE;
                return result;
            }

            /* Check whether the ready ping is set to high logic level, if not wait on a semaphore again */
            if( wiced_gpio_input_get( master->data_ready_pin ) == WICED_FALSE )
            {
                break;
            }
            else
            {
                //wiced_assert("Unexpected condition", 0!=0);
            }
        }
    }

    result = wiced_spi_transfer( master->device, &message, 1 );
    if( result != WICED_SUCCESS )
    {
        master->in_transaction = WICED_FALSE;
        return result;
    }

    /* Wait until data ready pin is set to high logic level( rising edge is detected ) */
    if( wiced_gpio_input_get( master->data_ready_pin ) == WICED_FALSE )
    {
        while(1)
        {
            result = wiced_rtos_get_semaphore( &master->data_ready_semaphore, DATA_READY_TIMEOUT_MS );
            if( result != WICED_SUCCESS )
            {
                master->in_transaction = WICED_FALSE;
                return result;
            }
            /* Check whether the ready ping is set to high logic level, if not wait on a semaphore again */
            if( wiced_gpio_input_get( master->data_ready_pin ) == WICED_TRUE )
            {
                break;
            }
            else
            {
                //wiced_assert("Unexpected condition", 0!=0);
            }
        }
    }
    /* Read status byte */
    message.length    = 1;
    message.tx_buffer = NULL;
    message.rx_buffer = &status;
    result = wiced_spi_transfer( master->device, &message, 1 );
    if ( result != WICED_SUCCESS )
    {
        master->in_transaction = WICED_FALSE;
        return result;
    }
    if ( status != SPI_SLAVE_TRANSFER_SUCCESS )
    {
        master->in_transaction = WICED_FALSE;
        return spi_slave_convert_status_to_result( status );
    }

    /* Read data value */
    message.length      = size;
    message.tx_buffer   = NULL;
    message.rx_buffer   = data_in;
    result = wiced_spi_transfer( master->device, &message, 1 );
    if ( result != WICED_SUCCESS )
    {
        master->in_transaction = WICED_FALSE;
        return result;
    }

    return WICED_SUCCESS;
}

static void spi_slave_ready_pin_irq_handler( void* arg )
{
    spi_master_t* master = (spi_master_t*)arg;

    if ( master->in_transaction == WICED_TRUE )
    {
        wiced_rtos_set_semaphore( &master->data_ready_semaphore );
    }
    else
    {
        wiced_rtos_send_asynchronous_event( WICED_NETWORKING_WORKER_THREAD, spi_slave_data_ready_worker_callback, arg );
    }
}

static wiced_result_t spi_slave_data_ready_worker_callback( void* arg )
{
    spi_master_t* master = (spi_master_t*)arg;

    if ( master->data_ready_callback != NULL )
    {
        master->data_ready_callback( master );
    }

    return WICED_SUCCESS;
}

static wiced_result_t spi_slave_convert_status_to_result( wiced_spi_slave_transfer_status_t status )
{
    switch ( status )
    {
        case SPI_SLAVE_TRANSFER_SUCCESS:             return WICED_PLATFORM_SUCCESS;
        case SPI_SLAVE_TRANSFER_INVALID_COMMAND:     return WICED_PLATFORM_SPI_SLAVE_INVALID_COMMAND;
        case SPI_SLAVE_TRANSFER_ADDRESS_UNAVAILABLE: return WICED_PLATFORM_SPI_SLAVE_ADDRESS_UNAVAILABLE;
        case SPI_SLAVE_TRANSFER_LENGTH_MISMATCH:     return WICED_PLATFORM_SPI_SLAVE_LENGTH_MISMATCH;
        case SPI_SLAVE_TRANSFER_READ_NOT_ALLOWED:    return WICED_PLATFORM_SPI_SLAVE_READ_NOT_ALLOWED;
        case SPI_SLAVE_TRANSFER_WRITE_NOT_ALLOWED:   return WICED_PLATFORM_SPI_SLAVE_WRITE_NOT_ALLOWED;
        case SPI_SLAVE_TRANSFER_HARDWARE_ERROR:      return WICED_PLATFORM_SPI_SLAVE_HARDWARE_ERROR;
        case SPI_SLAVE_TRANSFER_STATUS_MAX:          return WICED_ERROR;
        default:                                     return WICED_ERROR;
    }
}
