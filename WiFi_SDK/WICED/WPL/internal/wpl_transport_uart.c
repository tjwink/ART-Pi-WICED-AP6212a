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
#include "wpl_transport_uart.h"
#include "wpl_packet.h"
#include "platform.h"
#include "wpl_core.h"
#include "wpl_console.h"
#include "wiced_power_logger.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define WPL_UART_THREAD_NAME     "WPL UART"

#define WPL_UART_STACK_SIZE      ( 6 * 1024 )

#define WPL_TIME_TO_WAIT_FOR_CONSOLE        2000 /* msec */
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
static wiced_result_t wpl_transport_uart_read_handler( wpl_packet_t** packet );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_bool_t                                 wpl_transport_uart_thread_initialised = WICED_FALSE;
static wpl_transport_received_packet_handler_t      wpl_transport_received_packet_handler = NULL;

static volatile wiced_bool_t                  uart_thread_running = WICED_FALSE;
static wiced_thread_t                         uart_thread;

wiced_semaphore_t          wpl_stdio_uart_rx_semaphore;

/******************************************************
 *               Function Definitions
 ******************************************************/
void wpl_transport_uart_thread_main( uint32_t arg )
{
    wpl_packet_t* packet = NULL;
    WPRINT_APP_DEBUG( ( "wpl_transport_uart_thread_main: enter\n" ) );

    /* turn off buffers, so IO occurs immediately */
#ifdef _IONBF
    setvbuf( stdin, NULL, _IONBF, 0 );
    setvbuf( stdout, NULL, _IONBF, 0 );
    setvbuf( stderr, NULL, _IONBF, 0 );
#endif

    while( 1 )
    {
        WPRINT_APP_DEBUG( ( "wpl_transport_uart_thread_main: Getting into delay\n" ) );
        /* Wait for some time to give command console the opportunity to come up and take UART control */
        if ( wpl_get_mode( ) != WPL_MODE_NO_CONSOLE )
            wiced_rtos_delay_milliseconds( WPL_TIME_TO_WAIT_FOR_CONSOLE );
        WPRINT_APP_DEBUG( ( "wpl_transport_uart_thread_main: Out of delay\n" ) );

        /* Try to get the stdio uart control semaphore */
        wiced_rtos_get_semaphore( &wpl_stdio_uart_rx_semaphore, NEVER_TIMEOUT );

        /* Set to No Console mode */
        wpl_set_mode( WPL_MODE_NO_CONSOLE );
        while ( 1 )
        {
            if ( wpl_get_mode( ) != WPL_MODE_NO_CONSOLE )
            {
                /* PAD mode is not enabled, release the semaphore and wait for the mode to be enabled */
                wiced_rtos_set_semaphore( &wpl_stdio_uart_rx_semaphore );
                break;
            }
            if ( wpl_transport_uart_read_handler( &packet ) != WICED_SUCCESS )
            {
                continue;
            }
            /* Read successful. Notif y upper layer via callback that a new packet is available */
            wpl_transport_received_packet_handler( packet );
        }
    }
    WICED_END_OF_CURRENT_THREAD( );
}

wiced_result_t wpl_uart_transport_init( wpl_transport_received_packet_handler_t handler )
{
    wiced_result_t result = WICED_SUCCESS;

    if ( handler == NULL )
    {
        return WICED_BADARG;
    }

    if ( wpl_transport_uart_thread_initialised == WICED_TRUE )
    {
        return WICED_SUCCESS;
    }

    /* Create UART thread to receive the packets.
     */
    uart_thread_running     = WICED_TRUE;
    wpl_transport_received_packet_handler    = handler;

    /* Set the semaphore for Command Console or WPL */
    wiced_rtos_set_semaphore( &wpl_stdio_uart_rx_semaphore );

    result = wiced_rtos_create_thread( &uart_thread, WICED_DEFAULT_WORKER_PRIORITY, WPL_UART_THREAD_NAME, wpl_transport_uart_thread_main, WPL_UART_STACK_SIZE, NULL );
    if ( result != WICED_SUCCESS )
    {
        wiced_assert( "Error creating UART thread\n", result == WICED_SUCCESS );
        goto error;
    }

    return WICED_SUCCESS;

    error:
    wpl_uart_transport_deinit( );
    return result;
}

wiced_result_t wpl_uart_transport_deinit( void )
{
    if ( wpl_transport_uart_thread_initialised == WICED_FALSE )
        return WICED_SUCCESS;

    uart_thread_running = WICED_FALSE;
    wiced_rtos_delete_thread( &uart_thread );
    wpl_transport_received_packet_handler = NULL;
    wpl_transport_uart_thread_initialised      = WICED_FALSE;
    return WICED_SUCCESS;
}

wiced_result_t wpl_uart_transport_send_packet( wpl_packet_t* packet )
{
    wiced_result_t result = WICED_SUCCESS;

    if ( wpl_get_console_prints_status( ) )
    {
        uint32_t i = 0;
        for( i = 0; i < packet->packet_size; i++  )
        {
            WPRINT_APP_INFO( ( "0x%x ", *( packet->packet_start+i ) ) );
        }
        WPRINT_APP_INFO( ( "\n" ) );
    }
    else
    {
        result = wiced_uart_transmit_bytes( STDIO_UART, packet->packet_start, packet->packet_size ) ;
    }
    if ( result != WICED_SUCCESS )
    {
        wiced_assert( "Error transmitting WPL packet\n", result == WICED_SUCCESS );
        wpl_free_packet( packet );
        return result;
    }

    /* Destroy packet */
    return wpl_free_packet( packet );
}

wiced_result_t wpl_transport_uart_read_handler( wpl_packet_t** packet )
{
    uint8_t packet_byte=0;
    wiced_result_t    result;
    uint32_t payload_len =  0;
    uint32_t       expected_transfer_size = 1;

    /* Get the SYN byte */
    result = wiced_uart_receive_bytes( STDIO_UART, &packet_byte, &expected_transfer_size, NEVER_TIMEOUT );

    if ( ( result != WICED_SUCCESS ) || ( packet_byte != WPL_SYN_BYTE1 ) )
    {
        return WICED_PACKET_BUFFER_CORRUPT;
    }
    expected_transfer_size = 1;
    result = wiced_uart_receive_bytes( STDIO_UART, &packet_byte, &expected_transfer_size, NEVER_TIMEOUT );

    if ( ( result != WICED_SUCCESS ) || ( packet_byte != WPL_SYN_BYTE2 ) )
    {
        return WICED_PACKET_BUFFER_CORRUPT;
    }

    /* Get the packet type */
    expected_transfer_size = 1;
    result = wiced_uart_receive_bytes( STDIO_UART, &packet_byte, &expected_transfer_size, NEVER_TIMEOUT );
    if ( result != WICED_SUCCESS ) {
        return result;
    }

    WPRINT_APP_DEBUG( ( "WPL: packet:%d\n", packet_byte ) );
    /* Read the header and determine the   */
    switch ( packet_byte )
    {
        case CMD_TARGET_DETECTION:
        case CMD_GET_TARGET_WPL_VERSION:
        case CMD_GET_PROCESSOR_LIST:
        case CMD_LOG_REQUEST:
        case 'L':
        {
            payload_len = 0;
            break;
        }
        case CMD_GET_EVENTS_LIST:
        {
            payload_len = 1;
            break;
        }
        case CMD_GET_EVENT_DESCRIPTOR_LIST:
        {
            payload_len = 2;
            break;
        }
        case CMD_START_LOGGING:
        case CMD_STOP_LOGGING:
        {
            uint8_t events;
            /* Get number of events */
            expected_transfer_size = 1;
            result = wiced_uart_receive_bytes( STDIO_UART, &events, &expected_transfer_size, NEVER_TIMEOUT );
            if ( result != WICED_SUCCESS )
                return result;
            payload_len = events * 2;
            break;
        }
        case CMD_LOG_POLL_PERIOD:
            payload_len = 1;
            break;
        default:
            return WICED_UNSUPPORTED;
    }
    result = wpl_dynamic_allocate_packet( packet, WPL_PAD_REQUEST, packet_byte, payload_len );
    if ( result != WICED_SUCCESS )
        return result;

    /* Retrieve the payload data */
    if ( payload_len )
    {
        expected_transfer_size = payload_len;
        result = wiced_uart_receive_bytes( STDIO_UART, ( *packet )->payload_start, &expected_transfer_size, NEVER_TIMEOUT );
        if ( result != WICED_SUCCESS )
        {
            wpl_free_packet( *packet );
            return result;
        }
    }
    return WICED_SUCCESS;
}

