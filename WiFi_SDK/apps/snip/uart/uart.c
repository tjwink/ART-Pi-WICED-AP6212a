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
 * UART API Application
 *
 * This application demonstrates how to use the generic
 * WICED UART API to send and receive characters on the UART
 *
 * Features demonstrated
 *  - UART API
 *  - Ring buffer library
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 *   Follow the prompts printed on the terminal
 *
 * NOTE

 *   Standard I/O definition (STDIO_UART) can be found in
 *   <WICED-SDK>/platforms/<Platform>/platform.h.
 *   To access STDIO_UART using the generic UART API,
 *   it is necessary to add a global define WICED_DISABLE_STDIO
 *   in the uart.mk application makefile
 *
 */

#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define RX_BUFFER_SIZE    64
#define TEST_STR          "\r\nType something! Keystrokes are echoed to the terminal ...\r\n> "

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

/******************************************************
 *               Variable Definitions
 ******************************************************/

wiced_uart_config_t uart_config =
{
    .baud_rate    = 115200,
    .data_width   = DATA_WIDTH_8BIT,
    .parity       = NO_PARITY,
    .stop_bits    = STOP_BITS_1,
    .flow_control = FLOW_CONTROL_DISABLED,
};

wiced_ring_buffer_t rx_buffer;
uint8_t             rx_data[RX_BUFFER_SIZE];

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    char c;
    uint32_t expected_data_size = 1;

    /* Initialise ring buffer */
    ring_buffer_init(&rx_buffer, rx_data, RX_BUFFER_SIZE );

    /* Initialise UART. A ring buffer is used to hold received characters */
    wiced_uart_init( STDIO_UART, &uart_config, &rx_buffer );

    /* Send a test string to the terminal */
    wiced_uart_transmit_bytes( STDIO_UART, TEST_STR, sizeof( TEST_STR ) - 1 );

    /* Wait for user input. If received, echo it back to the terminal */
    while ( wiced_uart_receive_bytes( STDIO_UART, &c, &expected_data_size, WICED_NEVER_TIMEOUT ) == WICED_SUCCESS )
    {
        wiced_uart_transmit_bytes( STDIO_UART, &c, 1 );
        expected_data_size = 1;
    }
}
