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

#include <string.h>
#include "wiced_platform.h"
#include "platform_bluetooth.h"
#include "wwd_debug.h"
#include "wiced_rtos.h"
#include "wiced_uart.h"
#include "wiced_low_power.h"

/** @file
 *
 * Fast UART
 *
 */
/******************************************************
 *                      Macros
 ******************************************************/
#define BT_UART  ( WICED_UART_2 )
/******************************************************
 *                    Constants
 ******************************************************/
#define RX_BUFFER_SIZE    2048

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
extern wiced_result_t bt_issue_reset ( void );

/******************************************************
  *               External Variable Declarations
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
    .flow_control = FLOW_CONTROL_CTS_RTS,
};
wiced_ring_buffer_t rx_buffer;
uint8_t             rx_data[RX_BUFFER_SIZE];

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t wiced_hci_uart_init(void)
{
    wiced_bool_t is_warmboot = WICED_DEEP_SLEEP_IS_WARMBOOT( );

    if ( is_warmboot == WICED_FALSE )
    {
        if ( wiced_bt_control_pins[ WICED_BT_PIN_RESET ] != NULL )
        {
           /* Reset the Bluetooth */
           platform_gpio_init(wiced_bt_control_pins[WICED_BT_PIN_RESET],OUTPUT_PUSH_PULL);
           platform_gpio_output_high(wiced_bt_control_pins[WICED_BT_PIN_RESET]);
           platform_gpio_output_low(wiced_bt_control_pins[WICED_BT_PIN_RESET]);
           wiced_rtos_delay_milliseconds(500);
           platform_gpio_output_high(wiced_bt_control_pins[WICED_BT_PIN_RESET]);

        }

        else // Required for 43012
        {
           platform_gpio_init( wiced_bt_control_pins[ WICED_BT_PIN_POWER ], OUTPUT_PUSH_PULL );
           platform_gpio_output_high( wiced_bt_control_pins[ WICED_BT_PIN_POWER ] );
           platform_gpio_output_low( wiced_bt_control_pins[ WICED_BT_PIN_POWER ] );
           wiced_rtos_delay_milliseconds( 500 );
           platform_gpio_output_high( wiced_bt_control_pins[ WICED_BT_PIN_POWER ] );
        }

    }

    /* Initialise the HOST_WAKE and DEV_WAKE Line */
    platform_gpio_init( wiced_bt_control_pins[WICED_BT_PIN_HOST_WAKE], INPUT_HIGH_IMPEDANCE );
    platform_gpio_init( wiced_bt_control_pins[ WICED_BT_PIN_DEVICE_WAKE ], OUTPUT_PUSH_PULL );

    /* set the dev wake status to LOW */
    platform_gpio_output_low( wiced_bt_control_pins[ WICED_BT_PIN_DEVICE_WAKE ] );

    if ( is_warmboot == WICED_FALSE )
    {
        /* Initialise ring buffer */
        ring_buffer_init(&rx_buffer, rx_data, RX_BUFFER_SIZE );
        return ( wiced_uart_init( BT_UART, &uart_config, &rx_buffer ));
    }
    else
    {
        /* this check is only done for fly-wire setup */
        if( wiced_bt_config.featured_baud_rate == BAUDRATE_115KBPS )
        {
            /* Initialise ring buffer */
            ring_buffer_init(&rx_buffer, rx_data, RX_BUFFER_SIZE );
            return ( wiced_uart_init( BT_UART, &uart_config, &rx_buffer ));
        }

        return WICED_SUCCESS;
    }
}

wiced_result_t wiced_hci_uart_deinit( void )
{
     if(bt_issue_reset()!= WICED_BT_SUCCESS)
       return WICED_BT_ERROR;

         /* Reset the Bluetooth */
    if ( wiced_bt_control_pins[ WICED_BT_PIN_RESET ] != NULL )
    {
        platform_gpio_output_low(wiced_bt_control_pins[WICED_BT_PIN_RESET]);
    }
    else
    {
        platform_gpio_output_low( wiced_bt_control_pins[ WICED_BT_PIN_POWER ] );
    }

    ring_buffer_deinit(&rx_buffer);
    return ( wiced_uart_deinit(BT_UART) );
}


wiced_result_t wiced_hci_uart_reconfig(wiced_uart_config_t* config)
{

    /* Initialise ring buffer */
    wiced_uart_config_t bt_config;
    bt_config.baud_rate = config->baud_rate;
    bt_config.data_width = uart_config.data_width;
    bt_config.flow_control = uart_config.flow_control;
    bt_config.parity = uart_config.parity;
    bt_config.stop_bits = uart_config.stop_bits;

    ring_buffer_init(&rx_buffer, rx_data, RX_BUFFER_SIZE );
    return ( wiced_uart_init( BT_UART, &bt_config, &rx_buffer ));
}

wiced_result_t wiced_hci_uart_write(uint8_t* data, uint16_t length)
{
    return(wiced_uart_transmit_bytes( BT_UART, data, length ));
}

wiced_result_t wiced_hci_uart_read(uint8_t* data, uint32_t* length, uint32_t timeout_ms)
{
    return(wiced_uart_receive_bytes( BT_UART, data, length, timeout_ms ));
}
