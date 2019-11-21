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

#include "platform_peripheral.h"
#include "wwd_assert.h"
#include "wwd_rtos.h"
#include "platform_config.h"
#include "platform.h"
#include "wwd_platform_common.h"
#include "network/wwd_buffer_interface.h"
#include "platform/wwd_platform_interface.h"
#include "platform/wwd_bus_interface.h"
#include "platform/wwd_spi_interface.h"
#include "chip.h"

/******************************************************
 *             Constants
 ******************************************************/

/******************************************************
 *             Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

void        transfer_complete_callback( void );
static void spi_irq_handler           ( void* arg );

/******************************************************
 *             Variables
 ******************************************************/
extern const platform_spi_t        platform_spi_peripherals[];
extern const platform_gpio_t       platform_spi_chip_selects[];

/* Externed from platforms/<Platform>/platform.c */
const static platform_spi_config_t wlan_spi_config =
{
    .speed       = 5000000,
    .bits        = 8,
    .mode        = SPI_CLOCK_RISING_EDGE | SPI_CLOCK_IDLE_LOW | SPI_MSB_FIRST,
    .chip_select = &platform_spi_chip_selects[ PLATFORM_SPI_CS_WLAN ]
};
extern const platform_spi_t        platform_spi_peripherals[];

/******************************************************
 *             Function declarations
 ******************************************************/

/******************************************************
 *             Function definitions
 ******************************************************/

wwd_result_t host_platform_bus_init( void )
{
    /* Setup the interrupt input for WLAN_IRQ */
    platform_gpio_init( &wifi_spi_pins[WWD_PIN_SPI_IRQ], INPUT_HIGH_IMPEDANCE );
    //platform_gpio_irq_enable( &wifi_control_pins[WWD_PIN_IRQ], IRQ_TRIGGER_RISING_EDGE, spi_irq_handler, 0 );

#ifdef WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_0
    /* Set GPIO_B[1:0] to 01 to put WLAN module into gSPI mode */
    platform_gpio_init( &wifi_control_pins[WWD_PIN_BOOTSTRAP_0], OUTPUT_PUSH_PULL );
    platform_gpio_output_high( &wifi_control_pins[WWD_PIN_BOOTSTRAP_0] );
#endif
#ifdef WICED_WIFI_USE_GPIO_FOR_BOOTSTRAP_1
    platform_gpio_init( &wifi_control_pins[WWD_PIN_BOOTSTRAP_1], OUTPUT_PUSH_PULL );
    platform_gpio_output_low( &wifi_control_pins[WWD_PIN_BOOTSTRAP_1] );
    #endif

    return WICED_SUCCESS;
}

wwd_result_t host_platform_bus_deinit( void )
{
    return WICED_SUCCESS;
}

wwd_result_t host_platform_spi_transfer( wwd_bus_transfer_direction_t direction, uint8_t* buffer, uint16_t buffer_length )
{
    const platform_spi_t*       wlan_spi_base = &platform_spi_peripherals[WWD_SPI];
    wiced_result_t              result   = WWD_SUCCESS;
    platform_spi_message_segment_t segment;

    platform_spi_init( wlan_spi_base, &wlan_spi_config );

    segment.length    = buffer_length;
    segment.tx_buffer = buffer;
    segment.rx_buffer = direction == BUS_WRITE  ? NULL : buffer;

    result = platform_spi_transfer( wlan_spi_base, &wlan_spi_config, &segment, 1);

    platform_spi_deinit( wlan_spi_base );

    return result;
}

void transfer_complete_callback( void )
{
}

wwd_result_t host_platform_bus_enable_interrupt( void )
{
    platform_gpio_irq_enable( &wifi_spi_pins[WWD_PIN_SPI_IRQ], IRQ_TRIGGER_RISING_EDGE, spi_irq_handler, 0 );
    return  WICED_SUCCESS;
}

wwd_result_t host_platform_bus_disable_interrupt( void )
{
    platform_gpio_irq_disable( &wifi_spi_pins[WWD_PIN_SPI_IRQ] );
    return  WICED_SUCCESS;
}

void host_platform_bus_buffer_freed( wwd_buffer_dir_t direction )
{
    UNUSED_PARAMETER( direction );
    spi_irq_handler( &direction );
}

/******************************************************
 *             IRQ Handler definitions
 ******************************************************/

static void spi_irq_handler( void* arg )
{
    UNUSED_PARAMETER( arg );
    wwd_thread_notify_irq( );
}
