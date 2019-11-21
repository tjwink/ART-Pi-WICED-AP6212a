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
 *  Broadcom WLAN SDIO Protocol interface
 *
 *  Implements the WWD Bus Protocol Interface for SDIO
 *  Provides functions for initialising, de-intitialising 802.11 device,
 *  sending/receiving raw packets etc
 */

#include <string.h> /* For memcpy */
#include "wwd_assert.h"
#include "network/wwd_network_constants.h"
#include "network/wwd_buffer_interface.h"
#include "internal/wwd_sdpcm.h"
#include "internal/wwd_internal.h"
#include "RTOS/wwd_rtos_interface.h"
#include "platform/wwd_platform_interface.h"
#include "internal/bus_protocols/wwd_bus_protocol_interface.h"
#include "wwd_bus_internal.h"
#include "platform_mcu_peripheral.h"
#include "chip_constants.h"
#include "wiced_resource.h"   /* TODO: remove include dependency */
#include "resources.h"        /* TODO: remove include dependency */
#include "wifi_nvram_image.h" /* TODO: remove include dependency */
#include "wiced_utilities.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define ROUNDUP( x, y ) ((((x) + ((y) - 1)) / (y)) * (y))
#define htol32( i )     ( i )

/******************************************************
 *                    Constants
 ******************************************************/

#define WLAN_MEMORY_SIZE  ( 512 * 1024 )
#define WLAN_ADDR         ( 0x680000 )
#define RAM_SZ            ( 0x80000 )

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

static wwd_result_t boot_wlan( void );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static uint32_t     fake_backplane_window_addr          = 0;
static wiced_bool_t wwd_bus_flow_controlled             = WICED_FALSE;
static volatile wiced_bool_t resource_download_aborted  = WICED_FALSE;
/******************************************************
 *               Function Definitions
 ******************************************************/

/* Device data transfer functions */
wwd_result_t wwd_bus_send_buffer( wiced_buffer_t buffer )
{
    host_buffer_add_remove_at_front(&buffer, sizeof(wwd_buffer_header_t));
    wwd_bus_dma_transmit( buffer, host_buffer_get_current_piece_size( buffer ) );
    return WWD_SUCCESS;
}

void wwd_bus_set_resource_download_halt( wiced_bool_t halt )
{
    resource_download_aborted = halt;
}

wwd_result_t wwd_bus_write_wifi_firmware_image( void )
{
#ifndef NO_WIFI_FIRMWARE
    uint32_t offset = 0;
    uint32_t total_size;
    uint32_t reset_inst = 0;
    uint32_t size_read;

    total_size = (uint32_t) resource_get_size( &wifi_firmware_image );

    resource_read ( &wifi_firmware_image, 0, 4, &size_read, &reset_inst );

    while ( total_size > offset )
    {
        if ( resource_download_aborted == WICED_TRUE )
        {
            return WWD_UNFINISHED;
        }
        resource_read ( &wifi_firmware_image, 0, WLAN_MEMORY_SIZE, &size_read, (uint8_t *)(WLAN_ADDR+offset) );
        offset += size_read;
    }

    /*
     * copy reset vector to FLOPS
     * WLAN Address = {Programmable Register[31:18],
     * Current Transaction's HADDR[17:0]}
     */
    wwd_bus_write_reset_instruction( reset_inst );
#else
    wiced_assert("wifi_firmware_image is not included resource build", 0 == 1);
#endif
    return WWD_SUCCESS;
}

wwd_result_t wwd_bus_write_wifi_nvram_image( void )
{
    uint32_t varsize, varaddr, ramsz, phys_size, varsizew;
    uint32_t wlan_mem_start_addr;

    /* RAM size is 512K */
    ramsz = RAM_SZ;
    wlan_mem_start_addr = 0x680000;
    varsize = ROUNDUP(sizeof(wifi_nvram_image), 4);

    varaddr = (ramsz - 4) - varsize;

    varaddr += wlan_mem_start_addr;

    /* write Vars into WLAN RAM */
    memcpy((uint8_t *)varaddr, wifi_nvram_image, varsize);

    phys_size = ramsz;

    phys_size += wlan_mem_start_addr;

    varsize = ((phys_size - 4) - varaddr);
    varsizew = varsize / 4;
    varsizew = (~varsizew << 16) | (varsizew & 0x0000FFFF);
    varsizew = htol32(varsizew);
    memcpy((uint8_t *)(phys_size - 4), (uint8_t*)&varsizew, 4);

    return WWD_SUCCESS;
}

wwd_result_t wwd_bus_init( void )
{
    wwd_result_t result = WWD_SUCCESS;

    host_platform_reset_wifi( WICED_TRUE );
    host_rtos_delay_milliseconds( 1 );
    host_platform_power_wifi( WICED_FALSE );
    wwd_bus_prepare_firmware_download( );
    result = boot_wlan( );

    if ( result == WWD_SUCCESS )
    {
     /*
         * The enabling of SDIO internal clock is done in WLAN firmware.
         * Doing many access across AXI-bridge without proper sequencing will lead more instability
         */
        wwd_bus_dma_init( );

        /* Reinitialise STDIO UART if WLAN UART (UART4) is used */
        platform_reinit_wlan_stdio_uart( );
    }

    return result;
}

wwd_result_t wwd_bus_deinit( void )
{
    wwd_bus_dma_deinit();
    /* put device in reset. */
    host_platform_reset_wifi( WICED_TRUE );
    resource_download_aborted = WICED_FALSE;

    return WWD_SUCCESS;
}

uint32_t wwd_bus_packet_available_to_read( void )
{
    /* Tell WWD thread there's always a packet to read */
    return 1;
}

wwd_result_t wwd_bus_read_frame( wiced_buffer_t* buffer )
{
    uint32_t  intstatus;
    uint16_t* hwtag;

    intstatus = wwd_bus_get_dma_interrupt_status( );

    /* Handle DMA interrupts */
    if ( intstatus & DMA_TRANSMIT_INTERRUPT )
    {
        wwd_bus_reclaim_dma_tx_packets( );
    }

    if ( wwd_bus_get_available_dma_rx_buffer_space( ) == 0 )
    {
        wwd_bus_refill_dma_rx_buffer( );
        if ( wwd_bus_get_available_dma_rx_buffer_space( ) != 0 )
        {
            wwd_bus_unmask_dma_interrupt();
        }
    }


    /* Handle DMA errors */
    if ( intstatus & DMA_ERROR_MASK )
    {
        wwd_bus_refill_dma_rx_buffer( );
    }
    /* Handle DMA receive interrupt */
    *buffer = wwd_bus_dma_receive( &hwtag );
    if ( *buffer == NULL )
    {
        if ( wwd_bus_get_available_dma_rx_buffer_space( ) != 0 )
        {
            wwd_bus_unmask_dma_interrupt();
        }

        return WWD_NO_PACKET_TO_RECEIVE;
    }


    host_buffer_add_remove_at_front( buffer, -(int) sizeof(wwd_buffer_header_t) );
    wwd_sdpcm_update_credit((uint8_t*)hwtag);

    wwd_bus_refill_dma_rx_buffer( );
    /* where are buffers from dma_rx and dma_getnextrxp created? */

    return WWD_SUCCESS;



}

wwd_result_t wwd_ensure_wlan_bus_is_up( void )
{
    platform_pmu_wifi_needed( );
    return WWD_SUCCESS;
}

wwd_result_t wwd_allow_wlan_bus_to_sleep( void )
{
    platform_pmu_wifi_not_needed( );
    return WWD_SUCCESS;
}

wwd_result_t wwd_bus_set_flow_control( uint8_t value )
{
    if ( value != 0 )
    {
        wwd_bus_flow_controlled = WICED_TRUE;
    }
    else
    {
        wwd_bus_flow_controlled = WICED_FALSE;
    }
    return WWD_SUCCESS;
}

wiced_bool_t wwd_bus_is_flow_controlled( void )
{
    return wwd_bus_flow_controlled;
}

wwd_result_t wwd_bus_poke_wlan( void )
{
    return WWD_SUCCESS;
}

wwd_result_t wwd_bus_set_backplane_window( uint32_t addr )
{
    /* No such thing as a backplane window on 4390 */
    fake_backplane_window_addr = addr & (~((uint32_t)BACKPLANE_ADDRESS_MASK));
    return WWD_SUCCESS;
}

wwd_result_t wwd_bus_transfer_bytes( wwd_bus_transfer_direction_t direction, wwd_bus_function_t function, uint32_t address, uint16_t size, /*@in@*/ /*@out@*/ wwd_transfer_bytes_packet_t* data )
{
    if ( function != BACKPLANE_FUNCTION )
    {
        wiced_assert( "Only backplane available on 4390", 0 != 0 );
        return WWD_DOES_NOT_EXIST;
    }

    if ( direction == BUS_WRITE )
    {
        memcpy( (uint8_t *)(WLAN_ADDR + address + fake_backplane_window_addr), data->data, size );
        if ( address == 0 )
        {
            uint32_t resetinst = *((uint32_t*)data->data);
            wwd_bus_write_reset_instruction( resetinst );
        }
    }
    else
    {
        memcpy( data->data, (uint8_t *)(WLAN_ADDR + address + fake_backplane_window_addr), size );
    }
    return WWD_SUCCESS;
}

/* This function is needed so that the MTU size does not get compiled into the 4390 prebuilt library in releases */
uint32_t wwd_bus_get_rx_packet_size( void )
{
    return WICED_LINK_MTU;
}

static wwd_result_t boot_wlan( void )
{
    wwd_result_t result = WWD_SUCCESS;

#ifdef MFG_TEST_ALTERNATE_WLAN_DOWNLOAD

    UNUSED_PARAMETER(result);
    external_write_wifi_firmware_and_nvram_image( );

#else

    /* Load wlan firmware from sflash */
    result = wwd_bus_write_wifi_firmware_image();
    if ( result == WWD_UNFINISHED )
    {
        /* for user abort, then put wifi module into known good state */
        host_platform_reset_wifi( WICED_TRUE );
        /* power wifi is a no-op, so don't need to do anything there */
    }
    if ( result != WWD_SUCCESS )
    {
        /* stop here and return control to the user */
        return result;
    }    /* Load nvram from sflash */
    wwd_bus_write_wifi_nvram_image( );

#endif /* ifdef MFG_TEST_ALTERNATE_WLAN_DOWNLOAD */

    /* init wlan uart */
    wwd_bus_init_wlan_uart();

    /* Reset ARM core */
    wwd_bus_reset_wlan_core( );

    host_rtos_delay_milliseconds( 200 );

    return WWD_SUCCESS;
}

wwd_result_t wwd_bus_write_backplane_value( uint32_t address, uint8_t register_length, uint32_t value )
{
    MEMORY_BARRIER_AGAINST_COMPILER_REORDERING();

    if ( register_length == 4 )
    {
        REGISTER_WRITE_WITH_BARRIER( uint32_t, address, value );
    }
    else if ( register_length == 2 )
    {
        REGISTER_WRITE_WITH_BARRIER( uint16_t, address, value );
    }
    else if ( register_length == 1 )
    {
        REGISTER_WRITE_WITH_BARRIER( uint8_t, address, value );
    }
    else
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

wwd_result_t wwd_bus_read_backplane_value( uint32_t address, uint8_t register_length, /*@out@*/ uint8_t* value )
{
    MEMORY_BARRIER_AGAINST_COMPILER_REORDERING();

    if ( register_length == 4 )
    {
        *((uint32_t*)value) = REGISTER_READ( uint32_t, address );
    }
    else if ( register_length == 2 )
    {
        *((uint16_t*)value) = REGISTER_READ( uint16_t, address );
    }
    else if ( register_length == 1 )
    {
        *value = REGISTER_READ( uint8_t, address );
    }
    else
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}


