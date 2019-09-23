/*
 * Broadcom Proprietary and Confidential. Copyright 2016 Broadcom
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
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
#include "network/wwd_buffer_interface.h"
#include "internal/wwd_sdpcm.h"
#include "internal/wwd_internal.h"
#include "RTOS/wwd_rtos_interface.h"
#include "platform/wwd_platform_interface.h"
#include "internal/bus_protocols/wwd_bus_protocol_interface.h"
#include "chip_constants.h"
#include "wiced_resource.h"         /* TODO: remove include dependency */
#include "wiced_deep_sleep.h"
#include "resources.h"        /* TODO: remove include dependency */
#include "platform_map.h"
#include "platform_mcu_peripheral.h"
#include "wifi_nvram_image.h" /* TODO: remove include dependency */
#include "wiced_utilities.h"
#include "wiced_resource.h"
#include "platform_m2m.h"
#include "platform_peripheral.h"
#include "platform/wwd_bus_interface.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define VERIFY_RESULT( x )   { wwd_result_t verify_result; verify_result = (x); if ( verify_result != WWD_SUCCESS ) return verify_result; }

#define ROUNDUP(x, y)        ((((x) + ((y) - 1)) / (y)) * (y))

/******************************************************
 *             Constants
 ******************************************************/

#define I_XI                                         (1 << 24) /* Transmit Interrupt */

#define WLAN_RAM_STARTING_ADDRESS                    PLATFORM_ATCM_RAM_BASE(0x0)

#define FF_ROM_SHADOW_INDEX_REGISTER                 *((volatile uint32_t*)(PLATFORM_WLANCR4_REGBASE(0x080)))
#define FF_ROM_SHADOW_DATA_REGISTER                  *((volatile uint32_t*)(PLATFORM_WLANCR4_REGBASE(0x084)))

#ifndef PLATFORM_WLAN_DMA_RX_UNDERFLOW_THRESH
#define PLATFORM_WLAN_DMA_RX_UNDERFLOW_THRESH        3
#endif

#ifndef PLATFORM_WLAN_ALLOW_BUS_TO_SLEEP_DELAY_MS
#define PLATFORM_WLAN_ALLOW_BUS_TO_SLEEP_DELAY_MS    10
#endif

#define PLATFORM_WLAN_ALLOW_BUS_TO_SLEEP_INVALID_MS  ((uint32_t)-1)

/******************************************************
 *             Structures
 ******************************************************/

/******************************************************
 *             Variables
 ******************************************************/

/* Variables represent state and has to be reset in wwd_bus_deinit() */
static wiced_bool_t bus_is_up = WICED_FALSE;
static wiced_bool_t WICED_DEEP_SLEEP_SAVED_VAR(wwd_bus_flow_controlled) = WICED_FALSE;
static uint32_t fake_backplane_window_addr                              = 0;
static volatile wiced_bool_t refill_underflow                           = WICED_FALSE;
static volatile wiced_bool_t resource_download_aborted                  = WICED_FALSE;

#if PLATFORM_WLAN_ALLOW_BUS_TO_SLEEP_DELAY_MS
static wwd_time_t delayed_bus_release_deadline                          = 0;
static wiced_bool_t delayed_bus_release_scheduled                       = WICED_FALSE;
static uint32_t delayed_bus_release_timeout_ms                          = PLATFORM_WLAN_ALLOW_BUS_TO_SLEEP_DELAY_MS;
static volatile uint32_t delayed_bus_release_timeout_ms_request         = PLATFORM_WLAN_ALLOW_BUS_TO_SLEEP_INVALID_MS;
#define DELAYED_BUS_RELEASE_SCHEDULE(schedule) do { delayed_bus_release_scheduled = schedule; delayed_bus_release_deadline = 0; } while(0)
#else
#define DELAYED_BUS_RELEASE_SCHEDULE(schedule) do {} while(0)
#endif

/******************************************************
 *             Function declarations
 ******************************************************/

static wwd_result_t boot_wlan( void );
static wwd_result_t shutdown_wlan(void);
static void write_reset_instruction( uint32_t reset_inst );

/******************************************************
 *             Function definitions
 ******************************************************/

wwd_result_t wwd_bus_send_buffer( wiced_buffer_t buffer )
{
    m2m_dma_tx_data( buffer );
    DELAYED_BUS_RELEASE_SCHEDULE( WICED_TRUE );
    return WWD_SUCCESS;
}

wwd_result_t wwd_bus_init( void )
{
    wwd_result_t result = WWD_SUCCESS;

    PLATFORM_WLAN_POWERSAVE_RES_UP();
    result = boot_wlan();
    PLATFORM_WLAN_POWERSAVE_RES_DOWN( NULL, WICED_FALSE );

    platform_watchdog_kick();

    if ( result == WWD_SUCCESS )
    {
        M2M_INIT_DMA_SYNC();
    }

    return result;
}

wwd_result_t wwd_bus_resume_after_deep_sleep( void )
{
    M2M_INIT_DMA_SYNC();

    return WWD_SUCCESS;
}

wwd_result_t wwd_bus_deinit( void )
{
    while ( wwd_allow_wlan_bus_to_sleep() == WWD_PENDING )
    {
        host_rtos_delay_milliseconds( 1 );
    }

    PLATFORM_WLAN_POWERSAVE_RES_UP();

    /* Down M2M and WLAN */
    m2m_deinit_dma();
    shutdown_wlan();

    /* Put WLAN to reset. */
    host_platform_reset_wifi( WICED_TRUE );

    PLATFORM_WLAN_POWERSAVE_RES_DOWN( NULL, WICED_FALSE );

    /* Force resource down even if resource up/down is unbalanced */
    PLATFORM_WLAN_POWERSAVE_RES_DOWN( NULL, WICED_TRUE );

    /* Reset all state variables */
    bus_is_up = WICED_FALSE;
    wwd_bus_flow_controlled = WICED_FALSE;
    fake_backplane_window_addr = 0;
    refill_underflow = WICED_FALSE;
    resource_download_aborted = WICED_FALSE;
    DELAYED_BUS_RELEASE_SCHEDULE( WICED_FALSE );

    return WWD_SUCCESS;
}

uint32_t wwd_bus_packet_available_to_read(void)
{
    return 1;
}

static void wwd_bus_refill_dma( void )
{
    /*
     * Set flag and clear it later if filled above threshold.
     * Done this way if packet is freed during m2m_refill_dma()
     * it may be too late to set flag.
     */

    refill_underflow = WICED_TRUE;

    m2m_refill_dma( );

    if ( m2m_rxactive_dma() > PLATFORM_WLAN_DMA_RX_UNDERFLOW_THRESH )
    {
        refill_underflow = WICED_FALSE;
    }
}

wwd_result_t wwd_bus_read_frame( wiced_buffer_t* buffer )
{
    wwd_result_t result = WWD_SUCCESS;
    uint32_t     intstatus;
    uint16_t*    hwtag;
    void*        packet;
    wiced_bool_t signal_txdone;

    M2M_INIT_DMA_ASYNC( );

    wwd_bus_refill_dma( );

    intstatus = m2m_read_intstatus( &signal_txdone );

    /* Tell peer about txdone event */
    if ( signal_txdone )
    {
        wwd_ensure_wlan_bus_is_up( );
        m2m_signal_txdone( );
        DELAYED_BUS_RELEASE_SCHEDULE( WICED_TRUE );
    }

    /* Handle DMA interrupts */
    if ( ( intstatus & I_XI ) != 0 )
    {
        WPRINT_WWD_INFO(("DMA: TX reclaim\n"));
        m2m_dma_tx_reclaim( );
    }

    /* Handle DMA receive interrupt */
    packet = m2m_read_dma_packet( &hwtag );
    if ( packet == NULL )
    {
        WPRINT_WWD_INFO(("intstatus: 0x%x, NO PACKET\n", (uint)intstatus));
        result = WWD_NO_PACKET_TO_RECEIVE;
    }
    else
    {
        *buffer = packet;
        WPRINT_WWD_INFO(("read pkt , p0: 0x%x\n", (uint)packet));

        /* move the data pointer 12 bytes(sizeof(wwd_buffer_header_t))
         * back to the start of the pakcet
         */
        host_buffer_add_remove_at_front( buffer, -(int)sizeof(wwd_buffer_header_t) );
        wwd_sdpcm_update_credit( (uint8_t*) hwtag );

        DELAYED_BUS_RELEASE_SCHEDULE( WICED_TRUE );
    }

    wwd_bus_refill_dma( );

    return result;
}

wwd_result_t wwd_ensure_wlan_bus_is_up( void )
{
    if ( bus_is_up == WICED_FALSE )
    {
        M2M_POWERSAVE_COMM_TX_BEGIN( );
        bus_is_up = WICED_TRUE;
    }

    return WWD_SUCCESS;
}

wwd_result_t wwd_allow_wlan_bus_to_sleep( void )
{
    wwd_result_t result = WWD_SUCCESS;

    if ( bus_is_up == WICED_TRUE )
    {
        if ( M2M_POWERSAVE_COMM_TX_END( ) )
        {
            bus_is_up = WICED_FALSE;
        }
        else
        {
            /*
             * Not able to finish communications now, let's try later.
             * Returning this error code tells that WWD thread must not sleep and return to calling us as soon as possible.
             */
            result = WWD_PENDING;
        }
    }

    return result;
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

void platform_wlan_powersave_set_delayed_release_milliseconds( uint32_t time_ms )
{
#if PLATFORM_WLAN_ALLOW_BUS_TO_SLEEP_DELAY_MS

    delayed_bus_release_timeout_ms_request = time_ms;
    wwd_thread_notify( );

#else

    UNUSED_PARAMETER( time_ms );

#endif /* PLATFORM_WLAN_ALLOW_BUS_TO_SLEEP_DELAY_MS */
}

static uint32_t wwd_handle_delayed_bus_release( void )
{
    uint32_t time_until_release = 0;

#if PLATFORM_WLAN_ALLOW_BUS_TO_SLEEP_DELAY_MS

    if ( delayed_bus_release_timeout_ms_request != PLATFORM_WLAN_ALLOW_BUS_TO_SLEEP_INVALID_MS )
    {
        wiced_bool_t schedule = ( ( delayed_bus_release_scheduled != 0 ) || ( delayed_bus_release_deadline != 0 ) ) ? WICED_TRUE : WICED_FALSE;
        uint32_t     flags;

        WICED_SAVE_INTERRUPTS( flags );
        delayed_bus_release_timeout_ms         = delayed_bus_release_timeout_ms_request;
        delayed_bus_release_timeout_ms_request = PLATFORM_WLAN_ALLOW_BUS_TO_SLEEP_INVALID_MS;
        WICED_RESTORE_INTERRUPTS( flags );

        DELAYED_BUS_RELEASE_SCHEDULE( schedule );
    }

    if ( delayed_bus_release_scheduled )
    {
        delayed_bus_release_scheduled = WICED_FALSE;

        if ( delayed_bus_release_timeout_ms != 0 )
        {
            delayed_bus_release_deadline = host_rtos_get_time() + delayed_bus_release_timeout_ms;
            time_until_release = delayed_bus_release_timeout_ms;
        }
    }
    else if ( delayed_bus_release_deadline )
    {
        wwd_time_t now = host_rtos_get_time( );

        if ( delayed_bus_release_deadline - now <= delayed_bus_release_timeout_ms )
        {
            time_until_release = delayed_bus_release_deadline - now;
        }

        if ( time_until_release == 0 )
        {
            delayed_bus_release_deadline = 0;
        }
    }

    if ( time_until_release != 0 )
    {
        if ( !bus_is_up )
        {
            time_until_release = 0;
        }
        else if ( platform_mcu_powersave_is_permitted( ) && (platform_mcu_powersave_get_mode( ) == PLATFORM_MCU_POWERSAVE_MODE_DEEP_SLEEP ) )
        {
            time_until_release = 0;
        }
    }
#endif /* PLATFORM_WLAN_ALLOW_BUS_TO_SLEEP_DELAY_MS */

    return time_until_release;
}

void wwd_wait_for_wlan_event( host_semaphore_type_t* transceive_semaphore )
{
    uint32_t timeout_ms = 1;
    uint32_t delayed_release_timeout_ms;
    wwd_result_t result;

    REFERENCE_DEBUG_ONLY_VARIABLE( result );

    delayed_release_timeout_ms = wwd_handle_delayed_bus_release( );
    if ( delayed_release_timeout_ms != 0 )
    {
        timeout_ms = delayed_release_timeout_ms;
    }
    else
    {
        result = wwd_allow_wlan_bus_to_sleep( );
        wiced_assert( "Error setting wlan sleep", ( result == WWD_SUCCESS ) || ( result == WWD_PENDING ) );

        if ( result == WWD_SUCCESS )
        {
            timeout_ms = NEVER_TIMEOUT;
        }
    }

#ifdef M2M_RX_POLL_MODE

    UNUSED_PARAMETER( transceive_semaphore );
    UNUSED_VAR( timeout_ms );
    UNUSED_VAR( delayed_release_timeout_ms );
    host_rtos_delay_milliseconds( 10 );

#else

    host_platform_bus_enable_interrupt( );

    result = host_rtos_get_semaphore( transceive_semaphore, timeout_ms, WICED_FALSE );
    wiced_assert("Could not get wwd sleep semaphore\n", ( result == WWD_SUCCESS ) || ( result == WWD_TIMEOUT ) );

#endif /* M2M_RX_POLL_MODE */
}

static void wwd_notify_thread_atomically( void )
{
    uint32_t flags;

    WICED_SAVE_INTERRUPTS( flags );

    wwd_thread_notify_irq();

    WICED_RESTORE_INTERRUPTS( flags );
}

void host_platform_bus_buffer_freed( wwd_buffer_dir_t direction )
{
    if ( direction == WWD_NETWORK_RX )
    {
        if ( refill_underflow )
        {
            refill_underflow = WICED_FALSE;
            wwd_notify_thread_atomically();
        }
    }
}

/*
 * copy reset vector to FLOPS
 * WLAN Address = {Programmable Register[31:18],
 * Current Transaction's HADDR[17:0]}
 */
static void write_reset_instruction( uint32_t reset_instruction )
{
    FF_ROM_SHADOW_INDEX_REGISTER = 0x0;
    FF_ROM_SHADOW_DATA_REGISTER  = reset_instruction;
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
    uint32_t size_read;
    uint32_t reset_instruction;

    /* Halt ARM and remove from reset */
    WPRINT_WWD_INFO(("Reset wlan core..\n"));
    VERIFY_RESULT( wwd_reset_device_core( WLAN_ARM_CORE, WLAN_CORE_FLAG_CPU_HALT ) );

    total_size = (uint32_t) resource_get_size( &wifi_firmware_image );

    resource_read ( &wifi_firmware_image, 0, 4, &size_read, &reset_instruction );

    while ( total_size > offset )
    {
        if ( resource_download_aborted == WICED_TRUE )
        {
            return WWD_UNFINISHED;
        }

        resource_read ( &wifi_firmware_image, 0, PLATFORM_WLAN_RAM_SIZE, &size_read, (uint8_t *)(WLAN_RAM_STARTING_ADDRESS+offset) );
        offset += size_read;
    }

    WPRINT_WWD_INFO(("load_wlan_fw: write reset_inst : 0x%x\n", (uint)reset_instruction));

    write_reset_instruction( reset_instruction );
#else
    wiced_assert("wifi_firmware_image is not included resource build", 0 == 1);
#endif

    return WWD_SUCCESS;
}

/*
 * Load the nvram to the bottom of the WLAN TCM
 */
wwd_result_t wwd_bus_write_wifi_nvram_image( void )
{
    uint32_t nvram_size;
    uint32_t nvram_destination_address;
    uint32_t nvram_size_in_words;

    nvram_size = ROUNDUP(sizeof(wifi_nvram_image), 4);

    /* Put the NVRAM image at the end of RAM leaving the last 4 bytes for the size */
    nvram_destination_address = ( PLATFORM_WLAN_RAM_SIZE - 4 ) - nvram_size;
    nvram_destination_address += WLAN_RAM_STARTING_ADDRESS;

    /* Write NVRAM image into WLAN RAM */
    memcpy( (uint8_t *) nvram_destination_address, wifi_nvram_image, sizeof( wifi_nvram_image ) );

    /* Calculate the NVRAM image size in words (multiples of 4 bytes) and its bitwise inverse */
    nvram_size_in_words = nvram_size / 4;
    nvram_size_in_words = ( ~nvram_size_in_words << 16 ) | ( nvram_size_in_words & 0x0000FFFF );

    memcpy( (uint8_t*) ( WLAN_RAM_STARTING_ADDRESS + PLATFORM_WLAN_RAM_SIZE - 4 ), (uint8_t*) &nvram_size_in_words, 4 );

    return WWD_SUCCESS;
}

#ifdef WWD_TEST_NVRAM_OVERRIDE
wwd_result_t wwd_bus_get_wifi_nvram_image( char** nvram, uint32_t* size)
{
    if (nvram == NULL || size == NULL)
    {
        return WICED_ERROR;
    }

    *nvram = (char *)wifi_nvram_image;
    *size  = sizeof(wifi_nvram_image);

    return WWD_SUCCESS;
}
#endif

static wwd_result_t boot_wlan(void)
{
    wwd_result_t result = WICED_SUCCESS;

    /* Load WLAN firmware & NVRAM */
#ifdef MFG_TEST_ALTERNATE_WLAN_DOWNLOAD

    UNUSED_PARAMETER(result);
    VERIFY_RESULT( external_write_wifi_firmware_and_nvram_image( ) );

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
    }
    VERIFY_RESULT( wwd_bus_write_wifi_nvram_image( ) );

#endif /* ifdef MFG_TEST_ALTERNATE_WLAN_DOWNLOAD */

    /* Release ARM core */
    WPRINT_WWD_INFO(("Release WLAN core..\n"));
    VERIFY_RESULT( wwd_wlan_armcore_run( WLAN_ARM_CORE, WLAN_CORE_FLAG_NONE ) );

    return WWD_SUCCESS;
}

static wwd_result_t shutdown_wlan(void)
{
    /*  disable wlan core  */
    VERIFY_RESULT( wwd_disable_device_core( WLAN_ARM_CORE, WLAN_CORE_FLAG_CPU_HALT ) );

    /* Stop wlan side M2MDMA */
    m2m_wlan_dma_deinit();

    return WWD_SUCCESS;
}

wwd_result_t wwd_bus_set_backplane_window( uint32_t addr )
{
    /* No such thing as a backplane window on 4390 */
    fake_backplane_window_addr = addr & (~((uint32_t)BACKPLANE_ADDRESS_MASK));
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

wwd_result_t wwd_bus_transfer_bytes( wwd_bus_transfer_direction_t direction, wwd_bus_function_t function, uint32_t address, uint16_t size, /*@in@*/ /*@out@*/ wwd_transfer_bytes_packet_t* data )
{
    if ( function != BACKPLANE_FUNCTION )
    {
        wiced_assert( "Only backplane available on 43909", 0 != 0 );
        return WWD_DOES_NOT_EXIST;
    }

    if ( direction == BUS_WRITE )
    {
        memcpy( (uint8_t *)(address + fake_backplane_window_addr), data->data, size );
        if ( address == 0 )
        {
            uint32_t resetinst = *((uint32_t*)data->data);
            write_reset_instruction( resetinst );
        }
    }
    else
    {
        memcpy( data->data, (uint8_t *)(address + fake_backplane_window_addr), size );
    }
    return WWD_SUCCESS;
}
