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
 *
 */


#include <string.h>
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "network/wwd_buffer_interface.h"
#include "platform/wwd_resource_interface.h"
#include "network/wwd_network_constants.h"
#include "wwd_bus_protocol_interface.h"
#include "../wwd_internal.h"   /* TODO: fix include dependency */
#include "chip_constants.h"
#include "platform_toolchain.h"

#define INDIRECT_BUFFER_SIZE                    ( 1024 )
#define WWD_BUS_ROUND_UP_ALIGNMENT              ( 64 )
#ifdef WWD_DIRECT_RESOURCES
#define WWD_BUS_MAX_TRANSFER_SIZE               ( 16 * 1024 )
#else /* ifdef WWD_DIRECT_RESOURCES */
#define WWD_BUS_MAX_TRANSFER_SIZE               ( WWD_BUS_MAX_BACKPLANE_TRANSFER_SIZE )
#endif /* ifdef WWD_DIRECT_RESOURCES */

#define VERIFY_RESULT( x ) \
    { \
        wwd_result_t verify_result; \
        verify_result = (x); \
        if ( verify_result != WWD_SUCCESS ) \
        { \
            wiced_assert( "command failed", ( 0 == 1 )); \
            return verify_result; \
        } \
    }

static uint32_t                 backplane_window_current_base_address   = 0;
static volatile wiced_bool_t    resource_download_abort                 = WICED_FALSE;

static wwd_result_t download_resource( wwd_resource_t resource, uint32_t address );

void wwd_bus_init_backplane_window( void )
{
    backplane_window_current_base_address = 0;
}

__weak wwd_result_t wwd_bus_write_wifi_firmware_image( void )
{
    return download_resource( WWD_RESOURCE_WLAN_FIRMWARE, 0 );
}

__weak wwd_result_t wwd_bus_write_wifi_nvram_image( void )
{
    wwd_result_t result;
    uint32_t image_size;

    /* Get the size of the variable image */
    host_platform_resource_size( WWD_RESOURCE_WLAN_NVRAM, &image_size );

    /* Round up the size of the image */
    image_size = ROUND_UP( image_size, WWD_BUS_ROUND_UP_ALIGNMENT );

    /* Write image */
    result = download_resource( WWD_RESOURCE_WLAN_NVRAM, CHIP_RAM_SIZE - 4 - image_size );
    if ( result != WWD_SUCCESS )
    {
        return result;
    }

    /* Write the variable image size at the end */
    image_size = ( ~( image_size / 4 ) << 16 ) | ( image_size / 4 );

    result = wwd_bus_write_backplane_value( (uint32_t) ( CHIP_RAM_SIZE - 4 ), 4, image_size );
    if ( result != WWD_SUCCESS )
    {
        return result;
    }
    return WWD_SUCCESS;
}

__weak void wwd_bus_set_resource_download_halt( wiced_bool_t halt )
{
    resource_download_abort = halt;
}

#if defined( WWD_DIRECT_RESOURCES )

static wwd_result_t download_resource( wwd_resource_t resource, uint32_t address )
{
    uint32_t transfer_progress;
    uint16_t transfer_size;
    wwd_result_t result;
    const uint8_t* image;

    uint32_t image_size;
    host_platform_resource_size( resource, &image_size );

    result = host_platform_resource_size( resource, &image_size );

    if( result != WWD_SUCCESS )
    {
        WPRINT_WWD_ERROR(("Fatal error: download_resource doesn't exist\n"));
        return result;
    }

    if ( image_size <= 0 )
    {
        WPRINT_WWD_ERROR(("Fatal error: download_resource cannot load with invalid size\n"));
        return WWD_BADARG;
    }

    host_platform_resource_read_direct( resource, (const void**)&image );

    for ( transfer_progress = 0; transfer_progress < image_size; transfer_progress += transfer_size, address += transfer_size, image += transfer_size )
    {
        if ( resource_download_abort == WICED_TRUE )
        {
            WPRINT_WWD_ERROR(("Download_resource is aborted; terminating after %d iterations\n", transfer_progress));
            return WWD_UNFINISHED;
        }

        /* Set the backplane window */
        if ( WWD_SUCCESS != ( result = wwd_bus_set_backplane_window( address ) ) )
        {
            return result;
        }
        transfer_size = (uint16_t) MIN( WWD_BUS_MAX_TRANSFER_SIZE, (int) ( image_size - transfer_progress ) );

        /* Round up the size of the chunk */
        transfer_size = (uint16_t) ROUND_UP( transfer_size, WWD_BUS_ROUND_UP_ALIGNMENT );

        if ( ( result = wwd_bus_transfer_bytes( BUS_WRITE, BACKPLANE_FUNCTION, address & BACKPLANE_ADDRESS_MASK, transfer_size, (wwd_transfer_bytes_packet_t*) image ) ) != WWD_SUCCESS)
        {
            return result;
        }
#if 0
        {
            /* TODO: THIS VERIFY CODE IS CURRENTLY BROKEN - ONLY CHECKS 64 BYTES, NOT 16KB */
            /* Verify download of image data */
            uint8_t tmpbuff[64];
            if ( WWD_SUCCESS != ( result = wwd_bus_transfer_bytes( BUS_READ, BACKPLANE_FUNCTION, address & BACKPLANE_ADDRESS_MASK, 64, (wwd_transfer_bytes_packet_t*)tmpbuff ) ) )
            {
                return result;
            }
            if ( 0 != memcmp( tmpbuff, image, (size_t) 64 ) )
            {
                /* Verify failed */
                WPRINT_WWD_ERROR(("Verify of firmware/NVRAM image failed"));
            }
        }
#endif /* if 0 */
    }
    return WWD_SUCCESS;
}

#else /* ! defined( WWD_DIRECT_RESOURCES ) */

static wwd_result_t download_resource( wwd_resource_t resource, uint32_t address )
{
    uint32_t transfer_progress;

    uint32_t size;
    wwd_result_t result;

    result = host_platform_resource_size( resource, &size );

    if( result != WWD_SUCCESS )
    {
        WPRINT_WWD_ERROR(("Fatal error: download_resource doesn't exist\n"));
        return result;
    }

    if ( size <= 0 )
    {
        WPRINT_WWD_ERROR(("Fatal error: download_resource cannot load with invalid size\n"));
        return WWD_BADARG;
    }

    /* Transfer firmware image into the RAM */
    transfer_progress = 0;

    while ( transfer_progress < size )
    {
        wiced_buffer_t buffer;
        uint32_t buffer_size = INDIRECT_BUFFER_SIZE;
        uint8_t* packet;
        uint16_t transfer_size;
        uint32_t segment_size;

        do
        {
            result = host_buffer_get( &buffer, WWD_NETWORK_TX, (unsigned short) ( buffer_size + sizeof(wwd_buffer_header_t) ), WICED_FALSE );
        } while ( ( result == WWD_BUFFER_UNAVAILABLE_PERMANENT ) && ( ( buffer_size >>= 1 ) > 1 ) );

        if ( result != WWD_SUCCESS )
        {
            WPRINT_WWD_ERROR(("Fatal error: download_resource cannot allocate buffer\n"));
            return result;
        }
        packet = (uint8_t*) host_buffer_get_current_piece_data_pointer( buffer );

        host_platform_resource_read_indirect( resource, transfer_progress, packet + sizeof(wwd_buffer_header_t), buffer_size, &segment_size );

        for ( ; segment_size != 0; segment_size -= transfer_size, packet += transfer_size, transfer_progress += transfer_size, address += transfer_size )
        {
            if ( resource_download_abort == WICED_TRUE )
            {
                WPRINT_WWD_ERROR(("Download_resource is aborted; terminating after %lu iterations\n", transfer_progress));
                return WWD_UNFINISHED;
            }
            transfer_size = (uint16_t) MIN( WWD_BUS_MAX_TRANSFER_SIZE, segment_size );
            result = wwd_bus_set_backplane_window( address );
            if ( result != WWD_SUCCESS )
            {
                host_buffer_release( buffer, WWD_NETWORK_TX );
                return result;
            }
            result = wwd_bus_transfer_bytes( BUS_WRITE, BACKPLANE_FUNCTION, ( address & BACKPLANE_ADDRESS_MASK ), transfer_size, (wwd_transfer_bytes_packet_t*) ( packet + sizeof(wwd_buffer_queue_ptr_t)) );
            if ( result != WWD_SUCCESS )
            {
                host_buffer_release( buffer, WWD_NETWORK_TX );
                return result;
            }
        }

        host_buffer_release( buffer, WWD_NETWORK_TX );
    }
    return WWD_SUCCESS;
}

#endif /* if defined( WWD_DIRECT_RESOURCES ) */

/*
 * Update the backplane window registers
 */
__weak wwd_result_t wwd_bus_set_backplane_window( uint32_t addr )
{
    wwd_result_t result = WWD_BUS_WRITE_REGISTER_ERROR;
    uint32_t base = addr & ( (uint32_t) ~BACKPLANE_ADDRESS_MASK );

    if ( base == backplane_window_current_base_address )
    {
        return WWD_SUCCESS;
    }
    if ( ( base & 0xFF000000 ) != ( backplane_window_current_base_address & 0xFF000000 ) )
    {
        if ( WWD_SUCCESS != ( result = wwd_bus_write_register_value( BACKPLANE_FUNCTION, SDIO_BACKPLANE_ADDRESS_HIGH, (uint8_t) 1, ( base >> 24 ) ) ) )
        {
            return result;
        }
    }
    if ( ( base & 0x0FF0000 ) != ( backplane_window_current_base_address & 0x00FF0000 ) )
    {
        if ( WWD_SUCCESS != ( result = wwd_bus_write_register_value( BACKPLANE_FUNCTION, SDIO_BACKPLANE_ADDRESS_MID, (uint8_t) 1, ( base >> 16 ) ) ) )
        {
            return result;
        }
    }
    if ( ( base & 0x0000FF00 ) != ( backplane_window_current_base_address & 0x0000FF00 ) )
    {
        if ( WWD_SUCCESS != ( result = wwd_bus_write_register_value( BACKPLANE_FUNCTION, SDIO_BACKPLANE_ADDRESS_LOW, (uint8_t) 1, ( base >> 8 ) ) ) )
        {
            return result;
        }
    }

    backplane_window_current_base_address = base;
    return WWD_SUCCESS;
}

/* Default implementation of WWD bus resume function, which does nothing */
__weak wwd_result_t wwd_bus_resume_after_deep_sleep( void )
{
    wiced_assert( "In order to support deep-sleep platform need to implement this function", 0 );
    return WWD_UNSUPPORTED;
}

wwd_result_t wwd_bus_transfer_backplane_bytes( wwd_bus_transfer_direction_t direction, uint32_t address, uint32_t size, /*@in@*/ /*@out@*/ uint8_t* data )
{
    wiced_buffer_t pkt_buffer = NULL;
    uint8_t*       packet;
    uint32_t       transfer_size;
    uint32_t       remaining_buf_size;
    wwd_result_t   result;

    result = host_buffer_get( &pkt_buffer, ( direction == BUS_READ )? WWD_NETWORK_RX : WWD_NETWORK_TX,
                  (uint16_t) ( WWD_BUS_MAX_BACKPLANE_TRANSFER_SIZE + WWD_BUS_BACKPLANE_READ_PADD_SIZE +
                  WWD_BUS_HEADER_SIZE ), WICED_TRUE );
    if ( result != WWD_SUCCESS )
    {
        goto done;
    }
    packet = (uint8_t*) host_buffer_get_current_piece_data_pointer( pkt_buffer );

    result = wwd_ensure_wlan_bus_is_up();
    if ( result != WWD_SUCCESS )
    {
        goto done;
    }

    result = wwd_bus_set_backplane_window( address );
    if ( result != WWD_SUCCESS )
    {
        goto done;
    }

    remaining_buf_size = size;
    for (; remaining_buf_size != 0; remaining_buf_size -= transfer_size, address += transfer_size )
    {
        transfer_size = ( remaining_buf_size > WWD_BUS_MAX_BACKPLANE_TRANSFER_SIZE ) ?
                              WWD_BUS_MAX_BACKPLANE_TRANSFER_SIZE : remaining_buf_size;

        if ( direction == BUS_WRITE )
        {
            memcpy( packet + WWD_BUS_HEADER_SIZE, data + size - remaining_buf_size, transfer_size );
            result = wwd_bus_transfer_bytes( direction, BACKPLANE_FUNCTION, ( address & BACKPLANE_ADDRESS_MASK ),
                        (uint16_t) transfer_size, (wwd_transfer_bytes_packet_t *) packet );
            if ( result != WWD_SUCCESS )
            {
                goto done;
            }
        }
        else
        {
            result = wwd_bus_transfer_bytes( direction, BACKPLANE_FUNCTION, ( address & BACKPLANE_ADDRESS_MASK ),
                        (uint16_t) ( transfer_size + WWD_BUS_BACKPLANE_READ_PADD_SIZE ),
                        (wwd_transfer_bytes_packet_t *) packet );
            if ( result != WWD_SUCCESS )
            {
                goto done;
            }
            memcpy( data + size - remaining_buf_size,
                    packet + WWD_BUS_HEADER_SIZE + WWD_BUS_BACKPLANE_READ_PADD_SIZE, transfer_size );
        }
    }

done:
    wwd_bus_set_backplane_window( CHIPCOMMON_BASE_ADDRESS );
    if ( pkt_buffer != NULL )
    {
        host_buffer_release( pkt_buffer, ( direction == BUS_READ )? WWD_NETWORK_RX : WWD_NETWORK_TX );
    }
    return result;
}
