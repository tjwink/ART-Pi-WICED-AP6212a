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

#include <string.h>  /* For memcpy */
#include "wwd_assert.h"
#include "platform/wwd_platform_interface.h"
#include "platform/wwd_spi_interface.h"
#include "network/wwd_network_constants.h"
#include "network/wwd_buffer_interface.h"
#include "RTOS/wwd_rtos_interface.h"
#include "internal/bus_protocols/wwd_bus_protocol_interface.h"
#include "internal/wwd_internal.h"
#include "internal/wwd_bcmendian.h"
#include "internal/wwd_sdpcm.h"
#include "wwd_bus_protocol.h"
#include "chip_constants.h"
#ifdef NO_SUPPORT_FOR_HIGHSPEED_MODE
#include "platform/wwd_bus_interface.h"
#endif

/******************************************************
 *             Constants
 ******************************************************/

#define F2_READY_TIMEOUT_MS    (1000)
#define F2_READY_TIMEOUT_LOOPS (1000)
#define F1_READY_TIMEOUT_LOOPS (1000)
#ifdef NO_SUPPORT_FOR_HIGHSPEED_MODE
#define FEADBEAD_TIMEOUT_MS    (500)
#else
#define FEADBEAD_TIMEOUT_MS    (5000)
#endif
#define ALP_AVAIL_TIMEOUT_MS   (100)

/* function 1 OCP space */
#define SBSDIO_SB_OFT_ADDR_MASK  0x07FFF /* sb offset addr is <= 15 bits, 32k */
#define SBSDIO_SB_OFT_ADDR_LIMIT  0x08000
#define SBSDIO_SB_ACCESS_2_4B_FLAG  0x08000 /* with b15, maps to 32-bit SB access */

#ifndef HT_AVAIL_TIMEOUT_MS
#define HT_AVAIL_TIMEOUT_MS    (1000)
#endif

/* Taken from FALCON_5_90_195_26 dhd/sys/dhd_sdio.c. For 43362, MUST be >= 8 and word-aligned otherwise dongle fw crashes */
#define SPI_F2_WATERMARK       (32)

#define GSPI_PACKET_AVAILABLE  (1 << 8)
#define GSPI_UNDERFLOW         (1 << 1)

#define VERIFY_RESULT( x )     { wwd_result_t verify_result; verify_result = ( x ); if ( verify_result != WWD_SUCCESS ) return verify_result; }

#define SWAP32_16BIT_PARTS(val) ((uint32_t)(( ((uint32_t)(val)) >> 16) + ((((uint32_t)(val)) & 0xffff)<<16)))

#ifdef GSPI_USING_DSTATUS
#define WWD_BUS_GSPI_PACKET_OVERHEAD    ( sizeof( wwd_buffer_header_t ) + sizeof( uint32 ) )
#else
#define WWD_BUS_GSPI_PACKET_OVERHEAD    ( sizeof( wwd_buffer_header_t ) )
#endif

#define MAX_GSPI_TRANSFER_LEN  2048

#define H32TO16LE(x)           ( ( uint32_t ) ( ( ( ( uint32_t ) ( x ) & ( uint32_t ) 0x000000ffU ) << 8 ) | \
                                                ( ( ( uint32_t ) ( x ) & ( uint32_t ) 0x0000ff00U ) >> 8 ) | \
                                                ( ( ( uint32_t ) ( x ) & ( uint32_t ) 0x00ff0000U ) << 8 ) | \
                                                ( ( ( uint32_t ) ( x ) & ( uint32_t ) 0xff000000U ) >> 8 ) ) )

#ifndef WWD_THREAD_POLL_TIMEOUT
#define WWD_THREAD_POLL_TIMEOUT      (NEVER_TIMEOUT)
#endif /* WWD_THREAD_POLL_TIMEOUT */

#ifndef WWD_THREAD_POKE_TIMEOUT
#define WWD_THREAD_POKE_TIMEOUT      (100)
#endif /* WWD_THREAD_POKE_TIMEOUT */

typedef enum
{
   GSPI_INCREMENT_ADDRESS = 1,
   GSPI_FIXED_ADDRESS     = 0
} gspi_transfer_access_t;

/******************************************************
 *             Structures
 ******************************************************/

#pragma pack(1)

typedef struct
{
    wwd_bus_gspi_header_t      header;
    uint8_t            response_delay[4];
} gspi_backplane_f1_read_header_t;

#pragma pack()

typedef struct
{
    gspi_backplane_f1_read_header_t  gspi_header;
    uint32_t                         data[1];
} gspi_backplane_f1_read_packet_t;

/******************************************************
 *             Static variables
 ******************************************************/

static wiced_bool_t  wwd_bus_gspi_32bit = WICED_FALSE;
static const uint8_t wwd_bus_gspi_command_mapping[] =
{
    0,
    1
};

static wiced_bool_t wwd_bus_flow_controlled = WICED_FALSE;

/******************************************************
 *             Static Function Declarations
 ******************************************************/

static wwd_result_t wwd_download_firmware   ( void );
static wwd_result_t wwd_bus_transfer_buffer   ( wwd_bus_transfer_direction_t direction, wwd_bus_function_t function, uint32_t address, wiced_buffer_t buffer );

/******************************************************
 *             SPI Logging
 * Enable this section for logging of SPI transfers
 * by changing "if 0" to "if 1"
 ******************************************************/
#if 0

#define GSPI_LOG_SIZE        (110)
#define SDIO_LOG_HEADER_SIZE (0)   /*(0x30) */

typedef enum
{
    UNUSED,
    LOG_TX,
    LOG_RX
}gspi_log_direction_t;

typedef struct gSPI_log_entry_struct
{
    gspi_log_direction_t direction;
    wwd_bus_function_t       function;
    uint32_t             address;
    unsigned long        time;
    unsigned long        length;
#if ( SDIO_LOG_HEADER_SIZE != 0 )
    unsigned char        header[GSPI_LOG_HEADER_SIZE];
#endif /* if ( SDIO_LOG_HEADER_SIZE != 0 ) */
}gspi_log_entry_t;

static int               next_gspi_log_pos = 0;
static gspi_log_entry_t  gspi_log_data[GSPI_LOG_SIZE];

static void add_log_entry( gspi_log_direction_t dir, wwd_bus_function_t function, uint32_t address, unsigned long length, char* gspi_data )
{
    UNUSED_PARAMETER(gspi_data);
    gspi_log_data[next_gspi_log_pos].direction = dir;
    gspi_log_data[next_gspi_log_pos].function = function;
    gspi_log_data[next_gspi_log_pos].address = address;
    gspi_log_data[next_gspi_log_pos].time = host_rtos_get_time();
    gspi_log_data[next_gspi_log_pos].length = length;
#if ( SDIO_LOG_HEADER_SIZE != 0 )
    memcpy( gspi_log_data[next_gspi_log_pos].header, gspi_data, (length>=GSPI_LOG_HEADER_SIZE)?GSPI_LOG_HEADER_SIZE:length );
#endif /* if ( SDIO_LOG_HEADER_SIZE != 0 ) */
    next_gspi_log_pos++;
    if (next_gspi_log_pos >= GSPI_LOG_SIZE)
    {
        next_gspi_log_pos = 0;
    }
}
#else
#define add_log_entry( dir, function, address, length, gspi_data )
#endif

/******************************************************
 *             Global Function definitions
 ******************************************************/

wwd_result_t wwd_bus_send_buffer( wiced_buffer_t buffer )
{
    wwd_result_t result = wwd_bus_transfer_buffer( BUS_WRITE, WLAN_FUNCTION, 0, buffer );
    host_buffer_release( buffer, WWD_NETWORK_TX );
    if ( result == WWD_SUCCESS )
    {
        DELAYED_BUS_RELEASE_SCHEDULE( WICED_TRUE );
    }
    return result;
}

/*
 * Perform a transfer on the gSPI bus
 * Prerequisites: length < MAX_GSPI_TRANSFER_LEN
 */
static wwd_result_t wwd_bus_transfer_buffer( wwd_bus_transfer_direction_t direction, wwd_bus_function_t function, uint32_t address, wiced_buffer_t buffer )
{
    uint32_t* temp;
    wwd_result_t result;
    uint16_t newsize;
    wwd_buffer_header_t* header = (wwd_buffer_header_t*) host_buffer_get_current_piece_data_pointer( buffer );
    wwd_transfer_bytes_packet_t* gspipacket = (wwd_transfer_bytes_packet_t*) &header->bus_header;

    uint16_t size = (uint16_t) ( host_buffer_get_current_piece_size( buffer ) - sizeof( wwd_buffer_header_t ) );

#ifdef SUPPORT_BUFFER_CHAINING
#error BUFFER CHAINING NOT IMPLEMENTED IN GSPI YET
#endif /* ifdef SUPPORT_BUFFER_CHAINING */
    /* Form the gSPI header */
    gspipacket->bus_header.gspi_header = (wwd_bus_gspi_header_t) ( (uint32_t) ( ( wwd_bus_gspi_command_mapping[(int)direction] & 0x1     ) << 31 ) |
                                                           (uint32_t) ( ( GSPI_INCREMENT_ADDRESS               & 0x1     ) << 30 ) |
                                                           (uint32_t) ( ( function                             & 0x3     ) << 28 ) |
                                                           (uint32_t) ( ( address                              & 0x1FFFF ) << 11 ) |
                                                           (uint32_t) ( ( size                                 & 0x7FF   ) << 0 ) );

    /* Reshuffle the bits if we're not in 32 bit mode */
    if ( wwd_bus_gspi_32bit == WICED_FALSE )
    {
        /* Note: This typecast should always be valid if the buffer containing the GSpi packet has been correctly declared as 32-bit aligned */
        temp = (uint32_t*) &gspipacket->bus_header.gspi_header;
        *temp = H32TO16LE(*temp);
    }

    /* Round size up to 32-bit alignment */
    newsize = (uint16_t) ROUND_UP(size, 4);

    /* Send the data */
    if ( direction == BUS_WRITE )
    {
        /* Wait for FIFO to be ready to accept data */
        if ( function == WLAN_FUNCTION )
        {
            uint32_t wwd_bus_gspi_status;
            uint32_t loop_count = 0;
            while ( ( ( result = wwd_bus_read_register_value( BUS_FUNCTION, SPI_STATUS_REGISTER, (uint8_t) 4, (uint8_t*) &wwd_bus_gspi_status ) ) == WWD_SUCCESS ) &&
                    ( ( wwd_bus_gspi_status & ( 1 << 5 ) ) == 0 ) &&
                    ( loop_count < (uint32_t) F2_READY_TIMEOUT_LOOPS ) )
            {
                loop_count++;
            }
            if ( result != WWD_SUCCESS )
            {
                return result;
            }
            if ( loop_count >= (uint32_t) F2_READY_TIMEOUT_LOOPS )
            {
                WPRINT_WWD_ERROR(("Timeout waiting for data FIFO to be ready\n"));
                return WWD_TIMEOUT;
            }
        }

        add_log_entry( LOG_TX, function, address, size, (char*)&gspipacket->data );
    }
    result = host_platform_spi_transfer( direction, (uint8_t*) gspipacket, (uint16_t) ( newsize + sizeof(wwd_bus_gspi_header_t) ) );
    if ( direction == BUS_READ )
    {
        add_log_entry( LOG_RX, function, address, size, (char*)&gspipacket->data );
    }

    return result;
}

wwd_result_t wwd_bus_poke_wlan( void )
{
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

wwd_result_t wwd_bus_ack_interrupt(uint32_t intstatus)
{
    return wwd_bus_write_register_value( BUS_FUNCTION, SPI_INTERRUPT_REGISTER, (uint8_t) 2, intstatus );
}

uint32_t wwd_bus_packet_available_to_read(void)
{
    uint16_t interrupt_register;

    VERIFY_RESULT( wwd_ensure_wlan_bus_is_up());

    /* Read the interrupt register */
    if (wwd_bus_read_register_value( BUS_FUNCTION, SPI_INTERRUPT_REGISTER, (uint8_t) 2, (uint8_t*) &interrupt_register ) != WWD_SUCCESS)
    {
        goto return_with_error;
    }

    if ( ( interrupt_register & 0x0086 ) != 0 ) /* This should be 0x87, but occasional "data not available" errors are flagged seemingly for no reason */
    {
        /* Error condition detected */
        WPRINT_WWD_DEBUG(("Bus error condition detected\n"));
    }

    /* Clear interrupt register */
    if ( interrupt_register != 0 )
    {
        if (wwd_bus_write_register_value( BUS_FUNCTION, SPI_INTERRUPT_REGISTER, (uint8_t) 2, interrupt_register ) != WWD_SUCCESS)
        {
            goto return_with_error;
        }
    }

    return (uint32_t)((interrupt_register) & (F2_PACKET_AVAILABLE));

return_with_error:
    wiced_assert("Error accessing backplane", 0 != 0);
    return 0;
}

/*@only@*/ /*@null@*/ wwd_result_t wwd_bus_read_frame( wiced_buffer_t* buffer )
{
    uint32_t wwd_bus_gspi_status;
    wwd_result_t result;
    uint32_t wiced_gspi_bytes_pending;

    /* Ensure the wlan backplane bus is up */
    VERIFY_RESULT( wwd_ensure_wlan_bus_is_up() );

    do
    {
        result = wwd_bus_read_register_value( BUS_FUNCTION, SPI_STATUS_REGISTER, (uint8_t) 4, (uint8_t*) &wwd_bus_gspi_status );
        if ( result != WWD_SUCCESS )
        {
            return result;
        }
    } while ( wwd_bus_gspi_status == 0xFFFFFFFF );

    if ( ( wwd_bus_gspi_status & GSPI_PACKET_AVAILABLE ) != 0)
    {
        if ( ((( wwd_bus_gspi_status >> 9 ) & 0x7FF )== 0 ) ||
             ((( wwd_bus_gspi_status >> 9 ) & 0x7FF ) > ( WICED_LINK_MTU - WWD_BUS_GSPI_PACKET_OVERHEAD )) ||
               ( wwd_bus_gspi_status & GSPI_UNDERFLOW ))
        {
            wwd_bus_write_register_value( BACKPLANE_FUNCTION, SPI_FRAME_CONTROL, 1, ( 1 << 0 ));
            return WWD_NO_PACKET_TO_RECEIVE;
        }
    }

    wiced_gspi_bytes_pending = 0;

    if ( ( wwd_bus_gspi_status & GSPI_PACKET_AVAILABLE ) != 0 )
    {
        wiced_gspi_bytes_pending = ( ( wwd_bus_gspi_status >> 9 ) & 0x7FF );
    }

    if ( wiced_gspi_bytes_pending == 0 )
    {
        return WWD_NO_PACKET_TO_RECEIVE;
    }

    /* Allocate a suitable buffer */
#ifdef NO_SUPPORT_FOR_HIGHSPEED_MODE
    result = host_buffer_get( buffer, WWD_NETWORK_RX, (unsigned short)( wiced_gspi_bytes_pending + WWD_BUS_GSPI_PACKET_OVERHEAD), WICED_TRUE );
#else
    result = host_buffer_get( buffer, WWD_NETWORK_RX, (unsigned short)( wiced_gspi_bytes_pending + WWD_BUS_GSPI_PACKET_OVERHEAD), WICED_FALSE );
#endif
    if ( result != WWD_SUCCESS )
    {
        /* Read out the first 12 bytes to get the bus credit information */
        uint8_t temp_buffer[12 + sizeof(wwd_bus_header_t)];
        wwd_bus_transfer_bytes( BUS_READ, WLAN_FUNCTION, 0, 12, (wwd_transfer_bytes_packet_t*) temp_buffer );

        /* Abort the transfer to force the packet to be dropped */
        if ( wiced_gspi_bytes_pending > 12 )
        {
            wwd_bus_write_register_value( BACKPLANE_FUNCTION, SPI_FRAME_CONTROL, 1, ( 1 << 0 ) );
        }

        /* Process bus data credit information */
        wwd_sdpcm_update_credit( (uint8_t*) ( temp_buffer + sizeof(wwd_bus_header_t) ) );
        return result;
    }

    result = wwd_bus_transfer_buffer( BUS_READ, WLAN_FUNCTION, 0, *buffer );
    if ( result != WWD_SUCCESS)
    {
        host_buffer_release( *buffer, WWD_NETWORK_RX );
        return result;
    }

    DELAYED_BUS_RELEASE_SCHEDULE( WICED_TRUE );
    return WWD_SUCCESS;
}

wwd_result_t wwd_bus_init( void )
{
    uint32_t data = 0;
    uint32_t wwd_bus_gspi_status;
    uint16_t data16 = 0;
    uint32_t loop_count;
    wwd_result_t result;
    uint8_t init_data[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint32_t interrupt_polarity = 0;

    wwd_bus_gspi_32bit = WICED_FALSE;

#ifndef WWD_SPI_IRQ_FALLING_EDGE
    interrupt_polarity = INTERRUPT_POLARITY_HIGH;
#endif /* WWD_SPI_IRQ_FALLING_EDGE */

    wwd_bus_init_backplane_window( );

    host_platform_power_wifi( WICED_TRUE );
    host_platform_reset_wifi( WICED_TRUE );
    host_rtos_delay_milliseconds( (uint32_t) 1 );
    host_platform_reset_wifi( WICED_FALSE );

    /* Due to an chip issue, the first transfer will be corrupted.
     * This means a repeated safe read of a known value register is required until
     * the correct value is returned - signalling the bus is running.
     * This known value register must be accessed using fixed (non-incrementing) address
     * mode, hence a custom packet header must be constructed
     * Due to the chip issue, the data received could be left shifted by one bit.
     */
    loop_count = 0;
    do
    {
        wwd_bus_gspi_header_t* gspi_header = (wwd_bus_gspi_header_t*) init_data;

        *gspi_header = (wwd_bus_gspi_header_t) SWAP32_16BIT_PARTS( BCMSWAP32( (uint32_t)( ( wwd_bus_gspi_command_mapping[(int)BUS_READ] & 0x1      ) << 31) |
                                                                      (uint32_t)( ( GSPI_FIXED_ADDRESS                  & 0x1      ) << 30) |
                                                                      (uint32_t)( ( BUS_FUNCTION                        & 0x3      ) << 28) |
                                                                      (uint32_t)( ( SPI_READ_TEST_REGISTER              & 0x1FFFFu ) << 11) |
                                                                      (uint32_t)( ( 4u /*size*/                         & 0x7FFu   ) <<  0 ) ) );
        VERIFY_RESULT( host_platform_spi_transfer(BUS_READ, init_data, (uint16_t) 12 ) );
        loop_count++;
    } while ( ( NULL == memchr( &init_data[4], SPI_READ_TEST_REG_LSB,      (size_t) 8 ) ) &&
              ( NULL == memchr( &init_data[4], SPI_READ_TEST_REG_LSB_SFT1, (size_t) 8 ) ) &&
              ( NULL == memchr( &init_data[4], SPI_READ_TEST_REG_LSB_SFT2, (size_t) 8 ) ) &&
              ( NULL == memchr( &init_data[4], SPI_READ_TEST_REG_LSB_SFT3, (size_t) 8 ) ) &&
              ( loop_count < (uint32_t) FEADBEAD_TIMEOUT_MS ) &&
              ( host_rtos_delay_milliseconds( (uint32_t) 1 ), ( 1 == 1 ) ) );

    /* Keep/reset defaults for registers 0x0-0x4 except for, 0x0: Change word length to 32bit, set endianness, enable wakeup. 0x2: enable interrupt with status. */
#if defined(IL_BIGENDIAN)
    VERIFY_RESULT( wwd_bus_write_register_value(BUS_FUNCTION, SPI_BUS_CONTROL, (uint8_t) 4, (uint32_t) ( WORD_LENGTH_32 | (0 & ENDIAN_BIG) | ( interrupt_polarity & INTERRUPT_POLARITY_HIGH ) | WAKE_UP | (0x4 << (8*SPI_RESPONSE_DELAY)) | ((0 & STATUS_ENABLE) << (8*SPI_STATUS_ENABLE)) | ( INTR_WITH_STATUS << (8*SPI_STATUS_ENABLE)) ) ) );
#else
    VERIFY_RESULT( wwd_bus_write_register_value(BUS_FUNCTION, SPI_BUS_CONTROL, (uint8_t) 4, (uint32_t) ( WORD_LENGTH_32 | ENDIAN_BIG       | ( interrupt_polarity & INTERRUPT_POLARITY_HIGH ) | WAKE_UP | (0x4 << (8*SPI_RESPONSE_DELAY)) | ((0 & STATUS_ENABLE) << (8*SPI_STATUS_ENABLE)) | ( INTR_WITH_STATUS << (8*SPI_STATUS_ENABLE)) ) ) );
#endif
    wwd_bus_gspi_32bit = WICED_TRUE;
    VERIFY_RESULT( wwd_bus_read_register_value(BUS_FUNCTION, SPI_BUS_CONTROL, (uint8_t) 4, (uint8_t*)&data ) );

#ifdef NO_SUPPORT_FOR_HIGHSPEED_MODE
    /* Reset host SPI interface to re-sync */
    host_platform_bus_init( );
#endif

    /* Check feedbead can be read - i.e. the device is alive */
    data = 0;
    VERIFY_RESULT( wwd_bus_read_register_value( BUS_FUNCTION, SPI_READ_TEST_REGISTER, (uint8_t) 4, (uint8_t*) &data ) );

    if ( data != SPI_READ_TEST_REGISTER_VALUE )
    {
        WPRINT_WWD_ERROR(("Read %x, instead of 0xFEEDBEAD from the WLAN chip\n", (unsigned int)data ));
        return WWD_SPI_ID_READ_FAIL;
    }

    /* Make sure error interrupt bits are clear */
    VERIFY_RESULT( wwd_bus_write_register_value(BUS_FUNCTION, SPI_INTERRUPT_REGISTER, (uint8_t) 1, (uint32_t) ( DATA_UNAVAILABLE | COMMAND_ERROR | DATA_ERROR | F1_OVERFLOW ) ) );

    /* Enable a selection of interrupts */
    VERIFY_RESULT( wwd_bus_write_register_value(BUS_FUNCTION, SPI_INTERRUPT_ENABLE_REGISTER, (uint8_t) 2, (uint32_t) ( F2_F3_FIFO_RD_UNDERFLOW | F2_F3_FIFO_WR_OVERFLOW | COMMAND_ERROR | DATA_ERROR | F2_PACKET_AVAILABLE | F1_OVERFLOW ) ) );

    /* Request ALP */
    VERIFY_RESULT( wwd_bus_write_register_value(BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, (uint8_t) 1, SBSDIO_ALP_AVAIL_REQ ) );

    /* Wait until ALP is available */
    loop_count = 0;
    while ( ( ( result = wwd_bus_read_register_value( BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, (uint8_t) 2, (uint8_t*) &data16 ) ) == WWD_SUCCESS ) &&
            ( ( data16 & SBSDIO_ALP_AVAIL ) == 0 ) &&
            ( loop_count < (uint32_t) ALP_AVAIL_TIMEOUT_MS ) )
    {
        host_rtos_delay_milliseconds( (uint32_t) 1 );
        loop_count++;
    }
    if ( loop_count >= (uint32_t) ALP_AVAIL_TIMEOUT_MS )
    {
        return WWD_TIMEOUT;
    }
    if ( result != WWD_SUCCESS )
    {
        return result;
    }

    /* Clear request for ALP */
    VERIFY_RESULT( wwd_bus_write_register_value(BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, (uint8_t) 1, (uint32_t) 0) );

    /* Download the firmware */
    result = wwd_download_firmware( );

    /* user abort */
    if ( result == WWD_UNFINISHED )
    {
        host_platform_reset_wifi( WICED_TRUE );
        host_platform_power_wifi( WICED_FALSE );
        WPRINT_WWD_INFO(("User aborted download of firmware\n"));
        return result;
    }

    /* non user abort error */
    if ( result != WWD_SUCCESS )
    {
        WPRINT_WWD_ERROR(("Could not download firmware\n"));
        return result;
    }
    /* else, successfully downloaded the firmware; continue with waiting for WIFi to live */

    /* Wait for F2 to be ready */
    loop_count = 0;
    while ( ( ( result = wwd_bus_read_register_value( BUS_FUNCTION, SPI_STATUS_REGISTER, (uint8_t) 4, (uint8_t*) &wwd_bus_gspi_status ) ) == WWD_SUCCESS ) &&
            ( ( wwd_bus_gspi_status & ( 1 << 5 ) ) == 0 ) &&
            ( loop_count < (uint32_t) F2_READY_TIMEOUT_MS ) )
    {
        host_rtos_delay_milliseconds( (uint32_t) 1 );
        loop_count++;
    }
    if ( loop_count >= (uint32_t) F2_READY_TIMEOUT_MS )
    {
        /* If your system fails here, it could be due to incorrect NVRAM variables.
         * Check which 'wifi_nvram_image.h' file your platform is using, and
         * check that it matches the WLAN device on your platform, including the
         * crystal frequency.
         */
        WPRINT_WWD_ERROR(("Timeout while waiting for function 2 to be ready\n"));
        return WWD_TIMEOUT;
    }

    wwd_chip_specific_init();
    VERIFY_RESULT( wwd_ensure_wlan_bus_is_up( ) );

    return result;
}

wwd_result_t wwd_bus_deinit( void )
{
    wwd_allow_wlan_bus_to_sleep();

    /* put device in reset. */
    host_platform_reset_wifi( WICED_TRUE );
    wwd_bus_set_resource_download_halt( WICED_FALSE );
    DELAYED_BUS_RELEASE_SCHEDULE( WICED_FALSE );
    return WWD_SUCCESS;
}

void wwd_wait_for_wlan_event( host_semaphore_type_t* transceive_semaphore )
{
    wwd_result_t result = WWD_SUCCESS;
    uint32_t timeout_ms = 1;
    uint32_t delayed_release_timeout_ms;

    REFERENCE_DEBUG_ONLY_VARIABLE( result );

    delayed_release_timeout_ms = wwd_bus_handle_delayed_release( );
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

    /* Check if we have run out of bus credits */
    if ( wwd_sdpcm_get_available_credits( ) == 0 )
    {
        /* Keep poking the WLAN until it gives us more credits */
        result = wwd_bus_poke_wlan( );
        wiced_assert( "Poking failed!", result == WWD_SUCCESS );

        result = host_rtos_get_semaphore( transceive_semaphore, (uint32_t) MIN( timeout_ms, WWD_THREAD_POKE_TIMEOUT ), WICED_FALSE );
    }
    else
    {
        result = host_rtos_get_semaphore( transceive_semaphore, (uint32_t) MIN( timeout_ms, WWD_THREAD_POLL_TIMEOUT ), WICED_FALSE );
    }
    wiced_assert("Could not get wwd sleep semaphore\n", ( result == WWD_SUCCESS)||(result == WWD_TIMEOUT ) );

}

/******************************************************
 *     Function definitions for Protocol Common
 ******************************************************/

/*
 * Write a value to a register NOT on the backplane
 * Prerequisites: value_length <= 4
 */
wwd_result_t wwd_bus_write_register_value( wwd_bus_function_t function, uint32_t address, uint8_t value_length, uint32_t value )
{
    char gspi_internal_buffer[sizeof(wwd_bus_header_t) + sizeof(uint32_t) + sizeof(uint32_t)];
    wwd_transfer_bytes_packet_t* internal_gspi_packet = (wwd_transfer_bytes_packet_t*) gspi_internal_buffer;

    /* Flip the bytes if we're not in 32 bit mode */
    if ( wwd_bus_gspi_32bit == WICED_FALSE )
    {
        value = H32TO16LE(value);
    }
    /* Write the value and value_length into the packet */
    internal_gspi_packet->data[0] = value;

    /* Send it off */
    return wwd_bus_transfer_bytes( BUS_WRITE, function, address, value_length, internal_gspi_packet );
}

/*
 * Write a value to a register on the backplane
 * Prerequisites: value_length <= 4
 */
wwd_result_t wwd_bus_write_backplane_value( uint32_t address, uint8_t register_length, uint32_t value )
{
    VERIFY_RESULT( wwd_bus_set_backplane_window(address) );

    address &= SBSDIO_SB_OFT_ADDR_MASK;

    if (register_length == 4)
        address |= SBSDIO_SB_ACCESS_2_4B_FLAG;

    return wwd_bus_write_register_value( BACKPLANE_FUNCTION, address, register_length, value );
}

/*
 * Read the value of a register on the backplane
 * Prerequisites: value_length <= 4
 */
wwd_result_t wwd_bus_read_backplane_value( uint32_t address, uint8_t register_length, /*@out@*/ uint8_t* value )
{
    *value = 0;
    VERIFY_RESULT( wwd_bus_set_backplane_window(address) );

    address &= SBSDIO_SB_OFT_ADDR_MASK;

    if (register_length == 4)
        address |= SBSDIO_SB_ACCESS_2_4B_FLAG;

    return wwd_bus_read_register_value( BACKPLANE_FUNCTION, address, register_length, value );
}

wwd_result_t wwd_bus_transfer_bytes( wwd_bus_transfer_direction_t direction, wwd_bus_function_t function, uint32_t address, uint16_t size, /*@in@*/ /*@out@*/ wwd_transfer_bytes_packet_t* data )
{
    uint32_t* temp;
    wwd_result_t result;
    uint16_t newsize;

    data->bus_header.gspi_header = (wwd_bus_gspi_header_t) ( (uint32_t) ( ( wwd_bus_gspi_command_mapping[(int)direction] & 0x1     ) << 31 ) |
                                                     (uint32_t) ( ( GSPI_INCREMENT_ADDRESS               & 0x1     ) << 30 ) |
                                                     (uint32_t) ( ( function                             & 0x3     ) << 28 ) |
                                                     (uint32_t) ( ( address                              & 0x1FFFF ) << 11 ) |
                                                     (uint32_t) ( ( size                                 & 0x7FF   ) <<  0 ) );

    /* Reshuffle the bits if we're not in 32 bit mode */
    if ( wwd_bus_gspi_32bit == WICED_FALSE )
    {
        /* Note: This typecast should always be valid if the buffer containing the GSpi packet has been correctly declared as 32-bit aligned */
        temp  = (uint32_t*) data;
        *temp = H32TO16LE(*temp);
    }

    /* Round size up to 32-bit alignment */
    newsize = (uint16_t) ROUND_UP(size, 4);

    /* Ensure we are clear to write */
    if ( ( direction == BUS_WRITE ) && ( function == WLAN_FUNCTION ) )
    {
        uint32_t wwd_bus_gspi_status;
        uint32_t loop_count = 0;

        /* Verify the SDPCM size and stated size match */
        uint16_t* frametag_ptr = (uint16_t*) &data->data;
        if ( size != *frametag_ptr )
        {
            WPRINT_WWD_DEBUG(("Error - gSPI size does not match SDPCM size!\n"));
            return WWD_SPI_SIZE_MISMATCH;
        }

        /* Wait for WLAN FIFO to be ready to accept data */
        while ( ( ( result = wwd_bus_read_register_value( BUS_FUNCTION, SPI_STATUS_REGISTER, (uint8_t) 4, (uint8_t*) &wwd_bus_gspi_status ) ) == WWD_SUCCESS ) &&
                ( ( wwd_bus_gspi_status & ( 1 << 5 ) ) == 0 ) &&
                ( loop_count < (uint32_t) F2_READY_TIMEOUT_LOOPS ) )
        {
            ++loop_count;
        }

        if ( result != WWD_SUCCESS )
        {
            return result;
        }

        if ( loop_count >= (uint32_t) F2_READY_TIMEOUT_LOOPS )
        {
            WPRINT_WWD_DEBUG(("Timeout waiting for data FIFO to be ready\n"));
            return WWD_TIMEOUT;
        }

        add_log_entry( LOG_TX, function, address, (unsigned long)size, (char*)&data->data );
    }

    /* Send the data */
    result = host_platform_spi_transfer( direction, (uint8_t*) data, (uint16_t) ( newsize + sizeof(wwd_bus_gspi_header_t) ) );

    if ( direction == BUS_READ )
    {
        add_log_entry( LOG_RX, function, address, (unsigned long)((function == BACKPLANE_FUNCTION)?size-4:size), (function == BACKPLANE_FUNCTION)?((char*)&data->data)+4:(char*)&data->data );
    }

    return result;
}

/******************************************************
 *             Static  Function definitions
 ******************************************************/

static wwd_result_t wwd_download_firmware( void )
{
    uint8_t csr_val;
    wwd_result_t result;
    uint32_t loop_count = 0;

    VERIFY_RESULT( wwd_disable_device_core( WLAN_ARM_CORE, WLAN_CORE_FLAG_NONE ) );
    VERIFY_RESULT( wwd_disable_device_core( SOCRAM_CORE, WLAN_CORE_FLAG_NONE ) );
    VERIFY_RESULT( wwd_reset_device_core( SOCRAM_CORE, WLAN_CORE_FLAG_NONE ) );

    VERIFY_RESULT( wwd_chip_specific_socsram_init( ));

#ifdef MFG_TEST_ALTERNATE_WLAN_DOWNLOAD
    VERIFY_RESULT( external_write_wifi_firmware_and_nvram_image( ) );
#else
    VERIFY_RESULT( wwd_bus_write_wifi_firmware_image( ) );
    VERIFY_RESULT( wwd_bus_write_wifi_nvram_image( ) );
#endif /* ifdef MFG_TEST_ALTERNATE_WLAN_DOWNLOAD */

    /* Take the ARM core out of reset */
    VERIFY_RESULT( wwd_reset_device_core( WLAN_ARM_CORE, WLAN_CORE_FLAG_NONE ) );
    result = wwd_device_core_is_up( WLAN_ARM_CORE );
    if ( result != WWD_SUCCESS )
    {
        WPRINT_WWD_DEBUG(("Could not bring ARM core up\n"));
        return result;
    }

    /* Wait until the HT clock is available */
    while ( ( ( result = wwd_bus_read_register_value( BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, (uint8_t) 1, &csr_val ) ) == WWD_SUCCESS ) &&
            ( ( csr_val & SBSDIO_HT_AVAIL ) == 0 ) &&
            ( loop_count < (uint32_t) HT_AVAIL_TIMEOUT_MS ) )
    {
        host_rtos_delay_milliseconds( (uint32_t) 1 );
        loop_count++;
    }
    if ( loop_count >= (uint32_t) HT_AVAIL_TIMEOUT_MS )
    {
        return WWD_TIMEOUT;
    }
    if ( result != WWD_SUCCESS )
    {
        return result;
    }

    /* Set up the interrupt mask and enable interrupts */
    VERIFY_RESULT( wwd_bus_write_backplane_value(SDIO_INT_HOST_MASK, (uint8_t) 4, I_HMB_SW_MASK) );

    /* Lower F2 Watermark to avoid DMA Hang in F2 when SD Clock is stopped. */
    return wwd_bus_write_backplane_value( SDIO_FUNCTION2_WATERMARK, (uint8_t) 1, (uint32_t) SPI_F2_WATERMARK );
}

/*
 * Read the value of a register NOT on the backplane
 * Prerequisites: value_length <= 4
 */
wwd_result_t wwd_bus_read_register_value( wwd_bus_function_t function, uint32_t address, uint8_t value_length, /*@out@*/ uint8_t* value )
{
    uint32_t* data_ptr;
    wwd_result_t result;
    uint8_t padding = 0;

    char gspi_internal_buffer[sizeof(wwd_bus_header_t) + sizeof(uint32_t) + sizeof(uint32_t)];

    /* Clear the receiving part of memory and set the value_length */
    if ( function == BACKPLANE_FUNCTION )
    {
        gspi_backplane_f1_read_packet_t* pkt = (gspi_backplane_f1_read_packet_t*) gspi_internal_buffer;
        data_ptr = pkt->data;
        padding = 4; /* Add response delay */
    }
    else
    {
        wwd_transfer_bytes_packet_t* pkt = (wwd_transfer_bytes_packet_t*) gspi_internal_buffer;
        data_ptr = pkt->data;
    }

    *data_ptr = 0;
    result = wwd_bus_transfer_bytes( BUS_READ, function, address, (uint16_t)(value_length + padding), (wwd_transfer_bytes_packet_t*) gspi_internal_buffer );

    memcpy( value, data_ptr, value_length );

    return result;
}

wwd_result_t wwd_bus_specific_wakeup( void )
{
    uint32_t spi_bus_reg_value;

    /* Wake up WLAN SPI interface module */
    VERIFY_RESULT( wwd_bus_read_register_value( BUS_FUNCTION, SPI_BUS_CONTROL, sizeof(uint32_t), (uint8_t*)&spi_bus_reg_value ) );
    spi_bus_reg_value |= (uint32_t)( WAKE_UP );
    return wwd_bus_write_register_value( BUS_FUNCTION, SPI_BUS_CONTROL, sizeof(uint32_t), spi_bus_reg_value );
}

wwd_result_t wwd_bus_specific_sleep( void )
{
    uint32_t spi_bus_reg_value;

    /* Put SPI interface block to sleep */
    VERIFY_RESULT( wwd_bus_read_register_value( BUS_FUNCTION, SPI_BUS_CONTROL, sizeof(uint32_t), (uint8_t*)&spi_bus_reg_value ) );
    spi_bus_reg_value &= ~(uint32_t) ( WAKE_UP );
    return wwd_bus_write_register_value( BUS_FUNCTION, SPI_BUS_CONTROL, sizeof(uint32_t), spi_bus_reg_value );
}
