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

#include "string.h" /* for memcpy */
#include "platform_config.h"
#include "wiced_platform.h"
#include "platform.h"
#include "chip_constants.h"
#include "wiced.h"
#include "gspi_master.h"
#include "gspi_basic.h"
#include "gspi_sw_header.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *             Static variables
 ******************************************************/
static uint32_t gspi_interrupt_count = 0;
static const wiced_spi_device_t* gspi_slave;
static wiced_bool_t  gspi_32bit = WICED_FALSE;
static uint32_t     backplane_window_current_base_address   = 0;

static const uint8_t gspi_command_mapping[] =
{
    0,
    1
};

/******************************************************
 *             Variables
 ******************************************************/

/******************************************************
 *             Static Function Declarations
 ******************************************************/
static void gspi_irq_handler( void* arg );

static wiced_result_t gspi_get_slave_buffer_info( gspi_master_t* gspi_master );
static wiced_result_t gspi_slave_read_data( gspi_master_t *gspi_master , uint8_t* read_buffer );
static wiced_result_t gpsi_get_slave_response( gspi_master_t *gspi_master );
static wiced_result_t gspi_slave_bringup( void );
static wiced_result_t gspi_transfer( gspi_transfer_direction_t dir, uint8_t* buffer, uint16_t buffer_length );
static wiced_result_t gspi_set_backplane_window( uint32_t addr );
static wiced_result_t gspi_transfer_bytes( gspi_transfer_direction_t direction, gspi_function_t function, uint32_t address, uint16_t size, transfer_bytes_packet_t* data );

/******************************************************
 *             Function Declarations
 ******************************************************/
void platform_gspi_master_irq_init( platform_gpio_irq_callback_t handler, void* arg );
/******************************************************
 *             Function definitions
 ******************************************************/

static wiced_result_t gspi_set_backplane_window( uint32_t addr )
{
    wiced_result_t result = WICED_ERROR;
    uint32_t base = addr & ( (uint32_t) ~BACKPLANE_ADDRESS_MASK );

    if ( base == backplane_window_current_base_address )
    {
        return WICED_SUCCESS;
    }
    if ( ( base & 0xFF000000 ) != ( backplane_window_current_base_address & 0xFF000000 ) )
    {
        if ( WICED_SUCCESS != ( result = gspi_write_register_value( GSPI_BACKPLANE_FUNCTION, SDIO_BACKPLANE_ADDRESS_HIGH, (uint8_t) 1, ( base >> 24 ) ) ) )
        {
            WPRINT_LIB_DEBUG( ("%s, FAILED to set BACKPLANE address high\n", __FUNCTION__) );
            return result;
        }
    }
    if ( ( base & 0x0FF0000 ) != ( backplane_window_current_base_address & 0x00FF0000 ) )
    {
        if ( WICED_SUCCESS != ( result = gspi_write_register_value( GSPI_BACKPLANE_FUNCTION, SDIO_BACKPLANE_ADDRESS_MID, (uint8_t) 1, ( base >> 16 ) ) ) )
        {
            WPRINT_LIB_DEBUG( ("%s, FAILED to set BACKPLANE address mid\n", __FUNCTION__) );
            return result;
        }
    }
    if ( ( base & 0x0000FF00 ) != ( backplane_window_current_base_address & 0x0000FF00 ) )
    {
        if ( WICED_SUCCESS != ( result = gspi_write_register_value( GSPI_BACKPLANE_FUNCTION, SDIO_BACKPLANE_ADDRESS_LOW, (uint8_t) 1, ( base >> 8 ) ) ) )
        {
            WPRINT_LIB_DEBUG( ("%s, FAILED to set BACKPLANE address low\n", __FUNCTION__) );
            return result;
        }
    }

    backplane_window_current_base_address = base;

    return WICED_SUCCESS;
}

static wiced_result_t gspi_transfer( gspi_transfer_direction_t dir, uint8_t* buffer, uint16_t buffer_length )
{
    wiced_spi_message_segment_t       message;

    memset( &message, 0x00, sizeof( message ) );
    message.length      = buffer_length;
    message.tx_buffer   = buffer;
    message.rx_buffer   = ( dir == GSPI_BUS_WRITE ) ? ( NULL ) : ( buffer );

    return wiced_spi_transfer( gspi_slave, &message, 1 );
}

static wiced_result_t gspi_transfer_bytes( gspi_transfer_direction_t direction, gspi_function_t function, uint32_t address, uint16_t size, /*@in@*/ /*@out@*/ transfer_bytes_packet_t* data )
{
    uint32_t* temp;
    wiced_result_t result;
    uint16_t newsize;

    data->gspi_header = (gspi_header_t) ( (uint32_t) ( ( gspi_command_mapping[(int)direction] & 0x1     ) << 31 ) |
                                                     (uint32_t) ( ( GSPI_INCREMENT_ADDRESS               & 0x1     ) << 30 ) |
                                                     (uint32_t) ( ( function                             & 0x3     ) << 28 ) |
                                                     (uint32_t) ( ( address                              & 0x1FFFF ) << 11 ) |
                                                     (uint32_t) ( ( size                                 & 0x7FF   ) <<  0 ) );

    /* Reshuffle the bits if we're not in 32 bit mode */
    if ( gspi_32bit == WICED_FALSE )
    {
        /* Note: This typecast should always be valid if the buffer containing the GSpi packet has been correctly declared as 32-bit aligned */
        temp  = (uint32_t*) data;
        *temp = H32TO16LE(*temp);
    }

    /* Round size up to 32-bit alignment */
    newsize = (uint16_t) ROUND_UP(size, 4);

    /* Send the data */
    result = gspi_transfer( direction, (uint8_t*) data, (uint16_t) ( newsize + sizeof(gspi_header_t) ) );

    return result;
}

wiced_result_t gspi_read_register_value( gspi_function_t function, uint32_t address, uint8_t value_length, /*@out@*/ uint8_t* value )
{
    uint32_t* data_ptr;
    wiced_result_t result;
    uint8_t padding = 0;

    char gspi_internal_buffer[sizeof(gspi_header_t) + GSPI_RESPONSE_DELAY + sizeof(uint32_t)];

    /* Clear the receiving part of memory and set the value_length */
    if ( function == GSPI_BACKPLANE_FUNCTION )
    {
        gspi_backplane_f1_read_packet_t* pkt = (gspi_backplane_f1_read_packet_t*) gspi_internal_buffer;
        data_ptr = pkt->data;
        padding = GSPI_RESPONSE_DELAY; /* Add response delay */
    }
    else
    {
        transfer_bytes_packet_t* pkt = (transfer_bytes_packet_t*) gspi_internal_buffer;
        data_ptr = pkt->data;
    }
    *data_ptr = 0;
    result = gspi_transfer_bytes( GSPI_BUS_READ, function, address, (uint16_t)(value_length + padding), (transfer_bytes_packet_t*) gspi_internal_buffer );

    memcpy( value, data_ptr, value_length );

    return result;
}

wiced_result_t gspi_write_register_value( gspi_function_t function, uint32_t address, uint8_t value_length, uint32_t value )
{
    char gspi_internal_buffer[sizeof(gspi_header_t) + GSPI_RESPONSE_DELAY + sizeof(uint32_t)];
    transfer_bytes_packet_t* internal_gspi_packet = (transfer_bytes_packet_t*) gspi_internal_buffer;

    /* Flip the bytes if we're not in 32 bit mode */
    if ( gspi_32bit == WICED_FALSE )
    {
        value = H32TO16LE(value);
    }
    /* Write the value and value_length into the packet */
    internal_gspi_packet->data[0] = value;

    /* Send it off */
    return gspi_transfer_bytes( GSPI_BUS_WRITE, function, address, value_length, internal_gspi_packet );
}

wiced_result_t gspi_write_backplane_value( uint32_t address, uint8_t register_length, uint32_t value )
{
    wiced_result_t result = WICED_SUCCESS;
    VERIFY_RESULT( gspi_set_backplane_window(address) );
    VERIFY_RESULT( gspi_write_register_value( GSPI_BACKPLANE_FUNCTION, address & BACKPLANE_ADDRESS_MASK, register_length, value ) );

    return result;
}

/*
 * Read the value of a register on the backplane
 * Prerequisites: value_length <= 4
 */
wiced_result_t gspi_read_backplane_value( uint32_t address, uint8_t register_length, uint8_t* value )
{
    wiced_result_t result = WICED_SUCCESS;
    uint32_t backplane_addr;
    *value = 0;

    backplane_addr = (register_length == 4 )? ((address & BACKPLANE_ADDRESS_MASK) | 0x8000) : (address & BACKPLANE_ADDRESS_MASK);
    VERIFY_RESULT( gspi_set_backplane_window(address) );

    VERIFY_RESULT( gspi_read_register_value( GSPI_BACKPLANE_FUNCTION, backplane_addr, register_length, value ) );

    return result;
}

wiced_result_t gspi_transfer_backplane_bytes( gspi_transfer_direction_t direction, uint32_t address, uint32_t size, uint8_t* data )
{
    void*       pkt_buffer = NULL;
    uint8_t*       packet;
    uint32_t       transfer_size;
    uint32_t       remaining_buf_size;
    uint32_t       max_size;
    wiced_result_t   result;

    pkt_buffer = malloc(  GSPI_MAX_BACKPLANE_TRANSFER_SIZE + GSPI_BUS_BACKPLANE_READ_PADD_SIZE +
                  GSPI_BUS_HEADER_SIZE );

    if ( pkt_buffer == NULL )
    {
        WPRINT_LIB_ERROR( ("%s, Alloc buffer failed\n", __FUNCTION__) );
        result = WICED_ERROR;
        goto done;
    }

    packet = (uint8_t *)pkt_buffer;

    remaining_buf_size = size;
    for (; remaining_buf_size != 0; remaining_buf_size -= transfer_size, address += transfer_size )
    {
        /*
         * The first transfer might not be at 64-byte boundary, adjust transfer size for aligning
         * with 64-byte boundary. GSPI_MAX_BACKPLANE_TRANSFER_SIZE needs to be power of 2 for
         * following formula.
         */
        max_size = GSPI_MAX_BACKPLANE_TRANSFER_SIZE -
                    ( address & (uint32_t)( GSPI_MAX_BACKPLANE_TRANSFER_SIZE-1 ) );
        transfer_size = ( remaining_buf_size > max_size ) ? max_size : remaining_buf_size;

        result = gspi_set_backplane_window( address );
        if ( result != WICED_SUCCESS )
        {
            WPRINT_LIB_ERROR( ("Set backplane window failed\n") );
            goto done;
        }

        /* Write to gSPI slave */
        if ( direction == GSPI_BUS_WRITE )
        {
            memcpy( packet + GSPI_BUS_HEADER_SIZE, data + size - remaining_buf_size, transfer_size );
            result = gspi_transfer_bytes( direction, GSPI_BACKPLANE_FUNCTION, ( address & BACKPLANE_ADDRESS_MASK ),
                        (uint16_t) transfer_size, (transfer_bytes_packet_t *) packet );
            if ( result != WICED_SUCCESS )
            {
                WPRINT_LIB_INFO( ("GSPI write failed\n") );
                goto done;
            }
        }
        /* Read from gSPI slave */
        else
        {
            result = gspi_transfer_bytes( direction, GSPI_BACKPLANE_FUNCTION, ( address & BACKPLANE_ADDRESS_MASK ),
                        (uint16_t) ( transfer_size + GSPI_BUS_BACKPLANE_READ_PADD_SIZE ),
                        (transfer_bytes_packet_t *) packet );
            if ( result != WICED_SUCCESS )
            {
                WPRINT_LIB_INFO( ("GSPI READ failed\n") );
                goto done;
            }
            /* Copy read data from packet to buffer from offset (gSPI header + padd) */
            memcpy( data + size - remaining_buf_size,
                    packet + GSPI_BUS_HEADER_SIZE + GSPI_BUS_BACKPLANE_READ_PADD_SIZE, transfer_size );
        }
    }

done:
    if ( pkt_buffer )
        free( pkt_buffer );

    return result;
}

static wiced_result_t gspi_get_slave_buffer_info( gspi_master_t* gspi_master )
{
    wiced_result_t result = WICED_SUCCESS;
    gspi_shared_info_t gspi_shared;

    gspi_transfer_backplane_bytes( GSPI_BUS_READ, gspi_master->gspi_shared_address, sizeof( gspi_shared ), (uint8_t *)&gspi_shared );

    if ( gspi_shared.magic_number == GSPI_SHARED_MAGIC_NUMBER )
    {
        gspi_master->slave_to_master_buffer_address = gspi_shared.slave_to_master_buffer_address;
        gspi_master->master_to_slave_buffer_address = gspi_shared.master_to_slave_buffer_address;
        gspi_master->slave_to_master_buffer_size = gspi_shared.slave_to_master_buffer_size;
        gspi_master->master_to_slave_buffer_size = gspi_shared.master_to_slave_buffer_size;

        WPRINT_LIB_INFO( ("[%s], TX buffer addr: 0x%lx size: %ld, RX buffer addr: 0x%lx, size: %ld\n", __FUNCTION__,
                gspi_master->slave_to_master_buffer_address, gspi_master->slave_to_master_buffer_size,
                gspi_master->master_to_slave_buffer_address, gspi_master->master_to_slave_buffer_size) );
    }
    else
    {
        WPRINT_LIB_INFO( ("[%s]: Get slave buffer info FAILED!\n", __FUNCTION__) );
        return WICED_ERROR;
    }

    return result;

}

wiced_result_t gspi_master_init( gspi_master_t* gspi_master, wiced_spi_device_t* gspi_device )
{
    wiced_result_t result = WICED_SUCCESS;

    /* Check if the pointer is valid */
    if ( gspi_master == NULL || gspi_device == NULL )
    {
        WPRINT_LIB_ERROR( ("[%s], init failed!!\n", __FUNCTION__) );
        return WICED_BADARG;
    }

    /* Initialize the master structure */
    gspi_master->gspi_device = gspi_device;
    gspi_slave = gspi_device;

    /* Init response ready semaphore */
    result = wiced_rtos_init_semaphore( &gspi_master->response_ready_semaphore );

    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ("[%s], Init response ready sema FAILED\n", __FUNCTION__) );
        return result;
    }

    VERIFY_RESULT( wiced_spi_init( gspi_device ) );

    VERIFY_RESULT( gspi_slave_bringup( ) );

    gspi_wait_slave_ready();

    VERIFY_RESULT( gspi_get_shared_address( &gspi_master->gspi_shared_address ) );

    VERIFY_RESULT( gspi_get_slave_buffer_info( gspi_master ) );

    /* Setup the interrupt input for_IRQ */
    platform_gspi_master_irq_init(gspi_irq_handler, (void*)gspi_master );

    VERIFY_RESULT( gspi_enable_master_interrupt( ) );

    return result;
}


wiced_result_t gspi_dump_register( void )
{
    uint32_t data = 0;

    VERIFY_RESULT( gspi_read_register_value(GSPI_BUS_FUNCTION, SPI_BUS_CONTROL, (uint8_t) 1, (uint8_t*)&data ) );
    WPRINT_LIB_INFO( ("%s: SPI_BUS_CONTROL: 0x%lx\n", __FUNCTION__,  data) );
    data = 0;

    VERIFY_RESULT( gspi_read_register_value(GSPI_BUS_FUNCTION, SPI_STATUS_REGISTER, (uint8_t) 4, (uint8_t*)&data ) );
    WPRINT_LIB_INFO( ("%s: SPI_STATUS_REGISTER: 0x%lx\n", __FUNCTION__,  data) );
    data = 0;
    VERIFY_RESULT( gspi_read_register_value(GSPI_BUS_FUNCTION, SPI_FUNCTION1_INFO, (uint8_t) 2, (uint8_t*)&data ) );
    WPRINT_LIB_INFO( ("%s: SPI_FUNCTION1_INFO: 0x%lx\n", __FUNCTION__, data) );
    data = 0;

    VERIFY_RESULT( gspi_read_register_value(GSPI_BUS_FUNCTION, SPI_INTERRUPT_REGISTER, (uint8_t) 2, (uint8_t*)&data ) );
    WPRINT_LIB_INFO( ("%s: SPI_INTERRUPT_REGISTER: 0x%lx\n", __FUNCTION__, data) );
    data = 0;

    VERIFY_RESULT( gspi_read_register_value(GSPI_BUS_FUNCTION, SPI_STATUS_ENABLE, (uint8_t) 1, (uint8_t*)&data ) );
    WPRINT_LIB_INFO( ("%s: SPI_STATUS_ENABLE: 0x%lx\n", __FUNCTION__, data) );
    data = 0;

    VERIFY_RESULT( gspi_read_register_value(GSPI_BACKPLANE_FUNCTION, (uint32_t)0x300E, (uint8_t) 1, (uint8_t*)&data )  );
    WPRINT_LIB_INFO( ("%s: SDIO_CHIP_CLOCK_CSR: 0x%lx\n", __FUNCTION__, data) );
    data = 0;

    VERIFY_RESULT( gspi_read_register_value(GSPI_BUS_FUNCTION, SPI_RESP_DELAY_F1, (uint8_t) 1, (uint8_t*)&data )  );
    WPRINT_LIB_INFO( ("%s: SPI_RESP_DELAY_F1: 0x%lx\n", __FUNCTION__, data) );
    data = 0;

    return WICED_SUCCESS;
}

static wiced_result_t gspi_slave_bringup( void )
{
    uint32_t data = 0;
    uint16_t data16 = 0;
    uint32_t loop_count;
    wiced_result_t result;
    uint8_t init_data[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    uint32_t interrupt_polarity = 0;
    uint32_t clock_polarity = GSPI_CLOCK_POLARITY_LOW , clock_phase = GSPI_CLOCK_PHASE_NO_DELAY;

    gspi_32bit = WICED_FALSE;


    if (gspi_slave) {
        clock_polarity = ( gspi_slave->mode & SPI_CLOCK_IDLE_LOW )? GSPI_CLOCK_POLARITY_LOW : GSPI_CLOCK_POLARITY_HIGH;

        if (gspi_slave->mode & SPI_CLOCK_RISING_EDGE ) {
            clock_phase = (clock_polarity == GSPI_CLOCK_POLARITY_LOW) ? GSPI_CLOCK_PHASE_NO_DELAY : GSPI_CLOCK_PHASE_DELAY;
        } else {
            clock_phase = (clock_polarity == GSPI_CLOCK_POLARITY_LOW) ? GSPI_CLOCK_PHASE_DELAY : GSPI_CLOCK_PHASE_NO_DELAY;
        }
    }

#ifndef WWD_SPI_IRQ_FALLING_EDGE
    interrupt_polarity = INTERRUPT_POLARITY_HIGH;
#endif /* WWD_SPI_IRQ_FALLING_EDGE */

    platform_gspi_bringup_slave();
    gspi_wait_slave_up();

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
        gspi_header_t* gspi_header = (gspi_header_t*) init_data;

        *gspi_header = (gspi_header_t) SWAP32_16BIT_PARTS( BCMSWAP32( (uint32_t)( ( gspi_command_mapping[(int)GSPI_BUS_READ] & 0x1      ) << 31) |
                                                                      (uint32_t)( ( GSPI_FIXED_ADDRESS                  & 0x1      ) << 30) |
                                                                      (uint32_t)( ( GSPI_BUS_FUNCTION                        & 0x3      ) << 28) |
                                                                      (uint32_t)( ( SPI_READ_TEST_REGISTER              & 0x1FFFFu ) << 11) |
                                                                      (uint32_t)( ( 4u /*size*/                         & 0x7FFu   ) <<  0 ) ) );
        VERIFY_RESULT( gspi_transfer(GSPI_BUS_READ, init_data, (uint16_t) 12 ) );
        loop_count++;

    } while ( ( NULL == memchr( &init_data[4], SPI_READ_TEST_REG_LSB,      (size_t) 8 ) ) &&
              ( NULL == memchr( &init_data[4], SPI_READ_TEST_REG_LSB_SFT1, (size_t) 8 ) ) &&
              ( NULL == memchr( &init_data[4], SPI_READ_TEST_REG_LSB_SFT2, (size_t) 8 ) ) &&
              ( NULL == memchr( &init_data[4], SPI_READ_TEST_REG_LSB_SFT3, (size_t) 8 ) ) &&
              ( loop_count < (uint32_t) FEADBEAD_TIMEOUT_MS ) &&
              ( host_rtos_delay_milliseconds( (uint32_t) 1 ), ( 1 == 1 ) ) );

    /* Keep/reset defaults for registers 0x0-0x4 except for, 0x0: Change word length to 32bit, set endianness, enable wakeup. 0x2: enable interrupt with status. */
#if defined(IL_BIGENDIAN)
    VERIFY_RESULT( gspi_write_register_value(GSPI_BUS_FUNCTION, SPI_BUS_CONTROL, (uint8_t) 4, (uint32_t) ( WORD_LENGTH_32 | (0 & ENDIAN_BIG) | ( interrupt_polarity & INTERRUPT_POLARITY_HIGH ) | WAKE_UP | (0x4 << (8*SPI_RESPONSE_DELAY)) | ((0 & STATUS_ENABLE) << (8*SPI_STATUS_ENABLE)) | ( INTR_WITH_STATUS << (8*SPI_STATUS_ENABLE)) ) ) );
#else
    VERIFY_RESULT( gspi_write_register_value(GSPI_BUS_FUNCTION, SPI_BUS_CONTROL, (uint8_t) 4, (uint32_t) ( WORD_LENGTH_32 | ENDIAN_BIG       | ( interrupt_polarity & INTERRUPT_POLARITY_HIGH ) | WAKE_UP | (GSPI_RESPONSE_DELAY << (8*SPI_RESPONSE_DELAY)) |
                                                     ((clock_polarity & CLOCK_POLARITY)) | ((clock_phase & CLOCK_PHASE)) |
                                                     ((0 & STATUS_ENABLE) << (8*SPI_STATUS_ENABLE)) | ( INTR_WITH_STATUS << (8*SPI_STATUS_ENABLE)) ) ) );
#endif
    gspi_32bit = WICED_TRUE;
    VERIFY_RESULT( gspi_read_register_value(GSPI_BUS_FUNCTION, SPI_BUS_CONTROL, (uint8_t) 4, (uint8_t*)&data ) );

    /* Check feedbead can be read - i.e. the device is alive */
    data = 0;
    VERIFY_RESULT( gspi_read_register_value( GSPI_BUS_FUNCTION, SPI_READ_TEST_REGISTER, (uint8_t) 4, (uint8_t*) &data ) );

    if ( data != SPI_READ_TEST_REGISTER_VALUE )
    {
        WPRINT_WWD_ERROR(("Read %x, instead of 0xFEEDBEAD from the WLAN chip\n", (unsigned int)data ));
        return WICED_ERROR;
    }

    /* Make sure error interrupt bits are clear */
    VERIFY_RESULT( gspi_write_register_value(GSPI_BUS_FUNCTION, SPI_INTERRUPT_REGISTER, (uint8_t) 1, (uint32_t) ( DATA_UNAVAILABLE | COMMAND_ERROR | DATA_ERROR | F1_OVERFLOW ) ) );

    /* Enable a selection of interrupts */
    VERIFY_RESULT( gspi_write_register_value(GSPI_BUS_FUNCTION, SPI_INTERRUPT_ENABLE_REGISTER, (uint8_t) 2, (uint32_t) ( F2_F3_FIFO_RD_UNDERFLOW | F2_F3_FIFO_WR_OVERFLOW | COMMAND_ERROR | DATA_ERROR | F2_PACKET_AVAILABLE | F1_OVERFLOW | F1_INTR) ) );

    /* Set gSPI response delay */
    VERIFY_RESULT( gspi_write_register_value(GSPI_BUS_FUNCTION, SPI_RESP_DELAY_F1, (uint8_t) 1, (uint32_t )GSPI_RESPONSE_DELAY ) );

    /* Request ALP */
    VERIFY_RESULT( gspi_write_register_value(GSPI_BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, (uint8_t) 1, SBSDIO_ALP_AVAIL_REQ ) );

    /* Wait until ALP is available */
    loop_count = 0;
    while ( ( ( result = gspi_read_register_value( GSPI_BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, (uint8_t) 2, (uint8_t*) &data16 ) ) == WICED_SUCCESS ) &&
            ( ( data16 & SBSDIO_ALP_AVAIL ) == 0 ) &&
            ( loop_count < (uint32_t) ALP_AVAIL_TIMEOUT_MS ) )
    {
        host_rtos_delay_milliseconds( (uint32_t) 1 );
        loop_count++;
    }
    if ( loop_count >= (uint32_t) ALP_AVAIL_TIMEOUT_MS )
    {
        return WICED_ERROR;
    }
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    /* Clear request for ALP */
    VERIFY_RESULT( gspi_write_register_value(GSPI_BACKPLANE_FUNCTION, SDIO_CHIP_CLOCK_CSR, (uint8_t) 1, (uint32_t) 0) );

    return result;
}

static wiced_result_t gspi_slave_read_data( gspi_master_t *gspi_master , uint8_t* read_buffer )
{
    wiced_result_t      result = WICED_SUCCESS;
    gspi_slave_response_t rx_resp;
    uint32_t rx_length;

    VERIFY_RESULT( gspi_read_backplane_value( gspi_master->slave_to_master_buffer_address, sizeof( rx_resp ), (uint8_t *)&rx_resp ) );

    if ( rx_resp.status == GSPI_SLAVE_RESPONSE_GOOD )
    {
        rx_length = rx_resp.data_length;

        result = gspi_transfer_backplane_bytes( GSPI_BUS_READ, gspi_master->slave_to_master_buffer_address + sizeof( rx_resp ), rx_length, read_buffer );

        if ( result != WICED_SUCCESS )
            WPRINT_LIB_DEBUG( ("%s: Failed\n", __FUNCTION__) );
    }
    else
    {
        WPRINT_LIB_DEBUG( ("%s: Response failed: %ld\n", __FUNCTION__, (uint32_t)rx_resp.status) );
        return WICED_ERROR;
    }

    return result;
}

static wiced_result_t gpsi_get_slave_response( gspi_master_t *gspi_master )
{
    wiced_result_t      result = WICED_SUCCESS;
    gspi_slave_status_t       status;

    VERIFY_RESULT( gspi_read_backplane_value( gspi_master->slave_to_master_buffer_address, sizeof( status ), (uint8_t *)&status ) );

    switch ( status )
    {
        case GSPI_SLAVE_RESPONSE_GOOD:
            WPRINT_LIB_DEBUG( ("gspi slave response success\n") );
            break;
        case GSPI_SLAVE_RESPONSE_FAILED:
            WPRINT_LIB_DEBUG( ("gspi slave response error!\n") );
            break;
        default:
            WPRINT_LIB_DEBUG( ("Invalid gspi slave response\n") );
    }
    return result;
}

wiced_result_t gspi_write_command( gspi_master_t *gspi_master,  uint16_t address, uint16_t size, uint8_t* data_out )
{
    uint32_t data_total_size;
    wiced_result_t                    result = WICED_SUCCESS;
    gspi_sw_header_t *gspi_sw_tx_hdr;
    uint8_t *data_payload = NULL;

    data_total_size = sizeof( *gspi_sw_tx_hdr ) + size;

    if ( data_total_size > gspi_master->master_to_slave_buffer_size ) {
        WPRINT_LIB_ERROR( ("%s: Requested buffer is too large (%d bytes) for Slave\n",
                __FUNCTION__, data_total_size) );
        return WICED_ERROR;
    }

    data_payload = malloc( data_total_size );
    if ( !data_payload ) {
        WPRINT_LIB_ERROR( ("%s: Failed to allocate payload buffer!\n", __FUNCTION__) );
        return WICED_ERROR;
    }

    gspi_sw_tx_hdr = (gspi_sw_header_t *)data_payload;

    /* Fill in gspi sw TX header */
    gspi_sw_tx_hdr->direction = GSPI_MASTER_TRANSFER_WRITE;
    gspi_sw_tx_hdr->address = address;
    gspi_sw_tx_hdr->data_length = size;

    memcpy( (void *)&data_payload[sizeof( *gspi_sw_tx_hdr )], data_out, size );

    /* Transfer the write command to slave RX buffer */
    result = gspi_transfer_backplane_bytes( GSPI_BUS_WRITE, gspi_master->master_to_slave_buffer_address, data_total_size, data_payload );
    free( data_payload );

    if ( result != WICED_SUCCESS ) {
        WPRINT_LIB_ERROR( ("%s: Error in transferring data, result=%d\n", __FUNCTION__,
                (int)result) );
        return result;
    }

    /* Notify the slave the data is there */
    VERIFY_RESULT( gspi_notify_slave( ) );

    /* Wait for response from slave */
    result = wiced_rtos_get_semaphore( &gspi_master->response_ready_semaphore, RESPONSE_READY_TIMEOUT_MS );

    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_DEBUG( ("%s: Error wait response semaphore!!\n", __FUNCTION__) );
        return result;
    }

    VERIFY_RESULT( gspi_ack_slave_interrupt( ) );

    /* Get response from slave */
    return gpsi_get_slave_response( gspi_master );
}

wiced_result_t gspi_read_command( gspi_master_t* gspi_master, uint16_t address, uint16_t size, uint8_t* data_in )
{
    gspi_sw_header_t gspi_sw_rx_hdr;
    wiced_result_t  result = WICED_SUCCESS;


    /* Fill in gspi sw RX header */
    gspi_sw_rx_hdr.direction = GSPI_MASTER_TRANSFER_READ;
    gspi_sw_rx_hdr.address = address;
    gspi_sw_rx_hdr.data_length = size;

    WPRINT_LIB_DEBUG( ("[%s], dir: %d, addr: 0x%x, data_length: %d, slav_tx_buffer_addr: 0x%lx, size: %d\n", __FUNCTION__,
            gspi_sw_rx_hdr.direction,
            gspi_sw_rx_hdr.address,
            gspi_sw_rx_hdr.data_length,
            gspi_master->slave_to_master_buffer_address,
            size) );
    /* Transfer the read command to slave RX buffer */
    VERIFY_RESULT (gspi_transfer_backplane_bytes( GSPI_BUS_WRITE, gspi_master->master_to_slave_buffer_address, sizeof( gspi_sw_rx_hdr ), (uint8_t *)&gspi_sw_rx_hdr ) );

    /* Notify the slave the data is there */
    VERIFY_RESULT( gspi_notify_slave( ) );

    /* Wait for response from slave */
    result = wiced_rtos_get_semaphore( &gspi_master->response_ready_semaphore, RESPONSE_READY_TIMEOUT_MS );

    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_DEBUG( ("%s: Error wait response semaphore!!\n", __FUNCTION__) );
        return result;
    }
    VERIFY_RESULT( gspi_ack_slave_interrupt( ) );

    /* Get response from slave */
    return gspi_slave_read_data( gspi_master, data_in );

}

/******************************************************
 *             IRQ Handler definitions
 ******************************************************/

static void gspi_irq_handler( void* arg )
{
    gspi_master_t* master = (gspi_master_t*)arg;

    /* Indicate the response is ready */
    wiced_rtos_set_semaphore( &master->response_ready_semaphore );

    gspi_interrupt_count ++;
}
