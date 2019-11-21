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
#include "spi_flash.h"
#include "spi_flash_internal.h"
#include "spi_flash_platform_interface.h"
#include <string.h> /* for NULL */
#include "wwd_constants.h"


/*@access sflash_handle_t@*/ /* Lint: permit access to abstract sflash handle implementation */

static int generic_sflash_command      ( const sflash_handle_t* handle, sflash_command_t cmd, unsigned long num_initial_parameter_bytes, /*@null@*/ /*@observer@*/ const void* parameter_bytes, unsigned long num_data_bytes, /*@null@*/ /*@observer@*/ const void* const data_MOSI, /*@null@*/ /*@out@*/ /*@dependent@*/ void* const data_MISO );

device_id_to_flash_size_t sflash_device_id_size [SFLASH_ID_MAX_PARTS] =
{
        { SFLASH_ID_MX25L8006E,  SFLASH_SIZE_1MByte   },
        { SFLASH_ID_MX25L1606E,  SFLASH_SIZE_2MByte   },
        { SFLASH_ID_MX25L25635F, SLFASH_SIZE_32MByte  },
        { SFLASH_ID_MX25U1635F,  SFLASH_SIZE_2MByte   },
        { SFLASH_ID_SST25VF080B, SFLASH_SIZE_1MByte   },
        { SFLASH_ID_EN25QH16,    SFLASH_SIZE_2MByte   },
        { SFLASH_ID_N25Q512A,    SFLASH_SIZE_64MByte  },
        { SFLASH_ID_CY15B104Q,   SFLASH_SIZE_512KByte }

};

int sflash_read_ID( const sflash_handle_t* handle, /*@out@*/ device_id_t* data_addr )
{
    return generic_sflash_command( handle, SFLASH_READ_JEDEC_ID, 0, NULL, (unsigned long) 3, NULL, data_addr );
}

int sflash_write_enable( const sflash_handle_t* const handle )
{
    if ( handle->write_allowed == SFLASH_WRITE_ALLOWED )
    {
        unsigned char status_register;
        int status;

        /* Send write-enable command */
        status = generic_sflash_command( handle, SFLASH_WRITE_ENABLE, 0, NULL, 0, NULL, NULL );
        if ( status != 0 )
        {
            return status;
        }

        /* Check status register */
        if ( 0 != ( status = sflash_read_status_register( handle, &status_register ) ) )
        {
            return status;
        }

        /* Check if Block protect bits are set */
        if ( status_register != SFLASH_STATUS_REGISTER_WRITE_ENABLED )
        {
            /* Disable protection for all blocks */
            status = sflash_write_status_register( handle, (char) 0 );
            if ( status != 0 )
            {
                return status;
            }

            /* Re-Enable writing */
            status = generic_sflash_command( handle, SFLASH_WRITE_ENABLE, 0, NULL, 0, NULL, NULL );
            if ( status != 0 )
            {
                return status;
            }
        }
        return 0;
    }
    else
    {
        return -1;
    }
}

int sflash_chip_erase( const sflash_handle_t* const handle )
{
    int status = sflash_write_enable( handle );
    if ( status != 0 )
    {
        return status;
    }

#ifdef SFLASH_SUPPORT_MICRON_PARTS
    if ( handle->device_id == SFLASH_ID_N25Q512A )
        return generic_sflash_command( handle, SFLASH_DIE_ERASE_MICRON, 0, NULL, 0, NULL, NULL );
    else
#endif
    return generic_sflash_command( handle, SFLASH_CHIP_ERASE1, 0, NULL, 0, NULL, NULL );
}

#include "wwd_assert.h"

int sflash_sector_erase ( const sflash_handle_t* const handle, unsigned long device_address )
{

    char device_address_array[3] =  { (char) ( ( device_address & 0x00FF0000 ) >> 16 ),
                                      (char) ( ( device_address & 0x0000FF00 ) >>  8 ),
                                      (char) ( ( device_address & 0x000000FF ) >>  0 ) };

    int retval;
    int status = sflash_write_enable( handle );

    if ( status != 0 )
    {
        return status;
    }
    retval = generic_sflash_command( handle, SFLASH_SECTOR_ERASE, (unsigned long) 3, device_address_array, 0, NULL, NULL );


    wiced_assert("error", retval == 0);
    return retval;
}
int sflash_block_erase ( const sflash_handle_t* const handle, unsigned long device_address )
{

    char device_address_array[3] =  { (char) ( ( device_address & 0x00FF0000 ) >> 16 ),
                                      (char) ( ( device_address & 0x0000FF00 ) >>  8 ),
                                      (char) ( ( device_address & 0x000000FF ) >>  0 ) };
    int retval;
    int status = sflash_write_enable( handle );
    if ( status != 0 )
    {
        return status;
    }
    retval = generic_sflash_command( handle, SFLASH_BLOCK_ERASE_LARGE, (unsigned long) 3, device_address_array, 0, NULL, NULL );
    wiced_assert("error", retval == 0);
    return retval;
}
int sflash_read_status_register( const sflash_handle_t* const handle, /*@out@*/  /*@dependent@*/ unsigned char* const dest_addr )
{
    return generic_sflash_command( handle, SFLASH_READ_STATUS_REGISTER, 0, NULL, (unsigned long) 1, NULL, dest_addr );
}

int sflash_read_secure( const sflash_handle_t* const handle, unsigned long device_address, /*@out@*/ /*@dependent@*/ void* const data_addr, unsigned int size )
{
    UNUSED_PARAMETER( handle );
    UNUSED_PARAMETER( device_address );
    UNUSED_PARAMETER( data_addr );
    UNUSED_PARAMETER( size );
    wiced_assert( "sflash_read_secure not supported", WICED_FALSE );
    return 0;
}

int sflash_read( const sflash_handle_t* const handle, unsigned long device_address, /*@out@*/ /*@dependent@*/ void* const data_addr, unsigned int size )
{
    char device_address_array[3] =  { (char) ( ( device_address & 0x00FF0000 ) >> 16 ),
                                      (char) ( ( device_address & 0x0000FF00 ) >>  8 ),
                                      (char) ( ( device_address & 0x000000FF ) >>  0 ) };

    return generic_sflash_command( handle, SFLASH_READ, (unsigned long) 3, device_address_array, (unsigned long) size, NULL, (char *)data_addr );
}

int sflash_get_size( const sflash_handle_t* const handle, /*@out@*/ unsigned long* const size )
{
    uint32_t i = 0;
    *size = 0; /* Unknown size to start with */
    for ( i = 0 ; i < (sizeof(sflash_device_id_size)/sizeof(device_id_to_flash_size_t)) ; i ++ )
    {
        if ( handle->device_id == sflash_device_id_size[i].device_id )
        {
            *size = sflash_device_id_size[i].size;
            break;
        }
    }
    return 0;
}

int sflash_write( const sflash_handle_t* const handle, unsigned long device_address, /*@observer@*/ const void* const data_addr, unsigned int size )
{
    int status;
    unsigned int write_size;
    unsigned int max_write_size = (unsigned int) 1;
    int enable_before_every_write = 1;
    const unsigned char* data_addr_ptr = (const unsigned char*) data_addr;
    unsigned char curr_device_address[3];
    uint8_t result;

    if ( handle->write_allowed != SFLASH_WRITE_ALLOWED )
    {
        return -1;
    }

    /* Some manufacturers support programming an entire page in one command. */

#ifdef SFLASH_SUPPORT_MACRONIX_PARTS
    if ( SFLASH_MANUFACTURER( handle->device_id ) == SFLASH_MANUFACTURER_MACRONIX )
    {
        max_write_size = (unsigned int) 128;  /* TODO: this should be 256, but that causes write errors */
        enable_before_every_write = 1;
    }
#endif /* ifdef SFLASH_SUPPORT_MACRONIX_PARTS */
#ifdef SFLASH_SUPPORT_SST_PARTS
    if ( SFLASH_MANUFACTURER( handle->device_id ) == SFLASH_MANUFACTURER_SST )
    {
        max_write_size = (unsigned int) 1;
        enable_before_every_write = 1;
    }
#endif /* ifdef SFLASH_SUPPORT_SST_PARTS */
#ifdef SFLASH_SUPPORT_EON_PARTS
    if ( SFLASH_MANUFACTURER( handle->device_id ) == SFLASH_MANUFACTURER_EON )
    {
        max_write_size = (unsigned int) 1;
        enable_before_every_write = 1;
    }
#endif /* ifdef SFLASH_SUPPORT_EON_PARTS */
#ifdef SFLASH_SUPPORT_MICRON_PARTS
    if ( SFLASH_MANUFACTURER( handle->device_id ) == SFLASH_MANUFACTURER_MICRON )
    {
        max_write_size = (unsigned int) 128;
        enable_before_every_write = 1;
    }
#endif /* ifdef SFLASH_SUPPORT_MICRON_PARTS */


    if ( enable_before_every_write == 0 )
    {
        status = sflash_write_enable( handle );
        if ( status != 0 )
        {
            return status;
        }
    }

    /* Generic x-bytes-at-a-time write */

    while ( size > 0 )
    {
        write_size = ( size > max_write_size )? max_write_size : size;
        curr_device_address[0] = (unsigned char) ( ( device_address & 0x00FF0000 ) >> 16 );
        curr_device_address[1] = (unsigned char) ( ( device_address & 0x0000FF00 ) >>  8 );
        curr_device_address[2] = (unsigned char) ( ( device_address & 0x000000FF ) >>  0 );

        if ( ( enable_before_every_write == 1 ) &&
             ( 0 != ( status = sflash_write_enable( handle ) ) ) )
        {
            return status;
        }

        status = generic_sflash_command( handle, SFLASH_WRITE, (unsigned long) 3, curr_device_address, (unsigned long) write_size, data_addr_ptr, NULL );
        if ( status != 0 )
        {
            return status;
        }
#ifdef SFLASH_SUPPORT_MICRON_PARTS
    if ( handle->device_id == SFLASH_ID_N25Q512A )
    {
         sflash_read_flag_register(handle, &result);
         sflash_clear_flag_register(handle);
    }
#endif

        data_addr_ptr += write_size;
        device_address += write_size;
        size -= write_size;

    }

    return 0;
}
#ifdef SFLASH_SUPPORT_MICRON_PARTS
int sflash_clear_flag_register( const sflash_handle_t* const handle)
{
   return generic_sflash_command( handle, SFLASH_CLEAR_FLAG_STATUS_REGISTER, 0, NULL, 0, NULL, NULL );
}
int sflash_read_flag_register( const sflash_handle_t* const handle, unsigned char* const dest_addr )
{
    return generic_sflash_command( handle, SFLASH_READ_FLAG_STATUS_REGISTER, 0, NULL, (unsigned long) 1, NULL, dest_addr );
}
#endif

int sflash_write_secure( const sflash_handle_t* const handle, unsigned long device_address, /*@observer@*/ const void* const data_addr, unsigned int size )
{
    UNUSED_PARAMETER( handle );
    UNUSED_PARAMETER( device_address );
    UNUSED_PARAMETER( data_addr );
    UNUSED_PARAMETER( size );
    wiced_assert( "sflash_write_secure not supported", WICED_FALSE );
    return 0;
}

int sflash_write_status_register( const sflash_handle_t* const handle, unsigned char value )
{
    unsigned char status_register_val = value;
#ifdef SFLASH_SUPPORT_SST_PARTS
    /* SST parts require enabling writing to the status register */
    if ( SFLASH_MANUFACTURER( handle->device_id ) == SFLASH_MANUFACTURER_SST )
    {
        int status;
        if ( 0 != ( status = generic_sflash_command( handle, SFLASH_ENABLE_WRITE_STATUS_REGISTER, 0, NULL, 0, NULL, NULL ) ) )
        {
            return status;
        }
    }
#endif /* ifdef SFLASH_SUPPORT_SST_PARTS */

    return generic_sflash_command( handle, SFLASH_WRITE_STATUS_REGISTER, 0, NULL, (unsigned long) 1, &status_register_val, NULL );
}

int deinit_sflash( /*@out@*/ sflash_handle_t* const handle)
{
    int status;
    (void) handle;
    status = sflash_platform_deinit( );
    if ( status != 0 )
    {
        return status;
    }
    return 0;
}

int init_sflash( /*@out@*/ sflash_handle_t* const handle, /*@shared@*/ void* peripheral_id, sflash_write_allowed_t write_allowed_in )
{
    int status;
    device_id_t tmp_device_id;

    status = sflash_platform_init( peripheral_id, &handle->platform_peripheral );
    if ( status != 0 )
    {
        return status;
    }

    handle->write_allowed = write_allowed_in;
    handle->device_id     = 0;

    status = sflash_read_ID( handle, &tmp_device_id );
    if ( status != 0 )
    {
        return status;
    }

    handle->device_id = ( ((uint32_t) tmp_device_id.id[0]) << 16 ) +
                        ( ((uint32_t) tmp_device_id.id[1]) <<  8 ) +
                        ( ((uint32_t) tmp_device_id.id[2]) <<  0 );


    if ( write_allowed_in == SFLASH_WRITE_ALLOWED )
    {
        /* Enable writing */
        if (0 != ( status = sflash_write_enable( handle ) ) )
        {
            return status;
        }
    }

    return 0;
}


static inline int is_write_command( sflash_command_t cmd )
{
    return ( ( cmd == SFLASH_WRITE             ) ||
             ( cmd == SFLASH_CHIP_ERASE1       ) ||
             ( cmd == SFLASH_CHIP_ERASE2       ) ||
             ( cmd == SFLASH_SECTOR_ERASE      ) ||
             ( cmd == SFLASH_BLOCK_ERASE_MID   ) ||
             ( cmd == SFLASH_BLOCK_ERASE_LARGE ) ||
             ( cmd == SFLASH_DIE_ERASE_MICRON  ) ||
             ( cmd == SFLASH_SECTOR_ERASE_MICRON))? 1 : 0;
}

static int generic_sflash_command(                               const sflash_handle_t* const handle,
                                                                 sflash_command_t             cmd,
                                                                 unsigned long                num_initial_parameter_bytes,
                            /*@null@*/ /*@observer@*/            const void* const            parameter_bytes,
                                                                 unsigned long                num_data_bytes,
                            /*@null@*/ /*@observer@*/            const void* const            data_MOSI,
                            /*@null@*/ /*@out@*/ /*@dependent@*/ void* const                  data_MISO )
{
    int status;

    sflash_platform_message_segment_t segments[3] =
    {
            { &cmd,            NULL,       (unsigned long) 1 },
            { parameter_bytes, NULL,       num_initial_parameter_bytes },
            /*@-compdef@*/ /* Lint: Tell lint that it is OK that data_MISO is not completely defined */
            { data_MOSI,       data_MISO,  num_data_bytes }
            /*@+compdef@*/
    };

    status = sflash_platform_send_recv( handle->platform_peripheral, segments, (unsigned int) 3  );
    if ( status != 0 )
    {
        /*@-mustdefine@*/ /* Lint: do not need to define data_MISO due to failure */
        return status;
        /*@+mustdefine@*/
    }

    if ( is_write_command( cmd ) == 1 )
    {
        unsigned char status_register;
        /* write commands require waiting until chip is finished writing */

        do
        {
            status = sflash_read_status_register( handle, &status_register );
            if ( status != 0 )
            {
                /*@-mustdefine@*/ /* Lint: do not need to define data_MISO due to failure */
                return status;
                /*@+mustdefine@*/
            }
        } while( ( status_register & SFLASH_STATUS_REGISTER_BUSY ) != (unsigned char) 0 );

    }

    /*@-mustdefine@*/ /* Lint: lint does not realise data_MISO was set by sflash_platform_send_recv */
    return 0;
    /*@+mustdefine@*/
}

/*@noaccess sflash_handle_t@*/
