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

#include "wiced.h"
#include "gspi_slave.h"
#include "gspi_sw_header.h"
#include "gspi_basic.h"
#include "platform_cache.h"

/******************************************************
 *                   Enumerations
 ******************************************************/
/******************************************************
 *                    Structures
 ******************************************************/
/******************************************************
*                      Macros
******************************************************/

#define GSPI_SLAVE_THREAD_STACK_SIZE  ( 4096 )
#define GSPI_SLAVE_THREAD_PRIORITY    ( WICED_DEFAULT_LIBRARY_PRIORITY )

/******************************************************
 *               Variable Definitions
 ******************************************************/

static platform_gspi_slave_driver_t gspi_slave_driver;
static gspi_shared_info_t shared_info;

/******************************************************
 *             Static Function Declarations
 ******************************************************/
static void gspi_slave_send_status( gspi_slave_device_t* gspi_slave, gspi_slave_status_t status );
static void gspi_command_process_thread( uint32_t arg );
static void gspi_slave_indicate_slave_ready( void );
static void gspi_slave_init_shared_struct( uint32_t tx_buffer_address, uint32_t tx_buffer_size, uint32_t rx_buffer_address, uint32_t rx_buffer_size );

static void gspi_slave_indicate_slave_ready( void )
{
    platform_gspi_slave_indicate_slave_ready();
}

static void gspi_slave_init_shared_struct( uint32_t tx_buffer_address, uint32_t tx_buffer_size, uint32_t rx_buffer_address, uint32_t rx_buffer_size )
{
    memset( (void *)&shared_info, 0, sizeof( shared_info ) );

    shared_info.magic_number                   = GSPI_SHARED_MAGIC_NUMBER;
    shared_info.slave_to_master_buffer_address = tx_buffer_address;
    shared_info.slave_to_master_buffer_size    = tx_buffer_size;
    shared_info.master_to_slave_buffer_address = rx_buffer_address;
    shared_info.master_to_slave_buffer_size    = rx_buffer_size;

    platform_dcache_clean_range( &shared_info, sizeof( shared_info ) );

    platform_gspi_set_shared_address( (uint32_t)&shared_info );

    WPRINT_LIB_INFO( ("%s, set shared address:0x%lx\n", __FUNCTION__, (uint32_t)&shared_info) );
}

wiced_result_t gspi_slave_init( gspi_slave_device_t* gspi_slave, const gspi_slave_application_t* gspi_slave_app )
{
    wiced_result_t result;
    void *slave_to_master_buffer, *master_to_slave_buffer;

    result = platform_gspi_slave_init(&gspi_slave_driver);

    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ("[%s] Init slave failed!\n", __FUNCTION__) );
        return WICED_ERROR;
    }
    memset( gspi_slave, 0, sizeof( *gspi_slave ) );

    gspi_slave->gspi_slave_application = gspi_slave_app;
    gspi_slave->slave_to_master_buffer_size = sizeof( gspi_slave_response_t ) + gspi_slave_app->send_data_buffer_size;
    gspi_slave->master_to_slave_buffer_size = sizeof( gspi_sw_tx_header_t ) - sizeof( uint8_t ) + gspi_slave_app->receive_data_buffer_size;

    gspi_slave->quit = WICED_FALSE;

    slave_to_master_buffer      =  malloc_named( "GSPI slave TX buffer", gspi_slave->slave_to_master_buffer_size );

    if ( slave_to_master_buffer == NULL )
    {
        WPRINT_LIB_ERROR( ("[%s], alloc TX buffer failed!\n", __FUNCTION__) );
        return WICED_OUT_OF_HEAP_SPACE;
    }
    else
    {
        gspi_slave->slave_to_master_buffer = (uint8_t *)platform_addr_cached_to_uncached( slave_to_master_buffer );
    }

    master_to_slave_buffer      = (uint8_t*) malloc_named( "GSPI slave RX buffer", gspi_slave->master_to_slave_buffer_size );

    if ( master_to_slave_buffer == NULL )
    {
        free( slave_to_master_buffer );
        WPRINT_LIB_ERROR( ("[%s], alloc RX buffer failed!\n", __FUNCTION__) );
        return WICED_OUT_OF_HEAP_SPACE;
    }
    else
    {
        gspi_slave->master_to_slave_buffer = (uint8_t *)platform_addr_cached_to_uncached( master_to_slave_buffer );

    }

    memset( gspi_slave->slave_to_master_buffer, 0, gspi_slave->slave_to_master_buffer_size );
    memset( gspi_slave->master_to_slave_buffer, 0, gspi_slave->master_to_slave_buffer_size );

    gspi_slave_init_shared_struct( (uint32_t)gspi_slave->slave_to_master_buffer, gspi_slave->slave_to_master_buffer_size,
                                    (uint32_t)gspi_slave->master_to_slave_buffer, gspi_slave->master_to_slave_buffer_size );

    WPRINT_LIB_INFO( ("[%s], Address of slave_to_master_buffer: 0x%lx, master_to_slave_buffer: 0x%lx\n", __FUNCTION__,
                        (uint32_t)gspi_slave->slave_to_master_buffer, (uint32_t)gspi_slave->master_to_slave_buffer ) );

    /* Create SPI device thread */
    result = wiced_rtos_create_thread( &gspi_slave->thread, GSPI_SLAVE_THREAD_PRIORITY, "GSPI Slave", gspi_command_process_thread,
                                         GSPI_SLAVE_THREAD_STACK_SIZE, (void*) gspi_slave );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ("[%s], create thread failed!\n", __FUNCTION__) );
        goto cleanup;
    }
    gspi_slave_indicate_slave_ready();

    return WICED_SUCCESS;

  cleanup:

    free( slave_to_master_buffer );
    free( master_to_slave_buffer );
    gspi_slave->slave_to_master_buffer = NULL;
    gspi_slave->master_to_slave_buffer = NULL;

    return WICED_ERROR;
}

static void gspi_slave_send_status( gspi_slave_device_t* gspi_slave, gspi_slave_status_t status )
{
    platform_gspi_transfer_data( gspi_slave->slave_to_master_buffer , &status, sizeof( status ) );
    platform_gspi_slave_notify_master();
}

void gspi_slave_send_response_data( gspi_slave_device_t* gspi_slave, uint8_t* data, uint32_t data_length )
{
    gspi_slave_response_t gspi_slave_resp;
    gspi_slave_resp.status = GSPI_SLAVE_RESPONSE_GOOD;
    gspi_slave_resp.data_length = data_length;

    /* Transfer response */
    platform_gspi_transfer_data( gspi_slave->slave_to_master_buffer, (uint8_t*)&gspi_slave_resp, sizeof( gspi_slave_resp ) );

    /* Transfer data */
    platform_gspi_transfer_data( gspi_slave->slave_to_master_buffer + sizeof ( gspi_slave_resp ), data, data_length );

    platform_gspi_slave_notify_master();
}

static void gspi_command_process_thread( uint32_t arg )
{
    wiced_result_t              result;
    gspi_slave_device_t * gspi_slave = (gspi_slave_device_t *)(arg);

    gspi_sw_tx_header_t *gspi_sw_tx_hdr;

    WPRINT_LIB_ERROR( ("Start to receive GSPI command from master..\n") );

    while(gspi_slave->quit != WICED_TRUE )
    {
        result = (wiced_result_t)platform_gspi_slave_receive_command( &gspi_slave_driver, WICED_NEVER_TIMEOUT );

        if (result != WICED_SUCCESS )
            continue;

        gspi_sw_tx_hdr = (gspi_sw_tx_header_t *)gspi_slave->master_to_slave_buffer;

        if ( gspi_sw_tx_hdr->gspi_sw_hdr.direction == GSPI_MASTER_TRANSFER_WRITE )
        {
            gspi_slave_send_status( gspi_slave, GSPI_SLAVE_RESPONSE_GOOD );

        }

        gspi_slave->gspi_slave_application->command_callback( gspi_sw_tx_hdr );

    }
}
