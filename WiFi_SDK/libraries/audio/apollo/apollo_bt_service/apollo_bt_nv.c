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
#include "wiced_result.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "wiced_framework.h"
#include "wiced_rtos.h"
#include "wiced_bt_dev.h"
#include "apollo_bt_nv.h"


/******************************************************
 *                      Macros
 ******************************************************/
#define PAIRED_DEVICE_INFO_ENTRY_OFFSET( index )   ( BT_NV_PAIRED_DEVICES_TABLE_OFFSET+index*sizeof( apollo_bt_paired_device_info_t ) )
#define GET_INDEX_TO_DEVICE_ENTRY( i )             ( (dev_info_index_list[i]&0x7F) - 1 )

/******************************************************
 *                    Constants
 ******************************************************/
#define SIZEOF_HASH_TABLE                  ( sizeof( apollo_bt_nv_hash_table_t ) )
#define SIZEOF_PAIRED_DEVICES_TABLE        ( sizeof( apollo_bt_paired_device_info_t ) * APOLLO_BT_NV_MAX_LAST_PAIRED_DEVICES )
#define SIZEOF_LOCAL_ID_KEYS               ( sizeof(  wiced_bt_local_identity_keys_t ) )

#define BT_NV_HASH_TABLE_OFFSET            ( apollo_dct_offset_for_bt + OFFSETOF(apollo_bt_dct_t, bt_hash_table) )
#define BT_NV_PAIRED_DEVICES_TABLE_OFFSET  ( apollo_dct_offset_for_bt + OFFSETOF(apollo_bt_dct_t, bt_paired_device_info) )
#define BT_NV_LOCAL_ID_KEYS_OFFSET         ( apollo_dct_offset_for_bt + OFFSETOF(apollo_bt_dct_t, bt_local_id_keys) )

/******************************************************
*                   Enumerations
******************************************************/

/******************************************************
*                 Type Definitions
******************************************************/

/******************************************************
*               Function Declarations
******************************************************/

/******************************************************
*               Variables Definitions
******************************************************/
/*index indicates the priority ( i.e., lowest index, most recently used )
    value indicates:
    first 7 bits  from lsb - the entry's index in the info table
    the 8th bit indicates if the entry is used ( set ) or not ( reset ).
*/
static uint8_t *dev_info_index_list;
static uint8_t dev_list_used_num;
static wiced_mutex_t bt_nv_mutex;
static uint32_t apollo_dct_offset_for_bt;

/******************************************************
*               Function Definitions
******************************************************/
wiced_result_t apollo_bt_nv_init( uint32_t dct_offset_for_bt )
{
    UINT8 i;
    wiced_result_t result;

    result = wiced_rtos_init_mutex( &bt_nv_mutex );
    wiced_assert( "MUTEX init failed", result == WICED_SUCCESS );
    if(result != WICED_SUCCESS)
    {
        APPL_TRACE_ERROR1( "apollo_bt_nv_init: NV mutex initialization failed with error code %d",result );
        return result;
    }

    apollo_dct_offset_for_bt = dct_offset_for_bt;

    //find out how many entries are used up.
    result = wiced_dct_read_lock( ( void ** )&dev_info_index_list, WICED_TRUE, DCT_APP_SECTION, BT_NV_HASH_TABLE_OFFSET, SIZEOF_HASH_TABLE );
    wiced_assert( "index list read failed", result == WICED_SUCCESS );
    if(result != WICED_SUCCESS)
    {
        APPL_TRACE_ERROR1( "apollo_bt_nv_init: NV mutex initialization failed with error code %d",result );
        return result;
    }

    wiced_rtos_lock_mutex( &bt_nv_mutex );
    dev_list_used_num = 0;
    for( i=0; i<APOLLO_BT_NV_MAX_LAST_PAIRED_DEVICES; i++ )
    {
        if( dev_info_index_list[i]&0x80 )
        {
            dev_list_used_num++;
        }
    }
    APPL_TRACE_DEBUG1( "apollo_bt_nv_init: NV device info list initialized, no of devices in NV=%d",dev_list_used_num );
    wiced_rtos_unlock_mutex( &bt_nv_mutex );
    return WICED_SUCCESS;
}

void apollo_bt_nv_deinit( void )
{
    wiced_rtos_deinit_mutex( &bt_nv_mutex );
    wiced_dct_read_unlock( ( void * )dev_info_index_list, WICED_TRUE );
}

wiced_result_t apollo_bt_nv_update_device_link_key( wiced_bt_device_link_keys_t *in_device )
{
    int i, free_index = 0;
    UINT8 table_index;
    apollo_bt_paired_device_info_t *device;

    if( !in_device )
    {
        APPL_TRACE_ERROR0( "apollo_bt_nv_update_device_link_key: NULL argument" );
        return WICED_BADARG;
    }

    wiced_rtos_lock_mutex( &bt_nv_mutex );
    for( i=0; i<dev_list_used_num; i++ )
    {
        table_index = GET_INDEX_TO_DEVICE_ENTRY( i );

        wiced_dct_read_lock( ( void ** )&device, WICED_TRUE, DCT_APP_SECTION,
                    PAIRED_DEVICE_INFO_ENTRY_OFFSET(table_index), sizeof( apollo_bt_paired_device_info_t ) );

        if( !memcmp( device->device_link.bd_addr, in_device->bd_addr, sizeof( wiced_bt_device_address_t ) ) )
        {
            APPL_TRACE_DEBUG2( "apollo_bt_nv_update_device_link_key: Device to be updated found @ dev_info_index_list[%d]=%d\n", i, table_index );
            wiced_dct_write( ( const void * ) in_device, DCT_APP_SECTION,
                            PAIRED_DEVICE_INFO_ENTRY_OFFSET( table_index )+OFFSETOF( apollo_bt_paired_device_info_t, device_link ), sizeof( wiced_bt_device_link_keys_t ) );
            wiced_dct_read_unlock( ( void * ) device, WICED_TRUE );
            dev_info_index_list[i] = (table_index+1)|0x80;
            break; //end loop
        }
        wiced_dct_read_unlock( ( void * ) device, WICED_TRUE );
    }

    if(i == dev_list_used_num)
    {
        if( dev_list_used_num < APOLLO_BT_NV_MAX_LAST_PAIRED_DEVICES )
        {
            free_index = dev_list_used_num++;
            dev_info_index_list[free_index] = free_index + 1;
        }
        else
        {
            //get the last entry
            free_index = dev_list_used_num-1;
        }
        table_index = GET_INDEX_TO_DEVICE_ENTRY( free_index );
        APPL_TRACE_DEBUG3( "apollo_bt_nv_update_device_link_key: new entry, free_index=%d, table_index=%d, no. of devices=%d", free_index, table_index, dev_list_used_num );

        wiced_dct_write( ( void * )in_device, DCT_APP_SECTION,
                        PAIRED_DEVICE_INFO_ENTRY_OFFSET( table_index )+OFFSETOF( apollo_bt_paired_device_info_t, device_link ), sizeof( wiced_bt_device_link_keys_t ) );
        dev_info_index_list[free_index] = (table_index+1)|0x80;

        for( i=0;i<dev_list_used_num;i++ )
        {
            APPL_TRACE_DEBUG2( "apollo_bt_nv_update_device_link_key: dev_info_index_list[%d]=%d", i, GET_INDEX_TO_DEVICE_ENTRY( i ));
        }
        APPL_TRACE_DEBUG2( "apollo_bt_nv_update_device_link_key: Added device @ table position %d, no of devices in NV=%d", table_index, dev_list_used_num );
    }

    wiced_dct_write( dev_info_index_list, DCT_APP_SECTION, BT_NV_HASH_TABLE_OFFSET, SIZEOF_HASH_TABLE );
    wiced_rtos_unlock_mutex( &bt_nv_mutex );

    apollo_bt_nv_update_last_connected_device( in_device->bd_addr );
    return WICED_SUCCESS;
}


wiced_result_t apollo_bt_nv_update_last_connected_device( wiced_bt_device_address_t address )
{
    int i, j;
    UINT8 table_index;
    apollo_bt_paired_device_info_t *device;
    wiced_result_t ret = WICED_ERROR;

    wiced_rtos_lock_mutex( &bt_nv_mutex );
    for( i=0; i<dev_list_used_num; i++ )
    {
        table_index = GET_INDEX_TO_DEVICE_ENTRY( i );

        wiced_dct_read_lock( ( void ** )&device, WICED_FALSE, DCT_APP_SECTION,
                    PAIRED_DEVICE_INFO_ENTRY_OFFSET(table_index), sizeof( apollo_bt_paired_device_info_t ) );

        if( !memcmp( device->device_link.bd_addr, address, sizeof( wiced_bt_device_address_t ) ) )
        {
            APPL_TRACE_DEBUG2( "apollo_bt_nv_update_last_connected_device: Device found @ dev_info_index_list[%d]=%d", i, table_index );
            if( i != 0 ) //to avoid write if the entry is already at the top
            {
                for( j=i; j>0; j-- )
                {
                    dev_info_index_list[j] = dev_info_index_list[j-1];
                }
                dev_info_index_list[0] = (table_index+1)|0x80;

                wiced_dct_write( dev_info_index_list, DCT_APP_SECTION, BT_NV_HASH_TABLE_OFFSET, SIZEOF_HASH_TABLE );
                APPL_TRACE_DEBUG2( "apollo_bt_nv_update_last_connected_device: updated position dev_info_index_list[0]=%d, no of devices in NV=%d", table_index, dev_list_used_num );
            }
            else
            {
                APPL_TRACE_DEBUG0( "apollo_bt_nv_update_last_connected_device: already at 0th position in dev_info_index_list" );
            }
            ret = WICED_SUCCESS;
            wiced_dct_read_unlock( ( void * ) device, WICED_FALSE );
            break;
        }
        wiced_dct_read_unlock( ( void * ) device, WICED_FALSE );
    }
    wiced_rtos_unlock_mutex( &bt_nv_mutex );

    if( ret != WICED_SUCCESS )
        APPL_TRACE_ERROR0( "apollo_bt_nv_update_last_connected_device: Device not found" );

    return ret;
}

wiced_result_t apollo_bt_nv_get_device_info_by_addr( wiced_bt_device_address_t *address, apollo_bt_paired_device_info_t *out_device )
{
    int i;
    UINT8 table_index;
    apollo_bt_paired_device_info_t *device;
    wiced_result_t ret = WICED_ERROR;

    if( !out_device )
    {
        APPL_TRACE_ERROR0( "apollo_bt_nv_get_device_info_by_addr: NULL argument" );
        return WICED_BADARG;
    }

    wiced_rtos_lock_mutex( &bt_nv_mutex );
    for( i=0; i<dev_list_used_num; i++ )
    {
        table_index = GET_INDEX_TO_DEVICE_ENTRY( i );

        wiced_dct_read_lock( ( void ** )&device, WICED_FALSE, DCT_APP_SECTION,
                        PAIRED_DEVICE_INFO_ENTRY_OFFSET( table_index ), sizeof( apollo_bt_paired_device_info_t ) );

        if( !memcmp( device->device_link.bd_addr, address,sizeof( wiced_bt_device_address_t ) ) )
        {
            APPL_TRACE_DEBUG2( "apollo_bt_nv_get_device_info_by_addr: Device found @ dev_info_index_list[%d]=%d", i, table_index );
            memcpy( out_device, device, sizeof( apollo_bt_paired_device_info_t ) );

            wiced_dct_read_unlock( ( void * ) device, WICED_FALSE );
            ret = WICED_SUCCESS;
            break;
        }
        wiced_dct_read_unlock( ( void * ) device, WICED_FALSE );
    }
    wiced_rtos_unlock_mutex( &bt_nv_mutex );

    if( ret != WICED_SUCCESS )
    {
         APPL_TRACE_ERROR0( "apollo_bt_nv_get_device_info_by_addr: Device not found" );
    }

    return ret;
}


wiced_result_t apollo_bt_nv_get_device_info_by_index( uint8_t index, apollo_bt_paired_device_info_t *out_device )
{
    UINT8 table_index;
    apollo_bt_paired_device_info_t *device;

    if( index >= dev_list_used_num )
    {
        return WICED_BADARG;
    }

    if( !out_device )
    {
        return WICED_BADARG;
    }

        wiced_rtos_lock_mutex( &bt_nv_mutex );

        table_index = GET_INDEX_TO_DEVICE_ENTRY( index );

        wiced_dct_read_lock( ( void ** )&device, WICED_FALSE, DCT_APP_SECTION,
                        PAIRED_DEVICE_INFO_ENTRY_OFFSET( table_index ), sizeof( apollo_bt_paired_device_info_t ) );
        memcpy( out_device, device, sizeof( apollo_bt_paired_device_info_t ) );
        wiced_dct_read_unlock( ( void * ) device, WICED_FALSE );
        wiced_rtos_unlock_mutex( &bt_nv_mutex );

        APPL_TRACE_DEBUG2( "apollo_bt_nv_get_device_info_by_index: Device found @ dev_info_index_list[%d]=%d", index, table_index );
        return WICED_SUCCESS;
}

wiced_result_t apollo_bt_nv_delete_device_info( wiced_bt_device_address_t address )
{
    int i, j;
    UINT8 table_index;
    apollo_bt_paired_device_info_t *device;
    wiced_result_t ret = WICED_ERROR;

    wiced_rtos_lock_mutex( &bt_nv_mutex );
    for( i=0; i<dev_list_used_num; i++ )
    {
        table_index = GET_INDEX_TO_DEVICE_ENTRY( i );

        wiced_dct_read_lock( ( void ** )&device, WICED_FALSE, DCT_APP_SECTION,
                        PAIRED_DEVICE_INFO_ENTRY_OFFSET( table_index ), sizeof( apollo_bt_paired_device_info_t ) );
        if( !memcmp( device->device_link.bd_addr,address,sizeof( wiced_bt_device_address_t ) ) )
        {
            APPL_TRACE_DEBUG2( "apollo_bt_nv_delete_device_info: Device found @ dev_info_index_list[%d]=%d", i, table_index );
            wiced_dct_read_unlock( ( void * ) device, WICED_FALSE );

            dev_list_used_num--;
            for( j=i; j<dev_list_used_num; j++ )
            {
                dev_info_index_list[j] = dev_info_index_list[j+1];
            }
            dev_info_index_list[dev_list_used_num] = table_index&0x7F;

            wiced_dct_write( dev_info_index_list, DCT_APP_SECTION, BT_NV_HASH_TABLE_OFFSET, SIZEOF_HASH_TABLE );
            ret = WICED_SUCCESS;
            break;
        }
        wiced_dct_read_unlock( ( void * ) device, WICED_FALSE );
    }
    wiced_rtos_unlock_mutex( &bt_nv_mutex );

    if( ret != WICED_SUCCESS )
        APPL_TRACE_ERROR0( "apollo_bt_nv_delete_device_info: Device not found" );

    return ret;
}


wiced_result_t apollo_bt_nv_delete_device_info_list( void )
{
    wiced_rtos_lock_mutex( &bt_nv_mutex );
    memset(dev_info_index_list, 0, APOLLO_BT_NV_MAX_LAST_PAIRED_DEVICES);
    wiced_dct_write( dev_info_index_list, DCT_APP_SECTION, BT_NV_HASH_TABLE_OFFSET, SIZEOF_HASH_TABLE );
    APPL_TRACE_DEBUG0( "apollo_bt_nv_delete_device_info_list: deleted device list" );
    wiced_rtos_unlock_mutex( &bt_nv_mutex );
    return WICED_SUCCESS;
}

wiced_result_t apollo_bt_nv_update_local_id_keys( wiced_bt_local_identity_keys_t *keys)
{
    if(keys == NULL)
    {
        return WICED_BADARG;
    }

    wiced_rtos_lock_mutex( &bt_nv_mutex );
    wiced_dct_write( keys, DCT_APP_SECTION, BT_NV_LOCAL_ID_KEYS_OFFSET, SIZEOF_LOCAL_ID_KEYS );
    APPL_TRACE_DEBUG0( "apollo_bt_nv_update_local_id_keys: update_local_id_keys" );
    wiced_rtos_unlock_mutex( &bt_nv_mutex );
    return WICED_SUCCESS;
}

wiced_result_t apollo_bt_nv_get_local_id_keys( wiced_bt_local_identity_keys_t *keys)
{
    wiced_bt_local_identity_keys_t *key_in_dct;
    if(keys == NULL)
    {
        return WICED_BADARG;
    }

    wiced_rtos_lock_mutex( &bt_nv_mutex );
    APPL_TRACE_DEBUG0( "apollo_bt_nv_get_local_id_keys: get_local_id_keys" );
    wiced_dct_read_lock( ( void ** )&key_in_dct, WICED_FALSE, DCT_APP_SECTION, BT_NV_LOCAL_ID_KEYS_OFFSET, SIZEOF_LOCAL_ID_KEYS );
    memcpy(keys, key_in_dct, SIZEOF_LOCAL_ID_KEYS);

    wiced_rtos_unlock_mutex( &bt_nv_mutex );
    return WICED_SUCCESS;
}
