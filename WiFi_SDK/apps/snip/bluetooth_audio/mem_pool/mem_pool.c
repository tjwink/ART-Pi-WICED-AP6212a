/**
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
 *
 */

#include "wiced.h"
//#include "wiced_bt.h"
#include "mem_pool.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

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

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t bt_buffer_pool_init( bt_buffer_pool_handle_t* pool_handle, uint16_t buffer_count, uint16_t data_size )
{
    uint32_t       padding = (4 - (data_size%4))%4;
    uint32_t       buffer_size = sizeof(bt_buffer_t) + data_size + padding;
    uint32_t       pool_size = buffer_count * buffer_size;
    uint8_t*       buffer_ptr  = NULL;
    void*          buffer      = NULL;
    wiced_result_t result;
    uint32_t       a;
    bt_buffer_pool_t* pool;

#ifdef MEM_POOL_DEBUG
    WPRINT_APP_INFO( ("bt_buffer_pool_init: buffer size (Header+Data+padding) %d + %d + %d, Buffer Count %u, \n", sizeof(bt_buffer_t), data_size, (int) padding, (unsigned int)buffer_count) );
    WPRINT_APP_INFO( ("bt_buffer_pool_init: Pool size (Pool Header + total buffers size) %d + %u \n", sizeof(bt_buffer_pool_t), (unsigned int)pool_size) );
#endif
    pool = malloc( pool_size + sizeof(bt_buffer_pool_t) );
    if ( pool == NULL )
    {
        WPRINT_APP_INFO( ("bt_buffer_pool_init: Pool allocation failed, out of heap space \n") );
        return WICED_BT_OUT_OF_HEAP_SPACE;
    }
    memset( pool, 0, sizeof( *pool ) );

    buffer = (void*)( pool+1 );

    result = linked_list_init( &pool->pool_list );
    if ( result != WICED_BT_SUCCESS )
    {
        free(pool);
        return result;
    }

    pool->max_packet_count = pool->free_count = buffer_count;
#ifdef MEM_POOL_DEBUG
   pool->min_free_count = pool->free_count;
#endif
    pool->header_size      = sizeof( bt_buffer_t );
    pool->data_size        = data_size+padding;
    pool->pool_buffer      = buffer;
    buffer_ptr             = buffer;

    for ( a = 0; a < buffer_count; a++ )
    {
        bt_buffer_t* packet = (bt_buffer_t*)buffer_ptr;
#ifdef MEM_POOL_DEBUG
        WPRINT_APP_INFO( ("bt_buffer_pool_init: Buffer #%u, buffer ptr %p \n", (unsigned int)a, packet->data) );
#endif
        result = linked_list_insert_node_at_front( &pool->pool_list, &packet->node );

        if ( result == WICED_BT_SUCCESS )
        {
            packet->node.data = (void*)packet;
            packet->pool      = pool;
        }
        else
        {
            free(pool);
            return result;
        }

        buffer_ptr += buffer_size;
    }

    result = wiced_rtos_init_mutex( &pool->mutex );
    if(result != WICED_SUCCESS)
    {
        free(pool);
    }
    else
    {
        *pool_handle = (bt_buffer_pool_handle_t)pool;
        WPRINT_APP_INFO( ("bt_buffer_pool_init: Pool creation successful, pool handle %p \n", *pool_handle) );
    }

    return result;
}

wiced_result_t bt_buffer_pool_deinit( bt_buffer_pool_handle_t pool_handle )
{
    bt_buffer_pool_t* pool = (bt_buffer_pool_t*)pool_handle;

    if ( pool == NULL )
    {
        return WICED_BT_BADARG;
    }
    WPRINT_APP_INFO( ("bt_buffer_pool_deinit: destroying pool %p \n", pool_handle) );

    wiced_rtos_deinit_mutex( &pool->mutex );
    free( pool );

    return WICED_BT_SUCCESS;
}


#ifdef MEM_POOL_DEBUG
void* bt_buffer_pool_allocate_buffer_func( const char* alloc_string, bt_buffer_pool_handle_t pool_handle )
#else
void* bt_buffer_pool_allocate_buffer_func( bt_buffer_pool_handle_t pool_handle )
#endif
{
    linked_list_node_t* node   = NULL;
    bt_buffer_pool_t* pool = (bt_buffer_pool_t*)pool_handle;
    uint8_t* buffer = NULL;
    bt_buffer_t* packet;
    wiced_result_t result;

    if ( pool == NULL )
    {
        return NULL;
    }

    wiced_rtos_lock_mutex( &pool->mutex );

    result = linked_list_remove_node_from_rear( &pool->pool_list, &node );

    if ( result == WICED_BT_SUCCESS )
    {
        packet = (bt_buffer_t*)node->data;
        buffer = packet->data;
        pool->free_count--;
#ifdef MEM_POOL_DEBUG
        if(pool->free_count < pool->min_free_count)
            pool->min_free_count = pool->free_count;
        strlcpy(packet->alloc_string, alloc_string, sizeof(packet->alloc_string));
        memset(packet->free_string, 0, sizeof(packet->free_string));
        //WPRINT_APP_INFO( ("bt_buffer_pool_allocate_buffer: allocated by %s    pool handle 0x%x, buffer ptr %p free remaining %d\n", packet->alloc_string, (int)pool_handle, buffer, pool->free_count) );
#endif
    }
    else
    {
#ifdef MEM_POOL_DEBUG
        WPRINT_APP_INFO( ("bt_buffer_pool_allocate_buffer: %s failed to get a buffer. pool handle 0x%x, free_count %d\n", alloc_string, (int) pool_handle, pool->free_count) );
#else
        WPRINT_APP_INFO( ("bt_buffer_pool_allocate_buffer: failed. pool handle 0x%x, free_count %d\n", (int) pool_handle, pool->free_count) );
#endif
    }

    wiced_rtos_unlock_mutex( &pool->mutex );

    return (void*)buffer;
}

#ifdef MEM_POOL_DEBUG
wiced_result_t bt_buffer_pool_free_buffer_func( const char* free_string, void* buffer )
#else
wiced_result_t bt_buffer_pool_free_buffer_func( void* buffer )
#endif
{
    uint8_t* data = (uint8_t*)buffer;
    bt_buffer_t* packet;
    wiced_result_t result;

    if ( buffer == NULL )
    {
        return WICED_BT_BADARG;
    }
    packet = (bt_buffer_t*) (data - sizeof(bt_buffer_t));

    wiced_rtos_lock_mutex( &packet->pool->mutex );
    result = linked_list_insert_node_at_front( &packet->pool->pool_list, &packet->node );
    packet->pool->free_count++;
#ifdef MEM_POOL_DEBUG
    strlcpy(packet->free_string, free_string, sizeof(packet->free_string));
    //WPRINT_APP_INFO( ("bt_buffer_pool_free_buffer: alloced by: %s, freed by: %s, pool handle %p, buffer ptr %p free count after free %d\n", packet->alloc_string, packet->free_string, packet->pool, buffer, packet->pool->free_count) );
#endif
    wiced_rtos_unlock_mutex( &packet->pool->mutex );
    //WPRINT_APP_INFO( ("bt_buffer_pool_free_buffer: pool handle %p, buffer ptr %p free count after free %d\n", packet->pool, buffer, packet->pool->free_count) );

    return result;
}

uint32_t bt_buffer_pool_get_free_count( bt_buffer_pool_handle_t pool_handle )
{
    bt_buffer_pool_t* pool = (bt_buffer_pool_t*)pool_handle;

    return pool->free_count;
}

void bt_buffer_pool_print_debug_info( bt_buffer_pool_handle_t pool_handle )
{
#ifdef MEM_POOL_DEBUG
    bt_buffer_pool_t* pool = (bt_buffer_pool_t*)pool_handle;
    bt_buffer_t* buffer_ptr;
    uint32_t buffer_size = pool->header_size + pool->data_size;
    int i;

    WPRINT_APP_INFO( ("MEM_POOL: printing info for pool handle 0x%x\n", (int) pool_handle) );
    WPRINT_APP_INFO( ("Total buffers = %d, buffer size = %d, free count = %d, MIN Free count = %d\n", pool->max_packet_count,  pool->data_size, pool->free_count, pool->min_free_count) );

    for(i=0; i<pool->max_packet_count;i++)
    {
        buffer_ptr = (bt_buffer_t*)(((uint8_t*)pool->pool_buffer)+(buffer_size*i));
        WPRINT_APP_INFO( ("BUFFER[%2d] allocated by %s\tdeallocated by %s\n", i, buffer_ptr->alloc_string,  buffer_ptr->free_string) );
    }
#endif
}
