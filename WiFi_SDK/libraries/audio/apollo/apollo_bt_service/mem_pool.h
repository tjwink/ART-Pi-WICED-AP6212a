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
#pragma once

#include "wiced_rtos.h"
#include "linked_list.h"

#ifdef __cplusplus
extern "C" {
#endif

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

typedef struct bt_buffer      bt_buffer_t;
typedef struct bt_buffer_pool bt_buffer_pool_t;
typedef bt_buffer_pool_t* bt_buffer_pool_handle_t;

/******************************************************
 *                    Structures
 ******************************************************/

struct bt_buffer
{
    linked_list_node_t node;
    bt_buffer_pool_t*  pool;
#ifdef MEM_POOL_DEBUG
    char           alloc_string[50];
    char           free_string[50];
#endif
    uint8_t            data[];
};

struct bt_buffer_pool
{
    linked_list_t pool_list;
    uint8_t*      pool_buffer;
    wiced_mutex_t mutex;
    uint16_t      max_packet_count;
    uint16_t      header_size;
    uint16_t      data_size;
    /*members below this line needed only for debugging*/
    uint16_t      free_count;
#ifdef MEM_POOL_DEBUG
    uint16_t      min_free_count;
#endif
};

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t bt_buffer_pool_init( bt_buffer_pool_handle_t* pool_handle, uint16_t buffer_count, uint16_t data_size );

wiced_result_t bt_buffer_pool_deinit( bt_buffer_pool_handle_t pool_handle );

#ifdef MEM_POOL_DEBUG
void* bt_buffer_pool_allocate_buffer_func( const char* alloc_string, bt_buffer_pool_handle_t pool_handle );
#define bt_buffer_pool_allocate_buffer(pool_handle)    bt_buffer_pool_allocate_buffer_func( __func__, pool_handle )
#else
void* bt_buffer_pool_allocate_buffer_func( bt_buffer_pool_handle_t pool_handle );
#define bt_buffer_pool_allocate_buffer( pool_handle )  bt_buffer_pool_allocate_buffer_func( pool_handle )
#endif

#ifdef MEM_POOL_DEBUG
wiced_result_t bt_buffer_pool_free_buffer_func( const char* free_string, void* buffer );
#define bt_buffer_pool_free_buffer(buffer)  bt_buffer_pool_free_buffer_func( __func__, buffer )
#else
wiced_result_t bt_buffer_pool_free_buffer_func( void* buffer );
#define bt_buffer_pool_free_buffer(buffer)  bt_buffer_pool_free_buffer_func( buffer )

#endif

uint32_t bt_buffer_pool_get_free_count( bt_buffer_pool_handle_t pool_handle );

void bt_buffer_pool_print_debug_info( bt_buffer_pool_handle_t pool_handle );

#ifdef __cplusplus
} /* extern "C" */
#endif
