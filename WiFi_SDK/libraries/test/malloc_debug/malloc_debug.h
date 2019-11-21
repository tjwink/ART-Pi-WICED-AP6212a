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

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#if   defined( RTOS_FreeRTOS )
    #define malloc_get_current_thread( )    ( xTaskGetCurrentTaskHandle( ) )
#elif defined( RTOS_NoOS     )
    #define malloc_get_current_thread( )    ( NULL)
#elif defined( RTOS_ThreadX  )
    #define malloc_get_current_thread( )    ( tx_thread_identify( ) )
#elif defined( RTOS_LinuxOS )
    #define malloc_get_current_thread( )    ( ( void* )pthread_self( ) )
#else
    #error Unknown RTOS
#endif

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef void* malloc_thread_handle;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

extern void* calloc_named                  ( const char* name, size_t nelems, size_t elemsize );
extern void* calloc_named_hideleak         ( const char* name, size_t nelem, size_t elsize );
extern void* malloc_named                  ( const char* name, size_t size );
extern void* malloc_named_hideleak         ( const char* name, size_t size );
extern void  malloc_set_name               ( const char* name );
extern void  malloc_leak_set_ignored       ( leak_check_scope_t global_flag );
extern void  malloc_leak_set_base          ( leak_check_scope_t global_flag );
extern void  malloc_leak_check             ( malloc_thread_handle thread, leak_check_scope_t global_flag );
extern void  malloc_transfer_to_curr_thread( void* block );
extern void  malloc_transfer_to_thread     ( void* block, malloc_thread_handle thread );

#ifdef MALLOC_DEBUG_DISABLE_UNTIL_STARTED
extern void  malloc_debug_startup_finished( void );
#else
#define malloc_debug_startup_finished( )
#endif /* ifdef MALLOC_DEBUG_DISABLE_UNTIL_STARTED */

#ifdef __cplusplus
} /* extern "C" */
#endif
