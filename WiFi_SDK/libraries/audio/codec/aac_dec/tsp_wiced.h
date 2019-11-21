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

/**
 * @file tsp.h
 *
 * Collection of thread syncronization primitives MACRO
 *
 */
#ifndef _H_TSP_WICED_H_
#define _H_TSP_WICED_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <wiced_defaults.h>
#include <wiced_rtos.h>
#include <wiced_tcpip.h>


/*
 * audiopcm needs only basic things: threads, mutex, semaphore
 */
#define tsp_mutex_t       wiced_mutex_t
#define tsp_thread_t      wiced_thread_t
#define tsp_attr_t        uint32_t
#define tsp_cond_t        uint32_t
#define tsp_semaphore_t   wiced_semaphore_t

#define tsp_loop_return   void
#define tsp_loop_param    uint32_t

/*
 * THREAD MACROs
 */
#define TSP_THREAD_CREATE( err, thread, attr, func, arg, priority, stack, label ) \
    do                                                                            \
    {                                                                             \
        if ( WICED_SUCCESS != wiced_rtos_create_thread( &( thread ),              \
                                                        ( priority ),             \
                                                        label,                    \
                                                        ( func ),                 \
                                                        ( stack ),                \
                                                        ( arg ) ) )               \
        {                                                                         \
            ( err ) = -1;                                                         \
        }                                                                         \
    } while ( 0 );

#define TSP_THREAD_JOIN( err, thread )         \
    do                                         \
    {                                          \
        wiced_rtos_thread_join( &( thread ) ); \
    } while ( 0 );

#define TSP_THREAD_DELETE( err, thread )         \
    do                                           \
    {                                            \
        wiced_rtos_delete_thread( &( thread ) ); \
    } while ( 0 );

#define TSP_THREAD_EXIT( )              \
    do                                  \
    {                                   \
        WICED_END_OF_CURRENT_THREAD( ); \
    } while ( 0 );


/*
 * MUTEX MACROs
 */
#define TSP_MUTEX_INIT( err, mtx )                                \
    do                                                            \
    {                                                             \
        if ( WICED_SUCCESS != wiced_rtos_init_mutex( &( mtx ) ) ) \
        {                                                         \
            err = -1;                                             \
        }                                                         \
    } while ( 0 );

#define TSP_MUTEX_LOCK( err, mtx )          wiced_rtos_lock_mutex( &( mtx ) );
#define TSP_MUTEX_UNLOCK( err, mtx )        wiced_rtos_unlock_mutex( &( mtx ) );
#define TSP_MUTEX_DESTROY( err, mtx )       wiced_rtos_deinit_mutex( &( mtx ) );

/*
 * SEMAPHORE MACROs
 */
#define TSP_SEMAPHORE_INIT( err, sem )                                \
    do                                                                \
    {                                                                 \
        if ( WICED_SUCCESS != wiced_rtos_init_semaphore( &( sem ) ) ) \
        {                                                             \
            ( err ) = -1;                                             \
        }                                                             \
    } while ( 0 );


#define TSP_SEMAPHORE_TIMEOUT_MS            ( 100 )
#define TSP_SEMAPHORE_POST( err, sem )       wiced_rtos_set_semaphore( &( sem ) );
#define TSP_SEMAPHORE_WAIT( err, sem )       wiced_rtos_get_semaphore( &( sem ), TSP_SEMAPHORE_TIMEOUT_MS );
#define TSP_SEMAPHORE_DESTROY( err, sem )    wiced_rtos_deinit_semaphore( &( sem ) );


#define TSP_THREAD_RETURN_NULL( )           return;


#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* _H_TSP_H_ */
