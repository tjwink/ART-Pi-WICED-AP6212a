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
#include <stdio.h>
#include <stdlib.h>
#include "command_console.h"
#include "wifi_cert_traffic_stream.h"
#include "wiced_defaults.h"
#include "wiced_tcpip.h"
#include "wwd_wlioctl.h"

#define TS_THREAD_STACK_SIZE ( 2048 )
/* This is the priority offset between ACs. */
#define PRIO_OFFSET_BETWEEN_AC 2
/* All TX thread priority will be atleast this much lower than RX thread */
#define PRIO_BOOST_OF_RX_THREAD_WRT_TX 8

extern traffic_stream_t stream_table[NUM_STREAM_TABLE_ENTRIES];
extern int ac_priority[AC_COUNT];
extern int tx_ac_priority_num[AC_COUNT];
extern int rx_ac_priority_num[AC_COUNT];

void ts_thread_wrapper( uint32_t arg );

int spawn_ts_thread( void *ts_function, traffic_stream_t *ts )
{
    thread_details_t* detail = NULL;
    wiced_result_t ret;

    detail = calloc( 1, sizeof( thread_details_t ) );

    if ( detail == NULL )
    {
        printf( "calloc fail when spawning traffic stream thread\n" );
        return ERR_UNKNOWN;
    }
    detail->ts_function = ts_function;
    detail->ts = ts;
    detail->ts->thread_ptr = detail;

    int prio = WICED_APPLICATION_PRIORITY + 1; // Default + 1 priority for TRAFFIC_ANY direction

//#if ( defined(NETWORK_LwIP) )
    if ( ts->direction == TRAFFIC_SEND )
    {
        /* Priority of TX thread will be always be lower than the RX */
        prio = WICED_APPLICATION_PRIORITY + PRIO_BOOST_OF_RX_THREAD_WRT_TX + ac_priority[ts->ac] * PRIO_OFFSET_BETWEEN_AC + tx_ac_priority_num[ts->ac]++;
    }
    else if ( ts->direction == TRAFFIC_RECV )
    {
        prio = WICED_APPLICATION_PRIORITY + ac_priority[ts->ac] * PRIO_OFFSET_BETWEEN_AC + rx_ac_priority_num[ts->ac]++;;
    }
//#endif
    //printf("P %d AC %d\n", prio, ts->ac);
    if ( WICED_SUCCESS != (ret = wiced_rtos_create_thread( &detail->thread_handle, prio, (const char*)"ts_thread", (wiced_thread_function_t)ts_thread_wrapper, TS_THREAD_STACK_SIZE, (void *)detail )))
    {
        printf("failed to create thread %d\n", ret);
        return ERR_UNKNOWN;
    }

    wiced_rtos_delay_milliseconds(10);

    return ERR_CMD_OK;
}


void ts_thread_wrapper( uint32_t arg )
{
    thread_details_t* detail = (thread_details_t*) arg;
    //wiced_thread_t *tmp_hnd = NULL;

    //printf( "Started thread 0x%08X\n", (unsigned int)detail );


    if ( ( detail->ts->direction == TRAFFIC_RECV ) || ( detail->ts->direction == TRAFFIC_ANY ))
    {
        printf("status,COMPLETE\n" );
    }

    detail->ts_function( detail->ts );

    // All threads will exit to here. We use a mutex to ensure only one tx thread prints results for all tx threads that ran concurrently
    //printf( "Thread 0x%08X exited with return value %d\n", (unsigned int)detail, result );
    host_rtos_delay_milliseconds( 1000 ); // Delay to ensure the linux endpoint is ready and all local transmit streams are finished
    traffic_stream_t *ts = detail->ts;

    if ( ts->direction == TRAFFIC_SEND )
    {
        wiced_rtos_lock_mutex( &tx_done_mutex );
    }

    if ( ts->direction == TRAFFIC_SEND && ts->enabled ) // Print results from all tx streams terminating at the same time
    {
        int i;

        // Disable all transmit streams that finish at the same time so we only print one set of results
        for ( i = 0; i < NUM_STREAM_TABLE_ENTRIES; ++i )
        {
            if ( ts->stop_time == stream_table[i].stop_time && stream_table[i].direction == TRAFFIC_SEND )
            {
                stream_table[i].enabled = 0;
            }
        }

        wiced_rtos_delay_milliseconds( 1000 ); // Delay to ensure the linux endpoint is ready and all local transmit streams are finished

        // Print results for all transmit streams that finished at the same time
        printf("status,COMPLETE,streamID," );

        for ( i = 0; i < NUM_STREAM_TABLE_ENTRIES; ++i )
        {
            if ( ts->stop_time == stream_table[i].stop_time && stream_table[i].direction == TRAFFIC_SEND )
            {
                printf( "%u ", (unsigned int)stream_table[i].stream_id );
            }
        }

        printf( ",txFrames," );
        for ( i = 0; i < NUM_STREAM_TABLE_ENTRIES; ++i )
        {
            if ( ts->stop_time == stream_table[i].stop_time && stream_table[i].direction == TRAFFIC_SEND )
            {
                if (stream_table[i].frames_sent == 0)
                {
                    stream_table[i].frames_sent = 1; // If it's not set to a positive number the Sigma script locks up
                }
                printf( "%d ", stream_table[i].frames_sent );
            }
        }

        printf( ",rxFrames," );
        for ( i = 0; i < NUM_STREAM_TABLE_ENTRIES; ++i )
        {
            if ( ts->stop_time == stream_table[i].stop_time && stream_table[i].direction == TRAFFIC_SEND )
            {
                printf( "%d ", stream_table[i].frames_received );
            }
        }

        printf( ",txPayloadBytes," );
        for ( i = 0; i < NUM_STREAM_TABLE_ENTRIES; ++i )
        {
            if ( ts->stop_time == stream_table[i].stop_time && stream_table[i].direction == TRAFFIC_SEND )
            {
                printf( "%d ", stream_table[i].bytes_sent );
            }
        }

        printf( ",rxPayloadBytes," );
        for ( i = 0; i < NUM_STREAM_TABLE_ENTRIES; ++i )
        {
            if ( ts->stop_time == stream_table[i].stop_time && stream_table[i].direction == TRAFFIC_SEND )
            {
                printf( "%d ", stream_table[i].bytes_received );
            }
        }

        printf( ",outOfSequenceFrames," );
        for ( i = 0; i < NUM_STREAM_TABLE_ENTRIES; ++i )
        {
            if ( ts->stop_time == stream_table[i].stop_time && stream_table[i].direction == TRAFFIC_SEND )
            {
                printf( "%d ", stream_table[i].out_of_sequence_frames );
            }
        }

        printf( "\n" );
        printf( "> " );

    }

    if ( ts->direction == TRAFFIC_SEND )
    {
        tx_ac_priority_num[ts->ac] = 0;
        wiced_rtos_unlock_mutex( &tx_done_mutex );

        // Free the socket so we can reuse the port
        wiced_udp_delete_socket( &ts->tx_socket );
    }
    else
        rx_ac_priority_num[ts->ac] = 0;

    WICED_END_OF_THREAD(NULL);
}
