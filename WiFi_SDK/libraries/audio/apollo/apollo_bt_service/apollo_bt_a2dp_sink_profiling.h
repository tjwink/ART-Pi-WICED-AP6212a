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

/** @file
 *
 * Bluetooth Audio AVDT Sink Service profiling
 */
#pragma once

#include <stdint.h>
#include <stdio.h>
#include "wiced_time.h"
#include "wiced_rtos.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#ifdef APOLLO_BT_PROFILE
#define APOLLO_BT_A2DP_SINK_PROFILING_WRITE( )                                                  \
        do                                                                                      \
        {                                                                                       \
            a2dp_dump_array[a2dp_dump_array_index].seq_num   = p_audio_data->seq_num;           \
            a2dp_dump_array[a2dp_dump_array_index].length    = in_length;                       \
            a2dp_dump_array[a2dp_dump_array_index].timestamp = p_audio_data->timestamp;         \
            wiced_time_get_time(&a2dp_dump_array[a2dp_dump_array_index].systime_ms);            \
            a2dp_dump_array[a2dp_dump_array_index].systime   = host_platform_get_cycle_count(); \
            a2dp_dump_array_index++;                                                            \
        } while(0)
#else
#define APOLLO_BT_A2DP_SINK_PROFILING_WRITE( )
#endif /* APOLLO_BT_PROFILE */

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

typedef struct
{
    uint16_t seq_num;
    uint16_t length;
    uint32_t timestamp;
    uint32_t systime;
    uint32_t systime_ms;
} a2dp_chunk_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

#ifdef APOLLO_BT_PROFILE
extern a2dp_chunk_t *a2dp_dump_array;
extern uint32_t      a2dp_dump_array_index;
#endif /* APOLLO_BT_PROFILE */

/******************************************************
 *               Function Declarations
 ******************************************************/

extern uint32_t host_platform_get_cycle_count( void );

static inline void apollo_bt_a2dp_sink_profiling_dump(void)
{
#ifdef APOLLO_BT_PROFILE
    uint32_t i;
    double  jitter_msec = 0;
    uint32_t jitter_max_msec = 0;
    uint32_t jitter_min_msec = 0xffffffff;
    double average_jitter_msec = 0.0f;
    uint32_t systime_ms = 0;
    uint32_t timestamp = 0;

    printf("packet_seq_num, packet_time_of_arrival_msec, jitter_msec, avg_jitter_msec, packet_timestamp_msec\n");
    for( i = 0; i < a2dp_dump_array_index; i++ )
    {
        jitter_msec    = (i > 0) ? ((double)((uint32_t)(a2dp_dump_array[i].systime - a2dp_dump_array[i-1].systime))) / (double)320000 : 0;

        if ( i > 0 )
        {
            average_jitter_msec += jitter_msec;

            systime_ms = (a2dp_dump_array[i].systime_ms - a2dp_dump_array[0].systime_ms);
            timestamp  = (a2dp_dump_array[i].timestamp - a2dp_dump_array[0].timestamp);

            if ( (uint32_t)jitter_msec > jitter_max_msec )
            {
                jitter_max_msec = (uint32_t)jitter_msec;
            }
            if ( (uint32_t)jitter_msec < jitter_min_msec )
            {
                jitter_min_msec = (uint32_t)jitter_msec;
            }
        }

        printf("%hu,%lu,%f,%f,%lu\n",
                a2dp_dump_array[i].seq_num,
                systime_ms,
                jitter_msec,
                (i > 0) ? (average_jitter_msec / (double)i) : -1.0f,
                timestamp);

        wiced_rtos_delay_milliseconds(10);
    }
    a2dp_dump_array_index = 0;
    printf("\r\njitter_min=%lu, jitter_max=%lu\n", jitter_min_msec, jitter_max_msec);
#endif /* APOLLO_BT_PROFILE */
    return;
}

#ifdef __cplusplus
} /* extern "C" */
#endif
