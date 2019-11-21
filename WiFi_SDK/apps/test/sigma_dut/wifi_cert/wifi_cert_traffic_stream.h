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

#include <stdint.h>
#include "compat.h"
#include "wwd_constants.h"
#include "wiced_rtos.h"
#include "wiced_network.h"
#include "wiced.h"
#include "wiced_time.h"
#include "wiced.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_STREAM_TABLE_ENTRIES ( 4 )

typedef enum
{
    TRAFFIC_ANY        = 0x00,
    TRAFFIC_SEND       = 0x01,
    TRAFFIC_RECV       = 0x02,
} traffic_direction_t;

typedef enum
{
    PROFILE_FILE_TRANSFER = 0x01,
    PROFILE_MULTICAST     = 0x02,
    PROFILE_IPTV          = 0x03,
    PROFILE_TRANSACTION   = 0x04,
} traffic_profile_t;

typedef struct
{
    uint8_t allocated; // Indicates if stream table entry has been allocated for use
    uint8_t enabled; // Indicates if stream is currently active
    uint32_t stream_id;
    traffic_profile_t profile;
    traffic_direction_t direction;
    char dest_ipaddr[16];
    uint16_t dest_port;
    char src_ipaddr[16];
    uint16_t src_port;
    int payload_size;
    int frame_rate; // Frames per second
    int duration;
    int start_delay;
    wiced_qos_access_category_t ac;
    int max_frame_count;
    int frames_sent;
    int frames_received;
    int out_of_sequence_frames;
    int bytes_sent;
    int bytes_received;
    wiced_udp_socket_t tx_socket;
    wiced_udp_socket_t rx_socket;
    wiced_time_t stop_time;
    void* thread_ptr; // This allows the console thread to delete threads
} traffic_stream_t;

typedef struct
{
    wiced_thread_t thread_handle;
    traffic_stream_t *ts;
    int (*ts_function)( traffic_stream_t * );
} thread_details_t;

extern wiced_mutex_t  tx_done_mutex;

int udp_rx( traffic_stream_t* ts );
int udp_tx( traffic_stream_t* ts );
int udp_transactional( traffic_stream_t* ts );


#ifdef __cplusplus
} /* extern "C" */
#endif
