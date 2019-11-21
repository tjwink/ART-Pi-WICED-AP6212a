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
 * Bluetooth Remote Control interface
 *
 */
#pragma once

#include "wiced_result.h"
#include "wiced_codec_if.h"
#include "wiced_bt_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define APOLLO_BT_SONG_ARTIST_STR_SIZE (65)
#define APOLLO_BT_SONG_TITLE_STR_SIZE  (257)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    apollo_bt_a2dp_sink_t    *a2dp_sink_ctx;
    wiced_bool_t              rc_is_initialized;
    wiced_bt_device_address_t rc_peer_address;
    uint32_t                  rc_peer_features;
    wiced_bool_t              rc_is_connected;
    uint8_t                   rc_volume;
    uint32_t                  track_duration;
    int8_t                    song_artist[APOLLO_BT_SONG_ARTIST_STR_SIZE];
    int8_t                    song_title[APOLLO_BT_SONG_TITLE_STR_SIZE];
} apollo_bt_rc_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t apollo_bt_remote_control_init( void );
wiced_result_t apollo_bt_remote_control_deinit( void );
wiced_result_t apollo_bt_remote_control_connect( void );
void apollo_bt_remote_control_peer_address_reset( void );
void apollo_bt_remote_control_peer_address_set( wiced_bt_device_address_t peer_addr );

#ifdef __cplusplus
} /* extern "C" */
#endif
