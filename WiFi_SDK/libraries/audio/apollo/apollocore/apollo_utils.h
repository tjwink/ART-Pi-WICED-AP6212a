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
 * @file Apollo utility routines
 */

#pragma once

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

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/** Returns channel mapping bitmask
 *
 * @param[in] num_channels   : number of audio channels
 *
 * @return @ref int
 */
int apollo_get_chmap_value( int num_channels );


/** Returns number of audio channels
 *
 * @param[in] chmap          : channel map bitmask
 *
 * @return @ref int
 */
int apollo_get_num_channels( int chmap );


/** Returns volume attenuation index
 *
 * @param[in] vol_att_in_mB  : volume attenuation in milliBels
 *
 * @return @ref uint8_t
 */
uint8_t apollo_get_volume_attenuation_index( uint8_t vol_att_percent );


/** Returns volume attenuation in milliBels
 *
 * @param[in] vol_att_index  : volume index (from 0 to 64)
 *
 * @return @ref int32_t
 */
uint8_t apollo_get_volume_attenuation_in_percent( uint8_t vol_att_index );


/** Returns sink jitter index based on requested sink jitter
 *
 * @param[in]  requested_sink_jitter_nsec : requested sink jitter in nanoseconds
 * @param[out] actual_sink_jitter_nsec    : (optional) actually assigned sink jitter (larger or equal to request)
 *
 *  @return @ref uint32_t
 */
uint32_t apollo_get_sink_jitter_index( uint32_t requested_sink_jitter_nsec, uint32_t* actual_sink_jitter_nsec );


/** Returns sink jitter in nanoseconds based on index
 *
 * @param[in] sink_jitter_index           : sink jitter index
 *
 *  @return @ref uint32_t
 */
uint32_t apollo_get_sink_jitter_nsecs( uint32_t sink_jitter_index );


/** Returns latency target index based on requested latency target
 *
 * @param[in]  requested_latency_target_msec : requested latency target in msecs
 * @param[out] actual_latency_target_msec    : (optional) actually assigned latency target (larger or equal to request)
 *
 *  @return @ref uint32_t
 */
uint32_t apollo_get_latency_target_index( uint32_t requested_latency_target_msec, uint32_t* actual_latency_target_msec );


/** Returns latency target in milliseconds based on index
 *
 * @param[in] latency_target_index           : latency target index
 *
 *  @return @ref uint32_t
 */
uint32_t apollo_get_latency_target_msecs( uint32_t latency_target_index );

#ifdef __cplusplus
} /* extern "C" */
#endif
