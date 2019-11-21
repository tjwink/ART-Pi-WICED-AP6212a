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
 * @file Audio PLL tuning
 */

#pragma once

#include "wiced_result.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/*
 * set of default values appropriate for initialization of audio_pll_tuner_init_params_t
 */

#define AUDIO_PLL_TUNER_DEFAULT_ADJ_PPM_MAX                    (+2000)
#define AUDIO_PLL_TUNER_DEFAULT_ADJ_PPM_MIN                    (-2000)
#define AUDIO_PLL_TUNER_DEFAULT_ADJ_RATE_PPB_PER_MSEC          (2000)
#define AUDIO_PLL_TUNER_DEFAULT_ADJ_ATTACK_RATE                (20)
#define AUDIO_PLL_TUNER_DEFAULT_ADJ_DECAY_RATE                 (15)
#define AUDIO_PLL_TUNER_DEFAULT_LEVEL_CORRECTION_THRES_MAX     (+2000)
#define AUDIO_PLL_TUNER_DEFAULT_LEVEL_CORRECTION_THRES_MIN     (-2000)
#define AUDIO_PLL_TUNER_DEFAULT_ADJ_PROPORTIONAL_GAIN          (20)
#define AUDIO_PLL_TUNER_DEFAULT_ADJ_INTEGRAL_GAIN              (2)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    AUDIO_PLL_TUNER_TIME_UNIT_UNKNOWN          = -1,
    AUDIO_PLL_TUNER_TIME_UNIT_NANOSECONDS      =  0,
    AUDIO_PLL_TUNER_TIME_UNIT_PART_PER_BILLION =  1,
} audio_pll_tuner_time_unit_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct audio_pll_tuner_s *audio_pll_tuner_ref;

typedef struct
{
    uint64_t                    ts_reference;
    uint64_t                    ts_audio_timer;
    uint32_t                    payload_size;
    audio_pll_tuner_time_unit_t ts_audio_timer_unit;
} audio_pll_tuner_timestamp_t;

typedef wiced_result_t (*audio_pll_tuner_start_timer_cb_t)       (uint32_t audio_sample_count,     void *user_context);
typedef wiced_result_t (*audio_pll_tuner_stop_timer_cb_t)        (                                 void *user_context);
typedef wiced_result_t (*audio_pll_tuner_wait_for_period_cb_t)   (uint32_t timeout_ms,             void *user_context);
typedef wiced_result_t (*audio_pll_tuner_get_buffer_level_cb_t)  (uint32_t *level_in_bytes,        void *user_context);
typedef wiced_result_t (*audio_pll_tuner_set_ppb_cb_t)           (int32_t  ppb,                    void *user_context);
typedef wiced_result_t (*audio_pll_tuner_get_time_cb_t)          (audio_pll_tuner_timestamp_t *ts, void *user_context);


/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    int32_t                                 adjustment_ppm_max;
    int32_t                                 adjustment_ppm_min;
    int32_t                                 adjustment_ppb_per_msec;
    int32_t                                 adjustment_attack_rate;
    int32_t                                 adjustment_decay_rate;
    int32_t                                 level_correction_threshold_high;
    int32_t                                 level_correction_threshold_low;
    int32_t                                 adjustment_proportional_gain;
    int32_t                                 adjustment_integral_gain;
    void                                   *user_context;
    audio_pll_tuner_start_timer_cb_t        timer_start;
    audio_pll_tuner_stop_timer_cb_t         timer_stop;
    audio_pll_tuner_wait_for_period_cb_t    period_wait;
    audio_pll_tuner_get_buffer_level_cb_t   buffer_level_get;
    audio_pll_tuner_set_ppb_cb_t            ppb_set;
    audio_pll_tuner_get_time_cb_t           get_time;
} audio_pll_tuner_init_params_t;

typedef struct
{
    uint32_t target_buffer_level_bytes;
    uint32_t sample_rate;
    uint8_t  bits_per_sample;
    uint8_t  channels;
} audio_pll_tuner_start_params_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/** Initialize audio PLL tuner library
 *
 * @param[ in] params          : Audio PLL tuning input parameters
 * @param[out] pll_tuner_ptr   : Storage for Audio PLL tuner handle @ref audio_pll_tuner_ref
 *
 * @return @ref wiced_result_t
 */
wiced_result_t audio_pll_tuner_init( audio_pll_tuner_init_params_t *params, audio_pll_tuner_ref *pll_tuner_ptr );


/** Clean up audio PLL tuner library
 *
 * @param[ in] pll_tuner       : Audio PLL tuner handle
 *
 * @return @ref wiced_result_t
 */
wiced_result_t audio_pll_tuner_deinit( audio_pll_tuner_ref pll_tuner );


/** Start audio PLL tuning
 *
 * @param[ in] pll_tuner       : Audio PLL tuner handle
 * @param[ in] sample_rate     : Audio sample rate
 * @param[ in] bits_per_sample : Audio sample size (in bits)
 * @param[ in] channels        : Audio channel count
 *
 * @return @ref wiced_result_t
 */
wiced_result_t audio_pll_tuner_start( audio_pll_tuner_ref pll_tuner,  audio_pll_tuner_start_params_t *params );


/** Stop audio PLL tuning
 *
 * @param pll_tuner            : Audio PLL tuner handle
 *
 * @return @ref wiced_result_t
 */
wiced_result_t audio_pll_tuner_stop( audio_pll_tuner_ref pll_tuner );


/** Provide reference and audio timer timestamps to drive audio PLL tuning algorithm
 *
 * @param pll_tuner            : Audio PLL tuner handle
 * @param ts                   : Audio timestamps
 *
 * @return @ref wiced_result_t
 */
wiced_result_t audio_pll_tuner_push_timestamp( audio_pll_tuner_ref pll_tuner, audio_pll_tuner_timestamp_t *ts );

#ifdef __cplusplus
} /* extern "C" */
#endif
