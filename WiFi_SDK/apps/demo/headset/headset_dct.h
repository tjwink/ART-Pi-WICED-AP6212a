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

#ifdef __cplusplus
extern "C" {
#endif

#include "platform_audio.h"

/******************************************************
 *                      Macros
 ******************************************************/
/* Get WLAN Interface */
#define HEADSET_DCT_WLAN_INTERFACE(p_headset_context)   p_headset_context->dct_tables.dct_network->interface

/* Get Powersave parameters */
#define HEADSET_DCT_POWERSAVE_PARAMS(p_headset_context) p_headset_context->dct_tables.dct_app->powersave_dct

/* Get Audio parameters */
#define HEADSET_DCT_AUDIO_PARAMS(p_headset_context)     p_headset_context->dct_tables.dct_app->audio_dct

/******************************************************
 *                    Constants
 ******************************************************/
#define HEADSET_MCU_POWERSAVE_SLEEP_MODE     (PLATFORM_MCU_POWERSAVE_MODE_SLEEP)
/* tickless_never consumes more power but we need the tick to check the button input in a short time */
#define HEADSET_MCU_POWERSAVE_TICK_MODE      (PLATFORM_TICK_POWERSAVE_MODE_TICKLESS_NEVER)
#define HEADSET_MCU_POWERSAVE_CLOCK_FREQ     (PLATFORM_CPU_CLOCK_FREQUENCY_120_MHZ)
#define HEADSET_WLAN_POWERSAVE_MODE          (PM2_POWERSAVE_MODE)
#define HEADSET_WLAN_PM2_RET_ON_HIGH         (10)
#define HEADSET_WLAN_PM2_RET_ON_LOW          (200) /* Not used, just disabling the PM on low threshold state */

#define HEADSET_AUDIO_VOLUME_MIN             (0)
#define HEADSET_AUDIO_VOLUME_DEFAULT         (36)
#define HEADSET_AUDIO_VOLUME_MAX             (100)
#define HEADSET_AUDIO_NUM_HTTP_BUFFERS       (250)
#define HEADSET_AUDIO_NUM_AUDIO_BUFFERS      (80)
#define HEADSET_AUDIO_SIZE_AUDIO_BUFFERS     (2048)
#define HEADSET_AUDIO_SIZE_AUDIO_PERIOD      (512)
#define HEADSET_AUDIO_HTTP_THRESHOLD_HIGH    (HEADSET_AUDIO_NUM_HTTP_BUFFERS - 5)
/* Bigger low threshold makes stable RX and Smaller low threshold makes low power */
#define HEADSET_AUDIO_HTTP_THRESHOLD_LOW     (HEADSET_AUDIO_HTTP_THRESHOLD_HIGH - 95)
#define HEADSET_AUDIO_HTTP_PREROLL           (HEADSET_AUDIO_HTTP_THRESHOLD_HIGH)
#define HEADSET_AUDIO_HTTP_READ_INHIBIT      (0)
#define HEADSET_AUDIO_DEFAULT_APP_PLAYBACK   (0)


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
    int mcu_sleep_mode;
    int mcu_sleep_tick_mode;
    int mcu_clock_freq;

    /* WiFi PM mode; 0 = disable, 1 = PM1, 2 = PM2 */
    int wlan_pm_mode;
    /* PM2 return to sleep value
     * applied when audio buffer hits http high/low threshold */
    int wlan_pm2_ret_on_high;
    int wlan_pm2_ret_on_low;
} powersave_dct_t;

typedef struct
{
    int volume;
    platform_audio_device_id_t audio_device_tx;
    int app_playback;
    int http_buffer_num;
    int audio_buffer_num;
    int audio_buffer_size;
    int audio_period_size;
    int http_threshold_high;
    int http_threshold_low;
    int http_read_inhibit;
} audio_dct_t;

typedef struct
{
    powersave_dct_t     powersave_dct;
    audio_dct_t         audio_dct;
    /* TODO: Add BT, Airplay, ... */
} headset_dct_t;


#ifdef __cplusplus
} /* extern "C" */
#endif
