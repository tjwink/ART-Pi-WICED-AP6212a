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

#include "wiced_platform.h"
#include "wiced_log.h"
#include "platform_audio.h"
#include "apollocore.h"

#ifndef APOLLO_NO_BT
#include "apollo_bt_nv.h"
#endif

#if defined(OTA2_SUPPORT)
#include "daemons/ota2_service/wiced_ota2_service.h"
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#ifdef USE_UPNPAV
#define APOLLO_ROLE_DEFAULT                 APOLLO_ROLE_SOURCE
#define APOLLO_AUDIO_SOURCE_DEFAULT         APOLLO_AUDIO_SOURCE_EXTERNAL
#else
#define APOLLO_ROLE_DEFAULT                 APOLLO_ROLE_SINK
#define APOLLO_AUDIO_SOURCE_DEFAULT         APOLLO_AUDIO_SOURCE_BT
#endif

#define APOLLO_BUFFERING_MS_MIN             0
#define APOLLO_BUFFERING_MS_DEFAULT         10
#define APOLLO_BUFFERING_MS_MAX             1000

#define APOLLO_THRESHOLD_NS_MIN             10000
#define APOLLO_THRESHOLD_NS_DEFAULT         1000000
#define APOLLO_THRESHOLD_NS_MAX             10000000

#define APOLLO_VOLUME_MIN                   0
#define APOLLO_VOLUME_DEFAULT               70
#define APOLLO_VOLUME_MAX                   100

/* BURST LENGTH: for SLC use auto, for non SLC use 6 */
#define APOLLO_BURST_LENGTH_DEFAULT         APOLLO_STREAMER_BURST_AUTO_SLC

#define APOLLO_AUDIO_DEVICE_STRING_LENGTH   16

#define APOLLO_INPUT_SAMPLE_RATE_DEFAULT    PLATFORM_AUDIO_SAMPLE_RATE_48KHZ
#define APOLLO_INPUT_SAMPLE_SIZE_DEFAULT    PLATFORM_AUDIO_SAMPLE_SIZE_16_BIT
#define APOLLO_INPUT_CHANNEL_COUNT          2

#define APOLLO_PLL_TUNING_PPM_MAX_LOW       300
#define APOLLO_PLL_TUNING_PPM_MAX_DEFAULT   300
#define APOLLO_PLL_TUNING_PPM_MAX_HIGH      2000

#define APOLLO_PLL_TUNING_PPM_MIN_LOW       (-2000)
#define APOLLO_PLL_TUNING_PPM_MIN_DEFAULT   (-500)
#define APOLLO_PLL_TUNING_PPM_MIN_HIGH      (-300)

/******************************************************
 *                    Constants
 ******************************************************/

/* This is the default RMC for the device */
#define APOLLO_RMC_SSID       "apollo"
#define APOLLO_RMC_PASSPHRASE "abcd1234"
#ifdef USE_UPNPAV
#define APOLLO_RMC_BSS_TYPE   WICED_BSS_TYPE_INFRASTRUCTURE
#else
/* Default operating mode is Infrastructure mode */
#define APOLLO_RMC_BSS_TYPE   WICED_BSS_TYPE_INFRASTRUCTURE

/* This is for Ad-hoc mode (not recommended for new designs) */
/* #define APOLLO_RMC_BSS_TYPE   WICED_BSS_TYPE_ADHOC */
#endif
#define APOLLO_RMC_SECURITY   WICED_SECURITY_OPEN
#define APOLLO_RMC_CHANNEL    149
#define APOLLO_RMC_BAND       WICED_802_11_BAND_5GHZ

/*
 * define the current version (for upgrades) here or on make command line as args
 * make <application>-<platform> ota2_image APP_VERSION_FOR_OTA2_MAJOR=1 APP_VERSION_FOR_OTA2_MINOR=6
 */

#if defined(OTA2_SUPPORT)
#ifndef APP_VERSION_FOR_OTA2_MAJOR
#define APP_VERSION_FOR_OTA2_MAJOR  0
#endif
#ifndef APP_VERSION_FOR_OTA2_MINOR
#define APP_VERSION_FOR_OTA2_MINOR  0
#endif
#endif

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

#ifdef APOLLO_NO_BT
typedef struct
{
    int placeholder;
} apollo_bt_dummy_t;

typedef struct
{
    apollo_bt_dummy_t bt_hash_table;
    apollo_bt_dummy_t bt_paired_device_info[1];
    apollo_bt_dummy_t bt_local_id_keys[1];
} apollo_bt_dct_t;
#endif

typedef struct
{
    uint8_t             ssid_length;                            /**< SSID length                                        */
    uint8_t             ssid_name[ SSID_NAME_SIZE ];            /**< SSID name (AP name)                                */
    wiced_bss_type_t    bss_type;                               /**< Network type                                       */
    wiced_security_t    security;                               /**< Security type                                      */
    uint8_t             security_key_length;                    /**< Length of passphrase/Key                           */
    char                security_key[ SECURITY_KEY_SIZE ];      /**< Passphrase/Key                                     */
    uint8_t             channel;                                /**< Radio channel that the AP beacon was received on   */
    wiced_802_11_band_t band;                                   /**< Radio band                                         */
    int                 use_external_ap;                        /**< Source is NOT an AP (use external AP)              */
} apollo_rmc_ap_info_t;

typedef struct
{
    int                             is_configured;
    apollo_role_t                   apollo_role;
    char                            speaker_name[APOLLO_SPEAKER_NAME_LENGTH];
    APOLLO_CHANNEL_MAP_T            speaker_channel;
    WICED_LOG_LEVEL_T               log_level;
    int                             buffering_ms;
    uint32_t                        threshold_ns;
    int                             auto_start;
    int                             slc_send_duplicates;
    int                             clock_enable;   /* enable AS clock */
    int                             pll_tuning_enable;
    int                             pll_tuning_ppm_max;
    int                             pll_tuning_ppm_min;
    int                             volume;
    int                             payload_size;
    int                             burst_length;
    int                             shuffle_length;
    int                             source_type;
    wiced_ip_address_t              clientaddr;     /* Client IP address when sending audio */
    int                             rtp_port;

    platform_audio_device_id_t      audio_device_rx;     /* Audio capture device  */
    platform_audio_device_id_t      audio_device_tx;     /* Audio playback device */
    platform_audio_sample_rates_t   input_sample_rate;
    platform_audio_sample_sizes_t   input_sample_size;
    uint8_t                         input_channel_count;

    apollo_rmc_ap_info_t            rmc_info;      /* AP info used for RMC network */

#if defined(OTA2_SUPPORT)
    uint8_t         ota2_stop_playback_to_update;   /* set to 1 to stop music playback to get update */
    uint8_t         ota2_reboot_after_download;     /* automatic reboot after successful download */
    uint16_t        ota2_major_version;             /* define APP_VERSION_FOR_OTA2_MAJOR as make arg */
    uint16_t        ota2_minor_version;             /* define APP_VERSION_FOR_OTA2_MINOR as make arg */
    char            ota2_default_update_uri[WICED_OTA2_HTTP_QUERY_SIZE];    /* default Web address of update package */
#endif

} apollo_dct_t;

typedef struct
{
    apollo_dct_t    apollo_dct;
    apollo_bt_dct_t apollo_bt_dct;
} app_dct_t;

#ifdef __cplusplus
} /* extern "C" */
#endif
