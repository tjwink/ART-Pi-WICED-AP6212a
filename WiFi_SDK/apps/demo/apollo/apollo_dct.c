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
 */

#include "wiced_framework.h"
#include "platform_audio.h"
#include "apollo_dct.h"
#include "apollo_rtp_params.h"
#include "apollo_streamer.h"

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

/******************************************************
 *               Variables Definitions
 ******************************************************/

DEFINE_APP_DCT(app_dct_t)
{
    .apollo_dct =
    {
        .is_configured       = 1,
        .apollo_role         = APOLLO_ROLE_DEFAULT,
        .speaker_name        = "Apollo",
        .speaker_channel     = CHANNEL_MAP_FR,
        .log_level           = WICED_LOG_ERR,
        .buffering_ms        = APOLLO_BUFFERING_MS_DEFAULT,
        .threshold_ns        = APOLLO_THRESHOLD_NS_DEFAULT,
        .auto_start          = 1,
        .slc_send_duplicates = 0,
        .clock_enable        = 1,  /* 0 = blind push, 1 = use AS clock */
        .pll_tuning_enable   = 1,
        .pll_tuning_ppm_max  = APOLLO_PLL_TUNING_PPM_MAX_DEFAULT,
        .pll_tuning_ppm_min  = APOLLO_PLL_TUNING_PPM_MIN_DEFAULT,
        .volume              = APOLLO_VOLUME_DEFAULT,
        .payload_size        = RTP_PACKET_MAX_DATA,
        .burst_length        = APOLLO_BURST_LENGTH_DEFAULT,
        .shuffle_length      = 0,
        .source_type         = APOLLO_AUDIO_SOURCE_DEFAULT,
        .clientaddr          = { WICED_IPV4, { .v4 = (APOLLO_MULTICAST_IPV4_ADDRESS_DEFAULT) } },
        .rtp_port            = RTP_DEFAULT_PORT,
        .audio_device_rx     = PLATFORM_DEFAULT_AUDIO_INPUT,
        .audio_device_tx     = PLATFORM_DEFAULT_AUDIO_OUTPUT,
        .input_sample_rate   = APOLLO_INPUT_SAMPLE_RATE_DEFAULT,
        .input_sample_size   = APOLLO_INPUT_SAMPLE_SIZE_DEFAULT,
        .input_channel_count = APOLLO_INPUT_CHANNEL_COUNT,

#if defined(OTA2_SUPPORT)
        .ota2_stop_playback_to_update   = 1,
        .ota2_reboot_after_download     = 1,
        .ota2_major_version             = APP_VERSION_FOR_OTA2_MAJOR,
        .ota2_minor_version             = APP_VERSION_FOR_OTA2_MINOR,
        .ota2_default_update_uri        = "192.168.1.100/ota2_firmware_updates/OTA2_image_file.bin",
#endif
        .rmc_info =
        {
            .ssid_length            = sizeof(APOLLO_RMC_SSID) - 1,
            .ssid_name              = { APOLLO_RMC_SSID },
            .bss_type               = APOLLO_RMC_BSS_TYPE,
            .security               = APOLLO_RMC_SECURITY,
            .security_key_length    = sizeof(APOLLO_RMC_PASSPHRASE) - 1,
            .security_key           = { APOLLO_RMC_PASSPHRASE },
            .channel                = APOLLO_RMC_CHANNEL,
            .band                   = APOLLO_RMC_BAND,
            .use_external_ap        = WICED_FALSE,
        },
    },
    .apollo_bt_dct =
    {
        .bt_hash_table              = {0},
        .bt_paired_device_info[0]   = {0},
        .bt_local_id_keys           = {{0}}
    }
};

/******************************************************
 *               Function Definitions
 ******************************************************/
