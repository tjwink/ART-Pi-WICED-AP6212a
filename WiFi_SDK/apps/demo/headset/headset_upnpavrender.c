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
#include <malloc.h>

#include "wiced.h"
#include "headset_dct.h"
#include "headset_upnpavrender.h"

#include "wiced_log.h"
#include "audio_client.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define DEFAULT_LISTENING_PORT                  ( 49494 )
#define DEFAULT_UUID                            "37ddf93a-6644-4fe3-953c-5feccfc72990"
#define DEFAULT_FRIENDLY_NAME                   "MyUPnPAVRenderer"

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
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
static wiced_result_t headset_upnpavrender_event_cb(void* user_context, upnpavrender_event_t event, upnpavrender_event_data_t *event_data);
static wiced_result_t headset_upnpavrender_audio_client_event_cb(audio_client_ref handle, void* userdata, AUDIO_CLIENT_EVENT_T event, void* arg);

/*************************************************************
 * Event callback for upnpavrender
 *
 */
static wiced_result_t headset_upnpavrender_event_cb(void* user_context, upnpavrender_event_t event, upnpavrender_event_data_t *event_data)
{
    wiced_result_t result = WICED_SUCCESS;
    headset_upnpavrender_context_t* upnpavrender_context = NULL;
    wiced_interface_t wlan_interface = WICED_STA_INTERFACE;
    powersave_dct_t pwrsave_params = {0};
    extern unsigned char _heap[];
    extern unsigned char _eheap[];
    extern unsigned char* sbrk_heap_top;
    volatile struct mallinfo mi = mallinfo();

    if (user_context == NULL)
    {
        return WICED_ERROR;
    }

    /* Get an userdata; WLAN interface and WLAN powersave mode */
    upnpavrender_context = (headset_upnpavrender_context_t *)user_context;
    wlan_interface = upnpavrender_context->audio_userdata.wlan_interface;
    memcpy(&pwrsave_params, &upnpavrender_context->audio_userdata.powersave_params, sizeof(powersave_dct_t));

    switch (event)
    {
        case UPNPAVRENDER_EVENT_PLAY:
            /* Disable WLAN Powersave, it will be controlled by http watermark */
            if (pwrsave_params.wlan_pm_mode != NO_POWERSAVE_MODE)
            {
                result = wiced_wifi_disable_powersave_interface(wlan_interface);
            }

            if (result != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to disable wlan powersave %d\n", result);
            }

            /* Disable MCU Powersave while playing the audio */
            if (platform_mcu_powersave_is_permitted() == WICED_TRUE)
            {
                wiced_platform_mcu_disable_powersave();
            }

            /* Sometimes the audio decoder can't work because of low memory; do debug with this log */
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0,
                          "\n\n================Memory Usage=================\n"
                          "Heap Total                   : %7lu\n"
                          "Heap Free                    : %7lu\n"
                          "Malloc Allocated             : %7d\n"
                          "Malloc Free                  : %7d\n"
                          "Total Free                   : %7lu\n"
                          "=============================================\n\n",
                          (uint32_t)_eheap - (uint32_t)_heap,
                          (uint32_t)_eheap - (uint32_t)sbrk_heap_top,
                          mi.uordblks, mi.fordblks,
                          mi.fordblks + (uint32_t)_eheap - (uint32_t)sbrk_heap_top);
            break;
        case UPNPAVRENDER_EVENT_PAUSE:
        case UPNPAVRENDER_EVENT_STOP:
            /* Enable WLAN Powersave if we're not in play state */
            if (pwrsave_params.wlan_pm_mode == PM1_POWERSAVE_MODE)
            {
                result = wiced_wifi_enable_powersave_interface(wlan_interface);
            }
            else if (pwrsave_params.wlan_pm_mode == PM2_POWERSAVE_MODE)
            {
                result = wiced_wifi_enable_powersave_with_throughput_interface(pwrsave_params.wlan_pm2_ret_on_high, wlan_interface);
            }

            if (result != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to enable wlan powersave %d\n", result);
            }

            /* Enable MCU Powersave if we're not in play state */
            if (platform_mcu_powersave_is_permitted() == WICED_FALSE)
            {
                wiced_platform_mcu_enable_powersave();
            }
            break;
        case UPNPAVRENDER_EVENT_RESUME:
            break;
        case UPNPAVRENDER_EVENT_SEEK:
            break;
        case UPNPAVRENDER_EVENT_PLAYBACK_PROGRESS:
            break;
        case UPNPAVRENDER_EVENT_MUTE:
            break;
        case UPNPAVRENDER_EVENT_VOLUME:
            break;
        default:
            break;
    }

    return result;
}

/*************************************************************
 * Event callback for audio client
 *
 * Main purpose of this function is to save WLAN power with watermark feature
 */
static wiced_result_t headset_upnpavrender_audio_client_event_cb( audio_client_ref handle, void* userdata, AUDIO_CLIENT_EVENT_T event, void* arg )
{
    wiced_result_t result = WICED_SUCCESS;
    headset_upnpavrender_context_t* upnpavrender_context = NULL;
    wiced_interface_t wlan_interface = WICED_STA_INTERFACE;
    powersave_dct_t pwrsave_params = {0};
    wiced_bool_t mcu_clock_freq_for_mp3 = WICED_FALSE;
    AUDIO_CLIENT_CODEC_T audio_codec = AUDIO_CLIENT_CODEC_NULL;

    UNUSED_PARAMETER(handle);

    if (userdata == NULL)
    {
        return WICED_ERROR;
    }

    /* Get userdata */
    upnpavrender_context = (headset_upnpavrender_context_t *)userdata;
    wlan_interface = upnpavrender_context->audio_userdata.wlan_interface;
    memcpy(&pwrsave_params, &upnpavrender_context->audio_userdata.powersave_params, sizeof(powersave_dct_t));
    mcu_clock_freq_for_mp3 = upnpavrender_context->audio_userdata.mcu_clock_freq_for_mp3;

    switch(event)
    {
        /* Complete to find the audio codec */
        case AUDIO_CLIENT_EVENT_AUDIO_CODEC:
            /* Get audio codec */
            audio_codec = (AUDIO_CLIENT_CODEC_T) arg;

            /* Re tune the MCU clock frequecy for MP3 codec; it needs 320MHz to decode */
            if (audio_codec == AUDIO_CLIENT_CODEC_MP3)
            {
                if (pwrsave_params.mcu_clock_freq != PLATFORM_CPU_CLOCK_FREQUENCY_320_MHZ &&
                    mcu_clock_freq_for_mp3 == WICED_FALSE)
                {
                    platform_tick_stop();
                    platform_cpu_clock_init(PLATFORM_CPU_CLOCK_FREQUENCY_320_MHZ);
                    platform_tick_start();
                    upnpavrender_context->audio_userdata.mcu_clock_freq_for_mp3 = WICED_TRUE;
                }
            }
            else
            {
                if (pwrsave_params.mcu_clock_freq != PLATFORM_CPU_CLOCK_FREQUENCY_320_MHZ &&
                    mcu_clock_freq_for_mp3 == WICED_TRUE)
                {
                    platform_tick_stop();
                    platform_cpu_clock_init(pwrsave_params.mcu_clock_freq);
                    platform_tick_start();
                    upnpavrender_context->audio_userdata.mcu_clock_freq_for_mp3 = WICED_FALSE;
                }
            }
            break;
        /* Hits high watermark
         * means there are many http buffers to be processed so we can enter to WLAN powersave mode */
        case AUDIO_CLIENT_EVENT_DATA_THRESHOLD_HIGH:
            if (pwrsave_params.wlan_pm_mode == PM1_POWERSAVE_MODE)
            {
                result = wiced_wifi_enable_powersave_interface(wlan_interface);
            }
            else if (pwrsave_params.wlan_pm_mode == PM2_POWERSAVE_MODE)
            {
                result = wiced_wifi_enable_powersave_with_throughput_interface(pwrsave_params.wlan_pm2_ret_on_high, wlan_interface);
            }
            break;

        /* Hits low watermark
         * means we don't have enough http buffers so should get out from WLAN powrsave mode */
        case AUDIO_CLIENT_EVENT_DATA_THRESHOLD_LOW:
            if (pwrsave_params.wlan_pm_mode != NO_POWERSAVE_MODE)
            {
                result = wiced_wifi_disable_powersave_interface(wlan_interface);
            }
            break;

        /* TODO: Need a process the other audio events here? See other event types in AUDIO_CLIENT_EVENT_T */
        case AUDIO_CLIENT_EVENT_ERROR:
            wiced_log_msg( WLF_AUDIO, WICED_LOG_ERR, "Error in audio client event\n");
            break;
        default:
            break;
    }

    return result;
}

/*************************************************************
 * Init UPnP AV renderer parameters
 *
 */
wiced_result_t headset_upnpavrender_init( headset_upnpavrender_context_t** _upnpavrender_context, audio_dct_t* audio_params, powersave_dct_t* pwrsave_params, wiced_interface_t interface )
{
    wiced_result_t result = WICED_SUCCESS;
    headset_upnpavrender_context_t* upnpavrender_context = NULL;

    /* Init main context and userdata for audio client */
    upnpavrender_context = calloc_named("headset_upnpavrender", 1, sizeof(headset_upnpavrender_context_t));

    if (upnpavrender_context == NULL ||
        audio_params == NULL ||
        pwrsave_params == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to init upnpavrender context\n");
        result = WICED_ERROR;
        return result;
    }
    else
    {
        upnpavrender_service_params_t upnpavrender_params =
        {
            /* Renderer parameters */
            .interface      = interface,
            .listening_port = DEFAULT_LISTENING_PORT,
            .uuid           = DEFAULT_UUID,
            .friendly_name  = DEFAULT_FRIENDLY_NAME,
            .event_cb       = headset_upnpavrender_event_cb,
            .user_context   = (void *)upnpavrender_context,

            /* Audio Client parameters */
            .audio_client_params =
            {
                .interface                   = interface,
                .event_cb                    = headset_upnpavrender_audio_client_event_cb,
                .device_id                   = PLATFORM_DEFAULT_AUDIO_OUTPUT,
                .enable_playback             = WICED_TRUE,
                .release_device_on_stop      = WICED_FALSE,
                .disable_hls_streaming       = WICED_FALSE,
                .hls_max_entry_count         = 0,
                .userdata                    = (void *)upnpavrender_context,

                /* Tunable parameters by DCT */
                .data_buffer_num             = audio_params->http_buffer_num,
                .data_buffer_preroll         = audio_params->http_threshold_high,
                .audio_buffer_num            = audio_params->audio_buffer_num,
                .audio_buffer_size           = audio_params->audio_buffer_size,
                .audio_period_size           = audio_params->audio_period_size,
                .data_threshold_high         = audio_params->http_threshold_high,
                .data_threshold_low          = audio_params->http_threshold_low,
                .high_threshold_read_inhibit = audio_params->http_read_inhibit ? WICED_TRUE : WICED_FALSE,
                .volume                      = audio_params->volume,
            },
        };

        /* don't need the read inhibit feature if we don't use the wlan powersave feature */
        if (pwrsave_params->wlan_pm_mode == NO_POWERSAVE_MODE)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Disable the http_read_inhibit feature because we don't use the wlan powersave\n");
            upnpavrender_params.audio_client_params.high_threshold_read_inhibit = WICED_FALSE;
        }

        /* Copy the parameters into upnpavrender context */
        memcpy(&upnpavrender_context->service_params, &upnpavrender_params, sizeof(upnpavrender_service_params_t));

        /* Init user data */
        upnpavrender_context->audio_userdata.wlan_interface = interface;
        memcpy(&upnpavrender_context->audio_userdata.powersave_params, pwrsave_params, sizeof(powersave_dct_t));
        upnpavrender_context->audio_userdata.mcu_clock_freq_for_mp3 = WICED_FALSE;

        /* Save the upnpavrender context */
        *_upnpavrender_context = upnpavrender_context;
    }

    return result;
}

/*************************************************************
 * Deinit UPnP AV renderer handle
 *
 */
wiced_result_t headset_upnpavrender_deinit( headset_upnpavrender_context_t* upnpavrender_context )
{
    wiced_result_t result = WICED_SUCCESS;

    if (upnpavrender_context == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No upnpavrender handle! Failed to deinit UPnP AV renderer\n");
        result = WICED_ERROR;
        return result;
    }

    headset_upnpavrender_stop(upnpavrender_context);

    if (upnpavrender_context != NULL)
    {
        free(upnpavrender_context);
        upnpavrender_context = NULL;
    }

    return result;
}

/*************************************************************
 * To start the UPnP AV renderer
 *
 */
wiced_result_t headset_upnpavrender_start( headset_upnpavrender_context_t* upnpavrender_context )
{
    wiced_result_t result = WICED_ERROR;

    if (upnpavrender_context == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No upnpavrender handle! Failed to stop UPnP AV renderer\n");
        return WICED_ERROR;
    }

    result = upnpavrender_service_start( &upnpavrender_context->service_params, &upnpavrender_context->service_handle );

    if (result == WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "UPnP AV renderer is started\n");
    }
    else
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to start UPnP AV renderer\n");
    }

    return result;
}

/*************************************************************
 * To stop the UPnP AV renderer
 *
 */
wiced_result_t headset_upnpavrender_stop( headset_upnpavrender_context_t* upnpavrender_context )
{
    wiced_result_t result = WICED_ERROR;

    if (upnpavrender_context == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "No upnpavrender handle! Failed to stop UPnP AV renderer\n");
        return WICED_ERROR;
    }

    result = upnpavrender_service_stop( upnpavrender_context->service_handle );

    if (result == WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "UPnP AV renderer is stopped\n");
    }
    else
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Failed to stop UPnP AV renderer\n");
    }

    return result;
}
