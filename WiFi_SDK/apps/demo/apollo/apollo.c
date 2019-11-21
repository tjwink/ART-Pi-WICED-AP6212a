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

/** @file Apollo audio application.
 */

#include <unistd.h>
#include <malloc.h>

#include "apollo_context.h"
#include "platform_audio.h"
#include "dct/command_console_dct.h"
#include "apollo_debug.h"
#include "apollo_report.h"
#include "apollo_keypad.h"

#ifdef USE_UPNPAV
#include "apollo_upnpavrender.h"
#endif

#ifdef USE_AUDIO_DISPLAY
#include "audio_display.h"
#endif

#ifndef APOLLO_NO_BT
#include "apollo_bt_service.h"
#include "apollo_config_gatt_server.h"
#endif

#if defined(OTA2_SUPPORT)
#include "apollo_ota2_support.h"
#endif

#if defined(USE_AMBILIGHT)
#include "amblt_source_types.h"
#include "amblt_source.h"

#endif

/******************************************************
 *                      Macros
 ******************************************************/

#define APOLLO_STREAM_VOL_ATT_STEP (5)

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

static wiced_result_t apollo_start_network_services(apollo_app_t* apollo);
static wiced_result_t apollo_stop_network_services(apollo_app_t* apollo);

static wiced_result_t pno_app_result_callback(wiced_scan_handler_result_t* malloced_scan_result);

/******************************************************
 *               Variables Definitions
 ******************************************************/

static wiced_time_t apollo_start_time;

static const wiced_ip_setting_t ap_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS( 255,255,255,  0 ) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
};

static const wiced_ip_setting_t sink_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS( 192,168,  0,  2 ) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS( 255,255,255,  0 ) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS( 192,168,  0,  1 ) ),
};

apollo_app_t *g_apollo;

/******************************************************
 *               Function Definitions
 ******************************************************/

static void apollo_ethernet_packet_filter(wiced_buffer_t buffer, void* userdata)
{
    apollo_app_t* apollo = (apollo_app_t*)userdata;

    (void)buffer;

    if (apollo != NULL && apollo->tag == APOLLO_TAG_VALID)
    {
        wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_PTP_PACKET);
    }
}


static void apollo_link_up(void)
{
    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "STA Link Up\n");
}

static void apollo_link_down(void)
{
    apollo_app_t *apollo = g_apollo;

    if (apollo == NULL || (apollo->tag != APOLLO_TAG_VALID && !apollo->initializing))
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: Bad app structure\r\n", __FUNCTION__);
        return ;
    }

    /* If the STA interface isn't being used directly by RMC/apollo and the link goes down,
     * then start PNO to wait for the AP/SSID, re-establish link, and send us an event.
     */
    if (apollo->interface == WICED_AP_INTERFACE)
    {
        wiced_result_t result;
        wiced_ssid_t ssid;
        wiced_security_t security;

        bzero(&ssid, sizeof(ssid));
        ssid.length = apollo->dct_tables.dct_wifi->stored_ap_list[0].details.SSID.length;
        memcpy( ssid.value, apollo->dct_tables.dct_wifi->stored_ap_list[0].details.SSID.value, ssid.length);
        security = apollo->dct_tables.dct_wifi->stored_ap_list[0].details.security;

        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "STA Link Down, Start PNO with SSID %s\n", ssid.value);

        result = wiced_wifi_pno_start( &ssid, security, pno_app_result_callback, NULL );
        if ( result != WICED_SUCCESS ) {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to start PNO, error %d\n", result);
        }
    } else {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "STA Link Down\n");
    }
}

static wiced_result_t pno_app_result_callback( wiced_scan_handler_result_t* malloced_scan_result )
{
    wiced_result_t ret;

    /* must be done prior to free below -- this callback function owns the memory */
    malloc_transfer_to_curr_thread(malloced_scan_result);

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "PNO callback\n");

    /* when network is detected, force a reassoc via wiced_network_up */
    if ( wiced_network_is_up(WICED_STA_INTERFACE) == WICED_FALSE) {
        ret = wiced_network_up(WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL);

        if (ret == WICED_SUCCESS) {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "%s: Re-join succeeded; clearing pno.\n", __FUNCTION__);
        } else {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: Re-join failed. PNO still running.\n", __FUNCTION__);
        }
    }

    if (wiced_network_is_up( WICED_STA_INTERFACE) == WICED_TRUE) {
        /* PNO has detected a registered network, disable it now.
        * Note: this also clears event registration
        */
        ret = wiced_wifi_pno_stop();
        if (ret != WICED_SUCCESS) {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: Can't disable PNO %d\n", __FUNCTION__, ret);
        }
    }

    free( malloced_scan_result );

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Exit PNO event\n");
    return WICED_SUCCESS;
}


static wiced_interface_t apollo_get_rmc_interface(apollo_dct_collection_t* dct_tables)
{
    wiced_interface_t interface;

    interface = WICED_STA_INTERFACE;

    if (!wwd_wifi_is_mesh_enabled()) {
        if ((dct_tables != NULL) && (dct_tables->dct_app != NULL))
        {
            if ( (dct_tables->dct_app->rmc_info.bss_type == WICED_BSS_TYPE_INFRASTRUCTURE) &&
                 (dct_tables->dct_app->apollo_role == APOLLO_ROLE_SOURCE) &&
                 (dct_tables->dct_app->rmc_info.use_external_ap == 0))
            {
                interface = WICED_AP_INTERFACE;
            }
        }
    }

    return interface;
}

static void network_timer_callback(void *arg)
{
    apollo_app_t* apollo = (apollo_app_t *)arg;

    if (apollo && apollo->tag == APOLLO_TAG_VALID)
    {
        wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_NETWORK_TIMER);
    }
}

static void timer_callback(void *arg)
{
    apollo_app_t* apollo = (apollo_app_t *)arg;

    if (apollo && apollo->tag == APOLLO_TAG_VALID)
    {
        wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_TIMER);
    }
}

static wiced_result_t csa_receive_callback( wiced_udp_socket_t* socket, void *arg )
{
    apollo_app_t* apollo = (apollo_app_t *)arg;

    if (apollo && apollo->tag == APOLLO_TAG_VALID)
    {
        wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_RX_CSA);
    }

    return WICED_SUCCESS;
}


int apollo_log_output_handler(WICED_LOG_LEVEL_T level, char *logmsg)
{
    write(STDOUT_FILENO, logmsg, strlen(logmsg));

    return 0;
}


static wiced_result_t apollo_log_get_time(wiced_time_t* time)
{
    wiced_8021as_time_t as_time;
    wiced_time_t now;
    wiced_result_t result;

    /*
     * Get the current time.
     */

    /*
     * wiced_time_read_8021as() requires a valid audio sample rate in order to get a valid audio time;
     * however, if audio time is not required, audio sample rate MUST be set to 0
     */
    as_time.audio_sample_rate = 0;
    result = wiced_time_read_8021as(&as_time);

    if (result == WICED_SUCCESS)
    {
        *time = (as_time.master_secs * MILLISECONDS_PER_SECOND) + (as_time.master_nanosecs / NANOSECONDS_PER_MILLISECOND);
    }
    else
    {
        result = wiced_time_get_time(&now);
        *time  = now - apollo_start_time;
    }

    return result;
}


static int apollo_player_event_callback(apollo_player_ref handle, void* userdata, APOLLO_PLAYER_EVENT_T event, void* arg)
{
    apollo_app_t* apollo = (apollo_app_t*)userdata;
    const platform_audio_device_info_t* audio_device;
    apollo_audio_format_t* format;
    apollo_seq_err_t* seq_err;
    apollo_player_stats_t* stats;
    uint32_t num_lost;

    if (apollo == NULL || apollo->tag != APOLLO_TAG_VALID)
    {
        return -1;
    }

    switch (event)
    {
        case APOLLO_PLAYER_EVENT_PLAYBACK_STARTED:
            format = (apollo_audio_format_t*)arg;
            apollo->playback_active = WICED_TRUE;
            audio_device = platform_audio_device_get_info_by_id(apollo->dct_tables.dct_app->audio_device_tx);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Playback started using device %s\r\n", audio_device ? audio_device->device_name : "");
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio format is %u channels, %lu kHz, %u bps\r\n",
                          format->num_channels, format->sample_rate, format->bits_per_sample);
            if (wiced_rtos_is_timer_running(&apollo->timer) != WICED_SUCCESS)
            {
                wiced_rtos_start_timer(&apollo->timer);
            }
            break;

        case APOLLO_PLAYER_EVENT_PLAYBACK_STOPPED:
            stats = (apollo_player_stats_t*)arg;
            apollo->playback_active = WICED_FALSE;
            wiced_time_get_time(&apollo->playback_ended);

            /* Get non-persistent player stat(s) to update persistent Apollo stat(s) */
            apollo->total_rtp_packets_received         += stats->rtp_packets_received;
            apollo->total_rtp_packets_dropped          += stats->rtp_packets_dropped;
            apollo->total_audio_frames_played          += stats->audio_frames_played;
            apollo->total_audio_frames_dropped         += stats->audio_frames_dropped;
            apollo->total_audio_frames_inserted        += stats->audio_frames_inserted;
            apollo->total_audio_underruns              += stats->audio_underruns;
            apollo->total_audio_concealment_slcplc_cnt += stats->audio_concealment_slcplc_cnt;
            apollo->total_audio_concealment_fec_cnt    += stats->audio_concealment_fec_cnt;
            wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_AUTOSTOP);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "%lu bytes received in %lu packets\r\n", (uint32_t)stats->total_bytes_received, stats->rtp_packets_received);
            break;

        case APOLLO_PLAYER_EVENT_SEQ_ERROR:
            seq_err = (apollo_seq_err_t*)arg;
            if (seq_err->cur_seq > seq_err->last_valid_seq)
            {
                num_lost = seq_err->cur_seq - seq_err->last_valid_seq - 1;
            }
            else
            {
                num_lost = seq_err->cur_seq + (RTP_MAX_SEQ_NUM + 1) - seq_err->last_valid_seq - 1;
            }
            wiced_log_msg(WLF_AUDIO, WICED_LOG_WARNING, "SEQ error - cur %u, last %u (lost %u)\r\n", seq_err->cur_seq, seq_err->last_valid_seq, num_lost);

#ifdef USE_AUDIO_DISPLAY
            apollo_player_ioctl(apollo->player_handle, APOLLO_PLAYER_IOCTL_GET_STATS, &apollo->player_stats);
            if (audio_display_get_footer_options() & FOOTER_OPTION_APOLLO_RX)
            {
                audio_display_footer_update_time_info(apollo->player_stats.rtp_packets_received, apollo->player_stats.rtp_packets_dropped);
            }
#endif
            break;

        case APOLLO_PLAYER_EVENT_RTP_TIMING_FULL:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "RTP timing log is full\n");
            break;
    }

    return 0;
}

static int apollo_streamer_event_notification(apollo_streamer_ref handle, void* user_context, apollo_streamer_event_t event, apollo_streamer_event_data_t* event_data)
{
    int rc = 0;
    apollo_app_t* apollo = (apollo_app_t*)user_context;
    wiced_action_jump_when_not_true((apollo != NULL) && (apollo->tag == APOLLO_TAG_VALID), _exit, rc = -1);

    switch (event)
    {
        case APOLLO_STREAMER_EVENT_CONNECTED:
#ifdef USE_AUDIO_DISPLAY
            audio_display_header_update_options(BATTERY_ICON_IS_VISIBLE | BATTERY_ICON_SHOW_PERCENT | SIGNAL_STRENGTH_IS_VISIBLE | BLUETOOTH_IS_CONNECTED);
#endif
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "[%s] device is connected.\n", apollo_source_type_get_text(apollo->streamer_params.source_type));
            break;

        case APOLLO_STREAMER_EVENT_DISCONNECTED:
#ifdef USE_AUDIO_DISPLAY
            audio_display_header_update_options(BATTERY_ICON_IS_VISIBLE | BATTERY_ICON_SHOW_PERCENT | SIGNAL_STRENGTH_IS_VISIBLE);
#endif
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "[%s] device is disconnected.\n", apollo_source_type_get_text(apollo->streamer_params.source_type));
            break;

        case APOLLO_STREAMER_EVENT_PLAYBACK_STARTED:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "[%s] playback has started.\n", apollo_source_type_get_text(apollo->streamer_params.source_type));
            break;

        case APOLLO_STREAMER_EVENT_PLAYBACK_STOPPED:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "[%s] playback has stopped.\n", apollo_source_type_get_text(apollo->streamer_params.source_type));
            break;

        case APOLLO_STREAMER_EVENT_VOLUME_CHANGED:
            break;

        case APOLLO_STREAMER_EVENT_PLAYBACK_STATUS:
        {
            uint32_t position = event_data->playback.position_msecs/1000;
            uint32_t duration = event_data->playback.duration_msecs/1000;

#ifdef USE_AUDIO_DISPLAY
            audio_display_footer_update_time_info(position, duration);
#endif
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG2, "%s track: %lu / %lu secs\n",
                          apollo_source_type_get_text(apollo->streamer_params.source_type), position, duration);
        }
            break;

        case APOLLO_STREAMER_EVENT_TRACK_METADATA:
#ifdef USE_AUDIO_DISPLAY
            audio_display_footer_update_song_info((char *)event_data->metadata.title_utf8_str, (char *)event_data->metadata.artist_utf8_str);
#endif
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "[%s] track: [%s - %s]\n",
                          apollo_source_type_get_text(apollo->streamer_params.source_type),
                          event_data->metadata.artist_utf8_str, event_data->metadata.title_utf8_str);

            if (apollo->cmd_sender_handle != NULL)
            {
                apollo_cmd_metadata_t data_buffer;

                memset(&data_buffer, 0, sizeof(apollo_cmd_metadata_t));

                data_buffer.type                                      = APOLLO_METADATA_TITLE | APOLLO_METADATA_ARTIST;
                data_buffer.data_length[APOLLO_METADATA_TITLE_INDEX]  = 1 + strlen((char *)event_data->metadata.title_utf8_str);
                data_buffer.data[APOLLO_METADATA_TITLE_INDEX]         = (uint8_t *)event_data->metadata.title_utf8_str;
                data_buffer.data_length[APOLLO_METADATA_ARTIST_INDEX] = 1 + strlen((char *)event_data->metadata.artist_utf8_str);
                data_buffer.data[APOLLO_METADATA_ARTIST_INDEX]        = (uint8_t *)event_data->metadata.artist_utf8_str;

                apollo_cmd_sender_command(g_apollo->cmd_sender_handle, APOLLO_CMD_SENDER_COMMAND_METADATA, WICED_TRUE, NULL, &data_buffer);
            }
            break;

        case APOLLO_STREAMER_EVENT_BUFFER_RELEASED:
#ifdef USE_UPNPAV
            apollo_upnpavrender_release_buffer(apollo, event_data);
#endif
            break;

        default:
            break;
    }

 _exit:
    return rc;
}

#ifndef APOLLO_NO_BT
wiced_result_t gatt_event_callback(apollo_config_gatt_event_t event, apollo_config_gatt_server_dct_t *dct,  void *user_context)
{
    size_t         len;
    wiced_result_t result = WICED_SUCCESS;
    apollo_app_t*  apollo = (apollo_app_t *)user_context;

    switch ( event)
    {
        case APOLLO_CONFIG_GATT_EVENT_DCT_READ:
            dct->is_configured   = 1;
            dct->mode            = apollo->dct_tables.dct_app->apollo_role;
            dct->spk_channel_map = apollo->dct_tables.dct_app->speaker_channel;
            dct->spk_vol         = apollo->dct_tables.dct_app->volume;
            dct->src_type        = apollo->dct_tables.dct_app->source_type;
            dct->security        = apollo->dct_tables.dct_app->rmc_info.security;
            strlcpy(dct->nw_ssid_name, (char *)apollo->dct_tables.dct_app->rmc_info.ssid_name, sizeof(dct->nw_ssid_name));
            strlcpy(dct->nw_pass_phrase, (char *)apollo->dct_tables.dct_app->rmc_info.security_key, sizeof(dct->nw_pass_phrase));
            break;

        case APOLLO_CONFIG_GATT_EVENT_DCT_WRITE:
            apollo->dct_tables.dct_app->is_configured       = dct->is_configured;
            apollo->dct_tables.dct_app->apollo_role         = dct->mode;
            apollo->dct_tables.dct_app->speaker_channel     = dct->spk_channel_map;
            apollo->dct_tables.dct_app->volume              = dct->spk_vol;
            apollo->dct_tables.dct_app->source_type         = dct->src_type;
            apollo->dct_tables.dct_app->rmc_info.security   = dct->security;

            len = strlen(dct->nw_ssid_name);
            len = MIN(len, sizeof(dct->nw_ssid_name));
            len = MIN(len, sizeof(apollo->dct_tables.dct_app->rmc_info.ssid_name));
            memcpy(apollo->dct_tables.dct_app->rmc_info.ssid_name, dct->nw_ssid_name, len);
            apollo->dct_tables.dct_app->rmc_info.ssid_length = len;

            len = strlen(dct->nw_pass_phrase);
            len = MIN(len, sizeof(dct->nw_pass_phrase));
            len = MIN(len, sizeof(apollo->dct_tables.dct_app->rmc_info.security_key));
            memcpy(apollo->dct_tables.dct_app->rmc_info.security_key, dct->nw_pass_phrase, len);
            apollo->dct_tables.dct_app->rmc_info.security_key_length = len;

            if(apollo->dct_tables.dct_app->apollo_role == APOLLO_ROLE_SINK)
            {
                strlcpy((char *)apollo->dct_tables.dct_app->speaker_name,
                    dct->spk_name, sizeof(apollo->dct_tables.dct_app->speaker_name));
            }
#ifdef WICED_DCT_INCLUDE_BT_CONFIG
            if(apollo->dct_tables.dct_app->apollo_role == APOLLO_ROLE_SOURCE)
            {
                strlcpy((char*)apollo->dct_tables.dct_bt->bluetooth_device_name, dct->nw_ssid_name, sizeof(apollo->dct_tables.dct_bt->bluetooth_device_name));
            }
            else
            {
                strlcpy((char*)apollo->dct_tables.dct_bt->bluetooth_device_name, dct->spk_name, sizeof(apollo->dct_tables.dct_bt->bluetooth_device_name));
            }
#endif
            break;

        case APOLLO_CONFIG_GATT_EVENT_DCT_WRITE_COMPLETED:
            wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_CONFIG_GATT);
            break;

        default:
            result = WICED_ERROR;
            break;
    }

    return result;
}
#endif


static wiced_result_t apollo_stream_start(apollo_app_t* apollo)
{
    wiced_result_t result;
    int num_pool_packets;

    if (apollo->dct_tables.dct_app->burst_length == 0)
    {
        num_pool_packets = 20;
    }
    else if (apollo->dct_tables.dct_app->burst_length != APOLLO_STREAMER_BURST_AUTO_SLC)
    {
        num_pool_packets = apollo->dct_tables.dct_app->burst_length * 2 * 3;
    }
    else
    {
        num_pool_packets = RTP_AUDIO_BL_MAX_LENGTH * 2 * 2;
    }

    /*
     * Set streaming options (most from DCT)
     */

    memset(&apollo->streamer_params, 0, sizeof(apollo->streamer_params));

    apollo->streamer_params.user_context        = apollo;
    apollo->streamer_params.event_cb            = apollo_streamer_event_notification;
    apollo->streamer_params.source_type         = (apollo_audio_source_type_t)apollo->dct_tables.dct_app->source_type;
    apollo->streamer_params.iface               = apollo_get_rmc_interface(&apollo->dct_tables);
    apollo->streamer_params.port                = apollo->dct_tables.dct_app->rtp_port;
    apollo->streamer_params.num_pool_packets    = num_pool_packets;
    apollo->streamer_params.num_packets         = APOLLO_TX_PACKET_BUFFER_COUNT;
    apollo->streamer_params.max_payload_size    = apollo->dct_tables.dct_app->payload_size;
    apollo->streamer_params.burst_length        = apollo->dct_tables.dct_app->burst_length;
    apollo->streamer_params.shuffle_length      = apollo->dct_tables.dct_app->shuffle_length;
    apollo->streamer_params.slc_send_duplicates = apollo->dct_tables.dct_app->slc_send_duplicates;
    apollo->streamer_params.audio_device_rx     = apollo->dct_tables.dct_app->audio_device_rx;
    apollo->streamer_params.audio_device_tx     = apollo->dct_tables.dct_app->audio_device_tx;
    apollo->streamer_params.input_sample_rate   = apollo->dct_tables.dct_app->input_sample_rate;
    apollo->streamer_params.input_sample_size   = apollo->dct_tables.dct_app->input_sample_size;
    apollo->streamer_params.input_channel_count = apollo->dct_tables.dct_app->input_channel_count;

    SET_IPV4_ADDRESS(apollo->streamer_params.clientaddr, GET_IPV4_ADDRESS(apollo->dct_tables.dct_app->clientaddr));

    apollo->streamer_params.volume              = apollo->dct_tables.dct_app->volume;
    apollo->streamer_params.buffer_nodes        = APOLLO_TX_AUDIO_RENDER_BUFFER_NODES;
    apollo->streamer_params.buffer_ms           = apollo->dct_tables.dct_app->buffering_ms;
    apollo->streamer_params.threshold_ns        = apollo->dct_tables.dct_app->threshold_ns;
    apollo->streamer_params.clock_enable        = apollo->dct_tables.dct_app->clock_enable;
    apollo->streamer_params.pll_tuning_enable   = apollo->dct_tables.dct_app->pll_tuning_enable;
    apollo->streamer_params.pll_tuning_ppm_max  = apollo->dct_tables.dct_app->pll_tuning_ppm_max;
    apollo->streamer_params.pll_tuning_ppm_min  = apollo->dct_tables.dct_app->pll_tuning_ppm_min;

#if defined(USE_AMBILIGHT)
    /*
     * ambient light control
     * we are a source side so we initiate : AMBLT_FRONT + AMBLT_SOURCE
     */

    if(apollo->amblt_source == NULL )
    {
        amblt_source_err_t err_source = AMBLT_SOURCE_NO_ERR;
        amblt_front_err_t  err_front  = AMBLT_FRONT_NO_ERR;
        amblt_front_cfg_t  amblt_front_cfg;


        /* new source */
        amblt_source_new(&err_source, &apollo->amblt_source, 0/* zero means no verbose log*/);

        if( AMBLT_SOURCE_NO_ERR == err_source)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Ambilight source create: success.\r\n");
            /* start source */
            amblt_source_start(&err_source, apollo->amblt_source, NULL);
        }

        if( AMBLT_SOURCE_NO_ERR == err_source)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Ambilight source started: success.\r\n");
            /* new frontend */
            amblt_front_new( &err_front, &apollo->amblt_front, AMBLT_MOD_PROTO_CONSOLE, NULL);
        }

        if ( AMBLT_FRONT_NO_ERR == err_front )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Ambilight frontend create: success.\r\n");

            amblt_front_cfg.log_lvl  = 0;
            amblt_front_cfg.source   = apollo->amblt_source;

            /* start frontent */
            amblt_front_start( &err_front, apollo->amblt_front, &amblt_front_cfg);
        }

        if ( AMBLT_FRONT_NO_ERR == err_front )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Ambilight frontend started: success.\r\n");
        }


        if ( (AMBLT_FRONT_NO_ERR  != err_front) ||
             (AMBLT_SOURCE_NO_ERR != err_source) )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to create ambilight (frontend,source) obj.\r\n");
        }
    }
#endif

    /*
     * Fire off the apollo streamer; but wait five seconds for system to queiesce into steady state.
     */

    wiced_rtos_delay_milliseconds(5000);
    result = apollo_streamer_init(&apollo->streamer_params, &apollo->streamer_handle);

    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to initialize apollo_streamer\r\n");
    }

#ifdef USE_UPNPAV
    if (apollo->streamer_params.source_type == APOLLO_AUDIO_SOURCE_EXTERNAL)
    {
        result = apollo_upnpavrender_start(apollo);
        if (result != WICED_SUCCESS)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "apollo_upnpavrender_start() failed !\r\n");
        }

    }
#endif

    return result;
}


static void apollo_stream_stop(apollo_app_t* apollo)
{
    if (apollo == NULL || apollo->tag != APOLLO_TAG_VALID || apollo->streamer_handle == NULL)
    {
        return;
    }

#ifdef USE_UPNPAV
    if (apollo->streamer_params.source_type == APOLLO_AUDIO_SOURCE_EXTERNAL)
    {
        apollo_upnpavrender_stop(apollo);
    }
#endif

    apollo_streamer_deinit(apollo->streamer_handle);
    apollo->streamer_handle = NULL;
}


static void apollo_stream_volume_up(apollo_app_t* apollo)
{
    uint8_t vol_att = 0;

    if (apollo == NULL || apollo->tag != APOLLO_TAG_VALID || apollo->streamer_handle == NULL)
    {
        return;
    }

    if (apollo_streamer_send_command(apollo->streamer_handle, APOLLO_STREAMER_COMMAND_GET_VOLUME_ATTENUATION, &vol_att, sizeof(vol_att)) == WICED_SUCCESS)
    {
        vol_att += APOLLO_STREAM_VOL_ATT_STEP;
        if ( vol_att > APOLLO_VOLUME_MAX )
        {
            vol_att = APOLLO_VOLUME_MAX;
        }
        apollo_streamer_send_command(apollo->streamer_handle, APOLLO_STREAMER_COMMAND_SET_VOLUME_ATTENUATION, &vol_att, sizeof(vol_att));
    }
}


static void apollo_stream_volume_down(apollo_app_t* apollo)
{
    uint8_t vol_att = 0;

    if (apollo == NULL || apollo->tag != APOLLO_TAG_VALID || apollo->streamer_handle == NULL)
    {
        return;
    }

    if (apollo_streamer_send_command(apollo->streamer_handle, APOLLO_STREAMER_COMMAND_GET_VOLUME_ATTENUATION, &vol_att, sizeof(vol_att)) == WICED_SUCCESS)
    {
        if ( vol_att > APOLLO_STREAM_VOL_ATT_STEP )
        {
            vol_att -= APOLLO_STREAM_VOL_ATT_STEP;
        }
        else
        {
            vol_att = APOLLO_VOLUME_MIN;
        }
        apollo_streamer_send_command(apollo->streamer_handle, APOLLO_STREAMER_COMMAND_SET_VOLUME_ATTENUATION, &vol_att, sizeof(vol_att));
    }
}


static wiced_result_t apollo_play_start(apollo_app_t* apollo)
{
    apollo_player_params_t params;

    params.event_cb          = apollo_player_event_callback;
    params.userdata          = apollo;
    params.interface         = apollo_get_rmc_interface(&apollo->dct_tables);
    params.rtp_port          = apollo->dct_tables.dct_app->rtp_port;
    params.channel           = apollo->dct_tables.dct_app->speaker_channel;
    params.volume            = apollo->dct_tables.dct_app->volume;
    params.device_id         = apollo->dct_tables.dct_app->audio_device_tx;
    params.buffer_nodes      = APOLLO_RX_AUDIO_RENDER_BUFFER_NODES;
    params.buffer_ms         = apollo->dct_tables.dct_app->buffering_ms;
    params.threshold_ns      = apollo->dct_tables.dct_app->threshold_ns;
    params.clock_enable      = apollo->dct_tables.dct_app->clock_enable;
    params.pll_tuning_enable = apollo->dct_tables.dct_app->pll_tuning_enable;


#if defined(USE_AMBILIGHT)
    /*
     * ambient light control
     * we are a sink side so we initiate : AMBLT_BACK + AMBLT_SINK
     */
    if((apollo->amblt_sink == NULL ) && (apollo->amblt_back == NULL))
    {
        /* config i2c bus */
        apollo->amblt_i2c_hw.port          = WICED_I2C_2;
        apollo->amblt_i2c_hw.address       = 0x08;
        apollo->amblt_i2c_hw.address_width = I2C_ADDRESS_WIDTH_7BIT;
        apollo->amblt_i2c_hw.flags         = I2C_DEVICE_NO_DMA;
        /* note: psoc without clock stretching does not work well at high speed */
        /* but we have co-existance with the oled display on the same i2s bus   */
        /* so we are forced to fix the speed to HIGH */
        apollo->amblt_i2c_hw.speed_mode    = I2C_HIGH_SPEED_MODE;

        /* init backend */
        AMBLT_BACK_ERR err_back = AMBLT_BACK_NO_ERR;
        amblt_back_new(&err_back, &apollo->amblt_back, AMBLT_DRV_PSOC, AMBLT_BUS_PROTO_I2C, &apollo->amblt_psoc_hw, &apollo->amblt_i2c_hw);
        if ( AMBLT_BACK_NO_ERR == err_back )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Ambilight backend create: success.\r\n");

            amblt_back_set_bus_mtx(NULL, apollo->amblt_back, &apollo->i2c_bus_mtx);

            amblt_sink_err_t  err_sink = AMBLT_SINK_NO_ERR;
            amblt_sink_new(&err_sink, &apollo->amblt_sink, 5/* tbd : set to zero the log lvl here !*/);
            if( AMBLT_SINK_NO_ERR == err_sink)
            {
                /* set the speaker channel map for the ambilight processing */
                amblt_sink_start(&err_sink, apollo->amblt_sink, apollo->amblt_back, (apollo->dct_tables.dct_app->speaker_channel) );
                if(AMBLT_SINK_NO_ERR == err_sink)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Ambilight sink: started.\r\n");
                }
            }
        }
    }
#endif

    /*
     * Fire off the apollo player.
     */

    apollo->player_handle = apollo_player_init(&params);
    if (apollo->player_handle == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to initialize apollo_player\r\n");
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}


static void apollo_play_stop(apollo_app_t* apollo)
{
    if (apollo == NULL || apollo->tag != APOLLO_TAG_VALID || apollo->player_handle == NULL)
    {
        return;
    }

    apollo_player_deinit(apollo->player_handle);
    apollo->player_handle = NULL;
}


static void apollo_service_start(apollo_app_t* apollo)
{
    wiced_result_t result;

    if (apollo->dct_tables.dct_app->apollo_role == APOLLO_ROLE_SINK)
    {
        if (apollo->player_handle == NULL)
        {
            result = apollo_play_start(apollo);
            if (result != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error creating apollo player service\r\n");
            }
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Apollo player service already running\r\n");
        }
    }
    else
    {
        if (apollo->streamer_handle == NULL)
        {
            result = apollo_stream_start(apollo);
            if (result != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error creating apollo streamer service\r\n");
            }
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Apollo streamer service already running\r\n");
        }
    }
}


static void apollo_service_stop(apollo_app_t* apollo)
{
    if (apollo->dct_tables.dct_app->apollo_role == APOLLO_ROLE_SINK)
    {
        apollo_play_stop(apollo);
    }
    else
    {
        apollo_stream_stop(apollo);
    }
}


static void apollo_handle_timer(apollo_app_t* apollo)
{
    apollo_player_stats_t stats;
    apollo_report_stats_t* rstats;
    apollo_report_msg_t* msg;
    wiced_ip_address_t broadcast_addr;
    wiced_time_t curtime;
    wiced_packet_t* packet;
    wiced_result_t result;
    uint16_t available_space;
    uint32_t* reportbuf;
    int32_t rssi;
    int len;

    if (apollo->playback_active == WICED_FALSE || apollo->player_handle == NULL)
    {
        /*
         * See if it's been long enough since playback stopped that
         * we can go ahead and cancel the timer.
         */

        wiced_time_get_time(&curtime);
        if (curtime > apollo->playback_ended + PLAYBACK_TIMER_TIMEOUT_MSECS)
        {
            wiced_rtos_stop_timer(&apollo->timer);
            return;
        }
    }

    if (apollo_player_ioctl(apollo->player_handle, APOLLO_PLAYER_IOCTL_GET_STATS, &stats) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error querying player stats\n");
        memset(&stats, 0, sizeof(apollo_player_stats_t));
    }

    /*
     * Get a UDP packet from the packet pool.
     */

    len = ((sizeof(apollo_report_msg_t) + sizeof(apollo_report_stats_t) + 3) / 4) * 4;
    if ((result = wiced_packet_create_udp(&apollo->report_socket, len, &packet, (uint8_t**)&reportbuf, &available_space)) != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate report packet\n");
        return;
    }
    wiced_packet_set_data_end(packet, (uint8_t*)reportbuf + len);

    /*
     * Construct the report message.
     */

    msg = (apollo_report_msg_t*)reportbuf;
    msg->magic       = APOLLO_REPORT_MAGIC_TAG;
    msg->version     = APOLLO_REPORT_VERSION;
    msg->mac_addr[0] = apollo->mac_address.octet[0];
    msg->mac_addr[1] = apollo->mac_address.octet[1];
    msg->mac_addr[2] = apollo->mac_address.octet[2];
    msg->mac_addr[3] = apollo->mac_address.octet[3];
    msg->mac_addr[4] = apollo->mac_address.octet[4];
    msg->mac_addr[5] = apollo->mac_address.octet[5];
    msg->msg_type    = APOLLO_REPORT_MSG_STATS;
    msg->msg_length  = sizeof(apollo_report_stats_t);

    rstats = (apollo_report_stats_t*)msg->msg_data;

    wwd_wifi_get_rssi(&rssi);
    rstats->version                      = APOLLO_REPORT_STATS_VERSION;
    rstats->rssi                         = (int16_t)rssi;
    rstats->speaker_channel              = apollo->dct_tables.dct_app->speaker_channel;
    rstats->rtp_packets_received         = apollo->total_rtp_packets_received + stats.rtp_packets_received;
    rstats->rtp_packets_dropped          = apollo->total_rtp_packets_dropped  + stats.rtp_packets_dropped;
    rstats->audio_frames_played          = apollo->total_audio_frames_played  + stats.audio_frames_played;
    rstats->audio_frames_dropped         = apollo->total_audio_frames_dropped + stats.audio_frames_dropped;
    rstats->audio_concealment_slcplc_cnt = apollo->total_audio_concealment_slcplc_cnt + stats.audio_concealment_slcplc_cnt;
    rstats->audio_concealment_fec_cnt    = apollo->total_audio_concealment_fec_cnt    + stats.audio_concealment_fec_cnt;


    /*
     * And send it off.
     */

    SET_IPV4_ADDRESS(broadcast_addr, MAKE_IPV4_ADDRESS(255, 255, 255, 255));
    result = wiced_udp_send(&apollo->report_socket, &broadcast_addr, APOLLO_REPORT_PORT, packet);

    if (result != WICED_SUCCESS)
    {
        /*
         * Something went wrong. We need to release the packet back to the pool.
         */

        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error sending report packet\n");
        wiced_packet_delete(packet);
    }
}


static void apollo_handle_rtp_timing_cmd(apollo_app_t* apollo)
{
    apollo_player_timing_stats_t rtp_timing;
    uint32_t i;

    if (apollo->player_handle == NULL)
    {
        /*
         * RTP timing commands only valid for sink.
         */

        return;
    }

    switch (apollo->rtp_timing_cmd)
    {
        case APOLLO_CMD_RTP_TIMING_INIT:
            if (apollo_player_ioctl(apollo->player_handle, APOLLO_PLAYER_IOCTL_RTP_TIMING_INIT, (void*)apollo->rtp_timing_entries) != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error initializing RTP timing log with %u entries\n", apollo->rtp_timing_entries);
            }
            break;

        case APOLLO_CMD_RTP_TIMING_START:
            if (apollo_player_ioctl(apollo->player_handle, APOLLO_PLAYER_IOCTL_RTP_TIMING_START, NULL) != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error starting RTP timing log\n");
            }
            break;

        case APOLLO_CMD_RTP_TIMING_STOP:
            if (apollo_player_ioctl(apollo->player_handle, APOLLO_PLAYER_IOCTL_RTP_TIMING_STOP, NULL) != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error stopping RTP timing log\n");
            }
            break;

        case APOLLO_CMD_RTP_TIMING_RESET:
            if (apollo_player_ioctl(apollo->player_handle, APOLLO_PLAYER_IOCTL_RTP_TIMING_RESET, NULL) != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error resetting RTP timing log\n");
            }
            break;

        case APOLLO_CMD_RTP_TIMING_DUMP:
            if (apollo_player_ioctl(apollo->player_handle, APOLLO_PLAYER_IOCTL_RTP_TIMING_GET, &rtp_timing) != WICED_SUCCESS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error getting RTP timing log data\n");
                break;
            }

            /*
             * We're going to be outputting a lot of data to the log. Stop playback while we do so.
             */

            apollo_service_stop(apollo);

            wiced_log_printf("RTP Timing Log Dump:\n");
            for (i = 0; i < rtp_timing.number_entries; i++)
            {
                wiced_log_printf("  Seq: %04u  arrival time: %d.%09d\n", rtp_timing.rtp_entries[i].sequence_number,
                                 (int)(rtp_timing.rtp_entries[i].timestamp / NANOSECONDS_PER_SECOND), (int)(rtp_timing.rtp_entries[i].timestamp % NANOSECONDS_PER_SECOND));
            }
            break;

        default:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unknown RTP timing log command: %d\n", apollo->rtp_timing_cmd);
            break;
    }

    apollo->rtp_timing_cmd = APOLLO_CMD_RTP_TIMING_NONE;
}

static void apollo_mainloop(apollo_app_t* apollo)
{
    wiced_result_t result;
    uint32_t events;

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Begin apollo mainloop\r\n");

    /*
     * If auto start is set and the network is up then begin by sending ourselves a start event.
     */

#if defined(OTA2_SUPPORT)
    if (apollo->ota2_info.perform_ota2_update_at_start == WICED_TRUE)
    {
        /* copy default URI to prep for the update */
        strlcpy(apollo->ota2_info.update_uri, apollo->dct_tables.dct_app->ota2_default_update_uri, sizeof(apollo->ota2_info.update_uri) );
        wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_GET_UPDATE);
        apollo->ota2_info.perform_ota2_update_at_start = WICED_FALSE;

        /* NOTE: Since we skipped out of initialization early, the RMC network is *not* up.
         * The next auto_start test, which starts playback on boot-up, should fail.
         */
    }
#endif

    if (apollo->dct_tables.dct_app->auto_start && apollo->rmc_network_up)
    {
        wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_START);
    }

    while (apollo->tag == APOLLO_TAG_VALID)
    {
        events = 0;
        result = wiced_rtos_wait_for_event_flags(&apollo->events, APOLLO_ALL_EVENTS, &events,
                                                 WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if (result != WICED_SUCCESS)
        {
            continue;
        }

        if (events & APOLLO_EVENT_SHUTDOWN)
        {
            break;
        }

        if (events & APOLLO_EVENT_START)
        {
            apollo->stop_received = 0;
            apollo_service_start(apollo);
        }

        if (events & (APOLLO_EVENT_STOP | APOLLO_EVENT_AUTOSTOP))
        {
            apollo_service_stop(apollo);

            if (events & APOLLO_EVENT_STOP)
            {
                apollo->stop_received = 1;
            }
            else if (!apollo->stop_received)
            {
                /*
                 * Start playing again automatically.
                 */

                wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_START);
            }
        }

#ifndef APOLLO_NO_BT
        if (events & APOLLO_EVENT_CONFIG_GATT)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_WARNING, "Saving new BTLE GATT attributes and rebooting...\r\n");
            apollo_config_save(&apollo->dct_tables);
            apollo_config_deinit(&apollo->dct_tables);
            apollo_config_gatt_server_stop();
            apollo_service_stop(apollo);
            wiced_framework_reboot();
        }
#endif

        if (events & APOLLO_EVENT_RTP_TIMING)
        {
            apollo_handle_rtp_timing_cmd(apollo);
        }

        if (events & APOLLO_EVENT_TIMER)
        {
            apollo_handle_timer(apollo);
        }

        if (events & APOLLO_EVENT_RX_CSA)
        {
            wiced_packet_t* wiced_packet;
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Handling CSA Event\r\n");
            if (( result = wiced_udp_receive( &apollo->csa_socket, &wiced_packet, WICED_NO_WAIT )) == WICED_SUCCESS)
            {
                uint16_t data_length = 0, available_data_length;
                rmc_csa_msg_t *csa;
                uint8_t *csa_ptr = NULL;

                if (wiced_packet == NULL)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Read Null CSA Packet\r\n");
                }

                result = wiced_packet_get_data(wiced_packet, 0, &csa_ptr, &data_length, &available_data_length);
                if (result != WICED_SUCCESS || csa_ptr == NULL || data_length == 0 || (data_length != available_data_length))
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to find data in CSA Packet\r\n");
                }

                csa = (rmc_csa_msg_t *)csa_ptr;
                if (csa->msg_type == APOLLO_CSA_MSGTYPE)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "CSA chan %d, count %d, mode %d\r\n",
                        csa->cs.chspec & 0xff, csa->cs.count, csa->cs.mode);

                    if (wwd_wifi_send_csa(&csa->cs, WWD_AP_INTERFACE) != WWD_SUCCESS) {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Unable to issue CSA\r\n");
                    }
                }

                wiced_packet_delete(wiced_packet);
            }
        }

        if (events & APOLLO_EVENT_NETWORK_TIMER)
        {
            if (apollo->network_timer_ignore == WICED_FALSE)
            {
                /*
                 * Try and bring up the network.
                 */
                if (apollo_start_network_services(apollo) == WICED_SUCCESS)
                {
                    wiced_rtos_stop_timer(&apollo->network_timer);
                    if (apollo->dct_tables.dct_app->auto_start)
                    {
                        wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_START);
                    }
                }
            }
        }

        if (events & APOLLO_EVENT_RELOAD_DCT_WIFI)
        {
            apollo_config_reload_dct_wifi(&apollo->dct_tables);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "WiFi DCT changed\r\n");
        }

        if (events & APOLLO_EVENT_RELOAD_DCT_NETWORK)
        {
            apollo_config_reload_dct_network(&apollo->dct_tables);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Network DCT changed\r\n");
        }

        if (events & APOLLO_EVENT_RELOAD_DCT_BLUETOOTH)
        {
            apollo_config_reload_dct_bluetooth(&apollo->dct_tables);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Bluetooth DCT changed\r\n");
        }

        if (events & APOLLO_EVENT_RMC_DOWN)
        {
            apollo->stop_received = 1;
            wiced_rtos_stop_timer(&apollo->timer);
            apollo_service_stop(apollo);
            result = apollo_stop_network_services(apollo);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_stop_network_services() result: %d \r\n", result);

#if defined(OTA2_SUPPORT)
            /* WAR - This delay is needed, as it doesn't work if < 1500 ms.
             * The FW is unable to connect to an AP with a password.
             */
            wiced_rtos_delay_milliseconds(1500);

            /* Let OTA2 know that we have brought down our network */
            apollo_ota2_network_is_down(apollo);
#endif
         }

        if (events & APOLLO_EVENT_RMC_UP)
        {
            result = apollo_start_network_services(apollo);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "apollo_start_network_services() result: %d \r\n", result);
        }

        if (events & APOLLO_EVENT_VOLUME_UP)
        {
            apollo_stream_volume_up(apollo);
        }

        if (events & APOLLO_EVENT_VOLUME_DOWN)
        {
            apollo_stream_volume_down(apollo);
        }

        if (events & APOLLO_EVENT_PTP_PACKET)
        {
            wiced_wifi_set_iovar_value(IOVAR_STR_WD_DISABLE, 1 << 2, apollo->interface);
        }

#if defined(OTA2_SUPPORT)
        if (events & APOLLO_EVENT_GET_UPDATE)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "GET_UPDATE\r\n");

            apollo->ota2_info.cb_opaque                           = apollo;

            apollo->ota2_info.bg_params.log_level                = apollo->dct_tables.dct_app->log_level;

            apollo->ota2_info.bg_params.auto_update               = 1;       /* auto update on reboot (callback overrides)  */
            apollo->ota2_info.bg_params.initial_check_interval    = 1;       /* unused in this call (must be non-zero)      */
            apollo->ota2_info.bg_params.check_interval            = 0;       /* unused in this call                         */
            apollo->ota2_info.bg_params.retry_check_interval      = SECONDS_PER_MINUTE;     /* minimum retry is one minute  */
            apollo->ota2_info.bg_params.ota2_interface            = apollo->dct_tables.dct_network->interface;
            apollo->ota2_info.bg_params.default_ap_info           = NULL;    /* don't restart AP using wiced OTA2 routines  */
            apollo->ota2_info.bg_params.ota2_ap_info              = NULL;    /* use the DCT AP list                         */
            apollo->ota2_info.bg_params.ota2_ap_list              = apollo->dct_tables.dct_wifi->stored_ap_list;
            apollo->ota2_info.bg_params.ota2_ap_list_count        = CONFIG_AP_LIST_SIZE;

            /* This is NOT a blocking call, it runs asynch */
            result = apollo_ota2_start_bg_update(apollo);
            if (result != WICED_SUCCESS)
            {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "GET_UPDATE apollo_get_update() failed! %d \r\n", result);
            }
        }

        if (events & APOLLO_EVENT_START_TIMED_UPDATE)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "START_TIMED_UPDATE -- we check if we are streaming when the timer goes off \r\n");

            /* don't stop the OTA2 background service after we finish an update */
            apollo->ota2_info.cb_opaque                           = apollo;

            apollo->ota2_info.bg_params.log_level                = apollo->dct_tables.dct_app->log_level;

            apollo->ota2_info.bg_params.initial_check_interval    = SECONDS_PER_MINUTE * 1; /* initial update time from now                */
            apollo->ota2_info.bg_params.check_interval            = SECONDS_PER_DAY;         /* try once a day                              */

            apollo->ota2_info.bg_params.ota2_interface            = apollo->dct_tables.dct_network->interface;
            apollo->ota2_info.bg_params.retry_check_interval      = SECONDS_PER_MINUTE;     /* minimum retry is one minute                 */
            apollo->ota2_info.bg_params.default_ap_info           = NULL;                   /* don't restart AP using wiced OTA2           */
            apollo->ota2_info.bg_params.ota2_ap_info              = NULL;                   /* use the DCT AP list                         */
            apollo->ota2_info.bg_params.ota2_ap_list              = apollo->dct_tables.dct_wifi->stored_ap_list;
            apollo->ota2_info.bg_params.ota2_ap_list_count        = CONFIG_AP_LIST_SIZE;

            /* This is NOT a blocking call - it runs asynch */
            result = apollo_ota2_start_bg_update(apollo);
            if (result != WICED_SUCCESS)
            {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "apollo_start_bg_update() failed! %d \r\n", result);
            }
        }

        if (events & APOLLO_EVENT_STOP_TIMED_UPDATE)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE, "STOP_TIMED_UPDATE\r\n");
            result = apollo_ota2_stop_bg_update(apollo);
            if (result != WICED_SUCCESS)
            {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "apollo_stop_bg_update() failed! %d \r\n", result);
            }
        }

        if (events & APOLLO_EVENT_OTA2_STATUS)
        {
            apollo_ota2_status(apollo);
        }

        /* this flag is set in the OTA2 callback */
        if (events & APOLLO_EVENT_OTA2_UPDATE)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "download progress: %d \r\n", apollo->ota2_info.ota2_download_percentage);
        }
#endif  /* defined(OTA2_SUPPORT) */
    };

    /*
     * Make sure that playback has been shut down.
     */

    apollo_service_stop(apollo);

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "End apollo mainloop\r\n");
}


static wiced_result_t apollo_cmd_sender_callback(void* handle, void* userdata, APOLLO_CMD_SENDER_EVENT_T event, void* arg)
{
    apollo_app_t* apollo = (apollo_app_t*)userdata;
    apollo_peers_t* peers;
    int i;

    if (apollo == NULL || apollo->tag != APOLLO_TAG_VALID)
    {
        /*
         * Bad player handle so just return.
         */

        return WICED_SUCCESS;
    }

    if (event == APOLLO_CMD_SENDER_EVENT_DISCOVER_RESULTS)
    {
        peers = (apollo_peers_t*)arg;
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Found %d speakers on our network\r\n", peers->num_speakers);
        for (i = 0; i < peers->num_speakers; i++)
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "  Speaker name: %s, channel 0x%08lx, MAC %02x:%02x:%02x:%02x:%02x:%02x, IP %ld.%ld.%ld.%ld\r\n",
                          peers->speakers[i].config.speaker_name, peers->speakers[i].config.speaker_channel,
                          peers->speakers[i].mac.octet[0], peers->speakers[i].mac.octet[1], peers->speakers[i].mac.octet[2],
                          peers->speakers[i].mac.octet[3], peers->speakers[i].mac.octet[4], peers->speakers[i].mac.octet[5],
                          (peers->speakers[i].ipaddr >> 24) & 0xFF, (peers->speakers[i].ipaddr >> 16) & 0xFF,
                          (peers->speakers[i].ipaddr >>  8) & 0xFF, peers->speakers[i].ipaddr & 0xFF);
            }
    }
    else if (event == APOLLO_CMD_SENDER_EVENT_COMMAND_STATUS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "%d peers replied\r\n", (int)arg);
    }

    return WICED_SUCCESS;
}


static wiced_result_t apollo_cmd_callback(void* handle, void* userdata, APOLLO_CMD_EVENT_T event, void* arg)
{
    apollo_app_t* apollo = (apollo_app_t*)userdata;
    apollo_cmd_speaker_t* cmd_speaker;
    wiced_ip_address_t ip_addr;
    apollo_cmd_rtp_timing_t* rtp_timing;
    apollo_volume_t volume;
    apollo_cmd_metadata_t* data_buffer;

#if defined(USE_AMBILIGHT)
    apollo_cmd_ambilight_t* amblt_data;
#endif

    int log_level;
    int len;

    if (apollo == NULL || apollo->tag != APOLLO_TAG_VALID)
    {
        /*
         * Bad player handle so just return.
         */

        return WICED_SUCCESS;
    }

    switch (event)
    {
        case APOLLO_CMD_EVENT_QUERY_SPEAKER:
            cmd_speaker = (apollo_cmd_speaker_t*)arg;
            if (cmd_speaker)
            {
                cmd_speaker->speaker_name     = apollo->dct_tables.dct_app->speaker_name;
                cmd_speaker->speaker_name_len = strlen(apollo->dct_tables.dct_app->speaker_name);
                cmd_speaker->speaker_channel  = apollo->dct_tables.dct_app->speaker_channel;
            }
            break;

        case APOLLO_CMD_EVENT_SET_SPEAKER:
            cmd_speaker = (apollo_cmd_speaker_t*)arg;
            if (cmd_speaker == NULL)
            {
                break;
            }

            len = cmd_speaker->speaker_name_len;
            if (len > sizeof(apollo->dct_tables.dct_app->speaker_name) - 1)
            {
                len = sizeof(apollo->dct_tables.dct_app->speaker_name) - 1;
            }

            /*
             * If the speaker channel or name has changed than update our configuration.
             */

            if (cmd_speaker->speaker_channel != apollo->dct_tables.dct_app->speaker_channel ||
                strncmp(cmd_speaker->speaker_name, apollo->dct_tables.dct_app->speaker_name, len))
            {
                apollo->dct_tables.dct_app->speaker_channel = cmd_speaker->speaker_channel;
                strncpy(apollo->dct_tables.dct_app->speaker_name, cmd_speaker->speaker_name, len);
                apollo->dct_tables.dct_app->speaker_name[len] = '\0';
                wiced_dct_write((void*)apollo->dct_tables.dct_app, DCT_APP_SECTION, 0, sizeof(apollo_dct_t));
            }
            break;

        case APOLLO_CMD_EVENT_SET_VOLUME:
            memcpy(&volume, arg, sizeof(volume));
            if (volume.volume < APOLLO_VOLUME_MIN || volume.volume > APOLLO_VOLUME_MAX)
            {
                volume.volume = apollo->dct_tables.dct_app->volume;
            }

            /*
             * Do we need to adjust the volume?
             */

            if (volume.volume != apollo->dct_tables.dct_app->volume)
            {
                /*
                 * If we're currently playing audio then adjust the volume.
                 */

                if (apollo->player_handle)
                {
                    apollo_player_set_volume(apollo->player_handle, volume.volume);
                }

                /*
                 * And save the new volume information.
                 */

                if (volume.caching_mode & APOLLO_CACHE_TO_MEMORY)
                {
                    apollo->dct_tables.dct_app->volume = volume.volume;
                }
                if (volume.caching_mode & APOLLO_STORE_TO_NVRAM)
                {
                    wiced_dct_write((void*)apollo->dct_tables.dct_app, DCT_APP_SECTION, 0, sizeof(apollo_dct_t));
                }
            }
            break;

        case APOLLO_CMD_EVENT_SET_LOG_SERVER:
            ip_addr.version = WICED_IPV4;
            ip_addr.ip.v4   = (uint32_t)arg;

            if (ip_addr.ip.v4 != 0 && !apollo->network_logging_active)
            {
                if (apollo_debug_create_tcp_data_socket(&ip_addr, 0) == WICED_SUCCESS)
                {
                    wiced_ip_address_t ip_addr;

                    wiced_log_set_platform_output(apollo_debug_tcp_log_output_handler);

                    /*
                     * Output the IP and MAC addresses. This is helpful when processing the log files.
                     */

                    wiced_ip_get_ipv4_address(WICED_STA_INTERFACE, &ip_addr);
                    wiced_log_printf("Apollo IP address:" IPV4_PRINT_FORMAT " MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
                                      IPV4_SPLIT_TO_PRINT(ip_addr),
                                      apollo->mac_address.octet[0], apollo->mac_address.octet[1], apollo->mac_address.octet[2],
                                      apollo->mac_address.octet[3], apollo->mac_address.octet[4], apollo->mac_address.octet[5]);
                    apollo->network_logging_active = WICED_TRUE;
                }
            }
            else if (ip_addr.ip.v4 == 0 && apollo->network_logging_active)
            {
                wiced_log_set_platform_output(apollo_log_output_handler);
                apollo_debug_close_tcp_data_socket();
                apollo->network_logging_active = WICED_FALSE;
            }
            break;

        case APOLLO_CMD_EVENT_SET_LOG_LEVEL:
            log_level = (int)arg;
            wiced_log_set_facility_level(WLF_AUDIO, log_level);
            break;

        case APOLLO_CMD_EVENT_RTP_TIMING:
            rtp_timing = (apollo_cmd_rtp_timing_t*)arg;
            if (rtp_timing != NULL && apollo->player_handle)
            {
                apollo->rtp_timing_cmd     = rtp_timing->cmd;
                apollo->rtp_timing_entries = rtp_timing->num_entries;

                /*
                 * Tell the main loop it has work to do.
                 */

                wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_RTP_TIMING);
            }
            break;

        case APOLLO_CMD_EVENT_METADATA:
            data_buffer = (apollo_cmd_metadata_t*)arg;

            if (data_buffer->type & (APOLLO_METADATA_TITLE | APOLLO_METADATA_ARTIST))
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "BT track: [%.*s - %.*s]\n",
                              data_buffer->data_length[APOLLO_METADATA_ARTIST_INDEX], data_buffer->data[APOLLO_METADATA_ARTIST_INDEX],
                              data_buffer->data_length[APOLLO_METADATA_TITLE_INDEX], data_buffer->data[APOLLO_METADATA_TITLE_INDEX]);
            }
            break;

        case APOLLO_CMD_EVENT_AMBILIGHT:
#if defined(USE_AMBILIGHT)
            /*
             *  push the message to the sink
             */

            amblt_data = (apollo_cmd_ambilight_t*)arg;

            if( amblt_data->sink_data_length > 0 )
            {
                amblt_sink_err_t err_sink = AMBLT_SINK_NO_ERR;
                amblt_sink_push( &err_sink, apollo->amblt_sink,  &amblt_data->sink_data[0], amblt_data->sink_data_length);
                if(err_sink!=AMBLT_SINK_NO_ERR)
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Ambilight sink error on event: %d\r\n", event);
                }
            }
#endif
            break;

        default:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unknown command event: %d\r\n", event);
            break;
    }

    return WICED_SUCCESS;
}


static void apollo_shutdown(apollo_app_t* apollo)
{
    /*
     * Mark the app structure as invalid since we are shutting down.
     */

    apollo->tag = APOLLO_TAG_INVALID;

    /*
     * Remove our PTP packet filter.
     */

    host_network_set_ethertype_filter(WICED_ETHERTYPE_DOT1AS, apollo->interface, NULL, NULL);

    /*
     * Shutdown the console.
     */

    apollo_console_stop(apollo);

    /*
     * Shutdown button handler
     */

    apollo_button_handler_deinit(apollo);

    apollo_stop_network_services(apollo);

    wiced_rtos_deinit_timer(&apollo->timer);
    wiced_rtos_deinit_timer(&apollo->network_timer);
    wiced_rtos_deinit_event_flags(&apollo->events);


#if defined(USE_AMBILIGHT)

    /* destroy sink */
    if(NULL!=apollo->amblt_sink)
    {
        amblt_sink_stop(NULL, apollo->amblt_sink);
        amblt_sink_destroy(NULL, apollo->amblt_sink, 1);
    }
    /* destroy backend */
    if(NULL!=apollo->amblt_back)
    {
        amblt_back_destroy(NULL, &apollo->amblt_back);
    }

    /* destroy source */
    if(NULL!=apollo->amblt_source)
    {
        amblt_source_stop( NULL, apollo->amblt_source );
        amblt_source_destroy( NULL, &apollo->amblt_source, 1 /*TRUE*/);
    }

    /* destroy source */
    if(NULL!=apollo->amblt_front)
    {
        amblt_front_stop( NULL, apollo->amblt_front );
        amblt_front_destroy( NULL, &apollo->amblt_front, 1 /*TRUE*/);
    }
#endif

#ifndef APOLLO_NO_BT
    apollo_config_gatt_server_stop();

    apollo_bt_service_deinit();
#endif

    /* BUS_SERIALIZATION: deinit the bus lock */
    wiced_rtos_deinit_mutex( &( g_apollo->i2c_bus_mtx ) );

    free(apollo);
}

static void apollo_console_dct_callback(console_dct_struct_type_t struct_changed, void* app_data)
{
    apollo_app_t*           apollo;

    /* sanity check */
    if (app_data == NULL)
    {
        return;
    }

    apollo = (apollo_app_t*)app_data;
    switch(struct_changed)
    {
        case CONSOLE_DCT_STRUCT_TYPE_WIFI:
            /* Get WiFi configuration */
            wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_RELOAD_DCT_WIFI);
            break;
        case CONSOLE_DCT_STRUCT_TYPE_NETWORK:
            /* Get network configuration */
            wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_RELOAD_DCT_NETWORK);
            break;
        case CONSOLE_DCT_STRUCT_TYPE_BLUETOOTH:
#ifdef WICED_DCT_INCLUDE_BT_CONFIG
            /* Get BT configuration */
            wiced_rtos_set_event_flags(&apollo->events, APOLLO_EVENT_RELOAD_DCT_BLUETOOTH);
#endif
            break;
        default:
            break;
    }
}

static wiced_result_t apollo_start_network_services(apollo_app_t* apollo)
{
    wiced_config_ap_entry_t ap_entry;
    wiced_result_t result;
    uint32_t other_ifaces = 0;
    uint32_t wifi_flags;

    if (apollo->rmc_network_up == WICED_TRUE)
    {
        return WICED_SUCCESS;
    }

    wiced_dct_read_with_copy( &wifi_flags, DCT_MISC_SECTION, OFFSETOF(platform_dct_misc_config_t, wifi_flags), sizeof(uint32_t) );
    wwd_wifi_set_flags( &wifi_flags, 0 );

    apollo->rmc_network_coming_up = WICED_TRUE;

    /* Bring up the network interface */
    memset(&ap_entry, 0, sizeof(ap_entry));
    ap_entry.details.SSID.length = apollo->dct_tables.dct_app->rmc_info.ssid_length;
    memcpy(&ap_entry.details.SSID.value, &apollo->dct_tables.dct_app->rmc_info.ssid_name, apollo->dct_tables.dct_app->rmc_info.ssid_length);
    ap_entry.details.security    = apollo->dct_tables.dct_app->rmc_info.security;
    ap_entry.security_key_length = apollo->dct_tables.dct_app->rmc_info.security_key_length;
    memcpy(&ap_entry.security_key, &apollo->dct_tables.dct_app->rmc_info.security_key, apollo->dct_tables.dct_app->rmc_info.security_key_length);
    ap_entry.details.bss_type    = apollo->dct_tables.dct_app->rmc_info.bss_type;
    ap_entry.details.channel     = apollo->dct_tables.dct_app->rmc_info.channel;
    ap_entry.details.band        = apollo->dct_tables.dct_app->rmc_info.band;
    /*
     * for an infrastructure connection where we are the source and not connected to an AP
     * We must be the AP
     *
     * RMC Bss type         Source          Sink
     *  ADHOC               STA + DHCP      STA
     *  INFRA               AP + DHCP       STA
     *  INFRA (external AP) STA             STA
     */

    apollo->interface = apollo_get_rmc_interface(&apollo->dct_tables);

    /* Determine if any other interfaces should be used */
    if ((apollo->interface == WICED_AP_INTERFACE) &&
        (apollo->dct_tables.dct_wifi->stored_ap_list[0].details.SSID.length)) {
            other_ifaces |= (1 << WICED_STA_INTERFACE);
    }

    if (wwd_wifi_is_mesh_enabled()) {
        if (apollo->dct_tables.dct_app->apollo_role == APOLLO_ROLE_SOURCE)
            result = apollo_network_up_default(apollo->interface, &ap_entry, &ap_ip_settings, apollo->dct_tables.dct_app->apollo_role, other_ifaces);
        else
            result = apollo_network_up_default(apollo->interface, &ap_entry, &sink_ip_settings, apollo->dct_tables.dct_app->apollo_role, other_ifaces);
    } else {
            result = apollo_network_up_default(apollo->interface, &ap_entry, &ap_ip_settings, apollo->dct_tables.dct_app->apollo_role, other_ifaces);
    }

    if (result == WICED_SUCCESS)
    {
        apollo->rmc_network_up = WICED_TRUE;
    }
    else
    {
        /*
         * The network didn't initialize.
         */

        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Bringing up network interface failed (%d)\n", result);
        apollo->rmc_network_coming_up = WICED_FALSE;
        return result;
    }

    /* If Apollo is using Infra mode and DCT is specifying a non-NULL SSID
     * then try to bring up upstream facing STA side.
     */

    apollo->upstream_interface_valid = WICED_FALSE;
    if (other_ifaces & (1 << WICED_STA_INTERFACE)) {
        wiced_band_list_t   band_list;
        int32_t             band = 0;
        wiced_mac_t         mac;
        wwd_result_t        wwd_result;

        /*
         * Tweak the mac addr of the STA interface, otherwise it will be the same as AP
         */
        wwd_result = wwd_wifi_get_mac_address(&mac, WWD_STA_INTERFACE);
        if (wwd_result != WWD_SUCCESS) {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Get ether_addr failed\n");
        }

        /* Twiddle low bit of second to last octet */
        mac.octet[4] ^= 1;

        wwd_result = wwd_wifi_set_mac_address(mac);  /* XXX hardcoded to always set STA interface */
        if (wwd_result != WWD_SUCCESS) {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Set ether_addr failed\n");
        }

        /*
         * If multi band hw, set join prefs to 5 Ghz since Apollo prefers to run on 5Ghz
         */
        if (wwd_wifi_get_supported_band_list(&band_list) == WWD_SUCCESS) {
            if (band_list.number_of_bands == 2) {
                wwd_wifi_set_preferred_association_band(WLC_BAND_5G);
                if (wwd_wifi_get_preferred_association_band(&band) == WWD_SUCCESS) {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Preferred band for association is %sGHz\n",
                        band == WLC_BAND_2G ? "2.4" : "5");
                }
            }
        }

        /* Join the upstream AP */
        result = wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

        if (result != WICED_SUCCESS) {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "STA unable to join upstream AP (%d)\n", result);
            apollo_link_down();
        }
        else
        {
            apollo->upstream_interface_valid = WICED_TRUE;
            apollo->upstream_interface       = WICED_STA_INTERFACE;
        }
    } else if (apollo->interface == WICED_AP_INTERFACE) {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "NUll upstream SSID, skipping STA interface.\n");
    }

    /*
     * Join the multicast group.
     */

    SET_IPV4_ADDRESS(apollo->mrtp_ip_address, APOLLO_MULTICAST_IPV4_ADDRESS_DEFAULT);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Joining multicast group %d.%d.%d.%d\n",
            (int)((apollo->mrtp_ip_address.ip.v4 >> 24) & 0xFF), (int)((apollo->mrtp_ip_address.ip.v4 >> 16) & 0xFF),
            (int)((apollo->mrtp_ip_address.ip.v4 >> 8) & 0xFF),  (int)(apollo->mrtp_ip_address.ip.v4 & 0xFF));
    result = wiced_multicast_join(apollo->interface, &apollo->mrtp_ip_address);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_multicast_join() failed (%d)\n", result);
    }

    /*
     * Save our MAC address.
     */

    wwd_wifi_get_mac_address(&apollo->mac_address, apollo->interface);

    /*
     * Create the Apollo command instance.
     */

    apollo->cmd_handle = apollo_cmd_init(apollo->interface, apollo, apollo_cmd_callback);
    if (apollo->cmd_handle == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to create Apollo command instance\r\n");
    }

    /*
     * And the command sender instance.
     */

    apollo->cmd_sender_handle = apollo_cmd_sender_init(apollo->interface, apollo, apollo_cmd_sender_callback);

    /*
     * Create the UDP socket for reporting.
     */

    result = wiced_udp_create_socket(&apollo->report_socket, APOLLO_REPORT_PORT + 1, apollo->interface);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to create report socket\n");
    }
    else
    {
        apollo->report_socket_ptr = &apollo->report_socket;
    }

    /*
     * Create socket for incoming CSA requests from STA.
     * Only used by source when in Infrastructure mode.
     */
    if (apollo->interface == WICED_AP_INTERFACE)
    {
        result = wiced_udp_create_socket(&apollo->csa_socket, APOLLO_CSA_PORT, apollo->interface);
        if (result == WICED_SUCCESS)
        {
            apollo->csa_socket_ptr = &apollo->csa_socket;
            result = wiced_udp_register_callbacks( &apollo->csa_socket, csa_receive_callback, apollo );
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to create csa socket\n");
        }
    }

    apollo->rmc_network_up = WICED_TRUE;
    apollo->rmc_network_coming_up = WICED_FALSE;
    return result;
}


static wiced_result_t apollo_stop_network_services(apollo_app_t* apollo)
{
    wiced_result_t      result = WICED_SUCCESS;

    if (apollo->rmc_network_up == WICED_FALSE)
    {
        return WICED_SUCCESS;
    }

    /*
     * Close the UDP reporting socket.
     */

    if (apollo->report_socket_ptr != NULL)
    {
        wiced_udp_delete_socket(&apollo->report_socket);
        apollo->report_socket_ptr = NULL;
    }

    /*
     * Close the CSA request socket.
     */
    if ((apollo->interface == WICED_AP_INTERFACE) && (apollo->csa_socket_ptr != NULL)){
        wiced_udp_delete_socket(&apollo->csa_socket);
        apollo->csa_socket_ptr = NULL;
    }

    /*
     * Shut down the Command sender
     */

    if (apollo->cmd_sender_handle != NULL)
    {
        apollo_cmd_sender_deinit(apollo->cmd_sender_handle);
        apollo->cmd_sender_handle = NULL;
    }

    /*
     * Shut down the Apollo Command Instance
     */

    if (apollo->cmd_handle != NULL)
    {
        apollo_cmd_deinit(apollo->cmd_handle);
        apollo->cmd_handle = NULL;
    }

    /*
     * Leave multicast session
     */

    wiced_multicast_leave(apollo->interface, &apollo->mrtp_ip_address);

    /*
     * Shutdown the network
     *
     * for an infrastructure connection where we are the source and not connected to an AP
     * We must be the AP
     *
     * RMC Bss type         Source          Sink
     *  ADHOC               STA + DHCP      STA
     *  INFRA               AP + DHCP       STA
     *  INFRA (external AP) STA             STA
     */

    result = apollo_network_down(apollo->interface, apollo->dct_tables.dct_app->rmc_info.bss_type, apollo->dct_tables.dct_app->apollo_role);

    apollo->rmc_network_up = WICED_FALSE;

    return result;
}

static apollo_app_t* apollo_init(void)
{
    apollo_app_t* apollo;
    wiced_result_t result;
    uint32_t tag = APOLLO_TAG_VALID;
#ifndef APOLLO_NO_BT
    wiced_result_t result_bt_stack;
    apollo_bt_service_init_params_t bt_init_params = {0, };
    apollo_config_gatt_server_params_t gatt_params = {0, };
#endif

    /*
     * Initialize the logging subsystem.
     */

    result = wiced_log_init(WICED_LOG_ERR, apollo_log_output_handler, apollo_log_get_time);
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("wiced_log_init() failed !\r\n"));
    }

    /* this is needed here because apollo_set_nvram_mac() uses DCT before wiced_init() call */
    wiced_dct_init();

    /*
     * Temporary - set the device MAC address in the NVRAM.
     */

    apollo_set_nvram_mac();

    /* Initialize the device */
    result = wiced_init();
    if (result != WICED_SUCCESS)
    {
        return NULL;
    }

    /* Register callbacks */
    wiced_network_register_link_callback(apollo_link_up, apollo_link_down, WICED_STA_INTERFACE);

    /*
     * Note what time we started up.
     */

    result = wiced_time_get_time(&apollo_start_time);

    /*
     * Allocate the main application structure.
     */

    apollo = calloc_named("apollo", 1, sizeof(apollo_app_t));
    if (apollo == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Unable to allocate apollo structure\r\n");
        return NULL;
    }
    apollo->initializing = WICED_TRUE;

    /*
     * Create our event flags.
     */

    result = wiced_rtos_init_event_flags(&apollo->events);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error initializing event flags\r\n");
        tag = APOLLO_TAG_INVALID;
    }

    /*
     * Create a timer.
     */

    result = wiced_rtos_init_timer(&apollo->timer, PLAYBACK_TIMER_PERIOD_MSECS, timer_callback, apollo);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error initializing timer\n");
        tag = APOLLO_TAG_INVALID;
    }

    /*
     * Get DCT
     */
    result = apollo_config_init(&apollo->dct_tables);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "apollo_config_init() failed !\r\n");
        apollo_config_deinit(&apollo->dct_tables);
        tag = APOLLO_TAG_INVALID;
    }
    console_dct_register_callback(apollo_console_dct_callback, apollo);

    /*
     * Set the global apollo structure pointer. This pointer is used by the command console processing so
     * setting it here allows the user to interact with the console.
     */

    g_apollo = apollo;

    /*
     * Create the Apollo command console.
     */

    result = apollo_console_start(apollo);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error starting the Apollo command console\n");
        free(apollo);
        return NULL;
    }

    /*
     * Init button handler
     */

    apollo_button_handler_init(apollo);

    /*
     * BUS_SERIALIATION: Init bus lock
     */
    wiced_rtos_init_mutex( &( g_apollo->i2c_bus_mtx ) );


#if defined(USE_AMBILIGHT)
    /*
     * clear out (frontend/source) and (sink/backend) handles
     */
    apollo->amblt_front  = NULL;
    apollo->amblt_source = NULL;
    apollo->amblt_sink   = NULL;
    apollo->amblt_back   = NULL;
#endif

    /*
     * Set the current logging level.
     */

    wiced_log_set_all_levels(apollo->dct_tables.dct_app->log_level);

    /*
     * Set our hostname to the speaker name - this way DHCP servers will see which unique speaker acquired a lease
     * once the network init runs ...
     */

    result = wiced_network_set_hostname(apollo->dct_tables.dct_app->speaker_name);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't set hostname for dhcp_client_init!\r\n");
    }

    /*
     * Make sure that 802.1AS time is enabled.
     */

    wiced_time_enable_8021as();

    /*
     * Initialize nanosecond clock for later use
     */

    wiced_init_nanosecond_clock();

    /* Initialize platform audio */
    platform_init_audio();

    /* print out our current configuration */
    apollo_config_print_info(&apollo->dct_tables);

#if defined(OTA2_SUPPORT)
    /* check for one-button OTA2 update */
    if (apollo_ota2_support_check_for_one_button_update() == WICED_TRUE)
    {
        apollo->ota2_info.perform_ota2_update_at_start = WICED_TRUE;

        /* Set the tag here as we are returning without starting BT or the Apollo network.
         * We are going to jump right into an OTA2 update when we start the main loop.
         *
         * It is crucial for the one-button update functionality the RMC network is not started.
         */
        apollo->tag = tag;
        return apollo;
    }
#endif

#ifndef APOLLO_NO_BT
    /*
     * Start main BT service
     * Provide offset at the END of APP section; otherwise, BT stack will overwrite valid/current APP section !
     */

    bt_init_params.app_dct_offset = sizeof(apollo_dct_t);
    result_bt_stack = apollo_bt_service_init( &bt_init_params );
    if (result_bt_stack != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't start main BT service !\r\n");
    }

    gatt_params.user_context   = apollo;
    gatt_params.gatt_event_cbf = gatt_event_callback;
    result = apollo_config_gatt_server_start( &gatt_params );
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't start BT GATT service !\r\n");
    }
#endif

    /* Bring up the network interface */
    result = apollo_start_network_services(apollo);
    if (result != WICED_SUCCESS)
    {
        /*
         * The network didn't initialize but we don't want to consider that a fatal error.
         * Start a timer to periodically retry bringing up the network.
         */
        result = wiced_rtos_init_timer(&apollo->network_timer, NETWORK_INIT_TIMER_PERIOD_MSECS, network_timer_callback, apollo);
        if (result == WICED_SUCCESS)
        {
            wiced_rtos_start_timer(&apollo->network_timer);
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error initializing network timer\n");
        }
    }

    if (apollo->dct_tables.dct_app->apollo_role == APOLLO_ROLE_SINK)
    {
        /*
         * We need to disable the phy watchdog and install a hook to look for PTP packets.
         */

        wiced_wifi_set_iovar_value(IOVAR_STR_WD_DISABLE, 2, apollo->interface);
        host_network_set_ethertype_filter(WICED_ETHERTYPE_DOT1AS, apollo->interface, apollo_ethernet_packet_filter, apollo);
    }

    /*
     * Set the application tag to the correct state.
     */

    apollo->tag = tag;
    apollo->initializing = WICED_FALSE;

#ifdef USE_AUDIO_DISPLAY
    {
        uint32_t wifi_channel;
        apollo_dct_collection_t* dct_tables = &apollo->dct_tables;

        wiced_wifi_get_channel(&wifi_channel);
        audio_display_create_management_thread(&apollo->display_thread, WICED_AUDIO_DISPLAY_ENABLE_WIFI);

        /* bus serialization */
        audio_display_set_bus_mtx(&( g_apollo->i2c_bus_mtx ));

        memset(apollo->display_info, 0, sizeof(apollo->display_info));

        strlcpy(apollo->display_name, (char*)apollo->dct_tables.dct_app->rmc_info.ssid_name, sizeof(apollo->display_name));
        sprintf(apollo->display_info, "%s - %d/%d %s", (dct_tables->dct_app->apollo_role != APOLLO_ROLE_SOURCE ? "Sink" : "Source"),
                (int)wifi_channel, apollo->dct_tables.dct_app->rmc_info.channel,
                ((apollo->dct_tables.dct_app->rmc_info.band == WICED_802_11_BAND_2_4GHZ) ? "2.4GHz" : "5GHz"));

        audio_display_header_update_options(BATTERY_ICON_IS_VISIBLE |BATTERY_ICON_SHOW_PERCENT |SIGNAL_STRENGTH_IS_VISIBLE);
        audio_display_footer_update_song_info(apollo->display_name, apollo->display_info);
        audio_display_footer_update_options(FOOTER_IS_VISIBLE | FOOTER_CENTER_ALIGN | ((dct_tables->dct_app->apollo_role == APOLLO_ROLE_SOURCE) ? FOOTER_OPTION_APOLLO_TX : FOOTER_OPTION_APOLLO_RX));
#ifndef APOLLO_NO_BT
        if ( result_bt_stack == WICED_SUCCESS )
        {
            audio_display_header_update_options(BATTERY_ICON_IS_VISIBLE | BATTERY_ICON_SHOW_PERCENT | SIGNAL_STRENGTH_IS_VISIBLE | BLUETOOTH_IS_CONNECTED | INVERT_BLUETOOTH_ICON_COLORS);
        }
#endif
    }
#endif

    {
        uint32_t *chipregs = (uint32_t *)PLATFORM_CHIPCOMMON_REGBASE(0x0);
        uint8_t version;

        version = (*chipregs >> 16) & 0xF;
        wiced_log_printf("Chip version is %s\n", version == 1 ? "B0" : (version == 2 ? "B1" : (version == 3 ? "B2" : "Unknown")));
    }

    return apollo;
}

void application_start(void)
{
    apollo_app_t* apollo;

    /*
     * Main initialization.
     */

    if ((apollo = apollo_init()) == NULL)
    {
        return;
    }

    if (apollo->tag != APOLLO_TAG_VALID)
    {
        apollo_shutdown(apollo);
        return;
    }

    /*
     * Drop into our main loop.
     */

    apollo_mainloop(apollo);

    /*
     * Cleanup and exit.
     */

    g_apollo = NULL;
    apollo_shutdown(apollo);
}

