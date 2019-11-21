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

#include "apollo_context.h"
#include "wiced_audio.h"

/******************************************************
 *                      Macros
 ******************************************************/

#ifdef PLATFORM_L1_CACHE_BYTES
#define NUM_BUFFERS_POOL_SIZE(x)                ((WICED_LINK_MTU_ALIGNED + sizeof(wiced_packet_t) + 1) * (x))
#define APP_RX_BUFFER_POOL_SIZE                 NUM_BUFFERS_POOL_SIZE(UPNPAVRENDER_RX_BUFFER_COUNT)
#define UPNPAVRENDER_RX_BUFFER_POOL_SIZE        (APP_RX_BUFFER_POOL_SIZE + PLATFORM_L1_CACHE_BYTES)
#else
#define UPNPAVRENDER_RX_BUFFER_POOL_SIZE        (WICED_NETWORK_MTU_SIZE * UPNPAVRENDER_RX_BUFFER_COUNT)
#endif

/******************************************************
 *                    Constants
 ******************************************************/

#define DEFAULT_LISTENING_PORT                  ( 49494 )
#define DEFAULT_UUID                            "37ddf93a-6644-4fe3-953c-5febcfc7287a"
#define DEFAULT_FRIENDLY_NAME                   "UPnPAVRender"
#define AUDIO_PLAYER_NUM_HTTP_BUFFERS           ( 80 )
#define AUDIO_PLAYER_NUM_AUDIO_BUFFERS          ( 2 )
#define AUDIO_PLAYER_SIZE_AUDIO_BUFFERS         ( 2048 )
#define AUDIO_PLAYER_DATA_THRESH_PERCENT        ( 20 )
#define AUDIO_PLAYER_NUM_AUDIO_BUFFERS_THRESH   ( (AUDIO_PLAYER_DATA_THRESH_PERCENT * AUDIO_PLAYER_NUM_HTTP_BUFFERS) / 100 )
#define AUDIO_PLAYER_DATA_THRESH_HIGH           ( AUDIO_PLAYER_NUM_HTTP_BUFFERS - AUDIO_PLAYER_NUM_AUDIO_BUFFERS_THRESH )
#define AUDIO_PLAYER_DATA_THRESH_LOW            ( AUDIO_PLAYER_NUM_AUDIO_BUFFERS_THRESH )
#define AUDIO_PLAYER_VOLUME_DEFAULT             ( 60 )

#define UPNPAVRENDER_RX_BUFFER_COUNT            ( 24 )

#define MAC_ADDRRESS_STR_LENGTH                 ( 12 )

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    int buflen;
    int maxlen;
    objhandle_t buf_handle;
    uint8_t buf[0];
} audio_buf_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

static uint8_t* upnpavrender_rx_packets = NULL;

/******************************************************
 *               Function Definitions
 ******************************************************/

static wiced_result_t upnpavrender_main_event_cbf(void* user_context, upnpavrender_event_t event, upnpavrender_event_data_t *event_data)
{
    wiced_result_t result = WICED_SUCCESS;
    apollo_app_t*  apollo = (apollo_app_t*)user_context;

    switch (event)
    {
        case UPNPAVRENDER_EVENT_PLAY:
            break;

        case UPNPAVRENDER_EVENT_PAUSE:
            break;

        case UPNPAVRENDER_EVENT_RESUME:
            break;

        case UPNPAVRENDER_EVENT_STOP:
            break;

        case UPNPAVRENDER_EVENT_SEEK:
            break;

        case UPNPAVRENDER_EVENT_PLAYBACK_PROGRESS:
            break;

        case UPNPAVRENDER_EVENT_MUTE:
        {
            uint8_t vol_att = 0;

            apollo->upnpavrender_mute_enabled = event_data->mute_enabled;
            if ( apollo->upnpavrender_mute_enabled == WICED_FALSE )
            {
                vol_att = apollo->upnpavrender_volume;
            }
            apollo_streamer_send_command(apollo->streamer_handle, APOLLO_STREAMER_COMMAND_SET_VOLUME_ATTENUATION, &vol_att, sizeof(vol_att));
        }
            break;

        case UPNPAVRENDER_EVENT_VOLUME:
            if ( apollo->upnpavrender_mute_enabled == WICED_FALSE )
            {
                uint8_t vol_att = event_data->volume;

                apollo->upnpavrender_volume = vol_att;
                apollo_streamer_send_command(apollo->streamer_handle, APOLLO_STREAMER_COMMAND_SET_VOLUME_ATTENUATION, &vol_att, sizeof(vol_att));
            }
            break;

        default:
            break;
    }

    return result;
}

static wiced_result_t upnpavrender_ac_event_cbf(audio_client_ref handle, void* userdata, AUDIO_CLIENT_EVENT_T event, void* arg)
{
    wiced_result_t        result = WICED_SUCCESS;
    apollo_app_t*         apollo = (apollo_app_t*)userdata;
    wiced_audio_config_t* config = NULL;

    UNUSED_PARAMETER(handle);

    switch (event)
    {
        case AUDIO_CLIENT_EVENT_ERROR:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error from audio_client: %d\n", (int)arg);
            break;

        case AUDIO_CLIENT_EVENT_AUDIO_FORMAT:
            config = (wiced_audio_config_t*)arg;
            if ( config == NULL )
            {
                result = WICED_BADARG;
                break;
            }

            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio format:\n");
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "    num channels: %u\n", config->channels);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, " bits per sample: %u\n", config->bits_per_sample);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "     sample rate: %lu\n", config->sample_rate);
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "      frame size: %u\n", config->frame_size);

            if ( apollo->streamer_handle != NULL )
            {
                result = apollo_streamer_send_command(apollo->streamer_handle, APOLLO_STREAMER_COMMAND_SET_AUDIO_CONFIG, config, sizeof(*config));
                if ( result != WICED_SUCCESS )
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error returned from APOLLO_STREAMER_COMMAND_SET_AUDIO_CONFIG: %d\n", result);
                }
                else
                {
                    result = apollo_streamer_send_command(apollo->streamer_handle, APOLLO_STREAMER_COMMAND_START, NULL, 0);
                    if ( result != WICED_SUCCESS )
                    {
                        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error returned from APOLLO_STREAMER_COMMAND_START: %d\n", result);
                    }
                }
            }
            break;

        case AUDIO_CLIENT_EVENT_PLAYBACK_STARTED:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client has reached STARTED !\n");
            if ( apollo->upnpavrender_paused == WICED_TRUE )
            {
                result = apollo_streamer_send_command(apollo->streamer_handle, APOLLO_STREAMER_COMMAND_START, NULL, 0);
                if ( result != WICED_SUCCESS )
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error returned from APOLLO_STREAMER_COMMAND_START: %d\n", result);
                }
                apollo->upnpavrender_paused = WICED_FALSE;
            }
            break;

        case AUDIO_CLIENT_EVENT_PLAYBACK_PAUSED:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client has reached PAUSED !\n");
            apollo->upnpavrender_paused = WICED_TRUE;
            result = apollo_streamer_send_command(apollo->streamer_handle, APOLLO_STREAMER_COMMAND_STOP, NULL, 0);
            if ( result != WICED_SUCCESS )
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error returned from APOLLO_STREAMER_COMMAND_STOP: %d\n", result);
            }
            break;

        case AUDIO_CLIENT_EVENT_PLAYBACK_STOPPED:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client has reached STOPPED !\n");
        case AUDIO_CLIENT_EVENT_PLAYBACK_EOS:
            if ( event == AUDIO_CLIENT_EVENT_PLAYBACK_EOS )
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client has reached EOS !\n");
            }
            apollo->upnpavrender_paused = WICED_FALSE;
            if ( apollo->streamer_handle != NULL )
            {
                bufmgr_stats_t pool_stats;

                result = apollo_streamer_send_command(apollo->streamer_handle, APOLLO_STREAMER_COMMAND_STOP, NULL, 0);
                if ( result != WICED_SUCCESS )
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error returned from APOLLO_STREAMER_COMMAND_STOP: %d\n", result);
                }

                bufmgr_pool_get_stats(apollo->upnpavrender_buf_pool, &pool_stats);
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Audio buffer pool stats: %lu/%lu\n",
                              pool_stats.num_buf_alloc, pool_stats.num_handle_alloc);
            }
            break;

        case AUDIO_CLIENT_EVENT_DATA_THRESHOLD_HIGH:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client has hit HIGH data threshold !\n");
            break;

        case AUDIO_CLIENT_EVENT_DATA_THRESHOLD_LOW:
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "audio_client has hit LOW data threshold !\n");
            break;

        default:
            break;
    }

    return result;
}


static wiced_result_t upnpavrender_ac_buf_get_cbf(audio_client_ref handle, void* userdata, audio_client_buf_t* ac_buf, uint32_t timeout_ms)
{
    apollo_app_t* apollo = (apollo_app_t*)userdata;
    objhandle_t buf_handle;
    audio_buf_t* buf;
    uint32_t buf_size;

    UNUSED_PARAMETER(handle);

    if ((apollo == NULL) || (apollo->tag != APOLLO_TAG_VALID) ||(apollo->upnpavrender_buf_pool == NULL))
    {
        return WICED_ERROR;
    }

    if (apollo->upnpavrender_paused == WICED_TRUE)
    {
        wiced_rtos_delay_milliseconds(timeout_ms);
        return WICED_ERROR;
    }

    if (bufmgr_pool_alloc_buf(apollo->upnpavrender_buf_pool, &buf_handle, (char **)&buf, &buf_size, timeout_ms) != BUFMGR_OK)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG2, "Unable to allocate audio packet\n");
        return WICED_ERROR;
    }

    memset(buf, 0, sizeof(audio_buf_t));

    ac_buf->opaque = buf_handle;
    ac_buf->buf    = buf->buf;
    ac_buf->buflen = AUDIO_PLAYER_SIZE_AUDIO_BUFFERS;
    ac_buf->curlen = 0;
    ac_buf->offset = 0;
    ac_buf->flags  = 0;

    return WICED_SUCCESS;
}


static wiced_result_t upnpavrender_ac_buf_release_cbf(audio_client_ref handle, void* userdata, audio_client_buf_t* ac_buf)
{
    wiced_result_t        result       = WICED_SUCCESS;
    apollo_app_t*         apollo       = (apollo_app_t*)userdata;
    apollo_streamer_buf_t streamer_buf;

    UNUSED_PARAMETER(handle);

    if (apollo == NULL || apollo->tag != APOLLO_TAG_VALID || apollo->upnpavrender_buf_pool == NULL)
    {
        return WICED_ERROR;
    }

    if (apollo->streamer_handle != NULL)
    {
        /*
         * Pass this buffer on to apollo streamer
         */

        if ((ac_buf->buf != NULL) && (ac_buf->curlen > 0))
        {
            streamer_buf.buf    = ac_buf->buf;
            streamer_buf.buflen = ac_buf->buflen;
            streamer_buf.offset = ac_buf->offset;
            streamer_buf.curlen = ac_buf->curlen;
            streamer_buf.opaque = ac_buf->opaque;

            if (ac_buf->flags & AUDIO_CLIENT_BUF_FLAG_EOS)
            {
                streamer_buf.flags |= APOLLO_STREAMER_BUF_FLAG_EOS;
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "* Got EOS flag on buffer ! *\n");
            }
            result = apollo_streamer_send_command(apollo->streamer_handle, APOLLO_STREAMER_COMMAND_PUSH_BUFFER, &streamer_buf, sizeof(streamer_buf));
            if (result != WICED_SUCCESS)
            {
                /*
                 * Apollo streamer didn't accept the buffer, make sure that we don't leak it.
                 */

                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Error returned from APOLLO_STREAMER_COMMAND_PUSH_BUFFER: %d\n", result);

                if ((apollo->upnpavrender_buf_pool != NULL) && (ac_buf->opaque != NULL))
                {
                    bufmgr_pool_free_buf(apollo->upnpavrender_buf_pool, (objhandle_t)ac_buf->opaque);
                    ac_buf->opaque = NULL;
                }
            }
        }
        else
        {
            if (ac_buf->flags & AUDIO_CLIENT_BUF_FLAG_EOS)
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "* Got EOS flag on buffer *\n");
            }
        }
    }
    else
    {
        /*
         * No apollo streamer instance. Just release the buffer.
         */

        if ((apollo->upnpavrender_buf_pool != NULL) && (ac_buf->opaque != NULL))
        {
            bufmgr_pool_free_buf(apollo->upnpavrender_buf_pool, (objhandle_t)ac_buf->opaque);
            ac_buf->opaque = NULL;
        }

    }

    return result;
}


void apollo_upnpavrender_release_buffer(apollo_app_t* apollo, apollo_streamer_event_data_t* event_data)
{
    if ((apollo->upnpavrender_buf_pool != NULL) && (event_data->buffer->opaque != NULL))
    {
        bufmgr_pool_free_buf(apollo->upnpavrender_buf_pool, (objhandle_t)event_data->buffer->opaque);
    }
}


static wiced_result_t set_unique_device_name_and_uuid( char *input_name, char *input_uuid,
                                                       char *out_unique_device_name, uint8_t device_name_size,
                                                       char *out_uuid, uint8_t uuid_size)
{
    wiced_result_t  result;
    wiced_mac_t mac;
    int rc_snprintf;

    result = wiced_wifi_get_mac_address( &mac );
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_wifi_get_mac_address() failed !\n") );

    rc_snprintf = snprintf( out_unique_device_name, device_name_size, "%s_%02x%02x%02x", input_name, mac.octet[3], mac.octet[4], mac.octet[5] );
    if ( rc_snprintf < ( strlen(input_name) + strlen("_XXXXXX") ) )
    {
        result = WICED_ERROR;
        wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "snprintf() failed !\n") );
        goto _exit;
    }

    strlcpy(out_uuid, input_uuid, uuid_size);
    rc_snprintf = snprintf( out_uuid + UPNPAV_UUID_STR_LENGTH - MAC_ADDRRESS_STR_LENGTH, MAC_ADDRRESS_STR_LENGTH, "%02x%02x%02x%02x%02x%02x",
                            mac.octet[0], mac.octet[1], mac.octet[2], mac.octet[3], mac.octet[4], mac.octet[5] );
    if ( rc_snprintf < MAC_ADDRRESS_STR_LENGTH )
    {
        result = WICED_ERROR;
        wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "snprintf() failed !\n") );
        goto _exit;
    }

 _exit:
    return result;
}


wiced_result_t apollo_upnpavrender_start(apollo_app_t* apollo)
{
    wiced_result_t                result              = WICED_ERROR;
    wiced_interface_t             iface               = apollo->streamer_params.iface;
    bufmgr_status_t               rc;
    upnpavrender_service_params_t upnpavrender_params;

    memset(&upnpavrender_params, 0, sizeof(upnpavrender_params));
    upnpavrender_params.interface                                       = iface;
    upnpavrender_params.listening_port                                  = DEFAULT_LISTENING_PORT;
    upnpavrender_params.uuid                                            = DEFAULT_UUID;
    upnpavrender_params.friendly_name                                   = DEFAULT_FRIENDLY_NAME;
    upnpavrender_params.event_cb                                        = upnpavrender_main_event_cbf;
    upnpavrender_params.user_context                                    = apollo;
    upnpavrender_params.audio_client_params.event_cb                    = upnpavrender_ac_event_cbf;
    upnpavrender_params.audio_client_params.userdata                    = apollo;
    upnpavrender_params.audio_client_params.interface                   = iface;
    upnpavrender_params.audio_client_params.data_buffer_num             = AUDIO_PLAYER_NUM_HTTP_BUFFERS;
    upnpavrender_params.audio_client_params.audio_buffer_num            = AUDIO_PLAYER_NUM_AUDIO_BUFFERS;
    upnpavrender_params.audio_client_params.audio_buffer_size           = AUDIO_PLAYER_SIZE_AUDIO_BUFFERS;
    upnpavrender_params.audio_client_params.data_threshold_high         = AUDIO_PLAYER_DATA_THRESH_HIGH;
    upnpavrender_params.audio_client_params.data_threshold_low          = AUDIO_PLAYER_DATA_THRESH_LOW;
    upnpavrender_params.audio_client_params.device_id                   = 0;
    upnpavrender_params.audio_client_params.volume                      = AUDIO_PLAYER_VOLUME_DEFAULT;
    upnpavrender_params.audio_client_params.enable_playback             = WICED_FALSE;
    upnpavrender_params.audio_client_params.buffer_get                  = upnpavrender_ac_buf_get_cbf;
    upnpavrender_params.audio_client_params.buffer_release              = upnpavrender_ac_buf_release_cbf;
    upnpavrender_params.audio_client_params.high_threshold_read_inhibit = WICED_FALSE;
    upnpavrender_params.audio_client_params.release_device_on_stop      = WICED_FALSE;

    if ( (apollo->upstream_interface_valid == WICED_TRUE) && (wiced_network_is_ip_up(apollo->upstream_interface) == WICED_TRUE) )
    {
        upnpavrender_params.interface                     = apollo->upstream_interface;
        upnpavrender_params.audio_client_params.interface = apollo->upstream_interface;
    }

    rc = bufmgr_pool_create(&apollo->upnpavrender_buf_pool, "upnpavrender_bufs", sizeof(audio_buf_t) + AUDIO_PLAYER_SIZE_AUDIO_BUFFERS,
                            AUDIO_PLAYER_NUM_AUDIO_BUFFERS, AUDIO_PLAYER_NUM_AUDIO_BUFFERS, 0, sizeof(uint32_t));
    wiced_action_jump_when_not_true(rc == BUFMGR_OK, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "bufmgr_pool_create failed\n"));

    apollo->upnpavrender_volume       = (uint8_t)upnpavrender_params.audio_client_params.volume;
    apollo->upnpavrender_mute_enabled = WICED_FALSE;
    apollo->upnpavrender_paused       = WICED_FALSE;

    if ( upnpavrender_rx_packets == NULL )
    {
        upnpavrender_rx_packets = malloc(UPNPAVRENDER_RX_BUFFER_POOL_SIZE);
        wiced_action_jump_when_not_true(upnpavrender_rx_packets != NULL, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Can't create UPnPAV render RX buffer pool ! Out of memory !\n"));
        result = wiced_network_create_packet_pool(upnpavrender_rx_packets, UPNPAVRENDER_RX_BUFFER_POOL_SIZE, WICED_NETWORK_PACKET_RX);
        if ( result != WICED_SUCCESS)
        {
            free(upnpavrender_rx_packets);
            upnpavrender_rx_packets = NULL;
            wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_network_create_packet_pool() failed !\n");
            goto _exit;
        }
    }

    result = set_unique_device_name_and_uuid(DEFAULT_FRIENDLY_NAME, DEFAULT_UUID,
                                             apollo->upnpavrender_friendly_name, sizeof(apollo->upnpavrender_friendly_name),
                                             apollo->upnpavrender_uuid, sizeof(apollo->upnpavrender_uuid));
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "set_unique_device_name_and_uuid() failed !\n") );
    upnpavrender_params.uuid          = apollo->upnpavrender_uuid;
    upnpavrender_params.friendly_name = apollo->upnpavrender_friendly_name;

    result = upnpavrender_service_start(&upnpavrender_params, &apollo->upnpavrender_handle);
    wiced_action_jump_when_not_true(result == WICED_SUCCESS, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "upnpavrender_service_start() failed !\n"));

 _exit:
    return result;
}


wiced_result_t apollo_upnpavrender_stop(apollo_app_t* apollo)
{
    apollo_streamer_send_command(apollo->streamer_handle, APOLLO_STREAMER_COMMAND_STOP, NULL, 0);
    upnpavrender_service_stop(apollo->upnpavrender_handle);
    bufmgr_pool_destroy(apollo->upnpavrender_buf_pool);

    return WICED_SUCCESS;
}
