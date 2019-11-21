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
 * Bluetooth Audio AVDT Sink Service
 *
 *
 *
 * Notes: Currently supports 44.1kHz and 48kHz audio
 */

#include <stdlib.h>
#include <string.h>
#include "wiced_rtos.h"
#include "wiced_result.h"
#include "wiced_log.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_remote_control.h"
#include "apollo_bt_a2dp_sink_private.h"
#include "apollo_bt_service.h"
#include "apollo_bt_main_service_private.h"
#include "apollo_bt_a2dp_sink_profiling.h"
#include "apollo_bt_remote_control_private.h"
#include "apollo_bt_nv.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define BLUETOOTH_DEVICE_NAME                   "Apollo"

/*
 * Recommended max_bitpool for high quality audio
 */
#define BT_AUDIO_A2DP_SBC_MAX_BITPOOL           53

#define APOLLO_BT_A2DP_SINK_EVENT_TIMEOUT_MSECS (1000)

/******************************************************
 *                   Enumerations
 ******************************************************/

enum
{
    APOLLO_BT_A2DP_SINK_EVENT_DISCONNECTED = (1 << 0),

    APOLLO_BT_A2DP_SINK_EVENT_ALL          = 0xFFFFFFFF
};

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

static uint8_t uuid_list[] =
{
    0x08, 0x11, /* Headset */
    0x1E, 0x11, /* Handsfree */
    0x0E, 0x11, /* AV Remote Control */
    0x0B, 0x11, /* Audio Sink */
};

static wiced_bt_a2dp_codec_info_t bt_audio_codec_capabilities =
{
    .codec_id = WICED_BT_A2DP_SINK_CODEC_SBC,
    .cie =
        {
            .sbc =
            {
                (A2D_SBC_IE_SAMP_FREQ_44 | A2D_SBC_IE_SAMP_FREQ_48),    /* samp_freq */
                (A2D_SBC_IE_CH_MD_MONO | A2D_SBC_IE_CH_MD_STEREO |
                 A2D_SBC_IE_CH_MD_JOINT | A2D_SBC_IE_CH_MD_DUAL),       /* ch_mode */
                (A2D_SBC_IE_BLOCKS_16 | A2D_SBC_IE_BLOCKS_12 |
                 A2D_SBC_IE_BLOCKS_8 | A2D_SBC_IE_BLOCKS_4),            /* block_len */
                (A2D_SBC_IE_SUBBAND_4 | A2D_SBC_IE_SUBBAND_8),          /* num_subbands */
                (A2D_SBC_IE_ALLOC_MD_L | A2D_SBC_IE_ALLOC_MD_S),        /* alloc_mthd */
                BT_AUDIO_A2DP_SBC_MAX_BITPOOL,          /* max_bitpool for high quality audio */
                A2D_SBC_IE_MIN_BITPOOL                                  /* min_bitpool */
            }
        }
};

static wiced_bt_a2dp_config_data_t bt_audio_codec_config =
{
    .feature_mask = 0,
    .codec_capabilities =
    {
        .count = 1,
        .info = &bt_audio_codec_capabilities,
    }
};

static apollo_bt_a2dp_sink_t  g_apollo_bt_a2dp_sink;
static apollo_bt_a2dp_sink_t *g_p_apollo_bt_a2dp_sink = NULL;

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

/*Utility Functions */
#ifdef USE_WICED_HCI
static inline uint8_t get_bt_a2dp_route(apollo_bt_a2dp_sink_output_mode_t output_mode)
{
    uint8_t route;

    switch (output_mode)
    {
        case APOLLO_BT_A2DP_OUTPUT_MODE_I2S:
            route = WICED_BT_A2DP_ROUTE_I2S;
            break;

        default:
            route = WICED_BT_A2DP_ROUTE_COMPRESSED_TRANSPORT;
            break;
    }

    return route;
}
#endif /* USE_WICED_HCI */

wiced_result_t bt_audio_get_config_from_cie( wiced_bt_a2dp_codec_info_t* p_codec_config, bt_audio_config_t* p_audio_config)
{
    wiced_result_t result = WICED_BADARG;

    if(p_codec_config == NULL || p_audio_config == NULL)
    {
        return result;
    }

    if(p_codec_config->codec_id == WICED_BT_A2DP_SINK_CODEC_SBC)
    {
        if(p_codec_config->cie.sbc.samp_freq == A2D_SBC_IE_SAMP_FREQ_44)
        {
            p_audio_config->sample_rate = 44100;
        }
        else if(p_codec_config->cie.sbc.samp_freq == A2D_SBC_IE_SAMP_FREQ_48)
        {
            p_audio_config->sample_rate = 48000;
        }
        else if(p_codec_config->cie.sbc.samp_freq == A2D_SBC_IE_SAMP_FREQ_32)
        {
            p_audio_config->sample_rate = 32000;
        }
        else if(p_codec_config->cie.sbc.samp_freq == A2D_SBC_IE_SAMP_FREQ_16)
        {
            p_audio_config->sample_rate = 16000;
        }
        else
        {
            return WICED_ERROR;
        }

        if(p_codec_config->cie.sbc.ch_mode == A2D_SBC_IE_CH_MD_MONO)
        {
            p_audio_config->channels = 1;
        }
        else
        {
            p_audio_config->channels = 2;
        }

        p_audio_config->bits_per_sample = 16;

        p_audio_config->frame_size = (p_audio_config->channels * p_audio_config->bits_per_sample) / 8;

        result = WICED_SUCCESS;
    }

    return result;
}


/* DM Functions */
wiced_result_t bt_audio_write_eir( uint8_t *device_name )
{
    uint8_t eir_cfg[EIR_DATA_LENGTH] = {0};
    uint8_t* p = eir_cfg;
    uint8_t name_len = strlen((char*)device_name);

    *p++ = (uint8_t)(name_len+1);                 /* Length */
    *p++ = (uint8_t)EIR_COMPLETE_LOCAL_NAME_TYPE; /* EIR Data Type */
    memcpy(p, device_name, name_len);   /* Name string */
    p += name_len;

    *p++ = sizeof(uuid_list)+1;
    *p++ = (uint8_t) EIR_COMPLETE_16BITS_UUID_TYPE;
    memcpy(p, uuid_list, sizeof(uuid_list));

    return wiced_bt_dev_write_eir(eir_cfg, EIR_DATA_LENGTH);
}


void bt_audio_sink_data_cb( wiced_bt_a2dp_sink_codec_t codec_type, wiced_bt_a2dp_sink_audio_data_t* p_audio_data )
{
    bt_audio_codec_data_t* audio = NULL;
    uint8_t* in_audio;
    uint16_t in_length;
    wiced_result_t result;

    p_audio_data->p_pkt->len--;
    p_audio_data->p_pkt->offset++;
    in_length = p_audio_data->p_pkt->len;

    APOLLO_BT_A2DP_SINK_PROFILING_WRITE();

#ifdef BT_AUDIO_USE_MEM_POOL
    audio = bt_buffer_pool_allocate_buffer(g_p_apollo_bt_a2dp_sink->mem_pool);
#else
    audio = malloc( sizeof(bt_audio_codec_data_t)+in_length );
#endif
    if(audio == NULL)
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"bt_audio_sink_data_cb: buffer allocation failed !\n");
        return;
    }

    in_audio = ((uint8_t*)(p_audio_data->p_pkt+1))+p_audio_data->p_pkt->offset;
    memcpy(audio->data, in_audio, in_length);
    audio->length = in_length;
    audio->offset = 0;
    result = bt_audio_write_to_decoder_queue(g_p_apollo_bt_a2dp_sink, audio);
    if (result != WICED_SUCCESS)
    {
        /*
         * Failure writing to the decoder queue. Free the audio buffer
         * before we return so we don't leak memory.
         */
#ifdef BT_AUDIO_USE_MEM_POOL
        bt_buffer_pool_free_buffer(audio);
#else
        free(audio);
#endif
    }
}


/*Audio Sink Functions*/
static void bt_audio_sink_control_cb( wiced_bt_a2dp_sink_event_t event, wiced_bt_a2dp_sink_event_data_t* p_data)
{
    switch (event)
    {
        case WICED_BT_A2DP_SINK_CONNECT_EVT:
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"bt_audio_sink_control_cb:CONNECT EVENT \nstatus = %d\n", p_data->connect.result);

            if (p_data->connect.result == WICED_SUCCESS)
            {
                memcpy(g_p_apollo_bt_a2dp_sink->remote_address, p_data->connect.bd_addr, sizeof(wiced_bt_device_address_t));
#ifdef USE_WICED_HCI
                wiced_bt_a2dp_sink_change_route(g_p_apollo_bt_a2dp_sink->remote_address, get_bt_a2dp_route(g_apollo_bt_a2dp_sink.user_params.output_mode));
#endif /* USE_WICED_HCI */
                apollo_bt_remote_control_peer_address_set(p_data->connect.bd_addr);
                wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"bt_audio_sink_control_cb: Remote Bluetooth Address: [%02X:%02X:%02X:%02X:%02X:%02X]\n",
                              p_data->connect.bd_addr[0], p_data->connect.bd_addr[1], p_data->connect.bd_addr[2],
                              p_data->connect.bd_addr[3], p_data->connect.bd_addr[4], p_data->connect.bd_addr[5]);
                if ( g_p_apollo_bt_a2dp_sink->user_params.event_cbf != NULL )
                {
                    g_p_apollo_bt_a2dp_sink->user_params.event_cbf( APOLLO_BT_A2DP_EVENT_CONNECTED, NULL, g_p_apollo_bt_a2dp_sink->user_params.user_context );
                }
                g_p_apollo_bt_a2dp_sink->is_connected = WICED_TRUE;
            }
        }
        break;

        case WICED_BT_A2DP_SINK_DISCONNECT_EVT:
        {
            wiced_bt_device_address_t bda;

            memcpy( bda, p_data->disconnect.bd_addr, sizeof(wiced_bt_device_address_t) );
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"bt_audio_sink_control_cb:DISCONNECTED EVENT \nreason = %d \nRemote Bluetooth Address: [%02X:%02X:%02X:%02X:%02X:%02X]\n",
                                          p_data->disconnect.result, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
            apollo_bt_remote_control_peer_address_reset();
            memset(g_p_apollo_bt_a2dp_sink->remote_address, 0, sizeof(wiced_bt_device_address_t));
            if ( g_p_apollo_bt_a2dp_sink->user_params.event_cbf != NULL )
            {
                g_p_apollo_bt_a2dp_sink->user_params.event_cbf( APOLLO_BT_A2DP_EVENT_DISCONNECTED, NULL, g_p_apollo_bt_a2dp_sink->user_params.user_context );
            }
#ifdef BT_AUDIO_USE_MEM_POOL
            bt_buffer_pool_print_debug_info( g_p_apollo_bt_a2dp_sink->mem_pool );
#endif
            g_p_apollo_bt_a2dp_sink->is_connected = WICED_FALSE;
            wiced_rtos_set_event_flags( &g_p_apollo_bt_a2dp_sink->events, APOLLO_BT_A2DP_SINK_EVENT_DISCONNECTED );
        }
        break;

        case WICED_BT_A2DP_SINK_START_IND_EVT:
        case WICED_BT_A2DP_SINK_START_CFM_EVT:
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"bt_audio_sink_control_cb:STREAM START EVENT \nstatus = %d\n", p_data->start_ind.result);
            bt_audio_configure_decoder( g_p_apollo_bt_a2dp_sink, &g_p_apollo_bt_a2dp_sink->decoder_config );
#ifdef USE_WICED_HCI
            if (event == WICED_BT_A2DP_SINK_START_IND_EVT)
            {
                wiced_bt_a2dp_sink_start_rsp(g_p_apollo_bt_a2dp_sink->remote_address, A2D_SUCCESS);
            }
#endif
            if ( g_p_apollo_bt_a2dp_sink->user_params.event_cbf != NULL )
            {
                g_p_apollo_bt_a2dp_sink->user_params.event_cbf( APOLLO_BT_A2DP_EVENT_STARTED, NULL, g_p_apollo_bt_a2dp_sink->user_params.user_context );
            }
        }
        break;

        case WICED_BT_A2DP_SINK_SUSPEND_EVT:
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"bt_audio_sink_control_cb:STREAM SUSPEND EVENT \nstatus = %d\n", p_data->suspend.result);
            if ( g_p_apollo_bt_a2dp_sink->user_params.event_cbf != NULL )
            {
                g_p_apollo_bt_a2dp_sink->user_params.event_cbf( APOLLO_BT_A2DP_EVENT_STOPPED, NULL, g_p_apollo_bt_a2dp_sink->user_params.user_context );
            }
            bt_audio_reset_decoder_config( g_p_apollo_bt_a2dp_sink );
#ifdef BT_AUDIO_USE_MEM_POOL
            bt_buffer_pool_print_debug_info( g_p_apollo_bt_a2dp_sink->mem_pool );
#endif
            apollo_bt_a2dp_sink_profiling_dump();
        }
        break;

        case WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT:
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"bt_audio_sink_control_cb:CODEC CONFIG \nCODEC ID:0x%02X  FS:0x%02X  CH_MODE:0x%02X  BLOCK_LEN:0x%02X  NUM_SUBBANDS:0x%02X  ALLOC_METHOD:0x%02X  MAX_BITPOOL:0x%02X  MIN_BITPOOL:0x%02X\n",
                          p_data->codec_config.codec_id, p_data->codec_config.cie.sbc.samp_freq, p_data->codec_config.cie.sbc.ch_mode, p_data->codec_config.cie.sbc.block_len,
                          p_data->codec_config.cie.sbc.num_subbands, p_data->codec_config.cie.sbc.alloc_mthd, p_data->codec_config.cie.sbc.max_bitpool, p_data->codec_config.cie.sbc.min_bitpool);

            memcpy( &g_p_apollo_bt_a2dp_sink->decoder_config, &p_data->codec_config, sizeof(g_p_apollo_bt_a2dp_sink->decoder_config) );

            bt_audio_get_config_from_cie(&p_data->codec_config, &g_p_apollo_bt_a2dp_sink->audio_config);

            memset( &g_p_apollo_bt_a2dp_sink->user_event_data.codec_config, 0, sizeof(g_p_apollo_bt_a2dp_sink->user_event_data.codec_config));
            g_p_apollo_bt_a2dp_sink->user_event_data.codec_config.sample_rate     = g_p_apollo_bt_a2dp_sink->audio_config.sample_rate;
            g_p_apollo_bt_a2dp_sink->user_event_data.codec_config.bits_per_sample = g_p_apollo_bt_a2dp_sink->audio_config.bits_per_sample;
            g_p_apollo_bt_a2dp_sink->user_event_data.codec_config.channels        = g_p_apollo_bt_a2dp_sink->audio_config.channels;
            g_p_apollo_bt_a2dp_sink->user_event_data.codec_config.frame_size      = g_p_apollo_bt_a2dp_sink->audio_config.frame_size;

            if ( g_p_apollo_bt_a2dp_sink->user_params.event_cbf != NULL )
            {
                g_p_apollo_bt_a2dp_sink->user_params.event_cbf( APOLLO_BT_A2DP_EVENT_CODEC_CONFIG, &g_p_apollo_bt_a2dp_sink->user_event_data, g_p_apollo_bt_a2dp_sink->user_params.user_context );
            }
        }
        break;
    }
}


wiced_result_t apollo_bt_a2dp_sink_init( apollo_bt_a2dp_sink_init_params_t *params )
{
    wiced_result_t                 result        = WICED_ERROR;
    apollo_bt_paired_device_info_t out_device_unused;
#ifdef WICED_DCT_INCLUDE_BT_CONFIG
    /* Configure the Device Name and Class of Device from the DCT */
    platform_dct_bt_config_t*      dct_bt_config = NULL;
#endif

    wiced_action_jump_when_not_true(g_apollo_bt_a2dp_sink.is_initialized == WICED_FALSE, _exit,
                                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Apollo BT A2DP Sink: already initialized !\n"));

    g_apollo_bt_a2dp_sink.is_connected = WICED_FALSE;

    result = wiced_rtos_init_event_flags(&g_apollo_bt_a2dp_sink.events);
    wiced_action_jump_when_not_true(result == WICED_SUCCESS, _exit,
                                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Apollo BT A2DP Sink: wiced_rtos_init_event_flags() failed !\n"));

    g_apollo_bt_a2dp_sink.p_event_data = apollo_bt_service_get_management_evt_data();
    wiced_action_jump_when_not_true(g_apollo_bt_a2dp_sink.p_event_data != NULL, _exit,
                                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Apollo BT A2DP Sink: BT service is not initialized !\r\n"));
    wiced_action_jump_when_not_true(g_apollo_bt_a2dp_sink.p_event_data->enabled.status == WICED_BT_SUCCESS, _exit,
                                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Apollo BT A2DP Sink: BT service has not successfully started !\n"));

    g_p_apollo_bt_a2dp_sink = &g_apollo_bt_a2dp_sink;

    memcpy( &g_p_apollo_bt_a2dp_sink->user_params, params, sizeof(g_p_apollo_bt_a2dp_sink->user_params) );
    g_p_apollo_bt_a2dp_sink->codec_config = bt_audio_codec_config;
    g_p_apollo_bt_a2dp_sink->cfg_settings = wiced_bt_cfg_settings;

#ifdef BT_AUDIO_USE_MEM_POOL
    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "Apollo BT A2DP Sink: using BT MEMPOOL instead of malloc()\r\n");
    result = bt_buffer_pool_init(&g_p_apollo_bt_a2dp_sink->mem_pool, MEM_POOL_BUFFER_COUNT, MEM_POOL_BUFFER_SIZE);
    wiced_action_jump_when_not_true(result == WICED_SUCCESS, _exit,
                                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Apollo BT A2DP Sink: BT buffer pool init failed !\n"));
#endif

    result = bt_audio_decoder_context_init( g_p_apollo_bt_a2dp_sink );
    wiced_action_jump_when_not_true(result == WICED_SUCCESS, _exit,
                                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Apollo BT A2DP Sink: audio decoder init failed !\n"));

#ifdef WICED_DCT_INCLUDE_BT_CONFIG
    /* Read config */
    result = wiced_dct_read_lock( (void**) &dct_bt_config, WICED_TRUE, DCT_BT_CONFIG_SECTION, 0, sizeof(platform_dct_bt_config_t) );
    if ( result != WICED_SUCCESS )
    {
        result = bt_audio_write_eir( ( uint8_t* )BLUETOOTH_DEVICE_NAME );
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Apollo BT A2DP Sink: wiced_dct_read_lock(DCT_BT_CONFIG_SECTION) failed !!\n");
    }
    else
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"Apollo BT A2DP Sink: WICED DCT BT ADDR [%x:%x:%x:%x:%x:%x] \r\n",
                      dct_bt_config->bluetooth_device_address[0], dct_bt_config->bluetooth_device_address[1],
                      dct_bt_config->bluetooth_device_address[2], dct_bt_config->bluetooth_device_address[3],
                      dct_bt_config->bluetooth_device_address[4], dct_bt_config->bluetooth_device_address[5]);
        result = bt_audio_write_eir(dct_bt_config->bluetooth_device_name);
        wiced_dct_read_unlock( (void*) dct_bt_config, WICED_TRUE );
    }
#else
    result = bt_audio_write_eir( ( uint8_t* )BLUETOOTH_DEVICE_NAME );
#endif

    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"Apollo BT A2DP Sink: wiced_bt_dev_write_eir() result = 0x%x\n", (unsigned int)result);

    wiced_bt_dev_read_local_addr( g_p_apollo_bt_a2dp_sink->local_address );
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO,"Apollo BT A2DP Sink: Local Bluetooth Address is [%02X:%02X:%02X:%02X:%02X:%02X]\n",
                  g_p_apollo_bt_a2dp_sink->local_address[0], g_p_apollo_bt_a2dp_sink->local_address[1],
                  g_p_apollo_bt_a2dp_sink->local_address[2], g_p_apollo_bt_a2dp_sink->local_address[3],
                  g_p_apollo_bt_a2dp_sink->local_address[4], g_p_apollo_bt_a2dp_sink->local_address[5]);

    result = wiced_bt_sdp_db_init( (UINT8 *)sdp_database, wiced_bt_sdp_db_size );
    wiced_log_msg( WLF_AUDIO, WICED_LOG_INFO, "Apollo BT A2DP Sink: wiced_bt_sdp_db_init() result (bool) = 0x%x\n", ( unsigned int ) result );

    result = wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE,
                                              g_p_apollo_bt_a2dp_sink->cfg_settings.br_edr_scan_cfg.inquiry_scan_window,
                                              g_p_apollo_bt_a2dp_sink->cfg_settings.br_edr_scan_cfg.inquiry_scan_interval);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Apollo BT A2DP Sink: discoverability result = 0x%x\r\n", (unsigned int)result);

    result = wiced_bt_dev_set_connectability(BTM_CONNECTABLE,
                                             g_p_apollo_bt_a2dp_sink->cfg_settings.br_edr_scan_cfg.page_scan_window,
                                             g_p_apollo_bt_a2dp_sink->cfg_settings.br_edr_scan_cfg.page_scan_interval);
    wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Apollo BT A2DP Sink: connectability result = 0x%x\r\n", (unsigned int)result);

    result = wiced_bt_a2dp_sink_init(&g_p_apollo_bt_a2dp_sink->codec_config, (wiced_bt_a2dp_sink_control_cb_t) bt_audio_sink_control_cb, bt_audio_sink_data_cb);
    g_apollo_bt_a2dp_sink.is_initialized = WICED_TRUE;
    if ( result == WICED_SUCCESS )
    {
        if ( apollo_bt_remote_control_init() != WICED_SUCCESS )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Apollo BT A2DP Sink: wiced_bt_remote_control_init() failed with result = 0x%x\n", (unsigned int)result);
        }
    }
    wiced_action_jump_when_not_true(result == WICED_SUCCESS, _exit,
                                    wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Apollo BT A2DP Sink: wiced_bt_a2dp_sink_init() failed with result = 0x%x!\n", (unsigned int)result));

    if ( apollo_bt_nv_get_device_info_by_index(0, &out_device_unused) == WICED_SUCCESS )
    {
        apollo_bt_service_reconnection_timer_start();
    }

 _exit:
    g_apollo_bt_a2dp_sink.init_result = result;
    return result;
}


wiced_result_t apollo_bt_a2dp_sink_deinit( void )
{
    wiced_result_t result = WICED_ERROR;
    uint32_t       events;

    wiced_action_jump_when_not_true( g_apollo_bt_a2dp_sink.is_initialized == WICED_TRUE, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"Apollo BT A2DP Sink: not yet initialized !\n") );

    apollo_bt_remote_control_deinit();

    /* clear events */
    events = 0;
    wiced_rtos_wait_for_event_flags(&g_apollo_bt_a2dp_sink.events, APOLLO_BT_A2DP_SINK_EVENT_ALL,
                                    &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);

    result = wiced_bt_a2dp_sink_deinit();

    bt_audio_decoder_context_deinit( g_p_apollo_bt_a2dp_sink );

    if ( g_apollo_bt_a2dp_sink.is_connected == WICED_FALSE )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Apollo BT A2DP Sink: not CONNECTED ! !\n");
    }
    else
    {
        wiced_result_t wait_result;

        /* we're still connected to remote peer; wait for DISCONNECTED event for a little while */
        events = 0;
        wait_result = wiced_rtos_wait_for_event_flags(&g_apollo_bt_a2dp_sink.events, APOLLO_BT_A2DP_SINK_EVENT_ALL,
                                                      &events, WICED_TRUE, WAIT_FOR_ANY_EVENT, APOLLO_BT_A2DP_SINK_EVENT_TIMEOUT_MSECS);
        if ( events & APOLLO_BT_A2DP_SINK_EVENT_DISCONNECTED )
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Apollo BT A2DP Sink: got DISCONNECTED event !\n");
        }
        else
        {
            wiced_log_msg(WLF_AUDIO, WICED_LOG_INFO, "Apollo BT A2DP Sink: didn't get DISCONNECTED event (%d)!\n", (int)wait_result);
        }
    }

    wiced_rtos_deinit_event_flags(&g_apollo_bt_a2dp_sink.events);

#ifdef BT_AUDIO_USE_MEM_POOL
    bt_buffer_pool_deinit(g_p_apollo_bt_a2dp_sink->mem_pool);
#endif

    memset( g_p_apollo_bt_a2dp_sink, 0, sizeof(g_apollo_bt_a2dp_sink) );
    g_p_apollo_bt_a2dp_sink = NULL;
 _exit:
    return result;

}


wiced_result_t apollo_bt_a2dp_sink_connect( void )
{
    wiced_result_t                 result     = WICED_ERROR;
    apollo_bt_paired_device_info_t out_device;

    wiced_action_jump_when_not_true( (g_apollo_bt_a2dp_sink.is_initialized == WICED_TRUE) && (g_apollo_bt_a2dp_sink.init_result == WICED_SUCCESS), _exit,
                                     wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"Apollo BT A2DP Sink: not yet initialized !\n") );

    result = apollo_bt_nv_get_device_info_by_index(0, &out_device);
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE,"Apollo BT A2DP Sink: no previously connected device exist !\n") );

    result = wiced_bt_a2dp_sink_connect(out_device.device_link.bd_addr);
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, wiced_log_msg(WLF_AUDIO, WICED_LOG_NOTICE,"Apollo BT A2DP Sink: wiced_bt_a2dp_sink_connect() failed !\n"));

 _exit:
    return result;
}


wiced_result_t apollo_bt_a2dp_sink_set_ouput_mode( apollo_bt_a2dp_sink_output_mode_t output_mode )
{
    wiced_result_t result = WICED_ERROR;

#ifdef USE_WICED_HCI
    wiced_action_jump_when_not_true( (g_apollo_bt_a2dp_sink.is_initialized == WICED_TRUE) && (g_apollo_bt_a2dp_sink.init_result == WICED_SUCCESS), _exit,
                                      wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR,"Apollo BT A2DP Sink: not yet initialized !\n") );

    result = WICED_SUCCESS;
    if (output_mode != g_apollo_bt_a2dp_sink.user_params.output_mode)
    {
        result = wiced_bt_a2dp_sink_change_route(g_p_apollo_bt_a2dp_sink->remote_address, get_bt_a2dp_route(output_mode));
        if (result == WICED_SUCCESS)
        {
            g_apollo_bt_a2dp_sink.user_params.output_mode = output_mode;
        }
    }

    _exit:
#else /* !USE_WICED_HCI */
    result = WICED_UNSUPPORTED;
#endif /* USE_WICED_HCI */
    return result;
}


apollo_bt_a2dp_sink_t *apollo_bt_a2dp_get_context( void )
{
    return g_p_apollo_bt_a2dp_sink;
}
