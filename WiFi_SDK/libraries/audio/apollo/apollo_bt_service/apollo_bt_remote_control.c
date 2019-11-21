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
 * Bluetooth Remote Control Service
 *
 */

#include <stdlib.h>
#include "wiced_rtos.h"
#include "wiced_result.h"
#include "wiced_log.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_remote_control.h"
#include "apollo_bt_a2dp_sink_private.h"
#include "apollo_bt_service.h"
#include "apollo_bt_main_service_private.h"
#include "apollo_bt_remote_control_private.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define AVRC_LOCAL_FEATURES (REMOTE_CONTROL_FEATURE_CONTROLLER | REMOTE_CONTROL_FEATURE_TARGET)

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
 *                 Global Variables
 ******************************************************/

static apollo_bt_rc_t  g_apollo_bt_rc;

static uint8_t bt_rc_cmd_lookup[APOLLO_BT_REMOTE_CONTROL_CMD_MAX] =
{
    AVRC_ID_VOL_UP,
    AVRC_ID_VOL_DOWN,
    AVRC_ID_MUTE,
    AVRC_ID_PLAY,
    AVRC_ID_STOP,
    AVRC_ID_PAUSE,
    AVRC_ID_REWIND,
    AVRC_ID_FAST_FOR,
    AVRC_ID_FORWARD,
    AVRC_ID_BACKWARD
};

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

static inline char* remote_control_event_id_str ( uint8_t event_id )
{
    switch( event_id )
    {
        case AVRC_EVT_PLAY_STATUS_CHANGE:
            return "PLAY STATUS CHANGED";
        case AVRC_EVT_TRACK_CHANGE:
            return "TRACK CHANGED";
        case AVRC_EVT_TRACK_REACHED_END:
            return "TRACK REACHED END";
        case AVRC_EVT_TRACK_REACHED_START:
            return "TRACK REACHED START";
        case AVRC_EVT_PLAY_POS_CHANGED:
            return "PLAY POS CHANGED";
        case AVRC_EVT_NOW_PLAYING_CHANGE:
            return "PLAYING CHANGE";
        case AVRC_EVT_VOLUME_CHANGE:
            return "VOLUME CHANGED";
        default:
            return "EVENT NOT HANDLED";
        break;
    }
    return NULL;
}


static inline char* remote_control_play_status_str ( uint8_t status )
{
    switch( status )
    {
        case AVRC_PLAYSTATE_STOPPED:
            return "PLAYSTATE STOPPED";
        case AVRC_PLAYSTATE_PLAYING:
            return "PLAYSTATE PLAYING";
        case AVRC_PLAYSTATE_PAUSED:
            return "PLAYSTATE PAUSED";
        case AVRC_PLAYSTATE_FWD_SEEK:
            return "PLAYSTATE FWD SEEK";
        case AVRC_PLAYSTATE_REV_SEEK:
            return "PLAYSTATE REV SEEK";
        case AVRC_PLAYSTATE_ERROR:
            return "PLAYSTATE ERROR";
        default:
            return "EVENT NOT HANDLED";
        break;
    }
    return NULL;
}


static inline void bt_update_audio_player_volume( uint8_t new_volume )
{
    if ( g_apollo_bt_rc.a2dp_sink_ctx->user_params.rc_event_cbf != NULL )
    {
        apollo_bt_remote_control_event_data_t bt_volume =
        {
            .volume =
            {
                .volume_current = new_volume,
                .volume_max     = BT_AUDIO_VOLUME_MAX,
                .volume_min     = BT_AUDIO_VOLUME_MIN,
                .volume_step    = BT_AUDIO_VOLUME_STEP,
            },
        };

        g_apollo_bt_rc.a2dp_sink_ctx->user_params.rc_event_cbf( APOLLO_BT_REMOTE_CONTROL_EVENT_ABSOLUTE_VOLUME, &bt_volume, g_apollo_bt_rc.a2dp_sink_ctx->user_params.user_context );
    }
}


static void bt_audio_remote_control_connection_state_cback( wiced_bt_device_address_t remote_addr, wiced_result_t status, wiced_bt_remote_control_connection_state_t connection_state, uint32_t peer_features )
{
    wiced_bt_device_address_t null_addr = {0,};

    if ( status != WICED_SUCCESS )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: something went wrong, status = %d\n", __func__, status);
    }

    if ( connection_state == REMOTE_CONTROL_CONNECTED )
    {
        if ( memcmp(g_apollo_bt_rc.rc_peer_address, null_addr, sizeof(wiced_bt_device_address_t)) == 0 )
        {
            memcpy(g_apollo_bt_rc.rc_peer_address, remote_addr, sizeof(wiced_bt_device_address_t)) ;
        }
        else
        {
            if ( memcmp(remote_addr, g_apollo_bt_rc.rc_peer_address, sizeof(wiced_bt_device_address_t)) != 0 )
            {
                wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: AVRCP and A2DP peer address don't match !!!\n", __func__);
                return;
            }
        }

        g_apollo_bt_rc.rc_peer_features = peer_features;
        g_apollo_bt_rc.rc_is_connected  = WICED_TRUE;
        if ( g_apollo_bt_rc.a2dp_sink_ctx->user_params.rc_event_cbf != NULL )
        {
            g_apollo_bt_rc.a2dp_sink_ctx->user_params.rc_event_cbf( APOLLO_BT_REMOTE_CONTROL_EVENT_CONNECTED, NULL, g_apollo_bt_rc.a2dp_sink_ctx->user_params.user_context );
        }
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "%s: AVRCP is CONNECTED !\n", __func__);
    }
    else /* REMOTE_CONTROL_DISCONNECTED */
    {
        memset( g_apollo_bt_rc.rc_peer_address, 0, sizeof(wiced_bt_device_address_t) );
        g_apollo_bt_rc.rc_peer_features = 0;
        g_apollo_bt_rc.rc_is_connected  = WICED_FALSE;
        g_apollo_bt_rc.rc_volume        = BT_AUDIO_VOLUME_MAX;
        if ( g_apollo_bt_rc.a2dp_sink_ctx->user_params.rc_event_cbf != NULL )
        {
            g_apollo_bt_rc.a2dp_sink_ctx->user_params.rc_event_cbf( APOLLO_BT_REMOTE_CONTROL_EVENT_DISCONNECTED, NULL, g_apollo_bt_rc.a2dp_sink_ctx->user_params.user_context );
        }
        wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG0, "%s: AVRCP is DISCONNECTED !\n", __func__);
    }

    return;
}


static void bt_audio_remote_control_cmd_cback( wiced_bt_device_address_t remote_addr, wiced_bt_avrc_command_t *avrc_cmd )
{

    if ( memcmp(remote_addr, g_apollo_bt_rc.rc_peer_address, sizeof(wiced_bt_device_address_t)) != 0 )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: AVRCP and A2DP peer address don't match !!!\n", __func__);
        return;
    }

    switch ( avrc_cmd->pdu )
    {
        case AVRC_PDU_SET_ABSOLUTE_VOLUME:
        {
            bt_update_audio_player_volume(avrc_cmd->volume.volume);
            g_apollo_bt_rc.rc_volume = avrc_cmd->volume.volume;
            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "remote_control_cmd_cback AVRC_PDU_SET_ABSOLUTE_VOLUME vol:%d \n", avrc_cmd->volume.volume);

        }
            break;

        default:
            break;
    }
}


/** Response callback from peer device for AVRCP commands */
static void bt_audio_remote_control_rsp_cback( wiced_bt_device_address_t remote_addr, wiced_bt_avrc_response_t *avrc_rsp )
{
    if ( memcmp(remote_addr, g_apollo_bt_rc.rc_peer_address, sizeof(wiced_bt_device_address_t)) != 0 )
    {
        wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "%s: AVRCP and A2DP peer address don't match !!!\n", __func__);
        return;
    }

    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "Apollo BT AVRCP: %s pdu [%d] status [%d]\n", __func__, avrc_rsp->pdu, avrc_rsp->rsp.status );

    switch ( avrc_rsp->pdu )
    {
        case AVRC_PDU_GET_PLAY_STATUS:
        {
            wiced_bt_avrc_get_play_status_rsp_t *play_status = ( wiced_bt_avrc_get_play_status_rsp_t * )avrc_rsp;

            g_apollo_bt_rc.track_duration = play_status->song_len;

            if ( g_apollo_bt_rc.a2dp_sink_ctx->user_params.rc_event_cbf != NULL )
            {
                apollo_bt_remote_control_event_data_t bt_play =
                {
                    .playback_status =
                    {
                        .position = play_status->song_pos,
                        .duration = g_apollo_bt_rc.track_duration,
                    },
                };

                g_apollo_bt_rc.a2dp_sink_ctx->user_params.rc_event_cbf( APOLLO_BT_REMOTE_CONTROL_EVENT_TRACK_PLAYBACK_STATUS, &bt_play, g_apollo_bt_rc.a2dp_sink_ctx->user_params.user_context );
            }

            wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "Apollo BT AVRCP: %s GET PLAY STATUS: play status: %s song_pos [%lu] song_len [%lu]\n",__func__,
                          remote_control_play_status_str(play_status->play_status), play_status->song_pos, play_status->song_len );
        }
        break;

        case AVRC_PDU_GET_ELEMENT_ATTR:
        {
            wiced_bt_avrc_get_elem_attrs_rsp_t *elements = ( wiced_bt_avrc_get_elem_attrs_rsp_t* )avrc_rsp;

            if ( elements->num_attr > 1 )
            {
                uint32_t strlen_min = MIN(elements->p_attrs[0].name.str_len, (sizeof(g_apollo_bt_rc.song_title) - 1));

                memcpy( g_apollo_bt_rc.song_title, elements->p_attrs[0].name.p_str, strlen_min );
                g_apollo_bt_rc.song_title[strlen_min] = '\0';

                strlen_min = MIN(elements->p_attrs[1].name.str_len, (sizeof(g_apollo_bt_rc.song_artist) - 1));
                memcpy( g_apollo_bt_rc.song_artist, elements->p_attrs[1].name.p_str, strlen_min );
                g_apollo_bt_rc.song_artist[strlen_min] = '\0';

                if ( g_apollo_bt_rc.a2dp_sink_ctx->user_params.rc_event_cbf != NULL )
                {
                    apollo_bt_remote_control_event_data_t bt_meta =
                    {
                        .metadata =
                        {
                            .title  = g_apollo_bt_rc.song_title,
                            .artist = g_apollo_bt_rc.song_artist,
                        },
                    };

                    g_apollo_bt_rc.a2dp_sink_ctx->user_params.rc_event_cbf( APOLLO_BT_REMOTE_CONTROL_EVENT_TRACK_METADATA, &bt_meta, g_apollo_bt_rc.a2dp_sink_ctx->user_params.user_context );
                }
            }
        }
        break;

        case AVRC_PDU_REGISTER_NOTIFICATION:
        {
            wiced_bt_avrc_reg_notif_rsp_t *reg_notif = NULL;
            reg_notif = (wiced_bt_avrc_reg_notif_rsp_t *)avrc_rsp;

            switch ( reg_notif->event_id )
            {
                case AVRC_EVT_PLAY_STATUS_CHANGE:
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "Apollo BT AVRCP: %s REG_NOTIF: AVRC_EVT_PLAY_STATUS_CHANGE\n",__func__);
                    wiced_bt_remote_control_get_play_status_cmd( g_apollo_bt_rc.rc_peer_address );
                }
                break;

                case AVRC_EVT_TRACK_CHANGE:
                {
                    wiced_bt_avrc_uid_t element_id = {0, };
                    uint32_t attrs[] =
                    {
                            AVRC_MEDIA_ATTR_ID_TITLE,
                            AVRC_MEDIA_ATTR_ID_ARTIST,
                            AVRC_MEDIA_ATTR_ID_ALBUM,
                            AVRC_MEDIA_ATTR_ID_NUM_TRACKS,
                            AVRC_MEDIA_ATTR_ID_TRACK_NUM,
                            AVRC_MEDIA_ATTR_ID_PLAYING_TIME,
                            AVRC_MEDIA_ATTR_ID_GENRE
                    };
                    uint8_t num_attr = (uint8_t)((uint32_t)(sizeof(attrs) / sizeof(attrs[0])));

                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "Apollo BT AVRCP: %s REG_NOTIF: AVRC_EVT_TRACK_CHANGE\n",__func__);
                    wiced_bt_remote_control_get_element_attr_cmd( g_apollo_bt_rc.rc_peer_address, element_id, num_attr, attrs );
                }
                break;

                case AVRC_EVT_NOW_PLAYING_CHANGE:
                {
                    wiced_bt_avrc_uid_t element_id = {0};
                    uint32_t attrs    = AVRC_MEDIA_ATTR_ID_TITLE;
                    uint8_t  num_attr = 1;

                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "Apollo BT AVRCP: %s REG_NOTIF: AVRC_EVT_NOW_PLAYING_CHANGE\n",__func__);
                    wiced_bt_remote_control_get_element_attr_cmd( g_apollo_bt_rc.rc_peer_address, element_id, num_attr, &attrs );
                }
                break;

                case AVRC_EVT_TRACK_REACHED_END:
                case AVRC_EVT_TRACK_REACHED_START:
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "Apollo BT AVRCP: %s event-id [%s] : \n",__func__,
                                  remote_control_event_id_str(reg_notif->event_id));
                    wiced_bt_remote_control_get_play_status_cmd( g_apollo_bt_rc.rc_peer_address );
                }
                break;

                case AVRC_EVT_PLAY_POS_CHANGED:
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG4, "Apollo BT AVRCP: %s event-id [%s] : \n",__func__,
                                  remote_control_event_id_str(reg_notif->event_id));
                    wiced_bt_remote_control_get_play_status_cmd( g_apollo_bt_rc.rc_peer_address );
                }
                break;

                case AVRC_EVT_VOLUME_CHANGE:
                {
                    wiced_log_msg(WLF_AUDIO, WICED_LOG_DEBUG1, "Apollo BT AVRCP: %s event-id [%s] : volume: %d\n",__func__,
                                  remote_control_event_id_str(reg_notif->event_id), (int)reg_notif->param.volume);

                    g_apollo_bt_rc.rc_volume = reg_notif->param.volume;
                }
                break;
            }
        }
        break;

    }
}


static inline wiced_result_t bt_audio_a2dp_send_passthru_command( uint8_t cmd )
{
    wiced_result_t result;

    result = wiced_bt_remote_control_send_pass_through_cmd( g_apollo_bt_rc.rc_peer_address, cmd, AVRC_STATE_PRESS, 0, NULL );
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit,
                                     wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "wiced_bt_remote_control_send_pass_through_cmd(0x%x) failed with 0x%lx !\n", cmd, (uint32_t)result) );
    wiced_rtos_delay_milliseconds( 50 );
    result = wiced_bt_remote_control_send_pass_through_cmd( g_apollo_bt_rc.rc_peer_address, cmd, AVRC_STATE_RELEASE, 0, NULL );

 _exit:
    return result;
}


static inline wiced_result_t bt_audio_a2dp_update_volume( uint8_t vol_level )
{
    wiced_result_t result;

    bt_update_audio_player_volume(vol_level);

    /* send volume to peer */
    result = wiced_bt_remote_control_set_volume_cmd( g_apollo_bt_rc.rc_peer_address, vol_level );

    return result;
}


static inline wiced_result_t bt_audio_a2dp_volume_up( void )
{
    uint8_t vol_level = g_apollo_bt_rc.rc_volume;
    wiced_result_t result;

    if ( vol_level == BT_AUDIO_VOLUME_MAX )
    {
        return WICED_SUCCESS;
    }

    if ( vol_level == BT_AUDIO_VOLUME_MIN )
    {
        vol_level = (BT_AUDIO_VOLUME_STEP - 1);
    }
    else
    {
        vol_level += BT_AUDIO_VOLUME_STEP;
    }

    if ( vol_level > BT_AUDIO_VOLUME_MAX )
    {
        vol_level = BT_AUDIO_VOLUME_MAX;
    }

    result = bt_audio_a2dp_update_volume( vol_level );
    g_apollo_bt_rc.rc_volume = vol_level;

    return result;
}


static inline wiced_result_t bt_audio_a2dp_volume_down( void )
{
    wiced_result_t result;
    uint8_t vol_level = g_apollo_bt_rc.rc_volume;

    if ( vol_level == BT_AUDIO_VOLUME_MIN )
    {
        return WICED_SUCCESS;
    }

    if ( (vol_level - BT_AUDIO_VOLUME_STEP) < (BT_AUDIO_VOLUME_MIN) )
    {
        vol_level = BT_AUDIO_VOLUME_MIN ;
    }
    else
    {
        vol_level -= BT_AUDIO_VOLUME_STEP;
    }

    result = bt_audio_a2dp_update_volume( vol_level );
    g_apollo_bt_rc.rc_volume = vol_level;

    return result;
}


wiced_result_t apollo_bt_remote_control_init( void )
{
    wiced_result_t result;

    memset(&g_apollo_bt_rc, 0, sizeof(g_apollo_bt_rc));

    g_apollo_bt_rc.a2dp_sink_ctx = apollo_bt_a2dp_get_context();
    g_apollo_bt_rc.rc_volume     = BT_AUDIO_VOLUME_MAX;

    result = wiced_bt_remote_control_init( AVRC_LOCAL_FEATURES,
                                           bt_audio_remote_control_connection_state_cback,
                                           bt_audio_remote_control_cmd_cback,
                                           bt_audio_remote_control_rsp_cback );
    g_apollo_bt_rc.rc_is_initialized = WICED_TRUE;
    return result;
}


wiced_result_t apollo_bt_remote_control_deinit( void )
{
    wiced_result_t result;

    result = wiced_bt_remote_control_deinit();

    memset(&g_apollo_bt_rc, 0, sizeof(g_apollo_bt_rc));

    return result;
}


wiced_result_t apollo_bt_remote_control_send_command( apollo_bt_remote_control_command_t cmd )
{
    wiced_result_t result = WICED_ERROR;

    wiced_action_jump_when_not_true( (g_apollo_bt_rc.rc_is_initialized == WICED_TRUE) && (g_apollo_bt_rc.rc_is_connected == WICED_TRUE), _exit,
                                      wiced_log_msg(WLF_AUDIO, WICED_LOG_ERR, "Apollo BT remote control: not initialized or not connected to any peer\n") );

    switch ( cmd )
    {
        case APOLLO_BT_REMOTE_CONTROL_CMD_VOLUME_UP:
            result = bt_audio_a2dp_volume_up();
            break;

        case APOLLO_BT_REMOTE_CONTROL_CMD_VOLUME_DOWN:
            result = bt_audio_a2dp_volume_down();
            break;

        default:
            if ( cmd < APOLLO_BT_REMOTE_CONTROL_CMD_UNSUPPORTED )
            {
                result = bt_audio_a2dp_send_passthru_command(bt_rc_cmd_lookup[cmd]);
            }
            break;
    }

 _exit:
    return result;
}

void apollo_bt_remote_control_peer_address_reset( void )
{
#ifndef USE_WICED_HCI
    wiced_bt_remote_control_disconnect(g_apollo_bt_rc.rc_peer_address);
#endif /* USE_WICED_HCI */
    memset( g_apollo_bt_rc.rc_peer_address, 0, sizeof(g_apollo_bt_rc.rc_peer_address) );
}


void apollo_bt_remote_control_peer_address_set( wiced_bt_device_address_t peer_addr )
{
    memcpy( g_apollo_bt_rc.rc_peer_address, peer_addr, sizeof(g_apollo_bt_rc.rc_peer_address) );
#ifndef USE_WICED_HCI
    wiced_bt_remote_control_connect(peer_addr);
#endif /* USE_WICED_HCI */
}
