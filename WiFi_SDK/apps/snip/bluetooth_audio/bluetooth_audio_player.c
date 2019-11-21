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
#include "wiced.h"
#include "wiced_rtos.h"
#include "wiced_audio.h"
#include "bluetooth_audio.h"

#ifdef USE_MEM_POOL
#include "mem_pool.h"
#endif

#ifdef USE_AUDIO_PLL
#include "bluetooth_audio_pll_tuning.h"
#endif

/*****************************************************************************
**
**  Name:           bt_audio_wiced_audio_player.c
**
**  Description:    BTA AVK interface to the Wiced audio subsystem
**
*****************************************************************************/

/******************************************************
 *                      Macros
 ******************************************************/
#define VOLUME_CONVERSION(step,level,min)                    ((double)step*(double)level + min)

/******************************************************
 *                   Enumerations
 ******************************************************/
#ifdef USE_AUDIO_PLL
typedef enum
{
    AUDIO_PLL_CONSOLE_CMD_TARGET    = 0,
    AUDIO_PLL_CONSOLE_CMD_THRESHOLD,
    AUDIO_PLL_CONSOLE_CMD_LOG,
    AUDIO_PLL_CONSOLE_CMD_MAX,
} audio_pll_console_cmd_t;
#endif /* USE_AUDIO_PLL */

/******************************************************
 *                    Constants
 ******************************************************/
#define BT_AUDIO_DEFAULT_PERIOD_SIZE        ( 1024 )
#define BT_AUDIO_BUFFER_SIZE                ( 4*BT_AUDIO_DEFAULT_PERIOD_SIZE )
#define WAIT_TIME_DELTA                     (20)
#define BT_AUDIO_PLAYER_QUEUE_MAX_SIZE      (28)
#define BT_AUDIO_PLAYER_QUEUE_THRESHOLD     (10)
#define BT_AUDIO_BUFFER_UNDERRUN_THRESHOLD  ( 2*BT_AUDIO_DEFAULT_PERIOD_SIZE )

/* only used for initial sizing of PCM packet queue */
#define BT_AUDIO_DEFAULT_SAMPLE_RATE_HZ     (44100)
#define BT_AUDIO_DEFAULT_SAMPLE_SIZE_BYTE   (4)
#define BT_AUDIO_DEFAULT_SAMPLE_PER_PACKET  (128)

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
#ifdef USE_AUDIO_PLL
typedef struct
{
    char *cmd;
} cmd_lookup_t;
#endif /* USE_AUDIO_PLL */

/******************************************************
 *               Function Definitions
 ******************************************************/
static wiced_bool_t read_bluetooth_audio_data( uint8_t *buffer, uint16_t* size );

/******************************************************
 *               Variables Definitions
 ******************************************************/
bt_audio_context_t player;
#ifdef USE_MEM_POOL
extern bt_buffer_pool_handle_t  mem_pool;
#endif

#ifdef USE_AUDIO_PLL
static cmd_lookup_t command_lookup[AUDIO_PLL_CONSOLE_CMD_MAX] =
{
    { "target"    },
    { "threshold" },
    { "log"       },
};
#endif /* USE_AUDIO_PLL */

/******************************************************
 *               Function Definitions
 ******************************************************/

#ifdef USE_AUDIO_PLL
int audio_pll_console_command( int argc, char* argv[] )
{
    int i;
    int percent;
    int log_level;

    for ( i = 0; i < AUDIO_PLL_CONSOLE_CMD_MAX; ++i )
    {
        if ( strcmp(command_lookup[i].cmd, argv[0]) == 0 )
        {
            break;
        }
    }

    if ( i >= AUDIO_PLL_CONSOLE_CMD_MAX )
    {
        WPRINT_APP_ERROR ( ("Unrecognized command: %s\n", argv[0]) );
        return 0;
    }

    switch ( i )
    {
        case AUDIO_PLL_CONSOLE_CMD_TARGET:
            percent = atoi(argv[1]);
            if ( percent < 1 || percent > 100 )
            {
                WPRINT_APP_ERROR ( ("Buffer level target must be between 1% and 100% (inclusive)\n") );
            }
            else
            {
                player.audio_buffer_target_percent = percent;
            }
            break;

        case AUDIO_PLL_CONSOLE_CMD_THRESHOLD:
            percent = atoi(argv[1]);
            if ( percent < 1 || percent > 100 )
            {
                WPRINT_APP_ERROR ( ("Buffer playback threshold must be between 1% and 100% (inclusive)\n") );
            }
            else
            {
                player.audio_buffer_threshold_percent = percent;
            }
            break;

        case AUDIO_PLL_CONSOLE_CMD_LOG:
            log_level = atoi(argv[1]);
            if ( log_level < WICED_LOG_OFF || log_level > WICED_LOG_DEBUG4 )
            {
                WPRINT_APP_ERROR ( ("Log level must be between %d and %d (inclusive)\n", WICED_LOG_OFF, WICED_LOG_DEBUG4) );
            }
            else
            {
                wiced_log_set_facility_level(WLF_AUDIO, log_level);
            }
            break;

        default:
            break;
    }

    return 0;
}
#endif /* USE_AUDIO_PLL */

 wiced_bool_t is_bt_audio_player_initialized( void )
{
    return (wiced_bool_t)( player.state > BT_AUDIO_DEVICE_STATE_IDLE );
}

wiced_bool_t is_bt_audio_player_prepared( void )
{
    return (wiced_bool_t)( player.state == BT_AUDIO_DEVICE_STATE_CONFIGURED ||
                           player.state == BT_AUDIO_DEVICE_STATE_STOPPED );
}

wiced_result_t  bt_audio_init_player( void )
{
    wiced_result_t result = WICED_SUCCESS;

    if(player.state == BT_AUDIO_DEVICE_STATE_UNINITIALIZED)
    {
#ifdef USE_AUDIO_PLL
        player.audio_buffer_max_duration_msecs = BT_AUDIO_BUFFER_DEFAULT_MAX_DURATION_MSEC;
        player.audio_buffer_target_percent     = BT_AUDIO_BUFFER_DEFAULT_TARGET_PERCENT;
        player.audio_buffer_threshold_percent  = BT_AUDIO_BUFFER_DEFAULT_THRESHOLD_PERCENT;

        player.queue_max_entry_count           = (uint32_t) MILLISECONDS_TO_BYTES(player.audio_buffer_max_duration_msecs, BT_AUDIO_DEFAULT_SAMPLE_RATE_HZ, BT_AUDIO_DEFAULT_SAMPLE_SIZE_BYTE);
        player.queue_max_entry_count          /= (BT_AUDIO_DEFAULT_SAMPLE_SIZE_BYTE * BT_AUDIO_DEFAULT_SAMPLE_PER_PACKET );
        player.queue_max_entry_count++;

        if ( pll_tuner_create(&player) != WICED_SUCCESS )
        {
            WPRINT_APP_ERROR ( ("bt_audio_init_player: pll_tuner_create() failed with %d\n", result) );
        }
#else /* !USE_AUDIO_PLL */
        player.queue_max_entry_count = BT_AUDIO_PLAYER_QUEUE_MAX_SIZE;
#endif /* USE_AUDIO_PLL */

        player.queue_threshold_count = BT_AUDIO_PLAYER_QUEUE_THRESHOLD;

        result = wiced_rtos_init_queue(&player.queue, "PLAYER_QUEUE", sizeof(bt_audio_codec_data_t*), player.queue_max_entry_count);
        result = wiced_rtos_init_event_flags(&player.events);
        result = wiced_rtos_init_semaphore(&player.wait_for_cmd_completion);
        player.state = BT_AUDIO_DEVICE_STATE_IDLE;
    }
    WPRINT_APP_INFO ( ("bt_audio_init_player: pcm_queue_entry_count=%lu, result=%d\n", player.queue_max_entry_count, result) );
    return result;
}


wiced_result_t bt_audio_configure_player( bt_audio_config_t* p_audio_config )
{
    wiced_result_t    result;
    wiced_audio_config_t wiced_audio_conf = {0, };
    //WPRINT_APP_INFO(("bt_audio_configure_player: INIT\n"));

    if(p_audio_config == NULL)
        return WICED_BADARG;

    if(player.state != BT_AUDIO_DEVICE_STATE_IDLE)
        return WICED_ERROR;
#ifdef USE_HYBRID_MODE
     /* Initialize and configure audio framework for needed audio format */
    if ( p_audio_config->audio_route == WICED_BT_A2DP_ROUTE_COMPRESSED_TRANSPORT )
    {
        result = wiced_audio_init(PLATFORM_DEFAULT_AUDIO_OUTPUT, &player.bluetooth_audio_session_handle, BT_AUDIO_DEFAULT_PERIOD_SIZE);
    }
    else //Route I2S
    {
        result = wiced_audio_init(PLATFORM_DEFAULT_BT_AUDIO_OUTPUT, &player.bluetooth_audio_session_handle, BT_AUDIO_DEFAULT_PERIOD_SIZE);
    }
#else
    /* Initialize and configure audio framework for needed audio format */
    result = wiced_audio_init(PLATFORM_DEFAULT_AUDIO_OUTPUT, &player.bluetooth_audio_session_handle, BT_AUDIO_DEFAULT_PERIOD_SIZE);
#endif
    if( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("bt_audio_configure_player:  Error Initializing Wiced Audio Framework[err: %d]\n",result) );
        return result;
    }
#ifdef USE_HYBRID_MODE
    if ( p_audio_config->audio_route == WICED_BT_A2DP_ROUTE_COMPRESSED_TRANSPORT )
    {
        result = wiced_audio_create_buffer(player.bluetooth_audio_session_handle, BT_AUDIO_BUFFER_SIZE, NULL, NULL );
        if( result != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("bt_audio_configure_player: Error Registering Buffer to Wiced Audio Framework [err: %d]\n",result));
            return result;
        }
   }
#else
    result = wiced_audio_create_buffer(player.bluetooth_audio_session_handle, BT_AUDIO_BUFFER_SIZE, NULL, NULL );
    if( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("bt_audio_configure_player: Error Registering Buffer to Wiced Audio Framework [err: %d]\n",result));
        return result;
    }
#endif
   memcpy(&player.bluetooth_audio_config, p_audio_config, sizeof(bt_audio_config_t));
   WPRINT_APP_INFO(("bt_audio_configure_player: config sample_rate:%u channels:%d bps:%d\n", (unsigned int) player.bluetooth_audio_config.sample_rate,
                                            (int)player.bluetooth_audio_config.channels, (int)player.bluetooth_audio_config.bits_per_sample));

    wiced_audio_conf.sample_rate     = player.bluetooth_audio_config.sample_rate;
    wiced_audio_conf.channels        = player.bluetooth_audio_config.channels;
    wiced_audio_conf.bits_per_sample = player.bluetooth_audio_config.bits_per_sample;
    wiced_audio_conf.frame_size      = player.bluetooth_audio_config.frame_size;

    result = wiced_audio_configure( player.bluetooth_audio_session_handle, &wiced_audio_conf );
    if( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("bt_audio_configure_player: Error configuring Wiced Audio framework [err: %d]\n",result));
        return result;
    }
    player.state = BT_AUDIO_DEVICE_STATE_CONFIGURED;

    /* Update the Volume */
    result = bt_audio_update_player_volume( player.bluetooth_audio_config.volume );

    /*
     * Reset stats gathering
     */

    player.buffer_unavail_count    = 0;
    player.underrun_count          = 0;
    player.overrun_count           = 0;
    player.silence_insertion_count = 0;

    player.single_audio_period_duration = (uint32_t)BYTES_TO_MILLISECONDS( BT_AUDIO_DEFAULT_PERIOD_SIZE, player.bluetooth_audio_config.sample_rate, player.bluetooth_audio_config.frame_size);
#ifdef USE_AUDIO_PLL
    player.pcm_packet_length            = 0;
#endif /* USE_AUDIO_PLL */

    return result;
}


void bt_audio_write_to_player_buffer( bt_audio_codec_data_t* pcm )
{
    static uint8_t q_count = 0;
    wiced_result_t result;

    if( WICED_TRUE == is_bt_audio_player_initialized() )
    {
#ifdef USE_AUDIO_PLL
        if ( player.pcm_packet_length == 0 )
        {
            uint32_t threshold_msecs;

            threshold_msecs               = (player.audio_buffer_threshold_percent * player.audio_buffer_max_duration_msecs) / 100;
            player.pcm_packet_length      = pcm->length;
            player.queue_threshold_count  = (uint32_t) MILLISECONDS_TO_BYTES(threshold_msecs, player.bluetooth_audio_config.sample_rate, player.bluetooth_audio_config.frame_size);
            player.queue_threshold_count /= player.pcm_packet_length;
            player.queue_threshold_count++;
            WPRINT_APP_INFO ( ("%s: pcm_packet_length=%lu, queue_threshold_count=%lu\n", __func__, player.pcm_packet_length, player.queue_threshold_count) );
        }
#endif /* USE_AUDIO_PLL */
        result = wiced_rtos_push_to_queue( &player.queue, &pcm, WICED_NO_WAIT );
        if ( result != WICED_SUCCESS )
        {
            WPRINT_APP_INFO ( ("%s: push to queue failed, freeing buffer.\n", __func__) );
#ifdef USE_MEM_POOL
            bt_buffer_pool_free_buffer(pcm);
#else
            free(pcm);
#endif
            return;
        }
        if(player.state == BT_AUDIO_DEVICE_STATE_STARTED)
        {
            return;
        }
        else
        {
            q_count++;
            if( q_count >= player.queue_threshold_count )
            {
                wiced_rtos_set_event_flags(&player.events, BT_AUDIO_EVENT_START_PLAYER);
                //WPRINT_APP_INFO ( ("%s: Sending Player Start Event, pcm_length = %d\n", __func__, (int)pcm->length) );
                q_count = 0;
            }
        }
    }
    else
    {
#ifdef USE_MEM_POOL
        bt_buffer_pool_free_buffer(pcm);
#else
        free(pcm);
#endif
    }
}


wiced_result_t bt_audio_stop_player( void )
{
    if(player.state <= BT_AUDIO_DEVICE_STATE_IDLE)
        return WICED_SUCCESS;

    //WPRINT_APP_INFO( ("bt_audio_stop_player\n") );

    //Send stop event to reset the player state to IDLE
    wiced_rtos_set_event_flags(&player.events, BT_AUDIO_EVENT_STOP_PLAYER );
    wiced_rtos_get_semaphore(&player.wait_for_cmd_completion, 34); //max time player can wait for buffer @8K

    return WICED_SUCCESS;
}

wiced_result_t bt_audio_update_player_volume( uint8_t level )
{
    wiced_result_t result = WICED_ERROR;
    double min_volume_db = 0.0, max_volume_db = 0.0, volume_db, step_db;

    if(level > BT_AUDIO_VOLUME_MAX)
        level = BT_AUDIO_DEFAULT_VOLUME;

    if ( is_bt_audio_player_initialized() != WICED_TRUE )
    {
        WPRINT_APP_INFO (("bt_audio_update_player_volume: Player not initialized\n"));
         return result;
    }

    result = wiced_audio_get_volume_range( player.bluetooth_audio_session_handle, &min_volume_db, &max_volume_db );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("bt_audio_update_player_volume: wiced_audio_get_volume_range failed\n"));
        return result;
    }

    step_db = ( double )( max_volume_db - min_volume_db ) /( BT_AUDIO_VOLUME_MAX);
    volume_db = ( double ) VOLUME_CONVERSION( step_db, level, min_volume_db );

    return wiced_audio_set_volume( player.bluetooth_audio_session_handle, volume_db );
}


static wiced_result_t bt_audio_start( void )
{
    wiced_result_t    result = WICED_ERROR;

    if( is_bt_audio_player_prepared())
        result = wiced_audio_start(player.bluetooth_audio_session_handle);

    if(result == WICED_SUCCESS)
        player.state = BT_AUDIO_DEVICE_STATE_STARTED;

    WPRINT_APP_INFO(("bt_audio_start: Player start state [result: %d]\n",result));
    return result;
}

static void bt_audio_stop( void )
{
    uint8_t stop_audio=0;
    bt_audio_codec_data_t* pcm=NULL;

    if ( ( player.state > BT_AUDIO_DEVICE_STATE_IDLE) )
    {
        if ( player.state == BT_AUDIO_DEVICE_STATE_STARTED)
            stop_audio = 1;

        if(stop_audio)
        {
            wiced_audio_stop(player.bluetooth_audio_session_handle);
            if( WICED_SUCCESS != wiced_rtos_is_queue_empty(&player.queue) )
            {
                while( WICED_SUCCESS == wiced_rtos_pop_from_queue(&player.queue, &pcm, WICED_NO_WAIT) )
                {
                    if(pcm != NULL)
                    {
#ifdef USE_MEM_POOL
                        bt_buffer_pool_free_buffer(pcm);
#else
                        free(pcm);
#endif
                        pcm=NULL;
                    }
                }
            }
        }
        wiced_audio_deinit(player.bluetooth_audio_session_handle);
        player.state = BT_AUDIO_DEVICE_STATE_IDLE;
    }
    wiced_rtos_set_semaphore(&player.wait_for_cmd_completion);
}


wiced_result_t  bt_audio_deinit_player( void )
{
    wiced_result_t ret = WICED_SUCCESS;

#ifdef USE_AUDIO_PLL
    if ( pll_tuner_destroy(&player) != WICED_SUCCESS )
    {
        WPRINT_APP_ERROR ( ("bt_audio_deinit_player: pll_tuner_destroy() failed with %d\n", result) );
    }
#endif /* USE_AUDIO_PLL */

    wiced_rtos_deinit_queue(&player.queue);
    wiced_rtos_deinit_event_flags(&player.events);
    wiced_rtos_deinit_semaphore(&player.wait_for_cmd_completion);

    /* reset player control block */
    memset(&player, 0, sizeof(bt_audio_context_t));
    player.state = BT_AUDIO_DEVICE_STATE_UNINITIALIZED;
    return ret;
}


static wiced_bool_t read_bluetooth_audio_data( uint8_t *buffer, uint16_t* size )
{
    static bt_audio_codec_data_t* pcm = NULL;
    uint32_t       current_buffer_pos = 0;
    //uint32_t flags_set;
    wiced_result_t result;
    uint32_t       wait_time_limit    = player.single_audio_period_duration;
    wiced_bool_t   done               = WICED_FALSE;
    uint32_t       copy_len;
    uint32_t       audio_buffer_weight;

    UNUSED_VARIABLE(audio_buffer_weight);

    /* If there is a chunk which was not written in the buffer on the previous call */
    /* write it now */
    if(pcm != NULL)
    {
        if( pcm->length != 0)
        {
            memcpy( buffer, (pcm->data+pcm->offset), pcm->length );
            current_buffer_pos = pcm->length;
        }
#ifdef USE_MEM_POOL
        bt_buffer_pool_free_buffer(pcm);
#else
        free(pcm);
#endif
        pcm = NULL;
    }
    else
    {
        current_buffer_pos = 0;
    }

#ifdef USE_AUDIO_PLL
    if( player.state == BT_AUDIO_DEVICE_STATE_STARTED )
    {
        wiced_audio_get_current_buffer_weight( player.bluetooth_audio_session_handle, &audio_buffer_weight );
        wait_time_limit = BYTES_TO_MILLISECONDS( audio_buffer_weight, player.bluetooth_audio_config.sample_rate, player.bluetooth_audio_config.frame_size);
        if ( wait_time_limit > player.single_audio_period_duration )
        {
            wait_time_limit -= player.single_audio_period_duration;
        }
        else
        {
            wait_time_limit  = 0;
        }
    }
#endif /* USE_AUDIO_PLL */

    /* Read continuously packets from the bluetooth audio queue */
    while(wait_time_limit > 0)
    {
        result = wiced_rtos_pop_from_queue(&player.queue, &pcm, 1);
        if(result != WICED_SUCCESS)
        {
            wait_time_limit--;
            continue;
        }

        /*Calculate how many bytes can be copied*/
        copy_len = ( ( current_buffer_pos + pcm->length ) <= *size )?pcm->length:( *size - current_buffer_pos );

        memcpy(&buffer[current_buffer_pos], pcm->data, copy_len);
        current_buffer_pos += copy_len;

        if(copy_len == pcm->length)
        {
#ifdef USE_MEM_POOL
            bt_buffer_pool_free_buffer(pcm);
#else
            free(pcm);
#endif
            pcm = NULL;
        }
        else //if(copy_len < pcm->length)
        {
            pcm->offset += copy_len;
            pcm->length -= copy_len;
        }

        if(current_buffer_pos == *size)
        {
            done = WICED_TRUE;
            break;
        }
    }

    if( done != WICED_TRUE )
    {
#ifdef USE_AUDIO_PLL
        if( player.state == BT_AUDIO_DEVICE_STATE_STARTED )
        {
            /*
             * Audio engine is actively consuming PCM samples; let's check whether or not it's about to under-run
             */
            wiced_audio_get_current_buffer_weight( player.bluetooth_audio_session_handle, &audio_buffer_weight );

            if ( audio_buffer_weight < BT_AUDIO_BUFFER_UNDERRUN_THRESHOLD )
            {
                /*
                 * Audio engine is about to under-run; fill buffer with silence
                 */
                memset(&buffer[current_buffer_pos], 0x00, *size - current_buffer_pos);
                player.silence_insertion_count++;
            }
            else
            {
                /*
                 * No need to fill buffer with silence; audio engine buffer is not actually under-running !
                 * Simply record the amount PCM data written into the buffer
                 */
                *size = current_buffer_pos;
            }
        }
        else
        {
            /*
             * Audio engine is not actively consuming sample; simply record the amount PCM data written into the buffer
             */
            *size = current_buffer_pos;
        }
#else /* !USE_AUDIO_PLL */
        /* If the time to fill a buffer with at least one audio period is expired
         * Fill left part of the buffer with silence
         */
        memset(&buffer[current_buffer_pos], 0x00, *size - current_buffer_pos);
        player.silence_insertion_count++;
#endif /* USE_AUDIO_PLL */
        done = WICED_TRUE;
    }

    return done;
}


void bt_audio_player_task(uint32_t args)
{
    uint8_t*       ptr;
    uint16_t       n;
    uint16_t       available_bytes;
    wiced_result_t result;
    uint32_t       flags_set;

    for ( ;;)
    {
        /* Wait for an audio ready event( all bluetooth audio buffers must be filled before starting playback ) */
        flags_set = 0;
        result = wiced_rtos_wait_for_event_flags(&player.events, BT_AUDIO_EVENT_ALL, &flags_set, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if( result != WICED_SUCCESS )
            continue;

#ifdef USE_AUDIO_PLL
        if ( flags_set & BT_AUDIO_EVENT_START_PLAYER )
        {
            if ( pll_tuner_run(&player) != WICED_SUCCESS )
            {
                WPRINT_APP_ERROR ( ("bt_audio_player_task: pll_tuner_run() failed with %d\n", result) );
            }
        }

        if ( flags_set & BT_AUDIO_EVENT_STOP_PLAYER )
        {
            if ( pll_tuner_rest(&player) != WICED_SUCCESS )
            {
                WPRINT_APP_ERROR ( ("bt_audio_player_task: pll_tuner_rest() failed with %d\n", result) );
            }
        }
#endif /* USE_AUDIO_PLL */

        WPRINT_APP_INFO(("[BT-PLAYER] Start bluetooth playback \n"));
        /* Audio thread is started, now it will read contents of queued bluetooth audio buffers and give it for further processing
         *  to the audio framework */
        while( !(flags_set & BT_AUDIO_EVENT_STOP_PLAYER))
        {
            /* wait till at least one audio period is available for writing */
            if( player.state != BT_AUDIO_DEVICE_STATE_STARTED)
            {
                result = wiced_audio_wait_buffer(player.bluetooth_audio_session_handle, BT_AUDIO_BUFFER_SIZE, 0 );
                wiced_assert( "Cant get a period from the buffer.", result == WICED_SUCCESS );
                available_bytes = BT_AUDIO_BUFFER_SIZE;
            }
            else
            {
                /* Wait till at least one period is avaialable for writing */
                result = wiced_audio_wait_buffer(player.bluetooth_audio_session_handle, BT_AUDIO_DEFAULT_PERIOD_SIZE, player.single_audio_period_duration + 2);
                if( result != WICED_SUCCESS )
                {
                    /* Must do a recovery there */
                    wiced_audio_stop(player.bluetooth_audio_session_handle);
                    WPRINT_APP_DEBUG(("Recover after wait"));
                    player.state = BT_AUDIO_DEVICE_STATE_STOPPED;
                    player.underrun_count++;
                    continue;
                }
                available_bytes = BT_AUDIO_DEFAULT_PERIOD_SIZE;
            }

            while (available_bytes > 0)
            {
                n = available_bytes;
                result = wiced_audio_get_buffer( player.bluetooth_audio_session_handle, &ptr, &n );
                wiced_assert("Cant get an audio buffer", result == WICED_SUCCESS);
                if( result == WICED_ERROR )
                {
                    /* Underrun might have occured, recover by audio restart */
                    wiced_audio_stop(player.bluetooth_audio_session_handle);
                    WPRINT_APP_DEBUG(("Recover after get"));
                    player.state = BT_AUDIO_DEVICE_STATE_STOPPED;
                    player.underrun_count++;
                    break;
                }
                else if( result != WICED_SUCCESS )
                {
                    WPRINT_APP_DEBUG(("bt_audio_wiced_audio_player_task: No buffer avail"));
                    player.buffer_unavail_count++;
                    break;
                }

                if( n == 0 )
                {
                    /* must do a recovery there */
                    wiced_audio_stop(player.bluetooth_audio_session_handle);
                    WPRINT_APP_DEBUG(("Overrun occured"));
                    player.state = BT_AUDIO_DEVICE_STATE_STOPPED;
                    player.overrun_count++;
                    break;
                }

                if(WICED_FALSE == read_bluetooth_audio_data(ptr, &n ))
                {
                    break;
                }

                wiced_assert("Should never return 0 bytes", ( n != 0 ) );
                result = wiced_audio_release_buffer( player.bluetooth_audio_session_handle, n );
                if( result == WICED_ERROR )
                {
                    wiced_audio_stop(player.bluetooth_audio_session_handle);
                    WPRINT_APP_DEBUG(("Recover after buffer release"));
                    player.state = BT_AUDIO_DEVICE_STATE_STOPPED;
                    player.underrun_count++;
                    break;
                }

                wiced_assert("Cant release an audio slot", result == WICED_SUCCESS);
                available_bytes -= n;
                if (player.state != BT_AUDIO_DEVICE_STATE_STARTED && available_bytes == 0)
                {
                    bt_audio_start();
                }
            } /* while (available_bytes > 0) */
            result = wiced_rtos_wait_for_event_flags(&player.events, BT_AUDIO_EVENT_ALL, &flags_set, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);
        }
        bt_audio_stop();
        WPRINT_APP_INFO(("[%s] out of playback loop\n", __func__));
        WPRINT_APP_INFO(("Playback stats: UNDER=%lu, SILENCE=%lu, OVER=%lu, UNAVAIL=%lu\n",
                          player.underrun_count, player.silence_insertion_count, player.overrun_count, player.buffer_unavail_count));
    } /*end for (;;)*/
}

