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
 *                    Constants
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

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

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t bt_audio_update_player_volume( uint8_t level )
{
    wiced_result_t result = WICED_ERROR;
    double min_volume_db = 0.0, max_volume_db = 0.0, volume_db, step_db;

    if(level > BT_AUDIO_VOLUME_MAX)
        level = BT_AUDIO_DEFAULT_VOLUME;

    if ( is_bt_audio_device_initialized(BT_AUDIO_DEVICE_PLAYER) != WICED_TRUE )
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


void bt_audio_write_to_player_buffer( bt_audio_codec_data_t* pcm )
{
    static uint8_t q_count = 0;
    wiced_result_t result;

    if( WICED_TRUE == is_bt_audio_device_initialized(BT_AUDIO_DEVICE_PLAYER) )
    {
        result = wiced_rtos_push_to_queue( &player.queue, &pcm, WICED_NO_WAIT );
        if ( result != WICED_SUCCESS )
        {
            WPRINT_APP_DEBUG ( ("%s: push to queue failed, freeing buffer.\n", __func__) );
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
            if( q_count >= BT_AUDIO_QUEUE_THRESHOLD  )
            {
                bt_audio_start_loop(BT_AUDIO_DEVICE_PLAYER);
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


static wiced_bool_t read_bluetooth_audio_data( uint8_t *buffer, uint16_t* size )
{
    static bt_audio_codec_data_t* pcm = NULL;
    uint32_t        current_buffer_pos = 0;
    wiced_result_t result;
    uint32_t        wait_time_limit;
    wiced_bool_t    done = WICED_FALSE;
    uint32_t copy_len;

    /* If there is a chunk which was not written in the buffer on the previous call */
    /* write it know */
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

    wait_time_limit = ((double)((*size)*NUM_MSECONDS_IN_SECONDS))/((double)(player.bluetooth_audio_config.sample_rate*4));

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
            /* If the time to fill a buffer with at least one audio period is expired
             * Fill left part of the buffer with silence
             */
            memset(&buffer[current_buffer_pos], 0x00, *size - current_buffer_pos);
            done = WICED_TRUE;
        }

    return done;
}


void bt_audio_player_task(uint32_t args)
{
    uint8_t*          ptr;
    uint16_t          n;
    uint16_t          available_bytes;
    wiced_result_t    result;
    uint32_t   flags_set;

    for ( ;;)
    {
        /* Wait for an audio ready event( all bluetooth audio buffers must be filled before starting playback ) */
        result = wiced_rtos_wait_for_event_flags(&player.events, BT_AUDIO_EVENT_ALL, &flags_set, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if( result != WICED_SUCCESS )
            continue;

        WPRINT_APP_INFO(("[BT-PLAYER] Start bluetooth playback \n"));
        /* Audio thread is started, now it will read contents of queued bluetooth audio buffers and give it for further processing
         *  to the audio framework */
        while( !(flags_set & BT_AUDIO_EVENT_STOP_LOOP))
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
                uint32_t wait_time =        ((NUM_USECONDS_IN_SECONDS/player.bluetooth_audio_config.sample_rate) * BT_AUDIO_DEFAULT_PERIOD_SIZE)/NUM_MSECONDS_IN_SECONDS;
                result = wiced_audio_wait_buffer(player.bluetooth_audio_session_handle, BT_AUDIO_DEFAULT_PERIOD_SIZE, wait_time + 100);
                if( result != WICED_SUCCESS )
                {
                    /* Must do a recovery there */
                    wiced_audio_stop(player.bluetooth_audio_session_handle);
                    WPRINT_APP_INFO(("Recover after wait"));
                    player.state = BT_AUDIO_DEVICE_STATE_STOPPED;
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
                    WPRINT_APP_INFO(("Recover after get"));
                    player.state = BT_AUDIO_DEVICE_STATE_STOPPED;
                    break;
                }
                else if( result != WICED_SUCCESS )
                {
                    WPRINT_APP_INFO(("bt_audio_wiced_audio_player_task: No buffer avail"));
                    break;
                }
                if( n <= 0 )
                {
                    /* must do a recovery there */
                    wiced_audio_stop(player.bluetooth_audio_session_handle);
                    WPRINT_APP_INFO(("Overrun occured"));
                    player.state = BT_AUDIO_DEVICE_STATE_STOPPED;
                    break;
                }

                if( wiced_rtos_wait_for_event_flags(&player.events, BT_AUDIO_EVENT_STOP_LOOP, &flags_set, WICED_FALSE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT) == WICED_SUCCESS )
                {
                    if((flags_set & BT_AUDIO_EVENT_STOP_LOOP) == BT_AUDIO_EVENT_STOP_LOOP)
                    {
                        break;
                    }
                }

                if(WICED_FALSE == read_bluetooth_audio_data(ptr, &n ))
                    break;

                wiced_assert("Should never return 0 bytes", ( n != 0 ) );
                result = wiced_audio_release_buffer( player.bluetooth_audio_session_handle, n );
                if( result == WICED_ERROR )
                {
                    wiced_audio_stop(player.bluetooth_audio_session_handle);
                    WPRINT_APP_INFO(("Recover after buffer release"));
                    player.state = BT_AUDIO_DEVICE_STATE_STOPPED;
                    break;
                }

                wiced_assert("Cant release an audio slot", result == WICED_SUCCESS);
                available_bytes -= n;
                if (player.state != BT_AUDIO_DEVICE_STATE_STARTED && available_bytes == 0)
                {
                    bt_audio_start(BT_AUDIO_DEVICE_PLAYER);
                }
            } /* while (available_bytes > 0) */
            result = wiced_rtos_wait_for_event_flags(&player.events, BT_AUDIO_EVENT_ALL, &flags_set, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);
        }
        bt_audio_stop(BT_AUDIO_DEVICE_PLAYER);
        WPRINT_APP_INFO(("[bt_audio_wiced_audio_player_task] out of PLAYER loop\n"));
    } /*end for (;;)*/
}

