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
#include "wiced_audio.h"
#include "bt_types.h"
#include "bluetooth_audio.h"
#ifdef USE_MEM_POOL
#include "mem_pool.h"
#endif


/*****************************************************************************
**
**  Name:           bta_wiced_audio_player.c
**
**  Description:    BTA AVK interface to the Wiced audio subsystem
**
*****************************************************************************/


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define BT_SCO_OUTPUT_BUFFER_SIZE                    60*2 //30 samples of 2 bytes each for two channels

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

bt_audio_context_t recorder;

#ifdef USE_MEM_POOL
extern bt_buffer_pool_handle_t  mem_pool_pcm;
#endif

/******************************************************
*               Function Definitions
******************************************************/
void bt_audio_tx_recorder_data_to_sco( bt_audio_codec_data_t* pcm )
{
    uint16_t *p_sco_data = (uint16_t*)(pcm->data + pcm->offset);
    uint16_t len = pcm->length;
    int i = 0, j = 0;

    do
    {
        p_sco_data[j++] = p_sco_data[i];
        i += 2;
    }while(i < len);
    pcm->length = len/2;

    bt_audio_hfp_send_sco_data(pcm);
}

void write_bluetooth_audio_data( uint8_t *buffer, uint16_t size )
{
    static bt_audio_codec_data_t* pcm;
    uint8_t*        pdata = NULL;
    uint32_t        cpy_len = 0;
    uint32_t        remained_bytes_length = size;
#ifdef PUSH_SCO_TO_QUEUE
    wiced_result_t result;
#endif

    while(remained_bytes_length > 0)
    {
        if(!pcm)
        {
#ifdef USE_MEM_POOL
            pcm = bt_buffer_pool_allocate_buffer(mem_pool_pcm);
#else
            pcm = malloc( sizeof(bt_audio_codec_data_t)+BT_SCO_OUTPUT_BUFFER_SIZE );
#endif
            if(pcm == NULL)
            {
                return;
            }
            pcm->offset = 0;
            pcm->length = 0;
        }

        pdata = pcm->data+pcm->length;
        cpy_len = (remained_bytes_length >= (BT_SCO_OUTPUT_BUFFER_SIZE - pcm->length))?(BT_SCO_OUTPUT_BUFFER_SIZE - pcm->length):remained_bytes_length;
        memcpy(pdata, buffer+(size-remained_bytes_length), cpy_len);
        remained_bytes_length -= cpy_len;
        pcm->length += cpy_len;

        if(pcm->length == BT_SCO_OUTPUT_BUFFER_SIZE)
        {
#ifdef PUSH_SCO_TO_QUEUE
            result = wiced_rtos_push_to_queue( &recorder.queue, &pcm, WICED_NO_WAIT );
            if(result != WICED_SUCCESS)
            {
#ifdef USE_MEM_POOL
                bt_buffer_pool_free_buffer(pcm);
#else
                free(pcm);
#endif
            }
            pcm = NULL;
#else
            bt_audio_tx_recorder_data_to_sco(pcm);
            pcm->offset = 0;
            pcm->length = 0;
#endif
        }
    }
}

void bta_wiced_audio_recorder_task(uint32_t context)
{
    uint8_t*          ptr;
    uint16_t          n;
    wiced_result_t    result;
    uint16_t          available_bytes;
    uint32_t   flags_set;

    for( ;; )
    {

        result = wiced_rtos_wait_for_event_flags(&recorder.events, BT_AUDIO_EVENT_ALL, &flags_set, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if( result != WICED_SUCCESS )
        {
            WPRINT_APP_INFO( ("[BT-RECORDER] error in wait for event... \n" ));
            continue;
        }
        WPRINT_APP_INFO(( "[BT-RECORDER] Start task... \n" ));

        while( !(flags_set & BT_AUDIO_EVENT_STOP_LOOP))
        {
            //ensure that recorder is initialized at this point of time.
            uint32_t wait_time = ((NUM_USECONDS_IN_SECONDS/recorder.bluetooth_audio_config.sample_rate) * (BT_AUDIO_DEFAULT_PERIOD_SIZE/(recorder.bluetooth_audio_config.channels)))/NUM_MSECONDS_IN_SECONDS;
            if( recorder.state != BT_AUDIO_DEVICE_STATE_STARTED )
            {
                result = bt_audio_start(BT_AUDIO_DEVICE_RECORDER);
                //APPL_TRACE_ERROR0( "[BT-RECORDER] recorder started... " );
                if(result != WICED_SUCCESS)
                {
                    WPRINT_APP_INFO(( "%s: bt_audio_start result = %d\n",__func__, result ));
                    break;
                }
                wait_time = wait_time*3;
            }

            result = wiced_audio_wait_buffer(recorder.bluetooth_audio_session_handle, BT_AUDIO_DEFAULT_PERIOD_SIZE, wait_time+wait_time/10);
            available_bytes = BT_AUDIO_DEFAULT_PERIOD_SIZE;

            if( result != WICED_SUCCESS )
            {
                /* Must do a recovery there */
                WPRINT_APP_INFO(("%s: Recover after wait\n", __func__));
                wiced_rtos_delay_milliseconds(wait_time+wait_time/10);
                available_bytes = 0;
            }

            //APPL_TRACE_ERROR0( "[BT-RECORDER] wait buffer done..." );
            while (available_bytes > 0)
            {
                /* request for at least for a period size */
                n = available_bytes;
                result = wiced_audio_get_buffer( recorder.bluetooth_audio_session_handle, &ptr, &n );
                wiced_assert("Cant get an audio buffer\n", result == WICED_SUCCESS);
                //APPL_TRACE_ERROR0( "[BT-RECORDER] got buffer... " );
                if( result == WICED_ERROR )
                {
                    /* Underrun might have occured, recover by audio restart */
                    WPRINT_APP_INFO(("%s: Recover after get\n", __func__));
                    wiced_rtos_delay_milliseconds(wait_time+wait_time/10);
                    break;
                }
                else if( result != WICED_SUCCESS )
                {
                    WPRINT_APP_INFO(("%s: No buffer avail\n", __func__));
                    break;
                }

                //TODO is this a valid case?
                if( n <= 0 || ptr == NULL )
                {
                    /* must do a recovery there */
                    WPRINT_APP_INFO(("%s: Overrun occured\n",__func__));
                    wiced_rtos_delay_milliseconds(wait_time+wait_time/10);
                    break;
                }

                if( wiced_rtos_wait_for_event_flags(&recorder.events, BT_AUDIO_EVENT_STOP_LOOP, &flags_set, WICED_FALSE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT) == WICED_SUCCESS )
                {
                    if((flags_set & BT_AUDIO_EVENT_STOP_LOOP) == BT_AUDIO_EVENT_STOP_LOOP)
                    {
                        break;
                    }
                }

                write_bluetooth_audio_data(ptr, n );
                result = wiced_audio_release_buffer( recorder.bluetooth_audio_session_handle, n );
                available_bytes -= n;
                if( result == WICED_ERROR )
                {
                    WPRINT_APP_INFO(("%s: Recover after buffer release\n", __func__));
                    wiced_rtos_delay_milliseconds(wait_time+wait_time/10);
                    break;
                }
                wiced_assert("Cant release an audio slot", result == WICED_SUCCESS);
            }
            result = wiced_rtos_wait_for_event_flags(&recorder.events, BT_AUDIO_EVENT_ALL, &flags_set, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_NO_WAIT);
        }
        bt_audio_stop(BT_AUDIO_DEVICE_RECORDER);
        WPRINT_APP_INFO(("%s: Out of RECORDER loop\n", __func__));
    }
}
