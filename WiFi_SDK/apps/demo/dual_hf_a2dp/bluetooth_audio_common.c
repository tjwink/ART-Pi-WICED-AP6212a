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
#include "dual_a2dp_hfp_audio.h"

#ifdef USE_MEM_POOL
#include "mem_pool.h"
#endif

/*****************************************************************************
**
**  Name:           bt_audio_common.c
**
**  Description:    Interface to the Wiced audio subsystem
**
*****************************************************************************/

/******************************************************
 *                      Macros
 ******************************************************/
#define GET_DEVICE_CONTEXT_PTR(device_type) (( device_type == BT_AUDIO_DEVICE_PLAYER )?&player:&recorder)
#define BT_AUDIO_RECORDER_BUFFER_SIZE               WICED_AUDIO_BUFFER_ARRAY_DIM_SIZEOF(4, BT_AUDIO_DEFAULT_PERIOD_SIZE)

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/
extern bt_audio_context_t player;
extern bt_audio_context_t recorder;

#ifndef USE_WICED_HCI
uint8_t tx_buf[BT_AUDIO_RECORDER_BUFFER_SIZE];
#endif

/******************************************************
 *                   Enumerations
 ******************************************************/

enum dac_output_select
{
    DAC_OUTPUT_SELECT_MUTE    = 0,
    DAC_OUTPUT_SELECT_LCH     = 1,
    DAC_OUTPUT_SELECT_RCH     = 2,
    DAC_OUTPUT_SELECT_LCH_RCH = 3,
};

wiced_result_t bt_audio_configure_device( bt_audio_config_t* p_audio_config, bt_audio_device_type_t device_type, service_type_t service_type );
/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_bool_t is_bt_audio_device_initialized( bt_audio_device_type_t device_type )
{
    bt_audio_context_t* device = GET_DEVICE_CONTEXT_PTR(device_type) ;

    return (wiced_bool_t)( device->state > BT_AUDIO_DEVICE_STATE_IDLE );
}

wiced_bool_t is_bt_audio_device_prepared( bt_audio_device_type_t device_type )
{
    bt_audio_context_t* device = GET_DEVICE_CONTEXT_PTR(device_type) ;

    return (wiced_bool_t)( device->state == BT_AUDIO_DEVICE_STATE_CONFIGURED ||
                           device->state == BT_AUDIO_DEVICE_STATE_STOPPED );
}

wiced_result_t  bt_audio_init_device( bt_audio_device_type_t device_type )
{
    wiced_result_t result = WICED_SUCCESS;
    bt_audio_context_t* device = GET_DEVICE_CONTEXT_PTR(device_type) ;
    char queue_name[20];

    if(device_type == BT_AUDIO_DEVICE_PLAYER)
        strlcpy(queue_name, "PLAYER_Q", sizeof(queue_name));
    else
        strlcpy(queue_name, "RECORDER_Q", sizeof(queue_name));

    if(device->state == BT_AUDIO_DEVICE_STATE_UNINITIALIZED)
    {
        result = wiced_rtos_init_queue(&device->queue, queue_name, sizeof(bt_audio_codec_data_t*), BT_AUDIO_QUEUE_MAX_SIZE);
        result = wiced_rtos_init_event_flags(&device->events);
        result = wiced_rtos_init_semaphore(&device->wait_for_cmd_completion);
        device->state = BT_AUDIO_DEVICE_STATE_IDLE;
    }
    WPRINT_APP_INFO ( ("%s: result=%d\n", __func__, result) );
    return result;
}


wiced_result_t bt_audio_configure_device( bt_audio_config_t* p_audio_config, bt_audio_device_type_t device_type, service_type_t service_type )
{
    wiced_result_t             result;
    wiced_audio_config_t       wiced_audio_conf = {0, };
    bt_audio_context_t*        device           = GET_DEVICE_CONTEXT_PTR(device_type);
    platform_audio_device_id_t player_dev_id    = PLATFORM_DEFAULT_AUDIO_OUTPUT;
    platform_audio_device_id_t recorder_dev_id  = PLATFORM_DEFAULT_AUDIO_INPUT;

    WPRINT_APP_INFO(("bt_audio_configure_device: device = %d\n", device_type));

    if(p_audio_config == NULL)
        return WICED_BADARG;

    if(device->state != BT_AUDIO_DEVICE_STATE_IDLE)
        return WICED_ERROR;

#ifdef USE_WICED_HCI
    player_dev_id   = PLATFORM_DEFAULT_BT_AUDIO_OUTPUT;
    recorder_dev_id = PLATFORM_DEFAULT_BT_AUDIO_INPUT;
#endif

    /* Initialize and configure audio framework for needed audio format */
    if(device_type == BT_AUDIO_DEVICE_PLAYER)
        result = wiced_audio_init(player_dev_id, &device->bluetooth_audio_session_handle, BT_AUDIO_DEFAULT_PERIOD_SIZE);
    else //recorder
        result = wiced_audio_init(recorder_dev_id, &device->bluetooth_audio_session_handle, BT_AUDIO_DEFAULT_PERIOD_SIZE);

    if( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("bt_audio_configure_device: device %d,  Error Initializing Wiced Audio Framework[err: %d]\n",(unsigned int) device_type, result) );
        return result;
    }

#ifndef USE_WICED_HCI
    if(device_type == BT_AUDIO_DEVICE_PLAYER)
        result = wiced_audio_create_buffer(device->bluetooth_audio_session_handle, BT_AUDIO_BUFFER_SIZE, NULL, NULL );
    else //recorder
        result = wiced_audio_create_buffer(device->bluetooth_audio_session_handle, sizeof(tx_buf), WICED_AUDIO_BUFFER_ARRAY_PTR(tx_buf), NULL );
    if( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("bt_audio_configure_device: device %d Error Registering Buffer to Wiced Audio Framework [err: %d]\n",(unsigned int) device_type, result));
        return result;
    }
#endif

   memcpy(&device->bluetooth_audio_config, p_audio_config, sizeof(bt_audio_config_t));
   WPRINT_APP_INFO(("bt_audio_configure_device: device_type %d, config sample_rate:%u channels:%d bps:%d\n",
                                            (unsigned int) device_type, (unsigned int) device->bluetooth_audio_config.sample_rate,
                                            (int)device->bluetooth_audio_config.channels, (int)device->bluetooth_audio_config.bits_per_sample));

    wiced_audio_conf.sample_rate     = device->bluetooth_audio_config.sample_rate;
    wiced_audio_conf.channels        = device->bluetooth_audio_config.channels;
    wiced_audio_conf.bits_per_sample = device->bluetooth_audio_config.bits_per_sample;
    wiced_audio_conf.frame_size      = (wiced_audio_conf.channels * wiced_audio_conf.bits_per_sample) / 8;

    result = wiced_audio_configure( device->bluetooth_audio_session_handle, &wiced_audio_conf );
    if( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("bt_audio_configure_device: device %d Error configuring Wiced Audio framework [err: %d]\n", (unsigned int) device_type, result));
        return result;
    }
    device->state = BT_AUDIO_DEVICE_STATE_CONFIGURED;

#ifdef USE_WICED_HCI
    wiced_audio_start(device->bluetooth_audio_session_handle);
#endif

    //Set the default volume if player
    if(device_type == BT_AUDIO_DEVICE_PLAYER)
        result = bt_audio_update_player_volume(device->bluetooth_audio_config.volume);

#ifdef USE_WICED_HCI
    if ( device_type == BT_AUDIO_DEVICE_PLAYER )
    {
        wiced_audio_device_ioctl_data_t ioctl_data;
        wiced_bool_t                    service_type_valid = WICED_TRUE;

        if ( service_type == SERVICE_BT_A2DP )
        {
            /*
             * Left and right channels are swapped;
             * instructing audio device to take care of it.
             */
            ioctl_data.dac_output_mode.left_channel_select  = DAC_OUTPUT_SELECT_RCH;
            ioctl_data.dac_output_mode.right_channel_select = DAC_OUTPUT_SELECT_LCH;
        }
        else if ( service_type == SERVICE_BT_HFP )
        {
            /*
             * Only one channel is active (mono);
             * instructing audio device to perform a channel copy
             */
            ioctl_data.dac_output_mode.left_channel_select  = DAC_OUTPUT_SELECT_LCH_RCH;
            ioctl_data.dac_output_mode.right_channel_select = DAC_OUTPUT_SELECT_LCH_RCH;

        }
        else
        {
            service_type_valid = WICED_FALSE;
        }

        if ( service_type_valid )
        {
            result = wiced_audio_device_ioctl( device->bluetooth_audio_session_handle, WICED_AUDIO_IOCTL_SET_DAC_OUTPUT_MIXING, &ioctl_data );
            if ( result != WICED_SUCCESS )
            {
                WPRINT_APP_INFO(("bt_audio_configure_device: device %d Error wiced_audio_device_ioctl() [err: %d]\n", (unsigned int) device_type, result));
            }
        }
    }
#else
    UNUSED_PARAMETER(service_type);
#endif

    return result;
}

void bt_audio_start_loop(bt_audio_device_type_t device_type)
{
    bt_audio_context_t* device = GET_DEVICE_CONTEXT_PTR(device_type);

    wiced_rtos_set_event_flags(&device->events, BT_AUDIO_EVENT_START_LOOP);
    //WPRINT_APP_INFO(("%s: Send [%d] loop start event [result: %d]\n",__func__, device_type, result));
}

#ifdef USE_WICED_HCI
wiced_result_t bt_audio_end_loop( uint32_t device_type )
{
    WPRINT_APP_INFO(("%s,device_type:%x \n",__func__,(int) device_type));

    if(device_type & BT_AUDIO_DEVICE_PLAYER)
        bt_audio_stop(BT_AUDIO_DEVICE_PLAYER);

    if(device_type & BT_AUDIO_DEVICE_RECORDER)
        bt_audio_stop(BT_AUDIO_DEVICE_RECORDER);

    return WICED_SUCCESS;
}
#else
wiced_result_t bt_audio_end_loop( uint32_t device_type )
{
    bt_audio_context_t* p_player = NULL;
    bt_audio_context_t* p_recorder = NULL;

    if(device_type & BT_AUDIO_DEVICE_PLAYER)
    {
        if(player.state > BT_AUDIO_DEVICE_STATE_IDLE)
        {
            p_player = GET_DEVICE_CONTEXT_PTR((device_type & BT_AUDIO_DEVICE_PLAYER));
            wiced_rtos_set_event_flags(&p_player->events, BT_AUDIO_EVENT_STOP_LOOP );
        }
    }
    if(device_type & BT_AUDIO_DEVICE_RECORDER)
    {
        if(recorder.state > BT_AUDIO_DEVICE_STATE_IDLE)
        {
            p_recorder = GET_DEVICE_CONTEXT_PTR((device_type & BT_AUDIO_DEVICE_RECORDER));
            wiced_rtos_set_event_flags(&p_recorder->events, BT_AUDIO_EVENT_STOP_LOOP );
        }
    }

    //WPRINT_APP_INFO( ("%s device type 0x%x\n",__func__, (unsigned int) device_type) );
    //wait for the device to go to IDLE
    if(p_player)
        wiced_rtos_get_semaphore(&p_player->wait_for_cmd_completion, 34); //max time player can wait for buffer @8K
    if(p_recorder)
        wiced_rtos_get_semaphore(&p_recorder->wait_for_cmd_completion, 34); //max time player can wait for buffer @8K

    return WICED_SUCCESS;
}
#endif
wiced_result_t bt_audio_start( bt_audio_device_type_t device_type )
{
    wiced_result_t    result = WICED_ERROR;
    bt_audio_context_t* device = GET_DEVICE_CONTEXT_PTR(device_type) ;

    if( is_bt_audio_device_prepared(device_type))
        result = wiced_audio_start(device->bluetooth_audio_session_handle);

    if(result == WICED_SUCCESS)
        device->state = BT_AUDIO_DEVICE_STATE_STARTED;

    WPRINT_APP_INFO(("bt_audio_start: start state [device %d] [result: %d]\n",device_type, result));
    return result;
}

void bt_audio_stop( bt_audio_device_type_t device_type )
{
    uint8_t stop_audio=0;
    bt_audio_codec_data_t* pcm=NULL;
    bt_audio_context_t* device = GET_DEVICE_CONTEXT_PTR(device_type) ;

    if ( ( device->state > BT_AUDIO_DEVICE_STATE_IDLE) )
    {
        if ( device->state == BT_AUDIO_DEVICE_STATE_STARTED)
            stop_audio = 1;

        if(stop_audio)
        {
            wiced_audio_stop(device->bluetooth_audio_session_handle);
            if( WICED_SUCCESS != wiced_rtos_is_queue_empty(&device->queue) )
            {
                while( WICED_SUCCESS == wiced_rtos_pop_from_queue(&device->queue, &pcm, WICED_NO_WAIT) )
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
        wiced_audio_deinit(device->bluetooth_audio_session_handle);
        device->state = BT_AUDIO_DEVICE_STATE_IDLE;
    }
    wiced_rtos_set_semaphore(&device->wait_for_cmd_completion);
}


wiced_result_t  bt_audio_deinit_device( bt_audio_device_type_t device_type )
{
    wiced_result_t ret = WICED_SUCCESS;
    bt_audio_context_t* device = GET_DEVICE_CONTEXT_PTR(device_type) ;

    wiced_rtos_deinit_queue(&device->queue);
    wiced_rtos_deinit_event_flags(&device->events);
    wiced_rtos_deinit_semaphore(&device->wait_for_cmd_completion);

    /* reset player control block */
    memset(device, 0, sizeof(bt_audio_context_t));
    device->state = BT_AUDIO_DEVICE_STATE_UNINITIALIZED;
    return ret;
}

