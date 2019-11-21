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

/*****************************************************************************
**
**  Name:           bluetooth_audio_common_wiced_hci.c
**
**  Description:
**
*****************************************************************************/

/******************************************************
 *                      Macros
 ******************************************************/
#define VOLUME_CONVERSION(step,level,min)                    ((double)step*(double)level + min)

/******************************************************
 *                   Enumerations
 ******************************************************/
/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
/******************************************************
 *                    Structures
 ******************************************************/
/******************************************************
 *               Function Definitions
 ******************************************************/
/******************************************************
 *               Variables Definitions
 ******************************************************/
bt_audio_context_t player;
bt_audio_context_t recorder;
/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_bool_t is_bt_audio_player_initialized( void )
{
    return (wiced_bool_t)( player.state > BT_AUDIO_DEVICE_STATE_IDLE );
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

/*This is stub function*/
void bt_audio_player_task(uint32_t args)
{
    UNUSED_PARAMETER(args);
}

/*This is stub function*/
wiced_result_t bt_audio_write_to_decoder_queue(bt_audio_codec_data_t* audio)
{
    UNUSED_PARAMETER(audio);

    return WICED_SUCCESS;
}

/*This is stub function*/
wiced_result_t bt_audio_decoder_context_init( void )
{
    return WICED_SUCCESS;
}

/*This is stub function*/
wiced_result_t bt_audio_reset_decoder_config( void )
{
    return WICED_SUCCESS;
}

/*This is stub function*/
wiced_result_t bt_audio_configure_decoder(wiced_bt_a2dp_codec_info_t* decoder_config)
{
    UNUSED_PARAMETER(decoder_config);
    return WICED_SUCCESS;
}

/*This is stub function*/
void bt_audio_decoder_task( uint32_t arg )
{
    UNUSED_PARAMETER(arg);
}
void bta_wiced_audio_recorder_task(uint32_t context)
{

}
void bt_audio_write_to_player_buffer( bt_audio_codec_data_t* pcm )
{
    UNUSED_PARAMETER(pcm);
}

