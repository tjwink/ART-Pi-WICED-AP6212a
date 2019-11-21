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
 * Bluetooth A2DP Application Programming Interface
 *
 */
#pragma once

#include "wiced_bt_a2dp_sink.h"
#include "wiced_codec_if.h"
#include "wiced_bt_cfg.h"
#include "apollo_bt_service.h"

#ifdef BT_AUDIO_USE_MEM_POOL
#include "mem_pool.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************
**  constants
*****************************************************************************/

#define BT_AUDIO_DECODER_QUEUE_MAX_SIZE 40

#define EIR_DATA_LENGTH                 240
#define EIR_COMPLETE_LOCAL_NAME_TYPE    0x09
#define EIR_COMPLETE_16BITS_UUID_TYPE   0x03

#define BT_AUDIO_DECODER_TASK_PRIORITY  ( WICED_DEFAULT_LIBRARY_PRIORITY + 1)
#define BT_AUDIO_DECODER_STACK_SIZE     (4096)

#define MEM_POOL_BUFFER_COUNT           (30)
#define MEM_POOL_BUFFER_SIZE            (660)

#define BT_AUDIO_DEFAULT_VOLUME         63 /* mid of 0-127 */
#define BT_AUDIO_VOLUME_STEP             8
#define BT_AUDIO_VOLUME_MAX            127
#define BT_AUDIO_VOLUME_MIN              0

/*****************************************************************************
 ** enumerations
 *****************************************************************************/

typedef enum
{
    BT_AUDIO_CODEC_EVENT_DECODE        = 0x00000001,
    BT_AUDIO_CODEC_EVENT_CONFIGURE     = 0x00000002,
    BT_AUDIO_CODEC_EVENT_QUIT          = 0x00000004,
    BT_AUDIO_CODEC_EVENT_ALL           = 0xFFFFFFFF
}codec_event_flags_t;

/*****************************************************************************
**  type definitions
*****************************************************************************/

typedef void ( *a2dp_sink_callback )( wiced_bt_a2dp_sink_event_t event, wiced_bt_a2dp_sink_event_data_t *p_data );

typedef struct
{
    uint16_t  length;
    uint16_t  offset;
    uint8_t   data[]; //data of length size.
}bt_audio_codec_data_t;

typedef struct
{
    uint32_t sample_rate;
    uint8_t  bits_per_sample;
    uint8_t  channels;
    uint8_t  volume;
    uint32_t bit_rate;
    uint32_t frame_size;
} bt_audio_config_t;

typedef struct
{
    wiced_codec_interface_t*    wiced_sbc_if;
    wiced_queue_t               queue;
    wiced_event_flags_t         events;
} bt_audio_decoder_context_t;

typedef struct apollo_bt_a2dp_sink_s
{
    wiced_bool_t                      is_initialized;
    wiced_result_t                    init_result;
    wiced_event_flags_t               events;
    wiced_bool_t                      is_connected;
    apollo_bt_a2dp_sink_init_params_t user_params;
    apollo_bt_a2dp_sink_event_data_t  user_event_data;
    bt_audio_config_t                 audio_config;

    wiced_bt_management_evt_data_t   *p_event_data;
    wiced_bt_device_address_t         local_address;
    wiced_bt_device_address_t         remote_address;
    wiced_bt_a2dp_config_data_t       codec_config;
    wiced_bt_cfg_settings_t           cfg_settings;

    bt_buffer_pool_handle_t           mem_pool;

    bt_audio_decoder_context_t        decoder;
    wiced_bt_a2dp_codec_info_t        decoder_config;
    bt_audio_codec_data_t            *decoder_input_buffer;
    wiced_thread_t                    decoder_thread;
    wiced_thread_t                   *decoder_thread_ptr;
    wiced_bool_t                      decoder_thread_quit;
    uint8_t                           decoder_stack[BT_AUDIO_DECODER_STACK_SIZE];
} apollo_bt_a2dp_sink_t;

/*****************************************************************************
**  external function declarations
*****************************************************************************/

apollo_bt_a2dp_sink_t *apollo_bt_a2dp_get_context( void );

wiced_result_t bt_audio_get_config_from_cie( wiced_bt_a2dp_codec_info_t* p_codec_config, bt_audio_config_t* p_audio_config );
wiced_result_t bt_audio_decoder_context_init( apollo_bt_a2dp_sink_t *a2dp_sink );
wiced_result_t bt_audio_decoder_context_deinit( apollo_bt_a2dp_sink_t *a2dp_sink );
wiced_result_t bt_audio_reset_decoder_config( apollo_bt_a2dp_sink_t *a2dp_sink );
wiced_result_t bt_audio_configure_decoder( apollo_bt_a2dp_sink_t *a2dp_sink, wiced_bt_a2dp_codec_info_t* codec_config );
wiced_result_t bt_audio_write_to_decoder_queue( apollo_bt_a2dp_sink_t *a2dp_sink, bt_audio_codec_data_t* audio );

#ifdef __cplusplus
} /* extern "C" */
#endif
