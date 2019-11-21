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
 * Bluetooth Audio AVDT Sink Application
 *
 * The application demonstrates the following features:
 *  - Bluetooth intialization
 *  - A2DP sink
 *  - SBC Decoding
 *  - Audio playback
 *
 * Usage:
 *    On startup device will be discoverable and connectable,
 *    allowing a BT audio source to connect and stream audio.
 *
 * Notes: Currently supports 44.1kHz and 48kHz audio
 */

#include <stdlib.h>
#include "wiced.h"
#include "wiced_rtos.h"
#include "wiced_result.h"
#include "platform_audio.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "bluetooth_audio.h"
#include "bluetooth_a2dp.h"
#include "bluetooth_hfp.h"

#include "bluetooth_dm.h"
#ifdef USE_MEM_POOL
#include "mem_pool.h"
#endif

/******************************************************
 *                    Constants
 ******************************************************/
#define BT_AUDIO_TASK_PRIORITY_BASE     ( 6 )
#define BT_AUDIO_PLAYER_TASK_PRIORITY   ( BT_AUDIO_TASK_PRIORITY_BASE )
#define BT_AUDIO_RECORDER_TASK_PRIORITY    ( BT_AUDIO_TASK_PRIORITY_BASE + 1 )
#define BT_AUDIO_DECODER_TASK_PRIORITY    ( BT_AUDIO_TASK_PRIORITY_BASE + 2 )

#define PLAYER_STACK_SIZE                   0x1000
#define DECODER_STACK_SIZE                0x1000
#define RECORDER_STACK_SIZE               0x1000

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/
wiced_thread_t         decoder_thread;
wiced_thread_t         player_thread;
wiced_thread_t         recorder_thread;
uint8_t decoder_stack[DECODER_STACK_SIZE]__attribute__((section (".ccm")));
uint8_t player_stack[PLAYER_STACK_SIZE]__attribute__((section (".ccm")));
uint8_t recorder_stack[RECORDER_STACK_SIZE]__attribute__((section (".ccm")));

#ifdef USE_MEM_POOL
bt_buffer_pool_handle_t  mem_pool;
bt_buffer_pool_handle_t  mem_pool_pcm;
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t bt_audio_context_init( void )
{
    wiced_result_t result;
#ifdef USE_MEM_POOL
    result = bt_buffer_pool_init(&mem_pool, 30, 660);
    if(result != WICED_SUCCESS)
    {
        return result;
    }
    result = bt_buffer_pool_init(&mem_pool_pcm, 30, 660);
    if(result != WICED_SUCCESS)
    {
        return result;
    }
#endif
    result = bt_audio_decoder_context_init();
    if(result != WICED_SUCCESS)
    {
        return result;
    }

    result = bt_audio_init_device(BT_AUDIO_DEVICE_PLAYER);
    result = bt_audio_init_device(BT_AUDIO_DEVICE_RECORDER);

    return result;
}

void bt_audio_bluetooth_service_init( void )
{
    wiced_result_t result;

    WPRINT_APP_INFO ( ("Starting Bluetooth...\n") );

    result = bt_audio_context_init();
    if(result != WICED_SUCCESS)
    {
        WPRINT_APP_ERROR( ( "Failed to initialize Bluetooth App Context variables... aborting.\n" ) );
        return;
    }

    wiced_rtos_create_thread_with_stack( &player_thread, BT_AUDIO_PLAYER_TASK_PRIORITY, "PLAYER TASK", bt_audio_player_task, (UINT16 *)player_stack, PLAYER_STACK_SIZE, NULL );
    wiced_rtos_create_thread_with_stack( &decoder_thread, BT_AUDIO_DECODER_TASK_PRIORITY, "DECODER TASK", bt_audio_decoder_task, (UINT16 *)decoder_stack, DECODER_STACK_SIZE, NULL );
    wiced_rtos_create_thread_with_stack( &recorder_thread, BT_AUDIO_RECORDER_TASK_PRIORITY, "RECORDER TASK", bta_wiced_audio_recorder_task, (UINT16 *)recorder_stack, RECORDER_STACK_SIZE, NULL );
    /* Bluetooth core service comprises of the functionalities like Bluetooth Device Manager, Service-discovery, Device-discovery,
     * etc. Depending on the use-case or complexity HoGP(in future) might fall under Bluetooth core service
     * or will be put as a different service if really required.
     */
    bt_audio_add_a2dp_service();
    bt_audio_add_hfp_service();
    bt_audio_enable_bluetooth();
}
