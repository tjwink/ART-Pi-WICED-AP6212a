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
#include "wiced_rtos.h"
#include "wiced.h"
#include "wiced_duktape.h"
#include "callback_loop.h"
#include "timer.h"
#include "wss.h"
#include "xhr.h"
#include "audio.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define LOG_LABEL           "callback_loop"
#define LOG_DEBUG_ENABLE    0

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
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
static wiced_queue_t wiced_duktape_callback_queue;
static int wiced_duktape_callback_queue_initialized;
static int loopback_state;

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_duktape_callback_post_event( wiced_duktape_callback_queue_element_t *entry )
{
    wiced_result_t result = WICED_SUCCESS;
    if (wiced_duktape_callback_queue_initialized)
    {
        result = wiced_rtos_push_to_queue( &wiced_duktape_callback_queue, (void *)entry, WICED_NO_WAIT );
        if( result != WICED_SUCCESS )
        {
            LOGE("Failed to push element(%d-%d) to callback queue(%d)", entry->module_id, entry->event_id, result);
        }
    } else {
        LOGI("Duktape callback queue is not initialized yet");
    }
    return result;
}

wiced_result_t wiced_duktape_callback_loop_init( void )
{
    wiced_result_t result;

    result = wiced_rtos_init_queue(&wiced_duktape_callback_queue, "duktape_callback_queue",
                                sizeof(wiced_duktape_callback_queue_element_t), WICED_DUKTAPE_CALLBACK_QUEUE_SIZE );
    if (result == WICED_SUCCESS)
    {
        wiced_duktape_callback_queue_initialized = 1;
    } else {
        LOGE("Failed to initial message queue(%d)", result);
    }
    return result;
}

wiced_result_t wiced_duktape_callback_loop_deinit( void )
{
    if (wiced_duktape_callback_queue_initialized)
    {
        wiced_rtos_deinit_queue(&wiced_duktape_callback_queue);
        wiced_duktape_callback_queue_initialized = 0;
    }
    return 0;
}

void wiced_duktape_callback_loop_break( void )
{
    loopback_state = 0;
}

void wiced_duktape_callback_loop( void )
{
    wiced_result_t result;

    if (!wiced_duktape_callback_queue_initialized) {
        wiced_duktape_callback_loop_init();
    }

    loopback_state = 1;

    while( loopback_state )
    {
        // read and process message in the queue
        wiced_duktape_callback_queue_element_t message;
        result = wiced_rtos_pop_from_queue( &wiced_duktape_callback_queue, &message, WICED_NEVER_TIMEOUT );
        LOGD(" >> CALLBACK: ret=%d, module=%d, event=%d", result, message.module_id, message.event_id);
        if( result != WICED_SUCCESS )
        {
            LOGE("Error receiving from queue (err: %d)", result);
            continue;
        }

        switch ( message.module_id )
        {
            case WDCM_TIMER:
            {
                LOGD("Timer callback");
                timer_callback_handler(&message);
                break;
            }

            case WDCM_WEBSOCKET:
            {
                LOGD("Websocket callback");
                websocket_callback_handler(&message);
                break;
            }

            case WDCM_XMLHTTPREQUEST:
            {
                LOGD("XMLHttpRequest callback");
                xmlhttprequest_callback_handler(&message);
                break;
            }
            case WDCM_AUDIO:
            {
                LOGD("Audio callback");
                audio_callback_handler(&message);
                break;
            }
            default:
                break;
        }
    };
    return;
}
