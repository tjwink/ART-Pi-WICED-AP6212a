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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "duktape.h"
#include "wiced_log.h"

/******************************************************
 *                     Macros
 ******************************************************/

/* Logging macros for uniform log format
 * Notes:
 *  - These macros rely on the wiced_log utility, which means no logs are
 *    printed if wiced_log is not initialized
 *  - Each C file that uses the LOGn macros need to define a LOG_LABEL,
 *    preferrably with a maximum length of 16 characters so it can within
 *    the bounds set by LOG_LABEL_FMT
 *  - If the LOGD (debug level logging) macro is used, then that C file must
 *    also define LOG_DEBUG_ENABLE to either 1 or 0 to enable or disable the
 *    the debug logs, respectively. This extra compile-time mechanism is to
 *    enable per-C-file debug message output.
 */
#define LOG_LABEL_FMT       "[%-16s] "
#define LOGL( level, fmt, ... ) \
    wiced_log_msg( WLF_DUKTAPE, level, LOG_LABEL_FMT fmt "\n", \
                   LOG_LABEL, ##__VA_ARGS__ )

/* LOGn macros, where n={E:error, W:warning, N:notice, I:info, D:debug} */
#define LOGE( fmt, ... )    LOGL( WICED_LOG_ERR,     fmt, ##__VA_ARGS__ )
#define LOGW( fmt, ... )    LOGL( WICED_LOG_WARNING, fmt, ##__VA_ARGS__ )
#define LOGN( fmt, ... )    LOGL( WICED_LOG_NOTICE,  fmt, ##__VA_ARGS__ )
#define LOGI( fmt, ... )    LOGL( WICED_LOG_INFO,    fmt, ##__VA_ARGS__ )
#define LOGD( fmt, ... )                                    \
    do {                                                    \
        if ( LOG_DEBUG_ENABLE )                             \
            LOGL( WICED_LOG_DEBUG0, fmt, ##__VA_ARGS__ );   \
    } while (0)

#ifndef WICED_DUKTAPE_PANDORA_MODULES
#define WICED_DUKTAPE_WORKER_THREAD     (&wiced_duktape_worker_thread)
#endif

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

#ifndef WICED_DUKTAPE_PANDORA_MODULES
typedef duk_thread_state    wiced_duktape_state_t;
#endif

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct wiced_duktape_c_module_s
{
    const char* name;
    duk_ret_t   (*module_reg)( duk_context* ctx );
} wiced_duktape_c_module_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

#ifndef WICED_DUKTAPE_PANDORA_MODULES
extern wiced_worker_thread_t    wiced_duktape_worker_thread;
#endif

/******************************************************
 *               Function Declarations
 ******************************************************/

extern wiced_result_t wiced_duktape_init( const char* filesystem_device );

extern wiced_result_t wiced_duktape_change_filesystem( const char* filesystem_device );

extern wiced_result_t wiced_duktape_add_c_module( wiced_duktape_c_module_t* c_module );

extern wiced_result_t wiced_duktape_load_resources( void );

extern wiced_result_t wiced_duktape_eval_buffer( const char* buffer );

extern wiced_result_t wiced_duktape_eval_file( const char* filename );

extern wiced_result_t wiced_duktape_stop_eval( void );

#ifndef WICED_DUKTAPE_PANDORA_MODULES
extern wiced_result_t wiced_duktape_run_api_tests( const char* test );

extern duk_context* wiced_duktape_get_control( wiced_duktape_state_t* state );

extern void wiced_duktape_put_control( wiced_duktape_state_t* state );

extern wiced_result_t wiced_duktape_schedule_work( void* this, duk_ret_t (*func)( duk_context* ctx ), void* arg );
#endif

#ifdef __cplusplus
}
#endif
