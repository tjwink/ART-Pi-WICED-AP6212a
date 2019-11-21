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

/**
 * @file Wiced log routines
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "wiced_result.h"
#include "wiced_time.h"
#include "wiced_rtos.h"

#include "wiced_log.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#ifndef WICED_LOGBUF_SIZE
#define WICED_LOGBUF_SIZE (1024)
#endif

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    wiced_bool_t init;
    wiced_mutex_t mutex;
    WICED_LOG_LEVEL_T loglevel[WLF_MAX];
    wiced_time_t start_time;
    char logbuf[WICED_LOGBUF_SIZE];
    int seq_num;
    log_output platform_log;
    platform_get_time platform_time;
} wiced_log_data_t;

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

static wiced_log_data_t wiced_log;

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_log_init(WICED_LOG_LEVEL_T level, log_output platform_output, platform_get_time platform_time)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_time_t ts;
    int i;

    wiced_jump_when_not_true( (wiced_log.init == WICED_FALSE) , _exit );

    /*
     * Note our starting time.
     */

    result = wiced_time_get_time( &ts );
    wiced_action_jump_when_not_true( (result == WICED_SUCCESS) , _exit, result = WICED_ERROR );
    wiced_log.start_time = ts;

    /*
     * Create our mutex.
     */

    result = wiced_rtos_init_mutex( &wiced_log.mutex );
    wiced_action_jump_when_not_true( (result == WICED_SUCCESS) , _exit, result = WICED_ERROR );

    /*
     * And set the starting log level.
     */

    if ( (int)level < 0 )
    {
        level = 0;
    }
    if ( level >= WICED_LOG_MAX )
    {
        level = WICED_LOG_MAX - 1;
    }

    for (i = 0; i < WLF_MAX; i++)
    {
        wiced_log.loglevel[i] = level;
    }

    /*
     * Finally set the platform output and time routines.
     */

    wiced_log.platform_log  = platform_output;
    wiced_log.platform_time = platform_time;

    /*
     * All done.
     */

    wiced_log.init = WICED_TRUE;

_exit:
    return result;
}


wiced_result_t wiced_log_shutdown(void)
{
    wiced_result_t result = WICED_SUCCESS;

    wiced_action_jump_when_not_true( (wiced_log.init == WICED_TRUE) , _exit, result = WICED_ERROR );

    wiced_log.init = WICED_FALSE;

    result = wiced_rtos_deinit_mutex( &wiced_log.mutex );
    wiced_action_jump_when_not_true( (result == WICED_SUCCESS) , _exit, result = WICED_ERROR );

_exit:
    return result;
}


wiced_result_t wiced_log_set_platform_output(log_output platform_output)
{
    wiced_result_t result = WICED_SUCCESS;

    wiced_action_jump_when_not_true( (wiced_log.init == WICED_TRUE) , _exit, result = WICED_ERROR );

    wiced_log.platform_log = platform_output;

_exit:
    return result;
}


wiced_result_t wiced_log_set_platform_time(platform_get_time platform_time)
{
    wiced_result_t result = WICED_SUCCESS;

    wiced_action_jump_when_not_true( (wiced_log.init == WICED_TRUE) , _exit, result = WICED_ERROR );

    wiced_log.platform_time = platform_time;

_exit:
    return result;
}


wiced_result_t wiced_log_set_facility_level(WICED_LOG_FACILITY_T facility, WICED_LOG_LEVEL_T level)
{
    wiced_result_t result = WICED_SUCCESS;

    wiced_action_jump_when_not_true( (wiced_log.init == WICED_TRUE) , _exit, result = WICED_ERROR );

    if ((int)facility < 0 || facility >= WLF_MAX)
    {
        facility = WLF_DEF;
    }

    if ((int)level < 0)
    {
        level = 0;
    }
    if (level >= WICED_LOG_MAX)
    {
        level = WICED_LOG_MAX - 1;
    }
    wiced_log.loglevel[facility] = level;

_exit:
    return result;
}


wiced_result_t wiced_log_set_all_levels(WICED_LOG_LEVEL_T level)
{
    wiced_result_t result = WICED_SUCCESS;
    int i;

    wiced_action_jump_when_not_true( (wiced_log.init == WICED_TRUE) , _exit, result = WICED_ERROR );

    if ((int)level < 0)
    {
        level = 0;
    }
    if (level >= WICED_LOG_MAX)
    {
        level = WICED_LOG_MAX - 1;
    }

    for (i = 0; i < WLF_MAX; i++)
    {
        wiced_log.loglevel[i] = level;
    }

_exit:
    return result;
}


WICED_LOG_LEVEL_T wiced_log_get_facility_level(WICED_LOG_FACILITY_T facility)
{
    WICED_LOG_LEVEL_T local_loglevel = WICED_LOG_OFF;

    wiced_jump_when_not_true( (wiced_log.init == WICED_TRUE) , _exit );

    if ((int)facility < 0 || facility >= WLF_MAX)
    {
        facility = WLF_DEF;
    }

    local_loglevel = wiced_log.loglevel[facility];

_exit:
    return local_loglevel;
}


wiced_result_t wiced_log_msg(WICED_LOG_FACILITY_T facility, WICED_LOG_LEVEL_T level, const char *fmt, ...)
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_time_t now;
    wiced_time_t time_from_start;
    int hrs, mins, secs, ms;
    va_list args;
    int length;
    int len;

    wiced_action_jump_when_not_true( (wiced_log.init == WICED_TRUE) , _exit, result = WICED_ERROR );

    /*
     * Is logging enabled?
     */

    if ((int)facility < 0 || facility >= WLF_MAX)
    {
        facility = WLF_DEF;
    }

    wiced_jump_when_not_true( ((wiced_log.loglevel[facility] != WICED_LOG_OFF) && (level <= wiced_log.loglevel[facility])), _exit);
    wiced_jump_when_not_true( (wiced_log.platform_log != NULL), _exit);

    /*
     * Create the time stamp.
     */

    if (wiced_log.platform_time != NULL)
    {
        result = wiced_log.platform_time(&time_from_start);
    }
    else
    {
        result = wiced_time_get_time(&now);
        time_from_start = now - wiced_log.start_time;
    }
    wiced_action_jump_when_not_true( (result == WICED_SUCCESS) , _exit, result = WICED_ERROR );

    ms   = time_from_start % 1000;
    time_from_start /= 1000;

    secs = time_from_start % 60;
    time_from_start /= 60;

    mins = time_from_start % 60;
    hrs  = (time_from_start / 60) % 24;

    /*
     * We use a common buffer for composing the log messages so we need to protect against
     * multiple threads calling us simultaneously.
     */

    result = wiced_rtos_lock_mutex( &wiced_log.mutex );
    wiced_action_jump_when_not_true( (result == WICED_SUCCESS) , _exit, result = WICED_ERROR );

    len = snprintf(wiced_log.logbuf, WICED_LOGBUF_SIZE, "%04d %02d:%02d:%02d.%03d ", wiced_log.seq_num, hrs, mins, secs, ms);

    wiced_log.seq_num++;

    va_start(args, fmt);
    length = vsnprintf(&wiced_log.logbuf[len], WICED_LOGBUF_SIZE - len, fmt, args);
    if ( (length == -1) || (length >= WICED_LOGBUF_SIZE) )
    {
        /* The output to the buffer was truncated. */
        wiced_log.logbuf[WICED_LOGBUF_SIZE - 1] = '\0';
        length = WICED_LOGBUF_SIZE - 1;
    }
    va_end(args);

    wiced_log.platform_log(level, wiced_log.logbuf);

    result = wiced_rtos_unlock_mutex( &wiced_log.mutex );
    wiced_action_jump_when_not_true( (result == WICED_SUCCESS) , _exit, result = WICED_ERROR );

_exit:
    return result;
}


wiced_result_t wiced_log_printf(const char *fmt, ...)
{
    wiced_result_t result = WICED_SUCCESS;
    va_list args;
    int len;

    wiced_action_jump_when_not_true( (wiced_log.init == WICED_TRUE) , _exit, result = WICED_ERROR );
    wiced_jump_when_not_true( (wiced_log.platform_log != NULL), _exit);

    /*
     * We use a common buffer for composing the log messages so we need to protect against
     * multiple threads calling us simultaneously.
     */

    result = wiced_rtos_lock_mutex( &wiced_log.mutex );
    wiced_action_jump_when_not_true( (result == WICED_SUCCESS) , _exit, result = WICED_ERROR );

    va_start(args, fmt);
    len = vsnprintf(wiced_log.logbuf, WICED_LOGBUF_SIZE, fmt, args);
    if ( (len == -1) || (len >= WICED_LOGBUF_SIZE) )
    {
        /* The output to the buffer was truncated. */
        wiced_log.logbuf[WICED_LOGBUF_SIZE - 1] = '\0';
        len = WICED_LOGBUF_SIZE - 1;
    }
    va_end(args);

    wiced_log.platform_log(WICED_LOG_PRINTF, wiced_log.logbuf);

    result = wiced_rtos_unlock_mutex( &wiced_log.mutex );
    wiced_action_jump_when_not_true( (result == WICED_SUCCESS) , _exit, result = WICED_ERROR );

_exit:
    return result;
}


wiced_result_t wiced_log_vprintf(const char *fmt, va_list varg)
{
    wiced_result_t result = WICED_SUCCESS;
    int len;

    wiced_action_jump_when_not_true( (wiced_log.init == WICED_TRUE) , _exit, result = WICED_ERROR );
    wiced_jump_when_not_true( (wiced_log.platform_log != NULL), _exit);

    /*
     * We use a common buffer for composing the log messages so we need to protect against
     * multiple threads calling us simultaneously.
     */

    result = wiced_rtos_lock_mutex( &wiced_log.mutex );
    wiced_action_jump_when_not_true( (result == WICED_SUCCESS) , _exit, result = WICED_ERROR );

    len = vsnprintf(wiced_log.logbuf, WICED_LOGBUF_SIZE, fmt, varg);
    if ( (len == -1) || (len >= WICED_LOGBUF_SIZE) )
    {
        /* The output to the buffer was truncated. */
        wiced_log.logbuf[WICED_LOGBUF_SIZE - 1] = '\0';
        len = WICED_LOGBUF_SIZE - 1;
    }

    wiced_log.platform_log(WICED_LOG_PRINTF, wiced_log.logbuf);

    result = wiced_rtos_unlock_mutex( &wiced_log.mutex );
    wiced_action_jump_when_not_true( (result == WICED_SUCCESS) , _exit, result = WICED_ERROR );

_exit:
    return result;
}
