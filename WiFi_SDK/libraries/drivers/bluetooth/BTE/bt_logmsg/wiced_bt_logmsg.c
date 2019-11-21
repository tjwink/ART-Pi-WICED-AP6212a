/******************************************************************************
*
* $ Copyright Broadcom Corporation $
*
*****************************************************************************/


#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "wiced_rtos.h"
#include "wiced_time.h"
#include "wwd_debug.h"
#include "wiced_bt_logmsg.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define TRACE_TYPE_ERROR            0x00000000
#define TRACE_TYPE_MASK             0x0000007f
#define TRACE_GET_TYPE(x)           (((uint32_t)(x)) & TRACE_TYPE_MASK)

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
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/
static wiced_mutex_t global_trace_mutex;
static uint8_t logmsg_initialized;
static uint8_t wiced_log_enabled = 1;

/******************************************************
 *               Function Definitions
 ******************************************************/

void LogMsg_enable_log( uint8_t enable )
{
    wiced_log_enabled = enable?1:0;
}

wiced_result_t LogMsg_init( void )
{
   wiced_result_t ret;
    if( !logmsg_initialized )
    {
        ret = wiced_rtos_init_mutex( &global_trace_mutex );
        if( ret != WICED_SUCCESS )
            return ret;
        logmsg_initialized = 1;
    }
    return WICED_SUCCESS;
}

wiced_result_t LogMsg_cleanup( void )
{
    if( logmsg_initialized )
    {
        wiced_rtos_deinit_mutex( &global_trace_mutex );
        logmsg_initialized = 0;
    }
    return WICED_SUCCESS;
}

void LogMsg( uint32_t trace_set_mask, const char *format, ... )
{
    char buffer[256]; // Save stack space - make global
    char timeBuf[16];
    va_list ap;
    wiced_iso8601_time_t iso8601_time;

    if( !logmsg_initialized )
    {
        if( LogMsg_init( ) )
            return;
    }

    wiced_time_get_iso8601_time( &iso8601_time );

    if ((!wiced_log_enabled) && (TRACE_GET_TYPE(trace_set_mask) != TRACE_TYPE_ERROR))
    {
        /* If  wiced logging disabled, then only log errors */
        return;
    }
    // wiced_iso8601_time_t is a structure containing all the time fields
    // We "fake" a real time *string* by setting the 26th byte to null
    // This gets us HH:MM:SS.SSSSSS (no UTC timezone) when we print from iso8601_time.hour
    // DO NOT CHNANGE THIS!
    memset(((char*)&iso8601_time) + 26, 0, 1);

    sprintf(timeBuf, "%s", iso8601_time.hour);

    va_start(ap, format);
    vsprintf(buffer, format, ap);
    va_end(ap);

    wiced_rtos_lock_mutex(&global_trace_mutex);
    WPRINT_APP_INFO(("%s %s\r\n", timeBuf, buffer)); // Append user message with time
    wiced_rtos_unlock_mutex(&global_trace_mutex);
}

wiced_result_t LogMsg_build_trace_buffer(char* buffer, char* format, ...)
{
    va_list ap;
    int32_t ret;

    va_start(ap, format);
    ret = vsprintf(buffer, format, ap);
    va_end(ap);
    return (ret < 0) ? WICED_ERROR : WICED_SUCCESS;
}
