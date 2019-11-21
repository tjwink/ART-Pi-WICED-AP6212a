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
 * WICED Over The Air 2 Background Service interface (OTA2)
 *
 *        ***  PRELIMINARY - SUBJECT TO CHANGE  ***
 *
 * The OTA2 Service will periodically check and perform OTA updates
 *
 *  if no callback is registered
 *      OTA2 Service will perform default actions:
 *          - check for first update at initial_check_interval_seconds
 *          - check for updates at check_interval_seconds
 *          - download updates when available
 *          - extract & perform update on next power cycle
 *  else
 *      Inform the application via callback
 *          If Application returns WICED_SUCCESS
 *              OTA2 Service will perform default action
 *          If application returns WICED_ERROR
 *              Application will perform action
 */

#include <ctype.h>
#include "wiced.h"
#include "wiced_time.h"
#include "wiced_tcpip.h"
#include "wiced_rtos.h"
#include "wiced_log.h"
#include "http_stream.h"
#include "wiced_ota2_image.h"
#include "wiced_ota2_network.h"
#include "wiced_ota2_service.h"

/* define to show a bar graph on the console to show download progress */
#define SHOW_BAR_GRAPH
#define BAR_GRAPH_LENGTH 50

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define WICED_OTA2_SERVICE_TAG_VALID            0x61233216
#define WICED_OTA2_SERVICE_TAG_INVALID          0x61DEAD16

#define WICED_OTA2_WORKER_THREAD_PRIORITY       (WICED_DEFAULT_WORKER_PRIORITY)

#define WICED_OTA2_WORKER_STACK_SIZE            ( 5 * 1024)

#define WICED_OTA2_THREAD_SHUTDOWN_WAIT         (      100)  /* wait for a shutdown flag */
#define WICED_OTA2_WORKER_FLAGS_TIMEOUT         (       -1)  /* wait forever             */

#define WICED_OTA2_ANY_PORT                     (        0)

#define WICED_OTA2_HEADER_BUFFER_SIZE           (     1024)  /* large enough for the ota2 header + all component headers */

#define WAIT_FOR_APP_TO_SHUTDOWN_NETWORK_MS     (    20000)  /* After sending a OTA2_SERVICE_STATE_TIME_TO_CHECK_FOR_UPDATES callback notification,
                                                          * and the Application wants us to start the Download, we wait this long before
                                                          * changing the Network to connect to the defined AP.
                                                          * If the application notifies us sooner, we will start sooner.
                                                          */
#ifndef SECONDS_PER_MINUTE
#define SECONDS_PER_MINUTE                      (       60)
#endif

#define WICED_OTA2_TCP_CONNECT_TIMEOUT_MS       (  5000 )
#define WICED_OTA2_TCP_RECEIVE_TIMEOUT_MS       (  1000 )

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                  Enumerations
 ******************************************************/
typedef enum
{
    OTA2_EVENT_WORKER_THREAD_SHUTDOWN_NOW   = (1 <<  0),

    OTA2_EVENT_WORKER_START_INITIAL_TIMER   = (1 <<  1),
    OTA2_EVENT_WORKER_START_NEXT_TIMER      = (1 <<  2),
    OTA2_EVENT_WORKER_START_RETRY_TIMER     = (1 <<  3),

    OTA2_EVENT_WORKER_CHECK_FOR_UPDATES     = (1 <<  4),
    OTA2_EVENT_WORKER_START_DOWNLOAD        = (1 <<  5),

} OTA2_EVENTS_T;

#define OTA2_EVENT_WORKER_THREAD_EVENTS  ( OTA2_EVENT_WORKER_THREAD_SHUTDOWN_NOW |                                                                            \
                                           OTA2_EVENT_WORKER_START_INITIAL_TIMER | OTA2_EVENT_WORKER_START_NEXT_TIMER | OTA2_EVENT_WORKER_START_RETRY_TIMER | \
                                           OTA2_EVENT_WORKER_CHECK_FOR_UPDATES   | OTA2_EVENT_WORKER_START_DOWNLOAD  )

typedef enum
{
    OTA2_SERVICE_STATE_NOT_INITIALIZED  = 0,
    OTA2_SERVICE_STATE_INITIALIZED,                 /* 0x01 session has been initialized                            */

    OTA2_SERVICE_STATE_NONE,                        /* 0x02 background service has been started                     */

    OTA2_SERVICE_STATE_WAITING_FOR_TIMER,           /* 0x03 disconnected, timers are set, wait for a timed action   */

    OTA2_SERVICE_STATE_TIME_TO_CHECK_FOR_UPDATES,   /* 0x04 check to see if app wants to check for updates          */

    OTA2_SERVICE_STATE_STARTING_DOWNLOAD,           /* 0x05 starting the download                                   */

    OTA2_SERVICE_STATE_DOWNLOAD_DONE,               /* 0x06 TCP connected, done receiving DATA                      */

    OTA2_SERVICE_STATE_MAX                          /* Must be last */

} ota2_state_t;

/******************************************************
 *                    Structures
 ******************************************************/

/* session structure */
typedef struct wiced_ota2_service_session_s
{
        uint32_t                                tag;

        wiced_utc_time_ms_t                     session_start_time; /* date, time session was initialized - used to set timer   */
        wiced_utc_time_ms_t                     session_next_time;  /* date, time next timer                                    */
        wiced_utc_time_ms_t                     session_retry_time; /* time when last retry timer was started (0 = not active)  */
        wiced_utc_time_ms_t                     session_download_now_time;  /* when we actually start the download              */
        wiced_utc_time_ms_t                     session_last_time;  /* date, time we last checked for an update                 */

        wiced_utc_time_ms_t                     session_last_download_start; /* date, time we last started update               */
        wiced_utc_time_ms_t                     session_last_download_stop;  /* date, time we last update stopped               */


        volatile ota2_state_t                   ota2_state;
        wiced_ota2_service_status_t             last_error_status;  /* last error status                                        */

        wiced_time_t                            last_timer_value;   /* timer value                                              */
        wiced_time_t                            retry_timer_value;  /* timer value                                              */

        wiced_mutex_t                           ota2_mutex;         /* for updating internal state */

        ota2_service_callback                   cb_function;        /*  Application callback function and data                  */
        void*                                   cb_opaque;

        wiced_thread_t                          ota2_worker_thread;
        volatile wiced_thread_t*                ota2_worker_thread_ptr;

        wiced_event_flags_t                     ota2_worker_flags;  /* to signal worker thread */

        /* connect & download info */

        wiced_interface_t           ota2_interface;     /* use STA or Ethernet ? */

        wiced_config_ap_entry_t*    ota2_ap_info;       /* Alternate AP to use to connect to the OTA2 update server
                                                         * - This is optional. If the default AP has access to the
                                                         *   OTA2 update server, this can be NULL
                                                         */

        wiced_config_ap_entry_t*    default_ap_info;    /* Default AP to connect to after the OTA2 update is complete
                                                         * This is optional. If the default AP has access to the OTA2
                                                         * update server, this will be NULL
                                                         * - If the application needs a special access point
                                                         *   connection, and does not wish the OTA2 code to re-connect,
                                                         *   Set this to NULL.
                                                         */

        uint8_t                     ota2_ap_list_count; /* number of APs to try to connect to for OTA2 updating */
        wiced_config_ap_entry_t*    ota2_ap_list;       /* Alternate AP list to use to connect to the OTA2 update server
                                                         * - This is optional. This can be NULL
                                                         */

        wiced_config_ap_entry_t*    curr_network;       /* ptr equals current network info if connected to an AP.
                                                         * if NULL, we are not connected to a network
                                                         */

        char                            http_query[WICED_OTA2_HTTP_QUERY_SIZE];       /* for building the http query */
        char                            host_name[WICED_OTA2_HOST_NAME];
        char                            file_path[WICED_OTA2_FILE_PATH];
        wiced_tcp_socket_t              tcp_socket;
        wiced_bool_t                    tcp_socket_created;

        char                            last_connected_host_name[MAX_HTTP_HOST_NAME_SIZE + 1];

        uint32_t                        initial_check_interval_seconds; /* seconds before 1st update check                                  */
        uint32_t                        check_interval_seconds;         /* seconds between start of one check and start of next check       */
        uint32_t                        retry_check_interval_seconds;   /* seconds between re-try if initial contact to
                                                                         * server for update info fails
                                                                         * 0 = wait until next check_interval_seconds (no retry)            */
        uint16_t                        max_retries;            /* maximum retries per update attempt                                       */
        uint16_t                        retry_count;            /* current retry count                                                      */

        uint8_t                         auto_update;            /* Callback return value over-rides this parameter
                                                                 * Auto-update behavior if no callback registered.
                                                                 *   1 = Service will inform Bootloader to extract
                                                                 *       and update on next power cycle after download
                                                                 *   0 = Service will inform Bootloader that download
                                                                 *       is complete - Bootloader will NOT extract/update
                                                                 *       until user / application requests                  */

        /* timer */
        wiced_timer_t                   check_update_timer;
        wiced_timer_t                   retry_timer;          /* for starting a re-try, keep separate from the main timer */
        wiced_timer_t                   download_now_timer;   /* for starting the actual download                         */

        uint8_t                        header_buffer[WICED_OTA2_HEADER_BUFFER_SIZE];  /* for checking if the update is valid                      */

        /* debugging */
        int                             attempted_updates;
        int                             successful_updates;
        int                             failed_updates;
        int                             download_failures;
        int                             ota2_ap_failures;
        int                             tcp_failures;
        int                             tcp_timeouts;
        int                             default_ap_failures;

} wiced_ota2_service_session_t;

static wiced_ota2_service_session_t *g_only_one_session_at_a_time = NULL;

/******************************************************
 *               Variables Definitions
 ******************************************************/
/* template for HTTP GET */
static char ota2_get_request_template[] =
{
    "GET %s HTTP/1.1\r\n"
    "Host: %s%s \r\n"
    "\r\n"
};

/* for printing time values */
static char time_string[ sizeof(wiced_iso8601_time_t) + 1];

/****************************************************************
 *  Function Definitions
 ****************************************************************/

static ota2_state_t ota2_service_get_state(wiced_ota2_service_session_t* session);
static void ota2_service_stop_timer(wiced_ota2_service_session_t* session);
static void ota2_service_stop_retry_timer(wiced_ota2_service_session_t* session);
static void ota2_service_stop_download_now_timer(wiced_ota2_service_session_t* session);

/****************************************************************
 *  Internal functions
 ****************************************************************/
static void wiced_ota2_check_update_timer_handler( void* arg )
{
    wiced_ota2_service_session_t*      session;

    wiced_assert("wiced_ota2_check_update_timer_handler() ARG == NULL!", (arg != 0));
    session = (wiced_ota2_service_session_t*)arg;

    /* signal the worker thread to re-start this timer */
    wiced_rtos_set_event_flags(&session->ota2_worker_flags, OTA2_EVENT_WORKER_START_NEXT_TIMER);
    /* signal the worker thread to check if there is an update available */
    wiced_rtos_set_event_flags(&session->ota2_worker_flags, OTA2_EVENT_WORKER_CHECK_FOR_UPDATES);
}

static void wiced_ota2_retry_timer_handler( void* arg )
{
    wiced_ota2_service_session_t*      session;

    wiced_assert("wiced_ota2_retry_timer_handler() ARG == NULL!", (arg != 0));
    session = (wiced_ota2_service_session_t*)arg;

    /* do not trigger another timer start */
    /* signal the worker thread to check if there is an update available */
    wiced_rtos_set_event_flags(&session->ota2_worker_flags, OTA2_EVENT_WORKER_CHECK_FOR_UPDATES);
}

static void wiced_ota2_download_now_timer_handler( void* arg )
{
    wiced_ota2_service_session_t*   session;
    ota2_state_t                    state;

    wiced_assert("wiced_ota2_download_now_timer_handler() ARG == NULL!", (arg != 0));
    session = (wiced_ota2_service_session_t*)arg;

    state = ota2_service_get_state(session);
    if (state == OTA2_SERVICE_STATE_TIME_TO_CHECK_FOR_UPDATES)
    {
        /* signal the worker thread to start the download */
        wiced_rtos_set_event_flags(&session->ota2_worker_flags, OTA2_EVENT_WORKER_START_DOWNLOAD);
    }
}

static wiced_result_t ota2_service_start_timer(wiced_ota2_service_session_t* session, wiced_time_t time)
{
    if (session == NULL )
    {
        return WICED_ERROR;
    }

    if (wiced_rtos_is_current_thread( &session->ota2_worker_thread ) != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "START timer wrong thread\r\n");
    }

    session->last_timer_value = time;
    session->retry_count = 0;

    /* stop it if it is already running - we want to re-start with a new time */
    ota2_service_stop_timer(session);

    wiced_rtos_init_timer(&session->check_update_timer,  time, wiced_ota2_check_update_timer_handler, session);
    wiced_rtos_start_timer(&session->check_update_timer );

    return WICED_SUCCESS;
}

static void ota2_service_stop_timer(wiced_ota2_service_session_t* session)
{
    if (session == NULL )
    {
        return;
    }

    if (wiced_rtos_is_current_thread( &session->ota2_worker_thread ) != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "STOP timer wrong thread\r\n");
    }

    if (wiced_rtos_is_timer_running(&session->check_update_timer) == WICED_SUCCESS )
    {
        wiced_rtos_stop_timer(&session->check_update_timer);
        wiced_rtos_deinit_timer(&session->check_update_timer);
    }
}

static wiced_result_t ota2_service_start_retry_timer(wiced_ota2_service_session_t* session, wiced_time_t time)
{
    if (session == NULL )
    {
        return WICED_ERROR;
    }

    if (wiced_rtos_is_current_thread( &session->ota2_worker_thread ) != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "START timer wrong thread\r\n");
    }

    session->retry_timer_value = time;

    /* stop it if it is already running - we want to re-start with a new time */
    ota2_service_stop_retry_timer(session);

    wiced_rtos_init_timer(&session->retry_timer,  time, wiced_ota2_retry_timer_handler, session);
    wiced_rtos_start_timer(&session->retry_timer );

    return WICED_SUCCESS;
}

static void ota2_service_stop_retry_timer(wiced_ota2_service_session_t* session)
{
    if (session == NULL )
    {
        return;
    }

    if (wiced_rtos_is_current_thread( &session->ota2_worker_thread ) != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "STOP retry timer wrong thread\r\n");
    }

    if (wiced_rtos_is_timer_running(&session->retry_timer) == WICED_SUCCESS )
    {
        wiced_rtos_stop_timer(&session->retry_timer);
        wiced_rtos_deinit_timer(&session->retry_timer);
    }
}

static wiced_result_t ota2_service_start_download_now_timer(wiced_ota2_service_session_t* session, wiced_time_t time)
{
    if (session == NULL )
    {
        return WICED_ERROR;
    }

    if (wiced_rtos_is_current_thread( &session->ota2_worker_thread ) != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "START download now timer wrong thread\r\n");
    }

    /* stop it if it is already running - we want to re-start with a new time */
    ota2_service_stop_download_now_timer(session);

    wiced_rtos_init_timer(&session->download_now_timer, time, wiced_ota2_download_now_timer_handler, session);
    wiced_rtos_start_timer(&session->download_now_timer );

    return WICED_SUCCESS;
}

static void ota2_service_stop_download_now_timer(wiced_ota2_service_session_t* session)
{
    if (session == NULL )
    {
        return;
    }

    if (wiced_rtos_is_current_thread( &session->ota2_worker_thread ) != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "STOP download now timer wrong thread\r\n");
    }

    if (wiced_rtos_is_timer_running(&session->download_now_timer) == WICED_SUCCESS )
    {
        wiced_rtos_stop_timer(&session->download_now_timer);
        wiced_rtos_deinit_timer(&session->download_now_timer);
    }
    session->retry_timer_value = 0;
}

static char *wiced_ota2_service_convert_ms_to_time_string( wiced_time_t time)
{
    wiced_iso8601_time_t            iso8601_time;

    wiced_time_convert_utc_ms_to_iso8601( time, &iso8601_time );
    iso8601_time.T = ' ';
    iso8601_time.decimal = 0;
    memcpy(time_string, &iso8601_time, sizeof(wiced_iso8601_time_t));
    time_string[ sizeof(time_string)-1] = 0;
    return time_string;
}

static wiced_result_t  wiced_ota2_service_make_callback(wiced_ota2_service_session_t* session, wiced_ota2_service_status_t status, uint32_t value )
{
    wiced_result_t  cb_result = WICED_SUCCESS;

    if ((session != NULL) && (session->cb_function != NULL) )
    {
        cb_result = (session->cb_function)(session, status, value, session->cb_opaque);
    }

    return cb_result;
}

static wiced_result_t wiced_ota2_service_make_error_callback(wiced_ota2_service_session_t* session, wiced_ota2_service_status_t error_status)
{
    wiced_result_t              cb_result = WICED_SUCCESS;

    if ( session == NULL)
    {
        return WICED_BADARG;
    }
    session->last_error_status = error_status;

    cb_result = wiced_ota2_service_make_callback(session, error_status, 0 );

    return cb_result;
}


static ota2_state_t ota2_service_get_state(wiced_ota2_service_session_t* session)
{
    if (session == NULL)
    {
        return OTA2_SERVICE_STATE_NOT_INITIALIZED;
    }

    return session->ota2_state;
}

static wiced_result_t ota2_service_set_state(wiced_ota2_service_session_t* session, ota2_state_t new_state)
{
    if (wiced_rtos_lock_mutex( &session->ota2_mutex ) != WICED_SUCCESS)
    {
         wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "ota2_service_set_state() fail!\r\n");
        return WICED_ERROR;
    }

    session->ota2_state = new_state;

    wiced_rtos_unlock_mutex( &session->ota2_mutex );

    return WICED_SUCCESS;
}

/****************************************************************
 *  AP Network Up / Down
 ****************************************************************/

/*
 * Connects to session->ota2_ap_info if provided.
 *
 */
static wiced_result_t wiced_ota2_service_network_switch_to_ota2_ap(wiced_ota2_service_session_t* session )
{
    wiced_result_t result;

    if (session->ota2_ap_info == NULL )
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "wiced_ota2_service_network_switch_to_ota2_ap() No network to switch to.\r\n");
        return WICED_SUCCESS;
    }

    if (session->curr_network == session->ota2_ap_info )
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "wiced_ota2_service_network_switchto_ota2_ap() already connected to: %.*s\r\n", session->ota2_ap_info->details.SSID.length, session->ota2_ap_info ->details.SSID.value);
        return WICED_SUCCESS;
    }

    if (session->curr_network != NULL)
    {
        /* bring down current AP, we want to connect to a different one */
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "wiced_ota2_service_network_switch_to_ota2_ap() disconnecting\r\n");
        result = wiced_ota2_network_down(session->ota2_interface) ;
        if (result != WICED_SUCCESS)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "wiced_ota2_service_network_switch_to_ota2_ap() wiced_ota2_network_down() %d\r\n", result);
        }
        session->curr_network = NULL;
    }

    /* connect to the specified AP */
    result = wiced_ota2_network_up(session->ota2_interface, session->ota2_ap_info);

    if (result == WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "Connected to AP: %.*s\r\n", session->ota2_ap_info ->details.SSID.length, session->ota2_ap_info ->details.SSID.value);
        session->curr_network = session->ota2_ap_info ;
    }
    else
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "Connection to :%.*s FAILED !!!!!\r\n", session->ota2_ap_info ->details.SSID.length, session->ota2_ap_info ->details.SSID.value);
    }

    return result;
}

static wiced_result_t wiced_ota2_service_network_switch_to_default_ap(wiced_ota2_service_session_t* session )
{
    wiced_result_t result;

    if (session->ota2_interface == WICED_ETHERNET_INTERFACE)
    {
        /* Need to connect to an AP using Ethernet !!! */
        return WICED_SUCCESS;
    }

    if (session->default_ap_info == NULL )
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "wiced_ota2_service_network_switch_to_default_ap() NO network to switch to.\r\n");

        if (session->curr_network != NULL)
        {
            /* bring down current AP */
            wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "wiced_ota2_service_network_switch_to_default_ap() disconnecting\r\n");
            result = wiced_ota2_network_down(session->ota2_interface);
            if (result != WICED_SUCCESS)
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "wiced_ota2_service_network_switch_to_ota2_ap() wiced_ota2_network_down() %d\r\n", result);
            }
            session->curr_network = NULL;
        }
        return WICED_SUCCESS;
    }

    if (session->curr_network == session->default_ap_info )
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "wiced_ota2_service_network_switchto_default_ap() already connected to: %.*s\r\n", session->ota2_ap_info->details.SSID.length, session->ota2_ap_info ->details.SSID.value);
        return WICED_SUCCESS;
    }

    if (session->curr_network != NULL)
    {
        /* bring down current AP, we want to connect to a different one */
        wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "wiced_ota2_service_network_switch_to_default_ap() disconnecting\r\n");
        result = wiced_ota2_network_down(session->ota2_interface);
        if (result != WICED_SUCCESS)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "wiced_ota2_service_network_switch_to_ota2_ap() wiced_ota2_network_down() %d\r\n", result);
        }
        session->curr_network = NULL;
    }


    result = wiced_ota2_network_up(session->ota2_interface, session->default_ap_info );
    if (result == WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "Connected to :%.*s\r\n", session->ota2_ap_info->details.SSID.length, session->ota2_ap_info ->details.SSID.value);
        session->curr_network = session->default_ap_info ;
    }
    else
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "Connection to :%.*s FAILED !!!!!\r\n", session->ota2_ap_info->details.SSID.length, session->ota2_ap_info ->details.SSID.value);
    }

    return result;
}

/****************************************************************
 *  HTTP URI connect / disconnect Function Declarations
 ****************************************************************/

static wiced_result_t  ota2_service_check_socket_created(wiced_ota2_service_session_t* session)
{
    wiced_result_t result = WICED_ERROR;

    if (session->tcp_socket_created == WICED_FALSE)
    {
        result = wiced_tcp_create_socket( &session->tcp_socket, session->ota2_interface );
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_tcp_create_socket() failed! %d\r\n", result);
            return result;
        }

        result = WICED_SUCCESS;
        session->tcp_socket_created = WICED_TRUE;
    }

    return result;
}

static wiced_result_t  ota2_service_disconnect(wiced_ota2_service_session_t* session)
{
    wiced_result_t result = WICED_SUCCESS;

    if (session->tcp_socket_created == WICED_TRUE)
    {
        result = wiced_tcp_disconnect( &session->tcp_socket );
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "Call wiced_tcp_disconnect() result: %d!\r\n", result);
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_tcp_disconnect() failed!\r\n");
        }

        result = wiced_tcp_delete_socket( &session->tcp_socket );
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "Call wiced_tcp_delete_socket() result: %d!\r\n", result);
        if ( result != WICED_SUCCESS )
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_tcp_delete_socket() failed!\r\n");
        }
    }
    session->tcp_socket_created = WICED_FALSE;

    return result;
}

static wiced_result_t ota2_service_connect(wiced_ota2_service_session_t* session)
{
    wiced_result_t      result = WICED_SUCCESS;
    wiced_ip_address_t  ip_address;
    uint16_t            connect_tries;

    char                host_name[WICED_OTA2_HOST_NAME];
    char                file_path[WICED_OTA2_FILE_PATH];

    result = ota2_service_check_socket_created(session);
    if ( result != WICED_SUCCESS)
    {
        goto _connect_fail;
    }

    strlcpy(host_name, session->host_name, sizeof(host_name));
    strlcpy(file_path, session->file_path, sizeof(file_path));

    wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "Connect to: host:%s path:%s port:%d\r\n", host_name, file_path, WICED_OTA2_HTTP_PORT);

    result = wiced_hostname_lookup( host_name, &ip_address, 10000, WICED_STA_INTERFACE );
    if (result!= WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "ota2_connect() wiced_hostname_lookup(%s) failed!\r\n", host_name);
        goto _connect_fail;
    }

    /* try a few times to actually connect */
    connect_tries = 0;
    result = WICED_ERROR;
    while ((connect_tries < 3) && (result != WICED_SUCCESS) )
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "Try %d Connecting to %s  " IPV4_PRINT_FORMAT " : %d !\r\n",
                       connect_tries, host_name, IPV4_SPLIT_TO_PRINT(ip_address), WICED_OTA2_HTTP_PORT );

        result = wiced_tcp_connect( &session->tcp_socket, &ip_address, WICED_OTA2_HTTP_PORT, WICED_OTA2_TCP_CONNECT_TIMEOUT_MS );
        connect_tries++;
    }
    if ( result != WICED_SUCCESS )
    {
        goto _connect_fail;
    }
    wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "Connected to " IPV4_PRINT_FORMAT " : %d !\r\n",
                                               IPV4_SPLIT_TO_PRINT(ip_address), WICED_OTA2_HTTP_PORT);

    strlcpy(session->last_connected_host_name, host_name, sizeof(session->last_connected_host_name));

    wiced_ota2_service_make_callback(session, OTA2_SERVICE_SERVER_CONNECTED, 0);
    return result;

_connect_fail:
    wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "ota2_service_connect() wiced_tcp_connect() failed! %d\r\n", result);
    ota2_service_disconnect(session);
    return WICED_ERROR;
}

/* get the OTA Image file */
/* pass req_length = 4096 (one sector) if you want only the OTA2 header & component headers
 *                          and store in provided buffer (not saved to Staging area)
 * req_length = -1 means get the whole file.
 */

static wiced_result_t wiced_ota2_service_wget_update(wiced_ota2_service_session_t* session, uint8_t *req_buffer, uint32_t req_length)
{
    wiced_result_t      result;
    char                port_name[16] = "\0";
    wiced_packet_t*     reply_packet;
    uint32_t            offset;
    wiced_bool_t        done;
    uint32_t            content_length;
    uint32_t            wait_loop_count;
    http_header_t       length_header, range_header;
    uint32_t            range_start, range_end;
    const char*         hostname = session->host_name;
    const char*         filename = session->file_path;
    wiced_tcp_socket_t* tcp_socket = &session->tcp_socket;

    length_header.name = "Content-Length";
    length_header.value = NULL;

    range_header.name = "bytes";
    range_header.value = NULL;
    range_start = 0;
    range_end = 0;

    snprintf(port_name, sizeof(port_name), ":%d", WICED_OTA2_HTTP_PORT);

    snprintf(session->http_query, sizeof(session->http_query), ota2_get_request_template, filename, hostname, port_name);

    wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "Send query for OTA file: [%s]\r\n", session->http_query);

    result = wiced_tcp_send_buffer( tcp_socket, session->http_query, (uint16_t)strlen(session->http_query) );
    if ( result != WICED_SUCCESS )
    {
        if (result == WICED_TCPIP_SOCKET_CLOSED)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "wiced_tcp_send_buffer() wiced_tcp_receive() %d socket_closed!\r\n", result);
        }
        else
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "ota2_get_OTA_file() wiced_tcp_send_buffer() failed %d [%s]!\r\n", result, session->http_query);
        }

        return WICED_ERROR;
    }

    reply_packet = NULL;
    content_length = 0;
    result = WICED_SUCCESS;
    done = WICED_FALSE;
    offset = 0;
    wait_loop_count = 0;
    while ((result == WICED_SUCCESS) && (done == WICED_FALSE))
    {
        if (reply_packet != NULL)
        {
            /* free the packet */
            wiced_packet_delete( reply_packet );
        }
        reply_packet = NULL;
        length_header.value = NULL;
        range_header.value = NULL;

        result = wiced_tcp_receive( tcp_socket, &reply_packet, WICED_OTA2_TCP_RECEIVE_TIMEOUT_MS  );
        if (result == WICED_TCPIP_TIMEOUT)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "ota2_get_OTA_file() wiced_tcp_receive() %d timeout!\r\n", result);
            if (reply_packet != NULL)
            {
                /* free the packet */
                wiced_packet_delete( reply_packet );
            }
            reply_packet = NULL;

            if ( wait_loop_count++ < 3)
            {
                result = WICED_SUCCESS; /* so we stay in our loop */
            }
            else
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "ota2_get_OTA() wiced_ota2_write_data() Timed out received:%ld of %ld!\r\n",
                                                          offset, content_length);
                session->tcp_timeouts++;
                goto _wget_fail;
            }
        }
        else if (result == WICED_TCPIP_SOCKET_CLOSED)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "ota2_get_OTA_file() wiced_tcp_receive() %d socket_closed!\r\n", result);
            goto _wget_fail;
        }
        else if (reply_packet != NULL)
        {
            /* for this packet */
            uint8_t*            body;
            uint32_t            body_length;
            http_status_code_t  response_code;

            wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "ota2_get_OTA_file() wiced_tcp_receive() result:%d reply_packet:%p\r\n", result, reply_packet);

            /* we got a packet, clear this timeout counter */
            wait_loop_count = 0;

            if (result != WICED_SUCCESS)
            {
                result = WICED_SUCCESS; /* stay in the loop and try again */
                continue;
            }
            body = NULL;
            body_length = 0;

            response_code = 0;
            result = http_process_response( reply_packet, &response_code );
            wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "http_process_response() result:%d response:%d -- continue\r\n ", result, response_code);
            if (result != WICED_SUCCESS )
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "HTTP response result:%d code: %d, continue to process (could be no header)\r\n ", result, response_code);
                result = WICED_SUCCESS; /* so we try again */
            }
            else
            {

                if (response_code < 100)
                {
                    /* do nothing here */
                }
                else if (response_code < 200 )
                {
                    /* 1xx (Informational): The request was received, continuing process */
                    continue;
                }
                else if (response_code < 300 )
                {
                    /* 2xx (Successful): The request was successfully received, understood, and accepted */
                }
                else if (response_code < 400 )
                {
                    /* 3xx (Redirection): Further action needs to be taken in order to complete the request */
                    wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "HTTP response code: %d, redirection - code needed to handle this!\r\n ", response_code);
                    result = WICED_ERROR;
                    goto _wget_fail;
                }
                else
                {
                    /* 4xx (Client Error): The request contains bad syntax or cannot be fulfilled */
                    wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "HTTP response code: %d, ERROR!\r\n ", response_code);
                    result = WICED_ERROR;
                    goto _wget_fail;
                }
            }

            if (content_length == 0)
            {
                /* we don't know the size of the transfer yet, try to find it */
                if (http_extract_headers( reply_packet, &length_header, 1) == WICED_SUCCESS)
                {
                    if (length_header.value != NULL)
                    {
                        content_length = atol(length_header.value);
                    }
                }
            }
            if (range_start == 0)
            {
                range_end = 0;
                /* we don't know the size of the transfer yet, try to find it */
                if (http_extract_headers( reply_packet, &range_header, 1) == WICED_SUCCESS)
                {
                    if (range_header.value != NULL)
                    {
                        char *minus;
                        range_start = atol(range_header.value);
                        minus = strchr(range_header.value, '-');
                        if (minus != NULL)
                        {
                            /* skip the minus */
                            minus++;
                            range_end = atol(minus);
                        }
                        wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "HTTP DATA RANGE: %ld - %ld \r\n ", range_start, range_end);
                    }
                }
            }

            result = http_get_body( reply_packet, &body, &body_length );
            if ((result != WICED_SUCCESS) || (body == NULL))
            {
                /* get_body can fail if there is no header, try just getting the packet data */
                uint8_t* packet_data;
                uint16_t packet_data_length;
                uint16_t available_data_length;

                wiced_packet_get_data( reply_packet, 0, &packet_data, &packet_data_length, &available_data_length );

                wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "ota2_get_OTA() http_get_body() failed, just use the data: packet_data:%p packet_data_length:%d available_data_length:%d\r\n", packet_data, packet_data_length, available_data_length);

                if ((packet_data != NULL) && (available_data_length != 0) && (available_data_length == packet_data_length))
                {
                    body = packet_data;
                    body_length = packet_data_length;
                    result = WICED_SUCCESS;
                }
            }

            /* if we got data (not a header), save it */
            if ((body != NULL) && (body_length > 0) &&
                !((offset == 0) && (strncasecmp( (char*)body, "HTTP", 4) == 0)) )
            {
                wiced_result_t  cb_result;
                uint32_t        percent_done = 0;



                if ((offset > 0) && (content_length != 0))
                {
                    /* offset -2 so we never hit 100% while in this loop - it is communicated after we are finished */
                    percent_done = (((offset - 2)* 100) / content_length);
                }

#ifdef SHOW_BAR_GRAPH
                /* only print it if we are writing to FLASH. writing to a requested buffer is quick */
                if (req_buffer == NULL)
                {
                    char            bar_graph[BAR_GRAPH_LENGTH] = {0};
                    uint32_t        vert_bar;

                    vert_bar = 0;
                    if (content_length != 0)
                    {
                        vert_bar = (offset * (BAR_GRAPH_LENGTH - 2)) / content_length;
                    }

                    bar_graph[0] = '|';
                    bar_graph[BAR_GRAPH_LENGTH - 2] = '|';
                    memset( &bar_graph[1], '-', (BAR_GRAPH_LENGTH - 3));
                    bar_graph[vert_bar] = '|';
                    bar_graph[BAR_GRAPH_LENGTH - 1] = '\0';

                   wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "%s (%ld%%)\r", bar_graph, percent_done);
                }
#endif

                wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "Got data! offset:%ld body:%p length:%ld\r\n", offset, body, body_length);

                if (req_buffer != NULL)
                {
                    uint32_t chunk_size = body_length;
                    /* only want header read - don't save to staging area */
                    if ((offset + chunk_size) > req_length )
                    {
                        chunk_size = req_length - offset;
                    }

                    wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "ota2_get_update_header() header_buffer:%p size:0x%x (%d)\r\n", session->header_buffer, sizeof(session->header_buffer), sizeof(session->header_buffer));
                    wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "                                  dest:%p offset:%ld chunk:%ld size:%ld end:%p! \r\n", req_buffer, offset, chunk_size, req_length, &req_buffer[offset + chunk_size]);

                    memcpy(&req_buffer[offset], body, chunk_size);
                    offset += chunk_size;
                    if (offset >= req_length)
                    {
                        /* normal finish to this request */
                        result = WICED_SUCCESS;
                        goto _wget_fail;
                    }

                    if ((offset > 0) && (content_length != 0))
                    {
                        /* offset -2 so we never hit 100% while in this loop - it is communicated after we are finished */
                        percent_done = (((offset - 2) * 100) / content_length);
                    }

                    cb_result = wiced_ota2_service_make_callback(session, OTA2_SERVICE_DOWNLOAD_STATUS, percent_done );
                    if (cb_result != WICED_SUCCESS)
                    {
                        /* application said to stop the download */
                        /* drop out of loop */
                        done = WICED_TRUE;
                    }
                }
                else
                {
                    wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "writing! offset:%ld body:%p length:%ld\r\n", offset, body, body_length);

                    result = wiced_ota2_image_write_data(body, offset, body_length);
                    if (result == WICED_SUCCESS)
                    {
                        wiced_result_t  cb_result;
                        offset += body_length;

                        if ((offset > 0) && (content_length != 0))
                        {
                            /* offset -2 so we never hit 100% while in this loop - it is communicated after we are finished */
                            percent_done = (((offset - 2) * 100) / content_length);
                        }
                        cb_result = wiced_ota2_service_make_callback(session, OTA2_SERVICE_DOWNLOAD_STATUS, percent_done );
                        if (cb_result != WICED_SUCCESS)
                        {
                            /* application said to stop the download */
                            /* drop out of loop */
                            done = WICED_TRUE;
                        }
                    }
                    else
                    {
                        wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "wiced_ota2_image_write_data() failed %d\r", result);
                    }
                }
            }
        } /* reply packet != NULL */

        if ((content_length != 0) && (offset >= content_length))
        {
            wiced_ota2_image_update_staged_status(WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE);
            wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "ota2_get_OTA() Finished %ld >= %ld !\r\n", offset, content_length);
            done = WICED_TRUE;
        }

    } /* while result == success && done == WICED_FALSE */

    if (reply_packet != NULL)
    {
        /* free the packet */
        wiced_packet_delete( reply_packet );
    }
    reply_packet = NULL;

    if ((content_length != 0) && (offset < content_length))
    {
        wiced_ota2_image_update_staged_status(WICED_OTA2_IMAGE_INVALID);
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "ota2_get_OTA() FAILED %ld < %ld!\r\n", offset, content_length );
        result = WICED_ERROR;
    }

_wget_fail:
    if (reply_packet != NULL)
    {
        /* free the packet */
        wiced_packet_delete( reply_packet );
    }
    reply_packet = NULL;

    if ((result == WICED_SUCCESS) && (content_length != 0) && (offset >= content_length))
    {
        wiced_ota2_service_make_callback(session, OTA2_SERVICE_DOWNLOAD_STATUS, 100);
    }
    wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "\r\n ota2_get_OTA_file() Exiting %d\r\n", result);

    return result;

}

wiced_result_t wiced_ota2_service_get_the_update_file(wiced_ota2_service_session_t* session, wiced_result_t* update_result)
{
    wiced_result_t  result;

    /* assume error */
    *update_result = WICED_ERROR;

    result = ota2_service_connect(session);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_service_get_the_update() ota2_service_connect() failed!\r\n");
        session->tcp_failures++;
        goto _get_file_fail;
    }

    /* get the header */
    result = wiced_ota2_service_wget_update(session, session->header_buffer, sizeof(session->header_buffer));
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_service_get_the_update() wiced_ota2_service_wget_update(header) failed!\r\n");
        wiced_ota2_service_make_error_callback(session, OTA2_SERVICE_UPDATE_ERROR );
        session->download_failures++;
        goto _get_file_connected_fail;
    }

    /* swap the data to be correct for the platform */
    wiced_ota2_image_header_swap_network_order((wiced_ota2_image_header_t *)session->header_buffer, WICED_OTA2_IMAGE_SWAP_NETWORK_TO_HOST);
    if (wiced_ota2_service_make_callback(session, OTA2_SERVICE_UPDATE_AVAILABLE, (uint32_t)session->header_buffer) != WICED_SUCCESS)
    {
        /* application says do not download, this is not a failure */
        *update_result = WICED_SUCCESS;
        goto _get_file_connected_fail;
    }

    /* disconnect and re-connect to reset the file pointer */
    ota2_service_disconnect(session);
    result = ota2_service_connect(session);
    if ( result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_service_get_the_update() part2: ota2_service_connect() failed!!\r\n");
        session->tcp_failures++;
        goto _get_file_fail;
    }

    /* get the OTA2 file */
    result = wiced_ota2_service_wget_update(session, NULL, 0);
    if (result != WICED_SUCCESS)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_service_get_the_update() wiced_ota2_service_wget_update(file) failed!\r\n");
        wiced_ota2_service_make_error_callback(session, OTA2_SERVICE_UPDATE_ERROR );
        session->download_failures++;
        goto _get_file_connected_fail;
    }

    /* ask the application if it wants to do the update automatically on next boot */
    if ( ((session->cb_function == NULL) && (session->auto_update != 0)) ||
         (wiced_ota2_service_make_callback(session, OTA2_SERVICE_PERFORM_UPDATE, 0) == WICED_SUCCESS) )
    {
        /* automatically update on next boot */
        wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "Set  WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT!\r\n");
        result = wiced_ota2_image_update_staged_status(WICED_OTA2_IMAGE_EXTRACT_ON_NEXT_BOOT);
    }
    else
    {
        /* mark staging area download as complete, but not extract on next reboot */
        wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "set WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE (do not automatically update on reboot)!\r\n");
        result = wiced_ota2_image_update_staged_status(WICED_OTA2_IMAGE_DOWNLOAD_COMPLETE);
    }

    /* we succeeded in getting the file here */
    *update_result = WICED_SUCCESS;

_get_file_connected_fail:
    ota2_service_disconnect(session);

_get_file_fail:
    return result;
}

wiced_result_t wiced_ota2_service_get_the_update(wiced_ota2_service_session_t* session)
{
    wiced_result_t  result = WICED_ERROR;
    wiced_result_t  update_result = WICED_ERROR;

    /* this service will perform the download of the update */
    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "wiced_ota2_service_get_the_update() start!\r\n");

    session->attempted_updates++;

    if (session->ota2_interface == WICED_ETHERNET_INTERFACE)
    {
        result = wiced_ota2_network_up(session->ota2_interface, session->ota2_ap_info) ;
        wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "wiced_ota2_network_up( ethernet ) result %d!\r\n", result);
        if (result != WICED_SUCCESS)
        {
            goto _check_update_done;
        }

        /* try to get the update file */
        result = wiced_ota2_service_get_the_update_file(session, &update_result);
        if (update_result == WICED_SUCCESS)
        {
            result = WICED_SUCCESS;
        }
        goto _check_update_done;
    }

    if (session->ota2_ap_list != NULL)
    {
        uint32_t i;
        uint32_t ap_failures = 0;
        uint32_t tcp_failures = 0;

        wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "wiced_ota2_service_get_the_update() Use the ota2 list count:%d!\r\n", session->ota2_ap_list_count );

        /* try connecting to any ap in the given list */
        for (i = 0; i < session->ota2_ap_list_count; i++)
        {
            /* assume we can't connect to an AP */
            result = WICED_ERROR;

            session->ota2_ap_info = &session->ota2_ap_list[i];

            /* check for a valid SSID */
            if (session->ota2_ap_info->details.SSID.length > 0)
            {
                result = wiced_ota2_service_network_switch_to_ota2_ap( session );
                if ( result == WICED_SUCCESS)
                {
                    /* informational */
                    wiced_ota2_service_make_callback(session, OTA2_SERVICE_AP_CONNECTED, 0);

                    /* try to get the update file */
                    result = wiced_ota2_service_get_the_update_file(session, &update_result);
                    if (update_result == WICED_SUCCESS)
                    {
                        result = WICED_SUCCESS;
                        break;
                    }
                    if (result != WICED_SUCCESS)
                    {
                        tcp_failures++;
                        continue;
                    }
                }
                else
                {
                    ap_failures++;
                }
            }
        }

        if (result != WICED_SUCCESS)
        {
            if (ap_failures > 0)
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_service_get_the_update() wiced_ota2_service_network_switch_to_ota2_ap() failed (list)!\r\n");
                wiced_ota2_service_make_error_callback(session, OTA2_SERVICE_AP_CONNECT_ERROR);
                session->ota2_ap_failures++;
            }
            else if (tcp_failures > 0)
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_service_get_the_update() ota2_service_connect() failed (list)!\r\n");
                wiced_ota2_service_make_error_callback(session, OTA2_SERVICE_SERVER_CONNECT_ERROR );
            }
            goto _check_update_done;
        }

    }
    else if (session->ota2_ap_info != NULL)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "wiced_ota2_service_get_the_update() Use the ota2 info!\r\n");

        /* connect to alternate AP, if provided */
        if (wiced_ota2_service_network_switch_to_ota2_ap( session ) != WICED_SUCCESS)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "wiced_ota2_service_get_the_update() wiced_ota2_service_network_switch_to_ota2_ap() failed!\r\n");
            wiced_ota2_service_make_error_callback(session, OTA2_SERVICE_AP_CONNECT_ERROR);
            session->ota2_ap_failures++;
            goto _check_update_done;
        }

        /* informational */
        wiced_ota2_service_make_callback(session, OTA2_SERVICE_AP_CONNECTED, 0);

        /* try to get the update file */
        result = wiced_ota2_service_get_the_update_file(session, &update_result);
        if (update_result == WICED_SUCCESS)
        {
            result = WICED_SUCCESS;
        }
    }

_check_update_done:

    if (wiced_ota2_service_network_switch_to_default_ap( session ) != WICED_SUCCESS)
    {
        session->default_ap_failures++;
        wiced_ota2_service_make_error_callback(session, OTA2_SERVICE_AP_CONNECT_ERROR);
    }

    if (update_result == WICED_SUCCESS)
    {
        session->successful_updates++;
    }
    else
    {
        session->failed_updates++;
    }

    if (session->ota2_ap_list != NULL)
    {
        /* If we had a list, we used this temporarily, clear it out now that we're done */
        session->ota2_ap_info = NULL;
    }

    /* this update attempt is done */
    wiced_ota2_service_make_callback(session, OTA2_SERVICE_UPDATE_ENDED, update_result );

    return update_result;
}
/******************************************************
 *               Internal Function Definitions
 ******************************************************/

void wiced_ota2_worker_thread(uint32_t arg)
{
    wiced_result_t                  result;
    wiced_utc_time_ms_t             curr_utc_time;

    wiced_ota2_service_session_t*   session;

    wiced_assert("wiced_OTA2_worker_thread() ARG == NULL!", (arg != 0));
    session = (wiced_ota2_service_session_t*)arg;

    /* init our mutex */
    result = wiced_rtos_init_mutex( &session->ota2_mutex );
    wiced_assert("wiced_rtos_init_mutex() failed!", (result == WICED_SUCCESS));

    /* init our signal flags */
    result = wiced_rtos_init_event_flags(&session->ota2_worker_flags);
    wiced_assert("wiced_rtos_init_event_flags(&session->ota2_worker_flags) failed!", (result == WICED_SUCCESS));

    /* let the app know we started the service - informational */
    wiced_ota2_service_make_callback(session, OTA2_SERVICE_STARTED, 0);

    /* send a command to start the initial timer */
    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "wiced_ota2_service_start() signal initial timer\r\n");
    wiced_rtos_set_event_flags(&session->ota2_worker_flags, OTA2_EVENT_WORKER_START_INITIAL_TIMER);

    while(1)
    {

        wiced_result_t                  result = WICED_SUCCESS;
        wiced_result_t                  cb_result = WICED_SUCCESS;
        uint32_t                        events;

        events = 0;
        result = wiced_rtos_wait_for_event_flags(&session->ota2_worker_flags, OTA2_EVENT_WORKER_THREAD_EVENTS, &events,
                                                 WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_OTA2_WORKER_FLAGS_TIMEOUT);

        if (result != WICED_SUCCESS)
        {
            continue;
        }

        if (events & OTA2_EVENT_WORKER_THREAD_SHUTDOWN_NOW)
        {
            ota2_service_stop_timer(session);
            ota2_service_stop_retry_timer(session);
            break;
        }

        wiced_time_get_utc_time_ms( &curr_utc_time );

        if (events & OTA2_EVENT_WORKER_START_INITIAL_TIMER)
        {
            ota2_service_stop_timer(session);

            if (session->initial_check_interval_seconds > 0)
            {
                uint32_t next_ms = session->initial_check_interval_seconds * MILLISECONDS_PER_SECOND;

                /* start timer for time from now */
                session->session_next_time = curr_utc_time + next_ms;
                ota2_service_start_timer(session, next_ms);
                ota2_service_set_state(session, OTA2_SERVICE_STATE_WAITING_FOR_TIMER);
            }
            else
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2: DO NOT START_INITIAL_TIMER initial interval: %ld sec!\r\n", session->initial_check_interval_seconds);
                session->session_next_time = 0;
                ota2_service_set_state(session, OTA2_SERVICE_STATE_NONE);

                /* if no initial timer, start the regular timer */
                events |= OTA2_EVENT_WORKER_START_NEXT_TIMER;
            }
        }

        if (events & OTA2_EVENT_WORKER_START_NEXT_TIMER)
        {
            ota2_service_stop_timer(session);

            if (session->check_interval_seconds > 0)
            {
                uint32_t    next_ms = session->check_interval_seconds * MILLISECONDS_PER_SECOND;

                /* determine next check time, simple clock-step from next to next */
                while (curr_utc_time >= session->session_next_time)
                {
                    session->session_next_time += next_ms;
                }

                /* determine the offset from NOW */
                next_ms = session->session_next_time - curr_utc_time;

                ota2_service_start_timer(session, next_ms);
                ota2_service_set_state(session, OTA2_SERVICE_STATE_WAITING_FOR_TIMER);
            }
            else
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2: DO NOT START_NEXT_TIMER (interval:%ld sec)\r\n", session->check_interval_seconds);
                session->session_next_time = 0;                                         /* for info */
                ota2_service_set_state(session, OTA2_SERVICE_STATE_NONE);
            }
        }

        if (events & OTA2_EVENT_WORKER_START_RETRY_TIMER)
        {
            uint32_t    retry_ms = session->retry_check_interval_seconds * MILLISECONDS_PER_SECOND;

            ota2_service_stop_retry_timer(session);

            if (session->retry_check_interval_seconds == 0)
            {
                wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2: START_RETRY_TIMER retry is 0, don't retry!\r\n");
                continue;
            }

            /* determine info for next time from current time */
            session->session_retry_time = curr_utc_time + retry_ms;

            ota2_service_start_retry_timer(session, retry_ms);
            ota2_service_set_state(session, OTA2_SERVICE_STATE_WAITING_FOR_TIMER);
        }

        if (events & OTA2_EVENT_WORKER_CHECK_FOR_UPDATES)
        {
            /* It is time to check for an update - ask the application, and continue around the loop.
             * Do not try to get the actual update if the Application says to - just set the OTA2_EVENT_WORKER_START_DOWNLOAD flag.
             * This is important because we also want to re-start the check timer so we can stay
             * as accurate as possible.
             */

            /* ota2_service_stop_timer(session); Do not stop the timer - it will already have the next interval time set */
            ota2_service_stop_retry_timer(session);

            /* save the time we started the last check */
            wiced_time_get_utc_time_ms( &session->session_last_time );

            ota2_service_set_state(session, OTA2_SERVICE_STATE_TIME_TO_CHECK_FOR_UPDATES);

            /* make callback to see what application wants to do
             * if application replied with success, this service will try to get the update
             */
            cb_result = wiced_ota2_service_make_callback(session, OTA2_SERVICE_CHECK_FOR_UPDATE, 0);
            if (cb_result == WICED_SUCCESS)
            {
                /* wait for a bit so that the application can bring down it's network */
                ota2_service_start_download_now_timer(session, WAIT_FOR_APP_TO_SHUTDOWN_NETWORK_MS);

                /* determine info for next time from current time */
                session->session_download_now_time = curr_utc_time + WAIT_FOR_APP_TO_SHUTDOWN_NETWORK_MS;
            }
            else
            {
                /* if we have a check timer, set the state appropriately */
                if (session->check_interval_seconds != 0)
                {
                    ota2_service_set_state(session, OTA2_SERVICE_STATE_WAITING_FOR_TIMER);
                }
                else
                {
                    ota2_service_set_state(session, OTA2_SERVICE_STATE_NONE);
                }

                /* Let Application know this update attempt is over */
                wiced_ota2_service_make_callback(session, OTA2_SERVICE_UPDATE_ENDED, WICED_ERROR );
            }
        } /* CHECK_FOR_UPDATES */

        if (events & OTA2_EVENT_WORKER_START_DOWNLOAD)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2: OTA2_EVENT_WORKER_START_DOWNLOAD!\r\n");
            /* ota2_service_stop_timer(session); Do not stop the timer - it will already have the next interval time set */

            ota2_service_stop_retry_timer(session);
            ota2_service_stop_download_now_timer(session);

            /* Check if already started (may have gotten a second event from the application) */
            if(ota2_service_get_state(session) != OTA2_SERVICE_STATE_STARTING_DOWNLOAD)
            {
                ota2_service_set_state(session, OTA2_SERVICE_STATE_STARTING_DOWNLOAD);


                /* save the time we started the last check */
                wiced_time_get_utc_time_ms( &session->session_last_download_start );

                if (wiced_ota2_service_get_the_update(session) == WICED_SUCCESS)
                {
                    wiced_log_msg( WLF_OTA2, WICED_LOG_NOTICE, "OTA2: wiced_ota2_service_get_the_update() SUCCEEDED!\r\n");
                    ota2_service_set_state(session, OTA2_SERVICE_STATE_DOWNLOAD_DONE);
                    if (session->check_interval_seconds != 0)
                    {
                        wiced_rtos_set_event_flags(&session->ota2_worker_flags, OTA2_EVENT_WORKER_START_NEXT_TIMER);
                    }
                    else
                    {
                        ota2_service_set_state(session, OTA2_SERVICE_STATE_NONE);
                    }
                }
                else
                {
                    /* update failed here */
                    wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "OTA2: wiced_ota2_service_get_the_update() FAILED check if timers started!\r\n");
                    ota2_service_set_state(session, OTA2_SERVICE_STATE_DOWNLOAD_DONE);
                    if ((session->retry_check_interval_seconds != 0) && (session->retry_count++ < session->max_retries))
                    {
                        if (wiced_rtos_is_timer_running(&session->retry_timer) != WICED_SUCCESS )
                        {
                            wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "OTA2: wiced_ota2_service_get_the_update() FAILED set flag START_RETRY_TIMER!\r\n");
                            wiced_rtos_set_event_flags(&session->ota2_worker_flags, OTA2_EVENT_WORKER_START_RETRY_TIMER);
                        }
                        else
                        {
                            wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "OTA2: wiced_ota2_service_get_the_update() FAILED set state WAITING_FOR_TIMER !\r\n");
                            ota2_service_set_state(session, OTA2_SERVICE_STATE_WAITING_FOR_TIMER);
                        }
                    }
                    else  if (session->check_interval_seconds != 0)
                    {
                        wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "OTA2: wiced_ota2_service_get_the_update() FAILED set flag START_NEXT_TIMER!\r\n");
                        wiced_rtos_set_event_flags(&session->ota2_worker_flags, OTA2_EVENT_WORKER_START_NEXT_TIMER);
                    }
                    else
                    {
                        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "OTA2: wiced_ota2_service_get_the_update() FAILED set state NONE !\r\n");
                        ota2_service_set_state(session, OTA2_SERVICE_STATE_NONE);
                    }
                }

                wiced_time_get_utc_time_ms( &session->session_last_download_stop );

            } /* already started ? */
        } /* start download */

    } /* while */

    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2: Worker thread shutting down!\r\n");

    /* let the app know the service has stopped */
    wiced_ota2_service_make_callback(session, OTA2_SERVICE_STOPPED, 0);

    /* don't need a timer anymore */
    ota2_service_stop_timer(session);

    /* destroy signal flags */
    wiced_rtos_deinit_event_flags(&session->ota2_worker_flags);
    wiced_rtos_deinit_mutex( &session->ota2_mutex );

    WICED_END_OF_CURRENT_THREAD();
}

/******************************************************
 *               External Function Definitions
 ******************************************************/

/**
 * Initialize a timed backgound service to check for updates
 *
 * @param[in]  params - initialization parameter struct pointer
 * @param[in]  opaque - pointer returned to application in callback
 *
 * @return - session pointer
 *           NULL indicates error
 */
void*  wiced_ota2_service_init(wiced_ota2_backround_service_params_t *params, void* opaque)
{
    wiced_ota2_service_session_t*    new_session;

    if (params == NULL)
    {
        WPRINT_LIB_ERROR(("OTA2: wiced_ota2_service_init() No params!\r\n"));
        return NULL;
    }

#ifndef WICED_USE_ETHERNET_INTERFACE
    if (params->ota2_interface == WICED_ETHERNET_INTERFACE)
    {
        WPRINT_LIB_ERROR(("OTA2: wiced_ota2_service_init() No ETHERNET support on this platform!\r\n"));
        return NULL;
    }
#endif

    /* if there is already a session created, don't allow a second one */
    if (g_only_one_session_at_a_time != NULL)
    {
        WPRINT_LIB_ERROR(("OTA2: wiced_ota2_service_init() Session already initialized !\r\n"));
        return NULL;
    }

    /* validate settings */
    /* If there are no time values, let's bail - nothing for us to do ! */
    if (params->initial_check_interval == 0)
    {
        WPRINT_LIB_ERROR(("OTA2: initial_check_interval_seconds == 0 !\r\n"));
        return NULL;
    }

    if ((params->host_name == NULL) || (strlen(params->host_name) < 2))
    {
        WPRINT_LIB_ERROR(("OTA2: BAD host server: %p %s !\r\n", params->host_name, ((params->host_name == NULL) ? "<null>" : params->host_name)));
        return NULL;
    }

    if ((params->file_path == NULL) || (strlen(params->file_path) < 2))
    {
        WPRINT_LIB_ERROR(("OTA2: BAD file name: %p %s !\r\n", params->file_path, ((params->file_path == NULL) ? "<null>" : params->file_path) ));
        return NULL;
    }

    if (params->initial_check_interval > WICED_OTA2_MAX_INTERVAL_TIME)
    {
        WPRINT_LIB_ERROR(("Initial Check interval value is > %ld\r\n", WICED_OTA2_MAX_INTERVAL_TIME ));
        return NULL;
    }
    if (params->check_interval > WICED_OTA2_MAX_INTERVAL_TIME)
    {
        WPRINT_LIB_ERROR(("Check interval value is > %ld\r\n", WICED_OTA2_MAX_INTERVAL_TIME ));
        return NULL;
    }
    if (params->retry_check_interval > WICED_OTA2_MAX_INTERVAL_TIME)
    {
        WPRINT_LIB_ERROR(("Retry Check interval value is > %ld\r\n", WICED_OTA2_MAX_INTERVAL_TIME ));
        return NULL;
    }

    if ((params->retry_check_interval != 0) && (params->retry_check_interval < WICED_OTA2_MIN_RETRY_INTERVAL_TIME))
    {
        WPRINT_LIB_ERROR(("Minimum retry interval value is < %ld\r\n", WICED_OTA2_MIN_RETRY_INTERVAL_TIME ));
        return NULL;
    }

    new_session = (wiced_ota2_service_session_t*)calloc(1, sizeof(wiced_ota2_service_session_t));
    if( new_session == NULL)
    {
        WPRINT_LIB_ERROR(("OTA2: wiced_ota2_service_init() failed to calloc main service structure !\r\n"));
        return NULL;
    }

    new_session->cb_opaque = opaque;

    new_session->ota2_interface                 = params->ota2_interface;
    new_session->initial_check_interval_seconds = params->initial_check_interval;
    new_session->check_interval_seconds         = params->check_interval;
    new_session->retry_check_interval_seconds   = params->retry_check_interval;
    new_session->max_retries                    = params->max_retries;
    new_session->auto_update                    = params->auto_update;
    new_session->ota2_ap_info                   = params->ota2_ap_info;
    new_session->default_ap_info                = params->default_ap_info;
    new_session->ota2_ap_list_count             = params->ota2_ap_list_count;
    new_session->ota2_ap_list                   = params->ota2_ap_list;
    strlcpy(new_session->host_name, params->host_name, sizeof(new_session->host_name) - 1);
    strlcpy(new_session->file_path, params->file_path, sizeof(new_session->file_path) - 1);

    /* new session struct initialized */
    new_session->tag       = WICED_OTA2_SERVICE_TAG_VALID;

    wiced_time_get_utc_time_ms( &new_session->session_start_time );

    /* save the session pointer */
    g_only_one_session_at_a_time = new_session;

    /* mark as inited */
    new_session->ota2_state = OTA2_SERVICE_STATE_INITIALIZED;

    wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "OTA2: wiced_ota2_service_init() DONE %p state:%d!\r\n", new_session, new_session->ota2_state);

    return new_session;
}


/**
 * De-initialize the service
 *
 * @param[in]  session - value returned from wiced_ota2_init()
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG
 */
wiced_result_t  wiced_ota2_service_deinit(void* session_id)
{
    wiced_ota2_service_session_t*   session ;

    session = (wiced_ota2_service_session_t*)session_id;
    if (session == NULL)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "OTA2: wiced_ota2_service_deinit() Bad arg!\r\n");
        return WICED_BADARG;
    }
    if (session != g_only_one_session_at_a_time )
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "OTA2: wiced_ota2_service_deinit() Bad arg (session != global)!\r\n");
        return WICED_BADARG;
    }

    if (session->tag != WICED_OTA2_SERVICE_TAG_VALID)
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "OTA2: wiced_ota2_service_deinit() Already de-inited!\r\n");
        return WICED_SUCCESS;
    }

    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2: wiced_ota2_service_deinit() calling stop!\r\n");
    wiced_ota2_service_stop(session_id);

    session->tag = WICED_OTA2_SERVICE_TAG_INVALID;

    wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "OTA2: wiced_ota2_service_deinit() Done!\r\n");

    free(session);
    g_only_one_session_at_a_time = NULL;

    return WICED_SUCCESS;
}

/**
 * Start the service
 *
 * @param[in]  session - value returned from wiced_ota2_init()
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG
 */
wiced_result_t  wiced_ota2_service_start(void* session_id)
{
    wiced_result_t                  result;
    wiced_ota2_service_session_t*   session ;

    session = (wiced_ota2_service_session_t*)session_id;
    if ((session == NULL) || (session->tag != WICED_OTA2_SERVICE_TAG_VALID) || (session != g_only_one_session_at_a_time ))
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "OTA2: wiced_ota2_service_start() Bad arg (session %p != global %p)!\r\n", session, g_only_one_session_at_a_time);
        return WICED_BADARG;
    }

    wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "OTA2: wiced_ota2_service_start() session:%p session->worker_thread:%p (must be 0x00)\r\n", session, session->ota2_worker_thread_ptr);

    if (session->ota2_worker_thread_ptr ==  NULL)
    {
        /* Start worker thread */
        result = wiced_rtos_create_thread( &session->ota2_worker_thread, WICED_OTA2_WORKER_THREAD_PRIORITY, "OTA2 worker",
                                           wiced_ota2_worker_thread, WICED_OTA2_WORKER_STACK_SIZE, session);
        if (result != WICED_SUCCESS)
        {
            wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "OTA2: wiced_rtos_create_thread(worker) failed:%d\r\n", result);
            goto _start_error_exit;
        }
        else
        {
            session->ota2_worker_thread_ptr = &session->ota2_worker_thread;
        }
    }

    wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "OTA2: wiced_ota2_service_start() Done!\r\n");
    return WICED_SUCCESS;


_start_error_exit:
    wiced_log_msg( WLF_OTA2, WICED_LOG_WARNING, "OTA2: wiced_ota2_service_start() FAILED!\r\n");
    wiced_ota2_service_stop(session);
    return WICED_ERROR;
}
/**
 *  Let OTA2 know that the network is down so it can continue
 *  with the download - note, if this is not called, there is a timeout
 *  of 5 seconds after OTA2 sends callback OTA2_SERVICE_CHECK_FOR_UPDATE to App
 *
 * @param[in]  session_id - value returned from wiced_ota2_service_init()
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG
 */
wiced_result_t wiced_ota2_service_app_network_is_down(void* session_id)
{
    ota2_state_t                    state;
    wiced_ota2_service_session_t*   session = (wiced_ota2_service_session_t*)session_id;

    if ((session == NULL) || (session->tag != WICED_OTA2_SERVICE_TAG_VALID) || (session != g_only_one_session_at_a_time ))
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "OTA2: wiced_ota2_service_stop() Bad arg (session %p != global %p)!\r\n", session, g_only_one_session_at_a_time);
        return WICED_BADARG;
    }

    /* only signal start download if we are waiting for the app to bring down it's network connection */
    state = ota2_service_get_state(session);
    switch (state)
    {
        case OTA2_SERVICE_STATE_TIME_TO_CHECK_FOR_UPDATES:
            wiced_rtos_set_event_flags(&session->ota2_worker_flags, OTA2_EVENT_WORKER_START_DOWNLOAD);
            break;
        default:
        case OTA2_SERVICE_STATE_NOT_INITIALIZED:
        case OTA2_SERVICE_STATE_INITIALIZED:
        case OTA2_SERVICE_STATE_NONE:
        case OTA2_SERVICE_STATE_WAITING_FOR_TIMER:
        case OTA2_SERVICE_STATE_STARTING_DOWNLOAD:
        case OTA2_SERVICE_STATE_DOWNLOAD_DONE:
            break;
    }

    return WICED_SUCCESS;
}
/**
 * Stop the service
 *
 * @param[in]  session_id - value returned from wiced_ota2_init()
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG

 */
wiced_result_t wiced_ota2_service_stop(void* session_id)
{
    wiced_ota2_service_session_t* session = (wiced_ota2_service_session_t*)session_id;

    if ((session == NULL) || (session->tag != WICED_OTA2_SERVICE_TAG_VALID) || (session != g_only_one_session_at_a_time ))
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "OTA2: wiced_ota2_service_stop() Bad arg (session %p != global %p)!\r\n", session, g_only_one_session_at_a_time);
        return WICED_BADARG;
    }

    if (  session->ota2_worker_thread_ptr != NULL)
    {
        /* stop the service thread */
        wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "wiced_ota2_service_stop() stopping worker\r\n");
        wiced_rtos_set_event_flags(&session->ota2_worker_flags, OTA2_EVENT_WORKER_THREAD_SHUTDOWN_NOW);

        wiced_rtos_thread_force_awake( &session->ota2_worker_thread );
        wiced_rtos_thread_join( &session->ota2_worker_thread);
        wiced_rtos_delete_thread( &session->ota2_worker_thread);
        wiced_log_msg( WLF_OTA2, WICED_LOG_INFO, "wiced_ota2_service_stop() worker deleted :) \r\n");
    }
    session->ota2_worker_thread_ptr = NULL;

    wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "wiced_ota2_service_stop() DONE\r\n");

    return WICED_SUCCESS;
}

/**
 * Register or Un-register a callback function to handle the actual update check
 *
 * @param[in]  session_id  - value returned from wiced_ota2_init()
 * @param[in]  callback - callback function pointer (NULL to disable)
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG

 */
wiced_result_t  wiced_ota2_service_register_callback(void* session_id, ota2_service_callback update_callback)
{
    wiced_ota2_service_session_t* session = (wiced_ota2_service_session_t*)session_id;
    if ((session == NULL) || (session->tag != WICED_OTA2_SERVICE_TAG_VALID) || (session != g_only_one_session_at_a_time ))
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "OTA2: wiced_ota2_service_register_callback() Bad arg (session %p != global %p)!\r\n", session, g_only_one_session_at_a_time);
        return WICED_BADARG;
    }

    session->cb_function = update_callback;
    return WICED_SUCCESS;
}

/**
 * Force an update check now
 * NOTE: Does not affect the timed checks - this is separate
 * NOTE: Asynchronous, non-blocking
 * NOTE: Will start the background service if not running
 *
 * @param[in]  session_id - value returned from wiced_ota2_init()
 *
 * @return - WICED_SUCCESS - There was no current update in progress
 *                           Update check started
 *           WICED_ERROR   - No session_id or bad session_id or update in progress
 *           WICED_BADARG
 */
wiced_result_t  wiced_ota2_service_check_for_updates(void* session_id)
{
    wiced_result_t  result = WICED_SUCCESS;

    wiced_ota2_service_session_t* session = (wiced_ota2_service_session_t*)session_id;

    wiced_log_msg( WLF_OTA2, WICED_LOG_DEBUG0, "wiced_ota2_service_check_for_updates()\r\n");

    if ((session == NULL) || (session->tag != WICED_OTA2_SERVICE_TAG_VALID) || (session != g_only_one_session_at_a_time ))
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "OTA2: wiced_ota2_service_check_for_updates() Bad arg (session %p != global %p)!\r\n", session, g_only_one_session_at_a_time);
        return WICED_ERROR;
    }

    /* Make sure our thread is started */
    if (session->ota2_worker_thread_ptr == NULL)
    {
        result = wiced_ota2_service_start(session_id);
    }

    if (result == WICED_SUCCESS)
    {
        /* signal the worker thread to check NOW */
        wiced_rtos_set_event_flags(&session->ota2_worker_flags, OTA2_EVENT_WORKER_CHECK_FOR_UPDATES);
    }

    return result;
}

/**
 * Split a URI into host and file_path parts
 *
 * @param[in]  uri           - the URI of the file desired
 * @param[in]  host_buff     - pointer to where the host part of the URI will be stored
 * @param[in]  host_buff_len - length of host_buff
 * @param[in]  path_buff     - pointer to where the path part of the URI will be stored
 * @param[in]  path_buff_len - length of path_buff
 * @param[in]  port          - pointer to store the port number
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG
*/
wiced_result_t wiced_ota2_service_uri_split(const char* uri, char* host_buff, uint16_t host_buff_len, char* path_buff, uint16_t path_buff_len, uint16_t* port)
{
   const char *uri_start, *host_start, *host_end;
   const char *path_start;
   uint16_t host_len, path_len;

  if ((uri == NULL) || (host_buff == NULL) || (path_buff == NULL) || (port == NULL))
  {
      return WICED_ERROR;
  }

  *port = 0;

  /* drop http:// or htts://"  */
  uri_start = strstr(uri, "http");
  if (uri_start == NULL)
  {
      uri_start = uri;
  }
  if (strncasecmp(uri_start, "http://", 7) == 0)
  {
      uri_start += 7;
  }
  else if (strncasecmp(uri_start, "https://", 8) == 0)
  {
      uri_start += 8;
  }

  memset(host_buff, 0, host_buff_len);
  host_start = uri_start;
  host_len = strlen(host_start);
  host_end = strchr(host_start, ':');
  if (host_end != NULL)
  {
      *port = atoi(host_end + 1);
  }
  else
  {
      *port = 80;
      host_end = strchr(host_start, '/');
  }

  if (host_end != NULL)
  {
      host_len = host_end - host_start;
  }
  if( host_len > (host_buff_len - 1))
  {
      host_len = host_buff_len - 1;
  }
  memcpy(host_buff, host_start, host_len);

  memset(path_buff, 0, path_buff_len);
  path_start = strchr(host_start, '/');
  if( path_start != NULL)
  {
      path_len = strlen(path_start);
      if( path_len > (path_buff_len - 1))
      {
          path_len = path_buff_len - 1;
      }
      memcpy(path_buff, path_start, path_len);
  }

  return WICED_SUCCESS;
}


wiced_result_t wiced_ota2_service_status(void* session_id)
{
    wiced_utc_time_ms_t             utc_time_ms;
    wiced_ota2_service_session_t*   session;

    session = (wiced_ota2_service_session_t*)session_id;
    if ((session == NULL) || (session->tag != WICED_OTA2_SERVICE_TAG_VALID) || (session != g_only_one_session_at_a_time ))
    {
        wiced_log_msg( WLF_OTA2, WICED_LOG_ERR, "OTA2: wiced_ota2_service_status() Bad arg (session %p != global %p)!\r\n", session, g_only_one_session_at_a_time);
        return WICED_BADARG;
    }

    if (session->ota2_worker_thread_ptr == NULL)
    {
        return WICED_ERROR;
    }

    wiced_log_printf("OTA2 Service Info:\r\n");
    wiced_time_get_utc_time_ms( &utc_time_ms );
    wiced_log_printf("            current time: %s\r\n", wiced_ota2_service_convert_ms_to_time_string(utc_time_ms));
    wiced_log_printf("    session started time: %s\r\n", wiced_ota2_service_convert_ms_to_time_string(session->session_start_time));
    wiced_log_printf("  last update check time: %s\r\n", wiced_ota2_service_convert_ms_to_time_string(session->session_last_time));
    wiced_log_printf("     last download start: %s\r\n", wiced_ota2_service_convert_ms_to_time_string(session->session_last_download_start));
    wiced_log_printf("      last download stop: %s\r\n", wiced_ota2_service_convert_ms_to_time_string(session->session_last_download_stop));
    wiced_log_printf("  next update check time: %s\r\n", wiced_ota2_service_convert_ms_to_time_string(session->session_next_time));

    if (session->retry_timer_value != 0)
    {
        wiced_log_printf("   retry %d of %d time: %s\r\n", session->retry_count, session->max_retries, wiced_ota2_service_convert_ms_to_time_string(session->session_retry_time));
    }
    if (wiced_rtos_is_timer_running(&session->download_now_timer) == WICED_SUCCESS )
    {
        wiced_log_printf("  download now timer: %s\r\n", wiced_ota2_service_convert_ms_to_time_string(session->session_download_now_time));
    }

    wiced_log_printf("                   state: %d\r\n", session->ota2_state);
    wiced_log_printf("          ota2_interface: (%d) %s\r\n", session->ota2_interface,
                                                         (session->ota2_interface == WICED_ETHERNET_INTERFACE) ? "ETHERNET" :
                                                         (session->ota2_interface == WICED_STA_INTERFACE) ? "STA" :
                                                         (session->ota2_interface == WICED_AP_INTERFACE) ? "AP" :
                                                         (session->ota2_interface == WICED_P2P_INTERFACE) ? "P2P" : "OTHER" );
    wiced_log_printf("            ota2_ap_info: %p\r\n", session->ota2_ap_info);
    wiced_log_printf("         default_ap_info: %p\r\n", session->default_ap_info);
    wiced_log_printf("                     url: %s\r\n", session->host_name);
    wiced_log_printf("               file_name: %s\r\n", session->file_path);
    wiced_log_printf("  initial_check_interval_seconds: %ld sec\r\n", session->initial_check_interval_seconds);
    wiced_log_printf("          check_interval_seconds: %ld sec\r\n", session->check_interval_seconds);
    wiced_log_printf("    retry_check_interval_seconds: %ld sec\r\n", session->retry_check_interval_seconds);
    wiced_log_printf("             auto_update: %s\r\n", (session->auto_update != 0) ? "true" : "false");
    wiced_log_printf("         last ota2 error: %d\r\n", session->last_error_status);

    wiced_log_printf("\r\n       attempted updates: %d\r\n", session->attempted_updates);
    wiced_log_printf("      successful updates: %d\r\n", session->successful_updates);
    wiced_log_printf("          failed updates: %d\r\n", session->failed_updates);
    wiced_log_printf("            ota2 AP failures: %d\r\n", session->ota2_ap_failures);
    wiced_log_printf("                TCP failures: %d\r\n", session->tcp_failures);
    wiced_log_printf("                    TCP timeouts: %d\r\n", session->tcp_timeouts);
    wiced_log_printf("           download failures: %d\r\n", session->download_failures);
    wiced_log_printf("     default AP failures: %d\r\n", session->default_ap_failures);

    return WICED_SUCCESS;
}
