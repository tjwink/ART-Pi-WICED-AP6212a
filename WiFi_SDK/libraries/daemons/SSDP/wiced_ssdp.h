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
 *
 *  WICED SSDP Support
 *
 *  1) Call wiced_ssdp_init():
 *      Join Multicast group 239.255.255.250:1900
 *      Send Notify "ssdp:alive" at regular intervals
 *      Respond to M-Search requests
 *
 *  2) Call wiced_ssdp_send_msearch() - BLOCKING CALL
 *      Sends M-Search (3 sent in 1 second, and wait for responses)
 *      Gathers Responses (up to size of response buffer or designated timeout)
 *          - only saves unique responses
 *
 *  3) Call wiced_ssdp_deinit()
 *      Send Notify "ssdp:byebye"
 *
 *Internals:
 *  We create 1 thread to deal with Multicast packets
 *      - Sending M-Search packets
 *      - Replying to M-Search packets
 *      - Sending Notify packets
 *      - Receiving NOTIFY packets (We do not store info)
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "http_stream.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define SSDP_MULTICAST_PORT                 (1900)
#define SSDP_MULTICAST_IPV4_ADDRESS         MAKE_IPV4_ADDRESS(239, 255, 255, 250)

/* sufficient to store IP string + NULL -- "127.000.000.001\0" */
#define SSDP_INET_ADDRSTRLEN     (15 + 1)

/* Max number of headers from the query */
#define SSDP_HTTP_HEADERS_MAX           (16)

#define SSDP_DEFAULT_MSEARCH_SCAN_TIME  (3)             /* seconds ( 1 < time < 5) */
#define SSDP_DEFAULT_SEARCH_TARGET      "ssdp:all"      /* all SSDP devices */
#define SSDP_DEFAULT_USER_AGENT         "WICED/3.3.x"

#define SSDP_DEFAULT_SERVER_TYPE        "wiced_ssdp/1.0"
#define SSDP_DEFAULT_USN_TYPE           "ssdp:rootdevice"
#define SSDP_DEFAULT_PAGE_PATH_NAME     "index.html"

/* For M-Search Response Field Sizes */
#define WICED_SSDP_CACHE_CONTROL_MAX     (16)   /* ex: "max-age=172800"                                                 */
#define WICED_SSDP_LOCATION_MAX         (128)   /* ex: "http://192.168.10.51:8080/udap/api/data?target=smartText.xml"   */
#define WICED_SSDP_SEARCH_TARGET_MAX     (64)   /* ex: "urn:schemas-udap:service:smartText:1"                           */
#define WICED_SSDP_USN_MAX              (128)   /* ex: "uuid:33068e81-3306-0633-619b-9b61818e0633::urn:schemas-udap:service:smartText:1" */

#define WICED_SERVER_TYPE_MAX            (64)   /* Server type description          */
#define WICED_SSDP_NOTIFY_TYPE_MAX       (64)   /* SSDP notification type           */
#define WICED_SSDP_NOTIFY_SUBTYPE_MAX    (16)   /* SSDP notification Sub type ex:  "ssdp:alive" or "ssdp:byebye"    */

#define WICED_SSDP_MINIMUM_NOTIFY_TIME    (60)  /*   60 seconds (1 minute)      */
#define WICED_SSDP_NOTIFY_DEFAULT_TIME  (1800)  /* 1800 seconds (30 minutes)    */

/******************************************************
 *                   Enumerations
 ******************************************************/

/* log levels for the SSDP Library */
typedef enum
{
    SSDP_LOG_OFF        = 0,
    SSDP_LOG_LOW,
    SSDP_LOG_INFO,
    SSDP_LOG_DEBUG,
} WICED_SSDP_LOG_LEVEL_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
/**
 * Structure for callback when we receive a NOTIFY packet from a peer
 *
 * NOTE: cache_control and location are empty for "ssdp:byebye"
 */
typedef struct wiced_ssdp_notify_info_s
{
    wiced_ip_address_t  ip;                                                     /* IP of responder                  */
    char                ip_string[SSDP_INET_ADDRSTRLEN];
    char                cache_control[WICED_SSDP_CACHE_CONTROL_MAX + 1];        /* time host available in seconds   */
    char                location[WICED_SSDP_LOCATION_MAX + 1];                  /* URL of remote to get description */
    char                server[WICED_SERVER_TYPE_MAX + 1];                      /* Server type description          */
    char                nt[WICED_SSDP_NOTIFY_TYPE_MAX + 1];                     /* SSDP notification type           */
    char                nts[WICED_SSDP_NOTIFY_SUBTYPE_MAX + 1];                 /* SSDP notification Sub type       */
    char                usn[WICED_SSDP_USN_MAX + 1];                            /* Unique Service Name              */
} wiced_ssdp_notify_info_t;

/**
 * Structure for M-SEARCH responses we recive from peers
 *  One device/service/control point per response
 *      If device offers multiple services / control points, you will get multiple responses
 */
typedef struct wiced_ssdp_msearch_response_s
{
    wiced_ip_address_t  ip;                                                     /* IP of responder                  */
    char                ip_string[SSDP_INET_ADDRSTRLEN];
    char                cache_control[WICED_SSDP_CACHE_CONTROL_MAX + 1];        /* time host available in seconds   */
    char                location[WICED_SSDP_LOCATION_MAX + 1];                  /* URL of remote to get description */
    char                st[WICED_SSDP_SEARCH_TARGET_MAX + 1];                   /* SSDP search target               */
    char                usn[WICED_SSDP_USN_MAX + 1];                            /* Unique Service Name              */
} wiced_ssdp_msearch_response_t;

/**
 * Structure for M-SEARCH parameters
 */
typedef struct wiced_ssdp_msearch_params_s
{
    uint16_t                        msearch_scan_time;              /* amount of time to wait for responses             */
    char*                           msearch_search_target;          /* service to search for (eg: "ssdp:all")           */
    char*                           msearch_user_agent;             /* user_agent eq: "WICED/1.0" added to "UDAP/2.0"   */

    uint16_t                        response_array_size;            /* [in] number of response structures in the array  */
    uint16_t                        num_responses;                  /* [out] number of responses received               */
    wiced_ssdp_msearch_response_t*  responses;                      /* pointer to the response array to be filled in    */
} wiced_ssdp_msearch_params_t;

/**
 *  Structure to hold startup parameters for the SSDP server
 */
typedef struct wiced_ssdp_params_s
{

    /* M-Search REPLY / Notify parameters */
    uint16_t        server_port;                /* Server port for your web server                                      */
    char*           serve_page_path;            /* page path added to IP when responding to M-SEARCH request            */
                                                /* added to end of IP string "http://xxx.xxx.xxx.xxx/<serve_page_path>" */
    char*           notify_server_type;         /* default "quick_ssdp/1.0"                                             */
    char*           notify_usn_type;            /* default "ssdp:rootdevice"                                            */
    uint16_t        notify_time;                /* seconds between sending NOTIFY (60 <= time <= 1800 seconds)          */

    char*           uuid;                       /* uuid value ie: "7272e15a-2505-11e5-b345-feff819cdc9f"                */

    WICED_SSDP_LOG_LEVEL_T  log_level;
} wiced_ssdp_params_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
/*****************************************************************************/
/**
 *
 *  @defgroup ssdp          SSDP
 *  @ingroup  ipcoms
 *
 * Communication functions for SSDP (Simple Service Discovery Protocol)
 *
 *  @{
 */
/*****************************************************************************/

/** Callback for notifications
 *
 * Prototype for the user-defined function. Function is called when we recive a NOTIFY packet.
 *
 * @param nofity_info    : [in] ptr to info about the NOTIFY packet.
 * @param data           : [in] opaque app data
 *
 *      NOTES: The event_info structure is stored on the stack!
 *              It will not be around after the callback returns!
 *              Make a copy of info you want to keep the info!
 *
 */
typedef void (*wiced_ssdp_notify_callback_t)(wiced_ssdp_notify_info_t* nofity_info, void* data);

/** Start the SSDP daemon
 *  You must either
 *      set the start flags in the params (start_server, start_multicast)
 *  or
 *      call one of the start functions below
 *
 *
 * @param   params    : pointer to the parameter structure
 * @param   ssdp_info : pointer to store instance to use in subsequent calls
 * @param   interface : interface to initialize SSDP on.

 * @return  WICED_SUCCESS
 *          WICED_ERROR
 *          WICED_BADARG
 */
extern wiced_result_t wiced_ssdp_init( wiced_ssdp_params_t *ssdp_params, void** ssdp_info, wiced_interface_t interface );

/** Shut down the SSDP daemon
 *  This stops both the server and multicast messages
 *
 * @param   ssdp_info   : pointer info structure returned from wiced_ssdp_server_start()

 * @return  WICED_SUCCESS
 *          WICED_ERROR
 *          WICED_BADARG
 */
extern wiced_result_t wiced_ssdp_deinit( void* ssdp_info );

/** Register SSDP Notify callback
 *  Register a callback so the application can be notified when we receive a NOTIFY packet
 *
 *  NOTE: to disable the callback, use wiced_ssdp_notify_register_callback(ssdp_info, NULL, NULL);
 *
 * @param   ssdp_info   : pointer info structure returned from wiced_ssdp_init()
 * @param   callback    : callback to register (call with NULL to de-register)
 * @param   data        : returned in callback (opaque to ssdp support)
 *
 * @return  WICED_SUCCESS
 *          WICED_ERROR
 *          WICED_BADARG
 */
extern wiced_result_t wiced_ssdp_notify_register_callback( void *ssdp_info, wiced_ssdp_notify_callback_t callback, void *data );


/** Send an M-Search message and wait for responses
 *
 *  NOTE: this is a blocking call
 *
 * @param   ssdp_info           : pointer info structure returned from wiced_ssdp_server_start()
 * @param   params              : pointer to m-search send parameters
 *
 * @return  WICED_SUCCESS
 *          WICED_ERROR
 *          WICED_BADARG
 */
wiced_result_t wiced_ssdp_send_msearch_wait_for_results(void *ssdp_info, wiced_ssdp_msearch_params_t *params);

/** Set the log level for the SSDP library
 *
 * @param   ssdp_info   : pointer info structure returned from wiced_ssdp_server_start()
 * @param   log_level   : new log level
 *
 * @return  WICED_SUCCESS
 *          WICED_ERROR
 *          WICED_BADARG
 */
extern wiced_result_t wiced_ssdp_set_log_level( void* ssdp_info, WICED_SSDP_LOG_LEVEL_T  log_level );

/** Send debug information to the console
 *
 * @param   ssdp_info   : pointer info structure returned from wiced_ssdp_server_start()

 * @return  WICED_SUCCESS
 *          WICED_ERROR
 *          WICED_BADARG
 */
extern wiced_result_t wiced_ssdp_dump_debug_info( void* ssdp_info );

#ifdef __cplusplus
} /* extern "C" */
#endif
