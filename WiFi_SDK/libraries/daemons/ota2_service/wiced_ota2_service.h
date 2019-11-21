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
 * NOTE: Network must be up and connected to an AP before starting the service
 * NOTE: The platform must have an RTC for the interval update checking
 *
 * Before calling this API
 * - Network must be up and connected to an AP with
 *      access to the update server
 *
 * The OTA2 Service will periodically check and perform OTA updates
 *
 *  if no callback is registered
 *      OTA2 Service will perform default actions:
 *          - check for updates at check_interval
 *          - download updates when available
 *          - extract & perform update on next power cycle
 *  else
 *      Inform the application via callback
 *          If Application returns WICED_SUCCESS
 *              OTA Service will perform default action
 *          If application returns WICED_ERROR
 *              Application will perform action
 */
#pragma once


#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "wiced.h"
#include "wiced_log.h"
#include "platform_dct.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define WICED_OTA2_HTTP_QUERY_SIZE          1024
#define WICED_OTA2_HOST_NAME                 256
#define WICED_OTA2_FILE_PATH                 256

#define WICED_OTA2_HTTP_PORT                  80

#ifndef MILLISECONDS_PER_SECOND
#define MILLISECONDS_PER_SECOND             1000
#endif

/* no interval should be more than 365 days! */
#define WICED_OTA2_MAX_INTERVAL_TIME        (60 * 60 * 24 * 365)

/* no retry interval should be less than 1 minute */
#define WICED_OTA2_MIN_RETRY_INTERVAL_TIME  (60)
/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    OTA2_SERVICE_NOMINAL = 0,       /* Used internally, does not produce a callback
                                     */

    OTA2_SERVICE_STARTED,           /* Background timer thread has started
                                     * return - None - informational
                                     */

    OTA2_SERVICE_AP_CONNECT_ERROR,  /* Background timer thread failed to connect to supplied OTA2 AP
                                     * return - None - informational
                                     */

    OTA2_SERVICE_SERVER_CONNECT_ERROR,  /* Background timer thread failed to TCP connect to update server
                                         * return - None - informational
                                         */

    OTA2_SERVICE_AP_CONNECTED,      /* Background timer thread connected to OTA2 AP
                                     * return - None - informational
                                     */

    OTA2_SERVICE_SERVER_CONNECTED,  /* Background timer thread TCP connected to update server
                                     * return - None - informational
                                     */

    OTA2_SERVICE_CHECK_FOR_UPDATE,  /* Time to check for updates.
                                     * return - WICED_SUCCESS = Service will check for update availability
                                     *        - WICED_ERROR   = Application will check for update availability
                                     */

    OTA2_SERVICE_UPDATE_AVAILABLE,  /* Service has contacted server, update is available
                                     * value   - pointer to the wiced_ota2_image_header_t structure
                                     *           from the file on the server.
                                     *
                                     * return - WICED_SUCCESS = Application indicating that it wants the
                                     *                           OTA Service to perform the download
                                     *        - WICED_ERROR   = Application indicating that it will perform
                                     *                           the download, the OTA Service will do nothing.
                                     *                           If Application is going to ignore the update,
                                     *                           return WICED_ERROR
                                     */

    OTA2_SERVICE_DOWNLOAD_STATUS,   /* Download status
                                     *   NOTE: This will only occur when Service is performing download
                                     * value  -  % complete (0-100)
                                     * return - WICED_SUCCESS = Service will continue download
                                     *        - WICED_ERROR   = Service will STOP download and service will
                                     *                          issue OTA2_SERVICE_UPDATE_ERROR
                                     */

    OTA2_SERVICE_PERFORM_UPDATE,    /* Download is complete
                                     * return - WICED_SUCCESS = Service will inform Bootloader to extract
                                     *                          and update on next power cycle
                                     *        - WICED_ERROR   = Service will inform Bootloader that download
                                     *                          is complete - Bootloader will NOT extract
                                     */

    OTA2_SERVICE_UPDATE_ERROR,      /* There was an error in transmission
                                     * This will only occur if Error occurs when the Service
                                     *   is performing the data transfer.
                                     * return - WICED_SUCCESS = Service will retry using retry_timer
                                     *        - WICED_ERROR   = Service will retry on next check_interval
                                     *            Application can call wiced_ota2_service_check_for_updates()
                                     *            to run another check earlier
                                     */

    OTA2_SERVICE_UPDATE_ENDED,     /* All update actions for this check are complete.
                                    * value - WICED_SUCCESS - Service completed download successfully
                                    *       - WICED_ERROR   - Service failed OR Application stopped service from completing
                                    *                         download - in this case, the application must decide if
                                    *                         the download was successful
                                    * This callback is to allow the application to take any actions when
                                    *      the service is done checking / downloading an update
                                    *      It is called for all outcomes: successful or error.
                                    * return - None  - informational
                                    */

    OTA2_SERVICE_STOPPED,          /* Background service has stopped
                                    * return - None - informational
                                    */
} wiced_ota2_service_status_t;

/******************************************************
 *               Callback Function Definition
 ******************************************************/

/**
 *  Application callback for OTA service
 *  NOTE: This callback is called rather than the
 *          default checking for an update. Return value tells
 *          service how to handle the notification, or if the
 *          Application will handle the downloads - see .
 *
 * @param[in]  session - value returned from wiced_ota2_service_init()
 * @param[in]  status  - current status of service (wiced_ota2_service_status_t)
 * @param[in]  value   - value associated with status
 * @param[in]  opaque  - user supplied opaque pointer
 *
 * @return - WICED_SUCCESS  - Service will perform default action
 *           WICED_ERROR    - Application will perform action
 */
typedef wiced_result_t (*ota2_service_callback)(void* session_id,
                                               wiced_ota2_service_status_t status, uint32_t value,
                                               void* opaque );

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    char*           host_name;              /* host to get updates from                                     */
    char*           file_path;              /* filename to get                                              */
    uint16_t        port;                   /* port on host machine (default: HTTP_PORT = 80)               */

    uint32_t        initial_check_interval; /* seconds before first update check                            */
    uint32_t        check_interval;         /* seconds between checks                                       */
    uint32_t        retry_check_interval;   /* seconds between re-try if initial contact to
                                             * server for update info fails
                                             * 0 = wait until next check_interval                           */
    uint16_t        max_retries;            /* maximum retries per update event                             */

    uint8_t         auto_update;            /* Callback return value over-rides this parameter
                                             * Auto-update behavior if no callback registered.
                                             *   1 = Service will inform Bootloader to extract
                                             *       and update on next power cycle after download
                                             *   0 = Service will inform Bootloader that download
                                             *       is complete - Bootloader will NOT extract/update
                                             *       until user / application requests                      */

    wiced_interface_t           ota2_interface;     /* WICED_STA_INTERFACE or WICED_ETHERNET_INTERFACE */

    wiced_config_ap_entry_t*    ota2_ap_info;       /* Alternate AP to use to connect to the OTA2 update server
                                                     * - This is optional. If the default AP has access to the
                                                     *   OTA2 update server, this can be NULL
                                                     * - Use this or ota2_ap_list, not both. The list over-rides this.
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
                                                     * - This is optional, this can be NULL
                                                     * - Use this or ota2_ap_info, not both. This over-rides ota2_ap_info.
                                                     */

    WICED_LOG_LEVEL_T           log_level;          /* debug print log level */

} wiced_ota2_backround_service_params_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

/**
 * Initialize a timed backgound service to check for updates
 *
 * @param[in]  params - ptr to wiced_ota2_backround_service_params_t structure
 * @param[in]  opaque - application value passed to application in callback
 *
 * @return - session pointer
 *           NULL indicates error
 */
void*  wiced_ota2_service_init(wiced_ota2_backround_service_params_t *params, void* opaque);

/**
 * De-initialize the service
 *
 * @param[in]  session_id      - value returned from wiced_ota2_service_init()
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG
 */
wiced_result_t  wiced_ota2_service_deinit(void* session_id);

/**
 * Start the service
 *
 * NOTE: This is a non-blocking call (process is async)
 * NOTE: Register callbacks *before* calling start
 *
 * @param[in]  session_id - value returned from wiced_ota2_service_init()
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG
 */
wiced_result_t  wiced_ota2_service_start(void* session_id);

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
wiced_result_t wiced_ota2_service_app_network_is_down(void* session_id);

/**
 * Stop the service
 *
 * @param[in]  session_id - value returned from wiced_ota2_service_init()
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG

 */
wiced_result_t  wiced_ota2_service_stop(void* session_id);

/**
 * Register or Un-register a callback function to handle the actual update check
 *
 * @param[in]  session_id  - value returned from wiced_ota2_service_init()
 * @param[in]  callback - callback function pointer (NULL to disable)
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG

 */
wiced_result_t  wiced_ota2_service_register_callback(void* session_id, ota2_service_callback update_callback);

/**
 * Force an update check now
 * NOTE: This is a separate call from the scheduled update checking
 *      If a scheduled update is in progress, this call will fail
 *      If a scheduled update is waiting (not active), this call will
 *             - pause the update scheduler
 *             - do the update call (will appropriate callbacks to the application)
 *             - un-pause the update scheduler
 *
 * This will:
 *  - check for an update
 *  - If update is available
 *      if callback registered, will callback app
 *      if (no callback) or (callback returns WICED_SUCCESS)
 *         - download update
 *
 * @param[in]  session_id - value returned from wiced_ota2_service_init()
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG
 */
wiced_result_t  wiced_ota2_service_check_for_updates(void* session_id);

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
wiced_result_t wiced_ota2_service_uri_split(const char* uri, char* host_buff, uint16_t host_buff_len, char* path_buff, uint16_t path_buff_len, uint16_t* port);

/** Output status to console
 *
 * @param[in]  session_id - value returned from wiced_ota2_service_init()
 *
 * @return - WICED_SUCCESS
 *           WICED_ERROR
 *           WICED_BADARG
 */
wiced_result_t wiced_ota2_service_status(void* session_id);

#ifdef __cplusplus
} /*extern "C" */
#endif
