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

#include "http_stream.h"
#include "dns.h"

#include "ota2_test_dct.h"
#include "wiced_ota2_service.h"
#include "wiced_ota2_network.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define OTA2_THREAD_STACK_SIZE  8000

#define OTA2_HTTP_THREAD_PRIORITY       (WICED_DEFAULT_WORKER_PRIORITY)


#define SECONDS_PER_MINUTE              (60)
#define SECONDS_PER_HOUR                (SECONDS_PER_MINUTE * 60)
#define SECONDS_IN_24_HOURS             (SECONDS_PER_HOUR * 24) //86400

#define PLAYER_ALL_EVENTS       (-1)

/******************************************************
 *                   Enumerations
 ******************************************************/
    typedef enum {
        PLAYER_EVENT_STOP_TIMED_UPDATE               = (1 <<  0),
        PLAYER_EVENT_GET_UPDATE                            = (1 <<  1)

    } PLAYER_EVENTS_T;


/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct ota2_data_s {

    WICED_LOG_LEVEL_T              log_level;
    wiced_event_flags_t             events;

    platform_dct_network_config_t   dct_network;
    wiced_config_ap_entry_t         stored_ap_list[CONFIG_AP_LIST_SIZE];
    ota2_dct_t                      dct_app;

    /* source info for http streaming */
    char                            uri_to_stream[WICED_OTA2_HTTP_QUERY_SIZE];

    /* background service for OTA2 info */
    char                                    ota2_host_name[WICED_OTA2_HOST_NAME];
    char                                    ota2_file_path[WICED_OTA2_FILE_PATH];
    wiced_ota2_backround_service_params_t   ota2_bg_params;
    void*                                   ota2_bg_service;

} ota2_data_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

#ifdef __cplusplus
} /* extern "C" */
#endif
