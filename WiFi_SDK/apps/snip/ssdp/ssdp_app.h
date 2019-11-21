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

#include "ssdp_app_dct.h"
#include "http_stream.h"
#include "daemons/SSDP/wiced_ssdp.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define SSDP_DEFAULT_PORT           (6900)

#define APP_TAG_VALID                0x42EDBABE
#define APP_TAG_INVALID              0xDEADBEEF

#define MSEARCH_RESPONSES_MAX      (16)         /* maximum M-Search responses we will store */

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum {
    APP_EVENT_SHUTDOWN          = (1 << 0),

    APP_EVENT_SSDP_INIT         = (1 << 1),
    APP_EVENT_SSDP_DEINIT       = (1 << 2),

    APP_EVENT_SSDP_MSEARCH      = (1 << 3),
    APP_EVENT_SSDP_INFORMATION  = (1 << 4),

    APP_EVENT_SSDP_LOG_LEVEL_OFF   = (1 << 5),
    APP_EVENT_SSDP_LOG_LEVEL_LOW   = (1 << 6),
    APP_EVENT_SSDP_LOG_LEVEL_INFO  = (1 << 7),
    APP_EVENT_SSDP_LOG_LEVEL_DEBUG = (1 << 8),

} APP_EVENTS_T;

#define APP_ALL_EVENTS       (-1)

/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct application_info_s {
    uint32_t                        tag;

    int                             app_done;

    wiced_event_flags_t             events;

    wiced_ip_setting_t              ip_settings;

    application_dct_t               *dct_app;
    platform_dct_network_config_t   *dct_network;
    platform_dct_wifi_config_t      *dct_wifi;

    wiced_thread_t                  app_thread;
    wiced_thread_t                  *app_thread_ptr;

    wiced_ssdp_params_t             ssdp_params;
    void                            *ssdp_info;

    wiced_ssdp_msearch_response_t   msearch_responses[MSEARCH_RESPONSES_MAX];
} application_info_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/


#ifdef __cplusplus
} /* extern "C" */
#endif
