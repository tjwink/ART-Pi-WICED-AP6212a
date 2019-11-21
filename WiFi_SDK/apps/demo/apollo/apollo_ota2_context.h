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

#if defined(OTA2_SUPPORT)

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced.h"
#include "wiced_ota2_image.h"
#include "wiced_ota2_service.h"

#include "apollo_dct.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

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
        /* Set these values in Apollo before calling the update functions */
    char                                    update_uri[ WICED_OTA2_HTTP_QUERY_SIZE + 1 ];   /* URI of OTA2 Update file */
    wiced_ota2_backround_service_params_t   bg_params;          /* Parameters for the OTA2 background service initialization */
    void*                                   cb_opaque;          /* data passed to OTA2 callback function    */
    int                                     log_level;          /* log level                                */

    /* background service for OTA2 info */
    char                                    host_name[ WICED_OTA2_HOST_NAME + 1];
    char                                    file_path[ WICED_OTA2_FILE_PATH + 1];
    uint16_t                                port;
    void*                                   bg_service;

    /* apollo specific variables */
    wiced_bool_t                            perform_ota2_update_at_start;  /* If TRUE, one button OTA2 update */
    uint8_t                                 ota2_download_percentage;      /* 0-100 while downloading */

} apollo_ota2_service_info_t;


/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  /* defined(OTA2_SUPPORT) */
