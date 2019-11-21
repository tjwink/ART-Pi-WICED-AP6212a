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

#include "wiced_ota2_service.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/*
 * define the current version here or on make command line as args
 * make <application>-<platform> ota2_image APP_VERSION_FOR_OTA2_MAJOR=x APP_VERSION_FOR_OTA2_MINOR=y
 */
#if defined(OTA2_SUPPORT)
#ifndef APP_VERSION_FOR_OTA2_MAJOR
#define APP_VERSION_FOR_OTA2_MAJOR  1
#endif
#ifndef APP_VERSION_FOR_OTA2_MINOR
#define APP_VERSION_FOR_OTA2_MINOR  0
#endif
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

typedef struct ota2_dct_s
{
        uint32_t                    reboot_count;
        uint8_t                     ota2_reboot_after_download;                     /* automatic reboot after successful download */
        uint16_t                    ota2_major_version;                             /* define APP_VERSION_FOR_OTA2_MAJOR as make arg */
        uint16_t                    ota2_minor_version;                             /* define APP_VERSION_FOR_OTA2_MINOR as make arg */

        char                        ota2_update_uri[WICED_OTA2_HTTP_QUERY_SIZE];    /* default Web address of update package */
} ota2_dct_t;

#ifdef __cplusplus
} /* extern "C" */
#endif
