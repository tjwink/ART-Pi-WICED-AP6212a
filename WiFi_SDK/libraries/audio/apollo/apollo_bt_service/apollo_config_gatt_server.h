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
 *
 * This file provides definitions and function prototypes for Apollo config
 * device
 *
 */
#pragma once

#include "wiced_result.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define APOLLO_CONFIG_GATT_ATTRIBUTE_SIZE (22)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    APOLLO_CONFIG_GATT_EVENT_DCT_READ,
    APOLLO_CONFIG_GATT_EVENT_DCT_WRITE,
    APOLLO_CONFIG_GATT_EVENT_DCT_WRITE_COMPLETED
} apollo_config_gatt_event_t;


/******************************************************
 *                 Type Definitions
 ******************************************************/

typedef struct
{
    char                spk_name[APOLLO_CONFIG_GATT_ATTRIBUTE_SIZE];
    uint8_t             mode;
    uint8_t             is_configured;
    wiced_security_t    security;
    char                nw_ssid_name[APOLLO_CONFIG_GATT_ATTRIBUTE_SIZE];
    char                nw_pass_phrase[APOLLO_CONFIG_GATT_ATTRIBUTE_SIZE];
    uint8_t             spk_channel_map;
    uint8_t             spk_vol;
    uint8_t             src_type;
    uint8_t             src_vol;
} apollo_config_gatt_server_dct_t;

typedef wiced_result_t ( *gatt_event_cbf )( apollo_config_gatt_event_t event, apollo_config_gatt_server_dct_t *dct, void *user_context );

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    gatt_event_cbf  gatt_event_cbf;   /*!< GATT configuration service event callback @apollo_config_gatt_event_t */
    void           *user_context;     /*!< User context pointer                                                  */
} apollo_config_gatt_server_params_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/**
 * Start Bluetooth GATT configuration services.
 * apollo_bt_service_init() must be called first; otherwise, this call will fail.
 *
 * @param[in]  params          GATT server initialization parameters @apollo_config_gatt_server_params_t
 *
 * @return @ref wiced_result_t
 */
wiced_result_t apollo_config_gatt_server_start( apollo_config_gatt_server_params_t *params );


/**
 * Shutdown and cleanup GATT configuration service.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t apollo_config_gatt_server_stop ( void );

#ifdef __cplusplus
} /* extern "C" */
#endif
