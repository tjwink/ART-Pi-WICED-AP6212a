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

#include "wiced.h"
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

typedef struct
{
    platform_dct_network_config_t*  dct_network;
    platform_dct_wifi_config_t*     dct_wifi;
#ifdef WICED_DCT_INCLUDE_BT_CONFIG
    platform_dct_bt_config_t*       dct_bt;
#endif
    apollo_dct_t*                   dct_app;
} apollo_dct_collection_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
/* when console command changes the DCT data */
wiced_result_t apollo_config_reload_dct_wifi(apollo_dct_collection_t* dct_tables);
wiced_result_t apollo_config_reload_dct_network(apollo_dct_collection_t* dct_tables);
wiced_result_t apollo_config_reload_dct_bluetooth(apollo_dct_collection_t* dct_tables);

wiced_result_t apollo_config_init(apollo_dct_collection_t *dct_tables);
wiced_result_t apollo_config_deinit(apollo_dct_collection_t *dct_tables);
void apollo_set_config(apollo_dct_collection_t* dct_tables, int argc, char *argv[]);
void apollo_config_print_info(apollo_dct_collection_t* dct_tables);
wiced_result_t apollo_config_save(apollo_dct_collection_t* dct_tables);

/* helper functions */
char* apollo_source_type_get_text(int source_type);

#ifdef __cplusplus
} /* extern "C" */
#endif
