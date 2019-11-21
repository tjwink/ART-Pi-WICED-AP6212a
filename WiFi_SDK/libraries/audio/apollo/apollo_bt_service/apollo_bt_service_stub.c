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

/** @file Apollo audio application.
 *
 */

#include "wiced_result.h"
#include "apollo_bt_service.h"
#include "apollo_config_gatt_server.h"

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

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t apollo_bt_service_init( apollo_bt_service_init_params_t *params )
{
    UNUSED_PARAMETER(params);
    return WICED_UNSUPPORTED;
}


wiced_result_t apollo_bt_service_deinit( void )
{
    return WICED_UNSUPPORTED;
}


wiced_result_t apollo_bt_a2dp_sink_init  ( apollo_bt_a2dp_sink_init_params_t *params )
{
    UNUSED_PARAMETER(params);
    return WICED_UNSUPPORTED;
}


wiced_result_t apollo_bt_a2dp_sink_deinit( void )
{
    return WICED_UNSUPPORTED;
}


wiced_result_t apollo_bt_a2dp_sink_set_ouput_mode( apollo_bt_a2dp_sink_output_mode_t output_mode )
{
    UNUSED_PARAMETER(output_mode);
    return WICED_UNSUPPORTED;
}


wiced_result_t apollo_bt_remote_control_send_command( apollo_bt_remote_control_command_t cmd )
{
    UNUSED_PARAMETER(cmd);
    return WICED_UNSUPPORTED;
}


wiced_result_t apollo_config_gatt_server_start( apollo_config_gatt_server_params_t *params )
{
    UNUSED_PARAMETER(params);
    return WICED_UNSUPPORTED;
}


wiced_result_t apollo_config_gatt_server_stop( void )
{
    return WICED_UNSUPPORTED;
}
