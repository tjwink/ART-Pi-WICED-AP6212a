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

#include "mqtt_api.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define MESH_NODE_UNPROVISIONED         0   // NODE in UNPROVISIONED STATE
#define MESH_NODE_PROVISIONED           1   // NODE in PROVISIONED STATE
#define MESH_PROVISION_RESULT_SUCCESS   0   ///< Provisioning succeeded
#define MESH_PROVISION_RESULT_TIMEOUT   1   ///< Provisioning failed due to timeout
#define MESH_PROVISION_RESULT_FAILED    2   ///< Provisioning  failed

/******************************************************
 *                    Structures
 ******************************************************/
struct wiced_big_mqtt
{
        wiced_mqtt_object_t mqtt_object;
        char *topic;
        int  topic_len;
};

#ifdef __cplusplus
} /*extern "C" */
#endif
