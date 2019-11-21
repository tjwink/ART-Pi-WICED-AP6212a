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
 *  Home for common defines & configuration for Wiced AWS
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#define WICED_AWS_DEFAULT_INTERFACE                 (WICED_STA_INTERFACE)
#define WICED_AWS_MAX_CONNECTIONS                   (2)
#define WICED_AWS_MQTT_KEEP_ALIVE_TIMEOUT           (5)         //in seconds
#define WICED_AWS_DEFAULT_CONNECTION_RETRIES        (3)
#define WICED_AWS_DEFAULT_DNS_TIMEOUT               (10000)     // milliseconds
#define WICED_AWS_REQUEST_TIMEOUT                   (5000)      // milliseconds
#define WICED_AWS_IOT_DEFAULT_MQTT_PORT             (8883)

#define GREENGRASS_DISCOVERY_HTTP_REQUEST_URI_PREFIX  "/greengrass/discover/thing/"
#define WICED_AWS_GG_HTTPS_CONNECT_TIMEOUT          (2000)
#define WICED_AWS_GREENGRASS_DISCOVERY_TIMEOUT      (5000)

#define WICED_AWS_GG_METADATA_MAX_LENGTH            (128)
#define WICED_AWS_GG_THING_ARN_MAX_LENGTH           (128)
#define WICED_AWS_GG_GROUP_ID_MAX_LENGTH            (64)
#define WICED_AWS_GG_ROOT_CA_MAX_LENGTH             (2000)
#define WICED_AWS_GG_MAX_CONNECTIONS                (1)

#define WICED_AWS_GG_HTTPS_SERVER_PORT              (8443)

#define GG_GROUP_ID                                 "GGGroupId"
#define GG_GROUP_KEY                                "GGGroups"
#define GG_HOST_ADDRESS                             "HostAddress"
#define GG_PORT                                     "PortNumber"
#define GG_METADATA                                 "Metadata"
#define GG_ROOT_CAS                                 "CAs"
#define GG_CORE_THING_ARN                           "thingArn"
#define GG_BEGIN_CERTIFICATE                        "-----BEGIN CERTIFICATE-----"

#define verify_aws_type(aws, type)   ( (aws->transport == type) ? 1 : 0 )

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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/


#ifdef __cplusplus
} /*extern "C" */
#endif
