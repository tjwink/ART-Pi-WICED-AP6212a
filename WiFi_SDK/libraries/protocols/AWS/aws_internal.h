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
 *  AWS Internal data structures
 */
#pragma once

#include "wiced_aws.h"
#include "mqtt_common.h"
#include "http_client.h"

#ifdef __cplusplus
extern "C" {
#endif

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

typedef enum
{
    WICED_AWS_PUBLISH_TOPIC           = 0x01,
    WICED_AWS_SUBSCRIBE_TOPIC         = 0x02,
    WICED_AWS_PUBLISH_SUBSCRIBE_TOPIC = 0x03,
} wiced_aws_topic_operation;

typedef struct
{
    char*                       topic;
    wiced_aws_topic_operation   ops;
} wiced_aws_thing_topic;

typedef enum
{
    WICED_AWS_BROKER_IOT_CLOUD = 0,
    WICED_AWS_BROKER_GREENGRASS_CORE,
    WICED_AWS_BROKER_INVALID,
} wiced_aws_broker_type_t;

typedef struct
{
    wiced_mqtt_object_t                 base;
    wiced_aws_endpoint_info_t*          endpoint;
    wiced_mqtt_security_t               security;
    wiced_aws_transport_type_t          transport;
} wiced_aws_mqtt_obj_t;

typedef struct
{
    http_client_t                       client;
    http_client_configuration_info_t    config;
    wiced_bool_t                        client_connected;
} wiced_aws_https_obj_t;

typedef struct
{
    union {
        wiced_aws_mqtt_obj_t            mqtt;
        wiced_aws_https_obj_t           https;
    };

    wiced_aws_transport_type_t          transport;
    wiced_bool_t                        is_connected;
} wiced_aws_internal_handle_t;

struct wiced_aws_thing_internal
{
    wiced_aws_thing_info_t*             config;
    wiced_aws_callback_t                aws_callback;
    wiced_bool_t                        is_initialized;
    wiced_tls_identity_t*               tls_identity;   /* Same Identity Can be used across multiple AWS connections(or multiple tls_contexts') */
    wiced_aws_internal_handle_t*        mqtt_handle;     /* Right now, hardcoding it */
};

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t aws_internal_mqtt_init( wiced_aws_internal_handle_t* aws, wiced_aws_endpoint_info_t* ep );
wiced_result_t aws_internal_mqtt_connect( wiced_aws_internal_handle_t* aws );
wiced_result_t aws_internal_mqtt_disconnect( wiced_aws_internal_handle_t* aws );
wiced_result_t aws_internal_mqtt_publish( wiced_aws_internal_handle_t* aws, char* topic, uint8_t* data, uint32_t length, int qos );
wiced_result_t aws_internal_mqtt_subscribe( wiced_aws_internal_handle_t* aws, char* topic, int qos );
wiced_result_t aws_internal_mqtt_unsubscribe( wiced_aws_internal_handle_t* aws, char* topic );
wiced_result_t aws_internal_mqtt_deinit( wiced_aws_internal_handle_t* aws );

wiced_result_t aws_internal_https_init( wiced_aws_internal_handle_t* aws, wiced_aws_endpoint_info_t* ep );
wiced_result_t aws_internal_https_connect( wiced_aws_internal_handle_t* aws );
wiced_result_t aws_internal_https_disconnect( wiced_aws_internal_handle_t* aws );
wiced_result_t aws_internal_https_publish( wiced_aws_internal_handle_t* aws, char* topic, uint8_t* data, uint32_t length, int qos );
wiced_result_t aws_internal_https_deinit( wiced_aws_internal_handle_t* aws );

wiced_result_t aws_internal_discover_greengrass_cores( wiced_aws_endpoint_info_t* aws_iot_endpoint, wiced_aws_greengrass_callback_t gg_cb );

wiced_result_t aws_resolve_endpoint_address( wiced_aws_endpoint_info_t* endpoint );

extern struct wiced_aws_thing_internal g_aws_thing;

#ifdef __cplusplus
} /* extern "C" */
#endif
