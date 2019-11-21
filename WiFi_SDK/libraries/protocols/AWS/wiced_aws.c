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
 * Implementation for Wiced AWS APIs.
 *
 */

#include "stdint.h"
#include "aws_common.h"
#include "wiced_aws.h"
#include "aws_internal.h"
#include "wiced_tls.h"
#include "wwd_debug.h"

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
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

struct wiced_aws_thing_internal g_aws_thing = { 0 };

static wiced_tls_identity_t* create_tls_identity( void )
{
    wiced_result_t result = WICED_ERROR;
    wiced_tls_identity_t* tls_identity = NULL;
    wiced_aws_thing_security_info_t* credentials = NULL;

    credentials = g_aws_thing.config->credentials;

    if( credentials )
    {
        tls_identity = (wiced_tls_identity_t*) malloc(sizeof( wiced_tls_identity_t) );

        if( tls_identity )
        {
            result = wiced_tls_init_identity( tls_identity, (const char *)credentials->private_key, credentials->key_length,
                                                credentials->certificate, credentials->certificate_length );
            if( result != WICED_SUCCESS )
            {
                goto free_on_error;
            }
        }

        return tls_identity;
    }

free_on_error:
    free( tls_identity );
    return NULL;
}

static int verify_credentials( wiced_aws_thing_security_info_t* creds )
{
    if( creds->private_key && creds->key_length && creds->certificate
        && creds->certificate_length )
    {
        return 1;
    }
    return 0;
}

wiced_result_t aws_resolve_endpoint_address( wiced_aws_endpoint_info_t* endpoint )
{
    wiced_result_t ret;

    if( !endpoint )
        return WICED_BADARG;

    /* If AWS endpoint IP-address is non-zero, then skip the DNS look up */
    if( endpoint->ip_addr.ip.v4 )
    {
        /* do nothing */
        return WICED_SUCCESS;
    }
    else
    {
        ret = wiced_hostname_lookup( endpoint->uri, &endpoint->ip_addr, WICED_AWS_DEFAULT_DNS_TIMEOUT, WICED_AWS_DEFAULT_INTERFACE );
        WPRINT_APP_INFO( ( "[AWS] AWS endpoint: %s is at %u.%u.%u.%u\n", endpoint->uri,
                        (uint8_t)(GET_IPV4_ADDRESS(endpoint->ip_addr) >> 24),
                        (uint8_t)(GET_IPV4_ADDRESS(endpoint->ip_addr) >> 16),
                        (uint8_t)(GET_IPV4_ADDRESS(endpoint->ip_addr) >> 8),
                        (uint8_t)(GET_IPV4_ADDRESS(endpoint->ip_addr) >> 0) ) );
        if ( ret == WICED_ERROR || endpoint->ip_addr.ip.v4 == 0 )
        {
            WPRINT_APP_INFO(("[AWS] DNS Error...resolving %s\n", endpoint->uri));
            return ret;
        }
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_aws_init ( wiced_aws_thing_info_t* thing, wiced_aws_callback_t cb )
{
    if( g_aws_thing.is_initialized )
    {
        return WICED_ALREADY_INITIALIZED;
    }

    if( !thing || !cb )
    {
        return WICED_BADARG;
    }

    g_aws_thing.config          = thing;
    g_aws_thing.aws_callback    = cb;
    g_aws_thing.is_initialized  = WICED_TRUE;

    if( thing->credentials )
    {
        /* First check for AWS IoT credentials */
        if( !verify_credentials( thing->credentials ) )
        {
            return WICED_BADARG;
        }

        g_aws_thing.tls_identity = create_tls_identity();
        if( !g_aws_thing.tls_identity )
        {
            return WICED_ERROR;
        }
    }
    return WICED_SUCCESS;
}

wiced_result_t wiced_aws_discover( wiced_aws_endpoint_info_t* aws_iot_endpoint, wiced_aws_greengrass_callback_t gg_cb )
{
    if( !gg_cb || !aws_iot_endpoint )
    {
        return WICED_BADARG;
    }

    return aws_internal_discover_greengrass_cores(aws_iot_endpoint, gg_cb);
}

wiced_aws_handle_t wiced_aws_create_endpoint( wiced_aws_endpoint_info_t* endpoint )
{
    wiced_result_t result = WICED_ERROR;
    wiced_aws_internal_handle_t* aws = NULL;

    if( !endpoint )
        return (wiced_aws_handle_t)aws;

    aws = malloc( sizeof(wiced_aws_internal_handle_t) );
    aws->transport   = endpoint->transport;

    switch ( endpoint->transport )
    {
        case WICED_AWS_TRANSPORT_MQTT_NATIVE:
        case WICED_AWS_TRANSPORT_MQTT_WEBSOCKET:
        {
            result = aws_internal_mqtt_init(aws, endpoint);
            if( result != WICED_SUCCESS )
            {
                free(aws);
                aws = NULL;
            }
            g_aws_thing.mqtt_handle = aws;
            break;
        }
        case WICED_AWS_TRANSPORT_RESTFUL_HTTPS:
        {
            result = aws_internal_https_init(aws, endpoint);
            if( result != WICED_SUCCESS )
            {
                free(aws);
                aws = NULL;
            }
            break;
        }
        case WICED_AWS_TRANSPORT_INVALID:
        default:
        {
            break;
        }
    }
    return (wiced_aws_handle_t)aws;
}

wiced_result_t wiced_aws_connect( wiced_aws_handle_t handle )
{
    wiced_result_t result = WICED_ERROR;
    wiced_aws_internal_handle_t* aws = (wiced_aws_internal_handle_t *)handle;

    if( !aws )
    {
        return WICED_BADARG;
    }

    switch( aws->transport )
    {
        case WICED_AWS_TRANSPORT_MQTT_NATIVE:
        case WICED_AWS_TRANSPORT_MQTT_WEBSOCKET:
        {
            result = aws_internal_mqtt_connect(aws);
            break;
        }
        case WICED_AWS_TRANSPORT_RESTFUL_HTTPS:
        {
            result = aws_internal_https_connect(aws);
            break;
        }
        case WICED_AWS_TRANSPORT_INVALID:
        default:
        {
            return WICED_UNSUPPORTED;
        }
    }
    return result;
}

wiced_result_t wiced_aws_publish( wiced_aws_handle_t handle, char* topic, uint8_t* data, uint32_t length, wiced_aws_qos_level_t qos )
{
    wiced_aws_internal_handle_t* aws = (wiced_aws_internal_handle_t*)handle;
    wiced_result_t result = WICED_ERROR;

    if( !aws || !topic || !data || !length )
    {
        return WICED_BADARG;
    }
    switch( aws->transport )
    {
        case WICED_AWS_TRANSPORT_MQTT_NATIVE:
        case WICED_AWS_TRANSPORT_MQTT_WEBSOCKET:
        {
            result = aws_internal_mqtt_publish(aws, topic, data, length, qos);
            break;
        }
        case WICED_AWS_TRANSPORT_RESTFUL_HTTPS:
        {
            result = aws_internal_https_publish(aws, topic, data, length, qos);
            break;
        }

        case WICED_AWS_TRANSPORT_INVALID:
        default:
        {
            return WICED_UNSUPPORTED;
        }
    }
    return result;
}

wiced_result_t wiced_aws_subscribe( wiced_aws_handle_t handle, char* topic, wiced_aws_qos_level_t qos )
{
    wiced_aws_internal_handle_t* aws = (wiced_aws_internal_handle_t*)handle;
    wiced_result_t result = WICED_ERROR;

    if( !aws || !topic  )
    {
        return WICED_BADARG;
    }
    switch( aws->transport )
    {
        case WICED_AWS_TRANSPORT_MQTT_NATIVE:
        case WICED_AWS_TRANSPORT_MQTT_WEBSOCKET:
        {
            result = aws_internal_mqtt_subscribe(aws, topic, qos);
            break;
        }
        case WICED_AWS_TRANSPORT_RESTFUL_HTTPS:
        {
            result = WICED_BADARG;
            break;
        }
        case WICED_AWS_TRANSPORT_INVALID:
        default:
            result = WICED_UNSUPPORTED;
    }

    return result;
}

wiced_result_t wiced_aws_unsubscribe( wiced_aws_handle_t handle, char* topic )
{
    wiced_aws_internal_handle_t* aws = (wiced_aws_internal_handle_t*)handle;
    wiced_result_t result = WICED_ERROR;

    if( !aws || !topic  )
    {
        return WICED_BADARG;
    }
    switch( aws->transport )
    {
        case WICED_AWS_TRANSPORT_MQTT_NATIVE:
        case WICED_AWS_TRANSPORT_MQTT_WEBSOCKET:
        {
            result = aws_internal_mqtt_unsubscribe(aws, topic);
            break;
        }
        case WICED_AWS_TRANSPORT_RESTFUL_HTTPS:
        {
            result = WICED_BADARG;
            break;
        }
        case WICED_AWS_TRANSPORT_INVALID:
        default:
            result = WICED_UNSUPPORTED;
    }

    return result;
}

wiced_result_t wiced_aws_disconnect( wiced_aws_handle_t handle )
{
    wiced_aws_internal_handle_t* aws = (wiced_aws_internal_handle_t*)handle;
    wiced_result_t result = WICED_ERROR;

    if( !aws )
    {
        return WICED_BADARG;
    }
    switch( aws->transport )
    {
        case WICED_AWS_TRANSPORT_MQTT_NATIVE:
        case WICED_AWS_TRANSPORT_MQTT_WEBSOCKET:
        {
            result = aws_internal_mqtt_disconnect(aws);
            break;
        }
        case WICED_AWS_TRANSPORT_RESTFUL_HTTPS:
        {
            result = aws_internal_https_disconnect(aws);
            break;
        }
        case WICED_AWS_TRANSPORT_INVALID:
        default:
            result = WICED_UNSUPPORTED;
            break;
    }

    free(aws);

    return result;
}

wiced_result_t wiced_aws_deinit( void )
{

    /*
     1. First check if we have any existing connections? Is HTTP client still up & connected?
        Is there any AWS handle Still connected to a greengrass core?

     2. Close the open/connected aws-connection handles.

     3. Free up the TLS identity resources.

     4. reset the g_aws_thing.

     */

    wiced_tls_deinit_identity(g_aws_thing.tls_identity);
    free(g_aws_thing.tls_identity);

    g_aws_thing.tls_identity    = NULL;
    g_aws_thing.aws_callback    = NULL;
    g_aws_thing.is_initialized  = WICED_FALSE;
    g_aws_thing.config          = NULL;

    return WICED_SUCCESS;
}
