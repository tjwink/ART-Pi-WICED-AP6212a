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
 * Implementation for Wiced AWS RESTful HTTPS APIs
 *
 */

#include <stdint.h>
#include "aws_common.h"
#include "wiced_aws.h"
#include "aws_internal.h"

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

// static void aws_http_event_handler( http_client_t* client, http_event_t event, http_response_t* response )
// {
//     (void)client;
//     (void)event;
//     (void)response;
// }

wiced_result_t aws_internal_https_init( wiced_aws_internal_handle_t* aws, wiced_aws_endpoint_info_t* ep )
{
    if( !aws || !verify_aws_type( aws, WICED_AWS_TRANSPORT_RESTFUL_HTTPS ) )
        return WICED_BADARG;
    (void)ep;
    return WICED_UNSUPPORTED;
}

wiced_result_t aws_internal_https_connect( wiced_aws_internal_handle_t* aws )
{
    if( !aws || !verify_aws_type( aws, WICED_AWS_TRANSPORT_RESTFUL_HTTPS ) )
        return WICED_BADARG;
    return WICED_UNSUPPORTED;
}

wiced_result_t aws_internal_https_publish( wiced_aws_internal_handle_t* aws, char* topic, uint8_t* data, uint32_t length, int qos )
{
    if( !aws || !verify_aws_type( aws, WICED_AWS_TRANSPORT_RESTFUL_HTTPS ) )
        return WICED_BADARG;
    (void)topic;
    (void)data;
    (void)length;
    (void)qos;
    return WICED_UNSUPPORTED;
}

wiced_result_t aws_internal_https_disconnect( wiced_aws_internal_handle_t* aws )
{
    if( !aws || !verify_aws_type( aws, WICED_AWS_TRANSPORT_RESTFUL_HTTPS ) )
        return WICED_BADARG;
    return WICED_UNSUPPORTED;
}

wiced_result_t aws_internal_https_deinit( wiced_aws_internal_handle_t* aws )
{
    if( !aws || !verify_aws_type( aws, WICED_AWS_TRANSPORT_RESTFUL_HTTPS ) )
        return WICED_BADARG;
    return WICED_UNSUPPORTED;
}
