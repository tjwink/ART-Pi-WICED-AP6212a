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

/**
 * @file UPnP AV rendering service library stub
 */

#include <stdlib.h>

#include "upnp_av_render.h"

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
 *               Function Definitions
 ******************************************************/

wiced_result_t upnpavrender_service_start( upnpavrender_service_params_t* params, upnpavrender_service_ref* phandle )
{
    UNUSED_PARAMETER(params);
    UNUSED_PARAMETER(phandle);
    return WICED_UNSUPPORTED;
}

wiced_result_t upnpavrender_service_stop( upnpavrender_service_ref handle )
{
    UNUSED_PARAMETER(handle);
    return WICED_UNSUPPORTED;
}

wiced_result_t upnpavrender_service_pause( upnpavrender_service_ref handle )
{
    UNUSED_PARAMETER(handle);
    return WICED_UNSUPPORTED;
}

wiced_result_t upnpavrender_service_resume( upnpavrender_service_ref handle )
{
    UNUSED_PARAMETER(handle);
    return WICED_UNSUPPORTED;
}

wiced_result_t upnpavrender_service_stop_playing( upnpavrender_service_ref handle )
{
    UNUSED_PARAMETER(handle);
    return WICED_UNSUPPORTED;
}

wiced_result_t upnpavrender_service_get_transport_state( upnpavrender_service_ref handle, upnpavrender_transport_state_t *state )
{
    UNUSED_PARAMETER(handle);
    UNUSED_PARAMETER(state);
    return WICED_UNSUPPORTED;
}

wiced_result_t upnpavrender_service_set_volume( upnpavrender_service_ref handle, int volume )
{
    UNUSED_PARAMETER(handle);
    UNUSED_PARAMETER(volume);
    return WICED_UNSUPPORTED;
}

wiced_result_t upnpavrender_service_get_volume( upnpavrender_service_ref handle, int *volume )
{
    UNUSED_PARAMETER(handle);
    UNUSED_PARAMETER(volume);
    return WICED_UNSUPPORTED;
}

audio_client_ref upnpavrender_service_get_audio_handle( upnpavrender_service_ref handle )
{
    UNUSED_PARAMETER(handle);
    return (audio_client_ref)NULL;
}
