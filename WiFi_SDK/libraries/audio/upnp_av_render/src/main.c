/*
 * $ Copyright Broadcom Corporation $
 */

/* main.c - Main program routines
 *
 * Copyright (C) 2005-2007   Ivo Clarysse
 *
 * This file is part of GMediaRender.
 *
 * GMediaRender is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * GMediaRender is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Library General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GMediaRender; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 */

#define _GNU_SOURCE

#ifdef HAVE_CONFIG_H
    #include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "wiced_result.h"
#include "wiced_wifi.h"
#include "upnp_av_render.h"
#include "audio_client.h"

#include <threadutil/inc/ithread.h>
// For version strings of upnp
#include <upnp/inc/upnpconfig.h>
#include <upnp/inc/upnp.h>
#include <upnp/inc/upnptools.h>
#include <upnp/src/inc/upnputil.h>

#include "logging.h"
#include "output.h"
#include "upnp.h"
#include "upnp_control.h"
#include "upnp_device.h"
#include "upnp_renderer.h"
#include "upnp_transport.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define UPNPAVRENDER_SERVICE_ID ( 0xBEF0A345 )
#define AVTransport    0
#define RenderingControl    1

const char*            UPnPServiceType[] =
{
    "urn:schemas-upnp-org:service:AVTransport:1",
    "urn:schemas-upnp-org:service:RenderingControl:1"
};

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct upnpavrender_service_s
{
    uint32_t                       id;
    struct upnp_device_descriptor* upnp_renderer;
    struct upnp_device*            device;
    char                           ip_address_string[ 16 ];
    audio_client_ref               audio_client_handle;
} upnpavrender_service_t;

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const upnpavrender_transport_state_t transport_state_lookup_table[] =
{
    UPNPAVRENDER_TRANSPORT_STOPPED,
    UPNPAVRENDER_TRANSPORT_PLAYING,
    UPNPAVRENDER_TRANSPORT_TRANSITIONING,
    UPNPAVRENDER_TRANSPORT_PAUSED_PLAYBACK,
    UPNPAVRENDER_TRANSPORT_PAUSED_RECORDING,
    UPNPAVRENDER_TRANSPORT_RECORDING,
    UPNPAVRENDER_TRANSPORT_NO_MEDIA_PRESENT
};

/******************************************************
 *               Function Definitions
 ******************************************************/

#if defined( ENABLE_TRANSPORT_LOGGING ) || defined( ENABLE_CONTROL_LOGGING )
static void log_variable_change( void* userdata, int var_num,
                                 const char* variable_name,
                                 const char* old_value,
                                 const char* variable_value )
{
    const char* category = (const char*) userdata;
    int         needs_newline = variable_value[ strlen( variable_value ) - 1 ] != '\n';
    // Silly terminal codes. Set to empty strings if not needed.
    const char* var_start = Log_color_allowed( ) ? "\033[1m\033[34m" : "";
    const char* var_end = Log_color_allowed( ) ? "\033[0m" : "";

    Log_info( category, "%s%s%s: %s%s",
              var_start, variable_name, var_end,
              variable_value, needs_newline ? "\n" : "" );
}
#endif /* ENABLE_TRANSPORT_LOGGING || ENABLE_CONTROL_LOGGING */


static void init_logging( const char* log_file )
{
    char version[ 1024 ];

    snprintf( version, sizeof( version ), "[ gmediarender %s "
                                          "(libupnp-%s) ]",
              GM_COMPILE_VERSION, UPNP_VERSION_STRING );

    Log_init( log_file );
    Log_info( "main", "%s log started %s", PACKAGE_STRING, version );
}


void ipv4_to_string( const wiced_ip_address_t* addr, char* addr_str, uint32_t addr_str_size )
{
    uint8_t ipv4[ 4 ];

    memcpy( ipv4, &addr->ip.v4, sizeof( addr->ip.v4 ) );
    snprintf( addr_str, addr_str_size, "%u.%u.%u.%u", ipv4[ 3 ], ipv4[ 2 ], ipv4[ 1 ], ipv4[ 0 ] );
    Log_info( "main", "Using IPv4 address %s", addr_str );
}


wiced_result_t upnpavrender_service_start( upnpavrender_service_params_t* params, upnpavrender_service_ref* service_handle )
{
    wiced_result_t          result  = WICED_ERROR;
    upnpavrender_service_t* phandle = NULL;
    int                     rc;
    wiced_ip_address_t      ipv4;

    if ( ( params->listening_port != 0 ) && ( ( params->listening_port < 49152 ) || ( params->listening_port > 65535 ) ) )
    {
        // Somewhere obscure internally in libupnp, they clamp the
        // port to be outside of the IANA range, so at least 49152.
        // Instead of surprising the user by ignoring lower port
        // numbers, complain loudly.
        Log_error( "main", "ERROR: listening port needs to be in range [49152..65535] (but was set to %hu)", params->listening_port );
        result = WICED_BADARG;
        goto _exit;
    }

    phandle = calloc( 1, sizeof( upnpavrender_service_t ) );
    if ( phandle == NULL )
    {
        Log_error( "main", "ERROR: calloc() failed !" );
        result = WICED_OUT_OF_HEAP_SPACE;
        goto _exit;
    }
    phandle->id = UPNPAVRENDER_SERVICE_ID;

    init_logging( NULL );

    phandle->upnp_renderer = upnp_renderer_descriptor( params->friendly_name, params->uuid );
    if ( phandle->upnp_renderer == NULL )
    {
        Log_error( "main", "ERROR: Failed to get renderer descriptor" );
        result = WICED_ERROR;
        goto _exit;
    }

    result = wiced_ip_get_ipv4_address( params->interface, &ipv4 );
    wiced_action_jump_when_not_true( result == WICED_SUCCESS, _exit, Log_error( "main", "ERROR: wiced_ip_get_ipv4_address() failed !" ) );
    ipv4_to_string( &ipv4, phandle->ip_address_string, sizeof( phandle->ip_address_string ) );

    output_add_options(params);

    rc = output_init( NULL );
    if ( rc != 0 )
    {
        Log_error( "main", "ERROR: Failed to initialize Output subsystem" );
        result = WICED_ERROR;
        goto _exit;
    }

    phandle->audio_client_handle = (audio_client_ref) output_get_audio_handle();
    if ( phandle->audio_client_handle == NULL )
    {
        Log_error( "main", "ERROR: Failed to get audio client handle" );
        result = WICED_ERROR;
        goto _exit;
    }

    phandle->device = upnp_device_init( phandle->upnp_renderer, params->interface, phandle->ip_address_string, params->listening_port );
    if ( phandle->device == NULL )
    {
        Log_error( "main", "ERROR: Failed to initialize UPnP device" );
        result = WICED_ERROR;
        goto _exit;
    }

    upnp_transport_init( phandle->device );
    upnp_control_init( phandle->device );

    if (Log_info_enabled())
    {
#ifdef ENABLE_TRANSPORT_LOGGING
        upnp_transport_register_variable_listener(log_variable_change, (void*) "transport");
#endif
#ifdef ENABLE_CONTROL_LOGGING
        upnp_control_register_variable_listener(log_variable_change, (void*) "control");
#endif
    }

    Log_info("main", "Ready for rendering.");

    *service_handle = phandle;

    result = WICED_SUCCESS;

_exit:
    if ( result != WICED_SUCCESS )
    {
        free( phandle );
    }
    return result;
}


wiced_result_t upnpavrender_service_stop( upnpavrender_service_ref service_handle )
{
    wiced_result_t result = WICED_SUCCESS;

    if ( service_handle == NULL )
    {
        Log_error( "main", "ERROR: service handle is NULL !" );
        result = WICED_BADARG;
        goto _exit;
    }

    if ( service_handle->id != UPNPAVRENDER_SERVICE_ID )
    {
        Log_error( "main", "ERROR: service handle is not valid !" );
        result = WICED_BADARG;
        goto _exit;
    }

    upnp_control_deinit();
    upnp_transport_deinit();
    output_deinit();
    upnp_device_shutdown( service_handle->device );
    service_handle->id = 0;
    free(service_handle);

_exit:
    return result;
}

static IXML_Document* create_action_node( int service, const char* actionname, const char** param_name, char** param_val, int param_count )
{

    IXML_Document* actionNode = NULL;
    int param;

    if ( 0 == param_count )
    {
        actionNode = UpnpMakeAction( actionname, UPnPServiceType[ service ], 0, NULL );
    }
    else
    {
        for ( param = 0; param < param_count; param++ )
        {
            if ( UpnpAddToAction( &actionNode, actionname, UPnPServiceType[ service ], param_name[ param ], param_val[ param ] ) != UPNP_E_SUCCESS )
            {
                return NULL;
            }
        }
    }

    return actionNode;
}

/* TODO: CHECK TO USE XMLDOC */
static char *upnp_find_string( IXML_Document* start_node, const char *key )
{
    IXML_Node *node;

    node = (IXML_Node*) start_node;
    if ( node == NULL )
    {
        return NULL;
    }
    node = ixmlNode_getFirstChild( node );
    if ( node == NULL )
    {
        return NULL;
    }
    node = ixmlNode_getFirstChild( node );

    for ( /**/; node != NULL; node = ixmlNode_getNextSibling( node ) )
    {
        if ( strcmp( ixmlNode_getNodeName( node ), key ) == 0 )
        {
            node = ixmlNode_getFirstChild( node );
            const char *node_value = ( node != NULL ? ixmlNode_getNodeValue( node ) : NULL );
            return strdup( node_value != NULL ? node_value : "" );
        }
    }

    return NULL;
}

static struct action_event* create_action_event( int service, const char* actionname, const char** param_name, char** param_val, int param_count )
{
    struct action_event *event = NULL;
    struct Upnp_Action_Request *request = NULL;

    event = (struct action_event *) malloc( sizeof(struct action_event) );
    if ( event == NULL )
    {
        return NULL;
    }

    request = (struct Upnp_Action_Request *) malloc( sizeof(struct Upnp_Action_Request) );
    if ( request == NULL )
    {
        free( event );
        return NULL;
    }

    request->ActionRequest = create_action_node( service, actionname, param_name, param_val, param_count );
    if ( request->ActionRequest == NULL )
    {
        free( request );
        free( event );
        return NULL;
    }

    namecopy( request->ActionName, actionname );
    linecopy( request->ErrStr, "" );
    request->ActionResult = NULL;
    request->ErrCode = UPNP_E_SUCCESS;

    event->request = request;

    return event;
}

static wiced_result_t destroy_action_event( struct action_event *event )
{
    if ( event == NULL || event->request == NULL )
    {
        return WICED_ERROR;
    }

    if ( event->request->ActionRequest )
    {
        ixmlDocument_free( event->request->ActionRequest );
    }

    if ( event->request->ActionResult )
    {
        ixmlDocument_free( event->request->ActionResult );
    }

    free( event->request );
    free( event );

    return WICED_SUCCESS;
}

wiced_result_t upnpavrender_service_pause( upnpavrender_service_ref handle )
{
    wiced_result_t result = WICED_ERROR;
    struct action_event *event;
    const char* param = "InstanceID";
    char* param_val = "0";

    if ( handle == NULL )
    {
        Log_error( "main", "ERROR: service handle is NULL !" );
        result = WICED_BADARG;
        goto _exit;
    }

    if ( handle->id != UPNPAVRENDER_SERVICE_ID )
    {
        Log_error( "main", "ERROR: service handle is not valid !" );
        result = WICED_BADARG;
        goto _exit;
    }

    /*
     * <u:Pause xmlns:u="urn:schemas-upnp-org:service:AVTransport:1">
     * <InstanceID>0</InstanceID>
     * </u:Pause>
     */
    event = create_action_event( AVTransport, "Pause", &param, &param_val, 1 );

    if ( event == NULL )
    {
        return WICED_ERROR;
    }

    if ( !pause_stream( event ) )
    {
        result = WICED_SUCCESS;
    }

    destroy_action_event( event );

_exit:
    return result;
}

wiced_result_t upnpavrender_service_resume( upnpavrender_service_ref handle )
{
    wiced_result_t result = WICED_ERROR;
    struct action_event *event;
    const char* param = "InstanceID";
    char* param_val = "0";

    if ( handle == NULL )
    {
        Log_error( "main", "ERROR: service handle is NULL !" );
        result = WICED_BADARG;
        goto _exit;
    }

    if ( handle->id != UPNPAVRENDER_SERVICE_ID )
    {
        Log_error( "main", "ERROR: service handle is not valid !" );
        result = WICED_BADARG;
        goto _exit;
    }

    /*
     * <u:Play xmlns:u="urn:schemas-upnp-org:service:AVTransport:1">
     * <InstanceID>0</InstanceID>
     * </u:Play>
     */
    event = create_action_event( AVTransport, "Play", &param, &param_val, 1 );
    if ( event == NULL )
    {
        goto _exit;
    }

    if ( !play( event ) )
    {
        result = WICED_SUCCESS;
    }

    destroy_action_event( event );

_exit:
    return result;
}

wiced_result_t upnpavrender_service_stop_playing( upnpavrender_service_ref handle )
{
    wiced_result_t result = WICED_ERROR;
    struct action_event *event;
    const char* param = "InstanceID";
    char* param_val = "0";

    if ( handle == NULL )
    {
        Log_error( "main", "ERROR: service handle is NULL !" );
        result = WICED_BADARG;
        return result;
    }

    if ( handle->id != UPNPAVRENDER_SERVICE_ID )
    {
        Log_error( "main", "ERROR: service handle is not valid !" );
        result = WICED_BADARG;
        return result;
    }

    /*
     * <u:Stop xmlns:u="urn:schemas-upnp-org:service:AVTransport:1">
     * <InstanceID>0</InstanceID>
     * </u:Stop>
     */
    event = create_action_event( AVTransport, "Stop", &param, &param_val, 1 );
    if ( event == NULL )
    {
        return result;
    }

    if ( !stop( event ) )
    {
        result = WICED_SUCCESS;
    }

    destroy_action_event( event );
    return result;
}

wiced_result_t upnpavrender_service_get_volume( upnpavrender_service_ref handle , int *volume )
{
    wiced_result_t result = WICED_ERROR;
    struct action_event *event = NULL;
    const char* param = "InstanceID";
    char* param_val = "0";
    char* volume_str;

    if ( handle == NULL )
    {
        Log_error( "main", "ERROR: service handle is NULL !" );
        result = WICED_BADARG;
        goto _exit;
    }

    if ( handle->id != UPNPAVRENDER_SERVICE_ID )
    {
        Log_error( "main", "ERROR: service handle is not valid !" );
        result = WICED_BADARG;
        goto _exit;
    }

    if ( volume == NULL )
    {
        result = WICED_BADARG;
        goto _exit;
    }

    /*
     * <u:GetVolume xmlns:u="urn:schemas-upnp-org:service:RenderingControl:1">
     * <InstanceID>0</InstanceID>
     * </u:GetVolume>
     */

    event = create_action_event( RenderingControl, "GetVolume", &param, &param_val, 1 );

    if ( event == NULL )
    {
        goto _exit;
    }

    event->service = upnp_control_get_service( );
    event->status = 0;

    if ( get_volume( event ) )
    {
        goto _exit;
    }

    volume_str = upnp_find_string( event->request->ActionResult, "CurrentVolume" );

    if ( volume_str != NULL )
    {
        *volume = atoi( volume_str );
        free( volume_str );
        result = WICED_SUCCESS;
    }

 _exit:
    if ( event != NULL )
    {
        destroy_action_event( event );
    }
    return result;
}

wiced_result_t upnpavrender_service_set_volume( upnpavrender_service_ref handle, int volume )
{
    wiced_result_t result = WICED_ERROR;
    struct action_event *event;
    const char* param[ 2 ] =
    { "InstanceID", "DesiredVolume" };
    char* param_val[ 2 ] =
    { "0" };
    char param_val_a[ 4 ];

    if ( handle == NULL )
    {
        Log_error( "main", "ERROR: service handle is NULL !" );
        result = WICED_BADARG;
        goto _exit;
    }

    if ( handle->id != UPNPAVRENDER_SERVICE_ID )
    {
        Log_error( "main", "ERROR: service handle is not valid !" );
        result = WICED_BADARG;
        goto _exit;
    }

    param_val[ 1 ] = param_val_a;
    sprintf( param_val_a, "%d", volume );

    /*
     * <u:SetVolume xmlns:u="urn:schemas-upnp-org:service:RenderingControl:1">
     * <InstanceID>0</InstanceID>
     * <DesiredVolume>2</DesiredVolume>
     * </u:SetVolume>
     */

    event = create_action_event( RenderingControl, "SetVolume", param, param_val, 2 );

    if ( event == NULL )
    {
        goto _exit;
    }

    if ( !set_volume( event ) )
    {
        result = WICED_SUCCESS;
    }

    destroy_action_event( event );

_exit:
    return result;
}

wiced_result_t upnpavrender_service_get_transport_state( upnpavrender_service_ref handle, upnpavrender_transport_state_t *state )
{
    wiced_result_t result = WICED_SUCCESS;

    if ( handle == NULL )
    {
        Log_error( "main", "ERROR: service handle is NULL !" );
        result = WICED_BADARG;
        goto _exit;
    }

    if ( handle->id != UPNPAVRENDER_SERVICE_ID )
    {
        Log_error( "main", "ERROR: service handle is not valid !" );
        result = WICED_BADARG;
        goto _exit;
    }

    if ( state == NULL )
    {
        result = WICED_BADARG;
        goto _exit;
    }

    *state = transport_state_lookup_table[get_transport_state()];

_exit:
    return result;
}

audio_client_ref upnpavrender_service_get_audio_handle( upnpavrender_service_ref handle )
{
    audio_client_ref audio_client_handle = (audio_client_ref) NULL;

    if ( handle == NULL )
    {
        Log_error( "main", "ERROR: service handle is NULL !" );
        goto _exit;
    }

    if ( handle->id != UPNPAVRENDER_SERVICE_ID )
    {
        Log_error( "main", "ERROR: service handle is not valid !" );
        goto _exit;
    }

    audio_client_handle = handle->audio_client_handle;

_exit:
    return audio_client_handle;
}
