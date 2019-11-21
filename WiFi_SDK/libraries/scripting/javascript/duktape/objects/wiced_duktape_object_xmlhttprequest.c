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

#include "wiced.h"
#include "wiced_duktape.h"
#include "wiced_time.h"
#include "platform_config.h"
#include "http_client.h"

/* WICED Duktape XMLHttpRequest object
 * Notes:
 *  - At this time the XmlHttpRequest object only supports grabbing from
 *    HTTP(S) sources.
 */

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define LOG_LABEL                   "duk:obj:xhr"
#define LOG_DEBUG_ENABLE            0

#define DNS_TIMEOUT_MS              (10000)
#define CONNECT_TIMEOUT_MS          (10000)

#define HTTP_PREFIX                 "http://"
#define HTTPS_PREFIX                "https://"
#define HTTP_HEADER_ACCEPT          "Accept: "
#define HTTP_ALL                    "*/*"

/*
 * Object properties
 */

/* Macro for defining internal properties by using the "\xff" prefix */
#define INTERNAL_PROPERTY(name)     ("\xff" name)

#define INTERNAL_PROPERTY_ASYNC     INTERNAL_PROPERTY("async")
#define INTERNAL_PROPERTY_DATA      INTERNAL_PROPERTY("data")

#define PROPERTY_ONERROR            "onerror"
#define PROPERTY_ONLOAD             "onload"
#define PROPERTY_RESPONSETEXT       "responseText"
#define PROPERTY_STATUS             "status"

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum {
    CLIENT_STATE_INITIALIZED = 0,
    CLIENT_STATE_CONNECTED,
    CLIENT_STATE_REQUEST_SENT,
    CLIENT_STATE_REQUEST_RECEIVED
} xhr_object_client_state_s;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/* Object data structure */
typedef struct xhr_object_data_s
{
    http_client_t                       client;
    http_client_configuration_info_t    config;
    http_request_t                      request;
    http_method_t                       method;
    wiced_bool_t                        async;
    wiced_bool_t                        has_accept_header;
    xhr_object_client_state_s           state;
    wiced_bool_t                        has_error;
} xhr_object_data_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

static duk_ret_t xhr_object_onerror_callback( duk_context* ctx )
{
    duk_push_this( ctx ); /* -> [this] */

    duk_get_prop_string( ctx, -1, PROPERTY_ONERROR ); /* -> [this onerror] */

    if ( duk_is_function( ctx, -1 ))
    {
        duk_dup( ctx, -2 ); /* -> [this onerror this] */
        LOGD( "Calling onerror callback" );
        if ( duk_pcall_method( ctx, 0 /* nargs */ ) == DUK_EXEC_ERROR )
        {
            /* -> [this err] */

            LOGE( "Failed to call onload callback (%s)",
                  duk_safe_to_string( ctx, -1 ));
        }
        /* -> [this retval/err] */
    }
    else
    {
        LOGW( "'onerror' is not a valid function" );
    }
    /* -> [this onerror/retval/err] */

    duk_pop_2( ctx ); /* -> [] */

    return 0;
}

static duk_ret_t xhr_object_onload_callback( duk_context* ctx )
{
    duk_push_this( ctx ); /* -> [this] */

    duk_get_prop_string( ctx, -1, PROPERTY_ONLOAD ); /* -> [this onload] */

    if ( duk_is_function( ctx, -1 ))
    {
        duk_dup( ctx, -2 ); /* -> [this onload this] */
        LOGD( "Calling onload callback" );
        if ( duk_pcall_method( ctx, 0 /* nargs */ ) == DUK_EXEC_ERROR )
        {
            /* -> [this err] */

            LOGE( "Failed to call onload callback (%s)",
                  duk_safe_to_string( ctx, -1 ));
        }
        /* -> [this retval/err] */
    }
    else
    {
        LOGW( "'onload' is not a valid function" );
    }
    /* -> [this onload/retval/err] */

    duk_pop_2( ctx ); /* -> [] */

    return 0;
}

static void xhr_object_event_handler( http_client_t* client, http_event_t event, http_response_t* response )
{
    duk_context*            ctx;
    xhr_object_data_t*      data;
    wiced_duktape_state_t   state;
    void*                   this = client->user_data;

    /* FIXME: We really should not be calling a potentially blocking call here;
     *        fix might be to make a copy of the response until HTTP request is
     *        complete, and then call into Duktape from DUKTAPE_WORKER_THREAD
     */
    ctx = wiced_duktape_get_control( &state );
    if ( ctx == NULL )
    {
        LOGW( "Duktape heap destroyed- ignoring HTTP client event" );
        return;
    }

    duk_push_heapptr( ctx, client->user_data ); /* -> [this] */

    duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_DATA );
    /* -> [this data] */
    data = duk_get_buffer( ctx, -1, NULL );
    duk_pop( ctx ); /* -> [this] */

    switch ( event )
    {
        case HTTP_CONNECTED:
        {
            LOGD( "HTTP_CONNECTED" );
            break;
        }

        case HTTP_DISCONNECTED:
        {
            LOGD( "HTTP_DISCONNECTED" );
            break;
        }

        case HTTP_DATA_RECEIVED:
        {
            LOGD( "HTTP_DATA_RECEIVED" );

            if ( response->response_hdr != NULL )
            {
                http_status_line_t status_line;

#if LOG_DEBUG_ENABLE
                uint32_t i;
                LOGD( "===== Response header START =====" );
                for ( i = 0; i < response->response_hdr_length; i++ )
                {
                    printf( "%c", response->response_hdr[i] );
                }
                printf( "\n" );
                LOGD( "===== Response header END =======" );
#endif

                /* Get the status code */
                http_get_status_line( response->response_hdr,
                                      response->response_hdr_length,
                                      &status_line );
                LOGD( "Response status=%hu", status_line.code );

                /* Update the 'status' property with the status code */
                duk_push_string( ctx, PROPERTY_STATUS );
                /* -> [this propName] */
                duk_push_int( ctx, status_line.code );
                /* -> [this propName status] */
                duk_def_prop( ctx, -3,
                              DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_FORCE );
                /* -> [this] */
            }

            /* -> [this] */

            duk_get_prop_string( ctx, -1, PROPERTY_RESPONSETEXT );
            /* -> [this responseText] */

            duk_push_string( ctx, PROPERTY_RESPONSETEXT );
            /* -> [this responseText propName] */
            if ( duk_is_null( ctx, -2 ))
            {
                duk_push_sprintf( ctx, "%.*s", response->payload_data_length,
                                  (char*)response->payload );
            }
            else
            {
                duk_push_sprintf( ctx, "%s%.*s", duk_get_string( ctx, -2 ),
                                  response->payload_data_length,
                                  (char*)response->payload );
            }
            /* -> [this OLDresponseText propName responseText] */

            duk_def_prop( ctx, -4,
                          DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_FORCE );
            /* -> [this OLDresponseText] */
            duk_pop( ctx ); /* -> [this] */

            if ( response->remaining_length == 0 )
            {
                LOGD( "Received complete response" );

#if LOG_DEBUG_ENABLE
                duk_get_prop_string( ctx, -1, PROPERTY_RESPONSETEXT );
                /* -> [this responseText] */
                LOGD( "===== Response payload START =====" );
                printf( "%s", duk_get_string( ctx, -1 ));
                LOGD( "===== Response payload END =======" );
                duk_pop( ctx ); /* -> [this] */
#endif

                /* Deinit the request and disconnect from the client */
                http_request_deinit( &data->request );
                http_client_disconnect( &data->client );

                data->state = CLIENT_STATE_REQUEST_RECEIVED;

                duk_get_prop_string( ctx, -1, PROPERTY_ONLOAD );
                /* -> [this onload] */
                if ( duk_is_function( ctx, -1 ))
                {
                    LOGD( "Scheduling onload callback" );
                    if ( wiced_duktape_schedule_work( this,
                                                      xhr_object_onload_callback,
                                                      NULL ) != WICED_SUCCESS )
                    {
                        LOGE( "Failed to schedule onload callback" );
                    }
                }

                duk_pop( ctx ); /* -> [this] */
            }
            break;
        }
        default:
            LOGW( "Ignoring unknown HTTP client event '%u'", event );
            break;
    }

    duk_pop( ctx ); /* -> [] */

    wiced_duktape_put_control( &state );

    LOGD( "End of HTTP client event handler" );
}

static duk_ret_t xhr_object_finalizer( duk_context* ctx )
{
    xhr_object_data_t*  data;

    /* Arguments:
     * 0 = 'this'
     * 1 = heap destruction flag    (boolean)
     */

    if ( !duk_get_boolean( ctx, 1 ))
    {
        /* Free resources only when heap is destroyed */
        return 0;
    }

    LOGD( "Finalizer called ('this'=%p)", duk_get_heapptr( ctx, 0 ));

    duk_dup( ctx, 0 ); /* -> [this] */

    if ( duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_DATA ))
    {
        /* -> [this data] */

        data = duk_get_buffer( ctx, -1, NULL );

        switch ( data->state )
        {
            case CLIENT_STATE_REQUEST_SENT:
                http_request_deinit( &data->request );
            case CLIENT_STATE_CONNECTED:
                http_client_disconnect( &data->client );
            case CLIENT_STATE_REQUEST_RECEIVED:
            case CLIENT_STATE_INITIALIZED:
                break;
        }

        http_client_deinit( &data->client );
    }
    else
    {
        /* -> [this undefined] */

        duk_pop( ctx ); /* -> [this] */
        LOGD( "No data defined- why did we get called?" );
    }

    duk_pop( ctx ); /* -> [] */

    return 0;
}

static http_method_t xhr_object_method_str_to_enum( const char* str )
{
    if ( strcmp( str, HTTP_METHOD_GET ) == 0 )
    {
        return HTTP_GET;
    }
    else if ( strcmp( str, HTTP_METHOD_POST ) == 0 )
    {
        return HTTP_POST;
    }
    else if ( strcmp( str, HTTP_METHOD_PUT ) == 0 )
    {
        return HTTP_PUT;
    }
    else if ( strcmp( str, HTTP_METHOD_DELETE ) == 0 )
    {
        return HTTP_DELETE;
    }

    return HTTP_UNKNOWN;
}

static duk_ret_t xhr_object_open( duk_context* ctx )
{
    xhr_object_data_t*  data;
    wiced_ip_address_t  ip_addr;
    const char*         url;
    duk_size_t          len;
    http_method_t       method;
    uint16_t            port;
    const char*         slash;
    const char*         colon;
    const char*         hostname;
    const char*         path;
    http_security_t     security;
    http_header_field_t header;

    /* Arguments:
     * 0 = HTTP method  (required, string)
     * 1 = url          (required, string)
     * 2 = async        (optional, bool)
     * 3 = user         (optional, string, ignored)
     * 4 = password     (optional, string, ignored)
     */

    /* Notes:
     *  - The DOM XMLHttpRequest object doesn't make the connection until the
     *    send() function, but it is easier to do the connect here as we can
     *    initialize the HTTP request header and just append to it for each
     *    setRequestHeader() call, instead of saving that information somewhere
     *    until the send() call
     *  - Since this call is supposed to just return, keep track of the errors
     *    internally until the user calls the send() function
     */

    duk_require_string( ctx, 0 );
    url = duk_require_string( ctx, 1 );

    LOGD( "Creating new XMLHttpRequest METHOD='%s' URL='%s'",
          duk_get_string( ctx, 0 ), url );

    duk_push_this( ctx ); /* -> [this] */

    /*
     * Get the 'data' internal property
     */

    if ( duk_has_prop_string( ctx, -1, INTERNAL_PROPERTY_DATA ))
    {
        duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_DATA );
        /* -> [this data] */
        data = duk_get_buffer( ctx, -1, NULL );
        duk_pop( ctx ); /* -> [this] */

        LOGD( "Cleaning up HTTP client" );

        switch ( data->state )
        {
            case CLIENT_STATE_REQUEST_SENT:
                http_request_deinit( &data->request );
            case CLIENT_STATE_CONNECTED:
                http_client_disconnect( &data->client );
            case CLIENT_STATE_REQUEST_RECEIVED:
            case CLIENT_STATE_INITIALIZED:
                break;
        }

        data->state = CLIENT_STATE_INITIALIZED;
        data->has_error = WICED_FALSE;
        data->has_accept_header = WICED_FALSE;
    }
    else
    {
        /* Create a buffer for the 'data' property */
        data = duk_push_fixed_buffer( ctx,
                                      sizeof( xhr_object_data_t ));
        /* -> [this data] */

        LOGD( "Initializing HTTP client" );
        if ( http_client_init( &data->client, WICED_STA_INTERFACE,
                               xhr_object_event_handler,
                               NULL ) != WICED_SUCCESS )
        {
            duk_pop_2( ctx ); /* -> [] */

            LOGE( "Failed to initialize HTTP client" );
            return 0;
        }

        /* Save 'data' to 'this' */
        duk_put_prop_string( ctx, -2, INTERNAL_PROPERTY_DATA );
        /* -> [this] */

        /* Set the finalizer for 'this' to do cleanup */
        duk_push_c_function( ctx, xhr_object_finalizer, 2 );
        /* -> [this finalizer] */
        duk_set_finalizer( ctx, -2 ); /* -> [this] */
    }

    /* Only support explicity HTTP(S) URLs (for now) */
    if (( strncasecmp( url, HTTP_PREFIX, strlen( HTTP_PREFIX )) != 0 ) &&
        ( strncasecmp( url, HTTPS_PREFIX, strlen( HTTPS_PREFIX )) != 0 ))
    {
        data->has_error = WICED_TRUE;
        duk_pop( ctx ); /* -> [] */

        LOGE( "Unsupported URL '%s'", url );
        return 0;
    }

    /* Check if it's a supported METHOD */
    method = xhr_object_method_str_to_enum( duk_get_string( ctx, 0 ));
    if ( method == HTTP_UNKNOWN )
    {
        data->has_error = WICED_TRUE;
        duk_pop( ctx ); /* -> [] */

        LOGE( "Invalid/unsupported METHOD '%s'", duk_get_string( ctx, 0 ) );
        return 0;
    }

    if ( duk_is_boolean( ctx, 2 ) && !duk_get_boolean( ctx, 2 ))
    {
        data->async = WICED_FALSE;
    }
    else
    {
        data->async = WICED_TRUE;
    }

    data->method = method;

    /* -> [this] */

    /*
     * Reset 'status' and 'responseText' properties to default values
     */

    duk_push_string( ctx, PROPERTY_STATUS ); /* -> [this propName] */
    duk_push_int( ctx, 0 ); /* -> [this propName status] */
    duk_def_prop( ctx, -3, DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_FORCE );
    /* -> [this] */

    duk_push_string( ctx, PROPERTY_RESPONSETEXT ); /* -> [this propName] */
    duk_push_null( ctx ); /* -> [this propName responseText] */
    duk_def_prop( ctx, -3, DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_FORCE );
    /* -> [this] */

    /*
     * Parse the URL and extract out the hostname, port, and path
     */

    duk_dup( ctx, 1 ); /* -> [this url] */
    url = duk_get_lstring( ctx, -1, &len );

    /* Remove the HTTP(S) prefix from the URL and set default port value */
    if ( strncasecmp( url, HTTPS_PREFIX, strlen( HTTPS_PREFIX )) == 0 )
    {
        duk_substring( ctx, -1, strlen( HTTPS_PREFIX ), len );
        /* -> [this url] */

        security = HTTP_USE_TLS;
        port = 443;
    }
    else
    {
        duk_substring( ctx, -1, strlen( HTTP_PREFIX ), len );
        /* -> [this url] */

        security = HTTP_NO_SECURITY;
        port = 80;
    }

    /* Make a copy of the URL: one will be for hostname, the other for path */
    duk_dup( ctx, -1 ); /* -> [this hostname path] */
    hostname = duk_get_string( ctx, -2 );
    path = duk_get_lstring( ctx, -1, &len );

    /* Look for the '/' and ':' characters */
    slash = strchr( hostname, '/' );
    colon = strchr( hostname, ':' );

    if ( colon != NULL )
    {
        if (( slash == NULL ) || (( slash != NULL ) && ( slash > colon )))
        {
            sscanf( hostname, "%*[^:]:%hu", &port );

            /* Trim off everything from the colon onwards to get the hostname */
            duk_substring( ctx, -2, 0, colon - hostname );
            /* -> [this hostname path] */
        }
        else
        {
            /* Trim off everything starting from '/' */
            duk_substring( ctx, -2, 0, slash - hostname );
            /* -> [this hostname path] */
        }
    }
    else if ( slash != NULL )
    {
        /* Trim off everything starting from '/' */
        duk_substring( ctx, -2, 0, slash - hostname );
        /* -> [this hostname path] */
    }
    hostname = duk_get_string( ctx, -2 );

    /* Look for the '/' character */
    slash = strchr( path, '/' );
    if ( slash != NULL )
    {
        duk_substring( ctx, -1, slash - path, len );
        /* -> [this hostname path] */
    }
    else
    {
        duk_pop( ctx ); /* -> [this hostname] */
        duk_push_sprintf( ctx, "/" ); /* -> [this hostname path] */
    }
    path = duk_get_string( ctx, -1 );

    LOGD( "hostname='%s' port='%hu' path='%s'", hostname, port, path );

    LOGD( "Resolving IP address of '%s'", hostname );
    if ( wiced_hostname_lookup( hostname, &ip_addr,
                                DNS_TIMEOUT_MS,
                                WICED_STA_INTERFACE ) != WICED_SUCCESS )
    {
        data->has_error = WICED_TRUE;
        duk_pop_3( ctx ); /* -> [] */

        LOGE( "Failed to resolve IP address of '%s'", hostname );
        return 0;
    }

    LOGD( "Resolved '%s' to %u.%u.%u.%u", hostname,
          (uint8_t)( GET_IPV4_ADDRESS(ip_addr) >> 24 ),
          (uint8_t)( GET_IPV4_ADDRESS(ip_addr) >> 16 ),
          (uint8_t)( GET_IPV4_ADDRESS(ip_addr) >> 8 ),
          (uint8_t)( GET_IPV4_ADDRESS(ip_addr) >> 0 ));

    /* Configure HTTP client parameters */
    data->config.flag = HTTP_CLIENT_CONFIG_FLAG_SERVER_NAME |
                        HTTP_CLIENT_CONFIG_FLAG_MAX_FRAGMENT_LEN;
    data->config.server_name = (uint8_t*)hostname;
    data->config.max_fragment_length = TLS_FRAGMENT_LENGTH_1024;
    http_client_configure( &data->client, &data->config );

    data->client.peer_cn = NULL;

    /* Save 'this' to client's user data to use later in the event handler */
    data->client.user_data = duk_get_heapptr( ctx, -3 );

    LOGD( "Connect to '%s' (port='%hu' security='%u')", hostname, port,
          security );
    if ( http_client_connect( &data->client, &ip_addr, port, security,
                              CONNECT_TIMEOUT_MS ) != WICED_SUCCESS )
    {
        data->has_error = WICED_TRUE;
        duk_pop_3( ctx ); /* -> [] */

        LOGE( "Failed to connect to server '%s'", hostname );
        return 0;
    }

    LOGD( "HTTP client connected" );
    data->state = CLIENT_STATE_CONNECTED;

    /* Fill out the initial header with server information */
    header.field        = HTTP_HEADER_HOST;
    header.field_length = strlen( HTTP_HEADER_HOST );
    header.value        = (char*)hostname;
    header.value_length = strlen( hostname );

    /* Initialize and write the first header of request */
    http_request_init( &data->request, &data->client, method, path, HTTP_1_1 );
    http_request_write_header( &data->request, &header, 1 );

    duk_pop_3( ctx ); /* -> [] */

    return 0;
}

static duk_ret_t xhr_object_send( duk_context* ctx )
{
    xhr_object_data_t*   data = NULL;
    http_header_field_t  header;
    const char*          payload = NULL;
    duk_size_t           payload_len = 0;

    /* Arguments:
     * 0 = data (optional, string)
     */

    LOGD( "%s: start (top=%d)", __func__, duk_get_top( ctx ));

    duk_push_this( ctx ); /* -> [this] */

    if ( duk_has_prop_string( ctx, -1, INTERNAL_PROPERTY_DATA ))
    {
        duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_DATA );
        /* -> [this data] */
        data = duk_get_buffer( ctx, -1, NULL);
        duk_pop( ctx ); /* -> [this] */
    }

    if (( data == NULL ) || ( data->has_error ) ||
        ( data->state != CLIENT_STATE_CONNECTED ))
    {
        LOGE( "Encountered previous errors- calling onerror callback" );

        duk_get_prop_string( ctx, -1, PROPERTY_ONERROR );
        /* -> [this onerror] */
        if ( duk_is_function( ctx, -1 ))
        {
            LOGD( "Scheduling onerror callback" );
            if ( wiced_duktape_schedule_work( NULL,
                                              xhr_object_onerror_callback,
                                              NULL ) != WICED_SUCCESS )
            {
                LOGE( "Failed to schedule onload callback" );
            }
        }

        duk_pop_2( ctx ); /* -> [] */
        return 0;
    }

    if ( !data->has_accept_header )
    {
        /* Add an Accept header */
        header.field        = HTTP_HEADER_ACCEPT;
        header.field_length = strlen( HTTP_HEADER_ACCEPT );
        header.value        = HTTP_ALL;
        header.value_length = strlen( HTTP_ALL );

        /* Write header to request */
        http_request_write_header( &data->request, &header, 1 );
    }

    /* Payload only valid for HTTP PUT/POST requests */
    if ( duk_is_string( ctx, 0 ) &&
        (( data->method == HTTP_POST ) || ( data->method == HTTP_PUT )))
    {
        const char* value;

        payload = duk_get_lstring( ctx, 0, &payload_len );

        value = duk_push_sprintf( ctx, "%u", payload_len );
        /* -> [this value] */

        /* Add a Content-Length header */
        header.field        = HTTP_HEADER_CONTENT_LENGTH;
        header.field_length = strlen( HTTP_HEADER_CONTENT_LENGTH );
        header.value        = (char*)value;
        header.value_length = strlen( value );

        /* Write header to request */
        http_request_write_header( &data->request, &header, 1 );

        duk_pop( ctx ); /* -> [this] */
    }

    /* Complete the header */
    http_request_write_end_header( &data->request );

    if ( payload_len > 0 )
    {
        LOGD( "Writing %u bytes to HTTP message-body", payload_len );
        http_request_write( &data->request, (uint8_t*)payload, payload_len );
    }

    LOGD( "Sending HTTP request" );
    http_request_flush( &data->request );
    data->state = CLIENT_STATE_REQUEST_SENT;

    /* FIXME If not async, wait here until we are done */

    duk_pop( ctx ); /* -> [] */

    LOGD( "%s: end (top=%d)", __func__, duk_get_top( ctx ));
    return 0;
}

static duk_ret_t xhr_object_set_request_header( duk_context* ctx )
{
    xhr_object_data_t*   data;
    const char*          field;
    const char*          value;
    http_header_field_t  header;

    /* Arguments:
     * 0 = header field (required, string)
     * 1 = header value (required, string)
     */

    duk_require_string( ctx, 0 );
    duk_require_string( ctx, 1 );

    LOGD( "Setting XMLHttpRequest HEADER='%s' BODY='%s'",
          duk_get_string( ctx, 0 ),duk_get_string( ctx, 1 ));

    duk_push_this( ctx ); /* -> [this] */

    if ( !duk_has_prop_string( ctx, -1, INTERNAL_PROPERTY_DATA ))
    {
        duk_pop( ctx ); /* -> [] */

        LOGE( "Cannot find internal 'data' property- invalid state!" );
        return 0;
    }

    duk_get_prop_string( ctx, -1, INTERNAL_PROPERTY_DATA ); /* -> [this data] */
    data = duk_get_buffer( ctx, -1, NULL );
    duk_pop( ctx ); /* -> [this] */

    if ( data->state != CLIENT_STATE_CONNECTED )
    {
        duk_pop( ctx ); /* -> [] */

        LOGW( "Trying to set HTTP header in invalid HTTP client state" );
        return 0;
    }

    /* We need to do a bit of formatting with the header field first */
    field = duk_push_sprintf( ctx, "%s: ", duk_get_string( ctx, 0 ));
    /* -> [this field] */
    value = duk_get_string( ctx, 1 );

    /* Fill out the header */
    header.field        = (char*)field;
    header.field_length = strlen( field );
    header.value        = (char*)value;
    header.value_length = strlen( value );

    /* Write header to request */
    http_request_write_header( &data->request, &header, 1 );

    /* Check if header is Accept so we don't need to add it later */
    if ( strcmp( HTTP_HEADER_ACCEPT, field ) == 0 )
    {
        data->has_accept_header = WICED_TRUE;
    }

    duk_pop_2( ctx ); /* -> [] */

    return 0;
}

static const duk_function_list_entry xhr_object_funcs[] =
{
      /* Name */            /* Function */                  /* nargs */
    { "open",               xhr_object_open,                5 },
    { "send",               xhr_object_send,                1 },
    { "setRequestHeader",   xhr_object_set_request_header,  2 },
    { NULL, NULL, 0 }
};

static duk_ret_t xhr_object_constructor( duk_context* ctx )
{
    if ( !duk_is_constructor_call( ctx ))
    {
        LOGD( "Called as a normal function (non-constructor)" );

        /* Reject non-constructor call */
        return DUK_RET_TYPE_ERROR;
    }

    duk_push_this( ctx ); /* -> [this] */
    LOGD( "Constructor called ('this'=%p)", duk_get_heapptr( ctx, -1 ));

    /* Initialize object properties to default values
     * Notes:
     *  - Only a subset of possible properties are implemented at this time
     *  - Properties that trigger immediate actions upon modifications, e.g.
     *    currentTime to jump to position in time, should be implemented as
     *    accessor (getter/setter) properties
     *  - All other properties should be plain properties
     *  - RO accessor properties should use a setter that returns a type error
     *  - RO plain properties should set the property to be non-writable, and
     *    use force the property value update internally when needed
     */

    /* [RW] onerror: JS function to invoke when operation fails */
    duk_push_string( ctx, PROPERTY_ONERROR ); /* -> [this propName] */
    duk_push_undefined( ctx ); /* -> [this propName onerror] */
    duk_def_prop( ctx, -3,
                  DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_SET_ENUMERABLE |
                  DUK_DEFPROP_SET_WRITABLE ); /* -> [this] */

    /* [RW] onload: JS function to invoke when operation is successfully
     *              completed
     */
    duk_push_string( ctx, PROPERTY_ONLOAD ); /* -> [this propName] */
    duk_push_undefined( ctx ); /* -> [this propName onload] */
    duk_def_prop( ctx, -3,
                  DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_SET_ENUMERABLE |
                  DUK_DEFPROP_SET_WRITABLE ); /* -> [this] */

    /* [RO] responseText: request response text */
    duk_push_string( ctx, PROPERTY_RESPONSETEXT ); /* -> [this propName] */
    duk_push_null( ctx ); /* -> [this propName responseText] */
    duk_def_prop( ctx, -3,
                  DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_CLEAR_WRITABLE |
                  DUK_DEFPROP_SET_ENUMERABLE ); /* -> [this] */

    /* [RO] status: status of response of request */
    duk_push_string( ctx, PROPERTY_STATUS ); /* -> [this propName] */
    duk_push_int( ctx, 0 ); /* -> [this propName status] */
    duk_def_prop( ctx, -3,
                  DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_CLEAR_WRITABLE |
                  DUK_DEFPROP_SET_ENUMERABLE ); /* -> [this] */

    duk_pop( ctx ); /* -> [] */

    return 0;
}

void wiced_duktape_object_xmlhttprequest_init( duk_context* ctx )
{
    duk_push_c_function( ctx, xhr_object_constructor, 0 ); /* -> [func] */

    duk_push_object( ctx ); /* -> [func obj] */
    duk_put_function_list( ctx, -1, xhr_object_funcs );

    duk_put_prop_string( ctx, -2, "prototype" ); /* -> [func] */

    duk_put_global_string( ctx, "XMLHttpRequest" ); /* -> [] */
}
