/*
 * $ Copyright Cypress Semiconductor  $
 */

#include "http_client.h"
#include "xhr.h"
#include "wiced.h"
#include "wiced_duktape.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define POST_XHR_MESSAGE(id, handle, data) \
  do \
  { \
    wiced_duktape_callback_queue_element_t message; \
    message.module_id = WDCM_XMLHTTPREQUEST; \
    message.event_id = (uint16_t) id; \
    message.module_handle = handle; \
    message.event_data1 = NULL; \
    message.event_data2 = data; \
    wiced_duktape_callback_post_event(&message); \
  } while(0)

/******************************************************
 *                    Constants
 ******************************************************/

#define LOG_LABEL           "duk:xmlhttp"
#define LOG_DEBUG_ENABLE    0

#define DNS_TIMEOUT_MS              (10000)
#define CONNECT_TIMEOUT_MS          (10000)

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
static void http_event_handler( http_client_t* client, http_event_t event, http_response_t* response );
static void post_event_statechange(dpm_xhr_handle_t *handle, dpm_xhr_readystate_t state);
static void post_error(dpm_xhr_handle_t *handle, int errno);

#if LOG_DEBUG_ENABLE
static void print_data( char* data, uint32_t length );
#if 0
static void print_content( char* data, uint32_t length );
#endif
static void print_header( http_response_t* response );
#endif

static int digit(char c);
static int atoi_n(const char *restrict string, int n, int len, int *value_return);
static int parse_url(const char *url, int len, char *hostname, int hsize, char *path, int psize, int *port, int *security);

static linked_list_t* wiced_dpm_xhr_list_init(void);
static void wiced_dpm_xhr_list_deinit(void);
static void wiced_dpm_xhr_node_delete_all(void);
static void wiced_dpm_xhr_node_insert(dpm_xhr_handle_t *handle);
static void wiced_dpm_xhr_node_remove(dpm_xhr_handle_t *handle);

/******************************************************
 *               Variable Definitions
 ******************************************************/
static linked_list_t wiced_dpm_xhr_list, *wiced_dpm_xhr_list_ptr;

/******************************************************
 *               Function Definitions
 ******************************************************/
static linked_list_t* wiced_dpm_xhr_list_init(void)
{
    if (wiced_dpm_xhr_list_ptr == NULL)
    {
        wiced_dpm_xhr_list_ptr = &wiced_dpm_xhr_list;
        linked_list_init( wiced_dpm_xhr_list_ptr );
    }
    return wiced_dpm_xhr_list_ptr;
}

static void wiced_dpm_xhr_list_deinit(void)
{
    if (wiced_dpm_xhr_list_ptr != NULL)
    {
        linked_list_deinit( wiced_dpm_xhr_list_ptr );
        wiced_dpm_xhr_list_ptr = NULL;
    }
}

static void wiced_dpm_xhr_node_insert(dpm_xhr_handle_t *handle)
{
    linked_list_t* list_ptr = wiced_dpm_xhr_list_init();
    linked_list_set_node_data(&handle->node, handle);
    linked_list_insert_node_at_rear( list_ptr, &handle->node );
    return;
}

static void wiced_dpm_xhr_node_remove(dpm_xhr_handle_t *handle)
{
    linked_list_t* list_ptr = wiced_dpm_xhr_list_ptr;
    if (list_ptr == NULL) {
        LOGE("DPM XHR list is not initialized!");
        return;
    }
    linked_list_remove_node( list_ptr, &handle->node );
    return;
}

static void wiced_dpm_xhr_node_delete_all(void)
{
    linked_list_t* list_ptr = wiced_dpm_xhr_list_ptr;
    linked_list_node_t *node = NULL;
    dpm_xhr_handle_t *handle;
    int i = 0;
    uint32_t j = 0;

    if (list_ptr == NULL) {
        LOGE("DPM XHR list is not initialized!");
        return;
    }

    linked_list_get_front_node( list_ptr, &node );
    while (node != NULL) {
        handle = (dpm_xhr_handle_t *) node->data;

        http_request_deinit( &handle->request );
        http_client_disconnect( &handle->client );
        http_client_deinit( &handle->client);
        handle->data = duv_cleanup_handle(handle->ctx, handle->data);
        i++;
        node = node->next;
    }
    linked_list_get_count(list_ptr, &j);
    LOGI("DPM XHR: deleted(%d-%u)", i, j);
    wiced_dpm_xhr_list_deinit();
    return;
}

static duk_bool_t xhr_payload_type(duk_context* ctx, duk_idx_t index) {
  return  duk_is_string(ctx, index) ||
          duk_is_undefined(ctx, index) ||
          duk_is_object(ctx, index);
}

static duk_bool_t xhr_is_http_mode(duk_context* ctx, duk_idx_t index) {
  return  duk_is_boolean(ctx, index) ||
          duk_is_undefined(ctx, index);
}

/*  Support GET and POST only*/
static http_method_t method_str_to_enum( const char* str )
{
    if ( strcmp( str, HTTP_METHOD_GET ) == 0 )
    {
        return HTTP_GET;
    }
    else if ( strcmp( str, HTTP_METHOD_POST ) == 0 )
    {
        return HTTP_POST;
    }
    return HTTP_UNKNOWN;
}

static void post_error(dpm_xhr_handle_t *handle, int errno)
{
    handle->error = errno;
    POST_XHR_MESSAGE(LWS_XHR_CALLBACK_EVENT_ONERROR, handle, errno);
}

static void post_event_statechange(dpm_xhr_handle_t *handle, dpm_xhr_readystate_t state)
{
    handle->readyState = state;
    // POST_XHR_MESSAGE(LWS_XHR_CALLBACK_EVENT_ONREADYSTATECHANGE, handle, state);
}

#if LOG_DEBUG_ENABLE
static void print_data( char* data, uint32_t length )
{
    uint32_t i;
    for ( i = 0; i < length; i++ )
    {
        printf( "%c", data[i] );
    }
}

#if 0
static void print_content( char* data, uint32_t length )
{
    LOGD("===== Response Content START =====");
    print_data( (char*)data, length );
    LOGD("\n===== Response Content END   =====\n");
}
#endif
static void print_header( http_response_t* response )
{
    LOGD("===== Response header START =====");
    print_data( (char*)response->response_hdr, response->response_hdr_length );
    LOGD("\n===== Response header END   =======");
}
#endif

void xmlhttprequest_callback_handler(wiced_duktape_callback_queue_element_t *message)
{
    dpm_xhr_callback_event_id event_id = (dpm_xhr_callback_event_id) message->event_id;
    dpm_xhr_handle_t *handle = (dpm_xhr_handle_t *)message->module_handle;
    LOGD("XMLHTTPREQUET: callback_handler: handle=%p, event=%d", handle, event_id);

    switch ( event_id )
    {
        case LWS_XHR_CALLBACK_EVENT_ONLOAD:
        {
            duk_context *ctx = handle->ctx;
            void *msg = handle->body;
            size_t len = handle->body_length;
            LOGD("XHR ONLOAD: handle=%p  duv_handle=%p, body:%lu", handle, handle->data, len);

            // push the http status
            duk_push_int(ctx, handle->http_status);

            // push the http body
            if (msg != NULL && len > 0)
            {
                char *out = duk_push_fixed_buffer(ctx, len);
                if (out == NULL) {
                    LOGE("XHR ONLOAD: Failed to allocate buffer");
                    duk_push_undefined(ctx);
                } else {
                    memcpy(out, msg, len);
                    duk_buffer_to_string(ctx, -1);
                }

                // free the buffer
                free(msg);
                handle->body = NULL;
                handle->body_length = 0;
            } else {
                duk_push_undefined(ctx);
                LOGE("Invalid message: msg=%p, len=%d", msg, len);
            }
            duv_emit_event(ctx, handle->data, LWS_XHR_LOAD, 2);
            break;
        }
        case LWS_XHR_CALLBACK_EVENT_ONERROR:
        {
            duk_context *ctx = handle->ctx;
            int error = handle->error;
            LOGD("XHR ERROR: handle=%p duv_handle=%p errno=%d", handle, handle->data, error);

            // push the error number
            duk_push_int(ctx, error);
            duv_emit_event(ctx, handle->data, LWS_XHR_ERROR, 1);
            break;
        }
        case LWS_XHR_CALLBACK_EVENT_ONREADYSTATECHANGE:
        {
            LOGD("XHR READYSTATE: handle=%p state=%d", handle, message->event_data2);
            // handle->readyState = message->event_data2;
            break;
        }
        case LWS_XHR_CALLBACK_EVENT_HTTPSTATUS:
        {
            LOGD("XHR HTTPSTATUS: handle=%p status=%d", handle, message->event_data2);
            handle->http_status = message->event_data2;
            break;
        }
        case LWS_XHR_CALLBACK_EVENT_DISCONNECT:
        {
            handle->data = duv_cleanup_handle(handle->ctx, handle->data);
            http_client_deinit( &handle->client);
            LOGD("XHR DISCONNECT: handle=%p data=%d", handle, handle->data);
            // clean up
            if (handle->body) {
                LOGW("free the left-over buffer body=%p", handle->body);
                free(handle->body);
                handle->body = NULL;
                handle->body_length = 0;
            }
            wiced_dpm_xhr_node_remove(handle);
            break;
        }
        default:
            break;
    }
    return;
}

static void http_event_handler( http_client_t* client, http_event_t event, http_response_t* response )
{
    dpm_xhr_handle_t * handle = (dpm_xhr_handle_t *)client->user_data;
    LOGD("HTTP event: %hu handle=%p, client=%p, response=%p", event, handle, client, response);

    switch ( event )
    {
        case HTTP_CONNECTED:
        {
            LOGD( "HTTP_CONNECTED" );
            post_event_statechange(handle, LWS_XHR_READYSTATE_SENT);
            break;
        }

        case HTTP_DISCONNECTED:
        {
            LOGD( "HTTP_DISCONNECTED" );
            post_event_statechange(handle, LWS_XHR_READYSTATE_UNINIT);
            POST_XHR_MESSAGE(LWS_XHR_CALLBACK_EVENT_DISCONNECT, handle, 0);
            break;
        }

        case HTTP_DATA_RECEIVED:
        {
            LOGD( "HTTP_DATA_RECEIVED" );

            if ( response->response_hdr != NULL )
            {
                http_status_line_t status_line;

#if LOG_DEBUG_ENABLE
                print_header( response );
#endif
                /* Get the status code */
                http_get_status_line( response->response_hdr,
                                      response->response_hdr_length,
                                      &status_line );
                LOGD( "Response status=%hu", status_line.code );

                // update HTTP status
                POST_XHR_MESSAGE(LWS_XHR_CALLBACK_EVENT_HTTPSTATUS, handle, status_line.code);

                // update readyState
                post_event_statechange(handle, LWS_XHR_READYSTATE_RECEIVING);
            }

            LOGD( "Response payload=(%p-%hu)", response->payload, response->payload_data_length);
            if (response->payload != NULL && response->payload_data_length > 0) {
                uint8_t * mem;
                mem = realloc(handle->body, handle->body_length + response->payload_data_length + 2);
                if (mem != NULL) {
                    memcpy(&mem[handle->body_length], response->payload, response->payload_data_length);
                    handle->body_length += response->payload_data_length;
                    handle->body = mem;
                    LOGD( "Data received(%p-%lu)", handle->body, handle->body_length);
                } else {
                    LOGE("Failed to allocate memory to buffer the incoming data");
                    free(handle->body);
                    handle->body_length = 0;
                    post_error(handle, LWS_XHR_ERROR_NOMEM);
                }
            }

            if ( response->remaining_length == 0 )
            {
                LOGD( "Received complete response" );
                /* Deinit the request and disconnect from the client */
                http_request_deinit( &handle->request );
                http_client_disconnect(&handle->client );

                // update readyState
                post_event_statechange(handle, LWS_XHR_READYSTATE_LOADED);
                POST_XHR_MESSAGE(LWS_XHR_CALLBACK_EVENT_ONLOAD, handle, 0);
            }
            break;
        }
        default:
            LOGW( "Ignoring unknown HTTP client event '%u'", event );
            break;
    }
}

duk_ret_t dpm_new_xhr(duk_context *ctx) {
    dpm_xhr_handle_t *handle;
    dschema_check(ctx, (const duv_schema_entry[]) {
        {NULL}
    });

    handle = duk_push_fixed_buffer(ctx, sizeof(*handle));
    handle->ctx = ctx;
    handle->data = duv_setup_handle(ctx);
    handle->readyState = LWS_XHR_READYSTATE_UNINIT;
    handle->body = NULL;
    handle->body_length = 0;
    handle->error = LWS_XHR_ERROR_NONE;
    handle->relative_path = 0;
    LOGD("dpm_new_xhr, handle=%p", handle);

    wiced_dpm_xhr_node_insert(handle);

    return 1;
}

duk_ret_t dpm_xhr_open(duk_context *ctx) {
    dpm_xhr_handle_t* handle;
    wiced_ip_address_t  ip_addr;
    const char *method, *url;
    int async, port, security;
    size_t url_len;
    char hostname[128], path[256];
    http_header_field_t header;
    wiced_result_t result;

    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {"method", duk_is_string},
        {"url", duk_is_string},
        {"async", xhr_is_http_mode},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    LOGD("dpm_xhr_open handle=%p", handle);
    if (handle->readyState != LWS_XHR_READYSTATE_UNINIT &&
        handle->readyState != LWS_XHR_READYSTATE_LOADED)
    {
        LOGW("Current state: %d - Restart the connection!!!", handle->readyState);
        // Deinit the request and disconnect from the client
        http_request_deinit( &handle->request );
        http_client_disconnect( &handle->client );

        // clean up error and free buffer if necessary
        handle->error = LWS_XHR_ERROR_NONE;
        if (handle->body != NULL) {
            LOGW("Free the buffer: %p-%lu", handle->body, handle->body_length);
            free(handle->body);
            handle->body = NULL;
            handle->body_length = 0;
        }
    }

    // get and check the method
    method = duk_get_string(ctx, 1);
    handle->method = method_str_to_enum(method);
    if (handle->method == HTTP_UNKNOWN ) {
        LOGE("Invalid/unsupported METHOD '%s'", method);
        post_error(handle, LWS_XHR_ERROR_INVALID_METHOD);
        return 0;
    }

    // get and check the url
    url = duk_get_lstring(ctx, 2, &url_len);
    if ( 0 != parse_url(url, url_len, hostname, sizeof(hostname), path, sizeof(path), &port, &security))
    {
        if (strnicmp(url, "/", 1) == 0) {
            LOGI("Relative path - skip for now");
            handle->relative_path = 1;
            post_event_statechange(handle, LWS_XHR_READYSTATE_OPEN);
            return 0;
        } else {
            LOGE("Invalid URL: '%s'", url);
            post_error(handle, LWS_XHR_ERROR_INVALID_URL);
            return 0;
        }
    }
    handle->security = security == 0 ? HTTP_NO_SECURITY : HTTP_USE_TLS;

    // get and check async
    async = duk_get_boolean(ctx, 3);
    if (async == 0)
    {
        LOGE("support async mode only");
        post_error(handle, LWS_XHR_ERROR_SYNC_MODE);
        return 0;
    }

    LOGD("XMLHttpRequest");
    LOGD(" method  : %s", method);
    LOGD(" URL     : %s", url);
    LOGD("  => hostname: %s", hostname);
    LOGD("  => port    : %d", port);
    LOGD("  => scheme  : %s", security == 0 ? "http" : "https");
    LOGD("  => path    : %s", path);

    LOGD( "Initializing HTTP client" );
    result = http_client_init( &handle->client, WICED_STA_INTERFACE,
                               http_event_handler, NULL );
    if ( result != WICED_SUCCESS )
    {
        LOGE( "Failed to initialize HTTP client err=%d", result );
        post_error(handle, LWS_XHR_ERROR_INTERNAL);
        return 0;
    }
    /* Save handle to client's user data to use later in the event handler */
    handle->client.user_data = (void *) handle;

    LOGD( "Resolving IP address of '%s'", hostname );
    if ( wiced_hostname_lookup( hostname, &ip_addr,
                                DNS_TIMEOUT_MS,
                                WICED_STA_INTERFACE ) != WICED_SUCCESS )
    {
        LOGE( "Failed to resolve IP address of '%s'", hostname );
        post_error(handle, LWS_XHR_ERROR_HOSTLOOKUP);
        return 0;
    }

    LOGD( "Resolved '%s' to %u.%u.%u.%u", hostname,
          (uint8_t)( GET_IPV4_ADDRESS(ip_addr) >> 24 ),
          (uint8_t)( GET_IPV4_ADDRESS(ip_addr) >> 16 ),
          (uint8_t)( GET_IPV4_ADDRESS(ip_addr) >> 8 ),
          (uint8_t)( GET_IPV4_ADDRESS(ip_addr) >> 0 ));

    /* Configure HTTP client parameters */
    handle->config.flag = HTTP_CLIENT_CONFIG_FLAG_SERVER_NAME |
                          HTTP_CLIENT_CONFIG_FLAG_MAX_FRAGMENT_LEN;
    handle->config.server_name = (uint8_t*)hostname;
    handle->config.max_fragment_length = TLS_FRAGMENT_LENGTH_1024;
    http_client_configure( &handle->client, &handle->config );

    LOGD( "Connect to '%s' (port='%hu' security='%u')", hostname, port,
          security );
    if ( http_client_connect( &handle->client, &ip_addr, port, handle->security,
                              CONNECT_TIMEOUT_MS ) != WICED_SUCCESS )
    {
        LOGE( "Failed to connect to server '%s'", hostname );
        post_error(handle, LWS_XHR_ERROR_CONNECT);
        return 0;
    }

    /* Fill out the initial header with server information */
    header.field        = HTTP_HEADER_HOST;
    header.field_length = strlen( HTTP_HEADER_HOST ) - 1;
    header.value        = (char*)hostname;
    header.value_length = strlen( hostname );

    /* Initialize and write the first header of request */
    http_request_init( &handle->request, &handle->client, handle->method, path, HTTP_1_1 );
    http_request_write_header( &handle->request, &header, 1 );

    LOGD( "HTTP client connected" );
    post_event_statechange(handle, LWS_XHR_READYSTATE_OPEN);
    return 0;
}

duk_ret_t dpm_xhr_set_request_header(duk_context *ctx) {
    dpm_xhr_handle_t* handle;
    const char *field, *value;
    char *buf;
    size_t field_len, value_len;
    http_header_field_t header;

    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {"field", duk_is_string},
        {"value", duk_is_string},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    LOGD("dpm_xhr_set_request_header handle=%p readyState=%d", handle, handle->readyState);
    if (handle->readyState != LWS_XHR_READYSTATE_OPEN)
    {
        LOGW("Invalid state: %d", handle->readyState);
    }

    // check the connection error
    if (handle->error != LWS_XHR_ERROR_NONE) {
        LOGE("Connection error - %d", handle->error);
        post_error(handle, LWS_XHR_ERROR_CONNECT);
        return 0;
    }

    // get and check the header
    field = duk_get_lstring(ctx, 1, &field_len);
    field_len += 4;
    buf = malloc(field_len);
    if (buf == NULL) {
        post_error(handle, LWS_XHR_ERROR_NOMEM);
        return 0;
    }
    snprintf(buf, field_len, "%s: ", field);
    header.field = buf;
    header.field_length = strlen(buf);

    value = duk_get_lstring(ctx, 2, &value_len);
    header.value = (char*) value;
    header.value_length = value_len;

    /* Write header to request */
    LOGD("write header: '%s%s'", header.field, header.value);
    if (handle->relative_path == 0) {
        http_request_write_header( &handle->request, &header, 1 );
    }

    free(buf);
    return 0;
}

duk_ret_t dpm_xhr_send(duk_context *ctx) {
    dpm_xhr_handle_t* handle;
    http_header_field_t header;
    const char *payload;
    size_t payload_len;

    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {"payload", xhr_payload_type},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    LOGD("dpm_xhr_send handle=%p readyState=%d", handle, handle->readyState);

    payload = duk_get_lstring(ctx, 1, &payload_len);

    if (handle->relative_path == 1)
    {
        LOGD("Ignore relative path");
        post_event_statechange(handle, LWS_XHR_READYSTATE_SENT);
        handle->data = duv_cleanup_handle(handle->ctx, handle->data);
        wiced_dpm_xhr_node_remove(handle);
        return 0;
    }

    if (payload_len > 0 && handle->method == HTTP_GET)
    {
        LOGE("GET method with payload '%s'", payload);
        post_error(handle, LWS_XHR_ERROR_INVALID_PAYLOAD);
        return 0;
    }

    // check the connection error
    if (handle->error != LWS_XHR_ERROR_NONE) {
        LOGE("Connection error - %d", handle->error);
        post_error(handle, LWS_XHR_ERROR_CONNECT);
        return 0;
    }

    /* Add a Content-Length header if payload  */
    if (payload_len > 0) {
        char value[8];
        snprintf(value, 8, "%u", payload_len);
        header.field        = HTTP_HEADER_CONTENT_LENGTH;
        header.field_length = strlen( HTTP_HEADER_CONTENT_LENGTH ) - 1;
        header.value        = (char*)value;
        header.value_length = strlen( value );

        /* Write header to request */
        LOGD("write header: '%s%s'", header.field, header.value);
        http_request_write_header( &handle->request, &header, 1 );
    }

    /* Complete the header */
    http_request_write_end_header( &handle->request );

    /* Write payload */
    if ( payload_len > 0) {
        LOGD( "Writing %u bytes to HTTP message-body", payload_len );
        http_request_write( &handle->request, (uint8_t*)payload, payload_len );
    }

    LOGD( "Sending HTTP request" );
    http_request_flush( &handle->request );
    post_event_statechange(handle, LWS_XHR_READYSTATE_SENT);
    return 0;
}

duk_ret_t dpm_xhr_on_load(duk_context *ctx) {
    dpm_xhr_handle_t *handle;
    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {"onLoad", duk_is_function},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    LOGD("dpm_xhr_on_load: handle=%p, data=%p", handle, handle->data);
    duv_store_handler(ctx, handle->data, LWS_XHR_LOAD, 1);
    return 0;
}

duk_ret_t dpm_xhr_on_error(duk_context *ctx) {
    dpm_xhr_handle_t *handle;
    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {"onError", duk_is_function},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    LOGD("dpm_xhr_on_error: handle=%p, data=%p", handle, handle->data);
    duv_store_handler(ctx, handle->data, LWS_XHR_ERROR, 1);
    return 0;
}

duk_ret_t dpm_xhr_shutdown(duk_context *ctx) {

    dschema_check(ctx, (const duv_schema_entry[]) {
        {NULL}
    });

    LOGD("dpm_xhr_shutdown");
    wiced_dpm_xhr_node_delete_all();
    return 0;
}

// ============= utility function (not belongs to here ==============
static int digit(char c)
{
    if(c >= '0' && c <= '9')
        return 1;
    return 0;
}

static int atoi_n(const char *restrict string, int n, int len, int *value_return)
{
    int i = n;
    int val = 0;

    if(i >= len || !digit(string[i]))
        return -1;

    while(i < len && digit(string[i])) {
        val = val * 10 + (string[i] - '0');
        i++;
    }
    *value_return = val;
    return i;
}

static int parse_url(const char *url, int len, char *hostname, int hsize, char *path, int psize, int *port, int *security)
{
    int x=-1, y=-1, z, i = 0;
    int lport, lsec=0;

    if(len >= 7 && strnicmp(url, "http://", 7) == 0) {
        x = 7;
    } else if (len >= 8 && strnicmp(url, "https://", 8) == 0) {
        x = 8;
        lsec = 1;
    } else {
        return -1;
    }

    for(i = x; i < len; i++)
        if(url[i] == ':' || url[i] == '/')
            break;
    y = i;

    if(i < len && url[i] == ':') {
        int j;
        j = atoi_n(url, i + 1, len, &lport);
        if(j < 0) {
            lport = lsec == 0 ? 80 : 443;
        } else {
            i = j;
        }
    } else {
        lport = lsec == 0 ? 80 : 443;
    }
    z = i;

    if (y - x >= hsize || len - z > psize)
    {
        return -1;
    }

    memcpy(hostname, url + x, y - x);
    hostname[y -x] = '\0';
    memcpy(path, url + z, len - z);
    path[len - z] = '\0';

    *port = lport;
    *security = lsec;
    return 0;
}
