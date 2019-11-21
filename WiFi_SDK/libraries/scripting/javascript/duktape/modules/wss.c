/*
 * $ Copyright Cypress Semiconductor  $
 */

#include "wss.h"
#include "wiced_websocket.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define LOG_LABEL           "duk:wss"
#define LOG_DEBUG_ENABLE    0
#define MAX_WSS_CONNECTION  8

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
static duk_bool_t dschema_is_pointer(duk_context* ctx, duk_idx_t index);
static int add_wss_handle(wss_handle_t * handle);
static int delete_wss_handle(wss_handle_t * handle);
static wss_handle_t* get_wss_handle(wiced_wss_t * wsi);

/******************************************************
 *               Variable Definitions
 ******************************************************/
static wss_handle_t* g_wss_handles[MAX_WSS_CONNECTION];

/******************************************************
 *               Function Definitions
 ******************************************************/

static int add_wss_handle(wss_handle_t * handle)
{
    int i;
    for (i = 0; i < MAX_WSS_CONNECTION; i++) {
        if (g_wss_handles[i] == NULL) {
            g_wss_handles[i] = handle;
            LOGD("Save wss_handle(%p) at index(%d)", handle, i);
            return 0;
        }
    }

    LOGE("No more space to save wss handle(%p)", handle);
    return -1;
}

static int delete_wss_handle(wss_handle_t * handle)
{
    int i;
    for (i = 0; i < MAX_WSS_CONNECTION; i++) {
        if (g_wss_handles[i] == handle) {
            g_wss_handles[i] = NULL;
            LOGD("Remove wss_handle(%p) at index(%d)", handle, i);
            return 0;
        }
    }

    LOGE("Failed to remove handle(%p)", handle);
    return -1;
}

static wss_handle_t* get_wss_handle(wiced_wss_t * wsi)
{
    int i;
    for (i = 0; i < MAX_WSS_CONNECTION; i++) {
        wss_handle_t* handle = g_wss_handles[i];
        if (handle != NULL && handle->wsi == wsi) {
            LOGD("Find wss_handle(%p) at index(%d)", handle, i);
            return handle;
        }
    }

    LOGE("Can't find wss_handle for wsi(%p)", wsi);
    return NULL;
}

static duk_bool_t dschema_is_pointer(duk_context* ctx, duk_idx_t index) {
  return  duk_is_pointer(ctx, index) ||
          duk_is_undefined(ctx, index);
}

duk_ret_t dpm_new_wss(duk_context *ctx) {
    wss_handle_t *handle;
    wiced_wss_t *wsi;

    dschema_check(ctx, (const duv_schema_entry[]) {
        {"wsi", dschema_is_pointer},
        {NULL}
    });

    wsi = duk_get_pointer(ctx, 0);
    LOGD("new_wss: wsi=%p", wsi);

    handle = duk_push_fixed_buffer(ctx, sizeof(*handle));
    handle->wsi = wsi;
    handle->ctx = ctx;
    handle->data = duv_setup_handle(ctx);

    if (wsi != NULL) {
        add_wss_handle(handle);
    }

    return 1;
}

duk_ret_t dpm_wss_start(duk_context *ctx) {
    wss_handle_t *handle;
    int port;
    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {"port", duk_is_number},
        {"onConnection", duk_is_function},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    port = duk_get_number(ctx, 1);
    duv_store_handler(ctx, handle->data, LWS_CONNECTION, 2);
    LOGD("dpm_wss_start: handle=%p, port=%d", handle, port);
    start_wss(port, handle);
    return 0;
}

duk_ret_t dpm_wss_on_message(duk_context *ctx) {
    wss_handle_t *handle;
    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {"onMessage", duk_is_function},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    duv_store_handler(ctx, handle->data, LWS_ONMESSAGE, 1);
    LOGD("dpm_wss_on_message: handle=%p, data=%p", handle, handle->data);
    return 0;
}

duk_ret_t dpm_wss_send(duk_context *ctx) {
    wss_handle_t *handle;
    unsigned char *msg;
    size_t len;
    int n;
    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {"message", duk_is_string},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    msg = (unsigned char *) duk_get_lstring(ctx, 1, &len);

    n = wiced_wss_send_packets(handle->wsi, msg, len);

    LOGD("dpm_wss_send=%d", n);
    duk_push_int(ctx, n);
    return 1;
}

duk_ret_t dpm_wss_close(duk_context *ctx) {
    wss_handle_t *handle;
    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    wiced_websocket_close(handle->wsi, WEBSOCKET_CLOSE_STATUS_CODE_NORMAL, NULL);
    LOGD("dpm_wss_close wsi=%p", handle->wsi);
    return 0;
}

duk_ret_t dpm_wss_shutdown(duk_context *ctx) {
    dschema_check(ctx, (const duv_schema_entry[]) {
        {NULL}
    });

    LOGW("dpm_wss_shutdown");
    stop_wss();
    wiced_duktape_callback_loop_break();
    return 0;
}


duk_ret_t dpm_wss_on_close(duk_context *ctx) {
    wss_handle_t *handle;
    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {"onClose", duk_is_function},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    duv_store_handler(ctx, handle->data, LWS_CLOSED, 1);
    LOGD("dpm_wss_on_close handle=%p, data=%p", handle, handle->data);
    return 0;
}

void websocket_callback_handler(wiced_duktape_callback_queue_element_t *message)
{
    dpm_wss_callback_event_id event_id = (dpm_wss_callback_event_id) message->event_id;
    LOGD("websocket_callback_handler: event=%d", event_id);

    switch ( event_id )
    {
        case LWS_WSS_CALLBACK_EVENT_CONNECTION:
        {
            wss_handle_t * handle = (wss_handle_t *)message->module_handle;
            duk_context *ctx = handle->ctx;
            wiced_wss_t *wsi = (wiced_wss_t *) message->event_data1;
            LOGD("WSS CONNECTED: handle=%p, wsi=%p", handle, wsi);

            duk_push_pointer(ctx, wsi);
            duv_emit_event(ctx, handle->data, LWS_CONNECTION, 1);
            break;
        }
        case LWS_WSS_CALLBACK_EVENT_MESSAGE:
        {
            wiced_wss_t *wsi = (wiced_wss_t *) message->module_handle;
            wss_handle_t *handle = get_wss_handle(wsi);
            void *msg = message->event_data1;
            size_t len = message->event_data2;

            LOGD("WSS MESSAGE wsi=%p handle=%p  (%p-%lu)", wsi, handle, msg, len);

            duk_context *ctx = handle->ctx;

            if (msg != NULL && len > 0)
            {
                char *out = duk_push_fixed_buffer(ctx, len);
                // TODO: no data copying
                memcpy(out, msg, len);
                free(msg);

                duk_buffer_to_string(ctx, -1);
                duv_emit_event(ctx, handle->data, LWS_ONMESSAGE, 1);
            } else {
                LOGE("Invalid message: msg=%p, len=%d", msg, len);
            }
            break;
        }
        case LWS_WSS_CALLBACK_EVENT_CLOSE:
        {
            wiced_wss_t *wsi = (wiced_wss_t *) message->module_handle;
            wss_handle_t * handle = get_wss_handle(wsi);

            LOGD("WSS CLOSE: wsi=%p handle=%p ", wsi, handle);
            duk_context *ctx = handle->ctx;

            duv_emit_event(ctx, handle->data, LWS_CLOSED, 0);
            handle->data = duv_cleanup_handle(ctx, handle->data);
            delete_wss_handle(handle);
            break;
        }
    default:
        LOGE("Un-handled event: %d", event_id);
        break;
    }
    return;
}
