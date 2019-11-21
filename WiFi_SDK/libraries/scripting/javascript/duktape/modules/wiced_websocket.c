/*
 * $ Copyright Cypress Semiconductor  $
 */

#include "wiced.h"
#include "websocket.h"
#include "wss.h"
#include "wiced_duktape.h"
#include "wiced_websocket.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define LOG_LABEL               "duk:websocket"
#define LOG_DEBUG_ENABLE        0

#define BUFFER_LENGTH       (8192)
#define FINAL_FRAME         WICED_TRUE

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
static wiced_result_t wiced_wss_on_open_cb(void *arg);
static wiced_result_t wiced_wss_on_message_cb(void *arg, uint8_t* data, uint32_t length, wiced_websocket_frame_type_t type, wiced_websocket_frame_flags_t flags);
static wiced_result_t wiced_wss_on_close_cb(void *arg);
static wiced_result_t wiced_wss_on_error_cb(void *arg);
/******************************************************
 *               Variable Definitions
 ******************************************************/
static wiced_websocket_url_protocol_entry_t  wss_url_protocol_list[] =
{
    [0] =
    {
     .url = NULL,
     .protocols = NULL,
    },
};

static wiced_websocket_url_protocol_table_t wss_table =
{
    .count = 1,
    .entries = wss_url_protocol_list,
};

static wiced_websocket_callbacks_t wss_callbacks =
{
    .on_open    = wiced_wss_on_open_cb,
    .on_close   = wiced_wss_on_close_cb,
    .on_error   = wiced_wss_on_error_cb,
    .on_message = wiced_wss_on_message_cb,
};

static char rx_buffer[ BUFFER_LENGTH ];

static wiced_websocket_server_config_t wss_config =
{
    .max_connections            = 2,
    .heartbeat_duration         = 0,
    .url_protocol_table         = &wss_table,
    .rx_frame_buffer            = rx_buffer,
    .frame_buffer_length        = BUFFER_LENGTH,
};

static wiced_websocket_server_t wss_server;


/******************************************************
 *               Function Definitions
 ******************************************************/
extern void set_wss_running(int);

static wiced_result_t wiced_wss_on_open_cb(void *arg)
{
    wiced_duktape_callback_queue_element_t message;

    message.module_id = WDCM_WEBSOCKET;
    message.event_id = LWS_WSS_CALLBACK_EVENT_CONNECTION;
    message.module_handle = (void *) wss_server.user_data;
    message.event_data1 = (void *) arg;
    LOGD("CONNECTED: handle=%p, wsi=%p", wss_server.user_data, arg);
    wiced_duktape_callback_post_event(&message);
    return WICED_SUCCESS;
}

static wiced_result_t wiced_wss_on_message_cb(void *arg, uint8_t* data, uint32_t length, wiced_websocket_frame_type_t type, wiced_websocket_frame_flags_t flags)
{
    wiced_websocket_t* wss = ( wiced_websocket_t* )arg;

    LOGD("wiced_wss_on_message_cb: wss=%p type=%d", wss, type);

    switch( type )
    {
        case WEBSOCKET_TEXT_FRAME:
        {
            char * out = malloc( length );
            LOGD("MSG: out=%p length=%u-%u", out, strlen(data), length);
            if (out != NULL){
                wiced_duktape_callback_queue_element_t message;
                memcpy(out, data, length);
                message.module_id = WDCM_WEBSOCKET;
                message.event_id = LWS_WSS_CALLBACK_EVENT_MESSAGE;
                message.module_handle = (void *) wss;
                message.event_data1 = (void *) out;
                message.event_data2 = length;
                wiced_duktape_callback_post_event(&message);
            } else {
                LOGE("Failed to allocate buffer !!!");
            }
            break;
        }
        case WEBSOCKET_CONNECTION_CLOSE:
        {
            LOGD("WS CLOSE: wss=%p", wss);
            wiced_wss_on_close_cb(wss);
            break;
        }
        default:
        {
            LOGW("Event (%d) is not hanlded", type);
            break;
        }
    }
    return WICED_SUCCESS;
}

static wiced_result_t wiced_wss_on_close_cb(void *arg)
{
    wiced_duktape_callback_queue_element_t message;
    wiced_websocket_t* wss = ( wiced_websocket_t* )arg;

    LOGD("Connection closed. wss(%p)", wss);

    message.module_id = WDCM_WEBSOCKET;
    message.event_id = LWS_WSS_CALLBACK_EVENT_CLOSE;
    message.module_handle = wss;
    message.event_data1 = NULL;
    message.event_data2 = 0;
    wiced_duktape_callback_post_event(&message);
    return WICED_SUCCESS;
}

static wiced_result_t wiced_wss_on_error_cb(void *arg)
{
    wiced_duktape_callback_queue_element_t message;
    wiced_websocket_t* wss = ( wiced_websocket_t* )arg;

    /* sned close command instead */
    LOGE("ERR: websocket wss(%p)", wss);
    LOGW(" >>> send close message. wss(%p)", wss);

    message.module_id = WDCM_WEBSOCKET;
    message.event_id = LWS_WSS_CALLBACK_EVENT_CLOSE;
    message.module_handle = wss;
    message.event_data1 = NULL;
    message.event_data2 = 0;
    wiced_duktape_callback_post_event(&message);
    return WICED_SUCCESS;
}

int wiced_wss_send_packets(wiced_websocket_t* wss, void* buf, size_t len)
{
    wiced_result_t result;
    result = wiced_websocket_send(wss, buf, len, WEBSOCKET_TEXT_FRAME, WEBSOCKET_FRAME_FLAG_UNFRAGMENTED );
    LOGD("Send packets (%p-%u) on websocket (%p)", buf, len, wss);
    if (result == WICED_SUCCESS) {
        return len;
    } else {
        LOGE("Failed to send packets (%p-%u) on websocket (%p)", buf, len, wss);
        return -1;
    }
}
int start_wss(int port, wss_handle_t *handle )
{
    wiced_result_t result;

    result = wiced_websocket_server_start( &wss_server, &wss_config, &wss_callbacks, NULL, port, handle);
    LOGI("Start websocket server result=%d", result);
    if ( result != WICED_SUCCESS )
    {
        LOGE("Failed to start websocket server");
        return -1;
    }

    // notify DIAL that WSS is ready
    set_wss_running(1);

    return 0;
}

int stop_wss(void)
{
    LOGW("Stopping websocket server");
    wiced_websocket_server_stop(&wss_server);

    set_wss_running(0);
    LOGW("Stopped websocket server");
    return 0;
}

