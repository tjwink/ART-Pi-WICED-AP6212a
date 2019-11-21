/*
 * $ Copyright Cypress Semiconductor  $
 */

#include "timer.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define LOG_LABEL           "[duk:mod:timer]"
#define LOG_DEBUG_ENABLE    0

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
static void wiced_timer_cb(void* arg);
static linked_list_t* wiced_dpm_timer_list_init(void);
static void wiced_dpm_timer_list_deinit(void);
static void wiced_dpm_timer_node_delete_all(void);
static void wiced_dpm_timer_node_insert(dpm_timer_handle_t *handle);
static void wiced_dpm_timer_node_remove(dpm_timer_handle_t *handle);

/******************************************************
 *               Variable Definitions
 ******************************************************/
static linked_list_t wiced_dpm_timer_list, *wiced_dpm_timer_list_ptr;
static wiced_mutex_t wiced_dpm_timer_mutex;

/******************************************************
 *               Function Definitions
 ******************************************************/
static linked_list_t* wiced_dpm_timer_list_init(void)
{
    if (wiced_dpm_timer_list_ptr == NULL)
    {
        wiced_rtos_init_mutex( &wiced_dpm_timer_mutex );
        wiced_dpm_timer_list_ptr = &wiced_dpm_timer_list;
        linked_list_init( wiced_dpm_timer_list_ptr );
    }
    return wiced_dpm_timer_list_ptr;
}

static void wiced_dpm_timer_list_deinit(void)
{
    if (wiced_dpm_timer_list_ptr != NULL)
    {
        wiced_rtos_deinit_mutex( &wiced_dpm_timer_mutex );
        linked_list_deinit( wiced_dpm_timer_list_ptr );
        wiced_dpm_timer_list_ptr = NULL;
    }
}

static void wiced_dpm_timer_node_insert(dpm_timer_handle_t *handle)
{
    linked_list_t* list_ptr = wiced_dpm_timer_list_init();
    wiced_rtos_lock_mutex(&wiced_dpm_timer_mutex);
    linked_list_set_node_data(&handle->node, handle);
    linked_list_insert_node_at_rear( list_ptr, &handle->node );
    wiced_rtos_unlock_mutex(&wiced_dpm_timer_mutex);
    return;
}

static void wiced_dpm_timer_node_remove(dpm_timer_handle_t *handle)
{
    linked_list_t* list_ptr = wiced_dpm_timer_list_ptr;
    if (list_ptr == NULL) {
        LOGE("DPM timer list is not initialized!");
        return;
    }
    wiced_rtos_lock_mutex(&wiced_dpm_timer_mutex);
    linked_list_remove_node( list_ptr, &handle->node );
    wiced_rtos_unlock_mutex(&wiced_dpm_timer_mutex);
    return;
}

static void wiced_dpm_timer_node_delete_all(void)
{
    linked_list_t* list_ptr = wiced_dpm_timer_list_ptr;
    linked_list_node_t *node = NULL;
    dpm_timer_handle_t *handle;
    wiced_result_t result;
    int i = 0;
    uint32_t j = 0;

    if (list_ptr == NULL) {
        LOGE("DPM timer list is not initialized!");
        return;
    }

    wiced_rtos_lock_mutex(&wiced_dpm_timer_mutex);
    linked_list_get_front_node( list_ptr, &node );
    while (node != NULL) {
        handle = (dpm_timer_handle_t *) node->data;
        result = wiced_rtos_is_timer_running(&handle->timer);
        if (WICED_SUCCESS == result) {
            result = wiced_rtos_stop_timer(&handle->timer);
            if (WICED_SUCCESS != result) {
                LOGE("Failed to stop a timer(%p-%d)", &handle->timer, result);
            }
        }
        wiced_rtos_deinit_timer(&handle->timer);
        handle->data = duv_cleanup_handle(handle->ctx, handle->data);
        i++;
        node = node->next;
    }
    wiced_rtos_unlock_mutex(&wiced_dpm_timer_mutex);
    linked_list_get_count(list_ptr, &j);
    LOGI("DPM timer: deleted(%d-%u)", i, j);
    wiced_dpm_timer_list_deinit();
    return;
}

void timer_callback_handler(wiced_duktape_callback_queue_element_t *message)
{
    dpm_timer_callback_event_id event_id = (dpm_timer_callback_event_id) message->event_id;
    wiced_result_t result;

    switch ( event_id )
    {
        case LWS_TIMER_CALLBACK_EVENT_TIMEOUT:
        {
            dpm_timer_handle_t *handle = (dpm_timer_handle_t *)message->module_handle;
            if (handle != NULL && handle->data != NULL)
            {
                duv_emit_event(handle->ctx, handle->data, LWS_TIMEOUT, 0);
                /* check if it is a periodic timer */
                if ( handle->repeat == 0) {
                    // delete the timer
                    result = wiced_rtos_deinit_timer(&handle->timer);
                    if (result != WICED_SUCCESS) {
                        LOGE("Failed to deinit timer %p - %d", &handle->timer, result);
                    }
                    // free up the handle
                    handle->data = duv_cleanup_handle(handle->ctx, handle->data);
                    wiced_dpm_timer_node_remove(handle);
                }
            }
            else
            {
                LOGI("Timer has been deleteted (%p-%p)", handle, handle ? handle->data : NULL);
            }
            break;
        }
        case LWS_TIMER_CALLBACK_EVENT_INTERVAL:
        default:
            break;
    }
    return;
}

static void wiced_timer_cb(void* arg) {
    dpm_timer_handle_t *handle = (dpm_timer_handle_t *) arg;
    wiced_result_t result;
    wiced_duktape_callback_queue_element_t message;

    if (handle->repeat == 0) {
        // stop the timer if it is not periodic timer.
        result = wiced_rtos_stop_timer(&handle->timer);
        if (result != WICED_SUCCESS) {
            LOGE("Failed to stop timer %p - %d", &handle->timer, result);
        }
    }

    message.module_id = WDCM_TIMER;
    message.event_id = LWS_TIMER_CALLBACK_EVENT_TIMEOUT;
    message.module_handle = (void *)handle;
    message.event_data2 = 0;
    result = wiced_duktape_callback_post_event(&message);
    if (result != WICED_SUCCESS)
    {
        LOGE("Failed to post websocket connection event(%d)", result);
    }
    return;
}

duk_ret_t dpm_new_timer(duk_context *ctx) {
    dpm_timer_handle_t *handle;

    dschema_check(ctx, (const duv_schema_entry[]) {
        {NULL}
    });

    handle = duk_push_fixed_buffer(ctx, sizeof(*handle));
    handle->ctx = ctx;
    handle->data = duv_setup_handle(ctx);
    wiced_dpm_timer_node_insert(handle);
    LOGD("dpm_new_timer, handle=%p", handle);
    return 1;
}

duk_ret_t dpm_timer_start(duk_context *ctx) {
    dpm_timer_handle_t* handle;
    uint32_t timeout;
    wiced_result_t result;

    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {"timeout", duk_is_number},
        {"repeat", duk_is_boolean},
        {"ontimeout", duk_is_function},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    timeout = duk_get_uint(ctx, 1);
    handle->repeat = duk_get_boolean(ctx, 2);
    LOGD("dpm_start_timer, handle=%p - timer:%p-%lu-%u", handle, &handle->timer, timeout, handle->repeat);
    result = wiced_rtos_init_timer(&handle->timer, timeout, wiced_timer_cb, handle);
    if (result != WICED_SUCCESS)
    {
        LOGE("failed to start a timer(%p-%d)", &handle->timer, result);
        // TODO returne error
    }
    else
    {
        wiced_rtos_start_timer(&handle->timer);
    }
    duv_store_handler(ctx, handle->data, LWS_TIMEOUT, 3);
    return 0;
}

duk_ret_t dpm_timer_stop(duk_context *ctx) {
    dpm_timer_handle_t* handle;
    wiced_result_t result;

    dschema_check(ctx, (const duv_schema_entry[]) {
        {"handle", duk_is_fixed_buffer},
        {NULL}
    });

    handle = duk_get_buffer(ctx, 0, NULL);
    LOGD("dpm_timer_stop, handle=%p - timer:%p", handle, &handle->timer);
    result = wiced_rtos_is_timer_running(&handle->timer);
    if (WICED_SUCCESS == result) {
        result = wiced_rtos_stop_timer(&handle->timer);
        if (WICED_SUCCESS != result) {
            LOGE("Failed to stop a timer(%p-%d)", &handle->timer, result);
        }
        wiced_rtos_deinit_timer(&handle->timer);
        handle->data = duv_cleanup_handle(handle->ctx, handle->data);
        wiced_dpm_timer_node_remove(handle);
    }
    return 0;
}

duk_ret_t dpm_timer_shutdown(duk_context *ctx) {

    dschema_check(ctx, (const duv_schema_entry[]) {
        {NULL}
    });

    LOGD("dpm_timer_shutdown");
    wiced_dpm_timer_node_delete_all();
    return 0;
}
