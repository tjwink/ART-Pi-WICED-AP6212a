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

#if defined(TX_ENABLE_EVENT_TRACE)

#include "tx_api.h"
#include "wiced.h"
#include "TraceX.h"
#include "platform_config.h"  /* Platform-specific TraceX buffer location */

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define WORKER_THREAD_STACK_SIZE  (1024)
#define WORKER_THREAD_QUEUE_SIZE  (4)

#define TRACEX_FILTER_STR_NONE      "<none>"
#define TRACEX_FILTER_STR_UNKNOWN   "<unknown>"

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    wiced_bool_t              initialized;
    wiced_tracex_state_t      state;
    wiced_semaphore_t         sem;
    wiced_tracex_config_t     config;
    struct {
        wiced_bool_t          op_pending;
        wiced_worker_thread_t thread;
    } worker;
    struct {
       wiced_bool_t          connected;
       wiced_tcp_socket_t    socket;
    } tcp;
} tracex_t;

typedef struct
{
    uint32_t mask;
    char*    str;
} tracex_filter_t;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t tracex_init( void );
static wiced_result_t tracex_enable_nolock( void );
static void tracex_buf_full_cb( void* buf_start );
static wiced_result_t tracex_worker_thread( void* unused );

static wiced_result_t tcp_server_connect( void );
static void tcp_server_disconnect( void );
static wiced_result_t tcp_server_send_data( void );
static wiced_result_t tcp_server_send_data_pkt( uint8_t* data, size_t datalen );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static tracex_t tracex =
{
    .initialized = WICED_FALSE,
    .state = WICED_TRACEX_STATE_DISABLED,
    .config = WICED_TRACEX_DEFAULT_CONFIG_INITIALIZER,
    .tcp.connected = WICED_FALSE,
};

static const tracex_filter_t tracex_filters[] =
{
    { TX_TRACE_ALL_EVENTS,              "allthreadx"       },
    { TX_TRACE_INTERNAL_EVENTS,         "internal"         },
    { TX_TRACE_BLOCK_POOL_EVENTS,       "blockpool"        },
    { TX_TRACE_BYTE_POOL_EVENTS,        "bytepool"         },
    { TX_TRACE_EVENT_FLAGS_EVENTS,      "eventflags"       },
    { TX_TRACE_INTERRUPT_CONTROL_EVENT, "interruptcontrol" },
    { TX_TRACE_MUTEX_EVENTS,            "mutex"            },
    { TX_TRACE_QUEUE_EVENTS,            "queue"            },
    { TX_TRACE_SEMAPHORE_EVENTS,        "semaphore"        },
    { TX_TRACE_THREAD_EVENTS,           "thread"           },
    { TX_TRACE_TIME_EVENTS,             "time"             },
    { TX_TRACE_TIMER_EVENTS,            "timer"            },
    { TX_TRACE_USER_EVENTS,             "user"             },
    { 0,                                ""                 }    /* EOL */
};

#ifdef WICED_TRACEX_BUFFER_DECLARE
/* TraceX buffer needs to be 4-byte aligned */
uint32_t wiced_tracex_buf[WICED_TRACEX_BUFFER_SIZE/4];
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/

/*!
 ******************************************************************************
 * Enables TraceX
 *
 * @return  WICED_SUCCESS for success, WICED_NOT_CONNECTED for TCP server
 *          connection errors, WICED_BADVALUE if TraceX is already enabled,
 *          otherwise WICED_ERROR
 */

wiced_result_t wiced_tracex_enable( wiced_tracex_config_t* config )
{
    wiced_result_t result;

    if (( config == NULL ) ||
        ( config->buf.addr == NULL ) || ( config->buf.size == 0 ))
    {
        return WICED_BADARG;
    }

    if (( config->tcp_server.enable == WICED_TRUE ) &&
        ( config->tcp_server.max_data_len == 0 ))
    {
        return WICED_BADARG;
    }


    if ( tracex.initialized != WICED_TRUE )
    {
        if ( tracex_init() != WICED_SUCCESS )
        {
            return WICED_ERROR;
        }
    }

    if ( wiced_rtos_get_semaphore(&tracex.sem,
                                  WICED_WAIT_FOREVER) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    if ( tracex.state == WICED_TRACEX_STATE_ENABLED )
    {
        wiced_rtos_set_semaphore(&tracex.sem);
        return WICED_BADVALUE;
    }

    tracex.config = *config;
    result = tracex_enable_nolock();

    wiced_rtos_set_semaphore(&tracex.sem);

    return result;
}


/*!
 ******************************************************************************
 * Disables TraceX and sends out event buffer to TCP server (if specified)
 *
 * @return  WICED_SUCCESS for success, otherwise WICED_ERROR
 */

wiced_result_t wiced_tracex_disable( void )
{
    if ( tracex.initialized != WICED_TRUE )
    {
        return WICED_ERROR;
    }

    if ( wiced_rtos_get_semaphore(&tracex.sem, WICED_WAIT_FOREVER) !=
         WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    if ( tracex.state != WICED_TRACEX_STATE_ENABLED )
    {
        tracex.state = WICED_TRACEX_STATE_DISABLED;
        wiced_rtos_set_semaphore(&tracex.sem);
        return WICED_SUCCESS;
    }

    tx_trace_disable();
    tracex.state = WICED_TRACEX_STATE_DISABLED;

    if ( tracex.worker.op_pending == WICED_TRUE )
    {
        /* Wait for the worker thread to finish */
        wiced_rtos_set_semaphore(&tracex.sem);
        while ( tracex.worker.op_pending == WICED_TRUE )
        {
            wiced_rtos_delay_milliseconds(500);
        }

        if ( wiced_rtos_get_semaphore(&tracex.sem, WICED_WAIT_FOREVER) !=
             WICED_SUCCESS )
        {
            return WICED_ERROR;
        }
    }

    if ( tracex.tcp.connected == WICED_TRUE )
    {
        /* Might have been reenabled by the TCP send thread */
        tx_trace_disable();
        tracex.state = WICED_TRACEX_STATE_DISABLED;

        tcp_server_send_data();

        tcp_server_disconnect();
        tracex.tcp.connected = WICED_FALSE;
    }

    wiced_rtos_set_semaphore(&tracex.sem);

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * Gets the TraceX configuration
 *
 * @return  WICED_SUCCESS on success, otherwise WICED_ERROR
 */

wiced_result_t wiced_tracex_config_get( wiced_tracex_config_t* config )
{
    *config = tracex.config;

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * Gets the TraceX status
 *
 * @return  WICED_SUCCESS on success, otherwise WICED_ERROR
 */

wiced_result_t wiced_tracex_status_get( wiced_tracex_status_t* status )
{
    status->state = tracex.state;
    status->tcp_connected = tracex.tcp.connected;

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * Gets the list of TraceX filters in string format
 *
 * @return  WICED_SUCCESS on success, WICED_PARTIAL_RESULTS if string buffer is
 *          too small to contain all the filters, otherwise WICED_ERROR
 */

wiced_result_t wiced_tracex_filter_list_get( char *str, size_t len )
{
    int i;

    str[0] = '\0';
    len--;

    for ( i = 0;; i++ )
    {
        if ( tracex_filters[i].mask == 0 )
        {
            break;
        }

        if ( strlen(tracex_filters[i].str) > len )
        {
           return WICED_PARTIAL_RESULTS;
        }

        strcat(str, tracex_filters[i].str);
        strcat(str, ",");
        len -= strlen(tracex_filters[i].str) + 1;
    }

    /* Remove the trailing comma */
    str[strlen(str) - 1] = '\0';

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * Sets the TraceX filters
 *
 * @return  WICED_SUCCESS on success, otherwise WICED_ERROR
 */

wiced_result_t wiced_tracex_filter_set( uint32_t mask )
{
    tracex.config.filter = mask;

    if ( tracex.state == WICED_TRACEX_STATE_ENABLED )
    {
        if (( tx_trace_event_filter(mask) != TX_SUCCESS ) ||
            ( tx_trace_event_unfilter(~mask) != TX_SUCCESS ))
        {
            return WICED_ERROR;
        }
    }

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * Converts TraceX filter mask to filter strings
 *
 * @return  WICED_SUCCESS on success, WICED_PARTIAL_RESULTS if string is longer
 *          than provided buffer, and WICED_BADARG on bad arguments
 */
wiced_result_t wiced_tracex_filter_mask_to_str( uint32_t mask, char* str, size_t len )
{
    int i;

    if (( str == NULL ) || ( len == 0 ))
    {
        return WICED_BADARG;
    }

    str[0] = '\0';
    len--;

    for ( i = 0;; i++ )
    {
        if ( tracex_filters[i].mask == 0 )
        {
            break;
        }

        if ( mask & tracex_filters[i].mask )
        {
            if ( strlen(tracex_filters[i].str) > len )
            {
                /* Not enough room! Fill out the rest of string with dots */
                for ( i = 0; i < len; i++ )
                {
                    strcat(str, ".");
                }

                return WICED_PARTIAL_RESULTS;
            }

            strcat(str, tracex_filters[i].str);
            strcat(str, ",");
            len -= strlen(tracex_filters[i].str) + 1;

            mask &= ~tracex_filters[i].mask;
        }
    }

    /* Check for any unknown filter masks */
    if ( mask )
    {
        if ( strlen(TRACEX_FILTER_STR_UNKNOWN) > len )
        {
            /* Not enough room! Fill out the rest of string with dots */
            for ( i = 0; i < len; i++ )
            {
                strcat(str, ".");

                return WICED_PARTIAL_RESULTS;
            }
        }
        strcat(str, TRACEX_FILTER_STR_UNKNOWN);
    }
    else if ( strlen(str) == 0 )
    {
        if ( strlen(TRACEX_FILTER_STR_NONE) > len )
        {
            /* Not enough room! Fill out the rest of string with dots */
            for ( i = 0; i < len; i++ )
            {
                strcat(str, ".");

                return WICED_PARTIAL_RESULTS;
            }
        }
        strcat(str, TRACEX_FILTER_STR_NONE);
    }
    else
    {
        /* Remove the comma */
        str[strlen(str) - 1] = '\0';
    }

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * Converts TraceX filter strings to filter mask
 *
 * @return  WICED_SUCCESS on success, WICED_UNFINISHED if one or more strings
 *          are unrecognized
 */

wiced_result_t wiced_tracex_filter_str_to_mask( char* str, uint32_t* mask )
{
    wiced_result_t result = WICED_SUCCESS;
    char           filters[WICED_TRACEX_FILTER_MAX_STRING_LEN];
    char*          filter;
    char*          saveptr;

    if (( strlen(str) >= WICED_TRACEX_FILTER_MAX_STRING_LEN ) ||
        ( strncpy(filters, str, sizeof(filters)) == NULL ))
    {
        return WICED_BADARG;
    }

    *mask = 0;

    filter = strtok_r(filters, ",", &saveptr);
    while ( filter != NULL )
    {
        int i;

        for ( i = 0;; i++ )
        {
            if ( tracex_filters[i].mask == 0 )
            {
                result = WICED_UNFINISHED;
                break;
            }

            if ( strcmp(filter, tracex_filters[i].str) == 0 )
            {
                *mask |= tracex_filters[i].mask;
                break;
            }
        }

        filter = strtok_r(NULL, ",", &saveptr);
    }

    return result;
}

/*!
 ******************************************************************************
 * Send the TraceX buffer to a TCP server - useful for sending buffers that
 * were filled while not connected to a TCP server, such as at boot
 *
 * @return  WICED_SUCCESS for success, WICED_BADVALUE if TraceX is enabled,
 *          WICED_BADARG for invalid arguments, WICED_NOT_CONNECTED for TCP
 *          connection error, otherwise WICED_ERROR
 */

wiced_result_t wiced_tracex_buf_send( wiced_tracex_tcp_server_t* server )
{
    wiced_result_t result;

    if (( server == NULL ) || ( server->max_data_len == 0 ))
    {
            return WICED_BADARG;
    }

    if ( tracex.initialized != WICED_TRUE )
    {
        if ( tracex_init() != WICED_SUCCESS )
        {
            return WICED_ERROR;
        }
    }

    /* Grab the semaphore so TraceX won't be enabled while we are sending */
    if ( wiced_rtos_get_semaphore(&tracex.sem,
                                  WICED_WAIT_FOREVER) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    if ( tracex.state == WICED_TRACEX_STATE_ENABLED )
    {
        result = WICED_BADVALUE;
        goto out;
    }

    /* Update the TCP server information */
    tracex.config.tcp_server = *server;

    if ( tcp_server_connect() != WICED_SUCCESS )
    {
        result = WICED_NOT_CONNECTED;
        goto out;
    }

    if ( tcp_server_send_data() != WICED_SUCCESS )
    {
        result = WICED_ERROR;
    }
    else
    {
        result = WICED_SUCCESS;
    }

    tcp_server_disconnect();

out:
    wiced_rtos_set_semaphore(&tracex.sem);

    return result;
}

/*!
 ******************************************************************************
 * Initialize TraceX
 *
 * @return  WICED_SUCCESS for success, otherwise WICED_ERROR
 */

static wiced_result_t tracex_init( void )
{
    if ( tracex.initialized == WICED_TRUE )
    {
        return WICED_SUCCESS;
    }

    if ( wiced_rtos_init_semaphore(&tracex.sem) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    wiced_rtos_set_semaphore(&tracex.sem);

    if ( wiced_rtos_create_worker_thread(&tracex.worker.thread,
                                         WICED_DEFAULT_WORKER_PRIORITY,
                                         WORKER_THREAD_STACK_SIZE,
                                         WORKER_THREAD_QUEUE_SIZE) !=
         WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    tracex.initialized = WICED_TRUE;

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * Enables TraceX with the assumption that the semaphore is already gotten
 *
 * @return  WICED_SUCCESS for success, otherwise WICED_ERROR
 */

static wiced_result_t tracex_enable_nolock( void )
{
    wiced_result_t result;

    if ( tracex.config.tcp_server.enable == WICED_TRUE )
    {
        if ( tcp_server_connect() != WICED_SUCCESS )
        {
            tracex.state = WICED_TRACEX_STATE_DISABLED_TCP_ERR;
            result = WICED_NOT_CONNECTED;
            goto out;
        }

        tracex.tcp.connected = WICED_TRUE;
    }

    /* Success only if enabled; error out even if it's already running, which
     * means there's another process controlling TraceX outside of this API
     */
    if ( tx_trace_enable(tracex.config.buf.addr, tracex.config.buf.size,
                         tracex.config.buf.obj_cnt) != TX_SUCCESS )
    {
        tracex.state = WICED_TRACEX_STATE_DISABLED_TRACEX_ERR;
        result = WICED_ERROR;
        goto err_tcp_disconnect;
    }

    tracex.state = WICED_TRACEX_STATE_ENABLED;

    /* Apply the filters now because by default nothing is filtered */
    if ( wiced_tracex_filter_set(tracex.config.filter) != WICED_SUCCESS )
    {
        tracex.state = WICED_TRACEX_STATE_DISABLED_TRACEX_ERR;
        result = WICED_ERROR;
        goto err_disable_tracex;
    }

    if ( tx_trace_buffer_full_notify(tracex_buf_full_cb) != TX_SUCCESS )
    {
        tracex.state = WICED_TRACEX_STATE_DISABLED_TRACEX_ERR;
        result = WICED_ERROR;
        goto err_disable_tracex;
    }

    result = WICED_SUCCESS;
    goto out;

err_disable_tracex:
    tx_trace_disable();

err_tcp_disconnect:
    if ( tracex.tcp.connected == WICED_TRUE )
    {
        tcp_server_disconnect();
        tracex.tcp.connected = WICED_FALSE;
    }

out:
    return result;
}

/*!
 ******************************************************************************
 * Handles TraceX buffer full event; called with interrupts disabled!
 *
 * @return
 */

static void tracex_buf_full_cb( void* buffer_addr )
{
    if (( tracex.tcp.connected == WICED_TRUE ) ||
        ( tracex.config.loop_rec == WICED_FALSE ))
    {
        /* Disable TraceX but don't change the state just yet */
        tx_trace_disable();
        tracex.worker.op_pending = WICED_TRUE;

        /* This callback may be running in IRQ context, so schedule the worker
         * thread to do time consuming tasks
         */
        wiced_rtos_send_asynchronous_event(&tracex.worker.thread,
                                           tracex_worker_thread, NULL);
    }
}

/*!
 ******************************************************************************
 * Sends TraceX buffer to TCP server
 *
 * @return  WICED_SUCCESS for success, otherwise WICED_ERROR
 */

static wiced_result_t tracex_worker_thread( void* unused )
{
    if ( wiced_rtos_get_semaphore(&tracex.sem, WICED_WAIT_FOREVER) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    if ( tracex.tcp.connected == WICED_TRUE )
    {
        /* Send TraceX buffer to the TCP server, then disconnect */
        if ( tcp_server_send_data() != WICED_SUCCESS )
        {
           tracex.state = WICED_TRACEX_STATE_DISABLED_TCP_ERR;
        }

        tcp_server_disconnect();
        tracex.tcp.connected = WICED_FALSE;
    }

    /* There may be a chance that TraceX has been disabled already */
    if ( tracex.state == WICED_TRACEX_STATE_ENABLED )
    {
        if ( tracex.config.loop_rec == WICED_TRUE )
        {
            tracex_enable_nolock();
        }
        else
        {
            tracex.state = WICED_TRACEX_STATE_DISABLED_BUF_FULL;
        }
    }

    tracex.worker.op_pending = WICED_FALSE;

    wiced_rtos_set_semaphore(&tracex.sem);

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * Connects to TCP server
 *
 * @return  WICED_SUCCESS on success, otherwise WICED_ERROR
 */

static wiced_result_t tcp_server_connect( void )
{
    wiced_result_t result;
    int            retries;

    /* Create a TCP socket */
    if ( wiced_tcp_create_socket(&tracex.tcp.socket,
                                 WICED_STA_INTERFACE ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    /* Bind to the socket */
    wiced_tcp_bind(&tracex.tcp.socket, tracex.config.tcp_server.port);

    /* Connect to the remote TCP server, try several times */
    retries = 0;
    do
    {
        result = wiced_tcp_connect(&tracex.tcp.socket,
                                   &tracex.config.tcp_server.ip,
                                   tracex.config.tcp_server.port,
                                   tracex.config.tcp_server.timeout);
        retries++;
    } while (( result != WICED_SUCCESS ) &&
             ( retries < tracex.config.tcp_server.num_retries ));

    if ( result != WICED_SUCCESS )
    {
        wiced_tcp_delete_socket(&tracex.tcp.socket);
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * Disconnects from TCP server
 *
 * @return
 */

static void tcp_server_disconnect( void )
{
    wiced_tcp_disconnect(&tracex.tcp.socket);
    wiced_tcp_delete_socket(&tracex.tcp.socket);
}

/*!
 ******************************************************************************
 * Sends TraceX buffer to TCP server
 *
 * @return  WICED_SUCCESS on success, otherwise WICED_ERROR
 */

static wiced_result_t tcp_server_send_data( void )
{
    size_t   sent = 0;
    uint8_t* buf = (uint8_t*)tracex.config.buf.addr;

    for ( sent = 0; sent < tracex.config.buf.size; )
    {
        size_t size = tracex.config.buf.size - sent >
                      tracex.config.tcp_server.max_data_len ?
                      tracex.config.tcp_server.max_data_len :
                      tracex.config.buf.size - sent;

        if ( tcp_server_send_data_pkt(&buf[sent], size) !=  WICED_SUCCESS )
        {
            return WICED_ERROR;
        }

        sent += size;
    }

    return WICED_SUCCESS;
}

/*!
 ******************************************************************************
 * Sends a data packet to TCP server
 *
 * @return  WICED_SUCCESS on success, otherwise WICED_ERROR
 */

static wiced_result_t tcp_server_send_data_pkt( uint8_t* buffer, size_t length )
{
    wiced_packet_t* pkt;
    uint8_t*        pkt_data;
    uint16_t        pkt_data_avail_len;

    /* Create the TCP packet */
    if ( wiced_packet_create_tcp(&tracex.tcp.socket, length, &pkt, &pkt_data,
                                 &pkt_data_avail_len) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    if ( length > pkt_data_avail_len )
    {
        wiced_packet_delete(pkt);

        return WICED_ERROR;
    }

    /* Write the message into pkt_data */
    memcpy(pkt_data, buffer, length);

    /* Set the end of the data portion */
    wiced_packet_set_data_end(pkt, pkt_data + length);

    /* Send the TCP packet */
    if ( wiced_tcp_send_packet(&tracex.tcp.socket, pkt) != WICED_SUCCESS )
    {
        wiced_packet_delete(pkt);

        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

#endif /* defined(TX_ENABLE_EVENT_TRACE) */
