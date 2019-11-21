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
 *
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include "wiced_ssdp.h"

/******************************************************
 *                    Macros
 ******************************************************/
#define WPRINT_SSDP_DEBUG(level, arg)   if ( ( g_internal != NULL ) && ( g_internal->log_level >= level ) ) { WPRINT_LIB_INFO(arg); }

/******************************************************
 *                    Constants
 ******************************************************/
#define SSDP_TAG_VALID                (uint32_t)0x43BAAB34
#define SSDP_TAG_INVALID              (uint32_t)0xDEADBEEF

#define WICED_SSDP_REPLY_BUFF_SIZE      (2048)

#define WICED_SSDP_SERVER_LISTEN_BACKLOG             (20)  /* backlog 20 packets      */
#define APP_QUEUE_MAX_ENTRIES                        (20)  /* packet queue            */

#define SSDP_THREAD_SHUTDOWN_WAIT                   (100)  /* wait for a shutdown flag                          */
#define SSDP_FLAGS_TIMEOUT                          (100)  /* wait for a command before doing anything          */
#define SSDP_PACKET_TIMEOUT                         (200)  /* ms wait for getting udp packet in the callback    */
#define SSDP_QUEUE_PUSH_TIMEOUT                     (200)  /* ms wait for pushing packet to the queue           */
#define SSDP_QUEUE_POP_TIMEOUT                       (20)  /* ms wait for popping packet from the queue         */

#define WICED_SSDP_WORKER_THREAD_PRIORITY           (WICED_DEFAULT_WORKER_PRIORITY)

#define WICED_SSDP_WORKER_STACK_SIZE                (6 * 1024)

#define WICED_SSDP_UUID_MAX                 (36)
/* we fill in the last 12 digits from the last 6 octets of MAC */
#define SSDP_UUID_TEMPLATE                  "7272e15a-2505-11e5-b345-"

#define WICED_SSDP_MCAST_INITIAL_NOTIFY_FAST_COUNT      (5) /* send out the first notifications fairly quickly */
#define WICED_SSDP_ALWAYS_SEND_A_FEW_MSEARCH_PACKETS    (3) /* always send 3 M-SEARCH within 1 second and wit for responses */
#define WICED_SSDP_MSEARCH_SEND_TIME_1                 (21) /* send 2nd M-SEARCH 21ms after first */
#define WICED_SSDP_MSEARCH_SEND_TIME_2                 (43) /* send 3rd M-SEARCH 33ms after first  (we send 3 within < 100ms) */


#define NUM_MSECONDS_IN_SECOND                      (1000)
/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    SSDP_EVENT_WORKER_THREAD_SHUTDOWN   = (1 << 0),
    SSDP_EVENT_WORKER_THREAD_DONE       = (1 << 1),

    SSDP_EVENT_WORKER_PULL_PACKET       = (1 << 2),

    SSDP_EVENT_MCAST_SEND_NOTIFY        = (1 << 3),

} SSDP_EVENTS_T;

#define SSDP_EVENT_WORKER_THREAD_EVENTS  (SSDP_EVENT_WORKER_PULL_PACKET  | SSDP_EVENT_MCAST_SEND_NOTIFY | SSDP_EVENT_WORKER_THREAD_SHUTDOWN)

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
/* Callback info structure
 *
 * Passed to application in callback registered with wiced_ssdp_server_register_callback()
 * Provides information about the received SSDP packet and local IP information.
 * Also provides a buffer for a response.
 *
 */
typedef struct wiced_ssdp_callback_info_s
{

    wiced_packet_t*     packet;                                        /* UDP packet received         - DO NOT MODIFY !!!                      */
    wiced_ip_address_t  local_ip_addr;                                 /* Local IP address            - DO NOT MODIFY !!!                      */
                                                                       /*      will be SSDP_MULTICAST_IPV4_ADDRESS when rec'd on multicast     */
    char                local_ip_address_string[SSDP_INET_ADDRSTRLEN]; /* string version of local ip address   */
    uint16_t            local_port;                                    /* Local port                  - DO NOT MODIFY !!!                      */

    wiced_ip_address_t  peer_ip_addr;                                  /* Peer IP address            - DO NOT MODIFY !!!                       */
    uint16_t            peer_port;                                     /* Local port                 - DO NOT MODIFY !!!                       */

    /*
     * NOTE: request buffer has been modified
     *          - NULL characters added to end of headers
     *          - NULL characters added to end of header values
     *       Use request_headers list to find headers & values
     *
     *       DO NOT MODIFY any of these values!
     */
    char*               request_buffer;                                /* request pointer from packet - DO NOT MODIFY !!!                      */
    uint16_t            request_length;                                /* request pointer from packet - DO NOT MODIFY !!!                      */

    http_request_t      request_type;                                  /* HTTP request type           - DO NOT MODIFY !!!                      */
    uint16_t            request_num_headers;                           /* number of headers in packet - DO NOT MODIFY !!!                      */
    http_header_t       request_headers[SSDP_HTTP_HEADERS_MAX];        /* pointers to headers/values in packet         */
    char*               request_body;                                   /* pointer to request body     - DO NOT MODIFY !!!                     */
    uint32_t            request_body_length;                            /* length of request body      - DO NOT MODIFY !!!                     */

    uint16_t            do_not_reply;                                  /* if WICED_TRUE, do not reply - User set before return from callback   */
    uint16_t            reply_length;                                  /* length of actual Reply      - User set before return from callback   */
    uint16_t            reply_status;                                  /* reply status                - User set before return from callback   */

    void*               user_data;                                     /* user provided data                                                   */
} wiced_ssdp_callback_info_t;


/* This structure contains information about the HTTP request - used to put received packets on the queue */
typedef struct wiced_ssdp_request_info_s
{
    wiced_packet_t*     packet;
} wiced_ssdp_request_info_t;


typedef struct wiced_ssdp_internal_s
{
    uint32_t                        tag;

    /* provided by caller, cleaned for our use */
    wiced_interface_t               interface;                      /* WICED_STA_INTERFACE or WICED_AP_INTERFACE            */
    uint16_t                        server_port;                    /* port of server you are running separately            */
    char*                           serve_page_path;                /* path added to IP when responding to M-SEARCH request */

    /* Notify parameters */
    char*                           notify_server_type;             /* default "quick_ssdp/1.0"     */
    char*                           notify_usn_type;                /* default "ssdp:rootdevice"    */
    char*                           notify_search_target;           /* default "ssdp:all"           */
    char                            uuid[WICED_SSDP_UUID_MAX];      /* device UUID                  */
    WICED_SSDP_LOG_LEVEL_T          log_level;

    wiced_ssdp_notify_callback_t    notify_callback;                /* Application's callback when a NOTIFY is received */
    void*                           notify_callback_data;           /* Application's data for the callback              */

    /* created/used internally */
    wiced_ip_address_t              local_ip_address;
    char                            local_ip_address_string[SSDP_INET_ADDRSTRLEN + 1];

    char                            mcast_ip_address_string[SSDP_INET_ADDRSTRLEN + 1];

    /* handling incoming SSDP requests */
    wiced_thread_t                  ssdp_worker_thread;
    wiced_thread_t*                 ssdp_worker_thread_ptr;
    wiced_event_flags_t             ssdp_flags;
    wiced_queue_t                   ssdp_queue;
    char                            ssdp_reply_buffer[WICED_SSDP_REPLY_BUFF_SIZE];

    /* for multicast */
    uint16_t                        mcast_port;                 /* ALWAYS SSDP_MULTICAST_PORT           */
    wiced_ip_address_t              mcast_ip_address;           /* ALWAYS SSDP_MULTICAST_IPV4_ADDRESS   */
    wiced_udp_socket_t              mcast_udp_socket;           /* wiced UDP socket                     */
    wiced_bool_t                    mcast_socket_opened;        /* WICED_TRUE if we opened the socket   */
    uint16_t                        mcast_notify_time;          /* for sending out multicast NOTIFY     */
    wiced_timer_t                   mcast_notify_timer;         /* for sending out multicast NOTIFY     */
    uint32_t                        mcast_notify_timer_ms;      /* # ms we set timer for                */
    uint16_t                        mcast_notify_count;         /* # times we have sent a notify        */

    /* M-Search params & responses */
    wiced_bool_t                    msearch_active;             /* If WICED_TRUE, do M-SEARCH response processing       */
    uint16_t                        response_array_size;        /* provided by application                              */
    wiced_ssdp_msearch_response_t*  responses;                  /* provided by application                              */
    wiced_bool_t                    unique_responses_received;     /* report # unique responses                         */
    wiced_bool_t                    responses_received;         /* # responses received                                 */

    /* debug    */
    uint16_t                        debug_queue_max;            /* max count on the queue   */
    uint16_t                        debug_queue_count;          /* count stuff on the queue */
} wiced_ssdp_internal_t;


/******************************************************
 *               Global Variables
 ******************************************************/
extern wiced_ssdp_internal_t* g_internal;       /* internal data structure */

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t wiced_ssdp_v4_ip_addr_to_string     ( wiced_ip_address_t *addr, char* buff, uint16_t buff_len );
void           wiced_ssdp_process_packet           ( wiced_ssdp_internal_t* internal, wiced_ssdp_callback_info_t * cb_info );

wiced_result_t wiced_ssdp_internal_mcast_start     ( wiced_ssdp_internal_t*  internal, wiced_interface_t interface );
wiced_result_t wiced_ssdp_internal_mcast_stop      ( wiced_ssdp_internal_t*  internal );
wiced_result_t wiced_ssdp_multicast_send_notify    ( wiced_ssdp_internal_t* internal, wiced_bool_t byebye );
wiced_result_t wiced_ssdp_mcast_build_default_reply( wiced_ssdp_internal_t* internal, wiced_ssdp_callback_info_t * cb_info );

wiced_result_t wiced_ssdp_dump_callback_info       ( wiced_ssdp_callback_info_t* cb_info );

#ifdef __cplusplus
}
#endif // __cplusplus
