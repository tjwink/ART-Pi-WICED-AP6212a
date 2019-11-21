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

#ifdef __cplusplus
extern "C"
{
#endif

#include "wiced.h"
#include "wiced_rtos.h"
#include "bt_types.h"

#define HTTP_URI_LEN 200
#define HTTP_HEADER_LEN 200
#define HTTP_BODY_LEN 500

#define SERVER_PORT_HTTPS          ( 443 )
#define SERVER_PORT_HTTP           ( 80 )
#define CONNECT_TIMEOUT_MS         ( 15000 )
#define HTTP_REQUEST_TIMEOUT_VALUE (60000)
#define TOTAL_REQUESTS             ( 2 )

#define MAX_RETRY_COUNT_FOR_DNS_LOOKUP 5

typedef struct
{
        uint8_t http_proxy_control;
        uint8_t http_proxy_status;
        uint8_t https_security;
        uint8_t pad1;
        uint8_t http_proxy_uri[ HTTP_URI_LEN ];
        uint8_t http_proxy_header[ HTTP_HEADER_LEN ];
        uint8_t http_proxy_body[ HTTP_BODY_LEN ];
} http_proxy_info_t;
typedef struct
{
        BD_ADDR remote_addr;              // remote peer device address
        uint8_t pad[2];
        uint32_t timer_count;              // timer count
        uint32_t fine_timer_count;         // fine timer count
        uint16_t conn_id;                  // connection ID referenced by the stack
        uint16_t peer_mtu;                 // peer MTU
        uint8_t num_to_write;             // num msgs to send, incr on each button intr
        uint8_t flag_indication_sent;     // indicates waiting for ack/cfm
        uint8_t flag_stay_connected;      // stay connected or disconnect after all messages are sent
        uint8_t battery_level;            // dummy battery level
        http_proxy_info_t http_proxy_info;
} iot_gateway_device_t;

typedef enum
{
    HTTP_REQ_NULL = 0, HTTP_REQ_GET = 1, HTTP_REQ_HEAD, HTTP_REQ_POST, HTTP_REQ_PUT, HTTP_REQ_DELETE, HTTPS_REQ_GET, HTTPS_REQ_HEAD, HTTPS_REQ_POST, HTTPS_REQ_PUT, HTTPS_REQ_DELETE, HTTP_REQ_CANCEL, HTTP_REQ_OBSERVE

} http_control_point;


typedef enum
{
    REQUEST_POST = 0,
    REQUEST_OBSERVE,
} http_request_type;

/* queue for httpservice node*/
wiced_queue_t httpservq;
#define EVENT_QUEUE_DEPTH              (10)

#define IOT_GATEWAY_GATTS_MAX_CONN     4

iot_gateway_device_t iot_gateway_devices[ IOT_GATEWAY_GATTS_MAX_CONN ];

wiced_result_t init_http_client(void);

#ifdef __cplusplus
}
#endif
