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

#ifndef  __ALIYUN_PROTOCOL_H__
#define  __ALIYUN_PROTOCOL_H__

#include "aliyun_common.h"
#include "xml.h"
#include "base64.h"
#include "http_client.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define ALIYUN_ACCOUNT_ID   "YOUR ACCOUNT ID"
#define ALIYUN_REGION       "YOUR REGION (e.g. us-west-1)"
#define ALIYUN_ACCESS_KEY   "YOUR ACCESS KEY"
#define ALIYUN_SECRET_KEY   "YOUR SECRET KEY"
#define ALIYUN_QUEUE_NAME   "YOUR QUEUE NAME"

#define ALIYUN_XMLNS_URL    "http://mns.aliyuncs.com/doc/v1/"
#define ALIYUN_XMLNS        "xmlns=\"" ALIYUN_XMLNS_URL "\""

#define ALIYUN_CONTENT_TYPE "text/xml;charset=utf-8"
#define ALIYUN_XMNS_VERSION "2015-06-06"

#define ALIYUN_HOST "https://" ALIYUN_ACCOUNT_ID ".mns." ALIYUN_REGION ".aliyuncs.com"
#define ALIYUN_HOST_NO_HTTP ALIYUN_ACCOUNT_ID ".mns." ALIYUN_REGION ".aliyuncs.com"

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct{
    char *account_id;
    char *region;
    char *access_key;
    char *secret_key;
    char *queue_name;
}wiced_aliyun_queue_t;

typedef struct{
    char *xmns;
    char *date_string;
    char *sig_plain;
    char *signature;
    char *resource;
    char *authorization;
    char *content_length;
}wiced_aliyun_buffers_t;

typedef struct
{
    wiced_aliyun_queue_t *wiced_aliyun_queue;
    wiced_aliyun_buffers_t *wiced_aliyun_buffers;
    wiced_xml_t *wiced_xml;
    http_header_field_t *http_header;
    http_client_t *http_client;
}wiced_aliyun_t;

typedef enum{
    CREATEQUEUE,
    SETQUEUEATTRIBUTES,
    GETQUEUEATTRIBUTES,
    DELETEQUEUE,
    LISTQUEUE,
    SENDMESSAGE,
    BATCHSENDMESSAGE,
    RECEIVEMESSAGE,
    BATCHRECEIVEMESSAGE,
    DELETEMESSAGE,
    BATCHDELETEMESSAGE,
    PEEKMESSAGE,
    BATCHPEEKMESSAGE,
    CHANGEMESSAGEVISIBILITY
}wiced_aliyun_queue_function_t;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/** Initialize main structure for queue functions
 *
 * @param aliyun            : Main structure for queue funcitons
 * @param aliyun_queue      : Structure for account settings
 * @param aliyun_buffers    : Structure for buffers
 * @param xml               : Structure for XML payload
 * @param http_request      : HTTP request variable
 * @param http_header       : HTTP headers variable
 * @param http_client       : HTTP client variable
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_aliyun_init( wiced_aliyun_t *aliyun, wiced_aliyun_queue_t *aliyun_queue,  wiced_aliyun_buffers_t *aliyun_buffers, wiced_xml_t *xml, http_header_field_t *http_header, http_client_t *http_client );

/** Execute a queue functions
 *
 * @param aliyun            : Main structure for queue funcitons
 * @param queue_function    : Queue function to execute
 * @param option1           : Optional parameter 1 ( must be NULL if unused )
 * @param option2           : Optional parameter 2 ( must be NULL if unused )
 * @param option3           : Optional parameter 3 ( must be NULL if unused )
 *
 * |----------------------------|-------------------|-------------------|---------------|
 * |     Queue Function         |   Option 1        |   Option 2        |   Option 3    |
 * |----------------------------|-------------------|-------------------|---------------|
 * | CREATEQUEUE                |   N/A             |   N/A             |   N/A         |
 * | SETQUEUEATTRIBUTES         |   N/A             |   N/A             |   N/A         |
 * | GETQUEUEATTRIBUTES         |   N/A             |   N/A             |   N/A         |
 * | DELETEQUEUE                |   N/A             |   N/A             |   N/A         |
 * | LISTQUEUE                  | xmns_next_marker* | xmns_ret_number*  | xmns_prefix*  |
 * | SENDMESSAGE                |   N/A             |   N/A             |   N/A         |
 * | BATCHSENDMESSAGE           |   N/A             |   N/A             |   N/A         |
 * | RECEIVEMESSAGE             | wait_seconds*     |   N/A             |   N/A         |
 * | BATCHRECEIVEMESSAGE        | num_messages      | wait_seconds*     |   N/A         |
 * | DELETEMESSAGE              | receipt_handle    |   N/A             |   N/A         |
 * | BATCHDELETEMESSAGE         |   N/A             |   N/A             |   N/A         |
 * | PEEKMESSAGE                |   N/A             |   N/A             |   N/A         |
 * | BATCHPEEKMESSAGE           | num_messages      |   N/A             |   N/A         |
 * | CHANGEMESSAGEVISIBILITY    | receipt_handle    | visibility_timeout|   N/A         |
 * |----------------------------|-------------------|-------------------|---------------|
 *      * denotes optional parameter
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_aliyun_execute_function( wiced_aliyun_t *aliyun, http_request_t* request, wiced_aliyun_queue_function_t queue_function, char *option1, char *option2, char *option3);

/** Write xml declaration to xml buffer
 *
 * @param xml               : XML structure
 * @param queue_function    : Queue function to execute
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_xml_start( wiced_xml_t *xml, wiced_aliyun_queue_function_t queue_function );

/** Write xml end tag to xml buffer
 *
 * @param xml               : XML structure
 * @param queue_function    : Queue function to execute
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_xml_end( wiced_xml_t *xml, wiced_aliyun_queue_function_t queue_function );

/** Write xml message start tag to xml buffer
 *
 * @param xml           : XML structure
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_xml_start_message( wiced_xml_t *xml );

/** Write xml message end tag to xml buffer
 *
 * @param xml           : XML structure
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_xml_end_message( wiced_xml_t *xml );

/** Write xml start tag, element, end tag to xml buffer
 *
 * @param xml           : XML structure
 * @param parameter     : Tag Name
 * @param value         : XML element
 *
 * @return wiced_result_t
 */
wiced_result_t wiced_xml_add_parameter( wiced_xml_t *xml, char *parameter, char *value );


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  /* __ALIYUN_PROTOCOL_H__    */
