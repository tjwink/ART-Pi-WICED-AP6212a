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
#pragma once

#include "wiced_result.h"
#include "wiced_tcpip.h"
#include "wiced_rtos.h"
#include "linked_list.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define HTTP_CLRF        "\r\n"
#define HTTP_CRLF_CRLF   "\r\n\r\n"
#define HTTP_SPACE       " "
#define HTTP_COLON       ":"

#define HTTP_VERSION_1_0  "HTTP/1.0"
#define HTTP_VERSION_1_1  "HTTP/1.1"
#define HTTP_VERSION_2    "HTTP/2"

#define HTTP_METHOD_OPTIONS  "OPTIONS"
#define HTTP_METHOD_GET      "GET"
#define HTTP_METHOD_HEAD     "HEAD"
#define HTTP_METHOD_POST     "POST"
#define HTTP_METHOD_PUT      "PUT"
#define HTTP_METHOD_DELETE   "DELETE"
#define HTTP_METHOD_TRACE    "TRACE"
#define HTTP_METHOD_CONNECT  "CONNECT"

#define HTTP_HEADER_HOST            "Host: "
#define HTTP_HEADER_DATE            "Date: "
#define HTTP_HEADER_CONTENT_LENGTH  "Content-Length: "
#define HTTP_HEADER_CONTENT_TYPE    "Content-Type: "
#define HTTP_HEADER_CHUNKED         "Transfer-Encoding: chunked"

#define HTTP_HEADER_CONTENT_LEN_NO_COLON  "Content-Length"

/******************************************************
 *                   Enumerations
 ******************************************************/
/**
 * HTTP version that the client/server shall be configured with.
*/
typedef enum
{
    HTTP_1_0,
    HTTP_1_1,
    HTTP_2,
} http_version_t;

/**
 * HTTP RESTful methods per RFC2616
*/
typedef enum
{
    HTTP_UNKNOWN  = -1,
    HTTP_OPTIONS  =  0,
    HTTP_GET      =  1,
    HTTP_HEAD     =  2,
    HTTP_POST     =  3,
    HTTP_PUT      =  4,
    HTTP_DELETE   =  5,
    HTTP_TRACE    =  6,
    HTTP_CONNECT  =  7,

    HTTP_METHODS_MAX,   /* must be last! */
} http_method_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
/**
 * HTTP header fields
*/
typedef struct
{
    char*    field;        /* HTTP header field such as host, date, content_length etc. */
    uint16_t field_length; /* HTTP field length */
    char*    value;        /* Value corresponding to the HTTP header field */
    uint16_t value_length; /* HTTP field value length */
} http_header_field_t;

/**
 * HTTP status corresponding to a HTTP request
*/
typedef struct
{
    http_version_t version; /* HTTP version of the server */
    uint16_t       code;    /* HTTP status code */
} http_status_line_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/*****************************************************************************/
/**
 * @addtogroup http_helper      HTTP client helper
 * @ingroup http_client
 *
 * HTTP client utilitiy functions
 *
 *  @{
 */
/******************************************************
 *               Function Declarations
 ******************************************************/

/** Parse headers received in HTTP response.
 *
 * @param[in]  data                      : A pointer to the received response header.
 * @param[in]  length                    : Length of response header.
 * @param[in]  header                    : A pointer to http_header_field_t that will be matched against the response header.
 *                                                 Values of response headers will be filled.
 * @param[in]  number_of_header_fields   : Number of header fields pointed by header.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_parse_header( const uint8_t* data, uint16_t length, http_header_field_t* header, uint32_t number_of_header_fields );

/** Fetch the HTTP status code present in HTTP response.
 *
 * @param[in]  data         : A pointer to the HTTP response header.
 * @param[in]  length       : Length of the response header.
 * @param[out] status_line  : HTTP status code.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_get_status_line( const uint8_t* data, uint16_t length, http_status_line_t* status_line );

/** Split the HTTP header with the following HTTP header present in response.
 *
 * @param[in]  line         : A pointer to the response header received from server.
 * @param[in]  max_length   : Length of the response header.
 * @param[out] next_line    : Pointer to the subsequent header.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_split_line( const char* line, uint16_t max_length, char** next_line );

/** Fetch the next header present in HTTP response.
 *
 * @param[in]  line         : A pointer to the response header received in HTTP response.
 * @param[in]  max_length   : Length of the response header.
 * @param[out] next_line    : A pointer to the next header present in HTTP response.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_get_next_line( const char* line, uint16_t max_length, char** next_line );

/** Get the length of header present in HTTP response.
 *
 * @param[in]  line              : A pointer to the response header received in the HTTP response.
 * @param[in]  max_line_length   : Length of the response header.
 * @param[out] actual_length     : Total length of the HTTP header.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_get_line_length( const char* line, uint32_t max_line_length, uint32_t* actual_length );

/** Fetch the next header present in HTTP response with length.
 *
 * @param[in]  data              : A pointer to the HTTP response header received in the HTTP response.
 * @param[in]  data_length       : Length of the HTTP response header.
 * @param[out] next_line         : Pointer to the next header present in the HTTP response.
 * @param[out] line_length       : Length of the next HTTP response header.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_get_next_line_with_length( const char* data, uint16_t data_length, char** next_line, uint32_t* line_length );

/** Get the value of HTTP HOST header.
 *
 * @param[in]  line              : A pointer to the HTTP response header received in HTTP reseponse.
 * @param[in]  line_length       : Length of the HTTP response header.
 * @param[out] host              : Pointer to host name present in HTTP response header.
 * @param[out] host_length       : Length of host name.
 * @param[out] port              : Port number if present in HOST header; 0 otherwise.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_get_host( const char* line, uint16_t line_length, char** host, uint16_t* host_length, uint16_t* port );

/** Get the next string token.
 *
 * @param[in]  string            : A pointer to string.
 * @param[in]  string_length     : Length of the string.
 * @param[in]  delimeter         : String delimeter
 * @param[out] next_token        : Next token present in the string passed.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t http_get_next_string_token( const char* string, uint16_t string_length, char delimiter, char** next_token );

/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif
