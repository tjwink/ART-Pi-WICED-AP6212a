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

/**
 * @file  This defines internal functions which are used within the DNS protocol,
 *        and by Gedday and the DNS_redirect daemon
 *
 *        Customers should not use these functions directly as they are liable to change.
 */

#include "wiced_tcpip.h"
#include "wiced_time.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define DNS_MESSAGE_IS_A_RESPONSE           0x8000
#define DNS_MESSAGE_OPCODE                  0x7800
#define DNS_MESSAGE_AUTHORITATIVE           0x0400
#define DNS_MESSAGE_TRUNCATION              0x0200
#define DNS_MESSAGE_RECURSION_DESIRED       0x0100
#define DNS_MESSAGE_RECURSION_AVAILABLE     0x0080
#define DNS_MESSAGE_RESPONSE_CODE           0x000F

#define DNS_MESSAGE_RCODE_NXDOMAIN          (0x3)           /* Name does not exist */

#define RR_CACHE_FLUSH   0x8000

#define DNS_MAX_HOSTNAME_LEN                (255)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    DNS_NO_ERROR        = 0,
    DNS_FORMAT_ERROR    = 1,
    DNS_SERVER_FAILURE  = 2,
    DNS_NAME_ERROR      = 3,
    DNS_NOT_IMPLEMENTED = 4,
    DNS_REFUSED         = 5
} dns_message_response_code_t;

typedef enum
{
    RR_TYPE_A      = 1,
    RR_TYPE_NS     = 2,
    RR_TYPE_MD     = 3,
    RR_TYPE_MF     = 4,
    RR_TYPE_CNAME  = 5,
    RR_TYPE_SOA    = 6,
    RR_TYPE_MB     = 7,
    RR_TYPE_MG     = 8,
    RR_TYPE_MR     = 9,
    RR_TYPE_NULL   = 10,
    RR_TYPE_WKS    = 11,
    RR_TYPE_PTR    = 12,
    RR_TYPE_HINFO  = 13,
    RR_TYPE_MINFO  = 14,
    RR_TYPE_MX     = 15,
    RR_TYPE_TXT    = 16,
    RR_TYPE_AAAA   = 28,
    RR_TYPE_SRV    = 33,
    RR_TYPE_NSEC   = 47,
    RR_QTYPE_AXFR  = 252,
    RR_QTYPE_MAILB = 253,
    RR_QTYPE_AILA  = 254,
    RR_QTYPE_ANY   = 255
} dns_resource_record_type_t;

typedef enum
{
    RR_CLASS_IN  = 1,
    RR_CLASS_CS  = 2,
    RR_CLASS_CH  = 3,
    RR_CLASS_HS  = 4,
    RR_CLASS_ALL = 255
} dns_resource_record_class_t;



/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

#pragma pack(1)
typedef struct
{
    uint8_t* start_of_name;
    uint8_t* start_of_packet; /* Used for compressed names; */
} dns_name_t;

typedef struct
{
    uint16_t id;
    uint16_t flags;
    uint16_t question_count;
    uint16_t answer_count;
    uint16_t authoritative_answer_count;
    uint16_t additional_record_count;
} dns_message_header_t;

typedef struct
{
    dns_message_header_t* header; /* Also used as start of packet for compressed names */
    uint8_t*              iter;
    uint16_t              max_size;
    uint16_t              current_size;
} dns_message_iterator_t;

typedef struct
{
    uint16_t type;
    uint16_t class;
} dns_question_t;

typedef struct
{
    uint16_t  type;
    uint16_t  class;
    uint32_t  ttl;
    uint16_t  rd_length;
    uint8_t*  rdata;
} dns_record_t;

typedef struct
{
    const char* CPU;
    const char* OS;
} dns_hinfo_t;

typedef struct
{
    uint16_t  priority;
    uint16_t  weight;
    uint16_t  port;
    char      target[1]; /* Target name which will be larger than 1 byte */
} dns_srv_data_t;

typedef struct
{
    uint8_t block_number;
    uint8_t bitmap_size;
    uint8_t bitmap[1];
} dns_nsec_data_t;

typedef struct
{
    uint8_t block_number;
    uint8_t bitmap_size;
    uint8_t bitmap[1];
} dns_nsec_ipv4_only_t;

typedef struct
{
    uint8_t block_number;
    uint8_t bitmap_size;
    uint8_t bitmap[4];
} dns_nsec_ipv4_ipv6_t;

typedef struct
{
    uint8_t block_number;
    uint8_t bitmap_size;
    uint8_t bitmap[5];
} dns_nsec_service_t;

#pragma pack()

typedef struct
{
    char hostname[DNS_MAX_HOSTNAME_LEN];
    wiced_ip_address_t  ipv4_address;
    wiced_time_t        expiration_time;
    wiced_time_t        cache_time;
} dns_resolved_addr_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/* Functions to generate a custom DNS query */

/** Writes the question header using the provided iterator
 *
 * @param iter    Pointer to the iterator where to write the data
 * @param target  The name of the object or service being queried for, e.g. "device_name.local"
 * @param class   The class of the query, e.g. RR_CLASS_IN
 * @param type    The type of the query, e.g. RR_QTYPE_ANY
 *
 * @return @ref wiced_result_t
 *
 */
wiced_result_t   dns_write_question( dns_message_iterator_t* iter, const char* target, uint16_t class, uint16_t type );

wiced_result_t   dns_write_record  ( dns_message_iterator_t* iter, const char* name,   uint16_t class, uint16_t type, uint32_t TTL, const void* rdata );
void             dns_write_header  ( dns_message_iterator_t* iter, uint16_t id, uint16_t flags, uint16_t question_count, uint16_t answer_count, uint16_t authoritative_count, uint16_t additional_count );
wiced_result_t   dns_write_name    ( dns_message_iterator_t* iter, const char* src );
uint8_t*         dns_write_string  ( uint8_t* dest, const char* src );
wiced_result_t   dns_write_uint16  ( dns_message_iterator_t* iter, uint16_t data );
wiced_result_t   dns_write_uint32  ( dns_message_iterator_t* iter, uint32_t data );
wiced_result_t   dns_write_bytes   ( dns_message_iterator_t* iter, const uint8_t* data, uint16_t length );

/* Functions to aid processing received queries */
uint16_t        dns_read_uint16           ( const uint8_t* data );
uint32_t        dns_read_uint32           ( const uint8_t* data );
char*           dns_read_name             ( const uint8_t* data, uint16_t data_len, const dns_message_header_t* start_of_packet ); /* Note: This function returns a dynamically allocated string */
wiced_result_t  dns_get_next_question     ( dns_message_iterator_t* iter, uint8_t* iter_end, dns_question_t* q, dns_name_t* name );
wiced_result_t  dns_get_next_record       ( dns_message_iterator_t* iter, uint8_t* iter_end, dns_record_t* r,   dns_name_t* name );
void            dns_reset_iterator        ( dns_message_iterator_t* iter );
wiced_bool_t    dns_compare_name_to_string( const dns_name_t* name, const char* string );
wiced_result_t  dns_name_to_string        ( const dns_name_t* name, char* string_buf, uint8_t string_buf_len );
wiced_result_t  dns_skip_name             ( dns_message_iterator_t* iter, uint8_t* iter_end );

#ifdef __cplusplus
} /* extern "C" */
#endif
