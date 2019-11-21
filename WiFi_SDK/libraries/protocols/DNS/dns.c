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

#include "dns.h"
#include "dns_internal.h"
#include "string.h"
#include "wiced_network.h"
#include "wiced_tcpip.h"
#include "wwd_debug.h"
#include <wiced_utilities.h>
#include "wiced_time.h"
#include "wwd_assert.h"
#include "wiced_crypto.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define DNS_NAME_SIZE(name)                         (strlen((name)) + 1 + 1) /* one byte for a string length and one byte is a string null-terminator */
#define A_RECORD_RDATA_SIZE                         (4)
#define AAAA_RECORD_RDATA_SIZE                      (16)

/******************************************************
 *                    Constants
 ******************************************************/
#define DNS_PORT    (53)

#ifndef WICED_MAXIMUM_STORED_DNS_SERVERS
#define WICED_MAXIMUM_STORED_DNS_SERVERS (2)
#endif

#ifndef WICED_MAXIMUM_STORED_DNS_ADDRESSES
#define WICED_MAXIMUM_STORED_DNS_ADDRESSES  (0)
#endif

#define MS_PER_SECOND                       (1000)

/* Change to 1 to turn debug trace */
#define WICED_DNS_DEBUG   ( 0 )

#ifndef htons
#ifdef BIG_ENDIAN
#define htons(x)        ((uint16_t)(x))
#else
#define SWAP_SHORT( bytes ) ( ( ( ( bytes ) >> 8 ) & 0xff ) | \
    ( ( ( bytes ) << 8 ) & 0xff00 ) )
#define htons(x)        ((uint16_t)SWAP_SHORT((uint16_t)(x)))
#endif /* BIG_ENDIAN */
#endif /* htons */

#ifndef htonl
#ifdef BIG_ENDIAN
#define htonl(x)        ((uint32_t)(x))
#else
#define SWAP_LONG( bytes ) ( ( ( ( ( bytes ) & 0xff000000 ) >> 24 ) ) | \
    ( ( ( ( bytes ) & 0xff0000 ) >> 8 ) ) | \
    ( ( ( ( bytes ) & 0x00ff00 ) << 8 ) ) | \
    ( ( ( ( bytes ) & 0x000000ff ) << 24 ) ) )

#define htonl(x)        SWAP_LONG((uint32_t)(x))
#endif /* BIG_ENDIAN */
#endif /* htonl */

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

#if WICED_MAXIMUM_STORED_DNS_ADDRESSES
static wiced_result_t dns_check_address_cache(const char* hostname, wiced_ip_address_t* address);
static wiced_result_t dns_cache_address(const char* hostname, wiced_ip_address_t* address, uint32_t ttl);
#endif

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_ip_address_t dns_server_address_array[WICED_MAXIMUM_STORED_DNS_SERVERS];
static uint32_t           dns_server_address_count = 0;

#if WICED_MAXIMUM_STORED_DNS_ADDRESSES
static dns_resolved_addr_t dns_address_cache[WICED_MAXIMUM_STORED_DNS_ADDRESSES];
#endif


/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t dns_client_add_server_address( wiced_ip_address_t address )
{
    if ( dns_server_address_count >= WICED_MAXIMUM_STORED_DNS_SERVERS )
    {
        /* Server address array is full */
        return WICED_ERROR;
    }

    dns_server_address_array[dns_server_address_count++] = address;
    return WICED_SUCCESS;
}

wiced_result_t dns_client_remove_all_server_addresses( void )
{
    memset( dns_server_address_array, 0, sizeof( dns_server_address_array ) );
    dns_server_address_count = 0;
    return WICED_SUCCESS;
}

wiced_result_t dns_client_hostname_lookup( const char* hostname, wiced_resolved_ip_address_list* list, wiced_dns_lookup_address_type_t type, uint32_t timeout_ms, wiced_interface_t interface )
{
    uint32_t               a;
    wiced_packet_t*        packet;
    dns_message_iterator_t iter;
    uint8_t*               data_end;
    wiced_udp_socket_t     socket;
    wiced_result_t         result;
    uint32_t               remaining_time         = timeout_ms;
    wiced_bool_t           ipv6_address_found     = WICED_FALSE;
    wiced_ip_address_t     first_resolved_ipv6_addr = { WICED_INVALID_IP, .ip.v6 = {0} };
    wiced_ip_address_t     first_resolved_ipv4_addr = { WICED_INVALID_IP, .ip.v4 = 0 };
    char*                  temp_hostname          = (char*)hostname;
    uint16_t               hostname_length        = (uint16_t) strlen( hostname );
    uint16_t               id;
    dns_message_header_t*  p_aligned_header;
    int prefer_ipv4 = 0;
    int count = 0;
    int max_count;
    wiced_bool_t answer_found = WICED_FALSE;

#if WICED_MAXIMUM_STORED_DNS_ADDRESSES
    uint32_t               ttl = 0;
#endif

#if WICED_DNS_DEBUG
    wiced_time_t time;
    wiced_time_get_time( &time );
    WPRINT_LIB_INFO(("DNS lookup start @ %d\n\r", (int)time ));
#endif

#if WICED_MAXIMUM_STORED_DNS_ADDRESSES
    /* if we are caching - enforce 'single' entry from the cached list */
    if (dns_check_address_cache(hostname, &list->addresses[0] ) == WICED_SUCCESS)
    {
        list->count = 1;
        return WICED_SUCCESS;
    }
#endif

    if ( (type > WICED_DNS_LOOKUP_ADDR_ANY) || (type < WICED_DNS_LOOKUP_ADDR_IPV4) )
    {
        return WICED_BADARG;
    }

    /* if user has asked for single address and didn't specify which, we prefer sending IPv4 */
    if( list->count == 1 && type == WICED_DNS_LOOKUP_ADDR_ANY )
    {
        prefer_ipv4 = 1;
    }

    /* Create socket to send packets */
    if ( wiced_udp_create_socket( &socket, WICED_ANY_PORT, interface ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    max_count = list->count;
    while ( count < max_count && remaining_time > 0)
    {
        /* Send DNS query messages */
        for ( a = 0; a < dns_server_address_count; a++ )
        {
            wiced_ip_address_t host_ipv6_address;
            uint16_t           available_space;

            /* Create IPv4 query packet */
            packet = NULL;
            result =  wiced_packet_create_udp( &socket, (uint16_t) ( sizeof(dns_message_header_t) + sizeof(dns_question_t) + hostname_length ), &packet, (uint8_t**) &p_aligned_header, &available_space );
            iter.header = p_aligned_header;
            if ( result != WICED_SUCCESS )
            {
                goto exit;
            }

            iter.max_size = available_space;
            iter.current_size = 0;
            iter.iter = (uint8_t*) ( iter.header ) + sizeof(dns_message_header_t);

            wiced_crypto_get_random( &id, sizeof( id ) );
            dns_write_header( &iter, id, 0x100, 1, 0, 0, 0 );
            dns_write_question( &iter, temp_hostname, RR_CLASS_IN, RR_TYPE_A );
            wiced_packet_set_data_end( packet, iter.iter );

            /* Send IPv4 query packet */
            if ( wiced_udp_send( &socket, &dns_server_address_array[a], DNS_PORT, packet ) != WICED_SUCCESS )
            {
                goto exit;
            }
            packet = NULL;

            /* Check if host has global IPv6 address. If it does, send IPv6 query too */
            if ( wiced_ip_get_ipv6_address( interface, &host_ipv6_address, IPv6_GLOBAL_ADDRESS ) == WICED_SUCCESS )
            {
                /* Create IPv6 query packet */
                result = wiced_packet_create_udp( &socket, (uint16_t) ( sizeof(dns_message_header_t) + sizeof(dns_question_t) + hostname_length ), &packet, (uint8_t**) &p_aligned_header, &available_space );
                iter.header = p_aligned_header;
                if ( result != WICED_SUCCESS )
                {
                    goto exit;
                }

                iter.iter = (uint8_t*) ( iter.header ) + sizeof(dns_message_header_t);
                wiced_crypto_get_random( &id, sizeof( id ) );
                dns_write_header( &iter, id, 0x100, 1, 0, 0, 0 );
                dns_write_question( &iter, temp_hostname, RR_CLASS_IN, RR_TYPE_AAAA );
                wiced_packet_set_data_end( packet, iter.iter );

                /* Send IPv6 query packet */
                if ( wiced_udp_send( &socket, &dns_server_address_array[a], DNS_PORT, packet ) != WICED_SUCCESS )
                {
                    goto exit;
                }
                packet = NULL;
            }
        }

        /* Attempt to receive response packets */
        while ( count < max_count && remaining_time > 0 )
        {
            wiced_utc_time_ms_t current_time;
            wiced_utc_time_ms_t last_time;

            wiced_time_get_utc_time_ms( &last_time );

            packet = NULL;
            if ( wiced_udp_receive( &socket, &packet, remaining_time ) == WICED_SUCCESS )
            {
                uint16_t     data_length;
                uint16_t     available_data_length;

                /* Extract the data */
                p_aligned_header = NULL;
                result = wiced_packet_get_data( packet, 0, (uint8_t**) &p_aligned_header, &data_length, &available_data_length );
                if ( ( result != WICED_SUCCESS ) || ( data_length < available_data_length ) || ( data_length < sizeof(dns_message_header_t) ) )
                {
                    goto exit;
                }
                iter.header = p_aligned_header;
                iter.iter = (uint8_t*) ( iter.header ) + sizeof(dns_message_header_t);
                data_end = (uint8_t*) ( iter.header ) + data_length;

                /* Check if the message is a response (otherwise its a query) */
                if ( ntohs( iter.header->flags ) & DNS_MESSAGE_IS_A_RESPONSE )
                {
                    if ( ( ntohs(iter.header->flags ) & DNS_MESSAGE_RESPONSE_CODE ) == DNS_MESSAGE_RCODE_NXDOMAIN )
                    {
                        /*
                         * The DNS server is telling us that this host doesn't exist ... time to bail.
                         */

                        goto exit;
                    }

                    /* Skip all the questions */
                    for ( a = 0; a < htobe16( iter.header->question_count ); ++a )
                    {
                        /* Skip the name */
                        if ( dns_skip_name( &iter, data_end ) != WICED_SUCCESS )
                        {
                            goto exit;
                        }

                        /* Skip the type and class */
                        iter.iter += 4;
                    }

                    /* Process the answer */
                    for ( a = 0; (count < max_count) && ( a < htobe16( iter.header->answer_count ) ); ++a )
                    {
                        dns_name_t   name;
                        dns_record_t record;

                        if ( dns_get_next_record( &iter, data_end, &record, &name ) != WICED_SUCCESS )
                        {
                            goto exit;
                        }

                        switch ( record.type )
                        {
                            case RR_TYPE_CNAME:
                                /* Received an alias for our queried name. Restart DNS query for the new name */
                                if ( temp_hostname != hostname )
                                {
                                    /* Only free if we're certain we've malloc'd it*/
                                    free( (char*) temp_hostname );
                                }
                                temp_hostname = dns_read_name( record.rdata, record.rd_length, (dns_message_header_t*) name.start_of_packet );
                                if ( temp_hostname == NULL )
                                {
                                    goto exit;
                                }
                                hostname_length = (uint16_t) strlen( temp_hostname );

#if WICED_DNS_DEBUG
                                WPRINT_LIB_INFO( ("Found alias: %s\n", temp_hostname) );
#endif

                                break;

                            case RR_TYPE_A:
                            {
                                if( (type == WICED_DNS_LOOKUP_ADDR_IPV6) )
                                {
                                    break;
                                }

                                /* record the first IPv4 address from the answer records */
                                if( first_resolved_ipv4_addr.version == WICED_INVALID_IP )
                                {
                                    SET_IPV4_ADDRESS( first_resolved_ipv4_addr, htonl(*(uint32_t*)record.rdata ));
#if WICED_MAXIMUM_STORED_DNS_ADDRESSES
                                    ttl = record.ttl;
                                    dns_cache_address(hostname, &first_resolved_ipv4_addr, ttl);
#endif
                                }
                                answer_found = WICED_TRUE;

                                SET_IPV4_ADDRESS( list->addresses[count], htonl(*(uint32_t*)record.rdata) );

                                count++;

#if WICED_DNS_DEBUG
                                {
                                    uint8_t* ip = record.rdata;

                                    WPRINT_LIB_INFO( ( "Found IPv4 IP: %u.%u.%u.%u\n", ( ( ip[0] ) & 0xff ), ( ( ip[1] ) & 0xff ), ( ( ip[2] ) & 0xff ), ( ( ip[3] ) & 0xff ) ) );
                                }
#endif

                                break;
                            }

                            case RR_TYPE_AAAA:
                            {
                                uint8_t* aligned_rdata = record.rdata;
                                uint16_t* ip = (uint16_t*) ( aligned_rdata );

                                UNUSED_PARAMETER( ip );

                                if(type == WICED_DNS_LOOKUP_ADDR_IPV4)
                                {
                                    break;
                                }

                                /* record the first IPv6 address from the answer records */
                                if( first_resolved_ipv6_addr.version == WICED_INVALID_IP )
                                {
                                    SET_IPV6_ADDRESS( first_resolved_ipv6_addr, MAKE_IPV6_ADDRESS(htons(ip[0]), htons(ip[1]), htons(ip[2]), htons(ip[3]), htons(ip[4]), htons(ip[5]), htons(ip[6]), htons(ip[7])));
                                }

                                ipv6_address_found = WICED_TRUE;

                                /* if we reach here, the first answer record found was AAAA but AA is preferred - so ensure all the answer records are read to get AA record */
                                if( prefer_ipv4 )
                                {
                                    answer_found = WICED_FALSE;
                                }
                                else
                                {
                                    answer_found = WICED_TRUE;
                                    SET_IPV6_ADDRESS( list->addresses[count], MAKE_IPV6_ADDRESS(htons(ip[0]), htons(ip[1]), htons(ip[2]), htons(ip[3]), htons(ip[4]), htons(ip[5]), htons(ip[6]), htons(ip[7])));
                                    count++;
                                }

#if WICED_DNS_DEBUG
                                WPRINT_LIB_INFO( ( "Found IPv6 IP: %.4X:%.4X:%.4X:%.4X:%.4X:%.4X:%.4X:%.4X\n",
                                                   (unsigned int) ( ( htons(ip[0]) ) ),
                                                   (unsigned int) ( ( htons(ip[1]) ) ),
                                                   (unsigned int) ( ( htons(ip[2]) ) ),
                                                   (unsigned int) ( ( htons(ip[3]) ) ),
                                                   (unsigned int) ( ( htons(ip[4]) ) ),
                                                   (unsigned int) ( ( htons(ip[5]) ) ),
                                                   (unsigned int) ( ( htons(ip[6]) ) ),
                                                   (unsigned int) ( ( htons(ip[7]) ) ) ) );
#endif
                                break;
                            }

                            default:
                                break;
                        }
                    }

                    /* we were prefering for IPv4, but could not find it and instead found an IPv6 record */
                    if( prefer_ipv4 && (answer_found == WICED_FALSE) && (ipv6_address_found == WICED_TRUE) )
                    {
                        answer_found = WICED_TRUE;
                        list->addresses[count] = first_resolved_ipv6_addr;
                        count = 1;
                    }

                    /* if any IP-address record is found and we are done parsing all the answer records - skip waiting-out */
                    if( answer_found == WICED_TRUE )
                    {
                        max_count = MIN( max_count, count );
                    }

                    /* Skip any nameservers */
                    for ( a = 0; a < htobe16( iter.header->authoritative_answer_count ); ++a )
                    {
                        /* Skip the name */
                        if ( dns_skip_name( &iter, data_end ) != WICED_SUCCESS)
                        {
                            goto exit;
                        }

                        /* Skip the type, class and TTL */
                        iter.iter += 8;

                        /* Skip the data */
                        iter.iter += htobe16( *(uint16_t*) iter.iter ) + 2;
                    }

                    /* If no answer found yet they provided some additional A records we'll assume they are nameservers and try again */
                    if ( answer_found == WICED_FALSE && htobe16( iter.header->additional_record_count ) != 0 && htobe16( iter.header->authoritative_answer_count ) != 0 )
                    {
                        dns_name_t   name;
                        dns_record_t record;

                        /* Skip additional record */
                        if ( dns_get_next_record( &iter, data_end, &record, &name ) != WICED_SUCCESS )
                        {
                            goto exit;
                        }
                    }
                }

                wiced_packet_delete( packet );
                packet = NULL;
            }

            wiced_time_get_utc_time_ms( &current_time );
            if ( ( current_time - last_time ) > remaining_time )
            {
                remaining_time = 0;
            }
            else
            {
                remaining_time = remaining_time - (uint32_t) ( current_time - last_time );
            }
        }
    }

exit:
    if (packet != NULL)
    {
        wiced_packet_delete( packet );
        packet = NULL;
    }
    wiced_udp_delete_socket( &socket );

    if ( temp_hostname != hostname )
    {
        /* Only free if we're certain we've malloc'd it*/
        free( (char*) temp_hostname );
    }

#if WICED_DNS_DEBUG
    wiced_time_get_time( &time );
    WPRINT_LIB_INFO(("DNS ends @ %d\n\r", (int)time ));
#endif

    list->count = count;

    if( answer_found )
    {
        return WICED_SUCCESS;
    }

    return WICED_ERROR;
}


#if WICED_MAXIMUM_STORED_DNS_ADDRESSES
static wiced_result_t dns_check_address_cache(const char* hostname, wiced_ip_address_t* address)
{
    wiced_time_t current_time;
    size_t len;
    int i;

    /*
     * Sanity checking.
     */

    if (hostname == NULL || address == NULL)
    {
        return WICED_ERROR;
    }

    wiced_time_get_time(&current_time);

    len = strlen(hostname);

    /*
     * Loop through the cache and see if we have a match.
     */

    for (i = 0; i < WICED_MAXIMUM_STORED_DNS_ADDRESSES; i++)
    {
        if (dns_address_cache[i].expiration_time > 0)
        {
            /*
             * We have an entry in the cache. Is it still valid?
             */

            if (current_time < dns_address_cache[i].expiration_time)
            {
                /*
                 * Yep, let's see if it matches.
                 */

                if ((len == strlen(dns_address_cache[i].hostname)) && (strcasecmp(hostname, dns_address_cache[i].hostname) == 0))
                {
                    /*
                     * We have a match!
                     */

                    *address = dns_address_cache[i].ipv4_address;
                    return WICED_SUCCESS;
                }
            }
            else
            {
                /*
                 * This entry has expired. Clear it out.
                 */

                memset(&dns_address_cache[i], 0, sizeof(dns_address_cache[i]));
            }
        }
    }

    return WICED_ERROR;
}


static wiced_result_t dns_cache_address(const char* hostname, wiced_ip_address_t* address, uint32_t ttl)
{
    wiced_time_t current_time;
    wiced_time_t oldest_time;
    int oldest_index;
    int i;

    wiced_time_get_time(&current_time);

    /*
     * Loop through the cache and find an empty entry or the oldest entry.
     */

    oldest_index = 0;
    oldest_time  = (uint32_t)(-1);
    for (i = 0; i < WICED_MAXIMUM_STORED_DNS_ADDRESSES; i++)
    {
        if (dns_address_cache[i].expiration_time == 0 || dns_address_cache[i].expiration_time > current_time)
        {
            /*
             * We can use this entry.
             */

            break;
        }

        if (dns_address_cache[i].cache_time < oldest_time)
        {
            oldest_index = i;
            oldest_time  = dns_address_cache[i].cache_time;
        }
    }

    if (i == WICED_MAXIMUM_STORED_DNS_ADDRESSES)
    {
        i = oldest_index;
    }

    strlcpy(dns_address_cache[i].hostname, hostname, DNS_MAX_HOSTNAME_LEN);
    dns_address_cache[i].ipv4_address    = *address;
    dns_address_cache[i].expiration_time = current_time + (ttl * MS_PER_SECOND);
    dns_address_cache[i].cache_time      = current_time;

    return WICED_SUCCESS;
}
#endif


/*********************************
 * DNS packet creation functions
 *********************************/


wiced_result_t dns_write_question(dns_message_iterator_t* iter, const char* target, uint16_t class, uint16_t type)
{
    wiced_result_t result;

    /* Write target name, type and class */
    result = dns_write_name( iter, target );
    wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );

    result = dns_write_uint16( iter, type );
    wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );

    result = dns_write_uint16( iter, class );
    wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );

    return WICED_SUCCESS;
error_exit:
    wiced_assert("Write DNS question failed, potential packet buffer overflow", 0!=0);
    return result;
}

wiced_result_t dns_write_uint16 ( dns_message_iterator_t* iter, uint16_t data )
{
    if ( ( iter->current_size + 2 ) <= iter->max_size )
    {
        /* We cannot assume the uint8_t alignment of iter->iter so we can't just type cast and assign */
        iter->iter[ 0 ]    = (uint8_t) ( data >> 8 );
        iter->iter[ 1 ]    = (uint8_t) ( data & 0xFF );
        iter->iter         += 2;
        iter->current_size = (uint16_t) ( iter->current_size + 2 );
        return WICED_SUCCESS;
    }
    else
    {
        return WICED_PACKET_BUFFER_OVERFLOW;
    }
}

wiced_result_t dns_write_uint32 ( dns_message_iterator_t* iter, uint32_t data )
{
    if ( ( iter->current_size + 2 ) <= iter->max_size )
    {
        iter->iter[ 0 ]      = (uint8_t) ( data >> 24 );
        iter->iter[ 1 ]      = (uint8_t) ( data >> 16 );
        iter->iter[ 2 ]      = (uint8_t) ( data >>  8 );
        iter->iter[ 3 ]      = (uint8_t) ( data & 0xFF);
        iter->current_size   = (uint16_t) ( iter->current_size + 4 );
        iter->iter           += 4;
        return WICED_SUCCESS;
    }
    else
    {
        return WICED_PACKET_BUFFER_OVERFLOW;
    }
}

wiced_result_t dns_write_bytes ( dns_message_iterator_t* iter, const uint8_t* data, uint16_t length )
{
    int a;
    if ( ( iter->current_size + length ) <= iter->max_size )
    {
        for (a = 0; a < length; ++a)
        {
            iter->iter[a] = data[a];
        }
        iter->iter         += length;
        iter->current_size = (uint16_t) (iter->current_size + length);
        return WICED_SUCCESS;
    }
    else
    {
        return WICED_PACKET_BUFFER_OVERFLOW;
    }
}

/*
 * DNS packet processing functions
 */
wiced_result_t dns_get_next_question(dns_message_iterator_t* iter, uint8_t* iter_end, dns_question_t* q, dns_name_t* name)
{
    /* Set the name pointers and then skip it */
    name->start_of_name   = (uint8_t*) iter->iter;
    name->start_of_packet = (uint8_t*) iter->header;
    if ( dns_skip_name(iter, iter_end) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* Read the type and class */
    q->type  = dns_read_uint16(iter->iter);
    q->class = dns_read_uint16(iter->iter + 2);
    iter->iter += 4;
    return (iter->iter > iter_end) ? WICED_ERROR : WICED_SUCCESS;
}

wiced_result_t dns_get_next_record(dns_message_iterator_t* iter, uint8_t* iter_end, dns_record_t* r, dns_name_t* name)
{
    /* Set the name pointers and then skip it */
    name->start_of_name   = (uint8_t*) iter->iter;
    name->start_of_packet = (uint8_t*) iter->header;
    if ( dns_skip_name(iter, iter_end) != WICED_SUCCESS)
    {
        return WICED_ERROR;
    }

    /* Set the record values and the rdata pointer */
    r->type      = dns_read_uint16(iter->iter);
    r->class     = dns_read_uint16(iter->iter + 2);
    r->ttl       = dns_read_uint32(iter->iter + 4);
    r->rd_length = dns_read_uint16(iter->iter + 8);
    iter->iter += 10;
    r->rdata     = iter->iter;

    /* Skip the rdata */
    iter->iter += r->rd_length;
    return (iter->iter > iter_end) ? WICED_ERROR : WICED_SUCCESS;
}

void dns_reset_iterator( dns_message_iterator_t* iter )
{
    iter->iter = (uint8_t*) ( iter->header ) + sizeof(dns_message_header_t);
}

#if 0 /* unreferenced */
void dns_read_bytes(dns_message_iterator_t* iter, uint8_t* dest, uint16_t length)
{
    memcpy(iter->iter, dest, length);
    iter->iter += length;
}
#endif /* if 0 unreferenced */


wiced_bool_t dns_compare_name_to_string( const dns_name_t* name, const char* string )
{
    wiced_bool_t    finished = WICED_FALSE;
    wiced_bool_t    result = WICED_TRUE;
    uint8_t*        buffer = name->start_of_name;
    uint8_t         max_octets;

    max_octets = UINT8_MAX;
    while (!finished)
    {
        uint8_t sectionLength;
        int a;
        uint16_t offset;

        /* Check if the name is compressed. If so, find the uncompressed version */
        while ((*buffer & 0xC0) == 0xC0)
        {
            if (max_octets == 0)
            {
                /* At worst case, total number of octets can't be more than 255. Something wrong, return as failed */
                return  WICED_FALSE;
            }

            offset = (uint16_t) ( (*buffer++) << 8 );
            offset = (uint16_t) ( offset + *buffer );
            offset &= 0x3FFF;
            buffer = name->start_of_packet + offset;
            max_octets--;
        }

        /* Compare section */
        sectionLength = *(buffer++);
        if (sectionLength > 63)
        {
            /* Section length shouldn't be more than 63 bytes. Something wrong. Ref: RFC 1035. section 4.1.4. Message compression */
            return WICED_FALSE;
        }
        for (a=sectionLength; a != 0; --a)
        {
            uint8_t b = *buffer++;
            char c = *string++;
            if (b >= 'a' && b <= 'z')
            {
                b = (uint8_t) ( b - ('a' - 'A') );
            }
            if (c >= 'a' && c <= 'z')
            {
                c = (char) ( c - ('a' - 'A') );
            }
            if (b != c)
            {
                result   = WICED_FALSE;
                finished = WICED_TRUE;
                break;
            }
        }

        /* Check if we've finished comparing */
        if (finished == WICED_FALSE && (*buffer == 0 || *string == 0))
        {
            finished = WICED_TRUE;
            /* Check if one of the strings has more data */
            if (*buffer != 0 || *string != 0)
            {
                result = WICED_FALSE;
            }
        }
        else
        {
            /* Skip '.' */
            ++string;
        }
    }

    return result;
}

wiced_result_t dns_name_to_string( const dns_name_t* name, char* string_buf, uint8_t string_buf_len )
{
    wiced_bool_t finished = WICED_FALSE;
    wiced_result_t result = WICED_SUCCESS;
    uint8_t* buffer = name->start_of_name;
    char* string_buf_end;
    uint8_t max_octets;

    if (string_buf_len == 0)
    {
        return WICED_ERROR;
    }

    string_buf_end = string_buf + (string_buf_len - 1);
    max_octets = UINT8_MAX;
    while (!finished)
    {
        uint8_t sectionLength;
        int a;

        /* Check if the name is compressed. If so, find the uncompressed version */
        while ((*buffer & 0xC0) == 0xC0)
        {
            uint16_t offset;
            if (max_octets == 0)
            {
                /* At worst case, total number of octets can't be more than 255. Something wrong, return as failed */
                return  WICED_ERROR;
            }

            offset = (uint16_t) ( (*buffer++) << 8 );
            offset = (uint16_t) ( offset + *buffer );
            offset &= 0x3FFF;
            buffer = name->start_of_packet + offset;
            max_octets--;
        }

        /* Copy the section */
        sectionLength = *(buffer++);
        if (sectionLength > 63)
        {
            /* Section length shouldn't be more than 63 bytes. Something wrong. Ref: RFC 1035. section 4.1.4. Message compression */
            return WICED_ERROR;
        }

        for (a=sectionLength; a != 0; --a)
        {
            if (string_buf >= string_buf_end)
            {
                finished = WICED_TRUE;
                break;
            }
            *string_buf++ = (char)*buffer++;
        }

        /* Check if we've finished copying */
        if (finished == WICED_FALSE && (*buffer == 0))
        {
            finished = WICED_TRUE;
        }
        else
        {
            if (string_buf >= string_buf_end)
            {
                break;
            }
            /* Copy '.' */
            *string_buf++ = '.';
        }
    }
    *string_buf++ = '\0';

    return result;
}

/* Validates DNS name and finds length of the uncompressed name if DNS name is valid */
static wiced_result_t dns_validate_name( const uint8_t* data, const uint8_t* data_end, const uint8_t* start_of_packet, uint16_t* uncompressed_name_length)
{
    uint16_t       length = 0;
    uint8_t        sectionLength;
    uint8_t        max_octets;

    /* FIXME: <root> name record handling to be added */
    /* Find out length of the name and check if the name is with in the packet range */
    max_octets = UINT8_MAX;
    while ((data < data_end) && (*data != 0))
    {
        if (max_octets == 0)
        {
            /* At worst case, total number of octets can't be more than 255. Something wrong, return as failed */
            return  WICED_ERROR;
        }

        /* Check if the name is compressed */
        if ((*data & 0xC0) == 0xC0)
        {
            uint16_t offset = (uint16_t) ( (*data++) << 8 );
            if (data >= data_end)
            {
                return WICED_ERROR;
            }
            offset = (uint16_t) ( offset + *data );
            offset &= 0x3FFF;
            data = (uint8_t*) start_of_packet + offset;
        }
        else
        {
            sectionLength = *data;
            if (sectionLength > 63)
            {
                /* Section length shouldn't be more than 63 bytes. Something wrong. Ref: RFC 1035. section 4.1.4. Message compression */
                return WICED_ERROR;
            }
            length = (uint16_t) ( length + *data + 1 ); /* +1 for the '.', unless its the last section in which case its the ending null character */
            data += *data + 1;
        }
        max_octets--;
    }

    /* Check if the name is with in the packet range */
    if ((length == 0) || (data >= data_end))
    {
        return WICED_ERROR;
    }

    *uncompressed_name_length = length;
    return WICED_SUCCESS;
}

wiced_result_t dns_skip_name(dns_message_iterator_t* iter, uint8_t* iter_end)
{
    wiced_result_t  res = WICED_SUCCESS;
    const uint8_t*  start_of_packet = (const uint8_t*)iter->header;
    uint16_t        length = 0;

    /* Skip <root> name record */
    if( *iter->iter == 0 )
    {
        /* Skip the null byte */
        ++iter->iter;
        return WICED_SUCCESS;
    }

    /* Validate name */
    res = dns_validate_name((const uint8_t*)iter->iter, iter_end, start_of_packet, &length);
    if (res != WICED_SUCCESS)
    {
        return res;
    }

    /* Skip name */
    while ((iter->iter < iter_end) && (*iter->iter != 0))
    {
        /* Check if the name is compressed */
        if ((*iter->iter & 0xC0) == 0xC0)
        {
            iter->iter += 1; /* Normally this should be 2, but we have a ++ outside the while loop */
            break;
        }
        else
        {
            iter->iter += (uint32_t) *iter->iter + 1;
        }
    }
    /* Skip the null byte */
    ++iter->iter;
    return WICED_SUCCESS;
}

char* dns_read_name( const uint8_t* data, uint16_t data_len, const dns_message_header_t* start_of_packet )
{
    wiced_result_t res = WICED_SUCCESS;
    uint16_t       length = 0;
    const uint8_t* buffer;
    char*          string;
    char*          stringIter;
    const uint8_t* buf_end;
    uint8_t        sectionLength;

    /* FIXME: <root> name record handling to be added */
    /* Validate name and calculate uncompressed name length */
    buffer = data;
    buf_end = data + data_len;
    res = dns_validate_name(buffer, buf_end, (uint8_t*)start_of_packet, &length);
    if (res != WICED_SUCCESS)
    {
        return NULL;
    }

    /* Allocate memory for DNS name */
    string = malloc_named("dns", length);
    if ( string == NULL )
    {
        return NULL;
    }
    stringIter = string;

    buffer = data;
    while (*buffer != 0)
    {
        /* Check if the name is compressed. If yes, goto uncompressed version */
        if ((*buffer & 0xC0) == 0xC0)
        {
            uint16_t offset = (uint16_t) ( (*buffer++) << 8 );
            offset = (uint16_t) ( offset + *buffer );
            offset &= 0x3FFF;
            buffer = (uint8_t*) start_of_packet + offset;
        }
        else
        {
            /* Copy the section of the name */
            sectionLength = *(buffer++);
            memcpy(stringIter, buffer, sectionLength);
            stringIter += sectionLength;
            buffer += sectionLength;

            /* Assume there is a next section and add a '.' char */
            *stringIter++ = '.';
        }
    }
    /* null terminate */
    stringIter--;
    *stringIter = '\0';

    return string;
}

void dns_write_header(dns_message_iterator_t* iter, uint16_t id, uint16_t flags, uint16_t question_count, uint16_t answer_count, uint16_t authoritative_count, uint16_t additional_count)
{
    memset(iter->header, 0, sizeof(dns_message_header_t));
    iter->header->id                         = htons(id);
    iter->header->flags                      = htons(flags);
    iter->header->question_count             = htons(question_count);
    iter->header->authoritative_answer_count = htons(authoritative_count);
    iter->header->answer_count               = htons(answer_count);
    iter->header->additional_record_count    = htons(additional_count);
    iter->current_size                       = sizeof(dns_message_header_t);
}

wiced_result_t dns_write_record ( dns_message_iterator_t* iter, const char* name, uint16_t class, uint16_t type, uint32_t TTL, const void* rdata )
{
    uint8_t*       rd_length;
    uint8_t*       tempPtr;
    wiced_result_t result = WICED_SUCCESS;

    /* Write the name, type, class, TTL */
    result = dns_write_name( iter, name );
    wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );

    result = dns_write_uint16( iter, type );
    wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );

    result = dns_write_uint16( iter, class );
    wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );

    result = dns_write_uint32( iter, TTL );
    wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );

    /* Keep track of where we store the rdata length */
    wiced_action_jump_when_not_true( ( ( iter->current_size + 2 ) <= iter->max_size ), error_exit, result = WICED_PACKET_BUFFER_OVERFLOW );
    rd_length          = iter->iter;
    iter->iter         += 2;
    tempPtr            = iter->iter;
    iter->current_size = (uint16_t) (iter->current_size + 2);

    /* Append RDATA */
    switch (type)
    {
        case RR_TYPE_A:
                result = dns_write_bytes( iter, rdata, A_RECORD_RDATA_SIZE );
                wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );
            break;

        case RR_TYPE_AAAA:
                result = dns_write_bytes( iter, rdata, AAAA_RECORD_RDATA_SIZE );
                wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );
            break;

        case RR_TYPE_PTR:
                result = dns_write_name(iter, (const char*) rdata);
                wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );
            break;

        case RR_TYPE_TXT:
            if ( rdata != NULL )
            {
                //iter->iter = dns_write_string(iter->iter, rdata);
                memcpy( iter->iter, (char*)( rdata ), strlen( ( char* )rdata )  );
                iter->iter        += strlen( ( char* )rdata );
                iter->current_size = (uint16_t) ( iter->current_size + strlen( ( char* )rdata ) );
            }
            else
            {
                *iter->iter++ = 0;
                iter->current_size = (uint16_t) (iter->current_size + 1);
            }
            break;

        case RR_TYPE_SRV:
        {
            dns_srv_data_t* srv_rdata = (dns_srv_data_t*) rdata;
            result = dns_write_uint16( iter, srv_rdata->priority );
            wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );

            result = dns_write_uint16( iter, srv_rdata->weight   );
            wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );

            result = dns_write_uint16( iter, srv_rdata->port     );
            wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );

            /* Write the hostname */
            result = dns_write_name( iter, srv_rdata->target );
            wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );
            break;
        }

        case RR_TYPE_HINFO:
            iter->iter = dns_write_string(iter->iter, ((dns_hinfo_t*) rdata)->CPU);
            iter->iter = dns_write_string(iter->iter, ((dns_hinfo_t*) rdata)->OS);
            break;

        case RR_TYPE_NSEC:
        {
            dns_nsec_data_t* nsec_data = (dns_nsec_data_t*)rdata;
            result = dns_write_name( iter, name );
            wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );

            result = dns_write_bytes( iter, &nsec_data->block_number,  (uint16_t) sizeof( nsec_data->block_number ) );
            wiced_jump_when_not_true( result == WICED_SUCCESS, error_exit );
            break;
        }

        default:
            break;
    }

    /* Write the rdata length */
    rd_length[0] = (uint8_t) ( (iter->iter - tempPtr) >> 8 );
    rd_length[1] = (uint8_t) ( (iter->iter - tempPtr) & 0xFF );
    return WICED_SUCCESS;

error_exit:
    wiced_assert("Write record failed, potential packet buffer overflow", 0!=0);
    return result;
}

wiced_result_t dns_write_name ( dns_message_iterator_t* iter, const char* src )
{
    if ( ( iter->current_size + DNS_NAME_SIZE(src) ) <= iter->max_size )
    {
        iter->iter = dns_write_string( iter->iter, src );

        /* Add the ending null */
        *iter->iter++ = 0;

        /* string + 1 byte for null terminator and + 1 byte for string length */
        iter->current_size = ( uint16_t ) ( iter->current_size + DNS_NAME_SIZE(src) );
        return WICED_SUCCESS;
    }
    else
    {
        return WICED_PACKET_BUFFER_OVERFLOW;
    }
}

uint8_t* dns_write_string(uint8_t* dest, const char* src)
{
    uint8_t* segmentLengthPointer;
    uint8_t segmentLength;

    while (*src != 0)
    {
        /* Remember where we need to store the segment length and reset the counter */
        segmentLengthPointer = dest++;
        segmentLength = 0;

        /* Copy bytes until '.' or end of string */
        while (*src != '.' && *src != 0)
        {
            *dest++ = (uint8_t) *src++;
            ++segmentLength;
        }

        /* Store the length of the segment*/
        *segmentLengthPointer = segmentLength;

        /* Check if we stopped because of a '.', if so, skip it */
        if (*src == '.')
        {
            ++src;
        }
    }

    return dest;
}

uint16_t dns_read_uint16( const uint8_t* data )
{
    return htons(*((uint16_t*)data));
}

uint32_t dns_read_uint32( const uint8_t* data )
{
    return htonl(*((uint32_t*)data));
}

