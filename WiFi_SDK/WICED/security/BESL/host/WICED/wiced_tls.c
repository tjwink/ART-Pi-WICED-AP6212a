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

/** @file
 *
 */
#include "wwd_constants.h"
#include "wiced_tls.h"
#include "wiced_utilities.h"
#include "tls_host_api.h"
#include "wiced_crypto.h"
#include "crypto_constants.h"
#include "crypto_structures.h"
#include "wiced_time.h"
#include "wwd_assert.h"
#include "network/wwd_buffer_interface.h"
#include "besl_host_interface.h"
#include "wiced_security_internal.h"
#include "internal/wiced_internal_api.h"
#include "wiced_tcpip_tls_api.h"
#include "wiced_tcpip.h"
#include <string.h>
#include "tls_cipher_suites.h"
#include "wiced_defaults.h"

#ifndef WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY
#include "wiced_supplicant.h"
#endif /* WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY */

#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/debug.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/cipher.h"
#include "mbedtls/ssl_cache.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#ifndef WICED_TLS_MAX_RESUMABLE_SESSIONS
#define WICED_TLS_MAX_RESUMABLE_SESSIONS    4
#endif

/* Maximum number of session reconnects */
#define MAX_TLS_SESSION_AGE     32

#define SSL_IS_CLIENT            0
#define SSL_IS_SERVER            1
#define SESSION_NO_TIMEOUT       0

#define MAX_HANDSHAKE_WAIT       20000
#define TLS_SESSION_TIMEOUT      1000000

/* disabling enterprise security feature to compile */
//#define WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY

#define DFL_FORCE_CIPHER             0
#define DFL_ALLOW_LEGACY            -2
#define MAX_ALPN_PROTOCOLS_LIST     10

#if defined(MBEDTLS_SSL_ALPN)
    char *alpn_list[MAX_ALPN_PROTOCOLS_LIST];
#endif

#if defined(MBEDTLS_SSL_CACHE_C) && defined(MBEDTLS_SSL_SRV_C  )
    mbedtls_ssl_cache_context cache;
#endif
    void mbedtls_eap_ssl_context_reinit(mbedtls_ssl_context* ssl);


#if defined(WICED_TLS_CLI_CACHE_SESSION) && defined(MBEDTLS_SSL_CLI_C)

typedef struct wiced_ssl_cache_entry wiced_ssl_cache_entry;
typedef struct wiced_ssl_cache_context wiced_ssl_cache_context;
typedef struct wiced_ip_port wiced_ip_port;

struct wiced_ip_port
{
    wiced_ip_address_t      ip_address;
    UINT                    port_num;
};
struct wiced_ssl_cache_entry
{
    linked_list_node_t      this_node;
    wiced_ip_port           ip_port;
    mbedtls_ssl_session     tls_session;
};

struct wiced_ssl_cache_context
{
    int             max_entries;
    wiced_mutex_t   mutex;
    linked_list_t   cache_list;
};

wiced_ssl_cache_context wiced_ssl_cache;

static void wiced_ssl_cache_init( wiced_ssl_cache_context *cache )
{
    memset( cache, 0, sizeof( wiced_ssl_cache_context ) );
    cache->max_entries = WICED_TLS_CLI_CACHE_ENTRIES;
    wiced_rtos_init_mutex(&cache->mutex);
    linked_list_init( &cache->cache_list);
}

static wiced_bool_t compare_ip_port( linked_list_node_t* node_to_compare, void* user_data )
{
    wiced_bool_t result = WICED_FALSE;
    wiced_ssl_cache_entry* cache_entry = (wiced_ssl_cache_entry*) node_to_compare;
    wiced_ip_port  *ip_port = (wiced_ip_port*) user_data;

    if (ip_port == NULL)
        return result;

    if (ip_port->port_num == cache_entry->ip_port.port_num &&
            memcmp( &ip_port->ip_address, &cache_entry->ip_port.ip_address, sizeof(wiced_ip_address_t) ) == 0 )
    {
        result = WICED_TRUE;
    }

    return result;
}

wiced_result_t get_ssl_cache_entry(wiced_ip_address_t *ip_address, UINT port, wiced_ssl_cache_entry **entry)
{
    wiced_result_t result;
    wiced_ip_port  ip_port;

    ip_port.port_num = port;
    memcpy ( &ip_port.ip_address, ip_address, sizeof (wiced_ip_address_t) );

    result = linked_list_find_node( &wiced_ssl_cache.cache_list, compare_ip_port, (void*) &ip_port, (linked_list_node_t**) ( entry ) );
    if( result != WICED_SUCCESS )
    {
        *entry = NULL;
    }

    return result;
}

wiced_result_t add_ssl_cache_entry(wiced_ip_address_t *ip_address, UINT port, mbedtls_ssl_session *tls_session)
{
    wiced_ssl_cache_entry *entry = NULL;
    wiced_result_t result = WICED_ERROR;;

    entry = (wiced_ssl_cache_entry *) calloc ( 1,  sizeof(wiced_ssl_cache_entry ) );
    if ( entry != NULL)
    {
        entry->ip_port.port_num = port;
        memcpy ( &entry->ip_port.ip_address, ip_address, sizeof (wiced_ip_address_t) );
        memcpy ( &entry->tls_session, tls_session, sizeof (mbedtls_ssl_session) );

        result = linked_list_insert_node_at_front( &wiced_ssl_cache.cache_list, &entry->this_node );
    }

    return result;
}
#endif


/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
options opt_config;
/******************************************************
 *               Static Function Declarations
 ******************************************************/
extern wiced_tls_key_type_t type;

int wiced_crypto_random_ecc ( void *rng_state, unsigned char *output, size_t len );

#if defined(MBEDTLS_SSL_ALPN)
static int tls_add_alpn_extension( wiced_tls_context_t* context, wiced_tls_extension_t* extension );
#endif

#ifndef WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY
static int eap_ssl_receive_packet( void *ctx, unsigned char **buf, size_t len );
static wiced_result_t tls_eap_buffered_data       ( wiced_tls_workspace_t* context, besl_packet_t* packet );
#endif /* WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY */

static wiced_result_t tls_load_certificate_key ( wiced_tls_identity_t* identity, wiced_tls_credentials_info_t* credential, const uint8_t* certificate_data, uint32_t certificate_length, const char* private_key, const uint32_t key_length );

/******************************************************
 *               Variable Definitions
 ******************************************************/
mbedtls_x509_crt* root_ca_certificates = NULL;
char* trusted_ca = NULL;
char* certificate = NULL;
char* key = NULL;

#ifdef BESL
/* Diffie-Hellman Prime: 1024-bit MODP Group with 160-bit Prime Order Subgroup from RFC 5114 */
static const unsigned char diffie_hellman_prime_P[] =
{
        0xB1, 0x0B, 0x8F, 0x96, 0xA0, 0x80, 0xE0, 0x1D,
        0xDE, 0x92, 0xDE, 0x5E, 0xAE, 0x5D, 0x54, 0xEC,
        0x52, 0xC9, 0x9F, 0xBC, 0xFB, 0x06, 0xA3, 0xC6,
        0x9A, 0x6A, 0x9D, 0xCA, 0x52, 0xD2, 0x3B, 0x61,
        0x60, 0x73, 0xE2, 0x86, 0x75, 0xA2, 0x3D, 0x18,
        0x98, 0x38, 0xEF, 0x1E, 0x2E, 0xE6, 0x52, 0xC0,
        0x13, 0xEC, 0xB4, 0xAE, 0xA9, 0x06, 0x11, 0x23,
        0x24, 0x97, 0x5C, 0x3C, 0xD4, 0x9B, 0x83, 0xBF,
        0xAC, 0xCB, 0xDD, 0x7D, 0x90, 0xC4, 0xBD, 0x70,
        0x98, 0x48, 0x8E, 0x9C, 0x21, 0x9A, 0x73, 0x72,
        0x4E, 0xFF, 0xD6, 0xFA, 0xE5, 0x64, 0x47, 0x38,
        0xFA, 0xA3, 0x1A, 0x4F, 0xF5, 0x5B, 0xCC, 0xC0,
        0xA1, 0x51, 0xAF, 0x5F, 0x0D, 0xC8, 0xB4, 0xBD,
        0x45, 0xBF, 0x37, 0xDF, 0x36, 0x5C, 0x1A, 0x65,
        0xE6, 0x8C, 0xFD, 0xA7, 0x6D, 0x4D, 0xA7, 0x08,
        0xDF, 0x1F, 0xB2, 0xBC, 0x2E, 0x4A, 0x43, 0x71,
};

static const unsigned char diffie_hellman_prime_G[] =
{
        0xA4, 0xD1, 0xCB, 0xD5, 0xC3, 0xFD, 0x34, 0x12,
        0x67, 0x65, 0xA4, 0x42, 0xEF, 0xB9, 0x99, 0x05,
        0xF8, 0x10, 0x4D, 0xD2, 0x58, 0xAC, 0x50, 0x7F,
        0xD6, 0x40, 0x6C, 0xFF, 0x14, 0x26, 0x6D, 0x31,
        0x26, 0x6F, 0xEA, 0x1E, 0x5C, 0x41, 0x56, 0x4B,
        0x77, 0x7E, 0x69, 0x0F, 0x55, 0x04, 0xF2, 0x13,
        0x16, 0x02, 0x17, 0xB4, 0xB0, 0x1B, 0x88, 0x6A,
        0x5E, 0x91, 0x54, 0x7F, 0x9E, 0x27, 0x49, 0xF4,
        0xD7, 0xFB, 0xD7, 0xD3, 0xB9, 0xA9, 0x2E, 0xE1,
        0x90, 0x9D, 0x0D, 0x22, 0x63, 0xF8, 0x0A, 0x76,
        0xA6, 0xA2, 0x4C, 0x08, 0x7A, 0x09, 0x1F, 0x53,
        0x1D, 0xBF, 0x0A, 0x01, 0x69, 0xB6, 0xA2, 0x8A,
        0xD6, 0x62, 0xA4, 0xD1, 0x8E, 0x73, 0xAF, 0xA3,
        0x2D, 0x77, 0x9D, 0x59, 0x18, 0xD0, 0x8B, 0xC8,
        0x85, 0x8F, 0x4D, 0xCE, 0xF9, 0x7C, 0x2A, 0x24,
        0x85, 0x5E, 0x6E, 0xEB, 0x22, 0xB3, 0xB2, 0xE5,
};
#endif
/******************************************************
 *               Function Definitions
 ******************************************************/
static wiced_result_t tls_packetize_buffered_data( wiced_tls_workspace_t* context, wiced_packet_t** packet );
wiced_result_t wiced_tls_calculate_encrypt_buffer_length( wiced_tls_workspace_t* context, uint16_t payload_size, uint16_t* required_buff_size);


/* this function is used to calculate how much buffer is needed after encrypting the payload */
wiced_result_t wiced_tls_calculate_encrypt_buffer_length( wiced_tls_workspace_t* context, uint16_t payload_size, uint16_t* required_buff_size)
{
    wiced_result_t result;

    result = (wiced_result_t)tls_calculate_encrypt_buffer_length(context, required_buff_size, payload_size);

    return result;
}

static wiced_result_t tls_packetize_buffered_data( wiced_tls_workspace_t* context, wiced_packet_t** packet )
{
    uint8_t*       data;
    uint16_t       length;
    uint16_t       available_length = 0;
    uint16_t       record_length = 0;
    wiced_result_t result;
    uint32_t       amount_to_copy;

    (void)available_length;

    record_length = (uint16_t)context->in_msglen + sizeof(tls_record_header_t) + (context->transform_in->ivlen - context->transform_in->fixed_ivlen);

    /* Get a buffer and fill with decrypted content */
    result = wiced_packet_create_tcp( context->send_context, (uint16_t) MIN(1024, record_length - context->defragmentation_buffer_bytes_processed), (wiced_packet_t**) packet, &data, &length );
    if ( result  != WICED_SUCCESS )
    {
        *packet = NULL;
        return result;
    }
    if ( context->state == MBEDTLS_SSL_HANDSHAKE_OVER )
    {
        /* this doesn't need the extra space for the encryption header that a normal TLS socket would use - remove it */
        data -= sizeof(tls_record_header_t);
        wiced_packet_set_data_start((wiced_packet_t*) *packet, data);
    }

    amount_to_copy = (uint32_t) MIN( length, record_length - context->defragmentation_buffer_bytes_processed );
    memcpy( data, &context->defragmentation_buffer[context->defragmentation_buffer_bytes_processed ], amount_to_copy );
    wiced_packet_set_data_end( *packet, data + amount_to_copy );

    context->defragmentation_buffer_bytes_processed = (uint16_t) ( context->defragmentation_buffer_bytes_processed + amount_to_copy );

    WPRINT_SECURITY_DEBUG (("Received all fragmented data of length [%d], Packetize [%ld] bytes and give it to application  \n",  context->defragmentation_buffer_bytes_received, amount_to_copy ));

    /* Check if we've returned all the data to the user */
    if ( context->defragmentation_buffer_bytes_processed >= record_length )
    {
        tls_host_free_defragmentation_buffer(context->defragmentation_buffer);
        context->defragmentation_buffer                 = NULL;
        context->defragmentation_buffer_bytes_processed = 0;
        context->defragmentation_buffer_bytes_received  = 0;
        context->defragmentation_buffer_length          = 0;
        context->defragmentation_buffer_bytes_skipped   = 0;
    }

    return WICED_SUCCESS;
}

uint64_t tls_host_get_time_ms( void )
{
    uint64_t time_ms;
    wiced_time_get_utc_time_ms( (wiced_utc_time_ms_t*) &time_ms );
    return time_ms;
}

void* tls_host_malloc( const char* name, uint32_t size)
{
    (void) name;
    return malloc_named( name, size );
}

void tls_host_free(void* p)
{
    free( p );
}

static wiced_result_t tls_load_certificate_key ( wiced_tls_identity_t* identity, wiced_tls_credentials_info_t* credential,  const uint8_t* certificate_data, uint32_t certificate_length, const char* private_key, const uint32_t key_length )
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_bool_t loaded = WICED_FALSE;

    if ( ( certificate_data != NULL ) && ( certificate_length != 0 ) )
    {
        /* load x509 certificate */
        mbedtls_x509_crt_init( &credential->certificate );

        result = mbedtls_x509_crt_parse( &credential->certificate, (const unsigned char *) certificate_data, certificate_length );
        if ( result != 0 )
        {
            result = BESL_TLS_ERROR;
            goto ERROR_CERTIFICATE_INIT;
        }

        loaded = WICED_TRUE;
    }

    if ( ( private_key != NULL ) && ( key_length != 0 ) )
    {
        /* load key */
        mbedtls_pk_init( &credential->private_key );

        result = mbedtls_pk_parse_key( &credential->private_key, (const unsigned char *) private_key, key_length, NULL, 0 );
        if ( result != 0 )
        {
            result = BESL_TLS_ERROR;
            goto ERROR_KEY_INIT;
        }

        loaded = WICED_TRUE;
    }

    /* Either certificate or private key loaded, then only add node to credentials list */
    if ( loaded == WICED_TRUE )
    {
        if (( result =  linked_list_insert_node_at_rear( &identity->credentials, &credential->this_node )) != WICED_SUCCESS )
        {
            WPRINT_SECURITY_ERROR((" Failed to add credential info into credentials list \n"));
            goto ERROR_KEY_INIT;
        }
    }

    identity->custom_sign = NULL;

    return WICED_SUCCESS;

ERROR_KEY_INIT:
    mbedtls_pk_free( &credential->private_key );

ERROR_CERTIFICATE_INIT:
    mbedtls_x509_crt_free( &credential->certificate );

    return result;
}

wiced_result_t wiced_tls_init_identity( wiced_tls_identity_t* identity, const char* private_key, const uint32_t key_length, const uint8_t* certificate_data, uint32_t certificate_length )
{
    int result;
    wiced_tls_credentials_info_t* credential_info = NULL;

    wiced_assert( "Bad args", (identity != NULL) );

    if ( identity == NULL )
    {
        return WICED_ERROR;
    }

    memset( identity, 0, sizeof( *identity ) );

    /* allocate memory for credential info */
    if ( ( ( certificate_data != NULL ) && ( certificate_length != 0 ) ) || ( ( private_key != NULL ) && ( key_length != 0 ) ) )
    {
        credential_info = tls_host_malloc( "WICED TLS", sizeof( wiced_tls_credentials_info_t) );
        if ( credential_info == NULL )
        {
            WPRINT_SECURITY_ERROR((" No memory available \n"));
            return WICED_TLS_ERROR_OUT_OF_MEMORY;
        }

        memset ( credential_info, 0 , sizeof ( wiced_tls_credentials_info_t ));
    }

    /* initialize credentials list to store credential info */
    linked_list_init( &identity->credentials );

    if ( credential_info != NULL )
    {
        if ( ( result = tls_load_certificate_key( identity, credential_info, certificate_data, certificate_length, private_key, key_length ) != WICED_SUCCESS ) )
        {
            WPRINT_SECURITY_ERROR((" Failed to load certificate & private key \n"));
            goto ERROR_INIT;
        }
    }

    return WICED_SUCCESS;

ERROR_INIT:
    linked_list_deinit( &identity->credentials );
    if ( credential_info != NULL )
    {
        tls_host_free( credential_info );
    }

    return result;
}

wiced_result_t wiced_tls_add_identity( wiced_tls_identity_t* identity, wiced_tls_credentials_info_t* credential_info, const char* private_key, const uint32_t key_length, const uint8_t* certificate_data, uint32_t certificate_length )
{
    wiced_result_t result = WICED_SUCCESS;

    wiced_assert( "Bad args", (identity != NULL) );

    if ( identity == NULL || credential_info == NULL )
    {
        return WICED_ERROR;
    }

    if ( ( ( certificate_data == NULL ) && ( certificate_length == 0 ) ) || ( ( private_key == NULL ) && ( key_length == 0 ) ) )
    {
        return WICED_ERROR;
    }

    memset ( credential_info, 0 , sizeof ( wiced_tls_credentials_info_t ));

    if ( ( result = tls_load_certificate_key ( identity, credential_info, certificate_data, certificate_length, private_key, key_length ) != WICED_SUCCESS ) )
    {
        WPRINT_SECURITY_ERROR((" Failed to load certificate & private key \n"));
        return result;
    }

    return result;

}

wiced_result_t wiced_tls_remove_identity( wiced_tls_identity_t* identity, wiced_tls_credentials_info_t* credential_info )
{
    wiced_assert( "Bad args", (identity != NULL) );
    wiced_result_t result = WICED_SUCCESS;

    if ( identity == NULL || credential_info == NULL )
    {
        return WICED_ERROR;
    }

    mbedtls_x509_crt_free( &credential_info->certificate );
    mbedtls_pk_free( &credential_info->private_key );

    if ( ( result = linked_list_remove_node( &identity->credentials, &credential_info->this_node )) != WICED_SUCCESS )
    {
        WPRINT_SECURITY_ERROR((" Failed to remove credential info from credentials list \n"));
        return result;
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_tls_deinit_identity( wiced_tls_identity_t* tls_identity )
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_tls_credentials_info_t* current_node = NULL;
    wiced_tls_credentials_info_t* credential_info = NULL;

    if ( tls_identity != NULL )
    {
        /* find credential info in the credential list, Remove all pair of certificate & key from the identity list  */
        if ( ( result = linked_list_get_front_node( &tls_identity->credentials, (linked_list_node_t**) &current_node ) != WICED_SUCCESS ) )
        {
            WPRINT_SECURITY_DEBUG((" list is empty \n"));
            return result;
        }

        while ( current_node != NULL )
        {
            credential_info = (wiced_tls_credentials_info_t*) current_node;
            current_node = (wiced_tls_credentials_info_t*) credential_info->this_node.next;

            mbedtls_x509_crt_free( &credential_info->certificate );
            mbedtls_pk_free( &credential_info->private_key );

            if (( result = linked_list_remove_node( &tls_identity->credentials, &credential_info->this_node )) != WICED_SUCCESS )
            {
                WPRINT_SECURITY_ERROR((" Failed to remove credential information from the credentials list \n"));
                return result;
            }

            tls_host_free( credential_info );
        }
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_tls_init_root_ca_certificates( const char* trusted_ca_certificates, const uint32_t length )
{
    int result;

    if ( trusted_ca_certificates == NULL || length == 0 )
    {
        return WICED_SUCCESS;
    }
    wiced_tls_deinit_root_ca_certificates( );

    root_ca_certificates = tls_host_malloc( "tls", sizeof(*root_ca_certificates) );
    if ( root_ca_certificates == NULL )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    mbedtls_x509_crt_init( root_ca_certificates );

    /* Parse RootCA Certificate */
    result = mbedtls_x509_crt_parse( root_ca_certificates, (const unsigned char *) trusted_ca_certificates, length  );
    if ( result != 0 )
    {
        uint32_t res = (uint32_t)result;
        printf("Result from mbedtls_x509_crt_parse is %lx (%ld) (~%lx)\n", res, res, ~res);
        mbedtls_x509_crt_free( root_ca_certificates );
        tls_host_free( root_ca_certificates );
        root_ca_certificates = NULL;
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_tls_deinit_root_ca_certificates( void )
{
    if ( root_ca_certificates != NULL )
    {
        mbedtls_x509_crt_free( root_ca_certificates );
        tls_host_free( root_ca_certificates );
        root_ca_certificates = NULL;
    }
    return WICED_SUCCESS;
}

wiced_result_t wiced_tls_init_context( wiced_tls_context_t* context, wiced_tls_identity_t* identity, const char* peer_cn )
{
    if( context == NULL )
    {
        return WICED_ERROR;
    }
    memset( context, 0, sizeof(wiced_tls_context_t) );

    context->context.conf = tls_host_malloc ("tls", sizeof(mbedtls_ssl_config));
    if ( context->context.conf == NULL )
    {
        return WICED_OUT_OF_HEAP_SPACE;
    }

    memset( (mbedtls_ssl_config*)context->context.conf, 0, sizeof(mbedtls_ssl_config) );

#if defined(MBEDTLS_SSL_ALPN)
    memset( (void * ) alpn_list, 0, sizeof( alpn_list ) );
#endif

    context->context_id                                      = WICED_TLS_CONTEXT_ID;
    context->identity                                        = identity;
    ((mbedtls_ssl_config*)context->context.conf)->peer_cn    = peer_cn;

    return WICED_SUCCESS;
}

wiced_result_t wiced_tcp_enable_tls( wiced_tcp_socket_t* socket, void* context )
{
    socket->tls_context = context;
    return WICED_SUCCESS;
}

wiced_result_t wiced_tcp_start_tls( wiced_tcp_socket_t* socket, wiced_tls_endpoint_type_t type, wiced_tls_certificate_verification_t verification )
{
    return wiced_generic_start_tls_with_ciphers( socket->tls_context, socket, type, verification, NULL, TLS_TCP_TRANSPORT );
}

/* TODO : Need to simplify logic for all different cases in optimized manner
 * Multiple TLS records in single TCP packet
 * TLS record data fragmented into multiple TCP packet
 * TLS header fragmented into multiple TCP packet
 * TLS header in last packet and data in coming TCP packets
 */
static int ssl_receive_packet( void *ctx, unsigned char **buf, size_t len )
{
    wiced_result_t result;
    wiced_tcp_socket_t* socket = ( wiced_tcp_socket_t* ) ctx;
    uint16_t available_data_length = 0;
    uint16_t length = 0;

    WPRINT_SECURITY_DEBUG (("TLS library asked for [%d] bytes \n", len ));

    /* Case : Check if we are halfway through a fragmented record
     * These handles case when single TCP packet has multiple record [ex. Record R1 & R2 ]. and R2 record in packet1 is fragmented into multiple TCP packets. In this case R2 content in packet1 will
     * be copied into defragmentation buffer and then give content of R1 to application. When MBEDTLS calls receive callback for R2 content it should fall in this case to receive full content for R2.
     * */
    if ( socket->tls_context->context.defragmentation_buffer != NULL && socket->tls_context->context.defragmentation_buffer_bytes_received != socket->tls_context->context.defragmentation_buffer_length )
    {
        WPRINT_SECURITY_DEBUG (("halfway TLS defrag ## defragmentation buffer bytes received [%d] total defragmentaion bytes [%d] \n ", socket->tls_context->context.defragmentation_buffer_bytes_received, socket->tls_context->context.defragmentation_buffer_length ));
        /* Check if we already have number of bytes ( asked by MBEDTLS ) present in defragmentation buffer */
        if ( len < ( socket->tls_context->context.defragmentation_buffer_bytes_received - socket->tls_context->context.defragmentation_buffer_bytes_skipped ) )
        {
            WPRINT_SECURITY_DEBUG (("Already have [%d] bytes in defragmentation buffer, asked for [%d] \n ", socket->tls_context->context.defragmentation_buffer_bytes_received - socket->tls_context->context.defragmentation_buffer_bytes_skipped, len ));
            WPRINT_SECURITY_DEBUG (("defragmentation buffer bytes skipped [%d] \n ", socket->tls_context->context.defragmentation_buffer_bytes_skipped ));

            *buf = socket->tls_context->context.defragmentation_buffer;
            socket->tls_context->context.defragmentation_buffer_bytes_skipped += len;
            return WICED_SUCCESS;
        }
        else
        {
            result = tls_complete_record_defragmentation(&socket->tls_context->context, socket->tls_context->context.read_timeout);
            if ( result != (wiced_result_t) TLS_SUCCESS )
            {
                WPRINT_SECURITY_DEBUG (("halfway TLS record defragmentation failed with result : [%d] \n",  result ));
                socket->tls_context->context.defragmentation_buffer_bytes_skipped  = 0;
                return result;
            }

            socket->tls_context->context.defragmentation_buffer_bytes_skipped = sizeof(tls_record_header_t);

            /* We dont have enough data in defragmentation buffer, So go and read the multiple packets and accumulate whole data */
            WPRINT_SECURITY_DEBUG (("halfway defragmentation of TLS record completed \n"));

            if ( socket->tls_context->context.is_reading_header == 1 )
            {
                /* defragmentation buffer has both header & data. If MBEDTLS asked for header then always return base address of defragmentation buffer */
                *buf = socket->tls_context->context.defragmentation_buffer;
            }
            else
            {
                *buf = socket->tls_context->context.defragmentation_buffer + socket->tls_context->context.defragmentation_buffer_bytes_skipped;
            }

            return result;
        }
    }
    else if ( socket->tls_context->context.defragmentation_buffer != NULL && socket->tls_context->context.defragmentation_buffer_bytes_received == socket->tls_context->context.defragmentation_buffer_length )
    {
        /* received full content of defragmentation buffer */
        *buf = socket->tls_context->context.defragmentation_buffer + sizeof(tls_record_header_t);
        return WICED_SUCCESS;
    }

    /* If previously received packet is consumed, then go and read the next packet */
    if ( socket->tls_context->context.received_packet == NULL )
    {
        result = network_tcp_receive( socket, (wiced_packet_t**)&socket->tls_context->context.received_packet, socket->tls_context->context.read_timeout );
        if ( result != WICED_SUCCESS )
        {
            WPRINT_SECURITY_DEBUG (("network_tcp_receive failed with result : [%d] \n",  result ));
            return result;
        }

        socket->tls_context->context.received_packet_bytes_skipped = 0;
        socket->tls_context->context.received_packet_length = 0;

        result = wiced_packet_get_data( (wiced_packet_t*)socket->tls_context->context.received_packet, socket->tls_context->context.received_packet_bytes_skipped, (uint8_t**) ( buf ), (uint16_t*)&socket->tls_context->context.received_packet_length, &available_data_length );
        if ( result != WICED_SUCCESS )
        {
            WPRINT_SECURITY_ERROR ( ("Error in getting packet data : %d \n", result ) );
            return result;
        }

        WPRINT_SECURITY_DEBUG (("Received new TCP packet with length [%d] \n", socket->tls_context->context.received_packet_length ));

    }
    else
    {
        WPRINT_SECURITY_DEBUG (("Skip [%d] no of bytes from TCP received packet with length : [%d]\n", socket->tls_context->context.received_packet_bytes_skipped, socket->tls_context->context.received_packet_length ));

        result = wiced_packet_get_data( (wiced_packet_t*)socket->tls_context->context.received_packet, socket->tls_context->context.received_packet_bytes_skipped, (uint8_t**) ( buf ), (uint16_t*)&length, &available_data_length );
        if ( result != WICED_SUCCESS )
        {
            WPRINT_SECURITY_DEBUG (("wiced_packet_get_data failed with result : [%d] \n",  result ));
            return result;
        }

        /* Case : This logic handles case where TCP packet has last 5 bytes and that is the TLS header
         *        Find out the length of TLS record payload from TLS header of 5 bytes and allocate defragmentation buffer. Copy received TLS header and delete existing received packet.
         *        remaining data will be accumulated and copied into defragmentation buffer in next call when MBEDTLS asks for record payload.
         * */
        if ( socket->tls_context->context.is_reading_header == 1  && ( socket->tls_context->context.received_packet_length - socket->tls_context->context.received_packet_bytes_skipped ) == 5)
        {
             WPRINT_SECURITY_DEBUG ((" TCP packet has only TLS header of 5 bytes left case \n"));
             tls_record_t* temp_record = (tls_record_t*)*buf;

             /* Allocate a new, larger buffer. Copy the content from the old buffer and then free the old buffer */
             socket->tls_context->context.defragmentation_buffer_length = htobe16(temp_record->length) + sizeof(tls_record_header_t);

             socket->tls_context->context.defragmentation_buffer        = tls_host_get_defragmentation_buffer( socket->tls_context->context.defragmentation_buffer_length );
             if ( socket->tls_context->context.defragmentation_buffer == NULL )
             {
                 WPRINT_SECURITY_ERROR(("No memory available to allocate for defragmentation buffer \n"));
                 tls_host_free_packet( socket->tls_context->context.received_packet );
                 socket->tls_context->context.received_packet = NULL;
                 return TLS_ERROR_OUT_OF_MEMORY;
             }

             socket->tls_context->context.defragmentation_buffer_bytes_received = sizeof(tls_record_header_t);
             socket->tls_context->context.defragmentation_buffer_bytes_processed = ( sizeof(tls_record_header_t) + (socket->tls_context->context.transform_in->ivlen - socket->tls_context->context.transform_in->fixed_ivlen) );

             memcpy(socket->tls_context->context.defragmentation_buffer, temp_record, sizeof(tls_record_header_t));

             /* Application doesnt want us to read new packet */
             if ( socket->tls_context->context.packet_receive_option == TLS_AVOID_NEW_RECORD_PACKET_RECEIVE )
             {
                 WPRINT_SECURITY_DEBUG (("Avoid receiving new packet \n"));
                 return TLS_NO_DATA;
             }

             wiced_packet_delete ((wiced_packet_t*) socket->tls_context->context.received_packet);
             socket->tls_context->context.received_packet = NULL;
         }
    }


    /* Case : number of bytes asked by MBEDTLS is more than MTU size, so it is defragmentation case.
     * Allocate defragmentation buffer, Receive multiple TCP packets till all the data received, and copy content into defragmentation buffer.
     */
    if ( len > ( socket->tls_context->context.received_packet_length - socket->tls_context->context.received_packet_bytes_skipped ) )
    {
        /* Case : This logic handles case where TLS record header is fragmented into multiple TCP packet
         *        Check for flag is_reading_header, If it is set then MBEDTLS has asked for TLS header. If header is fragmented then allocate 5 bytes of defragmentation buffer
         *        and copy received bytes. Accumulate remaining data in tls_complete_record_defragmentation call.
         * */

        if ( socket->tls_context->context.is_reading_header == 1 && socket->tls_context->context.received_packet_length - socket->tls_context->context.received_packet_bytes_skipped < sizeof(tls_record_header_t))
        {
            WPRINT_SECURITY_DEBUG ((" TLS record header is fragmented into multiple TCP packet case \n"));

            /* Create defragmentation buffer of size defragmentation_buffer_length which includes TLS record header + data content. */
            socket->tls_context->context.defragmentation_buffer_length = sizeof(tls_record_header_t);
            socket->tls_context->context.defragmentation_buffer        = tls_host_get_defragmentation_buffer( socket->tls_context->context.defragmentation_buffer_length );
            if ( socket->tls_context->context.defragmentation_buffer == NULL )
            {
                WPRINT_SECURITY_ERROR(("No memory available to allocate for defragmentation buffer \n"));
                tls_host_free_packet( socket->tls_context->context.received_packet );
                socket->tls_context->context.received_packet = NULL;
                return TLS_ERROR_OUT_OF_MEMORY;
            }

            /* Copy available content into defragmentation buffer including TLS record header */
            memcpy(socket->tls_context->context.defragmentation_buffer, *buf, length);
            socket->tls_context->context.defragmentation_buffer_bytes_received = length;

            /* set the value of defragmentation_buffer_bytes_processed to minimum of length we received or TLS record header + encryption IV as these will be processed while decrypting the record*/
            if ( socket->tls_context->context.state == MBEDTLS_SSL_HANDSHAKE_OVER )
            {
                socket->tls_context->context.defragmentation_buffer_bytes_processed = MIN ( length, sizeof(tls_record_header_t) + (socket->tls_context->context.transform_in->ivlen - socket->tls_context->context.transform_in->fixed_ivlen) );
            }
        }
        else
        {
            /* Create defragmentation buffer of size defragmentation_buffer_length which includes TLS record header + data content. */
            socket->tls_context->context.defragmentation_buffer_length = len + sizeof(tls_record_header_t);
            socket->tls_context->context.defragmentation_buffer        = tls_host_get_defragmentation_buffer( socket->tls_context->context.defragmentation_buffer_length );
            if ( socket->tls_context->context.defragmentation_buffer == NULL )
            {
                WPRINT_SECURITY_ERROR(("No memory available to allocate for defragmentation buffer \n"));
                tls_host_free_packet( socket->tls_context->context.received_packet );
                socket->tls_context->context.received_packet = NULL;
                return TLS_ERROR_OUT_OF_MEMORY;
            }

            WPRINT_SECURITY_DEBUG (("Defragmentaion case : copying %d bytes to defrag buffer 0x%08x\n",  length + sizeof(tls_record_header_t), (unsigned int)socket->tls_context->context.defragmentation_buffer ));

            /* Copy available content into defragmentation buffer including TLS record header */
            memcpy(socket->tls_context->context.defragmentation_buffer, *buf - sizeof(tls_record_header_t), length + sizeof(tls_record_header_t));
            socket->tls_context->context.defragmentation_buffer_bytes_received = length + sizeof(tls_record_header_t);

            /* Assuming TLS record header already given to MBEDTLS and setting processed bytes including header + iv length
             * TODO : When TLS record itself fragmented then need to handle processed bytes accordingly. */
            if ( socket->tls_context->context.state == MBEDTLS_SSL_HANDSHAKE_OVER )
            {
                socket->tls_context->context.defragmentation_buffer_bytes_processed = ( sizeof(tls_record_header_t) + (socket->tls_context->context.transform_in->ivlen - socket->tls_context->context.transform_in->fixed_ivlen) );
            }
        }

        /* Application doesnt want us to read new packet */
        if ( socket->tls_context->context.packet_receive_option == TLS_AVOID_NEW_RECORD_PACKET_RECEIVE )
        {
            WPRINT_SECURITY_DEBUG (("Avoid receiving new packet \n"));
            return TLS_NO_DATA;
        }

        // "Consume" the data from the packet
        tls_consume_received_packet_bytes(&socket->tls_context->context, length + socket->tls_context->context.received_packet_bytes_skipped);

        result = tls_complete_record_defragmentation(&socket->tls_context->context, socket->tls_context->context.read_timeout);
        if (result != (wiced_result_t) TLS_SUCCESS )
        {
            WPRINT_SECURITY_DEBUG (("TLS record defragmentation failed with result : [%d] \n",  result ));
            return result;
        }

        WPRINT_SECURITY_DEBUG (("Completed TLS record defragmentation. Received [%d] bytes \n",  socket->tls_context->context.defragmentation_buffer_bytes_received ));

        if ( socket->tls_context->context.is_reading_header == 1 )
        {
            /* defragmentation buffer has both header & data. If MBEDTLS asked for header then always return base address of defragmentation buffer */
            *buf = socket->tls_context->context.defragmentation_buffer;
        }
        else
        {
            *buf = socket->tls_context->context.defragmentation_buffer + sizeof(tls_record_header_t);
        }
    }
    else
    {
        socket->tls_context->context.received_packet_bytes_skipped += len;
    }

    return WICED_SUCCESS;

}

#if defined(MBEDTLS_X509_CRT_PARSE_C)
/*
 * Enabled if debug_level > 1 in code below
 */
static int my_verify( void *data, mbedtls_x509_crt *crt, int depth, uint32_t *flags )
{
#if 0
    char buf[1024];
    ((void) data);

    printf ( "\nVerify requested for (Depth %d):\n", depth );
    mbedtls_x509_crt_info( buf, sizeof( buf ) - 1, "", crt );
    printf ( "%s", buf );

    if ( ( *flags ) == 0 )
        printf ( "  This certificate has no flags\n" );
    else
    {
        mbedtls_x509_crt_verify_info( buf, sizeof( buf ), "  ! ", *flags );
        printf ( "%s\n", buf );
    }
#endif
    return( 0 );
}
#if defined(MBEDTLS_DEBUG_C)
static void mbedtls_debug( void *ctx, int level, const char *file, int line, const char *str )
{
    ( (void) level );
    WPRINT_SECURITY_DEBUG(("%s:%04d: %s", file, line, str ));
}
#endif /* MBEDTLS_DEBUG_C */

#endif /* MBEDTLS_X509_CRT_PARSE_C */

mbedtls_entropy_context entropy;
mbedtls_ctr_drbg_context ctr_drbg;

wiced_result_t wiced_generic_start_tls_with_ciphers( wiced_tls_context_t* tls_context, void* referee, wiced_tls_endpoint_type_t type, wiced_tls_certificate_verification_t verification, const cipher_suite_t* cipher_list[], tls_transport_protocol_t transport_protocol )
{
    int             prev_state;
    uint64_t        start_time;
    wiced_result_t  result;
    int             min_ver;
    int             max_ver;

    int ret = 0;
    mbedtls_ssl_config* conf = (mbedtls_ssl_config*) tls_context->context.conf;
#if defined(WICED_TLS_CLI_CACHE_SESSION) && defined(MBEDTLS_SSL_CLI_C)
    wiced_tcp_socket_t* socket  = referee;
    wiced_ssl_cache_entry       *cache_entry = NULL;
    mbedtls_ssl_session         tls_session;
    wiced_result_t              entry_found = WICED_SUCCESS;
#endif
#if 0
#if defined(MBEDTLS_KEY_EXCHANGE__SOME__PSK_ENABLED)
    unsigned char psk[MBEDTLS_PSK_MAX_LEN];
    size_t psk_len = 0;
#endif
#endif
    const char *pers = "ssl_client2";

#if defined(MBEDTLS_TIMING_C)
    mbedtls_timing_delay_context timer;
#endif
#if defined(MBEDTLS_X509_CRT_PARSE_C)
#if 0
    uint32_t flags;
#endif
#endif

    /*
     * Make sure memory references are valid.
     */
    mbedtls_ctr_drbg_init( &ctr_drbg );

#if (WICED_TLS_MINOR_VERSION_MIN == 0)
    min_ver = MBEDTLS_SSL_MINOR_VERSION_1;
#elif (WICED_TLS_MINOR_VERSION_MIN == 1)
    min_ver = MBEDTLS_SSL_MINOR_VERSION_2;
#elif (WICED_TLS_MINOR_VERSION_MIN == 2)
    min_ver = MBEDTLS_SSL_MINOR_VERSION_3;
#endif
#if (WICED_TLS_MINOR_VERSION_MAX == 0)
    max_ver = MBEDTLS_SSL_MINOR_VERSION_1;
#elif (WICED_TLS_MINOR_VERSION_MAX == 1)
    max_ver = MBEDTLS_SSL_MINOR_VERSION_2;
#elif (WICED_TLS_MINOR_VERSION_MAX == 2)
    max_ver = MBEDTLS_SSL_MINOR_VERSION_3;
#endif
   opt_config.min_version = min_ver;
   opt_config.max_version = max_ver;

   opt_config.arc4 = MBEDTLS_SSL_ARC4_DISABLED;
   opt_config.auth_mode = MBEDTLS_SSL_VERIFY_REQUIRED;
   opt_config.mfl_code = MBEDTLS_SSL_MAX_FRAG_LEN_NONE;
   opt_config.trunc_hmac = MBEDTLS_SSL_TRUNC_HMAC_DISABLED;

#if defined(MBEDTLS_DEBUG_C)
   opt_config.debug_level = MBEDTLS_DEBUG_LOG_LEVEL;
    mbedtls_debug_set_threshold( opt_config.debug_level );
#endif /* MBEDTLS_DEBUG_C */

    if( opt_config.force_ciphersuite[0] > 0 )
    {
        const mbedtls_ssl_ciphersuite_t *ciphersuite_info;
        ciphersuite_info = mbedtls_ssl_ciphersuite_from_id( opt_config.force_ciphersuite[0] );

        if( opt_config.max_version != -1 &&
            ciphersuite_info->min_minor_ver > opt_config.max_version )
        {
            ret = 2;

        }
        if( opt_config.min_version != -1 &&
            ciphersuite_info->max_minor_ver < opt_config.min_version )
        {
            ret = 2;
        }

        /* If the server selects a version that's not supported by
         * this suite, then there will be no common ciphersuite... */
        if( opt_config.max_version == -1 ||
            opt_config.max_version > ciphersuite_info->max_minor_ver )
        {
            opt_config.max_version = ciphersuite_info->max_minor_ver;
        }
        if( opt_config.min_version < ciphersuite_info->min_minor_ver )
        {
            opt_config.min_version = ciphersuite_info->min_minor_ver;
            /* DTLS starts with TLS 1.1 */
            if( opt_config.transport == MBEDTLS_SSL_TRANSPORT_DATAGRAM &&
                opt_config.min_version < MBEDTLS_SSL_MINOR_VERSION_2 )
                opt_config.min_version = MBEDTLS_SSL_MINOR_VERSION_2;
        }

        /* Enable RC4 if needed and not explicitly disabled */
        if( ciphersuite_info->cipher == MBEDTLS_CIPHER_ARC4_128 )
        {
            if( opt_config.arc4 == MBEDTLS_SSL_ARC4_DISABLED )
            {
                ret = 2;
            }

            opt_config.arc4 = MBEDTLS_SSL_ARC4_ENABLED;
        }
    }

    /*
     * 0. Initialize the RNG and the session data
     */
    mbedtls_entropy_init( &entropy );
    if( ( ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy,
                               (const unsigned char *) pers,
                               strlen( pers )) ) != 0 )
    {
        WPRINT_SECURITY_ERROR(( " failed\n  ! mbedtls_ctr_drbg_seed returned -0x%x\n", ret ));
        result = TLS_ERROR;
        goto exit_with_inited_context;
    }

    /*
     * 3. Setup stuff
     */
    WPRINT_SECURITY_DEBUG( ( "  . Setting up the SSL/TLS structure... \n" ) );

    if ( type == WICED_TLS_AS_CLIENT )
    {
        if( ( ret = mbedtls_ssl_config_defaults( conf,
                MBEDTLS_SSL_IS_CLIENT,
                opt_config.transport,
                MBEDTLS_SSL_PRESET_DEFAULT ) ) != 0 )
        {
                WPRINT_SECURITY_ERROR ( ( " failed\n  ! mbedtls_ssl_config_defaults returned -0x%x\n\n", ret ) );
                result = TLS_ERROR;
                goto exit_with_inited_context;
        }
    }
    else
    {
        if( ( ret = mbedtls_ssl_config_defaults( conf,
                MBEDTLS_SSL_IS_SERVER,
                opt_config.transport,
                MBEDTLS_SSL_PRESET_DEFAULT ) ) != 0 )
        {
                WPRINT_SECURITY_ERROR ( ( " failed\n  ! mbedtls_ssl_config_defaults returned -0x%x\n\n", ret ) );
                result = TLS_ERROR;
                goto exit_with_inited_context;
        }
    }

#if defined(MBEDTLS_X509_CRT_PARSE_C)
    if( opt_config.debug_level > 0 )
        mbedtls_ssl_conf_verify( conf, my_verify, NULL );
#endif

    if ( root_ca_certificates != NULL )
    {
        mbedtls_ssl_conf_ca_chain( conf, root_ca_certificates, NULL );
        mbedtls_ssl_conf_authmode( conf, verification );
    }
    else
    {
        mbedtls_ssl_conf_authmode( conf, MBEDTLS_SSL_VERIFY_NONE );
    }

#if defined(MBEDTLS_SSL_TRUNCATED_HMAC)
    if( opt_config.trunc_hmac != DFL_TRUNC_HMAC )
        mbedtls_ssl_conf_truncated_hmac( &conf, opt_config.trunc_hmac );
#endif

#if defined(MBEDTLS_SSL_EXTENDED_MASTER_SECRET)
    if( opt_config.extended_ms != DFL_EXTENDED_MS )
        mbedtls_ssl_conf_extended_master_secret( &conf, opt_config.extended_ms );
#endif

#if defined(MBEDTLS_SSL_ENCRYPT_THEN_MAC)
    if( opt_config.etm != DFL_ETM )
        mbedtls_ssl_conf_encrypt_then_mac( &conf, opt_config.etm );
#endif

#if defined(MBEDTLS_SSL_CBC_RECORD_SPLITTING)
    if( opt_config.recsplit != DFL_RECSPLIT )
        mbedtls_ssl_conf_cbc_record_splitting( &conf, opt_config.recsplit
                                    ? MBEDTLS_SSL_CBC_RECORD_SPLITTING_ENABLED
                                    : MBEDTLS_SSL_CBC_RECORD_SPLITTING_DISABLED );
#endif

#if defined(MBEDTLS_DHM_C) & defined(MBEDTLS_CLI_C)
    if( opt_config.dhmlen != DFL_DHMLEN )
        mbedtls_ssl_conf_dhm_min_bitlen( &conf, opt_config.dhmlen );
#endif

    mbedtls_ssl_conf_rng( conf, wiced_crypto_random_ecc, NULL );

#if defined(MBEDTLS_DEBUG_C)
    mbedtls_ssl_conf_dbg( conf, mbedtls_debug, stdout );
#endif /* MBEDTLS_DEBUG_C */

    tls_context->context.transport_protocol = transport_protocol;

#if defined(MBEDTLS_SSL_CACHE_C) && defined(MBEDTLS_SSL_SRV_C  )
    if ( type == WICED_TLS_AS_SERVER )
    {
        /* Initialize cache only one time, when max entries are 0 */
        if ( cache.max_entries == 0 )
        {
            mbedtls_ssl_cache_init( &cache );
        }

        /* set number of maximum resume connection & timeout for the session */
        mbedtls_ssl_cache_set_max_entries( &cache, WICED_TLS_MAX_RESUMABLE_SESSIONS );
        cache.timeout = TLS_SESSION_TIMEOUT;

        mbedtls_ssl_conf_session_cache( conf, &cache, mbedtls_ssl_cache_get, mbedtls_ssl_cache_set );
    }
#endif

#if defined(WICED_TLS_CLI_CACHE_SESSION) && defined(MBEDTLS_SSL_CLI_C)
    if ( wiced_ssl_cache.max_entries == 0 )
    {
       wiced_ssl_cache_init( &wiced_ssl_cache );
    }
#endif

#if defined(MBEDTLS_SSL_SESSION_TICKETS)
    mbedtls_ssl_conf_session_tickets( &conf, opt_config.tickets );
#endif

    if( opt_config.force_ciphersuite[0] != DFL_FORCE_CIPHER )
        mbedtls_ssl_conf_ciphersuites( conf, opt_config.force_ciphersuite );

#if defined(MBEDTLS_ARC4_C)
    if( opt_config.arc4 != DFL_ARC4 )
        mbedtls_ssl_conf_arc4_support( &conf, opt_config.arc4 );
#endif

    if( opt_config.allow_legacy != DFL_ALLOW_LEGACY )
        mbedtls_ssl_conf_legacy_renegotiation( conf, opt_config.allow_legacy );
#if defined(MBEDTLS_SSL_RENEGOTIATION)
    mbedtls_ssl_conf_renegotiation( &conf, opt_config.renegotiation );
#endif

    if ( tls_context->identity != NULL )
    {
        wiced_tls_credentials_info_t* credential = NULL;

        linked_list_get_front_node( &tls_context->identity->credentials, (linked_list_node_t**) &credential );
        while ( credential != NULL )
        {
            if ( ( ret = mbedtls_ssl_conf_own_cert( conf, &credential->certificate, &credential->private_key ) ) != 0 )
            {
                WPRINT_SECURITY_ERROR ( ( " failed\n  ! mbedtls_ssl_conf_own_cert returned %d\n\n", ret ) );
                result = TLS_ERROR;
                goto exit_with_inited_context;
            }

            credential = (wiced_tls_credentials_info_t*) credential->this_node.next;
        }
    }
    else
    {
        conf->key_cert = NULL;
    }

#if 0
#if defined(MBEDTLS_KEY_EXCHANGE__SOME__PSK_ENABLED)
    if( ( ret = mbedtls_ssl_conf_psk( &conf, psk, psk_len,
                             (const unsigned char *) opt_config.psk_identity,
                             strlen( opt_config.psk_identity ) ) ) != 0 )
    {
        WPRINT_SECURITY_ERROR ( ( " failed\n  ! mbedtls_ssl_conf_psk returned %d\n\n", ret ) );
        result = TLS_ERROR;
        goto exit_with_inited_context;
    }
#endif
#endif

    mbedtls_ssl_conf_min_version( conf, MBEDTLS_SSL_MAJOR_VERSION_3, opt_config.min_version );
    mbedtls_ssl_conf_max_version( conf, MBEDTLS_SSL_MAJOR_VERSION_3, opt_config.max_version );

#if defined(MBEDTLS_SSL_FALLBACK_SCSV)
    if( opt_config.fallback != DFL_FALLBACK )
        mbedtls_ssl_conf_fallback( &conf, opt_config.fallback );
#endif

    if( ( ret = mbedtls_ssl_setup( &tls_context->context, conf ) ) != 0 )
    {
        WPRINT_SECURITY_ERROR ( ( " failed\n  ! mbedtls_ssl_setup returned -0x%x\n\n", ret ) );
        result = TLS_ERROR;
        goto exit_with_inited_context;
    }

    if ( transport_protocol == TLS_EAP_TRANSPORT )
    {
        /* reset the ssl context members else during roaming context will have garbage values. */
        mbedtls_eap_ssl_context_reinit( &tls_context->context );
    }

    /* If session is already set then assuming that this is resume connection, It is only applicable for client */
    if ( type == WICED_TLS_AS_CLIENT && tls_context->session != NULL)
    {
        if ( tls_context->session->id_len > 0 )
        {

            mbedtls_ssl_set_session( &tls_context->context, tls_context->session );
        }
    }

#if defined(WICED_TLS_CLI_CACHE_SESSION) && defined(MBEDTLS_SSL_CLI_C)
    if ( type == WICED_TLS_AS_CLIENT && tls_context->session == NULL && transport_protocol != TLS_EAP_TRANSPORT)
    {
        uint32_t count = 0;
        wiced_rtos_lock_mutex(&wiced_ssl_cache.mutex);
        result = linked_list_get_count(&wiced_ssl_cache.cache_list, &count);
        if(count)
        {
            entry_found = get_ssl_cache_entry((wiced_ip_address_t*)&socket->socket.nx_tcp_socket_connect_ip, socket->socket.nx_tcp_socket_connect_port, &cache_entry);
            if ( entry_found == WICED_SUCCESS && cache_entry != NULL && cache_entry->tls_session.id_len > 0)
            {
                mbedtls_ssl_set_session( &tls_context->context, &cache_entry->tls_session );
            }
        }
        else
        {
            entry_found = WICED_NOT_FOUND;
        }
        wiced_rtos_unlock_mutex(&wiced_ssl_cache.mutex);
    }
#endif


#if defined(MBEDTLS_KEY_EXCHANGE_ECJPAKE_ENABLED)
    if( opt_config.ecjpake_pw != DFL_ECJPAKE_PW )
    {
        if( ( ret = mbedtls_ssl_set_hs_ecjpake_password( &ssl,
                        (const unsigned char *) opt_config.ecjpake_pw,
                                        strlen( opt_config.ecjpake_pw ) ) ) != 0 )
        {
            WPRINT_SECURITY_ERROR ( ( " failed\n  ! mbedtls_ssl_set_hs_ecjpake_password returned %d\n\n", ret ) );
            result = TLS_ERROR;
            goto exit_with_inited_context;
        }
    }
#endif

    if ( transport_protocol == TLS_TCP_TRANSPORT )
    {
        mbedtls_ssl_set_bio( &tls_context->context, referee, ssl_flush_output, ssl_receive_packet, NULL );
    }
#ifndef WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY
    else if ( transport_protocol == TLS_EAP_TRANSPORT )
    {

        mbedtls_ssl_set_bio( &tls_context->context, referee, eap_ssl_flush_output, eap_ssl_receive_packet, NULL );
    }
#endif /* WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY */
    else
    {
        return WICED_ERROR;
    }

        tls_context->context.send_context = referee;
        tls_context->context.receive_context = referee;
        /* using same handshake timeout as read timeout */
        tls_context->context.read_timeout = TLS_HANDSHAKE_PACKET_TIMEOUT_MS;

#if defined(MBEDTLS_TIMING_C)
    mbedtls_ssl_set_timer_cb( &ssl, &timer, mbedtls_timing_set_delay,
                                            mbedtls_timing_get_delay );
#endif

    /*
     * 4. Handshake
     */
    prev_state = 0;
    start_time = tls_host_get_time_ms();
    do
    {
        uint64_t curr_time;
        /* TODO : Need to return all TLS errors to application instead of generic TLS_ERROR */
        if ( ( ret = mbedtls_ssl_handshake( &tls_context->context ) ) != 0 )
        {
            WPRINT_SECURITY_INFO (( "failed\n  ! mbedtls_ssl_handshake returned -0x%x\n", ret));
            result = TLS_ERROR;
            goto exit_with_inited_context;
        }

        /* break out if stuck */
        curr_time = tls_host_get_time_ms();
        if ( curr_time - start_time > MAX_HANDSHAKE_WAIT )
        {
            WPRINT_SECURITY_INFO(( "Timeout in SSL handshake\n" ));
            result = TLS_HANDSHAKE_TIMEOUT;
            goto exit_with_inited_context;
        }

        /* if no state change then wait on client */
        if ( prev_state == tls_context->context.state )
        {
            host_rtos_delay_milliseconds( 10 );
        }
        else /* otherwise process next state with no delay */
        {
            prev_state = tls_context->context.state;
        }
    } while ( tls_context->context.state != MBEDTLS_SSL_HANDSHAKE_OVER );


    if( ( ret = mbedtls_ssl_get_record_expansion( &tls_context->context ) ) >= 0 )
        WPRINT_SECURITY_DEBUG ( ( "    [ Record expansion is %d ]\n", ret ) );
    else
        WPRINT_SECURITY_DEBUG ( ( "    [ Record expansion is unknown (compression) ]\n" ) );

#if defined(MBEDTLS_SSL_MAX_FRAGMENT_LENGTH)
    WPRINT_LIB_DEBUG ( ( "    [ Maximum fragment length is %u ]\n",
                    (unsigned int) mbedtls_ssl_get_max_frag_len( &tls_context->context ) ) );
#endif

#if defined(MBEDTLS_SSL_ALPN)
        const char *alp = mbedtls_ssl_get_alpn_protocol( &tls_context->context );
        UNUSED_PARAMETER(alp);
        WPRINT_LIB_DEBUG ( ( "    [ Application Layer Protocol is %s ]\n",
                alp ? alp : "(none)" ) );
#endif

#if defined(MBEDTLS_X509_CRT_PARSE_C)
#if 0
    /*
     * 5. Verify the server certificate
     */
    WPRINT_SECURITY_DEBUG ( ( "  . Verifying peer X.509 certificate..." ) );

    if( ( flags = mbedtls_ssl_get_verify_result( &tls_context->context ) ) != 0 )
    {
        char vrfy_buf[512];

        WPRINT_SECURITY_DEBUG ( ( " failed\n" ) );

        mbedtls_x509_crt_verify_info( vrfy_buf, sizeof( vrfy_buf ), "  ! ", flags );

        WPRINT_SECURITY_DEBUG ( ( "%s\n", vrfy_buf ) );
    }
    else
        WPRINT_SECURITY_DEBUG ( ( " ok\n" ) );

    if( mbedtls_ssl_get_peer_cert( &tls_context->context ) != NULL )
    {
        WPRINT_SECURITY_DEBUG ( ( "  . Peer certificate information    ...\n" ) );
        mbedtls_x509_crt_info( (char *) buf, sizeof( buf ) - 1, "      ",
                       mbedtls_ssl_get_peer_cert( &tls_context->context ) );
        WPRINT_SECURITY_DEBUG ( ( "%s\n", buf ) );
    }
#endif /* MBEDTLS_X509_CRT_PARSE_C */
#endif

#if defined(MBEDTLS_SSL_RENEGOTIATION)
    if( opt_config.renegotiate )
    {
        /*
         * Perform renegotiation (this must be done when the server is waiting
         * for input from our side).
         */
        WPRINT_SECURITY_DEBUG ( "  . Performing renegotiation..." );
        fflush( stdout );
        while( ( ret = mbedtls_ssl_renegotiate( &ssl ) ) != 0 )
        {
            if( ret != MBEDTLS_ERR_SSL_WANT_READ &&
                ret != MBEDTLS_ERR_SSL_WANT_WRITE )
            {
                WPRINT_SECURITY_ERROR ( ( " failed\n  ! mbedtls_ssl_renegotiate returned %d\n\n", ret ) );
                result = TLS_ERROR;
                goto exit_with_inited_context;
            }
        }
    }
#endif /* MBEDTLS_SSL_RENEGOTIATION */

#if defined(WICED_TLS_CLI_CACHE_SESSION) && defined(MBEDTLS_SSL_CLI_C)
    do
    {
        if ( type == WICED_TLS_AS_CLIENT && transport_protocol != TLS_EAP_TRANSPORT )
        {
            memset ( &tls_session, 0, sizeof ( mbedtls_ssl_session ) );
            mbedtls_ssl_get_session( &tls_context->context, &tls_session );

            if ( tls_session.id_len > 0 )
            {
                uint32_t count;
                linked_list_node_t* removed_node = NULL;

                /* other simultaneous TLS connection on the same IP might have added/updated the entry for this IP.
                  So get the updated entry. */
                result = wiced_rtos_lock_mutex( &wiced_ssl_cache.mutex );
                if( result != WICED_SUCCESS )
                {
                    break;
                }
                entry_found = get_ssl_cache_entry( (wiced_ip_address_t*)&socket->socket.nx_tcp_socket_connect_ip, socket->socket.nx_tcp_socket_connect_port, &cache_entry );
                if ( entry_found == WICED_NOT_FOUND )
                {
                    result = linked_list_get_count( &wiced_ssl_cache.cache_list, &count );

                    if ( count > wiced_ssl_cache.max_entries )
                    {
                        linked_list_remove_node_from_rear( &wiced_ssl_cache.cache_list, &removed_node );
                    }
                    add_ssl_cache_entry( (wiced_ip_address_t*) &socket->socket.nx_tcp_socket_connect_ip, socket->socket.nx_tcp_socket_connect_port, &tls_session );
                }
                else if ( entry_found == WICED_SUCCESS )
                {
                    if( memcmp( tls_session.id, cache_entry->tls_session.id, tls_session.id_len ) != 0 )
                    {
                        /* session is present in the cache list, but server has sent new session ID due to session expiration */
                        linked_list_remove_node( &wiced_ssl_cache.cache_list, &cache_entry->this_node );
                        add_ssl_cache_entry( (wiced_ip_address_t*) &socket->socket.nx_tcp_socket_connect_ip, socket->socket.nx_tcp_socket_connect_port, &tls_session );
                    }
                }
                wiced_rtos_unlock_mutex( &wiced_ssl_cache.mutex );
            }
        }
    }while( 0 );
#endif

    return WICED_SUCCESS;

exit_with_inited_context:
    mbedtls_ssl_close_notify( &tls_context->context );
    if ( tls_context->context.conf != NULL )
    {
        mbedtls_ssl_config_cleanup( conf );
    }
    mbedtls_ssl_free( &tls_context->context );
    /* FIXME : For now, conf is malloced in tls_init_context, and mbedtls_ssl_free does memset of context so assigning conf pointer back so it can be resused. Need to fix it properly */
    tls_context->context.conf = conf;
    mbedtls_ctr_drbg_free( &ctr_drbg );
    mbedtls_entropy_free( &entropy );
    return result;
}

wiced_result_t wiced_tls_close_notify( wiced_tcp_socket_t* socket )
{
    if ( socket->tls_context != NULL )
    {
        mbedtls_ssl_close_notify( &socket->tls_context->context );
    }

    return WICED_SUCCESS;
}

wiced_result_t wiced_tls_deinit_context( wiced_tls_context_t* tls_context )
{
    if ( tls_context == NULL )
    {
        return WICED_ERROR;
    }
    mbedtls_ssl_config_free( (mbedtls_ssl_config*) tls_context->context.conf );
    if ( tls_context->context.conf != NULL )
    {
        tls_host_free((mbedtls_ssl_config*) tls_context->context.conf );
    }
    mbedtls_ssl_free( &tls_context->context );
    mbedtls_ctr_drbg_free( &ctr_drbg );
    mbedtls_entropy_free( &entropy );

    memset( tls_context, 0, sizeof( *tls_context ) );

    return WICED_SUCCESS;
}

wiced_result_t wiced_tls_reset_context( wiced_tls_context_t* tls_context )
{
#if 0
    mbedtls_ssl_session_free( &saved_session );
    mbedtls_ssl_config_free( (mbedtls_ssl_config*) tls_context->context.conf );
    tls_host_free((mbedtls_ssl_config*) tls_context->context.conf );
    mbedtls_ssl_free( &tls_context->context );
    mbedtls_ctr_drbg_free( &ctr_drbg );
    mbedtls_entropy_free( &entropy );
#endif
    return WICED_SUCCESS;
}
int wiced_crypto_random_ecc ( void *rng_state, unsigned char *output, size_t len )
{
    size_t use_len;
    int rnd;

    if( rng_state != NULL )
        rng_state  = NULL;

    while( len > 0 )
    {
        use_len = len;
        if( use_len > sizeof(int) )
            use_len = sizeof(int);

        rnd = rand();
        memcpy( output, &rnd, use_len );
        output += use_len;
        len -= use_len;
    }
    return 0;
}

wiced_result_t wiced_tls_encrypt_packet( wiced_tls_workspace_t* workspace, wiced_packet_t* packet )
{
    uint8_t* data;
    uint16_t length;
    uint16_t available;
    wiced_result_t result;
    int ret=0;

    if ( ( workspace == NULL ) || ( packet == NULL ) )
    {
        return WICED_ERROR;
    }
    if ( wiced_packet_get_data(packet, 0, &data, &length, &available) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    data -= sizeof(tls_record_header_t);
    result = (wiced_result_t) tls_host_set_packet_start((tls_packet_t*)packet, data);
    if ( result != WICED_SUCCESS)
    {
        return result;
    }

    workspace->out_hdr = data;
    workspace->out_len = workspace->out_hdr + 3;
    workspace->out_msg = workspace->out_hdr + sizeof(tls_record_header_t);
    workspace->out_iv = workspace->out_msg;
    workspace->out_msglen  = length;

    workspace->out_msgtype = MBEDTLS_SSL_MSG_APPLICATION_DATA;

    if( workspace->minor_ver >= MBEDTLS_SSL_MINOR_VERSION_2 )
    {
        memmove( workspace->out_msg + workspace->transform_out->ivlen - workspace->transform_out->fixed_ivlen, workspace->out_msg, workspace->out_msglen );
        workspace->out_msg = workspace->out_msg + workspace->transform_out->ivlen - workspace->transform_out->fixed_ivlen;
    }
    else
    {
        workspace->out_msg = workspace->out_iv;
    }

    if( ( ret = mbedtls_ssl_write_record( workspace ) ) != 0 )
    {
        WPRINT_SECURITY_ERROR( ( "Mbedtls_ssl_write_record failed \n" ) );
        return( ret );
    }

    wiced_packet_set_data_end(packet, data + workspace->out_left);
    return WICED_SUCCESS;
}

wiced_result_t wiced_tls_receive_packet( wiced_tcp_socket_t* socket, wiced_packet_t** packet, uint32_t timeout )
{
    int ret = 0;
    uint8_t* data = NULL;
    int max_fragment_len;

#if defined MBEDTLS_SSL_MAX_FRAGMENT_LENGTH
            max_fragment_len = (uint16_t) mbedtls_ssl_get_max_frag_len( &socket->tls_context->context );
#else
            max_fragment_len = MBEDTLS_SSL_MAX_CONTENT_LEN;
#endif

    socket->tls_context->context.read_timeout = timeout;

    /* check if application record had been fragmented in multiple TCP packets. Packetize received data in defragmentation buffer and give it to application */
    if ( socket->tls_context->context.defragmentation_buffer != NULL && ((socket->tls_context->context.in_hdr) == socket->tls_context->context.defragmentation_buffer ) && ( socket->tls_context->context.defragmentation_buffer_length == socket->tls_context->context.defragmentation_buffer_bytes_received))
    {
        return tls_packetize_buffered_data( &socket->tls_context->context, packet );
    }
    else
    {
        uint8_t * end_of_data = NULL;
        uint8_t* temp_record_data = NULL;

        socket->tls_context->context.packet_receive_option = TLS_RECEIVE_PACKET_IF_NEEDED;
        /* got and try to read new packet */
        if ( ( ret = mbedtls_ssl_read( &socket->tls_context->context, &data, max_fragment_len ) ) != WICED_SUCCESS )
        {
            WPRINT_SECURITY_DEBUG (("mbedtls_ssl_read failed with error : [%d] \n ",  ret ));
            if ( socket->tls_context->context.received_packet != NULL )
            {
                wiced_packet_delete((wiced_packet_t*) socket->tls_context->context.received_packet);
            }
            socket->tls_context->context.received_packet = NULL;
            socket->tls_context->context.in_msg = NULL;
            socket->tls_context->context.in_msglen = 0;
            socket->tls_context->context.in_len = NULL;
            socket->tls_context->context.in_offt = NULL;
            return ret;
        }

        if ( socket->tls_context->context.defragmentation_buffer != NULL )
        {
            if ( (socket->tls_context->context.in_hdr) == socket->tls_context->context.defragmentation_buffer )
            {
                return tls_packetize_buffered_data( &socket->tls_context->context, packet );
            }
        }

        end_of_data = socket->tls_context->context.in_msg + socket->tls_context->context.in_msglen;

        socket->tls_context->context.packet_receive_option = TLS_AVOID_NEW_RECORD_PACKET_RECEIVE;
        /* Receive all application record present in received packet */

        WPRINT_SECURITY_DEBUG (("check if multiple TLS records present in single TCP pkt\n"));
        while ( ( ret = mbedtls_ssl_read( &socket->tls_context->context, &temp_record_data, max_fragment_len ) ) == WICED_SUCCESS )
        {
            WPRINT_SECURITY_DEBUG (("Read multiple TLS records in single TCP packet got size : [%d]\n",  socket->tls_context->context.in_msglen ));
            /* Make the record data contiguous with the previous record */
            end_of_data = MEMCAT( end_of_data, temp_record_data, socket->tls_context->context.in_msglen );
        }

        if ( socket->tls_context->context.received_packet == NULL )
        {
            WPRINT_SECURITY_DEBUG (("No packet received \n"));
            return WICED_ERROR;
        }

        tls_host_set_packet_start( socket->tls_context->context.received_packet, data );
        wiced_packet_set_data_end( (wiced_packet_t*) socket->tls_context->context.received_packet, end_of_data );

        *packet = (wiced_packet_t*) socket->tls_context->context.received_packet;

        socket->tls_context->context.received_packet = NULL;
        socket->tls_context->context.received_packet_length = 0;
        socket->tls_context->context.in_msg = NULL;
        socket->tls_context->context.in_msglen = 0;
        socket->tls_context->context.in_len = NULL;
        socket->tls_context->context.in_offt = NULL;
    }

    return WICED_SUCCESS;
}

tls_result_t tls_host_set_packet_start( tls_packet_t* packet, uint8_t* start )
{
    wiced_packet_set_data_start((wiced_packet_t*)packet, start);
    return TLS_SUCCESS;
}

tls_result_t tls_host_set_packet_end( tls_packet_t* packet, uint8_t* end )
{
    wiced_packet_set_data_end((wiced_packet_t*)packet, end);
    return TLS_SUCCESS;
}

tls_result_t tls_host_get_packet_data( mbedtls_ssl_context* ssl, tls_packet_t* packet, uint32_t offset, uint8_t** data, uint16_t* data_length, uint16_t* available_data_length )
{
    uint16_t temp_length;
    uint16_t temp_available_length;

    if ( packet == NULL )
    {
        return TLS_ERROR_BAD_INPUT_DATA;
    }

    if ( ssl->transport_protocol == TLS_TCP_TRANSPORT )
    {
        wiced_result_t result = wiced_packet_get_data((wiced_packet_t*)packet, (uint16_t)offset, data, &temp_length, &temp_available_length);
        if ( result != WICED_SUCCESS)
        {
            return (tls_result_t) result;
        }
        *data_length = temp_length;
        *available_data_length = temp_available_length;
        return TLS_SUCCESS;
    }
#ifndef WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY
    else if ( ssl->transport_protocol == TLS_EAP_TRANSPORT )
    {
        besl_result_t result = supplicant_host_get_tls_data( (besl_packet_t) packet, (uint16_t)offset, data, &temp_length, &temp_available_length );
        if ( result != BESL_SUCCESS )
        {
            return (tls_result_t) result;
        }

        *data_length = temp_length;
        *available_data_length = temp_available_length;

        return TLS_SUCCESS;
    }
#endif /* WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY */
    return TLS_ERROR_BAD_INPUT_DATA;
}

/*
 * Calculates the maximium amount of payload that can fit in a given sized buffer
 */
wiced_result_t wiced_tls_calculate_overhead( wiced_tls_workspace_t* context, uint16_t available_space, uint16_t* header, uint16_t* footer )
{
    *header = 0;
    *footer = 0;
     mbedtls_cipher_mode_t mode;

    if( context != NULL && context->state == MBEDTLS_SSL_HANDSHAKE_OVER)
    {
        /* Add TLS record size */
        *header = sizeof(tls_record_header_t);

        mode = mbedtls_cipher_get_cipher_mode( &context->transform_out->cipher_ctx_enc );

        if( mode == MBEDTLS_MODE_GCM || mode == MBEDTLS_MODE_CCM )
        {
            unsigned char taglen = context->transform_out->ciphersuite_info->flags &
                                       MBEDTLS_CIPHERSUITE_SHORT_TAG ? 8 : 16;

            *footer += ( taglen + ( context->transform_out->ivlen - context->transform_out->fixed_ivlen ));
        }

        if ( mode == MBEDTLS_MODE_CBC )
        {
            *footer += ( available_space - *header - *footer ) % context->transform_out->ivlen + 1;

            if ( context->minor_ver >= MBEDTLS_SSL_MINOR_VERSION_2 )
            {
                *footer += context->transform_out->ivlen;
            }
        }

        if( mode == MBEDTLS_MODE_STREAM || ( mode == MBEDTLS_MODE_CBC ) )
         {
             *footer +=  context->transform_out->maclen;
         }
    }

    return WICED_SUCCESS;
}

tls_result_t tls_host_create_buffer( mbedtls_ssl_context* ssl, uint8_t** buffer, uint16_t buffer_size )
{
    wiced_assert("", ssl->outgoing_packet == NULL);

    /* Round requested buffer size up to next 64 byte chunk (required if encryption is active) */
    buffer_size = (uint16_t) ROUND_UP(buffer_size, 64);

    /* Check if requested buffer fits within a single MTU */
    if ( ( buffer_size < 1300) && ( ssl->transport_protocol == TLS_TCP_TRANSPORT ) ) /* TODO: Fix this */
    {
        uint16_t actual_packet_size;
        if ( wiced_packet_create_tcp( ssl->p_bio, buffer_size, (wiced_packet_t**) &ssl->outgoing_packet, buffer, &actual_packet_size ) != WICED_SUCCESS )
        {
            *buffer = NULL;
            /* Return is not currently used; however, need to define a TLS enum value for this case eventually */
            return ( tls_result_t )1;
        }
        if ( ssl->state == MBEDTLS_SSL_HANDSHAKE_OVER )
        {
            /* this doesn't need the extra space for the encryption header that a normal TLS socket would use - remove it */
            *buffer -= sizeof(tls_record_header_t);
            wiced_packet_set_data_start((wiced_packet_t*) ssl->outgoing_packet, *buffer);
        }
    }
    else if ( ssl->transport_protocol == TLS_UDP_TRANSPORT )
    {
        uint16_t actual_packet_size;

        if ( wiced_packet_create_udp( (wiced_udp_socket_t*) ssl->receive_context, buffer_size, (wiced_packet_t**) &ssl->outgoing_packet, buffer, &actual_packet_size ) != WICED_SUCCESS )
        {
            *buffer = NULL;
            return TLS_ERROR;
        }

        *buffer -= sizeof(dtls_record_header_t);
        wiced_packet_set_data_start( (wiced_packet_t*) ssl->outgoing_packet, *buffer );

    }
    else
    {
        /* Requested size bigger than a MTU or TLS_EAP_TRANSPORT */
        /* Malloc space */
        *buffer = tls_host_malloc("tls", buffer_size);
        if ( *buffer == NULL )
        {
            return TLS_ERROR_OUT_OF_MEMORY;
        }
        memset( *buffer, 0, buffer_size );
        ssl->out_buffer_size = buffer_size;
    }

    return TLS_SUCCESS;
}

int ssl_flush_output( void* socket, const uint8_t* buffer, size_t length )
{
    wiced_tcp_socket_t* tcp_socket = ( wiced_tcp_socket_t*) socket;
    mbedtls_ssl_context* ssl = &tcp_socket->tls_context->context;
    size_t temp_length = length;

    if ( ssl->transport_protocol == TLS_TCP_TRANSPORT )
    {
        if (ssl->outgoing_packet != NULL)
        {
            wiced_packet_set_data_start((wiced_packet_t*)ssl->outgoing_packet, (uint8_t*)buffer);
            wiced_packet_set_data_end((wiced_packet_t*)ssl->outgoing_packet, ((uint8_t*)buffer + length ));
            tls_host_send_tcp_packet(tcp_socket, ssl->outgoing_packet);
            ssl->outgoing_packet = NULL;
            ssl->out_buffer_size = 0;
        }
        else
        {
            uint16_t      actual_packet_size;
            tls_packet_t* temp_packet;
            uint8_t*      packet_buffer;
            uint8_t*      data = (uint8_t*) buffer;

            while (temp_length != 0)
            {
                uint16_t amount_to_copy;

                if ( wiced_packet_create_tcp( tcp_socket, (uint16_t)temp_length, (wiced_packet_t**) &temp_packet, &packet_buffer, &actual_packet_size ) != WICED_SUCCESS )
                {
                    tls_host_free((uint8_t*) buffer);
                    buffer = NULL;
                    return ( tls_result_t )1;
                }
//                if ( ssl->state == SSL_HANDSHAKE_OVER )
//                {
//                    /* this doesn't need the extra space for the encryption header that a normal TLS socket would use - remove it */
//                    packet_buffer -= sizeof(tls_record_header_t);
//                    wiced_packet_set_data_start((wiced_packet_t*) temp_packet, packet_buffer);
//                }
                amount_to_copy = (uint16_t) MIN(temp_length, actual_packet_size);
                packet_buffer = MEMCAT(packet_buffer, data, amount_to_copy);
                data   += amount_to_copy;
                temp_length -= amount_to_copy;
                wiced_packet_set_data_end((wiced_packet_t*)temp_packet, packet_buffer );
                tls_host_send_tcp_packet(tcp_socket, temp_packet);
            }

            tls_host_free( (uint8_t*) buffer);
            buffer = NULL;
        }
    }
    return length;
}


#ifndef WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY
wiced_result_t wiced_supplicant_start_tls( supplicant_workspace_t* supplicant, wiced_tls_endpoint_type_t type, wiced_tls_certificate_verification_t verification )
{
    host_rtos_delay_milliseconds( 10 );
    return wiced_generic_start_tls_with_ciphers( supplicant->tls_context, supplicant, type, verification, NULL, TLS_EAP_TRANSPORT );
}

wiced_result_t wiced_supplicant_enable_tls( supplicant_workspace_t* supplicant, void* context )
{
    supplicant->tls_context = context;
    return WICED_SUCCESS;
}

wiced_result_t wiced_tls_receive_eap_packet( supplicant_workspace_t* supplicant, besl_packet_t* packet, uint32_t timeout )
{
    int ret = 0;
    uint8_t* data = NULL;
    int max_fragment_len;

#if defined MBEDTLS_SSL_MAX_FRAGMENT_LENGTH
    max_fragment_len = (uint16_t) mbedtls_ssl_get_max_frag_len( &supplicant->tls_context->context );
#else
    max_fragment_len = MBEDTLS_SSL_MAX_CONTENT_LEN;
#endif

    supplicant->tls_context->context.read_timeout = timeout;

    if ( supplicant->tls_context->context.defragmentation_buffer_bytes_processed == supplicant->tls_context->context.defragmentation_buffer_bytes_received )
    {
        ret = mbedtls_ssl_read( &supplicant->tls_context->context, &data, max_fragment_len );
        if ( ret < 0 )
        {
            WPRINT_SECURITY_ERROR ( (" wiced_tls_receive_packet failed  : %d \n", ret) );
            if ( supplicant->tls_context->context.received_packet != NULL )
            {
                wiced_packet_delete( (wiced_packet_t*) supplicant->tls_context->context.received_packet );
            }
            supplicant->tls_context->context.received_packet = NULL;
            supplicant->tls_context->context.in_msg = NULL;
            supplicant->tls_context->context.in_msglen = 0;
            supplicant->tls_context->context.in_len = NULL;
            supplicant->tls_context->context.in_offt = NULL;
            return WICED_ERROR;
        }
    }
    /* Check if this record has been defragmented */
    if ( ( supplicant->tls_context->context.in_hdr ) == supplicant->tls_context->context.defragmentation_buffer )
    {
        return tls_eap_buffered_data( &supplicant->tls_context->context, packet );

    }
    else if ( ( ( supplicant->tls_context->context.in_hdr ) != supplicant->tls_context->context.defragmentation_buffer ) )
    {
        uint8_t * end_of_data = NULL;
        //uint8_t* temp_record_data = NULL;

        /* Set the packet start and end */
        if ( supplicant->tls_context->context.received_packet == NULL )
        {
            return WICED_ERROR;
        }

        end_of_data = supplicant->tls_context->context.in_msg + supplicant->tls_context->context.in_msglen;

        //        while ( supplicant->tls_context->context.received_packet_bytes_skipped != supplicant->tls_context->context.received_packet_length )
        //        {
        //            if ( ( ret = mbedtls_ssl_read( &supplicant->tls_context->context, &temp_record_data, max_fragment_len ) ) > 0 )
        //            {
        //                /* Make the record data contiguous with the previous record */
        //                end_of_data = MEMCAT( end_of_data, temp_record_data, ret );
        //            }
        //        }

        tls_host_set_packet_start( supplicant->tls_context->context.received_packet, data );
        tls_host_set_packet_end( supplicant->tls_context->context.received_packet, end_of_data );

        (*packet) = (besl_packet_t) supplicant->tls_context->context.received_packet;

        supplicant->tls_context->context.received_packet = NULL;
        supplicant->tls_context->context.received_packet_length = 0;
        supplicant->tls_context->context.received_packet_bytes_skipped = 0;
        supplicant->tls_context->context.in_msg = NULL;
        supplicant->tls_context->context.in_msglen = 0;
        supplicant->tls_context->context.in_len = NULL;
        supplicant->tls_context->context.in_offt = NULL;
    }

    return WICED_SUCCESS;
}
#endif /* WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY */

tls_result_t tls_host_send_tcp_packet( void* context, tls_packet_t* packet )
{
    if ( network_tcp_send_packet( (wiced_tcp_socket_t*)context, (wiced_packet_t*)packet) != WICED_SUCCESS )
    {
        wiced_packet_delete((wiced_packet_t*)packet);
    }
    return TLS_SUCCESS;
}

wiced_result_t wiced_tls_set_extension(wiced_tls_context_t* context, wiced_tls_extension_t* extension)
{
    int ret;
    wiced_result_t result = WICED_SUCCESS;

    UNUSED_PARAMETER(ret);

    /* set server name and maximum fragment length extension */
    if(context != NULL)
    {
        switch (extension->type)
        {
            case TLS_EXTENSION_TYPE_SERVER_NAME:
            {
#if defined MBEDTLS_SSL_SERVER_NAME_INDICATION
                if( ( ret = mbedtls_ssl_set_hostname( &context->context, (char*)extension->extension_data.server_name ) ) != 0 )
                {
                    WPRINT_SECURITY_ERROR ( ( " failed\n  ! mbedtls_ssl_set_hostname returned %d\n\n", ret ) );
                    return WICED_ERROR;
                }
#else
                WPRINT_SECURITY_INFO(("MBEDTLS_SSL_SERVER_NAME_INDICATION Flag is not enabled in config.h" ));
                return WICED_ERROR;
#endif
            }
            break;
            case TLS_EXTENSION_TYPE_MAX_FRAGMENT_LENGTH:
            {
#if defined MBEDTLS_SSL_MAX_FRAGMENT_LENGTH
                if( ( ret = mbedtls_ssl_conf_max_frag_len( (mbedtls_ssl_config*) context->context.conf, extension->extension_data.max_fragment_length ) ) != 0 )
                {
                    WPRINT_SECURITY_ERROR (( " failed\n  ! mbedtls_ssl_conf_max_frag_len returned %d\n\n", ret ));
                    return WICED_ERROR;
                }
#else
                WPRINT_SECURITY_INFO(("MBEDTLS_SSL_MAX_FRAGMENT_LENGTH Flag is not enabled in config.h" ));
                return WICED_ERROR;
#endif
            }
            break;
            case TLS_EXTENSION_TYPE_APPLICATION_LAYER_PROTOCOL_NEGOTIATION:
            {
#if defined MBEDTLS_SSL_ALPN
               if( ( ret = tls_add_alpn_extension( context, extension ) ) != 0 )
               {
                   WPRINT_SECURITY_ERROR (( " failed\n  ! tls_add_alpn_extension returned %d\n\n", ret ));
                   return WICED_ERROR;
               }
#else
                WPRINT_SECURITY_INFO(("MBEDTLS_SSL_ALPN Flag is not enabled in config.h" ));
                return WICED_ERROR;
#endif
            }
            break;
            default:
                WPRINT_SECURITY_INFO(("Unknown Extension. Currently not supported by WICED" ));
                result = WICED_ERROR;
                break;
        }
    }

    return result;
}

wiced_bool_t wiced_tls_is_encryption_enabled( wiced_tcp_socket_t* socket )
{
    if ( socket->tls_context != NULL && socket->tls_context->context.state == MBEDTLS_SSL_HANDSHAKE_OVER )
    {
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

/*
 * TLS support functions
 */
tls_result_t tls_host_free_packet( tls_packet_t* packet )
{
    wiced_packet_delete((wiced_packet_t*)packet);
    return TLS_SUCCESS;
}

void* tls_host_get_defragmentation_buffer ( uint16_t size )
{
    return malloc_named( "tls", size );
}

void  tls_host_free_defragmentation_buffer( void* buffer )
{
    tls_host_free( buffer );
    /* clear address from stack for security */
    buffer = NULL;
}

tls_result_t tls_host_receive_packet(mbedtls_ssl_context* ssl, tls_packet_t** packet, uint32_t timeout)
{
    tls_result_t              result = TLS_RECEIVE_FAILED;
    switch(ssl->transport_protocol)
    {
        case TLS_TCP_TRANSPORT:
            result = (tls_result_t) network_tcp_receive( (wiced_tcp_socket_t*) ssl->receive_context, (wiced_packet_t**) packet, timeout );
            break;

        case TLS_EAP_TRANSPORT:
#ifndef WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY
            result = (tls_result_t) supplicant_receive_eap_tls_packet( ssl->receive_context, packet, timeout );
            break;
#endif /* WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY */

        case TLS_UDP_TRANSPORT:
        default:
            wiced_assert( "unknown transport", 1 == 0 );
            break;
    }

    return result;
}

tls_result_t tls_calculate_encrypt_buffer_length( mbedtls_ssl_context* context, uint16_t* required_buff_size, uint16_t payload_size)
{
    uint16_t padlen = 0;
    mbedtls_cipher_mode_t mode;
    tls_result_t result = TLS_SUCCESS;

    if ( context != NULL && context->state == MBEDTLS_SSL_HANDSHAKE_OVER )
    {

       mode = mbedtls_cipher_get_cipher_mode( &context->transform_out->cipher_ctx_enc );

      *required_buff_size = payload_size;

      if( mode == MBEDTLS_MODE_GCM || mode == MBEDTLS_MODE_CCM )
      {
          unsigned char taglen = context->transform_out->ciphersuite_info->flags &
                                    MBEDTLS_CIPHERSUITE_SHORT_TAG ? 8 : 16;

          *required_buff_size += ( taglen + ( context->transform_out->ivlen - context->transform_out->fixed_ivlen ));
      }

      if ( mode == MBEDTLS_MODE_CBC )
      {
          padlen = context->transform_out->ivlen - (*required_buff_size + 1) % context->transform_out->ivlen;
          if ( padlen == context->transform_out->ivlen )
          {
              padlen = 0;
          }

          *required_buff_size += padlen + 1;

          if ( context->minor_ver >= MBEDTLS_SSL_MINOR_VERSION_2 )
          {
              *required_buff_size += context->transform_out->ivlen;
          }
      }

      if( mode == MBEDTLS_MODE_STREAM || ( mode == MBEDTLS_MODE_CBC ) )
      {
          *required_buff_size +=  context->transform_out->maclen;
      }

    }
    else
    {
        result = TLS_ERROR;
    }

    return result;
}

#if defined(MBEDTLS_SSL_ALPN)
static int tls_add_alpn_extension( wiced_tls_context_t* context, wiced_tls_extension_t* extension )
{
    int ret = 0, i = 0;
    uint8_t* p = NULL;

    p = (uint8_t*) extension->extension_data.alpn_protocol_list;

    /* Leave room for a final NULL in alpn_list */
    while( i < ((int)(sizeof(alpn_list) / sizeof(alpn_list[0])) - 1)  && *p != '\0' )
    {
        alpn_list[i++] = (char*)p;

        /* Terminate the current string and move on to next one */
        while( *p != ',' && *p != '\0' )
        {
            p++;
            if( *p == ',' )
            {
                *p++ = '\0';
                break;
            }
        }
    }

    if( ( ret = mbedtls_ssl_conf_alpn_protocols( (mbedtls_ssl_config*) context->context.conf, (const char**) alpn_list ) ) != 0 )
    {
        WPRINT_SECURITY_ERROR ( ( " failed\n  ! mbedtls_ssl_conf_alpn_protocols returned %d\n\n", ret ) );
        return ret;
    }

    return ret;
}
#endif

#ifndef WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY
int eap_ssl_flush_output( void* context, const uint8_t* buffer, size_t length )
{
    supplicant_workspace_t* supplicant = (supplicant_workspace_t*) context;
    mbedtls_ssl_context* ssl = &supplicant->tls_context->context;
    if ( ssl->transport_protocol == TLS_EAP_TRANSPORT )
    {
        supplicant_host_send_eap_tls_fragments( (supplicant_workspace_t*) supplicant, (uint8_t* )buffer, length );
        ssl->out_buffer_size = 0;
        tls_host_free( (uint8_t*) buffer );
        buffer = NULL;
    }

    return length;
}

static wiced_result_t tls_eap_buffered_data( wiced_tls_workspace_t* context, besl_packet_t* packet )
{
    uint16_t length;
    uint16_t available_length = 0;
    uint16_t record_length;
    besl_result_t result;
    uint32_t amount_to_copy;
    eap_packet_t* header;

    (void) available_length;

    record_length = (uint16_t) ( (uint16_t) context->in_msglen + sizeof(tls_record_header_t) );

    result = besl_host_create_packet( packet, (uint16_t) MIN(1024, record_length - context->defragmentation_buffer_bytes_processed) );
    if ( result != BESL_SUCCESS )
    {
        return (wiced_result_t) result;
    }
    header = (eap_packet_t*) besl_host_get_data( *packet );
    length = besl_host_get_packet_size( *packet );

    amount_to_copy = (uint32_t) MIN( length, record_length - context->defragmentation_buffer_bytes_processed );
    memcpy( header, &context->defragmentation_buffer[ context->defragmentation_buffer_bytes_processed ], amount_to_copy );

    context->defragmentation_buffer_bytes_processed = (uint16_t) ( context->defragmentation_buffer_bytes_processed + amount_to_copy );

    /* Check if we've returned all the data to the user */
    if ( context->defragmentation_buffer_bytes_processed >= record_length )
    {
        tls_host_free_defragmentation_buffer( context->defragmentation_buffer );
        context->defragmentation_buffer = 0;
        // context->current_record = NULL;
    }

    return WICED_SUCCESS;
}

static int eap_ssl_receive_packet( void *ctx, unsigned char **buf, size_t len )
{
    wiced_result_t result = WICED_SUCCESS;
    uint16_t available_data_length = 0;
    uint16_t length = 0;
    supplicant_workspace_t* workspace = (supplicant_workspace_t*) ctx;

    WPRINT_SECURITY_DEBUG (("TLS library asked for [%d] bytes \n", len ));

    /* Case : Check if we are halfway through a fragmented record
     * These handles case when single TCP packet has multiple record [ex. Record R1 & R2 ]. and R2 record in packet1 is fragmented into multiple TCP packets. In this case R2 content in packet1 will
     * be copied into defragmentation buffer and then give content of R1 to application. When MBEDTLS calls receive callback for R2 content it should fall in this case to receive full content for R2.
     * */
    if ( workspace->tls_context->context.defragmentation_buffer != NULL && workspace->tls_context->context.defragmentation_buffer_bytes_received != workspace->tls_context->context.defragmentation_buffer_length )
    {
        WPRINT_SECURITY_DEBUG (("halfway TLS defrag ## defragmentation buffer bytes received [%d] total defragmentaion bytes [%d] \n ", workspace->tls_context->context.defragmentation_buffer_bytes_received, workspace->tls_context->context.defragmentation_buffer_length ));
        /* Check if we already have number of bytes ( asked by MBEDTLS ) present in defragmentation buffer */
        if ( len < ( workspace->tls_context->context.defragmentation_buffer_bytes_received - workspace->tls_context->context.defragmentation_buffer_bytes_skipped ) )
        {
            WPRINT_SECURITY_DEBUG (("Already have [%d] bytes in defragmentation buffer, asked for [%d] \n ", workspace->tls_context->context.defragmentation_buffer_bytes_received - workspace->tls_context->context.defragmentation_buffer_bytes_skipped, len ));WPRINT_SECURITY_DEBUG (("defragmentation buffer bytes skipped [%d] \n ", workspace->tls_context->context.defragmentation_buffer_bytes_skipped ));

            *buf = workspace->tls_context->context.defragmentation_buffer;
            workspace->tls_context->context.defragmentation_buffer_bytes_skipped += len;
            return WICED_SUCCESS;
        }
        else
        {
            result = tls_complete_record_defragmentation( &workspace->tls_context->context, workspace->tls_context->context.read_timeout );
            if ( result != (wiced_result_t) TLS_SUCCESS )
            {
                WPRINT_SECURITY_DEBUG (("halfway TLS record defragmentation failed with result : [%d] \n", result ));
                workspace->tls_context->context.defragmentation_buffer_bytes_skipped = 0;
                return result;
            }

            /* We dont have enough data in defragmentation buffer, So go and read the multiple packets and accumulate whole data */
            WPRINT_SECURITY_DEBUG (("halfway defragmentation of TLS record completed \n"));

            *buf = workspace->tls_context->context.defragmentation_buffer + workspace->tls_context->context.defragmentation_buffer_bytes_skipped;
            return result;
        }
    }

    /* If previously received packet is consumed, then go and read the next packet */
    if ( workspace->tls_context->context.received_packet == NULL )
    {
        result = tls_host_receive_packet( &workspace->tls_context->context, &workspace->tls_context->context.received_packet, workspace->tls_context->context.read_timeout );

        if ( result != WICED_SUCCESS )
        {
            WPRINT_SECURITY_DEBUG (("network_tcp_receive failed with result : [%d] \n", result ));
            return result;
        }

        workspace->tls_context->context.received_packet_bytes_skipped = 0;
        workspace->tls_context->context.received_packet_length = 0;

        result = tls_host_get_packet_data( &workspace->tls_context->context, workspace->tls_context->context.received_packet, workspace->tls_context->context.received_packet_bytes_skipped, (uint8_t**) ( buf ), (uint16_t*) &workspace->tls_context->context.received_packet_length, &available_data_length );

        if ( result != WICED_SUCCESS )
        {
            WPRINT_SECURITY_ERROR ( ("Error in getting packet data : %d \n", result ) );
            return result;
        }

        WPRINT_SECURITY_DEBUG (("Received new TCP packet with length [%d] \n", workspace->tls_context->context.received_packet_length ));

    }
    else
    {
        WPRINT_SECURITY_DEBUG (("Skip [%d] no of bytes from TCP received packet with length : [%d]\n", workspace->tls_context->context.received_packet_bytes_skipped, workspace->tls_context->context.received_packet_length ));
        result = tls_host_get_packet_data( &workspace->tls_context->context, workspace->tls_context->context.received_packet, workspace->tls_context->context.received_packet_bytes_skipped, (uint8_t**) ( buf ), (uint16_t*) &length, &available_data_length );
        if ( result != WICED_SUCCESS )
        {
            WPRINT_SECURITY_DEBUG (("wiced_packet_get_data failed with result : [%d] \n", result ));
            return result;
        }
    }

    /* Case : number of bytes asked by MBEDTLS is more than MTU size, so it is defragmentation case.
     * Allocate defragmentation buffer, Receive multiple TCP packets till all the data received, and copy content into defragmentation buffer.
     */
    if ( len > ( workspace->tls_context->context.received_packet_length - workspace->tls_context->context.received_packet_bytes_skipped ) )
    {
        /* Create defragmentation buffer of size defragmentation_buffer_length which includes TLS record header + data content. */
        workspace->tls_context->context.defragmentation_buffer_length = len + sizeof(tls_record_header_t);
        workspace->tls_context->context.defragmentation_buffer = tls_host_get_defragmentation_buffer( workspace->tls_context->context.defragmentation_buffer_length );
        if ( workspace->tls_context->context.defragmentation_buffer == NULL )
        {
            WPRINT_SECURITY_ERROR(("No memory available to allocate for defragmentation buffer \n"));
            tls_host_free_packet( workspace->tls_context->context.received_packet );
            workspace->tls_context->context.received_packet = NULL;
            return TLS_ERROR_OUT_OF_MEMORY;
        }

        WPRINT_SECURITY_DEBUG (("Defragmentaion case : copying %d bytes to defrag buffer 0x%08x\n", length + sizeof(tls_record_header_t), (unsigned int)workspace->tls_context->context.defragmentation_buffer ));

        /* Copy available content into defragmentation buffer including TLS record header */
        memcpy( workspace->tls_context->context.defragmentation_buffer, *buf - sizeof(tls_record_header_t), length + sizeof(tls_record_header_t) );
        workspace->tls_context->context.defragmentation_buffer_bytes_received = length + sizeof(tls_record_header_t);

        /* set the value of defragmentation_buffer_bytes_processed to minimum of length we received or TLS record header + encryption IV as these will be processed while decrypting the record*/
        if ( workspace->tls_context->context.state == MBEDTLS_SSL_HANDSHAKE_OVER )
        {
            workspace->tls_context->context.defragmentation_buffer_bytes_processed = MIN(length, ( sizeof(tls_record_header_t) + (workspace->tls_context->context.transform_in->ivlen - workspace->tls_context->context.transform_in->fixed_ivlen) ));
        }

        /* Application doesnt want us to read new packet */
        if ( workspace->tls_context->context.packet_receive_option == TLS_AVOID_NEW_RECORD_PACKET_RECEIVE )
        {
            WPRINT_SECURITY_DEBUG (("Avoid receiving new packet \n"));
            return TLS_NO_DATA;
        }

        // "Consume" the data from the packet
        tls_consume_received_packet_bytes( &workspace->tls_context->context, length + workspace->tls_context->context.received_packet_bytes_skipped );

        result = tls_complete_record_defragmentation( &workspace->tls_context->context, workspace->tls_context->context.read_timeout );
        if ( result != (wiced_result_t) TLS_SUCCESS )
        {
            WPRINT_SECURITY_DEBUG (("TLS record defragmentation failed with result : [%d] \n", result ));
            return result;
        }

        WPRINT_SECURITY_DEBUG (("Completed TLS record defragmentation. Received [%d] bytes \n", workspace->tls_context->context.defragmentation_buffer_bytes_received ));

        *buf = workspace->tls_context->context.defragmentation_buffer + sizeof(tls_record_header_t);
    }
    else
    {
        workspace->tls_context->context.received_packet_bytes_skipped += len;
    }

    return WICED_SUCCESS;
}
#endif /* WICED_CONFIG_DISABLE_ENTERPRISE_SECURITY */
