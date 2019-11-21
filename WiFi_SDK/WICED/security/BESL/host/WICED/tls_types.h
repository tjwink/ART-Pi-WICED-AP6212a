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

#include "besl_structures.h"
#include "crypto_structures.h"
#include <time.h>
#include "cipher_suites.h"
#include "wwd_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "mbedtls/pk.h"
#include "mbedtls/aes.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/ssl.h"
#include "mbedtls/ssl_internal.h"
/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
/* WICED_MBEDTLS Start */
#define WICED_TLS_CONTEXT_ID                     (0xd309c08b)
#define MAX_EXT_DATA_LENGTH                      256
#define  TLS_HANDSHAKE_PACKET_TIMEOUT_MS         (20000)

typedef struct mbedtls_ssl_context  wiced_tls_workspace_t;
typedef struct mbedtls_ssl_session  wiced_tls_session_t;
typedef struct mbedtls_x509_crt wiced_tls_certificate_t;
typedef uint32_t             tls_packet_t;

typedef int (*wiced_tls_sign_certificate_verify)(  void* key ,rsa_hash_id_t hash_id, int32_t hashlen, const unsigned char *hash, unsigned char *sign, uint32_t* key_length, wiced_tls_key_type_t type );

typedef enum
{
    TLS_CERTIFICATE_IN_PEM_FORMAT,
    TLS_CERTIFICATE_IN_DER_FORMAT,
} wiced_tls_certificate_format_t;

typedef enum
{
    TLS_NO_VERIFICATION       = 0,
    TLS_VERIFICATION_OPTIONAL = 1,
    TLS_VERIFICATION_REQUIRED = 2,
} wiced_tls_certificate_verification_t;

typedef enum
{
    WICED_TLS_SIMPLE_CONTEXT,
    WICED_TLS_ADVANCED_CONTEXT,
} wiced_tls_context_type_t;

typedef enum
{
    WICED_TLS_AS_CLIENT = 0,
    WICED_TLS_AS_SERVER = 1
} wiced_tls_endpoint_type_t;

/*
 * SSL state machine
 */
typedef enum
{
    SSL_HELLO_REQUEST,
    SSL_CLIENT_HELLO,
    SSL_SERVER_HELLO,
    SSL_SERVER_CERTIFICATE,
    SSL_SERVER_KEY_EXCHANGE,
    SSL_CERTIFICATE_REQUEST,
    SSL_SERVER_HELLO_DONE,
    SSL_CLIENT_CERTIFICATE,
    SSL_CLIENT_KEY_EXCHANGE,
    SSL_CERTIFICATE_VERIFY,
    SSL_CLIENT_CHANGE_CIPHER_SPEC,
    SSL_CLIENT_FINISHED,
    SSL_SERVER_CHANGE_CIPHER_SPEC,
    SSL_SERVER_FINISHED,
    SSL_FLUSH_BUFFERS,
    SSL_HANDSHAKE_OVER
} tls_states_t;

typedef struct
{
    linked_list_node_t      this_node;

    mbedtls_pk_context      private_key;
    mbedtls_x509_crt        certificate;
} wiced_tls_credentials_info_t;

typedef struct
{
    linked_list_t                      credentials;
    wiced_tls_sign_certificate_verify  custom_sign;
} wiced_tls_identity_t;

typedef struct
{
    uint32_t                 context_id;
    wiced_tls_workspace_t    context;
    /* This session pointer is only used to resume connection for client, If application/library wants to resume connection it needs to pass pointer of previous stored session */
    wiced_tls_session_t*     session;
    wiced_tls_identity_t*    identity;
} wiced_tls_context_t;

typedef enum
{
    TLS_RESULT_LIST     (  TLS_      )  /* 5000 - 5999 */
} tls_result_t;

#pragma pack(1)

typedef struct
{
 uint8_t type;
 uint8_t major_version;
 uint8_t minor_version;
 uint16_t length;
 uint8_t message[1];
 }tls_record_t;
/* Helper structure to create TLS record */

typedef struct
{
    uint8_t  type;
    uint8_t  major_version;
    uint8_t  minor_version;
    uint16_t length;
} tls_record_header_t;

#pragma pack()

typedef enum
{
    TLS_FRAGMENT_LENGTH_512   =  1,
    TLS_FRAGMENT_LENGTH_1024  =  2,
    TLS_FRAGMENT_LENGTH_2048  =  3,
    TLS_FRAGMENT_LENGTH_4096  =  4,
} wiced_tls_max_fragment_length_t;

/* Reference: http://www.iana.org/assignments/tls-extensiontype-values/tls-extensiontype-values.xhtml */
typedef enum
{
    TLS_EXTENSION_TYPE_SERVER_NAME                              =  0,
    TLS_EXTENSION_TYPE_MAX_FRAGMENT_LENGTH                      =  1,
    TLS_EXTENSION_TYPE_APPLICATION_LAYER_PROTOCOL_NEGOTIATION   = 16,
} wiced_tls_extension_type_t;

typedef struct
{
    wiced_tls_extension_type_t           type;
    union
    {
        uint8_t*                         server_name;
        wiced_tls_max_fragment_length_t  max_fragment_length;
        /* Pointer to a 'null' terminated string, which contains a comma separated ALPN Protocol list. Currently MAX_ALPN_PROTOCOLS_LIST is 10 */
        uint8_t*                         alpn_protocol_list;
    } extension_data;
} wiced_tls_extension_t;

struct _ssl_extension
{
    uint16_t id;
    uint16_t used;
    uint16_t sz;
    uint8_t data[MAX_EXT_DATA_LENGTH+1];
};

/*
 * TODO : Need to remove this global options and keep it in configuration file.
 * global options used to configure MBEDTLS
 */
typedef struct
{
    const char *server_name;    /* hostname of the server (client only)     */
    const char *server_addr;    /* address of the server (client only)      */
    const char *server_port;    /* port on which the ssl service runs       */
    int debug_level;            /* level of debugging                       */
    int nbio;                   /* should I/O be blocking?                  */
    uint32_t read_timeout;      /* timeout on mbedtls_ssl_read() in milliseconds    */
    int max_resend;             /* DTLS times to resend on read timeout     */
    const char *request_page;   /* page on server to request                */
    int request_size;           /* pad request with header to requested size */
    const char *ca_file;        /* the file with the CA certificate(s)      */
    const char *ca_path;        /* the path with the CA certificate(s) reside */
    const char *crt_file;       /* the file with the client certificate     */
    const char *key_file;       /* the file with the client key             */
    const char *psk;            /* the pre-shared key                       */
    const char *ecjpake_pw;     /* the EC J-PAKE password                   */
    const char *psk_identity;   /* the pre-shared key identity              */
    int force_ciphersuite[2];   /* protocol/ciphersuite to use, or all      */
    int renegotiation;          /* enable / disable renegotiation           */
    int allow_legacy;           /* allow legacy renegotiation               */
    int renegotiate;            /* attempt renegotiation?                   */
    int renego_delay;           /* delay before enforcing renegotiation     */
    int exchanges;              /* number of data exchanges                 */
    int min_version;            /* minimum protocol version accepted        */
    int max_version;            /* maximum protocol version accepted        */
    int arc4;                   /* flag for arc4 suites support             */
    int auth_mode;              /* verify mode for connection               */
    unsigned char mfl_code;     /* code for maximum fragment length         */
    int trunc_hmac;             /* negotiate truncated hmac or not          */
    int recsplit;               /* enable record splitting?                 */
    int dhmlen;                 /* minimum DHM params len in bits           */
    int reconnect;              /* attempt to resume session                */
    int reco_delay;             /* delay in seconds before resuming session */
    int reconnect_hard;         /* unexpectedly reconnect from the same port */
    int tickets;                /* enable / disable session tickets         */
    const char *alpn_string;    /* ALPN supported protocols                 */
    int transport;              /* TLS or DTLS?                             */
    uint32_t hs_to_min;         /* Initial value of DTLS handshake timer    */
    uint32_t hs_to_max;         /* Max value of DTLS handshake timer        */
    int fallback;               /* is this a fallback connection?           */
    int extended_ms;            /* negotiate extended master secret?        */
    int etm;                    /* negotiate encrypt then mac?              */
} options;

typedef struct _ssl_extension ssl_extension;

/* WICED_MBEDTLS End */

#ifdef __cplusplus
} /*extern "C" */
#endif
