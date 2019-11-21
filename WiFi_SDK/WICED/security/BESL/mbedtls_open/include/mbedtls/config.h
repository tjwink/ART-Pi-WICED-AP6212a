/*
 *  Minimal configuration for TLS NSA Suite B Profile (RFC 6460)
 *
 *  Copyright (C) 2006-2015, ARM Limited, All Rights Reserved
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *  This file is part of mbed TLS (https://tls.mbed.org)
 */
/*
 * Minimal configuration for TLS NSA Suite B Profile (RFC 6460)
 *
 * Distinguishing features:
 * - no RSA or classic DH, fully based on ECC
 * - optimized for low RAM usage
 *
 * Possible improvements:
 * - if 128-bit security is enough, disable secp384r1 and SHA-512
 * - use embedded certs in DER format and disable PEM_PARSE_C and BASE64_C
 *
 * See README.txt for usage instructions.
 */

#ifndef MBEDTLS_CONFIG_H
#define MBEDTLS_CONFIG_H

#include "wiced_defaults.h"

/* System support */
#define MBEDTLS_HAVE_ASM
//#define MBEDTLS_HAVE_TIME

/* Enabling this macro improves TLS handshake performance. If this macro is enabled
 * mbedtls will invoke uECC functions for ECC crypto operations, else it will use the
 * mbedtls ecc functions. uECC does not support advanced security curves, so if
 * this flag is enabled advanced security curves are not supported.
 * Also this flag will not have any impact with deterministic ecdsa signing.
 * if MBEDTLS_ECDSA_DETERMINISTIC flag is enabled mbedtls ecc functions are used.
 */
#ifdef WICED_CONFIG_ENABLE_MBEDTLS_ECC_ALT
#define MBEDTLS_WICED_ECC_ALT
#endif

/* Enabling below macro improves the TLS handshake performance for DHE cipher suites. If this macro is enabled,
 * private key exponent x, required to compute g^X mod p in DHE operation, will be 256 bits(32 bytes).To get the
 * 128 bit of symmetric security 256 bits of private key sufficient. Disable flag, if requirement for symmetric
 * key size > 128 bits. Below are the TLS handshake time taken with local server with and without enabling below flag
 *
 *   TLS-DHE-RSA-WITH-AES-256-CBC-SHA256 ( Disabling below flag ) - 1842 ms
 *   TLS-DHE-RSA-WITH-AES-256-CBC-SHA256 ( Enabling below flag )  - 723 ms
 */
#define MBEDTLS_WICED_FAST_DHE

/* mbed TLS feature support */
#define MBEDTLS_ECP_DP_SECP192R1_ENABLED
#define MBEDTLS_ECP_DP_SECP224R1_ENABLED
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED
#define MBEDTLS_ECP_DP_SECP256K1_ENABLED

#ifndef WICED_CONFIG_DISABLE_ADVANCED_SECURITY_CURVES
#define MBEDTLS_ECP_DP_BP256R1_ENABLED
#define MBEDTLS_ECP_DP_SECP384R1_ENABLED
#endif

#define MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED
#define MBEDTLS_ECDH_C
#define MBEDTLS_ECDSA_C
#define MBEDTLS_ECP_C

/* Refer wiced_defaults.h to configure WICED_TLS_MINOR_VERSION_MIN and WICED_TLS_MINOR_VERSION_MAX */
#if (WICED_TLS_MINOR_VERSION_MIN == 0)
#define MBEDTLS_SSL_PROTO_TLS1      /* TLSv1_0 */
#endif

#if ( ((WICED_TLS_MINOR_VERSION_MIN <= 1) && (WICED_TLS_MINOR_VERSION_MAX >= 1)) )
#define MBEDTLS_SSL_PROTO_TLS1_1    /* TLSv1_1 */
#endif

#if ( ((WICED_TLS_MINOR_VERSION_MIN <= 2) && (WICED_TLS_MINOR_VERSION_MAX >= 2)) )
#define MBEDTLS_SSL_PROTO_TLS1_2   /* TLSv1_2 */
#endif

/* mbedtls ECDH cipher suite */
#define MBEDTLS_KEY_EXCHANGE_ECDH_ECDSA_ENABLED

#ifndef WICED_CONFIG_DISABLE_DTLS
/* mbedtls PSK support */
#define MBEDTLS_KEY_EXCHANGE__SOME__PSK_ENABLED
#define MBEDTLS_KEY_EXCHANGE_PSK_ENABLED
#endif /* WICED_CONFIG_DISABLE_DTLS */

/* mbedtls RSA cipher suite */
#define MBEDTLS_KEY_EXCHANGE_RSA_ENABLED
#define MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED
/* DHE RSA */
#define MBEDTLS_KEY_EXCHANGE_DHE_RSA_ENABLED
#define MBEDTLS_DHM_C
#define MBEDTLS_RSA_C
#define MBEDTLS_PKCS1_V15
#define MBEDTLS_PKCS1_V21
#define MBEDTLS_CIPHER_MODE_CBC


/* mbed TLS modules */
#define MBEDTLS_AES_C
#define MBEDTLS_DES_C
#define MBEDTLS_ASN1_PARSE_C
#define MBEDTLS_ASN1_WRITE_C
#define MBEDTLS_BIGNUM_C
#define MBEDTLS_CIPHER_C
#define MBEDTLS_CTR_DRBG_C
#define MBEDTLS_ENTROPY_C
#define MBEDTLS_GCM_C
#define MBEDTLS_CCM_C
#define MBEDTLS_CAMELLIA_C
#define MBEDTLS_MD_C
#define MBEDTLS_MD4_C
#define MBEDTLS_MD5_C
#define MBEDTLS_OID_C
#define MBEDTLS_PK_C
#define MBEDTLS_PK_PARSE_C
#define MBEDTLS_SHA1_C
#define MBEDTLS_SHA256_C
#define MBEDTLS_SHA384_C
#define MBEDTLS_SHA512_C


#ifdef PLATFORM_HAS_HW_CRYPTO_SUPPORT
#ifndef WICED_CONFIG_DONOT_USE_HW_CRYPTO
#define MBEDTLS_AES_ALT
#define MBEDTLS_DES_ALT
#define MBEDTLS_MD5_ALT
#define MBEDTLS_SHA256_ALT
#define MBEDTLS_SHA1_ALT
#endif /* #ifndef WICED_CONFIG_DONOT_USE_HW_CRYPTO */
#endif /* PLATFORM_HAS_HW_CRYPTO_SUPPORT */
#define MBEDTLS_SELF_TEST


#ifndef WICED_CONFIG_DISABLE_SSL_CLIENT
#define MBEDTLS_SSL_CLI_C
#endif

#ifndef WICED_CONFIG_DISABLE_SSL_SERVER
#define MBEDTLS_SSL_SRV_C
/* Resume TLS sessions */
#define MBEDTLS_SSL_CACHE_C
#endif


/* To use the session resumption feature of TLS, apps just need to enable WICED_TLS_CLI_CACHE_SESSION flag without
 * worrying about storing connection info. BESL library takes care of storing the connection info(ip, port, session info)
 * and resuming the connections. Number of entries to be stored is determined by WICED_TLS_CLI_CACHE_ENTRIES macro.
 */
//#define WICED_TLS_CLI_CACHE_SESSION

#ifdef WICED_TLS_CLI_CACHE_SESSION

#ifndef WICED_TLS_CLI_CACHE_ENTRIES
#define WICED_TLS_CLI_CACHE_ENTRIES 16
#endif

#endif

#define MBEDTLS_SSL_TLS_C
#define MBEDTLS_X509_CRT_PARSE_C
#define MBEDTLS_X509_USE_C

/* For test certificates */
#define MBEDTLS_BASE64_C
#define MBEDTLS_PEM_PARSE_C

/* Save RAM at the expense of ROM */
#define MBEDTLS_AES_ROM_TABLES

/* Save RAM by adjusting to our exact needs */
#define MBEDTLS_ECP_MAX_BITS    512
#define MBEDTLS_MPI_MAX_SIZE    1024 // 384 bits is 48 bytes

/* Save RAM at the expense of speed, see ecp.h */
#define MBEDTLS_ECP_WINDOW_SIZE        4
#define MBEDTLS_ECP_FIXED_POINT_OPTIM  0

/* Significant speed benefit at the expense of some ROM */
#define MBEDTLS_ECP_NIST_OPTIM

/*
 * You should adjust this to the exact number of sources you're using: default
 * is the "mbedtls_platform_entropy_poll" source, but you may want to add other ones.
 * Minimum is 2 for the entropy test suite.
 */
#define MBEDTLS_ENTROPY_MAX_SOURCES 2

/* extra added by Cypress */
#define MBEDTLS_NO_PLATFORM_ENTROPY
#define MBEDTLS_TEST_NULL_ENTROPY
#define MBEDTLS_ENTROPY_C
#define MBEDTLS_NO_DEFAULT_ENTROPY_SOURCES

/* Save ROM and a few bytes of RAM by specifying our own ciphersuite list
 * Currently with DTLS only MBEDTLS_TLS_PSK_WITH_AES_128_CCM_8 and MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_CCM_8 cipher suites are enabled.
 * PSK cipher suite is not supported for TLS.
 * */

#define MBEDTLS_SSL_CIPHERSUITES                        \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_CCM_8,         \
    MBEDTLS_TLS_PSK_WITH_AES_128_CCM_8,                 \
    MBEDTLS_TLS_DHE_RSA_WITH_AES_256_CBC_SHA256,        \
    MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_256_CBC_SHA256,   \
    MBEDTLS_TLS_DHE_RSA_WITH_AES_256_CBC_SHA,           \
    MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_256_CBC_SHA,      \
    MBEDTLS_TLS_DHE_RSA_WITH_AES_128_CBC_SHA256,        \
    MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_128_CBC_SHA256,   \
    MBEDTLS_TLS_DHE_RSA_WITH_AES_128_CBC_SHA,           \
    MBEDTLS_TLS_DHE_RSA_WITH_CAMELLIA_128_CBC_SHA,      \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_256_GCM_SHA384,    \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_GCM_SHA256,    \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_256_CCM_8,         \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA256,    \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_128_CBC_SHA,       \
    MBEDTLS_TLS_ECDHE_ECDSA_WITH_AES_256_CBC_SHA,       \
    MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_128_CBC_SHA,        \
    MBEDTLS_TLS_ECDH_ECDSA_WITH_AES_256_CBC_SHA,        \
    MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_GCM_SHA256,      \
    MBEDTLS_TLS_RSA_WITH_AES_256_CBC_SHA256,            \
    MBEDTLS_TLS_RSA_WITH_CAMELLIA_256_CBC_SHA256,       \
    MBEDTLS_TLS_RSA_WITH_AES_256_CBC_SHA,               \
    MBEDTLS_TLS_RSA_WITH_CAMELLIA_256_CBC_SHA,          \
    MBEDTLS_TLS_RSA_WITH_AES_128_CBC_SHA256,            \
    MBEDTLS_TLS_RSA_WITH_CAMELLIA_128_CBC_SHA256,       \
    MBEDTLS_TLS_RSA_WITH_AES_128_CBC_SHA,               \
    MBEDTLS_TLS_RSA_WITH_CAMELLIA_128_CBC_SHA

/* TLS extensions */
#define MBEDTLS_SSL_MAX_FRAGMENT_LENGTH
#define MBEDTLS_SSL_SERVER_NAME_INDICATION
#define MBEDTLS_SSL_ALPN


/**
 * Allow SHA-1 in the default TLS configuration for certificate signing.
 * Without this build-time option, SHA-1 support must be activated explicitly
 * through mbedtls_ssl_conf_cert_profile. Turning on this option is not
 * recommended because of it is possible to generte SHA-1 collisions, however
 * this may be safe for legacy infrastructure where additional controls apply.
 */
#define MBEDTLS_TLS_DEFAULT_ALLOW_SHA1_IN_CERTIFICATES

/**
 * Allow SHA-1 in the default TLS configuration for TLS 1.2 handshake
 * signature and ciphersuite selection. Without this build-time option, SHA-1
 * support must be activated explicitly through mbedtls_ssl_conf_sig_hashes.
 * The use of SHA-1 in TLS <= 1.1 and in HMAC-SHA-1 is always allowed by
 * default. At the time of writing, there is no practical attack on the use
 * of SHA-1 in handshake signatures, hence this option is turned on by default
 * for compatibility with existing peers.
 */
#define MBEDTLS_TLS_DEFAULT_ALLOW_SHA1_IN_KEY_EXCHANGE


/* changes for DTLS */
#ifndef WICED_CONFIG_DISABLE_DTLS
#define MBEDTLS_SSL_PROTO_DTLS
#define MBEDTLS_SSL_DTLS_HELLO_VERIFY
#define MBEDTLS_SSL_COOKIE_C
#define MBEDTLS_SSL_ALL_ALERT_MESSAGES
#endif /* WICED_CONFIG_DISABLE_DTLS */

/* Un-comment below macros to enable MBEDTLS for debug log
 * MBEDTLS_DEBUG_C
 * MBEDTLS_SSL_DEBUG_ALL
 * MBEDTLS_DEBUG_LOG_LEVEL
 * Also Enable WPRINT_SECURITY_DEBUG in wiced_defaults.h
 */
//#define MBEDTLS_DEBUG_C

//#define MBEDTLS_SSL_DEBUG_ALL

/* MBEDTLS Different Debug log levels
 *  - 0 No debug
 *  - 1 Error
 *  - 2 State change
 *  - 3 Informational
 *  - 4 Verbose
 */
//#define MBEDTLS_DEBUG_LOG_LEVEL 3

/*
 * Save RAM at the expense of interoperability: do this only if you control
 * both ends of the connection!  (See coments in "mbedtls/ssl.h".)
 * The minimum size here depends on the certificate chain used as well as the
 * typical size of records.
 */
//#define MBEDTLS_SSL_ENCRYPT_THEN_MAC
//#define MBEDTLS_SSL_MAX_CONTENT_LEN  2000
//#define MBEDTLS_SSL_ENCRYPT_THEN_MAC
#endif /* MBEDTLS_CONFIG_H */

