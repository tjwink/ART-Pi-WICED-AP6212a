/**
 * \file sha512.h
 *
 * \brief SHA-384 and SHA-512 cryptographic hash function
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
#ifndef MBEDTLS_SHA512_H
#define MBEDTLS_SHA512_H

#if !defined(MBEDTLS_CONFIG_FILE)
#include "config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include <stddef.h>
#include <stdint.h>
/* WICED_MBEDTLS Start */
#include "crypto_structures.h"
/* WICED_MBEDTLS End */


#if !defined(MBEDTLS_SHA512_ALT)
// Regular implementation
//

#ifdef __cplusplus
extern "C" {
#endif

/* WICED_MBEDTLS Start */

/* mbedtls_sha512_context structure has been
 * moved to crypto_structures.h */

/* WICED_MBEDTLS End */

/**
 * \brief          Initialize SHA-512 context
 *
 * \param ctx      SHA-512 context to be initialized
 */
void mbedtls_sha512_init( mbedtls_sha512_context *ctx );

/**
 * \brief          Clear SHA-512 context
 *
 * \param ctx      SHA-512 context to be cleared
 */
void mbedtls_sha512_free( mbedtls_sha512_context *ctx );

/**
 * \brief          Clone (the state of) a SHA-512 context
 *
 * \param dst      The destination context
 * \param src      The context to be cloned
 */
void mbedtls_sha512_clone( mbedtls_sha512_context *dst,
                           const mbedtls_sha512_context *src );

/**
 * \brief          SHA-512 context setup
 *
 * \param ctx      context to be initialized
 * \param is384    0 = use SHA512, 1 = use SHA384
 */
void mbedtls_sha512_starts( mbedtls_sha512_context *ctx, int is384 );

/**
 * \brief          SHA-512 process buffer
 *
 * \param ctx      SHA-512 context
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 */
void mbedtls_sha512_update( mbedtls_sha512_context *ctx, const unsigned char *input,
                    size_t ilen );

/**
 * \brief          SHA-512 final digest
 *
 * \param ctx      SHA-512 context
 * \param output   SHA-384/512 checksum result
 */
void mbedtls_sha512_finish( mbedtls_sha512_context *ctx, unsigned char output[64] );

#ifdef __cplusplus
}
#endif

#else  /* MBEDTLS_SHA512_ALT */
#include "sha512_alt.h"
#endif /* MBEDTLS_SHA512_ALT */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief          Output = SHA-512( input buffer )
 *
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 * \param output   SHA-384/512 checksum result
 * \param is384    0 = use SHA512, 1 = use SHA384
 */
void mbedtls_sha512( const unsigned char *input, size_t ilen,
             unsigned char output[64], int is384 );

/* WICED_MBEDTLS Start */
/**
 * \brief          SHA-512 HMAC context setup
 *
 * \param ctx      HMAC context to be initialized
 * \param is384    0 = use SHA512, 1 = use SHA384
 * \param key      HMAC secret key
 * \param keylen   length of the HMAC key
 */
void sha4_hmac_starts(sha4_hmac_context * ctx, const unsigned char *key,
              int32_t keylen, int32_t is384);

/**
 * \brief          SHA-512 HMAC process buffer
 *
 * \param ctx      HMAC context
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 */
void sha4_hmac_update(sha4_hmac_context * ctx, const unsigned char *input,
              int32_t ilen);

/**
 * \brief          SHA-512 HMAC final digest
 *
 * \param ctx      HMAC context
 * \param output   SHA-384/512 HMAC checksum result
 */
void sha4_hmac_finish(sha4_hmac_context * ctx, unsigned char output[64]);

/**
 * \brief          Output = HMAC-SHA-512( hmac key, input buffer )
 *
 * \param key      HMAC secret key
 * \param keylen   length of the HMAC key
 * \param input    buffer holding the  data
 * \param ilen     length of the input data
 * \param output   HMAC-SHA-384/512 result
 * \param is384    0 = use SHA512, 1 = use SHA384
 */
void sha4_hmac(const unsigned char *key, int32_t keylen,
           const unsigned char *input, int32_t ilen,
           unsigned char output[64], int32_t is384);
/* WICED_MBEDTLS End */

/*
 * \brief          HMAC Key Derivation Function for SHA-512 / SHA-384
 *
 * \description    Generates keying material using HKDF.
 *
 * \param salt[in]      The optional salt value (a non-secret random value);
 *                      if not provided (salt == NULL), it is set internally
 *                      to a string of HashLen(whichSha) zeros.
 * \param salt_len[in]  The length of the salt value.  (Ignored if salt == NULL.)
 * \param ikm[in]       Input keying material.
 * \param ikm_len[in]   The length of the input keying material.
 * \param info[in]      The optional context and application specific information.
 *                      If info == NULL or a zero-length string, it is ignored.
 * \param info_len[in]  The length of the optional context and application specific
 *                      information.  (Ignored if info == NULL.)
 * \param okm[out]      Where the HKDF is to be stored.
 * \param okm_len[in]   The length of the buffer to hold okm.
 *                      okm_len must be <= 255 * USHABlockSize(whichSha)
 * \param is384         0 = use SHA512, 1 = use SHA384
 *
 * \return              0 = success
 *
 */
int sha4_hkdf(
    const unsigned char *salt, int salt_len,
    const unsigned char *ikm, int ikm_len,
    const unsigned char *info, int info_len,
    uint8_t okm[ ], int okm_len,
    int32_t is384);
/**
 * \brief          Checkup routine
 *
 * \return         0 if successful, or 1 if the test failed
 */
int mbedtls_sha512_self_test( int verbose );

/* Internal use */
void mbedtls_sha512_process( mbedtls_sha512_context *ctx, const unsigned char data[128] );

#ifdef __cplusplus
}
#endif

#endif /* mbedtls_sha512.h */
