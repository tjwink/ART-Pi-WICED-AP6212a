/*
 *  FIPS-180-2 compliant SHA-256 implementation
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
 *  The SHA-256 Secure Hash Standard was published by NIST in 2002.
 *
 *  http://csrc.nist.gov/publications/fips/fips180-2/fips180-2.pdf
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

//#if defined(MBEDTLS_SHA256_C)

#include "mbedtls/sha256_alt.h"

#include <string.h>

#if defined(MBEDTLS_SELF_TEST)
#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#include <stdlib.h>
#define mbedtls_printf printf
#define mbedtls_calloc    calloc
#define mbedtls_free       free
#endif /* MBEDTLS_PLATFORM_C */
#endif /* MBEDTLS_SELF_TEST */

#if defined(MBEDTLS_SHA256_ALT)
void mbedtls_sha256_init( mbedtls_sha256_context *ctx )
{
    ctx->mbedtls_sha256_hw_ctx = malloc ( sizeof( mbedtls_sha256_hw_context ));
    if ( ctx->mbedtls_sha256_hw_ctx == NULL )
    {
        return;
    }

    memset (ctx->mbedtls_sha256_hw_ctx, 0, sizeof(mbedtls_sha256_hw_context));
    wiced_hw_sha256_init( (wiced_hw_sha256_context*) ctx->mbedtls_sha256_hw_ctx );
}

void mbedtls_sha256_free( mbedtls_sha256_context *ctx )
{
    if ( ctx->mbedtls_sha256_hw_ctx != NULL )
    {
        wiced_hw_sha256_free( (wiced_hw_sha256_context*) ctx->mbedtls_sha256_hw_ctx );
        free ( ctx->mbedtls_sha256_hw_ctx );
        ctx->mbedtls_sha256_hw_ctx = NULL;
    }
}

void mbedtls_sha256_clone( mbedtls_sha256_context *dst,
        const mbedtls_sha256_context *src )
{
    if ( dst->mbedtls_sha256_hw_ctx == NULL || src->mbedtls_sha256_hw_ctx == NULL )
    {
        return;
    }

    wiced_hw_sha256_clone(dst->mbedtls_sha256_hw_ctx,src->mbedtls_sha256_hw_ctx);
}

/*
 * SHA-256 context setup
 */
void mbedtls_sha256_starts( mbedtls_sha256_context *ctx, int is224 )
{
    if ( ctx->mbedtls_sha256_hw_ctx == NULL )
    {
        return;
    }

    wiced_hw_sha256_starts((wiced_hw_sha256_context*)ctx->mbedtls_sha256_hw_ctx, is224 );
}

void mbedtls_sha256_process( mbedtls_sha256_context *ctx, const unsigned char data[64] )
{
    UNUSED_PARAMETER( ctx );
    return;

}

/*
 * SHA-256 process buffer
 */
void mbedtls_sha256_update( mbedtls_sha256_context *ctx, const unsigned char *input,
        size_t ilen )
{
    if ( ctx->mbedtls_sha256_hw_ctx == NULL )
    {
        return;
    }

    wiced_hw_sha256_update( (wiced_hw_sha256_context*)ctx->mbedtls_sha256_hw_ctx, input, ilen );
}

/*
 * SHA-256 final digest
 */
void mbedtls_sha256_finish( mbedtls_sha256_context *ctx, unsigned char output[32] )
{
    if ( ctx->mbedtls_sha256_hw_ctx == NULL )
    {
        return;
    }

    wiced_hw_sha256_finish( (wiced_hw_sha256_context*) ctx->mbedtls_sha256_hw_ctx, output );
}

/*
 * output = SHA-256( input buffer )
 */
void mbedtls_sha256( const unsigned char *input, size_t ilen,
        unsigned char output[32], int is224 )
{
    wiced_hw_sha256(input, ilen, output, is224);
    return;
}

/* WICED_MBEDTLS Start */
/*
 * SHA-256 HMAC context setup
 */
void sha2_hmac_starts(sha2_hmac_context *ctx, const unsigned char *key, uint32_t keylen,
        int32_t is224)
{
    unsigned char sum[32];

    ctx->sha2_hmac_hw_ctx = malloc ( sizeof( sha2_hmac_hw_context ));
    if ( ctx->sha2_hmac_hw_ctx == NULL)
    {
        return;
    }

    memset ( ctx->sha2_hmac_hw_ctx, 0, sizeof(sha2_hmac_hw_context) );

    /* as per HMAC spec (rfc2104) if the key length is greater than block size (64)
       HASH of the key is used as key to the HMAC */
    if (keylen > 64) {
        mbedtls_sha256(key, keylen, sum, is224);
        keylen = (is224) ? 28 : 32;
        key = sum;
    }
    wiced_sha256_hmac_starts((wiced_hw_sha256_hmac_context*) ctx->sha2_hmac_hw_ctx, key, keylen,is224);

    return;
}

/*
 * SHA-256 HMAC process buffer
 */
void sha2_hmac_update(sha2_hmac_context *ctx, const unsigned char *input, uint32_t ilen)
{
    wiced_sha256_hmac_update( (wiced_hw_sha256_hmac_context*) ctx->sha2_hmac_hw_ctx, input, ilen );

    return;
}

/*
 * SHA-256 HMAC final digest
 */
void sha2_hmac_finish(sha2_hmac_context * ctx, unsigned char output[32])
{
    if ( ctx->sha2_hmac_hw_ctx != NULL )
    {
        wiced_sha256_hmac_finish( (wiced_hw_sha256_hmac_context*) ctx->sha2_hmac_hw_ctx, output );
        free(ctx->sha2_hmac_hw_ctx);
        ctx->sha2_hmac_hw_ctx = NULL;
    }

    return;
}

/*
 * output = HMAC-SHA-256( hmac key, input buffer )
 */
void sha2_hmac(const unsigned char *key, uint32_t keylen,
        const unsigned char *input, uint32_t ilen,
        unsigned char output[32], int32_t is224)
{
    wiced_sha256_hmac( key, keylen, input, ilen, output, is224 );

    return;
}
/* WICED_MBEDTLS End */
#endif

