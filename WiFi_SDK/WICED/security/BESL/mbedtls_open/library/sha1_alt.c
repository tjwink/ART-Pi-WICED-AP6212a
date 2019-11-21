/*
 *  FIPS-180-1 compliant SHA-1 implementation
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
 *  The SHA-1 standard was published by NIST in 1993.
 *
 *  http://www.itl.nist.gov/fipspubs/fip180-1.htm
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

//#if defined(MBEDTLS_SHA1_C)

#include "mbedtls/sha1_alt.h"

#include <string.h>

#if defined(MBEDTLS_SELF_TEST)
#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#define mbedtls_printf printf
#endif /* MBEDTLS_PLATFORM_C */
#endif /* MBEDTLS_SELF_TEST */

#if defined(MBEDTLS_SHA1_ALT)

void mbedtls_sha1_init( mbedtls_sha1_context *ctx )
{
    ctx->mbedtls_sha1_hw_ctx = malloc ( sizeof( mbedtls_sha1_hw_context ));
    if ( ctx->mbedtls_sha1_hw_ctx == NULL )
    {
        return;
    }

    memset (ctx->mbedtls_sha1_hw_ctx, 0, sizeof(mbedtls_sha1_hw_context));
    wiced_hw_sha1_init((wiced_hw_sha1_context*) ctx->mbedtls_sha1_hw_ctx);
}

void mbedtls_sha1_free( mbedtls_sha1_context *ctx )
{
    if ( ctx->mbedtls_sha1_hw_ctx != NULL )
    {
        wiced_hw_sha1_free((wiced_hw_sha1_context*) ctx->mbedtls_sha1_hw_ctx);
        free ( ctx->mbedtls_sha1_hw_ctx );
        ctx->mbedtls_sha1_hw_ctx = NULL;
    }
}

void mbedtls_sha1_clone( mbedtls_sha1_context *dst,
                         const mbedtls_sha1_context *src )
{
    if ( dst->mbedtls_sha1_hw_ctx == NULL || src->mbedtls_sha1_hw_ctx == NULL )
    {
        return;
    }

    wiced_hw_sha1_clone(dst->mbedtls_sha1_hw_ctx,src->mbedtls_sha1_hw_ctx);
}

/*
 * SHA-1 context setup
 */
void mbedtls_sha1_starts( mbedtls_sha1_context *ctx )
{
    if ( ctx->mbedtls_sha1_hw_ctx == NULL )
    {
        return;
    }

    wiced_hw_sha1_starts( (wiced_hw_sha1_context*) ctx->mbedtls_sha1_hw_ctx );
}

/*
 * SHA-1 process buffer
 */
void mbedtls_sha1_update( mbedtls_sha1_context *ctx, const unsigned char *input, size_t ilen )
{
    if ( ctx->mbedtls_sha1_hw_ctx == NULL )
    {
        return;
    }

    wiced_hw_sha1_update( (wiced_hw_sha1_context*) ctx->mbedtls_sha1_hw_ctx,  input,  ilen );
}

/*
 * SHA-1 final digest
 */
void mbedtls_sha1_finish( mbedtls_sha1_context *ctx, unsigned char output[20] )
{
    if ( ctx->mbedtls_sha1_hw_ctx == NULL )
    {
        return;
    }

    wiced_hw_sha1_finish( (wiced_hw_sha1_context*) ctx->mbedtls_sha1_hw_ctx, output );
}

void mbedtls_sha1_process( mbedtls_sha1_context *ctx, const unsigned char data[64] )
{
    UNUSED_PARAMETER( ctx );
    UNUSED_PARAMETER( data );
}
/*
 * output = SHA-1( input buffer )
 */
void mbedtls_sha1( const unsigned char *input, size_t ilen, unsigned char output[20] )
{
    wiced_hw_sha1( input, ilen, output );
    //platform_hwcrypto_sha1( input, ilen, output );
}
/* WICED_MBEDTLS Start */
/*
 * SHA-1 HMAC context setup
 */
void sha1_hmac_starts(sha1_hmac_context *ctx, const unsigned char *key, uint32_t keylen)
{
    unsigned char sum[20];
    ctx->sha1_hmac_hw_ctx = malloc ( sizeof( sha1_hmac_hw_context ));
    if ( ctx->sha1_hmac_hw_ctx == NULL)
    {
        return;
    }

    memset ( ctx->sha1_hmac_hw_ctx, 0, sizeof(sha1_hmac_hw_context) );

    /* as per HMAC spec (rfc2104) if the key length is greater than block size (64)
       HASH of the key is used as key to the HMAC */
    if (keylen > 64) {
        mbedtls_sha1(key, keylen, sum);
        keylen = 20;
        key = sum;
    }
    wiced_sha1_hmac_starts( (wiced_hw_sha1_hmac_context*) ctx->sha1_hmac_hw_ctx, key, keylen );
    return;
}

/*
 * SHA-1 HMAC process buffer
 */
void sha1_hmac_update(sha1_hmac_context *ctx, const unsigned char *input, uint32_t ilen)
{
    wiced_sha1_hmac_update( (wiced_hw_sha1_hmac_context*) ctx->sha1_hmac_hw_ctx, input, ilen );
    return;
}

/*
 * SHA-1 HMAC final digest
 */
void sha1_hmac_finish(sha1_hmac_context * ctx, unsigned char output[20])
{
    if ( ctx->sha1_hmac_hw_ctx != NULL )
    {
        wiced_sha1_hmac_finish( (wiced_hw_sha1_hmac_context*) ctx->sha1_hmac_hw_ctx, output );
        free(ctx->sha1_hmac_hw_ctx);
        ctx->sha1_hmac_hw_ctx = NULL;
    }

    return;
}

/*
 * output = HMAC-SHA-1( hmac key, input buffer )
 */
void sha1_hmac(const unsigned char *key, int32_t keylen,
           const unsigned char *input, int32_t ilen,
           unsigned char output[20])
{
    wiced_sha1_hmac( key, keylen, input, ilen, output );
    return;
}
/* WICED_MBEDTLS End */

#endif /* !MBEDTLS_SHA1_ALT */



//#endif /* MBEDTLS_SHA1_C */
