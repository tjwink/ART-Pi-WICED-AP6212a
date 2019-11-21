/*
 *  RFC 1321 compliant MD5 implementation
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
 *  The MD5 algorithm was designed by Ron Rivest in 1991.
 *
 *  http://www.ietf.org/rfc/rfc1321.txt
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#include "mbedtls/md5_alt.h"

#include <string.h>

#if defined(MBEDTLS_SELF_TEST)
#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#define mbedtls_printf printf
#endif /* MBEDTLS_PLATFORM_C */
#endif /* MBEDTLS_SELF_TEST */

#if defined(MBEDTLS_MD5_ALT)

void mbedtls_md5_init( mbedtls_md5_context *ctx )
{
    ctx->mbedtls_md5_hw_ctx = malloc ( sizeof( mbedtls_md5_hw_context ));
    if ( ctx->mbedtls_md5_hw_ctx == NULL )
    {
        return;
    }

    memset (ctx->mbedtls_md5_hw_ctx, 0, sizeof(mbedtls_md5_hw_context));

    wiced_hw_md5_init((wiced_hw_md5_context*) ctx->mbedtls_md5_hw_ctx );
}

void mbedtls_md5_free( mbedtls_md5_context *ctx )
{
    if ( ctx->mbedtls_md5_hw_ctx != NULL )
    {
        wiced_hw_md5_free((wiced_hw_md5_context*) ctx->mbedtls_md5_hw_ctx );
        free ( ctx->mbedtls_md5_hw_ctx );
        ctx->mbedtls_md5_hw_ctx = NULL;
    }
}

void mbedtls_md5_clone( mbedtls_md5_context *dst,
        const mbedtls_md5_context *src )
{
    if ( dst->mbedtls_md5_hw_ctx == NULL || src->mbedtls_md5_hw_ctx == NULL )
    {
        return;
    }

    wiced_hw_md5_clone(dst->mbedtls_md5_hw_ctx,src->mbedtls_md5_hw_ctx);
}

/*
 * MD5 context setup
 */
void mbedtls_md5_starts( mbedtls_md5_context *ctx )
{
    if ( ctx->mbedtls_md5_hw_ctx == NULL )
    {
        return;
    }

    wiced_hw_md5_starts((wiced_hw_md5_context*) ctx->mbedtls_md5_hw_ctx );
}

/*
 * MD5 process buffer
 */
void mbedtls_md5_update( mbedtls_md5_context *ctx, const unsigned char *input, size_t ilen )
{
    if ( ctx->mbedtls_md5_hw_ctx == NULL )
    {
        return;
    }

    wiced_hw_md5_update((wiced_hw_md5_context*) ctx->mbedtls_md5_hw_ctx, input, ilen );
}

/*
 * MD5 final digest
 */
void mbedtls_md5_finish( mbedtls_md5_context *ctx, unsigned char output[16] )
{
    if ( ctx->mbedtls_md5_hw_ctx == NULL )
    {
        return;
    }

    wiced_hw_md5_finish((wiced_hw_md5_context*) ctx->mbedtls_md5_hw_ctx, output );

}

void mbedtls_md5_process( mbedtls_md5_context *ctx, const unsigned char data[64] )
{
    UNUSED_PARAMETER( ctx );
    UNUSED_PARAMETER( data );
}
/*
 * output = MD5( input buffer )
 */
void mbedtls_md5( const unsigned char *input, size_t ilen, unsigned char output[16] )
{
    mbedtls_md5_context ctx;

    mbedtls_md5_init( &ctx );
    mbedtls_md5_starts( &ctx );
    mbedtls_md5_update( &ctx, input, ilen );
    mbedtls_md5_finish( &ctx, output );
    mbedtls_md5_free( &ctx );
}
/* WICED_MBEDTLS Start */
/* HMAC related functions */
/*
 * MD5 HMAC context setup
 */
void md5_hmac_starts(md5_hmac_context *ctx, const unsigned char *key, uint32_t keylen)
{
    unsigned char sum[16];

    ctx->md5_hmac_hw_ctx = malloc ( sizeof( md5_hmac_hw_context ));
    if ( ctx->md5_hmac_hw_ctx == NULL)
    {
        return;
    }
    memset ( ctx->md5_hmac_hw_ctx, 0, sizeof(md5_hmac_hw_context) );

    /* as per HMAC spec (rfc2104) if the key length is greater than block size (64)
       HASH of the key is used as key to the HMAC */
    if (keylen > 64) {
        mbedtls_md5(key, keylen, sum);
        keylen = 16;
        key = sum;
    }
    wiced_md5_hmac_starts((wiced_hw_md5_hmac_context*) ctx->md5_hmac_hw_ctx, key, keylen );
    return;

}

/*
 * MD5 HMAC process buffer
 */
void md5_hmac_update(md5_hmac_context *ctx, const unsigned char *input, uint32_t ilen)
{
    wiced_md5_hmac_update((wiced_hw_md5_hmac_context*) ctx->md5_hmac_hw_ctx, input, ilen );
    return;
}

/*
 * MD5 HMAC final digest
 */
void md5_hmac_finish(md5_hmac_context * ctx, unsigned char output[16])
{
    if ( ctx->md5_hmac_hw_ctx != NULL )
    {
        wiced_md5_hmac_finish((wiced_hw_md5_hmac_context*) ctx->md5_hmac_hw_ctx, output );
        free(ctx->md5_hmac_hw_ctx);
        ctx->md5_hmac_hw_ctx = NULL;
    }

    return;
}

/*
 * output = HMAC-MD5( hmac key, input buffer )
 */
void md5_hmac(const unsigned char *key, int32_t keylen,
        const unsigned char *input, int32_t ilen,
        unsigned char output[16])
{
    wiced_md5_hmac( key, keylen, input, ilen, output );
    return;
}
/* WICED_MBEDTLS End */
#endif /* !MBEDTLS_MD5_ALT */
