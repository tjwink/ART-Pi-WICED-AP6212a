/*
 *  FIPS-197 compliant AES implementation
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
 *  The AES block cipher was designed by Vincent Rijmen and Joan Daemen.
 *
 *  http://csrc.nist.gov/encryption/aes/rijndael/Rijndael.pdf
 *  http://csrc.nist.gov/publications/fips/fips197/fips-197.pdf
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(MBEDTLS_AES_C)

#include <string.h>

#include "mbedtls/aes_alt.h"

#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#define mbedtls_printf printf
#endif /* MBEDTLS_PLATFORM_C */
#endif /* MBEDTLS_SELF_TEST */

#if defined(MBEDTLS_AES_ALT)

void mbedtls_aes_init( mbedtls_aes_context *ctx )
{
    ctx->mbedtls_aes_hw_ctx = malloc ( sizeof( wiced_hw_aes_context ));
    if ( ctx->mbedtls_aes_hw_ctx == NULL )
    {
        return;
    }

    memset (ctx->mbedtls_aes_hw_ctx, 0, sizeof(wiced_hw_aes_context));
    wiced_hw_aes_init((wiced_hw_aes_context*) ctx->mbedtls_aes_hw_ctx );
}

void mbedtls_aes_free( mbedtls_aes_context *ctx )
{
    if ( ctx->mbedtls_aes_hw_ctx != NULL )
    {
        wiced_hw_aes_free((wiced_hw_aes_context*) ctx->mbedtls_aes_hw_ctx );
        free ( ctx->mbedtls_aes_hw_ctx );
        ctx->mbedtls_aes_hw_ctx = NULL;
    }
}

/*
 * AES key schedule (encryption)
 */
int mbedtls_aes_setkey_enc( mbedtls_aes_context *ctx, const unsigned char *key,
        unsigned int keybits )
{
    if ( ctx->mbedtls_aes_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_aes_setkey_enc((wiced_hw_aes_context*) ctx->mbedtls_aes_hw_ctx, key, keybits );
}

/*
 * AES key schedule (decryption)
 */
int mbedtls_aes_setkey_dec( mbedtls_aes_context *ctx, const unsigned char *key,
        unsigned int keybits )
{
    if ( ctx->mbedtls_aes_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_aes_setkey_dec((wiced_hw_aes_context*) ctx->mbedtls_aes_hw_ctx, key, keybits );

}

/*
 * AES-ECB block encryption
 */

void mbedtls_aes_encrypt( mbedtls_aes_context *ctx,
        const unsigned char input[16],
        unsigned char output[16] )
{
    if ( ctx->mbedtls_aes_hw_ctx == NULL )
    {
        return;
    }

    wiced_hw_aes_encrypt((wiced_hw_aes_context*) ctx->mbedtls_aes_hw_ctx, input, output);
}

/*
 * AES-ECB block decryption
 */

void mbedtls_aes_decrypt( mbedtls_aes_context *ctx,
        const unsigned char input[16],
        unsigned char output[16] )
{
    if ( ctx->mbedtls_aes_hw_ctx == NULL )
    {
        return;
    }

    wiced_hw_aes_decrypt((wiced_hw_aes_context*) ctx->mbedtls_aes_hw_ctx, input, output);
}

/*
 * AES-ECB block encryption/decryption
 */
int mbedtls_aes_crypt_ecb( mbedtls_aes_context *ctx,
        int mode,
        const unsigned char input[16],
        unsigned char output[16] )
{
    if ( ctx->mbedtls_aes_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_aes_crypt_ecb((wiced_hw_aes_context*) ctx->mbedtls_aes_hw_ctx, mode, input, output );
}

/*
 * AES-CBC buffer encryption/decryption
 */
int mbedtls_aes_crypt_cbc( mbedtls_aes_context *ctx,
        int mode,
        size_t length,
        unsigned char iv[16],
        const unsigned char *input,
        unsigned char *output )
{
    if ( ctx->mbedtls_aes_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_aes_crypt_cbc((wiced_hw_aes_context*) ctx->mbedtls_aes_hw_ctx, mode, length,iv,input, output );
}

/*
 * AES-CFB128 buffer encryption/decryption
 */
int mbedtls_aes_crypt_cfb128( mbedtls_aes_context *ctx,
        int mode,
        size_t length,
        size_t *iv_off,
        unsigned char iv[16],
        const unsigned char *input,
        unsigned char *output )
{
    if ( ctx->mbedtls_aes_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_aes_crypt_cfb128((wiced_hw_aes_context*) ctx->mbedtls_aes_hw_ctx, mode, length,iv_off,iv,input, output );
}

/*
 * AES-CFB8 buffer encryption/decryption
 */
int mbedtls_aes_crypt_cfb8( mbedtls_aes_context *ctx,
        int mode,
        size_t length,
        unsigned char iv[16],
        const unsigned char *input,
        unsigned char *output )
{
    if ( ctx->mbedtls_aes_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_aes_crypt_cfb8((wiced_hw_aes_context*) ctx->mbedtls_aes_hw_ctx, mode, length,iv,input, output );
}

/*
 * AES-CTR buffer encryption/decryption
 */
int mbedtls_aes_crypt_ctr( mbedtls_aes_context *ctx,
        size_t length,
        size_t *nc_off,
        unsigned char nonce_counter[16],
        unsigned char stream_block[16],
        const unsigned char *input,
        unsigned char *output )
{
    if ( ctx->mbedtls_aes_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_aes_crypt_ctr((wiced_hw_aes_context*) ctx->mbedtls_aes_hw_ctx,length,nc_off,nonce_counter,stream_block,input, output );

}

#endif /* MBEDTLS_AES_ALT */


