/*
 *  FIPS-46-3 compliant Triple-DES implementation
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
 *  DES, on which TDES is based, was originally designed by Horst Feistel
 *  at IBM in 1974, and was adopted as a standard by NIST (formerly NBS).
 *
 *  http://csrc.nist.gov/publications/fips/fips46-3/fips46-3.pdf
 */

#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined(MBEDTLS_DES_C)

#include "mbedtls/des_alt.h"

#include <string.h>

#if defined(MBEDTLS_DES_ALT)

void mbedtls_des_init( mbedtls_des_context *ctx )
{
    ctx->mbedtls_des_hw_ctx = malloc ( sizeof( wiced_hw_des_context ));
    if ( ctx->mbedtls_des_hw_ctx == NULL )
    {
        return;
    }

    memset (ctx->mbedtls_des_hw_ctx, 0, sizeof(wiced_hw_des_context));
    wiced_hw_des_init((wiced_hw_des_context*) ctx->mbedtls_des_hw_ctx );
}

void mbedtls_des_free( mbedtls_des_context *ctx )
{
    if ( ctx->mbedtls_des_hw_ctx != NULL )
    {
        wiced_hw_des_free((wiced_hw_des_context*) ctx->mbedtls_des_hw_ctx );
        free ( ctx->mbedtls_des_hw_ctx );
        ctx->mbedtls_des_hw_ctx = NULL;
    }
}

void mbedtls_des3_init( mbedtls_des3_context *ctx )
{
    ctx->mbedtls_des3_hw_ctx = malloc ( sizeof( wiced_hw_des_context ));
    if ( ctx->mbedtls_des3_hw_ctx == NULL )
    {
        return;
    }

    memset (ctx->mbedtls_des3_hw_ctx, 0, sizeof(wiced_hw_des_context));
    wiced_hw_des3_init((wiced_hw_des_context*) ctx->mbedtls_des3_hw_ctx );
}

void mbedtls_des3_free( mbedtls_des3_context *ctx )
{
    if ( ctx->mbedtls_des3_hw_ctx != NULL )
    {
        wiced_hw_des3_free((wiced_hw_des_context*) ctx->mbedtls_des3_hw_ctx );
        free ( ctx->mbedtls_des3_hw_ctx );
        ctx->mbedtls_des3_hw_ctx = NULL;
    }
}

/*
 * DES key schedule (56-bit, encryption)
 */
int mbedtls_des_setkey_enc( mbedtls_des_context *ctx, const unsigned char key[MBEDTLS_DES_KEY_SIZE] )
{
    if ( ctx->mbedtls_des_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_des_setkey_enc((wiced_hw_des_context*) ctx->mbedtls_des_hw_ctx, key );
}

/*
 * DES key schedule (56-bit, decryption)
 */
int mbedtls_des_setkey_dec( mbedtls_des_context *ctx, const unsigned char key[MBEDTLS_DES_KEY_SIZE] )
{
    if ( ctx->mbedtls_des_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_des_setkey_dec((wiced_hw_des_context*) ctx->mbedtls_des_hw_ctx, key );
}

/*
 * Triple-DES key schedule (112-bit, encryption)
 */
int mbedtls_des3_set2key_enc( mbedtls_des3_context *ctx,
                      const unsigned char key[MBEDTLS_DES_KEY_SIZE * 2] )
{
    if ( ctx->mbedtls_des3_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_des3_set2key_enc((wiced_hw_des_context*) ctx->mbedtls_des3_hw_ctx, key );
}

/*
 * Triple-DES key schedule (112-bit, decryption)
 */
int mbedtls_des3_set2key_dec( mbedtls_des3_context *ctx,
                      const unsigned char key[MBEDTLS_DES_KEY_SIZE * 2] )
{
    if ( ctx->mbedtls_des3_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_des3_set2key_dec((wiced_hw_des_context*) ctx->mbedtls_des3_hw_ctx, key );
}

/*
 * Triple-DES key schedule (168-bit, encryption)
 */
int mbedtls_des3_set3key_enc( mbedtls_des3_context *ctx,
                      const unsigned char key[MBEDTLS_DES_KEY_SIZE * 3] )
{
    if ( ctx->mbedtls_des3_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_des3_set3key_enc((wiced_hw_des_context*) ctx->mbedtls_des3_hw_ctx, key );
}

/*
 * Triple-DES key schedule (168-bit, decryption)
 */
int mbedtls_des3_set3key_dec( mbedtls_des3_context *ctx,
                      const unsigned char key[MBEDTLS_DES_KEY_SIZE * 3] )
{
    if ( ctx->mbedtls_des3_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_des3_set3key_dec((wiced_hw_des_context*) ctx->mbedtls_des3_hw_ctx, key );
}

/*
 * DES-ECB block encryption/decryption
 */
int mbedtls_des_crypt_ecb( mbedtls_des_context *ctx,
                    const unsigned char input[8],
                    unsigned char output[8] )
{
    if ( ctx->mbedtls_des_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_des_crypt_ecb( (wiced_hw_des_context*) ctx->mbedtls_des_hw_ctx, input, output );
}

/*
 * DES-CBC buffer encryption/decryption
 */
int mbedtls_des_crypt_cbc( mbedtls_des_context *ctx,
                    int mode,
                    size_t length,
                    unsigned char iv[8],
                    const unsigned char *input,
                    unsigned char *output )
{
    if ( ctx->mbedtls_des_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_des_crypt_cbc((wiced_hw_des_context*) ctx->mbedtls_des_hw_ctx, mode, length, iv, input, output );
}

/*
 * 3DES-ECB block encryption/decryption
 */
int mbedtls_des3_crypt_ecb( mbedtls_des3_context *ctx,
                     const unsigned char input[8],
                     unsigned char output[8] )
{
    if ( ctx->mbedtls_des3_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_des3_crypt_ecb((wiced_hw_des_context*) ctx->mbedtls_des3_hw_ctx, input, output );
}

/*
 * 3DES-CBC buffer encryption/decryption
 */
int mbedtls_des3_crypt_cbc( mbedtls_des3_context *ctx,
                     int mode,
                     size_t length,
                     unsigned char iv[8],
                     const unsigned char *input,
                     unsigned char *output )
{
    if ( ctx->mbedtls_des3_hw_ctx == NULL )
    {
        return -1;
    }

    return wiced_hw_des3_crypt_cbc( (wiced_hw_des_context*) ctx->mbedtls_des3_hw_ctx,mode,length,iv, input, output );
}

/* WICED_MBEDTLS Start */
int mbedtls_des_encrypt_ecb( mbedtls_des_context *des_ctx,
                    const unsigned char input[8],
                    unsigned char output[8] )

{
    if ( des_ctx->mbedtls_des_hw_ctx == NULL )
    {
        return -1;
    }

    wiced_hw_des_context* des_hw_ctx = (wiced_hw_des_context*) des_ctx->mbedtls_des_hw_ctx;
    des_hw_ctx->direction = MBEDTLS_DES_ENCRYPT;
    return mbedtls_des_crypt_ecb(des_ctx, input, output);
}
/* WICED_MBEDTLS End */

#endif /* MBEDTLS_DES_ALT */
#endif /* MBEDTLS_DES_C */
