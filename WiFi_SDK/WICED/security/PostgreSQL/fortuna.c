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

/*
 * fortuna.c
 * Fortuna-like PRNG.
 *
 * Copyright (c) 2005 Marko Kreen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * contrib/pgcrypto/fortuna.c
 */

#include <time.h>
#include <string.h>
#include <stdlib.h>

/* for porting */
#include "crypto_structures.h"
#include "wiced_security_internal.h"
#include "wwd_assert.h"

#include "fortuna.h"

/*
 * Why Fortuna-like: There does not seem to be any definitive reference
 * on Fortuna in the net.  Instead this implementation is based on
 * following references:
 *
 * http://en.wikipedia.org/wiki/Fortuna_(PRNG)
 *     - Wikipedia article
 * http://jlcooke.ca/random/
 *     - Jean-Luc Cooke Fortuna-based /dev/random driver for Linux.
 */

/*
 * There is some confusion about whether and how to carry forward
 * the state of the pools.  Seems like original Fortuna does not
 * do it, resetting hash after each request.  I guess expecting
 * feeding to happen more often that requesting.   This is absolutely
 * unsuitable for pgcrypto, as nothing asynchronous happens here.
 *
 * J.L. Cooke fixed this by feeding previous hash to new re-initialized
 * hash context.
 *
 * Fortuna predecessor Yarrow requires ability to query intermediate
 * 'final result' from hash, without affecting it.
 *
 * This implementation uses the Yarrow method - asking intermediate
 * results, but continuing with old state.
 */


/*
 * Algorithm parameters
 */

/*
 * How many pools.
 *
 * Original Fortuna uses 32 pools, that means 32'th pool is
 * used not earlier than in 13th year.  This is a waste in
 * pgcrypto, as we have very low-frequancy seeding.  Here
 * is preferable to have all entropy usable in reasonable time.
 *
 * With 23 pools, 23th pool is used after 9 days which seems
 * more sane.
 *
 * In our case the minimal cycle time would be bit longer
 * than the system-randomness feeding frequency.
 */

/******************************************************
 *                      Macros
 ******************************************************/
/*
 * How many pools.
 *
 * Original Fortuna uses 32 pools, that means 32'th pool is
 * used not earlier than in 13th year.  This is a waste in
 * pgcrypto, as we have very low-frequancy seeding.  Here
 * is preferable to have all entropy usable in reasonable time.
 *
 * With 23 pools, 23th pool is used after 9 days which seems
 * more sane.
 *
 * In our case the minimal cycle time would be bit longer
 * than the system-randomness feeding frequency.
 */
#define NUM_POOLS          23

/* in microseconds */
#define RESEED_INTERVAL_MS 100    /* 0.1 sec */

/* for one big request, reseed after this many bytes */
#define RESEED_BYTES       (1024*1024)

/*
 * Skip reseed if pool 0 has less than this many
 * bytes added since last reseed.
 */
#define POOL0_FILL        (256/8)

/*
 * Algorithm constants
 */

/* Both cipher key size and hash result size: 256 bit key */
#define BLOCK            32

/* cipher block size */
#define CIPH_BLOCK       16

#define MD_CTX          sha2_hmac_context
#define SHA2_256_BIT    0
#define CIPH_CTX        aes_context_t
#define MAX_UINT32      (uint32_t)0xffffffff

#define BYTE_TO_BITS( num_bytes ) ( num_bytes * 8 )
#define FORTUNA_DEBUG( args )
/******************************************************
 *                    Structures
 ******************************************************/

struct fortuna_state
{
    uint8_t      counter[CIPH_BLOCK];
    uint8_t      result[CIPH_BLOCK];
    uint8_t      key[BLOCK];
    MD_CTX       pool[NUM_POOLS];
    CIPH_CTX     ciph;
    unsigned     reseed_count;
    uint32_t     last_reseed_time;
    unsigned     pool0_bytes;
    unsigned     rnd_pos;
    int          tricks_done;
};

/******************************************************
 *                Type Definitions
 ******************************************************/
typedef struct fortuna_state FState;

/******************************************************
 *                Static functions
 ******************************************************/
static uint32_t prng_fortuna_get_random ( void );
static void     prng_fortuna_add_entropy( const void* buffer, uint16_t buffer_length );

/******************************************************
 *                Variable Definitions
 ******************************************************/
static FState *main_state = NULL;
static int    init_done = 0;
static wiced_crypto_prng_t prng_fortuna =
{
    .get_random  = &prng_fortuna_get_random,
    .add_entropy = &prng_fortuna_add_entropy
};
fortuna_client_time_func_t client_rtos_time_function = NULL;

static void
ciph_init(CIPH_CTX * ctx, const uint8_t *key, int klen)
{
    mbedtls_aes_setkey_enc( ctx, (const unsigned char *)key, (uint32_t)BYTE_TO_BITS( klen ) );
}

static void
ciph_encrypt(CIPH_CTX * ctx, const uint8_t *in, uint8_t *out)
{
    mbedtls_aes_crypt_ecb( ctx, AES_ENCRYPT, in, out );
}

static void
md_init(MD_CTX * ctx)
{
    mbedtls_sha256_starts( &ctx->ctx, SHA2_256_BIT );
}

static void
md_update(MD_CTX * ctx, const uint8_t *data, uint32_t len)
{
    mbedtls_sha256_update( &ctx->ctx, data, len );
}

static void
md_result(MD_CTX * ctx, uint8_t *dst)
{
    MD_CTX    tmp;

    /* preserve the current session, but get the "final" result from it */
    memcpy(&tmp, ctx, sizeof(*ctx));
    mbedtls_sha256_finish( &tmp.ctx, dst );

    /* clean up  for security sake */
    memset(&tmp, 0, sizeof(tmp));
}

/*
 * initialize state
 */
static void
init_state( void )
{
    int pool_index = 0;

    wiced_assert( "Fortuna was not enabled before use.\n", NULL != main_state );

    memset( main_state, 0, sizeof(*main_state));

    for ( pool_index = 0; pool_index < NUM_POOLS; pool_index++ )
    {
        md_init(&main_state->pool[pool_index]);
    }

    init_done = 1;
}

/*
 * Endianess does not matter.
 * It just needs to change without repeating.
 */
static void
inc_counter(FState *st)
{
    uint32_t* val = (uint32_t *) st->counter;

    if (++val[0])
        return;
    if (++val[1])
        return;
    if (++val[2])
        return;
    ++val[3];
}

/*
 * This is called 'cipher in counter mode'.
 */
static void
encrypt_counter(FState *st, uint8_t *dst)
{
    ciph_encrypt(&st->ciph, st->counter, dst);
    inc_counter(st);
}


/*
 * The time between reseed must be at least RESEED_INTERVAL
 * microseconds.
 * Note: so ensure the min time has passed.
 * This may produce some false negatives which seems ok
 */
static int
enough_time_passed(FState *st)
{
    int  ok = 0;
    uint32_t current_time;
    uint32_t elapsed_time = 0;

    current_time = client_rtos_time_function( );

    //account for possible wrapping of timer
    if ( current_time > st->last_reseed_time )
    {
        elapsed_time = current_time - st->last_reseed_time;
    }
    else
    {
        elapsed_time = MAX_UINT32 - st->last_reseed_time + current_time;
    }

    if ( elapsed_time >= RESEED_INTERVAL_MS || st->last_reseed_time == 0 )
    {
        ok = 1;
    }

    if ( ok )
    {
        st->last_reseed_time = current_time;
    }

    return ok;
}

/*
 * generate new key from all the pools
 */
static void
reseed(FState *st)
{
    unsigned    k;
    unsigned    n;
    MD_CTX        key_md;
    uint8_t        buf[BLOCK];

    /* set pool as empty */
    st->pool0_bytes = 0;

    /*
     * Both #0 and #1 reseed would use only pool 0. Just skip #0 then.
     */
    n = ++st->reseed_count;

    /*
     * The goal: use k-th pool only 1/(2^k) of the time.
     */
    md_init(&key_md);
    for (k = 0; k < NUM_POOLS; k++)
    {
        md_result(&st->pool[k], buf);
        md_update(&key_md, buf, BLOCK);

        if (n & 1 || !n)
            break;
        n >>= 1;
    }

    /* add old key into mix too */
    md_update(&key_md, st->key, BLOCK);

    /* now we have new key */
    md_result(&key_md, st->key);

    /* use new key */
    ciph_init(&st->ciph, st->key, sizeof(st->key));

    memset(&key_md, 0, sizeof(key_md));
    memset(buf, 0, BLOCK);
}

/*
 * Pick a random pool.  This uses key bytes as random source.
 */
static unsigned
get_rand_pool(FState *st)
{
    unsigned    rnd;
    /*
     * This slightly prefers lower pools - that is OK.
     */
    rnd = st->key[st->rnd_pos] % NUM_POOLS;
    st->rnd_pos++;
    if (st->rnd_pos >= BLOCK)
        st->rnd_pos = 0;

    return rnd;
}

/*
 * update pools
 */
static void
add_entropy( const uint8_t *data, unsigned len )
{
    unsigned    pos;
    uint8_t     hash[BLOCK];
    MD_CTX      md;
    FState     *st = main_state;

    FORTUNA_DEBUG(( "%s\n", __FUNCTION__ ));

    /* hash given data */
    md_init(&md);
    md_update(&md, data, len);
    md_result(&md, hash);

    /*
     * Make sure the pool 0 is initialized, then update randomly.
     */
    if (st->reseed_count == 0)
        pos = 0;
    else
        pos = get_rand_pool(st);
    md_update(&st->pool[pos], hash, BLOCK);

    if (pos == 0)
        st->pool0_bytes += len;

    memset(hash, 0, BLOCK);
    memset(&md, 0, sizeof(md));
    memset(&pos, 0, sizeof(pos));
}

/*
 * Just take 2 next blocks as new key
 */
static void
rekey(FState *st)
{
    encrypt_counter(st, st->key);
    encrypt_counter(st, st->key + CIPH_BLOCK);
    ciph_init(&st->ciph, st->key, sizeof(st->key));
}

/*
 * Hide public constants. (counter, pools > 0)
 *
 * This can also be viewed as spreading the startup
 * entropy over all of the components.
 */
static void
startup_tricks(FState *st)
{
    int            i;
    uint8_t        buf[BLOCK];

    /* Use next block as counter. */
    encrypt_counter(st, st->counter);

    FORTUNA_DEBUG(( "inside startup tricks\n" ));

    /* Now shuffle pools, excluding #0 */
    for (i = 1; i < NUM_POOLS; i++)
    {
        encrypt_counter(st, buf);
        encrypt_counter(st, buf + CIPH_BLOCK);
        md_update(&st->pool[i], buf, BLOCK);
    }
    memset(buf, 0, BLOCK);

    /* Hide the key. */
    rekey(st);

    /* This can be done only once. */
    st->tricks_done = 1;
}

static void
extract_data( unsigned count, uint8_t *dst )
{
    unsigned    n;
    unsigned    block_nr = 0;
    FState     *st = main_state;

    FORTUNA_DEBUG(( "%s\n", __FUNCTION__ ));

    /* Should we reseed? */
    if (st->pool0_bytes >= POOL0_FILL || st->reseed_count == 0)
    {
        if (enough_time_passed(st))
        {
            reseed(st);
        }
    }

    FORTUNA_DEBUG(( "pre-startup tricks\n" ));

    /* Do some randomization on first call */
    if (!st->tricks_done)
        startup_tricks(st);

    FORTUNA_DEBUG(( "loop through bytes\n" ));

    while (count > 0)
    {
        /* produce bytes */
        encrypt_counter(st, st->result);

        /* copy result */
        if (count > CIPH_BLOCK)
            n = CIPH_BLOCK;
        else
            n = count;
        memcpy(dst, st->result, n);
        dst += n;
        count -= n;

        /* must not give out too many bytes with one key */
        block_nr++;
        if (block_nr > (RESEED_BYTES / CIPH_BLOCK))
        {
            rekey(st);
            block_nr = 0;
        }
    }

    FORTUNA_DEBUG(( "rekey\n" ));

    /* Set new key for next request. */
    rekey(st);
}

/*
 * public interface
 */
int
fortuna_enable( wiced_crypto_prng_t **set_to_prng_interface, fortuna_client_time_func_t get_time_function )
{
    int ok = FORTUNA_ERROR;

    /* if not already allocated */
    if ( NULL == main_state )
    {
        main_state = calloc( 1, sizeof(*main_state) );
    }

    if ( NULL != main_state )
    {
        ok = FORTUNA_OK;
        *set_to_prng_interface = &prng_fortuna;
    }

    FORTUNA_DEBUG(( "Enablement; ok=%d\n", ok ));

    client_rtos_time_function = get_time_function;

    return ok;
}

void
fortuna_disable( void )
{
    /* erase to defeat attacks */
    memset( main_state, 0, sizeof( *main_state ) );

    free( main_state );

    /* reset to initial state */
    main_state                = NULL;
    client_rtos_time_function = NULL;

    /* ensure init is redone in future, if needed */
    init_done = 0;
}

void
fortuna_add_entropy(const uint8_t *data, unsigned len)
{
    if ( 0 == init_done )
    {
        init_state( );
    }

    if ( NULL == data || 0 == len )
    {
        return;
    }

    add_entropy( data, len );
}

void
fortuna_get_bytes(unsigned len, uint8_t *dst)
{
    if ( 0 == init_done )
    {
        init_state( );
    }

    if ( NULL == dst || 0 == len)
    {
        return;
    }

    extract_data( len, dst );
}

/* These two functions adapt from the WICED API to the fortuna API */
static uint32_t prng_fortuna_get_random( void )
{
    uint32_t rand;
    fortuna_get_bytes( sizeof(rand), (uint8_t*)&rand );
    return rand;
}

static void prng_fortuna_add_entropy( const void* buffer, uint16_t buffer_length )
{
    fortuna_add_entropy( (const uint8_t*)buffer, (unsigned)buffer_length );
}
