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
#if !defined(MBEDTLS_CONFIG_FILE)
#include "mbedtls/config.h"
#else
#include MBEDTLS_CONFIG_FILE
#endif

#if defined( MBEDTLS_WICED_ECC_ALT )
#include "uECC_vli.h"
#include "mbedtls/ecp.h"

static uECC_Curve wiced_get_ecc_curve( int grp_id )
{
    uECC_Curve ecc_Curve = NULL;
    switch( grp_id )
    {
#if uECC_SUPPORTS_secp192r1
        case MBEDTLS_ECP_DP_SECP192R1:
            ecc_Curve = uECC_secp192r1();
            break;
#endif
#if uECC_SUPPORTS_secp224r1
        case MBEDTLS_ECP_DP_SECP224R1:
            ecc_Curve = uECC_secp224r1();
            break;
#endif
#if uECC_SUPPORTS_secp256r1
        case MBEDTLS_ECP_DP_SECP256R1:
            ecc_Curve = uECC_secp256r1();
            break;
#endif
#if uECC_SUPPORTS_secp256k1
        case MBEDTLS_ECP_DP_SECP256K1:
            ecc_Curve = uECC_secp256k1();
            break;
#endif
        default:
            return NULL;
    }
    return ecc_Curve;
}

int wiced_ecp_gen_keypair( mbedtls_ecp_group *grp, mbedtls_mpi *d, mbedtls_ecp_point *Q )
{
    uint8_t private_key[32];
    uint8_t public_key[64];
    uECC_Curve ecc_Curve = NULL;
    int private_key_size, public_key_size;
    int ret;

    ecc_Curve = wiced_get_ecc_curve( grp->id );
    if( ecc_Curve == NULL)
    {
        return 1;
    }

    private_key_size = uECC_curve_private_key_size( ecc_Curve );
    public_key_size = uECC_curve_public_key_size( ecc_Curve );

    /* Allocate memory for private and public keys
     * mbedtls_mpi_grow takes size in 4 byte words
     */
    mbedtls_mpi_grow( d, private_key_size/4 );
    mbedtls_mpi_grow( &Q->X, public_key_size/2/4 );
    mbedtls_mpi_grow( &Q->Y, public_key_size/2/4 );
    ret = uECC_make_key( public_key, private_key, ecc_Curve );

    if( ret != 1 )
    {
        /* uECC_make_key returns 1 on success */
        return 1;
    }
    /* store private key */
    uECC_vli_bytesToNative( d->p, private_key, private_key_size );
    /* store public key */
    uECC_vli_bytesToNative( Q->X.p, public_key, public_key_size/2 );
    uECC_vli_bytesToNative( Q->Y.p, public_key + (public_key_size/2), public_key_size/2 );

    mbedtls_mpi_lset( &Q->Z, 1 );
    return 0;

}

int wiced_ecdh_compute_shared( mbedtls_ecp_group *grp, mbedtls_mpi *z, const mbedtls_ecp_point *Q, const mbedtls_mpi *d)
{
    uint8_t private_key[32];
    uint8_t public_key[64];
    uint8_t secret[32];
    uECC_Curve ecc_Curve = NULL;
    int private_key_size, public_key_size;
    int ret=0;

    ecc_Curve = wiced_get_ecc_curve( grp->id );
    if( ecc_Curve == NULL )
    {
        return 1;
    }

    private_key_size = uECC_curve_private_key_size( ecc_Curve );
    public_key_size = uECC_curve_public_key_size( ecc_Curve );

    uECC_vli_nativeToBytes( private_key, private_key_size, d->p );

    uECC_vli_nativeToBytes( public_key, public_key_size/2, Q->X.p );
    uECC_vli_nativeToBytes( public_key + (public_key_size/2), public_key_size/2, Q->Y.p );

    ret = uECC_shared_secret( public_key, private_key, secret, ecc_Curve );

    if( ret != 1 )
    {
        /* uECC_make_key returns 1 on success */
        return 1;
    }

    ret = mbedtls_mpi_grow( z, private_key_size/4 );

    uECC_vli_bytesToNative( z->p, secret, public_key_size/2 );
    return( ret );
}

int wiced_ecdsa_sign( mbedtls_ecp_group *grp, mbedtls_mpi *r, mbedtls_mpi *s, mbedtls_mpi *d, const unsigned char *hash, size_t hlen )
{
    uint8_t private_key[ 32 ];
    uint8_t signature[ 64 ] ;
    uECC_Curve ecc_Curve = NULL;
    int private_key_size, public_key_size;
    int ret = 0;

    ecc_Curve = wiced_get_ecc_curve( grp->id );
    if( ecc_Curve == NULL)
    {
        return 1;
    }

    private_key_size = uECC_curve_private_key_size( ecc_Curve );
    public_key_size = uECC_curve_public_key_size( ecc_Curve );

    uECC_vli_nativeToBytes( private_key, private_key_size, d->p );

    ret = uECC_sign( private_key, hash, hlen, signature, ecc_Curve );

    if( ret != 1 )
    {
        /* uECC_verify returns 1 on success */
        return MBEDTLS_ERR_ECP_INVALID_KEY;
    }
    ret = 0;

    mbedtls_mpi_grow( r, public_key_size/2 );
    mbedtls_mpi_grow( s, public_key_size/2 );

    uECC_vli_bytesToNative( r->p, signature, public_key_size/2 );
    uECC_vli_bytesToNative( s->p, signature + (public_key_size/2), public_key_size/2 );

    return ret;
}

int wiced_ecdsa_verify( mbedtls_ecp_group *grp, const unsigned char *hash, size_t hlen, mbedtls_ecp_point *Q, mbedtls_mpi *r, mbedtls_mpi *s )
{
    uint8_t public_key[ 64 ];
    uint8_t signature[ 64 ];
    int public_key_size;
    uECC_Curve ecc_Curve = NULL;
    int ret = 0;
    ecc_Curve = wiced_get_ecc_curve( grp->id );
    if( ecc_Curve == NULL)
    {
        return 1;
    }
    public_key_size = uECC_curve_public_key_size( ecc_Curve );

    uECC_vli_nativeToBytes( public_key, public_key_size/2, Q->X.p );
    uECC_vli_nativeToBytes( public_key + (public_key_size/2), public_key_size/2, Q->Y.p );

    uECC_vli_nativeToBytes( signature, public_key_size/2, r->p );
    uECC_vli_nativeToBytes( signature + (public_key_size/2), public_key_size/2, s->p );

    ret = uECC_verify( public_key, hash, hlen, signature, ecc_Curve );
    if(ret != 1)
    {
        /* uECC_verify returns 1 on success */
        return MBEDTLS_ERR_ECP_VERIFY_FAILED;
    }
    ret = 0;
    return ret;

}
#endif //MBEDTLS_WICED_ECC_ALT
