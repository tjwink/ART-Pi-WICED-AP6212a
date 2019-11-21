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

#include "wiced.h"
#include "message.h"
#include "key_public.h"
#include "key_private.h"
#include "mbedtls/rsa.h"
#include "mbedtls/sha256.h"
#include "crypto_constants.h"

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
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    int32_t ret, n;
    mbedtls_rsa_context rsa_priv;
    mbedtls_rsa_context rsa_pub;
    uint8_t hash[SHA256_DIGEST_LENGTH];
    uint8_t signature[RSA_KEY_SZ/8];

    WPRINT_APP_INFO(("\n\n\n*** RSA PKCS#1 demo ***\n\n"));

    /* 1. Init Private Key */
    WPRINT_APP_INFO(("RSA Private Key Init...\n"));

    memset( &rsa_priv, 0, sizeof( rsa_priv ) );

    mbedtls_rsa_init( &rsa_priv, MBEDTLS_RSA_PKCS_V15, 0 );

    if( ( ret = mbedtls_mpi_read_string( &rsa_priv.N , 16, RSA_PRIVATE_N ) ) != 0 ||
        ( ret = mbedtls_mpi_read_string( &rsa_priv.E , 16, RAS_PRIVATE_E ) ) != 0 ||
        ( ret = mbedtls_mpi_read_string( &rsa_priv.D , 16, RSA_PRIVATE_D ) ) != 0 ||
        ( ret = mbedtls_mpi_read_string( &rsa_priv.P , 16, RSA_PRIVATE_P ) ) != 0 ||
        ( ret = mbedtls_mpi_read_string( &rsa_priv.Q , 16, RSA_PRIVATE_Q ) ) != 0 ||
        ( ret = mbedtls_mpi_read_string( &rsa_priv.DP, 16, RSA_PRIVATE_DP ) ) != 0 ||
        ( ret = mbedtls_mpi_read_string( &rsa_priv.DQ, 16, RSA_PRIVATE_DQ ) ) != 0 ||
        ( ret = mbedtls_mpi_read_string( &rsa_priv.QP, 16, RSA_PRIVATE_QP ) ) != 0 )
    {
        WPRINT_APP_INFO(("RSA Private Key Init Failed: mpi_read_file returned %d\n", ( int ) ret));
        return;
    }

    rsa_priv.len = ( mpi_msb( &rsa_priv.N ) + 7 ) >> 3;

    WPRINT_APP_INFO(("RSA Private Key Init Done. key size = %u bits\n", ( unsigned ) rsa_priv.len * 8));

    /* 2. Init Public Key */
    WPRINT_APP_INFO(("RSA Public Key Init...\n"));

    memset( &rsa_pub, 0, sizeof( rsa_pub ) );

    mbedtls_rsa_init( &rsa_pub, MBEDTLS_RSA_PKCS_V15, 0 );

    if( ( ret = mbedtls_mpi_read_string( &rsa_pub.N , 16, RSA_PUBLIC_N ) ) != 0 ||
        ( ret = mbedtls_mpi_read_string( &rsa_pub.E , 16, RSA_PUBLIC_E ) ) != 0 )
    {
        WPRINT_APP_INFO(("RSA Public Key Init Failed: mpi_read_file returned %d\n", ( int ) ret));
        return;
    }

    rsa_pub.len = ( mpi_msb( (const mbedtls_mpi*) &rsa_priv.N ) + 7 ) >> 3;

    WPRINT_APP_INFO(("RSA Public Key Init Done. key size = %u bits\n", ( unsigned ) rsa_pub.len * 8));

    /* 3. Show Original Message */
    n = sizeof( MESSAGE );

    printf( "\nOriginal Message:\n");

    printf( "length = %d\n", ( int ) n );

    printf( "%s\n\n", MESSAGE );

    /* 4. Sign with private key & Verification with public key */

    memset( hash, 0, sizeof ( hash ));
    memset( signature, 0, sizeof ( signature ));

    mbedtls_sha256( ( const unsigned char * ) MESSAGE, n, hash, 0);

    WPRINT_APP_INFO(("PKCS#1 Signing with Private Key...\n"));
    if( ( ret = mbedtls_rsa_pkcs1_sign( &rsa_priv, NULL, NULL, MBEDTLS_RSA_PRIVATE, MBEDTLS_MD_SHA256,
                                0, hash, signature ) ) != 0 )
    {
        WPRINT_APP_INFO(("PKCS#1 Signing with Private Key Failed: rsa_pkcs1_sign returned %d\n", ( int ) ret ));
        return;
    }
    WPRINT_APP_INFO(("PKCS#1 Signing with Private Key Done.\n"));

    WPRINT_APP_INFO(("PKCS#1 Verifying Signature with Public Key...\n"));
    if( ( ret = mbedtls_rsa_pkcs1_verify( &rsa_priv, NULL, NULL, MBEDTLS_RSA_PUBLIC, MBEDTLS_MD_SHA256,
                                  0, hash, signature ) ) != 0 )
    {
        WPRINT_APP_INFO(("PKCS#1 verification with Public Key Failed: rsa_pkcs1_verify returned %d\n", ( int ) ret ));
        return;
    }
    WPRINT_APP_INFO(("PKCS#1 Verifying Signature with Public Key Done.\n"));

    /* 5. Sign with public key & Verification with private key */

    memset( hash, 0, sizeof ( hash ));
    memset( signature, 0, sizeof ( signature ));

    mbedtls_sha256( ( const unsigned char * ) MESSAGE, n, hash, 0);

    WPRINT_APP_INFO(("PKCS#1 Signing with Public Key...\n"));
    if( ( ret = mbedtls_rsa_pkcs1_sign( &rsa_priv, NULL, NULL, MBEDTLS_RSA_PUBLIC, MBEDTLS_MD_SHA256,
                                0, hash, signature ) ) != 0 )
    {
        WPRINT_APP_INFO(("PKCS#1 Signing with Public Key Failed: rsa_pkcs1_sign returned %d\n", ( int ) ret ));
        return;
    }
    WPRINT_APP_INFO(("PKCS#1 Signing with Public Key Done.\n"));

    WPRINT_APP_INFO(("PKCS#1 Verifying Signature with Private Key...\n"));
    if( ( ret = mbedtls_rsa_pkcs1_verify( &rsa_priv, NULL, NULL, MBEDTLS_RSA_PRIVATE, MBEDTLS_MD_SHA256,
                                  0, hash, signature ) ) != 0 )
    {
        WPRINT_APP_INFO(("PKCS#1 verification with Private Key Failed: rsa_pkcs1_verify returned %d\n", ( int ) ret ));
        return;
    }
    WPRINT_APP_INFO(("PKCS#1 Verifying Signature with Private Key Done.\n"));

    /* 6. uninit keyes */
    mbedtls_rsa_free( &rsa_priv );
    mbedtls_rsa_free( &rsa_pub );
}
