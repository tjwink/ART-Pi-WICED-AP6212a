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

#include <string.h>  /* for NULL */
#include "cipher_suites.h"

const char * cipher_names[] =
{
    [NULL_CIPHER              ] = "NULL",
    [RC4_40_CIPHER            ] = "RC4_40",
    [RC4_128_CIPHER           ] = "RC4_128",
    [RC2_CBC_40_CIPHER        ] = "RC2_CBC_40",
    [IDEA_CBC_CIPHER          ] = "IDEA_CBC",
    [DES40_CBC_CIPHER         ] = "DES40_CBC",
    [DES_CBC_CIPHER           ] = "DES_CBC",
    [DES_CBC_40_CIPHER        ] = "DES_CBC_40",
    [TRIPLE_DES_EDE_CBC_CIPHER] = "3DES_EDE_CBC",
    [AES_128_CBC_CIPHER       ] = "AES_128_CBC",
    [AES_256_CBC_CIPHER       ] = "AES_256_CBC",
    [AES_128_GCM_CIPHER       ] = "AES_128_GCM",
    [AES_256_GCM_CIPHER       ] = "AES_256_GCM",
    [AES_128_CCM_CIPHER       ] = "AES_128_CCM",
    [AES_256_CCM_CIPHER       ] = "AES_256_CCM",
    [AES_128_CCM_8_CIPHER     ] = "AES_128_CCM_8",
    [AES_256_CCM_8_CIPHER     ] = "AES_256_CCM_8",
    [CAMELLIA_128_CBC_CIPHER  ] = "CAMELLIA_128_CBC",
    [CAMELLIA_256_CBC_CIPHER  ] = "CAMELLIA_256_CBC",
    [CAMELLIA_128_GCM_CIPHER  ] = "CAMELLIA_128_GCM",
    [CAMELLIA_256_GCM_CIPHER  ] = "CAMELLIA_256_GCM",
    [SEED_CBC_CIPHER          ] = "SEED_CBC",
    [ARIA_128_CBC_CIPHER      ] = "ARIA_128_CBC",
    [ARIA_256_CBC_CIPHER      ] = "ARIA_256_CBC",
    [ARIA_128_GCM_CIPHER      ] = "ARIA_128_GCM",
    [ARIA_256_GCM_CIPHER      ] = "ARIA_256_GCM",
    [CHACHA20_POLY1305_CIPHER ] = "CHACHA20_POLY1305",
};

const char* keyscheme_names[] =
{
    [NULL_KEYSCHEME           ] = "NULL",
    [KRB5_KEYSCHEME           ] = "KRB5",
    [KRB5_EXPORT_KEYSCHEME    ] = "KRB5_EXPORT",
    [RSA_EXPORT_KEYSCHEME     ] = "RSA_EXPORT",
    [DH_DSS_EXPORT_KEYSCHEME  ] = "DH_DSS_EXPORT",
    [DHE_RSA_EXPORT_KEYSCHEME ] = "DHE_RSA_EXPORT",
    [DH_anon_KEYSCHEME        ] = "DH_anon",
    [DH_anon_EXPORT_KEYSCHEME ] = "DH_anon_EXPORT",
    [DH_RSA_EXPORT_KEYSCHEME  ] = "DH_RSA_EXPORT",
    [DHE_DSS_EXPORT_KEYSCHEME ] = "DHE_DSS_EXPORT",
    [ECDH_anon_KEYSCHEME      ] = "ECDH_anon",
    [RSA_KEYSCHEME            ] = "RSA",
    [DH_DSS_KEYSCHEME         ] = "DH_DSS",
    [DH_RSA_KEYSCHEME         ] = "DH_RSA",
    [DHE_DSS_KEYSCHEME        ] = "DHE_DSS",
    [DHE_RSA_KEYSCHEME        ] = "DHE_RSA",
    [ECDH_ECDSA_KEYSCHEME     ] = "ECDH_ECDSA",
    [ECDH_RSA_KEYSCHEME       ] = "ECDH_RSA",
    [ECDHE_RSA_KEYSCHEME      ] = "ECDHE_RSA",
    [ECDHE_ECDSA_KEYSCHEME    ] = "ECDHE_ECDSA",
    [PSK_KEYSCHEME            ] = "PSK",
    [RSA_PSK_KEYSCHEME        ] = "RSA_PSK",
    [DHE_PSK_KEYSCHEME        ] = "DHE_PSK",
    [ECDHE_PSK_KEYSCHEME      ] = "ECDHE_PSK",
    [SRP_SHA_KEYSCHEME        ] = "SRP_SHA",
    [SRP_SHA_RSA_KEYSCHEME    ] = "SRP_SHA_RSA",
    [SRP_SHA_DSS_KEYSCHEME    ] = "SRP_SHA_DSS",
};

const char* mac_names[] =
{
    [NULL_MAC          ] = "NULL",
    [MD5_MAC           ] = "MD5",
    [SHA_MAC           ] = "SHA",
    [SHA256_MAC        ] = "SHA256",
    [SHA384_MAC        ] = "SHA384",
    [AES_128_CCM_MAC   ] = "",
    [AES_256_CCM_MAC   ] = "",
    [AES_128_CCM_8_MAC ] = "",
    [AES_266_CCM_8_MAC ] = "",
};
