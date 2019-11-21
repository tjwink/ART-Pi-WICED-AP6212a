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
 *  Defines cryptography functions for encryption, decryption, and hashing
 *
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include "crypto_structures.h"
#include "besl_structures.h"
#include "tls_types.h"
#include "mbedtls/ssl.h"
#include "mbedtls/aes.h"
#include "mbedtls/des.h"
#include "mbedtls/sha1.h"
#include "mbedtls/sha256.h"
#include "mbedtls/sha512.h"
#include "mbedtls/md4.h"
#include "mbedtls/md5.h"
#include "mbedtls/arc4.h"
#include "mbedtls/rsa.h"
#include "mbedtls/camellia.h"
#include "mbedtls/dhm.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 * @cond               Macros
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
 *              Global Variables
 ******************************************************/

/******************************************************
 *              Function Declarations
 * @endcond
 ******************************************************/

/*****************************************************************************/
/** @defgroup crypto       Crypto functions
 *
 *  WICED Cryptography functions
 */
/*****************************************************************************/

/**
 * @brief          AES-CBC buffer encryption/decryption with partial block padding
 *
 * @param[in]     ctx    : AES context
 * @param[in]     mode   : AES_ENCRYPT or AES_DECRYPT
 * @param[in]     length : Length of the input data
 * @param[in,out] iv     : Initialization vector (updated after use)
 * @param[in]     input  : Buffer holding the input data
 * @param[out]    output : Buffer receiving the output data
 */
int aes_cbc_crypt_pad_length_padding( mbedtls_aes_context *ctx, aes_mode_type_t mode, uint32_t length, const unsigned char iv[16], const unsigned char *input, unsigned char *output );

/**
 * @brief                   AES-CCM encryption
 *
 * @param[in]  ctx               : AES context
 * @param[in]  length            : Length of the input data
 * @param[in]  aad_length        : Length of the additional associated data
 * @param[in]  nonce             : The nonce to use
 * @param[in]  nonce_length      : Length of nonce.
 * @param[in]  aad_input         : The buffer containing the additional associated data
 * @param[in]  plaintext_input   : Buffer holding the input data
 * @param[out] ciphertext_output : Buffer which receives the output ciphertext
 * @param[out] mac_output        : Buffer which recieves the output MAC
 */
int aes_encrypt_ccm( mbedtls_aes_context *ctx, uint32_t length, uint32_t aad_length, const unsigned char *nonce, uint8_t nonce_length, const unsigned char *aad_input, const unsigned char *plaintext_input, unsigned char *ciphertext_output, unsigned char mac_output[8] );

/**
 * @brief                   AES-CCM decryption
 *
 * @param[in]  ctx              : AES context
 * @param[in]  length           : Length of the input data
 * @param[in]  aad_length       : Length of the additional associated data
 * @param[in]  nonce            : The nonce to use
 * @param[in]  nonce_length     : Length of nonce.
 * @param[in]  aad_input        : The buffer containing the additional associated data
 * @param[in]  ciphertext_input : Buffer holding the input data
 * @param[out] plaintext_output : Buffer which receives the output plaintext
 */
int aes_decrypt_ccm( mbedtls_aes_context *ctx, uint32_t length, uint32_t aad_length,  const unsigned char *nonce, uint8_t nonce_length, const unsigned char *aad_input, const unsigned char *ciphertext_input, unsigned char *plaintext_output );

/*****************************************************************************/
/** @addtogroup md5       Poly1305
 *  @ingroup crypto
 *
 * Poly1305 Message-Authentication Code (MAC) functions
 *
 *  @{
 */
/*****************************************************************************/

/**
 *  Get the poly1305 Message-Authentication code for a buffer of message data
 *  The Key MUST ONLY BE USED ONCE ONLY
 *  The sender MUST NOT use poly1305_auth to authenticate
 *  more than one message under the same key. Authenticators
 *  for two messages under the same key should be expected
 *  to reveal enough information to allow forgeries of
 *  authenticators on other messages.
 *
 *  @param[out] mac          : The output message-authentication code
 *  @param[in]  message_data : The message data to be processed through the authenticator
 *  @param[in]  bytes        : Number of bytes of message data to read
 *  @param[in]  key          : The UNIQUE 32 byte key to be used for this session
 */
void poly1305_auth   ( unsigned char mac[16], const unsigned char *message_data, size_t bytes, const unsigned char key[32]);



/*  Use these functions for processing non-contiguous data */

/**
 *  Initialise a poly1305 session with a key.
 *  The Key MUST ONLY BE USED ONCE ONLY
 *  The sender MUST NOT use poly1305_auth to authenticate
 *  more than one message under the same key. Authenticators
 *  for two messages under the same key should be expected
 *  to reveal enough information to allow forgeries of
 *  authenticators on other messages.
 *
 *  @param[in] context : A poly1305_context which will be used for scratch space
 *                       until poly1305_finish is called
 *  @param[in] key     : The UNIQUE 32 byte key to be used for this session
 */
void poly1305_init   ( poly1305_context *context, const unsigned char key[32]);

/**
 *  Process message data through poly1305 authenticator
 *
 *  @param[in] context      : A poly1305_context which has been initialised with poly1305_init
 *                            and will be used for scratch space until poly1305_finish is called
 *  @param[in] message_data : The message data to be processed through the authenticator
 *  @param[in] bytes        : Number of bytes of message data to read
 */
void poly1305_update ( poly1305_context *context, const unsigned char *message_data, size_t bytes);

/**
 *  Finish processing a poly1305 authenticator session
 *
 *  @param[in]  context  :  A poly1305_context which has been filled via poly1305_init and
 *                          poly1305_update. Will not be be used subsequently.
 *  @param[out] mac      :  The output message-authentication code
 */
void poly1305_finish ( poly1305_context *context, unsigned char mac[16]);


/**
 *  Verify that two Message-Authentication Codes match
 *
 *  @param[in] mac1  the first message-authentication code
 *  @param[in] mac2  the second message-authentication code
 *
 *  @return  1 if mac1 matches mac2,    0 otherwise
 */
int poly1305_verify(const unsigned char mac1[16], const unsigned char mac2[16]);

/** @} */

/*****************************************************************************/
/** @addtogroup chacha       ChaCha
 *  @ingroup crypto
 *
 * ChaCha cipher functions
 *
 *  @{
 */
/*****************************************************************************/


/**
 * Initialise ChaCha context with a key
 *
 * @param[in] context  : A chacha_context_t that will be used as temporary storage whilst performing ChaCha calculations
 * @param[in] key      : The key data - must be either 256 bits (32 bytes) or 128 bits (16 bytes) in length
 * @param[in] key_bits : The length of the key - must be either 256 or 128
 */
extern void chacha_keysetup       ( chacha_context_t *context, const uint8_t *key, uint32_t key_bits );

/**
 * Add an initial value to a ChaCha context
 *
 * @param[in] context       : A chacha_context_t that has been initialised with @ref chacha_keysetup
 * @param[in] initial_value : The initial value data - 8 bytes
 */
extern void chacha_ivsetup        ( chacha_context_t *context, const uint8_t *initial_value );

/**
 * Add an nonce initial value to a ChaCha20 block context
 *
 * @param[in] context     : A chacha_context_t that has been initialised with @ref chacha_keysetup
 * @param[in] nonce       : The initial value data - 12 bytes
 * @param[in] block_count : The sequence number
 */
extern void chacha20_block_ivsetup( chacha_context_t *context, const uint8_t nonce[12], uint32_t block_count );

/**
 * Add an nonce initial value to a ChaCha20 TLS context
 *
 * @param[in] context     : A chacha_context_t that has been initialised with @ref chacha_keysetup
 * @param[in] nonce       : The initial value data - 8 bytes
 * @param[in] block_count : The sequence number
 */
extern void chacha20_tls_ivsetup( chacha_context_t *context, const uint8_t nonce[8], uint64_t block_count );

/**
 * Encrypt data with the ChaCha Cipher
 *
 * @param[in]  context    : A chacha_context_t that has been initialised with @ref chacha_keysetup
 * @param[in]  plaintext  : The plaintext data to be encoded.
 * @param[out] ciphertext : Receives the output encrypted data
 * @param[in]  bytes      : Size in bytes of the plaintext (and encrypted) data
 * @param[in]  rounds     : Number of encryption loops to perform e.g. ChaCha20 = 20 rounds
 */
extern void chacha_encrypt_bytes  ( chacha_context_t *context, const uint8_t *plaintext, uint8_t *ciphertext, uint32_t bytes, uint8_t rounds);

/**
 * Decrypt data with the ChaCha Cipher
 *
 * @param[in]  context    : A chacha_context_t that has been initialised with @ref chacha_keysetup
 * @param[in]  ciphertext : The encrypted data to be decrypted.
 * @param[out] plaintext  : Receives the output plaintext data
 * @param[in]  bytes      : Size in bytes of the encrypted (and plaintext) data
 * @param[in]  rounds     : Number of encryption loops to perform e.g. ChaCha20 = 20 rounds
 */
extern void chacha_decrypt_bytes  ( chacha_context_t *context, const uint8_t *ciphertext, uint8_t *plaintext, uint32_t bytes, uint8_t rounds );

/**
 * Generates a stream that can be used for key material using the chacha cipher
 *
 * @param[in]  context  : A chacha_context_t that has been initialised with @ref chacha_keysetup
 * @param[out] stream   : The buffer that will receive the key stream data
 * @param[in]  bytes    : The number of bytes of keystream data to write to the buffer
 * @param[in]  rounds   : Number of encryption loops to perform e.g. ChaCha20 = 20 rounds
 */
extern void chacha_keystream_bytes( chacha_context_t *context, uint8_t *stream, uint32_t bytes, uint8_t rounds );

/**
 * Pseudo-random key generator using the ChaCha20 Cipher
 *
 * As per section 2.3 of draft-nir-cfrg-chacha20-poly1305-05
 *
 * @param[in]  key           : A 256 bit chacha key
 * @param[in]  nonce         : A 96 bit nonce - THIS MUST NEVER BE REUSED WITH THE SAME KEY
 * @param[in]  block_count   : Sequence number
 * @param[out] output_random : The buffer that will receive the 64 bit random data
 */
extern void chacha20_block_function( const uint8_t key[32],
                                     const uint8_t nonce[12],
                                     uint32_t block_count,
                                     uint8_t output_random[64] );


/**
 * Decrypt using ChaCha20-Poly1305 AEAD  (IETF CFRG version)
 *
 * @param[in]  key                                  : The 256 bit key
 * @param[in]  fixed_common_value                   : 32 bit fixed value for nonce
 * @param[in]  non_repeating_initial_value          : Initial value for nonce
 *                                                  : THIS VALUE MUST *NEVER* BE REPEATED FOR THE SAME
 *                                                  : KEY. USE A COUNTER IF POSSIBLE.
 * @param[in]  additional_authenticated_data        : Additional data (non encrypted) to be authenticated in tag
 * @param[in]  additional_authenticated_data_length : The length of the additional data in bytes
 * @param[in]  input_crypt_data                     : The data to be encrypted/decrypted
 * @param[in]  crypt_data_length                    : Length of the data being encrypted/decrypted
 * @param[out] output_crypt_data                    : The output encrypted or decrypted data.
 * @param[out] output_tag                           : The authentication tag calculated for the data
 */
extern void chacha20_poly1305_aead_irtf_cfrg_encrypt(
        const uint8_t                       key[32],
        const uint8_t                       fixed_common_value[4],
        const uint8_t                       non_repeating_initial_value[8],
        const uint8_t*                      additional_authenticated_data,
        uint64_t                            additional_authenticated_data_length,
        const uint8_t*                      input_crypt_data,
        uint64_t                            crypt_data_length,
        uint8_t*                            output_crypt_data,
        uint8_t                             output_tag[16] );

/**
 * Decrypt using ChaCha20-Poly1305 AEAD  (IETF CFRG version)
 *
 * @param[in]  key                                  : The 256 bit key
 * @param[in]  fixed_common_value                   : 32 bit fixed value for nonce
 * @param[in]  non_repeating_initial_value          : Initial value for nonce
 *                                                  : THIS VALUE MUST *NEVER* BE REPEATED FOR THE SAME
 *                                                  : KEY. USE A COUNTER IF POSSIBLE.
 * @param[in]  additional_authenticated_data        : Additional data (non encrypted) to be authenticated in tag
 * @param[in]  additional_authenticated_data_length : The length of the additional data in bytes
 * @param[in]  input_crypt_data                     : The data to be encrypted/decrypted
 * @param[in]  crypt_data_length                    : Length of the data being encrypted/decrypted
 * @param[out] output_crypt_data                    : The output encrypted or decrypted data.
 * @param[out] output_tag                           : The authentication tag calculated for the data
 */
extern void chacha20_poly1305_aead_irtf_cfrg_decrypt(
        const uint8_t                       key[32],
        const uint8_t                       fixed_common_value[4],
        const uint8_t                       non_repeating_initial_value[8],
        const uint8_t*                      additional_authenticated_data,
        uint64_t                            additional_authenticated_data_length,
        const uint8_t*                      input_crypt_data,
        uint64_t                            crypt_data_length,
        uint8_t*                            output_crypt_data,
        uint8_t                             output_tag[16] );

/**
 * Encrypt using ChaCha20-Poly1305 AEAD (TLS version)
 *
 * @param[in]  key                                  : The 256 bit key
 * @param[in]  nonce                                : 8 byte nonce
 * @param[in]  additional_authenticated_data        : Additional data (non encrypted) to be authenticated in tag
 * @param[in]  additional_authenticated_data_length : The length of the additional data in bytes
 * @param[in]  input_crypt_data                     : The data to be encrypted
 * @param[in]  crypt_data_length                    : Length of the data being encrypted/decrypted
 * @param[out] output_crypt_data                    : The output encrypted or decrypted data.
 * @param[out] output_tag                           : The authentication tag calculated for the data
 */
void chacha20_poly1305_aead_tls_encrypt(
        const uint8_t                       key[32],
        const uint8_t                       nonce[8],
        const uint8_t*                      additional_authenticated_data,
        uint64_t                            additional_authenticated_data_length,
        const uint8_t*                      input_crypt_data,
        uint64_t                            crypt_data_length,
        uint8_t*                            output_crypt_data,   /* length is same as input_crypt_data */
        uint8_t                             output_tag[16] );

/** @} */

/**
 * Decrypt using ChaCha20-Poly1305 AEAD (TLS version)
 *
 * @param[in]  key                                  : The 256 bit key
 * @param[in]  nonce                                : 8 byte nonce
 * @param[in]  additional_authenticated_data        : Additional data (non encrypted) to be authenticated in tag
 * @param[in]  additional_authenticated_data_length : The length of the additional data in bytes
 * @param[in]  input_crypt_data                     : The data to be decrypted
 * @param[in]  crypt_data_length                    : Length of the data being encrypted/decrypted
 * @param[out] output_crypt_data                    : The output encrypted or decrypted data.
 * @param[out] output_tag                           : The authentication tag calculated for the data
 */
void chacha20_poly1305_aead_tls_decrypt(
        const uint8_t                       key[32],
        const uint8_t                       nonce[8],
        const uint8_t*                      additional_authenticated_data,
        uint64_t                            additional_authenticated_data_length,
        const uint8_t*                      input_crypt_data,
        uint64_t                            crypt_data_length,
        uint8_t*                            output_crypt_data,   /* length is same as input_crypt_data */
        uint8_t                             output_tag[16] );
/*****************************************************************************/
/** @addtogroup curve25519       Curve25519
 *  @ingroup crypto
 *
 * Curve25519 elliptic curve key generation functions
 *
 *  @{
 */
/*****************************************************************************/

/**
 * Calculate a public (shared) key given a basepoint and secret key
 *
 * @param[out] mypublic_output : Receives the 32 byte output shared key
 * @param[in]  secret          : The 32 byte secret key. Must have been randomly
 *                               generated then have the following operations performed
 *                               on it:
 *                                 secret[0]  &= 248;
 *                                 secret[31] &= 127;
 *                                 secret[31] |= 64;
 * @param[in]  basepoint       : The starting point for the calculation - usually the
 *                               public key of the other party
 *
 * @return 0 when successful
 */
int curve25519( uint8_t *mypublic_output, const uint8_t *secret, const uint8_t *basepoint );

/** @} */

/*****************************************************************************/
/** @addtogroup ed25519       Ed25519
 *  @ingroup crypto
 *
 * Ed25519 digital signature functions
 *
 *  @{
 */
/*****************************************************************************/

/**
 * Generate a Ed25519 public key from a secret key
 *
 * @param[in]  secret_key : The 32 byte secret key
 * @param[out] public_key : Receives the 32 byte output public key
 */
void ed25519_publickey( const ed25519_secret_key secret_key, ed25519_public_key public_key );


/**
 * Sign a message using Ed25519
 *
 * @param[in]  message_data     : The message data to sign
 * @param[in]  message_len      : The length in bytes of the message data
 * @param[in]  secret_key       : The 32 byte secret key
 * @param[in]  public_key       : The 32 byte public key
 * @param[out] signature_output : Receives the 64 byte output signature
 */
void ed25519_sign( const unsigned char *message_data, size_t message_len, const ed25519_secret_key secret_key, const ed25519_public_key public_key, ed25519_signature signature_output );

/**
 * Verify an Ed25519 message signature
 *
 * @param[in]  message_data       : The message data to verify
 * @param[in]  message_len        : The length in bytes of the message data
 * @param[in]  public_key         : The 32 byte public key
 * @param[out] signature Receives : The 64 byte output signature
 *
 * @return 0 if signature matches
 */
int ed25519_sign_open( const unsigned char *message_data, size_t message_len, const ed25519_public_key public_key, const ed25519_signature signature);



/*****************************************************************************/
/** @addtogroup SEED       SEED
 *  @ingroup crypto
 *
 * SEED cipher functions
 *
 *  @{
 */
/*****************************************************************************/

/**
 * @brief          Set SEED key
 *
 * @param[in] rawkey :  Encryption key
 * @param[in] ks     :  SEED context to be initialized
 */
void seed_set_key(const unsigned char rawkey[16], seed_context_t *ks);

/**
 * @brief          SEED-CBC buffer encryption
 *
 * @param[in]     ctx   :  SEED context
 * @param[in,out] ivec  :  Initialization vector (updated after use)
 * @param[in]     in    :  Buffer holding the input data
 * @param[in]     len   :  Length of the input data
 * @param[out]    out   :  Buffer that receives the output data
 */
void seed_cbc_encrypt(const seed_context_t* ctx, uint8_t ivec[16], const uint8_t *in, uint32_t len, uint8_t *out);

/**
 * @brief          SEED-CBC buffer decryption
 *
 * @param[in]     ctx   :  SEED context
 * @param[in,out] ivec  :  Initialization vector (updated after use)
 * @param[in]     in    :  Buffer holding the input data
 * @param[in]     len   :  Length of the input data
 * @param[out]    out   :  Buffer that receives the output data
 */
void seed_cbc_decrypt(const seed_context_t* ctx, uint8_t ivec[16], const uint8_t *in, uint32_t len, uint8_t *out);


/** @} */

/*****************************************************************************/
/** @addtogroup x509       x509
 *  @ingroup crypto
 *
 * x509 functions
 *
 *  @{
 */
/*****************************************************************************/

/**
 * @brief          Parse one or more certificates and add them to the chained list
 *
 * @param[in] chain  :  Points to the start of the chain
 * @param[in] buf    :  Buffer holding the certificate data
 * @param[in] buflen :  Size of the buffer
 *
 * @return         0 if successful, or a specific X509 error code
 */
int32_t x509parse_crt( x509_cert *chain, const unsigned char *buf, uint32_t buflen );

/**
 * @brief          Load one or more certificates and add them to the chained list
 *
 * @param[in] chain :  Points to the start of the chain
 * @param[in] path  :  Filename to read the certificates from
 *
 * @return         0 if successful, or a specific X509 error code
 */
int32_t x509parse_crtfile( x509_cert *chain, const char *path );

/**
 * @brief          Parse a public RSA key
 *
 * @param[in] rsa    : RSA context to be initialized
 * @param[in] buf    : Input buffer
 * @param[in] buflen : Size of the buffer
 * @param[in] pwd    : Password for decryption (optional)
 * @param[in] pwdlen : Size of the password
 *
 * @return         0 if successful, or a specific X509 error code
 */
int32_t x509parse_pubkey( rsa_context *rsa,
                   unsigned char *buf, int32_t buflen,
                   unsigned char *pwd, int32_t pwdlen );


/**
 * @brief          Parse a private RSA key
 *
 * @param[in] rsa    : RSA context to be initialized
 * @param[in] buf    : Input buffer
 * @param[in] buflen : Size of the buffer
 * @param[in] pwd    : Password for decryption (optional)
 * @param[in] pwdlen : Size of the password
 *
 * @return         0 if successful, or a specific X509 error code
 */
int32_t x509parse_key( rsa_context *rsa,
                       const unsigned char *buf, uint32_t buflen,
                       const unsigned char *pwd, uint32_t pwdlen );

/**
 * @brief          Load and parse a private RSA key
 *
 * @param[in] rsa  :  RSA context to be initialized
 * @param[in] path :  Filename to read the private key from
 * @param[in] pwd  :  Password to decrypt the file (can be NULL)
 *
 * @return         0 if successful, or a specific X509 error code
 */
int32_t x509parse_keyfile( rsa_context *rsa, const char *path, const char *pwd);

/**
 * @brief          Store the certificate DN in printable form into buf;
 *                 no more than (end - buf) characters will be written.
 *
 * @param[out] buf : Pointer to output buffer
 * @param[in]  end : Pointer to the last valid location in buf
 * @param[in]  dn  : Certificate DN
 *
 * @return       Number of characters written into buf
 */
int32_t x509parse_dn_gets( char *buf, const char *end, const x509_name *dn );

/**
 * @brief          Returns an informational string about the
 *                 certificate.
 *
 * @param[out] buf      : Pointer to output buffer
 * @param[in]  buf_size : Size of buf in bytes
 * @param[in]  prefix   : Prefix string
 * @param[in]  crt      : Pointer to the certificate
 *
 * @return      Pointer to buf or NULL if memory allocation fails.
 */
char *x509parse_cert_info(char *buf, size_t buf_size,
              const char *prefix, const x509_cert * crt);

/**
 * @brief          Return 0 if the certificate is still valid,
 *                 or BADCERT_EXPIRED
 *
 * @param[in] crt  :  Certificate to be checked
 *
 * @return         Return 0 if the certificate is still valid, or BADCERT_EXPIRED
 */
int32_t x509parse_expired( const x509_cert *crt );


/**
 * @brief          Verify the certificate signature
 *
 * @param[in]  crt      : A certificate to be verified
 * @param[in]  trust_ca : The trusted CA chain
 * @param[in]  cn       : Expected Common Name (can be set to
 *                        NULL if the CN must not be verified)
 * @param[out] flags    : Result of the verification
 *
 * @return         0 if successful or MYKROSSL_ERR_X509_SIG_VERIFY_FAILED,
 *                 in which case *flags will have one or more of
 *                 the following values set:
 *                      BADCERT_EXPIRED --
 *                      BADCERT_REVOKED --
 *                      BADCERT_CN_MISMATCH --
 *                      BADCERT_NOT_TRUSTED
 *
 * @note           TODO: add two arguments, depth and crl
 */
int32_t x509parse_verify( const x509_cert *crt,
                          const x509_cert *trust_ca,
                          const char *cn,
                          int32_t *flags );

/**
 * @brief          Unallocate all certificate data
 *
 * @param[in]  crt   : A certificate to be freed
 */
void x509_free( x509_cert *crt );

/** @} */

/*****************************************************************************/
/** @addtogroup 80211       802.11
 *  @ingroup crypto
 *
 *  802.11 functions
 *
 *  @{
 */
/*****************************************************************************/

extern besl_result_t besl_802_11_generate_pmk              ( const char* password, const unsigned char* ssid, int ssid_length, unsigned char* output );
extern besl_result_t besl_802_11_generate_random_passphrase( char* passphrase, const int passphrase_length );

/** @} */


/*****************************************************************************/
/** @addtogroup microrng       MICRORNG
 *  @ingroup crypto
 *
 *  microrang functions
 *
 *  @{
 */
/*****************************************************************************/

/**
 * @brief          MICRORNG initialization
 *
 * @param[in] state  :  MICRORNG state to be initialized
 *
 * caller can optionally supply an entropy value
 * in state that may be zero by default
 */
void microrng_init( microrng_state *state);

/**
 * @brief          MICRORNG rand function
 *
 * @param[in] p_state : Points to an MICRORNG state
 *
 * @return         A random int
 */
int32_t microrng_rand( void *p_state );

/** @} */

#ifdef __cplusplus
} /*extern "C" */
#endif
