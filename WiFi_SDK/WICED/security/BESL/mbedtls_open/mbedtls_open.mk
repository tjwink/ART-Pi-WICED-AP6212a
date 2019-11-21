#
# Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 # Cypress Semiconductor Corporation. All Rights Reserved.
 # This software, including source code, documentation and related
 # materials ("Software"), is owned by Cypress Semiconductor Corporation
 # or one of its subsidiaries ("Cypress") and is protected by and subject to
 # worldwide patent protection (United States and foreign),
 # United States copyright laws and international treaty provisions.
 # Therefore, you may use this Software only as provided in the license
 # agreement accompanying the software package from which you
 # obtained this Software ("EULA").
 # If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 # non-transferable license to copy, modify, and compile the Software
 # source code solely for use in connection with Cypress's
 # integrated circuit products. Any reproduction, modification, translation,
 # compilation, or representation of this Software except as specified
 # above is prohibited without the express written permission of Cypress.
 #
 # Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 # EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 # reserves the right to make changes to the Software without notice. Cypress
 # does not assume any liability arising out of the application or use of the
 # Software or any product or circuit described in the Software. Cypress does
 # not authorize its products for use in any products where a malfunction or
 # failure of the Cypress product may reasonably be expected to result in
 # significant property damage, injury or death ("High Risk Product"). By
 # including Cypress's product in a High Risk Product, the manufacturer
 # of such system or application assumes all risk of such use and in doing
 # so agrees to indemnify Cypress against all liability.
#

NAME := Lib_mbedTLS

ifneq ($(HOST_MCU_FAMILY),ix86)
$(NAME)_ALWAYS_OPTIMISE := 1
endif

$(NAME)_SOURCES +=  library/aes.c                     \
                    library/aesni.c                   \
                    library/arc4.c                    \
                    library/asn1parse.c               \
                    library/asn1write.c               \
                    library/base64.c                  \
                    library/bignum.c                  \
                    library/blowfish.c                \
                    library/camellia.c                \
                    library/ccm.c                     \
                    library/cipher.c                  \
                    library/cipher_wrap.c             \
                    library/cmac.c                    \
                    library/ctr_drbg.c                \
                    library/des.c                     \
                    library/dhm.c                     \
                    library/ecdh.c                    \
                    library/ecdsa.c                   \
                    library/ecjpake.c                 \
                    library/ecp.c                     \
                    library/ecp_curves.c              \
                    library/entropy.c                 \
                    library/entropy_poll.c            \
                    library/error.c                   \
                    library/gcm.c                     \
                    library/havege.c                  \
                    library/hmac_drbg.c               \
                    library/md.c                      \
                    library/md2.c                     \
                    library/md4.c                     \
                    library/md5.c                     \
                    library/md_wrap.c                 \
                    library/memory_buffer_alloc.c     \
                    library/oid.c                     \
                    library/padlock.c                 \
                    library/pem.c                     \
                    library/pk.c                      \
                    library/pk_wrap.c                 \
                    library/pkcs12.c                  \
                    library/pkcs5.c                   \
                    library/pkparse.c                 \
                    library/pkwrite.c                 \
                    library/platform.c                \
                    library/ripemd160.c               \
                    library/rsa.c                     \
                    library/sha1.c                    \
                    library/sha256.c                  \
                    library/sha512.c                  \
                    library/threading.c               \
                    library/timing.c                  \
                    library/version.c                 \
                    library/version_features.c        \
                    library/xtea.c                    \
                    library/certs.c                   \
                    library/pkcs11.c                  \
                    library/x509.c                    \
                    library/x509_create.c             \
                    library/x509_crl.c                \
                    library/x509_crt.c                \
                    library/x509_csr.c                \
                    library/x509write_crt.c           \
                    library/x509write_csr.c           \
                    library/debug.c                   \
                    library/net_sockets.c             \
                    library/ssl_cache.c               \
                    library/ssl_ciphersuites.c        \
                    library/ssl_cli.c                 \
                    library/ssl_cookie.c              \
                    library/ssl_srv.c                 \
                    library/ssl_ticket.c              \
                    library/ssl_tls.c                 \
                    library/ecp_alt.c                 \
                    library/aes_alt.c                 \
                    library/des_alt.c                 \
                    library/sha256_alt.c              \
                    library/sha1_alt.c                \
                    library/md5_alt.c

GLOBAL_INCLUDES +=  include
                    
ifeq ($(IAR),)
$(NAME)_CFLAGS =  -fno-strict-aliasing
endif
