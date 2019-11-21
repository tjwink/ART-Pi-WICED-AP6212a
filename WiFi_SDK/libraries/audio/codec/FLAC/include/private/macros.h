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

/* libFLAC - Free Lossless Audio Codec library
 * Copyright (C) 2012-2014  Xiph.org Foundation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * - Neither the name of the Xiph.org Foundation nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FLAC__PRIVATE__MACROS_H
#define FLAC__PRIVATE__MACROS_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined( __GNUC__ ) && ( __GNUC__ > 4 || ( __GNUC__ == 4 && __GNUC_MINOR__ >= 3 ) )

    #define flac_max( a, b )       \
    ( { __typeof__( a )_a = ( a ); \
        __typeof__( b )_b = ( b ); \
        _a > _b ? _a : _b; } )

    #define MIN_PASTE( A, B ) A ## B
    #define MIN_IMPL( A, B, L ) ( {                                                            \
        __typeof__( A )MIN_PASTE( __a, L ) = ( A );                                            \
        __typeof__( B )MIN_PASTE( __b, L ) = ( B );                                            \
        MIN_PASTE( __a, L ) < MIN_PASTE( __b, L ) ? MIN_PASTE( __a, L ) : MIN_PASTE( __b, L ); \
    } )

    #define flac_min( A, B ) MIN_IMPL( A, B, __COUNTER__ )

/* Whatever other unix that has sys/param.h */
#elif defined( HAVE_SYS_PARAM_H )
    #include <sys/param.h>
    #define flac_max( a, b ) MAX( a, b )
    #define flac_min( a, b ) MIN( a, b )

/* Windows VS has them in stdlib.h.. XXX:Untested */
#elif defined( _MSC_VER )
    #include <stdlib.h>
    #define flac_max( a, b ) __max( a, b )
    #define flac_min( a, b ) __min( a, b )
#endif

#ifndef MIN
    #define MIN( x, y )    ( ( x ) <= ( y ) ? ( x ) : ( y ) )
#endif

#ifndef MAX
    #define MAX( x, y )    ( ( x ) >= ( y ) ? ( x ) : ( y ) )
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
