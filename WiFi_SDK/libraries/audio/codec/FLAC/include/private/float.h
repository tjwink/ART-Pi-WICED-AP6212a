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
 * Copyright (C) 2004-2009  Josh Coalson
 * Copyright (C) 2011-2014  Xiph.Org Foundation
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

#ifndef FLAC__PRIVATE__FLOAT_H
#define FLAC__PRIVATE__FLOAT_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAVE_CONFIG_H
    #include <config.h>
#endif

#include "FLAC/ordinals.h"

/*
 * These typedefs make it easier to ensure that integer versions of
 * the library really only contain integer operations.  All the code
 * in libFLAC should use FLAC__float and FLAC__double in place of
 * float and double, and be protected by checks of the macro
 * FLAC__INTEGER_ONLY_LIBRARY.
 *
 * FLAC__real is the basic floating point type used in LPC analysis.
 */
#ifndef FLAC__INTEGER_ONLY_LIBRARY
typedef double FLAC__double;
typedef float  FLAC__float;

/*
 * WATCHOUT: changing FLAC__real will change the signatures of many
 * functions that have assembly language equivalents and break them.
 */
typedef float FLAC__real;
#else

/*
 * The convention for FLAC__fixedpoint is to use the upper 16 bits
 * for the integer part and lower 16 bits for the fractional part.
 */
typedef FLAC__int32 FLAC__fixedpoint;
extern const FLAC__fixedpoint FLAC__FP_ZERO;
extern const FLAC__fixedpoint FLAC__FP_ONE_HALF;
extern const FLAC__fixedpoint FLAC__FP_ONE;
extern const FLAC__fixedpoint FLAC__FP_LN2;
extern const FLAC__fixedpoint FLAC__FP_E;

    #define FLAC__fixedpoint_trunc( x ) ( ( x ) >> 16 )

    #define FLAC__fixedpoint_mul( x, y ) ( (FLAC__fixedpoint) ( ( (FLAC__int64) ( x ) * (FLAC__int64) ( y ) ) >> 16 ) )

    #define FLAC__fixedpoint_div( x, y ) ( (FLAC__fixedpoint) ( ( ( (FLAC__int64) ( x ) << 32 ) / (FLAC__int64) ( y ) ) >> 16 ) )

/*
 *    FLAC__fixedpoint_log2()
 *    --------------------------------------------------------------------
 *    Returns the base-2 logarithm of the fixed-point number 'x' using an
 *    algorithm by Knuth for x >= 1.0
 *
 *    'fracbits' is the number of fractional bits of 'x'.  'fracbits' must
 *    be < 32 and evenly divisible by 4 (0 is OK but not very precise).
 *
 *    'precision' roughly limits the number of iterations that are done;
 *    use (unsigned)(-1) for maximum precision.
 *
 *    If 'x' is less than one -- that is, x < (1<<fracbits) -- then this
 *    function will punt and return 0.
 *
 *    The return value will also have 'fracbits' fractional bits.
 */
FLAC__uint32 FLAC__fixedpoint_log2( FLAC__uint32 x, unsigned fracbits, unsigned precision );
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
