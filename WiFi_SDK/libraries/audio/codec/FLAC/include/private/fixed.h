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
 * Copyright (C) 2000-2009  Josh Coalson
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

#ifndef FLAC__PRIVATE__FIXED_H
#define FLAC__PRIVATE__FIXED_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef HAVE_CONFIG_H
    #include <config.h>
#endif

#include "private/cpu.h"
#include "private/float.h"
#include "FLAC/format.h"

/*
 *    FLAC__fixed_compute_best_predictor()
 *    --------------------------------------------------------------------
 *    Compute the best fixed predictor and the expected bits-per-sample
 *  of the residual signal for each order.  The _wide() version uses
 *  64-bit integers which is statistically necessary when bits-per-
 *  sample + log2(blocksize) > 30
 *
 *    IN data[0,data_len-1]
 *    IN data_len
 *    OUT residual_bits_per_sample[0,FLAC__MAX_FIXED_ORDER]
 */
#ifndef FLAC__INTEGER_ONLY_LIBRARY
unsigned FLAC__fixed_compute_best_predictor( const FLAC__int32 data[], unsigned data_len, FLAC__float residual_bits_per_sample[ FLAC__MAX_FIXED_ORDER + 1 ] );
unsigned FLAC__fixed_compute_best_predictor_wide( const FLAC__int32 data[], unsigned data_len, FLAC__float residual_bits_per_sample[ FLAC__MAX_FIXED_ORDER + 1 ] );
    # ifndef FLAC__NO_ASM
        #  if ( defined FLAC__CPU_IA32 || defined FLAC__CPU_X86_64 ) && defined FLAC__HAS_X86INTRIN
            #   ifdef FLAC__SSE2_SUPPORTED
unsigned FLAC__fixed_compute_best_predictor_intrin_sse2( const FLAC__int32 data[], unsigned data_len, FLAC__float residual_bits_per_sample[ FLAC__MAX_FIXED_ORDER + 1 ] );
unsigned FLAC__fixed_compute_best_predictor_wide_intrin_sse2( const FLAC__int32 data[], unsigned data_len, FLAC__float residual_bits_per_sample[ FLAC__MAX_FIXED_ORDER + 1 ] );
            #   endif
            #   ifdef FLAC__SSSE3_SUPPORTED
unsigned FLAC__fixed_compute_best_predictor_intrin_ssse3( const FLAC__int32 data[], unsigned data_len, FLAC__float residual_bits_per_sample[ FLAC__MAX_FIXED_ORDER + 1 ] );
unsigned FLAC__fixed_compute_best_predictor_wide_intrin_ssse3( const FLAC__int32 data[], unsigned data_len, FLAC__float residual_bits_per_sample[ FLAC__MAX_FIXED_ORDER + 1 ] );
            #   endif
        #  endif
        #  if defined FLAC__CPU_IA32 && defined FLAC__HAS_NASM
unsigned FLAC__fixed_compute_best_predictor_asm_ia32_mmx_cmov( const FLAC__int32 data[], unsigned data_len, FLAC__float residual_bits_per_sample[ FLAC__MAX_FIXED_ORDER + 1 ] );
        #  endif
    # endif
#else
unsigned FLAC__fixed_compute_best_predictor( const FLAC__int32 data[], unsigned data_len, FLAC__fixedpoint residual_bits_per_sample[ FLAC__MAX_FIXED_ORDER + 1 ] );
unsigned FLAC__fixed_compute_best_predictor_wide( const FLAC__int32 data[], unsigned data_len, FLAC__fixedpoint residual_bits_per_sample[ FLAC__MAX_FIXED_ORDER + 1 ] );
#endif

/*
 *    FLAC__fixed_compute_residual()
 *    --------------------------------------------------------------------
 *    Compute the residual signal obtained from sutracting the predicted
 *    signal from the original.
 *
 *    IN data[-order,data_len-1]        original signal (NOTE THE INDICES!)
 *    IN data_len                       length of original signal
 *    IN order <= FLAC__MAX_FIXED_ORDER fixed-predictor order
 *    OUT residual[0,data_len-1]        residual signal
 */
void FLAC__fixed_compute_residual( const FLAC__int32 data[], unsigned data_len, unsigned order, FLAC__int32 residual[] );

/*
 *    FLAC__fixed_restore_signal()
 *    --------------------------------------------------------------------
 *    Restore the original signal by summing the residual and the
 *    predictor.
 *
 *    IN residual[0,data_len-1]         residual signal
 *    IN data_len                       length of original signal
 *    IN order <= FLAC__MAX_FIXED_ORDER fixed-predictor order
 *    *** IMPORTANT: the caller must pass in the historical samples:
 *    IN  data[-order,-1]               previously-reconstructed historical samples
 *    OUT data[0,data_len-1]            original signal
 */
void FLAC__fixed_restore_signal( const FLAC__int32 residual[], unsigned data_len, unsigned order, FLAC__int32 data[] );

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
