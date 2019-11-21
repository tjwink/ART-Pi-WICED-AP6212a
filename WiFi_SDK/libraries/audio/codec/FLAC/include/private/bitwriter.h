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

#ifndef FLAC__PRIVATE__BITWRITER_H
#define FLAC__PRIVATE__BITWRITER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h> /* for FILE */
#include "FLAC/ordinals.h"

/*
 * opaque structure definition
 */
struct FLAC__BitWriter;
typedef struct FLAC__BitWriter FLAC__BitWriter;

/*
 * construction, deletion, initialization, etc functions
 */
FLAC__BitWriter* FLAC__bitwriter_new( void );
void             FLAC__bitwriter_delete( FLAC__BitWriter* bw );
FLAC__bool       FLAC__bitwriter_init( FLAC__BitWriter* bw );
void             FLAC__bitwriter_free( FLAC__BitWriter* bw ); /* does not 'free(buffer)' */
void             FLAC__bitwriter_clear( FLAC__BitWriter* bw );
void             FLAC__bitwriter_dump( const FLAC__BitWriter* bw, FILE* out );

/*
 * CRC functions
 *
 * non-const *bw because they have to cal FLAC__bitwriter_get_buffer()
 */
FLAC__bool FLAC__bitwriter_get_write_crc16( FLAC__BitWriter* bw, FLAC__uint16* crc );
FLAC__bool FLAC__bitwriter_get_write_crc8( FLAC__BitWriter* bw, FLAC__byte* crc );

/*
 * info functions
 */
FLAC__bool FLAC__bitwriter_is_byte_aligned( const FLAC__BitWriter* bw );
unsigned   FLAC__bitwriter_get_input_bits_unconsumed( const FLAC__BitWriter* bw ); /* can be called anytime, returns total # of bits unconsumed */

/*
 * direct buffer access
 *
 * there may be no calls on the bitwriter between get and release.
 * the bitwriter continues to own the returned buffer.
 * before get, bitwriter MUST be byte aligned: check with FLAC__bitwriter_is_byte_aligned()
 */
FLAC__bool FLAC__bitwriter_get_buffer( FLAC__BitWriter* bw, const FLAC__byte** buffer, size_t* bytes );
void       FLAC__bitwriter_release_buffer( FLAC__BitWriter* bw );

/*
 * write functions
 */
FLAC__bool FLAC__bitwriter_write_zeroes( FLAC__BitWriter* bw, unsigned bits );
FLAC__bool FLAC__bitwriter_write_raw_uint32( FLAC__BitWriter* bw, FLAC__uint32 val, unsigned bits );
FLAC__bool FLAC__bitwriter_write_raw_int32( FLAC__BitWriter* bw, FLAC__int32 val, unsigned bits );
FLAC__bool FLAC__bitwriter_write_raw_uint64( FLAC__BitWriter* bw, FLAC__uint64 val, unsigned bits );
FLAC__bool FLAC__bitwriter_write_raw_uint32_little_endian( FLAC__BitWriter* bw, FLAC__uint32 val ); /*only for bits=32*/
FLAC__bool FLAC__bitwriter_write_byte_block( FLAC__BitWriter* bw, const FLAC__byte vals[], unsigned nvals );
FLAC__bool FLAC__bitwriter_write_unary_unsigned( FLAC__BitWriter* bw, unsigned val );
unsigned   FLAC__bitwriter_rice_bits( FLAC__int32 val, unsigned parameter );

#if 0 /* UNUSED */
unsigned FLAC__bitwriter_golomb_bits_signed( int val, unsigned parameter );
unsigned FLAC__bitwriter_golomb_bits_unsigned( unsigned val, unsigned parameter );
#endif
FLAC__bool FLAC__bitwriter_write_rice_signed( FLAC__BitWriter* bw, FLAC__int32 val, unsigned parameter );
FLAC__bool FLAC__bitwriter_write_rice_signed_block( FLAC__BitWriter* bw, const FLAC__int32* vals, unsigned nvals, unsigned parameter );

#if 0 /* UNUSED */
FLAC__bool FLAC__bitwriter_write_golomb_signed( FLAC__BitWriter* bw, int val, unsigned parameter );
FLAC__bool FLAC__bitwriter_write_golomb_unsigned( FLAC__BitWriter* bw, unsigned val, unsigned parameter );
#endif
FLAC__bool FLAC__bitwriter_write_utf8_uint32( FLAC__BitWriter* bw, FLAC__uint32 val );
FLAC__bool FLAC__bitwriter_write_utf8_uint64( FLAC__BitWriter* bw, FLAC__uint64 val );
FLAC__bool FLAC__bitwriter_zero_pad_to_byte_boundary( FLAC__BitWriter* bw );

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
