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

/* libFLAC - Free Lossless Audio Codec
 * Copyright (C) 2002-2009  Josh Coalson
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

#ifndef FLAC__PRIVATE__OGG_ENCODER_ASPECT_H
#define FLAC__PRIVATE__OGG_ENCODER_ASPECT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ogg/ogg.h>

#include "FLAC/ordinals.h"
#include "FLAC/stream_encoder.h" /* for FLAC__StreamEncoderWriteStatus */

typedef struct FLAC__OggEncoderAspect
{
    /* these are storage for values that can be set through the API */
    long             serial_number;
    unsigned         num_metadata;

    /* these are for internal state related to Ogg encoding */
    ogg_stream_state stream_state;
    ogg_page         page;
    FLAC__bool       seen_magic; /* true if we've seen the fLaC magic in the write callback yet */
    FLAC__bool       is_first_packet;
    FLAC__uint64     samples_written;
} FLAC__OggEncoderAspect;

void       FLAC__ogg_encoder_aspect_set_serial_number( FLAC__OggEncoderAspect* aspect, long value );
FLAC__bool FLAC__ogg_encoder_aspect_set_num_metadata( FLAC__OggEncoderAspect* aspect, unsigned value );
void       FLAC__ogg_encoder_aspect_set_defaults( FLAC__OggEncoderAspect* aspect );
FLAC__bool FLAC__ogg_encoder_aspect_init( FLAC__OggEncoderAspect* aspect );
void       FLAC__ogg_encoder_aspect_finish( FLAC__OggEncoderAspect* aspect );

typedef FLAC__StreamEncoderWriteStatus (* FLAC__OggEncoderAspectWriteCallbackProxy)( const void* encoder, const FLAC__byte buffer[], size_t bytes, unsigned samples, unsigned current_frame, void* client_data );

FLAC__StreamEncoderWriteStatus FLAC__ogg_encoder_aspect_write_callback_wrapper( FLAC__OggEncoderAspect* aspect, const FLAC__byte buffer[], size_t bytes, unsigned samples, unsigned current_frame, FLAC__bool is_last_block, FLAC__OggEncoderAspectWriteCallbackProxy write_callback, void* encoder, void* client_data );

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
