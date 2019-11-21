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

/***************************************************************************
 *     Copyright (c) 2004-2014, Broadcom Corporation
 *     All Rights Reserved
 *     Confidential Property of Broadcom Corporation
 *
 *  THIS SOFTWARE MAY ONLY BE USED SUBJECT TO AN EXECUTED SOFTWARE LICENSE
 *  AGREEMENT  BETWEEN THE USER AND BROADCOM.  YOU HAVE NO RIGHT TO USE OR
 *  EXPLOIT THIS MATERIAL EXCEPT SUBJECT TO THE TERMS OF SUCH AN AGREEMENT.
 *
 ***************************************************************************/

#ifndef __MPEG_API_H__
#define __MPEG_API_H__

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Decoder frame info structure
 */
typedef struct
{
    uint32_t samplingfrequency;
    uint32_t numberofchannels;
    uint32_t framesize;
    uint32_t outbuffersize;
    uint32_t bitrate;
} mp3frameinfo;


/**
 * Decoder error types for API calls
 */
enum
{
    BRCM_MP3DEC_NO_ERROR                                = 0,
    BRCM_MP3DEC_MEM_ALLOCATION_ERROR                    = -1,     /* not enough memory allocated */
    BRCM_MP3DEC_UNSUPPORTED_BITSTREAM_CONFIG_ERROR      = -2,     /* bit stream configuration is not supported */
    BRCM_MP3DEC_NOT_ENOUGH_DATA_ERROR                   = -3,     /* not enough input data provided */
    BRCM_MP3DEC_BITSTREAM_DECODE_ERROR                  = -4,     /* Error in decoding the frame */
    BRCM_MP3DEC_DECODER_INTERNAL_ERROR                  = -5,     /* Internal decoder Failure */
    BRCM_MP3DEC_OUTPUT_BUFFER_SMALL_ERROR               = -6,     /* output buffer not enough */
    BRCM_MP3DEC_ERROR_NO_RESET                          = -7      /* internal error, no decoder reset required */
};


/**
 * brcm_mp3dec_getmem_requirement:
 *
 * returns the amount of memory to be allocated
 * by the application for the codec handle,
 * based on the pcm bps (16, 24, 32)
 * \param outpcm_width: bits for each PCM output sample
 *
 * \return: BRCM_MP3DEC error
 */
uint32_t brcm_mp3dec_getmem_requirement( uint32_t outpcm_width );


/**
 * brcm_mp3dec_createdecoder:
 *
 * create mp3 decoder inside the allocated handle.
 * \param mp3Handle: allocated handle memory area pointer
 * \param outpcm_width: number of bits for each output PCM sample
 *
 * \return: BRCM_MP3DEC error
 */
int32_t  brcm_mp3dec_createdecoder( void* mp3Handle, uint32_t outpcm_width );


/**
 * brcm_mp3dec_decodeframe:
 *
 * decode one frame from the input area and update input params
 * the pcm output buffer size must cointain the whole audio frame
 * which (worst case) can be 2048 samples (multiply for bit per sample)
 * \param mp3Handle: allocated handle memory area pointer
 * \param srcBuf: pointer to the input compressed data
 * \param srcUsed: i/o param for the size of srcBuf and
 * the amount of bits used after decoding one frame
 * \param dstBuf: pointer to the PCM output buffer
 * \param outSize: size in byte of the written PCM audio frame
 *
 * \return: BRCM_MP3DEC error
 */
int32_t brcm_mp3dec_decodeframe(
    void*     mp3Handle,
    char*     srcBuf,
    uint32_t* srcUsed,
    char*     dstBuf,
    uint32_t* outSize
    );


/**
 * brcm_mp3dec_getversion:
 *
 * return the version of the decoder
 *
 * \return: BRCM_MP3DEC error
 */
uint32_t brcm_mp3dec_getversion ( void );


/**
 * brcm_mp3dec_findsync:
 *
 * search for MP3 frame syncpoint before start decoding
 * \param mp3Handle: allocated handle memory area pointer
 * \param buf: pointer to the compressed sata buffer
 * \param offset: byte offset from the buf ptr to start searching
 * \param buf_size: size of the input compressed buffer
 *
 * \return: BRCM_MP3DEC error
 */
int32_t  brcm_mp3dec_findsync( void* mp3Handle, char* buf, uint32_t* offset, uint32_t buf_size );


/**
 * brcm_mp3dec_resetdecoder
 *
 * reset the decoder with the PCM width required
 * \param mp3Handle: allocated handle memory area pointer
 * \param outpcm_width: bit for each PCM sample
 *
 * \return: BRCM_MP3DEC error
 */
int32_t  brcm_mp3dec_resetdecoder( void* mp3Handle, uint32_t outpcm_width );


/**
 * brcm_mp3dec_deletedecoder:
 *
 * close and remove the codec
 * \param mp3Handle: allocated handle memory area pointer
 *
 * \return: BRCM_MP3DEC error
 */
int32_t  brcm_mp3dec_deletedecoder ( void* mp3Handle );


/**
 * brcm_mp3dec_getframeinfo:
 *
 * retrieve the information for a specific audio frame,
 * audio info do not change in a mp3 stream
 * \param mp3Handle: allocated handle memory area pointer
 * \param buf: pointer to the compressed audio data
 * \param bufSize: size of the compressed audio data buffer
 * \param frameOut: audio info
 *
 * \return: BRCM_MP3DEC error
 */
int32_t  brcm_mp3dec_getframeinfo ( void* mp3Handle, char* buf, uint32_t bufSize, mp3frameinfo* frameOut );

/**
 * brcm_mp3dec_getbitrate:
 *
 * retrieve current_frame_bitrate
 * \param mp3Handle: allocated handle memory area pointer
 *
 * \return: bitrate
 */
uint32_t brcm_mp3dec_getbitrate( void* mp3Handle );

/**
 * brcm_mp3dec_getsampleperframe:
 *
 * return the number of samples per frame
 * \param mp3Handle: allocated handle memory area pointer
 *
 * \return: Number of samples per frame
 */
uint16_t brcm_mp3dec_getsampleperframe( void* mp3Handle );

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
