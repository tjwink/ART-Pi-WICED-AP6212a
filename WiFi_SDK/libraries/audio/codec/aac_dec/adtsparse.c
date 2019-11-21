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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined ( LINUX )
    #include <netinet/in.h>
#endif

#include "globals.h"
#include "adtsparse_core.h"
#include "tsp.h"
#include "tpm.h"
#include "sysclk.h"

#include "logger.h"

#include "aacdec_types.h"
#include "aacdec_core.h"

#include "adtsparse_types.h"
#include "adtsparse.h"

/*
 * internal timeout for start/stop failures
 */
#define ADTSPARSE_TIMEOUT_MS ( (uint32_t) 200 )


/*
 * search & indexing flags
 */
#define SEARCH_EMPTY_FLAG                  ( (uint32_t) ( 0 ) )
#define SEARCH_NONE_FOUND                  ( (uint32_t) ( 0 ) )
#define SEARCH_SYNCWORD_FOUND        ( (uint32_t) ( 1 <<  1 ) )
#define SEARCH_PRIVATE_FOUND         ( (uint32_t) ( 1 <<  2 ) )
#define SEARCH_LAYER_FOUND           ( (uint32_t) ( 1 <<  3 ) )
#define SEARCH_CRC_FOUND             ( (uint32_t) ( 1 <<  4 ) )
/**/
#define SEARCH_RESERVED_31           ( (uint32_t) ( 1 << 31 ) )


/* ****************************************************************************** */
/* ****************************************************************************** */
/* PROTOTYPES                                                                     */
/* ****************************************************************************** */
/* ****************************************************************************** */
static void init_adtsparse( int8_t* err, adtsparse_t* adtsparse );
static void destroy_adtsparse( int8_t* err, adtsparse_t** adtsparse );

inline int8_t        match_adtsparse( int8_t* err, adtsparse_t* adtsparse, adtsparse_ctype_t checktype );
static inline int8_t check_ready_adtsparse( int8_t* err, adtsparse_t* adtsparse );

tsp_loop_return adts_dsp_loop( tsp_loop_param data );


/* ****************************************************************************** */
/* ****************************************************************************** */
/* DSP-PROTOTYPES                                                                 */
/* ****************************************************************************** */
/* ****************************************************************************** */
static inline void adts_find_frame( int8_t*      err,
                                    adtsparse_t* adtsparse );



/* ****************************************************************************** */
/* ****************************************************************************** */
/* TABLES                                                                         */
/* ****************************************************************************** */
/* ****************************************************************************** */

#if LOGGER_ENABLED
/* adts header values */
static const uint32_t adts_sampling_rate_lut[ ADTS_HDR_SAMPLING_RATE_MAX ] =
{ 96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350 };
#endif

/* adts channel_config mapping */
#define AACDEC_CHCFG_CHNUM ( 0 )
#define AACDEC_CHCFG_CHMAP ( 1 )

static const uint32_t adts_chmap_lut[][ 2 ] =
{
    { 0,                                                                                                                                                 0 }, /* see table 42 in ISO/IEC 13818-7, this value is undef */
    { 1, AACDEC_CHANNEL_MAP_C                                                                                                                              },
    { 2, AACDEC_CHANNEL_MAP_L   | AACDEC_CHANNEL_MAP_R                                                                                                     },
    { 3, AACDEC_CHANNEL_MAP_L   | AACDEC_CHANNEL_MAP_R   | AACDEC_CHANNEL_MAP_C                                                                            },
    { 4, AACDEC_CHANNEL_MAP_L   | AACDEC_CHANNEL_MAP_R   | AACDEC_CHANNEL_MAP_C   | AACDEC_CHANNEL_MAP_BC                                                  },
    { 5, AACDEC_CHANNEL_MAP_L   | AACDEC_CHANNEL_MAP_R   | AACDEC_CHANNEL_MAP_C   | AACDEC_CHANNEL_MAP_BL | AACDEC_CHANNEL_MAP_BR                          },
    { 6, AACDEC_CHANNEL_MAP_L   | AACDEC_CHANNEL_MAP_R   | AACDEC_CHANNEL_MAP_C   | AACDEC_CHANNEL_MAP_BL | AACDEC_CHANNEL_MAP_BR | AACDEC_CHANNEL_MAP_LFE },
    { 7, AACDEC_CHANNEL_MAP_L   | AACDEC_CHANNEL_MAP_R   | AACDEC_CHANNEL_MAP_C   | AACDEC_CHANNEL_MAP_BL | AACDEC_CHANNEL_MAP_BR |
      AACDEC_CHANNEL_MAP_LFE | AACDEC_CHANNEL_MAP_FLC | AACDEC_CHANNEL_MAP_FRC }
};

/* ****************************************************************************** */
/* ****************************************************************************** */
/* DSP                                                                            */
/* ****************************************************************************** */
/* ****************************************************************************** */
static inline void adts_find_frame( int8_t* err, adtsparse_t* adtsparse )
{
    int8_t       myerr = 0;

    uint32_t     i    = 0;

    uint8_t*     id2 = NULL;

    adtsparse_t* adtsp = adtsparse;

    adts_hdr_t   adts_hdr;

    /*
     * start from where we left
     */
    i = adtsp->search_idx;

    LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] adts_find_frame: start idx=%" PRIu32 " (raw_idx=%" PRIu32 ")\n",
             adtsp->label,
             i,
             adtsp->data_raw_w_idx );

    /*
     * loop over raw stream
     */
    while ( ( i < ( adtsp->data_raw_w_idx - 2 ) ) &&
            ( 0 == myerr ) &&
            ( !( ( adtsp->search_status & SEARCH_SYNCWORD_FOUND ) &&
                 ( adtsp->search_status & SEARCH_PRIVATE_FOUND  ) &&
                 ( adtsp->search_status & SEARCH_CRC_FOUND      ) ) ) )
    {
        id2 = (uint8_t*) ( adtsp->data_raw + i );

        /* SYNCWORD */
        if ( ( id2[ 0 ] == 0xFF ) &&
             ( ( id2[ 1 ] & 0xF0 ) == 0xF0 ) )
        {
            if ( adtsp->search_status & SEARCH_SYNCWORD_FOUND )
            {
                LOG_MSG( 0, LOG_ALWAYS, "[ADTSPARSE][%s] [ERROR] adts_find_frame: invalid stream (sync without Private and CRC)\n",
                         adtsp->label );
                myerr = ADTSPARSE_ERR_UNKNOWN;
                return;
            }

            /* SANITY CHECK minimum num of bytes needed */
            if ( adtsp->data_raw_w_idx < ( i + 9 ) )
            {
                if ( NULL != err )
                {
                    *err = ADTSPARSE_ERR_NEED_MORE_BITS;
                }

                return;
            }

            /* mark this finding */
            adtsp->search_status |= SEARCH_SYNCWORD_FOUND;

            adtsp->data_bitstream_skip_f = FALSE;

            LOG_MSG( adtsp->log_lvl, LOG_MEDIUM, "[ADTSPARSE][%s] adts_find_frame : frame found %x %x\n",
                     adtsp->label,
                     id2[ 0 ], id2[ 1 ] );

            adts_hdr.fixed.id                                 = ( id2[ 1 ] >> 3 ) & 0x01;
            adts_hdr.fixed.layer                              = ( id2[ 1 ] >> 1 ) & 0x03;
            adts_hdr.fixed.protection_absent                  = ( id2[ 1 ] >> 0 ) & 0x01;
            adts_hdr.fixed.profile                            = ( id2[ 2 ] >> 6 );

            adts_hdr.fixed.sampling_frequency_idx             = ( id2[ 2 ] >> 2 ) & 0x0F;
            adts_hdr.fixed.private_bit                        = ( id2[ 2 ] >> 1 ) & 0x01;
            adts_hdr.fixed.channel_configuration              = ( ( id2[ 2 ] & 0x01 ) << 2 ) | ( id2[ 3 ] >> 6 );
            adts_hdr.fixed.original_copy                      = ( id2[ 3 ] >> 5 ) & 0x01;
            adts_hdr.fixed.home                               = ( id2[ 3 ] >> 4 ) & 0x01;

            adts_hdr.variable.copyright_identification_bit    = ( id2[ 3 ] >> 3 ) & 0x01;
            adts_hdr.variable.copyright_identification_start  = ( id2[ 3 ] >> 2 ) & 0x01;
            adts_hdr.variable.aac_frame_length                = ( id2[ 3 ] & 0x03 ) << 11 | id2[ 4 ] << 3 | id2[ 5 ] >> 5;
            adts_hdr.variable.adts_buffer_fullness            = ( id2[ 5 ] & 0x1F ) << 6 | id2[ 6 ] >> 2;
            adts_hdr.variable.num_raw_data_blocks_in_frame    = ( id2[ 6 ] ) & 0x03;

            /* sanity checks on fixed header */
            if ( adts_hdr.fixed.layer != 0 )
            {
                LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s] ADTS ERROR: wrong layer in the hdr\n",
                         adtsp->label );
                myerr = -1;
            }

            if ( adts_hdr.fixed.sampling_frequency_idx >= ADTS_HDR_SAMPLING_RATE_MAX )
            {
                LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s] ADTS ERROR: invalid sampling frequency\n",
                         adtsp->label );
                myerr = -1;
            }

            if ( ( adts_hdr.fixed.profile == ADTS_HDR_AAC_LTP ) &&
                 ( adts_hdr.fixed.id == ADTS_HDR_ID_MPEG2 ) )
            {
                LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s] ADTS ERROR: invalid ID/PROFILE match (MPEG-2 with LTP)\n",
                         adtsp->label );
                myerr = -1;
            }


            if ( NO_ERR == myerr )
            {
                adtsp->adts_hdr.is_valid = TRUE;

                /* log format */
                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] ADTS_HDR_FIX ID=%s\n",
                         adtsp->label, ( adts_hdr.fixed.id == 0 ) ? "MPEG-2" : "MPEG-4" );

                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] ADTS_HDR_FIX LAYER=%" PRIu8 "\n",
                         adtsp->label, adts_hdr.fixed.layer );

                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] ADTS_HDR_FIX PROTECT=%" PRIu8 "\n",
                         adtsp->label, adts_hdr.fixed.protection_absent );

                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] ADTS_HDR_FIX PROFILE=%" PRIu8 "\n",
                         adtsp->label, adts_hdr.fixed.profile );

                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] ADTS_HDR_FIX SAMPLING_FREQ=%" PRIu8 " %" PRIu32 "Hz\n",
                         adtsp->label, adts_hdr.fixed.sampling_frequency_idx,
                         adts_sampling_rate_lut[ adts_hdr.fixed.sampling_frequency_idx ] );

                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] ADTS_HDR_FIX PRIVATE=%" PRIu8 "\n",
                         adtsp->label, adts_hdr.fixed.private_bit );

                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] ADTS_HDR_FIX CHANNEL_CFG=%" PRIu8 "\n",
                         adtsp->label, adts_hdr.fixed.channel_configuration );

                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] ADTS_HDR_FIX ORIGINAL/COPY=%" PRIu8 "\n",
                         adtsp->label, adts_hdr.fixed.original_copy );

                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] ADTS_HDR_FIX HOME=%" PRIu8 "\n",
                         adtsp->label, adts_hdr.fixed.home );

                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] ADTS_HDR_VAR FRAME_LENGTH=%" PRIu32 "\n",
                         adtsp->label, (uint32_t) adts_hdr.variable.aac_frame_length );

                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] ADTS_HDR_VAR BUFFER_FULLNESS=%" PRIu32 "\n",
                         adtsp->label, (uint32_t) adts_hdr.variable.adts_buffer_fullness );

                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] ADTS_HDR_VAR BLOCK=%" PRIu32 "\n",
                         adtsp->label, (uint32_t) adts_hdr.variable.num_raw_data_blocks_in_frame );

                adtsp->search_status |= SEARCH_PRIVATE_FOUND;
                adtsp->search_status |= SEARCH_CRC_FOUND;
            }
        }

        i++;
    }

    /*
     * if we got the DSC we pass the location and the size for it
     */

    LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s]s.return. (s.idx=%" PRIu32 ")(b.idx %" PRIu64 ") (%" PRIu8 " %" PRIu8 "%" PRIu8 ")\n\n",
             adtsp->label,
             adtsp->search_idx,
             adtsp->data_bitstream_idx,
             ( ( adtsp->search_status & SEARCH_CRC_FOUND ) != 0 ),
             ( ( adtsp->search_status & SEARCH_SYNCWORD_FOUND ) != 0 ),
             ( ( adtsp->search_status & SEARCH_PRIVATE_FOUND ) != 0 ) );


    if ( NULL != err )
    {
        *err = myerr;
    }
}


static int8_t adts_dsp_init_fdk(adtsparse_t* adtsp, uint32_t* fdk_dec_cb_size)
{
    adtsp->fdk_err = aacDecoder_GetFreeBytes(adtsp->fdk_handle, (UINT*)fdk_dec_cb_size);
    if (adtsp->fdk_err != AAC_DEC_OK)
    {
        LOG_ERR(0, LOG_ALWAYS, "[ADTSPARSE][%s] [ERROR] adts_dsp_init_fdk: unable to get decoder buffer size.\n", adtsp->label);

        return ADTSPARSE_ERR_UNKNOWN;
    }

    /* set extra/optional parameters : sample interleaving */
    adtsp->fdk_err = aacDecoder_SetParam(adtsp->fdk_handle, AAC_PCM_OUTPUT_INTERLEAVED, 1 /* interleaved */);

    if (adtsp->fdk_err != AAC_DEC_OK)
    {
        LOG_ERR(0, LOG_ALWAYS, "[ADTSPARSE][%s] [ERROR] adts_dsp_init_fdk: unable to set FDK optional paramters.\n", adtsp->label);

        return ADTSPARSE_ERR_UNKNOWN;
    }

    /* set extra/optional parameters : channel mapping WAV style  */
    adtsp->fdk_err = aacDecoder_SetParam(adtsp->fdk_handle, AAC_PCM_OUTPUT_CHANNEL_MAPPING, 1 /* WAV order*/);

    if (adtsp->fdk_err != AAC_DEC_OK)
    {
        LOG_ERR(0, LOG_ALWAYS, "[ADTSPARSE][%s] [ERROR] adts_dsp_init_fdk: unable to set FDK optional paramters.\n", adtsp->label);

        return ADTSPARSE_ERR_UNKNOWN;
    }

    /* OPTIONAL: if the target channel map is stereo (FL+FR) */
    /* it is safe to request to always output stereo sample  */
    /* even for mono or multi channel adts streams */
    if (adtsp->pcm_info_tgt_chmap == (AACDEC_CHANNEL_MAP_FL | AACDEC_CHANNEL_MAP_FR))
    {
        adtsp->fdk_err = aacDecoder_SetParam(adtsp->fdk_handle, AAC_PCM_MIN_OUTPUT_CHANNELS, 2 /* WAV order*/);

        if (adtsp->fdk_err != AAC_DEC_OK)
        {
            LOG_ERR(0, LOG_ALWAYS, "[ADTSPARSE][%s] [ERROR] adts_dsp_init_fdk: unable to set FDK min_out_chnum paramter.\n", adtsp->label);

            return ADTSPARSE_ERR_UNKNOWN;
        }

        adtsp->fdk_err = aacDecoder_SetParam(adtsp->fdk_handle, AAC_PCM_MAX_OUTPUT_CHANNELS, 2 /* WAV order*/);

        if (adtsp->fdk_err != AAC_DEC_OK)
        {
            LOG_ERR(0, LOG_ALWAYS, "[ADTSPARSE][%s] [ERROR] adts_dsp_init_fdk: unable to set FDK max_out_chnum paramter.\n", adtsp->label);

            return ADTSPARSE_ERR_UNKNOWN;
        }
    }

    return NO_ERR;
}


tsp_loop_return adts_dsp_loop( tsp_loop_param data )
{
    int8_t            myerr = NO_ERR;

    volatile uint8_t* ctrl = NULL;

    adtsparse_t*      adtsp = NULL;
    uint32_t          sample_size     = 0;

    /* INTERNALS */
    uint8_t           fdk_f           = TRUE;
    uint32_t          fdk_valid       = 0;
    uint32_t          fdk_dec_cb_size = 0;

    uint32_t          local_size = 0;

    uint32_t          bytes_pushed = 0;


    /* retrieve the component base */
    adtsp = (adtsparse_t*) data;

    /* SANITY CHECKS */
    if ( adtsp == NULL )
    {
        LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][-][ERROR] dsp_loop, no adtsparse base component\n" );
        myerr = ADTSPARSE_ERR_UNKNOWN;
        TSP_THREAD_EXIT( );
        TSP_THREAD_RETURN_NULL( );
    }
    else
    {
        if ( 0 == match_adtsparse( NULL, adtsp, ADTSPARSE_CTYPE_ANYTYPE ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][-][ERROR] dsp_loop, not a valid adtsparse component\n" );
            myerr = ADTSPARSE_ERR_UNKNOWN;
            TSP_THREAD_EXIT( );
            TSP_THREAD_RETURN_NULL( );
        }
        else
        {
            /* retrieve the control register */
            ctrl =  (volatile uint8_t*) &adtsp->ctrl;
        }
    }

    /* component must be in READY state */
    if ( ADTSPARSE_CSTATUS_READY != adtsp->status )
    {
        myerr = ADTSPARSE_ERR_UNKNOWN;
        LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s][ERROR] dsp_loop, adtsparse NOT in ready state\n", adtsp->label );
    }

    /* redundant */
    if ( ctrl == NULL )
    {
        myerr = ADTSPARSE_ERR_UNKNOWN;
        LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s][ERROR] dsp_loop, no ctrl\n", adtsp->label );
    }

    /*
     * INNER LOOP here
     */
    if ( NO_ERR == myerr )
    {
        /* set the status flag */
        TSP_MUTEX_LOCK( NULL, adtsp->ctrl_mtx );
        *ctrl = ( *ctrl ) | ADTSPARSE_CTRL_DSP_STATUS;
        TSP_MUTEX_UNLOCK( NULL, adtsp->ctrl_mtx );

        if ( ( *ctrl ) & ADTSPARSE_CTRL_DSP_STATUS )
        {
            LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] dsp_loop (STARTING) \n", adtsp->label );
            do
            {
                /** DSP thread
                 *  wake up on new data input and parse the available data in the DSP queue
                 *  note: as fast as possible for free_run clock, or paste based on DSP input SYSCLK values
                 */
                uint8_t search_complete_f = ( ( adtsp->search_status & SEARCH_SYNCWORD_FOUND ) &&
                                              ( adtsp->search_status & SEARCH_PRIVATE_FOUND ) &&
                                              ( adtsp->search_status & SEARCH_CRC_FOUND ) );

                while ( ( ( *ctrl ) & ADTSPARSE_CTRL_DSP_STATUS ) &&
                        ( ( *ctrl ) & ADTSPARSE_CTRL_ONOFF_STATUS ) &&
                        ( ( adtsp->data_fifo[ adtsp->data_fifo_r_idx ].status == 0 ) ||
                          ( FALSE == search_complete_f ) ) )
                {
                    TSP_SEMAPHORE_WAIT( NULL, adtsp->adts_thread_sem );

                    search_complete_f = ( ( adtsp->search_status & SEARCH_SYNCWORD_FOUND ) &&
                                          ( adtsp->search_status & SEARCH_PRIVATE_FOUND ) &&
                                          ( adtsp->search_status & SEARCH_CRC_FOUND ) );
                }

                /*
                 * inline CMD filter
                 */

                /* exit on EOS */
                if ( ( AACDEC_BUFFER_TYPE_CMD_EOS == adtsp->data_fifo[ adtsp->data_fifo_r_idx & adtsp->data_fifo_wrap ].type ) ||
                     ( AACDEC_BUFFER_TYPE_MAX <= adtsp->data_fifo[ adtsp->data_fifo_r_idx & adtsp->data_fifo_wrap ].type ) )
                {
                    LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] end-of-stream packet cmd.\n",
                             adtsp->label );
                    adtsp->data_fifo[ adtsp->data_fifo_r_idx ].status = 0;
                    myerr = ADTSPARSE_ERR_END_OF_STREAM;
                    break;
                }

                /*
                 * INIT the aac decoder (once) with the audio specific info from the stream search
                 */
                if ( ( TRUE == fdk_f ) && ( NO_ERR == myerr ) )
                {
                    fdk_f = FALSE;

                    myerr = adts_dsp_init_fdk(adtsp, &fdk_dec_cb_size);
                }

                /* INNER decoding FDK loop:
                 * push all packets in the input queue to the decoder
                 */

                while ( ( NO_ERR == myerr ) &&
                        ( ( *ctrl ) & ADTSPARSE_CTRL_DSP_STATUS   ) &&
                        ( ( *ctrl ) & ADTSPARSE_CTRL_ONOFF_STATUS ) &&
                        ( adtsp->data_fifo[ adtsp->data_fifo_r_idx ].status != 0 ) )
                {
                    /*
                     * inline CMD filter
                     */

                    /* exit on EOS */
                    if ( ( AACDEC_BUFFER_TYPE_CMD_EOS == adtsp->data_fifo[ adtsp->data_fifo_r_idx & adtsp->data_fifo_wrap ].type ) ||
                         ( AACDEC_BUFFER_TYPE_MAX <= adtsp->data_fifo[ adtsp->data_fifo_r_idx & adtsp->data_fifo_wrap ].type ) )
                    {
                        LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] end-of-stream packet cmd.\n",
                                 adtsp->label );
                        adtsp->data_fifo[ adtsp->data_fifo_r_idx ].status = 0;
                        myerr = ADTSPARSE_ERR_END_OF_STREAM;

                        continue;
                    }

                    /*
                     *  Decoder : FILL
                     */
                    sample_size = adtsp->data_fifo[ adtsp->data_fifo_r_idx ].length;

                    uint32_t push_size = ( sample_size - local_size );

                    uint8_t* data_ptr  = &adtsp->data_raw[ adtsp->data_fifo[ adtsp->data_fifo_r_idx ].raw_idx + local_size ];

                    if ( push_size > 0 )
                    {
                        /* PROFILING/TraceX */
                        TPM_USRLOG( NULL, &adtsp->tpm,
                                    TPM_ADTS_DSP_FDK_FILL_BEGIN,
                                    adtsp->data_fifo_r_idx,
                                    local_size,
                                    sample_size,
                                    push_size );

                        fdk_valid = push_size;

                        adtsp->fdk_err = aacDecoder_Fill( adtsp->fdk_handle, &data_ptr, (const UINT*) &push_size, (UINT*) &fdk_valid );
                        if ( adtsp->fdk_err != AAC_DEC_OK )
                        {
                            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s] FDK-DEC: error, Fill failed: %x\n",
                                     adtsp->label,
                                     adtsp->fdk_err );

                            break;
                        }
#if 0
                        /* PROFILING/TraceX */
                        TPM_USRLOG( NULL, &adtsp->tpm,
                                    TPM_ADTS_DSP_FDK_FILL_END,
                                    chunk_bitstream_offset,
                                    push_size,
                                    fdk_valid,
                                    adtsp->fdk_err );
#endif
                        bytes_pushed += (push_size - fdk_valid);
                        local_size   += ( push_size - fdk_valid );

                        /* RELEASE & ADVANCE to the next buffer if no more data are available */
                        if ( local_size == sample_size )
                        {
                            adtsp->data_fifo[ adtsp->data_fifo_r_idx ].status = 0;
                            adtsp->data_fifo_r_idx = ( adtsp->data_fifo_r_idx + 1 ) & adtsp->data_fifo_wrap;
                            local_size = 0;
                        }
                    }

                    /*
                     *  Decoder : DECODE
                     */
                    uint32_t tmp32 = 0;

                    adtsp->fdk_err = aacDecoder_GetFreeBytes( adtsp->fdk_handle, (UINT*) &tmp32 );
                    if ( adtsp->fdk_err != AAC_DEC_OK )
                    {
                        LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s] [ERROR] dsp_loop: unable to get decoder buffer size.\n",
                                 adtsp->label );

                        myerr = ADTSPARSE_ERR_UNKNOWN;
                    }

                    while ( ( NO_ERR == myerr ) && ( tmp32 < fdk_dec_cb_size ) && ( adtsp->fdk_err == AAC_DEC_OK ) )
                    {
                        /* PROFILING/TraceX */
                        TPM_USRLOG( NULL, &adtsp->tpm,
                                    TPM_ADTS_DSP_FDK_DECODE_BEGIN,
                                    push_size,
                                    fdk_valid,
                                    0,
                                    0 );

                        adtsp->fdk_err = aacDecoder_DecodeFrame( adtsp->fdk_handle, adtsp->data_decode, BUF_8K, 0 );

                        /* PROFILING/TraceX */
                        TPM_USRLOG( NULL, &adtsp->tpm,
                                    TPM_ADTS_DSP_FDK_DECODE_END,
                                    push_size,
                                    fdk_valid,
                                    0,
                                    adtsp->fdk_err );

                        if ( ( adtsp->fdk_err == AAC_DEC_NOT_ENOUGH_BITS ) ||
                             ( adtsp->fdk_err == AAC_DEC_TRANSPORT_SYNC_ERROR ) )
                        {
                            LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] [ERROR] Decode failed: aac_not_enough_bits %x\n",
                                     adtsp->label,
                                     adtsp->fdk_err );

                            continue;
                        }
                        else if ( adtsp->fdk_err != AAC_DEC_OK )
                        {
                            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s] [ERROR] Decode failed: %x\n",
                                     adtsp->label,
                                     adtsp->fdk_err );

                            myerr = ADTSPARSE_ERR_UNKNOWN;

                            continue;
                        }

                        LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] DECODED OK!\n", adtsp->label );

                        /*
                         *  Decoder : PCM callback
                         */
                        if ( ( NO_ERR == myerr ) && ( NULL != adtsp->output_push_callback ) )
                        {
                            /* describe sample */
                            CStreamInfo* info = aacDecoder_GetStreamInfo( adtsp->fdk_handle );

                            if ( !info || ( info->sampleRate <= 0 ) )
                            {
                                LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s] DECODED : No stream info\n", adtsp->label );
                            }
                            else
                            {
                                /* always rebuild the pcm info because with AAC+v2 streams */
                                /* we might have SBR info later in the stream and so */
                                /* chnum and sr might change while playing the stream */
                                /* note: this is possible expecially with webradio streams */

                                /* our FDK only sends s16LE output */
                                adtsp->pcm_info.bps      = 16;
                                adtsp->pcm_info.cbps     = 16;
                                adtsp->pcm_info.chnum    = info->numChannels;
                                adtsp->pcm_info.sr       = info->sampleRate;
                                adtsp->pcm_info.chmap    = adts_chmap_lut[ adtsp->adts_hdr.fixed.channel_configuration ][ AACDEC_CHCFG_CHMAP ];
                                adtsp->pcm_info.endian   = 0; /* little */
                                adtsp->pcm_info.sign     = 1; /* signed */
                                adtsp->pcm_info.floating = 0; /* int */


                                /* PROFILING/TraceX */
                                TPM_USRLOG( NULL, &adtsp->tpm,
                                            TPM_ADTS_DSP_PCM_PUSH,
                                            adtsp->pcm_info.chnum,
                                            adtsp->pcm_info.sr,
                                            adtsp->pcm_info.chmap,
                                            adtsp->pcm_info.cbps );

                                adtsp->output_push_callback(
                                    &myerr,
                                    adtsp,
                                    adtsp->decoder,
                                    &adtsp->pcm_info,
                                    adtsp->data_decode,
                                    ( info->frameSize * info->numChannels ),
                                    NULL );
                            }
                        } /* pcm out */

                        adtsp->fdk_err = aacDecoder_GetFreeBytes( adtsp->fdk_handle, (UINT*) &tmp32 );
                        if ( adtsp->fdk_err != AAC_DEC_OK )
                        {
                            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s] [ERROR] dsp_loop: unable to get decoder buffer size.\n",
                                     adtsp->label );

                            myerr = ADTSPARSE_ERR_UNKNOWN;
                        }
                    } /* while data in dec_cb */
                }
            } while ( ( ( *ctrl ) & ADTSPARSE_CTRL_ONOFF_STATUS ) && ( NO_ERR == myerr ) );
        }
        else
        {
            myerr = ADTSPARSE_ERR_UNKNOWN;
            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s][ERROR] dsp_loop could not update the ctrl register.\n", adtsp->label );
        }
    }

    /*
     * IMPORTANT:
     * DSP thread is done, signal a "inline" pcm bufffer with ID=0 to signal that there is no more data
     */
    if ( NULL != adtsp->output_push_callback )
    {
        /* ID code for the in-band signaling */
        uint32_t id_done = 0;

        /* optional payload for the in-band signaling */
        int16_t  buf_payload = 0;

        if ( NO_ERR == myerr )
        {
            id_done = PCMBUF_ID_CMD_EOP_OK;

            adtsp->output_push_callback(
                &myerr,
                adtsp,
                adtsp->decoder,
                &adtsp->pcm_info,
                &buf_payload, // adtsp->data_decode,
                0,            /* payload size */
                &id_done );
        }
        else
        {
            id_done = PCMBUF_ID_CMD_EOP_ERR;

            /* tbd: form a proper payload
             * once the error format is defined */
            adtsp->output_push_callback(
                &myerr,
                adtsp,
                adtsp->decoder,
                &adtsp->pcm_info,
                &buf_payload,
                0, /* payload size */
                &id_done );
        }
    }

    /*
     * quit.
     */
    TSP_MUTEX_LOCK( NULL, adtsp->ctrl_mtx );
    *ctrl = ( *ctrl ) & ( ~( ADTSPARSE_CTRL_DSP_STATUS ) );
    TSP_MUTEX_UNLOCK( NULL, adtsp->ctrl_mtx );

    LOG_MSG( 0, LOG_ALWAYS, "[ADTSPARSE][%s] dsp_loop quit. (%" PRIu8 ") \n", adtsp->label, myerr );


    /* exit with zero */
    TSP_THREAD_EXIT( );
    TSP_THREAD_RETURN_NULL( );
}


/* ****************************************************************************** */
/* ****************************************************************************** */
/* PRIVATE                                                                        */
/* ****************************************************************************** */
/* ****************************************************************************** */

/*
 * Function: check_ready
 */
static inline int8_t check_ready_adtsparse( int8_t* err, adtsparse_t* adtsparse )
{
    int8_t myerr = 0;
    int8_t ret = TRUE;

    if ( NULL == adtsparse )
    {
        return FALSE;
    }

    /* place here all the sanity checks to declare the component "READY" */
    if ( ( TRUE == ret ) && ( ADTSPARSE_CSTATUS_CREATED != adtsparse->status ) )
    {
        ret = FALSE;
    }

    /* this MUST be the only place that can bump the status to "READY" */
    if ( TRUE == ret )
    {
        adtsparse->status = ADTSPARSE_CSTATUS_READY;
    }


    if ( NULL != err )
    {
        *err = myerr;
    }

    return ret;
}


/*
 * Function: match
 */
inline int8_t match_adtsparse( int8_t* err, adtsparse_t* adtsparse, adtsparse_ctype_t checktype )
{
    int8_t ret = FALSE;

    int8_t myerr = NO_ERR;

    if ( NULL == adtsparse )
    {
        if ( NULL != err )
        {
            *err = ADTSPARSE_ERR_UNKNOWN;
        }

        return ret;
    }

    if ( ( ADTSPARSE_CTYPE_RESERVED < adtsparse->type ) && \
         ( ADTSPARSE_CTYPE_MAX > adtsparse->type ) )
    {
        ret = ( ADTSPARSE_WMARK == adtsparse->wmark ) ? TRUE : FALSE;
    }

    if ( ( ret == TRUE ) && ( ADTSPARSE_CTYPE_ANYTYPE != checktype ) )
    {
        ret = ( checktype == adtsparse->type ) ? TRUE : FALSE;
    }

    if ( NULL != err )
    {
        *err = myerr;
    }

    return ret;
}


/*
 * Function:
 *
 */
static void init_adtsparse( int8_t* err, adtsparse_t* adtsparse )
{
    int8_t       myerr = NO_ERR;

    adtsparse_t* adtsp = adtsparse;

    if ( NULL != adtsp )
    {
        LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][-] init_adtsparse\n" );

        /* init random seed */
        // srand((uint32_t)time(NULL));

        /* component base */
        adtsp->id    = 1; // should be random();
        adtsp->wmark = ADTSPARSE_WMARK;
        adtsp->type  = ADTSPARSE_CTYPE_UNKNOWN;

        /* We only support one mode: single threaded */
        adtsp->type = ADTSPARSE_CTYPE_SINGLE_THREAD;

        /* versioning stuff: the default is "0.0.0.none" */
        adtsp->major = 0;
        adtsp->minor = 0;
        strncpy( adtsp->revision, "0000000000000000", ( 16 + 1 ) ); /* revision is 32 char max */
        strncpy( adtsp->build,    "none",             ( 4 + 1 ) );  /* build is 5 char max */

        /* override versioning by build machine at compilation time */
#if ( defined( VERSION_MAJOR ) && defined( VERSION_MINOR ) && defined( VERSION_REVISION ) && defined( VERSION_TYPE ) )
        adtsp->major = VERSION_MAJOR;
        adtsp->minor = VERSION_MINOR;
        strncpy( adtsp->revision, VERSION_REVISION, 16 );
        strncpy( adtsp->build,    VERSION_TYPE,     4 );
#endif /* versioning override */

        /* CTRL register is internal */
        adtsp->ctrl = 0;
        strncpy( adtsp->label, "adtsp_cmp\0", 14 );

        /* in/out threads setup */
        TSP_MUTEX_INIT( myerr, adtsp->ctrl_mtx );

        /* adtsparse */
        TSP_SEMAPHORE_INIT( myerr, adtsp->adts_thread_sem );

        /* input buffer fifo init */
        adtsp->data_fifo_wrap  = BUF_NUM - 1;
        adtsp->data_fifo_w_idx = 0;
        adtsp->data_fifo_r_idx = 0;

        /* data raw fifo */
        adtsp->data_raw_w_idx  = 0;
        adtsp->data_raw_wrap   = BUF_RAW_SIZE - 1;

        /* bitstream indexing */
        adtsp->data_bitstream_idx    = 0;
        adtsp->data_bitstream_skip_f = FALSE;

        /* to allow skip bitstream */
        adtsp->data_size_prev = 0;

        /* search variables */
        adtsp->search_idx     = 0;
        adtsp->search_status  = SEARCH_NONE_FOUND;
        adtsp->adts_hdr.is_valid = FALSE;

        /* audio track */
        adtsp->adts_audio_track_info.creation_time     = 0;
        adtsp->adts_audio_track_info.modification_time = 0;
        adtsp->adts_audio_track_info.timescale         = 0;
        adtsp->adts_audio_track_info.duration          = 0;

        /* internals */
        adtsp->tpm.tool_type = TPM_TOOL_UNKNOWN;

        /* OUTPUT callbacks*/

        /* INPUT callbacks*/

        /* ALERT callbacks */

        /* external app ptr */

        /* update status */
        adtsp->status = ADTSPARSE_CSTATUS_CREATED;

        LOG_MSG( adtsp->log_lvl, LOG_MEDIUM, "[ADTSPARSE][%s] init_adtsparse: done.\n", adtsparse->label );
    }
    else
    {
        myerr = ADTSPARSE_ERR_UNKNOWN;
    }


    if ( NULL != err )
    {
        *err = myerr;
    }
}


static void destroy_adtsparse( int8_t* err, adtsparse_t** adtsparse )
{
    int8_t            myerr = NO_ERR;

    adtsparse_t*      adtsp = NULL;

    volatile uint8_t* ctrl = NULL;

    /*
     * SANITY CHECKS
     */

#if defined( DEBUG )
    /* sanity check on the pointers */
    if ( ( NULL == adtsparse ) || ( NULL == *adtsparse ) )
    {
        if ( NULL != err )
        {
            *err = ADTSPARSE_ERR_UNKNOWN;
        }
        return;
    }
#endif

    if ( NO_ERR == myerr )
    {
        adtsp = *adtsparse;

        ctrl = &adtsp->ctrl;

        /* mute the output push */
        adtsp->output_push_callback = NULL;

        /* DSP: trigger the thread until is gone */
        while ( ( *ctrl ) & ADTSPARSE_CTRL_DSP_STATUS )
        {
            TSP_SEMAPHORE_POST( NULL, adtsp->adts_thread_sem );
            SYSCLK_USLEEP( uSLEEP10ms );
        }
        TSP_THREAD_JOIN( NULL, adtsp->adts_thread );
        TSP_THREAD_DELETE( NULL, adtsp->adts_thread );
        TSP_SEMAPHORE_DESTROY( NULL, adtsp->adts_thread_sem );

        /* deallocate raw fifo */
        if ( NULL != adtsp->data_raw )
        {
            free( adtsp->data_raw );
            adtsp->data_raw = NULL;
        }

        /* deallocate buffer fifo */
        if ( NULL != adtsp->data_fifo )
        {
            free( adtsp->data_fifo );
            adtsp->data_fifo = NULL;
        }

#if ( ENABLE_FDK_WORKAROUND )
        if ( adtsp->fdk_handle )
        {
            /* BEGIN - WORKAROUND: FDK library */
            /* make a call decoder_Fill to detect the OUT_OF_MEMORY error */
            /* which internally will _Close() the aac decoder already */
            uint32_t  tmpBytes      = 0xFFFF;
            uint32_t* tmpBytesPtr   = &tmpBytes;
            uint32_t  tmpBytesValid = 0;
            uint32_t  tmpByteSize   = sizeof( uint32_t );

            adtsp->fdk_err = aacDecoder_Fill( adtsp->fdk_handle, (unsigned char**) &tmpBytesPtr,
                                              (const UINT*) &tmpByteSize, (UINT*) &tmpBytesValid );

            if ( AAC_DEC_OK != adtsp->fdk_err )
            {
                adtsp->fdk_handle = NULL;
            }
            /* END - WORKAROUND */
        }
#endif

        if ( adtsp->fdk_handle )
        {
            aacDecoder_Close( adtsp->fdk_handle );
        }

        if ( NULL != adtsp->data_decode )
        {
            free( adtsp->data_decode );
            adtsp->data_decode = NULL;
        }

        /* finally remove the component */
        if ( NULL != adtsp )
        {
            TSP_MUTEX_DESTROY( NULL, adtsp->ctrl_mtx );

            free( adtsp );
            *adtsparse = NULL;
        }

        LOG_MSG( 0, LOG_ALWAYS, "[ADTSPARSE][-] adtsparse_destroy. done. \n" );
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


/* ****************************************************************************** */
/* ****************************************************************************** */
/* PUBLIC                                                                         */
/* ****************************************************************************** */
/* ****************************************************************************** */



/* ************************ */
/* SYSTEM                   */
/* ************************ */

void adtsparse_new( int8_t* err,
                    void**  adtsparse,
                    void*   decoder,
                    uint8_t logging_level )
{
    int8_t       myerr = NO_ERR;

    adtsparse_t* adtsp = NULL;

    /*
     * SANITY CHECKS
     */

    /* sanity check on the pointers */
    if ( ( NULL == adtsparse ) )
    {
        myerr = ADTSPARSE_ERR_UNKNOWN;
    }

    /* check for previous errors */
    if ( NULL != err )
    {
        if ( NO_ERR != ( *err ) )
        {
            return;
        }
    }

    if ( NO_ERR == myerr )
    {
        if ( NULL != ( *adtsparse ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][-] [ERROR] adtsparse_new, input adtsparse ptr not NULL, FAIL\n" );
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
    }

    /*
     * MAIN
     */

    /* allocate component base */
    if ( NO_ERR == myerr )
    {
        if ( NULL == ( adtsp = (adtsparse_t*) calloc( 1, sizeof( adtsparse_t ) ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][-] [ERROR] adtsparse_new, adtsparse_t create FAIL\n" );
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
        else
        {
            adtsp->memory_footprint = sizeof( adtsparse_t );
        }
    }

    /* allocate input buffer queue */
    if ( NO_ERR == myerr )
    {
        if ( NULL == ( adtsp->data_fifo = (data_buf_t*) calloc( BUF_NUM, sizeof( data_buf_t ) ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][-] [ERROR] adtsparse_new, input buffer fifo create FAIL\n" );
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
        else
        {
            adtsp->memory_footprint += sizeof( data_buf_t ) * BUF_NUM;
        }
    }

    /* allocate input raw queue */
    if ( NO_ERR == myerr )
    {
        if ( NULL == ( adtsp->data_raw = (uint8_t*) calloc( BUF_RAW_SIZE, sizeof( uint8_t ) ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][-] [ERROR] adtsparse_new, input raw fifo create FAIL\n" );
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
        else
        {
            adtsp->memory_footprint += sizeof( uint8_t ) * BUF_RAW_SIZE;
        }
    }

    /* allocate decode */
    if ( NO_ERR == myerr )
    {
        if ( NULL == ( adtsp->data_decode = (int16_t*) calloc( BUF_8K / 2, sizeof( int16_t ) ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][-] [ERROR] adtsparse_new, decode buf create FAIL\n" );
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
        else
        {
            adtsp->memory_footprint += sizeof( uint16_t ) * BUF_8K / 2;
        }
    }

    /* set default values */
    if ( NO_ERR == myerr )
    {
        LOG_MSG( 0, LOG_ALWAYS, "[ADTSPARSE][-] adtsparse_new, adtsparse_t size=%" PRIu32 "\n", (uint32_t) sizeof( adtsparse_t ) );

        /* PRE-INIT */
        adtsp->id    = 1;
        adtsp->wmark = ADTSPARSE_WMARK;
        adtsp->type  = ADTSPARSE_CTYPE_UNKNOWN;

        adtsp->decoder    = decoder;
        adtsp->fdk_handle = NULL;

        adtsp->output_push_callback = NULL;

        /* logging */
        adtsp->log_lvl = LOG_MEDIUM;
        adtsp->log_err = LOG_ALWAYS;

        /* params/config values for sub-components  */
        adtsp->pcm_info_tgt_chmap = ( (aacdec_t*) decoder )->pcm_info_tgt_chmap;
    }


    if ( NO_ERR == myerr )
    {
        /* force the type */
        adtsp->type = ADTSPARSE_CTYPE_RESERVED;

        /* force the status*/
        adtsp->status = ADTSPARSE_CSTATUS_UNKNOWN;

        /* init the component */
        init_adtsparse( &myerr, adtsp );

        /* set the initial log level*/
        adtsparse_set_log_verbosity( NULL, adtsp, logging_level );

        LOG_MSG( adtsp->log_lvl, LOG_MEDIUM, "[ADTSPARSE][%s] adtsparse_new\n", adtsp->label );
    }


    /*
     * POST-INIT: open FDK-AAC decoder
     */
    if ( NO_ERR == myerr )
    {
        adtsp->fdk_handle = aacDecoder_Open( TT_MP4_ADTS, 1 );
        if (adtsp->fdk_handle == NULL)
        {
            myerr = ADTSPARSE_ERR_OUT_OF_MEMORY;
        }
    }

    if ( NO_ERR == myerr )
    {
        LOG_MSG( 0, LOG_ALWAYS, "[ADTSPARSE][%s] adtsparse_new, SUCCESS\n",            adtsp->label );
        LOG_MSG( 0, LOG_ALWAYS, "[ADTSPARSE][%s] adtsparse_new, memory=%" PRIu64 "\n", adtsp->label, adtsp->memory_footprint );

        /* return cmp */
        *adtsparse = (void*) adtsp;
    }
    else
    {
        LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][-] [ERROR] adtsparse_new, FAIL\n" );

        destroy_adtsparse( NULL, &adtsp );

        *adtsparse = NULL;
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


void adtsparse_destroy( int8_t* err,
                        void**  adtsparse )
{
    int8_t            myerr = NO_ERR;

    adtsparse_t*      adtsp = NULL;

    volatile uint8_t* ctrl = NULL;

    /*
     * SANITY CHECKS
     */

    /* sanity check on the pointers */
    if ( ( NULL == adtsparse ) || ( NULL == *adtsparse ) )
    {
        myerr = ADTSPARSE_ERR_UNKNOWN;
        if ( NULL != err )
        {
            *err = -1;
        }
        return;
    }

    adtsp = (adtsparse_t*) *adtsparse;


    if ( NO_ERR == myerr )
    {
        if ( 0 == match_adtsparse( NULL, adtsp, ADTSPARSE_CTYPE_ANYTYPE ) )
        {
            myerr = ADTSPARSE_ERR_UNKNOWN;
            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][-] [ERROR] adtsparse_destroy, adtsparse invalid.\n" );
        }
        else
        {
            /* retrieve the control register */
            ctrl =  (volatile uint8_t*) &adtsp->ctrl;
        }

        if ( ctrl == NULL )
        {
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
    }

    /* requirement: to allow destroy we need to be ready for it */
    if ( NO_ERR == myerr )
    {
        if ( ( adtsp->status != ADTSPARSE_CSTATUS_CREATED ) &&
             ( adtsp->status != ADTSPARSE_CSTATUS_STOPPED ) &&
             ( adtsp->status != ADTSPARSE_CSTATUS_FLUSHED ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s] [ERROR] adtsparse_destroy, not in a STOPPED/FLUSHED state.  \n", adtsp->label );
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
    }

    if ( NO_ERR == myerr )
    {
        LOG_MSG( 0, LOG_ALWAYS, "[ADTSPARSE][%s] adtsparse_destroy \n", adtsp->label );

        /* update the status */
        adtsp->status = ADTSPARSE_CSTATUS_STOPPED;

        /* signal to the ctrl register */
        TSP_MUTEX_LOCK( NULL, adtsp->ctrl_mtx );
        *ctrl = ( *ctrl ) & ( ~( ADTSPARSE_CTRL_ONOFF_STATUS ) );
        TSP_MUTEX_UNLOCK( NULL, adtsp->ctrl_mtx );

        /* force the exit */
        destroy_adtsparse( &myerr, (adtsparse_t**) adtsparse );

        /* erase the reference! */
        *adtsparse = NULL;
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


void adtsparse_set_log_verbosity( int8_t* err, void* adtsparse, uint8_t lvl )
{
    int8_t       myerr = NO_ERR;

    adtsparse_t* adtsp = (adtsparse_t*) adtsparse;

    /* sanity check */
    if ( NULL == adtsp )
    {
        myerr = ADTSPARSE_ERR_UNKNOWN;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_adtsparse( NULL, adtsp, ADTSPARSE_CTYPE_ANYTYPE ) )
        {
            myerr = ADTSPARSE_ERR_UNKNOWN;
            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][-] [ERROR] adtsparse_set_log_verbosity, adtsparse invalid.\n" );
        }
    }

    if ( NO_ERR == myerr )
    {
        adtsp->log_lvl = lvl;
        adtsp->log_err = LOG_ALWAYS;

        LOG_MSG( adtsp->log_lvl, LOG_MEDIUM, "[ADTSPARSE][%s] adtsparse_set_log_verbosity: %" PRIu8 "\n", adtsp->label, lvl );
    }

    if ( NULL != err )
    {
        *err = NO_ERR;
    }
}


void adtsparse_start( int8_t* err,
                      void*   adtsparse )
{
    int8_t            myerr = NO_ERR;

    adtsparse_t*      adtsp = (adtsparse_t*) adtsparse;

    volatile uint8_t* ctrl = NULL;

    uint8_t           tmp8 = 0;
    uint32_t          tmp32 = 0;

    /* sanity check on the pointers */
    if ( ( NULL == adtsparse ) )
    {
        myerr = ADTSPARSE_ERR_UNKNOWN;
    }

    /* check for previous errors */
    if ( NULL != err )
    {
        if ( NO_ERR != ( *err ) )
        {
            return;
        }
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_adtsparse( NULL, adtsp, ADTSPARSE_CTYPE_ANYTYPE ) )
        {
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
        else
        {
            /* retrieve the control register */
            ctrl =  (volatile uint8_t*) &adtsp->ctrl;
        }

        if ( ctrl == NULL )
        {
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
    }

    if ( NO_ERR == myerr )
    {
        /* make sure we have all we need*/
        check_ready_adtsparse( &myerr, adtsp );
    }
    else
    {
        if ( NULL != err )
        {
            *err = -1;
        }
        return;
    }

    /* SET the on/off CTRL register flag                                  */
    /* internal threads will exit immediately if you do not set this flag */
    if ( NO_ERR == myerr )
    {
        TSP_MUTEX_LOCK( NULL, adtsp->ctrl_mtx );
        *ctrl = ( *ctrl ) | ADTSPARSE_CTRL_ONOFF_STATUS;
        TSP_MUTEX_UNLOCK( NULL, adtsp->ctrl_mtx );
    }

    /*
     * now start the component (inner thread)
     */

    /* START audio thread: DSP */
    if ( NO_ERR == myerr )
    {
        TSP_THREAD_CREATE( myerr,
                           adtsp->adts_thread,
                           adtsp->adts_thread_attr,
                           adts_dsp_loop,
                           adtsp,
                           ADTSPARSE_DEFAULT_PRIORITY,
                           ADTSPARSE_DEFAULT_STACK_SIZE,
                           ADTSPARSE_DEFAULT_LABEL );

        if ( myerr != NO_ERR )
        {
            LOG_ERR( 0, LOG_ALWAYS,
                     "[ADTSPARSE][%s] [ERROR] cannot create ADTSPARSE thread\n", adtsp->label );
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
    }
    else
    {
        LOG_ERR( 0, LOG_ALWAYS,
                 "[ADTSPARSE][%s] [ERROR] skipping create ADTSPARSE thread\n", adtsp->label );
    }


    /* control loop for subsystem healty status */
    if ( myerr == NO_ERR )
    {
        if ( ( *ctrl ) & ADTSPARSE_CTRL_ONOFF_STATUS )
        {
            /*
             * wait until the subsystems are ready
             */
            tmp32 = 0;

            do
            {
                tmp8 = ( adtsp->ctrl & ADTSPARSE_CTRL_DSP_STATUS );

                if ( 0 == ( tmp32 & 0x0FF ) )
                {
                    LOG_MSG( adtsp->log_lvl, LOG_LOW,
                             "[ADTSPARSE][%s] waiting for sub-components CTRL=%" PRIu8 "\n", adtsp->label, adtsp->ctrl );
                }

                tmp32++;

                SYSCLK_USLEEP( uSLEEP1ms );
            } while ( !( tmp8 ) && ( ( *ctrl ) & ADTSPARSE_CTRL_ONOFF_STATUS ) && ( tmp32 < ADTSPARSE_TIMEOUT_MS ) );


            /* flag an error if we did not start in TIMEOUT ms */
            if ( !tmp8 )
            {
                LOG_ERR( 0, LOG_ALWAYS,
                         "[ADTSPARSE][%s] [ERROR] adtsparse_start, could not start,CTRL=%" PRIu8 "\n", adtsp->label, adtsp->ctrl );

                myerr = ADTSPARSE_ERR_UNKNOWN;
            }
            else
            {
                LOG_MSG( adtsp->log_lvl, LOG_LOW,
                         "[ADTSPARSE][%s] adtsparse_running, CTRL=%" PRIu8 "\n", adtsp->label, adtsp->ctrl );

                /* ready update status */
                adtsp->status = ADTSPARSE_CSTATUS_RUNNED;
            }
        }
    }

    /* auto shutoff on failure */
    if ( myerr != NO_ERR )
    {
        /* signal to the ctrl register */
        TSP_MUTEX_LOCK( NULL, adtsp->ctrl_mtx );
        *ctrl = ( *ctrl ) & ( ~( ADTSPARSE_CTRL_ONOFF_STATUS ) );
        TSP_MUTEX_UNLOCK( NULL, adtsp->ctrl_mtx );
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


void adtsparse_stop( int8_t* err,
                     void*   adtsparse )
{
    int8_t       myerr = NO_ERR;

    adtsparse_t* adtsp = (adtsparse_t*) adtsparse;


    /* sanity check on the pointers */
    if ( ( NULL == adtsparse ) )
    {
        myerr = ADTSPARSE_ERR_UNKNOWN;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_adtsparse( NULL, adtsp, ADTSPARSE_CTYPE_ANYTYPE ) )
        {
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
    }

    if ( NO_ERR == myerr )
    {
        if ( ADTSPARSE_CSTATUS_READY > adtsp->status )
        {
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
    }

    if ( NO_ERR == myerr )
    {
        LOG_MSG( adtsp->log_lvl, LOG_MEDIUM, "[ADTSPARSE][%s] adtsparse_stop.\n", adtsp->label );
    }

    /*
     * MAIN
     */
    if ( NO_ERR == myerr )
    {
        if ( ADTSPARSE_CSTATUS_STOPPED != adtsp->status )
        {
            LOG_MSG( 0, LOG_ALWAYS,
                     "[ADTSPARSE][%s] adtsparse STOPPED.\n", adtsp->label );

            adtsp->status = ADTSPARSE_CSTATUS_STOPPED;
        }
    }
    else
    {
        if ( NULL == adtsp )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s] [ERROR] adtsparse is NULL\n", "-" );
        }
        else
        {
            LOG_ERR( 0, LOG_ALWAYS,
                     "[ADTSPARSE][%s] [ERROR] adtsparse not ready for STOP (status=%" PRIu16 ")\n", adtsp->label, adtsp->status );
        }
    }


    if ( NULL != err )
    {
        *err = NO_ERR;
    }
}


void adtsparse_flush( int8_t* err,
                      void*   adtsparse )
{
    int8_t            myerr = NO_ERR;

    volatile uint8_t* ctrl = NULL;

    adtsparse_t*      adtsp = (adtsparse_t*) adtsparse;


    /* sanity check on the pointers */
    if ( ( NULL == adtsparse ) )
    {
        myerr = ADTSPARSE_ERR_UNKNOWN;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_adtsparse( NULL, adtsp, ADTSPARSE_CTYPE_ANYTYPE ) )
        {
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
        else
        {
            /* retrieve the control register */
            ctrl =  (volatile uint8_t*) &adtsp->ctrl;
        }
    }

    if ( NO_ERR == myerr )
    {
        if ( ADTSPARSE_CSTATUS_STOPPED != adtsp->status )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s][ERROR] adtsparse_flush, not in a STOPPED state\n", adtsp->label );
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
    }

    if ( NO_ERR == myerr && ctrl == NULL )
    {
        myerr = ADTSPARSE_ERR_UNKNOWN;
        LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s][ERROR] adtsparse_flush, no ctrl\n", adtsp->label );
    }

    /*
     * MAIN
     */
    if ( NO_ERR == myerr )
    {
        LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] adtsparse_flush, FLUSHING...\n", adtsp->label );

        while ( ( NO_ERR == myerr ) &&
                ( ( *ctrl ) & ADTSPARSE_CTRL_ONOFF_STATUS ) &&
                ( ( *ctrl ) & ADTSPARSE_CTRL_DSP_STATUS ) &&
                ( adtsp->data_fifo[ adtsp->data_fifo_r_idx ].status ) )
        {
            /* trigger DSP parser for decoding */
            TSP_SEMAPHORE_POST( NULL, adtsp->adts_thread_sem );
            SYSCLK_USLEEP( uSLEEP10ms );
        }

        LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] adtsparse_flush, FLUSHING... done.\n", adtsp->label );

        adtsp->status = ADTSPARSE_CSTATUS_FLUSHED;
    }
    else
    {
        if ( NULL == adtsp )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s] [ERROR] adtsparse_flush adtsparse is NULL\n", "-" );
        }
        else
        {
            LOG_ERR( 0, LOG_ALWAYS,
                     "[ADTSPARSE][%s] [ERROR] adtsparse not ready for FLUSH (status=%" PRIu16 ")\n", adtsp->label, adtsp->status );
        }
    }


    if ( NULL != err )
    {
        *err = NO_ERR;
    }
}


/* ***************************** */
/* SETTERS                       */
/* ***************************** */


void adtsparse_set_output_push_callback( int8_t*              err,
                                         void*                adtsparse,
                                         uint32_t ( *         output_push_callback )(
                                             int8_t*          err,
                                             void*            adtsparse,
                                             void*            decoder,
                                             adts_pcm_info_t* pcm_info,
                                             int16_t*         buf,
                                             uint32_t         buf_len,
                                             uint32_t*        buf_id ) )
{
    int8_t       myerr = NO_ERR;
    adtsparse_t* adtsp = (adtsparse_t*) adtsparse;

    /* sanity check */
    if ( NULL == adtsp )
    {
        myerr = ADTSPARSE_ERR_UNKNOWN;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_adtsparse( NULL, adtsp, ADTSPARSE_CTYPE_ANYTYPE ) )
        {
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
    }

    if ( ( NO_ERR == myerr ) && ( NULL != adtsp ) )
    {
        if ( NULL == adtsp->output_push_callback )
        {
            adtsp->output_push_callback = output_push_callback;
        }
        else
        {
            LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s] set_output_push_CBACK: failed\n", adtsp->label );
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


uint32_t adtsparse_input_push_buf( int8_t*  err,
                                   void*    adtsparse,
                                   char*    buf,
                                   uint32_t buf_len,
                                   uint8_t  buf_type )
{
    int8_t            myerr = NO_ERR;

    adtsparse_t*      adtsp = NULL;

    volatile uint8_t* ctrl = NULL;

    uint32_t          push_byte_cnt = 0;

    uint8_t           search_complete_f = FALSE;


    /* sanity check */
    if ( NULL == adtsparse )
    {
        if ( NULL != err )
        {
            *err = ADTSPARSE_ERR_UNKNOWN;
        }
        /* bail out on invalid component */
        return 0;
    }


    /* retrieve component base obj */
    adtsp = (adtsparse_t*) adtsparse;

    /* retrieve the control register */
    ctrl =  (volatile uint8_t*) &adtsp->ctrl;

    /*
     * this check could be too CPU heavy for production since it does happen
     * for each pkt triggering. Enabled only in DBG build.
     */
#if defined( DEBUG )
    if ( NO_ERR == myerr )
    {
        if ( 0 == match_adtsparse( NULL, adtsp, ADTSPARSE_CTYPE_ANYTYPE ) )
        {
            myerr = ADTSPARSE_ERR_UNKNOWN;
            goto _exit;
        }
    }
#endif /* DEBUG */

    if ( NO_ERR == myerr )
    {
        /* check for running component */
        if ( ( ADTSPARSE_CSTATUS_RUNNED != adtsp->status ) ||
             ( !( ( *ctrl ) & ADTSPARSE_CTRL_ONOFF_STATUS ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS,
                     "[ADTSPARSE][%s] [ERROR] input_push: adtsparse is NOT running (CTRL=%" PRIu8 ")\n", adtsp->label, adtsp->ctrl );
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
    }

    /* sanity check:
     * DSP must be alive
     */
    if ( !( ( *ctrl ) & ADTSPARSE_CTRL_DSP_STATUS ) )
    {
        LOG_ERR( 0, LOG_ALWAYS,
                 "[ADTSPARSE][%s] [ERROR] input_push: adtsparse DSP anomalous exit (CTRL=%" PRIu8 ")\n", adtsp->label, adtsp->ctrl );

        myerr = ADTSPARSE_ERR_UNKNOWN;
    }

    /* sanity check:
     * buffer must be valid in size
     */
    if ( ( NULL == buf ) || ( buf_len == 0 ) || ( buf_len > RTP_PKT_MTU_SIZE ) )
    {
        LOG_ERR( 0, LOG_ALWAYS,
                 "[ADTSPARSE][%s] [ERROR] input_push_buf: invalid buffer\n", adtsp->label );
        myerr = ADTSPARSE_ERR_UNKNOWN;
    }

    /* bail out if needed */
    if ( NO_ERR != myerr )
    {
        goto _exit;
    }


    /*
     * MAIN: add pkt into the input queue
     */
    if ( NO_ERR == myerr )
    {
        search_complete_f = ( ( adtsp->search_status & SEARCH_SYNCWORD_FOUND ) &&
                              ( adtsp->search_status & SEARCH_PRIVATE_FOUND ) &&
                              ( adtsp->search_status & SEARCH_CRC_FOUND ) );

        if ( !search_complete_f )
        {
            if ( ( BUF_NUM <= ( adtsp->data_fifo_w_idx + 1 ) ) ||
                 ( BUF_RAW_SIZE <= ( adtsp->data_raw_w_idx + buf_len ) ) )
            {
                LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s] ADTS SEARCH fifo overflow error, %" PRIu32 ", %" PRIu32 "\n",
                         adtsp->label,
                         adtsp->data_fifo_w_idx,
                         adtsp->data_fifo[ adtsp->data_fifo_w_idx ].raw_idx );

                /* no more space in the queue for sync search */
                myerr = ADTSPARSE_ERR_INPUT_QUEUE_OVERFLOW;
                goto _exit;
            }


            /*
             * skip incrementing if we need to discard data
             */
            if ( adtsp->data_bitstream_skip_f == TRUE )
            {
                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] ==>SAME SLOT! data_raw_w_idx=%" PRIu32 " data_size_prev=%" PRIu64 "\n",
                         adtsp->label,
                         adtsp->data_raw_w_idx,
                         adtsp->data_size_prev );

                adtsp->data_raw_w_idx -= adtsp->data_size_prev;
            }

            /*
             * getBuffer
             */
            LOG_MSG( adtsp->log_lvl, LOG_HIGH, "[ADTSPARSE][%s] slot #%" PRIu32 "\n", adtsp->label, adtsp->data_fifo_w_idx );

            adtsp->data_fifo[ adtsp->data_fifo_w_idx ].length        = buf_len;
            adtsp->data_fifo[ adtsp->data_fifo_w_idx ].raw_idx       = adtsp->data_raw_w_idx;
            adtsp->data_fifo[ adtsp->data_fifo_w_idx ].raw           = &adtsp->data_raw[ adtsp->data_raw_w_idx ];
            adtsp->data_fifo[ adtsp->data_fifo_w_idx ].bitstream_idx = adtsp->data_bitstream_idx;
            adtsp->data_fifo[ adtsp->data_fifo_w_idx ].type          = buf_type;

            /*
             * copy
             */
            LOG_MSG( 0, LOG_LOW, "[ADTSPARSE][%s] input_push_buf (s) data.idx=%" PRIu32 ", raw.idx=%" PRIu32 ", size=%" PRIu32 "\n",
                     adtsp->label,
                     adtsp->data_fifo_w_idx,
                     adtsp->data_raw_w_idx,
                     buf_len );

            memcpy( adtsp->data_fifo[ adtsp->data_fifo_w_idx ].raw, buf, buf_len );

            /* mark the slot only AFTER being populated */
            adtsp->data_fifo[ adtsp->data_fifo_w_idx ].status  = 1;


            push_byte_cnt = buf_len;

            /* PROFILING/TraceX */
            TPM_USRLOG( NULL, &adtsp->tpm,
                        TPM_ADTS_INPUT_PARSE_PUSH,
                        adtsp->data_fifo_w_idx,
                        adtsp->data_fifo_r_idx,
                        adtsp->data_raw_w_idx,
                        buf_len );

            /*
             * grow raw data fifo
             */
            adtsp->data_bitstream_idx += buf_len;

            adtsp->data_raw_w_idx += buf_len;

            /* keep track of the previous push */
            adtsp->data_size_prev = buf_len;


            /*
             * initial search: find or store in local data fifo
             */

            adts_find_frame( &myerr, adtsp );


            /* move index */
            if ( adtsp->data_bitstream_skip_f == FALSE )
            {
                adtsp->data_fifo_w_idx = ( adtsp->data_fifo_w_idx + 1 ) & adtsp->data_fifo_wrap;
            }
            else
            {
                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] ==>SAME SLOT!\n", adtsp->label );
            }

            /* validate search AFTER each push and reset fifo if needed */
            search_complete_f = ( ( adtsp->search_status & SEARCH_SYNCWORD_FOUND ) &&
                                  ( adtsp->search_status & SEARCH_PRIVATE_FOUND ) &&
                                  ( adtsp->search_status & SEARCH_CRC_FOUND ) );

            if ( search_complete_f )
            {
                LOG_MSG( adtsp->log_lvl, LOG_LOW, "[ADTSPARSE][%s] SEARCH COMPLETE, reset fifo\n", adtsp->label );

                /* keep last slot */
                uint32_t i_curr = ( adtsp->data_fifo_w_idx - 1 ) & adtsp->data_fifo_wrap;

                uint32_t i;
                for ( i = 0; i < BUF_NUM; i++ )
                {
                    if ( i != i_curr )
                    {
                        adtsp->data_fifo[ i ].status  = 0;
                        adtsp->data_fifo[ i ].length  = 0;
                        adtsp->data_fifo[ i ].raw_idx = 0;
                        adtsp->data_fifo[ i ].raw     = NULL;
                    }
                }

                adtsp->data_fifo_r_idx = i_curr;
            }
        }
        else
        {
            /*
             * sanity check on descriptor fifo
             */
            if ( adtsp->data_fifo[ ( adtsp->data_fifo_w_idx + 0 ) & adtsp->data_fifo_wrap ].status != 0 )
            {
                LOG_ERR( 0, LOG_ALWAYS, "[ADTSPARSE][%s] [ERROR]] overflow on INPUT FIFO, w_idx=%" PRIu32 " (status=%" PRIu8 ")\n",
                         adtsp->label,
                         adtsp->data_fifo_w_idx,
                         adtsp->data_fifo[ adtsp->data_fifo_w_idx ].status );

                myerr = ADTSPARSE_ERR_INPUT_QUEUE_OVERFLOW;
                // myerr = ADTSPARSE_ERR_UNKNOWN;

                /* trigger DSP parser for decoding */
                TSP_SEMAPHORE_POST( NULL, adtsp->adts_thread_sem );
            }

            /*
             * sanity check on raw fifo
             */
            {
                // uint32_t w_fifo_idx = (adtsp->data_fifo_w_idx-1) & adtsp->data_fifo_wrap;

                uint32_t GCC_UNUSED w_raw_idx = adtsp->data_fifo[ ( adtsp->data_fifo_w_idx - 1 ) & adtsp->data_fifo_wrap ].raw_idx;
                uint32_t            r_raw_idx = adtsp->data_fifo[ adtsp->data_fifo_r_idx ].raw_idx;

                /* check  written data */
                if ( ( adtsp->data_fifo[ ( adtsp->data_fifo_w_idx - 1 ) & adtsp->data_fifo_wrap ].status ) &&
                     ( adtsp->data_fifo[ adtsp->data_fifo_r_idx ].status ) )
                {
                    if ( buf_len > ( ( r_raw_idx - adtsp->data_raw_w_idx + BUF_RAW_SIZE ) & adtsp->data_raw_wrap ) )
                    {
                        LOG_ERR( adtsp->log_lvl, LOG_MEDIUM, "[ADTSPARSE][%s] [ERROR] BUF_CHECK overflow on INPUT RAW FIFO, "
                                                             "w_raw_idx=%" PRIu32 " (prev %" PRIu32 ") r_raw_idx=%" PRIu32 " diff=%" PRIu32 "->%" PRIu32 "->%" PRIu32 ") wrap=%" PRIu32 "\n",
                                 adtsp->label,
                                 adtsp->data_raw_w_idx,
                                 w_raw_idx,
                                 r_raw_idx,
                                 r_raw_idx - w_raw_idx,
                                 ( r_raw_idx - w_raw_idx + BUF_RAW_SIZE ),
                                 ( ( r_raw_idx - w_raw_idx + BUF_RAW_SIZE ) & adtsp->data_raw_wrap ),
                                 (uint32_t) adtsp->data_raw_wrap
                               );

                        myerr = ADTSPARSE_ERR_INPUT_QUEUE_OVERFLOW;

                        /* trigger DSP parser for decoding */
                        TSP_SEMAPHORE_POST( NULL, adtsp->adts_thread_sem );
                    }
                }
            }

            /*
             *  REWIND sanity check
             */
            if ( NO_ERR == myerr )
            {
                /* circular buffer on raw data, potentially needs (buf_len, max PKT_SIZE) space */
                if ( ( adtsp->data_raw_w_idx + buf_len ) >= BUF_RAW_SIZE )
                {
                    /* we are going to rewind the fifo, make sure we do not overlap */
                    /* with the current read idx */

                    if ( ( adtsp->data_fifo[ adtsp->data_fifo_r_idx ].status != 0 ) &&
                         ( adtsp->data_fifo[ adtsp->data_fifo_r_idx ].raw_idx <= buf_len ) )
                    {
                        LOG_ERR( adtsp->log_lvl, LOG_MEDIUM,
                                 "[ADTSPARSE][%s] [ERROR] BUF_CHECK overflow on REWIND for INPUT RAW FIFO, "
                                 "w_raw_idx=%" PRIu32 " fifo_r_idx=%" PRIu32 " r_raw_idx=%" PRIu32 "\n",
                                 adtsp->label,
                                 adtsp->data_raw_w_idx,
                                 adtsp->data_fifo_r_idx,
                                 adtsp->data_fifo[ adtsp->data_fifo_r_idx ].raw_idx
                               );

                        myerr = ADTSPARSE_ERR_INPUT_QUEUE_OVERFLOW;

                        /* trigger DSP parser for decoding */
                        TSP_SEMAPHORE_POST( NULL, adtsp->adts_thread_sem );
                    }
                    else
                    {
                        LOG_MSG( adtsp->log_lvl, LOG_MEDIUM, "[ADTSPARSE][%s] input_push_buf (p) -->REWIND f.idx=%" PRIu32 ", raw.w.idx=%" PRIu32 ", raw.r.idx=%" PRIu32 ", size=%" PRIu32 "\n",
                                 adtsp->label,
                                 adtsp->data_fifo_w_idx,
                                 adtsp->data_fifo[ ( adtsp->data_fifo_w_idx - 1 ) & adtsp->data_fifo_wrap ].raw_idx,
                                 adtsp->data_fifo[ adtsp->data_fifo_r_idx ].raw_idx,
                                 BUF_RAW_SIZE );

                        adtsp->data_raw_w_idx = 0;
                    }
                }
            }



            if ( NO_ERR == myerr )
            {
                /* PROFILING/TraceX */
                TPM_USRLOG( NULL, &adtsp->tpm,
                            TPM_ADTS_INPUT_AAC_PUSH,
                            adtsp->data_fifo_w_idx,
                            adtsp->data_fifo_r_idx,
                            adtsp->data_raw_w_idx,
                            buf_len );

                /* describe the buffer */
                LOG_MSG( 0, LOG_LOW, "[ADTSPARSE][%s] input_push_buf (p) f.idx=%" PRIu32 ", raw.w.idx=%" PRIu32 ", raw.w=%" PRIu32 ", raw.r=%" PRIu32 ", size=%" PRIu32 " \n",
                         adtsp->label,
                         adtsp->data_fifo_w_idx,
                         adtsp->data_raw_w_idx,
                         adtsp->data_fifo[ adtsp->data_fifo_w_idx ].raw_idx,
                         adtsp->data_fifo[ adtsp->data_fifo_r_idx ].raw_idx,
                         buf_len
                       );

                adtsp->data_fifo[ adtsp->data_fifo_w_idx ].raw_idx = adtsp->data_raw_w_idx;
                adtsp->data_fifo[ adtsp->data_fifo_w_idx ].raw     = &adtsp->data_raw[ adtsp->data_raw_w_idx ];

                /*
                 * copy
                 */
                memcpy( adtsp->data_fifo[ adtsp->data_fifo_w_idx ].raw, buf, buf_len );

                adtsp->data_fifo[ adtsp->data_fifo_w_idx ].length        = buf_len;
                adtsp->data_fifo[ adtsp->data_fifo_w_idx ].bitstream_idx = adtsp->data_bitstream_idx;
                adtsp->data_fifo[ adtsp->data_fifo_w_idx ].type          = buf_type;

                /* mark the slot only AFTER being populated */
                adtsp->data_fifo[ adtsp->data_fifo_w_idx ].status  = 1;


                /* indexing fifo for next data */
                adtsp->data_fifo_w_idx = ( adtsp->data_fifo_w_idx + 1 ) & adtsp->data_fifo_wrap;

                /* indexing raw data */
                adtsp->data_raw_w_idx = ( adtsp->data_raw_w_idx + buf_len ) & adtsp->data_raw_wrap;

                /* bitstream is increasing over 64 bit */
                adtsp->data_bitstream_idx += buf_len;

                /* trigger DSP parser for decoding */
                TSP_SEMAPHORE_POST( NULL, adtsp->adts_thread_sem );
            }


            if ( myerr == ADTSPARSE_ERR_INPUT_QUEUE_OVERFLOW )
            {
                /* trigger DSP parser for decoding */
                TSP_SEMAPHORE_POST( NULL, adtsp->adts_thread_sem );
            }
        }
    }


_exit:
    if ( NULL != err )
    {
        *err = myerr;
    }

    return push_byte_cnt;
}


void adtsparse_set_tpconfig( int8_t*    err,
                             void*      adtsparse,
                             void*      tpconfig,
                             tpm_tool_t tool_type )
{
    int8_t       myerr = NO_ERR;
    adtsparse_t* adtsp = (adtsparse_t*) adtsparse;

    /* sanity check */
    if ( NULL == adtsp )
    {
        myerr = ADTSPARSE_ERR_UNKNOWN;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_adtsparse( NULL, adtsp, ADTSPARSE_CTYPE_ANYTYPE ) )
        {
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
    }

    if ( ( NO_ERR == myerr ) && ( NULL != adtsp ) )
    {
        if ( tool_type < TPM_TOOL_MAX )
        {
            adtsp->tpm.tool_type   = tool_type;
            adtsp->tpm.tool_config = tpconfig;
        }
        else
        {
            myerr = -1;
        }
    }

    if ( NULL != err )
    {
        *err = NO_ERR;
    }
}


void adtsparse_set_start_offset( int8_t*  err,
                                 void*    adtsparse,
                                 uint32_t start_offset_ms )
{
    int8_t       myerr = NO_ERR;
    adtsparse_t* adtsp = (adtsparse_t*) adtsparse;

    /* sanity check */
    if ( NULL == adtsp )
    {
        myerr = ADTSPARSE_ERR_UNKNOWN;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_adtsparse( NULL, adtsp, ADTSPARSE_CTYPE_ANYTYPE ) )
        {
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
    }

    /*
     * Not supported yet.
     */

    myerr = -1;

    if ( NULL != err )
    {
        *err = NO_ERR;
    }
}


/* ******************************************************************************* */
/* GETTERS                                                                         */
/* ******************************************************************************* */
void adtsparse_get_trackInfo( int8_t*            err,
                              void*              adtsparse,
                              adts_track_info_t* adts_track_info )
{
    int8_t            myerr = NO_ERR;

    adtsparse_t*      adtsp = NULL;

    volatile uint8_t* ctrl = NULL;



    /* sanity check */
    if ( ( NULL == adtsparse ) || ( NULL == adts_track_info ) )
    {
        if ( NULL != err )
        {
            *err = ADTSPARSE_ERR_UNKNOWN;
        }

        return;
    }

    /* retrieve component base obj */
    adtsp = (adtsparse_t*) adtsparse;


    /*
     * this check could be too CPU heavy for production since it does happen
     * for each pkt triggering. Enabled only in DBG build.
     */
#if defined( DEBUG )
    if ( NO_ERR == myerr )
    {
        if ( 0 == match_adtsparse( NULL, adtsp, ADTSPARSE_CTYPE_ANYTYPE ) )
        {
            myerr = ADTSPARSE_ERR_UNKNOWN;
            return;
        }
    }
#endif /* DEBUG */

    if ( NO_ERR == myerr )
    {
        /* retrieve the control register */
        ctrl =  (volatile uint8_t*) &adtsp->ctrl;

        /* check for running component */
        if ( ( ADTSPARSE_CSTATUS_RUNNED != adtsp->status ) ||
             ( !( ( *ctrl ) & ADTSPARSE_CTRL_ONOFF_STATUS ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS,
                     "[ADTSPARSE][%s] [ERROR] input_push: adtsparse is NOT running (CTRL=%" PRIu8 ")\n", adtsp->label, adtsp->ctrl );
            myerr = ADTSPARSE_ERR_UNKNOWN;
        }
    }


    /* retrieve info */
    if ( NO_ERR == myerr )
    {
        /* NOTE: .aac headers do not provide info on time duration */
        adts_track_info->creation_time     = 0;
        adts_track_info->modification_time = 0;
        adts_track_info->timescale         = 0;
        adts_track_info->duration          = 0;
    }


    if ( NULL != err )
    {
        *err = myerr;
    }
}
