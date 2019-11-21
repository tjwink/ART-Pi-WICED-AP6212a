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

#include "mp4parse.h"
#include "adtsparse.h"
#include "logger.h"
#include "globals.h"

#include "tsp.h"
#include "tpm.h"


#include "aacdec_types.h"
#include "aacdec_core.h"


/* ****************************************************************************** */
/* ****************************************************************************** */
/* PROTOTYPES                                                                     */
/* ****************************************************************************** */
/* ****************************************************************************** */
static void init_aacdec( int8_t* err, aacdec_t* aacdec );
static void destroy_aacdec( int8_t* err, aacdec_t** aacdec, uint8_t force_f );

static inline int8_t match_aacdec( int8_t* err, aacdec_t* aacdec, aacdec_ctype_t checktype );
static inline int8_t check_ready_aacdec( int8_t* err, aacdec_t* aacdec );

static inline int8_t check_ready_aacdec( int8_t* err, aacdec_t* aacdec );

static inline void set_tpconfig( int8_t* err, aacdec_t* aacdec, void* tpconfig, tpm_tool_t tool_type );

static uint32_t pcm_output_push( int8_t*         err,
                                 void*           mp4parse,
                                 void*           decoder,
                                 mp4_pcm_info_t* pcm_info,
                                 int16_t*        buf,
                                 uint32_t        buf_len,
                                 uint32_t*       buf_id );


/* ****************************************************************************** */
/* ****************************************************************************** */
/* PRIVATE                                                                        */
/* ****************************************************************************** */
/* ****************************************************************************** */

/*
 * Function: check_ready
 */
static inline int8_t check_ready_aacdec( int8_t* err, aacdec_t* aacdec )
{
    int8_t myerr = 0;
    int8_t ret = TRUE;

    if ( NULL == aacdec )
    {
        return FALSE;
    }

    /* place here all the sanity checks to declare the component "READY" */
    if ( ( TRUE == ret ) && ( AACDEC_CSTATUS_CREATED != aacdec->status ) )
    {
        ret = FALSE;
    }

    /* this MUST be the only place that can bump the status to "READY" */
    if ( TRUE == ret )
    {
        aacdec->status = AACDEC_CSTATUS_READY;
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
static inline int8_t match_aacdec( int8_t* err, aacdec_t* aacdec, aacdec_ctype_t checktype )
{
    int8_t ret = FALSE;

    int8_t myerr = NO_ERR;

    if ( NULL == aacdec )
    {
        if ( NULL != err )
        {
            *err = -1;
        }

        return ret;
    }

    if ( ( AACDEC_CTYPE_RESERVED < aacdec->type ) && \
         ( AACDEC_CTYPE_MAX > aacdec->type ) )
    {
        ret = ( AACDEC_WMARK == aacdec->wmark ) ? TRUE : FALSE;
    }

    if ( ( ret == TRUE ) && ( AACDEC_CTYPE_ANYTYPE != checktype ) )
    {
        ret = ( checktype == aacdec->type ) ? TRUE : FALSE;
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
static void init_aacdec( int8_t* err, aacdec_t* aacdec )
{
    int8_t    myerr = NO_ERR;

    aacdec_t* adec = aacdec;

    if ( NULL != adec )
    {
        LOG_MSG( adec->log_lvl, LOG_LOW, "[AACDEC][-] init_aacdec\n" );

        /* init random seed */
        // srand((uint32_t)time(NULL));

        /* component base */
        adec->id    = 1; // should be random();
        adec->wmark = AACDEC_WMARK;
        adec->type  = AACDEC_CTYPE_UNKNOWN;

        /* We only support one mode: */
        adec->type = AACDEC_CTYPE_MULTI_THREAD;

        /* versioning stuff: the default is "0.0.0.none" */
        adec->major = 0;
        adec->minor = 0;
        strncpy( adec->revision, "0000000000000000", ( 16 + 1 ) ); /* revision is 32 char max */
        strncpy( adec->build,    "none",             ( 4 + 1 ) );  /* build is 5 char max */

        /* override versioning by build machine at compilation time */
#if ( defined( VERSION_MAJOR ) && defined( VERSION_MINOR ) && defined( VERSION_REVISION ) && defined( VERSION_TYPE ) )
        adec->major = VERSION_MAJOR;
        adec->minor = VERSION_MINOR;
        strncpy( adec->revision, VERSION_REVISION, 16 );
        strncpy( adec->build,    VERSION_TYPE,     4 );
#endif /* versioning override */

        /* CTRL register is internal */
        adec->ctrl = 0;
        strncpy( adec->label, "aacdec_cmp\0", 14 );

        /* in/out threads setup */
        TSP_MUTEX_INIT( myerr, adec->ctrl_mtx );


        /* internals */
        adec->mp4parse = NULL;

        adec->pcm_info_tgt_f = FALSE;

        adec->tpm.tool_type = TPM_TOOL_UNKNOWN;

        /* OUTPUT callbacks*/

        /* INPUT callbacks*/

        /* ALERT callbacks */

        /* external app ptr */

        /* update status */
        adec->status = AACDEC_CSTATUS_CREATED;

        LOG_MSG( adec->log_lvl, LOG_MEDIUM, "[AACDEC][%s] init_aacdec: done.\n", adec->label );
    }
    else
    {
        myerr = -1;
    }


    if ( NULL != err )
    {
        *err = myerr;
    }
}


static void destroy_aacdec( int8_t* err, aacdec_t** aacdec, uint8_t force_f )
{
    int8_t    myerr = NO_ERR;
    aacdec_t* adec = NULL;


    /*
     * SANITY CHECKS
     */

    /* sanity check on the pointers */
    if ( ( NULL == aacdec ) || ( NULL == *aacdec ) )
    {
        if ( NULL != err )
        {
            *err = -1;
        }
        return;
    }

    adec = *aacdec;

    if ( NO_ERR == myerr )
    {
        /* PROFILING/TraceX */
        TPM_STOP( err, adec->tpm );
        TPM_REPORT( err, adec->tpm );
    }


    if ( adec->stream_type == AACDEC_STREAM_M4A )
    {
        /* MP4 parser */
        if ( ( NO_ERR == myerr ) && ( FALSE == force_f ) )
        {
            LOG_MSG( adec->log_lvl, LOG_LOW, "[AACDEC][%s] aacdec_destroy: flush mp4parse first. \n", adec->label );
            mp4parse_flush( NULL, adec->mp4parse );
        }

        if ( NO_ERR == myerr )
        {
            mp4parse_destroy( &myerr, &adec->mp4parse );
        }
    }

    if ( adec->stream_type == AACDEC_STREAM_ADTS )
    {
        /* ADTS parser */
        if ( ( NO_ERR == myerr ) && ( FALSE == force_f ) )
        {
            LOG_MSG( adec->log_lvl, LOG_LOW, "[AACDEC][%s] aacdec_destroy: flush adtsparse first. \n", adec->label );
            adtsparse_flush( NULL, adec->mp4parse );
        }

        if ( NO_ERR == myerr )
        {
            adtsparse_destroy( &myerr, &adec->mp4parse );
        }
    }

    /* locals */
    if ( NO_ERR == myerr )
    {
        TSP_MUTEX_DESTROY( NULL, adec->ctrl_mtx );

        if ( NULL != adec->data_output )
        {
            free( adec->data_output );
            adec->data_output = NULL;
        }

        free( adec );
        *aacdec = NULL;
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


static inline void set_tpconfig( int8_t* err, aacdec_t* aacdec, void* tpconfig, tpm_tool_t tool_type )
{
    int8_t    myerr = 0;
    aacdec_t* adec = aacdec;

    if ( ( NULL == adec ) )
    {
        myerr = -1;
        if ( NULL != err )
        {
            *err = -1;
        }
        return;
    }

    /* check for previous errors */
    if ( NULL != err )
    {
        if ( NO_ERR != ( *err ) )
        {
            return;
        }
    }

    if ( tool_type < TPM_TOOL_MAX )
    {
        switch ( adec->stream_type )
        {
            case AACDEC_STREAM_M4A:
                mp4parse_set_tpconfig( &myerr,
                                       adec->mp4parse,
                                       tpconfig,
                                       tool_type );
                break;

            case AACDEC_STREAM_ADTS:
                adtsparse_set_tpconfig( &myerr,
                                        adec->mp4parse,
                                        tpconfig,
                                        tool_type );
                break;

            case AACDEC_STREAM_UNKNOWN:
                LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] set_tpconfig, auto-stream type t.b.d\n",   adec->label );

            default:
                LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] set_tpconfig, unsupported stream type.\n", adec->label );
                myerr = -1;
                break;
        }

        if ( NO_ERR == myerr )
        {
            adec->tpm.tool_type   = tool_type;
            adec->tpm.tool_config = tpconfig;
        }
    }
    else
    {
        myerr = -1;
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


/* ****************************************************************************** */
/* ****************************************************************************** */
/* TOOLS                                                                          */
/* ****************************************************************************** */
/* ****************************************************************************** */
static uint32_t pcm_output_push( int8_t*         err,
                                 void*           mp4parse,
                                 void*           decoder,
                                 mp4_pcm_info_t* pcm_info,
                                 int16_t*        buf,
                                 uint32_t        buf_len,
                                 uint32_t*       buf_id )
{
    uint32_t  i = 0;
    int8_t    myerr = 0;

    uint32_t  push_bytes = 0;


#if defined( DEBUG )
    if ( NULL == decoder )
    {
        myerr = -1;
    }
#endif

    aacdec_t* adec = decoder;

    /* mute gcc warnings */
    (void) ( mp4parse );
    (void) ( buf_id );

    LOG_MSG( adec->log_lvl, LOG_LOW, "[AACDEC][%s] pcm_output_push_CBACK! %" PRIu32 "  \n", adec->label, buf_len );

    /* **** */
    /* TBD: handle PCM chmap/bps format conversions here! */
    /* **** */


    /* PCM output callback */
    if ( ( NO_ERR == myerr ) && ( NULL != adec->output_push_callback ) )
    {
        if ( buf_id == NULL )
        {
            /* REAL PCM DATA */

            if ( ( buf_len > 0 ) && ( NULL != pcm_info ) )
            {
                for ( i = 0; i < buf_len; i++ )
                {
                    uint8_t* out = &adec->data_output[ 2 * i ];
                    out[ 0 ] = buf[ i ] & 0xff;
                    out[ 1 ] = buf[ i ] >> 8;
                }

                /* describe audio */
                adec->pcm_info_src.chnum       = pcm_info->chnum;
                adec->pcm_info_src.sr          = pcm_info->sr;
                adec->pcm_info_src.bps         = pcm_info->bps;
                adec->pcm_info_src.cbps        = pcm_info->cbps;
                adec->pcm_info_src.chmap       = pcm_info->chmap;
                adec->pcm_info_src.endian      = pcm_info->endian;
                adec->pcm_info_src.sign        = pcm_info->sign;
                adec->pcm_info_src.interleave  = pcm_info->interleave;
                adec->pcm_info_src.floating    = pcm_info->floating;


                if ( NULL != adec->output_push_callback )
                {
                    push_bytes = adec->output_push_callback(
                        &myerr,
                        adec,
                        adec->player,
                        &adec->pcm_info_src,
                        adec->data_output,
                        buf_len,
                        NULL );

                    // push_bytes_tot +=buf_len;
                }
            }
        }
        else
        {
            /* COMMAND PCM DATA */
            if ( ( *buf_id == PCMBUF_ID_CMD_EOP_OK ) ||
                 ( *buf_id == PCMBUF_ID_CMD_EOP_ERR ) )
            {
                uint32_t aacdec_buf_id = 0; /* audio_client END-OF-PLAYBACK */

                push_bytes = adec->output_push_callback(
                    &myerr,
                    adec,
                    adec->player,
                    &adec->pcm_info_src,
                    adec->data_output,
                    0 /* NO PAYLOAD */,
                    &aacdec_buf_id );
            }
        }
    }


    if ( NULL != err )
    {
        *err = myerr;
    }

    return push_bytes;
}


/* ****************************************************************************** */
/* ****************************************************************************** */
/* PUBLIC                                                                         */
/* ****************************************************************************** */
/* ****************************************************************************** */

void aacdec_new( int8_t*            err,
                 void**             aacdec,
                 void*              player,
                 uint8_t            logging_level,
                 aacdec_stream_t    stream_type,
                 aacdec_pcm_info_t* target_pcm_fmt,
                 uint32_t           target_chmap,
                 uint8_t            downmix_f )
{
    int8_t    myerr = NO_ERR;
    aacdec_t* adec = NULL;


    /*
     * SANITY CHECKS
     */

    /* sanity check on the pointers */
    if ( ( NULL == aacdec ) )
    {
        myerr = -1;
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
        if ( NULL != ( *aacdec ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][-] [ERROR] aacdec_new, aac ptr not NULL, FAIL\n" );
            myerr = -1;
        }
    }

    /*
     * MAIN
     */

    /* allocate component base */
    if ( NO_ERR == myerr )
    {
        if ( NULL == ( adec = (aacdec_t*) calloc( 1, sizeof( aacdec_t ) ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][-] [ERROR] aacdec_new, aacdec_t create FAIL\n" );
            myerr = -1;
        }
        else
        {
            adec->memory_footprint = sizeof( aacdec_t );
        }
    }

    /* allocate output queue */
    if ( NO_ERR == myerr )
    {
        if ( NULL == ( adec->data_output = (uint8_t*) calloc( OUTPUT_BUF_SIZE, sizeof( uint8_t ) ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][-] [ERROR] aadec_new, output buf create FAIL\n" );
            myerr = -1;
        }
        else
        {
            adec->memory_footprint += sizeof( uint8_t ) * OUTPUT_BUF_SIZE;
        }
    }

    /* set default values */
    if ( NO_ERR == myerr )
    {
        LOG_MSG( 0, LOG_ALWAYS, "[AACDEC][-] aacdec_new, aacdec_t size=%" PRIu64 "\n", (uint64_t) sizeof( aacdec_t ) );

        /* PRE-INIT */
        adec->id    = 1;
        adec->wmark = AACDEC_WMARK;
        adec->type  = AACDEC_CTYPE_UNKNOWN;

        adec->player = player;

        adec->output_push_callback = NULL;

        /* logging */
        adec->log_lvl = LOG_MEDIUM;
        adec->log_err = LOG_ALWAYS;

        /* params/config values for sub-components  */
    }


    if ( NO_ERR == myerr )
    {
        /* force the type */
        adec->type = AACDEC_CTYPE_RESERVED;

        /* force the status*/
        adec->status = AACDEC_CSTATUS_UNKNOWN;

        /* init the component */
        init_aacdec( &myerr, adec );

        /* stream transport type */
        adec->stream_type = stream_type;

        /* set the initial log level*/
        aacdec_set_log_verbosity( NULL, adec, logging_level );

        /* input PCM options */
        adec->pcm_info_downmix_f = downmix_f & 0x01;

        adec->pcm_info_tgt_chmap = target_chmap;

        if ( NULL != target_pcm_fmt )
        {
            adec->pcm_info_tgt_f = TRUE;
            memcpy( (void*) &adec->pcm_info_tgt, target_pcm_fmt, sizeof( aacdec_pcm_info_t ) );
        }
    }

    /* create mp4_parser/decoder */
    if ( NO_ERR == myerr )
    {
        switch ( adec->stream_type )
        {
            case AACDEC_STREAM_M4A:
                mp4parse_new( &myerr, &adec->mp4parse, adec, LOG_MEDIUM );
                break;

            case AACDEC_STREAM_ADTS:
                adtsparse_new( &myerr, &adec->mp4parse, adec, LOG_MEDIUM );
                break;

            case AACDEC_STREAM_UNKNOWN:
                LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] aacdec_new, auto-stream type t.b.d\n",   adec->label );

            default:
                LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] aacdec_new, unsupported stream type.\n", adec->label );
                myerr = -1;
                break;
        }
    }

    if ( NO_ERR == myerr )
    {
        /* set the initial log level and pcm cback*/

        switch ( adec->stream_type )
        {
            case AACDEC_STREAM_M4A:
                mp4parse_set_log_verbosity( &myerr, adec->mp4parse, logging_level );
                mp4parse_set_output_push_callback( &myerr,
                                                   adec->mp4parse,
                                                   &pcm_output_push );
                break;

            case AACDEC_STREAM_ADTS:
                adtsparse_set_log_verbosity( &myerr, adec->mp4parse, logging_level );
                adtsparse_set_output_push_callback( &myerr,
                                                    adec->mp4parse,
                                                    &pcm_output_push );
                break;

            case AACDEC_STREAM_UNKNOWN:
            default:
                myerr = -1;
                break;
        }
    }

    if ( NO_ERR == myerr )
    {
        /* POST-INIT: configure slc/plc if needed */

        LOG_MSG( 0, LOG_ALWAYS, "[AACDEC][%s] aacdec_new, SUCCESS\n",            adec->label );
        LOG_MSG( 0, LOG_ALWAYS, "[AACDEC][%s] aacdec_new, memory=%" PRIu64 "\n", adec->label, adec->memory_footprint );

        /* return cmp */
        *aacdec = (void*) adec;
    }
    else
    {
        LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][-] [ERROR] aacdec_new, FAIL\n" );

        destroy_aacdec( NULL, &adec, FALSE /*force_f*/ );

        *aacdec = NULL;
    }

    if ( NO_ERR == myerr )
    {
        /* PROFILING/TraceX */
        TPM_START( err, adec->tpm );
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


/*
 * Function:
 *
 */
void aacdec_set_log_verbosity( int8_t* err, void* aacdec, uint8_t lvl )
{
    int8_t    myerr = NO_ERR;
    aacdec_t* adec = (aacdec_t*) aacdec;

    /* sanity check */
    if ( NULL == adec )
    {
        myerr = -1;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_aacdec( NULL, adec, AACDEC_CTYPE_ANYTYPE ) )
        {
            myerr = -1;
            LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][-] [ERROR] aacdec_set_log_verbosity, aacdec invalid.\n" );
        }
    }

    if ( NO_ERR == myerr )
    {
        adec->log_lvl = lvl;
        adec->log_err = LOG_ALWAYS;

        switch ( adec->stream_type )
        {
            case AACDEC_STREAM_M4A:
                mp4parse_set_log_verbosity( &myerr, adec->mp4parse, lvl );
                break;

            case AACDEC_STREAM_ADTS:
                adtsparse_set_log_verbosity( &myerr, adec->mp4parse, lvl );
                break;

            case AACDEC_STREAM_UNKNOWN:
            default:
                LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] aacdec_set_log_verbosity, unsupported stream type.\n", adec->label );
                myerr = -1;
                break;
        }

        LOG_MSG( adec->log_lvl, LOG_MEDIUM, "[AACDEC][%s] aacde_set_log_verbosity: %" PRIu8 "\n", adec->label, lvl );
    }

    if ( NULL != err )
    {
        *err = NO_ERR;
    }
}


void aacdec_start( int8_t* err,
                   void*   aacdec )
{
    int8_t            myerr = NO_ERR;

    aacdec_t*         adec = (aacdec_t*) aacdec;

    volatile uint8_t* ctrl = NULL;

    /* sanity check on the pointers */
    if ( ( NULL == adec ) )
    {
        myerr = -1;
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
        if ( 0 == match_aacdec( NULL, adec, AACDEC_CTYPE_ANYTYPE ) )
        {
            myerr = -1;
        }
        else
        {
            /* retrieve the control register */
            ctrl =  (volatile uint8_t*) &adec->ctrl;
        }

        if ( ctrl == NULL )
        {
            myerr = -1;
        }
    }

    if ( NO_ERR == myerr )
    {
        /* make sure we have all we need*/
        check_ready_aacdec( &myerr, adec );
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
        TSP_MUTEX_LOCK( NULL, adec->ctrl_mtx );
        *ctrl = ( *ctrl ) | AACDEC_CTRL_ONOFF_STATUS;
        TSP_MUTEX_UNLOCK( NULL, adec->ctrl_mtx );
    }

    /*
     * start the inner parser/decoder
     */
    if ( NO_ERR == myerr )
    {
        switch ( adec->stream_type )
        {
            case AACDEC_STREAM_M4A:
                mp4parse_start( &myerr, adec->mp4parse );
                break;

            case AACDEC_STREAM_ADTS:
                adtsparse_start( &myerr, adec->mp4parse );
                break;

            case AACDEC_STREAM_UNKNOWN:
            default:
                LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] aacdec_start, unsupported stream type.\n", adec->label );
                myerr = -1;
                break;
        }
    }


    /*
     * now start the component (inner thread)
     */
    if ( NO_ERR == myerr )
    {
        LOG_MSG( adec->log_lvl, LOG_LOW, "[AACDEC][%s] aacdec_start: status RUNNED.\n", adec->label );


        /* ready update status */
        adec->status = AACDEC_CSTATUS_RUNNED;
    }


    /* auto shutoff on failure */
    if ( myerr != NO_ERR )
    {
        /* signal to the ctrl register */
        TSP_MUTEX_LOCK( NULL, adec->ctrl_mtx );
        *ctrl = ( *ctrl ) & ( ~( AACDEC_CTRL_ONOFF_STATUS ) );
        TSP_MUTEX_UNLOCK( NULL, adec->ctrl_mtx );
    }

    if ( NULL != err )
    {
        *err = NO_ERR;
    }
}


void aacdec_stop( int8_t* err,
                  void*   aacdec,
                  uint8_t flush_f )
{
    int8_t    myerr = NO_ERR;

    aacdec_t* adec = (aacdec_t*) aacdec;


    /* sanity check on the pointers */
    if ( ( NULL == adec ) )
    {
        myerr = -1;
    }

    /* check for previous errors */

    /*
     * if(NULL!=err)
     * {
     *  if(NO_ERR!=(*err))
     *  {
     *      return;
     *  }
     * }
     */

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_aacdec( NULL, adec, AACDEC_CTYPE_ANYTYPE ) )
        {
            myerr = -1;
        }
    }

    if ( NO_ERR == myerr )
    {
        if ( AACDEC_CSTATUS_READY > adec->status )
        {
            myerr = -1;
        }
    }

    /*
     * MAIN
     */
    if ( NO_ERR == myerr )
    {
        if ( AACDEC_CSTATUS_STOPPED != adec->status )
        {
            /* set STOPPED status to prevent input data push */
            adec->status = AACDEC_CSTATUS_STOPPED;


            if ( adec->stream_type == AACDEC_STREAM_M4A )
            {
                /* stop the mp4parser */
                mp4parse_stop( NULL, adec->mp4parse );

                /* flush playback the remaining of the buffers */
                if ( TRUE == flush_f )
                {
                    mp4parse_flush( NULL, adec->mp4parse );
                }
            }

            if ( adec->stream_type == AACDEC_STREAM_ADTS )
            {
                adtsparse_stop( NULL, adec->mp4parse );

                if ( TRUE == flush_f )
                {
                    adtsparse_flush( NULL, adec->mp4parse );
                }
            }

            LOG_MSG( 0, LOG_ALWAYS,
                     "[AACDEC][%s] aacdec STOPPED.\n", adec->label );
        }
    }
    else
    {
        if ( NULL == adec )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] aacdec is NULL\n", "-" );
        }
        else
        {
            LOG_ERR( 0, LOG_ALWAYS,
                     "[AACDEC][%s] [ERROR] aadec not ready for STOP (status=%" PRIu16 ")\n", adec->label, adec->status );
        }
    }


    if ( NULL != err )
    {
        *err = NO_ERR;
    }
}


void aacdec_flush( int8_t* err,
                   void*   aacdec )
{
    int8_t    myerr = NO_ERR;

    aacdec_t* adec = (aacdec_t*) aacdec;


    /* sanity check on the pointers */
    if ( ( NULL == adec ) )
    {
        myerr = -1;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_aacdec( NULL, adec, AACDEC_CTYPE_ANYTYPE ) )
        {
            myerr = -1;
        }
    }

    if ( NO_ERR == myerr )
    {
        // if(AACDEC_CSTATUS_READY > adec->status)
        if ( AACDEC_CSTATUS_STOPPED != adec->status )
        {
            myerr = -1;
        }
    }

    /*
     * MAIN
     */
    if ( NO_ERR == myerr )
    {
        switch ( adec->stream_type )
        {
            case AACDEC_STREAM_M4A:
                mp4parse_flush( &myerr, adec->mp4parse );
                break;

            case AACDEC_STREAM_ADTS:
                mp4parse_flush( &myerr, adec->mp4parse );
                break;

            default:
                LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] aacdec_flush, unsupported stream type.\n", adec->label );
                myerr = -1;
                break;
        }

        LOG_MSG( adec->log_lvl, LOG_LOW,
                 "[AACDEC][%s] aacdec FLUSHED. (err=%" PRIi8 ")\n", adec->label, myerr );

        adec->status = AACDEC_CSTATUS_FLUSHED;
    }
    else
    {
        if ( NULL == adec )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] aacdec_flush, aacdec is NULL\n", "-" );
        }
        else
        {
            LOG_ERR( 0, LOG_ALWAYS,
                     "[AACDEC][%s] [ERROR] aadec not ready for FLUSH (status=%" PRIu16 ")\n", adec->label, adec->status );
        }
    }


    if ( NULL != err )
    {
        *err = NO_ERR;
    }
}


void aacdec_destroy( int8_t* err,
                     void**  aacdec,
                     uint8_t force_f )
{
    int8_t            myerr = NO_ERR;

    aacdec_t*         adec = NULL;

    volatile uint8_t* ctrl = NULL;

    /*
     * SANITY CHECKS
     */

    /* sanity check on the pointers */
    if ( ( NULL == aacdec ) || ( NULL == *aacdec ) )
    {
        if ( NULL != err )
        {
            *err = -1;
        }
        return;
    }

    adec = (aacdec_t*) *aacdec;

    /* check for previous errors */

    /*
     * if(NULL!=err)
     * {
     *  if(NO_ERR!=(*err))
     *  {
     * err = -1;
     *      return;
     *  }
     * }
     */
    if ( NO_ERR == myerr )
    {
        if ( 0 == match_aacdec( NULL, adec, AACDEC_CTYPE_ANYTYPE ) )
        {
            myerr = -1;
            LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][-] [ERROR] aacdec_destroy, aacdec invalid.\n" );
        }
        else
        {
            /* retrieve the control register */
            ctrl =  (volatile uint8_t*) &adec->ctrl;
        }

        if ( ctrl == NULL )
        {
            myerr = -1;
        }
    }


    /* requirement: to allow destroy we need to be ready for it */
    if ( NO_ERR == myerr )
    {
        if ( ( adec->status != AACDEC_CSTATUS_CREATED ) &&
             ( adec->status != AACDEC_CSTATUS_STOPPED ) )
        {
            LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] aadec_destroy, not in a STOPPED state.  \n", adec->label );
            myerr = -1;
        }
    }

    if ( NO_ERR == myerr )
    {
        LOG_MSG( 0, LOG_ALWAYS, "[AACDEC][%s] aacdec_destroy \n", adec->label );

        /* update the status */
        adec->status = AACDEC_CSTATUS_STOPPED;

        /* signal to the ctrl register */
        TSP_MUTEX_LOCK( NULL, adec->ctrl_mtx );
        *ctrl = ( *ctrl ) & ( ~( AACDEC_CTRL_ONOFF_STATUS ) );
        TSP_MUTEX_UNLOCK( NULL, adec->ctrl_mtx );


        destroy_aacdec( &myerr, (aacdec_t**) aacdec, force_f );

        *aacdec = NULL;
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


/* ***************************** */
/* SETTERS                       */
/* ***************************** */
uint32_t aacdec_set_parameter( int8_t*        err,
                               void*          aacdec,
                               aacdec_param_t parameter,
                               uint32_t       value,
                               void*          data,
                               uint32_t       data_len )
{
    int8_t    myerr = NO_ERR;
    aacdec_t* adec = (aacdec_t*) aacdec;

    uint32_t  rv = 0;


    (void) data_len; /* gcc warning */


    /* sanity check */
    if ( NULL == adec )
    {
        myerr = -1;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_aacdec( NULL, adec, AACDEC_CTYPE_ANYTYPE ) )
        {
            myerr = -1;
        }
    }

    if ( ( NO_ERR == myerr ) && ( NULL != adec ) )
    {
        switch ( parameter )
        {
            case AACDEC_PARAM_TPCFG:
                set_tpconfig( &myerr, adec, data, value );
                break;

            case AACDEC_PARAM_START_OFFSET:
                if (adec->status == AACDEC_CSTATUS_CREATED)
                {
                    adec->start_offset_ms       = value;
                    adec->start_offset_callback = data;
                    switch ( adec->stream_type )
                    {
                        case AACDEC_STREAM_M4A:
                            mp4parse_set_start_offset( &myerr, adec->mp4parse, value );
                            break;

                        case AACDEC_STREAM_ADTS:
                            adtsparse_set_start_offset( &myerr, adec->mp4parse, value );
                            break;

                        case AACDEC_STREAM_UNKNOWN:
                        default:
                            LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] aacdec_set_parameter, unsupported stream type.\n", adec->label );
                            myerr = -1;
                            break;
                    }
                }
                else
                {
                    /*
                     * Only allowed to set the starting offset after the decoder has been created but
                     * before it is started.
                     */
                    LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] aacdec_set_parameter, not in a CREATED state.\n", adec->label );
                    myerr = -1;
                }
                break;

            case AACDEC_PARAM_STREAM_POSITION:
                if (adec->status >= AACDEC_CSTATUS_CREATED)
                {
                    switch ( adec->stream_type )
                    {
                        case AACDEC_STREAM_M4A:
                            mp4parse_set_stream_position( &myerr, adec->mp4parse, value );
                            break;

                        case AACDEC_STREAM_ADTS:
                        case AACDEC_STREAM_UNKNOWN:
                        default:
                            LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] aacdec_set_parameter, unsupported stream type.\n", adec->label );
                            myerr = -1;
                            break;
                    }
                }
                else
                {
                    myerr = -1;
                }
                break;

            default:
                myerr = -1;
        }
    }

    if ( NULL != err )
    {
        *err = myerr;
    }

    return rv;
}


void aacdec_set_output_push_callback( int8_t*                err,
                                      void*                  aacdec,
                                      uint32_t ( *           output_push_callback )(
                                          int8_t*            err,
                                          void*              aacdec,
                                          void*              player,
                                          aacdec_pcm_info_t* pcm_info,
                                          uint8_t*           buf,
                                          uint32_t           buf_len,
                                          uint32_t*          buf_id ) )
{
    int8_t    myerr = NO_ERR;
    aacdec_t* adec = (aacdec_t*) aacdec;

    /* sanity check */
    if ( NULL == adec )
    {
        myerr = -1;
    }

    if ( NO_ERR == myerr )
    {
        if ( 0 == match_aacdec( NULL, adec, AACDEC_CTYPE_ANYTYPE ) )
        {
            myerr = -1;
        }
    }

    if ( ( NO_ERR == myerr ) && ( NULL != adec ) )
    {
        if ( NULL == adec->output_push_callback )
        {
            LOG_MSG( 0, LOG_ALWAYS, "[AACDEC][%s] set_output_push_CBACK: done.\n", adec->label );
            adec->output_push_callback = output_push_callback;
        }
        else
        {
            LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] set_output_push_CBACK: failed\n", adec->label );
            myerr = -1;
        }
    }

    if ( NULL != err )
    {
        *err = myerr;
    }
}


uint32_t aacdec_input_push_buf( int8_t*  err,
                                void*    aacdec,
                                char*    buf,
                                uint32_t buf_len,
                                uint8_t  buf_type  )
{
    int8_t            myerr = NO_ERR;

    aacdec_t*         adec = NULL;

    volatile uint8_t* ctrl = NULL;

    uint32_t          push_byte_cnt = 0;


    /* sanity check */
    if ( NULL == aacdec )
    {
        if ( NULL != err )
        {
            *err = -1;
        }
        /* bail out on invalid component */
        return 0;
    }

    /* check for previous errors */
#if 0
    if ( NULL != err )
    {
        if ( NO_ERR != ( *err ) )
        {
            /* bail out on invalid component */
            return 0;
        }
    }
#endif

    /* retrieve component base obj */
    adec = (aacdec_t*) aacdec;


    /*
     * this check could be too CPU heavy for production since it does happen
     * for each pkt triggering. Enabled only in DBG build.
     */
#if defined( DEBUG )
    if ( NO_ERR == myerr )
    {
        if ( 0 == match_aacdec( NULL, adec, AACDEC_CTYPE_ANYTYPE ) )
        {
            myerr = -1;
            goto _exit;
        }
    }
#endif /* DEBUG */

    if ( NO_ERR == myerr )
    {
        /* retrieve the control register */
        ctrl =  (volatile uint8_t*) &adec->ctrl;

        /* drop input on stopped state */
        /* not an error condition, but we still exit */
        if ( ( AACDEC_CSTATUS_STOPPED == adec->status ) &&
             ( ( *ctrl ) & AACDEC_CTRL_ONOFF_STATUS ) )
        {
            LOG_MSG( adec->log_lvl, LOG_MEDIUM,
                     "[AACDEC][%s] [WARNING] input_push: aacdec is STOPPED, dropping input data (CTRL=%" PRIu8 ")\n", adec->label, adec->ctrl );
            myerr = NO_ERR;
            goto _exit;
        }

        /* check for running component */
        if ( ( AACDEC_CSTATUS_RUNNED != adec->status ) ||
             ( !( ( *ctrl ) & AACDEC_CTRL_ONOFF_STATUS ) ) )
        {
            LOG_ERR( 0, LOG_ALWAYS,
                     "[AACDEC][%s] [ERROR] input_push: aacdec is NOT running (CTRL=%" PRIu8 ")\n", adec->label, adec->ctrl );
            myerr = -1;
        }
    }

    /* sanity check:
     * buffer must be valid in size
     */
    if ( ( NULL == buf ) || ( buf_len == 0 ) || ( buf_len > RTP_PKT_MTU_SIZE ) )
    {
        LOG_ERR( 0, LOG_ALWAYS,
                 "[AACDEC][%s] [ERROR] input_push: invalid buffer\n", adec->label );
        myerr = -1;
    }

    /* buffer must be of a known type */
    if ( ( buf_type == AACDEC_BUFFER_TYPE_UNKNOWN ) || ( buf_type >= AACDEC_BUFFER_TYPE_MAX ) )
    {
        LOG_ERR( 0, LOG_ALWAYS,
                 "[AACDEC][%s] [ERROR] input_push: invalid buffer type\n", adec->label );
        myerr = -1;
    }

    /* bail out if needed */
    if ( NO_ERR != myerr )
    {
        LOG_ERR( 0, LOG_ALWAYS,
                 "[AACDEC][%s] [ERROR] input_push: sanity check failed, skip push.\n", adec->label );

        goto _exit;
    }


    /*
     * MAIN: push buffer to the demuxer
     */
    if ( NO_ERR == myerr )
    {
        LOG_MSG( 0, LOG_HIGH, "[AACDEC][%s] input_push, size=%" PRIu32 "\n", adec->label, buf_len );

        /* FilippiG */
        /* ToDo: change this to a funcptr once we have cleaned up the struct */

        switch ( adec->stream_type )
        {
            case AACDEC_STREAM_M4A:
                push_byte_cnt = mp4parse_input_push_buf( &myerr, adec->mp4parse, buf, buf_len, buf_type );
                break;

            case AACDEC_STREAM_ADTS:
                push_byte_cnt = adtsparse_input_push_buf( &myerr, adec->mp4parse, buf, buf_len, buf_type );
                break;

            default:
                LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] input_push, unsupported stream type.\n", adec->label );
                myerr = -1;
                break;
        }
    }


_exit:
    if ( NULL != err )
    {
        *err = myerr;
    }

    return push_byte_cnt;
}


/* ******************************************************************************* */
/* GETTERS                                                                         */
/* ******************************************************************************* */
void aacdec_get_trackInfo( int8_t*              err,
                           void*                aacdec,
                           aacdec_track_info_t* audio_track_info )
{
    int8_t           myerr = NO_ERR;

    aacdec_t*        adec = NULL;

    mp4_track_info_t mp4_track_info;

    /* sanity check */
    if ( ( NULL == aacdec ) || ( NULL == audio_track_info ) )
    {
        if ( NULL != err )
        {
            *err = -1;
        }
        /* bail out on invalid component */
        return;
    }

    /* check for previous errors */
    if ( NULL != err )
    {
        if ( NO_ERR != ( *err ) )
        {
            *err = -1;
            /* bail out on invalid component */
            return;
        }
    }

    /* retrieve component base obj */
    adec = (aacdec_t*) aacdec;


    /*
     * this check could be too CPU heavy for production since it does happen
     * for each pkt triggering. Enabled only in DBG build.
     */
#if defined( DEBUG )
    if ( NO_ERR == myerr )
    {
        if ( 0 == match_aacdec( NULL, adec, AACDEC_CTYPE_ANYTYPE ) )
        {
            myerr = -1;
        }
    }
#endif /* DEBUG */

    if ( NO_ERR == myerr )
    {
        int8_t tmp8 = -1;

        switch ( adec->stream_type )
        {
            case AACDEC_STREAM_M4A:
                mp4parse_get_trackInfo( &tmp8,
                                        adec->mp4parse,
                                        &mp4_track_info );
                break;

            case AACDEC_STREAM_ADTS:
                adtsparse_get_trackInfo( &tmp8,
                                         adec->mp4parse,
                                         &mp4_track_info );
                break;

            default:
                LOG_ERR( 0, LOG_ALWAYS, "[AACDEC][%s] [ERROR] aacdec_get_trackInfo, unsupported stream type.\n", adec->label );
                myerr = -1;
                break;
        }

        if ( NO_ERR == tmp8 )
        {
            audio_track_info->creation_time     = mp4_track_info.creation_time;
            audio_track_info->modification_time = mp4_track_info.modification_time;

            /* see 14496-12 section 8.4.2.3 */
            if ( ( audio_track_info->duration == 0xFFFFFFFF ) ||
                 ( audio_track_info->timescale == 0 ) )
            {
                /* duration/timescale cannot be set */
                audio_track_info->timescale         = 1;
                audio_track_info->duration          = 0;
            }
            else
            {
                audio_track_info->timescale         = mp4_track_info.timescale;
                audio_track_info->duration          = mp4_track_info.duration;
            }
        }
        else
        {
            myerr = -1;
        }
    }


    if ( NULL != err )
    {
        *err = myerr;
    }
}
