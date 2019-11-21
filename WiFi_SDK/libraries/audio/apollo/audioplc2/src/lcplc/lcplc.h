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
/* $Id: lcplc.h 1.40 2011/03/23 21:08:19 rzopf Exp $ */

/* $Log: lcplc.h $
 * Revision 1.40  2011/03/23 21:08:19  rzopf
 * convert xq to long
 *
 * Revision 1.39  2011/03/23 20:50:35  rzopf
 * *** empty log message ***
 *
 * Revision 1.38  2011/03/22 23:04:13  rzopf
 * further speed up/ clean up.
 *
 * Revision 1.37  2011/03/22 16:48:07  rzopf
 * malloc of xqbuf, but not working
 *
 * Revision 1.36  2011/03/22 15:20:30  rzopf
 * cleanup.
 *
 * Revision 1.35  2011/03/10 18:21:48  rzopf
 * AAC 48kHz functionality
 *
 * Revision 1.34  2011/02/25 17:16:29  rzopf
 * 32kHz SBC
 *
 * Revision 1.33  2011/02/24 17:49:11  rzopf
 * 16kHz SBC FR working.
 *
 * Revision 1.32  2011/02/23 18:16:52  rzopf
 * *** empty log message ***
 *
 * Revision 1.31  2011/02/11 21:55:08  rzopf
 * restructured LC-PLC for improved sub-frame flexibility.
 *
 * Revision 1.30  2011/02/07 21:40:39  rzopf
 * modified for 48kHz aac
 *
 * Revision 1.29  2011/01/14 01:31:28  rzopf
 * support of high sampling rates (ie. 48kHz).
 *
 * Revision 1.28  2010/11/09 18:15:55  rzopf
 * added cvsd memory compensation for when no re-encoding h/w is available.
 *
 * Revision 1.27  2010/01/07 22:51:04  rzopf
 * *** empty log message ***
 *
 * Revision 1.26  2010/01/06 18:02:29  rzopf
 * Cleanup
 *
 * Revision 1.25  2009/12/23 23:29:54  rzopf
 * *** empty log message ***
 *
 * Revision 1.24  2009/12/18 20:28:07  rzopf
 * 8kHz version basically in place now.  16kHz untested.
 *
 * Revision 1.23  2009/12/17 21:13:03  rzopf
 * intermediate version
 *
 * Revision 1.22  2009/12/16 18:16:23  rzopf
 * intermediate version - making code run-time switchable between nb/wb
 *
 * Revision 1.21  2009/12/15 00:51:05  rzopf
 * add Packet Loss Rate attenuation.
 *
 * Revision 1.20  2009/10/24 01:58:27  rzopf
 * fixed point speech classifier.
 *
 * Revision 1.19  2009/10/24 01:06:28  rzopf
 * configurable length of zirr and do.
 *
 * Revision 1.18  2009/10/23 23:08:56  rzopf
 * configurable frame size
 *
 * Revision 1.17  2009/05/04 18:56:52  rzopf
 * Added noise-mixing switch, but the noise mixing code is not working
 * properly and should not be enabled.
 *
 * Revision 1.16  2009/04/30 19:12:00  rzopf
 * *** empty log message ***
 *
 * Revision 1.15  2009/04/27 21:25:04  rzopf
 * *** empty log message ***
 *
 * Revision 1.14  2009/04/27 17:28:36  rzopf
 * *** empty log message ***
 *
 * Revision 1.13  2009/04/23 20:35:17  rzopf
 * *** empty log message ***
 *
 * Revision 1.12  2009/01/27 16:49:46  rzopf
 * added a state variable to store the original pitch
 *
 * Revision 1.11  2009/01/27 02:41:19  rzopf
 * ZIR signal.
 *
 * Revision 1.10  2009/01/27 00:52:52  rzopf
 * *** empty log message ***
 *
 * Revision 1.9  2009/01/26 23:55:49  rzopf
 * Changes to integrate the PLC with SBC.
 *
 * Revision 1.8  2008/07/28 22:02:50  rzopf
 * 16kHz
 *
 * Revision 1.7  2008/07/16 16:16:33  rzopf
 * added state variable gaf for gain attenuation factor.
 *
 * Revision 1.6  2007/11/20 21:16:51  rzopf
 * Includes the #defines for controlling the new "mini-plc" version
 * now implemented in lcplc.c.  To enable this new version, set
 * #define MINI_PLC=1.
 *
 * Revision 1.5  2007/09/13 18:37:47  rzopf
 * added MPY_32_16 macro for ARM.
 *
 * Revision 1.4  2007/07/26 23:08:29  rzopf
 * added gain compensation and delayed OLA code.
 *
 * Revision 1.3  2007/07/24 19:59:44  rzopf
 * updating to latest version.
 *
 * Revision 1.2  2007/06/04 21:32:52  rzopf
 * Changed frame size from 40 to 30.
 *
 * Revision 1.1  2007/06/04 19:54:32  rzopf
 * Initial Version.
 *
 */
#ifndef LCPLC_H
#define LCPLC_H

#ifdef __cplusplus
extern "C" {
#endif

/* PLC Method */
#define  FR_      0       /* Frame Repeat */
#define  LCPLC_   1       /* Pitch Extrapolation */

#define  SPEECHPLC      0
#define  MUSICPLC       1
#define  SPEECHMUSICPLC 2

#define MAX_CODEC_CONFIGS 2

/*--------------------------------------------------------------------------------------------------
 * #define AFFECTING MEMORY USAGE
 * ==============================
 *
 * The #defines in this section are used for memory allocation.  They should be set according to the
 * vector requirements of the supported configurations.
 * --------------------------------------------------------------------------------------------------*/

/* LEFMAX = max(midpp+hppr+rpsr+pwsz+frsz) for all supported configurations */
/*  cvsd 30  sample frame = 293 */
/*  esbc 120 sample frame = 646 */ /* + 40 for 160 frame size */
// #define  LEFMAX         5000 //(646+40)           /* % Length to the End of current Frame of xq()          */

/* OLALMAX = max(olal, olalf) for all supported configurations */
#define  OLALMAX        1024 // 60

/* DOMAX = max(dola) for all supported configurations */
#define  DOMAX           512  // 38             /* max delay OLA */

#define  MAX8KFRSZ       60   /* Maximum frsz length for 8k config  */

#define  MAXRINGL        1024 /* maximum ringing length for filter ringing OLA */

// #define XQBUFLEN           (LEFMAX+OLALMAX+DOMAX)
/*--------------------------------------------------------------------------------------------------*/
/*  END OF #define AFFECTING MEMORY USAGE */
/*--------------------------------------------------------------------------------------------------*/


/* The #defines below do not change */
#define  SMDTH       4915                               /* % 1.15 Sum of Magnitude Difference THreshold for pitch submultiples */
#define  STFO        8                                  /* % Short-Term Filter Order for ringing calculation           */
#define  LTRATT      24576                              /* % 0.75 Long-Term Ringing ATTenuation factor from PWE scaling factor */
#define  GCINI       9830                               /* % 0.3 Q15 Gain Correction INItial value */
#define  GCATT       32545                              /* % 0.9932 Q15 Gain Correction ATTenuation factor */
#define  PLRATTB      ( (int16_t) ( 0.99 * 32768.0 ) )  /* decay factor for signal attenuation at high loss rates */
#define  PLR_B        ( (int16_t) ( 0.975 * 32768.0 ) ) /* Running Mean Beta Factor for Packet Loss Rate Estimation */
#define  PLR_B1       ( (int16_t) ( 32768l - PLR_B ) )  /* 1 - PLR_B */

/* CVSD Step Size Compensation */
#define  SSATT         0.9952
#define  FSSATT        32611

/* Macros */
#if ARM_MATH == 1
    #define MPY_32_16( x32, y16 ) (int32_t) ( ( (__int64) ( x32 ) * (__int64) ( y16 ) ) >> 15 )
#else
    #define EXTRACT_HI( x ) ( x >> 16 )
    #define EXTRACT_LO( x ) ( ( x - ( ( x >> 16 ) << 16 ) ) >> 1 )
    #define MPY_32_16( x32, y16 ) ( ( ( EXTRACT_HI( x32 ) * ( (int32_t) y16 ) ) + ( ( EXTRACT_LO( x32 ) * ( (int32_t) y16 ) ) >> 15 ) ) << 1 )
#endif

struct LCPLC_config        /* Sampling frequency, codec, or customer dependent configs */
{
    int16_t  sf;           /* Sampling Frequency                                             */
    int16_t  sfratio;      /* Ratio of sampling frequency to 8kHz (sf/8)                     */
    // int16_t  frsz;           /* Frame Size (samples)                                           */
    int16_t  dola;         /* Delayed OverLap Add length (samples)                           */
    int16_t  reencsw;      /* Re-Encoding in software is required (1) or done in h/w (0)     */
    int16_t  nsub;         /* Number of Subframes                                            */
    // int16_t  subfrsz;        /* Subframe Size (samples)                                        */
    int16_t  plratt_max;   /* Maximum attenuation once the PLR reaches PLRATTEND (State variable used for CGS           */
    int16_t  plrattstrt;   /* Attenuation begins at this PLR %, with the attenuation ramping down from 1 (CGS Variable) */
    int16_t  plrattend;    /* Attenuation reaches its maximum once the PLR % hits this rate (CGS Variable)              */
    int16_t  gattst;       /* % Gain ATTenuation STarting frame (into erasure)               */
    int16_t  gattf;        /* % Gain ATTenuation Factor (per sample)                         */
    long     attslp;       /* Adaptive Attenuation Curve implemented as slope+y-intercept. ((1.0-PLRATT_MIN)/(PLRATT_STRT-PLRATT_END)) */
    long     attyinter;    /* Adaptive Attenuation Curve implemented as slope+y-intercept. (1.0-PLRATT_SLP*PLRATT_STRT)                */
    int16_t  gattfu;       /* Gain Recovery Factor                                           */
    int16_t  decf;         /* DECimation Factor for coarse pitch period search               */
    int16_t  midpp;        /* MIDdle point of the pitch period range                         */
    int16_t  hppr;         /* Half the Pitch Period Range                                    */
    int16_t  pwsz;         /* Pitch analysis Window SiZe for 8 kHz lowband (15 ms)           */
    int16_t  smwsz;        /* pitch Sub-Multiple search Window SiZe (fixed)                  */
    int16_t  sfwsz;        /* Scale Factor Window Size                                       */
    int16_t  rpsr;         /* Refined Pitch Search Range (# of samples in delta pitch)       */
    int16_t  olal;         /* OverLap Add Length                                             */
    int16_t  olalf;        /* OverLap Add Length First good frame                            */
    int16_t* ola;          /* pointer to be set to OLA table                                 */
    int16_t* olaf;         /* pointer to be set to OLA table for use in first good frame     */
    int16_t  minpp;        /* MINimum Pitch Period (in # of 8 kHz samples) = (MIDPP-HPPR)    */
    int16_t  stwsz;        /* int16_t-Term predictive analysis Window SiZe                     */
    int16_t  maxpp2;       /* largest value that is a power of 2 and is less than the max pp (midpp+hppr) */
    int16_t  usefiltring;  /* use filter ringing in first bad frame? 0=NO, 1=YES             */
    int16_t  applyola2zsr; /* apply the overlap-add window to the codec ZSR signal?  0/1     */
    int16_t* olauv;        /* ola window for unvoiced (uncorrelated) signals                 */
    int16_t* olauvf;       /* ola window for unvoiced (uncorrelated) signals in first good frame */
    int16_t  folding;      /* window folding (ie. MDCT) present in the codec? (0=NO, 1=YES)  */
    int16_t* olafold;      /* overlap-add window for folding                                 */
    int16_t* zirolav;      /* overlap add window for zir/ringing in first bad frame - voiced */
    int16_t* zirolauv;     /* overlap add window for zir/ringing in first bad frame - unvoiced */
    int16_t  zirwin;       /* 1= window is applied to incoming signal, 0 = no window applied */
    int16_t  ringl;        /* length of ringing and zir windows in the case that zirwin=0 and ringing is used */
    int16_t  maxzsrolal;   /* max zsrolal (including all possible first good frame windows */
    int16_t  zirolastart;  /* the number of samples before zir ola begins                    */
    int16_t  zirolal;      /* zir ola length                                                 */
    int16_t  fprr;         /* final pitch refinement range                                   */
    int16_t  fprwl;        /* final pitch refinement window length                           */
    int16_t  fprdecf;      /* final pitch refinement decimation factor                       */
};

/* LCPLC State Information */
struct LCPLC_State
{
    int16_t                ValidMemCheck;
    int16_t                frsz;
    int16_t                cfecount;   /* continuous frame erasure count                     */
    int16_t                pp;         /* pitch period                                       */
    int16_t                opp;        /* orig pp                                            */
    int16_t                sf;         /* scaling factor for pwe                             */
#if 0
    long*                  xq;         /* signal buffer                                      */
#else
    // XXX
    int16_t*               xq;
#endif
    int16_t                gc;                            /* Gain Correction                                    */
    int16_t                gaf;                           /* gain attenuation factor                            */
    int16_t                natt;                          /* Number of consecutively lost samples before atten. */
    int16_t                Class;                         /* Speech frame classification                        */
    uint32_t               idum;                          /* Random number generator seed memory                */
    int16_t                plr;                           /* Packet Loss Rate                                   */
    int32_t                plratt;                        /* Packet Loss Rate based ATTenuation factor          */
    int16_t                ssc;
    int16_t                ss;                            // step size estimate;
    int16_t                yhat;                          // cvsd acc estimate;
    int16_t                ss_1;                          // step size estimate from last frame;
    int16_t                yhat_1;                        // cvsd acc estimate from last frame;
    int16_t                last_BFI;                      // Bad Frame Indicator from the last frame
    int16_t                BEC;                           // Bit Error Concealment Detection Status (1= biterror, 0=no error)
    int16_t                fpp;                           /* frames per packet */
    struct   LCPLC_config* config[ MAX_CODEC_CONFIGS ];   /* LC-PLC pitch extrapolation (speech) configurations */
    struct   LCPLC_config* frconfig[ MAX_CODEC_CONFIGS ]; /* Frame Repeat Configurations */
    int16_t                ConfigSetting;                 /* SPEECHPLC, MUSICPLC, SPEECHMUSICPLC                */
    int16_t                xqbuflen;                      /* length of xq buffer */
    int16_t                MusicClassHist;
    int16_t                delay;
    int16_t                frameshift;
    long                   ldelayin[ SL_MAX ];
};



/* Prototypes */
int LC_PLC_erasure( struct LCPLC_State* plc_state, void* out, int16_t* CodecRingBuf,
                    int16_t zirl, int16_t PlcMethod, int16_t CodecConfig, int16_t IOformat );
int LC_PLC( struct LCPLC_State* plc_state, void* in, void* out, int16_t nzsr, int16_t PlcMethod, int16_t CodecConfig, int16_t IOformat );
int Init_LC_PLC( struct LCPLC_State* plc_state, int sf, int frmsz, int delay );
int Free_LC_PLC( struct LCPLC_State* plc_state );
int allocLC_PLC( struct LCPLC_State** plc_state, int sf, int frmsz );

#ifdef __cplusplus
}
#endif
#endif /* LCPLC_H */
