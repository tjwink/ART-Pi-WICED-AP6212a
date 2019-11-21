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
/* $Id: audioPLC.h 1.6 2011/03/22 21:38:41 rzopf Exp $ */

/* $Log: audioPLC.h $
 * Revision 1.6  2011/03/22 21:38:41  rzopf
 * code cleanup.
 *
 * Revision 1.5  2011/03/16 17:35:07  rzopf
 * classifier complete for stereo.
 *
 * Revision 1.4  2011/03/11 20:26:48  rzopf
 * stereo support
 *
 * Revision 1.3  2011/02/23 21:51:13  rzopf
 * *** empty log message ***
 *
 * Revision 1.2  2011/02/11 22:01:41  rzopf
 * *** empty log message ***
 *
 * Revision 1.1  2011/01/14 01:33:21  rzopf
 * Initial Version.
 *
 * */

/* Copyright 2010 Broadcom Corporation.  All Rights Reserved. */
/* Author: Rob Zopf                                           */
/* January 6, 2011                                            */

#ifndef AUDIOPLC_H
#define AUDIOPLC_H

#ifdef __cplusplus
extern "C" {
#endif

#define GOOD_FRAME   0
#define BAD_FRAME    1

#ifndef  MAX_CHAN
// #define  MAX_CHAN 2     /* maximum number of supported channels - stereo support */
#endif

#define  DISCRIM_FS     0.02  /* Frame size (in sec) for speech/music discriminator    */
#define  DISCRIM_WL     50    /* window length for energy buffer with mler algorithm   */
#define  MAX_FRSZ       372   // 1024        /* maximum frame size (in samples) for supported codecs  */
#define  ESHIFT         6     /* 20ms frame size, subsampled to 4kHz gives 80 samples per Energy computation */
#define  CLASSHISTFF    32440 /* Forgetting factor 0.99 */
#define  CLASS_THRES    ( (int16_t) ( 32768.0 * 0.1 ) )

struct SMD_STRUCT
{
    int16_t nbuf;
    int16_t buflen;
    int16_t mler;
    int32_t Ebuf[ DISCRIM_WL ];
};

struct AUDIOPLC_STRUCT
{
    int16_t               ValidMemCheck;
    struct   LCPLC_State* plc_state[ MAX_CHAN ]; /* 2 for stereo support */
    struct   SMD_STRUCT   smd_state[ MAX_CHAN ];
    int16_t               SoftClass[ MAX_CHAN ];
    int16_t               fpp;
    int16_t               frsz;
    int16_t               cfecount;
    int16_t               nchan;
    int16_t               ClassHist[ MAX_CHAN ];
};



/*************************************************************************************
 * Function init_audioPLC()
 *
 * Purpose:
 *
 * Inputs:
 *
 *
 * Outputs
 *
 * Return   -
 *
 *************************************************************************************/
int initAudioPLC( struct AUDIOPLC_STRUCT* aplc, int sf, int frsz, int nchan, int delay );

int freeAudioPLC( struct AUDIOPLC_STRUCT* aplc, int nchan );

int allocAudioPLC( struct AUDIOPLC_STRUCT** aplc, int nchan, int sf, int frmsz );

/*************************************************************************************
 * Function SpeechMusicDiscriminator()
 *
 * Purpose: Compute an indication of the degree that the input is either music (0)
 *          or speech (1).
 *
 * Inputs:
 *          inbuf - pointer to the input signal.  Length is given by aplc->frsz
 *          smd   - pointer to the speech music discriminator memory
 *          aplc  - pointer to the audio plc memory
 *
 * Outputs: softclass - soft decision => 0 = music, 32767 = speech.
 *
 * Return:  1 = updated softclass flag
 *          0 = no update, not enough new data in buffer
 *
 *************************************************************************************/
int16_t SpeechMusicDiscriminator( int16_t* inbuf, struct SMD_STRUCT* smd, struct LCPLC_State* plc_state, int16_t frsz, int16_t* softclass );


#ifdef __cplusplus
}
#endif
#endif /* AUDIOPLC_H */
