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

/*****************************************************************************
 * samplePLC.h
 *
 *
 * Robert Zopf
 * Broadcom Corp.
 * Office of the CTO
 * 08/13/2015
 *****************************************************************************/

#ifndef _SAMPLEPLC_H
#define _SAMPLEPLC_H


#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    SWLSAR, LINEAR
} SAMPLE_CONCEALMENT_METHOD;


typedef struct
{
#if ENABLE_FLOATING
    double*                   aswin; // assymetric window for LPC analysis
    double                    Emax;
    double                    a[ LPCORD_MAX + 1 ];
#endif
    int32_t                   SF;  // sampling frequency in Hz
    int16_t                   WL;  // window length
    int16_t                   SL;  // shuffling length

    // int16_t    hbuf[(int32_t)(SF_MAX*WS)];     // history buffer
    int16_t*                  hbuf; // this will now point into the xq buffer in audioPLC
    int16_t                   hq;   // number of samples to the point of the beginning of the current frame to fix
    int16_t*                  hbufplc;
    int16_t                   a_16[ LPCORD_MAX + 1 ];
    int16_t                   Q_a_16;
    int16_t                   lastsamplefixed;
    SAMPLE_CONCEALMENT_METHOD Method;

    int32_t                   Emax_32;
    int16_t*                  aswin_16; // assymetric window for LPC analysis
} T_SAMPLEPLC_STRUCT;

typedef struct
{
    T_SAMPLEPLC_STRUCT* samplePLC_state[ MAX_CHAN ];
} T_SAMPLE_CONCEALMENT_STRUCT;

int initSamplePLC( T_SAMPLEPLC_STRUCT* mem, int32_t sf, int16_t SL );
int allocSamplePLC( T_SAMPLEPLC_STRUCT** mem );
int freeSamplePLC( T_SAMPLEPLC_STRUCT* mem );

void UpdateSamplePLC( T_SAMPLEPLC_STRUCT* mem, int16_t* inbuf, int16_t L );
int  samplePLC( T_SAMPLEPLC_STRUCT* mem, int16_t* inbuf, int16_t L, int16_t SL, int16_t* SampleStatus, int16_t flag );
int  samplePLC32( T_SAMPLEPLC_STRUCT* mem, int32_t* inbuf, int16_t L, int16_t SL, int16_t* SampleStatus, int16_t flag );
int  initSampleConcealment( T_SAMPLE_CONCEALMENT_STRUCT* mem, int32_t sf, int16_t SL, int nchan );
int  allocSampleConcealment( T_SAMPLE_CONCEALMENT_STRUCT** mem, int nchan );
int  freeSampleConcealment( T_SAMPLE_CONCEALMENT_STRUCT* mem, int nchan );


#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* _SAMPLEPLC_H */
