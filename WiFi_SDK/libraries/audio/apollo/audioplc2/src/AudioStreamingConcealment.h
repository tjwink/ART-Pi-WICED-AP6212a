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

/*************************************************************************
 * AudioStreamingConcealment
 *
 *************************************************************************/

#ifndef _AUDIOSTREAMINGCONCEALMENT_H
#define _AUDIOSTREAMINGCONCEALMENT_H

#ifdef __cplusplus
extern "C" {
#endif

#define _16BIT 0
#define _24BIT 1

typedef struct
{
    int16_t                      ValidMemCheck;
    int16_t                      slcMemCheck;
    int16_t                      plcMemCheck;
    T_SAMPLE_CONCEALMENT_STRUCT* sampleConcealmentmem;
    struct AUDIOPLC_STRUCT*      audioPLCmem;
    int16_t                      SL;
    int16_t                      nChan;
    int16_t                      frsz;
    int16_t                      IOformat;
} T_AUDIOSTREAMINGCONCEALMENT_STRUCT;

int initAudioStreamingConcealment( T_AUDIOSTREAMINGCONCEALMENT_STRUCT* mem, int32_t sf, int16_t SL, int frsz, int nchan, int slc_enable, int16_t IOformat );
int freeAudioStreamingConcealment( T_AUDIOSTREAMINGCONCEALMENT_STRUCT* mem, int nchan );
int allocAudioStreamingConcealment( T_AUDIOSTREAMINGCONCEALMENT_STRUCT** mem, int nchan, int sf, int frsz, int slc_enable );
int AudioStreamingConcealment( T_AUDIOSTREAMINGCONCEALMENT_STRUCT* mem, void* inbuf[], void* outbuf[], char* FrameStatus );

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif // _AUDIOSTREAMINGCONCEALMENT_H
