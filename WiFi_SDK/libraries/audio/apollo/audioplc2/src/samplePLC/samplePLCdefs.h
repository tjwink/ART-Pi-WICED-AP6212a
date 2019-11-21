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
 * samplePLCdefs.h
 *
 *
 * Robert Zopf
 * Broadcom Corp.
 * Office of the CTO
 * 08/13/2015
 *****************************************************************************/

#ifndef _SAMPLEPLCDEFS
#define _SAMPLEPLCDEFS


#ifdef __cplusplus
extern "C" {
#endif

#define ENABLE_FLOATING 0           // for fixed point debugging

#define ONE_OVER_WS     ( 100 )     // one over Window Size in seconds
#define SF_MAX          ( 96000 )   // Max sampling frequency
#define WIN_MAX         ( SF_MAX / ONE_OVER_WS )
#define SL_MAX          ( 18 )
#define LPCORD_MAX      ( 10 )        // SL_MAX
#define LPCORD          ( 10 )

#define ANAWINL         ( 0.01 )
#define MAXNEPW         ( 4 )         // Number of errors per window

// #define MAX_BUF_LEN     3000

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif // _SAMPLEPLCDEFS
