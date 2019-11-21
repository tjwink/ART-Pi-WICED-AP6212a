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

/*
 * DIGOLE b/w image (one bit is one pixel) size 32x32
 */
#ifndef _UGUI_IMG_MONO_H
#define _UGUI_IMG_MONO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced.h"

const uint8_t img_mono[] =
{
    0x0,  0x3f, 0xf8,  0x0,  0x0,  0x1, 0xff,  0x0,  0x3, 0xfc,  0x0, 0xc0,  0x0,  0xf, 0xff,
    0xe0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x3f, 0xfb, 0xf0,  0x0,  0x0,  0x0,  0x0, 0x3f, 0xfc,
    0x0,  0x1c,  0x0,  0x0,  0x0,  0x0, 0x7f, 0xa0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0, 0x7f,
    0x80,  0x0,  0x0,  0x1, 0xf0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x0,  0x1, 0xff, 0x30,  0x0,
    0x0,   0x0,  0x0,  0x0, 0xff, 0xff, 0xfe,  0x5,  0x0,  0x0,  0x0,  0x0, 0xff, 0xff, 0xfe,
    0x0,   0x0,  0x0,  0x0,  0x0, 0x7f, 0xff, 0xff, 0x1e,  0x0, 0xff, 0xff, 0x1e, 0x7f, 0x80,
    0x2,  0x14,  0x0, 0xff, 0xfe, 0x3c,  0x0,  0x0,  0x0,  0x0,  0x0, 0x7f, 0xfc, 0xf8,  0x0,
    0x0,   0x0,  0x0,  0xf, 0xff, 0xf8, 0xf0,  0x0,  0x0,  0x0,  0x0,  0x7, 0xff, 0xf1, 0xc0,
    0x0,   0x0,  0x0,  0x0,  0x0, 0x7f, 0xe2,  0x0,
};

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif
