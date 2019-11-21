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
#ifndef _UGUI_FONT_U8G_UNIFONT_76_H
#define _UGUI_FONT_U8G_UNIFONT_76_H

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced.h"

const uint8_t ugui_u8g_font_walkman[] =
{
    0,  32,  60,   0,   0,  60,   0,   0,   0,   0,  32,  52,   0,   0,   0,  60
    ,    0,   0,   0,   0,  28,   0,  60, 255, 255, 255, 255, 255, 255, 255, 255, 255
    ,  255, 255, 255, 255, 255, 255,  30,  59, 236,  32,   0,   0,   0,   3, 128, 0
    ,    0,   7, 224,   0,   0,  15, 240,   0,   0,  15, 240,   0,   0,  15, 240, 0
    ,    0,  15, 240,   0,   0,   7, 240,   0,   0,   7, 224,   0,   0,  15, 224, 0
    ,    0,  15, 128,   0,   0,  15, 128,   0,   0,  31, 128,   0,   0,  31, 192, 0
    ,    0,  63, 224,   0,   0,  63, 224,   0,   0,  63, 224,   0,   0,  63, 224, 0
    ,    0,  63, 240,   0,   0,  55, 240,   0,   0, 119, 240,   0,   0, 127, 240, 0
    ,    0, 127, 240,   0,   0, 127, 240,   0,   0, 127, 240,   0,   0, 255, 252, 0
    ,    0, 255, 255,   0,   0, 255, 255, 192,   0, 255, 231, 224,   0, 255, 225, 224
    ,    0, 255, 225, 224,   0, 255, 240,   0,   1, 223, 240,   0,   3, 223, 248, 0
    ,    3, 223, 248,   0,   3, 255, 248,   0,   1, 191, 252,   0,   0,  63, 252, 0
    ,    0,  63, 252,   0,   0, 127, 254,   0,   0, 126, 254,   0,   0, 126, 126, 0
    ,    0, 124,  62,   0,   0, 252,  31,   0,   1, 248,  31,   0,   3, 248,  31, 0
    ,    7, 240,  31,   0,   7, 224,  31,   0,   7, 192,  15,   0,  15, 192,  15, 0
    ,   15, 128,  15, 128,  31,   0,  15, 128,  30,   0,  15, 128,  56,   0,   7, 128
    ,  248,   0,   7, 128, 240,   0,   3, 128, 240,   0,   3, 192, 248,   0,   3, 192
    ,  120,   0,   3, 224,  56,   0,   3, 252,  24,  59, 177,  29,   0,   0,   0, 63
    ,    0,   0,  63, 128,   0, 127, 128,   0, 127, 128,   0, 127, 128,   0,  63, 192
    ,    0,  63, 128,   0,  63, 128,   0,  63,   0,   0,  62,   0,   0,  62,   0, 0
    ,  127,   0,   0, 127, 128,   0, 255, 128,   0, 255, 128,   0, 255, 192,   0, 255
    ,  128,   0, 255, 192,   0, 255, 192,   0, 255, 192,   0, 255, 192,   0, 255, 192
    ,    0, 255, 192,   0, 255, 192,   0, 255, 192,   0, 255, 192,   0, 255, 192, 0
    ,  255, 224,   0, 255, 240,   0, 255, 248,   0, 255, 252,   1, 255, 252,   3, 191
    ,  216,   3, 255, 224,   3, 223, 224,   1, 159, 224,   0,  31, 224,   0,  31, 224
    ,    0,  31, 224,   0,  15, 240,   0,  15, 240,   0,  15, 240,   0,  63, 240, 0
    ,  127, 240,   0, 255, 240,   1, 253, 240,   3, 249, 240,   7, 241, 240,   7, 193
    ,  240,  15, 129, 240,  63,   0, 240, 252,   0, 240, 248,   0, 240, 120,   0, 112
    ,   56,   0, 112,  56,   0, 112,  56,   0, 120,  28,   0, 124,   0,   0, 127, 16
    ,   60, 120,  25,   0,   0,   0, 248,   1, 252,   1, 254,   3, 254,   3, 252, 1
    ,  254,   1, 252,   1, 252,   1, 252,   3, 240,   3, 240,   3, 240,   7, 248, 7
    ,  252,   7, 252,   7, 252,   7, 252,   7, 252,   3, 254,   3, 254,   3, 254, 3
    ,  254,   3, 252,   3, 252,   3, 252,   3, 252,   3, 252,   7, 252,   7, 252, 15
    ,  248,  15, 248,   7, 252,   7, 252,   3, 252,   3, 252,   3, 252,   3, 254, 3
    ,  254,   1, 254,   1, 254,   1, 254,   1, 255,   1, 255,   0, 255,   1, 254, 3
    ,  248,   7, 248,   7, 224,  15, 240,  31, 240,  63, 240, 249, 240, 249, 224, 240
    ,  224, 240, 224, 112, 224, 120, 224,  56, 240,  24, 248,   0, 254,  23,  59, 177
    ,   25,   0,   0,   1, 248,   0,   3, 252,   0,   3, 252,   0,   3, 252,   0, 3
    ,  252,   0,   3, 252,   0,   1, 252,   0,   1, 252,   0,   1, 248,   0,   3, 224
    ,    0,   3, 224,   0,   7, 240,   0,  15, 248,   0,  15, 248,   0,  15, 252, 0
    ,   15, 252,   0,  15, 252,   0,  15, 252,   0,  15, 252,   0,  15, 252,   0, 15
    ,  252,   0,  15, 252,   0,  31, 254,   0,  31, 255, 128,  31, 255, 192,  31, 255
    ,  240,  31, 251, 252,  31, 248, 124,  31, 240,  60,  63, 248,   0,  63, 252, 0
    ,  127, 254,   0, 255, 255,   0, 255, 255,   0, 239, 255, 128,  79, 255, 128, 15
    ,  255, 128,  15, 223, 192,  15, 143, 224,  31, 135, 224,  31,   3, 224,  31, 3
    ,  224,  31,   3, 224,  62,   3, 224,  62,   3, 224,  62,   3, 224, 126,   3, 224
    ,  124,   3, 224, 124,   3, 192, 124,   3, 192, 124,   3, 192,  56,   1, 192, 56
    ,    1, 192,  56,   1, 192, 120,   1, 240, 120,   1, 254, 124,   1, 254, 126, 1
    ,  240, 127, 128, 128,  28,  60, 240,  28,   0,   0,   0,   8,   0,   0,   0, 63
    ,    0,   0,   0, 127,   0,   0,   0, 127,   0,   0,   0, 127,   0,   0,   0, 127
    ,  128,   0,   0, 127, 128,   0,   0,  63, 128,   0,   0,  63,   0,   0,   0, 127
    ,    0,   0,   0, 124,   0,   0,   0, 252,   0,   0,   0, 254,   0,   0,   0, 254
    ,    0,   0,   1, 255,   0,   0,   1, 255,   0,   0,   1, 255, 128,   0,   1, 255
    ,  128,   0,   3, 255, 128,   0,   3, 255, 128,   0,   3, 255, 128,   0,   3, 255
    ,  128,   0,   3, 255, 128,   0,   3, 255, 240,   0,   7, 255, 255,   0,   7, 255
    ,  255, 128,   7, 255,  63, 128,  15, 255,   7, 128,  15, 254,   0,   0,  15, 254
    ,    0,   0,  15, 254,   0,   0,  31, 255,   0,   0,  31, 255,   0,   0,  63, 255
    ,  128,   0,  63, 255, 128,   0,  59, 255, 192,   0,   3, 255, 192,   0,   3, 255
    ,  192,   0,   3, 255, 192,   0,   3, 239, 192,   0,   3, 231, 192,   0,   7, 199
    ,  224,   0,   7, 195, 224,   0,   7, 195, 224,   0,  15, 129, 240,   0,  31, 129
    ,  240,   0,  31,   1, 240,   0,  30,   1, 248,   0,  62,   1, 248,   0,  62, 0
    ,  248,   0,  62,   0, 252,   0,  60,   0, 124,   0,  60,   0,  60,   0,  56, 0
    ,   62,   0,  56,   0,  30,   0, 112,   0,  15, 240, 120,   0,   7, 240, 248, 0
    ,    7, 240, 252,   0,   7, 128, 126,   0,   7, 0
};

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif
