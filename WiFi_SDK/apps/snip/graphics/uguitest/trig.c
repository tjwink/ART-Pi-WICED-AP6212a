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

#include "trig.h"


/* float fast computation for SIN/COS with quadratic approximation */

/*
 * basically two parabolas for both curves,
 * good enough for simple visual effects
 */
float pseudo_sin( float t )
{
    float   y = 0;
    float   xx_tmp = 0;

    /* limit input infinite domain to -PI:PI */
    int32_t period = (int32_t) ( t / PSEUDO_PI );

    float   x = ( t - ( period * PSEUDO_PI ) );

    /* swap sign on odd parabola cyclic periods */
    x = ( period & 0x01 ) ? ( -x ) : x;

    /* equations
     * ( x < 0 )
     *  y = 1.27323954 * x + .405284735 * xx_tmp;
     * ( x > 0 )
     *  y = 1.27323954 * x - 0.405284735 * xx_tmp;
     */

    y = 1.27323954 * x;

    xx_tmp = .405284735 * x * x;

    y = ( x < 0 ) ? ( y + xx_tmp ) : ( y - xx_tmp );

    return y;
}


float pseudo_cos( float t )
{
    return pseudo_sin( t + PSEUDO_PI / 2 );
}
