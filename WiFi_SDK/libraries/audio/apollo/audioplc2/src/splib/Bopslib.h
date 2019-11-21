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

/**************************************************************************************
 * Bopslib.c
 *
 * This file contains basic operators for 16/32/64 bit fixed point math
 *
 * Author: Robert Zopf
 * Broadcom Corp.
 * 06/09/2015
 *
 * - the basic operators that can be written as a macro have both a function version
 *   and a macro version.  The macro version is included for fast execution if the
 *   code is going to be compiled and not hand ported.  When writing your code,
 *   try and use the versions of the operators that have a macro version.  This is
 *   generally the version that does not have clipping.
 * - overflow checking is included in the function versions of the operators that
 *   include macros.  Hence, you can check if there is overflow when using an
 *   operator and select the non-clipping version if no overflow is found.
 **************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

#define INLINE_OPS   1
#define OF_CHECK     0     // overflow check
#define INLINE_MATH_OPS    // INLINE OPTIMIZATION FOR MATH ROUTINES
#ifdef DEBUG
    #undef INLINE_MATH_OPS // turn off inline operation in debug mode
#endif

#define MAX_32T ( (int32_t) 0x7fffffffL )
#define MIN_32T ( (int32_t) 0x80000000L )
#define MAX_16T ( (int16_t) 0x7fff )
#define MIN_16T ( (int16_t) 0x8000 )
#define MAX64_T ( (int64_t) 0x7fffffffffffffff )
#define MIN64_T ( (int64_t) 0x8000000000000000 )

#if defined (__GNUC__) && (__GNUC__ >= 7)
#define __inline static __inline
#endif /* #if defined (__GNUC__) && (__GNUC__ >= 7) */

// These ones are not inlined
#ifndef INLINE_MATH_OPS
int16_t norm( int16_t in16 );

#else
__inline int16_t clzs( int16_t x )
{   /* This implementation assumes that short_t is at most 32 bits. */
    int32_t xl = ( (int32_t) ( x ) );
    int16_t lz = 0;

    ( xl >> 8 ) ? ( xl >>= 8 ) : ( lz += 8 );
    ( xl >> 4 ) ? ( xl >>= 4 ) : ( lz += 4 );
    ( xl >> 2 ) ? ( xl >>= 2 ) : ( lz += 2 );
    ( xl >> 1 ) ? ( xl >>= 1 ) : ( lz += 1 );
    if ( xl == 0 )
    {
        lz += 1;
    }
    return lz;
}


__inline int16_t norm( int16_t x )
{
    return clzs( ( x << 1 ) ^ x ) & 15;
}
#endif
int16_t LL_norm( int64_t in64 );

#ifndef INLINE_MATH_OPS
int32_t L_addc( int32_t x32, int32_t y32 );

#else
/* 32-bit add with clipping */
__inline int32_t L_addc( int32_t x32, int32_t y32 )
{
    int32_t res32;

    res32 = x32 + y32;
    if ( ( x32 < 0 ) && ( y32 < 0 ) && ( res32 > 0 ) )
    {
        res32 = MIN_32T;
    }
    else if ( ( x32 > 0 ) && ( y32 > 0 ) && ( res32 < 0 ) )
    {
        res32 = MAX_32T;
    }
    return res32;
}
#endif // #ifndef INLINE_MATH_OPS
#ifndef INLINE_MATH_OPS
int32_t L_absolute( int32_t in32 );

#else
__inline int32_t L_absolute( int32_t in32 )
{
    int32_t out32;

    out32 = in32;
    if ( in32 == MIN_32T )
    {
        out32 = MAX_32T;
    }
    else if ( in32 < 0 )
    {
        out32 = -in32;
    }
    return out32;
}
#endif // INLINE_MATH_OPS
int16_t absolute( int16_t in16 );
int32_t L_subc( int32_t x32, int32_t y32 );
int16_t addc( int16_t x16, int16_t y16 );
int32_t L_multc( int16_t x16, int16_t y16 );
int32_t L_msuc( int32_t mac32, int16_t x16, int16_t y16 );
int32_t L_shlpc( int32_t in32, int16_t shft /* + only */ );
int32_t L_shlc( int32_t in32, int16_t shft );
int32_t L_shrc( int32_t in32, int16_t shft );
void    gethilo( int32_t in32, int16_t* hi16, int16_t* lo16 );
int16_t L_norm( int32_t in32 );
int16_t rnd( int32_t in32 );

// int32_t L_shrp_r( int32_t in_32, int16_t shft /* + only */ );
int16_t shlpc( int16_t in16, int16_t shft /* + only */ );

#if INLINE_OPS
    #define  LL_mac( acc, x, y )                  ( acc + ( ( (int64_t) ( x ) ) * ( y ) ) )
    #define  mult32_16( in32, in16 )            ( (int32_t) ( ( (int64_t) ( in32 ) * (int64_t) ( in16 ) ) >> 15 ) )
    #define  mult32_16s( in32, in16 )           ( (int32_t) ( ( (int64_t) ( in32 ) * (int64_t) ( in16 ) ) >> 16 ) ) // ARM
    #define  L_sub( x32, y32 )                  ( ( (int32_t) ( x32 ) ) - ( y32 ) )
    #define  add( x16, y16 )                    ( ( x16 ) + ( y16 ) )
    #define  sub( x16, y16 )                    ( ( x16 ) - ( y16 ) )
    #define  L_add( x32, y32 )                  ( ( (int32_t) ( x32 ) ) + ( y32 ) )
    #define  L_shlp( in32, shft /* + only */ )  ( ( (int32_t) ( in32 ) ) << ( shft ) )
    #define  L_mult( x16, y16 )                 ( ( ( (int32_t) ( x16 ) ) * ( y16 ) ) << 1 )
    #define  L_msu( mac32, x16, y16 )           ( L_sub( mac32, L_mult( x16, y16 ) ) )
    #define  L_shrp( in32, shft /* + only */ )  ( ( in32 ) >> ( shft ) )
    #define  LL_mult0( x32, y32 )               ( ( (int64_t) ( x32 ) ) * ( y32 ) )
    #define  L_negate( in32 )                   ( ( in32 == MIN_32T ) ? MAX_32T : -in32 )
    #define  L_mult0( x16, y16 )                ( ( (int32_t) ( x16 ) ) * ( y16 ) )
    #define  mult_r( x16, y16 )                 ( (int16_t) ( ( ( ( (int32_t) ( x16 ) ) * ( y16 ) ) + 16384 ) >> 15 ) )
    #define  mult( x16, y16 )                   ( (int16_t) ( ( ( (int32_t) ( x16 ) ) * ( y16 ) ) >> 15 ) )
    #define  L_mac0( acc32, x16, y16 )          ( ( ( (int32_t) ( x16 ) ) * ( y16 ) ) + acc32 )
    #define  L_msu0( acc32, x16, y16 )          ( -( ( (int32_t) ( x16 ) ) * ( y16 ) ) + acc32 )
    #define  shlp( in16, shft )                   ( ( in16 ) << ( shft ) )
#else
int64_t LL_mac( int64_t acc, int32_t x, int32_t y );
int32_t mult32_16( int32_t in32, int16_t in16 );
int32_t mult32_16s( int32_t in32, int16_t in16 );
int32_t L_sub( int32_t x32, int32_t y32 );
int16_t add( int16_t x16, int16_t y16 );
int16_t sub( int16_t x16, int16_t y16 );
int32_t L_add( int32_t x32, int32_t y32 );
int32_t L_shlp( int32_t in32, int16_t shft /* + only */ );
int32_t L_mult( int16_t x16, int16_t y16 );
int32_t L_msu( int32_t mac32, int16_t x16, int16_t y16 );
int32_t L_shrp( int32_t in32, int16_t shft /* + only */ );
int64_t LL_mult0( int32_t x32, int32_t y32 );
int32_t L_negate( int32_t in32 );
int32_t L_mult0( int16_t x16, int16_t y16 );
int16_t mult_r( int16_t x16, int16_t y16 );
int16_t mult( int16_t x16, int16_t y16 );
int32_t L_mac0( int32_t acc32, int16_t x16, int16_t y16 );
int32_t L_msu0( int32_t acc32, int16_t x16, int16_t y16 );
int16_t shlp( int16_t in16, int16_t shft );
#endif

#ifndef INLINE_MATH_OPS
int32_t L_shrp_r( int32_t in_32, int16_t shft /* + only */ );

#else
__inline int32_t L_shrp_r( int32_t in_32, int16_t shft /* + only */ )
{
    int32_t res_32;

    res_32 = L_shrp( in_32, shft );
    if ( shft > 0 )
    {
        if ( in_32 > 0 )
        {
            if ( L_sub( in_32, L_shlp( res_32, shft ) ) >= ( L_shlp( 1, shft - 1 ) ) )
            {
                res_32 = L_add( res_32, 1 );
            }
        }
        else
        {
            if ( L_sub( in_32, L_shlp( res_32, shft ) ) > ( L_shlp( 1, shft - 1 ) ) )
            {
                res_32 = L_add( res_32, 1 );
            }
        }
    }
    return res_32;
}
#endif // #ifndef INLINE_MATH_OPS

#define sat( w32, w16 )     w16 = (Word16) w32; if ( w32 > 32767 ) { w16 = 32767; } if ( w32 < -32768 ) { w16 = -32768; }

#ifdef __cplusplus
}
#endif
