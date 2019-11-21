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

/** @file
 *
 * Entropy Application
 *
 * Features demonstrated
 *  - How to capture random numbers from WICED rand() function and compute entropy
 *  - How to enable and use the different type of random number generators
 * Application Instructions
 *
 *   Open apps/test/entropy.mk and configure options wanted to be compiled into the app.
 *
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 *   When the application runs, it will compute the entropy of the PRNG(s)
 *   for N_RANDOM_BYTES samples (as one can define below) and then
 *   print the results to the console.
 *
 *   For more information, see also: http://www.fourmilab.ch/random/
 *
 *
 */

#include "wiced.h"
#include "wiced_crypto.h"

#include "iso8859.h"
#include "randtest.h"
#include "ctype.h"
#include "math.h"

/******************************************************
 *                      Macros
 ******************************************************/
//Note: Fortuna PRNG will likely break at 64Kbit, due to not having enough heap
// #define N_RANDOM_BYTES 4096 /* 32Kbit */

#define N_RANDOM_BYTES 64 /* 512-bit */

// #define N_RANDOM_BYTES 32 /* 256-bit */
// #define N_RANDOM_BYTES 16 /* 128-bit */

/******************************************************
 *                    Constants
 ******************************************************/

#define UPDATE  "January 28th, 2008"

#ifndef FALSE
    #define FALSE 0
#endif

#ifndef TRUE
    #define TRUE  1
#endif

#ifdef M_PI
    #define PI   M_PI
#else
    #define PI   3.14159265358979323846
#endif


/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

int    i, oc, opt;
long   ccount[ 256 ];     /* Bins to count occurrences of values */
long   totalc;        /* Total character count */
char*  samp;
double montepi, chip, scc, ent, mean, chisq;



int    counts = FALSE,    /* Print character counts */
       fold = FALSE,      /* Fold upper to lower */
       binary = FALSE,    /* Treat input as a bitstream */
       terse = FALSE;     /* Terse (CSV format) output */


/******************************************************
 *               Function Definitions
 ******************************************************/
extern double pochisq( const double ax, const int df );

static wiced_result_t test_current_prng( void );
static void test_run_print_result( wiced_result_t r );

/* Tell user how things went */
static void test_run_print_result( wiced_result_t r )
{
    if ( WICED_SUCCESS == r )
    {
        printf("Run succeeded\n");
    }
    else
    {
        printf("Run failed. Error %d\n", (int)r);
    }
}

/* Get correct prng started, then run them */
void application_start( )
{
    /* seed the PRNG via wlan */
    {
        wiced_init( );
    }

    printf("-------\nDefault PRNG:\n--------\n");
    test_current_prng( );

#ifdef WICED_SECURE_PRNG_FORTUNA_ENABLE
    {
        wiced_result_t r;

        r = wiced_crypto_use_secure_prng( );
        printf( "\n-------\nSecure PRNG:\n--------\n" );
        if ( WICED_SUCCESS == r )
        {
            /* get seed for entropy for the algorithm */

            /* now run through same */
            test_current_prng( );
        }
        else
        {
            printf( "Unable to enable secure PRNG. %s\n", ( WICED_OUT_OF_HEAP_SPACE == r ) ? "Out of memory" : "Error" );
        }

        /* !!!Remember to switch back to regular PRNG after done with secure PRNG, as secure PRNG takes around 6K of heap!!! */
        r = wiced_crypto_use_default_prng( );
    }
#endif /* WICED_SECURE_PRNG_FORTUNA_ENABLE */

    printf( "\nDone - press reset on board to continue!\n" );
    while ( 1 )
    {
    }
}

static wiced_result_t test_current_prng( void )
{
    wiced_result_t r;

    uint8_t        rb[ N_RANDOM_BYTES ];

    totalc = 0;

    samp = binary ? "bit" : "byte";
    memset( ccount, 0, sizeof ccount );

    /* Initialise for calculations */
    rt_init( binary );

    printf( "\n\nRANDOM TEST (%d bytes)\n", N_RANDOM_BYTES );

    if ( WICED_SUCCESS != ( r = wiced_crypto_get_random( rb, N_RANDOM_BYTES ) ) )
    {
        printf( "error: %d - wiced_crypto_get_random", r );
    }

    /* update bins ... */
    for ( i = 0; i < N_RANDOM_BYTES; i++ )
    {
        unsigned char ocb;

        if ( fold && isISOalpha( rb[ i ] ) && isISOupper( rb[ i ] ) )
        {
            rb[ i ] = toISOlower( rb[ i ] );
        }
        ocb = (unsigned char) rb[ i ];

        totalc += binary ? 8 : 1;
        if ( binary )
        {
            int           b;
            unsigned char ob = rb[ i ];
            for ( b = 0; b < 8; b++ )
            {
                ccount[ ob & 1 ]++;
                ob >>= 1;
            }
        }
        else
        {
            ccount[ ocb ]++;      /* Update counter for this bin */
        }
        rt_add( &ocb, 1 );
    }

    /* Complete calculation and return sequence metrics */
    rt_end( &ent, &chisq, &mean, &montepi, &scc );


    if ( terse )
    {
        printf( "0,File-%ss,Entropy,Chi-square,Mean,Monte-Carlo-Pi,Serial-Correlation\n",
                binary ? "bit" : "byte" );
        printf( "1,%ld,%f,%f,%f,%f,%f\n",
                totalc, ent, chisq, mean, montepi, scc );
    }

    /* Calculate probability of observed distribution occurring from
     * the results of the Chi-Square test */

    chip = pochisq( chisq, ( binary ? 1 : 255 ) );

    /* Print bin counts if requested */

    if ( counts )
    {
        if ( terse )
        {
            printf( "2,Value,Occurrences,Fraction\n" );
        }
        else
        {
            printf( "Value Char Occurrences Fraction\n" );
        }
        for ( i = 0; i < ( binary ? 2 : 256 ); i++ )
        {
            if ( terse )
            {
                printf( "3,%d,%ld,%f\n", i,
                        ccount[ i ], ( (double) ccount[ i ] / totalc ) );
            }
            else
            {
                if ( ccount[ i ] > 0 )
                {
                    printf( "%3d   %c   %10ld   %f\n", i,

                            /* The following expression shows ISO 8859-1
                             * Latin1 characters and blanks out other codes.
                             * The test for ISO space replaces the ISO
                             * non-blanking space (0xA0) with a regular
                             *         ASCII space, guaranteeing it's rendered
                             *         properly even when the font doesn't contain
                             * that character, which is the case with many
                             * X fonts. */
                            ( !isISOprint( i ) || isISOspace( i ) ) ? ' ' : i,
                            ccount[ i ], ( (double) ccount[ i ] / totalc ) );
                }
            }
        }
        if ( !terse )
        {
            printf( "\nTotal:    %10ld   %f\n\n", totalc, 1.0 );
        }
    }

    /* Print calculated results */

    if ( !terse )
    {
        printf( "Entropy = %f bits per %s.\n",            ent,    samp );
        printf( "\nOptimum compression would reduce the size\n" );
        printf( "of this %ld %s file by %d percent.\n\n", totalc, samp,
                (short) ( ( 100 * ( ( binary ? 1 : 8 ) - ent ) /
                            ( binary ? 1.0 : 8.0 ) ) ) );
        printf(
            "Chi square distribution for %ld samples is %1.2f, and randomly\n",
            totalc, chisq );
        if ( chip < 0.0001 )
        {
            printf( "would exceed this value less than 0.01 percent of the times.\n\n" );
        }
        else if ( chip > 0.9999 )
        {
            printf( "would exceed this value more than than 99.99 percent of the times.\n\n" );
        }
        else
        {
            printf( "would exceed this value %1.2f percent of the times.\n\n",
                    chip * 100 );
        }
        printf(
            "Arithmetic mean value of data %ss is %1.4f (%.1f = random).\n",
            samp, mean, binary ? 0.5 : 127.5 );
        printf( "Monte Carlo value for Pi is %1.9f (error %1.2f percent).\n",
                montepi, 100.0 * ( fabs( PI - montepi ) / PI ) );
        printf( "Serial correlation coefficient is " );
        if ( scc >= -99999 )
        {
            printf( "%1.6f (totally uncorrelated = 0.0).\n", scc );
        }
        else
        {
            printf( "undefined (all values equal!).\n" );
        }
    }

    test_run_print_result( r );

    return r;
}
