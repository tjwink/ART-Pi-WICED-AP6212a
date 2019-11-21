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

/**
 * @file logger.h
 *
 * flexible logging macros for console output. These macros will be disabled when building
 * in RELEASE mode so that the code from printf is NOT part of the binary app.
 *
 * Copyright (C) 2012 Broadcom Corporation
 *
 * $Id$
 */
#ifndef _H_LOGGER_H_
#define _H_LOGGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>


#define LOG_ALWAYS  ( (int8_t) ( 0 ) )
#define LOG_DEFAULT ( (int8_t) ( 0 ) )
#define LOG_LOW     ( (int8_t) ( 1 ) )
#define LOG_MEDIUM  ( (int8_t) ( 2 ) )
#define LOG_HIGH    ( (int8_t) ( 3 ) )
#define LOG_MAX     ( (int8_t) ( 4 ) )
#define LOG_ALL     ( (int8_t) ( 5 ) )
#define LOG_NONE    ( (int8_t) ( 255 ) )


typedef struct log_
{
    FILE* log_file;
} log_t;


/*
 * Note: these are compiled only in debug,
 *       i.e. when the NDEBUG define is NOT defined.
 */
#if !defined( DEBUG )

/*
 * FLAG that tells the status of the LOGGER
 */
    #define LOGGER_ENABLED ( 0 )

/*
 * EMPTY wrapping macros
 */
    #define LOG_MSG( verbose, log_lvl, ... ) { }
    #define LOG_ERR( verbose, log_lvl, ... ) { }

#else // _DEBUG

/*
 * FLAG that tells the status of the LOGGER
 */
    #define LOGGER_ENABLED ( 1 )

/*
 * WRAPPING macros
 */
    #define LOG_MSG( verbose, log_lvl, ... )  \
    do                                        \
    {                                         \
        if ( ( verbose ) >= ( log_lvl ) ) {   \
            fprintf( stdout, __VA_ARGS__ ); } \
    } while ( 0 );


    #define LOG_ERR( verbose, log_lvl, ... )  \
    do                                        \
    {                                         \
        if ( ( verbose ) >= ( log_lvl ) ) {   \
            fprintf( stdout, __VA_ARGS__ ); } \
    } while ( 0 );
#endif /* NDEBUG */

#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /* _H_LOGGER_H_ */
