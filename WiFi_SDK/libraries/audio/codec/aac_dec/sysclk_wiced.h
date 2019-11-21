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
 * @file sysclk.h
 *
 */

#ifndef _H_SYSCLK_WICED_H_
#define _H_SYSCLK_WICED_H_

#ifdef __cplusplus
extern "C" {
#endif


/*
 * USLEEP
 */
#define uSLEEP1us         ( 1 )
#define uSLEEP10us       ( 10 )
#define uSLEEP100us     ( 100 )

#define uSLEEP1ms      ( 1000 )
#define uSLEEP10ms    ( 10000 )
#define uSLEEP100ms   ( 10000 )

#define uSLEEP1s    ( 1000000 )

/* audiopcm does not use usleep other than wait for loop control,
 * so rounding to the lowest ms is enough accuracy
 */
#define SYSCLK_USLEEP( usec )                                   \
    do                                                          \
    {                                                           \
        if ( ( usec / 1000 ) > 0 ) {                            \
            wiced_rtos_delay_milliseconds( ( usec / 1000 ) ); } \
    } while ( 0 );



#ifdef __cplusplus
} /* extern "C" */
#endif
#endif /*_H_SYSCLKC_WICED_H_ */
