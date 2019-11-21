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
 * GPIO API Application
 *
 * This application demonstrates how to use the WICED GPIO API
 * to toggle LEDs and read button states
 *
 * Features demonstrated
 *  - GPIO API
 *
 */

#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#ifndef WICED_LED2
#define WICED_LED2  WICED_LED1
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

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    wiced_bool_t led1 = WICED_FALSE;
    wiced_bool_t led2 = WICED_FALSE;
    wiced_bool_t button1_pressed;
    wiced_bool_t button2_pressed;

    /* Initialise the WICED device */
    wiced_init();

    WPRINT_APP_INFO( ( "The LEDs are flashing. Holding a button will force the corresponding LED on\n" ) );

    while ( 1 )
    {
        /* Read the state of Button 1 */
        button1_pressed = wiced_gpio_input_get( WICED_BUTTON1 ) ? WICED_FALSE : WICED_TRUE;  /* The button has inverse logic */

        if ( button1_pressed == WICED_TRUE )
        {   /* Turn LED1 on */
            wiced_gpio_output_high( WICED_LED1 );
        }
        else
        {   /* Toggle LED1 */
            if ( led1 == WICED_TRUE )
            {
                wiced_gpio_output_low( WICED_LED1 );
                led1 = WICED_FALSE;
            }
            else
            {
                wiced_gpio_output_high( WICED_LED1 );
                led1 = WICED_TRUE;
            }
        }

        /* Read the state of Button 2 */
        button2_pressed = wiced_gpio_input_get( WICED_BUTTON2 ) ? WICED_FALSE : WICED_TRUE;  /* The button has inverse logic */

        if ( button2_pressed == WICED_TRUE )
        {   /* Turn LED2 on */
            wiced_gpio_output_high( WICED_LED2 );
        }
        else
        {   /* Toggle LED2 */
            if ( led2 == WICED_TRUE )
            {
                wiced_gpio_output_low( WICED_LED2 );
                led2 = WICED_FALSE;
            }
            else
            {
                wiced_gpio_output_high( WICED_LED2 );
                led2 = WICED_TRUE;
            }
        }

        wiced_rtos_delay_milliseconds( 250 );
    }
}
