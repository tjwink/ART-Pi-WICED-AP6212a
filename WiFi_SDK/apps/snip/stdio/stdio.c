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
 * STDIO Application
 *
 * Features demonstrated
 *  - How to use the UART for STDIO operations
 *
 * Application Instructions
 *   Connect a PC terminal to the serial port of the WICED Eval board,
 *   then build and download the application as described in the WICED
 *   Quick Start Guide
 *
 *   When the application runs, follow the prompts printed on the terminal.
 *
 */

#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define NAME_LENGTH     20

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
    char c;
    int  name_index  = 0;
    char name[NAME_LENGTH + 1]; /* extra byte for NULL terminator */

    memset( name, 0, sizeof( name ) );

    wiced_core_init();

    /* Remove buffering from all std streams */
    setvbuf( stdin,  NULL, _IONBF, 0 );
    setvbuf( stdout, NULL, _IONBF, 0 );
    setvbuf( stderr, NULL, _IONBF, 0 );


    while (1)
    {
        name_index = 0;
        /* Read the users name from standard input */
        printf( "\nEnter your name (up to %d characters)\n", NAME_LENGTH );
        do
        {
           c =  getchar();
           name[name_index] = c;
           name_index++;
           name[name_index] = 0;  /* terminate string */

           printf("%c", c); /* repeat character back to user */

           if (name_index >= NAME_LENGTH )
           {
               break;
           }

        } while( c != 0x0d );

        printf( "\n\nHello %s, welcome to WICED.\n", name );

        while (1)
        {
            printf( "\n%s, Type any digit from 0 to 9:", name );
            c = getchar();

            printf("%c\n", c); /* repeat character back to user */

            if ( (c < '0') || (c > '9'))
            {
                printf( "\n\'%c\' is not a digit, %s!\n", c, name );
            }
            else
            {
                break;
            }
        }

        printf( "Your number is %d\n", c - '0' );
    }
}
