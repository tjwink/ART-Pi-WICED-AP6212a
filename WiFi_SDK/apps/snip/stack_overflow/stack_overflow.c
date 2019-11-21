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
 * Stack Overflow Application
 *
 * Features demonstrated
 *  - How to set the application stack size
 *
 * The Application Stack
 *   The main application runs in its own thread, and just like
 *   every thread in the system, the app thread has a stack
 *   to manipulate local variables.
 *
 *   By default, the application thread stack size is set by the
 *   the #define WICED_DEFAULT_APPLICATION_STACK_SIZE in wiced_defaults.h.
 *
 *   To change the application stack size, the default stack size
 *   can be changed, or alternately, a global define can be added
 *   to the application makefile (located in the same directory as
 *   this file) as follows:
 *      #GLOBAL_DEFINES := APPLICATION_STACK_SIZE=XXXXX
 *
 * Overflowing the Stack
 *   If the application declares a local variable that requires more
 *   memory than is available on stack, the application stack will
 *   overflow.
 *
 *   This will have an indeterminate effect on the system, since
 *   the area of memory that is inadvertently overwritten may be
 *   critical to system operation, or it may be otherwise unused
 *   memory. Either way, it's a very bad idea to let this happen!
 *
 *   The demo included in this file calls a function that declares
 *   an array with 10240 bytes. Since the default application stack
 *   size is 6192 bytes, when the function is called, the stack
 *   is blown with unknown affects. In actual fact, since this app
 *   is so simple, there may not actually be any noticeable side
 *   effects -- but you can never be sure, and therein lies the
 *   trouble.
 *
 */

#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#ifndef APPLICATION_STACK_SIZE
#define APPLICATION_STACK_SIZE   WICED_DEFAULT_APPLICATION_STACK_SIZE
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

static void blow_the_stack(void);

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{

    WPRINT_APP_INFO( ( "Application stack = %u\r\n", APPLICATION_STACK_SIZE ) );

    blow_the_stack();

}

static void blow_the_stack(void)
{
    uint8_t excessively_large_array[10240];

    memset(excessively_large_array, 0, sizeof(excessively_large_array));
}
