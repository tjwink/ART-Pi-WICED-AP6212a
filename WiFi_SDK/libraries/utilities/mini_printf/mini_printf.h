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
 * Small footprint formatted print directly to stdout
 *
 * NOTES:
 *  These values are upgraded to 32-bit values when variable args are used.
 *      char        unsigned char
 *      int8_t      uint8_t
 *      int16_t     uint16_t
 *
 * Uses:
 *    platform_stdio_write()
 *    va_start(), va_arg(), va_stop()
 *    strlen()
 *    memset(), memmove().
 *
 * Supports:
 *
 *  %%      - Prints the '%' character
 *
 *  %d      - decimal number
 *              supports negative
 *              ignores 'l' modifier (all values upgraded to longs)
 *              supports leading spaces and zeros
 *
 *                  mini_printf("%d %2d %04d %ld\r\n", num1, num2, num3, num4);
 *
 *  %x      - hexadecimal number
 *              ignores 'l' modifier (all values upgraded to longs)
 *              supports leading spaces and zeros
 *
 *                  mini_printf("0x%x 0x%2x 0x%04x 0x%lx\r\n", num1, num2, num3, num4);
 *
 *  %c      - single character
 *
 *                  mini_printf("%c %c%c\r\n", 'W', 'H', 'i');
 *
 *  %s      - character string
 *
 *                  mini_printf("%s %s", my_string, "YeeHaw\r\n");
 *
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif
/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

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

int mini_printf( const char* format, ...);
int hex_dump_print(const void* data, uint16_t length, int show_ascii);

#ifdef __cplusplus
} /* extern "C" */
#endif
