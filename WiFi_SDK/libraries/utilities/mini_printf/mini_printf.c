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

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

#include "platform.h"
#include "platform_stdio.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define NUMBER_STRING_LENGTH            16          /* will handle all 32 bit integers + ending '\0' */

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

static char number_string[ NUMBER_STRING_LENGTH ];

static int number_print(int32_t number_in, int num_digits, int leading_zeros, int base)
{
    int         i;
    uint32_t    number;
    int         negative = 0;
    int         digits_printed = 0;

    memset( number_string, 0x00, NUMBER_STRING_LENGTH);

    /* use a 32 bit unsigned number internally */
    number = (uint32_t)number_in;

    /* if negative and decimal base, put a '-' sign later */
    if ((number_in < 0) && (base == 10))
    {
        negative = 1;
        number = -number_in;
    }

    i = NUMBER_STRING_LENGTH - 1;
    number_string[i--] = '\0';
    while( i > 0 )
    {
        /* make sure we print at least 1 '0' ! */
        if( (number != 0) || (digits_printed == 0))
        {
            /* write lowest digit, then back up 1 */
            number_string[i--] = (((number % base) < 0x0a) ? ('0' + (number % base)) : (('a' - 0x0a) + (number % base)));
            digits_printed++;
        }
        else if( num_digits > 0)
        {
            number_string[i--] = (leading_zeros != 0) ? '0' : ' ';
            digits_printed++;
        }
        else if (negative != 0)
        {
            number_string[i--] = '-';
            negative = 0;
            digits_printed++;
        }
        else
        {
            break;
        }

        /* reduce by 1 digit in the given base */
        number /= base;

        num_digits--;
    }

    if (i > 0)
    {
        /* move to the start of the string number_stringer */
        memmove( &number_string[0], &number_string[i + 1], (NUMBER_STRING_LENGTH - i));
    }


    return (digits_printed + 1);
}

/* small, minimum support, formatted print */
int mini_printf( const char *format, ...)
{
    const char* percent_ptr;
    const char* curr_ptr;
    int count, precision;
    int leading_zeros, num_digits;
    int out_count;
    va_list ap;

    va_start(ap, format);

    out_count = 0;
    curr_ptr = format;
    while( (curr_ptr != NULL) && (*curr_ptr != '\0') )
    {
        leading_zeros = 0;
        num_digits = 0;

        if (*curr_ptr == '%')
        {
            percent_ptr = curr_ptr;
        }
        else
        {
            percent_ptr = (char *)strchr(curr_ptr, '%');
        }
        if (percent_ptr != NULL)
        {
            count = percent_ptr - curr_ptr;

            if (count > 0)
            {
                /* print up to the % */
                platform_stdio_write( curr_ptr, count );
                out_count += count;
            }

            count = 0;
            precision = 0; /* for defining string len using format "%.*s"  */
            percent_ptr++;  /* skip the '%' */
            /* skip the % and up to the identifier - we only handle %d, %x, %p, %c, %s
             * if there are format params for leading 0's or long vars, skip them
             */
            while( (*percent_ptr != '\0') && (*percent_ptr != '%') &&
                   (*percent_ptr != 'x')  && (*percent_ptr != 'p') &&
                   (*percent_ptr != 'X')  && (*percent_ptr != 'P') &&
                   (*percent_ptr != 'd')  &&
                   (*percent_ptr != 'c')  && (*percent_ptr != 's'))
            {
                switch(*percent_ptr)
                {
                    case '.':
                        if (*(percent_ptr + 1) == '*')
                        {
                            precision = va_arg(ap, uint32_t);
                            break;
                        }
                        break;
                    case '0':
                        if ((leading_zeros == 0) && (num_digits == 0))
                        {
                            leading_zeros = 1;
                            break;
                        }
                        /* fall through */
                    case '1':
                    case '2':
                    case '3':
                    case '4':
                    case '5':
                    case '6':
                    case '7':
                    case '8':
                    case '9':
                        num_digits *= 10;
                        num_digits += *percent_ptr - '0';
                        if (num_digits > (NUMBER_STRING_LENGTH - 1))
                        {
                            num_digits = NUMBER_STRING_LENGTH - 1;
                        }
                        break;
                    case 'l':
                        /* long_data = 1; ignore - 8, 16 bit numbers are promoted to int through ... args */
                        /* fall through */
                    default:
                        break;
                }
                percent_ptr++;
            }

            if (*percent_ptr == '%')
            {
                count = 1;
                platform_stdio_write( "%", count );
            }
            else if (*percent_ptr == 'd')
            {
                int32_t my_num = va_arg(ap, int32_t);

                count = number_print(my_num, num_digits, leading_zeros, 10);
                platform_stdio_write( number_string, count );
            }
            else if ((*percent_ptr == 'x') || (*percent_ptr == 'X') || (*percent_ptr == 'p') || (*percent_ptr == 'P'))
            {
                uint32_t my_num = va_arg(ap, uint32_t);

                if ((*percent_ptr == 'p') || (*percent_ptr == 'P'))
                {
                    /* print leading "0x" for addresses */
                    platform_stdio_write( "0x", 2 );
                }
                count = number_print(my_num, num_digits, leading_zeros, 16);
                platform_stdio_write( number_string, count );
            }
            else if (*percent_ptr == 'c')
            {
                char my_char = va_arg(ap, int);

                count = 1;
                platform_stdio_write( &my_char, count );
            }
            else if (*percent_ptr == 's')
            {
                char* my_string = va_arg(ap, char *);
                if (precision != 0)
                {
                    count = precision;
                }
                else
                {
                    count = strlen(my_string);
                }
                if (count > 0)
                {
                    platform_stdio_write( my_string, count );
                }
            }

            percent_ptr ++;    /* skip the format character */
            curr_ptr = percent_ptr;

            out_count += count;
        }
        else
        {
            count = strlen(curr_ptr);
            if (count > 0)
            {
                platform_stdio_write( curr_ptr, count );
            }
            out_count += count;

            curr_ptr = NULL;
        }
    }

    va_end(ap);

    return out_count;
}

int hex_dump_print(const void* data_ptr, uint16_t length, int show_ascii)
{
    uint8_t*  data = (uint8_t*)data_ptr;
    uint8_t* char_ptr;
    int i, count;


    if ((data == NULL) || (length == 0))
    {
        return -1;
    }

    count = 0;
    char_ptr = data;
    while (length > 0)
    {

        i = 0;
        while ((length > 0) && (i < 16))
        {
            mini_printf(" %02x", *data);
            i++;
            data++;
            length--;
            count++;
        }

        if (show_ascii != 0)
        {
            int fill = 16 - i;
            /* fill in for < 16 */
            while(fill > 0)
            {
                mini_printf("   ");
                fill--;
            }

            /* space between numbers and chars */
            mini_printf("    ");

            while (i > 0)
            {
                mini_printf("%c",(isprint(*char_ptr) ? *char_ptr : '.'));
                char_ptr++;
                i--;
            }
        }

        mini_printf("\r\n");
    }

    return count;
}
