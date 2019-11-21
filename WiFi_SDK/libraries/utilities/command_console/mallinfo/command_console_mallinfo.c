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

/* For IAR, the <malloc.h> is implemented with advanced
 * heap handling which is defined in <iar_dlmalloc.h>.
 * The corresponding function name has a __iar_dlxxx prefix
 */
#if defined( __GNUC__ )
#include <malloc.h>
#define MALLINFO mallinfo
#elif defined ( __IAR_SYSTEMS_ICC__ )
#include <iar_dlmalloc.h>
#define MALLINFO __iar_dlmallinfo
#endif

#include <stdio.h>
#include "command_console.h"

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

/*!
 ******************************************************************************
 * Print memory allocation information.
 *
 * @param[in] argc  Unused.
 * @param[in] argv  Unused.
 *
 * @return    Console error code indicating if the command ran correctly.
 */

int malloc_info_command( int argc, char *argv[] )
{
    volatile struct mallinfo mi = MALLINFO( );

    printf( "malloc_info {\r\n"
            "\tarena:   \t%5d;\t/* total space allocated from system */\r\n", mi.arena );
    printf( "\tordblks: \t%5d;\t/* number of non-inuse chunks */\r\n", mi.ordblks );
    printf( "\tsmblks:  \t%5d;\t/* unused -- always zero */\r\n", mi.smblks );
    printf( "\thblks:   \t%5d;\t/* number of mmapped regions */\r\n", mi.hblks );
    printf( "\thblkhd:  \t%5d;\t/* total space in mmapped regions */\r\n", mi.hblkhd );
    printf( "\tusmblks: \t%5d;\t/* unused -- always zero */\r\n", mi.usmblks );
    printf( "\tfsmblks: \t%5d;\t/* unused -- always zero */\r\n", mi.fsmblks );
    printf( "\tuordblks:\t%5d;\t/* total allocated space */\r\n", mi.uordblks );
    printf( "\tfordblks:\t%5d;\t/* total non-inuse space */\r\n", mi.fordblks );
    printf( "\tkeepcost:\t%5d;\t/* top-most, releasable (via malloc_trim) space */\r\n"
            "};\r\n", mi.keepcost );

    printf( "******Mallinfo summary******\r\n");
    printf( "Total heap size: %d bytes\r\n", mi.arena );
    printf( "Memory in use: %d bytes\r\n", mi.usmblks + mi.uordblks );

    return ERR_CMD_OK;
} /* malloc_info_command */


/*!
 ******************************************************************************
 * Print dynamic heap information.
 *
 * @param[in] argc  Unused.
 * @param[in] argv  Unused.
 *
 * @return    Console error code indicating if the command ran correctly.
 */

int heap_info_command( int argc, char* argv[] )
{
    extern unsigned char _heap[];
    extern unsigned char _eheap[];
    extern unsigned char *sbrk_heap_top;
    volatile struct mallinfo mi = MALLINFO();

    printf("sbrk heap size:    %7lu\n", (uint32_t)_eheap - (uint32_t)_heap);
    printf("sbrk current free: %7lu \n", (uint32_t)_eheap - (uint32_t)sbrk_heap_top);

    printf("malloc arena:      %7d\n", mi.arena);
    printf("malloc allocated:  %7d\n", mi.uordblks);
    printf("malloc free:       %7d\n", mi.fordblks);

    printf("\nTotal free memory: %7lu\n", mi.fordblks + (uint32_t)_eheap - (uint32_t)sbrk_heap_top);

    return ERR_CMD_OK;
} /* heap_info_command */
