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
//#include "platform_cmsis.h"
#include <stdint.h>
#include <stdlib.h>
#include <yfuns.h>
#include <wwd_assert.h>

int errno;
extern void platform_stdio_write( const char* str, uint32_t len );
extern void platform_stdio_read( char* str, uint32_t len );

size_t __write( int handle, const unsigned char * buffer, size_t size )
{

    if ( buffer == 0 )
    {
        return 0;
    }

    platform_stdio_write( (const char*)buffer, size );

    return size;
}

size_t __read( int handle, unsigned char * buffer, size_t size )
{

    if ( buffer == 0 )
    {
        return 0;
    }

    platform_stdio_read( (char*)buffer, size );

    return size;
}

/* Stubbed for now. */
long __lseek(int handle, long offset, int whence)
{
  wiced_assert( "unimplemented", 0 != 0 );
  return -1;
}

/* Stubbed for now. */
int __open (const char *filename, int mode)
{
    wiced_assert( "unimplemented", 0 != 0 );
    return -1;
}

/* Stubbed for now. */
int __close(int handle)
{
  wiced_assert( "unimplemented", 0 != 0 );
  return 0;
}

/* Stubbed for now. */
int remove(const char * filename)
{
  wiced_assert( "unimplemented", 0 != 0 );
  return 0;
}
