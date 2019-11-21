/**************************************************************************/
/*                                                                        */
/*            Copyright (c) 1996-2013 by Express Logic Inc.               */
/*                                                                        */
/*  This software is copyrighted by and is the sole property of Express   */
/*  Logic, Inc.  All rights, title, ownership, or other interests         */
/*  in the software remain the property of Express Logic, Inc.  This      */
/*  software may only be used in accordance with the corresponding        */
/*  license agreement.  Any unauthorized use, duplication, transmission,  */
/*  distribution, or disclosure of this software is expressly forbidden.  */
/*                                                                        */
/*  This Copyright notice may not be removed or modified without prior    */
/*  written consent of Express Logic, Inc.                                */
/*                                                                        */
/*  Express Logic, Inc. reserves the right to modify this software        */
/*  without notice.                                                       */
/*                                                                        */
/*  Express Logic, Inc.                     info@expresslogic.com         */
/*  11423 West Bernardo Court               http://www.expresslogic.com   */
/*  San Diego, CA  92127                                                  */
/*                                                                        */
/**************************************************************************/

#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>

int _nx_snprintf(char *str, size_t size, const char *format, ...);

int _nx_snprintf(char *str, size_t size, const char *format, ...)
{
    int retval;
    va_list args;
    va_start(args,format);
    retval = vsnprintf(str,size,format,args);
    va_end(args);
    if ( ( str != NULL ) && ( size > 0 ) )
    {
        str[ size - 1 ] = 0; /* ensure terminating null */
    }
    return retval;
}
