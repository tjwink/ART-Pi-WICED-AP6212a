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

/* libFLAC - Free Lossless Audio Codec library
 * Copyright (C) 2012-2014  Xiph.org Foundation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * - Neither the name of the Xiph.org Foundation nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* This is the prefered location of all CPP hackery to make $random_compiler
 * work like something approaching a C99 (or maybe more accurately GNU99)
 * compiler.
 *
 * It is assumed that this header will be included after "config.h".
 */

#ifndef FLAC__SHARE__COMPAT_H
#define FLAC__SHARE__COMPAT_H

#if defined _WIN32 && !defined __CYGWIN__
/* where MSVC puts unlink() */
    # include <io.h>
#else
    # include <unistd.h>
#endif

#if defined _MSC_VER || defined __BORLANDC__ || defined __MINGW32__
    #include <sys/types.h>      /* for off_t */
    #define FLAC__off_t __int64 /* use this instead of off_t to fix the 2 GB limit */
    #if !defined __MINGW32__
        #define fseeko _fseeki64
        #define ftello _ftelli64
    #else /* MinGW */
        #if !defined( HAVE_FSEEKO )
            #define fseeko fseeko64
            #define ftello ftello64
        #endif
    #endif
#else
    #define FLAC__off_t off_t
#endif

#if HAVE_INTTYPES_H
    #define __STDC_FORMAT_MACROS
    #include <inttypes.h>
#endif

#if defined( _MSC_VER )
    #define strtoll _strtoi64
    #define strtoull _strtoui64
#endif

#if defined( _MSC_VER )
    #define inline __inline
#endif

#if defined __INTEL_COMPILER || ( defined _MSC_VER && defined _WIN64 )
/* MSVS generates VERY slow 32-bit code with __restrict */
    #define flac_restrict __restrict
#elif defined __GNUC__
    #define flac_restrict __restrict__
#else
    #define flac_restrict
#endif

#define FLAC__U64L( x ) x ## ULL

#if defined _MSC_VER || defined __BORLANDC__ || defined __MINGW32__
    #define FLAC__STRCASECMP stricmp
    #define FLAC__STRNCASECMP strnicmp
#else
    #define FLAC__STRCASECMP strcasecmp
    #define FLAC__STRNCASECMP strncasecmp
#endif

#if defined _MSC_VER || defined __MINGW32__ || defined __CYGWIN__ || defined __EMX__
    #include <io.h>     /* for _setmode(), chmod() */
    #include <fcntl.h>  /* for _O_BINARY */
#else
    #include <unistd.h> /* for chown(), unlink() */
#endif

#if defined _MSC_VER || defined __BORLANDC__ || defined __MINGW32__
    #if defined __BORLANDC__
        #include <utime.h>     /* for utime() */
    #else
        #include <sys/utime.h> /* for utime() */
    #endif
#else
    #include <sys/types.h> /* some flavors of BSD (like OS X) require this to get time_t */
    #include <utime.h>     /* for utime() */
#endif

#if defined _MSC_VER
    #  if _MSC_VER >= 1600
/* Visual Studio 2010 has decent C99 support */
        #    include <stdint.h>
        #    define PRIu64 "llu"
        #    define PRId64 "lld"
        #    define PRIx64 "llx"
    #  else
        #    include <limits.h>
        #    ifndef UINT32_MAX
            #      define UINT32_MAX _UI32_MAX
        #    endif
typedef unsigned __int64 uint64_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int16 uint16_t;
typedef unsigned __int8  uint8_t;
typedef __int64          int64_t;
typedef __int32          int32_t;
typedef __int16          int16_t;
typedef __int8           int8_t;
        #    define PRIu64 "I64u"
        #    define PRId64 "I64d"
        #    define PRIx64 "I64x"
    #  endif
#endif /* defined _MSC_VER */

#ifdef _WIN32
/* All char* strings are in UTF-8 format. Added to support Unicode files on Windows */
    #include "share/win_utf8_io.h"

    #define flac_printf printf_utf8
    #define flac_fprintf fprintf_utf8
    #define flac_vfprintf vfprintf_utf8
    #define flac_fopen fopen_utf8
    #define flac_chmod chmod_utf8
    #define flac_utime utime_utf8
    #define flac_unlink unlink_utf8
    #define flac_rename rename_utf8
    #define flac_stat _stat64_utf8

#else

    #define flac_printf printf
    #define flac_fprintf fprintf
    #define flac_vfprintf vfprintf
    #define flac_fopen fopen
    #define flac_chmod chmod
// BRCM change:   compile issue with time
// #define flac_utime utime
    #define flac_utime( a, b ) b
    #define flac_unlink unlink
    #define flac_rename rename
    #define flac_stat stat
#endif

#ifdef _WIN32
    #define flac_stat_s __stat64 /* stat struct */
    #define flac_fstat _fstat64
#else
    #define flac_stat_s stat /* stat struct */
    #define flac_fstat fstat
#endif

#ifndef M_LN2
    #define M_LN2 0.69314718055994530942
#endif
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

/* FLAC needs to compile and work correctly on systems with a normal ISO C99
 * snprintf as well as Microsoft Visual Studio which has an non-standards
 * conformant snprint_s function.
 *
 * This function wraps the MS version to behave more like the the ISO version.
 */
#include <stdarg.h>
#ifdef __cplusplus
extern "C" {
#endif
int flac_snprintf( char* str, size_t size, const char* fmt, ... );
int flac_vsnprintf( char* str, size_t size, const char* fmt, va_list va );

#ifdef __cplusplus
};
#endif
#endif /* FLAC__SHARE__COMPAT_H */
