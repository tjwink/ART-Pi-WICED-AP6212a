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
 * Copyright (C) 2000-2009  Josh Coalson
 * Copyright (C) 2011-2014  Xiph.Org Foundation
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

#ifndef FLAC__EXPORT_H
#define FLAC__EXPORT_H

/** \file include/FLAC/export.h
 *
 *  \brief
 *  This module contains #defines and symbols for exporting function
 *  calls, and providing version information and compiled-in features.
 *
 *  See the \link flac_export export \endlink module.
 */

/** \defgroup flac_export FLAC/export.h: export symbols
 *  \ingroup flac
 *
 *  \brief
 *  This module contains #defines and symbols for exporting function
 *  calls, and providing version information and compiled-in features.
 *
 *  If you are compiling with MSVC and will link to the static library
 *  (libFLAC.lib) you should define FLAC__NO_DLL in your project to
 *  make sure the symbols are exported properly.
 *
 * \{
 */

#if defined( FLAC__NO_DLL )
    #define FLAC_API

#elif defined( _MSC_VER )
    #ifdef FLAC_API_EXPORTS
        #define    FLAC_API __declspec( dllexport )
    #else
        #define FLAC_API __declspec( dllimport )
    #endif

#elif defined( FLAC__USE_VISIBILITY_ATTR )
    #define FLAC_API __attribute__ ( ( visibility( "default" ) ) )

#else
    #define FLAC_API
#endif

/** These #defines will mirror the libtool-based library version number, see
 * http://www.gnu.org/software/libtool/manual/libtool.html#Libtool-versioning
 */
#define FLAC_API_VERSION_CURRENT 11
#define FLAC_API_VERSION_REVISION 0 /**< see above */
#define FLAC_API_VERSION_AGE 3      /**< see above */

#ifdef __cplusplus
extern "C" {
#endif

/** \c 1 if the library has been compiled with support for Ogg FLAC, else \c 0. */
extern FLAC_API int FLAC_API_SUPPORTS_OGG_FLAC;

#ifdef __cplusplus
}
#endif

/* \} */
#endif
