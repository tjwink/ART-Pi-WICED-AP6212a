/* logging.c - Logging facility
 *
 * Copyright (C) 2013 Henner Zeller
 *
 * This file is part of GMediaRender.
 *
 * GMediaRender is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * GMediaRender is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Library General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GMediaRender; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, 
 * MA 02110-1301, USA.
 *
 */

#define _GNU_SOURCE

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "wwd_debug.h"
#include "wiced_time.h"
#include "wiced_log.h"

#include "logging.h"
#include "config.h"

static int enable_color = 0;

static const char *const kInfoHighlight  = "\033[1mINFO  ";
static const char *const kErrorHighlight = "\033[1m\033[31mERROR ";
static const char *const kTermReset      = "\033[0m";

static const char *info_markup_start_  = "INFO  ";
static const char *error_markup_start_ = "ERROR ";
static const char *markup_end_ = "";

void Log_init(const char *filename)
{
	UNUSED_PARAMETER(filename);

	if (enable_color) {
		info_markup_start_  = kInfoHighlight;
		error_markup_start_ = kErrorHighlight;
		markup_end_         = kTermReset;
	}
}

int Log_color_allowed(void) { return enable_color; }
int Log_info_enabled(void) { return 1; }
int Log_error_enabled(void) { return 1; }

static void Log_internal(const char *markup_start, const char *category, const char *format, va_list ap)
{
    wiced_time_t   now    = 0;
    uint32_t       hrs;
    uint32_t       mins;
    uint32_t       secs;
    uint32_t       ms;

    if ( wiced_time_get_time( &now ) != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR( ( "wiced_time_get_time() failed !\n" ) );
    }

    ms   = now % 1000;
    now /= 1000;
    secs = now % 60;
    now /= 60;
    mins = now % 60;
    hrs  = (now / 60) % 24;

    wiced_log_printf("%s[%02d:%02d:%02d.%03d | %s]%s ", markup_start, hrs, mins, secs, ms, category, markup_end_);
    wiced_log_vprintf(format, ap);
    wiced_log_printf("\n");
}

void Log_info(const char *category, const char *format, ...) {
	va_list ap;

	if ( wiced_log_get_facility_level(WLF_AUDIO) < WICED_LOG_INFO )
	{
	    return;
	}

	va_start(ap, format);
	Log_internal(info_markup_start_, category, format, ap);
	va_end(ap);
}

void Log_error(const char *category, const char *format, ...) {
    va_list ap;

    if ( wiced_log_get_facility_level(WLF_AUDIO) < WICED_LOG_ERR )
    {
        return;
    }

    va_start(ap, format);
    Log_internal(error_markup_start_, category, format, ap);
    va_end(ap);
}
