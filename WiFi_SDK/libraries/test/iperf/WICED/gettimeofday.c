/*
 * $ Copyright Broadcom Corporation $
 */

#if defined(WICED) && !defined(HAVE_GETTIMEOFDAY)

#if defined(HAVE_CONFIG_H) && !defined(INCLUDED_CONFIG_H_)
    /* NOTE: config.h doesn't have guard includes! */
    #define INCLUDED_CONFIG_H_
    #include "config.h"
#endif /* defined(HAVE_CONFIG_H) && !defined(INCLUDED_CONFIG_H_) */

#include "headers.h"
#include "gettimeofday.h"

#ifdef __cplusplus
extern "C"
#endif /* __cplusplus */
int wiced_gettimeofday( struct timeval* tv, void* timezone ) {
    UNUSED_PARAMETER( timezone );

    wwd_time_t time_ms = host_rtos_get_time( );
    tv->tv_sec = ( time_ms / 1000 /* to convert ms to sec */ );
    tv->tv_usec = ( time_ms - ( tv->tv_sec * 1000 ) ) * 1000;

    /* Success */
    return 0;
} // end gettimeofday

#endif /* defined(WICED) && !defined(HAVE_GETTIMEOFDAY) */
