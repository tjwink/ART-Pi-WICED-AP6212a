/*
 * $ Copyright Broadcom Corporation $
 */

#if !defined(HAVE_USLEEP) && defined(WICED)

#include "headers.h"

int usleep(useconds_t usec) {
    return host_rtos_delay_milliseconds( usec / 1000 );
}

#endif /* !defined(HAVE_USLEEP) && defined(WICED) */
