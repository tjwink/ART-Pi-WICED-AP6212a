/*
 * $ Copyright Broadcom Corporation $
 */

#include "headers.h"
#include "iperf_debug.h"

#if IPERF_DEBUG
//static Mutex debugprint_mutex;
static int   debugprint_mutex_init = 0;

void debug_get_mutex() {
    if (debugprint_mutex_init != 0)
        Mutex_Lock(&debugprint_mutex);
}

void debug_release_mutex() {
    if (debugprint_mutex_init != 0)
        Mutex_Unlock(&debugprint_mutex);
}
#endif /* IPERF_DEBUG */

void debug_init() {
#if IPERF_DEBUG
    Mutex_Initialize(&debugprint_mutex);
    debugprint_mutex_init = 1;
#endif /* IPERF_DEBUG */
}

void debug_destroy() {
#if IPERF_DEBUG
    Mutex_Destroy(&debugprint_mutex);
    debugprint_mutex_init = 0;
#endif /* IPERF_DEBUG */
}
