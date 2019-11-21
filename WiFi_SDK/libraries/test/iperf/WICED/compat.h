/*
 * $ Copyright Broadcom Corporation $
 */

/**
 * @file compat.h
 * @brief An attempt to make ThreadX/NetX more compatible with POSIX functions.
 */

#ifndef THREADXNETX_COMPAT_H_
#define THREADXNETX_COMPAT_H_

#ifdef __cplusplus
extern "C" {
#endif


#include "inet.h"
#include "netdb.h"

/* Include "nx_api.h" before "sockets.h" to avoid double definition of types
 * such as ULONG/USHORT.
 */

#if defined(NETWORK_NetX)
#include "nx_api.h"
#include "NetX/sockets.h"
#include "netx_applications/dns/nx_dns.h"
#elif defined(NETWORK_NetX_Duo)
#include "nx_api.h"
#include "NetX_Duo/sockets.h"
#include "netx_applications/dns/nxd_dns.h"
#elif defined(NETWORK_LwIP)
#include "LwIP/sockets.h"
#endif



#ifdef __cplusplus
}
#endif

#ifndef socklen_t
#define socklen_t INT
#endif /* socklen_t */

#endif /* THREADXNETX_COMPAT_H_ */
