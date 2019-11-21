/*
 * $ Copyright Broadcom Corporation $
 */

#ifndef INET_H_
#define INET_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NETWORK_LwIP
#define INADDR_NONE         IP_ADDRESS( 255, 255, 255, 255 )    /* 255.255.255.255 */
#define INADDR_LOOPBACK     IP_ADDRESS( 127,   0,   0,   1 )    /* 127.0.0.1 */
#if ( !defined(ENABLE_NX_BSD_SOCKET) && !defined(ENABLE_NXD_BSD_SOCKET) )
#define INADDR_ANY          IP_ADDRESS(   0,   0,   0,   0 )    /* 0.0.0.0 */
#endif
#define INADDR_BROADCAST    IP_ADDRESS( 255, 255, 255, 255 )    /* 255.255.255.255 */
#endif

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* INET_H_ */
