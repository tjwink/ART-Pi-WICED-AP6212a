/*
 * $Copyright Open Cypress Semiconductor$
 *
 * $Id: epivers.h.in,v 13.33 2010-09-08 22:08:53 csm Exp $
 *
*/

#ifndef _epivers_h_
#define _epivers_h_

#define	EPI_MAJOR_VERSION	7

#define	EPI_MINOR_VERSION	10

#define	EPI_RC_NUMBER		323

#define	EPI_INCREMENTAL_NUMBER	21

#define	EPI_BUILD_NUMBER	0

#define	EPI_VERSION		7, 10, 323, 21

#define	EPI_VERSION_NUM		0x070a1431

#define EPI_VERSION_DEV		7.10.323

/* Driver Version String, ASCII, 32 chars max */
#ifdef BCMINTERNAL
#define	EPI_VERSION_STR		"7.10.323.21 (r508257 BCMINT)"
#else
#ifdef WLTEST
#define	EPI_VERSION_STR		"7.10.323.21 (r508257 WLTEST)"
#else
#define	EPI_VERSION_STR		"7.10.323.21 (r508257)"
#endif
#endif /* BCMINTERNAL */

#endif /* _epivers_h_ */
