/*
 * $Copyright Open Cypress Semiconductor$
 *
 * $Id: epivers.h.in,v 13.33 2010-09-08 22:08:53 csm Exp $
 *
*/

#ifndef _epivers_h_
#define _epivers_h_

// BIS120RC4PHY_REL_7_16_99_19

#define	EPI_MAJOR_VERSION	7

#define	EPI_MINOR_VERSION	16

#define	EPI_RC_NUMBER		99

#define	EPI_INCREMENTAL_NUMBER	19

#define	EPI_BUILD_NUMBER	0

#define	EPI_VERSION		7, 16, 99, 19

#define	EPI_VERSION_NUM		0x07106313

#define EPI_VERSION_DEV		7.16.99

/* Driver Version String, ASCII, 32 chars max */
#ifdef BCMINTERNAL
#define	EPI_VERSION_STR		"7.16.99.19 (r511984 BCMINT)"
#else
#ifdef WLTEST
#define	EPI_VERSION_STR		"7.16.99.19 (r511984 WLTEST)"
#else
#define	EPI_VERSION_STR		"7.16.99.19 (r511984)"
#endif
#endif /* BCMINTERNAL */

#endif /* _epivers_h_ */
