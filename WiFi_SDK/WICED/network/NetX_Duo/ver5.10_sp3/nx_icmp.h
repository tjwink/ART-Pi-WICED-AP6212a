/**************************************************************************/
/*                                                                        */
/*            Copyright (c) 1996-2016 by Express Logic Inc.               */
/*                                                                        */
/*  This software is copyrighted by and is the sole property of Express   */
/*  Logic, Inc.  All rights, title, ownership, or other interests         */
/*  in the software remain the property of Express Logic, Inc.  This      */
/*  software may only be used in accordance with the corresponding        */
/*  license agreement.  Any unauthorized use, duplication, transmission,  */
/*  distribution, or disclosure of this software is expressly forbidden.  */
/*                                                                        */
/*  This Copyright notice may not be removed or modified without prior    */
/*  written consent of Express Logic, Inc.                                */
/*                                                                        */
/*  Express Logic, Inc. reserves the right to modify this software        */
/*  without notice.                                                       */
/*                                                                        */
/*  Express Logic, Inc.                     info@expresslogic.com         */
/*  11423 West Bernardo Court               http://www.expresslogic.com   */
/*  San Diego, CA  92127                                                  */
/*                                                                        */
/**************************************************************************/


/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** NetX Component                                                        */
/**                                                                       */
/**   Internet Control Message Protocol (ICMP)                            */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/
/*                                                                        */
/*  COMPONENT DEFINITION                                   RELEASE        */
/*                                                                        */
/*    nx_icmp.h                                           PORTABLE C      */
/*                                                           5.10         */
/*  AUTHOR                                                                */
/*                                                                        */
/*    William E. Lamie, Express Logic, Inc.                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file defines the NetX Internet Control Message Protocol (ICMP) */
/*    component, including all data types and external references.  It is */
/*    assumed that nx_api.h and nx_port.h have already been included.     */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  12-12-2005     William E. Lamie         Initial Version 5.0           */
/*  08-09-2007     William E. Lamie         Modified comment(s),          */
/*                                            resulting in version 5.1    */
/*  12-30-2007     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.2    */
/*  08-03-2009     William E. Lamie         Modified comment(s),          */
/*                                            resulting in version 5.3    */
/*  11-23-2009     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.4    */
/*  06-01-2010     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.5    */
/*  10-10-2011     Yuxin Zhou               Modified comment(s), added    */
/*                                            an API to handle ping       */
/*                                            in a multihome systems,     */
/*                                            resulting in version 5.6    */
/*  01-31-2013     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.7    */
/*  01-12-2015     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.8    */
/*  02-22-2016     Yuxin Zhou               Modified comment(s), added    */
/*                                            function declarations,      */
/*                                            exposed external functions, */
/*                                            resulting in version 5.9    */
/*  05-10-2016     Yuxin Zhou               Modified comment(s), and      */
/*                                            removed unused code,        */
/*                                            resulting in version 5.10   */
/*                                                                        */
/**************************************************************************/

#ifndef NX_ICMP_H
#define NX_ICMP_H

VOID _nx_icmp_packet_receive(NX_IP *ip_ptr, NX_PACKET *packet_ptr);
VOID _nx_icmp_queue_process(NX_IP *ip_ptr);
VOID _nx_icmp_packet_process(NX_IP *ip_ptr, NX_PACKET *packet_ptr);

UINT _nxd_icmp_ping(NX_IP *ip_ptr, NXD_ADDRESS *ip_address,
                    CHAR *data_ptr, ULONG data_size,
                    NX_PACKET **response_ptr, ULONG wait_option);
UINT _nxd_icmp_source_ping(NX_IP *ip_ptr, NXD_ADDRESS *ip_address,
                           UINT address_index, CHAR *data_ptr, ULONG data_size,
                           NX_PACKET **response_ptr, ULONG wait_option);

UINT _nxd_icmp_enable(NX_IP *ip_ptr);

UINT _nxde_icmp_ping(NX_IP *ip_ptr, NXD_ADDRESS *ip_address,
                     CHAR *data_ptr, ULONG data_size,
                     NX_PACKET **response_ptr, ULONG wait_option);
UINT _nxde_icmp_source_ping(NX_IP *ip_ptr, NXD_ADDRESS *ip_address,
                            UINT address_index, CHAR *data_ptr, ULONG data_size,
                            NX_PACKET **response_ptr, ULONG wait_option);
UINT _nxde_icmp_enable(NX_IP *ip_ptr);

#include "nx_icmpv4.h"
#include "nx_icmpv6.h"

#endif

