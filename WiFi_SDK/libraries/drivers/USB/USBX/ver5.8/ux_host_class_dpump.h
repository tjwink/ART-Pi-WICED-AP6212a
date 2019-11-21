/**************************************************************************/ 
/*                                                                        */ 
/*            Copyright (c) 1996-2017 by Express Logic Inc.               */ 
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
/** USBX Component                                                        */ 
/**                                                                       */
/**   Host Data Pump Class                                                */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_host_class_dpump.h                               PORTABLE C      */ 
/*                                                           5.8          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Thierry Giron, Express Logic Inc.                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX demo data pump class.                                          */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  07-01-2007     TCRG                     Initial Version 5.0           */ 
/*  11-11-2008     TCRG                     Modified comment(s),          */ 
/*                                            resulting in version 5.2    */ 
/*  07-10-2009     TCRG                     Modified comment(s), and      */ 
/*                                            added trace logic,          */ 
/*                                            resulting in version 5.3    */ 
/*  06-13-2010     TCRG                     Modified comment(s),          */ 
/*                                            resulting in version 5.4    */ 
/*  09-01-2011     TCRG                     Modified comment(s),          */ 
/*                                            resulting in version 5.5    */ 
/*  10-10-2012     TCRG                     Modified comment(s),          */ 
/*                                            resulting in version 5.6    */ 
/*  06-01-2014     TCRG                     Modified comment(s),          */ 
/*                                            resulting in version 5.7    */ 
/*  06-01-2017     TCRG                     Modified comment(s),          */ 
/*                                            resulting in version 5.8    */ 
/*                                                                        */ 
/**************************************************************************/ 

#ifndef UX_HOST_CLASS_DPUMP_H
#define UX_HOST_CLASS_DPUMP_H


/* Define Data Pump Class constants.  */

#define UX_HOST_CLASS_DPUMP_CLASS_TRANSFER_TIMEOUT            300000
#define UX_HOST_CLASS_DPUMP_CLASS                             0x99
#define UX_HOST_CLASS_DPUMP_SUBCLASS                          0x99
#define UX_HOST_CLASS_DPUMP_PROTOCOL                          0x99

/* Define Data Pump Class packet equivalences.  */
#define UX_HOST_CLASS_DPUMP_PACKET_SIZE                       128

/* Define Data Pump Class Ioctl functions.  */
#define UX_HOST_CLASS_DPUMP_SELECT_ALTERNATE_SETTING          1    

/* Define Data Pump Class string constants.  */

#define    UX_HOST_CLASS_DPUMP_GENERIC_NAME                   "USB DPUMP"


/* Define Printer Class function prototypes.  */

UINT    _ux_host_class_dpump_activate(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_dpump_configure(UX_HOST_CLASS_DPUMP *dpump);
UINT    _ux_host_class_dpump_deactivate(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_dpump_endpoints_get(UX_HOST_CLASS_DPUMP *dpump);
UINT    _ux_host_class_dpump_entry(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_dpump_read (UX_HOST_CLASS_DPUMP *dpump, UCHAR *data_pointer, 
                                    ULONG requested_length, ULONG *actual_length);
UINT    _ux_host_class_dpump_write(UX_HOST_CLASS_DPUMP *dpump, UCHAR * data_pointer, 
                                    ULONG requested_length, ULONG *actual_length);
UINT    _ux_host_class_dpump_ioctl(UX_HOST_CLASS_DPUMP *dpump, ULONG ioctl_function,
                                    VOID *parameter);


#define ux_host_class_dpump_entry                               _ux_host_class_dpump_entry
#define ux_host_class_dpump_read                                _ux_host_class_dpump_read
#define ux_host_class_dpump_write                               _ux_host_class_dpump_write
#define ux_host_class_dpump_ioctl                               _ux_host_class_dpump_ioctl

#endif
