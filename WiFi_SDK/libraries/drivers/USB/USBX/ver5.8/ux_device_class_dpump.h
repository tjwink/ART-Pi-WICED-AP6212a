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
/**   Device Data Pump Class                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_device_class_dpump.h                             PORTABLE C      */ 
/*                                                           5.8          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Thierry Giron, Express Logic Inc.                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX device dpump class.                                            */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  07-01-2007     TCRG                     Initial Version 5.0           */ 
/*  11-11-2008     TCRG                     Modified comment(s), added    */ 
/*                                            data pump class structure,  */ 
/*                                            and added read/write APIs,  */ 
/*                                            resulting in  version 5.2   */ 
/*  07-10-2009     TCRG                     Modified comment(s), and      */ 
/*                                            added trace logic,          */ 
/*                                            resulting in version 5.3    */ 
/*  06-13-2010     TCRG                     Modified comment(s),          */ 
/*                                            resulting in version 5.4    */ 
/*  09-01-2011     TCRG                     Modified comment(s),          */ 
/*                                            resulting in version 5.5    */ 
/*  10-10-2012     TCRG                     Modified comment(s), and      */ 
/*                                            allow for change of         */ 
/*                                            alternate settings,         */ 
/*                                            resulting in version 5.6    */ 
/*  06-01-2014     TCRG                     Modified comment(s),          */ 
/*                                            resulting in version 5.7    */ 
/*  06-01-2017     TCRG                     Modified comment(s),          */ 
/*                                            resulting in version 5.8    */ 
/*                                                                        */ 
/**************************************************************************/ 

#ifndef UX_DEVICE_CLASS_DPUMP_H
#define UX_DEVICE_CLASS_DPUMP_H


/* Define Storage Class USB Class constants.  */

#define UX_SLAVE_CLASS_DPUMP_CLASS                              0x99
#define UX_SLAVE_CLASS_DPUMP_SUBCLASS                           0x99
#define UX_SLAVE_CLASS_DPUMP_PROTOCOL                           0x99

/* Define Data Pump Class packet equivalences.  */
#define UX_DEVICE_CLASS_DPUMP_PACKET_SIZE                       128


/* Define Slave DPUMP Class Calling Parameter structure */

typedef struct UX_SLAVE_CLASS_DPUMP_PARAMETER_STRUCT
{
    VOID                    (*ux_slave_class_dpump_instance_activate)(VOID *);
    VOID                    (*ux_slave_class_dpump_instance_deactivate)(VOID *);

} UX_SLAVE_CLASS_DPUMP_PARAMETER;

/* Define Slave Data Pump Class structure.  */

typedef struct UX_SLAVE_CLASS_DPUMP_STRUCT
{
    UX_SLAVE_INTERFACE                  *ux_slave_class_dpump_interface;
    UX_SLAVE_CLASS_DPUMP_PARAMETER      ux_slave_class_dpump_parameter;
    UX_SLAVE_ENDPOINT                   *ux_slave_class_dpump_bulkin_endpoint;
    UX_SLAVE_ENDPOINT                   *ux_slave_class_dpump_bulkout_endpoint;
    ULONG                               ux_slave_class_dpump_alternate_setting;
    

} UX_SLAVE_CLASS_DPUMP;

/* Define Device Data Pump Class prototypes.  */

UINT    _ux_device_class_dpump_initialize(UX_SLAVE_CLASS_COMMAND *command);
UINT    _ux_device_class_dpump_activate(UX_SLAVE_CLASS_COMMAND *command);
UINT    _ux_device_class_dpump_deactivate(UX_SLAVE_CLASS_COMMAND *command);
UINT    _ux_device_class_dpump_entry(UX_SLAVE_CLASS_COMMAND *command);
UINT    _ux_device_class_dpump_read(UX_SLAVE_CLASS_DPUMP *dpump, UCHAR *buffer, 
                                ULONG requested_length, ULONG *actual_length);
UINT    _ux_device_class_dpump_write(UX_SLAVE_CLASS_DPUMP *dpump, UCHAR *buffer, 
                                ULONG requested_length, ULONG *actual_length);
UINT    _ux_device_class_dpump_change(UX_SLAVE_CLASS_COMMAND *command);
                                
/* Define Device DPUMP Class API prototypes.  */

#define ux_device_class_dpump_entry                               _ux_device_class_dpump_entry
#define ux_device_class_dpump_read                                _ux_device_class_dpump_read
#define ux_device_class_dpump_write                               _ux_device_class_dpump_write

#endif
