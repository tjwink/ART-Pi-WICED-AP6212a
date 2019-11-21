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
/**   HID Remote Control Class                                            */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_host_class_hid_remote_control.h                  PORTABLE C      */ 
/*                                                           5.8          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Thierry Giron, Express Logic Inc.                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX HID remote control class.                                      */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  07-01-2007     TCRG                     Initial Version 5.0           */ 
/*  07-04-2008     TCRG                     Modified comment(s),          */ 
/*                                            resulting in version 5.1    */ 
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

#ifndef UX_HOST_CLASS_HID_REMOTE_CONTROL_H
#define UX_HOST_CLASS_HID_REMOTE_CONTROL_H


/* Define HID Remote Control Class constants.  */

#define UX_HOST_CLASS_HID_REMOTE_CONTROL_BUFFER_LENGTH          128
#define UX_HOST_CLASS_HID_REMOTE_CONTROL_USAGE_ARRAY_LENGTH     64


/* Define HID Remote Control Class structure.  */

typedef struct UX_HOST_CLASS_HID_REMOTE_CONTROL_STRUCT
{

    ULONG           ux_host_class_hid_remote_control_state;    
    UX_HOST_CLASS_HID   *ux_host_class_hid_remote_control_hid;
    ULONG           *ux_host_class_hid_remote_control_usage_array;
    ULONG           *ux_host_class_hid_remote_control_usage_array_head;
    ULONG           *ux_host_class_hid_remote_control_usage_array_tail;
} UX_HOST_CLASS_HID_REMOTE_CONTROL;

/* Define HID Remote Control Class function prototypes.  */

VOID    _ux_host_class_hid_remote_control_callback(UX_HOST_CLASS_HID_REPORT_CALLBACK *callback);
UINT    _ux_host_class_hid_remote_control_activate(UX_HOST_CLASS_HID_CLIENT_COMMAND *command);
UINT    _ux_host_class_hid_remote_control_deactivate(UX_HOST_CLASS_HID_CLIENT_COMMAND *command);
UINT    _ux_host_class_hid_remote_control_entry(UX_HOST_CLASS_HID_CLIENT_COMMAND *command);
UINT    _ux_host_class_hid_remote_control_usage_get(UX_HOST_CLASS_HID_REMOTE_CONTROL *remote_control_instance, ULONG *usage, ULONG *value);

/* Define HID Keyboard Class API prototypes.  */

#define ux_host_class_hid_remote_control_entry                   _ux_host_class_hid_remote_control_entry
#define ux_host_class_hid_remote_control_usage_get               _ux_host_class_hid_remote_control_usage_get

#endif

