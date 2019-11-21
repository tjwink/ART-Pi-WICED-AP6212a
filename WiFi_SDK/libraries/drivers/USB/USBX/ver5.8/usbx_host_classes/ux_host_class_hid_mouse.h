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
/**   HID Mouse Client Class                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_host_class_hid_mouse.h                           PORTABLE C      */ 
/*                                                           5.8          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Thierry Giron, Express Logic Inc.                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX HID mouse class.                                               */ 
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

#ifndef UX_HOST_CLASS_HID_MOUSE_H
#define UX_HOST_CLASS_HID_MOUSE_H


/* Define HID Mouse Class constants.  */

#define UX_HOST_CLASS_HID_MOUSE_BUFFER_LENGTH                   128
#define UX_HOST_CLASS_HID_MOUSE_USAGE_ARRAY_LENGTH              64

#define UX_HOST_CLASS_HID_MOUSE_BUTTON_1                        0x00090001
#define UX_HOST_CLASS_HID_MOUSE_BUTTON_2                        0x00090002
#define UX_HOST_CLASS_HID_MOUSE_BUTTON_3                        0x00090003

#define UX_HOST_CLASS_HID_MOUSE_AXIS_X                          0x00010030
#define UX_HOST_CLASS_HID_MOUSE_AXIS_Y                          0x00010031


#define UX_HOST_CLASS_HID_MOUSE_BUTTON_1_PRESSED                0x01
#define UX_HOST_CLASS_HID_MOUSE_BUTTON_2_PRESSED                0x02
#define UX_HOST_CLASS_HID_MOUSE_BUTTON_3_PRESSED                0x04


/* Define HID Mouse Class structure.  */

typedef struct UX_HOST_CLASS_HID_MOUSE_STRUCT
{

    ULONG           ux_host_class_hid_mouse_state; 
    UX_HOST_CLASS_HID   *ux_host_class_hid_mouse_hid;
    USHORT          ux_host_class_hid_mouse_id;    
    SLONG           ux_host_class_hid_mouse_x_position;
    SLONG           ux_host_class_hid_mouse_y_position;
    ULONG           ux_host_class_hid_mouse_buttons;
} UX_HOST_CLASS_HID_MOUSE;

/* Define HID Mouse Class function prototypes.  */

UINT    _ux_host_class_hid_mouse_activate(UX_HOST_CLASS_HID_CLIENT_COMMAND *command);
VOID    _ux_host_class_hid_mouse_callback(UX_HOST_CLASS_HID_REPORT_CALLBACK *callback);
UINT    _ux_host_class_hid_mouse_deactivate(UX_HOST_CLASS_HID_CLIENT_COMMAND  *command);
UINT    _ux_host_class_hid_mouse_entry(UX_HOST_CLASS_HID_CLIENT_COMMAND *command);
UINT    _ux_host_class_hid_mouse_buttons_get(UX_HOST_CLASS_HID_MOUSE *mouse_instance, 
                                            ULONG *mouse_buttons);
UINT    _ux_host_class_hid_mouse_position_get(UX_HOST_CLASS_HID_MOUSE *mouse_instance, 
                                            SLONG *mouse_x_position, 
                                            SLONG *mouse_y_position);

/* Define HID Mouse Class API prototypes.  */

#define ux_host_class_hid_mouse_entry                       _ux_host_class_hid_mouse_entry
#define ux_host_class_hid_mouse_buttons_get                 _ux_host_class_hid_mouse_buttons_get
#define ux_host_class_hid_mouse_position_get                _ux_host_class_hid_mouse_position_get

#endif
