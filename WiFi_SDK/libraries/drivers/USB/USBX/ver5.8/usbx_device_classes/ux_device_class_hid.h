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
/**   HID Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_device_class_hid.h                               PORTABLE C      */ 
/*                                                           5.8          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Thierry Giron, Express Logic Inc.                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX HID class.                                                     */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  11-11-2008     TCRG                     Initial Version 5.2           */ 
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
/*                                            added parameter structures  */ 
/*                                            and modified prototypes,    */ 
/*                                            resulting in version 5.8    */ 
/*                                                                        */ 
/**************************************************************************/ 

#ifndef UX_DEVICE_CLASS_HID_H
#define UX_DEVICE_CLASS_HID_H

/* Define HID Class constants.  */

#define UX_DEVICE_CLASS_HID_CLASS                                   0x03
#define UX_DEVICE_CLASS_HID_SUBCLASS                                0X00
#define UX_DEVICE_CLASS_HID_PROTOCOL                                0X00

/* Define HID Class commands.  */

#define UX_DEVICE_CLASS_HID_COMMAND_GET_REPORT                      0x01
#define UX_DEVICE_CLASS_HID_COMMAND_GET_IDLE                        0x02
#define UX_DEVICE_CLASS_HID_COMMAND_GET_PROTOCOL                    0x03
#define UX_DEVICE_CLASS_HID_COMMAND_SET_REPORT                      0x09
#define UX_DEVICE_CLASS_HID_COMMAND_SET_IDLE                        0x0A
#define UX_DEVICE_CLASS_HID_COMMAND_SET_PROTOCOL                    0x0B

/* Define HID Class Descriptor types.  */

#define UX_DEVICE_CLASS_HID_DESCRIPTOR_HID                          0x21
#define UX_DEVICE_CLASS_HID_DESCRIPTOR_REPORT                       0x22
#define UX_DEVICE_CLASS_HID_DESCRIPTOR_PHYSICAL                     0x23

/* Define HID Report Types.  */

#define UX_DEVICE_CLASS_HID_REPORT_TYPE_INPUT                       0x1
#define UX_DEVICE_CLASS_HID_REPORT_TYPE_OUTPUT                      0x2
#define UX_DEVICE_CLASS_HID_REPORT_TYPE_FEATURE                     0x3

/* Define HID event info structure.  */
#define UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH                     64   /* #WICED#: Increase from 32 to 64 for HS max 64bytes characteristics. */
#define UX_DEVICE_CLASS_HID_NEW_EVENT                               1
#define    UX_DEVICE_CLASS_HID_MAX_EVENTS_QUEUE                     16

typedef struct UX_SLAVE_CLASS_HID_EVENT_STRUCT
{
    ULONG                   ux_device_class_hid_event_report_id;
    ULONG                   ux_device_class_hid_event_report_type;
    UCHAR                   ux_device_class_hid_event_buffer[UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH];
    ULONG                   ux_device_class_hid_event_length;
    
} UX_SLAVE_CLASS_HID_EVENT;

/* Define HID structure.  */

typedef struct UX_SLAVE_CLASS_HID_STRUCT
{

    UX_SLAVE_INTERFACE              *ux_slave_class_hid_interface;
    UX_SLAVE_ENDPOINT               *ux_device_class_hid_interrupt_endpoint;
    UINT                            ux_device_class_hid_state;
    UINT                            (*ux_device_class_hid_callback)(struct UX_SLAVE_CLASS_HID_STRUCT *hid, UX_SLAVE_CLASS_HID_EVENT *);
    UCHAR                           *ux_device_class_hid_report_address;
    ULONG                           ux_device_class_hid_report_length;
    TX_EVENT_FLAGS_GROUP            ux_device_class_hid_event_flags_group;
    UX_SLAVE_CLASS_HID_EVENT        *ux_device_class_hid_event_array;
    UX_SLAVE_CLASS_HID_EVENT        *ux_device_class_hid_event_array_head;
    UX_SLAVE_CLASS_HID_EVENT        *ux_device_class_hid_event_array_tail;
    UX_SLAVE_CLASS_HID_EVENT        *ux_device_class_hid_event_array_end;
                                                                
} UX_SLAVE_CLASS_HID;

/* Define HID initialization command structure.  */

typedef struct UX_SLAVE_CLASS_HID_PARAMETER_STRUCT
{

    UCHAR                   *ux_device_class_hid_parameter_report_address;
    ULONG                   ux_device_class_hid_parameter_report_length;
    UINT                    (*ux_device_class_hid_parameter_callback)(struct UX_SLAVE_CLASS_HID_STRUCT *hid, UX_SLAVE_CLASS_HID_EVENT *);

} UX_SLAVE_CLASS_HID_PARAMETER;


/* Define Pima Class function prototypes.  */
UINT  _ux_device_class_hid_descriptor_send(UX_SLAVE_CLASS_HID *hid, ULONG descriptor_type, 
                                            ULONG request_index, ULONG host_length);
UINT  _ux_device_class_hid_activate(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_hid_deactivate(UX_SLAVE_CLASS_COMMAND *command);
VOID  _ux_device_class_hid_control_complete(UX_SLAVE_TRANSFER *transfer_request);
VOID  _ux_device_class_hid_control_request(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_hid_entry(UX_SLAVE_CLASS_COMMAND *command);
VOID  _ux_device_class_hid_interrupt_thread(ULONG hid_class);
UINT  _ux_device_class_hid_initialize(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_hid_uninitialize(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_hid_event_set(UX_SLAVE_CLASS_HID *hid, 
                                      UX_SLAVE_CLASS_HID_EVENT *hid_event);
UINT  _ux_device_class_hid_event_get(UX_SLAVE_CLASS_HID *hid, 
                                      UX_SLAVE_CLASS_HID_EVENT *hid_event);
UINT  _ux_device_class_hid_report_set(UX_SLAVE_CLASS_HID *hid, ULONG descriptor_type, 
                                            ULONG request_index, ULONG host_length);
UINT  _ux_device_class_hid_report_get(UX_SLAVE_CLASS_HID *hid, ULONG descriptor_type, 
                                            ULONG request_index, ULONG host_length);
                                      
                                      
/* Define Device HID Class API prototypes.  */

#define ux_device_class_hid_entry        _ux_device_class_hid_entry   
#define ux_device_class_hid_event_set    _ux_device_class_hid_event_set
#define ux_device_class_hid_event_get    _ux_device_class_hid_event_get
#define ux_device_class_hid_report_set   _ux_device_class_hid_report_set
#define ux_device_class_hid_report_get   _ux_device_class_hid_report_get


#endif
