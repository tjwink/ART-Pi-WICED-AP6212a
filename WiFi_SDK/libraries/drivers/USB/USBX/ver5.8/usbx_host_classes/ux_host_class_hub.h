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
/**   HUB Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_host_class_hub.h                                 PORTABLE C      */ 
/*                                                           5.8          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Thierry Giron, Express Logic Inc.                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX HUB class.                                                     */ 
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

#ifndef UX_HOST_CLASS_HUB_H
#define UX_HOST_CLASS_HUB_H


/* Define HUB Class constants.  */

#define UX_HOST_CLASS_HUB_CLASS                                 9
#define UX_HOST_CLASS_HUB_PROTOCOL_FS                           0
#define UX_HOST_CLASS_HUB_PROTOCOL_SINGLE_TT                    1
#define UX_HOST_CLASS_HUB_PROTOCOL_MULTIPLE_TT                  2


/* Define HUB Class descriptor field constants.  */

#define UX_HOST_CLASS_HUB_GANG_POWER_SWITCHING                  0x00
#define UX_HOST_CLASS_HUB_INDIVIDUAL_POWER_SWITCHING            0x01
#define UX_HOST_CLASS_HUB_NO_POWER_SWITCHING                    0x02
                                                        
#define UX_HOST_CLASS_HUB_COMPOUND_DEVICE                       0x04

#define UX_HOST_CLASS_HUB_GLOBAL_OVERCURRENT                    0x00
#define UX_HOST_CLASS_HUB_INDIVIDUAL_OVERCURRENT                0x08
#define UX_HOST_CLASS_HUB_NO_OVERCURRENT                        0x10


/* Define HUB Class command constants.  */

#define UX_HOST_CLASS_HUB_GET_STATUS                            0x00
#define UX_HOST_CLASS_HUB_CLEAR_FEATURE                         0x01
#define UX_HOST_CLASS_HUB_GET_STATE                             0x02
#define UX_HOST_CLASS_HUB_SET_FEATURE                           0x03
#define UX_HOST_CLASS_HUB_GET_DESCRIPTOR                        0x06
#define UX_HOST_CLASS_HUB_SET_DESCRIPTOR                        0x07


/* Define HUB Class set_feature command constants.  */

#define UX_HOST_CLASS_HUB_PORT_CONNECTION                       0x00
#define UX_HOST_CLASS_HUB_PORT_ENABLE                           0x01
#define UX_HOST_CLASS_HUB_PORT_SUSPEND                          0x02
#define UX_HOST_CLASS_HUB_PORT_OVER_CURRENT                     0x03
#define UX_HOST_CLASS_HUB_PORT_RESET                            0x04
#define UX_HOST_CLASS_HUB_PORT_POWER                            0x08
#define UX_HOST_CLASS_HUB_PORT_LOW_SPEED                        0x09
#define UX_HOST_CLASS_HUB_C_PORT_CONNECTION                     0x10
#define UX_HOST_CLASS_HUB_C_PORT_ENABLE                         0x11
#define UX_HOST_CLASS_HUB_C_PORT_SUSPEND                        0x12
#define UX_HOST_CLASS_HUB_C_PORT_OVER_CURRENT                   0x13
#define UX_HOST_CLASS_HUB_C_PORT_RESET                          0x14


/* Define HUB Class port status constants.  */

#define UX_HOST_CLASS_HUB_PORT_STATUS_CONNECTION                0x0001
#define UX_HOST_CLASS_HUB_PORT_STATUS_ENABLE                    0x0002
#define UX_HOST_CLASS_HUB_PORT_STATUS_SUSPEND                   0x0004
#define UX_HOST_CLASS_HUB_PORT_STATUS_OVER_CURRENT              0x0008
#define UX_HOST_CLASS_HUB_PORT_STATUS_RESET                     0x0010
#define UX_HOST_CLASS_HUB_PORT_STATUS_POWER                     0x0100
#define UX_HOST_CLASS_HUB_PORT_STATUS_LOW_SPEED                 0x0200
#define UX_HOST_CLASS_HUB_PORT_STATUS_HIGH_SPEED                0x0400


/* Define HUB Class port change constants.  */

#define UX_HOST_CLASS_HUB_PORT_CHANGE_CONNECTION                0x00001
#define UX_HOST_CLASS_HUB_PORT_CHANGE_ENABLE                    0x00002
#define UX_HOST_CLASS_HUB_PORT_CHANGE_SUSPEND                   0x00004
#define UX_HOST_CLASS_HUB_PORT_CHANGE_OVER_CURRENT              0x00008
#define UX_HOST_CLASS_HUB_PORT_CHANGE_RESET                     0x00010


/* Define HUB Class other constants.  */

#define UX_HOST_CLASS_HUB_ENABLE_RETRY_COUNT                    3
#define UX_HOST_CLASS_HUB_ENABLE_RETRY_DELAY                    100
#define UX_HOST_CLASS_HUB_ENUMERATION_RETRY                     3
#define UX_HOST_CLASS_HUB_ENUMERATION_RETRY_DELAY               300
#define UX_HOST_CLASS_HUB_NOT_POWERED                           8

#define UX_HUB_DESCRIPTOR_ENTRIES                               8
#define UX_HUB_DESCRIPTOR_LENGTH                                9

/* Define HUB Class structure.  */

#define UX_MAX_HUB_PORTS                                        15

typedef struct UX_HUB_DESCRIPTOR_STRUCT
{

    ULONG           bLength;
    ULONG           bDescriptorType;
    ULONG           bNbPorts;
    ULONG           wHubCharacteristics;
    ULONG           bPwrOn2PwrGood;
    ULONG           bHubContrCurrent;
    ULONG           bDeviceRemovable;
    ULONG           bPortPwrCtrlMask;
} UX_HUB_DESCRIPTOR;


/* Define HUB Class instance structure.  */

typedef struct UX_HOST_CLASS_HUB_STRUCT
{

    struct UX_HOST_CLASS_HUB_STRUCT              
                    *ux_host_class_hub_next_instance;
    UX_HOST_CLASS   *ux_host_class_hub_class;
    UX_DEVICE       *ux_host_class_hub_device;
    UX_ENDPOINT     *ux_host_class_hub_interrupt_endpoint;
    UX_INTERFACE    *ux_host_class_hub_interface;
    UINT            ux_host_class_hub_instance_status;
    UINT            ux_host_class_hub_state;
    UINT            ux_host_class_hub_enumeration_retry_count;
    UINT            ux_host_class_hub_change_semaphore;
    struct UX_HUB_DESCRIPTOR_STRUCT         
                    ux_host_class_hub_descriptor;
    TX_SEMAPHORE    ux_host_class_hub_semaphore;
    UINT            ux_host_class_hub_port_state;
} UX_HOST_CLASS_HUB;


/* Define HUB Class function prototypes.  */

UINT    _ux_host_class_hub_activate(UX_HOST_CLASS_COMMAND *command);
VOID    _ux_host_class_hub_change_detect(VOID);
UINT    _ux_host_class_hub_change_process(UX_HOST_CLASS_HUB *hub);
UINT    _ux_host_class_hub_configure(UX_HOST_CLASS_HUB *hub);
UINT    _ux_host_class_hub_deactivate(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_hub_descriptor_get(UX_HOST_CLASS_HUB *hub);
UINT    _ux_host_class_hub_entry(UX_HOST_CLASS_COMMAND *command);
UINT    _ux_host_class_hub_feature(UX_HOST_CLASS_HUB *hub, UINT port, UINT command, UINT function);
UINT    _ux_host_class_hub_hub_change_process(UX_HOST_CLASS_HUB *hub);
UINT    _ux_host_class_hub_interrupt_endpoint_start(UX_HOST_CLASS_HUB *hub);
VOID    _ux_host_class_hub_port_change_connection_process(UX_HOST_CLASS_HUB *hub, UINT port, UINT port_status);
VOID    _ux_host_class_hub_port_change_enable_process(UX_HOST_CLASS_HUB *hub, UINT port, UINT port_status);
VOID    _ux_host_class_hub_port_change_over_current_process(UX_HOST_CLASS_HUB *hub, UINT port, UINT port_status);
UINT    _ux_host_class_hub_port_change_process(UX_HOST_CLASS_HUB *hub, UINT port);
VOID    _ux_host_class_hub_port_change_reset_process(UX_HOST_CLASS_HUB *hub, UINT port, UINT port_status);
VOID    _ux_host_class_hub_port_change_suspend_process(UX_HOST_CLASS_HUB *hub, UINT port, UINT port_status);
UINT    _ux_host_class_hub_port_reset(UX_HOST_CLASS_HUB *hub, UINT port);
UINT    _ux_host_class_hub_ports_power(UX_HOST_CLASS_HUB *hub);
UINT    _ux_host_class_hub_status_get(UX_HOST_CLASS_HUB *hub, UINT port, USHORT *port_status, USHORT *port_change);
VOID    _ux_host_class_hub_transfer_request_completed(UX_TRANSFER *transfer_request);

#endif

