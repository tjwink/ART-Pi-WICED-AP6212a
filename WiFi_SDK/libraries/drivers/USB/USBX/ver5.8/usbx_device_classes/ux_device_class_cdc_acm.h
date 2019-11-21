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
/**   CDC Class                                                           */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_device_class_cdc_acm.h                           PORTABLE C      */ 
/*                                                           5.8          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Thierry Giron, Express Logic Inc.                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This file defines the equivalences for the USBX Device Class CDC    */ 
/*    ACM component.                                                      */ 
/*                                                                        */ 
/*  RELEASE HISTORY                                                       */ 
/*                                                                        */ 
/*    DATE              NAME                      DESCRIPTION             */ 
/*                                                                        */ 
/*  07-01-2007     TCRG                     Initial Version 5.0           */ 
/*  11-11-2008     TCRG                     Modified comment(s),          */ 
/*                                            updated CDC parameter       */ 
/*                                            typedef, removed unused     */ 
/*                                            function prototypes,        */ 
/*                                            and added new               */ 
/*                                            function prototypes,        */ 
/*                                            resulting in  version 5.2   */ 
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

#ifndef UX_DEVICE_CLASS_CDC_ACM_H
#define UX_DEVICE_CLASS_CDC_ACM_H


/* Define CDC Class USB Class constants.  */
#define UX_SLAVE_CLASS_CDC_ACM_CLASS                                    10

/* Device CDC Requests */
#define UX_SLAVE_CLASS_CDC_ACM_SEND_ENCAPSULATED_COMMAND                0x00
#define UX_SLAVE_CLASS_CDC_ACM_GET_ENCAPSULATED_RESPONSE                0x01
#define UX_SLAVE_CLASS_CDC_ACM_SET_COMM_FEATURE                         0x02
#define UX_SLAVE_CLASS_CDC_ACM_GET_COMM_FEATURE                         0x03
#define UX_SLAVE_CLASS_CDC_ACM_CLEAR_COMM_FEATURE                       0x04
#define UX_SLAVE_CLASS_CDC_ACM_SET_AUX_LINE_STATE                       0x10
#define UX_SLAVE_CLASS_CDC_ACM_SET_HOOK_STATE                           0x11
#define UX_SLAVE_CLASS_CDC_ACM_PULSE_SETUP                              0x12
#define UX_SLAVE_CLASS_CDC_ACM_SEND_PULSE                               0x13
#define UX_SLAVE_CLASS_CDC_ACM_SET_PULSE_TIME                           0x14
#define UX_SLAVE_CLASS_CDC_ACM_RING_AUX_JACK                            0x15
#define UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING                          0x20
#define UX_SLAVE_CLASS_CDC_ACM_GET_LINE_CODING                          0x21
#define UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE                   0x22
#define UX_SLAVE_CLASS_CDC_ACM_SEND_BREAK                               0x23
#define UX_SLAVE_CLASS_CDC_ACM_SET_RINGER_PARMS                         0x30
#define UX_SLAVE_CLASS_CDC_ACM_GET_RINGER_PARMS                         0x31
#define UX_SLAVE_CLASS_CDC_ACM_SET_OPERATION_PARMS                      0x32
#define UX_SLAVE_CLASS_CDC_ACM_GET_OPERATION_PARMS                      0x33
#define UX_SLAVE_CLASS_CDC_ACM_SET_LINE_PARMS                           0x34
#define UX_SLAVE_CLASS_CDC_ACM_GET_LINE_PARMS                           0x35
#define UX_SLAVE_CLASS_CDC_ACM_DIAL_DIGITS                              0x36
#define UX_SLAVE_CLASS_CDC_ACM_SET_UNIT_PARAMETER                       0x37
#define UX_SLAVE_CLASS_CDC_ACM_GET_UNIT_PARAMETER                       0x38
#define UX_SLAVE_CLASS_CDC_ACM_CLEAR_UNIT_PARAMETER                     0x39
#define UX_SLAVE_CLASS_CDC_ACM_GET_PROFILE                              0x3A
#define UX_SLAVE_CLASS_CDC_ACM_SET_ETHERNET_MULTICAST_FILTERS           0x40
#define UX_SLAVE_CLASS_CDC_ACM_SET_ETHERNET_POWER_MANAGEMENT_PATTERN    0x41
#define UX_SLAVE_CLASS_CDC_ACM_GET_ETHERNET_POWER_MANAGEMENT_PATTERN    0x42
#define UX_SLAVE_CLASS_CDC_ACM_SET_ETHERNET_PACKET_FILTER               0x43
#define UX_SLAVE_CLASS_CDC_ACM_GET_ETHERNET_STATISTIC                   0x44
#define UX_SLAVE_CLASS_CDC_ACM_SET_ATM_DATA_FORMAT                      0x50
#define UX_SLAVE_CLASS_CDC_ACM_GET_ATM_DEVICE_STATISTICS                0x51
#define UX_SLAVE_CLASS_CDC_ACM_SET_ATM_DEFAULT_VC                       0x52
#define UX_SLAVE_CLASS_CDC_ACM_GET_ATM_VC_STATISTICS                    0x53

/* Default line coding values.  */
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_BAUDRATE                     115200
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_STOP_BIT                     1
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARITY                       0
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_DATA_BIT                     8

/* Define line coding structure.  */
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_BAUDRATE_STRUCT              0
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_STOP_BIT_STRUCT              4
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARITY_STRUCT                5
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_DATA_BIT_STRUCT              6
#define UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_RESPONSE_SIZE                7

/* Define line state bits.  */
#define UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_DTR                           1
#define UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_RTS                           2

/* Define IOCTL functions.  */
#define UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING                    1
#define UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING                    2
#define UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_STATE                     3

/* Define Slave CDC Class Calling Parameter structure */

typedef struct UX_SLAVE_CLASS_CDC_ACM_PARAMETER_STRUCT
{
    VOID                    (*ux_slave_class_cdc_acm_instance_activate)(VOID *);
    VOID                    (*ux_slave_class_cdc_acm_instance_deactivate)(VOID *);

} UX_SLAVE_CLASS_CDC_ACM_PARAMETER;

/* Define CDC Class structure.  */

typedef struct UX_SLAVE_CLASS_CDC_ACM_STRUCT
{
    UX_SLAVE_INTERFACE                  *ux_slave_class_cdc_acm_interface;
    UX_SLAVE_CLASS_CDC_ACM_PARAMETER    ux_slave_class_cdc_acm_parameter;
    TX_MUTEX                            ux_slave_class_cdc_acm_endpoint_in_mutex;
    TX_MUTEX                            ux_slave_class_cdc_acm_endpoint_out_mutex;
    ULONG                               ux_slave_class_cdc_acm_baudrate;
    UCHAR                               ux_slave_class_cdc_acm_stop_bit;
    UCHAR                               ux_slave_class_cdc_acm_parity;
    UCHAR                               ux_slave_class_cdc_acm_data_bit;
    UCHAR                               ux_slave_class_cdc_acm_data_dtr_state;
    UCHAR                               ux_slave_class_cdc_acm_data_rts_state;
    
} UX_SLAVE_CLASS_CDC_ACM;

/* Define some CDC Class structures */

typedef struct UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER_STRUCT 
{
    ULONG                               ux_slave_class_cdc_acm_parameter_baudrate;
    UCHAR                               ux_slave_class_cdc_acm_parameter_stop_bit;
    UCHAR                               ux_slave_class_cdc_acm_parameter_parity;
    UCHAR                               ux_slave_class_cdc_acm_parameter_data_bit;
    
} UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER;

typedef struct UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_PARAMETER_STRUCT 
{
    ULONG                               ux_slave_class_cdc_acm_parameter_rts;
    UCHAR                               ux_slave_class_cdc_acm_parameter_dtr;
    
} UX_SLAVE_CLASS_CDC_ACM_LINE_STATE_PARAMETER;


/* Requests - Ethernet Networking Control Model */

#define UX_SLAVE_CLASS_CDC_ACM_SEND_ENCAPSULATED_COMMAND                        0x00        
                                        /* Issues a command in the format of the supported control
                                           protocol. The intent of this mechanism is to support
                                           networking devices (e.g., host-based cable modems)
                                           that require an additional vendor-defined interface for
                                           media specific hardware configuration and
                                           management.  */
#define UX_SLAVE_CLASS_CDC_ACM_GET_ENCAPSULATED_RESPONSE                        0x01        
                                        /* Requests a response in the format of the supported
                                           control protocol.  */
#define UX_SLAVE_CLASS_CDC_ACM_SET_ETHERNET_MULTICAST_FILTERS                   0x40        
                                        /* As applications are loaded and unloaded on the host,
                                           the networking transport will instruct the device�s MAC
                                           driver to change settings of the Networking device�s
                                           multicast filters.  */
#define UX_SLAVE_CLASS_CDC_ACM_SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER     0x41        
                                        /* Some hosts are able to conserve energy and stay quiet
                                           in a 'sleeping' state while not being used. USB
                                           Networking devices may provide special pattern filtering
                                           hardware that enables it to wake up the attached host
                                           on demand when something is attempting to contact the
                                           host (e.g., an incoming web browser connection).
                                           Primitives are needed in management plane to negotiate
                                           the setting of these special filters  */
#define UX_SLAVE_CLASS_CDC_ACM_GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER     0x42 
                                        /* Retrieves the status of the above power management
                                           pattern filter setting  */
#define UX_SLAVE_CLASS_CDC_ACM_SET_ETHERNET_PACKET_FILTER                       0x43 
                                        /* Sets device filter for running a network analyzer
                                           application on the host machine  */
#define UX_SLAVE_CLASS_CDC_ACM_GET_ETHERNET_STATISTIC                           0x44 
                                        /* Retrieves Ethernet device statistics such as frames
                                           transmitted, frames received, and bad frames received.  */

/* Define buffer length for IN/OUT pipes.  */

#define UX_SLAVE_CLASS_CDC_ACM_BUFFER_SIZE                  4096


/* Define Device CDC Class prototypes.  */

UINT  _ux_device_class_cdc_acm_activate(UX_SLAVE_CLASS_COMMAND *command);
VOID  _ux_device_class_cdc_acm_control_complete(UX_SLAVE_TRANSFER *transfer_request);
VOID  _ux_device_class_cdc_acm_control_request(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_acm_deactivate(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_acm_entry(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_acm_initialize(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_acm_uninitialize(UX_SLAVE_CLASS_COMMAND *command);
UINT  _ux_device_class_cdc_acm_write(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UCHAR *buffer, 
                                ULONG requested_length, ULONG *actual_length);
UINT  _ux_device_class_cdc_acm_read(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UCHAR *buffer, 
                                ULONG requested_length, ULONG *actual_length);
VOID  _ux_device_class_cdc_acm_thread(ULONG cdc_acm_class);
UINT  _ux_device_class_cdc_acm_ioctl(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, ULONG ioctl_function,
                                    VOID *parameter);

/* Define Device CDC Class API prototypes.  */

#define ux_device_class_cdc_acm_entry    _ux_device_class_cdc_acm_entry
#define ux_device_class_cdc_acm_read     _ux_device_class_cdc_acm_read 
#define ux_device_class_cdc_acm_write    _ux_device_class_cdc_acm_write

#endif /* UX_DEVICE_CLASS_CDC_ACM_H */
