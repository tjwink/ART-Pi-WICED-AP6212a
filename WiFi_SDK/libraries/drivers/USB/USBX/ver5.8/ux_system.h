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
/**   System                                                              */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_system.h                                         PORTABLE C      */ 
/*                                                           5.8          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Thierry Giron, Express Logic Inc.                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX main system component.                                         */ 
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

#ifndef UX_SYSTEM_HOST_H
#define UX_SYSTEM_HOST_H

/* Define System component function prototypes.  */

UINT  _ux_system_initialize(VOID *regular_memory_pool_start, ULONG regular_memory_size, 
                            VOID *cache_safe_memory_pool_start, ULONG cache_safe_memory_size);
UINT  _ux_system_uninitialize(VOID);

/* Define System component external data references.  */

extern UX_SYSTEM *_ux_system;
extern UX_SYSTEM_HOST *_ux_system_host;
extern UX_SYSTEM_SLAVE *_ux_system_slave;
extern UX_SYSTEM_OTG *_ux_system_otg;
extern UCHAR _ux_system_endpoint_descriptor_structure[];
extern UCHAR _ux_system_device_descriptor_structure[];
extern UCHAR _ux_system_configuration_descriptor_structure[];
extern UCHAR _ux_system_interface_descriptor_structure[];
extern UCHAR _ux_system_string_descriptor_structure[];
extern UCHAR _ux_system_dfu_functional_descriptor_structure[];
extern UCHAR _ux_system_hub_descriptor_structure[];
extern UCHAR _ux_system_hid_descriptor_structure[];
extern UCHAR _ux_system_class_audio_interface_descriptor_structure[];
extern UCHAR _ux_system_class_audio_input_terminal_descriptor_structure[];
extern UCHAR _ux_system_class_audio_output_terminal_descriptor_structure[];
extern UCHAR _ux_system_class_audio_feature_unit_descriptor_structure[];
extern UCHAR _ux_system_class_audio_streaming_interface_descriptor_structure[];
extern UCHAR _ux_system_class_audio_streaming_endpoint_descriptor_structure[];
extern UCHAR _ux_system_class_pima_storage_structure[];
extern UCHAR _ux_system_class_pima_object_structure[];
extern UCHAR _ux_system_ecm_interface_descriptor_structure[];

extern UINT  _ux_system_host_hcd_periodic_tree_entries[32]; 

extern UCHAR _ux_system_host_class_hub_name[];  
extern UCHAR _ux_system_host_class_printer_name[]; 
extern UCHAR _ux_system_host_class_storage_name[]; 
extern UCHAR _ux_system_host_class_hid_name[];     
extern UCHAR _ux_system_host_class_audio_name[];   
extern UCHAR _ux_system_host_class_cdc_acm_name[];   
extern UCHAR _ux_system_host_class_cdc_dlc_name[];   
extern UCHAR _ux_system_host_class_cdc_ecm_name[];   
extern UCHAR _ux_system_host_class_prolific_name[];   
extern UCHAR _ux_system_host_class_dpump_name[];  
extern UCHAR _ux_system_host_class_pima_name[];  
extern UCHAR _ux_system_host_class_asix_name[];   
extern UCHAR _ux_system_host_class_swar_name[];   
extern UCHAR _ux_system_host_class_gser_name[];   
extern UCHAR _ux_system_host_class_hid_client_remote_control_name[];
extern UCHAR _ux_system_host_class_hid_client_mouse_name[]; 
extern UCHAR _ux_system_host_class_hid_client_keyboard_name[]; 

extern UCHAR _ux_system_host_hcd_ohci_name[]; 
extern UCHAR _ux_system_host_hcd_ehci_name[]; 
extern UCHAR _ux_system_host_hcd_isp1161_name[]; 
extern UCHAR _ux_system_host_hcd_isp1362_name[]; 
extern UCHAR _ux_system_host_hcd_sh2_name[]; 
extern UCHAR _ux_system_host_hcd_rx_name[]; 
extern UCHAR _ux_system_host_hcd_pic32_name[]; 
extern UCHAR _ux_system_host_hcd_stm32_name[]; 
extern UCHAR _ux_system_host_hcd_musb_name[];
extern UCHAR _ux_system_host_hcd_atm7_name[];
extern UCHAR _ux_system_host_hcd_simulator_name[]; 

extern UCHAR _ux_system_slave_class_storage_name[]; 
extern UCHAR _ux_system_slave_class_storage_vendor_id[]; 
extern UCHAR _ux_system_slave_class_storage_product_id[]; 
extern UCHAR _ux_system_slave_class_storage_product_rev[]; 
extern UCHAR _ux_system_slave_class_storage_product_serial[]; 
extern UCHAR _ux_system_slave_class_cdc_acm_name[]; 
extern UCHAR _ux_system_slave_class_dpump_name[]; 
extern UCHAR _ux_system_slave_class_pima_name[]; 
extern UCHAR _ux_system_slave_class_hid_name[]; 
extern UCHAR _ux_system_slave_class_rndis_name[]; 
extern UCHAR _ux_system_slave_class_cdc_ecm_name[]; 
extern UCHAR _ux_system_slave_class_dfu_name[];



#endif

