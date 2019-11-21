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
/**   EHCI Controller                                                     */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_hcd_ehci.h                                       PORTABLE C      */ 
/*                                                           5.8          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Thierry Giron, Express Logic Inc.                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX host EHCI Controller.                                          */ 
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
/*  10-10-2012     TCRG                     Modified comment(s), and      */ 
/*                                            added support for dynamic   */ 
/*                                            frame list,                 */ 
/*                                            resulting in version 5.6    */ 
/*  06-01-2014     TCRG                     Modified comment(s),          */ 
/*                                            resulting in version 5.7    */ 
/*  06-01-2017     TCRG                     Modified comment(s),          */ 
/*                                            resulting in version 5.8    */ 
/*                                                                        */ 
/**************************************************************************/ 

#ifndef UX_HCD_EHCI_H
#define UX_HCD_EHCI_H


/* Define EHCI generic definitions.  */

#define UX_EHCI_CONTROLLER                                  2
#define UX_EHCI_MAX_PAYLOAD                                 16384
#define UX_EHCI_FRAME_DELAY                                 4 
#define UX_EHCI_PAGE_SIZE                                   4096
#define UX_EHCI_PAGE_ALIGN                                  0xfffff000


/* Define EHCI host controller capability registers.  */

#define EHCI_HCCR_CAP_LENGTH                                0x00
#define EHCI_HCCR_HCS_PARAMS                                0x01
#define EHCI_HCCR_HCC_PARAMS                                0x02
#define EHCI_HCCR_HCSP_PORT_ROUTE                           0x03


/* Define EHCI host controller registers.  */

#define EHCI_HCOR_USB_COMMAND                               (hcd_ehci -> ux_hcd_ehci_hcor + 0x00)
#define EHCI_HCOR_USB_STATUS                                (hcd_ehci -> ux_hcd_ehci_hcor + 0x01)
#define EHCI_HCOR_USB_INTERRUPT                             (hcd_ehci -> ux_hcd_ehci_hcor + 0x02)
#define EHCI_HCOR_FRAME_INDEX                               (hcd_ehci -> ux_hcd_ehci_hcor + 0x03)
#define EHCI_HCOR_FRAME_LIST_BASE_ADDRESS                   (hcd_ehci -> ux_hcd_ehci_hcor + 0x05)
#define EHCI_HCOR_ASYNCH_LIST_ADDRESS                       (hcd_ehci -> ux_hcd_ehci_hcor + 0x06)
#define EHCI_HCOR_CONFIG_FLAG                               (hcd_ehci -> ux_hcd_ehci_hcor + 0x10)
#define EHCI_HCOR_PORT_SC                                   (hcd_ehci -> ux_hcd_ehci_hcor + 0x11)


/* Define EHCI IO control register values.  */

#define EHCI_HC_IO_RS                                       0x00000001
#define EHCI_HC_IO_HCRESET                                  0x00000002
#define EHCI_HC_IO_PSE                                      0x00000010
#define EHCI_HC_IO_ASE                                      0x00000020
#define EHCI_HC_IO_IAAD                                     0x00000040   
#define EHCI_HC_IO_ITC                                      0x00010000   
#define EHCI_HC_IO_FRAME_SIZE_1024                          0x00000000
#define EHCI_HC_IO_FRAME_SIZE_512                           0x00000004
#define EHCI_HC_IO_FRAME_SIZE_256                           0x00000008
#define EHCI_HC_IO_FRAME_SIZE_128                           0x0000000C
#define EHCI_HC_IO_FRAME_SIZE_64                            0x00008000
#define EHCI_HC_IO_FRAME_SIZE_32                            0x00008004

/* The number if entries in the periodic tree can be changed to save space IF and only IF the PFLF flag in the HCCPARAMS register
   allows it. Setting values less than 1024 in controllers without the ability to change the Frame List Size leads to a EHCI crash.  */
   
#ifndef UX_EHCI_FRAME_LIST_ENTRIES
#define UX_EHCI_FRAME_LIST_ENTRIES                          1024
#endif
#define UX_EHCI_FRAME_LIST_MASK                             EHCI_HC_IO_FRAME_SIZE_1024

/* Define EHCI HCOR status register.  */

#define EHCI_HC_STS_USB_INT                                 0x00000001
#define EHCI_HC_STS_USB_ERR_INT                             0x00000002
#define EHCI_HC_STS_PCD                                     0x00000004
#define EHCI_HC_STS_FLR                                     0x00000008
#define EHCI_HC_STS_HSE                                     0x00000010
#define EHCI_HC_STS_IAA                                     0x00000020
#define EHCI_HC_STS_HC_HALTED                               0x00001000
#define EHCI_HC_STS_RECLAMATION                             0x00002000
#define EHCI_HC_STS_PSS                                     0x00004000
#define EHCI_HC_STS_ASS                                     0x00008000

#define EHCI_HC_INTERRUPT_ENABLE_NORMAL                     (EHCI_HC_STS_USB_INT|EHCI_HC_STS_USB_ERR_INT|EHCI_HC_STS_PCD|EHCI_HC_STS_HSE|EHCI_HC_STS_IAA)


/* Define EHCI HCOR root HUB command/status.  */

#define EHCI_HC_RH_PPC                                      0x00000010
#define EHCI_HC_RH_PSM                                      0x00000100
#define EHCI_HC_RH_NPS                                      0x00000200
#define EHCI_HC_RH_DT                                       0x00000400
#define EHCI_HC_RH_OCPM                                     0x00000800
#define EHCI_HC_RH_NOCP                                     0x00001000

#define EHCI_HC_PS_CCS                                      0x00000001
#define EHCI_HC_PS_CSC                                      0x00000002
#define EHCI_HC_PS_PE                                       0x00000004
#define EHCI_HC_PS_PEC                                      0x00000008
#define EHCI_HC_PS_OCA                                      0x00000010
#define EHCI_HC_PS_OCC                                      0x00000020
#define EHCI_HC_PS_FPR                                      0x00000040
#define EHCI_HC_PS_SUSPEND                                  0x00000080
#define EHCI_HC_PS_PR                                       0x00000100
#define EHCI_HC_PS_PP                                       0x00001000
#define EHCI_HC_PS_SPEED_MASK                               0x00000c00
#define EHCI_HC_PS_SPEED_LOW                                0x00000400
#define EHCI_HC_PS_PO                                       0x00002000
#define EHCI_HC_PS_EMBEDDED_TT_SPEED_MASK                   0x0c000000
#define EHCI_HC_PS_EMBEDDED_TT_SPEED_FULL                   0x00000000
#define EHCI_HC_PS_EMBEDDED_TT_SPEED_LOW                    0x04000000
#define EHCI_HC_PS_EMBEDDED_TT_SPEED_HIGH                   0x08000000

#define EHCI_HC_RH_POWER_STABLE_DELAY                       25
#define EHCI_HC_RH_RESET_DELAY                              50
#define EHCI_HC_RH_RESET_SETTLE_DELAY                       5


/* Define EHCI interrupt status register definitions.  */

#define EHCI_HC_INT_IE                                      0x00000001
#define EHCI_HC_INT_EIE                                     0x00000002
#define EHCI_HC_INT_PCIE                                    0x00000004    
#define EHCI_HC_INT_FLRE                                    0x00000008
#define EHCI_HC_INT_HSER                                    0x00000010
#define EHCI_HC_INT_IAAE                                    0x00000020


/* Define EHCI frame interval definition.  */

#define EHCI_HC_FM_INTERVAL_CLEAR                           0x8000ffff
#define EHCI_HC_FM_INTERVAL_SET                             0x27780000


/* Define EHCI static definition.  */

#define UX_EHCI_AVAILABLE_BANDWIDTH                         920
#define UX_EHCI_STOP                                        0
#define UX_EHCI_START                                       1
#define UX_EHCI_ROUTE_TO_LOCAL_HC                           1
#define UX_EHCI_INIT_DELAY                                  1000
#define UX_EHCI_RESET_RETRY                                 1000
#define UX_EHCI_RESET_DELAY                                 100
#define UX_EHCI_PORT_RESET_RETRY                            10
#define UX_EHCI_PORT_RESET_DELAY                            50


/* Define EHCI initialization values.  */

#define UX_EHCI_COMMAND_STATUS_RESET                        0
#define UX_EHCI_INIT_RESET_DELAY                            10


/* Define EHCI completion code errors.  */

#define UX_EHCI_NO_ERROR                                    0x00
#define UX_EHCI_ERROR_CRC                                   0x01
#define UX_EHCI_ERROR_BIT_STUFFING                          0x02
#define UX_EHCI_ERROR_DATA_TOGGLE                           0x03
#define UX_EHCI_ERROR_STALL                                 0x04
#define UX_EHCI_ERROR_DEVICE_NOT_RESPONDING                 0x05
#define UX_EHCI_ERROR_PID_FAILURE                           0x06
#define UX_EHCI_ERROR_DATA_OVERRUN                          0x08
#define UX_EHCI_ERROR_DATA_UNDERRUN                         0x09
#define UX_EHCI_ERROR_BUFFER_OVERRUN                        0x0c
#define UX_EHCI_ERROR_BUFFER_UNDERRUN                       0x0d
#define UX_EHCI_ERROR_NOT_ACCESSED                          0x0f
#define UX_EHCI_ERROR_NAK                                   0x10
#define UX_EHCI_ERROR_BABBLE                                0x11


/* Define the EHCI structure.  */

typedef struct UX_HCD_EHCI_STRUCT
{                                      

    struct UX_HCD_STRUCT
                    *ux_hcd_ehci_hcd_owner;
    ULONG           ux_hcd_ehci_hcor;
    struct UX_EHCI_ED_STRUCT              
                    **ux_hcd_ehci_frame_list;
    ULONG           *ux_hcd_ehci_base;
    UINT            ux_hcd_ehci_nb_root_hubs;
    struct UX_EHCI_TD_STRUCT              
                    *ux_hcd_ehci_done_head;
    struct UX_EHCI_ED_STRUCT              
                    *ux_hcd_ehci_ed_list;
    struct UX_EHCI_TD_STRUCT              
                    *ux_hcd_ehci_td_list;
    struct UX_EHCI_FSISO_TD_STRUCT        
                    *ux_hcd_ehci_fsiso_td_list;
    struct UX_EHCI_HSISO_TD_STRUCT        
                    *ux_hcd_ehci_hsiso_td_list;
    struct UX_EHCI_ED_STRUCT              
                    *ux_hcd_ehci_asynch_head_list;
    struct UX_EHCI_ED_STRUCT              
                    *ux_hcd_ehci_asynch_first_list;
    struct UX_EHCI_ED_STRUCT              
                    *ux_hcd_ehci_asynch_last_list;
    TX_SEMAPHORE    ux_hcd_ehci_protect_semaphore;
    TX_SEMAPHORE    ux_hcd_ehci_doorbell_semaphore;
    ULONG           ux_hcd_ehci_frame_list_size;
    ULONG           ux_hcd_ehci_interrupt_count;
    ULONG           ux_hcd_ehci_embedded_tt;
} UX_HCD_EHCI;


/* Define EHCI ED structure.  */ 

typedef struct UX_EHCI_ED_STRUCT
{                                      

    struct UX_EHCI_ED_STRUCT              
                    *ux_ehci_ed_queue_head;
    ULONG           ux_ehci_ed_cap0;
    ULONG           ux_ehci_ed_cap1;
    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_ed_current_td;
    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_ed_queue_element;
    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_ed_alternate_td;
    ULONG           ux_ehci_ed_state;
    VOID            *ux_ehci_ed_bp0; 
    VOID            *ux_ehci_ed_bp1; 
    VOID            *ux_ehci_ed_bp2; 
    VOID            *ux_ehci_ed_bp3; 
    VOID            *ux_ehci_ed_bp4; 
    struct UX_ENDPOINT_STRUCT             
                    *ux_ehci_ed_endpoint;
    ULONG           ux_ehci_ed_frame;
    struct UX_EHCI_ED_STRUCT              
                    *ux_ehci_ed_next_ed;
    struct UX_EHCI_ED_STRUCT              
                    *ux_ehci_ed_previous_ed;
    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_ed_first_td;
    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_ed_last_td;
    ULONG           ux_ehci_ed_status;
    ULONG           ux_ehci_ed_reserved[5];
} UX_EHCI_ED;


/* Define EHCI ED bitmap.  */

#define UX_EHCI_QH_TYP_ITD                                  0
#define UX_EHCI_QH_TYP_QH                                   2
#define UX_EHCI_QH_TYP_SITD                                 4
#define UX_EHCI_QH_TYP_FSTN                                 6

#define UX_EHCI_QH_T                                        1
#define UX_EHCI_QH_STATIC                                   0x80000000

#define UX_EHCI_QH_MPS_LOC                                  16
#define UX_EHCI_QH_MPS_MASK                                 0x07ff0000
#define UX_EHCI_QH_NCR                                      0xf0000000
#define UX_EHCI_QH_CEF                                      0x08000000
#define UX_EHCI_QH_ED_AD_LOC                                8 
#define UX_EHCI_QH_HBPM                                     0x40000000
#define UX_EHCI_QH_HEAD                                     0x00008000

#define UX_EHCI_QH_HIGH_SPEED                               0x00002000
#define UX_EHCI_QH_LOW_SPEED                                0x00001000

#define UX_EHCI_QH_HUB_ADDR_LOC                             16
#define UX_EHCI_QH_PORT_NUMBER_LOC                          23
#define UX_EHCI_QH_C_MASK                                   0x00001c00
#define UX_EHCI_QH_IS_MASK                                  0x00000001

#define UX_EHCI_QH_DTC                                      0x00004000
#define UX_EHCI_QH_TOGGLE                                   0x80000000
#define UX_EHCI_LINK_ADDRESS_MASK                           0xfffffff0
#define UX_EHCI_TOGGLE_0                                    0
#define UX_EHCI_TOGGLE_1                                    0x80000000


/* Define EHCI TD structure.  */

typedef struct UX_EHCI_TD_STRUCT
{                                      

    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_td_link_pointer;
    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_td_alternate_link_pointer;
    ULONG           ux_ehci_td_control;
    VOID            *ux_ehci_td_bp0; 
    VOID            *ux_ehci_td_bp1; 
    VOID            *ux_ehci_td_bp2; 
    VOID            *ux_ehci_td_bp3; 
    VOID            *ux_ehci_td_bp4; 
    struct UX_TRANSFER_STRUCT             
                    *ux_ehci_td_transfer_request;
    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_td_next_td_transfer_request;
    struct UX_EHCI_ED_STRUCT              
                    *ux_ehci_td_ed;
    ULONG           ux_ehci_td_length;
    ULONG           ux_ehci_td_status;
    ULONG           ux_ehci_td_phase;
    ULONG           ux_ehci_td_reserved_2[2];
} UX_EHCI_TD;


/* Define EHCI TD bitmap.  */

#define UX_EHCI_TD_T                                        1
#define UX_EHCI_TD_LG_LOC                                   16
#define UX_EHCI_TD_LG_MASK                                  0x7fff
#define UX_EHCI_TD_IOC                                      0x00008000
#define UX_EHCI_TD_CERR                                     0x00000c00

#define UX_EHCI_TD_PING                                     1
#define UX_EHCI_TD_DO_COMPLETE_SPLIT                        2
#define UX_EHCI_TD_MISSED_MICRO_FRAMES                      4
#define UX_EHCI_TD_TRANSACTION_ERROR                        8
#define UX_EHCI_TD_BABBLE_DETECTED                          0x10
#define UX_EHCI_TD_DATA_BUFFER_ERROR                        0x20
#define UX_EHCI_TD_HALTED                                   0x40
#define UX_EHCI_TD_ACTIVE                                   0x80

#define UX_EHCI_PID_OUT                                     0x00000000
#define UX_EHCI_PID_IN                                      0x00000100
#define UX_EHCI_PID_SETUP                                   0x00000200
#define UX_EHCI_PID_MASK                                    0x00000300

#define  UX_EHCI_TD_SETUP_PHASE                             0x00010000 
#define  UX_EHCI_TD_DATA_PHASE                              0x00020000 
#define  UX_EHCI_TD_STATUS_PHASE                            0x00040000 


/* Define EHCI ISOCHRONOUS TD structure.  */

typedef struct UX_EHCI_HSISO_TD_STRUCT
{

    struct UX_EHCI_ED_STRUCT              
                    ux_ehci_hsiso_td_link_pointer;
    VOID            *ux_ehci_hsiso_td_transaction0;
    VOID            *ux_ehci_hsiso_td_transaction1;
    VOID            *ux_ehci_hsiso_td_transaction2;
    VOID            *ux_ehci_hsiso_td_transaction3;
    VOID            *ux_ehci_hsiso_td_transaction4;
    VOID            *ux_ehci_hsiso_td_transaction5;
    VOID            *ux_ehci_hsiso_td_transaction6;
    VOID            *ux_ehci_hsiso_td_transaction7;
    VOID            *ux_ehci_hsiso_td_bp0; 
    VOID            *ux_ehci_hsiso_td_bp1; 
    VOID            *ux_ehci_hsiso_td_bp2; 
    VOID            *ux_ehci_hsiso_td_bp3; 
    VOID            *ux_ehci_hsiso_td_bp4; 
    VOID            *ux_ehci_hsiso_td_bp5; 
    VOID            *ux_ehci_hsiso_td_bp6; 
    VOID            *ux_ehci_hsiso_td_bp7; 
    ULONG           ux_ehci_hsiso_td_status;
    struct UX_TRANSFER_STRUCT             
                    *ux_ehci_hsiso_td_transfer_request;
    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_hsiso_td_next_td_transfer_request;
    struct UX_EHCI_ED_STRUCT              
                    *ux_ehci_hsiso_td_ed;
} UX_EHCI_HSISO_TD;


/* Define EHCI FS ISOCHRONOUS TD structure.  */

typedef struct UX_EHCI_FSISO_TD_STRUCT
{

    struct UX_EHCI_ED_STRUCT              
                    ux_ehci_fsiso_td_link_pointer;
    VOID            *ux_ehci_fsiso_td_transaction0;
    VOID            *ux_ehci_fsiso_td_transaction1;
    VOID            *ux_ehci_fsiso_td_transaction2;
    VOID            *ux_ehci_fsiso_td_transaction3;
    VOID            *ux_ehci_fsiso_td_transaction4;
    VOID            *ux_ehci_fsiso_td_transaction5;
    VOID            *ux_ehci_fsiso_td_transaction6;
    VOID            *ux_ehci_fsiso_td_transaction7;
    VOID            *ux_ehci_fsiso_td_bp0; 
    VOID            *ux_ehci_fsiso_td_bp1; 
    VOID            *ux_ehci_fsiso_td_bp2; 
    VOID            *ux_ehci_fsiso_td_bp3; 
    VOID            *ux_ehci_fsiso_td_bp4; 
    VOID            *ux_ehci_fsiso_td_bp5; 
    VOID            *ux_ehci_fsiso_td_bp6; 
    VOID            *ux_ehci_fsiso_td_bp7; 
    ULONG           ux_ehci_fsiso_td_status;
    struct UX_TRANSFER_STRUCT             
                    *ux_ehci_fsiso_td_transfer_request;
    struct UX_EHCI_TD_STRUCT              
                    *ux_ehci_fsiso_td_next_td_transfer_request;
    struct UX_EHCI_ED_STRUCT              
                    *ux_ehci_fsiso_td_ed;
} UX_EHCI_FSISO_TD;


/* Define EHCI function prototypes.  */

UX_EHCI_TD          *_ux_hcd_ehci_asynch_td_process(UX_EHCI_ED *ed, UX_EHCI_TD *td);
UINT    _ux_hcd_ehci_asynchronous_endpoint_create(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ehci_asynchronous_endpoint_destroy(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ehci_controller_disable(UX_HCD_EHCI *hcd_ehci);
VOID    _ux_hcd_ehci_done_queue_process(UX_HCD_EHCI *hcd_ehci);
VOID    _ux_hcd_ehci_door_bell_wait(UX_HCD_EHCI *hcd_ehci);
UINT    _ux_hcd_ehci_ed_clean(UX_EHCI_ED *ed);
UX_EHCI_ED          *_ux_hcd_ehci_ed_obtain(UX_HCD_EHCI *hcd_ehci);
UINT    _ux_hcd_ehci_endpoint_reset(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ehci_entry(UX_HCD *hcd, UINT function, VOID *parameter);
UINT    _ux_hcd_ehci_frame_number_get(UX_HCD_EHCI *hcd_ehci, ULONG *frame_number);
VOID    _ux_hcd_ehci_frame_number_set(UX_HCD_EHCI *hcd_ehci, ULONG frame_number);
UX_EHCI_FSISO_TD    *_ux_hcd_ehci_fsisochronous_td_obtain(UX_HCD_EHCI *hcd_ehci);
UX_EHCI_HSISO_TD    *_ux_hcd_ehci_hsisochronous_td_obtain(UX_HCD_EHCI *hcd_ehci);
UINT    _ux_hcd_ehci_initialize(UX_HCD *hcd);
UINT    _ux_hcd_ehci_interrupt_endpoint_create(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ehci_interrupt_endpoint_destroy(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint);
VOID    _ux_hcd_ehci_interrupt_handler(VOID);
UINT    _ux_hcd_ehci_isochronous_endpoint_create(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint);
UINT    _ux_hcd_ehci_isochronous_endpoint_destroy(UX_HCD_EHCI *hcd_ehci, UX_ENDPOINT *endpoint);
UX_EHCI_ED          *_ux_hcd_ehci_least_traffic_list_get(UX_HCD_EHCI *hcd_ehci);
VOID    _ux_hcd_ehci_next_td_clean(UX_EHCI_TD *td);
UINT    _ux_hcd_ehci_periodic_tree_create(UX_HCD_EHCI *hcd_ehci);
UINT    _ux_hcd_ehci_port_disable(UX_HCD_EHCI *hcd_ehci, ULONG port_index);
UINT    _ux_hcd_ehci_port_reset(UX_HCD_EHCI *hcd_ehci, ULONG port_index);
UINT    _ux_hcd_ehci_port_resume(UX_HCD_EHCI *hcd_ehci, UINT port_index);
ULONG   _ux_hcd_ehci_port_status_get(UX_HCD_EHCI *hcd_ehci, ULONG port_index);
UINT    _ux_hcd_ehci_port_suspend(UX_HCD_EHCI *hcd_ehci, ULONG port_index);
UINT    _ux_hcd_ehci_power_down_port(UX_HCD_EHCI *hcd_ehci, ULONG port_index);
UINT    _ux_hcd_ehci_power_on_port(UX_HCD_EHCI *hcd_ehci, ULONG port_index);
VOID    _ux_hcd_ehci_power_root_hubs(UX_HCD_EHCI *hcd_ehci);
ULONG   _ux_hcd_ehci_register_read(UX_HCD_EHCI *hcd_ehci, ULONG ehci_register);
VOID    _ux_hcd_ehci_register_write(UX_HCD_EHCI *hcd_ehci, ULONG ehci_register, ULONG value);
UX_EHCI_TD          *_ux_hcd_ehci_regular_td_obtain(UX_HCD_EHCI *hcd_ehci);
UINT    _ux_hcd_ehci_request_bulk_transfer(UX_HCD_EHCI *hcd_ehci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ehci_request_control_transfer(UX_HCD_EHCI *hcd_ehci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ehci_request_interrupt_transfer(UX_HCD_EHCI *hcd_ehci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ehci_request_isochronous_transfer(UX_HCD_EHCI *hcd_ehci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ehci_request_transfer(UX_HCD_EHCI *hcd_ehci, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ehci_request_transfer_add(UX_HCD_EHCI *hcd_ehci, UX_EHCI_ED *ed, ULONG phase, ULONG pid,
                                    ULONG toggle, UCHAR * buffer_address, ULONG buffer_length, UX_TRANSFER *transfer_request);
UINT    _ux_hcd_ehci_transfer_abort(UX_HCD_EHCI *hcd_ehci,UX_TRANSFER *transfer_request);
VOID    _ux_hcd_ehci_transfer_request_process(UX_TRANSFER *transfer_request);

#define ux_hcd_ehci_initialize                      _ux_hcd_ehci_initialize
#define ux_hcd_ehci_interrupt_handler               _ux_hcd_ehci_interrupt_handler

#endif

