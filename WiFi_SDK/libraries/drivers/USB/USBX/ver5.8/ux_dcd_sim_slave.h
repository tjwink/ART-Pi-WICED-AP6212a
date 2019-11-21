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
/**   Slave Simulator Controller Driver                                   */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  COMPONENT DEFINITION                                   RELEASE        */ 
/*                                                                        */ 
/*    ux_dcd_sim_slave.h                                  PORTABLE C      */ 
/*                                                           5.8          */ 
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Thierry Giron, Express Logic Inc.                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This file contains all the header and extern functions used by the  */
/*    USBX slave simulator. It is designed to work ONLY with the USBX     */ 
/*    host simulator.                                                     */ 
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

#ifndef UX_DCD_SIM_SLAVE_H
#define UX_DCD_SIM_SLAVE_H


/* Define USB slave simulator major equivalences.  */

#define UX_DCD_SIM_SLAVE_SLAVE_CONTROLLER                       98
#define UX_DCD_SIM_SLAVE_MAX_ED                                 16


/* Define USB slave simulator error code register bits.  */

#define UX_DCD_SIM_SLAVE_ERROR_TRANSMISSION_OK                  0x00000001
#define UX_DCD_SIM_SLAVE_ERROR_CODE_MASK                        0x0000000e
#define UX_DCD_SIM_SLAVE_ERROR_CODE_SHIFT                       0x00000001
#define UX_DCD_SIM_SLAVE_ERROR_CODE_PID_ERROR                   0x00000001
#define UX_DCD_SIM_SLAVE_ERROR_CODE_PID_UNKNOWN                 0x00000002
#define UX_DCD_SIM_SLAVE_ERROR_CODE_UNEXPECTED_PACKET           0x00000003
#define UX_DCD_SIM_SLAVE_ERROR_CODE_TOKEN_CRC                   0x00000004
#define UX_DCD_SIM_SLAVE_ERROR_CODE_DATA_CRC                    0x00000005
#define UX_DCD_SIM_SLAVE_ERROR_CODE_TIME_OUT                    0x00000006
#define UX_DCD_SIM_SLAVE_ERROR_CODE_BABBLE                      0x00000007
#define UX_DCD_SIM_SLAVE_ERROR_CODE_UNEXPECTED_EOP              0x00000008
#define UX_DCD_SIM_SLAVE_ERROR_CODE_NAK                         0x00000009
#define UX_DCD_SIM_SLAVE_ERROR_CODE_STALLED                     0x0000000a
#define UX_DCD_SIM_SLAVE_ERROR_CODE_OVERFLOW                    0x0000000b
#define UX_DCD_SIM_SLAVE_ERROR_CODE_EMPTY_PACKET                0x0000000c
#define UX_DCD_SIM_SLAVE_ERROR_CODE_BIT_STUFFING                0x0000000d
#define UX_DCD_SIM_SLAVE_ERROR_CODE_SYNC_ERROR                  0x0000000e
#define UX_DCD_SIM_SLAVE_ERROR_CODE_DATA_TOGGLE                 0x0000000f


/* Define USB slave simulator physical endpoint status definition.  */

#define UX_DCD_SIM_SLAVE_ED_STATUS_UNUSED                       0
#define UX_DCD_SIM_SLAVE_ED_STATUS_USED                         1
#define UX_DCD_SIM_SLAVE_ED_STATUS_TRANSFER                     2
#define UX_DCD_SIM_SLAVE_ED_STATUS_STALLED                      4


/* Define USB slave simulator physical endpoint structure.  */

typedef struct UX_DCD_SIM_SLAVE_ED_STRUCT 
{

    ULONG           ux_sim_slave_ed_status;
    ULONG           ux_sim_slave_ed_index;
    ULONG           ux_sim_slave_ed_payload_length;
    ULONG           ux_sim_slave_ed_ping_pong;
    ULONG           ux_sim_slave_ed_status_register;
    ULONG           ux_sim_slave_ed_configuration_value;
    struct UX_SLAVE_ENDPOINT_STRUCT             
                    *ux_sim_slave_ed_endpoint;
} UX_DCD_SIM_SLAVE_ED;


/* Define USB slave simulator DCD structure definition.  */

typedef struct UX_DCD_SIM_SLAVE_STRUCT
{                                 

    struct UX_SLAVE_DCD_STRUCT                 
                    *ux_dcd_sim_slave_dcd_owner;
    struct UX_DCD_SIM_SLAVE_ED_STRUCT              
                    ux_dcd_sim_slave_ed[UX_DCD_SIM_SLAVE_MAX_ED];
} UX_DCD_SIM_SLAVE;


/* Define slave simulator function prototypes.  */

UINT    _ux_dcd_sim_slave_address_set(UX_DCD_SIM_SLAVE *dcd_sim_slave, ULONG address);
UINT    _ux_dcd_sim_slave_endpoint_create(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_ENDPOINT *endpoint);
UINT    _ux_dcd_sim_slave_endpoint_destroy(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_ENDPOINT *endpoint);
UINT    _ux_dcd_sim_slave_endpoint_reset(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_ENDPOINT *endpoint);
UINT    _ux_dcd_sim_slave_endpoint_stall(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_ENDPOINT *endpoint);
UINT    _ux_dcd_sim_slave_endpoint_status(UX_DCD_SIM_SLAVE *dcd_sim_slave, ULONG endpoint_index);
UINT    _ux_dcd_sim_slave_frame_number_get(UX_DCD_SIM_SLAVE *dcd_sim_slave, ULONG *frame_number);
UINT    _ux_dcd_sim_slave_function(UX_SLAVE_DCD *dcd, UINT function, VOID *parameter);
UINT    _ux_dcd_sim_slave_initialize(VOID);
UINT    _ux_dcd_sim_slave_initialize_complete(VOID);
UINT    _ux_dcd_sim_slave_state_change(UX_DCD_SIM_SLAVE *dcd_sim_slave, ULONG state);
UINT    _ux_dcd_sim_slave_transfer_request(UX_DCD_SIM_SLAVE *dcd_sim_slave, UX_SLAVE_TRANSFER *transfer_request);

/* Define Device Simulator Class API prototypes.  */

#define ux_dcd_sim_slave_initialize                 _ux_dcd_sim_slave_initialize
#endif

