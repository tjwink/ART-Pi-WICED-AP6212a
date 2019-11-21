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
/**   User Specific                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/


/**************************************************************************/ 
/*                                                                        */ 
/*  PORT SPECIFIC C INFORMATION                            RELEASE        */ 
/*                                                                        */ 
/*    ux_user.h                                           PORTABLE C      */ 
/*                                                           5.8          */ 
/*                                                                        */
/*  AUTHOR                                                                */ 
/*                                                                        */ 
/*    Thierry Giron, Express Logic Inc.                                   */ 
/*                                                                        */ 
/*  DESCRIPTION                                                           */ 
/*                                                                        */ 
/*    This file contains user defines for configuring USBX in specific    */ 
/*    ways. This file will have an effect only if the application and     */ 
/*    USBX library are built with UX_INCLUDE_USER_DEFINE_FILE defined.    */ 
/*    Note that all the defines in this file may also be made on the      */ 
/*    command line when building USBX library and application objects.    */ 
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
/*                                            added host/device split     */ 
/*                                            #define                     */ 
/*                                            resulting in version 5.8    */ 
/*                                                                        */ 
/**************************************************************************/ 

#ifndef UX_USER_H
#define UX_USER_H


/* Define various build options for the USBX port.  The application should either make changes
   here by commenting or un-commenting the conditional compilation defined OR supply the defines 
   though the compiler's equivalent of the -D option.  */

#define UX_THREAD_STACK_SIZE                      (4 * 1024)

/* #WICED#: Isolate host and slave mode thread stack sizes and make them configurable from user-layer. */
#define UX_HOST_THREAD_STACK_SIZE                 ( _ux_system_host -> ux_system_host_thread_stack_size )
#define UX_SLAVE_THREAD_STACK_SIZE                ( _ux_system_slave -> ux_system_slave_thread_stack_size )


/* Override various options with default values already assigned in ux_api.h or ux_port.h. Please 
   also refer to ux_port.h for descriptions on each of these options.  */

/* Defined, this value represents how many ticks per seconds for a specific hardware platform. 
   The default is 1000 indicating 1 tick per millisecond.  */

/* #define UX_PERIODIC_RATE 1000
*/
//#define UX_PERIODIC_RATE (TX_TIMER_TICKS_PER_SECOND)   /* #WICED#: WICED use default, 11/03/2017. */

/* Defined, this value is the maximum number of classes that can be loaded by USBX. This value
   represents the class container and not the number of instances of a class. For instance, if a
   particular implementation of USBX needs the hub class, the printer class, and the storage
   class, then the UX_MAX_CLASSES value can be set to 3 regardless of the number of devices 
   that belong to these classes.  */

/* #define UX_MAX_CLASSES  3
*/


/* Defined, this value is the maximum number of classes in the device stack that can be loaded by
   USBX.  */

/* #define UX_MAX_SLAVE_CLASSES    1
*/

/* Defined, this value is the maximum number of interfaces in the device framework.  */

/* #define UX_MAX_SLAVE_INTERFACES    16
*/

/* Defined, this value represents the number of different host controllers available in the system. 
   For USB 1.1 support, this value will usually be 1. For USB 2.0 support, this value can be more 
   than 1. This value represents the number of concurrent host controllers running at the same time. 
   If for instance there are two instances of OHCI running, or one EHCI and one OHCI controller
   running, the UX_MAX_HCD should be set to 2.  */

/* #define UX_MAX_HCD  1
*/


/* Defined, this value represents the maximum number of devices that can be attached to the USB.
   Normally, the theoretical maximum number on a single USB is 127 devices. This value can be 
   scaled down to conserve memory. Note that this value represents the total number of devices 
   regardless of the number of USB buses in the system.  */

/* #define UX_MAX_DEVICES  127
*/


/* Defined, this value represents the current number of SCSI logical units represented in the device
   storage class driver.  */

/* #define UX_MAX_SLAVE_LUN    1
*/


/* Defined, this value represents the maximum number of SCSI logical units represented in the
   host storage class driver.  */
   
/* #define UX_MAX_HOST_LUN 1
*/


/* Defined, this value represents the maximum number of bytes received on a control endpoint in
   the device stack. The default is 256 bytes but can be reduced in memory constraint environments.  */

/* #define UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH 256
*/


/* Defined, this value represents the maximum number of bytes received on a bulk endpoint in the 
   device stack. The default is 4096 bytes but can be reduced in memory constraint environments.  
   For cd-rom support in the storage class, this value cannot be less that 2048. */

#define UX_SLAVE_REQUEST_DATA_MAX_LENGTH    (1024 * 2)


/* Defined, this value represents the maximum number of bytes that a storage payload can send/receive.
   The default is 8K bytes but can be reduced in memory constraint environments.  */
#define UX_HOST_CLASS_STORAGE_MEMORY_BUFFER_SIZE            (1024 * 8)

/* Defined, this value represents the maximum number of Ed, regular TDs and Isochronous TDs. These values
   depend on the type of host controller and can be reduced in memory constraint environments.  */

#define UX_MAX_ED                                           80
#define UX_MAX_TD                                           128
#define UX_MAX_ISO_TD                                       64 //1 //#WICED#: Enlarge ISO TD limit to get ISO transfer running

/* Defined, this value represents the maximum size of the HID decompressed buffer. This cannot be determined
   in advance so we allocate a big block, usually 4K but for simple HID devices like keyboard and mouse
   it can be reduced a lot. */

#define UX_HOST_CLASS_HID_DECOMPRESSION_BUFFER              128

/* Defined, this value represents the maximum number of HID usages for a HID device. 
   Default is 1024 but for simple HID devices like keyboard and mouse it can be reduced a lot. */

#define UX_HOST_CLASS_HID_USAGES                            512


/* Defined, this value represents the maximum number of media for the host storage class. 
   Default is 8 but for memory contrained resource systems this can ne reduced to 1. */

#define UX_HOST_CLASS_STORAGE_MAX_MEDIA                     2

/* Defined, this value forces the memory allocation scheme to enforce alignement
   of memory with the UX_SAFE_ALIGN field
*/

/* #define UX_ENFORCE_SAFE_ALIGNMENT   */

/* Defined, this value represents the number of packets in the CDC_ECM device class.
*/

#define UX_DEVICE_CLASS_CDC_ECM_NX_PKPOOL_ENTRIES           4

/* Define, this value will only enable the host side of usbx.  */
/* #define UX_HOST_SIDE_ONLY   */

/* Define, this value will only enable the device side of usbx.  */
/* #define UX_DEVICE_SIDE_ONLY   */

/* Defined, this value will include the OTG polling thread. OTG can only be active if both host/device are present.
*/

#ifndef UX_HOST_SIDE_ONLY 
#ifndef UX_DEVICE_SIDE_ONLY 

/* #define UX_OTG_SUPPORT */

#endif 
#endif 

/* Defined, this value represents the maximum size of single tansfers for the SCSI data phase.
*/

//#define UX_HOST_CLASS_STORAGE_MAX_TRANSFER_SIZE             (1024 * 1)
#define UX_HOST_CLASS_STORAGE_MAX_TRANSFER_SIZE             (1024 * 64)   /* #WICED#: WICED use, 07/24/2015. */

/* Defined, this value represents the size of the log pool.
*/
#define UX_DEBUG_LOG_SIZE                                   (1024 * 16)


/* DEBUG includes and macros for a specific platform go here.  */
#ifdef UX_INCLUDE_USER_DEFINE_BSP
#include "usb_bsp.h"
#include "usbh_hcs.h"
#include "usbh_stdreq.h"
#include "usbh_core.h"
#endif 

#endif 

