/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

#include "typedefs.h"
#include "bcmdevs.h"
#include "wiced_osl.h"
#include "wiced.h"
#include "wiced_utilities.h"
#include "wiced_management.h"
#include "wiced_resource.h"
#include "wiced_framework.h"
#include "command_console.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "platform_dct.h"
#include "platform_usb.h"
#include "ux_api.h"
#include "ux_dcd_bcm4390x.h"
#include "ux_device_stack.h"
#include "ux_device_class_cdc_acm.h"
#include "usb_device_cdc_acm_read_write.h"


/******************************************************
 *                      Macros
 ******************************************************/
/* Parameters to init USB Device */
#define USB_DEVICE_POOL_SIZE                    (64*1024) //Static usage after init for this application used ~36KB
#define USB_DEVICE_DMA_POOL_SIZE                (16*1024) //Static usage after init for this application used ~5KB
#define USB_DEVICE_DMA_POOL_ALIGN               5

/******************************************************
 *                    Constants
 ******************************************************/
/* Parameters to init Console Command */
#define MAX_LINE_LENGTH                         (128)
#define MAX_HISTORY_LENGTH                      (20)

/* Parameters to init USB Device. Usually keep using default values, adjustable for user scenario */
#define USB_DEVICE_THREAD_STACK_SIZE            (4*1024)
#define MAX_DEVICE_CLASS_DRIVER                 (3)

/* Parameters for USB Device Descriptors */
#define DEVICE_FRAMEWORK_LENGTH_FULL_SPEED      (93)
#define DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED      (103)
#define STRING_FRAMEWORK_LENGTH                 (47)
#define LANGUAGE_ID_FRAMEWORK_LENGTH            (2)

/* Application read/write buffer size definition */
#define MAX_CDC_ACM_BUFFER_SIZE                 (1024)

/* ASCII code definition */
#define HEX_ASCII_CHAR_CR                       (0x0D)
#define HEX_ASCII_CHAR_LF                       (0x0A)

/* Application delay time parameters */
#define USB_DEVICE_APP_CONNECT_SCAN_DELAY_TIME  (1000) //ms

/* USB Device console command sets */
#define USB_DEVICE_APP_COMMANDS \
    { (char*) "connect",        usb_device_app_cdc_acm_conn,        0, NULL, NULL, (char*) "",  (char*) "cdc-acm connect" }, \
    { (char*) "shutdown",       usb_device_app_cdc_acm_shutdown,    0, NULL, NULL, (char*) "",  (char*) "cdc-acm shutdown" }, \
    { (char*) "loopback",       usb_device_app_cdc_acm_loopback,    0, NULL, NULL, (char*) "",  (char*) "cdc-acm loopback" }, \
    { (char*) "console_rw",     usb_device_app_cdc_acm_console_rw,  0, NULL, NULL, (char*) "",  (char*) "cdc-acm console connect & loopback" },


/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/
typedef struct
{
    void        *memory_pool;
    uint32_t    memory_pool_size;
    void        *dma_memory_pool;
    uint32_t    dma_memory_pool_size;
} wiced_usb_device_memory_t;

/******************************************************
 *               Variable Definitions
 ******************************************************/
/* USB Device console command sets */
static char line_buffer[MAX_LINE_LENGTH];
static char history_buffer_storage[MAX_LINE_LENGTH * MAX_HISTORY_LENGTH];

static const command_t commands[] =
{
    USB_DEVICE_APP_COMMANDS
    CMD_TABLE_END
};

/* USB Device initialization  */
wiced_usb_device_memory_t               usb20d_memcfg = {0};

platform_usb_device_dci_resource_t      usb20d_dci_info[USB_DEVICE_CONTROLLER_INTERFACE_MAX];
uint32_t                                usb20d_dci_num = 0;

static wiced_bool_t                     usb20d_init_completed = WICED_FALSE;

/* USB CDC-ACM Device class  */
static UX_SLAVE_CLASS_CDC_ACM           *cdc_acm = UX_NULL;
static UCHAR                            cdc_acm_buffer[MAX_CDC_ACM_BUFFER_SIZE];
static UX_SLAVE_CLASS_CDC_ACM_PARAMETER cdc_acm_parameter = {0};

/*
 *
 * [USB Descriptors]
 * http://www.beyondlogic.org/usbnutshell/usb5.shtml#DeviceDescriptors
 *
 *
 */

unsigned char device_framework_full_speed[] = {

    /* Device descriptor     18 bytes
       0x02 bDeviceClass:    CDC class code
       0x00 bDeviceSubclass: CDC class sub code
       0x00 bDeviceProtocol: CDC Device protocol

       idVendor & idProduct - http://www.linux-usb.org/usb.ids
    */
    0x12, 0x01, 0x10, 0x01,
    0xEF, 0x02, 0x01,
    0x08,
    0xB4, 0x04, 0x02, 0x00,
    0x00, 0x01,
    0x01, 0x02, 0x03,
    0x01,

    /* Configuration 1 descriptor 9 bytes */
    0x09, 0x02, 0x4b, 0x00,
    0x02, 0x01, 0x00,
    0x40, 0x00,

    /* Interface association descriptor. 8 bytes.  */
    0x08, 0x0b, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00,

    /* Communication Class Interface Descriptor Requirement. 9 bytes.   */
    0x09, 0x04, 0x00,
    0x00,
    0x01,
    0x02, 0x02, 0x01,
    0x00,

    /* Header Functional Descriptor 5 bytes */
    0x05, 0x24, 0x00,
    0x10, 0x01,

    /* ACM Functional Descriptor 4 bytes */
    0x04, 0x24, 0x02,
    0x0f,

    /* Union Functional Descriptor 5 bytes */
    0x05, 0x24, 0x06,
    0x00,                          /* Master interface */
    0x01,                          /* Slave interface  */

    /* Call Management Functional Descriptor 5 bytes */
    0x05, 0x24, 0x01,
    0x03,
    0x01,                          /* Data interface   */

    /* Endpoint 1 descriptor 7 bytes (Interrupt IN Transfer Type) */
    0x07, 0x05, 0x83,
    0x03,
    0x08, 0x00,
    0xFF,

    /* Data Class Interface Descriptor Requirement 9 bytes */
    0x09, 0x04, 0x01,
    0x00,
    0x02,
    0x0A, 0x00, 0x00,
    0x00,

    /* First alternate setting Endpoint 1 descriptor 7 bytes (Bulk OUT Transfer Type)*/
    0x07, 0x05, 0x02,
    0x02,
    0x40, 0x00,
    0x00,

    /* Endpoint 2 descriptor 7 bytes (Bulk IN Transfer Type) */
    0x07, 0x05, 0x81,
    0x02,
    0x40, 0x00,
    0x00,

};

unsigned char device_framework_high_speed[] = {

    /* Device descriptor
       0x02 bDeviceClass:    CDC class code
       0x00 bDeviceSubclass: CDC class sub code
       0x00 bDeviceProtocol: CDC Device protocol

       idVendor & idProduct - http://www.linux-usb.org/usb.ids
    */
    0x12, 0x01, 0x00, 0x02,
    0xEF, 0x02, 0x01,
    0x40,
    0xB4, 0x04, 0x02, 0x00,
    0x00, 0x01,
    0x01, 0x02, 0x03,
    0x01,

    /* Device qualifier descriptor */
    0x0a, 0x06, 0x00, 0x02,
    0x02, 0x00, 0x00,
    0x40,
    0x01,
    0x00,

    /* Configuration 1 descriptor */
    0x09, 0x02, 0x4b, 0x00,
    0x02, 0x01, 0x00,
   0xc0, 0x01, /* Set bmAttributes to UC_BUS_POWERED|UC_SELF_POWERED, bMaxPower to 2mA */

    /* Interface association descriptor. */
    0x08, 0x0b, 0x00, 0x02, 0x02, 0x02, 0x00, 0x00,

    /* Communication Class Interface Descriptor Requirement */
    0x09, 0x04, 0x00,
    0x00,
    0x01,
    0x02, 0x02, 0x01,
    0x00,

    /* Header Functional Descriptor */
    0x05, 0x24, 0x00,
    0x10, 0x01,

    /* ACM Functional Descriptor */
    0x04, 0x24, 0x02,
    0x0f,

    /* Union Functional Descriptor */
    0x05, 0x24, 0x06,
    0x00,
    0x01,

    /* Call Management Functional Descriptor */
    0x05, 0x24, 0x01,
    0x00,
    0x01,

    /* Endpoint 1 descriptor */
    0x07, 0x05, 0x83,
    0x03,
    0x08, 0x00,
    0xFF,

    /* Data Class Interface Descriptor Requirement */
    0x09, 0x04, 0x01,
    0x00,
    0x02,
    0x0A, 0x00, 0x00,
    0x00,

    /* First alternate setting Endpoint 1 descriptor */
    0x07, 0x05, 0x02,
    0x02,
    0x00, 0x02,
    0x00,

    /* Endpoint 2 descriptor */
    0x07, 0x05, 0x81,
    0x02,
    0x00, 0x02,
    0x00,

};

unsigned char string_framework[] = {

    /* Manufacturer string descriptor : Index 1 - "Cypress Semiconductor" */
    0x09, 0x04, 0x01, 0x15,
    0x43, 0x79, 0x70, 0x72, 0x65, 0x73, 0x73, 0x20,
    0x53, 0x65, 0x6d, 0x69, 0x63, 0x6f, 0x6e, 0x64,
    0x75, 0x63, 0x74, 0x6f, 0x72,

    /* Product string descriptor : Index 2 - "Cypress USB UART" */
    0x09, 0x04, 0x02, 0x10,
    0x43, 0x79, 0x70, 0x72, 0x65, 0x73, 0x73, 0x20,
    0x55, 0x53, 0x42, 0x20, 0x55, 0x41, 0x52, 0x54,

    /* Serial Number string descriptor : Index 3 - "0001" */
    0x09, 0x04, 0x03, 0x04,
    0x30, 0x30, 0x30, 0x31
};

/* Multiple languages are supported on the device, to add
   a language besides english, the unicode language code must
   be appended to the language_id_framework array and the length
   adjusted accordingly. */
unsigned char language_id_framework[] = {

/* English. */
0x09, 0x04
};


/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_result_t wiced_usb_device_cdc_acm_init( void );
static wiced_result_t wiced_usb_device_cdc_acm_deinit( void );
static void wiced_usb_device_usbx_isr( void );
static UINT wiced_usb_device_usbx_evt_callback( ULONG event );
static void cdc_acm_instance_activate( void  *cdc_acm_instance );
static void cdc_acm_instance_deactivate( void *cdc_acm_instance );

/******************************************************
 *               Function Definitions
 ******************************************************/
void application_start( void )
{
    /* Initialize the device */
    wiced_init();

    WPRINT_APP_INFO( ("\n") );
    WPRINT_APP_INFO( ("+-------------------------------------------------+\n") );
    WPRINT_APP_INFO( ("+ USB20 Device CDC-ACM Class Application          +\n") );
    WPRINT_APP_INFO( ("+-------------------------------------------------+\n") );

    /* Run console application function  */
    command_console_init( STDIO_UART, MAX_LINE_LENGTH, line_buffer, MAX_HISTORY_LENGTH, history_buffer_storage, " " );
    console_add_cmd_table( commands );
}

static wiced_result_t wiced_usb_device_cdc_acm_init( void )
{
    uint pool_size;
    dmaaddr_t pool_base;
    platform_usb_device_dci_resource_t *brcm_resource = NULL;
    UX_USER_CONFIG_DEVICE ux_user_obj = {0};
    UINT status;
    UINT i;

    WPRINT_APP_INFO( ("USB CDC-ACM Device init starting...\n") );

    /* Sanity check  */
    if ( usb20d_init_completed == WICED_TRUE )
    {
        WPRINT_APP_ERROR( ("Already init!\n") );
        goto exit;
    }

    /* Allocate memory pool  */
    if ( (usb20d_memcfg.memory_pool = malloc( USB_DEVICE_POOL_SIZE )) == NULL )
    {
        WPRINT_APP_ERROR( ("Alloc usb memory pool failed!\n") );
        goto exit;
    }
    usb20d_memcfg.memory_pool_size = USB_DEVICE_POOL_SIZE;

    if ( (usb20d_memcfg.dma_memory_pool = osl_dma_alloc_consistent( USB_DEVICE_DMA_POOL_SIZE, USB_DEVICE_DMA_POOL_ALIGN, &pool_size, &pool_base )) == NULL )
    {
        WPRINT_APP_ERROR( ("Alloc usb dma memory pool failed!\n") );
        goto exit;
    }
    usb20d_memcfg.dma_memory_pool_size = pool_size;

    /* Initialize USBX Memory */
    status = ux_system_initialize( usb20d_memcfg.memory_pool, usb20d_memcfg.memory_pool_size, usb20d_memcfg.dma_memory_pool, usb20d_memcfg.dma_memory_pool_size );
    WPRINT_APP_DEBUG( ("Initialized USBX. status=%d\n", status) );

    /* The code below is required for installing the device portion of USBX.  */
    /* Initialize USBX Device Stack (Use USBX system default values if given UX_USER_CONFIG_DEVICE as NULL) */
    ux_user_obj.ux_user_config_device_max_class         = MAX_DEVICE_CLASS_DRIVER;
    ux_user_obj.ux_user_config_device_thread_stack_size = USB_DEVICE_THREAD_STACK_SIZE;

    status = ux_device_stack_initialize( device_framework_high_speed, DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED,
                                         device_framework_full_speed, DEVICE_FRAMEWORK_LENGTH_FULL_SPEED,
                                         string_framework, STRING_FRAMEWORK_LENGTH,
                                         language_id_framework, LANGUAGE_ID_FRAMEWORK_LENGTH,
                                         wiced_usb_device_usbx_evt_callback,
                                         &ux_user_obj );
    if( status != UX_SUCCESS )
    {
        WPRINT_APP_ERROR( ("USBX device stack init failed. status=%d\n", status) );
        goto exit;
    }

    /* Set the parameters for callback when insertion/extraction of a CDC-ACM device.  */
    cdc_acm_parameter.ux_slave_class_cdc_acm_instance_activate   = cdc_acm_instance_activate;
    cdc_acm_parameter.ux_slave_class_cdc_acm_instance_deactivate = cdc_acm_instance_deactivate;

    /* Initialize the device cdc-acm class. This class owns both interfaces starting with 0. */
    status = ux_device_stack_class_register( _ux_system_slave_class_cdc_acm_name, ux_device_class_cdc_acm_entry, 1, 0, &cdc_acm_parameter );
    if( status != UX_SUCCESS )
    {
        WPRINT_APP_ERROR( ("USBX device cdc-acm class register failed. status=%d\n", status) );
        goto exit;
    }

    /* Init USB20 Device HW.  */
    WPRINT_APP_DEBUG( ("Init USB20 Device HW\n") );
    status = platform_usb_device_init();
    if ( status != PLATFORM_SUCCESS )
    {
        WPRINT_APP_ERROR( ("USB20 Device HW init failed. status=%d\n", status) );
        goto exit;
    }
    status = platform_usb_device_init_irq( wiced_usb_device_usbx_isr );
    if ( status != PLATFORM_SUCCESS )
    {
        WPRINT_APP_ERROR( ("USB20 Device irq init failed. status=%d\n", status) );
        goto exit;
    }
    WPRINT_APP_DEBUG( ("USB20 Device HW init ok!\n") );

    /* Obtain USB20 Device DCI resources for USB device controller driver  */
    status = platform_usb_device_get_dci_resource( (platform_usb_device_dci_resource_t *) usb20d_dci_info, sizeof( usb20d_dci_info ), &usb20d_dci_num );
    if ( status != PLATFORM_SUCCESS )
    {
        WPRINT_APP_ERROR( ("USB20 Device DCI resources get failed. status=%d\n", status) );
        goto exit;
    }
    WPRINT_APP_DEBUG( ("USB20 Device HW supports %lu DCI resources\n", usb20d_dci_num) );

    for (i = 0; i < usb20d_dci_num; i ++)
    {
        if (usb20d_dci_info[i].usb_device_dci_type == USB_DEVICE_CONTROLLER_INTERFACE_BRCM)
        {
            brcm_resource = &usb20d_dci_info[i];
        }
    }
    if ( brcm_resource == NULL )
    {
        WPRINT_APP_ERROR( ("No USB20 Device resources for USBX\n") );
        goto exit;
    }

    /* Init USBX Device controller  */
    WPRINT_APP_INFO( ("USB20 Device io=0x%08lx, irq=%lu\n", brcm_resource->usb_device_dci_ioaddress, brcm_resource->usb_device_dci_irq_number) );

    status = ux_dcd_bcm4390x_initialize(brcm_resource->usb_device_dci_ioaddress, (ULONG)(brcm_resource->usb_device_dci_private_data));
    if( status != UX_SUCCESS )
    {
        WPRINT_APP_ERROR( ("USBX BCM4390x device controller init failed. status=%d\n", status) );
        goto exit;
    }
    WPRINT_APP_DEBUG( ("USBX BCM4390x device controller init ok!\n") );

    /* Enable USB20 Device interrupt  */
    status = platform_usb_device_enable_irq();
    if ( status != PLATFORM_SUCCESS )
    {
        WPRINT_APP_ERROR( ("USB20 Device irq enable failed. status=%d\n", status) );
        goto exit;
    }
    WPRINT_APP_DEBUG( ("USB20 IRQ enabled!\n") );

    /* Set init completed flag  */
    usb20d_init_completed = WICED_TRUE;
    WPRINT_APP_INFO( ("USB20 Device init completed!!!\n") );

    return WICED_SUCCESS;

exit:
    if(usb20d_memcfg.memory_pool)
        free(usb20d_memcfg.memory_pool);
    if(usb20d_memcfg.dma_memory_pool)
        osl_dma_free_consistent(usb20d_memcfg.dma_memory_pool);

    return WICED_ERROR;
}

static wiced_result_t wiced_usb_device_cdc_acm_deinit( void )
{
    WPRINT_APP_INFO( ("USB CDC-ACM Device deinit starting...\n") );

    /* Sanity check  */
    if ( usb20d_init_completed != WICED_TRUE )
    {
        WPRINT_APP_ERROR( ("Not init yet!\n") );
        goto exit;
    }

    /* Disconnect device stack.  */
    _ux_device_stack_disconnect();

    /* Unregister the class.  */
    _ux_device_stack_class_unregister(_ux_system_slave_class_cdc_acm_name, ux_device_class_cdc_acm_entry);

    /* Deinitialize the device side of USBX.  */
    _ux_device_stack_uninitialize();

    /* Deinitialize the device controller driver.  */
    _ux_dcd_bcm4390x_uninitialize();
    WPRINT_APP_DEBUG( ("USBX BCM4390x device controller deinit ok!\n") );

    /* And finally deinit the USBX system resources.  */
    _ux_system_uninitialize();

    /* Free memory used by USBX system.  */
    if(usb20d_memcfg.memory_pool)
        free(usb20d_memcfg.memory_pool);
    if(usb20d_memcfg.dma_memory_pool)
        osl_dma_free_consistent(usb20d_memcfg.dma_memory_pool);

    WPRINT_APP_DEBUG( ("USBX system deinit completed!\n") );

    /* Disable USB20 Device interrupt & Deinit USB20 Device HW.  */
    platform_usb_device_deinit();
    WPRINT_APP_DEBUG( ("USB20 Device HW deinit completed!\n") );

    /* Reset init completed flag  */
    usb20d_init_completed = WICED_FALSE;
    WPRINT_APP_INFO( ("USB20 Device deinit completed!!!\n") );

    return WICED_SUCCESS;

exit:
    return WICED_ERROR;
}

static void wiced_usb_device_usbx_isr( void )
{
    _ux_dcd_bcm4390x_interrupt_handler();
}

static void cdc_acm_instance_activate( void  *cdc_acm_instance )
{
    /* Save the CDC-ACM instance.  */
    WPRINT_APP_DEBUG( ("USB20D CDC-ACM device activate\n") );
    cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*) cdc_acm_instance;
}

static void cdc_acm_instance_deactivate( void *cdc_acm_instance )
{
    /* Reset the CDC-ACM instance.  */
    WPRINT_APP_DEBUG( ("USB20D CDC-ACM device deactivate\n") );
    cdc_acm = UX_NULL;
}

static UINT wiced_usb_device_usbx_evt_callback( ULONG event )
{
    WPRINT_APP_DEBUG( ("USB20 Device: event (%ld)\n", event) );

    switch ( event )
    {
        case UX_DEVICE_RESET:
            WPRINT_APP_DEBUG( ("UX_DEVICE_RESET\n") );
            break;

        case UX_DEVICE_ATTACHED:
            WPRINT_APP_DEBUG( ("UX_DEVICE_ATTACHED\n") );
            break;

        case UX_DEVICE_ADDRESSED:
            WPRINT_APP_DEBUG( ("UX_DEVICE_ADDRESSED\n") );
            break;

        case UX_DEVICE_CONFIGURED:
            WPRINT_APP_DEBUG( ("UX_DEVICE_CONFIGURED\n") );
            break;

        case UX_DEVICE_SUSPENDED:
            WPRINT_APP_DEBUG( ("UX_DEVICE_SUSPENDED\n") );
            break;

        case UX_DEVICE_RESUMED:
            WPRINT_APP_DEBUG( ("UX_DEVICE_RESUMED\n") );
            break;

        case UX_DEVICE_SELF_POWERED_STATE:
            WPRINT_APP_DEBUG( ("UX_DEVICE_SELF_POWERED_STATE\n") );
            break;

        case UX_DEVICE_BUS_POWERED_STATE:
            WPRINT_APP_DEBUG( ("UX_DEVICE_BUS_POWERED_STATE\n") );
            break;

        case UX_DEVICE_REMOTE_WAKEUP:
            WPRINT_APP_DEBUG( ("UX_DEVICE_REMOTE_WAKEUP\n") );
            break;

        case UX_DEVICE_BUS_RESET_COMPLETED:
            WPRINT_APP_DEBUG( ("UX_DEVICE_BUS_RESET_COMPLETED\n") );
            break;

        case UX_DEVICE_REMOVED:
            WPRINT_APP_DEBUG( ("UX_DEVICE_REMOVED\n") );
            break;

        case UX_DEVICE_FORCE_DISCONNECT:
            WPRINT_APP_DEBUG( ("UX_DEVICE_FORCE_DISCONNECT\n") );
            break;

        default :
            WPRINT_APP_DEBUG( ("USB20 Device: unknown event (%ld)\n", event) );
            break;
    }

    return UX_SUCCESS;
}

int usb_device_app_cdc_acm_conn( int argc, char* argv[] )
{
    UINT status;

    WPRINT_APP_INFO( ("+++START: cdc-acm connect\n") );

    /* Init USB CDC-ACM Device SW & USB20 Device HW */
    status = wiced_usb_device_cdc_acm_init();
    if( status != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("USB CDC-ACM Device init failed. status=%d\n", status) );
        return ERR_UNKNOWN;
    }

    WPRINT_APP_INFO( ("USB CDC-ACM Device init ok!\n") );

    /* Wait CDC class is mounted  */
    while ( cdc_acm == UX_NULL )
    {
        WPRINT_APP_INFO( ("Waiting for CDC-ACM device connecting...\n") );
        wiced_rtos_delay_milliseconds( USB_DEVICE_APP_CONNECT_SCAN_DELAY_TIME );
    }
    WPRINT_APP_INFO( ("\n*** USB CDC-ACM device connected OK!!! ***\n\n") );

    WPRINT_APP_INFO( ("\n*** Note that: ***\n") );
    WPRINT_APP_INFO( ("1). Now please setup/config the correct Serial-Port on your terminal program.\n") );
    WPRINT_APP_INFO( ("2). After terminal program setup/config, execute <loopback> command.\n\n") );

    WPRINT_APP_INFO( ("---END: cdc-acm connect\n") );
    return ERR_CMD_OK;
}

int usb_device_app_cdc_acm_shutdown( int argc, char* argv[] )
{
    UINT status;

    WPRINT_APP_INFO( ("+++START: cdc-acm shutdown\n") );

    /* Deinit USB CDC-ACM Device SW & USB20 Device HW */
    status = wiced_usb_device_cdc_acm_deinit();
    if( status != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("USB CDC-ACM Device deinit failed. status=%d\n", status) );
        return ERR_UNKNOWN;
    }

    WPRINT_APP_INFO( ("USB CDC-ACM Device deinit ok!\n") );

    WPRINT_APP_INFO( ("---END: cdc-acm shutdown\n") );
    return ERR_CMD_OK;
}

int usb_device_app_cdc_acm_loopback( int argc, char* argv[] )
{
    UINT status;
    ULONG actual_length;
    ULONG requested_length;

    WPRINT_APP_INFO( ("+++START: cdc-acm loopback\n") );
    WPRINT_APP_INFO( ("\n*** Please input any key from your terminal program ***\n\n") );

    UNUSED_PARAMETER(status);
    UNUSED_PARAMETER(actual_length);
    UNUSED_PARAMETER(requested_length);

    while( 1 )
    {
        /* Ensure the CDC class is mounted.  */
        while( cdc_acm != UX_NULL )
        {
            /* Read from the CDC class.  */
            status = ux_device_class_cdc_acm_read( cdc_acm, cdc_acm_buffer, MAX_CDC_ACM_BUFFER_SIZE, &actual_length );

            if ( actual_length != 0 )
            {
                prhex("\nRx", cdc_acm_buffer, actual_length);
            }
            if ( (actual_length == 1) && ((cdc_acm_buffer[0] >= 0x20) && (cdc_acm_buffer[0] <= 0x7E)) )
            {
                WPRINT_APP_INFO( ("  Printable ascii key = %c\n", cdc_acm_buffer[0]) );
            }

            /* The actual length becomes the requested length.  */
            requested_length = actual_length;

            /* Check the status.  If OK, we will write to the CDC instance.  */
            status = ux_device_class_cdc_acm_write( cdc_acm, cdc_acm_buffer, requested_length, &actual_length );

            /* Check for CR/LF.  */
            if ( cdc_acm_buffer[requested_length - 1] == '\r' )
            {
                /* Copy LF value into user buffer.  */
                ux_utility_memory_copy( cdc_acm_buffer, "\n",  1 );

                /* And send it again.  */
                status = ux_device_class_cdc_acm_write( cdc_acm, cdc_acm_buffer, 1, &actual_length );
            }
        }

        WPRINT_APP_INFO( ("Waiting for CDC-ACM device connecting...\n") );
        wiced_rtos_delay_milliseconds( USB_DEVICE_APP_CONNECT_SCAN_DELAY_TIME );
    }

    WPRINT_APP_INFO( ("---END: cdc-acm loopback\n") );
    return ERR_CMD_OK;
}

int usb_device_app_cdc_acm_console_rw( int argc, char* argv[] )
{
    UINT status;
    ULONG actual_length;
    ULONG requested_length;

    WPRINT_APP_INFO( ("+++START: cdc-acm console connect & loopback\n") );

    status = wiced_usb_device_cdc_acm_init();
    if( status != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("USB CDC-ACM Device init failed. status=%d\n", status) );
        return ERR_UNKNOWN;
    }
    WPRINT_APP_INFO( ("USB CDC-ACM Device init ok!\n") );

    /* Wait CDC class is mounted  */
    while ( cdc_acm == UX_NULL )
    {
        WPRINT_APP_INFO( ("Waiting for CDC-ACM device connecting...\n") );
        wiced_rtos_delay_milliseconds( USB_DEVICE_APP_CONNECT_SCAN_DELAY_TIME );
    }
    WPRINT_APP_INFO( ("\n*** USB CDC-ACM device connected OK!!! ***\n\n") );
    WPRINT_APP_INFO( ("\n*** Please input any key from your terminal program ***\n\n") );

    while( 1 )
    {
        /* Ensure the CDC class is mounted.  */
        while( cdc_acm != UX_NULL )
        {
            /* Read from the CDC class.  */
            status = ux_device_class_cdc_acm_read( cdc_acm, cdc_acm_buffer, MAX_CDC_ACM_BUFFER_SIZE, &actual_length );

            if ( actual_length != 0 )
            {
                prhex("\nRx", cdc_acm_buffer, actual_length);
            }
            if ( (actual_length == 1) && ((cdc_acm_buffer[0] >= 0x20) && (cdc_acm_buffer[0] <= 0x7E)) )
            {
                WPRINT_APP_INFO( ("  Printable ascii key = %c\n", cdc_acm_buffer[0]) );
            }

            /* The actual length becomes the requested length.  */
            requested_length = actual_length;

            /* Check the status.  If OK, we will write to the CDC instance.  */
            status = ux_device_class_cdc_acm_write( cdc_acm, cdc_acm_buffer, requested_length, &actual_length );

            /* Check for CR/LF.  */
            if ( cdc_acm_buffer[requested_length - 1] == '\r' )
            {
                /* Copy LF value into user buffer.  */
                ux_utility_memory_copy( cdc_acm_buffer, "\n",  1 );

                /* And send it again.  */
                status = ux_device_class_cdc_acm_write( cdc_acm, cdc_acm_buffer, 1, &actual_length );
            }
        }

        WPRINT_APP_INFO( ("Waiting for CDC-ACM device connecting...\n") );
        wiced_rtos_delay_milliseconds( USB_DEVICE_APP_CONNECT_SCAN_DELAY_TIME );
    }

    WPRINT_APP_INFO( ("---END: cdc-acm console connect & loopback\n") );
    return ERR_CMD_OK;
}

