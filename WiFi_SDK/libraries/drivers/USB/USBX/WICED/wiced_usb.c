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

/** @file
 *  Wiced USB Stack/Driver layer - USBX
 */

#include "typedefs.h"
#include "bcmdevs.h"
#include "wiced_osl.h"
#include "wiced.h"
#include "wiced_usb.h"
#include "wiced_usb_usbx.h"
#include "wiced_utilities.h"
#include "wiced_filesystem.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "platform_dct.h"
#include "platform_usb.h"
#include "internal/wiced_internal_api.h"
#include "wiced_framework.h"
#include "ux_api.h"
#include "ux_hcd_ehci.h"
#include "ux_hcd_ohci.h"
#include "ux_host_class_hub.h"
#include "ux_host_class_storage.h"
#include "ux_host_class_audio.h"
#include "ux_host_class_hid.h"
#include "ux_host_class_hid_keyboard.h"
#include "ux_host_class_hid_mouse.h"
#include "ux_host_class_hid_remote_control.h"
#include "ux_host_class_cdc_acm.h"
#include "ux_host_class_prolific.h"

/******************************************************
 *                      Macros
 ******************************************************/
#define USB_POOL_SIZE           (128*1024) //Use 80KB or above for application
#define USB_DMA_POOL_SIZE       (128*1024) //Use 80KB or above for application
#define USB_DMA_POOL_ALIGN      5

/******************************************************
 *                    Constants
 ******************************************************/

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
} wiced_usb_memory_t;

/******************************************************
 *                 Static Variables
 ******************************************************/
/* USB Host initialization  */
wiced_usb_user_config_t             usb20h_user_cfg = {0};
wiced_usb_memory_t                  usb20h_memcfg = {0};

platform_usb_host_hci_resource_t    usb20h_hci_info[USB_HOST_CONTROLLER_INTERFACE_MAX];
uint32_t                            usb20h_hci_num = 0;

/* Host STORAGE class (Noted that: Now support only ONE storage partition!) */
static UX_HOST_CLASS_STORAGE        *storage = NULL;
static UX_HOST_CLASS_STORAGE_MEDIA  *storage_media = NULL;
static FX_MEDIA                     *media = NULL;
static wiced_filesystem_t           usbdisk_media_handle = {0};
static filesystem_list_t            *usbdisk_filesystem = NULL;

/* Host AUDIO class  */
static UX_HOST_CLASS_AUDIO          *audio[USB_AUDIO_DEVICE_HANDLE_MAX];
static uint32_t                     audio_device_index = 0;

/* Host HID class  */
static UX_HOST_CLASS_HID            *hid;
static UX_HOST_CLASS_HID_CLIENT     *hid_client;
static UX_HOST_CLASS_HID_KEYBOARD   *keyboard;
static UX_HOST_CLASS_HID_MOUSE      *mouse;

/* Host CDC-ACM class  */
static UX_HOST_CLASS_CDC_ACM            *cdc_acm[USB_CDC_ACM_DEVICE_HANDLE_MAX];
static uint32_t                         cdc_acm_device_index = 0;

/* Host Prolific PL2303 (CDC-ACM Like) class  */
static UX_HOST_CLASS_PROLIFIC           *prolific;

/******************************************************
 *               Function Declarations
 ******************************************************/
static void wiced_usb_host_usbx_isr( void );
static UINT wiced_usb_host_usbx_host_evt_callback( ULONG evt, UX_HOST_CLASS *class, void *instance );

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t wiced_usb_host_init( wiced_usb_user_config_t *config )
{
    uint pool_size;
    dmaaddr_t pool_base;
#ifndef WICED_USB_OHCI_ONLY
    platform_usb_host_hci_resource_t *ehci_resource = NULL;
#endif  /* WICED_USB_OHCI_ONLY */
    platform_usb_host_hci_resource_t *ohci_resource = NULL;
    UX_USER_CONFIG_HOST ux_user_obj = {0};
    UINT status;
    UINT i;

    WPRINT_PLATFORM_INFO( ("USB20 Host init starting...\n") );

    /* Get user config from application  */
    memcpy(&usb20h_user_cfg, config, sizeof(wiced_usb_user_config_t));

    /* Allocate memory pool  */
    if ((usb20h_memcfg.memory_pool = malloc(USB_POOL_SIZE)) == NULL)
    {
        WPRINT_PLATFORM_ERROR( ("Alloc usb memory pool failed!\n") );
        goto exit;
    }
    usb20h_memcfg.memory_pool_size = USB_POOL_SIZE;

    if ((usb20h_memcfg.dma_memory_pool = osl_dma_alloc_consistent(USB_DMA_POOL_SIZE, USB_DMA_POOL_ALIGN, &pool_size, &pool_base)) == NULL)
    {
        WPRINT_PLATFORM_ERROR( ("Alloc usb dma memory pool failed!\n") );
        goto exit;
    }
    usb20h_memcfg.dma_memory_pool_size = pool_size;

    /* Initialize USBX Memory */
    status = ux_system_initialize(usb20h_memcfg.memory_pool, usb20h_memcfg.memory_pool_size, usb20h_memcfg.dma_memory_pool, usb20h_memcfg.dma_memory_pool_size);
    WPRINT_PLATFORM_DEBUG( ("Initialized USBX. status=%d\n", status) );

    /* The code below is required for installing the host portion of USBX.  */
    /* Initialize USBX Host Stack (Use USBX system default values if given UX_USER_CONFIG_HOST as NULL) */
    ux_user_obj.ux_user_config_host_max_class   = usb20h_user_cfg.host_max_class;
    ux_user_obj.ux_user_config_host_max_hcd     = usb20h_user_cfg.host_max_hcd;
    ux_user_obj.ux_user_config_host_max_devices = usb20h_user_cfg.host_max_devices;
    ux_user_obj.ux_user_config_host_max_ed      = usb20h_user_cfg.host_max_ed;
    ux_user_obj.ux_user_config_host_max_td      = usb20h_user_cfg.host_max_td;
    ux_user_obj.ux_user_config_host_max_iso_td  = usb20h_user_cfg.host_max_iso_td;
    ux_user_obj.ux_user_config_host_thread_stack_size  = usb20h_user_cfg.host_thread_stack_size;

    status = ux_host_stack_initialize(wiced_usb_host_usbx_host_evt_callback, &ux_user_obj);
    if(status != UX_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USBX host stack init failed. status=%d\n", status) );
        goto exit;
    }

    /* Register all the host class drivers for this USBX implementation.  */
    /* Host HUB class  */
    status = ux_host_stack_class_register(_ux_system_host_class_hub_name, ux_host_class_hub_entry);
    if(status != UX_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USBX host HUB class init failed. status=%d\n", status) );
        goto exit;
    }
    /* Host STORAGE class  */
    status = ux_host_stack_class_register(_ux_system_host_class_storage_name, ux_host_class_storage_entry);
    if (status != UX_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USBX host STORAGE class init failed. status=%d\n", status) );
        goto exit;
    }
    /* Host HID class  */
    status = ux_host_stack_class_register(_ux_system_host_class_hid_name, ux_host_class_hid_entry);
    if (status != UX_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USBX host HID class init failed. status=%d\n", status) );
        goto exit;
    }
    /* Host AUDIO class  */
    status = ux_host_stack_class_register(_ux_system_host_class_audio_name, ux_host_class_audio_entry);
    if (status != UX_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USBX host AUDIO class init failed. status=%d\n", status) );
        goto exit;
    }
    /* Host CDC-ACM class  */
    status = ux_host_stack_class_register(_ux_system_host_class_cdc_acm_name, ux_host_class_cdc_acm_entry);
    if (status != UX_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USBX host CDC-ACM class init failed. status=%d\n", status) );
        goto exit;
    }
    /* Host Prolific PL2303 (CDC-ACM Like) class  */
    status = ux_host_stack_class_register(_ux_system_host_class_prolific_name, ux_host_class_prolific_entry);
    if (status != UX_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USBX host PROLIFIC PL2303 class init failed. status=%d\n", status) );
        goto exit;
    }

    /* Register all the HID clients.  */
    /* HID Keyboard client  */
    status = ux_host_class_hid_client_register(_ux_system_host_class_hid_client_keyboard_name, ux_host_class_hid_keyboard_entry);
    if (status != UX_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USBX host HID Keyboard client init failed. status=%d\n", status) );
        goto exit;
    }
    /* HID Mouse client  */
    status = ux_host_class_hid_client_register(_ux_system_host_class_hid_client_mouse_name, ux_host_class_hid_mouse_entry);
    if (status != UX_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USBX host HID Mouse client init failed. status=%d\n", status) );
        goto exit;
    }

    /* Init USB20 Host HW.  */
    WPRINT_PLATFORM_DEBUG( ("Init USB20 Host HW\n") );
    status = platform_usb_host_init();
    if (status != PLATFORM_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USB20 Host HW init failed. status=%d\n", status) );
        goto exit;
    }
    status = platform_usb_host_init_irq(wiced_usb_host_usbx_isr);
    if (status != PLATFORM_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USB20 Host irq init failed. status=%d\n", status) );
        goto exit;
    }
    WPRINT_PLATFORM_DEBUG( ("USB20 Host HW init ok!\n") );

    /* Obtain USB20 Host HCI resources for USB stack driver  */
    status = platform_usb_host_get_hci_resource((platform_usb_host_hci_resource_t *)usb20h_hci_info, sizeof(usb20h_hci_info), &usb20h_hci_num);
    if (status != PLATFORM_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USB20 Host HCI resources get failed. status=%d\n", status) );
        goto exit;
    }
    WPRINT_PLATFORM_DEBUG( ("USB20 Host HW supports %lu HCI resources\n", usb20h_hci_num) );

    for (i = 0; i < usb20h_hci_num; i ++)
    {
#ifndef WICED_USB_OHCI_ONLY
        if (usb20h_hci_info[i].usb_host_hci_type == USB_HOST_CONTROLLER_INTERFACE_EHCI)
        {
            ehci_resource = &usb20h_hci_info[i];
        }
#endif  /* WICED_USB_OHCI_ONLY */
        if (usb20h_hci_info[i].usb_host_hci_type == USB_HOST_CONTROLLER_INTERFACE_OHCI)
        {
            ohci_resource = &usb20h_hci_info[i];
        }
    }
#ifndef WICED_USB_OHCI_ONLY
    if ( ehci_resource == NULL )
    {
        WPRINT_PLATFORM_ERROR( ("No USB20 EHCI resources for USBX\n") );
        goto exit;
    }
#endif  /* WICED_USB_OHCI_ONLY */
    if ( ohci_resource == NULL )
    {
        WPRINT_PLATFORM_ERROR( ("No USB20 OHCI resources for USBX\n") );
        goto exit;
    }

#ifndef WICED_USB_OHCI_ONLY
    /* Register all the USB20 Host controllers available in this system */
    /* EHCI Host controller  */
    status = ux_host_stack_hcd_register(_ux_system_host_hcd_ehci_name, ux_hcd_ehci_initialize, ehci_resource->usb_host_hci_ioaddress, ehci_resource->usb_host_hci_irq_number); //0x18006000
    if(status != UX_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USBX ECHI init failed. status=%d\n", status) );
        goto exit;
    }
    WPRINT_PLATFORM_DEBUG( ("USBX EHCI init ok!\n") );
#endif  /* WICED_USB_OHCI_ONLY */

    /* OHCI Host controller  */
    status = ux_host_stack_hcd_register(_ux_system_host_hcd_ohci_name, ux_hcd_ohci_initialize, ohci_resource->usb_host_hci_ioaddress, ohci_resource->usb_host_hci_irq_number); //0x1800d000
    if(status != UX_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USBX OHCI init failed. status=%d\n", status) );
        goto exit;
    }
    WPRINT_PLATFORM_DEBUG( ("USBX OHCI init ok!\n") );

    /* USB host post initialization */
    platform_usb_host_post_init();

    /* Enable USB20 EHCI/OHCI interrupt  */
    status = platform_usb_host_enable_irq();
    if (status != PLATFORM_SUCCESS)
    {
        WPRINT_PLATFORM_ERROR( ("USB20 Host irq enable failed. status=%d\n", status) );
        goto exit;
    }
    WPRINT_PLATFORM_DEBUG( ("USB20 IRQ enabled!\n") );

    WPRINT_PLATFORM_INFO( ("USB20 Host init completed!!!\n") );
    return WICED_SUCCESS;

exit:
    if(usb20h_memcfg.memory_pool)
        free(usb20h_memcfg.memory_pool);
    if(usb20h_memcfg.dma_memory_pool)
        osl_dma_free_consistent(usb20h_memcfg.dma_memory_pool);

    return WICED_ERROR;
}

wiced_result_t wiced_usb_host_deinit( void )
{
    memset((void*)&usb20h_user_cfg, 0, sizeof(usb20h_user_cfg));
    return WICED_SUCCESS;
}

/******************************************************
 *            Static Function Definitions
 ******************************************************/
static void wiced_usb_host_usbx_isr( void )
{
#ifndef WICED_USB_OHCI_ONLY
    _ux_hcd_ehci_interrupt_handler();
#endif  /* WICED_USB_OHCI_ONLY */
    _ux_hcd_ohci_interrupt_handler();
}

static UINT wiced_usb_host_usbx_host_evt_callback( ULONG evt, UX_HOST_CLASS *class, void *instance )
{
    UINT post_event = USB_HOST_EVENT_UNDEFINED;

    /* Check the class and pass to specific host class event callback handle.  */
    if (ux_utility_memory_compare(class -> ux_host_class_name, _ux_system_host_class_hid_name,
        ux_utility_string_length_get(_ux_system_host_class_hid_name)) == UX_SUCCESS)
    {
        /*
         * USB Host HID Class
         */
        switch (evt)
        {
            case UX_DEVICE_INSERTION:
                WPRINT_PLATFORM_INFO( ("#UX#HostClassHid#: event UX_DEVICE_INSERTION\n") );
                post_event = USB_HOST_EVENT_HID_DEVICE_INSERTION;

                /* Retrieve the class instance.  */
                hid = (UX_HOST_CLASS_HID *)instance;

                /* Get the HID client */
                hid_client = hid -> ux_host_class_hid_client;

                /* Ensure the client is valid.  */
                if (hid_client != UX_NULL)
                {
                    /* Ensure the client instance is live.  */
                    if (hid_client -> ux_host_class_hid_client_local_instance != UX_NULL)
                    {
                        /* Check the class if this is a Keyboard HID device.  */
                        if (ux_utility_memory_compare(hid_client -> ux_host_class_hid_client_name, _ux_system_host_class_hid_client_keyboard_name,
                            ux_utility_string_length_get(_ux_system_host_class_hid_client_keyboard_name)) == UX_SUCCESS)
                        {
                            /* We have found the keyboard.  */
                            keyboard = (UX_HOST_CLASS_HID_KEYBOARD *)hid_client -> ux_host_class_hid_client_local_instance;
                            WPRINT_PLATFORM_INFO( ("#UX#HostClassHid#: Found Keyboard HID Class device\n") );
                        }
                        /* Check the class if this is a Mouse HID device.  */
                        else if (ux_utility_memory_compare(hid_client -> ux_host_class_hid_client_name, _ux_system_host_class_hid_client_mouse_name,
                            ux_utility_string_length_get(_ux_system_host_class_hid_client_mouse_name)) == UX_SUCCESS)
                        {
                            /* We have found the mouse.  */
                            mouse = (UX_HOST_CLASS_HID_MOUSE *)hid_client -> ux_host_class_hid_client_local_instance;
                            WPRINT_PLATFORM_INFO( ("#UX#HostClassHid#: Found Mouse HID Class device\n") );
                        }
                        else
                        {
                            WPRINT_PLATFORM_INFO( ("#UX#HostClassHid#: Unknown HID Class device\n") );
                        }
                    }
                }
                break;

            case UX_DEVICE_REMOVAL:
                WPRINT_PLATFORM_INFO( ("#UX#HostClassHid#: event UX_DEVICE_REMOVAL\n") );
                post_event = USB_HOST_EVENT_HID_DEVICE_REMOVAL;

                /*  Check if the hid device is removed. Due to single USB port, now we assume all devices are cleaned at one removal! */
                if (instance == hid)
                {
                    /* Reset pointers to null.  */
                    hid = NULL;
                    hid_client = NULL;
                    keyboard = NULL;
                    mouse = NULL;
                }
                break;

            default :
                WPRINT_PLATFORM_INFO( ("#UX#HostClassHid#: Unknown event\n") );
                break;
        }
    }
    else if (ux_utility_memory_compare(class -> ux_host_class_name, _ux_system_host_class_storage_name,
        ux_utility_string_length_get(_ux_system_host_class_storage_name)) == UX_SUCCESS)
    {
        /*
         * USB Host Mass Storage Class
         */

        UX_HOST_CLASS *storage_class;
        UINT status;

        switch (evt)
        {
            case UX_DEVICE_INSERTION:
                WPRINT_PLATFORM_INFO( ("#UX#HostClassStorage#: event UX_DEVICE_INSERTION\n") );
                post_event = USB_HOST_EVENT_STORAGE_DEVICE_INSERTION;

                /* Retrieve the class instance.  */
                storage = (UX_HOST_CLASS_STORAGE *)instance;
                WPRINT_PLATFORM_INFO( ("#UX#HostClassStorage#: Found Storage Class device\n") );

                /* Check if we got a storage class device. */
                WPRINT_PLATFORM_INFO( ("Getting usb storage......\n") );
                status =  ux_host_stack_class_get(_ux_system_host_class_storage_name, &storage_class);
                if (status != UX_SUCCESS)
                {
                    WPRINT_PLATFORM_INFO( ("storage class get failed. status=%d\n", status) );
                    break;
                }

                if (storage_class == class)
                {
                    /* We only get the first media attached to the class container.  */
                    if (media == NULL)
                    {
                        storage_media = class -> ux_host_class_media;
                        media = &storage_media -> ux_host_class_storage_media;

                        if ( usbdisk_filesystem == NULL )
                        {
                            /* Lookup "USB" entry from all_filesystem_devices */
                            const filesystem_list_t* curr_item = all_filesystem_devices;

                            while ( curr_item->device != NULL )
                            {
                                if ( strcmp ( "USB", curr_item->name ) == 0 )
                                {
                                    /* Found requested device */
                                    usbdisk_filesystem = (filesystem_list_t*) curr_item;
                                    break;
                                }
                                curr_item++;
                            }

                            if ( curr_item->device == NULL )
                            {
                                /* Not found - print options */
                                WPRINT_PLATFORM_INFO( ("Filesystem 'USB' not found\n") );
                                return UX_ERROR;
                            }
                        }

                        if ( usbdisk_filesystem->device->device_specific_data == NULL )
                        {
                            usbdisk_filesystem->device->device_specific_data = (void*)media;

                            /* Mount filesystem */
                            wiced_filesystem_mount( usbdisk_filesystem->device, usbdisk_filesystem->type, &usbdisk_media_handle, USB_DISK_STORAGE_MOUNT_NAME );
                            WPRINT_PLATFORM_INFO( ("usb storage get ok!!!\n") );
                        }
                        else
                        {
                            WPRINT_PLATFORM_INFO( ("device specific data is not NULL, mount fail!\n") );
                            return UX_ERROR;
                        }
                    }
                }
                break;

            case UX_DEVICE_REMOVAL:
                WPRINT_PLATFORM_INFO( ("#UX#HostClassStorage#: event UX_DEVICE_REMOVAL\n") );
                post_event = USB_HOST_EVENT_STORAGE_DEVICE_REMOVAL;

                /*  Check if the storage device is removed. Due to single USB port, now we assume all devices are cleaned at one removal! */
                if (instance == storage)
                {
                    /* Unmount filesystem */
                    wiced_filesystem_unmount( &usbdisk_media_handle );

                    /* Reset pointers to null.  */
                    usbdisk_filesystem->device->device_specific_data = NULL;
                    memset((void*)&usbdisk_media_handle, 0, sizeof(wiced_filesystem_t));
                    media = NULL;
                    storage_media = NULL;
                    storage = NULL;
                }
                break;

            default :
                WPRINT_PLATFORM_INFO( ("#UX#HostClassStorage#: Unknown event\n") );
                break;
        }
    }
    else if (ux_utility_memory_compare(class -> ux_host_class_name, _ux_system_host_class_audio_name,
        ux_utility_string_length_get(_ux_system_host_class_audio_name)) == UX_SUCCESS)
    {
        /*
         * USB Host Audio Class
         */

        UINT i;

        switch (evt)
        {
            case UX_DEVICE_INSERTION:
                WPRINT_PLATFORM_INFO( ("#UX#HostClassAudio#: event UX_DEVICE_INSERTION\n") );
                post_event = USB_HOST_EVENT_AUDIO_DEVICE_INSERTION;

                /* Retrieve the class instance.  */
                if (audio_device_index < USB_AUDIO_DEVICE_HANDLE_MAX)
                {
                    audio[audio_device_index] = (UX_HOST_CLASS_AUDIO *)instance;
                    WPRINT_PLATFORM_INFO( ("#UX#HostClassAudio#: Found Audio Class device (with index %lu)\n", audio_device_index) );
                    audio_device_index ++;
                }
                else
                {
                    WPRINT_PLATFORM_INFO( ("#UX#HostClassAudio#: Audio Class device number exceeds max (%d)!\n", USB_AUDIO_DEVICE_HANDLE_MAX) );
                }
                break;

            case UX_DEVICE_REMOVAL:
                WPRINT_PLATFORM_INFO( ("#UX#HostClassAudio#: event UX_DEVICE_REMOVAL\n") );
                post_event = USB_HOST_EVENT_AUDIO_DEVICE_REMOVAL;

                /*  Check if the audio device is removed. Due to single USB port, now we assume all devices are cleaned at one removal! */
                for (i = 0; i < USB_AUDIO_DEVICE_HANDLE_MAX; i++)
                {
                    /* Reset pointers to null.  */
                    audio[i] = NULL;
                }
                audio_device_index = 0;
                break;

            default :
                WPRINT_PLATFORM_INFO( ("#UX#HostClassAudio#: Unknown event\n") );
                break;
        }
    }
    else if (ux_utility_memory_compare(class -> ux_host_class_name, _ux_system_host_class_cdc_acm_name,
        ux_utility_string_length_get(_ux_system_host_class_cdc_acm_name)) == UX_SUCCESS)
    {
        /*
         * USB Host CDC-ACM Class
         */

        UINT i;

        switch (evt)
        {
            case UX_DEVICE_INSERTION:
                WPRINT_PLATFORM_INFO( ("#UX#HostClassCDCACM#: event UX_DEVICE_INSERTION\n") );
                post_event = USB_HOST_EVENT_CDC_ACM_DEVICE_INSERTION;

                /* Retrieve the class instance.  */
                if (cdc_acm_device_index < USB_CDC_ACM_DEVICE_HANDLE_MAX)
                {
                    cdc_acm[cdc_acm_device_index] = (UX_HOST_CLASS_CDC_ACM *)instance;
                    WPRINT_PLATFORM_INFO( ("#UX#HostClassCDCACM#: Found CDC-ACM Class device (with index %lu)\n", cdc_acm_device_index) );
                    cdc_acm_device_index ++;
                }
                else
                {
                    WPRINT_PLATFORM_INFO( ("#UX#HostClassCDCACM#: CDC-ACM Class device number exceeds max (%d)!\n", USB_CDC_ACM_DEVICE_HANDLE_MAX) );
                }
                break;

            case UX_DEVICE_REMOVAL:
                WPRINT_PLATFORM_INFO( ("#UX#HostClassCDCACM#: event UX_DEVICE_REMOVAL\n") );
                post_event = USB_HOST_EVENT_CDC_ACM_DEVICE_REMOVAL;

                /*  Check if the cdc-acm device is removed. Due to single USB port, now we assume all devices are cleaned at one removal! */
                for (i = 0; i < USB_CDC_ACM_DEVICE_HANDLE_MAX; i++)
                {
                    /* Reset pointers to null.  */
                    cdc_acm[i] = NULL;
                }
                cdc_acm_device_index = 0;
                break;

            default :
                WPRINT_PLATFORM_INFO( ("#UX#HostClassCDCACM#: Unknown event\n") );
                break;
        }
    }
    else if (ux_utility_memory_compare(class -> ux_host_class_name, _ux_system_host_class_prolific_name,
        ux_utility_string_length_get(_ux_system_host_class_prolific_name)) == UX_SUCCESS)
    {
        /*
         * USB Host Prolific PL2303 (CDC-ACM Like) Class
         */
        switch (evt)
        {
            case UX_DEVICE_INSERTION:
                WPRINT_PLATFORM_INFO( ("#UX#HostClassProlific#: event UX_DEVICE_INSERTION\n") );
                post_event = USB_HOST_EVENT_PROLIFIC_DEVICE_INSERTION;

                /* Retrieve the class instance.  */
                prolific = (UX_HOST_CLASS_PROLIFIC *)instance;
                WPRINT_PLATFORM_INFO( ("#UX#HostClassProlific#: Found Prolific PL2303 Class device\n") );
                break;

            case UX_DEVICE_REMOVAL:
                WPRINT_PLATFORM_INFO( ("#UX#HostClassProlific#: event UX_DEVICE_REMOVAL\n") );
                post_event = USB_HOST_EVENT_PROLIFIC_DEVICE_REMOVAL;

                /*  Check if the prolific device is removed. Due to single USB port, now we assume all devices are cleaned at one removal! */
                if (instance == prolific)
                {
                    /* Reset pointers to null.  */
                    prolific = NULL;
                }
                break;

            default :
                WPRINT_PLATFORM_INFO( ("#UX#HostClassProlific#: Unknown event\n") );
                break;
        }
    }
    else
    {
        WPRINT_PLATFORM_INFO( ("UX %s event (%lu)\n", (char *)class -> ux_host_class_name, evt) );
    }

    /* Post event up to user application  */
    if (usb20h_user_cfg.host_event_callback)
    {
        usb20h_user_cfg.host_event_callback(post_event , (void *)class, (void *)instance);
    }

    return UX_SUCCESS;
}
