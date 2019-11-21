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
#include "ux_device_class_hid.h"
#include "usb_device_hid_keyboard.h"


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
#define DEVICE_FRAMEWORK_LENGTH_FULL_SPEED      (52)
#define DEVICE_FRAMEWORK_LENGTH_HIGH_SPEED      (62)
#define STRING_FRAMEWORK_LENGTH                 (40)
#define LANGUAGE_ID_FRAMEWORK_LENGTH            (2)
#define HID_KEYBOARD_REPORT_LENGTH              (63)

/* Application read/write buffer size definition */
#define MAX_KEYBOARD_QUEUE_LENGTH               (1024)

/* ASCII code definition */
#define HEX_ASCII_CHAR_CR                       (0x0D)
#define HEX_ASCII_CHAR_LF                       (0x0A)

/* Application delay time parameters */
#define DEVICE_READY_WAITING_PERIOD             (3000) //In ms
#define KEYBOARD_KEY_SEND_CYCLE_TIME            (2000) //In ms

/* USB Device console command sets */
#define USB_DEVICE_APP_COMMANDS \
    { (char*) "hid_keyboard",       usb_device_app_hid_keyboard,  0, NULL, NULL, (char*) "",  (char*) "hid keyboard test" },


/**
 * [USB HID protocol - Keyboard Reports]
 * https://docs.mbed.com/docs/ble-hid/en/latest/api/md_doc_HID.html
 *
 * Format:
 *         [modifier, reserved, Key1, Key2, Key3, Key4, Key6, Key7]
 *
 * Ex:
 * 'a' report:  [0, 0, 4, 0, 0, 0, 0, 0]
 * Null report: [0, 0, 0, 0, 0, 0, 0, 0] - Treat as "key release"!
 * 'A' report:  [2, 0, 4, 0, 0, 0, 0, 0]
 *
 */
#define HID_NUM_LOCK_MASK                   (1)
#define HID_CAPS_LOCK_MASK                  (2)
#define HID_KEYBOARD_EVENT_LENGTH           (8)

/**
 * Modifier masks - used for the first byte in the HID report.
 */
#define USB_HID_KEY_MOD_NONE    0x00 // None Modifier
#define USB_HID_KEY_MOD_LCTRL   0x01 // Left Control
#define USB_HID_KEY_MOD_LSHIFT  0x02 // Left Shift
#define USB_HID_KEY_MOD_LALT    0x04 // Left Alt
#define USB_HID_KEY_MOD_LMETA   0x08 // Left GUI (Win/Apple/Meta key)
#define USB_HID_KEY_MOD_RCTRL   0x10 // Right Control
#define USB_HID_KEY_MOD_RSHIFT  0x20 // Right Shift
#define USB_HID_KEY_MOD_RALT    0x40 // Right Alt
#define USB_HID_KEY_MOD_RMETA   0x80 // Right GUI (Win/Apple/Meta key)

/**
 * The second byte in the report is reserved, 0x00
 */
#define USB_HID_KEY_BYTE_RSVD   0x00 // Reserved 2nd byte

/**
 * Scan codes - last N slots in the HID report (usually 6).
 * 0x00 if no key pressed.
 *
 * If more than N keys are pressed, the HID reports
 * KEY_ERR_OVF in all slots to indicate this condition.
 */
#define USB_HID_KEY_NONE        0x00 // No key pressed
#define USB_HID_KEY_ERR_OVF     0x01 // Keyboard Error Roll Over - used for all slots if too many keys are pressed ("Phantom key")
#define USB_HID_KEY_POST_FAIL   0x02 // Keyboard POST Fail
#define USB_HID_KEY_ERR_UNDEF   0x03 // Keyboard Error Undefined

#define USB_HID_KEY_A           0x04 // Keyboard a and A
#define USB_HID_KEY_B           0x05 // Keyboard b and B
#define USB_HID_KEY_C           0x06 // Keyboard c and C
#define USB_HID_KEY_D           0x07 // Keyboard d and D
#define USB_HID_KEY_E           0x08 // Keyboard e and E
#define USB_HID_KEY_F           0x09 // Keyboard f and F
#define USB_HID_KEY_G           0x0a // Keyboard g and G
#define USB_HID_KEY_H           0x0b // Keyboard h and H
#define USB_HID_KEY_I           0x0c // Keyboard i and I
#define USB_HID_KEY_J           0x0d // Keyboard j and J
#define USB_HID_KEY_K           0x0e // Keyboard k and K
#define USB_HID_KEY_L           0x0f // Keyboard l and L
#define USB_HID_KEY_M           0x10 // Keyboard m and M
#define USB_HID_KEY_N           0x11 // Keyboard n and N
#define USB_HID_KEY_O           0x12 // Keyboard o and O
#define USB_HID_KEY_P           0x13 // Keyboard p and P
#define USB_HID_KEY_Q           0x14 // Keyboard q and Q
#define USB_HID_KEY_R           0x15 // Keyboard r and R
#define USB_HID_KEY_S           0x16 // Keyboard s and S
#define USB_HID_KEY_T           0x17 // Keyboard t and T
#define USB_HID_KEY_U           0x18 // Keyboard u and U
#define USB_HID_KEY_V           0x19 // Keyboard v and V
#define USB_HID_KEY_W           0x1a // Keyboard w and W
#define USB_HID_KEY_X           0x1b // Keyboard x and X
#define USB_HID_KEY_Y           0x1c // Keyboard y and Y
#define USB_HID_KEY_Z           0x1d // Keyboard z and Z

#define USB_HID_KEY_1           0x1e // Keyboard 1 and !
#define USB_HID_KEY_2           0x1f // Keyboard 2 and @
#define USB_HID_KEY_3           0x20 // Keyboard 3 and #
#define USB_HID_KEY_4           0x21 // Keyboard 4 and $
#define USB_HID_KEY_5           0x22 // Keyboard 5 and %
#define USB_HID_KEY_6           0x23 // Keyboard 6 and ^
#define USB_HID_KEY_7           0x24 // Keyboard 7 and &
#define USB_HID_KEY_8           0x25 // Keyboard 8 and *
#define USB_HID_KEY_9           0x26 // Keyboard 9 and (
#define USB_HID_KEY_0           0x27 // Keyboard 0 and )

#define USB_HID_KEY_ENTER       0x28 // Keyboard Return (ENTER)
#define USB_HID_KEY_ESC         0x29 // Keyboard ESCAPE
#define USB_HID_KEY_BACKSPACE   0x2a // Keyboard DELETE (Backspace)
#define USB_HID_KEY_TAB         0x2b // Keyboard Tab
#define USB_HID_KEY_SPACE       0x2c // Keyboard Spacebar
#define USB_HID_KEY_MINUS       0x2d // Keyboard - and _
#define USB_HID_KEY_EQUAL       0x2e // Keyboard = and +
#define USB_HID_KEY_LEFTBRACE   0x2f // Keyboard [ and {
#define USB_HID_KEY_RIGHTBRACE  0x30 // Keyboard ] and }
#define USB_HID_KEY_BACKSLASH   0x31 // Keyboard \ and |
#define USB_HID_KEY_HASHTILDE   0x32 // Keyboard Non-US # and ~
#define USB_HID_KEY_SEMICOLON   0x33 // Keyboard ; and :
#define USB_HID_KEY_APOSTROPHE  0x34 // Keyboard ' and "
#define USB_HID_KEY_GRAVE       0x35 // Keyboard ` and ~
#define USB_HID_KEY_COMMA       0x36 // Keyboard , and <
#define USB_HID_KEY_DOT         0x37 // Keyboard . and >
#define USB_HID_KEY_SLASH       0x38 // Keyboard / and ?
#define USB_HID_KEY_CAPSLOCK    0x39 // Keyboard Caps Lock

#define USB_HID_KEY_F1          0x3a // Keyboard F1
#define USB_HID_KEY_F2          0x3b // Keyboard F2
#define USB_HID_KEY_F3          0x3c // Keyboard F3
#define USB_HID_KEY_F4          0x3d // Keyboard F4
#define USB_HID_KEY_F5          0x3e // Keyboard F5
#define USB_HID_KEY_F6          0x3f // Keyboard F6
#define USB_HID_KEY_F7          0x40 // Keyboard F7
#define USB_HID_KEY_F8          0x41 // Keyboard F8
#define USB_HID_KEY_F9          0x42 // Keyboard F9
#define USB_HID_KEY_F10         0x43 // Keyboard F10
#define USB_HID_KEY_F11         0x44 // Keyboard F11
#define USB_HID_KEY_F12         0x45 // Keyboard F12

#define USB_HID_KEY_SYSRQ       0x46 // Keyboard Print Screen
#define USB_HID_KEY_SCROLLLOCK  0x47 // Keyboard Scroll Lock
#define USB_HID_KEY_PAUSE       0x48 // Keyboard Pause
#define USB_HID_KEY_INSERT      0x49 // Keyboard Insert
#define USB_HID_KEY_HOME        0x4a // Keyboard Home
#define USB_HID_KEY_PAGEUP      0x4b // Keyboard Page Up
#define USB_HID_KEY_DELETE      0x4c // Keyboard Delete Forward
#define USB_HID_KEY_END         0x4d // Keyboard End
#define USB_HID_KEY_PAGEDOWN    0x4e // Keyboard Page Down
#define USB_HID_KEY_RIGHT       0x4f // Keyboard Right Arrow
#define USB_HID_KEY_LEFT        0x50 // Keyboard Left Arrow
#define USB_HID_KEY_DOWN        0x51 // Keyboard Down Arrow
#define USB_HID_KEY_UP          0x52 // Keyboard Up Arrow

#define USB_HID_KEY_NUMLOCK     0x53 // Keyboard Num Lock and Clear
#define USB_HID_KEY_KPSLASH     0x54 // Keypad /
#define USB_HID_KEY_KPASTERISK  0x55 // Keypad *
#define USB_HID_KEY_KPMINUS     0x56 // Keypad -
#define USB_HID_KEY_KPPLUS      0x57 // Keypad +
#define USB_HID_KEY_KPENTER     0x58 // Keypad ENTER
#define USB_HID_KEY_KP1         0x59 // Keypad 1 and End
#define USB_HID_KEY_KP2         0x5a // Keypad 2 and Down Arrow
#define USB_HID_KEY_KP3         0x5b // Keypad 3 and PageDn
#define USB_HID_KEY_KP4         0x5c // Keypad 4 and Left Arrow
#define USB_HID_KEY_KP5         0x5d // Keypad 5
#define USB_HID_KEY_KP6         0x5e // Keypad 6 and Right Arrow
#define USB_HID_KEY_KP7         0x5f // Keypad 7 and Home
#define USB_HID_KEY_KP8         0x60 // Keypad 8 and Up Arrow
#define USB_HID_KEY_KP9         0x61 // Keypad 9 and Page Up
#define USB_HID_KEY_KP0         0x62 // Keypad 0 and Insert
#define USB_HID_KEY_KPDOT       0x63 // Keypad . and Delete

#define USB_HID_KEY_102ND       0x64 // Keyboard Non-US \ and |
#define USB_HID_KEY_COMPOSE     0x65 // Keyboard Application
#define USB_HID_KEY_POWER       0x66 // Keyboard Power
#define USB_HID_KEY_KPEQUAL     0x67 // Keypad =

#define USB_HID_KEY_F13         0x68 // Keyboard F13
#define USB_HID_KEY_F14         0x69 // Keyboard F14
#define USB_HID_KEY_F15         0x6a // Keyboard F15
#define USB_HID_KEY_F16         0x6b // Keyboard F16
#define USB_HID_KEY_F17         0x6c // Keyboard F17
#define USB_HID_KEY_F18         0x6d // Keyboard F18
#define USB_HID_KEY_F19         0x6e // Keyboard F19
#define USB_HID_KEY_F20         0x6f // Keyboard F20
#define USB_HID_KEY_F21         0x70 // Keyboard F21
#define USB_HID_KEY_F22         0x71 // Keyboard F22
#define USB_HID_KEY_F23         0x72 // Keyboard F23
#define USB_HID_KEY_F24         0x73 // Keyboard F24


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

/* USB HID Keyboard Device class  */
static UX_SLAVE_CLASS_HID_PARAMETER     hid_parameter = {0};
static ULONG                            num_lock_flag  = UX_FALSE;
static ULONG                            caps_lock_flag = UX_FALSE;


/*
 *
 * [USB Descriptors]
 * http://www.beyondlogic.org/usbnutshell/usb5.shtml#DeviceDescriptors
 *
 *
 */
unsigned char device_framework_full_speed[] = {

    /* Device descriptor */
    0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x08,
    0xb4, 0x04, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01,

    /* Configuration descriptor */
    0x09, 0x02, 0x22, 0x00, 0x01, 0x01, 0x00, 0xc0,
    0x32,

    /* Interface descriptor */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00,
    0x00,

    /* HID descriptor */
    0x09, 0x21, 0x10, 0x01, 0x21, 0x01, 0x22, 0x3f,
    0x00,

    /* Endpoint descriptor (Interrupt) */
    0x07, 0x05, 0x81, 0x03, 0x10, 0x00, 0x08

};

unsigned char device_framework_high_speed[] = {

    /* Device descriptor */
    0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
    0xb4, 0x04, 0x02, 0x01, 0x01, 0x00, 0x01, 0x02,
    0x03, 0x01,

    /* Device qualifier descriptor */
    0x0a, 0x06, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40,
    0x01, 0x00,

    /* Configuration descriptor */
    0x09, 0x02, 0x22, 0x00, 0x01, 0x01, 0x00, 0xc0,
    0x32,

    /* Interface descriptor */
    0x09, 0x04, 0x00, 0x00, 0x01, 0x03, 0x00, 0x00,
    0x00,

    /* HID descriptor */
    0x09, 0x21, 0x10, 0x01, 0x21, 0x01, 0x22, 0x3f,
    0x00,

    /* Endpoint descriptor (Interrupt) */
    0x07, 0x05, 0x81, 0x03, 0x10, 0x00, 0x08

};

unsigned char string_framework[] = {

    /* Manufacturer string descriptor : Index 1 */
    0x09, 0x04, 0x01, 0x0c,
    0x45, 0x78, 0x70, 0x72,0x65, 0x73, 0x20, 0x4c,
    0x6f, 0x67, 0x69, 0x63,

    /* Product string descriptor : Index 2 */
    0x09, 0x04, 0x02, 0x0c,
    0x55, 0x53, 0x42, 0x20, 0x4b, 0x65, 0x79, 0x62,
    0x6f, 0x61, 0x72, 0x64,

    /* Serial Number string descriptor : Index 3 */
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


unsigned char hid_keyboard_report[HID_KEYBOARD_REPORT_LENGTH] = {

    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};

/******************************************************
 *               Function Declarations
 ******************************************************/
static wiced_result_t wiced_usb_device_hid_keyboard_init( void );
static void wiced_usb_device_usbx_isr( void );
static UINT wiced_usb_device_usbx_evt_callback( ULONG event );
static UINT hid_keyboard_callback( UX_SLAVE_CLASS_HID *hid, UX_SLAVE_CLASS_HID_EVENT *hid_event );

/******************************************************
 *               Function Definitions
 ******************************************************/
void application_start( void )
{
    /* Initialize the device */
    wiced_init();

    WPRINT_APP_INFO( ("\n") );
    WPRINT_APP_INFO( ("+-------------------------------------------------+\n") );
    WPRINT_APP_INFO( ("+ USB20 Device HID Keyboard Class Application     +\n") );
    WPRINT_APP_INFO( ("+-------------------------------------------------+\n") );

    /* Run console application function  */
    command_console_init( STDIO_UART, MAX_LINE_LENGTH, line_buffer, MAX_HISTORY_LENGTH, history_buffer_storage, " " );
    console_add_cmd_table( commands );
}

static wiced_result_t wiced_usb_device_hid_keyboard_init( void )
{
    uint pool_size;
    dmaaddr_t pool_base;
    platform_usb_device_dci_resource_t *brcm_resource = NULL;
    UX_USER_CONFIG_DEVICE ux_user_obj = {0};
    UINT status;
    UINT i;

    WPRINT_APP_INFO( ("USB HID Keyboard Device init starting...\n") );

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

    /* Initialize the hid class parameters for a keyboard.  */
    hid_parameter.ux_device_class_hid_parameter_report_address = hid_keyboard_report;
    hid_parameter.ux_device_class_hid_parameter_report_length  = HID_KEYBOARD_REPORT_LENGTH;
    hid_parameter.ux_device_class_hid_parameter_callback       = hid_keyboard_callback;

    /* Initilize the device hid class. The class is connected with interface 0 */
    status = ux_device_stack_class_register(_ux_system_slave_class_hid_name, ux_device_class_hid_entry, 1, 0, (VOID *)&hid_parameter);
    if( status != UX_SUCCESS )
    {
        WPRINT_APP_ERROR( ("USBX device hid keyboard class register failed. status=%d\n", status) );
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

    WPRINT_APP_INFO( ("USB20 Device init completed!!!\n") );
    return WICED_SUCCESS;

exit:
    if(usb20d_memcfg.memory_pool)
        free(usb20d_memcfg.memory_pool);
    if(usb20d_memcfg.dma_memory_pool)
        osl_dma_free_consistent(usb20d_memcfg.dma_memory_pool);

    return WICED_ERROR;
}

static void wiced_usb_device_usbx_isr( void )
{
    _ux_dcd_bcm4390x_interrupt_handler();
}

static UINT hid_keyboard_callback( UX_SLAVE_CLASS_HID *hid, UX_SLAVE_CLASS_HID_EVENT *hid_event )
{
    /* There was an event.  Analyze it.  Is it NUM LOCK ? */
    if (hid_event -> ux_device_class_hid_event_buffer[0] & HID_NUM_LOCK_MASK)
        /* Set the Num lock flag.  */
        num_lock_flag = UX_TRUE;
    else
        /* Reset the Num lock flag.  */
        num_lock_flag = UX_FALSE;

    /* There was an event.  Analyze it.  Is it CAPS LOCK ? */
    if (hid_event -> ux_device_class_hid_event_buffer[0] & HID_CAPS_LOCK_MASK)
        /* Set the Caps lock flag.  */
        caps_lock_flag = UX_TRUE;
    else
        /* Reset the Caps lock flag.  */
        caps_lock_flag = UX_FALSE;

    return (UX_SUCCESS);
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

/**
 * USB Device app: HID Keyboard test
 *
 * Prerequisite:
 *     USB Device Controller: BCM4390X
 *     USB Device Class: HID
 *     USB Host: Win7/Linux PC
 *
 * Notes:
 *     This test will adopt usb device class driver to make the usb device appear as a HID Keyboard device on a usb host.
 *     Plug bcm4390x usb device into Host PC, test will start immediately after correct bus enumeraton.
 *
 *     During test, bcm4390x keyboard will send "press key" and "release key" information to the Host PC, which will interpret it differently depending on the Host PC's application.
 *     When Host PC receives a "press key" report, it writes the letter or uses it as a command as explained above.
 *     It will keep performing the same action, until it receives a "release key" report.
 *
 *     In this test, bcm4390x keyboard will send 'a' to 'z' characters repeatly every 2 seconds to Host PC.
 *     You may open one text editor program on Host PC, and the editor screen will dump 'a' to 'z' alphabet periodically. (One char for every 2 seconds)
 *     User can check the Host PC editor window to verify the result.
 *
 * @@ USB Interrupt transfer!!
 */
int usb_device_app_hid_keyboard( int argc, char* argv[] )
{
    UX_SLAVE_DEVICE                 *device;
    UX_SLAVE_INTERFACE              *interface;
    UX_SLAVE_CLASS_HID              *hid;
    UX_SLAVE_CLASS_HID_EVENT        hid_event;
    UCHAR                           key;
    UINT                            status;

    WPRINT_APP_INFO( ("+++START: hid keyboard test\n") );

    status = wiced_usb_device_hid_keyboard_init();
    if( status != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("USB HID Keyboard Device init failed. status=%d\n", status) );
        return ERR_UNKNOWN;
    }
    WPRINT_APP_INFO( ("USB HID Keyboard Device init ok!\n") );


    /* Get the pointer to the device.  */
    device = &_ux_system_slave -> ux_system_slave_device;

    /* Set the first key to 'a' which is 04.  */
    key = USB_HID_KEY_A;

    /* Reset the HID event structure.  */
    memset( (void*)&hid_event, 0, sizeof(UX_SLAVE_CLASS_HID_EVENT) );

    while( 1 )
    {
        /* Is the device configured ? */
        while( device -> ux_slave_device_state != UX_DEVICE_CONFIGURED )
        {
            WPRINT_APP_INFO( ("Waiting for HID Keyboard Device connecting...\n") );
            wiced_rtos_delay_milliseconds( DEVICE_READY_WAITING_PERIOD );
        }

        WPRINT_APP_INFO( ("\n*** USB HID Keyboard Device connected OK!!! ***\n\n") );

        WPRINT_APP_INFO( ("*** Test program started sending keyboard key to HOST PC every 2 seconds. ***\n") );
        WPRINT_APP_INFO( ("*** Please check it on your HOST PC foreground terminal!!! ***\n") );

        /* Until the device stays configured.  */
        while( device -> ux_slave_device_state == UX_DEVICE_CONFIGURED )
        {
            /* Get the interface.  We use the first interface, this is a simple device.  */
            interface = device -> ux_slave_device_first_interface;

            /* Form that interface, derive the HID owner.  */
            hid = interface -> ux_slave_interface_class_instance;

            /* Wait for 2 seconds. */
            wiced_rtos_delay_milliseconds( KEYBOARD_KEY_SEND_CYCLE_TIME );

            /* Then insert a key into the keyboard event.  Length is fixed to 8.  */
            hid_event.ux_device_class_hid_event_length = HID_KEYBOARD_EVENT_LENGTH;

            /* First byte is a modifier byte.  */
            hid_event.ux_device_class_hid_event_buffer[0] = USB_HID_KEY_MOD_NONE;

            /* Second byte is reserved. */
            hid_event.ux_device_class_hid_event_buffer[1] = USB_HID_KEY_BYTE_RSVD;

            /* The 6 next bytes are keys. We only have one key here.  */
            hid_event.ux_device_class_hid_event_buffer[2] = key;

            /* Set the keyboard event.  */
            ux_device_class_hid_event_set( hid, &hid_event );

            /* Next event has the key depressed.  */
            hid_event.ux_device_class_hid_event_buffer[2] = USB_HID_KEY_NONE;

            /* Length is fixed to 8.  */
            hid_event.ux_device_class_hid_event_length = HID_KEYBOARD_EVENT_LENGTH;

            /* Set the keyboard event.  */
            ux_device_class_hid_event_set( hid, &hid_event );

            /* Are we at the end of alphabet ?  */
            if (key < USB_HID_KEY_Z)
            {
                /* Next key.  */
                key++;
            }
            else
            {
                /* Start over again.  */
                key = USB_HID_KEY_A;
            }
        }
    }

    WPRINT_APP_INFO( ("---END: hid keyboard test\n") );
    return ERR_CMD_OK;
}



