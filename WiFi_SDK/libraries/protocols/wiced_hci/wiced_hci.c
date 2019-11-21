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
 *
 * HOST OFFLOADING: wiced_hci implementation
 *
 */

#include <string.h>
#include "wiced_hci.h"
#include "wiced_rtos.h"
#include "wiced_uart.h"
#include "wwd_debug.h"
#include "bt_firmware.h"
#include "wiced_low_power.h"
#include "wiced_uart.h"
#include "platform_bluetooth.h"

/******************************************************
 *                    Constants
 ******************************************************/
#define WICED_HCI_QUEUE_MAX_ENTRIES               20
#define WICED_HCI_CMD_THREAD_STACK_SIZE       (4096)
#define WICED_HCI_NUM_UART_THREADS                 3
#define WICED_HCI_HEADER_LENGTH                    5

/******************************************************
 *                   Structures
 ******************************************************/

/**
 * Queue element for app framework queue.
 */
typedef struct
{
    void                (*function)(void);
    wiced_bool_t        wait_for_event_complete;
} thread_queue_element_t;

/**
 * Context data for the big HCI.
 *
 */
typedef struct
{
    wiced_thread_t              thread_handle[WICED_HCI_NUM_UART_THREADS];
    wiced_queue_t               queue;
    wiced_hci_cb                evt_cb[MAX_CONTROL_GROUP];

} wiced_hci_context_t;


/******************************************************
 *               Static Function Declarations
 ******************************************************/
static void wiced_hci_cmd_thread( uint32_t args );
static void wiced_hci_read_thread(uint32_t args);
static void wiced_hci_write_command(uint16_t command, uint8_t* payload, uint16_t length);

/* moving this to global, as the Free RTOS stack size is only 4096 bytes
 * the wiced_hci_read_thread()  thread is declaring a stack buffer of 2048
 * and in turn calls parse_wiced_pkt() which is also declaring local variable of
 * buffer size 2048 bytes on stack causing stack overflow in FreeRTOS-LwIP
 */
uint8_t data_parsepkt[2048];
uint8_t data_hciread[2048];

/******************************************************
 *               External Function Declarations
 ******************************************************/
extern const char brcm_patch_version[];
extern const uint8_t brcm_patchram_buf[];
extern const int brcm_patch_ram_length;

/******************************************************
 *               Variable Definitions
 ******************************************************/
uint8_t               wiced_hci_cmd_thread_stack[WICED_HCI_CMD_THREAD_STACK_SIZE] __attribute__((section (".ccm")));
uint8_t               wiced_hci_read_thread_stack[WICED_HCI_CMD_THREAD_STACK_SIZE] __attribute__((section (".ccm")));
wiced_hci_context_t   wiced_hci_context;
/******************************************************
 *               Function Definitions
 ******************************************************/

static void wiced_hci_cmd_thread( uint32_t args )
{
    thread_queue_element_t message;
    wiced_result_t result;


    while( WICED_TRUE )
    {
        result = wiced_rtos_pop_from_queue(&wiced_hci_context.queue, &message, WICED_NEVER_TIMEOUT );
        if( result != WICED_SUCCESS )
        {
            WPRINT_APP_ERROR(("[%s] Error receiving event from app-queue\n",__func__) );
            continue;
        }

        if( message.function )
        {
            message.function();
        }

        if( message.wait_for_event_complete == WICED_TRUE )
        {

        }
    }
}

static void parse_wiced_pkt(void)
{
    uint32_t length = 2;
    uint8_t control_gp = 0;
    uint16_t control_cmd = 0;

    memset(data_parsepkt, 0, sizeof(data_parsepkt));

    if( (wiced_hci_uart_read(data_parsepkt, &length, WICED_NEVER_TIMEOUT)) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("[%s]\n",__func__));
        return;
    }

    if (length == 0)
        return;

    control_cmd = data_parsepkt[0] + (data_parsepkt[1] << 8);
    control_gp = HCI_CONTROL_GROUP(data_parsepkt[0] + (data_parsepkt[1] << 8) );

    length= 2;
    if( (wiced_hci_uart_read(data_parsepkt, &length,WICED_NEVER_TIMEOUT)) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("[%s]\n",__func__));
        return;
    }

    if (length == 0)
        return;

    length = data_parsepkt[0] + (data_parsepkt[1] << 8);

    if ( length > 0 )
    {
        if( (wiced_hci_uart_read(data_parsepkt, &length, WICED_NEVER_TIMEOUT)) != WICED_SUCCESS)
        {
            WPRINT_APP_INFO(("[%s]\n",__func__));
            return;
        }
        if (length == 0)
            return;
    }

    switch(control_gp)
    {

        case HCI_CONTROL_GROUP_DEVICE:
        case HCI_CONTROL_GROUP_LE:
        case HCI_CONTROL_GROUP_GATT:
        case HCI_CONTROL_GROUP_HF:
        case HCI_CONTROL_GROUP_SPP:
        case HCI_CONTROL_GROUP_AUDIO:
        case HCI_CONTROL_GROUP_HIDD:
        case HCI_CONTROL_GROUP_AVRC_TARGET:
        case HCI_CONTROL_GROUP_TEST:
        case HCI_CONTROL_GROUP_AIO:
        case HCI_CONTROL_GROUP_TIME:
        case HCI_CONTROL_GROUP_ANCS:
        case HCI_CONTROL_GROUP_ALERT:
        case HCI_CONTROL_GROUP_IAP2:
        case HCI_CONTROL_GROUP_AG:
        case HCI_CONTROL_GROUP_LN:
        case HCI_CONTROL_GROUP_BSG:
        case HCI_CONTROL_GROUP_AVRC_CONTROLLER:
        case HCI_CONTROL_GROUP_SIXLO:
        case HCI_CONTROL_GROUP_AUDIO_SINK:
        case HCI_CONTROL_GROUP_MESH:
        case HCI_CONTROL_GROUP_ZB_GENERAL:
        case HCI_CONTROL_GROUP_ZB_GATEWAY:
        case HCI_CONTROL_GROUP_ZB_ZCL:
        case HCI_CONTROL_GROUP_ZB_ZCL_GENERAL:
        case HCI_CONTROL_GROUP_ZB_QT:
        case HCI_CONTROL_GROUP_ZB_VENDOR:
            if(wiced_hci_context.evt_cb[control_gp])
            {
                wiced_hci_context.evt_cb[control_gp]( control_cmd, data_parsepkt, length );
            }
            break;
        case HCI_CONTROL_GROUP_MISC:
        default:
            WPRINT_APP_INFO(("[%s %d]COMMAND NOT SUPPORTED\n",__func__,__LINE__));
            break;

    }


}

static void wiced_hci_read_thread(uint32_t args)
{
    uint32_t  length = 1;
    wiced_result_t result = WICED_SUCCESS;
    wiced_bool_t is_warmboot = WICED_DEEP_SLEEP_IS_WARMBOOT( );
    platform_uart_config_t bt_uart_config;

    memset(data_hciread, 0, sizeof(data_hciread));
#if !defined(WICED_HCI_FW_DOWNLOAD_BYPASS)
    /* By default is_warm_boot is false, indicating cold_boot scenario. */
    if (is_warmboot == WICED_TRUE)
    {
        WPRINT_APP_INFO(("Warmboot : Not downloading BT Firmware...\n"));


        {
            memset( &bt_uart_config, 0, sizeof( bt_uart_config ) );
            /* Update host uart baudrate*/
            bt_uart_config.baud_rate = BAUDRATE_3MBPS;
            result = wiced_hci_uart_reconfig( &bt_uart_config );
            if (result != WICED_BT_SUCCESS)
            {
                WPRINT_APP_INFO(( "bt_host_update_baudrate Fail!%d\n", result));
            }
        }
        if(wiced_hci_context.evt_cb[HCI_CONTROL_GROUP_DEVICE])
        {
            WPRINT_APP_INFO(( "Generating DEVICE_STARTED event in WARM_BOOT\n" ));
            wiced_hci_context.evt_cb[HCI_CONTROL_GROUP_DEVICE]( HCI_CONTROL_EVENT_DEVICE_STARTED, data_hciread, sizeof(data_hciread) );
        }
        //else do nothing for fly-wire setup since baud rate would already been configured
    }
    else
    {
        WPRINT_APP_INFO(("Downloading Firmware...\n"));
        result = bt_firmware_download( brcm_patchram_buf, brcm_patch_ram_length, brcm_patch_version );
        if ( result != WICED_BT_SUCCESS )
        {
            WPRINT_APP_INFO( ( "Error downloading HCI firmware\n" ) );
            return;
        }
        WPRINT_APP_INFO(("\n\nFirmware Downloaded\n"));
    }
#else
    UNUSED_VARIABLE( result );
    UNUSED_VARIABLE( is_warmboot );
    UNUSED_VARIABLE( bt_uart_config );
    WPRINT_APP_INFO(("Bypass Firmware Download\n"));
#endif

    while( WICED_TRUE )
    {
        if( (wiced_hci_uart_read(data_hciread, &length, WICED_NEVER_TIMEOUT)) != WICED_SUCCESS)
        {
            WPRINT_APP_INFO(("[%s]\n",__func__));
            continue;
        }

        switch(data_hciread[0])
        {
            case HCI_WICED_PKT:
                parse_wiced_pkt();
                /*
                 * call the appropriate evt_cb in the wiced_hci_context
                 */
                break;

            default:
                break;
        }

    }

}

static void wiced_hci_write_command(uint16_t command, uint8_t* payload, uint16_t length)
{
    uint8_t    data[1040]; // TODO: make this dynamic
    uint32_t   header = 0;

    data[header++] = HCI_WICED_PKT;
    data[header++] = command & 0xff;
    data[header++] = (command >> 8) & 0xff;
    data[header++] = length & 0xff;
    data[header++] = (length >> 8) & 0xff;

    if(length != 0)
        memcpy(&data[header], payload, length);
    wiced_hci_uart_write(data,length+WICED_HCI_HEADER_LENGTH);

}

void wiced_hci_send(uint32_t opcode, uint8_t* data, uint16_t length)
{
    wiced_hci_write_command(opcode, data, length);
}

wiced_result_t wiced_hci_up()
{
    wiced_result_t result = WICED_SUCCESS;

    /* initialize the uart */
    result = wiced_hci_uart_init();
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("[%s %d] UART initialization failed\n",__func__,__LINE__));
        return result;
    }

    /* create a read thread for the hci */
    result = wiced_rtos_init_queue( &wiced_hci_context.queue, "wiced_hci_queue", sizeof(thread_queue_element_t), WICED_HCI_QUEUE_MAX_ENTRIES );
    if( result != WICED_SUCCESS )
    {
        return result;
    }

    result = wiced_rtos_create_thread_with_stack( &wiced_hci_context.thread_handle[0],
                                          WICED_DEFAULT_WORKER_PRIORITY,
                                          "wiced_hci_cmd_thread",
                                          wiced_hci_cmd_thread,
                                          wiced_hci_cmd_thread_stack,
                                          WICED_HCI_CMD_THREAD_STACK_SIZE,
                                          NULL );
    if( result != WICED_SUCCESS )
    {
        return result;
    }

    result = wiced_rtos_create_thread_with_stack( &wiced_hci_context.thread_handle[1],
                                              WICED_DEFAULT_WORKER_PRIORITY,
                                              "wiced_hci_read_thread",
                                              wiced_hci_read_thread,
                                              wiced_hci_read_thread_stack,
                                              WICED_HCI_CMD_THREAD_STACK_SIZE,
                                              NULL );
    return result;
}

wiced_result_t wiced_hci_down(void)
{
    wiced_result_t result= WICED_SUCCESS;
    WPRINT_APP_INFO((" wiced_hci_down\n"));

    /* Kill the threads created */
    result = wiced_rtos_delete_thread(&wiced_hci_context.thread_handle[0]); // command thread
    result = wiced_rtos_delete_thread(&wiced_hci_context.thread_handle[1]); // read thread

    /* deinit the queue */
    result = wiced_rtos_deinit_queue(&wiced_hci_context.queue);

    /* de-initialize the UART */
    result = wiced_hci_uart_deinit();
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("[%s %d] UART de-initialization failed\n",__func__,__LINE__));
        return result;
    }

    /* set the control block to 0 */
    memset(data_parsepkt, 0, sizeof(data_parsepkt));
    memset(data_hciread, 0, sizeof(data_hciread));
    memset(&wiced_hci_context, 0 ,sizeof(wiced_hci_context));

    return result;
}

wiced_result_t wiced_hci_set_event_callback(control_group_t group, wiced_hci_cb evt_cb)
{
    if ( group >= MAX_CONTROL_GROUP)
    {
        return WICED_ERROR;
    }

    if( group < DEVICE )
    {
        return WICED_ERROR;
    }

    wiced_hci_context.evt_cb[group] = evt_cb;

    return WICED_SUCCESS;
}
