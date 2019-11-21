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

/** @bluetooth_mesh.c
 *
 * This handles the mesh api functions when USE_WICED_HCI is defined
 *
 */

#include "wiced_hci_bt_mesh.h"
#include <string.h>
#include <stdlib.h>
#include "wiced_hci.h"
#include "wwd_debug.h"
#include "bt_types.h"
#include "wiced_rtos.h"
#include "wiced_hci_bt_internal_common.h"

/******************************************************
  *                   Structures
  ******************************************************/
 typedef struct wiced_hci_bt_mesh_context {
         wiced_bt_mesh_provision_end_cb_t     prov_end_cb;
         wiced_bt_mesh_core_gatt_send_cb_t    proxy_data_cb;
         wiced_bt_mesh_write_nvram_data_cb_t  write_nvram_data_cb;
         wiced_bt_mesh_status_cb_t            mesh_status_cb;

}wiced_hci_bt_mesh_context_t;

/******************************************************
  *               Variable Definitions
  ******************************************************/
wiced_hci_bt_mesh_context_t wh_bt_mesh_context;

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static void wiced_hci_mesh_cb(uint16_t event, uint8_t* payload, uint32_t len);
/******************************************************
  *               Function Definitions
  ******************************************************/

static void wiced_hci_mesh_cb(uint16_t event, uint8_t* payload, uint32_t len)
{
    uint8_t*                     p = payload;

    WICED_HCI_DEBUG_LOG(("[%s]\n",__func__));

    switch(event)
    {
        case HCI_CONTROL_MESH_EVENT_PROVISIONING_STATUS:
        {
            uint32_t conn_id = 0x01;
            uint8_t result = *p;
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_MESH_EVENT_PROVISIONING_STATUS : %02X \n",result));
            (*wh_bt_mesh_context.prov_end_cb)(conn_id, result);
        }
        break;
        case HCI_CONTROL_MESH_EVENT_PROXY_DATA:
        {
            uint32_t packet_len;
            uint8_t *packet;
            STREAM_TO_UINT32(packet_len, p);
            packet = (uint8_t*) malloc(packet_len);
            STREAM_TO_ARRAY8(packet, p);

            (*wh_bt_mesh_context.proxy_data_cb)(packet, packet_len);
        }
        break;
        case HCI_CONTROL_EVENT_DEVICE_STARTED :
        {

            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_EVENT_DEVICE_STARTED "));
            //TO DO : should hci be initialized again ?
            wiced_rtos_delay_milliseconds(1000);
            //call wiced_bt_mesh_proxy_connect
            wiced_bt_mesh_proxy_connect(1);
        }
        break;
        case HCI_CONTROL_MESH_EVENT_NVRAM_WRITE:
        {
            WICED_HCI_DEBUG_LOG(("recvd HCI_CONTROL_MESH_EVENT_NVRAM_WRITE command , nvm_idx : %d , len : %lu \n", *p, len));
            uint8_t nvram_id;
            uint8_t *packet;
            STREAM_TO_UINT8(nvram_id, p);
            packet = (uint8_t*) malloc(len-1);
            memcpy(packet,p,len-1);
            (*wh_bt_mesh_context.write_nvram_data_cb)(nvram_id, packet, len-1);
        }
        break;

        case HCI_CONTROL_MESH_EVENT_MESH_STATUS :
        {
            uint8_t result = *p;
            WICED_HCI_DEBUG_LOG(("\n Received HCI_CONTROL_MESH_EVENT_MESH_STATUS = %02X\n ", result));
            (*wh_bt_mesh_context.mesh_status_cb)(result);
             break;
        }
        default:
            WICED_HCI_DEBUG_LOG(("%s EVENT not handled\n", __FUNCTION__));
            break;
    }
}

wiced_result_t wiced_bt_mesh_proxy_connect(uint8_t connection_state)
 {
     uint8_t*        data  = NULL;
     uint16_t        length  = 1;
     wiced_result_t   result = WICED_SUCCESS;

     WICED_HCI_DEBUG_LOG(("[%s]\n",__func__));
     /* allocate memory for the data */
     data = (uint8_t*)calloc(length, sizeof(uint8_t));
     data[0] = connection_state;
     wiced_hci_send( HCI_CONTROL_MESH_COMMAND_CONNECT_PROXY , data, length );
     free(data);
     return result;
 }


wiced_result_t wiced_bt_mesh_send_proxy_packet(uint8_t* p_data, uint8_t data_len)
 {
     uint8_t*        data  = NULL;
     uint16_t        length  = data_len;
     wiced_result_t   result = WICED_SUCCESS;

     WICED_HCI_DEBUG_LOG(("[%s]\n",__func__));
     /* allocate memory for the data */
     data = (uint8_t*)calloc(length, sizeof(uint8_t));
     memcpy(data,p_data,length);
     //ARRAY_TO_STREAM(data, p_data, data_len);
     wiced_hci_send( HCI_CONTROL_MESH_COMMAND_SEND_PROXY_DATA , data, length );
     free(data);
     return result;
 }

wiced_result_t wiced_bt_mesh_uart_init()
 {
    wiced_result_t result = WICED_SUCCESS;
     WICED_HCI_DEBUG_LOG(("[%s]\n",__func__));
      wiced_hci_up();
     wiced_rtos_delay_milliseconds(1000);
     WICED_HCI_DEBUG_LOG((" \n hoping HCI is up \n"));
     return result;
 }

wiced_result_t wiced_bt_mesh_init( wiced_bt_mesh_provision_end_cb_t prov_end_cb,
                                   wiced_bt_mesh_core_gatt_send_cb_t proxy_data_cb,
                                   wiced_bt_mesh_write_nvram_data_cb_t write_nvram_data_cb,
                                   wiced_bt_mesh_status_cb_t mesh_status_cb )
{
    wiced_result_t result = WICED_SUCCESS;
    WICED_HCI_DEBUG_LOG(("[%s]\n",__func__));
    /* Copy the callback information */
    wh_bt_mesh_context.prov_end_cb         = prov_end_cb;
    wh_bt_mesh_context.proxy_data_cb       = proxy_data_cb;
    wh_bt_mesh_context.write_nvram_data_cb = write_nvram_data_cb;
    wh_bt_mesh_context.mesh_status_cb      = mesh_status_cb;
    /* set the event callback */
    wiced_hci_set_event_callback( MESH, wiced_hci_mesh_cb );
    wiced_hci_send( HCI_CONTROL_MESH_COMMAND_START , NULL, 1 );
    return result;
}


wiced_result_t wiced_bt_mesh_reboot()
{
    wiced_result_t result = WICED_SUCCESS;
    WICED_HCI_DEBUG_LOG(("[%s]\n",__func__));
    wiced_hci_send( HCI_CONTROL_MESH_COMMAND_STACK_INIT , NULL, 1 );
    return result;
}

wiced_result_t wiced_bt_mesh_push_nvram_data(uint8_t *data_in , uint8_t data_len , uint8_t idx)
{
    uint8_t*                data  = NULL;
    uint8_t*                p     = NULL;
    uint16_t                length  = 0;
    length = data_len+1;
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    p[0] = idx;
    p++;
    memcpy(p, data_in ,data_len);
    wiced_hci_send(HCI_CONTROL_MESH_COMMAND_PUSH_NVRAM_DATA, data, length);
    free(data);
    return WICED_SUCCESS;

}
