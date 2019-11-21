/*
 * $ Copyright Broadcom Corporation $
 */

/**
 * @file wiced_hci_bt_ble.c
 *
 * This handles the BLE api functions when USE_WICED_HCI is defined.
 *
 */

#include <string.h>
#include <stdlib.h>
#include "wiced_hci.h"
#include "wwd_debug.h"
#include "wiced_bt_ble.h"
#include "wiced_hci_bt_internal_common.h"

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Structures
 ******************************************************/
typedef struct _wiced_hci_bt_ble_context
{
        wiced_hci_cb                  ble_context_cb;
} wiced_hci_bt_ble_context_t;
/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               External Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/
wiced_hci_bt_ble_context_t     wh_bt_ble_context;

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t wiced_bt_start_advertisements(wiced_bt_ble_advert_mode_t advert_mode, wiced_bt_ble_address_type_t directed_advertisement_bdaddr_type, wiced_bt_device_address_ptr_t directed_advertisement_bdaddr_ptr)
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t        data[2] ;
    uint16_t       length = 1;

    switch(advert_mode)
    {
        case BTM_BLE_ADVERT_OFF:
            data[0] = 1; /* Dont send advertisements */
            length = 1;
            break;

        /** TODO:
         * Require support for the below.
         * As of now, everything will be sent to the lower layer as an advertisement enable.
         */

        case BTM_BLE_ADVERT_DIRECTED_HIGH:
        case BTM_BLE_ADVERT_DIRECTED_LOW:
        case BTM_BLE_ADVERT_UNDIRECTED_HIGH:
        case BTM_BLE_ADVERT_UNDIRECTED_LOW:
        case BTM_BLE_ADVERT_NONCONN_HIGH:
        case BTM_BLE_ADVERT_NONCONN_LOW:
        case BTM_BLE_ADVERT_DISCOVERABLE_HIGH:
        case BTM_BLE_ADVERT_DISCOVERABLE_LOW:
            data[0] = 1;
            break;

        default:
            break;
    }

    wiced_hci_send(HCI_CONTROL_LE_COMMAND_ADVERTISE,
                   &data[0],
                   length);

    return result;
}

wiced_result_t wiced_bt_ble_set_raw_advertisement_data(UINT8 num_elem,
                                                       wiced_bt_ble_advert_elem_t *p_data)
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t data[100] ;
    uint16_t length       = 0;
    uint16_t count        = 0;
    uint16_t point        = 1;

    for ( count = 0; count  < num_elem; count++ )
    {
        length += sizeof(wiced_bt_ble_advert_type_t); /* advert_type */
        length += sizeof(uint16_t);/* length field */
        length += p_data[count].len * sizeof(uint8_t) + sizeof(uint8_t); //adding null char byte
    }

    data[0] = num_elem;

    for ( count = 0; count < num_elem; count++ )
    {
        data[point++] = p_data[count].advert_type;

        data[point++] = (uint8_t)((p_data[count].len & 0xff00)>> 8);
        data[point++] = (p_data[count].len & 0x00ff);

        memcpy(&data[point], p_data[count].p_data, (p_data[count].len)*sizeof(uint8_t) + 1 );
        point += (p_data[count].len)*sizeof(uint8_t) + 1;
    }

    wiced_hci_send(HCI_CONTROL_LE_COMMAND_SET_RAW_ADVERTISE_DATA,
                   data,
                   length);

    WICED_HCI_DEBUG_LOG(("[%s %d] done\n",__func__,__LINE__));
    return result;
}

void wiced_bt_ble_security_grant(wiced_bt_device_address_t bd_addr, uint8_t res)
{
    uint8_t data[sizeof(wiced_bt_device_address_t) + sizeof(uint8_t)];
    uint16_t length = sizeof(wiced_bt_device_address_t) + sizeof(uint8_t);

    memcpy(data, bd_addr, sizeof(wiced_bt_device_address_t));
    memcpy(&data[sizeof(wiced_bt_device_address_t)],&res,sizeof(uint8_t));
    wiced_hci_send(HCI_CONTROL_LE_COMMAND_SECURITY_GRANT, data, length);

}

/* Stub functions */
wiced_result_t wiced_bt_ble_read_adv_tx_power( wiced_bt_ble_compl_cback *p_cb )
{
    UNUSED_PARAMETER(p_cb);
    return WICED_UNSUPPORTED;
}

wiced_result_t wiced_bt_ble_set_adv_tx_power( int power )
{
    UNUSED_PARAMETER(power);
    return WICED_UNSUPPORTED;
}
