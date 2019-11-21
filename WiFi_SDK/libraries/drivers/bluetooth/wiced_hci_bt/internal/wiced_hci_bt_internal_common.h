/*
 * Copyright 2016, Cypress Semiconductor
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Cypress Semiconductor;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Cypress Semiconductor.
 */

#include "wiced_bt_gatt.h"

/******************************************************
  *                    Constants
  ******************************************************/
#ifdef ENABLE_BT_PROTOCOL_TRACES
#define WICED_HCI_DEBUG_LOG(ARGS) WPRINT_APP_INFO(ARGS)
#else
#define WICED_HCI_DEBUG_LOG(ARGS)
#endif

/******************************************************
 *                   Structures
 ******************************************************/
typedef struct _wiced_hci_bt_gatt_context {
        wiced_hci_cb                  gatt_context_cb;
        wiced_bt_gatt_cback_t*        gatt_mgmt_cb;
} wiced_hci_bt_gatt_context_t;

typedef struct _wiced_hci_bt_dm_context {
    wiced_hci_cb                               dm_context_cb;
    wiced_bt_management_cback_t*               dm_mgmt_cb;
    wiced_bt_dev_cmpl_cback_t*                 dm_cmpl_cb;
    wiced_bt_connection_status_change_cback_t* dm_callstatus_cb;
    wiced_bt_device_address_t                  bd_addr;
    uint16_t                                   nvram_id;
} wiced_hci_bt_dm_context_t;

