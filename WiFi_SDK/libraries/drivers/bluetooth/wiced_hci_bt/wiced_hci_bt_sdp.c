/*
 * $ Copyright Broadcom Corporation $
 */


/**
 * @file wiced_hci_bt_sdp.c
 *
 *  This Initializes the SDP db when USE_WICED_HCI is defined.
 *
 */

#include <string.h>
#include <stdlib.h>
#include "wiced_hci.h"
#include "wiced_bt_sdp.h"

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               External Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_bool_t wiced_bt_sdp_db_init (UINT8 *p_gatt_db, UINT16 size)
{
    return (TRUE);
}

