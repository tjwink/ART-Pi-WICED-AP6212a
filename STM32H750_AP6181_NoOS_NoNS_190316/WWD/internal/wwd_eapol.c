/*
 * Broadcom Proprietary and Confidential. Copyright 2016 Broadcom
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */

/** @file
 *
 */

#include "network/wwd_buffer_interface.h"
#include "wwd_network_constants.h"
#include "wwd_eapol.h"
#include <stdio.h>

/******************************************************
 *                      Macros
 ******************************************************/

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

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/

eapol_packet_handler_t wwd_eapol_packet_handler = NULL;

/******************************************************
 *               Function Definitions
 ******************************************************/

void wwd_eapol_receive_eapol_packet( /*@only@*/ wiced_buffer_t buffer, wwd_interface_t interface )
{
    if ( buffer != NULL )
    {
        if ( wwd_eapol_packet_handler != NULL )
        {
            wwd_eapol_packet_handler( buffer, interface );
        }
        else
        {
            host_buffer_release( buffer, WWD_NETWORK_RX );
        }
    }
}

uint8_t* wwd_eapol_get_eapol_data( wwd_eapol_packet_t packet )
{
    return host_buffer_get_current_piece_data_pointer( packet );
}

uint16_t wwd_get_eapol_packet_size( wwd_eapol_packet_t packet )
{
    return host_buffer_get_current_piece_size( packet );
}

wwd_result_t wwd_eapol_register_receive_handler( eapol_packet_handler_t eapol_packet_handler )
{
    if ( wwd_eapol_packet_handler == NULL )
    {
        wwd_eapol_packet_handler = eapol_packet_handler;
        return WWD_SUCCESS;
    }
    return WWD_HANDLER_ALREADY_REGISTERED;
}

void wwd_eapol_unregister_receive_handler( void )
{
    wwd_eapol_packet_handler = NULL;
}
