/**
 * $ Copyright Broadcom Corporation $
 */

/** @file
 *
 */

#include "wiced.h"
#include "bt_bus.h"
#include "bt_hci.h"
#include "bt_hci_interface.h"
#include "bt_packet_internal.h"

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
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

wiced_result_t bt_mfgtest_transport_driver_bus_read_handler( bt_packet_t** packet )
{
    wiced_hci_command_header_t wiced_header;
    hci_event_header_t header;
    wiced_result_t     result;
    uint32_t content_length = 0;

    /* Get the packet type (assuming that we will not receive packets < 5 bytes in length )*/
    result = bt_bus_receive( (uint8_t*)&wiced_header, sizeof( wiced_header ), WICED_NEVER_TIMEOUT );
    if ( result != WICED_BT_SUCCESS )
    {
        return result;
    }

    if(HCI_EVENT_PACKET == wiced_header.packet_type)
    {
        memcpy(&header, &wiced_header, sizeof( header ));

        /* Allocate buffer for the incoming packet. Always use dynamic packet pool for the  */
        result = bt_packet_pool_dynamic_allocate_packet( packet, sizeof( header ), header.content_length );
        if ( result != WICED_BT_SUCCESS )
        {
            return result;
        }

        ( *packet )->data_end  = ( *packet )->data_start + header.content_length;

        /* decrement by 2 since we already have 2 bytes extra when reading header */
        header.content_length -= 2;

        content_length = header.content_length;

        /* increment by 2 since we already have 2 bytes extra when reading header */
        ( *packet )->data_start += 2;

    }
    else if ((HCI_WICED_PACKET == wiced_header.packet_type) || (HCI_ACL_DATA_PACKET == wiced_header.packet_type))
    {
        /* Allocate buffer for the incoming packet. Always use dynamic packet pool for the  */
        result = bt_packet_pool_dynamic_allocate_packet( packet, sizeof( wiced_header ), wiced_header.content_length );
        if ( result != WICED_BT_SUCCESS )
        {
            return result;
        }

        ( *packet )->data_end  = ( *packet )->data_start + wiced_header.content_length;

        content_length = wiced_header.content_length;
    }
    else
    {
        return WICED_BT_UNKNOWN_PACKET;
    }

    /* Copy header to the packet */
    memcpy( ( *packet )->packet_start, &wiced_header, sizeof( wiced_header ) );

    if ( content_length > 0 )
    {
        /* Receive the remainder of the packet */
        result = bt_bus_receive( (uint8_t*)( ( *packet )->data_start ), content_length, WICED_NEVER_TIMEOUT );
        if ( result != WICED_BT_SUCCESS )
        {
            /* Failed to receive the remainder of the data. Release packet and return error */
            bt_packet_pool_free_packet( *packet );
            return result;
        }
    }

    if ( wiced_header.packet_type == 0xff )
    {
        /* Unknown packet type. Release packet and return error */
        bt_packet_pool_free_packet( *packet );
        return WICED_BT_UNKNOWN_PACKET;
    }

    /* Packet successfully received. Pass up to the transport thread and return success */
    return WICED_BT_SUCCESS;
}
