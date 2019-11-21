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

/**
 * @file
 *
 * This is an example of a simple 'ping' (ICMP request-reply) command.
 *
 */

#include "tx_api.h"
#include "tx_thread.h"
#include "nx_api.h"
#include "nx_icmp.h"
#include "wiced_management.h"
#include "netx_applications/dhcp/nxd_dhcp_client.h"
#include "wwd_wifi.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "network/wwd_buffer_interface.h"
#include "RTOS/wwd_rtos_interface.h"
#include "wiced_network.h"
#include "command_console.h"
#include "wiced.h"
#include <math.h>

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define PING_RCV_TIMEO        ( 1000 )  /* ping receive timeout - in milliseconds */
#ifdef WICED_USE_128_POOL_FOR_IP_STACK
#define PING_MAX_PAYLOAD_SIZE ( 32 ) /* ping max size is 32 bytes only */
#else
#define PING_MAX_PAYLOAD_SIZE ( 10000 ) /* ping max size */
#endif
#define ENABLE_LONG_PING /* Long ping enabled by default */

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

static UINT  my_nx_icmp_ping(NX_IP *ip_ptr, ULONG ip_address, CHAR *data_ptr, ULONG data_size, NX_PACKET **response_ptr, ULONG wait_option);

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const uint8_t long_ping_payload[PING_MAX_PAYLOAD_SIZE] = { 0 };

/******************************************************
 *               Function Definitions
 ******************************************************/

int ping( int argc, char *argv[] )
{
    wiced_ip_address_t ping_target;
    NX_PACKET*         response_ptr;
    char*              data_ptr    = NULL;
    uint32_t           data_length = 0;
    wwd_interface_t    interface   = WWD_STA_INTERFACE; /* Default interface for this ping function */
    NX_IP*             ip_handle;
    NX_INTERFACE*      nx_interface;
    int                tx_pkt = 0, pkt_loss = 0;

    if ( argc == 1 )
    {
        return ERR_INSUFFICENT_ARGS;
    }

    int i        = 0;
    int len      = 32;
    int num      = 1;
    int interval = 1000;
    wiced_bool_t continuous = WICED_FALSE;
    int temp[4];

    sscanf( argv[1], "%d.%d.%d.%d", &temp[0], &temp[1], &temp[2], &temp[3] );
    ping_target.ip.v4 = temp[0] << 24 | temp[1] << 16 | temp[2] << 8 | temp[3];
    WPRINT_APP_INFO ( ("PING %u.%u.%u.%u\r\n", (unsigned char) ( ( ping_target.ip.v4 >> 24 ) & 0xff ),
                                             (unsigned char) ( ( ping_target.ip.v4 >> 16 ) & 0xff ),
                                             (unsigned char) ( ( ping_target.ip.v4 >>  8 ) & 0xff ),
                                             (unsigned char) ( ( ping_target.ip.v4 >>  0 ) & 0xff ) ));

    /* Try to match the target IP address to an interface by comparing it with the network that is assigned to each interface.
     * Note that if more than one interface is mapped to the same IP subnet then this will match the first one found.
     */
    for ( i = 0; i < WICED_INTERFACE_MAX; i++ )
    {
        ip_handle = &IP_HANDLE( i );
        if ( ip_handle != NULL )
        {
            nx_interface = &ip_handle->nx_ip_interface[0];
            /* Check that the interface has an assigned subnet */
            if ( ( nx_interface->nx_interface_ip_network_mask != 0 ) && ( nx_interface->nx_interface_ip_network != 0 ) )
            {
                /* Match the network portion of the target address */
                if ( ( ping_target.ip.v4 & (uint32_t)nx_interface->nx_interface_ip_network_mask ) == (uint32_t)nx_interface->nx_interface_ip_network )
                {
                    interface = (wwd_interface_t)i;
                    break;
                }
            }
        }
    }

    for (i = 2; i < argc; i++ )
    {
        switch (argv[i][1])
        {
            case 'i':
                interval = atoi(argv[i+1]);
                if ( interval < 0 )
                {
                    WPRINT_APP_INFO(("min interval 0\n\r"));
                    return ERR_CMD_OK;
                }
                WPRINT_APP_INFO(("interval: %d milliseconds\n\r", interval));
                i++;
                break;

            case 'l':
                len = atoi(argv[i+1]);
                if ( ( len > PING_MAX_PAYLOAD_SIZE ) || ( len < 0 ) )
                {
                    WPRINT_APP_INFO(("max ping length: %d, min: 0\n\r", PING_MAX_PAYLOAD_SIZE));
                    return ERR_CMD_OK;
                }
                WPRINT_APP_INFO(("length: %d\n\r", len));
                i++;
                break;

            case 'n':
                num = atoi(argv[i+1]);
                if ( num < 1 )
                {
                    WPRINT_APP_INFO(("min number of packets 1\n\r"));
                    return ERR_CMD_OK;
                }
                WPRINT_APP_INFO(("number : %d\n\r", num));
                i++;
                break;

            case 't':
                continuous = WICED_TRUE;
                WPRINT_APP_INFO(("continuous...\n\r"));
                break;

            default:
                WPRINT_APP_INFO(("Not supported, ignoring: %s\n\r", argv[i]));
            break;
        }
    }

    data_ptr    = (char*)long_ping_payload;
    data_length = len;

    wiced_time_t send_time;
    wiced_time_t reply_time;
    wiced_time_t overall_time = 0;
    wiced_time_t overall_time2 = 0;
    wiced_time_t rtt_min = ~0, rtt_max = 0;
    UINT status;

    /* Intentionally loops forever when continuous == WICED_TRUE */
    /* coverity[loop_top] */
    while (( num > 0 ) || ( continuous == WICED_TRUE ) )
    {
        status = WICED_ERROR;

        /* Send ping and wait for reply */
        send_time = host_rtos_get_time( );

        if ( wwd_wifi_is_ready_to_transceive( interface ) == WWD_SUCCESS )
        {
            status = my_nx_icmp_ping( &IP_HANDLE( interface ), ping_target.ip.v4, data_ptr, data_length, &response_ptr, PING_RCV_TIMEO * SYSTICK_FREQUENCY / 1000 );
        }

        reply_time = host_rtos_get_time( );

        /* Print result */
        if ( status == NX_SUCCESS )
        {
            wiced_time_t ping_time = reply_time - send_time;
            WPRINT_APP_INFO(("Ping Reply %dms\r\n", (int) ping_time ));
            nx_packet_release( response_ptr );
            overall_time += ping_time;
            overall_time2 += ping_time * ping_time;
            if ( rtt_min > ping_time )
            {
                rtt_min = ping_time;
            }
            if ( rtt_max < ping_time )
            {
                rtt_max = ping_time;
            }
        }
        else if ( status == NX_NO_RESPONSE )
        {
            WPRINT_APP_INFO(("Ping timeout\r\n"));
            pkt_loss++;
        }
        else
        {
            WPRINT_APP_INFO(("Ping error\r\n"));
            pkt_loss++;
        }
        num--;
        tx_pkt++;
        if ( ( ( num > 0 ) || ( continuous == WICED_TRUE ) ) && ( ( send_time + interval ) > reply_time ) )
        {
            wiced_rtos_delay_milliseconds( ( send_time + interval ) - reply_time );
        }
    }

    wiced_time_t rtt_mdev = 0;
    float rtt_avg = 0.0;
    int received = tx_pkt - pkt_loss;
    if( received )
    {
        rtt_mdev = sqrt(overall_time2/(received) - overall_time * overall_time/(received * received));
        rtt_avg = overall_time/received;
    }
    WPRINT_APP_INFO(("--- %u.%u.%u.%u ping statistics ---\r\n",
                     (unsigned char) ( ( ping_target.ip.v4 >> 24 ) & 0xff ),
                     (unsigned char) ( ( ping_target.ip.v4 >> 16 ) & 0xff ),
                     (unsigned char) ( ( ping_target.ip.v4 >>  8 ) & 0xff ),
                     (unsigned char) ( ( ping_target.ip.v4 >>  0 ) & 0xff ) ));
    WPRINT_APP_INFO(("%u packets transmitted, %u received, %d%% packet loss, time %dms\r\n",
                     tx_pkt, received, (pkt_loss*100)/tx_pkt, (int)overall_time));
    WPRINT_APP_INFO(("rtt min/avg/max/mdev = %d/%.1f/%d/%d ms\r\n",
                     received?(int)rtt_min:0, rtt_avg,
                     (int)rtt_max, (int)rtt_mdev));

    return ERR_CMD_OK;
}


/**************************************************************************/
/*                                                                        */
/*  FUNCTION                                               RELEASE        */
/*                                                                        */
/*    _nx_icmp_ping                                       PORTABLE C      */
/*                                                           5.4          */
/*  AUTHOR                                                                */
/*                                                                        */
/*    William E. Lamie, Express Logic, Inc.                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This function builds an ICMP ping request packet and calls the      */
/*    associated driver to send it out on the network.  The function will */
/*    then suspend for the specified time waiting for the ICMP ping       */
/*    response.                                                           */
/*                                                                        */
/*  INPUT                                                                 */
/*                                                                        */
/*    ip_ptr                                Pointer to IP instance        */
/*    ip_address                            IP address to ping            */
/*    data_ptr                              User Data pointer             */
/*    data_size                             Size of User Data             */
/*    response_ptr                          Pointer to Response Packet    */
/*    wait_option                           Suspension option             */
/*                                                                        */
/*  OUTPUT                                                                */
/*                                                                        */
/*    None                                                                */
/*                                                                        */
/*  CALLS                                                                 */
/*                                                                        */
/*    _nx_icmp_checksum_compute             Computer ICMP checksum        */
/*    _nx_ip_packet_send                    IP packet send function       */
/*    _nx_ip_route_find                     Find a suitable outgoing      */
/*                                            interface.                  */
/*    _nx_packet_allocate                   Allocate a packet for the     */
/*                                            ICMP ping request           */
/*    _nx_packet_release                    Release packet on error       */
/*    tx_mutex_get                          Obtain protection mutex       */
/*    tx_mutex_put                          Release protection mutex      */
/*    _tx_thread_system_suspend             Suspend thread                */
/*    _tx_thread_system_preempt_check       Check for preemption          */
/*                                                                        */
/*  CALLED BY                                                             */
/*                                                                        */
/*    Application Code                                                    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  12-12-2005     William E. Lamie         Initial Version 5.0           */
/*  08-09-2007     William E. Lamie         Modified comment(s),          */
/*                                            resulting in version 5.1    */
/*  07-04-2009     William E. Lamie         Modified comment(s), and added*/
/*                                            logic for trace support,    */
/*                                            resulting in version 5.2    */
/*  12-31-2009     Yuxin Zhou               Added multihome support,      */
/*                                            removed internal debug logic*/
/*                                            resulting in version 5.3    */
/*  06-30-2011     Yuxin Zhou               Modified comment(s),          */
/*                                            resulting in version 5.4    */
/*                                                                        */
/**************************************************************************/
static UINT  my_nx_icmp_ping(NX_IP *ip_ptr, ULONG ip_address,
                    CHAR *data_ptr, ULONG data_size,
                    NX_PACKET **response_ptr, ULONG wait_option)
{

TX_INTERRUPT_SAVE_AREA

UINT            status;
NX_PACKET       *request_ptr;
NX_ICMP_HEADER  *header_ptr;
#ifndef NX_DISABLE_ICMP_TX_CHECKSUM
ULONG           checksum;
#endif /* NX_DISABLE_ICMP_TX_CHECKSUM */
ULONG           sequence;
TX_THREAD      *thread_ptr;
#ifndef WICED_NETTYPE_ROM
ULONG           next_hop_address;
#endif

    /* Clear the destination pointer.  */
    *response_ptr =  NX_NULL;

    /* Allocate a packet to place the ICMP echo request message in.  */
    status =  nx_packet_allocate(ip_ptr -> nx_ip_default_packet_pool, &request_ptr, NX_ICMP_PACKET, wait_option);
    if (status)
    {

        /* Error getting packet, so just get out!  */
        return(status);
    }

#ifndef ENABLE_LONG_PING
    /* Determine if the size of the data and the ICMP header is larger than
       the packet payload area.  */
    if ((data_size + NX_ICMP_HEADER_SIZE) >
            (ULONG) (request_ptr -> nx_packet_data_end - request_ptr -> nx_packet_append_ptr))
    {

        /* Release the packet.  */
        nx_packet_release(request_ptr);

        /* Error, the data area is too big for the default packet payload.  */
        return(NX_OVERFLOW);
    }
#endif

    /* Find a suitable interface for sending the ping packet. */
#ifdef WICED_NETTYPE_ROM
    if(_nx_ip_route_find(ip_ptr, ip_address, &request_ptr -> nx_packet_ip_interface, &request_ptr->nx_packet_next_hop_address) != NX_SUCCESS)
#else
    if(_nx_ip_route_find(ip_ptr, ip_address, &request_ptr -> nx_packet_ip_interface, &next_hop_address) != NX_SUCCESS)
#endif
    {

        /* No suitable interface configured. */

        /* Release the packet. */
        nx_packet_release(request_ptr);


        /* Return the error status. */
        return(NX_IP_ADDRESS_ERROR);
    }

#ifdef ENABLE_LONG_PING
    /* Now, copy the data into the NetX packet.  */
    status = nx_packet_data_append(request_ptr, (VOID *) data_ptr, data_size, ip_ptr -> nx_ip_default_packet_pool, 2000);

    /* Was the data copy successful?  */
    if (status)
    {
        WPRINT_APP_INFO(("%s: data_append error\n", __FUNCTION__));
        /* Release the packet.  */
        nx_packet_release(request_ptr);

        return(status);
    }
#else

    /* Determine if the size of the data and the ICMP header is larger than
       the packet payload area.  */
    if ((data_size + NX_ICMP_HEADER_SIZE) >
            (ULONG) (request_ptr -> nx_packet_data_end - request_ptr -> nx_packet_append_ptr))
    {

        /* Release the packet.  */
        nx_packet_release(request_ptr);

        /* Error, the data area is too big for the default packet payload.  */
        return(NX_OVERFLOW);
    }
    /* Copy the data into the packet payload area.  */
    memcpy(request_ptr -> nx_packet_prepend_ptr+sizeof(NX_ICMP_HEADER), data_ptr, data_size);

    /* Calculate the ICMP echo request message size and store it in the
       packet header.  */
    request_ptr -> nx_packet_length =  data_size + NX_ICMP_HEADER_SIZE;

    /* Setup the append pointer to the end of the message.  */
    request_ptr -> nx_packet_append_ptr =  request_ptr -> nx_packet_prepend_ptr + (data_size + NX_ICMP_HEADER_SIZE);
#endif

#ifndef NX_DISABLE_ICMP_INFO
    /* Increment the ICMP ping count.  */
    ip_ptr -> nx_ip_pings_sent++;
#endif

    /* Build the ICMP request packet.  */
    /* Setup the pointer to the message area.  */
    header_ptr =  (NX_ICMP_HEADER *) request_ptr -> nx_packet_prepend_ptr;

    /* Write the ICMP type into the message.  Use the lower 16-bits of the IP address for
       the ICMP identifier.  */
    header_ptr -> nx_icmp_header_word_0 =  (ULONG) (NX_ICMP_ECHO_REQUEST_TYPE << 24);
    sequence =                             (ip_ptr -> nx_ip_icmp_sequence++ & NX_LOWER_16_MASK);
    header_ptr -> nx_icmp_header_word_1 =  (ULONG) (request_ptr -> nx_packet_ip_interface -> nx_interface_ip_address << 16) | sequence;

    /* If NX_LITTLE_ENDIAN is defined, the headers need to be swapped to match
       that of the data area.  */
    NX_CHANGE_ULONG_ENDIAN(header_ptr -> nx_icmp_header_word_0);
    NX_CHANGE_ULONG_ENDIAN(header_ptr -> nx_icmp_header_word_1);

#ifndef NX_DISABLE_ICMP_TX_CHECKSUM
    /* Compute the checksum of the ICMP packet.  */
    checksum = _nx_ip_checksum_compute(request_ptr, NX_IP_ICMP,
                                       request_ptr->nx_packet_length,
                                       /* ICMPV4 checksum does not include
                                       src/dest addresses */
                                       NX_NULL, NX_NULL);

    /* If NX_LITTLE_ENDIAN is defined, the headers need to be swapped back so
       we can place the checksum in the ICMP header.  */
    NX_CHANGE_ULONG_ENDIAN(header_ptr -> nx_icmp_header_word_0);

    /* Place the checksum into the first header word.  */
    header_ptr -> nx_icmp_header_word_0 =  header_ptr -> nx_icmp_header_word_0 | (~checksum & NX_LOWER_16_MASK);

    /* If NX_LITTLE_ENDIAN is defined, the first header word needs to be swapped
       back.  */
    NX_CHANGE_ULONG_ENDIAN(header_ptr -> nx_icmp_header_word_0);
#endif /* NX_DISABLE_ICMP_TX_CHECKSUM */

    /* Obtain the IP internal mutex to prevent against possible suspension later in the
       call to IP packet send.  */
    tx_mutex_get(&(ip_ptr -> nx_ip_protection), TX_WAIT_FOREVER);

    /* Disable interrupts.  */
    TX_DISABLE

    /* Temporarily disable preemption.  */
    _tx_thread_preempt_disable++;

    /* Pickup thread pointer.  */
    thread_ptr =  _tx_thread_current_ptr;

    /* Determine if the request specifies suspension.  */
    if (wait_option)
    {

        /* Prepare for suspension of this thread.  */

        /* Setup cleanup routine pointer.  */
        thread_ptr -> tx_thread_suspend_cleanup =  _nx_icmp_cleanup;

        thread_ptr -> tx_thread_suspend_status =   NX_NO_RESPONSE;

        /* Setup cleanup information, i.e. this pool control
           block.  */
        thread_ptr -> tx_thread_suspend_control_block =  (void *) ip_ptr;

        /* Save the return packet pointer address as well.  */
        thread_ptr -> tx_thread_additional_suspend_info =  (void *) response_ptr;

        /* Save the sequence number so this can be matched up with an ICMP
           response later.  */
        thread_ptr -> tx_thread_suspend_info =  sequence;

        /* Setup suspension list.  */
        if (ip_ptr -> nx_ip_icmp_ping_suspension_list)
        {

            /* This list is not NULL, add current thread to the end. */
            thread_ptr -> tx_thread_suspended_next =
                        ip_ptr -> nx_ip_icmp_ping_suspension_list;
            thread_ptr -> tx_thread_suspended_previous =
                        (ip_ptr -> nx_ip_icmp_ping_suspension_list) -> tx_thread_suspended_previous;
            ((ip_ptr -> nx_ip_icmp_ping_suspension_list) -> tx_thread_suspended_previous) -> tx_thread_suspended_next =
                        thread_ptr;
            (ip_ptr -> nx_ip_icmp_ping_suspension_list) -> tx_thread_suspended_previous =   thread_ptr;
        }
        else
        {

            /* No other threads are suspended.  Setup the head pointer and
               just setup this threads pointers to itself.  */
            ip_ptr -> nx_ip_icmp_ping_suspension_list =    thread_ptr;
            thread_ptr -> tx_thread_suspended_next =       thread_ptr;
            thread_ptr -> tx_thread_suspended_previous =   thread_ptr;
        }

        /* Increment the suspended thread count.  */
        ip_ptr -> nx_ip_icmp_ping_suspended_count++;

        /* Set the state to suspended.  */
        thread_ptr -> tx_thread_state =  TX_TCP_IP;

        /* Set the suspending flag.  */
        thread_ptr -> tx_thread_suspending =  TX_TRUE;

        /* Save the timeout value.  */
        thread_ptr -> tx_thread_timer.tx_timer_internal_remaining_ticks =  wait_option;
    }

    /* Restore interrupts.  */
    TX_RESTORE

    /* Send the ICMP packet to the IP component.  */
#ifdef WICED_NETTYPE_ROM
    _nx_ip_packet_send(ip_ptr, request_ptr, ip_address,
                    NX_IP_NORMAL, NX_IP_TIME_TO_LIVE, NX_IP_ICMP, NX_FRAGMENT_OKAY);
#else
    _nx_ip_packet_send(ip_ptr, request_ptr, ip_address,
                    NX_IP_NORMAL, NX_IP_TIME_TO_LIVE, NX_IP_ICMP, NX_FRAGMENT_OKAY, next_hop_address);
#endif

    /* If wait option is requested, suspend the thread.  */
    if (wait_option)
    {

        /* Release the protection on the ARP list.  */
        tx_mutex_put(&(ip_ptr -> nx_ip_protection));

        /* Call actual thread suspension routine.  */
        _tx_thread_system_suspend(thread_ptr);

        /* Return the status from the thread control block.  */
        return(thread_ptr -> tx_thread_suspend_status);
    }
    else
    {

        /* Disable interrupts.  */
        TX_DISABLE

        /* Release preemption disable.  */
        _tx_thread_preempt_disable--;

        /* Restore interrupts.  */
        TX_RESTORE

        /* Release the protection mutex.  */
        tx_mutex_put(&(ip_ptr -> nx_ip_protection));

        /* Check for preemption.  */
        _tx_thread_system_preempt_check();

        /* Immediate return, return error completion.  */
        return(NX_NO_RESPONSE);
    }
}
