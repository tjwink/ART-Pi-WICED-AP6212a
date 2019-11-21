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
 * UDP Transmit Application
 *
 * This application snippet demonstrates how to send a UDP packet
 * to a network client (and optionally receive a UDP response).
 *
 * Features demonstrated
 *  - Wi-Fi softAP mode
 *  - DHCP server
 *  - UDP transmit
 *
 * Application Instructions
 *   1. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *   2. Ensure Python 2.7.x (*NOT* 3.x) is installed on your computer
 *   3. Connect your computer using Wi-Fi to "WICED UDP Transmit App"
 *        - SoftAP credentials are defined in wifi_config_dct.h
 *   4. Open a command shell
 *   5. Run the python UDP echo server as follows from the udp_transmit dir
 *      c:\<WICED-SDK>\Apps\snip\udp_transmit> c:\path\to\Python27\python.exe udp_echo_server.py
 *        - Ensure your firewall allows UDP for Python on port 50007
 *
 *   The network to be used can be changed by the #define WICED_NETWORK_INTERFACE in wifi_config_dct.h
 *   In the case of using AP or STA mode, change the AP_SSID and AP_PASSPHRASE accordingly.
 *
 *   When WICED_NETWORK_INTERFACE is set as WICED_AP_INTERFACE in wifi_config_dct.h,
 *   The WICED application starts a softAP, and regularly sends a broadcast
 *   UDP packet containing a sequence number. The WICED app prints the
 *   sequence number of the transmitted packet to the UART.
 *
 *   The computer running the UDP echo server receives the
 *   packet and echoes it back to the WICED application. If the
 *   #define GET_UDP_RESPONSE is enabled the WICED app prints the
 *   sequence number of any received packet to the UART.
 *
 *   When the Wi-Fi client (computer) joins the WICED softAP,
 *   it receives an IP address such as 192.168.0.2. To force
 *   the app to send UDP packets directly to the computer (rather than
 *   to a broadcast address), comment out the #define UDP_TARGET_IS_BROADCAST
 *   and change the target IP address to the IP address of your computer.
 *
 */

#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define UDP_MAX_DATA_LENGTH         30
#define UDP_RX_TIMEOUT              1
#define UDP_TX_INTERVAL             1
#define UDP_RX_INTERVAL             1
#define UDP_TARGET_PORT             50007
#define UDP_TARGET_IS_BROADCAST
#define GET_UDP_RESPONSE

#ifdef UDP_TARGET_IS_BROADCAST
#define UDP_TARGET_IP MAKE_IPV4_ADDRESS(192,168,0,255)
#else
#define UDP_TARGET_IP MAKE_IPV4_ADDRESS(192,168,0,2)
#endif

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

static wiced_result_t tx_udp_packet();
static wiced_result_t rx_udp_packet();

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192,168,  0,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255,255,255,  0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192,168,  0,  1) ),
};

static wiced_udp_socket_t  udp_socket;
static wiced_timed_event_t udp_tx_event;

static uint32_t tx_count   = 0;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    wiced_interface_t interface;
    wiced_result_t result;

    /* Initialise the device and WICED framework */
    wiced_init( );

    /* Bring up the softAP and network interface */
    result = wiced_network_up_default( &interface, &device_init_ip_settings );

    if( result != WICED_SUCCESS )
    {
        printf("Bringing up network interface failed !\r\n");
    }

    /* Create UDP socket */
    if (wiced_udp_create_socket(&udp_socket, UDP_TARGET_PORT, interface) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO( ("UDP socket creation failed\n") );
    }

    /* Register a function to send UDP packets */
    wiced_rtos_register_timed_event( &udp_tx_event, WICED_NETWORKING_WORKER_THREAD, &tx_udp_packet, UDP_TX_INTERVAL * SECONDS, 0 );

    WPRINT_APP_INFO( ("Sending a UDP packet every %d seconds ...\n", UDP_TX_INTERVAL) );

#ifdef GET_UDP_RESPONSE
    while ( 1 )
    {
        /* Try to receive a UDP response */
        rx_udp_packet( NEVER_TIMEOUT );
    }
#endif
}


/*
 * Sends a UDP packet
 */
wiced_result_t tx_udp_packet()
{
    wiced_packet_t*          packet;
    char*                    udp_data;
    uint16_t                 available_data_length;
    const wiced_ip_address_t INITIALISER_IPV4_ADDRESS( target_ip_addr, UDP_TARGET_IP );

    /* Create the UDP packet */
    if ( wiced_packet_create_udp( &udp_socket, UDP_MAX_DATA_LENGTH, &packet, (uint8_t**) &udp_data, &available_data_length ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("UDP tx packet creation failed\n") );
        return WICED_ERROR;
    }

    /* Write packet number into the UDP packet data */
    sprintf( udp_data, "%d", (int) tx_count++ );

    /* Set the end of the data portion */
    wiced_packet_set_data_end( packet, (uint8_t*) udp_data + UDP_MAX_DATA_LENGTH );

    /* Send the UDP packet */
    if ( wiced_udp_send( &udp_socket, &target_ip_addr, UDP_TARGET_PORT, packet ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("UDP packet send failed\n") );
        wiced_packet_delete( packet ); /* Delete packet, since the send failed */
        return WICED_ERROR;
    }

    /*
     * NOTE : It is not necessary to delete the packet created above, the packet
     *        will be automatically deleted *AFTER* it has been successfully sent
     */

    WPRINT_APP_INFO( ("sent: %d\n", (int)tx_count) );

    return WICED_SUCCESS;
}


/*
 * Attempts to receive a UDP packet
 */
wiced_result_t rx_udp_packet(uint32_t timeout)
{
    wiced_packet_t* packet;
    char*           udp_data;
    uint16_t        data_length;
    uint16_t        available_data_length;

    /* Wait for UDP packet */
    wiced_result_t result = wiced_udp_receive( &udp_socket, &packet, timeout );

    if ( ( result == WICED_ERROR ) || ( result == WICED_TIMEOUT ) )
    {
        return result;
    }

    wiced_packet_get_data( packet, 0, (uint8_t**) &udp_data, &data_length, &available_data_length );

    if (data_length != available_data_length)
    {
        WPRINT_APP_INFO(("Fragmented packets not supported\n"));
        return WICED_ERROR;
    }

    /* Null terminate the received string */
    udp_data[ data_length ] = '\x0';

    WPRINT_APP_INFO( ("%s\n\n", udp_data) );

    /* Delete packet as it is no longer needed */
    wiced_packet_delete( packet );

    return WICED_SUCCESS;
}
