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
 * UDP Receive Application
 *
 * This application snippet demonstrates how to receive a UDP packet
 * from a network client (and optionally send a response)
 *
 * Features demonstrated
 *  - Wi-Fi softAP mode
 *  - DHCP server
 *  - UDP receive (and response)
 *
 * Application Instructions
 *   1. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *   2. Ensure Python 2.7.x (*NOT* 3.x) is installed on your computer
 *   3. Connect your computer using Wi-Fi to "WICED UDP Receive App"
 *        - SoftAP credentials are defined in wifi_config_dct.h
 *   4. Open a command shell
 *   5. Run the python UDP transmit script as follows from the udp_receive dir
 *      c:\<WICED-SDK>\Apps\snip\udp_receive> c:\path\to\Python27\python.exe udp_transmit.py
 *        - Ensure your firewall allows UDP for Python on port 50007
 *
 *   The WICED application starts a softAP, and then regularly checks to
 *   see if a UDP packet has been received from a connected client.
 *   An optional response is sent directly to the client.
 *
 *   The udp_transmit script sends a broadcast UDP packet from your computer
 *   to the WICED application. The received packet contents are extracted by the
 *   WICED application and printed to the terminal.
 *
 *   To disable the UDP response, comment out the #define SEND_UDP_RESPONSE
 *
 *   The network to be used can be changed by the #define WICED_NETWORK_INTERFACE in wifi_config_dct.h
 *   In the case of using AP or STA mode, change the AP_SSID and AP_PASSPHRASE accordingly.
 *
 *
 */

#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define RX_WAIT_TIMEOUT        (1*SECONDS)
#define PORTNUM                (50007)           /* UDP port */
#define SEND_UDP_RESPONSE


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

static wiced_result_t process_received_udp_packet( );
static wiced_result_t send_udp_response( char* buffer, uint16_t buffer_length, wiced_ip_address_t ip_addr, uint32_t port );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192,168,  0,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255,255,255,  0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192,168,  0,  1) ),
};

static wiced_timed_event_t process_udp_rx_event;
static wiced_udp_socket_t  udp_socket;


/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    wiced_interface_t interface;
    wiced_result_t result;

    /* Initialise the device and WICED framework */
    wiced_init( );

    result = wiced_network_up_default( &interface, &device_init_ip_settings );

    if( result != WICED_SUCCESS )
    {
        printf("Bringing up network interface failed !\r\n");
    }

    /* Create UDP socket */
    if (wiced_udp_create_socket(&udp_socket, PORTNUM, interface) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO( ("UDP socket creation failed\n") );
    }

    /* Register a function to process received UDP packets */
    wiced_rtos_register_timed_event( &process_udp_rx_event, WICED_NETWORKING_WORKER_THREAD, &process_received_udp_packet, 1*SECONDS, 0 );

    WPRINT_APP_INFO(("Waiting for UDP packets ...\n"));
}


wiced_result_t process_received_udp_packet()
{
    wiced_packet_t*           packet;
    char*                     rx_data;
    static uint16_t           rx_data_length;
    uint16_t                  available_data_length;
    static wiced_ip_address_t udp_src_ip_addr;
    static uint16_t           udp_src_port;

    /* Wait for UDP packet */
    wiced_result_t result = wiced_udp_receive( &udp_socket, &packet, RX_WAIT_TIMEOUT );

    if ( ( result == WICED_ERROR ) || ( result == WICED_TIMEOUT ) )
    {
        return result;
    }

    /* Get info about the received UDP packet */
    wiced_udp_packet_get_info( packet, &udp_src_ip_addr, &udp_src_port );

    /* Extract the received data from the UDP packet */
    wiced_packet_get_data( packet, 0, (uint8_t**) &rx_data, &rx_data_length, &available_data_length );

    if (rx_data_length != available_data_length)
    {
        WPRINT_APP_INFO(("Fragmented packets not supproted\n"));
        return WICED_ERROR;
    }

    /* Null terminate the received data, just in case the sender didn't do this */
    rx_data[ rx_data_length ] = '\x0';

    WPRINT_APP_INFO ( ("UDP Rx: \"%s\" from IP %u.%u.%u.%u:%d\n", rx_data,
                                                                  (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >> 24 ) & 0xff ),
                                                                  (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >> 16 ) & 0xff ),
                                                                  (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >>  8 ) & 0xff ),
                                                                  (unsigned char) ( ( GET_IPV4_ADDRESS(udp_src_ip_addr) >>  0 ) & 0xff ),
                                                                  udp_src_port ) );
#ifdef SEND_UDP_RESPONSE
    /* Echo the received data to the sender */
    send_udp_response( rx_data, rx_data_length, udp_src_ip_addr, PORTNUM );
#endif

    /* Delete the received packet, it is no longer needed */
    wiced_packet_delete( packet );

    return WICED_SUCCESS;
}


static wiced_result_t send_udp_response (char* buffer, uint16_t buffer_length, wiced_ip_address_t ip_addr, uint32_t port)
{
    wiced_packet_t*          packet;
    char*                    tx_data;
    uint16_t                 available_data_length;
    const wiced_ip_address_t INITIALISER_IPV4_ADDRESS( target_ip_addr, GET_IPV4_ADDRESS(ip_addr) );

    /* Create the UDP packet. Memory for the TX data is automatically allocated */
    if ( wiced_packet_create_udp( &udp_socket, buffer_length, &packet, (uint8_t**) &tx_data, &available_data_length ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("UDP tx packet creation failed\n") );
        return WICED_ERROR;
    }

    /* Copy buffer into tx_data which is located inside the UDP packet */
    memcpy( tx_data, buffer, buffer_length + 1 );

    /* Set the end of the data portion of the packet */
    wiced_packet_set_data_end( packet, (uint8_t*) tx_data + buffer_length );

    /* Send the UDP packet */
    if ( wiced_udp_send( &udp_socket, &target_ip_addr, port, packet ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("UDP packet send failed\n") );
        /* Delete packet, since the send failed */
        wiced_packet_delete( packet );
    }
    else
    {
        WPRINT_APP_INFO( ("UDP Tx: \"echo: %s\"\n\n", tx_data) );
    }

    /*
     * NOTE : It is not necessary to delete the packet created above, the packet
     *        will be automatically deleted *AFTER* it has been successfully sent
     */

    return WICED_SUCCESS;
}
