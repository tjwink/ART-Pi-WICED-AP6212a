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
 * TCP Async Server Application
 *
 * This application snippet demonstrates how to use a softAP to create a
 * Wi-Fi network, and how to create a TCP server using asynchronous callbacks
 * to handle TCP client connections
 *
 * Features demonstrated
 *  - Wi-Fi softAP mode
 *  - DHCP server
 *  - Asynchronous TCP transmit and receive
 *  - TCP keep alive
 *
 * Application Instructions
 *   1. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *   2. Connect your computer to the WICED softAP with the security credentials
 *      shown in wifi_config_dct.h : WICED TCP Server Async App / abcd1234
 *   3. Ensure Python 2.7.x (*NOT* 3.x) is installed on your computer
 *   4. Open a command shell
 *   5. Run the python TCP client as follows from the tcp_async_server directory
 *     c:\<WICED-SDK>\Apps\snip\tcp_async_server> c:\path\to\Python27\python.exe tcp_client.py
 *       - Ensure your firewall allows TCP for Python on port 50007
 *
 * When the python TCP client runs on your computer, it sends a message "Hello WICED!"
 * to the WICED TCP server. When the message is received, it is printed to the
 * WICED serial port and appears on the terminal.
 *
 * If the TCP keep alive flag is set, the client keeps the TCP connection
 * open after the message is sent, otherwise the connection is closed.
 *
 */

#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define TCP_SERVER_LISTEN_PORT              (50007)
#define TCP_PACKET_MAX_DATA_LENGTH          (30)

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

static wiced_result_t client_connected_callback   ( wiced_tcp_socket_t* socket, void* arg );
static wiced_result_t client_disconnected_callback( wiced_tcp_socket_t* socket, void* arg );
static wiced_result_t received_data_callback      ( wiced_tcp_socket_t* socket, void* arg );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static wiced_tcp_socket_t tcp_server_socket;

static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255, 255, 255, 0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
};

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( void )
{
    wiced_result_t result;

    /* Initialise the device and WICED framework */
    wiced_init( );

    /* Bring up the network interface */
    wiced_network_up( WICED_AP_INTERFACE, WICED_USE_INTERNAL_DHCP_SERVER, &device_init_ip_settings );

    /* Create a TCP server socket */
    if ( wiced_tcp_create_socket( &tcp_server_socket, WICED_AP_INTERFACE ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("TCP socket creation failed\r\n") );
    }

    /* Register callbacks to handle various TCP events */
    result = wiced_tcp_register_callbacks( &tcp_server_socket, client_connected_callback, received_data_callback, client_disconnected_callback, NULL );

    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("TCP server socket initialization failed\r\n") );
    }

    /* Start TCP server to listen for connections */
    if ( wiced_tcp_listen( &tcp_server_socket, TCP_SERVER_LISTEN_PORT ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("TCP server socket initialization failed\r\n") );
        wiced_tcp_delete_socket( &tcp_server_socket );
        return;
    }

    WPRINT_APP_INFO( ("Async tcp server started. Listening on port %d \r\n", TCP_SERVER_LISTEN_PORT) );
}

static wiced_result_t client_connected_callback( wiced_tcp_socket_t* socket, void* arg )
{
    wiced_result_t      result;
    wiced_ip_address_t  ipaddr;
    uint16_t            port;

    UNUSED_PARAMETER( arg );

    /* Accept connection request */
    result = wiced_tcp_accept( socket );
    if( result == WICED_SUCCESS )
    {
        /* Extract IP address and the Port of the connected client */
        wiced_tcp_server_peer( socket, &ipaddr, &port );

        WPRINT_APP_INFO(("Accepted connection from :: "));

        WPRINT_APP_INFO ( ("IP %u.%u.%u.%u : %d\r\n", (unsigned char) ( ( GET_IPV4_ADDRESS(ipaddr) >> 24 ) & 0xff ),
                                                      (unsigned char) ( ( GET_IPV4_ADDRESS(ipaddr) >> 16 ) & 0xff ),
                                                      (unsigned char) ( ( GET_IPV4_ADDRESS(ipaddr) >>  8 ) & 0xff ),
                                                      (unsigned char) ( ( GET_IPV4_ADDRESS(ipaddr) >>  0 ) & 0xff ),
                                                      port ) );
        return WICED_SUCCESS;
    }
    return WICED_ERROR;
}

static wiced_result_t client_disconnected_callback( wiced_tcp_socket_t* socket, void* arg )
{
    UNUSED_PARAMETER( arg );

    WPRINT_APP_INFO(("Client disconnected\r\n\r\n"));

    wiced_tcp_disconnect(socket);
    /* Start listening on the socket again */
    if ( wiced_tcp_listen( socket, TCP_SERVER_LISTEN_PORT ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("TCP server socket re-initialization failed\r\n") );
        wiced_tcp_delete_socket( socket );
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

static wiced_result_t received_data_callback( wiced_tcp_socket_t* socket, void* arg )
{
    wiced_result_t      result;
    wiced_packet_t*     tx_packet;
    char*               tx_data;
    wiced_packet_t*     rx_packet = NULL;
    char*               request;
    uint16_t            request_length;
    uint16_t            available_data_length;

    result = wiced_tcp_receive( socket, &rx_packet, WICED_WAIT_FOREVER );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }

    wiced_packet_get_data( rx_packet, 0, (uint8_t**) &request, &request_length, &available_data_length );

    if (request_length != available_data_length)
    {
        WPRINT_APP_INFO(("Fragmented packets not supported\r\n"));
        return WICED_ERROR;
    }

    /* Null terminate the received string */
    request[request_length] = '\x0';
    WPRINT_APP_INFO(("Received data: %s \r\n", request));

    /* Send echo back */
    if ( wiced_packet_create_tcp( socket, TCP_PACKET_MAX_DATA_LENGTH, &tx_packet, (uint8_t**)&tx_data, &available_data_length ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("TCP packet creation failed\r\n"));
        return WICED_ERROR;
    }

    /* Write the message into tx_data"  */
    tx_data[request_length] = '\x0';
    memcpy( tx_data, request, request_length );

    /* Set the end of the data portion */
    wiced_packet_set_data_end( tx_packet, (uint8_t*)tx_data + request_length );

    /* Send the TCP packet */
    if ( wiced_tcp_send_packet( socket, tx_packet ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("TCP packet send failed\r\n") );

        /* Delete packet, since the send failed */
        wiced_packet_delete( tx_packet );
    }
    WPRINT_APP_INFO(("Echo data: %s\r\n", tx_data));

    /* Release a packet */
    wiced_packet_delete( rx_packet );
    return WICED_SUCCESS;
}
