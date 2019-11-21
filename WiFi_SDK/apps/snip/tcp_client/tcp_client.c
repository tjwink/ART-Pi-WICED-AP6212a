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
 * TCP Client Application
 *
 * This application snippet demonstrates how to connect to a Wi-Fi
 * network and communicate with a TCP server
 *
 * Features demonstrated
 *  - Wi-Fi client mode
 *  - DHCP client
 *  - TCP transmit and receive
 *
 * Application Instructions
 *   1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *      in the wifi_config_dct.h header file to match your Wi-Fi access point
 *   2. Ensure your computer is connected to the same Wi-Fi access point.
 *   3. Determine the computer's IP address for the Wi-Fi interface.
 *   4. Change the #define TCP_SERVER_IP_ADDRESS in the code below to match
 *      the computer's IP address
 *   5. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *   6. Ensure Python 2.7.x (*NOT* 3.x) is installed on your computer
 *   7. Open a command shell
 *   8. Run the python TCP echo server as follows from the tcp_client directory
 *     c:\<WICED-SDK>\Apps\snip\tcp_client> c:\path\to\Python27\python.exe tcp_echo_server.py
 *       - Ensure your firewall allows TCP for Python on port 50007
 *
 * Every TCP_CLIENT_INTERVAL seconds, the app establishes a connection
 * with the remote TCP server, sends a message "Hello from WICED" and
 * receives an echo of the message in response. The response is printed
 * on the serial console.
 *
 * The network to be used can be changed by the #define WICED_NETWORK_INTERFACE in wifi_config_dct.h
 * In the case of using AP or STA mode, change the AP_SSID and AP_PASSPHRASE accordingly.
 *
 */

#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define TCP_PACKET_MAX_DATA_LENGTH        30
#define TCP_CLIENT_INTERVAL               2
#define TCP_SERVER_PORT                   50007
#define TCP_CLIENT_CONNECT_TIMEOUT        500
#define TCP_CLIENT_RECEIVE_TIMEOUT        300
#define TCP_CONNECTION_NUMBER_OF_RETRIES  3

/* Change the server IP address to match the TCP echo server address */
#define TCP_SERVER_IP_ADDRESS MAKE_IPV4_ADDRESS(192,168,1,1)


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

static wiced_result_t tcp_client();

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192,168,  0,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255,255,255,  0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192,168,  0,  1) ),
};

static wiced_tcp_socket_t  tcp_client_socket;
static wiced_timed_event_t tcp_client_event;

/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start(void)
{
    wiced_interface_t interface;
    wiced_result_t result;

    /* Initialise the device and WICED framework */
    wiced_init( );

    /* Bring up the network interface */
    result = wiced_network_up_default( &interface, &device_init_ip_settings );

    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("Bringing up network interface failed !\n") );
    }

    /* Create a TCP socket */
    if ( wiced_tcp_create_socket( &tcp_client_socket, interface ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ("TCP socket creation failed\n") );
    }

    /* Bind to the socket */
    wiced_tcp_bind( &tcp_client_socket, TCP_SERVER_PORT );

    /* Register a function to send TCP packets */
    wiced_rtos_register_timed_event( &tcp_client_event, WICED_NETWORKING_WORKER_THREAD, &tcp_client, TCP_CLIENT_INTERVAL * SECONDS, 0 );

    WPRINT_APP_INFO(("Connecting to the remote TCP server every %d seconds ...\n", TCP_CLIENT_INTERVAL));

}


wiced_result_t tcp_client( void* arg )
{
    wiced_result_t           result;
    wiced_packet_t*          packet;
    wiced_packet_t*          rx_packet;
    char*                    tx_data;
    char*                    rx_data;
    uint16_t                 rx_data_length;
    uint16_t                 available_data_length;
    const wiced_ip_address_t INITIALISER_IPV4_ADDRESS( server_ip_address, TCP_SERVER_IP_ADDRESS );
    int                      connection_retries;
    UNUSED_PARAMETER( arg );

    /* Connect to the remote TCP server, try several times */
    connection_retries = 0;
    do
    {
        result = wiced_tcp_connect( &tcp_client_socket, &server_ip_address, TCP_SERVER_PORT, TCP_CLIENT_CONNECT_TIMEOUT );
        connection_retries++;
    }
    while( ( result != WICED_SUCCESS ) && ( connection_retries < TCP_CONNECTION_NUMBER_OF_RETRIES ) );
    if( result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("Unable to connect to the server! Halt.\n"));
    }

    /* Create the TCP packet. Memory for the tx_data is automatically allocated */
    if (wiced_packet_create_tcp(&tcp_client_socket, TCP_PACKET_MAX_DATA_LENGTH, &packet, (uint8_t**)&tx_data, &available_data_length) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("TCP packet creation failed\n"));
        return WICED_ERROR;
    }

    /* Write the message into tx_data"  */
    sprintf(tx_data, "%s", "Hello from WICED\n");

    /* Set the end of the data portion */
    wiced_packet_set_data_end(packet, (uint8_t*)tx_data + strlen(tx_data));

    /* Send the TCP packet */
    if (wiced_tcp_send_packet(&tcp_client_socket, packet) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("TCP packet send failed\n"));

        /* Delete packet, since the send failed */
        wiced_packet_delete(packet);

        /* Close the connection */
        wiced_tcp_disconnect(&tcp_client_socket);
        return WICED_ERROR;
    }

    /* Receive a response from the server and print it out to the serial console */
    result = wiced_tcp_receive(&tcp_client_socket, &rx_packet, TCP_CLIENT_RECEIVE_TIMEOUT);
    if( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("TCP packet reception failed\n"));

        /* Delete packet, since the receive failed */
        wiced_packet_delete(rx_packet);

        /* Close the connection */
        wiced_tcp_disconnect(&tcp_client_socket);
        return WICED_ERROR;
    }

    /* Get the contents of the received packet */
    wiced_packet_get_data(rx_packet, 0, (uint8_t**)&rx_data, &rx_data_length, &available_data_length);

    if (rx_data_length != available_data_length)
    {
        WPRINT_APP_INFO(("Fragmented packets not supported\n"));

        /* Delete packet, since the receive failed */
        wiced_packet_delete(rx_packet);

        /* Close the connection */
        wiced_tcp_disconnect(&tcp_client_socket);
        return WICED_ERROR;
    }

    /* Null terminate the received string */
    rx_data[rx_data_length] = '\x0';
    WPRINT_APP_INFO(("%s", rx_data));

    /* Delete the packet and terminate the connection */
    wiced_packet_delete(rx_packet);
    wiced_tcp_disconnect(&tcp_client_socket);

    return WICED_SUCCESS;

}


