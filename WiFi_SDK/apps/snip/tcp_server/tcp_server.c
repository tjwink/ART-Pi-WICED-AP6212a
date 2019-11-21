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
 * This application snippet demonstrates how to used the WICED
 * TCP API to implement a TCP server
 *
 * Features demonstrated
 *  - Wi-Fi client mode
 *  - DHCP client
 *  - TCP transmit and receive
 *  - TCP keep alive
 *
 * Application Instructions
 *   1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *      in the wifi_config_dct.h header file to match your Wi-Fi access point
 *   2. Ensure your computer is connected to the same Wi-Fi access point.
 *   3. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *   4. After the WICED board connects to your AP, look at the terminal
 *      output to find the IP address it received
 *   5. Open the file tcp_client.py and change the DEFAULT_IP address
 *      variable to match the IP address of the WICED board
 *   6. Ensure Python 2.7.x (*NOT* 3.x) is installed on your computer
 *   7. Open a command shell
 *   8. Run the python TCP client as follows from the tcp_server directory
 *     c:\<WICED-SDK>\Apps\snip\tcp_server> c:\path\to\Python27\python.exe tcp_client.py
 *       - Ensure your firewall allows TCP for Python on port 50007
 *       - IP address, port and keep alive options may be provided on the command line
 *         (or the defaults in the file may be used)
 *
 * When the python TCP client runs on your computer, it sends a message "Hello WICED!"
 * to the WICED TCP server. When the message is received, it is printed to the
 * WICED serial port and appears on the terminal.
 *
 * If the TCP keep alive flag is set, the client keeps the TCP connection
 * open after the message is sent, otherwise the connection is closed.
 *
 * The network to be used can be changed by the #define WICED_NETWORK_INTERFACE in wifi_config_dct.h
 * In the case of using AP or STA mode, change the AP_SSID and AP_PASSPHRASE accordingly.
 *
 */

#include "wiced.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define TCP_PACKET_MAX_DATA_LENGTH          (30)
#define TCP_SERVER_LISTEN_PORT              (50007)
#define TCP_SERVER_THREAD_PRIORITY          (WICED_DEFAULT_LIBRARY_PRIORITY)
/* Stack size should cater for printf calls */
#define TCP_SERVER_STACK_SIZE               (6200)
#define TCP_SERVER_COMMAND_MAX_SIZE         (10)
#define TCP_PACKET_MAX_DATA_LENGTH          (30)

/* Enable this define to demonstrate tcp keep alive procedure */
#define TCP_KEEPALIVE_ENABLED

/* Keepalive will be sent every 2 seconds */
#define TCP_SERVER_KEEP_ALIVE_INTERVAL      (2)
/* Retry 10 times */
#define TCP_SERVER_KEEP_ALIVE_PROBES        (5)
/* Initiate keepalive check after 5 seconds of silence on a tcp socket */
#define TCP_SERVER_KEEP_ALIVE_TIME          (5)
#define TCP_SILENCE_DELAY                   (30)


/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
typedef struct
{
    wiced_bool_t quit;
    wiced_tcp_socket_t socket;
}tcp_server_handle_t;
/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static void tcp_server_thread_main(uint32_t arg);
static wiced_result_t tcp_server_process(  tcp_server_handle_t* server, wiced_packet_t* rx_packet );

/******************************************************
 *               Variable Definitions
 ******************************************************/

static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192,168,  0,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255,255,255,  0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192,168,  0,  1) ),
};

static wiced_thread_t      tcp_thread;
static tcp_server_handle_t tcp_server_handle;
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

    if( result != WICED_SUCCESS )
    {
        printf("Bringing up network interface failed !\r\n");
    }

    /* Create a TCP server socket */
    if (wiced_tcp_create_socket(&tcp_server_handle.socket, interface) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("TCP socket creation failed\n"));
    }

    if (wiced_tcp_listen( &tcp_server_handle.socket, TCP_SERVER_LISTEN_PORT ) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("TCP server socket initialization failed\n"));
        wiced_tcp_delete_socket(&tcp_server_handle.socket);
        return;
    }

    /* Start a tcp server thread */
    WPRINT_APP_INFO(("Creating tcp server thread \n"));
    wiced_rtos_create_thread(&tcp_thread, TCP_SERVER_THREAD_PRIORITY, "Demo tcp server", tcp_server_thread_main, TCP_SERVER_STACK_SIZE, &tcp_server_handle);

}

static wiced_result_t tcp_server_process(  tcp_server_handle_t* server, wiced_packet_t* rx_packet )
{
    char*           request;
    uint16_t        request_length;
    uint16_t        available_data_length;
    wiced_packet_t* tx_packet;
    char*           tx_data;

    wiced_packet_get_data( rx_packet, 0, (uint8_t**) &request, &request_length, &available_data_length );

    if (request_length != available_data_length)
    {
        WPRINT_APP_INFO(("Fragmented packets not supported\n"));
        return WICED_ERROR;
    }

    /* Null terminate the received string */
    request[request_length] = '\x0';
    WPRINT_APP_INFO(("Received data: %s \n", request));

    /* Send echo back */
    if (wiced_packet_create_tcp(&server->socket, TCP_PACKET_MAX_DATA_LENGTH, &tx_packet, (uint8_t**)&tx_data, &available_data_length) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("TCP packet creation failed\n"));
        return WICED_ERROR;
    }

    /* Write the message into tx_data"  */
    tx_data[request_length] = '\x0';
    memcpy(tx_data, request, request_length);

    /* Set the end of the data portion */
    wiced_packet_set_data_end(tx_packet, (uint8_t*)tx_data + request_length);

    /* Send the TCP packet */
    if (wiced_tcp_send_packet(&server->socket, tx_packet) != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("TCP packet send failed\n"));

        /* Delete packet, since the send failed */
        wiced_packet_delete(tx_packet);

        server->quit=WICED_TRUE;
        return WICED_ERROR;
    }
    WPRINT_APP_INFO(("Echo data: %s\n", tx_data));

    return WICED_SUCCESS;
}

static void tcp_server_thread_main(uint32_t arg)
{
    tcp_server_handle_t* server = (tcp_server_handle_t*) arg;

    while ( server->quit != WICED_TRUE )
    {
        wiced_packet_t* temp_packet = NULL;

        /* Wait for a connection */
        wiced_result_t result = wiced_tcp_accept( &server->socket );

#ifdef TCP_KEEPALIVE_ENABLED
        result = wiced_tcp_enable_keepalive(&server->socket, TCP_SERVER_KEEP_ALIVE_INTERVAL, TCP_SERVER_KEEP_ALIVE_PROBES, TCP_SERVER_KEEP_ALIVE_TIME );
        if( result != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("Keep alive initialization failed \n"));
        }
#endif /* TCP_KEEPALIVE_ENABLED */

        if ( result == WICED_SUCCESS )
        {
            /* Receive the query from the TCP client */
            if (wiced_tcp_receive( &server->socket, &temp_packet, WICED_WAIT_FOREVER ) == WICED_SUCCESS)
            {
                /* Process the client request */
                tcp_server_process( server, temp_packet );

                /* Delete the packet, we're done with it */
                wiced_packet_delete( temp_packet );

#ifdef TCP_KEEPALIVE_ENABLED
                WPRINT_APP_INFO(("Waiting for data on a socket\n"));
                /* Check keepalive: wait to see whether the keepalive protocol has commenced */
                /* This is achieved by waiting forever for a packet to be received on the TCP connection*/
                if (wiced_tcp_receive( &server->socket, &temp_packet, WICED_WAIT_FOREVER ) == WICED_SUCCESS)
                {
                    tcp_server_process( server, temp_packet );
                    /* Release the packet, we don't need it any more */
                    wiced_packet_delete( temp_packet );
                }
                else
                {
                    WPRINT_APP_INFO(("Connection has been dropped by networking stack\n\n"));
                }
#endif /* TCP_KEEPALIVE_ENABLED */

            }
            else
            {
                /* Send failed or connection has been lost, close the existing connection and */
                /* get ready to accept the next one */
                wiced_tcp_disconnect( &server->socket );
            }
        }
    }
    WPRINT_APP_INFO(("Disconnect\n"));

    wiced_tcp_disconnect( &server->socket );

    WICED_END_OF_CURRENT_THREAD( );
}

