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

/** @ws_erver.c
 *
 * Reference application for Websocket server application
 *
 */

#include "websocket.h"
#include "wiced_resource.h"
#include "resources.h"
#include "wiced_tls.h"
#include "command_console.h"

/******************************************************
 *                      Macros
 ******************************************************/

#define WEBSOCKET_SERVER_CONSOLE_COMMANDS \
    { (char*) "start",    websocket_server_start,    0,      NULL, NULL, (char *)"", (char *)"Start Websocket server" }, \
    { (char*) "stop",     websocket_server_stop,     0,      NULL, NULL, (char *)"", (char *)"Stop Websocket server"  }, \

/******************************************************
 *                    Constants
 ******************************************************/

#define BUFFER_LENGTH                                       (2048)
#define FINAL_FRAME                                         WICED_TRUE

#define WEBSOCKET_SERVER_CONSOLE_COMMAND_HISTORY_LENGTH     (10)
#define WEBSOCKET_SERVER_COMMAND_LENGTH                     (50)

/* comment-out below macro to make this server listen for non-secure client connections */
#define USE_WEBSOCKET_SECURE_SERVER

#define WEBSOCKET_SECURE_SERVER_PORT                        (443)
#define WEBSOCKET_SERVER_PORT                               (80)

#define MAX_NUM_OF_SUB_PROTOCOLS                            (2)

/* bumped it to same as rx_buffer size to echo-back bigger frames as well */
#define MAX_ECHO_LENGTH                                     (BUFFER_LENGTH)

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

static wiced_result_t websocket_server_on_open_callback     ( void* websocket );
static wiced_result_t websocket_server_on_close_callback    ( void* websocket );
static wiced_result_t websocket_server_on_error_callback    ( void* websocket );
static wiced_result_t websocket_server_on_message_callback  ( void* websocket, uint8_t* data, uint32_t length, wiced_websocket_frame_type_t type, wiced_websocket_frame_flags_t flags );
static int            websocket_server_start                ( int argc, char *argv[] );
static int            websocket_server_stop                 ( int argc, char *argv[] );

/******************************************************
 *               Variable Definitions
 ******************************************************/

const char myserver_protocol_list[MAX_NUM_OF_SUB_PROTOCOLS][10] = { "soap", "wamp" };

static wiced_websocket_url_protocol_entry_t  myserver_url_protocol_list[] =
{
    [0] =
    {
     .url = "ws://192.168.1.2:80/",
     .protocols = NULL,
    },
    [1] =
    {
     .url = "ws://192.168.1.2:80/chat",
     .protocols = (const char**)myserver_protocol_list,
    },
};

static wiced_websocket_url_protocol_table_t myserver_table =
{
    .count = 2,
    .entries = myserver_url_protocol_list,
};

static wiced_websocket_callbacks_t myserver_callbacks =
{
    .on_open    = websocket_server_on_open_callback,
    .on_close   = websocket_server_on_close_callback,
    .on_error   = websocket_server_on_error_callback,
    .on_message = websocket_server_on_message_callback,
};

static char rx_buffer[ BUFFER_LENGTH ];

static wiced_websocket_server_config_t myserver_config =
{
    .max_connections            = 4,
    .heartbeat_duration         = 0,
    .url_protocol_table         = &myserver_table,
    .rx_frame_buffer            = (uint8_t *)rx_buffer,
    .frame_buffer_length        = BUFFER_LENGTH,
};

static wiced_websocket_server_t myserver;

wiced_tls_identity_t myserver_tls_identity;

static char websocket_server_command_buffer[WEBSOCKET_SERVER_COMMAND_LENGTH];
static char websocket_server_command_history_buffer[WEBSOCKET_SERVER_COMMAND_LENGTH * WEBSOCKET_SERVER_CONSOLE_COMMAND_HISTORY_LENGTH];

/******************************************************
 *                    Structures
 ******************************************************/

const command_t websocket_server_console_command_table[] =
{
    WEBSOCKET_SERVER_CONSOLE_COMMANDS
    CMD_TABLE_END
};

/******************************************************
 *               Function Definitions
 ******************************************************/
/****************************************************************
 *  Console command Function Declarations
 ****************************************************************/

static void print_binary_data(uint8_t* data, uint32_t length)
{
    int i = 0;
    int offset = 0;

    printf("\n===== Start of Data Blob =====\n");
    for ( ; i < length/4; i++, offset = i*4 )
    {
        printf("\r\n");
        printf("%x\t%x\t%x\t%x\t", data[offset], data[offset + 1], data[offset + 2], data[offset + 3] );
    }

    printf("\r\n");
    if( length % 4 )
    {
        for( i = 0; i < length%4; i++ )
        {
            printf("%x\t", data[offset + i] );
        }
    }
    printf("\r\n");

    printf("\n===== End of Data Blob =====\n");
}

static int websocket_server_start(int argc, char *argv[])
{

    if( myserver.sockets != NULL)
    {
        WPRINT_APP_INFO( ( "\r\n[App] websocket server is already listening\r\n" ) );
        return WICED_SUCCESS;
    }

#ifdef USE_WEBSOCKET_SECURE_SERVER
    if ( wiced_websocket_server_start( &myserver, &myserver_config, &myserver_callbacks, &myserver_tls_identity, WEBSOCKET_SECURE_SERVER_PORT, NULL ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "[App] Failed to start Secure Websocket Server\n" ) );
        return WICED_ERROR;
    }
#else
    if ( wiced_websocket_server_start( &myserver, &myserver_config, &myserver_callbacks, NULL, WEBSOCKET_SERVER_PORT, NULL ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "[App] Failed to start Non-secure Websocket Server\n" ) );
        return WICED_ERROR;
    }
#endif

    WPRINT_APP_INFO( ("[App] WebSocket Server running(listening...)\n") );

    return WICED_SUCCESS;
}

static int websocket_server_stop(int argc, char *argv[])
{
    if(myserver.sockets == NULL)
    {
        WPRINT_APP_INFO(( "[App] websocket server not started\n") );
        return WICED_SUCCESS;
    }
    if( wiced_websocket_server_stop(&myserver) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "[App] websocket server stop failed\n") );
        return WICED_ERROR;
    }

    WPRINT_APP_INFO(( "[App] websocket server stopped\n") );

    return WICED_SUCCESS;
}

static wiced_result_t websocket_server_on_open_callback ( void* socket )
{
    UNUSED_PARAMETER(socket);
    WPRINT_APP_INFO(("[App] Connection established @websocket:%p\n", socket ) );
    return WICED_SUCCESS;
}

static wiced_result_t websocket_server_on_error_callback ( void* socket )
{
    wiced_websocket_t* websocket = ( wiced_websocket_t* )socket;
    WPRINT_APP_INFO(("[App] Error[%d] @websocket:%p\n", websocket->error_type, socket ) );
    return WICED_SUCCESS;
}

static wiced_result_t websocket_server_on_close_callback( void* socket )
{
    wiced_websocket_t* websocket = ( wiced_websocket_t* )socket;
    UNUSED_VARIABLE(websocket);
    WPRINT_APP_INFO(("[App] Connection Closed @websocket:%p\n", socket ) );
    return WICED_SUCCESS;
}

static wiced_result_t send_echo_frame( wiced_websocket_t* websocket, uint8_t* data, uint32_t length, wiced_websocket_frame_type_t type )
{
    uint8_t echo_payload[MAX_ECHO_LENGTH] = {0};
    memcpy( echo_payload, data, MIN(MAX_ECHO_LENGTH, length) );
    WICED_VERIFY( wiced_websocket_send( websocket, echo_payload, MIN(MAX_ECHO_LENGTH, length), type, WEBSOCKET_FRAME_FLAG_UNFRAGMENTED) );
    return WICED_SUCCESS;
}

static wiced_result_t websocket_server_on_message_callback( void* socket, uint8_t* data, uint32_t length, wiced_websocket_frame_type_t type, wiced_websocket_frame_flags_t flags )
{
    wiced_websocket_t* websocket = ( wiced_websocket_t* )socket;

    WPRINT_APP_INFO( ( "[App] Message received @websocket: %p\n", websocket) );

    switch( type )
    {
        case WEBSOCKET_TEXT_FRAME :
        {
            WPRINT_APP_INFO( ( "\t{ TEXT" ));
            WPRINT_APP_DEBUG( (" [final? %s continuation? %s]",( (int)flags == WEBSOCKET_FRAME_FLAG_UNFRAGMENTED || (int)flags == WEBSOCKET_FRAME_FLAG_FRAGMENTED_FINAL ) ? "yes" :"no",
                ( (int) flags == WEBSOCKET_FRAME_FLAG_FRAGMENTED_FIRST || (int) flags == WEBSOCKET_FRAME_FLAG_CONTINUED || (int)flags == WEBSOCKET_FRAME_FLAG_FRAGMENTED_FINAL ) ? "yes":"no" ) );
            WPRINT_APP_INFO((" }\r\n"));

            WPRINT_APP_INFO( ( "\t%.*s\r\n", (int)length, (char*)data ) );

            send_echo_frame( websocket, data, length, type );
            break;
        }

        case WEBSOCKET_BINARY_FRAME :
        {
            WPRINT_APP_INFO( ( "\t{ BLOB" ));
            WPRINT_APP_DEBUG( (" [final? %s continuation? %s]",( (int)flags == WEBSOCKET_FRAME_FLAG_UNFRAGMENTED || (int)flags == WEBSOCKET_FRAME_FLAG_FRAGMENTED_FINAL ) ? "yes" :"no",
                ( (int) flags == WEBSOCKET_FRAME_FLAG_FRAGMENTED_FIRST || (int) flags == WEBSOCKET_FRAME_FLAG_CONTINUED || (int)flags == WEBSOCKET_FRAME_FLAG_FRAGMENTED_FINAL ) ? "yes":"no" ) );
            WPRINT_APP_INFO((" }\r\n"));
            if( data && (length != 0) )
            {
                print_binary_data(data, length);
            }

            //send_echo_frame( websocket, data, length, type );

            break;
        }
        case WEBSOCKET_PONG:
        {
            WPRINT_APP_INFO( ( "\t{ PONG } ::\r\n" ) );
            if( data )
            {
                WPRINT_APP_INFO( ( "\tData: %.*s\r\n", (int)length, (char*)data ) );
            }
            break;
        }

        case WEBSOCKET_PING:
        {
            WPRINT_APP_INFO( ( "\t{ PING } ::\r\n" ) );
            if( data )
            {
                WPRINT_APP_INFO( ( "\tData: %.*s\r\n", (int)length, (char*)data ) );
            }
            send_echo_frame(websocket, data, length, WEBSOCKET_PONG);
            break;
        }

        case WEBSOCKET_CONNECTION_CLOSE:
        {
            wiced_result_t result;
            WPRINT_APP_INFO( ( "\t{ CLOSE } ::\r\n" ) );
            WPRINT_APP_INFO( ( "\tReplying with Close Frame\r\n" ) );
            result = wiced_websocket_close( websocket, WEBSOCKET_CLOSE_STATUS_CODE_NORMAL, NULL );
            if( result != WICED_SUCCESS )
            {
                WPRINT_APP_INFO( ( "\tError sending Close frame\r\n" ) );
            }
            break;
        }

        case WEBSOCKET_CONTINUATION_FRAME:
        default:
            WPRINT_APP_INFO( ( "\t(RESERVED/CONTINUATION) - Not handled\r\n" ) );
            break;
    }

    printf("\r\n");

    return WICED_SUCCESS;
}

void application_start( )
{
    wiced_result_t result = WICED_ERROR;
#ifdef USE_WEBSOCKET_SECURE_SERVER
    platform_dct_security_t* dct_security = NULL;
#endif
    wiced_init( );

    if ( wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL ) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO( ( "\r\n[App] Failed to bring up network\r\n" ) );
        return;
    }

    WPRINT_APP_INFO(( "[App] Network initialized\n") );

#ifdef USE_WEBSOCKET_SECURE_SERVER

    /* Set-up TLS identity structure with some server certificate etc. */

    /* Lock the DCT to allow us to access the certificate and key */
    WPRINT_APP_INFO( ( "[App] Read the certificate Key from DCT\n" ) );
    result = wiced_dct_read_lock( (void**) &dct_security, WICED_FALSE, DCT_SECURITY_SECTION, 0, sizeof( *dct_security ) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("[App] Unable to lock DCT to read certificate\n"));
    }

    /* Setup TLS identity */
    result = wiced_tls_init_identity( &myserver_tls_identity, dct_security->private_key, strlen( dct_security->private_key ), (uint8_t*) dct_security->certificate, strlen( dct_security->certificate ) );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "[App] Unable to initialize TLS identity. Error = [%d]\n", result ));
    }

    /* Finished accessing the certificates */
    result = wiced_dct_read_unlock( dct_security, WICED_FALSE );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(( "[App] DCT Read Unlock Failed. Error = [%d]\n", result ));
    }

#endif // End of USE_WEBSOCKET_SECURE_SERVER

    WPRINT_APP_INFO( ( "Websocket Server console start\n") );
    result = command_console_init(STDIO_UART, sizeof(websocket_server_command_buffer), websocket_server_command_buffer,
                                    WEBSOCKET_SERVER_CONSOLE_COMMAND_HISTORY_LENGTH, websocket_server_command_history_buffer, " ");

    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("[App] Error Starting the command console\r\n"));
        return;
    }
    result = console_add_cmd_table( websocket_server_console_command_table );
    if (result != WICED_SUCCESS)
    {
        WPRINT_APP_INFO(("[App] Error Adding command table\r\n"));
        return;
    }

}
