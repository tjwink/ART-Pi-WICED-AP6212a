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

#if defined(TX_ENABLE_EVENT_TRACE)

#include "command_console.h"
#include "command_console_tracex.h"
#include "TraceX.h"
#include "tx_api.h"
#include "wiced.h"
#include "platform_config.h"  /* Platform-specific TraceX buffer location */

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

/*!
 ******************************************************************************
 * Enables TraceX
 *
 * @return  0 for success, otherwise error
 */

int command_console_tracex_enable( int argc, char* argv[] )
{
    int                   i;
    wiced_result_t        result;
    wiced_tracex_config_t config;

    /* Start with current config and just change the things we care about */
    if ( wiced_tracex_config_get(&config) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to get TraceX configuration\n"));
        return ERR_UNKNOWN;
    }

    config.loop_rec = WICED_FALSE;
    config.tcp_server.enable = WICED_FALSE;

    /* Parse arguments */
    for ( i = 1; i < argc; i++ )
    {
        switch ( argv[i][1] )
        {
            case 'l': /* Loop recording */
            {
                config.loop_rec = WICED_TRUE;
                break;
            }
            case 't': /* TCP server IP address */
            {
                uint32_t a, b, c, d;

                if ( i == ( argc - 1 ))
                {
                    WPRINT_APP_INFO(("Missing value for option -%c\n",
                                     argv[i][1]));
                    return ERR_INSUFFICENT_ARGS;
                }

                if ( sscanf(argv[i + 1], "%lu.%lu.%lu.%lu", &a, &b, &c, &d) != 4 )
                {
                    WPRINT_APP_INFO(("Invalid TCP server IP address\n"));
                    return ERR_INSUFFICENT_ARGS;
                }

                SET_IPV4_ADDRESS(config.tcp_server.ip,
                                 MAKE_IPV4_ADDRESS( a, b, c, d ));
                config.tcp_server.enable = WICED_TRUE;

                i++;
                break;
            }
            case 'p': /* Server port */
            {
                if ( i == ( argc - 1 ))
                {
                    WPRINT_APP_INFO(("Missing value for option -%c\n",
                                     argv[i][1]));
                    return ERR_INSUFFICENT_ARGS;
                }

                config.tcp_server.port = strtoul( argv[i + 1], NULL, 0 );
                config.tcp_server.enable = WICED_TRUE;

                i++;
                break;
            }
            case 'd': /* Data packet length */
            {
                if ( i == ( argc - 1 ))
                {
                    WPRINT_APP_INFO(("Missing value for option -%c\n",
                                     argv[i][1]));
                    return ERR_INSUFFICENT_ARGS;
                }

                config.tcp_server.max_data_len = strtoul( argv[i + 1], NULL, 0 );
                config.tcp_server.enable = WICED_TRUE;

                i++;
                break;
            }
            default:
            {
                WPRINT_APP_INFO(("Option '%s' not supported\n", argv[i]));
                return ERR_UNKNOWN;
            }
        }
    }

    WPRINT_APP_INFO(("Enabling TraceX\n"));
    result = wiced_tracex_enable(&config);
    if ( result == WICED_NOT_CONNECTED )
    {
        WPRINT_APP_INFO(("Failed to connect to TCP server (%lu.%lu.%lu.%lu:%u)\n",
                         (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 24) & 0xFF,
                         (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 16) & 0xFF,
                         (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 8) & 0xFF,
                         (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 0) & 0xFF,
                         config.tcp_server.port));
        return ERR_UNKNOWN;
    }
    else if ( result == WICED_BADVALUE )
    {
        WPRINT_APP_INFO(("TraceX was already enabled\n"));
        return ERR_UNKNOWN;
    }
    else if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to enable TraceX\n"));
        return ERR_UNKNOWN;
    }

    return ERR_CMD_OK;
}

/*!
 ******************************************************************************
 * Disables TraceX and sends out event buffer to TCP server (if specified)
 *
 * @return  0 for success, otherwise error
 */

int command_console_tracex_disable( int argc, char* argv[] )
{
    wiced_tracex_status_t status;

    if ( wiced_tracex_status_get(&status) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to get TraceX status\n"));
        return ERR_UNKNOWN;
    }

    if ( status.state < WICED_TRACEX_STATE_ENABLED )
    {
        WPRINT_APP_INFO(("TraceX not running\n"));
        return ERR_CMD_OK;
    }

    WPRINT_APP_INFO(("Disabling TraceX\n"));
    if ( wiced_tracex_disable() != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to disable TraceX\n"));
        return ERR_UNKNOWN;
    }

    return ERR_CMD_OK;
}

/*!
 ******************************************************************************
 * Restarts TraceX with current configuration
 *
 * @return  0 for success, otherwise error
 */

int command_console_tracex_restart( int argc, char* argv[] )
{
    wiced_tracex_status_t status;
    wiced_tracex_config_t config;
    wiced_result_t        result;

    if ( wiced_tracex_status_get(&status) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to get TraceX status\n"));
        return ERR_UNKNOWN;
    }

    if ( wiced_tracex_config_get(&config) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to get TraceX configuration\n"));
        return ERR_UNKNOWN;
    }

    /* Restart from any state except if disabled by user or first run */
    if ( status.state == WICED_TRACEX_STATE_DISABLED )
    {
        WPRINT_APP_INFO(("TraceX not running\n"));
        return ERR_UNKNOWN;
    }

    if ( status.state == WICED_TRACEX_STATE_ENABLED )
    {
        WPRINT_APP_INFO(("Disabling TraceX\n"));
        if ( wiced_tracex_disable() != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("Failed to disable TraceX\n"));
            return ERR_UNKNOWN;
        }
    }
    else
    {
        WPRINT_APP_INFO(("TraceX already disabled\n"));
    }

    WPRINT_APP_INFO(("Enabling TraceX\n"));
    result = wiced_tracex_enable(&config);
    if ( result == WICED_NOT_CONNECTED )
    {
        WPRINT_APP_INFO(("Failed to connect to TCP server (%lu.%lu.%lu.%lu:%u)\n",
                         (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 24) & 0xFF,
                         (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 16) & 0xFF,
                         (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 8) & 0xFF,
                         (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 0) & 0xFF,
                         config.tcp_server.port));
        return ERR_UNKNOWN;
    }
    else if ( result == WICED_BADVALUE )
    {
        WPRINT_APP_INFO(("TraceX was already enabled\n"));
        return ERR_UNKNOWN;
    }
    else if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to enable TraceX\n"));
        return ERR_UNKNOWN;
    }

    return ERR_CMD_OK;
}

/*!
 ******************************************************************************
 * Prints out TraceX status
 *
 * @return  0 for success, otherwise error
 */

int command_console_tracex_status( int argc, char* argv[] )
{
    wiced_tracex_config_t config;
    wiced_tracex_status_t status;
    char                  str[WICED_TRACEX_FILTER_MAX_STRING_LEN];

    if ( wiced_tracex_status_get(&status) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to get TraceX status\n"));
        return ERR_UNKNOWN;
    }

    if ( wiced_tracex_config_get(&config) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to get TraceX configuration\n"));
        return ERR_UNKNOWN;
    }

    if ( wiced_tracex_filter_mask_to_str(config.filter, str, sizeof(str)) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to get TraceX filters\n"));
        return ERR_UNKNOWN;
    }

    WPRINT_APP_INFO(("TraceX state   : "));
    switch ( status.state )
    {
        case WICED_TRACEX_STATE_DISABLED:
        {
            WPRINT_APP_INFO(("Disabled\n"));
            break;
        }
        case WICED_TRACEX_STATE_DISABLED_BUF_FULL:
        {
            WPRINT_APP_INFO(("Disabled (buffer full)\n"));
            break;
        }
        case WICED_TRACEX_STATE_DISABLED_TCP_ERR:
        {
            WPRINT_APP_INFO(("Disabled (TCP error)\n"));
            break;
        }
        case WICED_TRACEX_STATE_DISABLED_TRACEX_ERR:
        {
            WPRINT_APP_INFO(("Disabled (TraceX error)\n"));
            break;
        }
        case WICED_TRACEX_STATE_ENABLED:
        {
            WPRINT_APP_INFO(("Enabled\n"));
            break;
        }
        default:
        {
            WPRINT_APP_INFO(("Unknown\n"));
            break;
        }
    }

    WPRINT_APP_INFO(("Buffer         : %u bytes @ %p with %lu objects\n",
                     config.buf.size, config.buf.addr, config.buf.obj_cnt));
    WPRINT_APP_INFO(("Loop recording : %s\n",
                     config.loop_rec == WICED_TRUE ? "Enabled" : "Disabled"));
    WPRINT_APP_INFO(("Filters        : '%s' (0x%08lx)\n", str, config.filter));
    WPRINT_APP_INFO(("TCP server     : %lu.%lu.%lu.%lu:%u (%s); max data length of %lu bytes\n",
                     (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 24) & 0xFF,
                     (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 16) & 0xFF,
                     (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 8) & 0xFF,
                     (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 0) & 0xFF,
                     config.tcp_server.port,
                     config.tcp_server.enable == WICED_FALSE ? "Disabled" :
                     status.tcp_connected == WICED_TRUE ? "Connected" :
                     "Disconnected", config.tcp_server.max_data_len));

    return ERR_CMD_OK;
}

/*!
 ******************************************************************************
 * Prints/modifies TraceX filters
 *
 * @return  0 for success, otherwise error
 */

int command_console_tracex_filter( int argc, char* argv[] )
{
    int                   i;
    wiced_tracex_config_t config;
    char                  str[WICED_TRACEX_FILTER_MAX_STRING_LEN];
    uint32_t              mask;
    wiced_bool_t          modified = WICED_FALSE;

    if ( wiced_tracex_config_get(&config) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to get TraceX configuration\n"));
        return ERR_UNKNOWN;
    }

    if ( argc == 1 )
    {
        wiced_tracex_filter_mask_to_str(config.filter, str, sizeof(str));

        WPRINT_APP_INFO(("Current TraceX filters: '%s' (0x%08lx)\n",
                         str, config.filter));
        return ERR_CMD_OK;
    }

    /* Parse arguments */
    for ( i = 1; i < argc; i++ )
    {
        switch ( argv[i][1] )
        {
            case 'a': /* Filter all */
            {
                config.filter = ~0;
                modified = WICED_TRUE;
                break;
            }
            case 'c': /* Clear all filters */
            {
                config.filter = 0;
                modified = WICED_TRUE;
                break;
            }
            case 'd': /* Default filter */
            {
                config.filter = WICED_TRACEX_DEFAULT_FILTER;
                modified = WICED_TRUE;
                break;
            }
            case 'f': /* Filter by name */
            {
                if ( i == ( argc - 1 ))
                {
                    WPRINT_APP_INFO(("Missing value for option -%c\n",
                                     argv[i][1]));
                    return ERR_INSUFFICENT_ARGS;
                }

                if ( strlen(argv[i + 1]) >= sizeof(str) )
                {
                    WPRINT_APP_INFO(("Filter string too long (max %u chars)\n",
                                     sizeof(str)));
                    return ERR_UNKNOWN_CMD;
                }

                if (( strncpy(str, argv[i + 1], sizeof(str)) == NULL ) ||
                    ( wiced_tracex_filter_str_to_mask(str, &mask) != WICED_SUCCESS ))
                {
                    WPRINT_APP_INFO(("Failed to process filter string\n"));
                    return ERR_UNKNOWN_CMD;
                }

                config.filter |= mask;
                modified = WICED_TRUE;

                i++;
                break;
            }
            case 'F': /* Filter by mask */
            {
                if ( i == ( argc - 1 ))
                {
                    WPRINT_APP_INFO(("Missing value for option -%c\n",
                                     argv[i][1]));
                    return ERR_INSUFFICENT_ARGS;
                }

                config.filter |= strtoul(argv[i + 1], NULL, 0);
                modified = WICED_TRUE;

                i++;
                break;
            }
            case 'u': /* Unfilter by name */
            {
                if ( i == ( argc - 1 ))
                {
                    WPRINT_APP_INFO(("Missing value for option -%c\n",
                                     argv[i][1]));
                    return ERR_INSUFFICENT_ARGS;
                }

                if ( strlen(argv[i + 1] ) >= sizeof(str) )
                {
                    WPRINT_APP_INFO(("Unfilter string too long (max %u chars)\n",
                                     sizeof(str)));
                    return ERR_UNKNOWN_CMD;
                }

                if (( strncpy(str, argv[i + 1], sizeof(str)) == NULL ) ||
                    ( wiced_tracex_filter_str_to_mask(str, &mask) != WICED_SUCCESS ))
                {
                    WPRINT_APP_INFO(("Failed to process unfilter string\n"));
                    return ERR_UNKNOWN_CMD;
                }

                config.filter &= ~(mask);
                modified = WICED_TRUE;

                i++;
                break;
            }
            case 'U': /* Unfilter by mask */
            {
                if ( i == ( argc - 1 ))
                {
                    WPRINT_APP_INFO(("Missing value for option -%c\n",
                                     argv[i][1]));
                    return ERR_INSUFFICENT_ARGS;
                }

                config.filter &= ~(strtoul(argv[i + 1], NULL, 0));
                modified = WICED_TRUE;

                i++;
                break;
            }
            case 'l': /* List out all known filters */
            {
                char* filter;
                char* saveptr;

                if ( wiced_tracex_filter_list_get(str, sizeof(str)) != WICED_SUCCESS )
                {
                    WPRINT_APP_INFO(("Failed to get list of TraceX filters\n"));
                    return ERR_UNKNOWN;
                }

                WPRINT_APP_INFO(("All known TraceX filters:\n"));
                filter = strtok_r(str, ",", &saveptr);
                while ( filter != NULL )
                {
                    if ( wiced_tracex_filter_str_to_mask(filter, &mask) != WICED_SUCCESS )
                    {
                        WPRINT_APP_INFO(("Failed to process TraceX filter '%s'\n",
                                         filter));
                        return ERR_UNKNOWN;
                    }

                    WPRINT_APP_INFO(("\t0x%08lx\t%s\n", mask, filter));

                    filter = strtok_r(NULL, ",", &saveptr);
                }

                break;
            }
            default:
            {
                WPRINT_APP_INFO(("Not supported, ignoring: %s\n", argv[i]));
                return ERR_UNKNOWN;
            }
        }
    }

    if ( modified == WICED_TRUE )
    {
        wiced_tracex_filter_mask_to_str(config.filter, str, sizeof(str));

        WPRINT_APP_INFO(("Setting new TraceX filters: '%s' (0x%08lx)\n", str,
                         config.filter));
        if ( wiced_tracex_filter_set(config.filter) != WICED_SUCCESS )
        {
            WPRINT_APP_INFO(("Failed to set TraceX filters\n"));
            return ERR_UNKNOWN;
        }
    }

    return ERR_CMD_OK;
}


int command_console_tracex_send(int argc, char* argv[])
{
    int                   i;
    wiced_result_t        result;
    wiced_tracex_config_t config;

    if ( wiced_tracex_config_get(&config) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to get TraceX configuration\n"));
        return ERR_UNKNOWN;
    }

    config.tcp_server.enable = WICED_TRUE;

    /* Parse arguments */
    for ( i = 1; i < argc; i++ )
    {
        switch ( argv[i][1] )
        {
            case 't': /* TCP server IP address */
            {
                uint32_t a, b, c, d;

                if ( i == ( argc - 1 ))
                {
                    WPRINT_APP_INFO(("Missing value for option -%c\n",
                                     argv[i][1]));
                    return ERR_INSUFFICENT_ARGS;
                }

                if ( sscanf(argv[i + 1], "%lu.%lu.%lu.%lu", &a, &b, &c, &d) != 4 )
                {
                    WPRINT_APP_INFO(("Invalid TCP server IP address\n"));
                    return ERR_INSUFFICENT_ARGS;
                }

                SET_IPV4_ADDRESS(config.tcp_server.ip,
                                 MAKE_IPV4_ADDRESS( a, b, c, d ));

                i++;
                break;
            }
            case 'p': /* Server port */
            {
                if ( i == ( argc - 1 ))
                {
                    WPRINT_APP_INFO(("Missing value for option -%c\n",
                                     argv[i][1]));
                    return ERR_INSUFFICENT_ARGS;
                }

                config.tcp_server.port = strtoul( argv[i + 1], NULL, 0 );

                i++;
                break;
            }
            case 'd': /* Data packet length */
            {
                if ( i == ( argc - 1 ))
                {
                    WPRINT_APP_INFO(("Missing value for option -%c\n",
                                     argv[i][1]));
                    return ERR_INSUFFICENT_ARGS;
                }

                config.tcp_server.max_data_len = strtoul( argv[i + 1], NULL, 0 );

                i++;
                break;
            }
            default:
            {
                WPRINT_APP_INFO(("Option '%s' not supported\n", argv[i]));
                return ERR_UNKNOWN;
            }
        }
    }

    WPRINT_APP_INFO(("Sending TraceX buffer to TCP server (%lu.%lu.%lu.%lu:%u)\n",
                     (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 24) & 0xFF,
                     (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 16) & 0xFF,
                     (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 8) & 0xFF,
                     (GET_IPV4_ADDRESS(config.tcp_server.ip) >> 0) & 0xFF,
                     config.tcp_server.port));
    result = wiced_tracex_buf_send( &config.tcp_server );
    if ( result == WICED_BADVALUE )
    {
        WPRINT_APP_INFO(("Cannot send TraceX buffer while TraceX is enabled\n"));
        return ERR_UNKNOWN;
    }
    else if ( result == WICED_NOT_CONNECTED )
    {
        WPRINT_APP_INFO(("Failed to connect to TCP server\n"));
        return ERR_UNKNOWN;
    }
    else if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to send TraceX buffer\n"));
        return ERR_UNKNOWN;
    }

    return ERR_CMD_OK;
}


/*!
 ******************************************************************************
 * Tests TraceX by inserting events
 *
 * @return  0 for success, otherwise error
 */

int command_console_tracex_test( int argc, char* argv[] )
{
    wiced_tracex_status_t status;
    uint32_t              i;
    uint32_t              num = 1;
    ULONG                 id = TX_TRACE_USER_EVENT_START;

    if ( wiced_tracex_status_get(&status) != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to get TraceX status\n"));
        return ERR_UNKNOWN;
    }

    if ( status.state != WICED_TRACEX_STATE_ENABLED )
    {
        WPRINT_APP_INFO(("TraceX not running\n"));
        return ERR_UNKNOWN;
    }

    /* Parse arguments */
    for ( i = 1; i < argc; i++ )
    {
        switch ( argv[i][1] )
        {
            case 'n':
                if (i == (argc - 1))
                {
                    WPRINT_APP_INFO(("Missing value for option -%c\n",
                                     argv[i][1]));
                    return ERR_INSUFFICENT_ARGS;
                }

                num = strtoul(argv[i + 1], NULL, 0);
                if ( num < 1 )
                {
                    WPRINT_APP_INFO(("Invalid number of events\n"));
                    return ERR_UNKNOWN;
                }

                i++;
                break;
            case 'i':
                if ( i == ( argc - 1 ))
                {
                    WPRINT_APP_INFO(("Missing value for option -%c\n",
                                     argv[i][1]));
                    return ERR_INSUFFICENT_ARGS;
                }

                id = strtoul(argv[i + 1], NULL, 0);
                if (( id < TX_TRACE_USER_EVENT_START ) ||
                    ( id > TX_TRACE_USER_EVENT_END ))
                {
                    WPRINT_APP_INFO(("Invalid event ID; valid range %d - %d\n",
                                     TX_TRACE_USER_EVENT_START,
                                     TX_TRACE_USER_EVENT_END));
                    return ERR_UNKNOWN;
                }
                i++;
                break;
           default:
                WPRINT_APP_INFO(("Not supported, ignoring: %s\n\r", argv[i]));
                return ERR_UNKNOWN;
        }
    }

    WPRINT_APP_INFO(("Inserting %lu test TraceX event(s)\n", num));
    for ( i = 0; i < num; i++ )
    {
        if ( tx_trace_user_event_insert(id, i, i, i, i) != TX_SUCCESS )
        {
            WPRINT_APP_INFO(("Failed to insert event %lu - aborting\n",
                             i + 1));
            break;
        }
    }

    return ERR_CMD_OK;
}

#endif /* defined(TX_ENABLE_EVENT_TRACE) */
