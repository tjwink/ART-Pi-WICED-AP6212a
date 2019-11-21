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
 * Factory Reset Application
 *
 * ------------------------------------------------------
 * PLEASE read the following documentation before trying
 * this application!
 * ------------------------------------------------------
 *
 * This application reboots or causes a factory_reset on the next boot.
 * It is meant to be used with snip.ota_fr on boards that do not have a "Factory Reset" button.
 *
 * Features demonstrated
 *  - Factory Reset process
 *
 * WICED Multi-Application Support:
 * =================================
 * As of WICED-SDK 3.1.1, WICED Application Framework (WAF) supports loading and storing of multiple
 * application binaries in the external serial flash. Up to 8 binaries are supported. The first
 * five binaries are reserved for internal usage and the last three binaries are free for users to use.
 * The binaries are organised as follows:
 *  - Factory reset application (FR_APP)
 *  - DCT backup image (DCT_IMAGE)
 *  - OTA upgrade application (OTA_APP)
 *  - Resources file system (FILESYSTEM_IMAGE)
 *  - WIFI firmware (WIFI_FIRMWARE)
 *  - Application 0 (APP0)
 *  - Application 1 (APP1)
 *  - Application 2 (APP2)
 *
 * Snippet Application:
 * =========================
 * Follow the instructions in snip/ota_fr/ota_fr.c to build and run that application.
 * In Step 4, use this application instead of snip.scan
 *            make snip.ota_fr_no_buttons-<PLATFORM>
 *
 * Perform factory reset from the console by typing 'factory_reset'
 *   > factory_reset
 *
 *   - The WICED evaluation board will reboot and run the factory reset (OTA_FR) application.
 *     Observe the log messages at the terminal to confirm the factory reset is completed successfully.
 *
 */

#include "wiced.h"
#include "waf_platform.h"
#include "wwd_debug.h"
#include "wiced_framework.h"
#include "command_console.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define CONSOLE_COMMAND_MAX_LENGTH      80
#define CONSOLE_COMMAND_HISTORY_LENGTH  10

#define CONSOLE_COMMANDS \
    { (char*) "factory_reset",    console_command, 0, NULL, NULL, (char *)"", (char *)"Factory Reset" }, \

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum
{
    CONSOLE_CMD_FACTORY_RESET = 0,

    CONSOLE_CMD_MAX
} console_commands_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

static int  console_command  (int argc, char *argv[]);

/******************************************************
 *               Variable Definitions
 ******************************************************/

#ifdef CONNECTING_TO_ADHOC_NETWORK
static const wiced_ip_setting_t device_init_ip_settings =
{
    INITIALISER_IPV4_ADDRESS( .ip_address, MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
    INITIALISER_IPV4_ADDRESS( .netmask,    MAKE_IPV4_ADDRESS(255, 255, 255, 0) ),
    INITIALISER_IPV4_ADDRESS( .gateway,    MAKE_IPV4_ADDRESS(192, 168, 10,  1) ),
};
#endif /* #ifdef CONNECTING_TO_ADHOC_NETWORK */

 static char command_buffer[CONSOLE_COMMAND_MAX_LENGTH];
 static char command_history_buffer[CONSOLE_COMMAND_MAX_LENGTH * CONSOLE_COMMAND_HISTORY_LENGTH];

const command_t command_table[] =
{
    CONSOLE_COMMANDS
    CMD_TABLE_END
};

static char * command_lookup[CONSOLE_CMD_MAX] =
{
    "factory_reset",
};
/******************************************************
 *               Function Definitions
 ******************************************************/

void application_start( )
{
    wiced_result_t  result;

    wiced_init( );

    WPRINT_APP_INFO( ( "\r\nHi, I'm used for testing ota_fr on boards with no buttons!\r\n" ) );
    WPRINT_APP_INFO( ( "To test Factory Reset, type 'factory_reset'\r\n\r\n" ) );

    /*
     * Create the command console.
     */

    result = command_console_init(STDIO_UART, sizeof(command_buffer), command_buffer, CONSOLE_COMMAND_HISTORY_LENGTH, command_history_buffer, " ");
    if (result == WICED_SUCCESS)
    {
        console_add_cmd_table(command_table);
    }

    while ( 1 )
    {
        wiced_rtos_delay_milliseconds( 100 );
    }
}

int console_command(int argc, char *argv[])
{
    int i;
    /*
     * Lookup the command in our table.
     */

    for (i = 0; i < CONSOLE_CMD_MAX; ++i)
    {
        if (strcmp(command_lookup[i], argv[0]) == 0)
            break;
    }

    if (i >= CONSOLE_CMD_MAX)
    {
        WPRINT_APP_INFO( ( "Unrecognized command: %s\r\n", argv[0]) );
        return ERR_CMD_OK;
    }

    switch (i)
    {
        case CONSOLE_CMD_FACTORY_RESET:
            wiced_dct_restore_factory_reset( );
            wiced_waf_app_set_boot( DCT_FR_APP_INDEX, PLATFORM_DEFAULT_LOAD );
            wiced_framework_reboot();
            break;

        default:
            WPRINT_APP_INFO( ("%s: command not found.\n", argv[0]) );
            break;
    }

    return ERR_CMD_OK;
}


