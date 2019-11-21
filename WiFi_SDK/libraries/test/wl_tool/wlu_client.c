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

/*
 * Linux port of wl command line utility
 */

#include "typedefs.h"
#include <stdio.h>
#include "bcmutils.h"
#include "wlu_cmd.h"
#include <string.h>
#include "wlu_remote.h"
#include "wlu.h"
#include <stdlib.h>

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define INTERACTIVE_NUM_ARGS             (15)
#define INTERACTIVE_MAX_INPUT_LENGTH    (256)
#define RWL_WIFI_JOIN_DELAY               (5)

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
static cmd_t* wl_find_cmd   ( char* name );
static int    do_interactive( void* ifr );
static int    wl_do_cmd     ( void* ifr, char **argv );
static int    process_args  ( void* ifr, char **argv );

/******************************************************
 *               Variable Definitions
 ******************************************************/
static uint    interactive_flag = 0;
int            remote_type      = NO_REMOTE;
unsigned short defined_debug    = DEBUG_ERR | DEBUG_INFO;

extern char g_rem_ifname[];

#ifdef RWL_SOCKET
#define DEFAULT_TCP_PORT 60000
#define DEFAULT_SERVER_IP "192.168.1.2"
char *server_ip_address = DEFAULT_SERVER_IP;
int server_port=DEFAULT_TCP_PORT;
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/
int main(int argc, char **argv)
{
    void*  ifr;
    char *ifname = NULL;
    int err = 0;
    int help = 0;
    int status = CMD_WL;
    void* serialHandle = NULL;
    struct ipv4_addr temp;


    wlu_av0 = argv[0];

    wlu_init();

    argv++;

    if ((status = wl_option(&argv, &ifname, &help)) == CMD_OPT) {
        if (ifname)
        {
            strncpy(g_rem_ifname, ifname, IFNAMSIZ);
        }
    }

    int found;
    do
    {
        found = 0;
        /* RWL socket transport Usage: --serial port_name */
        if ( *argv && strncmp ( *argv, "--serial", strlen( *argv )) == 0) {
            found = 1;
            argv++;
            remote_type = REMOTE_SERIAL;

            if (!(*argv)) {
                rwl_usage(remote_type);
                return err;
            }


            if ((serialHandle = rwl_open_transport(remote_type, *argv, 0, 0)) == NULL) {
                DPRINT_ERR(ERR, "serial device open error\n");
                return -1;
            }

            argv++;
            ifr = (void*) serialHandle;

        }
#ifdef RWL_SOCKET
       if ( *argv && strncmp( *argv, "--socket", strlen( *argv ) ) == 0 )
               {
                   found = 1;
                   argv++ ;
                   remote_type = REMOTE_SOCKET;

                   if ( !( *argv ) )
                   {
                       rwl_usage( remote_type );
                       return err;
                   }

                   DPRINT_ERR( ERR, "Opening Socket\n");
                   if ( ( serialHandle = rwl_open_transport( remote_type, *argv, 0, 0 ) ) == NULL )
                   {
                       DPRINT_ERR( ERR, "Socket Open Error\n" );
                       return -1;
                   }
                   else{
                       DPRINT_ERR( ERR, "Socket open success.\n" );
                   }

                   server_ip_address = *argv;
                   argv++ ;

                   if(isdigit(*argv[0])){
                   server_port = atoi(*argv);
                   argv++;
                   }

                   ifr = (void*) serialHandle;

               }
#endif
        if (( *argv ) && (strlen( *argv ) > 2) &&
            (strncmp( *argv, "--interactive", strlen( *argv )) == 0)) {
            interactive_flag = 1;
            found = 1;
            argv++;
        }
    } while ( found == 1 );

    if ( remote_type == NO_REMOTE )
    {
        printf( "Error: --serial <port> must be specified for Wiced serial WL app\n");
        return -1;
    }

    /* Check if wifi adapter is supported and get the ioctl version */
    if ( wl_check(ifr) )
    {
        printf("Wifi adapter unsupported\n");
        return -1;
    }

    if (interactive_flag == 1) {
        err = do_interactive(ifr);
        return err;
    }

    if ((*argv) && (interactive_flag == 0)) {
        err = process_args(ifr, argv);
        return err;
    }
    rwl_usage(remote_type);

    if (remote_type != NO_REMOTE )
        rwl_close_transport(remote_type, ifr);

    return err;
}

/*
 * Function called for  'local' execution and for 'remote' non-interactive session
 * (shell cmd, wl cmd)
 */
static int process_args( void* ifr, char **argv)
{
    char *ifname = NULL;
    int help = 0;
    int status = 0, retry;
    int err = 0;
    cmd_t *cmd = NULL;

    while (*argv) {
        if ((strcmp (*argv, "sh") == 0) && (remote_type != NO_REMOTE)) {
            argv++; /* Get the shell command */
            if (*argv) {
                err = rwl_shell_cmd_proc( ifr, argv, SHELL_CMD);
            } else {
                DPRINT_ERR(ERR, "Enter the shell \
                    command(e.g ls(Linux) or dir(Win CE) \n");
                err = -1;
            }
            return err;
        }

#ifdef RWLASD
        if ((strcmp (*argv, "asd") == 0) && (remote_type != NO_REMOTE)) {
            argv++; /* Get the asd command */
            if (*argv) {
                err = rwl_shell_cmd_proc((void*)ifr, argv, ASD_CMD);
            } else {
                DPRINT_ERR(ERR, "Enter the asd command (e.g ca_get_version \n");
                err = -1;
            }
            return err;
        }
#endif

        if ((status = wl_option(&argv, &ifname, &help)) == CMD_OPT) {
            if (help)
                break;
            if (ifname)
            {
                strncpy(g_rem_ifname, ifname, IFNAMSIZ);
            }
            continue;
        }
        /* parse error */
        else if (status == CMD_ERR)
            break;

        /* search for command */
        cmd = wl_find_cmd(*argv);

        /* do command */
        if ( cmd )
        {
            err = (*cmd->func)((void *) ifr, cmd, argv);
        }
        else
        {
            /* if not found, use default set_var and get_var commands */
            err = wl_varcmd.func((void *) ifr, cmd, argv);
            if ( err == 0 )
            {
                /* get/set var succeeded - pretend we used the command all along */
                cmd = &wl_varcmd;
            }
        }
        break;
    } /* while loop end */

/* provide for help on a particular command */
    if (help && *argv) {
        cmd = wl_find_cmd(*argv);
        if (cmd) {
            wl_cmd_usage(stdout, cmd);
        } else {
            DPRINT_ERR(ERR, "%s: Unrecognized command \"%s\", type -h for help\n",
                                                                      wlu_av0, *argv);
        }
    } else if (!cmd)
        wl_usage(stdout, NULL);
    else if (err == USAGE_ERROR)
        wl_cmd_usage(stderr, cmd);
    else if (err != 0)
        wl_printlasterror((void *) ifr);

    return err;
}

/* Function called for 'local' interactive session and for 'remote' interactive session */
static int
do_interactive( void* ifr )
{
    int err = 0, retry;

    while (1) {
        char *fgsret;
        char line[INTERACTIVE_MAX_INPUT_LENGTH];
        fprintf(stdout, "> ");
        fflush( stdout );
        fflush( stderr );
        fgsret = fgets(line, sizeof(line), stdin);

        /* end of file */
        if (fgsret == NULL)
            break;
        if (line[0] == '\n')
            continue;

        if (strlen (line) > 0) {
            /* skip past first arg if it's "wl" and parse up arguments */
            char *argv[INTERACTIVE_NUM_ARGS];
            int argc;
            char *token;
            argc = 0;

            while ((argc < (INTERACTIVE_NUM_ARGS - 1)) &&
                   ((token = strtok(argc ? NULL : line, " \t\n")) != NULL)) {
                argv[argc++] = token;
            }
            argv[argc] = NULL;

            if (strcmp(argv[0], "q") == 0 || strcmp(argv[0], "exit") == 0) {
                break;
            }

            if ((strcmp(argv[0], "sh") == 0) && (remote_type != NO_REMOTE))  {
                if (argv[1]) {
                    process_args(ifr, argv);
                } else {
                    DPRINT_ERR(ERR, "Give shell command");
                    continue;
                }
            } else { /* end shell */
                err = wl_do_cmd(ifr, argv);
            } /* end of wl */
            fflush( stdout );
            fflush( stderr );
        } /* end of strlen (line) > 0 */
    } /* while (1) */

    return err;
}

/*
 * find command in argv and execute it
 * Won't handle changing ifname yet, expects that to happen with the --interactive
 * Return an error if unable to find/execute command
 */
static int
wl_do_cmd( void* ifr, char **argv)
{
    cmd_t *cmd = NULL;
    int err = 0;
    int help = 0;
    char *ifname = NULL;
    int status = CMD_WL;

    /* skip over 'wl' if it's there */
    if (*argv && strcmp (*argv, "wl") == 0) {
        argv++;
    }

    /* handle help or interface name changes */
    if (*argv && (status = wl_option (&argv, &ifname, &help)) == CMD_OPT) {
        if (ifname) {
            strncpy(g_rem_ifname, ifname, IFNAMSIZ);
        }
    }

    /* in case wl_option eats all the args */
    if (!*argv) {
        return err;
    }

    if (status != CMD_ERR) {
        /* search for command */
        cmd = wl_find_cmd(*argv);

        /* defaults to using the set_var and get_var commands */
        if (!cmd) {
            cmd = &wl_varcmd;
        }
        /* do command */
        err = (*cmd->func)((void *)ifr, cmd, argv);
    }
    /* provide for help on a particular command */
    if (help && *argv) {
      cmd = wl_find_cmd(*argv);
     if (cmd) {
        wl_cmd_usage(stdout, cmd);
    } else {
            DPRINT_ERR(ERR, "%s: Unrecognized command \"%s\", type -h for help\n",
                   wlu_av0, *argv);
           }
    } else if (!cmd)
        wl_usage(stdout, NULL);
    else if (err == USAGE_ERROR)
        wl_cmd_usage(stderr, cmd);
    else if (err == IOCTL_ERROR)
        wl_printlasterror((void *)ifr);

    return err;
}

/* Search the wl_cmds table for a matching command name.
 * Return the matching command or NULL if no match found.
 */

#if (defined  WLCMD_VER_01 || defined CHIP43012C0 )
#define MAX_MODULES 2048
/* common function to find a command */
cmd_t *
wl_find_cmd(char *name)
{
    int i;
    cmd_t **cmd = {NULL};
    int module_count = 0;


    module_count = wl_get_module_count();

    cmd = wl_get_module_cmds();

    /* search cmd in modules */
    for (i = 0; i < module_count; i++) {

        /* search cmd in one cmd table */
        for ( ; (cmd[i] != NULL && cmd[i]->name); cmd[i]++) {
            /* stop if we find a matching name */
            if (!strcmp(cmd[i]->name, name)) {
                break;
            }
        }

        /* if a match was found, break out of module loop */
        if (cmd[i]->name != NULL) {
            break;
        }
    }

    if ( cmd[i] == NULL )
    {
        return NULL;
    }
    return (cmd[i]->name != NULL) ? cmd[i] : NULL;
}
#else
static cmd_t *
wl_find_cmd(char* name)
{
    cmd_t *cmd = NULL;

    /* search the wl_cmds for a matching name */
    for (cmd = wl_cmds; cmd->name && strcmp(cmd->name, name); cmd++);

    if (cmd->name == NULL)
        cmd = NULL;

    return cmd;
}
#endif
int
wl_get(void *wl, int cmd, void *buf, int len)
{
    unsigned long longlen = len;
#ifdef RWL_SOCKET
    setServerDetails(server_ip_address,server_port);
#endif
    return rwl_queryinformation_fe( wl, cmd, buf, &longlen, 0, REMOTE_GET_IOCTL );
}

int
wl_set(void *wl, int cmd, void *buf, int len)
{
    unsigned long longlen = len;
#ifdef RWL_SOCKET
    setServerDetails(server_ip_address,server_port);
#endif
    return rwl_setinformation_fe( wl, cmd, buf, &longlen, 0, REMOTE_SET_IOCTL );
}

/* take an array of arguments and copy into a single buffer, separating args with ' ' */
static int
wl_serialize_args( char **argv, int min_output_buffer_size, char **output_buffer, int *output_buffer_len )
{
    int err           = 0;
    int argc          = 0;
    int buf_len       = 0;
    char *buf         = NULL;
    char *buf_print   = NULL;

    /* init return values to safe */
    *output_buffer     = NULL;
    *output_buffer_len = 0;

    /* check input */
    if ( *argv == NULL )
    {
        err = FAIL;
        goto exit;
    }

    /* count args and change into a single buffer to send */
    while ( argv[argc] != NULL )
    {
        /* add 1 for NULL or whitespace separator */
        buf_len += ( strlen(argv[argc]) + 1 );
        argc++;
    }

    buf_len = MAX( buf_len, min_output_buffer_size );

    buf = malloc( buf_len );

    if ( buf == NULL )
    {
        err = FAIL;
        goto exit;
    }

    memset( buf, 0, buf_len );
    argc = 0;
    buf_print = buf;

    /* translate argv into a single buffer for sending over to board */
    while ( argv[argc] != NULL )
    {
         buf_print += snprintf( buf_print, strlen(argv[argc]) + 1, "%s", argv[argc] );
         argc++;

        /* separate the args */
        if (argv[argc] != NULL)
        {
            *buf_print = ' ';
            buf_print++;
        }
    }

    /* modify output variables */
    *output_buffer     = buf;
    *output_buffer_len = buf_len;

exit:
    return err;
}

/* Generic wl cmd sender -- use if there is no special processing */
int
wl_cmd_exec(void *wl, cmd_t *cmd, char **argv)
{
    char *buffer   = NULL;
    int buffer_len = 0;
    int err        = 0;

    if ( *argv == NULL )
    {
        err = FAIL;
        goto exit;
    }

    err = wl_serialize_args( argv, 0, &buffer, &buffer_len );

    if ( err != BCME_OK )
    {
        goto exit;
    }

    err = wlu_set( wl, WLC_SET_VAR, buffer, buffer_len );

    if (err != BCME_OK)
    {
        printf( "Failed %s err=%d\n", argv[0], err );
    }

    free( buffer );

exit:
    return err;
}

/* Send and print a request to get the current SDK IP state */
int
wl_wwd_ip_state(void *wl, cmd_t *cmd, char **argv)
{
    int err;
    char *buffer   = NULL;
    int buffer_len = 0;

    UNUSED_PARAMETER(cmd);

    err = wl_serialize_args( argv, 180, &buffer, &buffer_len );

    if ( err != 0 )
    {
        return err;
    }

    memset( buffer, 0 , sizeof ( buffer ) );
    memcpy( buffer, cmd->name, strlen( cmd->name ) );
    printf( "Sending %s...\n", buffer );

    err = wl_get( wl, WLC_GET_VAR, buffer, buffer_len );
    if ( err != BCME_OK )
    {
        printf( "Failed query err=%d\n", err );
    }
    else
    {
        printf("%s\n", buffer);
    }

    free(buffer);

    return err;
}
