/*
 * wl command-line swiss-army-knife utility. Generic (not OS-specific)
 * implementation that interfaces to WLAN driver API (wl_drv.h).
 *
 *
 * $ Copyright Cypress Semiconductor 2008 $
 * $Id: wlu_generic.c 378881 2013-01-15 05:41:08Z richarch $
 */

/* ---- Include Files ---------------------------------------------------- */
#undef BCMDRIVER
#include "typedefs.h"
#include "bcmutils.h"
#include <stdio.h>
#include <string.h>

#include "wlioctl.h"
#include "wlu.h"
#include "wlu_remote.h"


/* ---- Public Variables ------------------------------------------------- */

/* ---- Private Constants and Types -------------------------------------- */

#define NUM_ARGS	64
#define DEFAULT_INTERFACE       "eth0"

/* ---- Private Variables ------------------------------------------------ */

/* ---- Private Function Prototypes -------------------------------------- */

static cmd_t* wl_find_cmd(char* name);
extern int wl_ioctl(void *wl, int cmd, void *buf, int len, bool set);
extern const char *wlu_av0;

/* ---- Functions -------------------------------------------------------- */
static int wlu_post_process_error( const char *name, size_t name_len, int bcmerror );

/****************************************************************************
* Function:   wl_find_cmd
*
* Purpose:    Search the wl_cmds table for a matching command name.
*             Return the matching command or NULL if no match found.
*
* Parameters: name (in) Name of command to find.
*
* Returns:    Command structure. NULL if not found.
*****************************************************************************
*/

#ifdef WLCMD_VER_01

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


/* ----------------------------------------------------------------------- */
int
wl_get(void *wl, int cmd, void *buf, int len)
{
   return wl_ioctl( wl, cmd, buf, len, FALSE );
}

/* ----------------------------------------------------------------------- */
int
wl_set(void *wl, int cmd, void *buf, int len)
{
    return wl_ioctl( wl, cmd, buf, len, TRUE );
}


/* ----------------------------------------------------------------------- */
int
wlu_main_args(int argc, char **argv)
{
	cmd_t *cmd = NULL;
	int err = 0;
	int help = 0;
	int status = CMD_WL;
	char *interface_option;
	char *interface = DEFAULT_INTERFACE;

	wlu_av0 = argv[0];

	/* Skip first arg. */
	argc--;
	argv++;

	wlu_init();

	while (*argv != NULL) {

		/* command option */
		if ((status = wl_option(&argv, &interface_option, &help)) == CMD_OPT) {
			if (help)
			{
				break;
			}
			else if ( interface_option != NULL )
			{
				interface = interface_option;
			}

			continue;
		}
		/* parse error */
		else if (status == CMD_ERR)
			break;
		/* wl command */
		/*
		 * else if (status == CMD_WL)
		 *	;
		 */
		set_interface( NULL, interface );

		/* Check if wifi adapter is supported and get the ioctl version */
		if (wl_check(NULL))
		{
			printf("WiFi adapter unsupported\n");
			return 0;
		}

		/* search for command */
		cmd = wl_find_cmd(*argv);

		/* defaults to using the set_var and get_var commands */
		if (!cmd)
			cmd = &wl_varcmd;

		/* do command */
		err = (*cmd->func)(NULL, cmd, argv);
		break;
	}

    err = wlu_post_process_error( cmd->name, strlen( cmd->name ), err );

	if (help && *argv) {
		cmd = wl_find_cmd(*argv);
		if (cmd)
			wl_cmd_usage(stdout, cmd);
		else {
			printf("%s: Unrecognized command \"%s\", type -h for help\n",
			       wlu_av0, *argv);
		}
	}
    else if ( !cmd )
    {
        wl_usage(stdout, NULL);
    }
    else if ( err == BCME_UNSUPPORTED )
    {
        fprintf(stderr, "wl: error %d\n", err );
    }
	else if (err == BCME_ERROR)
    {
		wl_cmd_usage(stderr, cmd);
    }
	else if (err != BCME_OK)
    {
		wl_printlasterror(NULL);
    }

	return 0;
}

static int wlu_post_process_error( const char *name, size_t name_len, int bcmerror )
{
#if ( WICED_PAYLOAD_MTU < 8192 )
    /* iovars that sometimes won't work with shorter buffer sizes */
    const char *unsupported_iovars[] =
    {
        "dump",
        "scanresults",
        NULL
    };
    int unsupported_index = 0;

    /* some iovars will not work due to shorter mtu length; mark as unsupported */
    if ( bcmerror != BCME_OK )
    {
        for ( unsupported_index = 0 ; unsupported_iovars[unsupported_index] != NULL ; unsupported_index++ )
        {
            if ( name_len == strlen( unsupported_iovars[unsupported_index] ) &&
                strncmp( unsupported_iovars[unsupported_index], name, name_len ) == 0)
            {
                bcmerror = BCME_UNSUPPORTED;
            }
        }
    }
#endif /* WICED_PAYLOAD_MTU */

    return bcmerror;
}
