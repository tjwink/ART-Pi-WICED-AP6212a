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

#include "wiced.h"
#include "wiced_filesystem.h"
#include "resources.h"
#include "platform_config.h"
#include "resources.h"
#include "platform_resource.h"
#include "wiced_duktape.h"
#include "duk_module_node.h"
#ifdef WICED_DUKTAPE_MEMORY_POOL_ALLOC
#include "duk_alloc_hybrid.h"
#endif
#ifdef WICED_DUKTAPE_LOGGER
#include "duk_logging.h"
#endif
#ifdef WICED_DUKTAPE_TESTS_API
#include "wiced_duktape_tests_api_list.h"
#endif
#ifdef WICED_DUKTAPE_PANDORA_MODULES
#include "dpm.h"
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define LOG_LABEL           "duk:lib"
#define LOG_DEBUG_ENABLE    0

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct duktape_resource_s
{
    const resource_hnd_t*   resource;
    const char*             filename;
} duktape_resource_t;


typedef struct duktape_c_module_list_s
{
    struct duktape_c_module_list_s* next;
    wiced_duktape_c_module_t        c_module;
} duktape_c_module_list_t;

#ifndef WICED_DUKTAPE_PANDORA_MODULES
typedef struct wiced_duktape_scheduled_work_s
{
    duk_ret_t       (*func)( duk_context* ctx );
    void*           this;
    void*           arg;
} wiced_duktape_scheduled_work_t;
#endif

/******************************************************
 *               Static Function Declarations
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

#ifdef WICED_DUKTAPE_MODULE_WIFI
extern duk_ret_t wiced_duktape_module_wifi_register( duk_context* ctx );
#endif

#ifdef WICED_DUKTAPE_MODULE_TIME
extern duk_ret_t wiced_duktape_module_time_register( duk_context* ctx );
#endif

#ifdef WICED_DUKTAPE_OBJECT_AUDIO
extern void wiced_duktape_object_audio_init( duk_context* ctx );
#endif

#ifdef WICED_DUKTAPE_OBJECT_TIME
extern void wiced_duktape_object_time_init( duk_context* ctx );
#endif

#ifdef WICED_DUKTAPE_OBJECT_XMLHTTPREQUEST
extern void wiced_duktape_object_xmlhttprequest_init( duk_context* ctx );
#endif

/******************************************************
 *               Variable Definitions
 ******************************************************/

static duktape_resource_t resource_list[] = {
    WICED_DUKTAPE_RESOURCES
};

static wiced_duktape_c_module_t included_c_modules[] =
{
#ifdef WICED_DUKTAPE_MODULE_TIME
    { "time", wiced_duktape_module_time_register },
#endif
#ifdef WICED_DUKTAPE_MODULE_WIFI
    { "wifi", wiced_duktape_module_wifi_register },
#endif
#ifdef WICED_DUKTAPE_PANDORA_MODULES
    { "dpm", dukopen_dpm },
#endif
    { NULL, NULL }
};

static struct
{
    wiced_bool_t                initialized;
    wiced_filesystem_t          fs_handle;
    wiced_bool_t                fs_mounted;
    wiced_filesystem_t*         fs_ptr;
    duk_context*                ctx;
    wiced_semaphore_t           sem;
#ifdef WICED_DUKTAPE_MEMORY_POOL_ALLOC
    void*                       memory_pool_data;
#endif
    duktape_c_module_list_t*    c_module_list;
} duktape;

#ifndef WICED_DUKTAPE_PANDORA_MODULES
wiced_worker_thread_t   wiced_duktape_worker_thread;
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/


static wiced_result_t duktape_get_filesystem_handle( const char* filesystem_device )
{
    const filesystem_list_t*    fs = all_filesystem_devices;

    /* Check if filesystem is valid */
    while ( fs->device != NULL )
    {
        if ( strcmp( filesystem_device, fs->name ) == 0 )
        {
            LOGD( "Found filesystem device '%s'", fs->name );
            break;
        }

        fs++;
    }

    if ( fs->device == NULL )
    {
        LOGE( "Could not find filesystem device '%s'", filesystem_device );
        return WICED_ERROR;
    }

    /* Check if device is already mounted; if not, then mount it */
    LOGD( "Attemping to get filesystem handle" );
    duktape.fs_ptr = wiced_filesystem_retrieve_mounted_fs_handle( fs->name );
    if ( duktape.fs_ptr == NULL )
    {
        wiced_result_t result;

        LOGD( "Mounting filesystem device '%s'", fs->name );

        result = wiced_filesystem_mount( fs->device, fs->type,
                                         &duktape.fs_handle, fs->name );
        if ( result != WICED_SUCCESS )
        {
            LOGE( "Failed to mount filesystem device '%s' (result=%d)",
                  fs->name, result );
            return WICED_ERROR;
        }

        LOGI( "Mounted filesystem device '%s'", fs->name );

        duktape.fs_ptr = &duktape.fs_handle;
        duktape.fs_mounted = WICED_TRUE;
    }

    return WICED_SUCCESS;
}

static wiced_result_t duktape_copy_resource( duktape_resource_t* res )
{
    wiced_result_t  result = WICED_SUCCESS;
    wiced_file_t    file;
    uint32_t        pos = 0;
    uint32_t        bytes_avail = resource_get_size( res->resource );

    LOGI( "Copying resource file '%s' (%lu bytes)", res->filename, bytes_avail );

    /* Check for directories and create them */
    if ( strchr( res->filename, '/' ))
    {
        char*   path = strdup( res->filename );
        int     pos = 0;

        while ( strchr( &path[pos], '/' ))
        {
            char*   dir;
            char*   rest;

            rest = strstr( &path[pos], "/" );
            dir = strndup( path, strlen( path ) - strlen( rest ));
            LOGD( "dir=%s, rest=%s", dir, rest );

            result = wiced_filesystem_dir_create( duktape.fs_ptr, dir );
            free(dir);

            if ( result != WICED_SUCCESS )
            {
                LOGE( "Failed to create directory '%s'", dir );
                break;
            }

            pos += strlen(&path[pos]) - strlen(rest) + 1;
        }

        free( path );

        if ( result != WICED_SUCCESS )
        {
            return WICED_ERROR;
        }
    }

    /* Open file for writing */
    result = wiced_filesystem_file_open( duktape.fs_ptr, &file, res->filename,
                                         WICED_FILESYSTEM_OPEN_ZERO_LENGTH );
    if ( result != WICED_SUCCESS )
    {
        LOGE( "Failed to create file '%s'", res->filename );
        return WICED_ERROR;
    }

    while ( bytes_avail > 0 && result == WICED_SUCCESS )
    {
        resource_result_t   resource_result;
        uint32_t            bytes_r;
        uint64_t            bytes_w;
        const void*         data;

        resource_result = resource_get_readonly_buffer( res->resource, pos,
                                                        0x7fffffff, &bytes_r,
                                                        &data );
        if ( resource_result != RESOURCE_SUCCESS )
        {
            LOGE( "Failed to read from resource file '%s'", res->filename );
            result = WICED_ERROR;
            break;
        }

        LOGD( "Read %lu bytes from '%s'", bytes_r, res->filename );

        do
        {
            result = wiced_filesystem_file_write( &file, data, bytes_r,
                                                  &bytes_w );
            if ( result != WICED_SUCCESS )
            {
                LOGE( "Failed to write to file '%s'", res->filename );
                result = WICED_ERROR;
                break;
            }

            LOGD( "Wrote %lu bytes", (uint32_t)( bytes_w & 0xFFFFFFFF ));

            bytes_avail -= (uint32_t)bytes_w;
            bytes_r -= (uint32_t)bytes_w;
            pos += (uint32_t)bytes_w;
        } while ( bytes_r > 0 );

        resource_free_readonly_buffer( res->resource, data );
    }

    wiced_filesystem_file_close( &file );

    return result;
}

#ifdef WICED_DUKTAPE_FATAL_HANDLER
static void duktape_fatal_handler( void* udata, const char* msg )
{
    int countdown = 5;

    LOGE( "*** Encountered a FATAL Duktape error '%s' ***",
          msg ? msg : "no message" );

    LOGE( "*** Rebooting in " );
    while ( countdown > 0 )
    {
        LOGE( "%d", countdown );
        countdown--;
    }

    wiced_framework_reboot();
}
#endif

static duk_ret_t duktape_print( duk_context* ctx )
{
    duk_push_string( ctx, " " );
    duk_insert( ctx, 0 );
    duk_join( ctx, duk_get_top(ctx) - 1 );

    /* Note: if wiced_log is not enabled, then no prints come out... */
    wiced_log_msg( WLF_DUKTAPE, WICED_LOG_ERR, LOG_LABEL_FMT "%s\n",
                   "duk:print", duk_safe_to_string( ctx, -1 ));

    return 0;
}

#ifdef WICED_DUKTAPE_TESTS_API
static duk_ret_t duktape_printf( duk_context* ctx )
{
    duk_push_string( ctx, " " );
    duk_insert( ctx, 0 );
    duk_join( ctx, duk_get_top(ctx) - 1 );

    /* No fancy formatting- useful for Duktape tests */
    printf( "%s\n", duk_safe_to_string( ctx, -1 ));

    return 0;
}
#endif

static duk_ret_t duktape_module_resolve(duk_context *ctx)
{
    const char*                 requested_id;
    const char*                 parent_id;
    const char*                 resolved_id;
    duktape_c_module_list_t*    list = duktape.c_module_list;
    wiced_bool_t                is_nodejs_module = WICED_FALSE;

    requested_id = duk_require_string( ctx, 0 );
    parent_id = duk_require_string( ctx, 1 );

    LOGD( "Resolving module (requested_id='%s' parent_id='%s')", requested_id,
          parent_id );

    /* Check if module matches a C module first */
    while ( list != NULL )
    {
        if ( strlen( list->c_module.name ) == strlen( requested_id ) &&
             strcmp( list->c_module.name, requested_id ) == 0 )
        {
            LOGD( "Found matching C module '%s'", requested_id );

            /* Keep the module name unchanged */
            duk_dup( ctx, 0 );

            goto out;
        }

        list = list->next;
    }

    /* Not a C module, so assume an Ecmascript module; resolve to filename */
    resolved_id = duk_push_sprintf( ctx, "%s", requested_id );

    /* If the requested ID does not have the ".js" extension, then we assume it
     * is a node.js-style module
     */
    if ( strlen( resolved_id ) < 4 ||
         strcmp( &resolved_id[strlen( resolved_id ) - 3], ".js" ) != 0 )
    {
        LOGD( "'%s' is a node.js module", resolved_id );
        is_nodejs_module = WICED_TRUE;
    }

    /* Replace relative paths with full paths in order to make sure all modules
     * have a unique module ID and are not loaded twice
     */
    if ( strstr( resolved_id, "./" ) != NULL ||
         strstr( resolved_id, "../" ) != NULL )
    {
        int         up_dir = 0;
        const char* path;

        /* We need a valid parent ID to resolve relative paths */
        if ( strstr( parent_id, ".js" ) == NULL )
        {
            /* Pop 'resolved_id' off stack */
            duk_pop( ctx );

            LOGE( "Cannot resolve Ecmascript module '%s' with relative path",
                  requested_id );
            duk_error( ctx, DUK_ERR_TYPE_ERROR,
                       "Cannot resolve Ecmascript module '%s' with relative path",
                       requested_id );
            return 1;
        }

        /* If module is a node.js-style module and it has a relative path, then
         * the requested ID is referring to an Ecmascript, so add the ".js"
         * extension
         */
        if ( is_nodejs_module )
        {
            resolved_id = duk_push_sprintf( ctx, "%s.js", resolved_id );

            /* Remove old 'resolved_id' off stack */
            duk_remove( ctx, -2 );
        }

        /* Strip out all preceeding "./" and "../" */
        while ( strstr( resolved_id, "./" ) != NULL ||
                strstr( resolved_id, "../" ) != NULL )
        {
            LOGD( "resolved_id=%s", resolved_id );

            if ( strncmp( resolved_id, "./", 2 ) == 0 )
            {
                LOGD( "Removing './' from '%s'", resolved_id );
                resolved_id = duk_push_sprintf( ctx, "%s", &resolved_id[2] );

                /* Remove old 'resolved_id' off stack */
                duk_remove( ctx, -2 );
            }
            else if ( strncmp( resolved_id, "../", 3 ) == 0 )
            {
                up_dir++;

                LOGD( "Removing '../' from '%s'", resolved_id );
                resolved_id = duk_push_sprintf( ctx, "%s", &resolved_id[3] );

                /* Remove old 'resolved_id' off stack */
                duk_remove( ctx, -2 );
            }
            else
            {
                /* Pop 'resolved_id' off stack */
                duk_pop( ctx );

                /* The "./" or "../" relative paths are not at the front! */
                LOGE( "Unable to resolve path of Ecmascript module '%s'",
                      requested_id );
                duk_error( ctx, DUK_ERR_TYPE_ERROR,
                           "Unable to resolve path of Ecmascript module '%s'",
                          requested_id );
                return 1;
            }
        }

        LOGD( "up_dir=%d resolved_id=%s", up_dir, resolved_id );

        /* Use the parent_id as basis for the first part of the path */
        path = duk_push_sprintf( ctx, "%.*s",
                                 strrchr( parent_id, '/' ) - parent_id,
                                 parent_id );

        LOGD( "path=%s", path );

        while ( up_dir-- > 0 )
        {
            if ( strrchr( path, '/' ) == NULL )
            {
                /* Pop 'resolved_id' and 'path' off stack */
                duk_pop_2( ctx );

                LOGE( "Unable to resolve path of Ecmascript module '%s'",
                      requested_id );
                duk_error( ctx, DUK_ERR_TYPE_ERROR,
                           "Unable to resolve path of Ecmascript module '%s'",
                          requested_id );
                return 1;
            }

            path = duk_push_sprintf( ctx, "%.*s", strrchr( path, '/' ) - path,
                                     path );
            duk_remove( ctx, -2 );

            LOGD( "path=%s", path );
        }

        /* Concatenate the first part of the path with the resolved ID */
        resolved_id = duk_push_sprintf( ctx, "%s/%s", path, resolved_id );

        /* Remove 'path' off stack */
        duk_remove( ctx, -2 );

        /* Remove old 'resolved_id' off stack */
        duk_remove( ctx, -2 );
    }
    else if ( is_nodejs_module )
    {
        /* No relative paths, so it lives under the modules directory. The
         * name is either the name of the Ecmascript without the .js extension,
         * or of a directory of that name with the actual Ecmascript as index.js
         * inside that directory. For now, we only support the former case.
         */
        /* TODO Support modules that take the format <module>/index.js */
        resolved_id = duk_push_sprintf( ctx, "/modules/%s.js", resolved_id );

        /* Remove old 'resolved_id' off stack */
        duk_remove( ctx, -2 );
    }

    /* Prepend '/' if needed */
    if ( resolved_id[0] != '/' )
    {
        duk_push_sprintf( ctx, "/%s", duk_get_string( ctx, -1 ));

        /* Remove old 'resolved_id' off stack */
        duk_remove( ctx, -2 );
    }

out:
    LOGD( "Resolved module '%s' -> '%s'", requested_id,
          duk_get_string( ctx, -1 ));

    /* Return the resolved ID on the stack */
    return 1;
}

static duk_ret_t duktape_module_load(duk_context *ctx)
{
    const char*                 filename;
    size_t                      filename_len;
    const char*                 module_id;
    wiced_file_t                file_handle;
    wiced_dir_entry_details_t   file_details;
    char*                       buffer;
    uint64_t                    bytes = 0;
    uint32_t                    pos = 0;

    module_id = duk_require_string( ctx, 0 );

    duk_get_prop_string( ctx, 2, "filename" );
    filename = duk_require_lstring( ctx, -1, &filename_len );

    LOGD( "Loading module (module_id='%s' filename='%s')", module_id,
          filename );

    /* C modules do not end in '.js' */
    if ( strstr( filename, ".js" ) == NULL )
    {
        duktape_c_module_list_t*    list = duktape.c_module_list;

        LOGD( "Loading C module '%s'", module_id );

        while ( list != NULL )
        {
            if ( strlen( list->c_module.name ) == strlen( module_id ) &&
                 strcmp( list->c_module.name, module_id ) == 0 )
            {
                /* Keep the module name unchanged */
                duk_push_c_function( ctx, list->c_module.module_reg, 0 );
                duk_call( ctx, 0 );

                /* Overwrite module.exports */
                duk_put_prop_string( ctx, 2, "exports" );

                /* Return undefined, no Ecmascript source code */
                return 0;
            }

            list = list->next;
        }

        LOGE( "Failed to find C module '%s'", module_id );

        duk_error( ctx, DUK_ERR_TYPE_ERROR, "Failed to find C module: '%s'",
                   module_id );
        goto out;
    }

    LOGD( "Loading Ecmascript module '%s'", module_id );

    if ( duktape.fs_ptr == NULL )
    {
        LOGE( "Filesystem not mounted- cannot load Ecmascript modules '%s'",
              module_id );

        duk_error( ctx, DUK_ERR_TYPE_ERROR,
                   "Filesystem not mounted- cannot load Ecmascript module '%s'",
                   module_id );
        return 1;
    }

    LOGD( "Opening file '%s' for reading", filename );
    if ( wiced_filesystem_file_open( duktape.fs_ptr, &file_handle, filename,
                                     WICED_FILESYSTEM_OPEN_FOR_READ ) !=
         WICED_SUCCESS )
    {
        LOGE( "Failed to open module file '%s'", filename );

        duk_error( ctx, DUK_ERR_TYPE_ERROR, "Failed to open module file '%s'",
                   filename );
        return 1;
    }

    LOGD( "Getting file details of module file '%s'", filename );
    if ( wiced_filesystem_file_get_details( duktape.fs_ptr, filename,
                                            &file_details ) != WICED_SUCCESS )
    {
        LOGE( "Failed to get details of module file '%s'", filename );

        duk_error( ctx, DUK_ERR_TYPE_ERROR,
                   "Failed to get details of module file '%s'", filename );
        goto close_file;
    }

    LOGD( "Module file '%s' is %lu bytes in size", filename,
          (uint32_t)file_details.size );

    /* Allocate a new buffer on the stack for the module */
    buffer = (char*)duk_push_fixed_buffer( ctx, (uint32_t)file_details.size );
    if ( buffer == NULL )
    {
        LOGE( "Failed to allocate memory for module '%s'", module_id );
        duk_error( ctx, DUK_ERR_TYPE_ERROR,
                   "Failed to allocate memory for module '%s'", module_id );
        goto close_file;
    }

    do
    {
        if ( wiced_filesystem_file_read( &file_handle, &buffer[pos],
                                         (uint32_t)file_details.size - pos,
                                         &bytes ) != WICED_SUCCESS )
        {
            /* Pop 'buffer' off stack */
            duk_pop( ctx );

            LOGE( "Failed to read from module file '%s'", filename );
            duk_error( ctx, DUK_ERR_TYPE_ERROR,
                       "Failed to read from module file '%s'", filename );
            goto close_file;
        }

        LOGD( "Read %lu bytes from '%s'", (uint32_t)bytes, filename );

        pos += (uint32_t)bytes;
    } while ( pos < (uint32_t)file_details.size );

    LOGD( "Module to load:\n%s", buffer );

    /* Convert 'buffer' contents to string */
    duk_buffer_to_string( ctx, -1 );

close_file:
    wiced_filesystem_file_close( &file_handle );

out:
    return 1;
}

#if !defined(WICED_DUKTAPE_MEMORY_POOL_ALLOC) || defined(WICED_DUKTAPE_TESTS_API)
static void *duktape_alloc( void* udata, duk_size_t size )
{
    return malloc( size );
}

static void duktape_free( void *udata, void* ptr)
{
    free( ptr );
}

static void *duktape_realloc( void *udata, void *ptr, duk_size_t size )
{
    /* Special case: ptr is non-NULL and size is 0 -> equivalent to free */
    if ( ptr != NULL && size == 0 )
    {
        duktape_free( udata, ptr );
        return NULL;
    }

    return realloc( ptr, size );
}
#endif

static wiced_result_t duktape_create_heap( void )
{
    duk_alloc_function      alloc_func = NULL;
    duk_realloc_function    realloc_func = NULL;
    duk_free_function       free_func = NULL;
    duk_fatal_function      fatal_func = NULL;
    void*                   udata = NULL;

    LOGD( "Creating duktape heap" );

#ifdef WICED_DUKTAPE_MEMORY_POOL_ALLOC
    /* Use Duktape's example 'hybrid' memory allocators, which uses memory pools
     * to cut down on memory fragmentation
     */
    duktape.memory_pool_data = duk_alloc_hybrid_init();

    alloc_func   = duk_alloc_hybrid;
    realloc_func = duk_realloc_hybrid;
    free_func    = duk_free_hybrid;
    udata        = duktape.memory_pool_data;
#else
    /* Use our super simple wrapper functions */
    alloc_func   = duktape_alloc;
    realloc_func = duktape_realloc;
    free_func    = duktape_free;
#endif

#ifdef WICED_DUKTAPE_FATAL_HANDLER
    fatal_func = duktape_fatal_handler;
#endif

    duktape.ctx = duk_create_heap( alloc_func, realloc_func, free_func,
                                   udata, fatal_func );
    if ( duktape.ctx == NULL )
    {
        LOGE( "Failed to create duktape heap" );

#ifdef WICED_DUKTAPE_MEMORY_POOL_ALLOC
        if ( duktape.memory_pool_data != NULL )
        {
            LOGD( "Freeing memory pool data" );
            duk_alloc_hybrid_deinit( &duktape.memory_pool_data );
        }
#endif
        return WICED_ERROR;
    }
    LOGD( "Duktape heap created!" );

    /* Provide a print function */
    duk_push_global_object( duktape.ctx ); /* -> [global] */
    duk_push_string( duktape.ctx, "print" ); /* -> [global print] */
    duk_push_c_function( duktape.ctx, duktape_print, DUK_VARARGS );
    /* -> [global print func] */
    duk_def_prop( duktape.ctx, -3,
                  DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_SET_WRITABLE |
                  DUK_DEFPROP_SET_CONFIGURABLE ); /* -> [global] */
    duk_pop( duktape.ctx ); /* -> [] */

    /* Register functions to handle module resolving and loading */
    duk_push_object( duktape.ctx );
    duk_push_c_function( duktape.ctx, duktape_module_resolve, DUK_VARARGS );
    duk_put_prop_string( duktape.ctx, -2, "resolve" );
    duk_push_c_function( duktape.ctx, duktape_module_load, DUK_VARARGS );
    duk_put_prop_string( duktape.ctx, -2, "load" );
    duk_module_node_init( duktape.ctx );

#ifdef WICED_DUKTAPE_LOGGER
    duk_logging_init( duktape.ctx, 0 );
#ifndef WICED_DUKTAPE_PANDORA_MODULES
    duk_eval_string( duktape.ctx, "var logger = new Duktape.Logger(null)" );
#endif
#endif

    /* Initialize any objects */
#ifdef WICED_DUKTAPE_OBJECT_AUDIO
    wiced_duktape_object_audio_init( duktape.ctx );
#endif
#ifdef WICED_DUKTAPE_OBJECT_TIME
    wiced_duktape_object_time_init( duktape.ctx );
#endif
#ifdef WICED_DUKTAPE_OBJECT_XMLHTTPREQUEST
    wiced_duktape_object_xmlhttprequest_init( duktape.ctx );
#endif

    return WICED_SUCCESS;
}

#ifndef WICED_DUKTAPE_PANDORA_MODULES
static wiced_result_t duktape_worker_thread( void* arg )
{
    wiced_result_t                  result = WICED_SUCCESS;
    wiced_duktape_scheduled_work_t* work;
    duk_context*                    ctx;
    wiced_duktape_state_t           state;

    wiced_assert( "Bad args", arg != NULL );

    work = (wiced_duktape_scheduled_work_t*)arg;

    /* Get control of Duktape */
    ctx = wiced_duktape_get_control( &state );
    if ( ctx == NULL )
    {
        LOGW( "Duktape heap was destroyed" );
        return WICED_SUCCESS;
    }

    duk_push_c_function( ctx, work->func, work->arg == NULL ? 0 : 1 );
    /* -> [func] */
    duk_push_heapptr( ctx, work->this ); /* -> [func this] */

    if ( work->arg != NULL )
    {
        duk_push_pointer( ctx, work->arg ); /* -> [func this arg] */
    }

    if ( duk_pcall_method( ctx, work->arg == NULL ? 0 : 1 ) == DUK_EXEC_ERROR )
    {
        /* -> [err] */
        LOGE( "Failed to call scheduled work (%s)",
              duk_safe_to_string( ctx, -1 ));
        result = WICED_ERROR;
    }
    /* -> [retval/err] */

    duk_pop( ctx ); /* -> [] */

    /* Release control of Duktape */
    wiced_duktape_put_control( &state );

    free( work );

    return result;
}
#endif

/** Returns current (UTC) time
 *
 * @notes   Used by Duktape engine only
 *
 * @param[in] ctx   : Duktape context (unused)
 *
 * @return  WICED_SUCCESS on success, else WICED_ERROR
 */

duk_double_t wiced_duktape_now( duk_context* ctx )
{
    wiced_utc_time_ms_t utc;
    duk_double_t        dt;

    UNUSED_PARAMETER( ctx );

    wiced_time_get_utc_time_ms( &utc );

    dt = (duk_double_t) utc;

    return dt;
}

/** Initialize Duktape and get a handle to the filesystem
 *
 * @param[in] filesystem_device : Optional filesystem device to use; needed for
 *                                evaluating files directly from filesystem
 *
 * @return    WICED_SUCCESS on success, else WICED_ERROR
 */

wiced_result_t wiced_duktape_init( const char* filesystem_device )
{
    wiced_duktape_c_module_t*   c_module = included_c_modules;

    if ( duktape.initialized == WICED_TRUE )
    {
        LOGD( "Already initialized" );
        return WICED_SUCCESS;
    }

    if ( wiced_rtos_init_semaphore( &duktape.sem ) != WICED_SUCCESS )
    {
        LOGE( "Failed to initialize Duktape control semaphore" );
        return WICED_ERROR;
    }

#ifndef WICED_DUKTAPE_PANDORA_MODULES
    /* Create a worker thread */
    if ( wiced_rtos_create_worker_thread( &wiced_duktape_worker_thread,
                                          WICED_DEFAULT_WORKER_PRIORITY,
                                          4*1024, 10 ) != WICED_SUCCESS )
    {
        LOGE( "Failed to create worker thread" );
        return WICED_ERROR;
    }
#endif

    if ( filesystem_device != NULL )
    {
        /* Get a filesystem handle */
        if ( duktape_get_filesystem_handle( filesystem_device ) !=
             WICED_SUCCESS )
        {
            LOGE( "Failed to get handle for filesystem device '%s'",
                  filesystem_device );
            return WICED_ERROR;
        }
    }

    wiced_rtos_set_semaphore( &duktape.sem );

    duktape.initialized = WICED_TRUE;

    /* Load up the C modules list with internal modules */
    while ( c_module->name != NULL )
    {
        wiced_duktape_add_c_module( c_module );
        c_module++;
    }

    return WICED_SUCCESS;
}


/** Change filesystem device that Duktape uses
 *
 * @notes   Filesystem device can only be changed if all Duktape evaluations
 *          have been stopped; resources must be reloaded by user after the
 *          switch
 *
 * #param[in] filesystem_device : Filesystem device to use; use NULL to not
 *                                use any
 *
 * @return  WICED_SUCCESS on success, else WICED_ERROR
 */

wiced_result_t wiced_duktape_change_filesystem( const char* filesystem_device )
{
    if ( duktape.initialized != WICED_TRUE )
    {
        LOGD( "Duktape not initialized" );
        return WICED_ERROR;
    }

    if ( wiced_rtos_get_semaphore( &duktape.sem,
                                   WICED_WAIT_FOREVER ) != WICED_SUCCESS )
    {
        LOGE( "Failed to get semaphore" );
        return WICED_ERROR;
    }

    if ( duktape.ctx != NULL )
    {
        LOGE( "Cannot change filesystem while Duktape is evaluating" );
        wiced_rtos_set_semaphore( &duktape.sem );
        return WICED_ERROR;
    }

    /* Unmount the filesystem if it was mounted by us */
    if ( duktape.fs_mounted == WICED_TRUE )
    {
        wiced_result_t result;

        result = wiced_filesystem_unmount( &duktape.fs_handle );
        if ( result != WICED_SUCCESS )
        {
            LOGE( "Failed to unmount filesystem device (result=%d)", result );
            wiced_rtos_set_semaphore( &duktape.sem );
            return WICED_ERROR;
        }
    }

    /* Reset the filesystem variables */
    duktape.fs_ptr = NULL;
    duktape.fs_mounted = WICED_FALSE;

    if ( filesystem_device != NULL )
    {
        /* Get a filesystem handle */
        if ( duktape_get_filesystem_handle( filesystem_device ) !=
             WICED_SUCCESS )
        {
            LOGE( "Failed to get handle for filesystem device '%s'",
                  filesystem_device );
            return WICED_ERROR;
        }
    }

    wiced_rtos_set_semaphore( &duktape.sem );

    return WICED_SUCCESS;
}

/** Add C module to Duktape
 *
 * @param[in] c_module  : C module structure
 *
 * @return  WICED_SUCCESS on success, else WICED_ERROR
 */

wiced_result_t wiced_duktape_add_c_module( wiced_duktape_c_module_t* c_module )
{
    duktape_c_module_list_t* entry;

    if (( c_module == NULL ) || ( c_module->name == NULL ) ||
        ( c_module->module_reg == NULL ))
    {
        return WICED_ERROR;
    }

    LOGD( "Adding C module '%s'", c_module->name );
    LOGD( "Allocating %u bytes for C module list structure",
          sizeof( duktape_c_module_list_t ));

    entry = malloc( sizeof( duktape_c_module_list_t ));
    if ( entry == NULL )
    {
        LOGE( "Failed to allocate memory for C module structure" );
        return WICED_ERROR;
    }

    memcpy( &entry->c_module, c_module, sizeof( wiced_duktape_c_module_t ));

    entry->next = duktape.c_module_list;
    duktape.c_module_list = entry;

    LOGI( "Added C module '%s' to list of C modules", entry->c_module.name );

    return WICED_SUCCESS;
}

/** Load Duktape resources to the filesystem
 *
 * @return  WICED_SUCCESS on success, else WICED_ERROR
 */

wiced_result_t wiced_duktape_load_resources( void )
{
    int i;

    if ( duktape.initialized != WICED_TRUE )
    {
        LOGD( "Duktape not initialized" );
        return WICED_ERROR;
    }

    if ( wiced_rtos_get_semaphore( &duktape.sem,
                                   WICED_WAIT_FOREVER ) != WICED_SUCCESS )
    {
        LOGE( "Failed to get semaphore" );
        return WICED_ERROR;
    }

    if ( duktape.fs_ptr == NULL )
    {
        LOGE( "No filesystem loaded" );
        wiced_rtos_set_semaphore( &duktape.sem );
        return WICED_ERROR;
    }

    LOGD("Copying resource files to filesystem");

    for ( i = 0; i < sizeof( resource_list ) / sizeof( resource_list[0] ); i++ )
    {
        LOGD( "Copying '%s' to filesystem", resource_list[i].filename );
        duktape_copy_resource( &resource_list[i] );
    }

    LOGD("Done copying resource files to filesystem");

    wiced_rtos_set_semaphore( &duktape.sem );

    return WICED_SUCCESS;
}

/** Evaluate a script in buffer with Duktape
 *
 * @param[in] buffer    : NULL-terminated buffer containing script
 *
 * @return  WICED_SUCCESS on success, else WICED_ERROR
 */

wiced_result_t wiced_duktape_eval_buffer( const char* buffer )
{
    wiced_result_t  result = WICED_SUCCESS;

    if ( buffer == NULL )
    {
        LOGE( "Missing buffer" );
        return WICED_ERROR;
    }

    if ( duktape.initialized != WICED_TRUE )
    {
        LOGE( "Duktape not initialized" );
        return WICED_ERROR;
    }

    if ( wiced_rtos_get_semaphore( &duktape.sem,
                                   WICED_WAIT_FOREVER ) != WICED_SUCCESS )
    {
        LOGE( "Failed to get semaphore" );
        return WICED_ERROR;
    }

    if ( duktape.ctx == NULL )
    {
        if ( duktape_create_heap() != WICED_SUCCESS )
        {
            wiced_rtos_set_semaphore( &duktape.sem );
            return WICED_ERROR;
        }
    }

    LOGD( "Buffer to evaluate:\n%s", buffer );

    if ( duk_peval_string( duktape.ctx, buffer ) != 0 )
    {
        LOGE( "Duktape eval failed (%s)",
              duk_safe_to_string( duktape.ctx, -1 ));
        result = WICED_ERROR;
    }
    duk_pop( duktape.ctx );

    wiced_rtos_set_semaphore( &duktape.sem );

    return result;
}

/** Evaluate a script file with Duktape
 *
 * @param[in] filename  : Full path of script file
 *
 * @return  WICED_SUCCESS on success, else WICED_ERROR
 */

wiced_result_t wiced_duktape_eval_file( const char* filename )
{
    wiced_result_t              result;
    wiced_file_t                file_handle;
    wiced_dir_entry_details_t   file_details;
    char*                       buffer;
    uint32_t                    buffer_size;
    uint64_t                    bytes = 0;
    uint32_t                    pos = 0;

    if ( duktape.initialized != WICED_TRUE )
    {
        LOGE( "Duktape not initialized" );
        return WICED_ERROR;
    }

    if ( wiced_rtos_get_semaphore( &duktape.sem,
                                   WICED_WAIT_FOREVER ) != WICED_SUCCESS )
    {
        LOGE( "Failed to get semaphore" );
        return WICED_ERROR;
    }

    if ( duktape.fs_ptr == NULL )
    {
        LOGE( "Filesystem not configured" );
        result = WICED_ERROR;
        goto out;
    }

    LOGD( "Opening file for read" );
    result = wiced_filesystem_file_open( duktape.fs_ptr, &file_handle, filename,
                                         WICED_FILESYSTEM_OPEN_FOR_READ );
    if ( result != WICED_SUCCESS )
    {
        LOGE( "Failed to open file '%s'", filename );
        result = WICED_ERROR;
        goto out;
    }

    LOGD( "Getting file details" );
    result = wiced_filesystem_file_get_details( duktape.fs_ptr, filename,
                                                &file_details );
    if ( result != WICED_SUCCESS )
    {
        LOGE( "Failed to get details of '%s'", filename );
        result = WICED_ERROR;
        goto out;
    }

    /* A bug prevents 64-bit integers from getting printed, so just print
     * 32 bits worth of size
     */
    LOGI( "File '%s' is %lu bytes in size", filename,
          (uint32_t)file_details.size );

    /* We need at least 1 byte at the very end for '\0' so Duktape can use it to
     * end of buffer. Just to make things nice, make the buffer size a multiple
     * of 4-bytes, i.e., in the worst case scenario, we allocate 4 more bytes
     * than needed.
     */
    buffer_size = ((uint32_t)file_details.size + 4 ) & ~0x3;

    buffer = (char*)calloc( 1, buffer_size );
    if ( buffer == NULL )
    {
        LOGE( "Failed to allocate memory for read buffer" );
        result = WICED_ERROR;
        goto close_file;
    }

    LOGD( "Allocated temporary buffer @ %p (%lu bytes)", buffer, buffer_size );

    do
    {
        result = wiced_filesystem_file_read( &file_handle, &buffer[pos],
                                             (uint32_t)file_details.size - pos,
                                             &bytes );
        if ( result != WICED_SUCCESS )
        {
            LOGE( "Failed to read from file '%s'", filename );
            result = WICED_ERROR;
            goto free_buffer;
        }

        LOGD( "Read %lu bytes from '%s'", (uint32_t)bytes, filename );

        pos += (uint32_t)bytes;
    } while ( pos < (uint32_t)file_details.size );


    if ( duktape.ctx == NULL )
    {
        /* Create the Duktape heap */
        if ( duktape_create_heap() != WICED_SUCCESS )
        {
            result = WICED_ERROR;
            goto free_buffer;
        }
    }

    LOGD( "Buffer to evaluate:\n%s", buffer );

    if ( duk_pcompile_string( duktape.ctx, DUK_COMPILE_EVAL, buffer ) != 0 )
    {
        LOGE( "Duktape compile failed (%s)",
              duk_safe_to_string( duktape.ctx, -1 ));

        duk_pop( duktape.ctx );
        result = WICED_ERROR;
        goto free_buffer;
    }
    else
    {
        /* Free the buffer to save some memory */
        free( buffer );
        buffer = NULL;

        if ( duk_pcall( duktape.ctx, 0 ) != 0 )
        {
            LOGE( "Duktape call failed (%s)",
                  duk_safe_to_string( duktape.ctx, -1 ));

            duk_pop( duktape.ctx );
            result = WICED_ERROR;
        }
        duk_pop( duktape.ctx );
    }

    result = WICED_SUCCESS;

free_buffer:
    free( buffer );

close_file:
    wiced_filesystem_file_close( &file_handle );

out:
    wiced_rtos_set_semaphore( &duktape.sem );

    return result;
}

/** Stop the running instance of Duktape and destroy the heap
 *
 * @return    WICED_SUCCESS on success, else WICED_ERROR
 */

wiced_result_t wiced_duktape_stop_eval( void )
{
    if ( duktape.initialized != WICED_TRUE )
    {
        LOGE( "Duktape not initialized" );
        return WICED_ERROR;
    }

    if ( wiced_rtos_get_semaphore( &duktape.sem,
                                   WICED_WAIT_FOREVER ) != WICED_SUCCESS )
    {
        LOGE( "Failed to get semaphore" );
        return WICED_ERROR;
    }

    if ( duktape.ctx != NULL )
    {
        LOGD( "Destroying duktape heap" );
        duk_destroy_heap( duktape.ctx );
        duktape.ctx = NULL;

#ifdef WICED_DUKTAPE_MEMORY_POOL_ALLOC
        if ( duktape.memory_pool_data != NULL )
        {
            LOGD( "Freeing memory pool data" );
            duk_alloc_hybrid_deinit( &duktape.memory_pool_data );
        }
#endif
    }

    wiced_rtos_set_semaphore( &duktape.sem );

    return WICED_SUCCESS;
}

#ifndef WICED_DUKTAPE_PANDORA_MODULES

/** Get control of the main Duktape thread
 *
 * @param[in] state : Variable to save the state of the Duktape thread
 *
 * @return  WICED_SUCCESS on success, else WICED_ERROR
 */

duk_context* wiced_duktape_get_control( wiced_duktape_state_t* state )
{
    wiced_assert( "Bad args", state != NULL );

    if ( duktape.initialized != WICED_TRUE )
    {
        LOGE( "Duktape not initialized" );
        return NULL;
    }

    LOGD( "Getting control semaphore" );
    if ( wiced_rtos_get_semaphore( &duktape.sem,
                                   WICED_WAIT_FOREVER ) != WICED_SUCCESS )
    {
        LOGE( "Failed to get semaphore" );
        return NULL;
    }

    if ( duktape.ctx == NULL )
    {
        LOGE( "Duktape is not running!" );
        wiced_rtos_set_semaphore( &duktape.sem );
        return NULL;
    }

    /* Suspend the current call stack */
    duk_suspend( duktape.ctx, state );

    return duktape.ctx;
}

/** Release control of the main Duktape thread
 *
 * @param[in] state : Variable containing the saved state of the Duktape thread
 *
 * @return  None
 */

void wiced_duktape_put_control( wiced_duktape_state_t* state )
{
    wiced_assert( "Bad args", state != NULL );

    if ( duktape.initialized != WICED_TRUE )
    {
        LOGE( "Duktape not initialized" );
        return;
    }

    /* Resume the previous call stack */
    duk_resume( duktape.ctx, state );

    LOGD( "Setting control semaphore" );
    wiced_rtos_set_semaphore( &duktape.sem );
}

/** Schedule a Duktape/C function to run from a worker thread
 *
 * @param[in] this  : 'this' pointer; may be NULL only if calling from the main
 *                    Duktape thread
 * @param[in] func  : Duktape/C function to run
 * @param[in] arg   : Optional argument to pass to Duktape/C function
 *
 * @return  WICED_SUCCESS on success, else WICED_ERROR
 */
wiced_result_t wiced_duktape_schedule_work( void* this, duk_ret_t (*func)( duk_context* ctx ), void* arg )
{
    wiced_duktape_scheduled_work_t* work;

    wiced_assert( "Bad args", func != NULL );

    work = calloc( 1, sizeof( wiced_duktape_scheduled_work_t ));
    if ( work == NULL )
    {
        LOGE( "Failed to allocate memory for scheduled work" );
        return WICED_ERROR;
    }

    work->func = func;
    work->arg = arg;
    if ( this != NULL )
    {
        work->this = this;
    }
    else
    {
        duk_push_this( duktape.ctx ); /* -> [this] */
        work->this = duk_get_heapptr( duktape.ctx, -1 );
        duk_pop( duktape.ctx ); /* -> [] */
    }
    LOGD( "Scheduling work (work->func=%p, work->this=%p, work->arg=%p)",
          work->func, work->this, work->arg );
    if ( wiced_rtos_send_asynchronous_event( WICED_DUKTAPE_WORKER_THREAD,
                                             duktape_worker_thread,
                                             work ) != WICED_SUCCESS )
    {
        free( work );

        LOGE( "Failed to schedule work" );
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

/** Run one or all of the Duktape API tests
 *
 * @param[in] test  : Optional name of single test to run
 *
 * @return  WICED_SUCCESS on success, else WICED_ERROR
 */

wiced_result_t wiced_duktape_run_api_tests( const char* test )
{
#ifdef WICED_DUKTAPE_TESTS_API
    duk_context*    ctx;
    int             i;

    if ( duktape.initialized != WICED_TRUE )
    {
        LOGE( "Duktape not initialized" );
        return WICED_ERROR;
    }

    if ( wiced_rtos_get_semaphore( &duktape.sem,
                                   WICED_WAIT_FOREVER ) != WICED_SUCCESS )
    {
        LOGE( "Failed to get semaphore" );
        return WICED_ERROR;
    }

    if ( duktape.ctx != NULL )
    {
        LOGE( "Cannot run API tests while Duktape is already running" );
        wiced_rtos_set_semaphore( &duktape.sem );
        return WICED_ERROR;
    }

    for ( i = 0;; i++ )
    {
        if ( wiced_duktape_tests_api_list[i].name == NULL )
        {
            break;
        }

        if ( test != NULL &&
             strcmp( test, wiced_duktape_tests_api_list[i].name ) != 0 )
        {
            continue;
        }

        /* Create the Duktape heap */
        ctx = duk_create_heap( duktape_alloc, duktape_realloc, duktape_free,
                               NULL, NULL );
        if ( ctx == NULL )
        {
            LOGE( "Failed to create duktape heap" );

            return WICED_ERROR;
        }

        /* Provide a print function */
        duk_push_global_object( ctx );
        duk_push_string( ctx, "print" );
        duk_push_c_function( ctx, duktape_printf, DUK_VARARGS );
        duk_def_prop( ctx, -3,
                      DUK_DEFPROP_HAVE_VALUE | DUK_DEFPROP_SET_WRITABLE |
                      DUK_DEFPROP_SET_CONFIGURABLE );
        duk_pop( ctx );

        printf( "+++ %s\n", wiced_duktape_tests_api_list[i].name );
        wiced_duktape_tests_api_list[i].func( ctx );

        duk_destroy_heap( ctx );
        ctx = NULL;
    }

    wiced_rtos_set_semaphore( &duktape.sem );

    return WICED_SUCCESS;
#else
    LOGE( "Duktape API tests are not compiled" );
    return WICED_ERROR;
#endif /* WICED_DUKTAPE_TESTS_API */
}
#endif
