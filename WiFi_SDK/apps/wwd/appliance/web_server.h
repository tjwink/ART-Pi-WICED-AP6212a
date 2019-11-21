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

/**
 * @file
 * Provides an interface for the content handler list which
 * defines the handlers that provide web page HTML data
 *
 */

#ifndef WEB_SERVER_H_INCLUDED
#define WEB_SERVER_H_INCLUDED

#include "wwd_constants.h"
#include "wwd_debug.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                     Macros
 ******************************************************/
#ifndef WEB_SERVER_NO_PRINT
#define WEB_SERVER_ERROR_PRINT( x )  WINFO_APP_ERROR( x )
#define WEB_SERVER_PRINT( x )        WINFO_APP( x )
#define WEB_SERVER_STACK_SIZE        (4 * 1024)
#else
#define WEB_SERVER_ERROR_PRINT( x )  wiced_assert( "", 0!=0 )
#define WEB_SERVER_PRINT( x )
#define WEB_SERVER_STACK_SIZE        (1024)
#endif

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/
/**
 * Prototype for URL handler functions which serve a web page data
 *
 * @param  socket     : the socket to which the data will be sent
 * @param  params     : a byte array containing any parameters included in the URL
 * @param  params_len : size of the params byte array in bytes
 *
 * @return use return to indicate success=0, error<0 or end-server=1
 */
typedef int (*url_processor_t)( void * socket, char * params, int params_len );


/******************************************************
 *                    Structures
 ******************************************************/

/**
 * Structure of one element of the URL handler list
 * Simply maps a URL path to a handler function
 */
typedef struct
{
    const char* const     url;
    const char* const     mime_type;
    const url_processor_t processor;
} url_list_elem_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/**
 * Prototype for sending web page data from within URL handler
 *
 * @param  socket : the socket to which the data will be sent
 * @param  data   : a pointer to the data to be transmitted
 * @param  length : length in bytes of data buffer
 */

void send_web_data( void * socket, unsigned char * data, unsigned long length );

/**
 * Processes web server requests
 *
 * Finds the path and query parts of a request,
 * Allocates a packet for the response
 * Checks whether there is a path matching the request
 * If there is, a success HTTP header is added, and the handler function for that path is called to fill the content.
 * If there is not, the 404 HTTP header is written to the response packet
 *
 * @param server_url_list : the content list from which to serve pages
 * @param url             : character array containing the request string - NOT null terminated
 * @param url_len         : length in bytes of url character array
 * @param socket          : pointer to the socket to which the response will be sent
 * @param peer_ip_address : the IPv4 address of the connected peer (for debug printing)
 *
 * @return use return to indicate success=0, error<0 or end-server=1
 */

int process_url_request( const url_list_elem_t * server_url_list, char * url, int url_len, void* socket, unsigned long peer_ip_address );


/**
 * Runs the web server
 *
 * Opens port 80 and serves web pages from the provided content list
 *
 * @param bind_address_in    : IP address on which the web server will run
 * @param server_url_list_in : the content list of web pages
 */

void run_webserver( uint32_t bind_address_in, const url_list_elem_t * server_url_list_in );



void start_web_server_thread( uint32_t bind_address_in, const url_list_elem_t * server_url_list_in );
wiced_bool_t web_server_is_running( void );
void quit_web_server( void );


#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* ifndef WEB_SERVER_H_INCLUDED */
