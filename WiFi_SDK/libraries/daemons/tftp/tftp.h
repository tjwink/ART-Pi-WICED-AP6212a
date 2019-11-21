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
#pragma once

#include "wiced.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define TFTP_ERROR_NOT_DEFINED          (0)
#define TFTP_ERROR_FILE_NOT_FOUND       (1)
#define TFTP_ERROR_ACCESS_VIOLATION     (2)
#define TFTP_ERROR_DISK_FULL            (3)
#define TFTP_ERROR_ILLEGAL_OPERATION    (4)
#define TFTP_ERROR_UNKNOWN_TID          (5)
#define TFTP_ERROR_FILE_EXISTS          (6)
#define TFTP_ERROR_NO_SUCH_USER         (7)
#define TFTP_NO_ERROR                   (255)

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct tftp_client_s tftp_client_t;

typedef enum
{
    TFTP_MODE_NETASCII = 0,
    TFTP_MODE_OCTET = 1,
    TFTP_MODE_MAIL = 2
}tftp_mode_t;

typedef enum
{
    TFTP_GET,
    TFTP_PUT
}tftp_request_t;

typedef struct tftp_s
{
        uint16_t                timeout;
        uint16_t                block_size;
        uint32_t                transfer_size;
        const char*             filename;
        tftp_mode_t             mode;
        tftp_request_t          request;
}tftp_t;

typedef struct tftp_callback_s
{
        wiced_result_t (*tftp_establish) ( tftp_t* tftp, void* p_user );
        wiced_result_t (*tftp_read)      ( tftp_t* tftp, uint8_t* data, void* p_user );
        wiced_result_t (*tftp_write)     ( tftp_t* tftp, uint8_t* data, void* p_user );
        wiced_result_t (*tftp_close)     ( tftp_t* tftp, int status, void* p_user );
}tftp_callback_t;

typedef struct tftp_connection_s
{
        wiced_thread_t          thread;
        wiced_udp_socket_t      socket;
        wiced_interface_t       interface;
        wiced_ip_address_t      ip;
        uint16_t                tid;
        uint16_t                blk_num;
        uint16_t                retry;
        tftp_t                  tftp;
        void*                   p_user;
        tftp_callback_t         callback;
    /* flag to check server completion */
        uint8_t                 tftp_complete;
}tftp_connection_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/* tftp client management functions */
wiced_result_t tftp_server_start( tftp_connection_t* server, wiced_interface_t interface, tftp_callback_t* callback, void* p_user );

wiced_result_t tftp_server_stop ( tftp_connection_t* server );

wiced_result_t tftp_client_get  ( tftp_connection_t* client, wiced_ip_address_t host, wiced_interface_t interface, const char * filename, tftp_mode_t mode, tftp_callback_t *callback, void *p_user );

wiced_result_t tftp_client_put  ( tftp_connection_t* client, wiced_ip_address_t host, wiced_interface_t interface, const char * filename, tftp_mode_t mode, tftp_callback_t *callback, void *p_user );


#ifdef __cplusplus
} /* extern "C" */
#endif
