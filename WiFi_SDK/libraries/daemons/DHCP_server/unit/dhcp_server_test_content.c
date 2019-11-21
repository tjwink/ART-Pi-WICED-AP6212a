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
 */

#include "dhcp_server_unit.h"

const dhcp_header_t normal_dhcp_discover_header =
{
        .opcode = BOOTP_OP_REQUEST,
        .hardware_type  = BOOTP_HTYPE_ETHERNET,
        .hardware_addr_len   = 6,

        .client_hardware_addr = { 0x02, 0x0A, 0xF7, 0x5d, 0x6a, 0x5d },
        .transaction_id = 1,


};


const char dhcp_magic[] = { DHCP_MAGIC_COOKIE };
const char dhcp_end[] = { DHCP_OPTION_END };
const char dhcp_discover[] = { DHCP_OPTION_CODE_DHCP_MESSAGE_TYPE, 1, DHCPDISCOVER };
const char dhcp_req_ip[] = { DHCP_OPTION_CODE_REQUEST_IP_ADDRESS, 4, 192, 168, 0, 100 };



const dhcp_option_t normal_dhcp_discover_options[] =
{

        MAKE_DHCP_OPTION( dhcp_magic ),
        MAKE_DHCP_OPTION( dhcp_discover ),
        MAKE_DHCP_OPTION( dhcp_end ),
        { NULL, 0 }
};

const dhcp_option_t dhcp_discover_w_request_options[] =
{

        MAKE_DHCP_OPTION( dhcp_magic ),
        MAKE_DHCP_OPTION( dhcp_discover ),
        MAKE_DHCP_OPTION( dhcp_req_ip ),
        MAKE_DHCP_OPTION( dhcp_end ),
        { NULL, 0 }
};
